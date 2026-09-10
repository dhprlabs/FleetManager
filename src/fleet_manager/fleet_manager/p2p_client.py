#!/usr/bin/env python3
"""
P2P Client API
--------------
Clean API wrapper for robot-side modules to send and receive messages
over the simulated P2P wireless transport layer.

Supported message types:
  - STATE_BEACON
  - TASK_STATUS
  - MAXSUM_MESSAGE
  - INTENT
  - RESERVATION_REQUEST
  - CUSTOM
"""

import json
from typing import Callable, Dict, List, Optional

from fleet_interfaces.msg import P2PMessage, P2PNeighborList, P2PEvent


class P2PClient:
    def __init__(self, node, on_message: Optional[Callable[[P2PMessage], None]] = None):
        """
        Initializes the P2P Client within a robot node.
        
        :param node: The parent ROS 2 Node
        :param on_message: Optional fallback message handler callback
        """
        self.node = node
        # Default robot_id from namespace or node parameter
        default_id = node.get_namespace().strip('/') or 'robot_1'
        self.robot_id = getattr(node, 'robot_id', default_id)

        self.on_message_callback = on_message
        self.message_handlers: Dict[str, Callable[[P2PMessage], None]] = {}
        self.reachable_peers: List[str] = []
        self.peer_distances: Dict[str, float] = {}

        # Local P2P transport interface (namespaced within the robot)
        self.outbound_pub = self.node.create_publisher(P2PMessage, 'p2p_outbound', 20)
        self.inbound_sub = self.node.create_subscription(
            P2PMessage, 'p2p_inbound', self._handle_inbound, 20
        )
        self.neighbors_sub = self.node.create_subscription(
            P2PNeighborList, 'p2p_neighbors', self._handle_neighbors, 10
        )
        self.events_sub = self.node.create_subscription(
            P2PEvent, 'p2p_events', self._handle_event, 10
        )

        self._seq = 0

    def register_handler(self, message_type: str, callback: Callable[[P2PMessage], None]):
        """Registers a specialized callback for a message type."""
        self.message_handlers[message_type] = callback

    def is_peer_reachable(self, peer_id: str) -> bool:
        """Returns True if the peer is currently within radio communication range."""
        return peer_id in self.reachable_peers

    def get_reachable_peers(self) -> List[str]:
        """Returns a list of robot IDs currently within radio communication range."""
        return list(self.reachable_peers)

    def get_peer_distance(self, peer_id: str) -> float:
        """Returns the last known distance to the given peer, or infinity if not seen."""
        return self.peer_distances.get(peer_id, float('inf'))

    def broadcast(self, message_type: str, payload=None) -> bool:
        """
        Broadcasts a message to all currently reachable neighbors within radio range.
        
        :param message_type: Type string (e.g. 'STATE_BEACON', 'MAXSUM_MESSAGE')
        :param payload: Dict, list, or primitive serializable to JSON/str
        :return: True if dispatched to local transport
        """
        return self.send_to('*', message_type, payload)

    def send_to(self, target_robot_id: str, message_type: str, payload=None) -> bool:
        """
        Sends a directed message to a specific robot peer.
        
        :param target_robot_id: Peer ID (e.g. 'robot2') or '*' for broadcast
        :param message_type: Type string
        :param payload: Message payload
        :return: True if dispatched to local transport
        """
        msg = P2PMessage()
        msg.source_robot_id = self.robot_id
        msg.target_robot_id = target_robot_id
        msg.message_type = message_type

        if isinstance(payload, (dict, list)):
            msg.payload = json.dumps(payload)
        elif payload is not None:
            msg.payload = str(payload)
        else:
            msg.payload = ''

        msg.timestamp = self.node.get_clock().now().to_msg()
        self._seq += 1
        msg.sequence_number = self._seq

        self.outbound_pub.publish(msg)
        return True

    def _handle_inbound(self, msg: P2PMessage):
        """Dispatches incoming P2P message to registered handlers or default callback."""
        handler = self.message_handlers.get(msg.message_type)
        if handler:
            handler(msg)
        elif self.on_message_callback:
            self.on_message_callback(msg)

    def _handle_neighbors(self, msg: P2PNeighborList):
        """Updates the list of reachable neighbors."""
        self.reachable_peers = list(msg.reachable_peers)
        self.peer_distances = dict(zip(msg.reachable_peers, msg.peer_distances))

    def _handle_event(self, msg: P2PEvent):
        """Event hook for connection/disconnection changes."""
        pass
