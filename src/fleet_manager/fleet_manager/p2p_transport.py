#!/usr/bin/env python3
"""
P2P Transport Node
------------------
Runs identically on each robot.
Simulates a decentralized radio communication medium based on physical distance.

Rules:
  - R_i <-> R_j if distance(R_i, R_j) <= communication_radius
  - R_i ↛ R_j if distance(R_i, R_j) > communication_radius
  - Simulated dead zones and packet loss
  - Connection/disconnection event tracking and neighbour discovery
  - Supports unicast and neighbour broadcast
"""

import json
import math
import random
import time
from typing import Dict, List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

from fleet_interfaces.msg import (
    P2PMessage,
    P2PBeacon,
    P2PEvent,
    P2PNeighborList,
    RobotState
)


class P2PTransport(Node):
    def __init__(self):
        super().__init__('p2p_transport')

        default_id = self.get_namespace().strip('/') or 'robot_1'
        self.declare_parameter('robot_id', default_id)
        self.robot_id = self.get_parameter('robot_id').get_parameter_value().string_value

        # Radio & Physical Medium Parameters
        self.declare_parameter('communication_radius', 8.0)  # meters
        self.declare_parameter('beacon_rate', 2.0)           # Hz
        self.declare_parameter('peer_timeout', 3.0)          # seconds
        self.declare_parameter('packet_dropout_rate', 0.0)   # 0.0 (none) to 1.0 (all dropped)
        self.declare_parameter('dead_zones', '')             # JSON list of [x_min, y_min, x_max, y_max]
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)

        self.comm_radius = self.get_parameter('communication_radius').get_parameter_value().double_value
        self.beacon_rate = self.get_parameter('beacon_rate').get_parameter_value().double_value
        self.peer_timeout = self.get_parameter('peer_timeout').get_parameter_value().double_value
        self.dropout_rate = self.get_parameter('packet_dropout_rate').get_parameter_value().double_value

        # Dead zones configuration: list of tuples (xmin, ymin, xmax, ymax)
        self.dead_zones: List[Tuple[float, float, float, float]] = []
        dead_zones_str = self.get_parameter('dead_zones').get_parameter_value().string_value
        if dead_zones_str:
            try:
                parsed = json.loads(dead_zones_str)
                for dz in parsed:
                    if len(dz) == 4:
                        self.dead_zones.append((float(dz[0]), float(dz[1]), float(dz[2]), float(dz[3])))
            except Exception as e:
                self.get_logger().warn(f'Failed to parse dead_zones parameter: {e}')

        # Current Robot Pose — starts with initial parameter, updated via AMCL/odom
        self._amcl_initialized = False
        self.current_x = self.get_parameter('initial_x').get_parameter_value().double_value
        self.current_y = self.get_parameter('initial_y').get_parameter_value().double_value
        self.current_z = 0.0

        # Peer Discovery & Connection State
        # peer_id -> {'connected': bool, 'last_seen': float, 'x': float, 'y': float, 'dist': float}
        self.peers: Dict[str, dict] = {}

        # -------------------------------------------------------------
        # ROS 2 Subscriptions & Publishers
        # -------------------------------------------------------------
        # Local Robot Pose Sources
        self.create_subscription(RobotState, 'robot_state', self._handle_robot_state, 10)
        self.create_subscription(Odometry, 'odom', self._handle_odom, 10)
        amcl_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self._handle_amcl_pose, amcl_qos)

        # Local Node Interface (Namespaced within robot)
        self.inbound_pub = self.create_publisher(P2PMessage, 'p2p_inbound', 20)
        self.events_pub = self.create_publisher(P2PEvent, 'p2p_events', 10)
        self.neighbors_pub = self.create_publisher(P2PNeighborList, 'p2p_neighbors', 10)
        self.create_subscription(P2PMessage, 'p2p_outbound', self._handle_outbound_message, 20)

        # Shared Wireless Medium (Global Topics)
        self.beacon_pub = self.create_publisher(P2PBeacon, '/fleet/wireless_beacons', 50)
        self.create_subscription(P2PBeacon, '/fleet/wireless_beacons', self._handle_wireless_beacon, 50)

        self.wireless_packet_pub = self.create_publisher(P2PMessage, '/fleet/wireless_packets', 100)
        self.create_subscription(P2PMessage, '/fleet/wireless_packets', self._handle_wireless_packet, 100)

        # Timers
        self.beacon_timer = self.create_timer(1.0 / self.beacon_rate, self._broadcast_beacon)
        self.maintenance_timer = self.create_timer(0.5, self._check_peer_timeouts)

        self.get_logger().info(
            f'[{self.robot_id}] P2P Transport active | Radio radius: {self.comm_radius:.1f}m | '
            f'Waiting for first AMCL pose before beaconing...'
        )

    # -----------------------------------------------------------------
    # Local Pose Handlers
    # -----------------------------------------------------------------
    def _handle_robot_state(self, msg: RobotState):
        self.current_x = msg.current_pose.pose.position.x
        self.current_y = msg.current_pose.pose.position.y
        self.current_z = msg.current_pose.pose.position.z
        if not self._amcl_initialized:
            self._amcl_initialized = True
            self.get_logger().info(
                f'[{self.robot_id}] P2P fix received via robot_state at ({self.current_x:.2f}, {self.current_y:.2f}) — beaconing enabled.')

    def _handle_odom(self, msg: Odometry):
        # Do not overwrite global map pose with local odom frame coordinates
        pass

    def _handle_amcl_pose(self, msg: PoseWithCovarianceStamped):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_z = msg.pose.pose.position.z
        if not self._amcl_initialized:
            self._amcl_initialized = True
            self.get_logger().info(
                f'[{self.robot_id}] P2P AMCL fix received at ({self.current_x:.2f}, {self.current_y:.2f}) — beaconing enabled.')

    # -----------------------------------------------------------------
    # RF Range & Channel Modeling
    # -----------------------------------------------------------------
    def _distance_to(self, target_x: float, target_y: float) -> float:
        return math.hypot(self.current_x - target_x, self.current_y - target_y)

    def _is_in_dead_zone(self, x: float, y: float) -> bool:
        for xmin, ymin, xmax, ymax in self.dead_zones:
            if xmin <= x <= xmax and ymin <= y <= ymax:
                return True
        return False

    def _can_communicate(self, peer_x: float, peer_y: float) -> Tuple[bool, float, str]:
        """
        Evaluates RF connectivity between this robot and peer coordinates.
        Returns: (reachable_bool, distance, reason)
        """
        dist = self._distance_to(peer_x, peer_y)

        # 1. Radio range check
        if dist > self.comm_radius:
            return False, dist, f'out of radio range ({dist:.2f}m > {self.comm_radius:.2f}m)'

        # 2. Dead zone check
        if self._is_in_dead_zone(self.current_x, self.current_y) or self._is_in_dead_zone(peer_x, peer_y):
            return False, dist, 'obstructed by dead zone'

        # 3. Simulated packet dropout
        if self.dropout_rate > 0.0 and random.random() < self.dropout_rate:
            return False, dist, 'simulated RF packet drop'

        return True, dist, 'connected'

    # -----------------------------------------------------------------
    # Discovery & Beaconing
    # -----------------------------------------------------------------
    def _broadcast_beacon(self):
        if not self._amcl_initialized:
            return
        beacon = P2PBeacon()
        beacon.robot_id = self.robot_id
        beacon.pose.header.stamp = self.get_clock().now().to_msg()
        beacon.pose.header.frame_id = 'map'
        beacon.pose.pose.position.x = float(self.current_x)
        beacon.pose.pose.position.y = float(self.current_y)
        beacon.pose.pose.position.z = float(self.current_z)
        beacon.pose.pose.orientation.w = 1.0
        beacon.timestamp = beacon.pose.header.stamp

        self.beacon_pub.publish(beacon)

    def _handle_wireless_beacon(self, beacon: P2PBeacon):
        peer_id = beacon.robot_id
        if peer_id == self.robot_id:
            return  # Ignore self

        peer_x = beacon.pose.pose.position.x
        peer_y = beacon.pose.pose.position.y
        dist = self._distance_to(peer_x, peer_y)
        now_sec = time.time()

        is_new_peer = peer_id not in self.peers
        if is_new_peer:
            self.peers[peer_id] = {
                'connected': False,
                'last_seen': now_sec,
                'x': peer_x,
                'y': peer_y,
                'dist': dist
            }

        peer_entry = self.peers[peer_id]
        peer_entry['last_seen'] = now_sec
        peer_entry['x'] = peer_x
        peer_entry['y'] = peer_y
        peer_entry['dist'] = dist

        was_connected = peer_entry['connected']

        # Determine connection eligibility (distance <= comm_radius and no dead zone)
        is_reachable, _, _ = self._can_communicate(peer_x, peer_y)

        if is_reachable and not was_connected:
            # Transition: CONNECTED
            peer_entry['connected'] = True
            self.get_logger().info(
                f'[{self.robot_id}] Connected to {peer_id} (distance: {dist:.1f} m <= {self.comm_radius:.1f} m)'
            )
            self._emit_event(peer_id, P2PEvent.EVENT_CONNECTED, dist)
            self._publish_neighbors()

        elif not is_reachable and was_connected:
            # Transition: DISCONNECTED
            peer_entry['connected'] = False
            self.get_logger().info(
                f'[{self.robot_id}] Disconnected from {peer_id} (distance: {dist:.1f} m > {self.comm_radius:.1f} m)'
            )
            self._emit_event(peer_id, P2PEvent.EVENT_DISCONNECTED, dist)
            self._publish_neighbors()

    def _check_peer_timeouts(self):
        """Checks for peers that have timed out or moved out of range."""
        now_sec = time.time()
        changed = False

        for peer_id, peer_entry in list(self.peers.items()):
            dist = self._distance_to(peer_entry['x'], peer_entry['y'])
            peer_entry['dist'] = dist
            elapsed = now_sec - peer_entry['last_seen']

            if peer_entry['connected']:
                if elapsed > self.peer_timeout or dist > self.comm_radius:
                    peer_entry['connected'] = False
                    reason = f'distance: {dist:.1f} m > {self.comm_radius:.1f} m' if dist > self.comm_radius else 'heartbeat timeout'
                    self.get_logger().info(
                        f'[{self.robot_id}] Disconnected from {peer_id} ({reason})'
                    )
                    self._emit_event(peer_id, P2PEvent.EVENT_DISCONNECTED, dist)
                    changed = True

        if changed or len(self.peers) > 0:
            self._publish_neighbors()

    def _emit_event(self, peer_id: str, event_type: int, distance: float):
        evt = P2PEvent()
        evt.robot_id = self.robot_id
        evt.peer_id = peer_id
        evt.event_type = event_type
        evt.distance = float(distance)
        evt.timestamp = self.get_clock().now().to_msg()
        self.events_pub.publish(evt)

    def _publish_neighbors(self):
        msg = P2PNeighborList()
        msg.robot_id = self.robot_id
        msg.timestamp = self.get_clock().now().to_msg()

        reachable = []
        distances = []
        for peer_id, entry in self.peers.items():
            if entry['connected']:
                reachable.append(peer_id)
                distances.append(float(entry['dist']))

        msg.reachable_peers = reachable
        msg.peer_distances = distances
        self.neighbors_pub.publish(msg)

    # -----------------------------------------------------------------
    # Wireless Packet Routing & Filtering
    # -----------------------------------------------------------------
    def _handle_outbound_message(self, msg: P2PMessage):
        """Handles outbound packet from local robot module and broadcasts to wireless channel."""
        # Attach current robot pose as transmission origin
        msg.source_robot_id = self.robot_id
        msg.sender_pose.header.frame_id = 'map'
        msg.sender_pose.header.stamp = self.get_clock().now().to_msg()
        msg.sender_pose.pose.position.x = float(self.current_x)
        msg.sender_pose.pose.position.y = float(self.current_y)
        msg.sender_pose.pose.position.z = float(self.current_z)
        msg.sender_pose.pose.orientation.w = 1.0

        self.wireless_packet_pub.publish(msg)

    def _handle_wireless_packet(self, msg: P2PMessage):
        """Receives packet from wireless channel and delivers locally if within radio range."""
        sender_id = msg.source_robot_id
        if sender_id == self.robot_id:
            return  # Ignore own transmission

        # Addressing check: either broadcast '*' or unicast explicitly to this robot
        if msg.target_robot_id not in ('*', self.robot_id):
            return

        # Physical medium distance check
        sender_x = msg.sender_pose.pose.position.x
        sender_y = msg.sender_pose.pose.position.y
        can_recv, dist, reason = self._can_communicate(sender_x, sender_y)

        if not can_recv:
            # Dropped due to range, deadzone, or simulated loss
            if reason == 'simulated RF packet drop':
                self._emit_event(sender_id, P2PEvent.EVENT_DROPOUT, dist)
            return

        # Deliver to local robot stack
        self.inbound_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = P2PTransport()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == '__main__':
    main()
