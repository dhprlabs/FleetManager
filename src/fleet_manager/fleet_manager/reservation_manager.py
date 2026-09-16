#!/usr/bin/env python3
"""
Reservation Manager Node — Phase 12 Implementation
--------------------------------------------------
Runs identically on each robot.
Manages mutually exclusive spatial-temporal reservations for narrow single-lane aisle segments.

Architecture:
  Max-Sum ──► Task Ownership ──► Bundle Manager ──► Nav2 ──► Reservation Manager ──► PIBT / ORCA

Responsibilities:
  1. Represents narrow single-lane aisle segments explicitly with bounding boxes.
  2. Subscribes and publishes to /traffic/reserve and /traffic/release (and /fleet/reservations).
  3. Deterministic contention resolution using (priority, Lamport clock, robot_id).
  4. Only one robot can own a mutually exclusive segment at a time; others wait.
  5. Promotes waiting robots when current holder releases the segment.
  6. Heartbeat/expiry safety lease to recover from communication loss without deadlock.
  7. Integrates with virtual P2P reachability (P2PClient).
"""

import json
import math
import os
import time
from typing import Dict, List, Optional, Set, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Point
from std_msgs.msg import String
from fleet_interfaces.msg import P2PMessage, Reservation
from fleet_manager.p2p_client import P2PClient
from fleet_manager.audit_log import audit_event


# ─────────────────────────────────────────────────────────────────────────────
# Default Single-Lane Aisle Segments in Warehouse Map
# ─────────────────────────────────────────────────────────────────────────────

DEFAULT_AISLE_SEGMENTS = {
    'aisle_1': {
        'segment_id': 'aisle_1',
        'x_min': 1.5, 'x_max': 4.0,
        'y_min': 0.0, 'y_max': 2.5,
        'is_single_lane': True,
        'description': 'Main picking aisle 1 (narrow single lane)'
    },
    'aisle_2': {
        'segment_id': 'aisle_2',
        'x_min': 1.5, 'x_max': 4.0,
        'y_min': -3.5, 'y_max': -1.0,
        'is_single_lane': True,
        'description': 'Main picking aisle 2 (narrow single lane)'
    },
    'aisle_3': {
        'segment_id': 'aisle_3',
        'x_min': -4.5, 'x_max': -2.0,
        'y_min': -2.0, 'y_max': 2.0,
        'is_single_lane': True,
        'description': 'Dropoff transit corridor (narrow single lane)'
    },
}


class AisleState:
    """State tracking for an individual single-lane segment."""
    def __init__(self, segment_id: str, x_min: float, x_max: float, y_min: float, y_max: float):
        self.segment_id = segment_id
        self.x_min = x_min
        self.x_max = x_max
        self.y_min = y_min
        self.y_max = y_max
        self.current_holder: Optional[Reservation] = None
        self.holder_start_time: float = 0.0
        self.wait_queue: List[Reservation] = []

    def contains_point(self, x: float, y: float) -> bool:
        return (self.x_min <= x <= self.x_max) and (self.y_min <= y <= self.y_max)


def reservation_sort_key(res: Reservation) -> Tuple[int, int, str]:
    """
    Deterministic sorting key for contention resolution:
      1. Higher priority wins (-priority).
      2. Lower Lamport clock wins (earlier request).
      3. Lexicographical robot_id tie-break.
    """
    return (-int(res.priority), int(res.lamport_clock), str(res.robot_id))


class ReservationManager(Node):
    def __init__(self):
        super().__init__('reservation_manager')

        default_id = self.get_namespace().strip('/') or 'robot_1'
        self.declare_parameter('robot_id', default_id)
        self.robot_id = self.get_parameter('robot_id').get_parameter_value().string_value

        self.declare_parameter('safety_timeout_sec', 10.0)
        self.safety_timeout_sec = self.get_parameter('safety_timeout_sec').get_parameter_value().double_value

        self.declare_parameter(
            'aisle_config_file',
            '/home/mangal-devanshu/sih_ws/FleetManager/src/fleet_manager/config/aisle_segments.json'
        )
        self.aisle_config_file = self.get_parameter('aisle_config_file').get_parameter_value().string_value

        # Lamport logical clock for deterministic traffic ordering
        self.lamport_clock: int = 0
        self._seen_res_ids: Set[str] = set()

        # Aisle Segments (loaded from persistent JSON or fallback to defaults)
        self.aisle_specs = self._load_aisle_specs()
        self.aisles: Dict[str, AisleState] = {}
        for seg_id, spec in self.aisle_specs.items():
            self.aisles[seg_id] = AisleState(
                seg_id, float(spec['x_min']), float(spec['x_max']), float(spec['y_min']), float(spec['y_max'])
            )

        # Local reservations held or requested by this robot
        self.my_active_reservations: Dict[str, Reservation] = {}  # segment_id -> Reservation

        # P2P client for virtual network coordination
        try:
            self.p2p = P2PClient(node=self)
            self.p2p.register_handler('TRAFFIC_RESERVE', self._on_p2p_traffic_reserve)
            self.p2p.register_handler('TRAFFIC_RELEASE', self._on_p2p_traffic_release)
        except Exception as e:
            self.p2p = None
            self.get_logger().warn(f'[{self.robot_id}] P2PClient init skipped: {e}')

        # ── ROS Interfaces ─────────────────────────────────────────────────
        # /traffic/reserve
        self.traffic_reserve_pub = self.create_publisher(Reservation, '/traffic/reserve', 10)
        self.traffic_reserve_sub = self.create_subscription(
            Reservation, '/traffic/reserve', self._handle_reserve_msg, 10
        )

        # /traffic/release
        self.traffic_release_pub = self.create_publisher(Reservation, '/traffic/release', 10)
        self.traffic_release_sub = self.create_subscription(
            Reservation, '/traffic/release', self._handle_release_msg, 10
        )

        # /fleet/reservations (global status / monitoring)
        self.fleet_res_pub = self.create_publisher(Reservation, '/fleet/reservations', 10)
        self.fleet_res_sub = self.create_subscription(
            Reservation, '/fleet/reservations', self._handle_reserve_msg, 10
        )
        self.traffic_event_pub = self.create_publisher(String, '/fleet/traffic_events', 50)

        # /fleet/aisle_config: dynamic synchronization with UI and other nodes
        latching_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.aisle_config_pub = self.create_publisher(String, '/fleet/aisle_config', latching_qos)
        self.aisle_config_sub = self.create_subscription(
            String, '/fleet/aisle_config', self._handle_aisle_config_msg, 10
        )

        # Periodic watchdog timer for safety lease timeouts (1 Hz)
        self.watchdog_timer = self.create_timer(1.0, self._watchdog_check)

        # Publish initial configuration once
        self._publish_aisle_config()

        self.get_logger().info(
            f'[{self.robot_id}] Reservation Manager online | '
            f'Single-lane segments: {list(self.aisles.keys())} | Safety timeout: {self.safety_timeout_sec}s'
        )

    @staticmethod
    def load_aisle_specs(config_file: Optional[str] = None) -> dict:
        """Loads aisle specifications from JSON file if present, else DEFAULT_AISLE_SEGMENTS."""
        if config_file and os.path.exists(config_file):
            try:
                with open(config_file, 'r', encoding='utf-8') as f:
                    data = json.load(f)
                    if isinstance(data, dict):
                        specs = data.get('aisles', data)
                        if isinstance(specs, dict) and specs:
                            return specs
                        elif isinstance(specs, list):
                            return {item.get('segment_id', item.get('id')): item for item in specs if 'segment_id' in item or 'id' in item}
                    elif isinstance(data, list):
                        return {item.get('segment_id', item.get('id')): item for item in data if 'segment_id' in item or 'id' in item}
            except Exception:
                pass
        return dict(DEFAULT_AISLE_SEGMENTS)

    @staticmethod
    def save_aisle_specs(config_file: Optional[str], specs: dict) -> bool:
        """Persists aisle specifications to disk."""
        if not config_file:
            return False
        try:
            os.makedirs(os.path.dirname(config_file), exist_ok=True)
            with open(config_file, 'w', encoding='utf-8') as f:
                json.dump(specs, f, indent=2)
            return True
        except Exception:
            return False

    def _load_aisle_specs(self) -> dict:
        return self.load_aisle_specs(self.aisle_config_file)

    def _save_aisle_specs(self, specs: dict):
        if self.save_aisle_specs(self.aisle_config_file, specs):
            self.get_logger().info(f'[{self.robot_id}] Persisted {len(specs)} aisles to {self.aisle_config_file}')
        else:
            self.get_logger().warn(f'[{self.robot_id}] Failed saving to {self.aisle_config_file}')

    def _publish_aisle_config(self):
        """Publishes the current aisle configuration to /fleet/aisle_config."""
        msg = String()
        msg.data = json.dumps({
            'aisles': self.aisle_specs,
            'source_robot_id': self.robot_id,
            'timestamp': self.get_clock().now().nanoseconds
        })
        self.aisle_config_pub.publish(msg)

    def _handle_aisle_config_msg(self, msg: String):
        """Handles incoming /fleet/aisle_config updates from the UI or peer nodes."""
        try:
            payload = json.loads(msg.data)
            source = payload.get('source_robot_id')
            if source == self.robot_id:
                return  # echo of our own publish
            raw_aisles = payload.get('aisles', payload)
            new_specs = {}
            if isinstance(raw_aisles, dict):
                new_specs = raw_aisles
            elif isinstance(raw_aisles, list):
                for item in raw_aisles:
                    if isinstance(item, dict) and 'segment_id' in item:
                        new_specs[item['segment_id']] = item

            if not new_specs:
                return

            # Update in-memory AisleState dictionary while preserving active holders
            self.aisle_specs = new_specs
            old_aisles = dict(self.aisles)
            self.aisles.clear()
            for seg_id, spec in new_specs.items():
                aisle = AisleState(
                    seg_id,
                    float(spec.get('x_min', 0.0)),
                    float(spec.get('x_max', 0.0)),
                    float(spec.get('y_min', 0.0)),
                    float(spec.get('y_max', 0.0))
                )
                if seg_id in old_aisles:
                    aisle.current_holder = old_aisles[seg_id].current_holder
                    aisle.holder_start_time = old_aisles[seg_id].holder_start_time
                    aisle.wait_queue = old_aisles[seg_id].wait_queue
                self.aisles[seg_id] = aisle

            self.get_logger().info(
                f'[{self.robot_id}] Dynamic aisle configuration updated: {list(self.aisles.keys())}'
            )
            self._save_aisle_specs(new_specs)
        except Exception as e:
            self.get_logger().warn(f'[{self.robot_id}] Error parsing /fleet/aisle_config: {e}')

    # ──────────────────────────────────────────────────────────────────────────
    # Geometry & Aisle Lookup
    # ──────────────────────────────────────────────────────────────────────────

    def find_segment_for_pose(self, x: float, y: float) -> Optional[str]:
        """Returns the segment_id if the given (x, y) pose lies inside a single-lane segment."""
        for seg_id, aisle in self.aisles.items():
            if aisle.contains_point(x, y):
                return seg_id
        return None

    def _audit(self, event: str, robot_id: Optional[str] = None, level: str = 'info', **details):
        """Publish an event attributed to the robot affected by the reservation."""
        record = audit_event(self.get_logger(), 'reservation_manager', event, robot_id or self.robot_id,
                             level=level, **details)
        message = String()
        message.data = json.dumps(record, separators=(',', ':'), default=str)
        self.traffic_event_pub.publish(message)

    def get_segment_bounds(self, segment_id: str) -> Tuple[Point, Point]:
        """Returns (zone_min, zone_max) for the given segment_id."""
        aisle = self.aisles.get(segment_id)
        p_min = Point()
        p_max = Point()
        if aisle:
            p_min.x = float(aisle.x_min)
            p_min.y = float(aisle.y_min)
            p_max.x = float(aisle.x_max)
            p_max.y = float(aisle.y_max)
        return p_min, p_max

    # ──────────────────────────────────────────────────────────────────────────
    # Public Reservation API (Local Robot Invocation)
    # ──────────────────────────────────────────────────────────────────────────

    def request_aisle_reservation(self, segment_id: str, task_id: str = "", priority: int = 1) -> bool:
        """
        Invoked by the local robot (e.g. nav2_bridge) before entering a single-lane segment.
        Returns True if granted immediately, False if placed in waiting queue.
        """
        if segment_id not in self.aisles:
            self.get_logger().warn(f'[{self.robot_id}] Unknown segment {segment_id} requested.')
            return True

        self.lamport_clock += 1
        res_id = f"{self.robot_id}:{segment_id}:{self.lamport_clock}"
        self._seen_res_ids.add(res_id)

        p_min, p_max = self.get_segment_bounds(segment_id)
        now_time = self.get_clock().now()

        res = Reservation()
        res.reservation_id = res_id
        res.robot_id = self.robot_id
        res.segment_id = segment_id
        res.task_id = task_id
        res.state = Reservation.STATE_REQUESTED
        res.lamport_clock = int(self.lamport_clock)
        res.priority = int(priority)
        res.zone_min = p_min
        res.zone_max = p_max
        res.request_time = now_time.to_msg()
        res.start_time = now_time.to_msg()

        # Broadcast reservation request locally, on /traffic/reserve, and over P2P
        self._publish_reservation_request(res)
        self._audit('reservation_requested', reservation_id=res_id, segment_id=segment_id,
                    task_id=task_id, priority=priority, lamport_clock=self.lamport_clock)

        # Process locally
        granted = self._process_reservation_request(res)
        return granted

    def release_aisle_reservation(self, segment_id: str):
        """
        Invoked by the local robot upon leaving a single-lane segment.
        Releases the reservation and notifies waiting robots.
        """
        if segment_id not in self.aisles:
            return

        self.lamport_clock += 1
        res_id = f"{self.robot_id}:{segment_id}:{self.lamport_clock}"
        self._seen_res_ids.add(res_id)

        now_time = self.get_clock().now()
        p_min, p_max = self.get_segment_bounds(segment_id)

        res = Reservation()
        res.reservation_id = res_id
        res.robot_id = self.robot_id
        res.segment_id = segment_id
        res.state = Reservation.STATE_RELEASED
        res.lamport_clock = int(self.lamport_clock)
        res.zone_min = p_min
        res.zone_max = p_max
        res.end_time = now_time.to_msg()

        self._publish_reservation_release(res)
        self._audit('reservation_release_requested', reservation_id=res_id,
                    segment_id=segment_id, lamport_clock=self.lamport_clock)
        self._process_reservation_release(res)

    def is_reservation_granted(self, segment_id: str) -> bool:
        """Returns True if this robot currently holds the active reservation for segment_id."""
        aisle = self.aisles.get(segment_id)
        if not aisle:
            return True
        return (aisle.current_holder is not None and aisle.current_holder.robot_id == self.robot_id)

    # ──────────────────────────────────────────────────────────────────────────
    # Core Reservation Logic & Deterministic Contention Resolution
    # ──────────────────────────────────────────────────────────────────────────

    def _process_reservation_request(self, res: Reservation) -> bool:
        """
        Deterministically processes a reservation request for a single-lane segment.
        Returns True if granted, False if queued.
        """
        segment_id = res.segment_id
        if segment_id not in self.aisles:
            return False

        aisle = self.aisles[segment_id]
        self.lamport_clock = max(self.lamport_clock, res.lamport_clock) + 1

        # Case 1: Already held by requester
        if aisle.current_holder is not None and aisle.current_holder.robot_id == res.robot_id:
            res.state = Reservation.STATE_GRANTED
            aisle.current_holder = res
            aisle.holder_start_time = time.monotonic()
            if res.robot_id == self.robot_id:
                self.my_active_reservations[segment_id] = res
            return True

        # Case 2: Segment is free
        if aisle.current_holder is None:
            res.state = Reservation.STATE_GRANTED
            aisle.current_holder = res
            aisle.holder_start_time = time.monotonic()
            if res.robot_id == self.robot_id:
                self.my_active_reservations[segment_id] = res

            self.get_logger().info(
                f'[{self.robot_id}] [TRAFFIC] ✓ Segment "{segment_id}" GRANTED to {res.robot_id} '
                f'(priority={res.priority}, Lamport={res.lamport_clock}).'
            )
            # If we granted it to ourselves, announce grant
            if res.robot_id == self.robot_id:
                self._broadcast_grant(res)
            self._audit('reservation_granted', robot_id=res.robot_id, reservation_id=res.reservation_id,
                        segment_id=segment_id, holder=res.robot_id, task_id=res.task_id,
                        priority=res.priority, lamport_clock=res.lamport_clock,
                        wait_queue=[q.robot_id for q in aisle.wait_queue])
            return True

        # Case 3: Segment is currently held by someone else
        curr_holder = aisle.current_holder

        # Requester must wait — insert into deterministic priority wait queue
        res.state = Reservation.STATE_WAITING
        # Avoid duplicate queue entries
        aisle.wait_queue = [q for q in aisle.wait_queue if q.robot_id != res.robot_id]
        aisle.wait_queue.append(res)
        aisle.wait_queue.sort(key=reservation_sort_key)

        queue_pos = [q.robot_id for q in aisle.wait_queue].index(res.robot_id) + 1
        self.get_logger().info(
            f'[{self.robot_id}] [TRAFFIC] ✗ Segment "{segment_id}" occupied by {curr_holder.robot_id}. '
            f'Requester {res.robot_id} placed in WAITING queue (position {queue_pos}/{len(aisle.wait_queue)}).'
        )
        self._audit('reservation_queued', robot_id=res.robot_id, reservation_id=res.reservation_id,
                    segment_id=segment_id, requester=res.robot_id,
                    current_holder=curr_holder.robot_id, queue_position=queue_pos,
                    wait_queue=[q.robot_id for q in aisle.wait_queue],
                    priority=res.priority, lamport_clock=res.lamport_clock)
        return False

    def _process_reservation_release(self, res: Reservation):
        """Processes a segment release and promotes the top waiting robot."""
        segment_id = res.segment_id
        if segment_id not in self.aisles:
            return

        aisle = self.aisles[segment_id]
        self.lamport_clock = max(self.lamport_clock, res.lamport_clock) + 1

        # Check if releaser is current holder
        if aisle.current_holder is not None and aisle.current_holder.robot_id == res.robot_id:
            self.get_logger().info(
                f'[{self.robot_id}] [TRAFFIC] Segment "{segment_id}" RELEASED by {res.robot_id}.'
            )
            aisle.current_holder = None
            self._audit('reservation_released', robot_id=res.robot_id, reservation_id=res.reservation_id,
                        segment_id=segment_id, holder=res.robot_id)
            if res.robot_id == self.robot_id and segment_id in self.my_active_reservations:
                del self.my_active_reservations[segment_id]

            # Promote next waiting robot in deterministic order
            self._promote_next_waiter(segment_id)
        else:
            # Releaser was in wait queue (cancelled its request)
            aisle.wait_queue = [q for q in aisle.wait_queue if q.robot_id != res.robot_id]

    def _promote_next_waiter(self, segment_id: str):
        """Promotes the first robot in the deterministic wait queue to GRANTED."""
        aisle = self.aisles.get(segment_id)
        if not aisle or not aisle.wait_queue:
            return

        next_res = aisle.wait_queue.pop(0)
        next_res.state = Reservation.STATE_GRANTED
        aisle.current_holder = next_res
        aisle.holder_start_time = time.monotonic()

        self.get_logger().info(
            f'[{self.robot_id}] [TRAFFIC] ★ Segment "{segment_id}" PROMOTED/GRANTED to '
            f'waiting robot {next_res.robot_id} (priority={next_res.priority}, Lamport={next_res.lamport_clock}).'
        )
        self._audit('reservation_promoted', robot_id=next_res.robot_id, reservation_id=next_res.reservation_id,
                    segment_id=segment_id, holder=next_res.robot_id,
                    priority=next_res.priority, lamport_clock=next_res.lamport_clock,
                    remaining_wait_queue=[q.robot_id for q in aisle.wait_queue])

        if next_res.robot_id == self.robot_id:
            self.my_active_reservations[segment_id] = next_res
            self._broadcast_grant(next_res)

    def _broadcast_grant(self, res: Reservation):
        """Broadcasts grant confirmation on /traffic/reserve and P2P."""
        self.traffic_reserve_pub.publish(res)
        self.fleet_res_pub.publish(res)
        if self.p2p:
            self.p2p.broadcast('TRAFFIC_RESERVE', payload=self._res_to_dict(res))

    # ──────────────────────────────────────────────────────────────────────────
    # Communication Loss Watchdog & Deadlock Recovery
    # ──────────────────────────────────────────────────────────────────────────

    def _watchdog_check(self):
        """
        Heartbeat / safety lease check:
        Prevents warehouse deadlock when a robot disconnects or crashes while holding a reservation.
        """
        now = time.monotonic()
        for seg_id, aisle in self.aisles.items():
            if aisle.current_holder is None:
                continue

            holder = aisle.current_holder
            elapsed = now - aisle.holder_start_time

            # Check if lease expired
            if elapsed > self.safety_timeout_sec:
                self.get_logger().warn(
                    f'[{self.robot_id}] [TRAFFIC] ⚠ Stale reservation detected for segment "{seg_id}" '
                    f'held by {holder.robot_id} (held {elapsed:.1f}s > {self.safety_timeout_sec}s timeout). '
                    f'Expiring reservation to recover physical traffic.'
                )
                # If we were the holder, release local active lock
                if holder.robot_id == self.robot_id and seg_id in self.my_active_reservations:
                    del self.my_active_reservations[seg_id]

                aisle.current_holder = None
                self._audit('reservation_expired', robot_id=holder.robot_id, level='warn', segment_id=seg_id,
                            holder=holder.robot_id, held_seconds=round(elapsed, 3),
                            timeout_seconds=self.safety_timeout_sec,
                            recovery='promote_next_waiter')
                self._promote_next_waiter(seg_id)

    # ──────────────────────────────────────────────────────────────────────────
    # Message Handlers & Serialization
    # ──────────────────────────────────────────────────────────────────────────

    def _handle_reserve_msg(self, msg: Reservation):
        if msg.reservation_id and msg.reservation_id in self._seen_res_ids:
            return
        if msg.reservation_id:
            self._seen_res_ids.add(msg.reservation_id)

        if msg.robot_id == self.robot_id:
            return

        self._process_reservation_request(msg)

    def _handle_release_msg(self, msg: Reservation):
        if msg.reservation_id and msg.reservation_id in self._seen_res_ids:
            return
        if msg.reservation_id:
            self._seen_res_ids.add(msg.reservation_id)

        if msg.robot_id == self.robot_id:
            return

        self._process_reservation_release(msg)

    def _on_p2p_traffic_reserve(self, msg: P2PMessage):
        """P2P handler for reservation requests."""
        try:
            d = json.loads(msg.payload)
            res = self._dict_to_res(d)
            if res.reservation_id and res.reservation_id in self._seen_res_ids:
                return
            if res.reservation_id:
                self._seen_res_ids.add(res.reservation_id)
            if res.robot_id == self.robot_id:
                return
            self._process_reservation_request(res)
        except Exception as e:
            self.get_logger().warn(f'[{self.robot_id}] [TRAFFIC] Error parsing P2P TRAFFIC_RESERVE: {e}')

    def _on_p2p_traffic_release(self, msg: P2PMessage):
        """P2P handler for reservation releases."""
        try:
            d = json.loads(msg.payload)
            res = self._dict_to_res(d)
            if res.reservation_id and res.reservation_id in self._seen_res_ids:
                return
            if res.reservation_id:
                self._seen_res_ids.add(res.reservation_id)
            if res.robot_id == self.robot_id:
                return
            self._process_reservation_release(res)
        except Exception as e:
            self.get_logger().warn(f'[{self.robot_id}] [TRAFFIC] Error parsing P2P TRAFFIC_RELEASE: {e}')

    def _publish_reservation_request(self, res: Reservation):
        self.traffic_reserve_pub.publish(res)
        self.fleet_res_pub.publish(res)
        if self.p2p:
            self.p2p.broadcast('TRAFFIC_RESERVE', payload=self._res_to_dict(res))

    def _publish_reservation_release(self, res: Reservation):
        self.traffic_release_pub.publish(res)
        self.fleet_res_pub.publish(res)
        if self.p2p:
            self.p2p.broadcast('TRAFFIC_RELEASE', payload=self._res_to_dict(res))

    def _res_to_dict(self, res: Reservation) -> dict:
        return {
            'reservation_id': res.reservation_id,
            'robot_id': res.robot_id,
            'segment_id': res.segment_id,
            'task_id': res.task_id,
            'state': int(res.state),
            'lamport_clock': int(res.lamport_clock),
            'priority': int(res.priority),
            'x_min': float(res.zone_min.x),
            'y_min': float(res.zone_min.y),
            'x_max': float(res.zone_max.x),
            'y_max': float(res.zone_max.y),
        }

    def _dict_to_res(self, d: dict) -> Reservation:
        res = Reservation()
        res.reservation_id = d.get('reservation_id', '')
        res.robot_id = d.get('robot_id', '')
        res.segment_id = d.get('segment_id', '')
        res.task_id = d.get('task_id', '')
        res.state = int(d.get('state', Reservation.STATE_REQUESTED))
        res.lamport_clock = int(d.get('lamport_clock', 0))
        res.priority = int(d.get('priority', 1))
        res.zone_min.x = float(d.get('x_min', 0.0))
        res.zone_min.y = float(d.get('y_min', 0.0))
        res.zone_max.x = float(d.get('x_max', 0.0))
        res.zone_max.y = float(d.get('y_max', 0.0))
        return res


def main(args=None):
    rclpy.init(args=args)
    node = ReservationManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
