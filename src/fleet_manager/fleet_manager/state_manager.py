#!/usr/bin/env python3
"""
State Manager Node — Phase 3 Distributed State Manager
-------------------------------------------------------
Runs identically on each robot.

Responsibilities:
  1. Track own robot state (pose via odom/amcl, battery, status, task).
  2. Maintain a distributed world view of all robots and tasks using
     Lamport logical clocks and LWW deterministic merging.
  3. Propagate state changes to peers via P2P (STATE_BEACON, TASK_STATUS).
  4. Handle reconnection: send STATE_SYNC_REQUEST on connect event,
     reply to STATE_SYNC_REQUEST with full WorldView.
  5. Publish a /world_view topic so other local nodes can read the full
     fleet picture without going through P2P.
"""

import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener

from fleet_interfaces.msg import (
    RobotState,
    RobotWorldEntry,
    TaskWorldEntry,
    WorldView,
    Task,
    TaskPool,
    P2PMessage,
    P2PEvent,
)
from fleet_manager.fleet_state import FleetState, RobotEntry, TaskEntry, get_state_rank
from fleet_manager.p2p_client import P2PClient


class StateManager(Node):
    def __init__(self):
        super().__init__('state_manager')

        default_id = self.get_namespace().strip('/') or 'robot_1'
        self.declare_parameter('robot_id', default_id)
        self.robot_id = self.get_parameter('robot_id').get_parameter_value().string_value

        self.declare_parameter('update_rate', 2.0)
        self.declare_parameter('beacon_rate', 1.0)
        self.declare_parameter('dock_x', 5.0)
        self.declare_parameter('dock_y', 12.0)
        self.declare_parameter('broadcaster_x', 5.0)
        self.declare_parameter('broadcaster_y', 12.0)
        self.declare_parameter('communication_radius', 6.0)
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)

        self.rate = self.get_parameter('update_rate').get_parameter_value().double_value
        self.beacon_rate = self.get_parameter('beacon_rate').get_parameter_value().double_value
        self.dock_x = self.get_parameter('dock_x').get_parameter_value().double_value
        self.dock_y = self.get_parameter('dock_y').get_parameter_value().double_value
        self.broadcaster_x = self.get_parameter('broadcaster_x').get_parameter_value().double_value
        self.broadcaster_y = self.get_parameter('broadcaster_y').get_parameter_value().double_value
        self.comm_radius = self.get_parameter('communication_radius').get_parameter_value().double_value
        init_x = self.get_parameter('initial_x').get_parameter_value().double_value
        init_y = self.get_parameter('initial_y').get_parameter_value().double_value

        # Guard flag — suppress all publishing until AMCL delivers first pose
        self._amcl_initialized = False

        # ── Fleet State Store (Phase 10.1: CRDT-backed) ───────────────────
        def _crdt_log(msg: str):
            # Routine robot pose/heartbeat updates occur continuously at 5-10 Hz.
            # Log routine robot updates at DEBUG to keep terminal logs clean and readable.
            # Lifecycle events (tasks, state transitions, reconciliations) remain at INFO.
            if 'UPDATE entity=robot' in msg or 'Duplicate operation' in msg:
                self.get_logger().debug(msg)
            else:
                self.get_logger().info(msg)

        self.fs = FleetState(
            robot_id=self.robot_id,
            log_callback=_crdt_log,
        )

        # Initialise own entry with initial spawn pose; updated on amcl_pose/TF
        own = RobotEntry(
            robot_id=self.robot_id,
            x=init_x, y=init_y,
            battery=100.0,
            status=RobotState.STATUS_IDLE,
        )
        self.fs.update_own_robot(own)

        # ── Local Publishers ───────────────────────────────────────────────
        self.state_pub = self.create_publisher(RobotState, 'robot_state', 10)
        self.fleet_state_pub = self.create_publisher(RobotState, '/fleet/robot_states', 10)
        self.world_view_pub = self.create_publisher(WorldView, 'world_view', 10)
        self.fleet_world_view_pub = self.create_publisher(WorldView, '/fleet/world_views', 10)

        # ── Local Subscriptions ────────────────────────────────────────────
        self.create_subscription(Odometry, 'odom', self._odom_cb, 10)

        amcl_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self._amcl_cb, amcl_qos)

        # Task pool from broadcaster (initial seed)
        latching_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.create_subscription(TaskPool, '/fleet/task_pool', self._task_pool_cb, latching_qos)
        self.create_subscription(Task, '/fleet/task_events', self._task_event_cb, 10)
        self.create_subscription(Task, 'task_events', self._local_task_event_cb, 10)

        # P2P event subscription for reconnect handling
        self.create_subscription(P2PEvent, 'p2p_events', self._p2p_event_cb, 10)

        # ── P2P Client ─────────────────────────────────────────────────────
        self.p2p = P2PClient(node=self)
        self.p2p.register_handler('STATE_BEACON', self._on_state_beacon)
        self.p2p.register_handler('TASK_STATUS', self._on_task_status)
        self.p2p.register_handler('TASK_POOL', self._on_p2p_task_pool)
        self.p2p.register_handler('STATE_SYNC_REQUEST', self._on_sync_request)
        self.p2p.register_handler('STATE_SYNC_RESPONSE', self._on_sync_response)
        self.p2p.register_handler('CRDT_OPS', self._on_crdt_ops)

        # Tracks (task_id, lamport_clock) tuples already relayed to prevent gossip loops.
        self._relayed_task_ops: set = set()


        # ── TF Buffer & Listener ───────────────────────────────────────────
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ── Timers ─────────────────────────────────────────────────────────
        self.state_timer = self.create_timer(1.0 / self.rate, self._publish_local_state)
        self.beacon_timer = self.create_timer(1.0 / self.beacon_rate, self._broadcast_state_beacon)
        self.world_view_timer = self.create_timer(2.0, self._publish_world_view)
        self.log_timer = self.create_timer(5.0, self._log_world_summary)

        self.get_logger().info(
            f'[{self.robot_id}] Distributed State Manager online | '
            f'Lamport clock: {self.fs.clock} | Waiting for first AMCL/TF pose before publishing...'
        )

    # ──────────────────────────────────────────────────────────────────────────
    # Own Pose Updates
    # ──────────────────────────────────────────────────────────────────────────

    def _update_pose_from_tf(self):
        target_frame = f'{self.robot_id}/base_footprint'
        try:
            if self.tf_buffer.can_transform('map', target_frame, rclpy.time.Time()):
                t = self.tf_buffer.lookup_transform('map', target_frame, rclpy.time.Time())
                p = t.transform.translation
                o = t.transform.rotation
                own = self.fs.get_own_state()
                if own:
                    changed = (abs(own.x - p.x) > 0.01 or abs(own.y - p.y) > 0.01)
                    own.x, own.y, own.z = p.x, p.y, p.z
                    own.qx, own.qy, own.qz, own.qw = o.x, o.y, o.z, o.w
                    if changed:
                        self.fs.update_own_robot(own)
                    if not self._amcl_initialized:
                        self._amcl_initialized = True
                        self.get_logger().info(
                            f'[{self.robot_id}] TF localization active at ({p.x:.2f}, {p.y:.2f}) — state publishing enabled.'
                        )
        except Exception:
            pass

    def _odom_cb(self, msg: Odometry):
        own = self.fs.get_own_state()
        if own is None:
            return
        v = msg.twist.twist
        own.vx, own.vy = v.linear.x, v.linear.y

    def _amcl_cb(self, msg: PoseWithCovarianceStamped):
        own = self.fs.get_own_state()
        if own is None:
            return
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        changed = (abs(own.x - p.x) > 0.01 or abs(own.y - p.y) > 0.01)
        own.x, own.y, own.z = p.x, p.y, p.z
        own.qx, own.qy, own.qz, own.qw = o.x, o.y, o.z, o.w
        if changed:
            self.fs.update_own_robot(own)
        if not self._amcl_initialized:
            self._amcl_initialized = True
            self.get_logger().info(
                f'[{self.robot_id}] AMCL initialised at ({p.x:.2f}, {p.y:.2f}) — state publishing enabled.')

    # ──────────────────────────────────────────────────────────────────────────
    # Task Pool Seeding
    # ──────────────────────────────────────────────────────────────────────────

    def _task_pool_cb(self, msg: TaskPool):
        own = self.fs.get_own_state()
        cur_x = own.x if own else 0.0
        cur_y = own.y if own else 0.0
        dist_to_broadcaster = math.hypot(cur_x - self.broadcaster_x, cur_y - self.broadcaster_y)
        if dist_to_broadcaster > self.comm_radius:
            return
        for task in msg.tasks:
            self._ingest_task_ros(task)

    def _task_event_cb(self, task: Task):
        # `/fleet/*` is globally observable in simulation.  Treat it as a
        # broadcaster source only; task execution updates beyond radio range
        # arrive through the local or P2P paths instead.
        own = self.fs.get_own_state()
        cur_x = own.x if own else 0.0
        cur_y = own.y if own else 0.0
        if math.hypot(cur_x - self.broadcaster_x, cur_y - self.broadcaster_y) > self.comm_radius:
            return
        self._ingest_task_ros(task)

    def _local_task_event_cb(self, task: Task):
        """Accept this robot's Nav2 lifecycle event at any dock distance."""
        self._ingest_task_ros(task)

    def _ingest_task_ros(self, task: Task):
        entry = TaskEntry(
            task_id=task.task_id,
            state=task.state,
            assigned_robot_id=task.assigned_robot_id,
            version=task.version,
            priority=task.priority,
            pickup_x=task.pickup_pose.pose.position.x,
            pickup_y=task.pickup_pose.pose.position.y,
            dropoff_x=task.dropoff_pose.pose.position.x,
            dropoff_y=task.dropoff_pose.pose.position.y,
            wall_time=time.time(),
            source_robot_id='task_broadcaster',
        )
        existing = self.fs.get_task(task.task_id)
        if existing is None or get_state_rank(task.state) > get_state_rank(existing.state):
            self.fs.tick()
            entry.lamport_clock = self.fs.clock
            self.fs.merge_task(entry)

    def _on_p2p_task_pool(self, msg: P2PMessage):
        """Receives task pool broadcast routed over physical wireless channel."""
        try:
            payload = json.loads(msg.payload)
            tasks = payload.get('tasks', [])
        except Exception as e:
            self.get_logger().warn(f'[{self.robot_id}] Failed to parse P2P TASK_POOL: {e}')
            return

        for t_dict in tasks:
            entry = TaskEntry(
                task_id=t_dict['task_id'],
                state=t_dict.get('state', Task.STATE_AVAILABLE),
                assigned_robot_id=t_dict.get('assigned_robot_id', ''),
                version=t_dict.get('version', 1),
                priority=t_dict.get('priority', 1),
                pickup_x=float(t_dict['pickup_x']),
                pickup_y=float(t_dict['pickup_y']),
                dropoff_x=float(t_dict['dropoff_x']),
                dropoff_y=float(t_dict['dropoff_y']),
                wall_time=time.time(),
                source_robot_id='dock_station',
            )
            existing = self.fs.get_task(entry.task_id)
            if existing is None or get_state_rank(entry.state) > get_state_rank(existing.state):
                self.fs.tick()
                entry.lamport_clock = self.fs.clock
                self.fs.merge_task(entry)
                self.get_logger().info(
                    f'[{self.robot_id}] Ingested task {entry.task_id} via Dock Station P2P broadcast'
                )

    # ──────────────────────────────────────────────────────────────────────────
    # P2P Handlers — Receive
    # ──────────────────────────────────────────────────────────────────────────

    def _on_state_beacon(self, msg: P2PMessage):
        try:
            d = json.loads(msg.payload)
            entry = self.fs.robot_entry_from_dict(d)
        except Exception:
            return
        changed = self.fs.merge_robot(entry)
        if changed:
            self.get_logger().debug(
                f'[{self.robot_id}] Merged robot state from {entry.robot_id} '
                f'(clock: {entry.lamport_clock}, pos: ({entry.x:.1f}, {entry.y:.1f}))'
            )

    def _on_task_status(self, msg: P2PMessage):
        try:
            d = json.loads(msg.payload)
            entry = self.fs.task_entry_from_dict(d)
        except Exception:
            return
        # Patch missing geometry from local knowledge (sender may have omitted zeros)
        existing = self.fs.get_task(entry.task_id)
        if existing:
            if entry.pickup_x == 0.0 and entry.pickup_y == 0.0:
                entry.pickup_x = existing.pickup_x
                entry.pickup_y = existing.pickup_y
            if entry.dropoff_x == 0.0 and entry.dropoff_y == 0.0:
                entry.dropoff_x = existing.dropoff_x
                entry.dropoff_y = existing.dropoff_y
            if not entry.source_robot_id:
                entry.source_robot_id = msg.source_robot_id or existing.source_robot_id
            if entry.lamport_clock == 0:
                entry.lamport_clock = max(existing.lamport_clock + 1, self.fs.clock)
        changed = self.fs.merge_task(entry)
        if changed:
            self.get_logger().debug(
                f'[{self.robot_id}] [CRDT] Merged TASK_STATUS for {entry.task_id} '
                f'state={entry.state} lamport={entry.lamport_clock}'
            )
            # ── Multi-hop gossip relay (Root Cause 3 fix) ──────────────────
            # Re-broadcast this state update to all reachable peers so the
            # information can cross radio partitions through intermediate nodes.
            # Use a (task_id, lamport_clock) dedup key to break relay loops.
            relay_key = (entry.task_id, entry.lamport_clock)
            if relay_key not in self._relayed_task_ops:
                self._relayed_task_ops.add(relay_key)
                # Bound the set size to prevent unbounded memory growth.
                if len(self._relayed_task_ops) > 2000:
                    self._relayed_task_ops = set(list(self._relayed_task_ops)[-1000:])
                updated = self.fs.get_task(entry.task_id)
                if updated:
                    self.get_logger().info(
                        f'[{self.robot_id}] [GOSSIP] Relaying TASK_STATUS {entry.task_id} '
                        f'state={entry.state} L={entry.lamport_clock} from {msg.source_robot_id}'
                    )
                    self.p2p.broadcast('TASK_STATUS', self.fs.task_entry_to_dict(updated))


    def _on_sync_request(self, msg: P2PMessage):
        """Peer is requesting a full world-view dump (reconnection)."""
        peer_id = msg.source_robot_id
        self.get_logger().info(
            f'[{self.robot_id}] STATE_SYNC_REQUEST from {peer_id} — sending full WorldView + CRDT op log'
        )
        # world_view_to_dict() now includes 'ops_tasks' and 'ops_robots'
        wv = self.fs.world_view_to_dict()
        self.p2p.send_to(peer_id, 'STATE_SYNC_RESPONSE', wv)

    def _on_sync_response(self, msg: P2PMessage):
        """Received a full world-view dump from a reconnected peer."""
        peer_id = msg.source_robot_id
        try:
            d = json.loads(msg.payload)
            # world_view_from_dict returns 5-tuple in Phase 10.1
            robots, tasks, peer_clock, ops_tasks, ops_robots = self.fs.world_view_from_dict(d)
        except Exception as e:
            self.get_logger().warn(f'[{self.robot_id}] Failed to parse sync response: {e}')
            return
        result = self.fs.merge_world_view(
            robots, tasks,
            ops_tasks=ops_tasks,
            ops_robots=ops_robots,
        )
        self.get_logger().info(
            f'[{self.robot_id}] [CRDT] Reconnect merge completed from {peer_id} | '
            f'+{result["robot_updates"]} robots, +{result["task_updates"]} tasks | '
            f'Lamport clock now: {self.fs.clock}'
        )
        if result.get('allocation_changed'):
            self.get_logger().info(
                f'[{self.robot_id}] [CRDT] Allocation-relevant state changed after reconnect sync'
            )

    def _on_crdt_ops(self, msg: P2PMessage):
        """Receives incremental CRDT op-log delta from a peer."""
        try:
            d = json.loads(msg.payload)
        except Exception:
            return
        ops_tasks  = d.get('ops_tasks', [])
        ops_robots = d.get('ops_robots', [])
        if ops_tasks or ops_robots:
            result = self.fs.merge_world_view(
                [], [],
                ops_tasks=ops_tasks,
                ops_robots=ops_robots,
            )
            if result.get('task_updates', 0) > 0:
                self.get_logger().debug(
                    f'[{self.robot_id}] [CRDT] Incremental ops from {msg.source_robot_id}: '
                    f'+{result["task_updates"]} task ops'
                )

    # ──────────────────────────────────────────────────────────────────────────
    # P2P Event Handler — Reconnection
    # ──────────────────────────────────────────────────────────────────────────

    def _p2p_event_cb(self, msg: P2PEvent):
        if msg.event_type == P2PEvent.EVENT_CONNECTED:
            peer_id = msg.peer_id
            self.get_logger().info(
                f'[{self.robot_id}] Reconnected to {peer_id} (dist: {msg.distance:.1f}m) '
                f'— requesting state sync'
            )
            # Send sync request so peer replies with its world view
            self.p2p.send_to(peer_id, 'STATE_SYNC_REQUEST', {})

    # ──────────────────────────────────────────────────────────────────────────
    # P2P Broadcast — Own State Beacon
    # ──────────────────────────────────────────────────────────────────────────

    def _broadcast_state_beacon(self):
        self._update_pose_from_tf()
        if not self._amcl_initialized:
            return
        own = self.fs.get_own_state()
        if own is None:
            return
        self.p2p.broadcast('STATE_BEACON', self.fs.robot_entry_to_dict(own))

    def broadcast_task_status(self, task_id: str):
        """Called by task_manager when a task state changes locally."""
        entry = self.fs.get_task(task_id)
        if entry:
            self.p2p.broadcast('TASK_STATUS', self.fs.task_entry_to_dict(entry))

    # ──────────────────────────────────────────────────────────────────────────
    # Local Publishers
    # ──────────────────────────────────────────────────────────────────────────

    def _publish_local_state(self):
        self._update_pose_from_tf()
        if not self._amcl_initialized:
            return
        own = self.fs.get_own_state()
        if own is None:
            return
        msg = RobotState()
        msg.robot_id = self.robot_id
        msg.current_pose.header.frame_id = 'map'
        msg.current_pose.header.stamp = self.get_clock().now().to_msg()
        msg.current_pose.pose.position.x = own.x
        msg.current_pose.pose.position.y = own.y
        msg.current_pose.pose.position.z = own.z
        msg.current_pose.pose.orientation.x = own.qx
        msg.current_pose.pose.orientation.y = own.qy
        msg.current_pose.pose.orientation.z = own.qz
        msg.current_pose.pose.orientation.w = own.qw
        msg.current_velocity.linear.x = own.vx
        msg.current_velocity.linear.y = own.vy
        msg.battery_level = float(own.battery)
        msg.status = own.status
        msg.current_task_id = own.task_id
        self.state_pub.publish(msg)
        self.fleet_state_pub.publish(msg)

    def _publish_world_view(self):
        if not self._amcl_initialized:
            return
        now = self.get_clock().now().to_msg()
        msg = WorldView()
        msg.robot_id = self.robot_id
        msg.lamport_clock = self.fs.clock
        msg.timestamp = now

        own = self.fs.get_own_state()
        if own:
            msg.own_state = self._robot_entry_to_ros(own, now)

        for r in self.fs.get_peer_robots():
            msg.peer_states.append(self._robot_entry_to_ros(r, now))

        for t in self.fs.get_all_tasks():
            msg.task_states.append(self._task_entry_to_ros(t, now))

        self.world_view_pub.publish(msg)
        self.fleet_world_view_pub.publish(msg)

    def _robot_entry_to_ros(self, e: RobotEntry, now) -> RobotWorldEntry:
        rwe = RobotWorldEntry()
        rwe.robot_id = e.robot_id
        rwe.current_pose.header.frame_id = 'map'
        rwe.current_pose.header.stamp = now
        rwe.current_pose.pose.position.x = e.x
        rwe.current_pose.pose.position.y = e.y
        rwe.current_pose.pose.position.z = e.z
        rwe.current_pose.pose.orientation.x = e.qx
        rwe.current_pose.pose.orientation.y = e.qy
        rwe.current_pose.pose.orientation.z = e.qz
        rwe.current_pose.pose.orientation.w = e.qw
        rwe.current_velocity.linear.x = e.vx
        rwe.current_velocity.linear.y = e.vy
        rwe.battery_level = float(e.battery)
        rwe.status = e.status
        rwe.current_task_id = e.task_id
        rwe.current_task_state = e.task_state
        rwe.destination.header.frame_id = 'map'
        rwe.destination.pose.position.x = e.dest_x
        rwe.destination.pose.position.y = e.dest_y
        rwe.lamport_clock = e.lamport_clock
        rwe.source_robot_id = e.source_robot_id
        # wall_time as sec/nanosec
        sec = int(e.wall_time)
        rwe.wall_time.sec = sec
        rwe.wall_time.nanosec = int((e.wall_time - sec) * 1e9)
        return rwe

    def _task_entry_to_ros(self, e: TaskEntry, now) -> TaskWorldEntry:
        twe = TaskWorldEntry()
        twe.task_id = e.task_id
        twe.state = e.state
        twe.assigned_robot_id = e.assigned_robot_id
        twe.version = e.version
        twe.priority = e.priority
        twe.pickup_pose.header.frame_id = 'map'
        twe.pickup_pose.header.stamp = now
        twe.pickup_pose.pose.position.x = e.pickup_x
        twe.pickup_pose.pose.position.y = e.pickup_y
        twe.dropoff_pose.header.frame_id = 'map'
        twe.dropoff_pose.header.stamp = now
        twe.dropoff_pose.pose.position.x = e.dropoff_x
        twe.dropoff_pose.pose.position.y = e.dropoff_y
        twe.lamport_clock = e.lamport_clock
        twe.source_robot_id = e.source_robot_id
        sec = int(e.wall_time)
        twe.wall_time.sec = sec
        twe.wall_time.nanosec = int((e.wall_time - sec) * 1e9)
        return twe

    # ──────────────────────────────────────────────────────────────────────────
    # Periodic World-View Summary Log
    # ──────────────────────────────────────────────────────────────────────────

    def _log_world_summary(self):
        robots = self.fs.get_all_robots()
        tasks = self.fs.get_all_tasks()
        robot_lines = [
            f'  {r.robot_id}: ({r.x:.1f}, {r.y:.1f}) | '
            f'status={r.status} | task={r.task_id or "none"} | L={r.lamport_clock}'
            for r in robots
        ]
        task_lines = [
            f'  {t.task_id}: state={t.state} | '
            f'assigned={t.assigned_robot_id or "none"} | L={t.lamport_clock}'
            for t in tasks
        ]
        self.get_logger().info(
            f'[{self.robot_id}] WorldView (Lamport L={self.fs.clock}) | '
            f'{len(robots)} robots, {len(tasks)} tasks\n'
            + ('\n'.join(robot_lines) or '  (none)') + '\n'
            + ('  Tasks:\n' + '\n'.join(task_lines) if task_lines else '  Tasks: (none)')
        )


def main(args=None):
    rclpy.init(args=args)
    node = StateManager()
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
