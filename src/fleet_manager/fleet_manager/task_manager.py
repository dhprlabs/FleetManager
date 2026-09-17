#!/usr/bin/env python3
"""
Task Manager Node
-----------------
Runs identically on each robot.
Receives task broadcasts from Task Broadcaster and P2P peers.
Integrates with the Distributed State Manager (FleetState) via the
/world_view topic to maintain a consistent global task picture.
Also propagates task state changes over P2P using TASK_STATUS messages.
"""

import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseWithCovarianceStamped
from fleet_interfaces.msg import Task, TaskPool, WorldView, P2PMessage, RobotState
from fleet_manager.fleet_state import FleetState, TaskEntry
from fleet_manager.p2p_client import P2PClient


_STATE_NAMES = {
    Task.STATE_AVAILABLE: 'AVAILABLE',
    Task.STATE_ALLOCATED: 'ALLOCATED',
    Task.STATE_IN_PROGRESS: 'IN_PROGRESS',
    Task.STATE_COMPLETED: 'COMPLETED',
    Task.STATE_FAILED: 'FAILED',
    Task.STATE_CANCELLED: 'CANCELLED',
}


class TaskManager(Node):
    def __init__(self):
        super().__init__('task_manager')

        default_id = self.get_namespace().strip('/') or 'robot_1'
        self.declare_parameter('robot_id', default_id)
        self.declare_parameter('dock_x', 5.0)
        self.declare_parameter('dock_y', 12.0)
        self.declare_parameter('broadcaster_x', 5.0)
        self.declare_parameter('broadcaster_y', 12.0)
        self.declare_parameter('communication_radius', 6.0)
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)

        self.robot_id = self.get_parameter('robot_id').get_parameter_value().string_value
        self.dock_x = self.get_parameter('dock_x').get_parameter_value().double_value
        self.dock_y = self.get_parameter('dock_y').get_parameter_value().double_value
        self.broadcaster_x = self.get_parameter('broadcaster_x').get_parameter_value().double_value
        self.broadcaster_y = self.get_parameter('broadcaster_y').get_parameter_value().double_value
        self.comm_radius = self.get_parameter('communication_radius').get_parameter_value().double_value
        init_x = self.get_parameter('initial_x').get_parameter_value().double_value
        init_y = self.get_parameter('initial_y').get_parameter_value().double_value

        self.current_x = init_x
        self.current_y = init_y
        self._has_pose = False  # Set True once robot_state or amcl_pose first arrives

        # Local distributed state store (tasks portion mirrors state_manager's)
        self.fs = FleetState(robot_id=self.robot_id)

        latching_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # ── Subscriptions ──────────────────────────────────────────────────
        self.create_subscription(RobotState, 'robot_state', self._handle_robot_state, 10)
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self._handle_amcl_pose, 10)
        self.create_subscription(TaskPool, '/fleet/task_pool', self._handle_task_pool, latching_qos)
        self.create_subscription(Task, '/fleet/task_events', self._handle_task_event, 10)

        # ── P2P Client ─────────────────────────────────────────────────────
        self.p2p = P2PClient(node=self)
        self.p2p.register_handler('TASK_STATUS', self._on_p2p_task_status)
        self.p2p.register_handler('TASK_POOL', self._on_p2p_task_pool)
        self.p2p.register_handler('TASK_OWNERSHIP', self._on_p2p_task_ownership)

        # ── Publishers ─────────────────────────────────────────────────────
        self.local_task_pub = self.create_publisher(Task, 'assigned_task', 10)

        self.get_logger().info(
            f'[{self.robot_id}] Task Manager initialized — distributed task tracking active.'
        )

    # ──────────────────────────────────────────────────────────────────────────
    # Ingest from Task Broadcaster
    # ──────────────────────────────────────────────────────────────────────────

    def _handle_robot_state(self, msg: RobotState):
        self.current_x = msg.current_pose.pose.position.x
        self.current_y = msg.current_pose.pose.position.y
        self._has_pose = True

    def _handle_amcl_pose(self, msg: PoseWithCovarianceStamped):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self._has_pose = True

    def _handle_task_pool(self, msg: TaskPool):
        # Enforce physical radio range to task broadcaster
        dist_to_broadcaster = math.hypot(self.current_x - self.broadcaster_x, self.current_y - self.broadcaster_y)
        if dist_to_broadcaster > self.comm_radius:
            return
        for task in msg.tasks:
            self._ingest_task(task, from_broadcaster=True)

    def _handle_task_event(self, task: Task):
        dist_to_broadcaster = math.hypot(self.current_x - self.broadcaster_x, self.current_y - self.broadcaster_y)
        if dist_to_broadcaster > self.comm_radius:
            return
        self._ingest_task(task, from_broadcaster=True)

    def _ingest_task(self, task: Task, from_broadcaster: bool = False):
        existing = self.fs.get_task(task.task_id)
        state_str = _STATE_NAMES.get(task.state, 'UNKNOWN')

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
            source_robot_id='task_broadcaster' if from_broadcaster else self.robot_id,
        )

        if existing is None:
            # New task: tick clock and log
            self.fs.tick()
            entry.lamport_clock = self.fs.clock
            self.fs.merge_task(entry)
            self.get_logger().info(
                f'[{self.robot_id}] Received task: {task.task_id} {state_str} '
                f'(L={entry.lamport_clock})'
            )
        elif existing.state != task.state:
            # State changed: merge with LWW and broadcast
            self.fs.tick()
            entry.lamport_clock = self.fs.clock
            changed = self.fs.merge_task(entry)
            if changed:
                self.get_logger().info(
                    f'[{self.robot_id}] Task {task.task_id} updated → {state_str} '
                    f'(L={entry.lamport_clock})'
                )
                # Propagate the state change to peers
                self.p2p.broadcast('TASK_STATUS', self.fs.task_entry_to_dict(entry))

    # ──────────────────────────────────────────────────────────────────────────
    # Ingest from P2P peers
    # ──────────────────────────────────────────────────────────────────────────

    def _on_p2p_task_status(self, msg: P2PMessage):
        try:
            d = json.loads(msg.payload)
            entry = self.fs.task_entry_from_dict(d)
        except Exception:
            return
        existing = self.fs.get_task(entry.task_id)
        changed = self.fs.merge_task(entry)
        if changed:
            state_str = _STATE_NAMES.get(entry.state, 'UNKNOWN')
            self.get_logger().info(
                f'[{self.robot_id}] P2P task update: {entry.task_id} → {state_str} '
                f'from {entry.source_robot_id} (L={entry.lamport_clock})'
            )

    def _on_p2p_task_pool(self, msg: P2PMessage):
        """Receives task pool broadcast over wireless medium from dock station."""
        try:
            payload = json.loads(msg.payload)
            tasks = payload.get('tasks', [])
        except Exception:
            return

        for t_dict in tasks:
            task = Task()
            task.task_id = t_dict['task_id']
            task.state = t_dict.get('state', Task.STATE_AVAILABLE)
            task.priority = t_dict.get('priority', 1)
            task.pickup_pose.header.frame_id = 'map'
            task.pickup_pose.pose.position.x = float(t_dict['pickup_x'])
            task.pickup_pose.pose.position.y = float(t_dict['pickup_y'])
            task.dropoff_pose.header.frame_id = 'map'
            task.dropoff_pose.pose.position.x = float(t_dict['dropoff_x'])
            task.dropoff_pose.pose.position.y = float(t_dict['dropoff_y'])
            task.assigned_robot_id = t_dict.get('assigned_robot_id', '')
            task.version = t_dict.get('version', 1)
            self._ingest_task(task, from_broadcaster=True)

    def _on_p2p_task_ownership(self, msg: P2PMessage):
        """Receives task ownership resolution from Max-Sum."""
        try:
            payload = json.loads(msg.payload)
            task_id = payload['task_id']
            owner_id = payload['robot_id']
        except Exception:
            return

        existing = self.fs.get_task(task_id)
        if existing:
            existing.assigned_robot_id = owner_id
            existing.state = Task.STATE_ALLOCATED
            self.fs.tick()
            existing.lamport_clock = self.fs.clock
            self.fs.merge_task(existing)

            self.get_logger().info(
                f'[{self.robot_id}] Task {task_id} allocated to {owner_id} (L={existing.lamport_clock})'
            )

            if owner_id == self.robot_id:
                # Publish to local assigned_task for Nav2 bridge / bundle manager
                t_msg = Task()
                t_msg.task_id = existing.task_id
                t_msg.state = Task.STATE_ALLOCATED
                t_msg.assigned_robot_id = self.robot_id
                t_msg.priority = existing.priority
                t_msg.pickup_pose.header.frame_id = 'map'
                t_msg.pickup_pose.pose.position.x = existing.pickup_x
                t_msg.pickup_pose.pose.position.y = existing.pickup_y
                t_msg.dropoff_pose.header.frame_id = 'map'
                t_msg.dropoff_pose.pose.position.x = existing.dropoff_x
                t_msg.dropoff_pose.pose.position.y = existing.dropoff_y
                self.local_task_pub.publish(t_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TaskManager()
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
