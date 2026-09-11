#!/usr/bin/env python3
"""
Max-Sum Allocator Node — Phase 5 Implementation
------------------------------------------------
Runs identically on each robot.
Performs decentralized multi-robot task allocation using Factor Graph Max-Sum.

Responsibilities:
1. Subscribes to /world_view (from State Manager) to get live fleet poses and task states.
2. Interacts with P2P transport (via P2PClient) for exchanging Max-Sum messages.
3. Solves task ownership via MaxSumSolver.
4. Publishes TaskOwnership on local 'task_ownership' and '/fleet/task_ownership'.
5. Provides extensive debug logging: iteration, sender, receiver, task, utility, belief, owner.
"""

import json
import rclpy
from rclpy.node import Node
from typing import List, Optional, Set

from fleet_interfaces.msg import (
    MaxSumMessage,
    P2PNeighborList,
    TaskOwnership,
    WorldView,
    Task,
    TaskState,
)
from fleet_manager.p2p_client import P2PClient
from fleet_manager.factor_graph import (
    RobotInfo,
    TaskInfo,
    UtilityWeights,
)
from fleet_manager.maxsum_engine import (
    MaxSumConfig,
    MaxSumSolver,
    run_multi_round_maxsum_allocation,
)


class MaxSumAllocator(Node):
    def __init__(self):
        super().__init__('maxsum_allocator')

        default_id = self.get_namespace().strip('/') or 'robot_1'
        self.declare_parameter('robot_id', default_id)
        self.declare_parameter('max_iterations', 20)
        self.declare_parameter('convergence_threshold', 1e-3)
        self.declare_parameter('allocation_interval', 4.0)  # Run allocation every N seconds
        self.declare_parameter('weight_distance', 0.35)
        self.declare_parameter('weight_time', 0.25)
        self.declare_parameter('weight_battery', 0.25)
        self.declare_parameter('weight_workload', 0.15)

        self.robot_id = self.get_parameter('robot_id').get_parameter_value().string_value
        max_iter = self.get_parameter('max_iterations').get_parameter_value().integer_value
        conv_thresh = self.get_parameter('convergence_threshold').get_parameter_value().double_value

        self.config = MaxSumConfig(
            max_iterations=max_iter,
            convergence_threshold=conv_thresh,
            damping_factor=0.4,
            verbose=True,
        )

        self.weights = UtilityWeights(
            wd=self.get_parameter('weight_distance').get_parameter_value().double_value,
            wt=self.get_parameter('weight_time').get_parameter_value().double_value,
            wb=self.get_parameter('weight_battery').get_parameter_value().double_value,
            wl=self.get_parameter('weight_workload').get_parameter_value().double_value,
        )

        # Initialize P2P Client
        self.p2p = P2PClient(node=self)
        self.p2p.register_handler('MAXSUM_MESSAGE', self._handle_p2p_maxsum)

        # Subscriptions
        self.world_view_sub = self.create_subscription(
            WorldView, 'world_view', self._handle_world_view, 10
        )
        # Track current reachable peers from P2P transport
        self.reachable_peers: Set[str] = set()
        self.neighbors_sub = self.create_subscription(
            P2PNeighborList, 'p2p_neighbors', self._handle_neighbors, 10
        )

        # Publishers
        self.ownership_pub = self.create_publisher(TaskOwnership, 'task_ownership', 10)
        self.fleet_ownership_pub = self.create_publisher(TaskOwnership, '/fleet/task_ownership', 10)
        self.maxsum_msg_pub = self.create_publisher(MaxSumMessage, 'maxsum_messages', 20)

        # State storage
        self.latest_world_view: Optional[WorldView] = None
        self.my_owned_tasks: List[str] = []
        self.last_allocation_ownership: dict = {}

        # Periodic allocation timer
        interval = self.get_parameter('allocation_interval').get_parameter_value().double_value
        self.timer = self.create_timer(interval, self.trigger_allocation_cycle)

        self.get_logger().info(
            f'[{self.robot_id}] Max-Sum Allocator initialized. '
            f'MaxIter={max_iter}, Threshold={conv_thresh}, Interval={interval}s'
        )

    def _handle_world_view(self, msg: WorldView):
        """Stores the latest distributed fleet world view."""
        self.latest_world_view = msg

    def _handle_neighbors(self, msg: P2PNeighborList):
        """Tracks which peers are currently in communication range."""
        self.reachable_peers = set(msg.reachable_peers)
        self.get_logger().debug(
            f'[{self.robot_id}] Reachable peers: {self.reachable_peers}'
        )

    def _handle_p2p_maxsum(self, msg):
        """Processes incoming Max-Sum messages delivered over P2P RF channel."""
        self.get_logger().debug(
            f'[{self.robot_id}] Inbound Max-Sum packet from {msg.source_robot_id}: {msg.message_type}'
        )

    def _log_solver(self, text: str):
        self.get_logger().info(f'[{self.robot_id}.maxsum] {text}')

    def trigger_allocation_cycle(self):
        """
        Runs one Max-Sum task allocation cycle.

        Key constraints:
        - Only robots reachable via P2P (in comms range) are included.
          A late-joining robot will only see itself until it connects.
        - Only STATE_AVAILABLE tasks are bid on.
        - Tasks already owned by this robot are skipped (no double-bidding).
        """
        if not self.latest_world_view:
            return

        # Guard: world view not yet populated (own state missing robot_id)
        own = self.latest_world_view.own_state
        if not own.robot_id:
            return

        # 1. Collect robots: self + peers that are currently IN comms range
        robots_info = []
        robots_info.append(RobotInfo(
            robot_id=own.robot_id,
            x=own.current_pose.pose.position.x,
            y=own.current_pose.pose.position.y,
            battery_level=own.battery_level,
            current_workload=len(self.my_owned_tasks),
        ))

        for peer in self.latest_world_view.peer_states:
            # Only include peers that are currently in comms range.
            # If reachable_peers is empty (P2P not yet initialised or truly
            # isolated), fall back to including all known peers so the robot
            # can still allocate tasks to itself on first boot.
            if self.reachable_peers and peer.robot_id not in self.reachable_peers:
                self.get_logger().debug(
                    f'[{self.robot_id}] Skipping out-of-range peer {peer.robot_id} in Max-Sum'
                )
                continue
            robots_info.append(RobotInfo(
                robot_id=peer.robot_id,
                x=peer.current_pose.pose.position.x,
                y=peer.current_pose.pose.position.y,
                battery_level=peer.battery_level,
                current_workload=0,
            ))

        # 2. Collect only AVAILABLE (unallocated) tasks — skip anything already claimed.
        # Tasks become ALLOCATED the moment a robot wins ownership and broadcasts
        # TASK_STATUS, so peers won't double-bid in the next cycle.
        available_tasks = []
        for t in self.latest_world_view.task_states:
            if t.state != Task.STATE_AVAILABLE:
                continue  # Already allocated / in-progress / completed
            if t.task_id in self.my_owned_tasks:
                continue  # This robot already owns it; no need to re-bid
            available_tasks.append(TaskInfo(
                task_id=t.task_id,
                pickup_x=t.pickup_pose.pose.position.x,
                pickup_y=t.pickup_pose.pose.position.y,
                delivery_x=t.dropoff_pose.pose.position.x,
                delivery_y=t.dropoff_pose.pose.position.y,
                priority=t.priority,
            ))


        if not available_tasks:
            return

        self.get_logger().info(
            f'[{self.robot_id}] ──► Max-Sum cycle | '
            f'{len(available_tasks)} available tasks | '
            f'{len(robots_info)} robots in range (peers: {[r.robot_id for r in robots_info[1:]]})'
        )

        # 3. Run multi-round Max-Sum allocation
        ownership = run_multi_round_maxsum_allocation(
            robots=robots_info,
            tasks=available_tasks,
            weights=self.weights,
            config=self.config,
            log_callback=self._log_solver
        )

        self.last_allocation_ownership = ownership

        # 4. Check if any new tasks were assigned to THIS robot
        # Build a quick lookup of task info from available_tasks for the broadcast
        task_info_map = {t.task_id: t for t in available_tasks}

        for task_id, owner_robot in ownership.items():
            if owner_robot == self.robot_id and task_id not in self.my_owned_tasks:
                self.my_owned_tasks.append(task_id)
                self.get_logger().info(f'[{self.robot_id}] ★ WON TASK OWNERSHIP: {task_id} ►► {self.robot_id}')

                # Publish ownership (local + fleet)
                msg = TaskOwnership()
                msg.task_id = task_id
                msg.robot_id = self.robot_id
                msg.status = TaskOwnership.STATUS_CLAIMED
                msg.timestamp = self.get_clock().now().to_msg()
                self.ownership_pub.publish(msg)
                self.fleet_ownership_pub.publish(msg)

                # Immediately broadcast ALLOCATED state over P2P so peers'
                # world views are updated before their next allocation cycle,
                # preventing them from double-bidding on this task.
                self.p2p.broadcast('TASK_OWNERSHIP', payload={
                    'task_id': task_id,
                    'robot_id': self.robot_id,
                    'timestamp': self.get_clock().now().nanoseconds
                })
                # Also broadcast TASK_STATUS with ALLOCATED so peers' FleetState
                # marks this task as non-available in their next cycle.
                t_info = task_info_map.get(task_id)
                self.p2p.broadcast('TASK_STATUS', payload={
                    'task_id': task_id,
                    'state': Task.STATE_ALLOCATED,
                    'assigned_robot_id': self.robot_id,
                    'version': 1,
                    'priority': t_info.priority if t_info else 1,
                    'pickup_x': t_info.pickup_x if t_info else 0.0,
                    'pickup_y': t_info.pickup_y if t_info else 0.0,
                    'dropoff_x': t_info.delivery_x if t_info else 0.0,
                    'dropoff_y': t_info.delivery_y if t_info else 0.0,
                    'lamport_clock': 0,
                    'wall_time': 0.0,
                    'source_robot_id': self.robot_id,
                })


def main(args=None):
    rclpy.init(args=args)
    node = MaxSumAllocator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
