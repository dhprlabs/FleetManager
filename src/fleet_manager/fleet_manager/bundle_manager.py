#!/usr/bin/env python3
"""
Bundle Manager Node — Phase 6 Implementation
---------------------------------------------
Runs identically on each robot.
Converts an unordered set of owned tasks from Max-Sum into an optimal,
ordered execution queue based on incremental route cost:
  - Current robot pose
  - Pickup position
  - Dropoff position
  - Distance & transition legs
  - Estimated execution time
  - Existing bundle route (locking in-progress tasks)

Outputs an ordered queue:
  - current_task: string
  - remaining_tasks: string[]
  - task_ids: string[] (current_task + remaining_tasks)

Dynamically updates when:
  1. A new ownership result is received from Max-Sum.
  2. A task is completed by the Task Execution Manager.
"""

import itertools
import math
import time
from typing import Dict, List, Optional, Set, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from fleet_interfaces.msg import (
    Bundle,
    RobotState,
    Task,
    TaskOwnership,
    TaskPool,
    WorldView,
)


def compute_route_cost(
    start_x: float,
    start_y: float,
    task_order: List[str],
    task_details: Dict[str, Dict],
    nominal_speed: float = 0.5,
    service_time: float = 2.0,
) -> Tuple[float, float]:
    """
    Computes total distance and estimated execution time for an ordered list of tasks.
    Route: (start) ──► Pickup_1 ──► Dropoff_1 ──► Pickup_2 ──► Dropoff_2 ...
    """
    if not task_order:
        return 0.0, 0.0

    total_dist = 0.0
    curr_x, curr_y = start_x, start_y

    for t_id in task_order:
        if t_id not in task_details:
            continue
        t = task_details[t_id]
        px, py = t['pickup']
        dx, dy = t['dropoff']

        # Transition leg from current/previous dropoff to next pickup
        trans_dist = math.hypot(px - curr_x, py - curr_y)
        # Service trip leg from pickup to dropoff
        trip_dist = math.hypot(dx - px, dy - py)

        total_dist += (trans_dist + trip_dist)
        curr_x, curr_y = dx, dy

    # Estimated execution time: travel time + service dwell time per stop (2 stops per task)
    est_time = (total_dist / nominal_speed) + (len(task_order) * 2 * service_time)
    return total_dist, est_time


def optimize_task_sequence(
    start_x: float,
    start_y: float,
    tasks_to_order: List[str],
    task_details: Dict[str, Dict],
    nominal_speed: float = 0.5,
    service_time: float = 2.0,
) -> Tuple[List[str], float, float]:
    """
    Finds the optimal permutation of tasks minimizing incremental route cost.
    Uses exact permutation search for small sets (<= 7 tasks) and
    cheapest insertion heuristic for larger sets.
    """
    if not tasks_to_order:
        return [], 0.0, 0.0

    if len(tasks_to_order) <= 7:
        best_seq = list(tasks_to_order)
        best_dist, best_time = compute_route_cost(
            start_x, start_y, best_seq, task_details, nominal_speed, service_time
        )

        for perm in itertools.permutations(tasks_to_order):
            perm_list = list(perm)
            d, t = compute_route_cost(
                start_x, start_y, perm_list, task_details, nominal_speed, service_time
            )
            if d < best_dist:
                best_dist = d
                best_time = t
                best_seq = perm_list

        return best_seq, best_dist, best_time
    else:
        # Cheapest insertion heuristic
        ordered = [tasks_to_order[0]]
        remaining = set(tasks_to_order[1:])
        while remaining:
            best_cand = None
            best_pos = 0
            best_cost = float('inf')
            for cand in remaining:
                for pos in range(len(ordered) + 1):
                    test_seq = ordered[:pos] + [cand] + ordered[pos:]
                    d, _ = compute_route_cost(
                        start_x, start_y, test_seq, task_details, nominal_speed, service_time
                    )
                    if d < best_cost:
                        best_cost = d
                        best_cand = cand
                        best_pos = pos
            ordered.insert(best_pos, best_cand)
            remaining.remove(best_cand)

        d, t = compute_route_cost(
            start_x, start_y, ordered, task_details, nominal_speed, service_time
        )
        return ordered, d, t


class BundleManager(Node):
    def __init__(self):
        super().__init__('bundle_manager')

        default_id = self.get_namespace().strip('/') or 'robot_1'
        self.declare_parameter('robot_id', default_id)
        self.robot_id = self.get_parameter('robot_id').get_parameter_value().string_value

        self.declare_parameter('nominal_speed', 0.5)
        self.declare_parameter('service_time', 2.0)
        self.nominal_speed = self.get_parameter('nominal_speed').get_parameter_value().double_value
        self.service_time = self.get_parameter('service_time').get_parameter_value().double_value

        # Queue State
        self.owned_tasks: Set[str] = set()
        self.completed_tasks: Set[str] = set()
        self.current_task: str = ""
        self.remaining_tasks: List[str] = []
        self.task_details: Dict[str, Dict] = {}

        # Live Robot Pose
        self.current_x: float = 0.0
        self.current_y: float = 0.0
        self.total_estimated_cost: float = 0.0

        # Subscriptions
        self.ownership_sub = self.create_subscription(
            TaskOwnership, 'task_ownership', self._handle_ownership, 10
        )
        self.fleet_ownership_sub = self.create_subscription(
            TaskOwnership, '/fleet/task_ownership', self._handle_ownership, 10
        )
        self.world_view_sub = self.create_subscription(
            WorldView, 'world_view', self._handle_world_view, 10
        )
        self.robot_state_sub = self.create_subscription(
            RobotState, 'robot_state', self._handle_robot_state, 10
        )
        self.task_events_sub = self.create_subscription(
            Task, '/fleet/task_events', self._handle_task_event, 10
        )

        latching_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.task_pool_sub = self.create_subscription(
            TaskPool, '/fleet/task_pool', self._handle_task_pool, latching_qos
        )

        # Publishers
        self.bundle_pub = self.create_publisher(Bundle, 'bundle', 10)
        self.fleet_bundle_pub = self.create_publisher(Bundle, '/fleet/bundles', 10)

        # Periodic refresh timer (1 Hz)
        self.timer = self.create_timer(1.0, self.publish_bundle)

        self.get_logger().info(
            f'[{self.robot_id}] Local Bundle Manager initialized | '
            f'Speed: {self.nominal_speed} m/s | Dwell: {self.service_time}s'
        )

    # ──────────────────────────────────────────────────────────────────────────
    # Inbound Event Handlers
    # ──────────────────────────────────────────────────────────────────────────

    def _handle_robot_state(self, msg: RobotState):
        self.current_x = msg.current_pose.pose.position.x
        self.current_y = msg.current_pose.pose.position.y

    def _handle_task_pool(self, msg: TaskPool):
        for t in msg.tasks:
            self._store_task_detail(t)

    def _handle_world_view(self, msg: WorldView):
        # Update live pose
        self.current_x = msg.own_state.current_pose.pose.position.x
        self.current_y = msg.own_state.current_pose.pose.position.y

        # Update task details
        for t in msg.task_states:
            self.task_details[t.task_id] = {
                'pickup': (t.pickup_pose.pose.position.x, t.pickup_pose.pose.position.y),
                'dropoff': (t.dropoff_pose.pose.position.x, t.dropoff_pose.pose.position.y),
                'priority': t.priority,
                'state': t.state,
            }
            if t.state == Task.STATE_COMPLETED and t.task_id in self.owned_tasks:
                self.handle_task_completed(t.task_id)

    def _store_task_detail(self, t: Task):
        self.task_details[t.task_id] = {
            'pickup': (t.pickup_pose.pose.position.x, t.pickup_pose.pose.position.y),
            'dropoff': (t.dropoff_pose.pose.position.x, t.dropoff_pose.pose.position.y),
            'priority': t.priority,
            'state': t.state,
        }

    def _handle_ownership(self, msg: TaskOwnership):
        """Processes ownership won from Max-Sum."""
        if msg.robot_id == self.robot_id:
            if msg.status in (TaskOwnership.STATUS_CLAIMED, TaskOwnership.STATUS_CONFIRMED):
                if msg.task_id not in self.owned_tasks and msg.task_id not in self.completed_tasks:
                    self.owned_tasks.add(msg.task_id)
                    self.get_logger().info(
                        f'[{self.robot_id}] Ownership received for {msg.task_id}. Re-ordering bundle...'
                    )
                    self.recompute_bundle()
            elif msg.status == TaskOwnership.STATUS_RELEASED:
                if msg.task_id in self.owned_tasks:
                    self.owned_tasks.remove(msg.task_id)
                    if self.current_task == msg.task_id:
                        self.current_task = ""
                    if msg.task_id in self.remaining_tasks:
                        self.remaining_tasks.remove(msg.task_id)
                    self.recompute_bundle()

    def _handle_task_event(self, task: Task):
        """Watches for task completions."""
        self._store_task_detail(task)
        if task.state == Task.STATE_COMPLETED and task.task_id in self.owned_tasks:
            self.handle_task_completed(task.task_id)

    def handle_task_completed(self, task_id: str):
        """Called when Task Execution Manager marks a task completed."""
        if task_id in self.completed_tasks and task_id not in self.owned_tasks:
            return

        self.get_logger().info(f'[{self.robot_id}] Task {task_id} COMPLETED. Advancing bundle queue...')
        self.completed_tasks.add(task_id)
        self.owned_tasks.discard(task_id)

        if self.current_task == task_id:
            self.current_task = ""

        if task_id in self.remaining_tasks:
            self.remaining_tasks.remove(task_id)

        self.recompute_bundle()

    # ──────────────────────────────────────────────────────────────────────────
    # Bundle Recomputation & Incremental Route Cost Optimization
    # ──────────────────────────────────────────────────────────────────────────

    def recompute_bundle(self):
        """
        Converts the unordered set of owned tasks into an optimal ordered execution queue:
          current_task: head of queue (locked if already active)
          remaining_tasks: ordered by minimum incremental route cost
        """
        # Tasks needing execution
        unstarted = [
            t for t in self.owned_tasks
            if t != self.current_task and t not in self.completed_tasks
        ]

        # Determine start location for sequencing unstarted tasks
        if self.current_task and self.current_task in self.task_details:
            # Robot will be at the dropoff location of current_task when it becomes free
            start_x, start_y = self.task_details[self.current_task]['dropoff']
        else:
            # Robot starts from its current location
            start_x, start_y = self.current_x, self.current_y

        # Optimize the remaining tasks
        best_remaining, rem_dist, rem_time = optimize_task_sequence(
            start_x,
            start_y,
            unstarted,
            self.task_details,
            self.nominal_speed,
            self.service_time,
        )
        self.remaining_tasks = best_remaining

        # If no current task is active and remaining tasks exist, promote the first
        if not self.current_task and self.remaining_tasks:
            self.current_task = self.remaining_tasks.pop(0)

            # Re-sequence remaining tasks starting from the dropoff of the new current_task
            if self.current_task in self.task_details:
                c_dx, c_dy = self.task_details[self.current_task]['dropoff']
                self.remaining_tasks, _, _ = optimize_task_sequence(
                    c_dx,
                    c_dy,
                    self.remaining_tasks,
                    self.task_details,
                    self.nominal_speed,
                    self.service_time,
                )

        # Compute total route cost for the entire bundle (current + remaining)
        full_bundle = ([self.current_task] if self.current_task else []) + list(self.remaining_tasks)
        total_dist, total_time = compute_route_cost(
            self.current_x,
            self.current_y,
            full_bundle,
            self.task_details,
            self.nominal_speed,
            self.service_time,
        )
        self.total_estimated_cost = total_dist

        self.get_logger().info(
            f'[{self.robot_id}] ──► Bundle Updated: '
            f'current_task="{self.current_task}", '
            f'remaining={self.remaining_tasks} | '
            f'Total Dist: {total_dist:.2f}m, Est Time: {total_time:.1f}s'
        )
        self.publish_bundle()

    def publish_bundle(self):
        """Publishes the current bundle to local and fleet topics."""
        full_bundle = ([self.current_task] if self.current_task else []) + list(self.remaining_tasks)

        msg = Bundle()
        msg.robot_id = self.robot_id
        msg.current_task = self.current_task
        msg.remaining_tasks = list(self.remaining_tasks)
        msg.task_ids = full_bundle
        msg.total_estimated_cost = float(self.total_estimated_cost)
        msg.timestamp = self.get_clock().now().to_msg()

        self.bundle_pub.publish(msg)
        self.fleet_bundle_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BundleManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
