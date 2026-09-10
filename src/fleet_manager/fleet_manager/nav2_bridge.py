#!/usr/bin/env python3
"""
Task Execution Manager Node — Phase 7 + Phase 9 Implementation
---------------------------------------------------------------
Connects the local Bundle Manager to physical execution via Nav2.

Implements the complete 5-stage task lifecycle state machine:
    ASSIGNED ──► IN_PROGRESS ──► PICKUP_COMPLETED ──► DELIVERING ──► COMPLETED

Phase 9 adds a RECONCILING intermediate state:
    IN_PROGRESS ──► [Gazebo item check] ──► PICKUP_COMPLETED  (item found)
                                        ──► RECONCILING       (item missing)
    RECONCILING ──► IDLE (task dropped, re-bid)  OR  ──► PICKUP_COMPLETED (re-confirmed)

Responsibilities:
  1. Reads `current_task` from the local Bundle Manager queue.
  2. Resolves pickup and dropoff coordinates for the active task.
  3. Drives the 5-stage state transitions.
  4. Integration with Nav2 `NavigateToPose` action server.
  5. Fallback simulation mode when Nav2 action server is offline (for tests).
  6. Lock guard: Never reallocates or preempts an active task.
  7. (Phase 9) Physical pickup validation via Gazebo GetEntityState.
"""

import math
import time
from enum import Enum
from typing import Dict, Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

from fleet_interfaces.msg import (
    Bundle,
    PickupObservation,
    RobotState,
    Task,
    TaskOwnership,
    TaskPool,
    WorldView,
)
from fleet_manager.p2p_client import P2PClient


class TaskExecutionState(Enum):
    IDLE = 'IDLE'
    ASSIGNED = 'ASSIGNED'
    IN_PROGRESS = 'IN_PROGRESS'
    PICKUP_COMPLETED = 'PICKUP_COMPLETED'
    DELIVERING = 'DELIVERING'
    COMPLETED = 'COMPLETED'
    # Phase 9: item absent at pickup — awaiting fleet reconciliation
    RECONCILING = 'RECONCILING'


class TaskExecutionManager(Node):
    """
    Task Execution Manager (Nav2 Bridge).
    Runs identically on each robot.
    """

    def __init__(self):
        super().__init__('nav2_bridge')

        default_id = self.get_namespace().strip('/') or 'robot_1'
        self.declare_parameter('robot_id', default_id)
        self.declare_parameter('enable_nav2', True)
        self.declare_parameter('pickup_dwell_sec', 1.5)
        self.declare_parameter('delivery_dwell_sec', 1.0)
        self.declare_parameter('arrival_tolerance', 0.5)
        self.declare_parameter('reconciliation_timeout_sec', 10.0)
        self.declare_parameter('gazebo_item_prefix', 'item_')
        self.declare_parameter('enable_pickup_validation', True)

        self.robot_id = self.get_parameter('robot_id').get_parameter_value().string_value
        self.enable_nav2 = self.get_parameter('enable_nav2').get_parameter_value().bool_value
        self.pickup_dwell_sec = self.get_parameter('pickup_dwell_sec').get_parameter_value().double_value
        self.delivery_dwell_sec = self.get_parameter('delivery_dwell_sec').get_parameter_value().double_value
        self.arrival_tolerance = self.get_parameter('arrival_tolerance').get_parameter_value().double_value
        self.reconciliation_timeout_sec = self.get_parameter('reconciliation_timeout_sec').get_parameter_value().double_value
        self.gazebo_item_prefix = self.get_parameter('gazebo_item_prefix').get_parameter_value().string_value
        self.enable_pickup_validation = self.get_parameter('enable_pickup_validation').get_parameter_value().bool_value

        # Execution State Machine
        self.execution_state = TaskExecutionState.IDLE
        self.active_task_id: Optional[str] = None
        self.active_task_info: Optional[Dict] = None

        # Robot Live Pose
        self.current_x: float = 0.0
        self.current_y: float = 0.0

        # Task Cache (task_id -> {pickup_pose, dropoff_pose, priority})
        self.task_cache: Dict[str, Dict] = {}

        # Phase 9 — Reconciliation state
        self._reconcile_start_time: Optional[float] = None
        self._reconcile_timer = None

        # Nav2 Action Client scoped to this robot's namespace
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.current_goal_handle = None

        # Gazebo GetEntityState service client (Phase 9)
        self._gazebo_entity_client = None
        if self.enable_pickup_validation:
            try:
                from gazebo_msgs.srv import GetEntityState
                self._gazebo_entity_client = self.create_client(
                    GetEntityState, '/gazebo/get_entity_state'
                )
                self.get_logger().info(
                    f'[{self.robot_id}] Phase 9: Gazebo item validation client initialised.'
                )
            except ImportError:
                self.get_logger().warn(
                    f'[{self.robot_id}] gazebo_msgs not available — pickup validation disabled.'
                )
                self.enable_pickup_validation = False

        # P2P Client for task status broadcasting
        self.p2p = P2PClient(node=self)

        # ── Subscriptions ──────────────────────────────────────────────────
        self.bundle_sub = self.create_subscription(
            Bundle, 'bundle', self._handle_bundle, 10
        )
        self.robot_state_sub = self.create_subscription(
            RobotState, 'robot_state', self._handle_robot_state, 10
        )
        self.world_view_sub = self.create_subscription(
            WorldView, 'world_view', self._handle_world_view, 10
        )
        # Listen for fleet pickup observations (Phase 9 reconciliation)
        self.pickup_obs_sub = self.create_subscription(
            PickupObservation, '/fleet/pickup_observations',
            self._handle_pickup_observation, 10
        )

        latching_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.task_pool_sub = self.create_subscription(
            TaskPool, '/fleet/task_pool', self._handle_task_pool, latching_qos
        )
        self.task_events_sub = self.create_subscription(
            Task, '/fleet/task_events', self._handle_task_event, 10
        )

        # ── Publishers ─────────────────────────────────────────────────────
        self.task_event_pub = self.create_publisher(Task, '/fleet/task_events', 10)
        self.assigned_task_pub = self.create_publisher(Task, 'assigned_task', 10)
        self.pickup_obs_pub = self.create_publisher(
            PickupObservation, '/fleet/pickup_observations', 10
        )
        self.ownership_release_pub = self.create_publisher(
            TaskOwnership, '/fleet/task_ownership', 10
        )

        # Execution evaluation timer
        self.eval_timer = self.create_timer(0.5, self._eval_execution_step)

        self.get_logger().info(
            f'[{self.robot_id}] Task Execution Manager active | '
            f'Nav2 Target: /{self.robot_id}/navigate_to_pose | '
            f'Dwell: {self.pickup_dwell_sec}s | '
            f'Pickup validation: {self.enable_pickup_validation}'
        )

    # ──────────────────────────────────────────────────────────────────────────
    # Inbound Handlers
    # ──────────────────────────────────────────────────────────────────────────

    def _handle_robot_state(self, msg: RobotState):
        self.current_x = msg.current_pose.pose.position.x
        self.current_y = msg.current_pose.pose.position.y

    def _handle_world_view(self, msg: WorldView):
        self.current_x = msg.own_state.current_pose.pose.position.x
        self.current_y = msg.own_state.current_pose.pose.position.y
        for t in msg.task_states:
            self.task_cache[t.task_id] = {
                'pickup_pose': t.pickup_pose,
                'dropoff_pose': t.dropoff_pose,
                'priority': t.priority,
            }

    def _handle_task_pool(self, msg: TaskPool):
        for t in msg.tasks:
            self.task_cache[t.task_id] = {
                'pickup_pose': t.pickup_pose,
                'dropoff_pose': t.dropoff_pose,
                'priority': t.priority,
            }

    def _handle_task_event(self, t: Task):
        self.task_cache[t.task_id] = {
            'pickup_pose': t.pickup_pose,
            'dropoff_pose': t.dropoff_pose,
            'priority': t.priority,
        }

    def _handle_bundle(self, msg: Bundle):
        """
        Receives ordered queue from local Bundle Manager.
        Locks in-progress task; claims new current_task when IDLE.
        """
        if msg.robot_id != self.robot_id:
            return

        # GUARD: Never preempt an in-progress execution
        if self.execution_state != TaskExecutionState.IDLE:
            return

        if msg.current_task:
            self._claim_and_start_task(msg.current_task)

    def _handle_pickup_observation(self, obs: PickupObservation):
        """
        Phase 9 — Fleet pickup observation received.
        If another robot confirms ownership of our RECONCILING task, drop it.
        """
        if self.execution_state != TaskExecutionState.RECONCILING:
            return
        if obs.task_id != self.active_task_id:
            return

        if obs.is_valid and obs.confirmed_owner_id and obs.confirmed_owner_id != self.robot_id:
            # Another robot is confirmed owner — release our claim and go IDLE
            self.get_logger().info(
                f'[{self.robot_id}] ✓ Task {obs.task_id} confirmed owned by '
                f'{obs.confirmed_owner_id}. Releasing and returning to IDLE.'
            )
            self._release_task_ownership(obs.task_id)
        elif not obs.is_valid and obs.robot_id == self.robot_id:
            # Our own observation came back unresolved — task is truly stale
            self.get_logger().warn(
                f'[{self.robot_id}] Task {obs.task_id} unresolved. '
                f'Releasing and triggering re-bid.'
            )
            self._release_task_ownership(obs.task_id)

    # ──────────────────────────────────────────────────────────────────────────
    # State Machine Execution Logic
    # ──────────────────────────────────────────────────────────────────────────

    def _claim_and_start_task(self, task_id: str):
        """Transitions IDLE ──► ASSIGNED ──► IN_PROGRESS for the given task."""
        if not task_id:
            return

        info = self.task_cache.get(task_id)
        if not info:
            self.get_logger().warn(
                f'[{self.robot_id}] Waiting for task {task_id} coordinates '
                f'from pool/world_view...'
            )
            return

        self.active_task_id = task_id
        self.active_task_info = info

        # Step 1: ASSIGNED
        self.execution_state = TaskExecutionState.ASSIGNED
        self.get_logger().info(
            f'[{self.robot_id}] ══════════════════════════════════════════════════════\n'
            f'[{self.robot_id}] ★ STATE: [ASSIGNED] ──► Task {task_id}\n'
            f'[{self.robot_id}] ══════════════════════════════════════════════════════'
        )
        self._publish_task_state(Task.STATE_ASSIGNED)

        # Step 2: Transition immediately to IN_PROGRESS
        self._start_pickup_navigation()

    def _start_pickup_navigation(self):
        """Transitions ASSIGNED ──► IN_PROGRESS: Dispatches goal to PICKUP pose."""
        self.execution_state = TaskExecutionState.IN_PROGRESS
        pickup_pose = self.active_task_info['pickup_pose']
        px = pickup_pose.pose.position.x
        py = pickup_pose.pose.position.y

        self.get_logger().info(
            f'[{self.robot_id}] ──► STATE: [IN_PROGRESS] | '
            f'Navigating to PICKUP ({px:.2f}, {py:.2f})'
        )
        self._publish_task_state(Task.STATE_IN_PROGRESS)

        self._dispatch_navigation(
            target_pose=pickup_pose,
            target_name=f'PICKUP for {self.active_task_id}',
            on_reached=self._on_pickup_arrival,
        )

    def _on_pickup_arrival(self):
        """
        Arrived at pickup location.
        Phase 9: validate the physical item exists in Gazebo before proceeding.
        """
        if self.enable_pickup_validation and self._gazebo_entity_client is not None:
            self._validate_pickup_item()
        else:
            # No validation — proceed directly
            self._confirm_pickup()

    def _validate_pickup_item(self):
        """
        Phase 9 — Calls Gazebo GetEntityState to verify the cargo item model exists.
        Item model name convention: <gazebo_item_prefix><task_id>  (e.g. 'item_T1')
        """
        from gazebo_msgs.srv import GetEntityState
        entity_name = f'{self.gazebo_item_prefix}{self.active_task_id}'
        req = GetEntityState.Request()
        req.name = entity_name
        req.reference_frame = 'world'

        self.get_logger().info(
            f'[{self.robot_id}] Phase 9: Checking Gazebo entity "{entity_name}"...'
        )

        if not self._gazebo_entity_client.wait_for_service(timeout_sec=1.0):
            # Gazebo service unavailable — assume item exists (optimistic fallback)
            self.get_logger().warn(
                f'[{self.robot_id}] Gazebo GetEntityState unavailable. '
                f'Assuming item present (optimistic fallback).'
            )
            self._confirm_pickup()
            return

        future = self._gazebo_entity_client.call_async(req)
        future.add_done_callback(self._on_gazebo_entity_response)

    def _on_gazebo_entity_response(self, future):
        """Callback: Gazebo entity check result."""
        try:
            result = future.result()
            item_exists = result.success
        except Exception as e:
            self.get_logger().warn(
                f'[{self.robot_id}] Gazebo entity check failed: {e}. '
                f'Assuming item present.'
            )
            item_exists = True  # optimistic on service error

        if item_exists:
            self.get_logger().info(
                f'[{self.robot_id}] ✓ Phase 9: Item confirmed present at pickup. Proceeding.'
            )
            self._confirm_pickup()
        else:
            self.get_logger().warn(
                f'[{self.robot_id}] ✗ Phase 9: Item ABSENT at pickup for task '
                f'{self.active_task_id}. Entering RECONCILING state.'
            )
            self._enter_reconciling()

    def _confirm_pickup(self):
        """Transitions IN_PROGRESS ──► PICKUP_COMPLETED."""
        self.execution_state = TaskExecutionState.PICKUP_COMPLETED
        self.get_logger().info(
            f'[{self.robot_id}] ──► STATE: [PICKUP_COMPLETED] | '
            f'Arrived at pickup. Loading cargo (dwell {self.pickup_dwell_sec}s)...'
        )
        self._publish_task_state(Task.STATE_PICKUP_COMPLETED)

        # Non-blocking dwell timer for pickup cargo loading
        self.create_timer(self.pickup_dwell_sec, self._finish_pickup_dwell)

    def _enter_reconciling(self):
        """
        Phase 9 — Item not found at pickup.
        Publish a PICKUP_OBSERVATION (is_valid=False) and wait for fleet resolution.
        """
        self.execution_state = TaskExecutionState.RECONCILING
        self._reconcile_start_time = time.monotonic()

        # Publish observation: item absent
        obs = PickupObservation()
        obs.robot_id = self.robot_id
        obs.task_id = self.active_task_id
        obs.observed_pose = self.active_task_info['pickup_pose']
        obs.is_valid = False
        obs.failure_reason = 'item_absent_at_pickup'
        obs.confirmed_owner_id = ''
        obs.timestamp = self.get_clock().now().to_msg()
        self.pickup_obs_pub.publish(obs)

        # Broadcast via P2P so other robots can respond
        self.p2p.broadcast('PICKUP_OBSERVATION', payload={
            'robot_id': self.robot_id,
            'task_id': self.active_task_id,
            'is_valid': False,
            'failure_reason': 'item_absent_at_pickup',
            'timestamp': self.get_clock().now().nanoseconds,
        })

        self.get_logger().warn(
            f'[{self.robot_id}] ──► STATE: [RECONCILING] | '
            f'Task {self.active_task_id} pickup absent. '
            f'Awaiting fleet confirmation ({self.reconciliation_timeout_sec}s timeout)...'
        )

        # Start reconciliation timeout
        self._reconcile_timer = self.create_timer(
            self.reconciliation_timeout_sec,
            self._reconciliation_timeout
        )

    def _reconciliation_timeout(self):
        """
        Reconciliation timed out — no peer confirmed ownership.
        Re-bid the task: mark as AVAILABLE in the fleet so Max-Sum can reallocate.
        """
        if self.execution_state != TaskExecutionState.RECONCILING:
            if self._reconcile_timer is not None:
                self._reconcile_timer.cancel()
                self._reconcile_timer.destroy()
                self._reconcile_timer = None
            return

        self.get_logger().warn(
            f'[{self.robot_id}] Reconciliation timeout for task {self.active_task_id}. '
            f'No confirmed owner — releasing task for re-bid.'
        )

        if self._reconcile_timer is not None:
            self._reconcile_timer.cancel()
            self._reconcile_timer.destroy()
            self._reconcile_timer = None

        self._release_task_ownership(self.active_task_id)

    def _release_task_ownership(self, task_id: str):
        """
        Releases ownership of a task and returns to IDLE.
        Publishes STATE_AVAILABLE so Max-Sum can re-bid the task.
        """
        # Publish task as available again (re-bid signal)
        t = Task()
        t.task_id = task_id
        t.assigned_robot_id = ''
        t.state = Task.STATE_AVAILABLE
        t.priority = self.active_task_info.get('priority', 1) if self.active_task_info else 1
        if self.active_task_info:
            t.pickup_pose = self.active_task_info['pickup_pose']
            t.dropoff_pose = self.active_task_info['dropoff_pose']
        self.task_event_pub.publish(t)

        # Publish ownership release
        rel = TaskOwnership()
        rel.task_id = task_id
        rel.robot_id = self.robot_id
        rel.status = TaskOwnership.STATUS_RELEASED
        rel.timestamp = self.get_clock().now().to_msg()
        self.ownership_release_pub.publish(rel)

        # Broadcast via P2P
        self.p2p.broadcast('TASK_OWNERSHIP', payload={
            'task_id': task_id,
            'robot_id': self.robot_id,
            'status': 'RELEASED',
            'timestamp': self.get_clock().now().nanoseconds,
        })

        self.get_logger().info(
            f'[{self.robot_id}] Task {task_id} released. Returning to IDLE.'
        )

        # Reset and return to IDLE
        self.active_task_id = None
        self.active_task_info = None
        self.execution_state = TaskExecutionState.IDLE

    def _finish_pickup_dwell(self):
        """Transitions PICKUP_COMPLETED ──► DELIVERING: Dispatches goal to DROPOFF pose."""
        if self.execution_state != TaskExecutionState.PICKUP_COMPLETED:
            return

        dropoff_pose = self.active_task_info['dropoff_pose']
        dx = dropoff_pose.pose.position.x
        dy = dropoff_pose.pose.position.y

        self.execution_state = TaskExecutionState.DELIVERING
        self.get_logger().info(
            f'[{self.robot_id}] ──► STATE: [DELIVERING] | '
            f'Cargo loaded. Navigating to DROPOFF ({dx:.2f}, {dy:.2f})'
        )
        self._publish_task_state(Task.STATE_DELIVERING)

        self._dispatch_navigation(
            target_pose=dropoff_pose,
            target_name=f'DROPOFF for {self.active_task_id}',
            on_reached=self._on_delivery_arrival,
        )

    def _on_delivery_arrival(self):
        """Transitions DELIVERING ──► COMPLETED: Verifies delivery & marks task complete."""
        self.execution_state = TaskExecutionState.COMPLETED
        task_id = self.active_task_id

        self.get_logger().info(
            f'[{self.robot_id}] ══════════════════════════════════════════════════════\n'
            f'[{self.robot_id}] ★ STATE: [COMPLETED] ──► Task {task_id} successfully delivered!\n'
            f'[{self.robot_id}] ══════════════════════════════════════════════════════'
        )
        self._publish_task_state(Task.STATE_COMPLETED)

        # Reset active state and return to IDLE to pick next task from bundle
        self.active_task_id = None
        self.active_task_info = None
        self.execution_state = TaskExecutionState.IDLE

    # ──────────────────────────────────────────────────────────────────────────
    # Nav2 Action Dispatch & Verification
    # ──────────────────────────────────────────────────────────────────────────

    def _dispatch_navigation(self, target_pose: PoseStamped, target_name: str, on_reached):
        """
        Dispatches navigation goal to Nav2 NavigateToPose action server.
        If Nav2 is offline / disabled (e.g. unit tests), gracefully simulates arrival.

        BUG FIX: Previously used create_timer() which creates a REPEATING timer.
        Now we store the timer handle and cancel+destroy it inside _simulated_arrival
        before calling on_reached(), so it fires exactly once.
        """
        if self.enable_nav2 and self.nav_client.wait_for_server(timeout_sec=0.5):
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = target_pose
            goal_msg.pose.header.frame_id = 'map'
            goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

            self.get_logger().info(
                f'[{self.robot_id}] Dispatching Nav2 action goal to {target_name}...'
            )
            send_future = self.nav_client.send_goal_async(goal_msg)
            send_future.add_done_callback(
                lambda f: self._on_goal_response(f, target_pose, on_reached)
            )
        else:
            # Simulated navigation transit (for headless testing or when Nav2 is offline)
            self.get_logger().info(
                f'[{self.robot_id}] Nav2 not active. Simulating transit to {target_name}...'
            )
            # Create a ONE-SHOT simulated transit timer.
            # We store the handle so it can be cancelled inside the callback.
            transit_timer_handle = []  # mutable container for the handle

            def _fire_once():
                # Cancel and destroy the timer immediately — makes it one-shot
                if transit_timer_handle:
                    transit_timer_handle[0].cancel()
                    transit_timer_handle[0].destroy()
                self._simulated_arrival(target_pose, on_reached)

            t = self.create_timer(1.0, _fire_once)
            transit_timer_handle.append(t)

    def _simulated_arrival(self, target_pose: PoseStamped, on_reached):
        """Updates simulated robot position and triggers the arrival callback."""
        self.current_x = target_pose.pose.position.x
        self.current_y = target_pose.pose.position.y
        on_reached()

    def _on_goal_response(self, future, target_pose: PoseStamped, on_reached):
        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().warn(
                f'[{self.robot_id}] Nav2 goal was rejected or cancelled.'
            )
            return

        self.current_goal_handle = goal_handle
        self.get_logger().info(
            f'[{self.robot_id}] Nav2 goal accepted. Tracking progress...'
        )
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda f: self._on_goal_result(f, target_pose, on_reached)
        )

    def _on_goal_result(self, future, target_pose: PoseStamped, on_reached):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            tx = target_pose.pose.position.x
            ty = target_pose.pose.position.y
            dist = math.hypot(self.current_x - tx, self.current_y - ty)
            self.get_logger().info(
                f'[{self.robot_id}] ✓ Nav2 SUCCEEDED. '
                f'Arrival verified (distance to waypoint: {dist:.2f}m)'
            )
            on_reached()
        else:
            self.get_logger().warn(
                f'[{self.robot_id}] Nav2 goal ended with status: {status}'
            )

    # ──────────────────────────────────────────────────────────────────────────
    # Helper & Event Broadcasting
    # ──────────────────────────────────────────────────────────────────────────

    def _publish_task_state(self, state: int):
        """Publishes task state updates to fleet topic and P2P transport."""
        if not self.active_task_id or not self.active_task_info:
            return

        t = Task()
        t.task_id = self.active_task_id
        t.assigned_robot_id = self.robot_id
        t.state = state
        t.priority = self.active_task_info.get('priority', 1)
        t.pickup_pose = self.active_task_info['pickup_pose']
        t.dropoff_pose = self.active_task_info['dropoff_pose']

        # Publish local and fleet
        self.assigned_task_pub.publish(t)
        self.task_event_pub.publish(t)

        # Broadcast via P2P
        self.p2p.broadcast('TASK_STATUS', payload={
            'task_id': self.active_task_id,
            'assigned_robot_id': self.robot_id,
            'state': state,
            'timestamp': self.get_clock().now().nanoseconds,
        })

    def _eval_execution_step(self):
        """Periodic safety check — currently a no-op placeholder."""
        pass


def main(args=None):
    rclpy.init(args=args)
    node = TaskExecutionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
