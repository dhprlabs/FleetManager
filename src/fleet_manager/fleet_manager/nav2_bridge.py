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

import json
import math
import time
from enum import Enum
from typing import Dict, Optional, Set, Tuple

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose

from fleet_interfaces.msg import (
    Bundle,
    P2PMessage,
    PickupObservation,
    Reservation,
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
        self.declare_parameter('arrival_tolerance', 0.8)
        self.declare_parameter('reconciliation_timeout_sec', 10.0)
        self.declare_parameter('gazebo_item_prefix', 'item_')
        self.declare_parameter('enable_pickup_validation', True)
        self.declare_parameter('dock_x', 5.0)
        self.declare_parameter('dock_y', 12.0)
        self.declare_parameter('broadcaster_x', 5.0)
        self.declare_parameter('broadcaster_y', 12.0)
        self.declare_parameter('communication_radius', 6.0)
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)

        self.robot_id = self.get_parameter('robot_id').get_parameter_value().string_value
        self.enable_nav2 = self.get_parameter('enable_nav2').get_parameter_value().bool_value
        self.pickup_dwell_sec = self.get_parameter('pickup_dwell_sec').get_parameter_value().double_value
        self.delivery_dwell_sec = self.get_parameter('delivery_dwell_sec').get_parameter_value().double_value
        self.arrival_tolerance = self.get_parameter('arrival_tolerance').get_parameter_value().double_value
        self.reconciliation_timeout_sec = self.get_parameter('reconciliation_timeout_sec').get_parameter_value().double_value
        self.gazebo_item_prefix = self.get_parameter('gazebo_item_prefix').get_parameter_value().string_value
        self.enable_pickup_validation = self.get_parameter('enable_pickup_validation').get_parameter_value().bool_value
        self.comm_radius = self.get_parameter('communication_radius').get_parameter_value().double_value
        self.broadcaster_x = self.get_parameter('broadcaster_x').get_parameter_value().double_value
        self.broadcaster_y = self.get_parameter('broadcaster_y').get_parameter_value().double_value
        init_x = self.get_parameter('initial_x').get_parameter_value().double_value
        init_y = self.get_parameter('initial_y').get_parameter_value().double_value
        # Treat initial pose as the robot's dock position
        self.dock_x = self.get_parameter('dock_x').get_parameter_value().double_value if self.has_parameter('dock_x') else init_x
        self.dock_y = self.get_parameter('dock_y').get_parameter_value().double_value if self.has_parameter('dock_y') else init_y
        if self.dock_x == 5.0 and self.dock_y == 12.0:
            self.dock_x, self.dock_y = init_x, init_y

        # Execution State Machine
        self.execution_state = TaskExecutionState.IDLE
        self.active_task_id: Optional[str] = None
        self.active_task_info: Optional[Dict] = None
        self._dwell_timer = None

        # Dock-return fallback tracking (Root Cause 2)
        self._dock_return_active: bool = False
        self._dock_return_generation: int = 0

        # Robot Live Pose
        self.current_x: float = init_x
        self.current_y: float = init_y

        # Task Cache (task_id -> {pickup_pose, dropoff_pose, priority})
        self.task_cache: Dict[str, Dict] = {}

        # Phase 9 — Reconciliation state
        self._reconcile_start_time: Optional[float] = None
        self._reconcile_timer = None

        # Nav2 Action Client scoped to this robot's namespace
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.current_goal_handle = None
        self._goal_generation = 0

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

        # CRDT / Phase 11 tracking
        self._seen_obs_ids: Set[str] = set()
        self.lamport_clock: int = 0
        self._world_view_task_states: Dict[str, Tuple[int, str]] = {}
        self._transit_timer = None

        # P2P Client for task status broadcasting
        self.p2p = P2PClient(node=self)
        try:
            self.p2p.register_handler('PICKUP_OBSERVATION', self._on_p2p_pickup_obs)
        except Exception:
            pass

        # ── Subscriptions ──────────────────────────────────────────────────
        self.bundle_sub = self.create_subscription(
            Bundle, 'bundle', self._handle_bundle, 10
        )
        self.robot_state_sub = self.create_subscription(
            RobotState, 'robot_state', self._handle_robot_state, 10
        )
        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped, 'amcl_pose', self._handle_amcl_pose, 10
        )
        self.world_view_sub = self.create_subscription(
            WorldView, 'world_view', self._handle_world_view, 10
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
        self.task_event_pub = self.create_publisher(Task, 'task_events', 10)
        self.fleet_task_event_pub = self.create_publisher(Task, '/fleet/task_events', 10)
        self.assigned_task_pub = self.create_publisher(Task, 'assigned_task', 10)
        self.pickup_obs_pub = self.create_publisher(
            PickupObservation, '/fleet/pickup_observations', 10
        )
        self.tasks_pickup_obs_pub = self.create_publisher(
            PickupObservation, '/tasks/pickup_observation', 10
        )
        # NOTE: no publisher to the global '/fleet/task_ownership' topic here.
        # Ownership release is communicated via the P2P TASK_OWNERSHIP
        # broadcast below, which is naturally gated by radio reachability;
        # a global-topic publish would let out-of-range robots see this
        # release with no connectivity basis for it (split-brain).

        # Phase 12 traffic reservation interfaces
        self.traffic_reserve_pub = self.create_publisher(Reservation, '/traffic/reserve', 10)
        self.traffic_release_pub = self.create_publisher(Reservation, '/traffic/release', 10)
        self.traffic_reserve_sub = self.create_subscription(
            Reservation, '/traffic/reserve', self._handle_traffic_reserve_msg, 10
        )
        self._current_held_aisle: Optional[str] = None

        # Execution evaluation timer
        self.eval_timer = self.create_timer(0.5, self._eval_execution_step)

        self.get_logger().info(
            f'[{self.robot_id}] Task Execution Manager active | '
            f'Nav2 Target: /{self.robot_id}/navigate_to_pose | '
            f'Dwell: {self.pickup_dwell_sec}s | '
            f'Pickup validation: {self.enable_pickup_validation} | '
            f'Home Dock: ({self.dock_x:.2f}, {self.dock_y:.2f})'
        )

    # ──────────────────────────────────────────────────────────────────────────
    # Inbound Handlers
    # ──────────────────────────────────────────────────────────────────────────

    def _handle_robot_state(self, msg: RobotState):
        self.current_x = msg.current_pose.pose.position.x
        self.current_y = msg.current_pose.pose.position.y

    def _handle_amcl_pose(self, msg: PoseWithCovarianceStamped):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

    def _handle_world_view(self, msg: WorldView):
        self.current_x = msg.own_state.current_pose.pose.position.x
        self.current_y = msg.own_state.current_pose.pose.position.y
        self.lamport_clock = max(self.lamport_clock, msg.lamport_clock)
        for t in msg.task_states:
            self.task_cache[t.task_id] = {
                'pickup_pose': t.pickup_pose,
                'dropoff_pose': t.dropoff_pose,
                'priority': t.priority,
            }
            self._world_view_task_states[t.task_id] = (t.state, t.assigned_robot_id)

            # Check if active task was physically executed / completed by another robot in CRDT
            if self.active_task_id and t.task_id == self.active_task_id:
                if t.state in (Task.STATE_PICKUP_COMPLETED, Task.STATE_DELIVERING, Task.STATE_COMPLETED):
                    if t.assigned_robot_id and t.assigned_robot_id != self.robot_id:
                        self.get_logger().warn(
                            f'[{self.robot_id}] [NAV2] Active task {self.active_task_id} confirmed {t.state} '
                            f'by robot {t.assigned_robot_id} in CRDT world view. Aborting.'
                        )
                        self._abort_active_task(self.active_task_id, release_ownership=False)
                elif (self.execution_state == TaskExecutionState.IN_PROGRESS and
                      t.state == Task.STATE_ALLOCATED and
                      t.assigned_robot_id and t.assigned_robot_id != self.robot_id):
                    # A synchronized allocation may hand pre-pickup work to a
                    # closer peer. Cancel without releasing the peer's claim.
                    self.get_logger().info(
                        f'[{self.robot_id}] [NAV2] Pre-pickup handoff of {t.task_id} '
                        f'to {t.assigned_robot_id}; cancelling local goal.'
                    )
                    self._abort_active_task(self.active_task_id, release_ownership=False)

    def _handle_task_pool(self, msg: TaskPool):
        # Ignore global DDS broadcast if robot is out of radio range of task broadcaster
        dist_to_broadcaster = math.hypot(self.current_x - self.broadcaster_x, self.current_y - self.broadcaster_y)
        if dist_to_broadcaster > self.comm_radius:
            return
        for t in msg.tasks:
            self.task_cache[t.task_id] = {
                'pickup_pose': t.pickup_pose,
                'dropoff_pose': t.dropoff_pose,
                'priority': t.priority,
            }

    def _handle_task_event(self, t: Task):
        dist_to_broadcaster = math.hypot(self.current_x - self.broadcaster_x, self.current_y - self.broadcaster_y)
        if dist_to_broadcaster > self.comm_radius:
            return
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

        # GUARD: If active task was invalidated from bundle, safely abort
        if self.execution_state != TaskExecutionState.IDLE:
            if self.active_task_id and self.active_task_id not in msg.task_ids:
                self.get_logger().warn(
                    f'[{self.robot_id}] [NAV2] Active task {self.active_task_id} no longer in bundle. Aborting.'
                )
                # Ownership was removed by a fleet handoff; publishing another
                # AVAILABLE update here could overwrite the new owner's claim.
                self._abort_active_task(self.active_task_id, release_ownership=False)
            return

        if msg.current_task:
            self._claim_and_start_task(msg.current_task)

    def _on_p2p_pickup_obs(self, msg: P2PMessage):
        """Handles PICKUP_OBSERVATION incoming over P2P virtual network."""
        try:
            d = json.loads(msg.payload)
            obs = PickupObservation()
            obs.obs_id = d.get('obs_id', '')
            obs.robot_id = d.get('robot_id', msg.source_robot_id)
            obs.task_id = d.get('task_id', '')
            obs.is_valid = d.get('is_valid', False)
            obs.item_present = d.get('item_present', False)
            obs.failure_reason = d.get('failure_reason', '')
            obs.confirmed_owner_id = d.get('confirmed_owner_id', '')
            obs.lamport_clock = d.get('lamport_clock', 0)
            self._handle_pickup_observation(obs)
        except Exception as e:
            self.get_logger().warn(f'[{self.robot_id}] [NAV2] Error parsing P2P PICKUP_OBSERVATION: {e}')

    def _handle_pickup_observation(self, obs: PickupObservation):
        """
        Phase 9 & 11 — Fleet pickup observation received.
        Deduplicates by obs_id.
        If another robot confirms ownership/completion of our task, abort it.
        """
        if obs.obs_id:
            if obs.obs_id in self._seen_obs_ids:
                return
            self._seen_obs_ids.add(obs.obs_id)

        # If another robot confirms ownership/completion of our active task
        if obs.is_valid and obs.confirmed_owner_id and obs.confirmed_owner_id != self.robot_id:
            if obs.task_id == self.active_task_id:
                self.get_logger().info(
                    f'[{self.robot_id}] [NAV2] ✓ Task {obs.task_id} confirmed owned by '
                    f'{obs.confirmed_owner_id}. Aborting active execution.'
                )
                self._abort_active_task(obs.task_id, release_ownership=False)
            return

        # If we are reconciling this task
        if self.execution_state == TaskExecutionState.RECONCILING and obs.task_id == self.active_task_id:
            if not obs.is_valid and (obs.failure_reason == 'no_confirmed_owner' or obs.robot_id == self.robot_id):
                self.get_logger().warn(
                    f'[{self.robot_id}] [NAV2] Task {obs.task_id} unconfirmed by peers. '
                    f'Marking completed by other entity.'
                )
                if self._reconcile_timer is not None:
                    self._reconcile_timer.cancel()
                    self._reconcile_timer.destroy()
                    self._reconcile_timer = None
                self._mark_task_completed_by_other(obs.task_id, 'other_entity')
                self._abort_active_task(obs.task_id, release_ownership=False)

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
        if self.execution_state != TaskExecutionState.IN_PROGRESS or not self.active_task_id:
            return
        self._goal_generation += 1
        self.current_goal_handle = None
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
                f'[{self.robot_id}] ✓ Phase 11: Item confirmed present at pickup. Proceeding.'
            )
            self._confirm_pickup()
        else:
            # Item absent at pickup: the item was taken or was never here.
            # Release ownership so Max-Sum can re-bid the task.  Do NOT mark
            # STATE_COMPLETED — that would permanently remove the task from
            # the pool.  Instead broadcast STATE_AVAILABLE so the task goes
            # back into contention.
            self.get_logger().warn(
                f'[{self.robot_id}] ✗ Phase 11: Item ABSENT at pickup for task '
                f'{self.active_task_id}. Releasing for re-bid.'
            )
            self._abort_active_task(self.active_task_id, release_ownership=True)

    def _confirm_pickup(self):
        """Transitions IN_PROGRESS ──► PICKUP_COMPLETED."""
        self.execution_state = TaskExecutionState.PICKUP_COMPLETED
        self.lamport_clock += 1
        obs_id = f"{self.robot_id}:{self.lamport_clock}"
        self._seen_obs_ids.add(obs_id)

        # Publish observation: item confirmed physically present
        obs = PickupObservation()
        obs.obs_id = obs_id
        obs.robot_id = self.robot_id
        obs.task_id = self.active_task_id
        obs.observed_pose = self.active_task_info['pickup_pose']
        obs.is_valid = True
        obs.item_present = True
        obs.failure_reason = ''
        obs.confirmed_owner_id = self.robot_id
        obs.lamport_clock = int(self.lamport_clock)
        obs.timestamp = self.get_clock().now().to_msg()
        self.pickup_obs_pub.publish(obs)
        self.tasks_pickup_obs_pub.publish(obs)

        if self.p2p:
            self.p2p.broadcast('PICKUP_OBSERVATION', payload={
                'obs_id': obs_id,
                'robot_id': self.robot_id,
                'task_id': self.active_task_id,
                'is_valid': True,
                'item_present': True,
                'failure_reason': '',
                'confirmed_owner_id': self.robot_id,
                'lamport_clock': int(self.lamport_clock),
                'timestamp': self.get_clock().now().nanoseconds,
            })

        self.get_logger().info(
            f'[{self.robot_id}] ──► STATE: [PICKUP_COMPLETED] | '
            f'Arrived at pickup. Loading cargo (dwell {self.pickup_dwell_sec}s)...'
        )
        self._publish_task_state(Task.STATE_PICKUP_COMPLETED)

        # Non-blocking dwell timer for pickup cargo loading
        self._dwell_timer = self.create_timer(self.pickup_dwell_sec, self._finish_pickup_dwell)

    def _enter_reconciling(self):
        """
        Phase 11 — Item not found at pickup.
        Publish a PICKUP_OBSERVATION (is_valid=False, item_present=False) and wait for fleet resolution.
        """
        self.execution_state = TaskExecutionState.RECONCILING
        self._reconcile_start_time = time.monotonic()
        self.lamport_clock += 1
        obs_id = f"{self.robot_id}:{self.lamport_clock}"
        self._seen_obs_ids.add(obs_id)

        # Publish observation: item absent
        obs = PickupObservation()
        obs.obs_id = obs_id
        obs.robot_id = self.robot_id
        obs.task_id = self.active_task_id
        obs.observed_pose = self.active_task_info['pickup_pose']
        obs.is_valid = False
        obs.item_present = False
        obs.failure_reason = 'item_absent_at_pickup'
        obs.confirmed_owner_id = ''
        obs.lamport_clock = int(self.lamport_clock)
        obs.timestamp = self.get_clock().now().to_msg()
        self.pickup_obs_pub.publish(obs)
        self.tasks_pickup_obs_pub.publish(obs)

        # Broadcast via P2P so other robots can respond
        if self.p2p:
            self.p2p.broadcast('PICKUP_OBSERVATION', payload={
                'obs_id': obs_id,
                'robot_id': self.robot_id,
                'task_id': self.active_task_id,
                'is_valid': False,
                'item_present': False,
                'failure_reason': 'item_absent_at_pickup',
                'lamport_clock': int(self.lamport_clock),
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

    def _mark_task_completed_by_other(self, task_id: str, other_entity: str = 'other_entity'):
        """
        Marks task completed by another robot or external entity when item is absent at pickup.
        Does NOT release ownership for re-bid, but updates fleet, local state, and CRDT to STATE_COMPLETED.
        """
        self.get_logger().info(
            f'[{self.robot_id}] Marking task {task_id} COMPLETED by entity "{other_entity}".'
        )
        self._world_view_task_states[task_id] = (Task.STATE_COMPLETED, other_entity)

        info = self.active_task_info or self.task_cache.get(task_id, {})

        # Publish task completed event locally and globally
        t = Task()
        t.task_id = task_id
        t.assigned_robot_id = other_entity
        t.state = Task.STATE_COMPLETED
        t.priority = info.get('priority', 1) if info else 1
        if info and 'pickup_pose' in info:
            t.pickup_pose = info['pickup_pose']
            t.dropoff_pose = info['dropoff_pose']

        self.assigned_task_pub.publish(t)
        self.task_event_pub.publish(t)
        self.fleet_task_event_pub.publish(t)

        # Broadcast via P2P
        if self.p2p:
            px = float(t.pickup_pose.pose.position.x) if (info and 'pickup_pose' in info) else 0.0
            py = float(t.pickup_pose.pose.position.y) if (info and 'pickup_pose' in info) else 0.0
            dx = float(t.dropoff_pose.pose.position.x) if (info and 'dropoff_pose' in info) else 0.0
            dy = float(t.dropoff_pose.pose.position.y) if (info and 'dropoff_pose' in info) else 0.0
            priority = int(t.priority)

            self.p2p.broadcast('TASK_STATUS', payload={
                'task_id': task_id,
                'assigned_robot_id': other_entity,
                'state': Task.STATE_COMPLETED,
                'priority': priority,
                'pickup_x': px,
                'pickup_y': py,
                'dropoff_x': dx,
                'dropoff_y': dy,
                'source_robot_id': self.robot_id,
                'timestamp': self.get_clock().now().nanoseconds,
            })

            # Broadcast valid pickup observation with confirmed owner so peers drop duplicate claims
            self.lamport_clock += 1
            obs_id = f"{self.robot_id}:{self.lamport_clock}"
            self._seen_obs_ids.add(obs_id)
            self.p2p.broadcast('PICKUP_OBSERVATION', payload={
                'obs_id': obs_id,
                'robot_id': self.robot_id,
                'task_id': task_id,
                'is_valid': True,
                'item_present': False,
                'failure_reason': 'completed_by_other',
                'confirmed_owner_id': other_entity,
                'lamport_clock': int(self.lamport_clock),
                'timestamp': self.get_clock().now().nanoseconds,
            })

    def _reconciliation_timeout(self):
        """
        Reconciliation timed out.
        Mark task as COMPLETED by other entity (since item was absent at pickup)
        and abort without re-bid.
        """
        if self.execution_state != TaskExecutionState.RECONCILING:
            if self._reconcile_timer is not None:
                self._reconcile_timer.cancel()
                self._reconcile_timer.destroy()
                self._reconcile_timer = None
            return

        if self._reconcile_timer is not None:
            self._reconcile_timer.cancel()
            self._reconcile_timer.destroy()
            self._reconcile_timer = None

        cached = self._world_view_task_states.get(self.active_task_id)
        other_entity = cached[1] if (cached and cached[1] and cached[1] != self.robot_id) else 'other_entity'

        self.get_logger().warn(
            f'[{self.robot_id}] Reconciliation timeout for task {self.active_task_id}. '
            f'Item absent at pickup; marking COMPLETED by entity "{other_entity}".'
        )
        task_id = self.active_task_id
        self._mark_task_completed_by_other(task_id, other_entity)
        self._abort_active_task(task_id, release_ownership=False)

    def _abort_active_task(self, task_id: Optional[str] = None, *, release_ownership: bool = True):
        """
        Phase 11 — Safely cancels active navigation, clears dwell and reconcile timers,
        releases task ownership, and returns to IDLE.
        """
        tid = task_id or self.active_task_id
        self.get_logger().warn(f'[{self.robot_id}] [NAV2] Aborting active task {tid}...')
        self._goal_generation += 1

        # 1. Cancel Nav2 action goal if running
        if self.current_goal_handle is not None:
            try:
                self.current_goal_handle.cancel_goal_async()
            except Exception as e:
                self.get_logger().warn(f'[{self.robot_id}] Error cancelling goal: {e}')
            self.current_goal_handle = None

        # 2. Cancel simulated transit timer if running
        if self._transit_timer is not None:
            self._transit_timer.cancel()
            self._transit_timer.destroy()
            self._transit_timer = None

        # 3. Cancel dwell and reconcile timers
        if self._dwell_timer is not None:
            self._dwell_timer.cancel()
            self._dwell_timer.destroy()
            self._dwell_timer = None
        if self._reconcile_timer is not None:
            self._reconcile_timer.cancel()
            self._reconcile_timer.destroy()
            self._reconcile_timer = None

        # 4. Release aisle reservation if held
        self._release_aisle_if_held()

        # 5. Release task ownership if it was our active task
        if tid and release_ownership:
            self._release_task_ownership(tid)
        else:
            self.active_task_id = None
            self.active_task_info = None
            self.execution_state = TaskExecutionState.IDLE

    def _handle_traffic_reserve_msg(self, msg: Reservation):
        """Receives traffic reservation grants."""
        if msg.robot_id == self.robot_id and msg.state == Reservation.STATE_GRANTED:
            self._current_held_aisle = msg.segment_id

    def _release_aisle_if_held(self):
        """Releases physical single-lane aisle reservation if held."""
        if self._current_held_aisle:
            seg = self._current_held_aisle
            self._current_held_aisle = None
            res = Reservation()
            res.reservation_id = f"{self.robot_id}:{seg}:release"
            res.robot_id = self.robot_id
            res.segment_id = seg
            res.state = Reservation.STATE_RELEASED
            self.traffic_release_pub.publish(res)
            self.get_logger().info(f'[{self.robot_id}] [TRAFFIC] Released aisle reservation for {seg}.')

    def _release_task_ownership(self, task_id: str):
        """
        Releases ownership of a task and returns to IDLE.
        Publishes STATE_AVAILABLE so Max-Sum can re-bid the task.
        Broadcasts both TASK_OWNERSHIP and TASK_STATUS via P2P so CRDT is updated.
        """
        # Release aisle reservation if held
        self._release_aisle_if_held()
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

        # Broadcast ownership release via P2P (reachability-gated)
        if self.p2p:
            self.p2p.broadcast('TASK_OWNERSHIP', payload={
                'task_id': task_id,
                'robot_id': self.robot_id,
                'status': TaskOwnership.STATUS_RELEASED,
                'timestamp': self.get_clock().now().nanoseconds,
            })

            px = float(self.active_task_info['pickup_pose'].pose.position.x) if (self.active_task_info and 'pickup_pose' in self.active_task_info) else 0.0
            py = float(self.active_task_info['pickup_pose'].pose.position.y) if (self.active_task_info and 'pickup_pose' in self.active_task_info) else 0.0
            dx = float(self.active_task_info['dropoff_pose'].pose.position.x) if (self.active_task_info and 'dropoff_pose' in self.active_task_info) else 0.0
            dy = float(self.active_task_info['dropoff_pose'].pose.position.y) if (self.active_task_info and 'dropoff_pose' in self.active_task_info) else 0.0
            priority = int(self.active_task_info.get('priority', 1)) if self.active_task_info else 1

            self.p2p.broadcast('TASK_STATUS', payload={
                'task_id': task_id,
                'assigned_robot_id': '',
                'state': Task.STATE_AVAILABLE,
                'priority': priority,
                'pickup_x': px,
                'pickup_y': py,
                'dropoff_x': dx,
                'dropoff_y': dy,
                'source_robot_id': self.robot_id,
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
        if self._dwell_timer is not None:
            self._dwell_timer.cancel()
            self._dwell_timer.destroy()
            self._dwell_timer = None

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
        if self.execution_state != TaskExecutionState.DELIVERING or not self.active_task_id:
            return
        self._goal_generation += 1
        self.current_goal_handle = None
        self.execution_state = TaskExecutionState.COMPLETED
        task_id = self.active_task_id

        self.get_logger().info(
            f'[{self.robot_id}] ══════════════════════════════════════════════════════\n'
            f'[{self.robot_id}] ★ STATE: [COMPLETED] ──► Task {task_id} successfully delivered!\n'
            f'[{self.robot_id}] ══════════════════════════════════════════════════════'
        )
        self._publish_task_state(Task.STATE_COMPLETED)

        # Release aisle reservation if held
        self._release_aisle_if_held()

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
        """
        task_id = self.active_task_id
        self._goal_generation += 1
        goal_generation = self._goal_generation
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
                lambda f: self._on_goal_response(
                    f, target_pose, on_reached, task_id, goal_generation
                )
            )
        else:
            # Simulated navigation transit (for headless testing or when Nav2 is offline)
            self.get_logger().info(
                f'[{self.robot_id}] Nav2 not active. Simulating transit to {target_name}...'
            )

            def _fire_once():
                if self._transit_timer is not None:
                    self._transit_timer.cancel()
                    self._transit_timer.destroy()
                    self._transit_timer = None
                if self.active_task_id == task_id and self._goal_generation == goal_generation:
                    self._simulated_arrival(target_pose, on_reached)

            if self._transit_timer is not None:
                self._transit_timer.cancel()
                self._transit_timer.destroy()
            self._transit_timer = self.create_timer(1.0, _fire_once)

    def _simulated_arrival(self, target_pose: PoseStamped, on_reached):
        """Updates simulated robot position and triggers the arrival callback."""
        self.current_x = target_pose.pose.position.x
        self.current_y = target_pose.pose.position.y
        on_reached()

    def _on_goal_response(self, future, target_pose: PoseStamped, on_reached,
                          task_id: Optional[str], goal_generation: int):
        if self.active_task_id != task_id or self._goal_generation != goal_generation:
            return
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
            lambda f: self._on_goal_result(
                f, target_pose, on_reached, task_id, goal_generation
            )
        )

    def _on_goal_result(self, future, target_pose: PoseStamped, on_reached,
                        task_id: Optional[str], goal_generation: int):
        if self.active_task_id != task_id or self._goal_generation != goal_generation:
            return
        status = future.result().status
        tx = target_pose.pose.position.x
        ty = target_pose.pose.position.y
        dist = math.hypot(self.current_x - tx, self.current_y - ty)
        tol = max(self.arrival_tolerance, 0.85)

        # Allow arrival if status is SUCCEEDED or if physical distance is within tolerance
        if status == GoalStatus.STATUS_SUCCEEDED or dist <= tol:
            self.current_goal_handle = None
            self.get_logger().info(
                f'[{self.robot_id}] ✓ Arrival verified (status: {status}, '
                f'distance to waypoint: {dist:.2f}m <= {tol:.2f}m)'
            )
            on_reached()
        else:
            # Nav2 aborted, cancelled, or failed.  The execution state machine is
            # now stuck and the task will never complete unless we release it.
            # Publish STATE_BLOCKED so Max-Sum triggers a reallocation re-bid,
            # then release ownership and return to IDLE.
            self.get_logger().error(
                f'[{self.robot_id}] Nav2 goal FAILED (status: {status}, '
                f'distance: {dist:.2f}m > tol: {tol:.2f}m). '
                f'Marking task {self.active_task_id} BLOCKED and releasing for reallocation.'
            )
            self.current_goal_handle = None
            if self.active_task_id and self.active_task_info:
                self._publish_task_state(Task.STATE_BLOCKED)
            self._abort_active_task(self.active_task_id, release_ownership=True)

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
        self.fleet_task_event_pub.publish(t)

        # Broadcast via P2P with full coordinate and authority metadata
        self.p2p.broadcast('TASK_STATUS', payload={
            'task_id': self.active_task_id,
            'assigned_robot_id': self.robot_id,
            'state': state,
            'priority': int(t.priority),
            'pickup_x': float(t.pickup_pose.pose.position.x),
            'pickup_y': float(t.pickup_pose.pose.position.y),
            'dropoff_x': float(t.dropoff_pose.pose.position.x),
            'dropoff_y': float(t.dropoff_pose.pose.position.y),
            'source_robot_id': self.robot_id,
            'timestamp': self.get_clock().now().nanoseconds,
        })

    def _eval_execution_step(self):
        """Periodic safety check, proximity arrival watchdog, and dock-return fallback.

        Watchdog fires ONLY when the Nav2 goal handle has already cleared (i.e.
        Nav2 finished without the result callback firing).  This prevents the
        watchdog from racing an in-flight Nav2 goal and causing a premature
        arrival trigger.
        """
        tol = max(self.arrival_tolerance, 0.8)

        # ── IDLE: dock-return fallback (Root Cause 2) ──────────────────────────
        # When the robot has no active task and is outside comm_radius of the dock
        # it cannot receive new task broadcasts or gossip with peers.  Navigate it
        # back to the dock so it re-enters the radio mesh.
        if self.execution_state == TaskExecutionState.IDLE and not self.active_task_id:
            dist_to_dock = math.hypot(self.current_x - self.dock_x, self.current_y - self.dock_y)
            if dist_to_dock > self.comm_radius and not self._dock_return_active:
                self._dock_return_active = True
                self._dock_return_generation += 1
                generation = self._dock_return_generation
                self.get_logger().info(
                    f'[{self.robot_id}] [DOCK_RETURN] IDLE and {dist_to_dock:.1f}m from dock '
                    f'(radius={self.comm_radius}m). Navigating back to dock at '
                    f'({self.dock_x:.1f}, {self.dock_y:.1f})...'
                )
                dock_pose = PoseStamped()
                dock_pose.header.frame_id = 'map'
                dock_pose.header.stamp = self.get_clock().now().to_msg()
                dock_pose.pose.position.x = self.dock_x
                dock_pose.pose.position.y = self.dock_y
                dock_pose.pose.orientation.w = 1.0

                def _on_dock_arrived():
                    if self._dock_return_generation != generation:
                        return
                    self._dock_return_active = False
                    self.get_logger().info(
                        f'[{self.robot_id}] [DOCK_RETURN] Arrived at dock. '
                        f'Now within comm_radius — resuming normal operation.'
                    )

                self._dispatch_navigation(
                    target_pose=dock_pose,
                    target_name='DOCK (fallback return)',
                    on_reached=_on_dock_arrived,
                )
            elif dist_to_dock <= self.comm_radius and self._dock_return_active:
                # Arrived back within range; the arrival callback will clear the flag.
                # Safety: clear it here too in case the callback was missed.
                self._dock_return_active = False
            return

        # If a task was just assigned, cancel any in-flight dock-return.
        if self._dock_return_active and self.active_task_id:
            self._dock_return_active = False
            self._dock_return_generation += 1
            if self.current_goal_handle:
                try:
                    self.current_goal_handle.cancel_goal_async()
                except Exception:
                    pass
                self.current_goal_handle = None

        if not self.active_task_id or not self.active_task_info:
            return

        # ── Active task proximity watchdog ────────────────────────────────────
        if self.execution_state == TaskExecutionState.DELIVERING:
            dropoff = self.active_task_info.get('dropoff_pose')
            if dropoff:
                dx = dropoff.pose.position.x
                dy = dropoff.pose.position.y
                dist = math.hypot(self.current_x - dx, self.current_y - dy)
                # Watchdog for DELIVERING: same guard — only fire when goal handle cleared.
                watchdog_tol = max(self.arrival_tolerance, 0.35)
                if dist <= watchdog_tol and self.current_goal_handle is None:
                    self.get_logger().info(
                        f'[{self.robot_id}] Proximity arrival watchdog triggered at DROPOFF '
                        f'(dist: {dist:.2f}m <= {watchdog_tol:.2f}m, goal_handle cleared) '
                        f'for task {self.active_task_id}'
                    )
                    self._on_delivery_arrival()

        elif self.execution_state == TaskExecutionState.IN_PROGRESS:
            pickup = self.active_task_info.get('pickup_pose')
            if pickup:
                px = pickup.pose.position.x
                py = pickup.pose.position.y
                dist = math.hypot(self.current_x - px, self.current_y - py)
                # Watchdog hard-floor: 0.35 m (one robot radius).
                # Only fire when the Nav2 goal handle has cleared so we do not
                # race a still-running Nav2 action.  0.8 m was too aggressive
                # and triggered pickup arrival before Nav2 finished decelerating.
                watchdog_tol = max(self.arrival_tolerance, 0.35)
                if dist <= watchdog_tol and self.current_goal_handle is None:
                    self.get_logger().info(
                        f'[{self.robot_id}] Proximity arrival watchdog triggered at PICKUP '
                        f'(dist: {dist:.2f}m <= {watchdog_tol:.2f}m, goal_handle cleared) '
                        f'for task {self.active_task_id}'
                    )
                    self._on_pickup_arrival()



def main(args=None):
    rclpy.init(args=args)
    node = TaskExecutionManager()
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