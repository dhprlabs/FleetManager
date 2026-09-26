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
from std_msgs.msg import String

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

try:
    from fleet_manager.reservation_manager import DEFAULT_AISLE_SEGMENTS, ReservationManager
except ImportError:
    from fleet_manager.reservation_manager import DEFAULT_AISLE_SEGMENTS
    ReservationManager = None


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
        self.declare_parameter('broadcaster_x', 9.53)
        self.declare_parameter('broadcaster_y', -1.526)
        self.declare_parameter('communication_radius', 6.0)
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        # Extra distance (m) beyond the aisle bounds the robot must travel before
        # its aisle lease is released (~ robot half-length + slack).
        self.declare_parameter('aisle_exit_margin', 0.5)
        # SAME file ConflictResolver loads its aisle map from. Pass the same
        # value to both nodes from the launch file so they cannot diverge.
        self.declare_parameter(
            'aisle_config_file',
            '/home/mangal-devanshu/sih_ws/FleetManager/src/fleet_manager/config/aisle_segments.json'
        )

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
        self.aisle_exit_margin = self.get_parameter('aisle_exit_margin').get_parameter_value().double_value
        aisle_config_file = self.get_parameter('aisle_config_file').get_parameter_value().string_value
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

        # Traffic pause/resume tracking
        self._paused_for_traffic: bool = False
        self._traffic_pause_time: float = 0.0
        self._saved_target_pose: Optional[PoseStamped] = None
        self._saved_target_name: Optional[str] = None
        self._saved_on_reached = None

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

        # ── Aisle map ──────────────────────────────────────────────────────
        # Must be IDENTICAL to ConflictResolver's map, otherwise the bridge
        # releases the lease while the resolver still considers the robot inside.
        # 1) load the JSON the resolver uses, 2) fall back to package defaults,
        # 3) replace with /fleet/aisle_config (transient-local) when it arrives.
        self._aisle_segments, aisle_source = self._load_initial_aisles(aisle_config_file)

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
        self.dock_config_sub = self.create_subscription(
            String, '/fleet/dock_config', self._handle_dock_config, 10
        )
        # ReservationManager publishes the aisle map once, latched. A volatile
        # subscription would never receive it, so match the resolver's QoS.
        aisle_config_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.aisle_config_sub = self.create_subscription(
            String, '/fleet/aisle_config', self._handle_aisle_config_msg, aisle_config_qos
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
        self.fleet_res_pub = self.create_publisher(Reservation, '/fleet/reservations', 10)
        self.traffic_reserve_sub = self.create_subscription(
            Reservation, '/traffic/reserve', self._handle_traffic_reserve_msg, 10
        )
        self.fleet_res_sub = self.create_subscription(
            Reservation, '/fleet/reservations', self._handle_traffic_reserve_msg, 10
        )
        self.traffic_events_sub = self.create_subscription(
            String, '/fleet/traffic_events', self._handle_traffic_event_msg, 10
        )
        self._current_held_aisle: Optional[str] = None
        self._waiting_for_aisle: Optional[str] = None

        # Aisles whose lease is still held because the robot has not physically
        # left them yet (released by _check_pending_aisle_release once clear,
        # unless ConflictResolver releases it first).
        self._pending_release_segs: Set[str] = set()

        self._last_aisle_log_time = 0.0
        self._last_aisle_state = None

        self.get_logger().info(
            f'[{self.robot_id}] Task Execution Manager active | '
            f'Nav2 Target: /{self.robot_id}/navigate_to_pose | '
            f'Dwell: {self.pickup_dwell_sec}s | '
            f'Pickup validation: {self.enable_pickup_validation} | '
            f'Home Dock: ({self.dock_x:.2f}, {self.dock_y:.2f}) | '
            f'Aisle exit margin: {self.aisle_exit_margin}m'
        )
        self._log_aisle_bounds(aisle_source)

        # Execution evaluation timer
        self.eval_timer = self.create_timer(0.5, self._eval_execution_step)

    # ──────────────────────────────────────────────────────────────────────────
    # Aisle geometry helpers
    # ──────────────────────────────────────────────────────────────────────────

    def _load_initial_aisles(self, path: str) -> Tuple[Dict[str, dict], str]:
        """Loads the aisle map exactly like ConflictResolver does."""
        if ReservationManager is not None:
            try:
                specs = ReservationManager.load_aisle_specs(path)
                if specs:
                    return dict(specs), f'json file {path}'
                self.get_logger().warn(
                    f'[{self.robot_id}] [AISLES] {path} returned no aisles; using defaults.'
                )
            except Exception as e:
                self.get_logger().warn(
                    f'[{self.robot_id}] [AISLES] Could not load {path}: {e}; using defaults.'
                )
        else:
            self.get_logger().warn(
                f'[{self.robot_id}] [AISLES] ReservationManager not importable; using defaults.'
            )
        return dict(DEFAULT_AISLE_SEGMENTS), 'DEFAULT_AISLE_SEGMENTS (fallback)'

    def _get_aisle_at_point(self, x: float, y: float) -> Optional[str]:
        """Check if a point is inside any single-lane aisle segment."""
        for seg_id, spec in self._aisle_segments.items():
            if (spec['x_min'] <= x <= spec['x_max'] and
                spec['y_min'] <= y <= spec['y_max'] and
                spec.get('is_single_lane', True)):
                return seg_id
        return None

    def _get_aisle_bounds(self, segment_id: str) -> Optional[Tuple[float, float, float, float]]:
        """Get aisle bounds (x_min, x_max, y_min, y_max)."""
        spec = self._aisle_segments.get(segment_id)
        if spec:
            return (spec['x_min'], spec['x_max'], spec['y_min'], spec['y_max'])
        return None

    def _log_aisle_bounds(self, origin: str):
        """Logs every aisle's bounds so they can be compared with ConflictResolver's."""
        try:
            parts = []
            for seg_id in sorted(self._aisle_segments.keys()):
                b = self._get_aisle_bounds(seg_id)
                if b:
                    parts.append(f'{seg_id}=x[{b[0]:.2f},{b[1]:.2f}] y[{b[2]:.2f},{b[3]:.2f}]')
            self.get_logger().info(
                f'[{self.robot_id}] [AISLES] bounds from {origin}: ' + (' | '.join(parts) or '(none)')
            )
        except Exception as e:
            self.get_logger().warn(f'[{self.robot_id}] [AISLES] could not log bounds: {e}')

    def _handle_aisle_config_msg(self, msg: String):
        """Handles aisle map updates (normalised exactly like ConflictResolver)."""
        try:
            payload = json.loads(msg.data)
            raw_aisles = payload.get('aisles', payload)
            new_segments = {}

            if isinstance(raw_aisles, dict):
                for seg_id, spec in raw_aisles.items():
                    new_segments[seg_id] = {
                        'segment_id': seg_id,
                        'x_min': float(spec.get('x_min', 0.0)),
                        'x_max': float(spec.get('x_max', 0.0)),
                        'y_min': float(spec.get('y_min', 0.0)),
                        'y_max': float(spec.get('y_max', 0.0)),
                        'is_single_lane': bool(spec.get('is_single_lane', True)),
                    }
            elif isinstance(raw_aisles, list):
                for item in raw_aisles:
                    seg_id = item.get('segment_id') or item.get('id')
                    if not seg_id:
                        continue
                    new_segments[seg_id] = {
                        'segment_id': seg_id,
                        'x_min': float(item.get('x_min', 0.0)),
                        'x_max': float(item.get('x_max', 0.0)),
                        'y_min': float(item.get('y_min', 0.0)),
                        'y_max': float(item.get('y_max', 0.0)),
                        'is_single_lane': bool(item.get('is_single_lane', True)),
                    }

            if new_segments:
                self._aisle_segments = new_segments
                self._log_aisle_bounds('/fleet/aisle_config')
        except Exception as e:
            self.get_logger().warn(f'[{self.robot_id}] Error parsing /fleet/aisle_config: {e}')

    def _is_within_aisle(self, seg: str, margin: float = 0.0) -> Optional[bool]:
        """True if the robot is inside aisle `seg` expanded by `margin` metres.

        Returns None when the bridge has no bounds for `seg` (so the caller can
        fail safe instead of assuming the robot is outside).
        """
        spec = self._aisle_segments.get(seg)
        if not spec:
            return None
        return (spec['x_min'] - margin <= self.current_x <= spec['x_max'] + margin and
                spec['y_min'] - margin <= self.current_y <= spec['y_max'] + margin)

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
        Gazebo entity inspection disabled: directly confirm pickup and rely on
        fleet CRDT / P2P state for ownership and task completion tracking.
        """
        if self.execution_state != TaskExecutionState.IN_PROGRESS or not self.active_task_id:
            return
        self._goal_generation += 1
        self.current_goal_handle = None
        # Gazebo entity check bypassed - directly confirm pickup
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
        """Tracks reservation state for this robot and resumes paused navigation on a grant."""
        seg = msg.segment_id

        # Another robot now holds / is using the aisle: any release we had
        # pending is moot (we no longer hold it).
        if msg.robot_id != self.robot_id:
            if msg.state in (Reservation.STATE_GRANTED, Reservation.STATE_ACTIVE):
                self._pending_release_segs.discard(seg)
            return

        # ConflictResolver (or the manager) already released / expired OUR lease
        # (e.g. it auto-releases when the robot exits the aisle). Keep the bridge's
        # view in sync so we never send a duplicate release that could hit the next holder.
        if msg.state in (Reservation.STATE_RELEASED, Reservation.STATE_EXPIRED):
            self._pending_release_segs.discard(seg)
            if self._current_held_aisle == seg:
                self._current_held_aisle = None
            return

        if msg.state == Reservation.STATE_GRANTED:
            self._current_held_aisle = seg
            if self._waiting_for_aisle == seg:
                self._waiting_for_aisle = None
            # We hold this aisle again: cancel any pending exit-release for it.
            self._pending_release_segs.discard(seg)
            if self._paused_for_traffic and self._saved_target_pose is not None:
                self.get_logger().info(
                    f'[{self.robot_id}] [TRAFFIC] Reservation GRANTED for {seg}! '
                    f'Resuming navigation to {self._saved_target_name} from current position ({self.current_x:.2f}, {self.current_y:.2f}).'
                )
                self._paused_for_traffic = False
                target_pose = self._saved_target_pose
                target_name = self._saved_target_name
                on_reached = self._saved_on_reached
                self._saved_target_pose = None
                self._saved_target_name = None
                self._saved_on_reached = None
                self._dispatch_navigation(target_pose, target_name, on_reached)

    def _handle_traffic_event_msg(self, msg: String):
        """Reacts to traffic events from conflict_resolver.

        Blocking events (pibt_wait, orca_avoidance):
          - On first receipt: cancel the active Nav2 goal and enter traffic-pause.
          - On subsequent receipts while already paused: refresh the deadlock
            watchdog clock so the 60 s timeout measures *silence from the blocker*,
            not total accumulated wait time.  A legitimately blocked robot will
            keep receiving these events every ~100 ms; the watchdog fires only when
            the blocker stops signalling unexpectedly.

        Clearing events (conflict_resolved, traffic_clear):
          - Resume navigation only when a reservation grant is already held,
            so the robot does not re-enter a still-occupied aisle.
        """
        try:
            payload = json.loads(msg.data)
            robot_id = payload.get('robot_id')
            event = payload.get('event')
            if robot_id != self.robot_id:
                return

            # ── Blocking events ───────────────────────────────────────────────
            _BLOCKING_EVENTS = ('pibt_wait', 'orca_avoidance', 'apf_avoidance')
            if event in _BLOCKING_EVENTS:
                seg_id = payload.get('segment_id')
                if event == 'pibt_wait' and seg_id:
                    # Only track waiting_for_aisle if robot's target or current position is in that aisle
                    target_seg = None
                    if self._saved_target_pose is not None:
                        target_seg = self._get_aisle_at_point(
                            self._saved_target_pose.pose.position.x,
                            self._saved_target_pose.pose.position.y
                        )
                    robot_in_seg = bool(self._is_within_aisle(seg_id, self.aisle_exit_margin))
                    if robot_in_seg or target_seg == seg_id:
                        self._waiting_for_aisle = seg_id
                    else:
                        self._waiting_for_aisle = None
                elif event in ('orca_avoidance', 'apf_avoidance'):
                    self._waiting_for_aisle = None

                if not self._paused_for_traffic and self.current_goal_handle is not None:
                    # First block: cancel active Nav2 goal and enter pause.
                    self.get_logger().warn(
                        f'[{self.robot_id}] [TRAFFIC] Blocking event "{event}" — '
                        f'pausing Nav2 goal to prevent wheel-slip & recovery spins.'
                    )
                    self._paused_for_traffic = True
                    try:
                        self.current_goal_handle.cancel_goal_async()
                    except Exception as e:
                        self.get_logger().warn(
                            f'[{self.robot_id}] Could not cancel goal handle: {e}'
                        )
                    self.current_goal_handle = None
                # Always refresh the watchdog clock while actively blocked.
                # This way the deadlock timeout measures silence from the blocker,
                # not the total time the robot has been waiting.
                if self._paused_for_traffic:
                    self._traffic_pause_time = time.monotonic()

            # ── Clearing events ───────────────────────────────────────────────
            elif event in ('conflict_resolved', 'traffic_clear'):
                if self._paused_for_traffic and self._saved_target_pose is not None:
                    target_seg = self._get_aisle_at_point(
                        self._saved_target_pose.pose.position.x,
                        self._saved_target_pose.pose.position.y
                    )
                    waiting_aisle = self._waiting_for_aisle or target_seg

                    # Only require holding a reservation if the robot's target actually
                    # requires entering that aisle OR the robot is currently inside it.
                    robot_in_waiting_aisle = (
                        bool(waiting_aisle and self._is_within_aisle(waiting_aisle, self.aisle_exit_margin))
                    )
                    target_in_waiting_aisle = (target_seg == waiting_aisle and waiting_aisle is not None)
                    needs_grant = target_in_waiting_aisle or robot_in_waiting_aisle

                    if needs_grant:
                        if self._current_held_aisle and self._current_held_aisle == waiting_aisle:
                            self.get_logger().info(
                                f'[{self.robot_id}] [TRAFFIC] Conflict cleared + grant confirmed for {waiting_aisle}. '
                                f'Resuming navigation to {self._saved_target_name}.'
                            )
                            self._paused_for_traffic = False
                            self._waiting_for_aisle = None
                            target_pose = self._saved_target_pose
                            target_name = self._saved_target_name
                            on_reached = self._saved_on_reached
                            self._saved_target_pose = None
                            self._saved_target_name = None
                            self._saved_on_reached = None
                            self._dispatch_navigation(target_pose, target_name, on_reached)
                        else:
                            self.get_logger().info(
                                f'[{self.robot_id}] [TRAFFIC] "{event}" received but waiting for '
                                f'aisle reservation grant for {waiting_aisle} — staying paused until grant arrives.'
                            )
                    else:
                        # Open space conflict or vacated aisle cleared: safe to resume immediately
                        self.get_logger().info(
                            f'[{self.robot_id}] [TRAFFIC] Conflict cleared for {waiting_aisle or "open space"} '
                            f'(target not in aisle) — resuming navigation to {self._saved_target_name}.'
                        )
                        self._paused_for_traffic = False
                        self._waiting_for_aisle = None
                        target_pose = self._saved_target_pose
                        target_name = self._saved_target_name
                        on_reached = self._saved_on_reached
                        self._saved_target_pose = None
                        self._saved_target_name = None
                        self._saved_on_reached = None
                        self._dispatch_navigation(target_pose, target_name, on_reached)
        except Exception:
            pass

    # ──────────────────────────────────────────────────────────────────────────
    # Aisle lease release (held until the robot is physically out of the aisle)
    # ──────────────────────────────────────────────────────────────────────────

    def _publish_aisle_release(self, seg: str):
        """Publishes a RELEASED reservation for `seg` on the traffic and fleet topics."""
        # Unique id per release: peers dedupe by reservation_id, so a
        # constant id would silently drop every release after the first.
        now_sec = self.get_clock().now().nanoseconds / 1e9
        res = Reservation()
        res.reservation_id = f"{self.robot_id}:{seg}:release:{now_sec:.3f}"
        res.robot_id = self.robot_id
        res.segment_id = seg
        res.state = Reservation.STATE_RELEASED
        self.traffic_release_pub.publish(res)
        # ConflictResolver tracks holder state on the fleet topic; mirror
        # the release so all safety layers converge on the same lifecycle.
        self.fleet_res_pub.publish(res)
        self.get_logger().info(
            f'[{self.robot_id}] [TRAFFIC] Released aisle reservation for {seg} '
            f'at pos=({self.current_x:.2f},{self.current_y:.2f}) '
            f'bounds={self._get_aisle_bounds(seg)} margin={self.aisle_exit_margin}.'
        )

    def _release_aisle_if_held(self):
        """Releases the single-lane aisle lease, but only once the robot is
        physically clear of the aisle (plus `aisle_exit_margin`).

        - Still in/near the aisle  -> keep the lease; ConflictResolver releases it
          when the robot exits, or `_check_pending_aisle_release` does as a fallback.
        - Bounds for the aisle unknown to this node -> fail safe: do NOT publish a
          release (we cannot tell whether the robot is still inside) and leave it to
          ConflictResolver. Logged loudly so the config can be fixed.
        """
        seg = self._current_held_aisle
        if not seg:
            return
        self._current_held_aisle = None

        inside = self._is_within_aisle(seg, self.aisle_exit_margin)
        if inside is None:
            self.get_logger().error(
                f'[{self.robot_id}] [TRAFFIC] No bounds known for {seg} (known: '
                f'{sorted(self._aisle_segments.keys())}); NOT releasing the lease from the bridge. '
                f'Check aisle_config_file / /fleet/aisle_config.'
            )
            return

        if inside:
            self.get_logger().info(
                f'[{self.robot_id}] [TRAFFIC] Task ended but robot still in/near {seg} '
                f'at pos=({self.current_x:.2f},{self.current_y:.2f}) bounds={self._get_aisle_bounds(seg)}. '
                f'Holding lease until it exits.'
            )
            self._pending_release_segs.add(seg)
            return

        self._publish_aisle_release(seg)

    def _check_pending_aisle_release(self):
        """Releases any deferred aisle lease once the robot has left that aisle."""
        for seg in list(self._pending_release_segs):
            inside = self._is_within_aisle(seg, self.aisle_exit_margin)
            if inside is None:
                self.get_logger().error(
                    f'[{self.robot_id}] [TRAFFIC] Lost bounds for {seg} while release pending; '
                    f'dropping pending release (ConflictResolver must release it).'
                )
                self._pending_release_segs.discard(seg)
            elif not inside:
                self._pending_release_segs.discard(seg)
                self._publish_aisle_release(seg)

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

        # Release aisle reservation if held (deferred until the robot exits the aisle)
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
        self._saved_target_pose = target_pose
        self._saved_target_name = target_name
        self._saved_on_reached = on_reached
        # Clear stale aisle-wait from a previous navigation leg (e.g. pickup→delivery
        # transition). If the NEW route also needs an aisle, a fresh pibt_wait from
        # ConflictResolver will re-populate _waiting_for_aisle.
        self._waiting_for_aisle = None
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
        # If the goal cancellation was intentionally initiated due to traffic wait, do not abort
        if self._paused_for_traffic:
            self.get_logger().info(
                f'[{self.robot_id}] Nav2 goal pause acknowledged for traffic wait. Awaiting reservation grant.'
            )
            self.current_goal_handle = None
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
            # Nav2 aborted, cancelled, or failed.
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

    def _handle_dock_config(self, msg: String):
        """Update the task-broadcaster position used for radio-range checks."""
        try:
            config = json.loads(msg.data)
            x, y = float(config['x']), float(config['y'])
            if not (math.isfinite(x) and math.isfinite(y)):
                raise ValueError('coordinates must be finite')
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            self.get_logger().warn(f'Ignoring invalid dock configuration: {exc}')
            return
        self.broadcaster_x, self.broadcaster_y = x, y

    def _eval_execution_step(self):
        """Periodic safety check, proximity arrival watchdog, and dock-return fallback.

        Watchdog fires ONLY when the Nav2 goal handle has already cleared (i.e.
        Nav2 finished without the result callback firing).  This prevents the
        watchdog from racing an in-flight Nav2 goal and causing a premature
        arrival trigger.
        """
        tol = max(self.arrival_tolerance, 0.8)

        # Release any deferred aisle lease once the robot has physically left the
        # aisle. Must run before the IDLE early-return below, because a robot that
        # just finished its task is IDLE.
        self._check_pending_aisle_release()

        # ── Traffic pause watchdog (deadlock safety net) ──────────────────────
        # Resume ONLY as a last-resort if we've been paused for an unreasonably
        # long time (60s). Normal resume happens via _handle_traffic_reserve_msg
        # when a GRANTED reservation arrives, or via the conflict_resolved event
        # when a grant is already held. This timeout prevents indefinite parking
        # due to lost messages.
        if self._paused_for_traffic and self._saved_target_pose is not None:
            pause_dur = time.monotonic() - getattr(self, '_traffic_pause_time', 0.0)
            if pause_dur >= 60.0:
                self.get_logger().warn(
                    f'[{self.robot_id}] [TRAFFIC] Safety watchdog: paused {pause_dur:.0f}s — '
                    f'resuming navigation to {self._saved_target_name} (possible lost grant).'
                )
                self._paused_for_traffic = False
                target_pose = self._saved_target_pose
                target_name = self._saved_target_name
                on_reached = self._saved_on_reached
                self._saved_target_pose = None
                self._saved_target_name = None
                self._saved_on_reached = None
                self._dispatch_navigation(target_pose, target_name, on_reached)
                return

        # ── IDLE: dock-return fallback (Root Cause 2) ──────────────────────────
        # When the robot has no active task and is outside comm_radius of the dock
        # it cannot receive new task broadcasts or gossip with peers.  Navigate it
        # back to the dock so it re-enters the radio mesh.
        if self.execution_state == TaskExecutionState.IDLE and not self.active_task_id:
            dist_to_dock = math.hypot(self.current_x - self.dock_x, self.current_y - self.dock_y)
            dock_arrival_tol = max(self.arrival_tolerance, 0.8)
            if dist_to_dock > dock_arrival_tol and not self._dock_return_active:
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
            elif dist_to_dock <= dock_arrival_tol and self._dock_return_active:
                # Arrived back within dock range; the arrival callback will clear the flag.
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

        # ─────────────────────────────────────────────────────────────────────
        # SESSION LOGGING: Aisle entry/exit detection
        # ─────────────────────────────────────────────────────────────────────
        current_aisle = self._get_aisle_at_point(self.current_x, self.current_y)
        now = time.monotonic()

        # Log aisle state changes
        if current_aisle != self._last_aisle_state:
            self.get_logger().info(
                f'[{self.robot_id}] [SESSION] Aisle change: {self._last_aisle_state} -> {current_aisle} '
                f'at pos=({self.current_x:.2f},{self.current_y:.2f}) '
                f'task={self.active_task_id} state={self.execution_state.value}'
            )
            self._last_aisle_state = current_aisle

        # Periodic position log every 3 seconds when in an aisle
        if current_aisle and (now - self._last_aisle_log_time) > 3.0:
            self._last_aisle_log_time = now
            bounds = self._get_aisle_bounds(current_aisle)
            if bounds:
                self.get_logger().info(
                    f'[{self.robot_id}] [SESSION] In aisle {current_aisle} '
                    f'bounds=({bounds[0]:.1f},{bounds[1]:.1f},{bounds[2]:.1f},{bounds[3]:.1f}) '
                    f'pos=({self.current_x:.2f},{self.current_y:.2f}) '
                    f'task={self.active_task_id} state={self.execution_state.value} '
                    f'held_aisle={self._current_held_aisle}'
                )

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
