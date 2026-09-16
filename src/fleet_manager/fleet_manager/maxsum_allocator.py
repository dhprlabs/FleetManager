#!/usr/bin/env python3
"""
Max-Sum Allocator Node — Dynamic Task Reallocation (Phase 14 Implementation)
----------------------------------------------------------------------------
Runs identically on each robot.
Performs event-driven dynamic task reallocation using Damped Binary Max-Sum.

CRITICAL PRINCIPLES:
  1. Max-Sum does NOT continuously re-optimize the entire fleet on timer ticks or robot movement.
  2. Reallocation is strictly EVENT-DRIVEN.
     Triggers:
       - New AVAILABLE task(s)
       - BLOCKED task (navigation failure, repeated replanning, excessive energy, physical conflict)
       - Robot late join / reconnection with pending tasks
       - Task completion boundary (robot becomes idle)
       - Pickup validation re-bid (Phase 11 item absence)
       - Configured allocation event / execution failure
  3. Builds an AFFECTED sub-factor graph containing only affected tasks and eligible candidate robots.
  4. Cargo-committed tasks (PICKUP_COMPLETED, DELIVERING) are strictly protected;
     an en-route pickup may transfer only when the saving is material.
  5. Deterministic stability hysteresis bonus prevents ping-pong thrashing on tiny utility deltas.
  6. CRDT state and Bundle Manager are updated upon reallocation.
  7. Clear observable [REALLOCATION] logging.
"""

import json
import math
import time
from typing import Dict, List, Optional, Set, Tuple

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from fleet_interfaces.msg import (
        MaxSumMessage,
        P2PNeighborList,
        PickupObservation,
        TaskOwnership,
        WorldView,
        Task,
        RobotState,
    )
    _ROS_AVAILABLE = True
except ImportError:
    _ROS_AVAILABLE = False
    class Node:
        def __init__(self, name: str): pass
    class Task:
        STATE_AVAILABLE = 0
        STATE_ALLOCATED = 1
        STATE_ASSIGNED = 1
        STATE_IN_PROGRESS = 2
        STATE_COMPLETED = 3
        STATE_FAILED = 4
        STATE_CANCELLED = 5
        STATE_PICKUP_COMPLETED = 6
        STATE_DELIVERING = 7
        STATE_BLOCKED = 8
    class TaskOwnership:
        STATUS_CLAIMED = 1
        STATUS_CONFIRMED = 2
        STATUS_RELEASED = 3
    class MaxSumMessage: pass
    class P2PNeighborList: pass
    class PickupObservation: pass
    class WorldView: pass
    class RobotState: pass
    class String:
        def __init__(self): self.data = ''

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
from fleet_manager.audit_log import audit_event
try:
    from fleet_manager.p2p_client import P2PClient
except ImportError:
    P2PClient = None


# ─────────────────────────────────────────────────────────────────────────────
# State Eligibility Constants
# ─────────────────────────────────────────────────────────────────────────────

# Physically committed states — strictly protected from reallocation
_COMMITTED_STATES = {
    Task.STATE_IN_PROGRESS,
    Task.STATE_PICKUP_COMPLETED,
    Task.STATE_DELIVERING,
}

# Terminal states — excluded from all allocation
_TERMINAL_STATES = {
    Task.STATE_COMPLETED,
    Task.STATE_CANCELLED,
    Task.STATE_FAILED,
}

# Eligible states for (re-)allocation
_ELIGIBLE_STATES = {
    Task.STATE_AVAILABLE,
    Task.STATE_BLOCKED,
}

# A task remains transferable until pickup is physically confirmed. The Nav2
# bridge marks the trip to the pickup pose IN_PROGRESS, so that state is also
# deliberately included here. PICKUP_COMPLETED and DELIVERING stay protected.
_PRE_PICKUP_STATES = _ELIGIBLE_STATES | {
    Task.STATE_ALLOCATED,
    Task.STATE_IN_PROGRESS,
}

# A join-triggered handoff of an already-dispatched pickup is worth the
# cancellation/restart cost only when it is clearly better.  At the fleet's
# nominal 0.6 m/s cap this corresponds to at least ten seconds and 20% sooner.
_PRE_PICKUP_HANDOFF_MIN_SAVING_S = 10.0
_PRE_PICKUP_HANDOFF_MIN_RATIO = 0.20
_NOMINAL_TRAVEL_SPEED_MPS = 0.6

# Trigger Types
TRIGGER_NEW_TASK = 'NEW_TASK'
TRIGGER_BLOCKED = 'BLOCKED'
TRIGGER_ROBOT_JOIN = 'ROBOT_JOIN'
TRIGGER_TASK_COMPLETED = 'TASK_COMPLETED'
TRIGGER_PICKUP_VALIDATION = 'PICKUP_VALIDATION_REBID'
TRIGGER_EXPLICIT = 'EXPLICIT_EVENT'
TRIGGER_EXECUTION_FAILURE = 'EXECUTION_FAILURE'


# ─────────────────────────────────────────────────────────────────────────────
# Dynamic Reallocation Coordinator
# ─────────────────────────────────────────────────────────────────────────────

class DynamicReallocationCoordinator:
    """
    Coordinates event-driven reallocation using Damped Binary Max-Sum
    on affected sub-problems with stability hysteresis.
    """
    def __init__(
        self,
        robot_id: str,
        config: Optional[MaxSumConfig] = None,
        weights: Optional[UtilityWeights] = None,
        log_callback: Optional[callable] = None,
        audit_callback: Optional[callable] = None,
    ):
        self.robot_id = robot_id
        self.config = config or MaxSumConfig(
            max_iterations=20,
            convergence_threshold=0.01,
            damping_factor=0.5,
            prune_threshold=15.0,
            hysteresis_bonus=0.15,
            verbose=False,
        )
        self.weights = weights or UtilityWeights(wd=0.35, wt=0.25, wb=0.25, wl=0.15)
        self.logger = log_callback or print
        self.audit = audit_callback or (lambda event, trace_id=None, **details: None)

        # Tracking state for change detection
        self.last_known_task_states: Dict[str, int] = {}       # task_id -> state
        self.last_known_task_owners: Dict[str, str] = {}       # task_id -> assigned_robot_id
        self.last_known_peers: Set[str] = set()
        self.current_ownership: Dict[str, str] = {}            # task_id -> owner_robot_id

    def detect_triggers(
        self,
        tasks: List[TaskInfo],
        task_states: Dict[str, int],
        task_owners: Dict[str, str],
        reachable_peers: Set[str],
    ) -> List[Tuple[str, List[str]]]:
        """
        Detects if any state change qualifies as an allocation trigger.
        Returns list of (trigger_type, affected_task_ids).
        """
        triggers: List[Tuple[str, List[str]]] = []

        # 1. Check for BLOCKED tasks
        blocked_tasks = []
        for tid, state in task_states.items():
            if state == Task.STATE_BLOCKED and self.last_known_task_states.get(tid) != Task.STATE_BLOCKED:
                blocked_tasks.append(tid)
        if blocked_tasks:
            triggers.append((TRIGGER_BLOCKED, blocked_tasks))

        # 2. Check for New AVAILABLE tasks
        new_avail = []
        for tid, state in task_states.items():
            if state == Task.STATE_AVAILABLE and (
                tid not in self.last_known_task_states or
                self.last_known_task_states[tid] not in _ELIGIBLE_STATES
            ):
                new_avail.append(tid)
        if new_avail:
            triggers.append((TRIGGER_NEW_TASK, new_avail))

        # 3. Check for Task Completion boundary
        completed_tasks = []
        for tid, state in task_states.items():
            if state == Task.STATE_COMPLETED and self.last_known_task_states.get(tid) != Task.STATE_COMPLETED:
                completed_tasks.append(tid)
        if completed_tasks:
            triggers.append((TRIGGER_TASK_COMPLETED, completed_tasks))

        # 4. A late join rebids every synchronized, unstarted task.  This
        # includes queued ALLOCATED work, but never pickup/execution work.
        new_peers = reachable_peers - self.last_known_peers
        has_reallocatable_tasks = any(s in _PRE_PICKUP_STATES for s in task_states.values())
        if new_peers and has_reallocatable_tasks:
            triggers.append((TRIGGER_ROBOT_JOIN, [t for t, s in task_states.items() if s in _PRE_PICKUP_STATES]))

        # Update cache
        self.last_known_task_states = dict(task_states)
        self.last_known_task_owners = dict(task_owners)
        self.last_known_peers = set(reachable_peers)

        return triggers

    def build_affected_problem(
        self,
        trigger: str,
        affected_task_ids: List[str],
        all_tasks: Dict[str, TaskInfo],
        task_states: Dict[str, int],
        task_owners: Dict[str, str],
        robots: List[RobotInfo],
        reachable_peers: Set[str],
    ) -> Tuple[List[TaskInfo], List[RobotInfo], Dict[str, str]]:
        """
        Constructs the affected sub-factor graph.
        Excludes:
          - Terminal tasks (COMPLETED, CANCELLED, FAILED)
          - Physically committed tasks (IN_PROGRESS, PICKUP_COMPLETED, DELIVERING)
          - Infeasible robots (low battery, out of P2P reach, unroutable)
        """
        # A join or explicit reallocation event may redistribute already queued work.
        # Other triggers only consider unowned/blocked work.
        allowed_states = _PRE_PICKUP_STATES if trigger in (TRIGGER_ROBOT_JOIN, TRIGGER_EXPLICIT) else _ELIGIBLE_STATES
        # Select affected tasks
        candidate_task_ids: Set[str] = set()
        if affected_task_ids:
            for tid in affected_task_ids:
                state = task_states.get(tid, Task.STATE_AVAILABLE)
                if state in allowed_states and state not in _TERMINAL_STATES and state not in _COMMITTED_STATES:
                    candidate_task_ids.add(tid)
        else:
            for tid, state in task_states.items():
                if state in allowed_states and state not in _TERMINAL_STATES and state not in _COMMITTED_STATES:
                    candidate_task_ids.add(tid)

        affected_tasks = [all_tasks[tid] for tid in candidate_task_ids if tid in all_tasks]

        # Select candidate robots
        candidate_robots: List[RobotInfo] = []
        for r in robots:
            # P2P connectivity filter: robot must be reachable or self
            if r.robot_id != self.robot_id and reachable_peers and r.robot_id not in reachable_peers:
                continue
            # Battery feasibility filter
            if r.battery_level < r.battery_min_threshold:
                continue
            candidate_robots.append(r)

        # Build incumbent map for stability hysteresis
        incumbents: Dict[str, str] = {}
        for t in affected_tasks:
            curr_owner = task_owners.get(t.task_id)
            if curr_owner:
                incumbents[t.task_id] = curr_owner

        return affected_tasks, candidate_robots, incumbents

    def run_reallocation(
        self,
        trigger: str,
        affected_task_ids: List[str],
        all_tasks: Dict[str, TaskInfo],
        task_states: Dict[str, int],
        task_owners: Dict[str, str],
        robots: List[RobotInfo],
        reachable_peers: Set[str],
    ) -> Dict[str, str]:
        """
        Runs Damped Binary Max-Sum on the affected sub-factor graph.
        Returns: {task_id: new_owner_robot_id}
        """
        affected_tasks, candidate_robots, incumbents = self.build_affected_problem(
            trigger=trigger,
            affected_task_ids=affected_task_ids,
            all_tasks=all_tasks,
            task_states=task_states,
            task_owners=task_owners,
            robots=robots,
            reachable_peers=reachable_peers,
        )

        if not affected_tasks or not candidate_robots:
            self.audit('allocation_skipped', trigger=trigger,
                       reason='no_eligible_tasks_or_robots',
                       affected_task_ids=sorted(affected_task_ids))
            return {}

        task_str = ",".join(t.task_id for t in affected_tasks)
        robots_str = ",".join(r.robot_id for r in candidate_robots)

        trace_id = f'{self.robot_id}:{trigger}:{time.monotonic_ns()}'
        # Observable reallocation logging
        self.logger(f"[REALLOCATION] Trigger={trigger} task={task_str}")
        self.audit('allocation_started', trace_id, trigger=trigger,
                   affected_tasks=sorted(t.task_id for t in affected_tasks),
                   eligible_robots=sorted(r.robot_id for r in candidate_robots),
                   reachable_peers=sorted(reachable_peers),
                   incumbents=incumbents,
                   weights={'distance': self.weights.wd, 'time': self.weights.wt,
                            'battery': self.weights.wb, 'workload': self.weights.wl},
                   config={'max_iterations': self.config.max_iterations,
                           'damping_factor': self.config.damping_factor,
                           'prune_threshold': self.config.prune_threshold})
        self.logger(f"[REALLOCATION] Eligible robots={robots_str}")
        self.logger(f"[REALLOCATION] Building affected factor graph")

        # Each binary solve permits one task per robot.  Repeat the solve over
        # remaining tasks while advancing virtual robot route/workload state,
        # so Max-Sum ownership can populate executable multi-task bundles.
        round_count = 0

        def _record_round(round_result: Dict):
            nonlocal round_count
            round_count = round_result['round']
            self.audit('binary_round_decided', trace_id, **round_result)

        new_ownership = run_multi_round_maxsum_allocation(
            robots=candidate_robots,
            tasks=affected_tasks,
            weights=self.weights,
            config=self.config,
            log_callback=lambda msg: None,
            incumbents=incumbents,
            round_callback=_record_round,
        )
        self.audit('allocation_decided', trace_id, allocation_mode='sequential_binary_maxsum',
                   rounds=round_count, ownership=new_ownership)

        self.logger(f"[REALLOCATION] Binary Max-Sum rounds={round_count}")

        for tid, winner in new_ownership.items():
            if winner:
                self.logger(f"[REALLOCATION] Winner={tid} -> {winner}")
                self.logger(f"[REALLOCATION] CRDT ownership update")
                self.logger(f"[REALLOCATION] Bundle updated")
                self.logger(f"[REALLOCATION] {winner} executing {tid}")

        # Update cache
        self.current_ownership.update(new_ownership)
        return new_ownership


# ─────────────────────────────────────────────────────────────────────────────
# ROS 2 MaxSumAllocator Node Implementation
# ─────────────────────────────────────────────────────────────────────────────

class MaxSumAllocator(Node):
    def __init__(self):
        super().__init__('maxsum_allocator')

        default_id = self.get_namespace().strip('/') or 'robot_1'
        self.declare_parameter('robot_id', default_id)
        self.declare_parameter('max_iterations', 20)
        self.declare_parameter('convergence_threshold', 0.01)
        self.declare_parameter('damping_factor', 0.5)
        self.declare_parameter('prune_threshold', 15.0)
        self.declare_parameter('hysteresis_bonus', 0.15)
        self.declare_parameter('weight_distance', 0.35)
        self.declare_parameter('weight_time', 0.25)
        self.declare_parameter('weight_battery', 0.25)
        self.declare_parameter('weight_workload', 0.15)

        self.robot_id = self.get_parameter('robot_id').get_parameter_value().string_value
        max_iter = self.get_parameter('max_iterations').get_parameter_value().integer_value
        conv_thresh = self.get_parameter('convergence_threshold').get_parameter_value().double_value
        damping = self.get_parameter('damping_factor').get_parameter_value().double_value
        prune_thresh = self.get_parameter('prune_threshold').get_parameter_value().double_value
        hysteresis = self.get_parameter('hysteresis_bonus').get_parameter_value().double_value

        self.config = MaxSumConfig(
            max_iterations=max_iter,
            convergence_threshold=conv_thresh,
            damping_factor=damping,
            prune_threshold=prune_thresh,
            hysteresis_bonus=hysteresis,
            verbose=False,
        )

        self.weights = UtilityWeights(
            wd=self.get_parameter('weight_distance').get_parameter_value().double_value,
            wt=self.get_parameter('weight_time').get_parameter_value().double_value,
            wb=self.get_parameter('weight_battery').get_parameter_value().double_value,
            wl=self.get_parameter('weight_workload').get_parameter_value().double_value,
        )

        self.coordinator = DynamicReallocationCoordinator(
            robot_id=self.robot_id,
            config=self.config,
            weights=self.weights,
            log_callback=self._log_info,
            audit_callback=self._audit,
        )

        # P2P Client
        if P2PClient is not None:
            self.p2p = P2PClient(node=self)
            self.p2p.register_handler('MAXSUM_MESSAGE', self._handle_p2p_maxsum)
            self.p2p.register_handler('ALLOCATION_DECISION', self._handle_p2p_allocation_decision)
        else:
            self.p2p = None

        # Subscriptions
        self.world_view_sub = self.create_subscription(
            WorldView, 'world_view', self._handle_world_view, 10
        )
        self.reachable_peers: Set[str] = set()
        self.neighbors_sub = self.create_subscription(
            P2PNeighborList, 'p2p_neighbors', self._handle_neighbors, 10
        )
        self.pickup_obs_sub = self.create_subscription(
            PickupObservation, '/fleet/pickup_observations', self._handle_pickup_observation, 10
        )

        # Publishers
        # NOTE: ownership is intentionally published only on the per-robot
        # namespaced 'task_ownership' topic (consumed locally) plus the P2P
        # TASK_OWNERSHIP broadcast (gated by radio reachability). A prior
        # '/fleet/task_ownership' global publisher was removed here: it bypassed
        # P2P partitioning entirely, letting out-of-range robots observe
        # ownership changes they had no connectivity basis for (split-brain).
        self.ownership_pub = self.create_publisher(TaskOwnership, 'task_ownership', 10)
        self.maxsum_msg_pub = self.create_publisher(MaxSumMessage, 'maxsum_messages', 20)
        self.decision_log_pub = self.create_publisher(String, '/fleet/decision_log', 50)

        # Local state
        self.latest_world_view: Optional[WorldView] = None
        self.my_owned_tasks: List[str] = []
        self.task_cache: Dict[str, TaskInfo] = {}
        # A P2P neighbour is eligible only after its synchronized state appears
        # in WorldView.  This avoids allocating during the join/sync race.
        self.pending_join_peers: Set[str] = set()
        self._seen_decision_ids: Set[str] = set()

        self.get_logger().info(
            f'[{self.robot_id}] Dynamic Event-Driven Max-Sum Allocator initialized. '
            f'MaxIter={max_iter} | Hysteresis={hysteresis}'
        )

    def _log_info(self, msg: str):
        self.get_logger().info(f'[{self.robot_id}] {msg}')

    def _audit(self, event: str, trace_id: Optional[str] = None, **details):
        record = audit_event(self.get_logger(), 'maxsum_allocator', event, self.robot_id,
                             trace_id=trace_id, **details)
        if hasattr(self, 'decision_log_pub'):
            message = String()
            message.data = json.dumps(record, default=str, separators=(',', ':'))
            self.decision_log_pub.publish(message)

    def _handle_neighbors(self, msg: P2PNeighborList):
        new_peers = set(msg.reachable_peers)
        if new_peers != self.reachable_peers:
            previous = set(self.reachable_peers)
            previous_peers = sorted(previous)
            joined_peers = new_peers - previous
            disconnected_peers = previous - new_peers
            self.reachable_peers = new_peers
            self._audit('peer_topology_changed', previous_peers=previous_peers,
                        reachable_peers=sorted(new_peers), joined_peers=sorted(joined_peers),
                        disconnected_peers=sorted(disconnected_peers))
            if joined_peers:
                self.pending_join_peers.update(joined_peers)
                self._audit('join_waiting_for_state_sync', joined_peers=sorted(joined_peers),
                            pending_join_peers=sorted(self.pending_join_peers))
            if disconnected_peers:
                self.pending_join_peers.difference_update(disconnected_peers)

    def _handle_pickup_observation(self, obs: PickupObservation):
        """Phase 11 integration: triggers reallocation if item is missing / requires re-bid."""
        if not obs.item_present:
            self.get_logger().warn(
                f'[{self.robot_id}] [REALLOCATION] Pickup observation reported task {obs.task_id} missing. '
                f'Triggering dynamic reallocation.'
            )
            self._release_task_locally(obs.task_id)
            self._execute_reallocation(TRIGGER_PICKUP_VALIDATION, [obs.task_id])

    def _handle_p2p_maxsum(self, msg):
        pass

    def _handle_p2p_allocation_decision(self, msg):
        """Commit a reachable peer's deterministic allocation result locally."""
        try:
            decision = json.loads(msg.payload)
            task_id = decision['task_id']
            winner = decision['winner_robot_id']
            decision_id = decision['decision_id']
        except (TypeError, ValueError, KeyError) as error:
            self._audit('allocation_decision_rejected', level='warn', reason='invalid_payload',
                        source_robot_id=msg.source_robot_id, error=str(error))
            return
        if winner != self.robot_id or decision_id in self._seen_decision_ids:
            return
        self._seen_decision_ids.add(decision_id)
        trigger = decision.get('trigger', 'P2P_ALLOCATION_DECISION')
        if not self._task_is_reallocatable(task_id, trigger):
            self._audit('allocation_decision_rejected', level='warn', task_id=task_id,
                        decision_id=decision_id, source_robot_id=msg.source_robot_id,
                        reason='task_is_no_longer_reallocatable')
            return
        self._claim_task_locally(task_id, trigger,
                                 decision_id, source_robot_id=msg.source_robot_id)

    def _task_is_reallocatable(self, task_id: str, trigger: str) -> bool:
        if not self.latest_world_view:
            return False
        allowed_states = _PRE_PICKUP_STATES if trigger == TRIGGER_ROBOT_JOIN else _ELIGIBLE_STATES
        return any(t.task_id == task_id and t.state in allowed_states
                   for t in self.latest_world_view.task_states)

    def _handle_world_view(self, msg: WorldView):
        self.latest_world_view = msg
        world_view_peer_ids = {peer.robot_id for peer in msg.peer_states}
        synchronized_peers = self.reachable_peers & world_view_peer_ids
        waiting_for_sync = self.pending_join_peers - world_view_peer_ids
        synchronized_joins = self.pending_join_peers & synchronized_peers
        self._audit('world_view_received', task_count=len(msg.task_states),
                    peer_count=len(msg.peer_states), reachable_peers=sorted(self.reachable_peers),
                    synchronized_peers=sorted(synchronized_peers))
        if waiting_for_sync:
            self._audit('join_state_sync_pending', pending_join_peers=sorted(waiting_for_sync),
                        world_view_peers=sorted(world_view_peer_ids))
        if synchronized_joins:
            self.pending_join_peers.difference_update(synchronized_joins)
            self._audit('join_state_sync_complete', joined_peers=sorted(synchronized_joins),
                        synchronized_peers=sorted(synchronized_peers))
        # Only expose peers whose current state is present.  detect_triggers
        # will therefore emit ROBOT_JOIN on this synchronized snapshot, not on
        # a raw P2P topology event.
        self._check_and_trigger(reachable_peers=synchronized_peers)

    def _check_and_trigger(self, trigger_hint: Optional[str] = None,
                           reachable_peers: Optional[Set[str]] = None):
        """Checks if world view changes warrant an event-driven reallocation."""
        if not self.latest_world_view:
            return

        task_states: Dict[str, int] = {}
        task_owners: Dict[str, str] = {}
        all_tasks: Dict[str, TaskInfo] = {}

        for t in self.latest_world_view.task_states:
            task_states[t.task_id] = t.state
            task_owners[t.task_id] = t.assigned_robot_id
            ti = TaskInfo(
                task_id=t.task_id,
                pickup_x=t.pickup_pose.pose.position.x,
                pickup_y=t.pickup_pose.pose.position.y,
                delivery_x=t.dropoff_pose.pose.position.x,
                delivery_y=t.dropoff_pose.pose.position.y,
                priority=t.priority,
            )
            all_tasks[t.task_id] = ti
            self.task_cache[t.task_id] = ti

        synchronized_peers = self.reachable_peers if reachable_peers is None else reachable_peers

        # Detect triggers
        triggers = self.coordinator.detect_triggers(
            tasks=list(all_tasks.values()),
            task_states=task_states,
            task_owners=task_owners,
            reachable_peers=synchronized_peers,
        )

        if trigger_hint and not triggers:
            triggers = [(trigger_hint, [t for t, s in task_states.items() if s in _ELIGIBLE_STATES])]

        for trig_type, affected_ids in triggers:
            self._audit('allocation_triggered', trigger=trig_type,
                        affected_tasks=sorted(affected_ids),
                        trigger_hint=trigger_hint or '',
                        synchronized_peers=sorted(synchronized_peers))
            # If BLOCKED and we owned it, safely release
            if trig_type == TRIGGER_BLOCKED:
                for bid in affected_ids:
                    if bid in self.my_owned_tasks:
                        self._release_task_locally(bid)

            self._execute_reallocation(trig_type, affected_ids)

    def _release_task_locally(self, task_id: str):
        """Safely releases a task locally and notifies fleet."""
        if task_id in self.my_owned_tasks:
            self.my_owned_tasks.remove(task_id)

        msg = TaskOwnership()
        msg.task_id = task_id
        msg.robot_id = self.robot_id
        msg.status = TaskOwnership.STATUS_RELEASED
        msg.timestamp = self.get_clock().now().to_msg()
        self.ownership_pub.publish(msg)
        self._audit('task_released', task_id=task_id, reason='reallocation')

        if self.p2p:
            self.p2p.broadcast('TASK_OWNERSHIP', payload={
                'task_id': task_id,
                'robot_id': self.robot_id,
                'status': TaskOwnership.STATUS_RELEASED,
                'timestamp': self.get_clock().now().nanoseconds,
            })
            self.p2p.broadcast('TASK_STATUS', payload={
                'task_id': task_id,
                'state': Task.STATE_AVAILABLE,
                'assigned_robot_id': '',
                'lamport_clock': 0,
                'source_robot_id': self.robot_id,
            })

    def _execute_reallocation(self, trigger: str, affected_task_ids: List[str]):
        """Runs the affected Max-Sum problem and updates ownership."""
        if not self.latest_world_view:
            return

        own = self.latest_world_view.own_state
        # Every node must calculate workload from the same replicated task
        # view.  Giving peers workload=0 made otherwise identical nodes solve
        # different allocation problems after synchronization.
        workloads: Dict[str, int] = {}
        for task in self.latest_world_view.task_states:
            if task.state in _PRE_PICKUP_STATES and task.assigned_robot_id:
                workloads[task.assigned_robot_id] = workloads.get(task.assigned_robot_id, 0) + 1
        robots_info: List[RobotInfo] = [
            RobotInfo(
                robot_id=own.robot_id,
                x=own.current_pose.pose.position.x,
                y=own.current_pose.pose.position.y,
                battery_level=own.battery_level,
                current_workload=workloads.get(own.robot_id, 0),
            )
        ]

        for peer in self.latest_world_view.peer_states:
            # WorldView may retain a peer briefly after radio loss.  A
            # decentralized solve may use only direct, currently reachable
            # peers; otherwise an isolated robot could assign work remotely.
            if peer.robot_id not in self.reachable_peers:
                continue
            robots_info.append(RobotInfo(
                robot_id=peer.robot_id,
                x=peer.current_pose.pose.position.x,
                y=peer.current_pose.pose.position.y,
                battery_level=peer.battery_level,
                current_workload=workloads.get(peer.robot_id, 0),
            ))

        task_states = {t.task_id: t.state for t in self.latest_world_view.task_states}
        task_owners = {t.task_id: t.assigned_robot_id for t in self.latest_world_view.task_states}

        new_ownership = self.coordinator.run_reallocation(
            trigger=trigger,
            affected_task_ids=affected_task_ids,
            all_tasks=self.task_cache,
            task_states=task_states,
            task_owners=task_owners,
            robots=robots_info,
            reachable_peers=self.reachable_peers,
        )

        # A task already travelling to pickup can change hands, but do not
        # interrupt it for a marginally better peer.  Every synchronized node
        # has the same pose/task snapshot, so this filter remains deterministic.
        if trigger == TRIGGER_ROBOT_JOIN:
            robot_by_id = {robot.robot_id: robot for robot in robots_info}
            for task_id, winner in list(new_ownership.items()):
                incumbent = task_owners.get(task_id, '')
                if (task_states.get(task_id) == Task.STATE_IN_PROGRESS and incumbent and
                        winner != incumbent and not self._is_material_pre_pickup_handoff(
                            task_id, incumbent, winner, robot_by_id)):
                    new_ownership[task_id] = incumbent
                    self._audit('pre_pickup_handoff_skipped', task_id=task_id,
                                incumbent=incumbent, challenger=winner,
                                reason='insufficient_arrival_saving')

        # If the deterministic result moves one of our queued tasks, release
        # it before the winner claims it.  This leaves in-progress/picked-up
        # tasks untouched and lets each bundle converge from ownership events.
        states = {task.task_id: task.state for task in self.latest_world_view.task_states}
        for task_id in list(self.my_owned_tasks):
            winner = new_ownership.get(task_id)
            if winner and winner != self.robot_id and states.get(task_id) in _PRE_PICKUP_STATES:
                self._release_task_locally(task_id)
                self._audit('ownership_transferred', task_id=task_id,
                            previous_owner=self.robot_id, new_owner=winner,
                            trigger=trigger)

        for task_id, winner in new_ownership.items():
            decision_id = (
                f'{self.robot_id}:{trigger}:{task_id}:'
                f'{self.get_clock().now().nanoseconds}'
            )
            if winner == self.robot_id:
                self._claim_task_locally(task_id, trigger, decision_id)
            elif self.p2p:
                self.p2p.send_to(winner, 'ALLOCATION_DECISION', payload={
                    'decision_id': decision_id,
                    'task_id': task_id,
                    'winner_robot_id': winner,
                    'trigger': trigger,
                })
                self._audit('allocation_decision_sent', task_id=task_id, winner=winner,
                            decision_id=decision_id, trigger=trigger)

    def _is_material_pre_pickup_handoff(self, task_id: str, incumbent: str,
                                        challenger: str,
                                        robots: Dict[str, RobotInfo]) -> bool:
        """True when the challenger reaches this pickup materially sooner."""
        task = self.task_cache.get(task_id)
        old_robot = robots.get(incumbent)
        new_robot = robots.get(challenger)
        if not task or not old_robot or not new_robot:
            return False
        old_eta = math.hypot(old_robot.x - task.pickup_x, old_robot.y - task.pickup_y) / _NOMINAL_TRAVEL_SPEED_MPS
        new_eta = math.hypot(new_robot.x - task.pickup_x, new_robot.y - task.pickup_y) / _NOMINAL_TRAVEL_SPEED_MPS
        saving = old_eta - new_eta
        return saving >= _PRE_PICKUP_HANDOFF_MIN_SAVING_S and saving / max(old_eta, 1.0) >= _PRE_PICKUP_HANDOFF_MIN_RATIO

    def _claim_task_locally(self, task_id: str, trigger: str, decision_id: str,
                            source_robot_id: str = ''):
        """Publish the sole owner commitment and replicate it to reachable peers."""
        if task_id in self.my_owned_tasks:
            return
        self.my_owned_tasks.append(task_id)
        msg = TaskOwnership()
        msg.task_id = task_id
        msg.robot_id = self.robot_id
        msg.status = TaskOwnership.STATUS_CLAIMED
        msg.timestamp = self.get_clock().now().to_msg()
        self.ownership_pub.publish(msg)
        self._audit('ownership_claim_published', task_id=task_id, winner=self.robot_id,
                    trigger=trigger, decision_id=decision_id, source_robot_id=source_robot_id)
        if self.p2p:
            self.p2p.broadcast('TASK_OWNERSHIP', payload={
                'task_id': task_id, 'robot_id': self.robot_id,
                'status': TaskOwnership.STATUS_CLAIMED,
                'timestamp': self.get_clock().now().nanoseconds,
            })
            t_info = self.task_cache.get(task_id)
            self.p2p.broadcast('TASK_STATUS', payload={
                'task_id': task_id, 'state': Task.STATE_ALLOCATED,
                'assigned_robot_id': self.robot_id,
                'priority': t_info.priority if t_info else 1,
                'lamport_clock': 0, 'source_robot_id': self.robot_id,
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