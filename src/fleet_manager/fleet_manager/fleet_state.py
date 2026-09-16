#!/usr/bin/env python3
"""
Fleet State — Distributed World View Library (Phase 10.1: CRDT-backed)
------------------------------------------------------------------------
Pure-Python, no ROS dependencies.

Implements:
  - Lamport logical clock (unchanged from Phase 3)
  - LWW-Element-Set CRDT for task state (via crdt_store.LWWElementSet)
  - Snapshot LWW for robot pose beacons (unchanged — too high frequency
    to include in the op log without unbounded growth)
  - Full world-view serialisation / deserialisation (JSON ↔ ROS messages)
    Extended: world_view_to_dict() now includes CRDT op logs (ops_tasks,
    ops_robots) so peers can replay operations on reconnect.

Public API is fully backward-compatible with the Phase 3 version.

Used by state_manager, task_manager, and all coordination modules.
"""

import json
import time
from dataclasses import dataclass, field, asdict
from typing import Dict, List, Optional, Tuple

from fleet_manager.crdt_store import (
    LWWElementSet,
    CrdtOp,
    CrdtOpType,
    ApplyResult,
    ReplayResult,
)


# ─────────────────────────────────────────────────────────────────────────────
# Data Entries  (unchanged from Phase 3 — keep same field names)
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class RobotEntry:
    """In-memory representation of a known robot's state."""
    robot_id: str
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    qx: float = 0.0
    qy: float = 0.0
    qz: float = 0.0
    qw: float = 1.0
    vx: float = 0.0
    vy: float = 0.0
    battery: float = 100.0
    status: int = 0
    task_id: str = ''
    task_state: str = ''
    dest_x: float = 0.0
    dest_y: float = 0.0
    lamport_clock: int = 0
    wall_time: float = 0.0
    source_robot_id: str = ''


@dataclass
class TaskEntry:
    """In-memory representation of a known task's state."""
    task_id: str
    state: int = 0
    assigned_robot_id: str = ''
    version: int = 0
    priority: int = 1
    pickup_x: float = 0.0
    pickup_y: float = 0.0
    dropoff_x: float = 0.0
    dropoff_y: float = 0.0
    lamport_clock: int = 0
    wall_time: float = 0.0
    source_robot_id: str = ''


# ─────────────────────────────────────────────────────────────────────────────
# Task state rank (unchanged from Phase 3)
# ─────────────────────────────────────────────────────────────────────────────

TASK_STATE_RANK = {
    0: 0,  # STATE_AVAILABLE
    1: 1,  # STATE_ALLOCATED / STATE_ASSIGNED
    2: 2,  # STATE_IN_PROGRESS
    6: 3,  # STATE_PICKUP_COMPLETED
    7: 4,  # STATE_DELIVERING
    3: 5,  # STATE_COMPLETED  (Terminal)
    4: 5,  # STATE_FAILED     (Terminal)
    5: 5,  # STATE_CANCELLED  (Terminal)
}


def get_state_rank(state: int) -> int:
    """Returns monotonic lifecycle rank of a task state."""
    return TASK_STATE_RANK.get(int(state), int(state))


# ─────────────────────────────────────────────────────────────────────────────
# Internal helpers: entry ↔ dict payload
# ─────────────────────────────────────────────────────────────────────────────

def _task_entry_to_payload(e: TaskEntry) -> dict:
    return {
        'task_id':          e.task_id,
        'state':            e.state,
        'assigned_robot_id': e.assigned_robot_id,
        'version':          e.version,
        'priority':         e.priority,
        'pickup_x':         e.pickup_x,
        'pickup_y':         e.pickup_y,
        'dropoff_x':        e.dropoff_x,
        'dropoff_y':        e.dropoff_y,
        'lamport_clock':    e.lamport_clock,
        'wall_time':        e.wall_time,
        'source_robot_id':  e.source_robot_id,
    }


def _task_entry_from_payload(d: dict) -> TaskEntry:
    return TaskEntry(
        task_id          = d.get('task_id', ''),
        state            = int(d.get('state', 0)),
        assigned_robot_id= d.get('assigned_robot_id', ''),
        version          = int(d.get('version', 0)),
        priority         = int(d.get('priority', 1)),
        pickup_x         = float(d.get('pickup_x', 0.0)),
        pickup_y         = float(d.get('pickup_y', 0.0)),
        dropoff_x        = float(d.get('dropoff_x', 0.0)),
        dropoff_y        = float(d.get('dropoff_y', 0.0)),
        lamport_clock    = int(d.get('lamport_clock', 0)),
        wall_time        = float(d.get('wall_time', 0.0)),
        source_robot_id  = d.get('source_robot_id', ''),
    )


def _robot_entry_to_payload(e: RobotEntry) -> dict:
    return asdict(e)


def _robot_entry_from_payload(d: dict) -> RobotEntry:
    return RobotEntry(**{k: v for k, v in d.items() if k in RobotEntry.__dataclass_fields__})


# ─────────────────────────────────────────────────────────────────────────────
# Fleet State Store
# ─────────────────────────────────────────────────────────────────────────────

class FleetState:
    """
    Distributed World View Store.

    Phase 10.1 upgrade: task state is now backed by an operation-based
    LWW-Element-Set CRDT (crdt_store.LWWElementSet).  Robot state keeps
    the existing snapshot LWW (pose beacons update too frequently for an
    unbounded op log).

    Public API is fully backward-compatible with the Phase 3 version.
    Additional API:
      - merge_world_view() now accepts and processes CRDT op logs.
      - world_view_to_dict() now includes 'ops_tasks' (and 'ops_robots'
        for robot CRDT compat) in its output.
      - apply_task_op(op) / apply_robot_op(op) for direct CRDT apply.
    """

    def __init__(self, robot_id: str, log_callback=None):
        self.robot_id = robot_id
        self._lamport = 0

        # Callback for [CRDT] log lines — injected by StateManager
        self._log_cb = log_callback or (lambda m: None)

        # ── CRDT stores ──────────────────────────────────────────────────
        self._crdt_tasks = LWWElementSet(is_task_set=True,  log_callback=self._log_cb)
        self._crdt_robots = LWWElementSet(is_task_set=False, log_callback=self._log_cb)

        # ── Snapshot LWW for robot states (fast pose beacons)  ───────────
        # _robots mirrors _crdt_robots materialized view, but is the primary
        # store because robots update too frequently for op-log accumulation.
        # We keep _crdt_robots as a secondary log for reconnect op exchange.
        self._robots: Dict[str, RobotEntry] = {}

    # ─── Lamport clock ─────────────────────────────────────────────────────

    @property
    def clock(self) -> int:
        return self._lamport

    def tick(self) -> int:
        """Increment own Lamport clock (local event)."""
        self._lamport += 1
        return self._lamport

    def advance(self, received_clock: int) -> int:
        """Advance clock on message receipt: L = max(L, L_msg) + 1."""
        self._lamport = max(self._lamport, received_clock) + 1
        return self._lamport

    # ─── LWW helper (kept for robot snapshot merging) ──────────────────────

    @staticmethod
    def _lww_wins(ic: int, is_: str, ec: int, es: str) -> bool:
        if ic > ec:
            return True
        if ic == ec:
            return is_ > es
        return False

    # ─── Robot state (snapshot LWW — unchanged behaviour) ──────────────────

    def merge_robot(self, entry: RobotEntry) -> bool:
        """Merge a received robot state entry using snapshot LWW."""
        existing = self._robots.get(entry.robot_id)
        if existing is None or self._lww_wins(
            entry.lamport_clock, entry.source_robot_id,
            existing.lamport_clock, existing.source_robot_id,
        ):
            self._robots[entry.robot_id] = entry
            self.advance(entry.lamport_clock)
            # Also feed into CRDT robot log for reconnect sync
            self._crdt_robots.add(
                entity_id    = entry.robot_id,
                origin_robot = entry.source_robot_id or entry.robot_id,
                lamport      = entry.lamport_clock or self._lamport,
                payload      = _robot_entry_to_payload(entry),
            )
            return True
        return False

    def update_own_robot(self, entry: RobotEntry) -> RobotEntry:
        """Update own robot state, auto-tick clock."""
        self.tick()
        entry.lamport_clock  = self._lamport
        entry.wall_time      = time.time()
        entry.source_robot_id = self.robot_id
        self._robots[self.robot_id] = entry
        self._crdt_robots.add(
            entity_id    = self.robot_id,
            origin_robot = self.robot_id,
            lamport      = self._lamport,
            payload      = _robot_entry_to_payload(entry),
        )
        return entry

    # ─── Task state (CRDT-backed) ───────────────────────────────────────────

    def merge_task(self, entry: TaskEntry) -> bool:
        """
        Merge a received task state entry via the CRDT.

        Drop-in replacement for the Phase 3 LWW merge:
        - Idempotent on duplicate Lamport+source combinations.
        - Physical-state lock enforced inside LWWElementSet.
        - Returns True if the materialized view changed.
        """
        self.advance(entry.lamport_clock)
        payload = _task_entry_to_payload(entry)
        ar = self._crdt_tasks.add(
            entity_id    = entry.task_id,
            origin_robot = entry.source_robot_id or self.robot_id,
            lamport      = entry.lamport_clock or self._lamport,
            payload      = payload,
        )
        return ar.materialized_changed

    def update_own_task(self, entry: TaskEntry) -> TaskEntry:
        """Update a task where this robot is the authority. Auto-ticks clock."""
        self.tick()
        entry.lamport_clock  = self._lamport
        entry.wall_time      = time.time()
        entry.source_robot_id = self.robot_id
        self._crdt_tasks.add(
            entity_id    = entry.task_id,
            origin_robot = self.robot_id,
            lamport      = self._lamport,
            payload      = _task_entry_to_payload(entry),
        )
        return entry

    def remove_task(self, task_id: str) -> bool:
        """
        Logically remove a task from the CRDT.
        Returns True if the remove was accepted (task existed and was not locked).
        """
        self.tick()
        ar = self._crdt_tasks.remove(
            entity_id    = task_id,
            origin_robot = self.robot_id,
            lamport      = self._lamport,
        )
        return ar.accepted

    # ─── Direct CRDT op apply (for reconnect replay) ───────────────────────

    def apply_task_op(self, op: CrdtOp) -> ApplyResult:
        """Apply a raw CRDT op to the task set."""
        ar = self._crdt_tasks.apply(op)
        if ar.materialized_changed:
            # Sync Lamport clock
            self.advance(op.lamport)
        return ar

    def apply_robot_op(self, op: CrdtOp) -> ApplyResult:
        """Apply a raw CRDT op to the robot set."""
        ar = self._crdt_robots.apply(op)
        if ar.materialized_changed:
            self.advance(op.lamport)
            # Sync snapshot store
            payload = self._crdt_robots.get(op.entity_id)
            if payload:
                self._robots[op.entity_id] = _robot_entry_from_payload(payload)
        return ar

    # ─── Bulk world-view merge (reconnect) ─────────────────────────────────

    def merge_world_view(self, robots: List[RobotEntry], tasks: List[TaskEntry],
                         ops_tasks: Optional[List[dict]] = None,
                         ops_robots: Optional[List[dict]] = None) -> dict:
        """
        Merge a bulk world-view snapshot received during reconnection.

        Priority order:
        1. Replay CRDT op logs (ops_tasks / ops_robots) — most complete.
        2. Fall back to snapshot entries for entities without op-log coverage.

        Returns a summary dict of changes made.
        """
        robot_updates = 0
        task_updates = 0
        allocation_changed = False

        # ── 1. Replay task CRDT ops (primary) ──────────────────────────
        if ops_tasks:
            rr = self._crdt_tasks.replay_ops(ops_tasks)
            task_updates += rr.new_ops
            if rr.allocation_changed:
                allocation_changed = True
            # Advance clock for all replayed ops
            for d in ops_tasks:
                self.advance(d.get('lamport', 0))

        # ── 2. Replay robot CRDT ops (secondary) ───────────────────────
        if ops_robots:
            rr_r = self._crdt_robots.replay_ops(ops_robots)
            robot_updates += rr_r.new_ops
            # Sync snapshot store for changed robots
            for eid in rr_r.changed_ids:
                payload = self._crdt_robots.get(eid)
                if payload and eid != self.robot_id:
                    self._robots[eid] = _robot_entry_from_payload(payload)

        # ── 3. Snapshot fallback for tasks not covered by op log ────────
        for t in tasks:
            existing = self.get_task(t.task_id)
            if existing is None:
                # Unknown task — ingest via CRDT
                if self.merge_task(t):
                    task_updates += 1
                    allocation_changed = True

        # ── 4. Snapshot fallback for robots ─────────────────────────────
        for r in robots:
            if r.robot_id != self.robot_id:
                if self.merge_robot(r):
                    robot_updates += 1

        return {
            'robot_updates':     robot_updates,
            'task_updates':      task_updates,
            'allocation_changed': allocation_changed,
        }

    # ─── Getters ────────────────────────────────────────────────────────────

    def get_robot(self, robot_id: str) -> Optional[RobotEntry]:
        return self._robots.get(robot_id)

    def get_own_state(self) -> Optional[RobotEntry]:
        return self._robots.get(self.robot_id)

    def get_all_robots(self) -> List[RobotEntry]:
        return list(self._robots.values())

    def get_peer_robots(self) -> List[RobotEntry]:
        return [r for rid, r in self._robots.items() if rid != self.robot_id]

    def get_task(self, task_id: str) -> Optional[TaskEntry]:
        payload = self._crdt_tasks.get(task_id)
        if payload is None:
            return None
        return _task_entry_from_payload(payload)

    def get_all_tasks(self) -> List[TaskEntry]:
        return [_task_entry_from_payload(p) for p in self._crdt_tasks.get_all()]

    def known_robot_ids(self) -> List[str]:
        return list(self._robots.keys())

    def known_task_ids(self) -> List[str]:
        return self._crdt_tasks.known_ids()

    # ─── Serialisation ───────────────────────────────────────────────────────

    def robot_entry_to_dict(self, e: RobotEntry) -> dict:
        return asdict(e)

    def task_entry_to_dict(self, e: TaskEntry) -> dict:
        return asdict(e)

    def robot_entry_from_dict(self, d: dict) -> RobotEntry:
        return _robot_entry_from_payload(d)

    def task_entry_from_dict(self, d: dict) -> TaskEntry:
        return _task_entry_from_payload(d)

    def world_view_to_dict(self) -> dict:
        """
        Serialise world view for P2P sync.
        Includes CRDT op logs (ops_tasks, ops_robots) in addition to
        the legacy snapshot lists (robots, tasks) for backward compat.
        """
        return {
            'robot_id':    self.robot_id,
            'lamport_clock': self._lamport,
            'robots':      [self.robot_entry_to_dict(r) for r in self.get_all_robots()],
            'tasks':       [self.task_entry_to_dict(t) for t in self.get_all_tasks()],
            # Phase 10.1 additions — CRDT op logs for precise reconciliation
            'ops_tasks':   self._crdt_tasks.dump_ops(),
            'ops_robots':  self._crdt_robots.dump_ops(),
        }

    def world_view_from_dict(self, d: dict) -> Tuple[List[RobotEntry], List[TaskEntry],
                                                      int, List[dict], List[dict]]:
        """
        Deserialise world view received from a peer.
        Returns (robots, tasks, peer_lamport, ops_tasks, ops_robots).
        Backward-compatible: ops_tasks / ops_robots default to [] if absent.
        """
        robots    = [self.robot_entry_from_dict(r) for r in d.get('robots', [])]
        tasks     = [self.task_entry_from_dict(t)  for t in d.get('tasks', [])]
        peer_clock = d.get('lamport_clock', 0)
        ops_tasks  = d.get('ops_tasks', [])
        ops_robots = d.get('ops_robots', [])
        return robots, tasks, peer_clock, ops_tasks, ops_robots

    # ─── ROS Message Converters (unchanged from Phase 3) ────────────────────

    def robot_entry_from_ros(self, ros_entry) -> RobotEntry:
        """Convert fleet_interfaces/msg/RobotWorldEntry to RobotEntry."""
        p = ros_entry.current_pose.pose
        v = ros_entry.current_velocity
        d = ros_entry.destination.pose
        return RobotEntry(
            robot_id     = ros_entry.robot_id,
            x=p.position.x, y=p.position.y, z=p.position.z,
            qx=p.orientation.x, qy=p.orientation.y,
            qz=p.orientation.z, qw=p.orientation.w,
            vx=v.linear.x, vy=v.linear.y,
            battery      = float(ros_entry.battery_level),
            status       = ros_entry.status,
            task_id      = ros_entry.current_task_id,
            task_state   = ros_entry.current_task_state,
            dest_x       = d.position.x, dest_y=d.position.y,
            lamport_clock= ros_entry.lamport_clock,
            wall_time    = ros_entry.wall_time.sec + ros_entry.wall_time.nanosec * 1e-9,
            source_robot_id = ros_entry.source_robot_id,
        )

    def task_entry_from_task_ros(self, ros_task) -> TaskEntry:
        """Convert fleet_interfaces/msg/Task to TaskEntry."""
        return TaskEntry(
            task_id      = ros_task.task_id,
            state        = ros_task.state,
            assigned_robot_id = ros_task.assigned_robot_id,
            version      = ros_task.version,
            priority     = ros_task.priority,
            pickup_x     = ros_task.pickup_pose.pose.position.x,
            pickup_y     = ros_task.pickup_pose.pose.position.y,
            dropoff_x    = ros_task.dropoff_pose.pose.position.x,
            dropoff_y    = ros_task.dropoff_pose.pose.position.y,
            lamport_clock= 0,
            wall_time    = time.time(),
            source_robot_id = 'task_broadcaster',
        )

    def __repr__(self):
        return (
            f'FleetState(robot_id={self.robot_id!r}, clock={self._lamport}, '
            f'robots={list(self._robots.keys())}, '
            f'tasks={self.known_task_ids()})'
        )
