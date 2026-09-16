#!/usr/bin/env python3
"""
Operation-Based LWW-Element-Set CRDT
--------------------------------------
Pure Python — no ROS dependencies.

This module implements the core CRDT primitives used by FleetState to make
distributed task state synchronisation correct under:
  - Duplicate operation delivery     (idempotent apply)
  - Out-of-order operation delivery  (Lamport ordering wins regardless)
  - Delayed / disconnected ops       (log replay at reconnect)
  - Concurrent conflicting updates   (deterministic LWW tie-break)

Design
------
* An *operation* is an immutable record that captures what happened, who
  generated it, and when (Lamport clock).
* The CRDT log is the source of truth.  The *materialized state* (TaskEntry /
  RobotEntry dicts) is derived by replaying the log.
* LWW ordering:  higher Lamport clock wins.  Equal clocks: lexicographically
  larger origin_robot wins.  This is deterministic, commutative, and associative.
* Physical-state lock:  once a task reaches IN_PROGRESS or beyond, no stale
  op may revert it to a lower-rank state.

Public API
----------
CrdtOp            — immutable operation record
LWWElementSet     — per-entity-type CRDT log + materialized view
  .apply(op)      → ApplyResult  (idempotent)
  .remove(entity_id, origin, lamport, payload)  → ApplyResult
  .get(entity_id) → dict | None
  .get_all()      → List[dict]  (all non-removed entities)
  .dump_ops()     → List[dict]  (for P2P transfer)
  .replay_ops(ops_list) → ReplayResult
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field, asdict
from enum import IntEnum
from typing import Dict, List, Optional, Set, Tuple


# ---------------------------------------------------------------------------
# Task state rank — mirrors fleet_state.TASK_STATE_RANK
# Must stay in sync if fleet_state changes enum values.
# ---------------------------------------------------------------------------

_TASK_STATE_RANK: Dict[int, int] = {
    0: 0,  # AVAILABLE
    1: 1,  # ALLOCATED / ASSIGNED
    2: 2,  # IN_PROGRESS
    6: 3,  # PICKUP_COMPLETED
    7: 4,  # DELIVERING
    3: 5,  # COMPLETED  (terminal)
    4: 5,  # FAILED     (terminal)
    5: 5,  # CANCELLED  (terminal)
}

# States at rank >= 2 are physically locked — no downgrade allowed
_PHYSICAL_LOCK_RANK = 2

# States that matter for Max-Sum allocation decisions
_ALLOCATION_RELEVANT_STATES: Set[int] = {0, 1}  # AVAILABLE, ALLOCATED


def _task_rank(state: int) -> int:
    return _TASK_STATE_RANK.get(int(state), int(state))


def _is_physically_locked(state: int) -> bool:
    return _task_rank(state) >= _PHYSICAL_LOCK_RANK


def _is_allocation_relevant(state: int) -> bool:
    return int(state) in _ALLOCATION_RELEVANT_STATES


# ---------------------------------------------------------------------------
# Operation types
# ---------------------------------------------------------------------------

class CrdtOpType(IntEnum):
    ADD_TASK    = 1   # Create or update a task
    REMOVE_TASK = 2   # Mark a task as removed (logical delete)
    ADD_ROBOT   = 3   # Create or update a robot state
    REMOVE_ROBOT = 4  # Mark a robot as removed (rare)


# ---------------------------------------------------------------------------
# Operation record
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class CrdtOp:
    """
    Immutable CRDT operation.

    op_id:        Unique identifier = f"{origin_robot}:{lamport}"
                  Uniqueness relies on Lamport uniqueness per robot.
    op_type:      CrdtOpType
    entity_id:    task_id or robot_id being operated on
    origin_robot: Robot that generated this operation
    lamport:      Lamport clock at generation time (ordering primitive)
    payload:      Full state snapshot at time of operation (dict)
                  For REMOVE ops, payload may be minimal (just entity_id).
    """
    op_id:        str
    op_type:      int         # CrdtOpType value (int for easy JSON round-trip)
    entity_id:    str
    origin_robot: str
    lamport:      int
    payload:      dict = field(default_factory=dict, hash=False, compare=False)

    # ------------------------------------------------------------------
    # Serialisation
    # ------------------------------------------------------------------
    def to_dict(self) -> dict:
        return {
            'op_id':        self.op_id,
            'op_type':      int(self.op_type),
            'entity_id':    self.entity_id,
            'origin_robot': self.origin_robot,
            'lamport':      self.lamport,
            'payload':      self.payload,
        }

    @staticmethod
    def from_dict(d: dict) -> 'CrdtOp':
        return CrdtOp(
            op_id        = d['op_id'],
            op_type      = int(d['op_type']),
            entity_id    = d['entity_id'],
            origin_robot = d['origin_robot'],
            lamport      = int(d['lamport']),
            payload      = d.get('payload', {}),
        )

    @staticmethod
    def make_id(origin_robot: str, lamport: int) -> str:
        return f"{origin_robot}:{lamport}"


# ---------------------------------------------------------------------------
# Apply / Replay result types
# ---------------------------------------------------------------------------

@dataclass
class ApplyResult:
    accepted:           bool   = False  # Was the op new/winning?
    duplicate:          bool   = False  # Already seen this op_id
    physical_lock:      bool   = False  # Rejected because entity is locked
    out_of_order:       bool   = False  # Accepted but arrived late (lower lamport)
    materialized_changed: bool = False  # Did the materialized view change?
    allocation_changed: bool   = False  # Did allocation-relevant state change?
    entity_id:          str    = ''


@dataclass
class ReplayResult:
    total_ops:          int  = 0
    new_ops:            int  = 0
    duplicate_ops:      int  = 0
    lock_rejected_ops:  int  = 0
    changed_ids:        List[str] = field(default_factory=list)
    allocation_changed: bool = False


# ---------------------------------------------------------------------------
# LWW-Element-Set CRDT
# ---------------------------------------------------------------------------

class LWWElementSet:
    """
    Operation-based LWW-Element-Set CRDT for a single entity type
    (all tasks, or all robots).

    Internal structure
    ------------------
    _log      : Dict[op_id, CrdtOp]            — complete operation log
    _by_entity: Dict[entity_id, List[op_id]]    — index: entity → its ops
    _current  : Dict[entity_id, CrdtOp | None] — current winning op
                None means entity is removed
    _removed  : Set[entity_id]                  — logical delete set

    Merge rules
    -----------
    1. op_id uniqueness guarantees idempotency.
    2. For each entity, the winning op is argmax over all ops by LWW order.
    3. If the highest-clock op is a REMOVE, the entity is in the remove set.
    4. Physical-state lock: if current materialized state is locked, incoming
       op that would downgrade the state is rejected.
    """

    # OP types that are REMOVE operations (entity_id → deleted)
    _REMOVE_TYPES = {int(CrdtOpType.REMOVE_TASK), int(CrdtOpType.REMOVE_ROBOT)}

    def __init__(self, is_task_set: bool = True, log_callback=None):
        """
        is_task_set: True for task CRDT (enables physical-lock logic).
                     False for robot CRDT (no lifecycle locking).
        log_callback: callable(msg: str) for [CRDT] log lines.
        """
        self._is_task = is_task_set
        self._log_cb = log_callback or (lambda m: None)

        self._log:       Dict[str, CrdtOp]         = {}
        self._by_entity: Dict[str, List[str]]       = {}  # entity_id → [op_id]
        self._current:   Dict[str, Optional[CrdtOp]] = {}
        self._removed:   Set[str]                   = set()

    # ------------------------------------------------------------------
    # LWW comparison helper
    # ------------------------------------------------------------------
    @staticmethod
    def _lww_beats(a: CrdtOp, b: CrdtOp) -> bool:
        """Returns True if op `a` beats op `b` in LWW ordering."""
        if a.lamport != b.lamport:
            return a.lamport > b.lamport
        return a.origin_robot > b.origin_robot  # deterministic tie-break

    # ------------------------------------------------------------------
    # Physical-state lock check  (task sets only)
    # ------------------------------------------------------------------
    def _check_physical_lock(self, entity_id: str, incoming_state: int) -> bool:
        """
        Returns True if the incoming state would violate the physical lock.
        Only meaningful for task entities.
        """
        if not self._is_task:
            return False
        cur_op = self._current.get(entity_id)
        if cur_op is None:
            return False
        cur_state = cur_op.payload.get('state', 0)
        if _is_physically_locked(cur_state) and _task_rank(incoming_state) < _task_rank(cur_state):
            return True
        return False

    # ------------------------------------------------------------------
    # Core: apply a single operation
    # ------------------------------------------------------------------
    def apply(self, op: CrdtOp) -> ApplyResult:
        """
        Apply a CRDT operation.  Idempotent — same op_id applied twice
        produces identical state.

        Returns an ApplyResult describing what happened.
        """
        result = ApplyResult(entity_id=op.entity_id)

        # 1. Idempotency: skip already-seen op_ids
        if op.op_id in self._log:
            result.duplicate = True
            self._log_cb(f'[CRDT] Duplicate operation ignored op_id={op.op_id} entity={op.entity_id}')
            return result

        # 2. Physical-state lock check
        is_remove = op.op_type in self._REMOVE_TYPES
        incoming_state = op.payload.get('state', 0) if not is_remove else 0
        if self._is_task and not is_remove:
            if self._check_physical_lock(op.entity_id, incoming_state):
                cur_state = self._current[op.entity_id].payload.get('state', '?')
                self._log_cb(
                    f'[CRDT] Physical lock preserved for task={op.entity_id} '
                    f'state={cur_state} (incoming state={incoming_state} from {op.origin_robot} lamport={op.lamport})'
                )
                result.physical_lock = True
                return result

        # 3. Add to log and entity index
        self._log[op.op_id] = op
        self._by_entity.setdefault(op.entity_id, []).append(op.op_id)

        # 4. Reconcile: find new winner for this entity
        result = self._reconcile(op.entity_id, result)
        return result

    def _reconcile(self, entity_id: str, result: ApplyResult) -> ApplyResult:
        """
        Re-evaluate which operation wins for a given entity by scanning
        all ops for that entity and picking the LWW winner.
        Updates _current and _removed accordingly.
        """
        op_ids = self._by_entity.get(entity_id, [])
        if not op_ids:
            return result

        prev_op = self._current.get(entity_id)
        prev_state = prev_op.payload.get('state', -1) if prev_op else -1

        # Find LWW winner
        winner: Optional[CrdtOp] = None
        for oid in op_ids:
            op = self._log[oid]
            if winner is None or self._lww_beats(op, winner):
                winner = op

        assert winner is not None
        is_remove = winner.op_type in self._REMOVE_TYPES

        # Check if materialized view changed
        old_winner = self._current.get(entity_id)
        view_changed = (
            old_winner is None or
            old_winner.op_id != winner.op_id
        )

        self._current[entity_id] = None if is_remove else winner

        if is_remove:
            was_removed = entity_id in self._removed
            self._removed.add(entity_id)
            result.materialized_changed = not was_removed
            result.accepted = True
            self._log_cb(f'[CRDT] REMOVE entity={entity_id} by {winner.origin_robot} lamport={winner.lamport}')
        else:
            self._removed.discard(entity_id)
            if view_changed:
                new_state = winner.payload.get('state', -1)
                result.materialized_changed = True
                result.accepted = True

                # Detect out-of-order: new winner's lamport < latest previously applied
                if old_winner and old_winner.lamport > winner.lamport:
                    result.out_of_order = True
                    self._log_cb(
                        f'[CRDT] Out-of-order operation reconciled entity={entity_id} '
                        f'old_lamport={old_winner.lamport} new_lamport={winner.lamport}'
                    )
                elif old_winner is None:
                    self._log_cb(
                        f'[CRDT] ADD entity={entity_id} '
                        f'origin={winner.origin_robot} lamport={winner.lamport} state={winner.payload.get("state","?")}'
                    )
                else:
                    self._log_cb(
                        f'[CRDT] UPDATE entity={entity_id} '
                        f'origin={winner.origin_robot} lamport={winner.lamport} state={winner.payload.get("state","?")}'
                    )

                # Detect allocation-relevant change (tasks only)
                if self._is_task:
                    old_alloc = _is_allocation_relevant(prev_state) if prev_state >= 0 else True
                    new_alloc = _is_allocation_relevant(new_state)
                    assigned_changed = (
                        old_winner is None or
                        old_winner.payload.get('assigned_robot_id', '') !=
                        winner.payload.get('assigned_robot_id', '')
                    )
                    if old_alloc != new_alloc or assigned_changed or old_winner is None:
                        result.allocation_changed = True
                        self._log_cb(f'[CRDT] Allocation-relevant state changed for entity={entity_id}')

        result.entity_id = entity_id
        return result

    # ------------------------------------------------------------------
    # Convenience: build and apply an ADD/UPDATE op
    # ------------------------------------------------------------------
    def add(self, entity_id: str, origin_robot: str, lamport: int, payload: dict,
            op_type: int = None) -> ApplyResult:
        """Build and apply an ADD/UPDATE op.  op_type defaults to ADD_TASK or ADD_ROBOT."""
        if op_type is None:
            op_type = int(CrdtOpType.ADD_TASK) if self._is_task else int(CrdtOpType.ADD_ROBOT)
        op = CrdtOp(
            op_id        = CrdtOp.make_id(origin_robot, lamport),
            op_type      = op_type,
            entity_id    = entity_id,
            origin_robot = origin_robot,
            lamport      = lamport,
            payload      = dict(payload),
        )
        return self.apply(op)

    def remove(self, entity_id: str, origin_robot: str, lamport: int,
               payload: Optional[dict] = None) -> ApplyResult:
        """Build and apply a REMOVE op."""
        rm_type = int(CrdtOpType.REMOVE_TASK) if self._is_task else int(CrdtOpType.REMOVE_ROBOT)
        op = CrdtOp(
            op_id        = CrdtOp.make_id(origin_robot, lamport),
            op_type      = rm_type,
            entity_id    = entity_id,
            origin_robot = origin_robot,
            lamport      = lamport,
            payload      = payload or {'entity_id': entity_id},
        )
        return self.apply(op)

    # ------------------------------------------------------------------
    # Materialized view getters
    # ------------------------------------------------------------------
    def get(self, entity_id: str) -> Optional[dict]:
        """Return current materialized payload, or None if removed/unknown."""
        op = self._current.get(entity_id)
        if op is None:
            return None
        return dict(op.payload)

    def get_all(self) -> List[dict]:
        """Return payloads for all non-removed entities."""
        result = []
        for eid, op in self._current.items():
            if op is not None:
                result.append(dict(op.payload))
        return result

    def known_ids(self) -> List[str]:
        """Return IDs of all non-removed entities."""
        return [eid for eid, op in self._current.items() if op is not None]

    def is_removed(self, entity_id: str) -> bool:
        return entity_id in self._removed

    # ------------------------------------------------------------------
    # Serialisation for P2P transfer
    # ------------------------------------------------------------------
    def dump_ops(self) -> List[dict]:
        """Serialise all operations for P2P transfer (reconnect sync)."""
        return [op.to_dict() for op in self._log.values()]

    def replay_ops(self, ops_list: List[dict]) -> ReplayResult:
        """
        Bulk-apply a list of ops received from a peer (e.g. after reconnect).
        Fully idempotent — safe to call with overlapping op logs.

        Returns a ReplayResult summarising what changed.
        """
        rr = ReplayResult(total_ops=len(ops_list))
        for d in ops_list:
            try:
                op = CrdtOp.from_dict(d)
            except (KeyError, TypeError, ValueError):
                continue  # malformed — skip

            ar = self.apply(op)
            if ar.duplicate:
                rr.duplicate_ops += 1
            elif ar.physical_lock:
                rr.lock_rejected_ops += 1
            else:
                rr.new_ops += 1
                if ar.materialized_changed and op.entity_id not in rr.changed_ids:
                    rr.changed_ids.append(op.entity_id)
                if ar.allocation_changed:
                    rr.allocation_changed = True

        if rr.new_ops > 0:
            self._log_cb(
                f'[CRDT] Reconnect merge completed: '
                f'{rr.new_ops} new, {rr.duplicate_ops} dup, '
                f'{rr.lock_rejected_ops} lock-rejected '
                f'of {rr.total_ops} total ops | '
                f'changed={rr.changed_ids} alloc_changed={rr.allocation_changed}'
            )

        return rr

    def __len__(self) -> int:
        return len(self._log)

    def __repr__(self) -> str:
        return (
            f'LWWElementSet(entities={len(self._current)}, '
            f'removed={len(self._removed)}, log={len(self._log)})'
        )
