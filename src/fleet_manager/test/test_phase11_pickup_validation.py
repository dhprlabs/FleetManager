#!/usr/bin/env python3
"""
Phase 11 Tests — Physical Pickup Validation & Distributed Reconciliation
========================================================================
Pure Python — no active ROS core or Gazebo required.

Tests:
  A. Normal pickup (item exists, transitions to PICKUP_COMPLETED, obs published, CRDT op recorded)
  B. Stale late-join pickup (R4 owns T5, R2 already completed T5, item absent, T5 invalidated from bundle)
  C. Item absent but no confirmed owner (observation published, task released as AVAILABLE for Max-Sum re-bid)
  D. Duplicate pickup observation (deduplication via unique obs_id, idempotent execution)
  E. Current task protection (R1 executing T1 is not preempted by stale broadcast, physical lock preserved)

Run with:
    cd /home/mangal-devanshu/sih_ws/FleetManager
    python3 src/fleet_manager/test/test_phase11_pickup_validation.py
"""

import sys
import os
from typing import Dict, List, Optional, Set, Tuple

_pkg_dir = os.path.join(os.path.dirname(__file__), '..', '..')
sys.path.insert(0, _pkg_dir)

from fleet_manager.crdt_store import (
    LWWElementSet, CrdtOp, CrdtOpType, ApplyResult,
)
from fleet_manager.fleet_state import FleetState
from fleet_manager.nav2_bridge import TaskExecutionState
from fleet_manager.bundle_manager import optimize_task_sequence, compute_route_cost

LOGS: List[str] = []

def log(msg: str):
    LOGS.append(msg)
    print(msg)


# State constants (matching Task.STATE_*)
STATE_AVAILABLE        = 0
STATE_ALLOCATED        = 1
STATE_IN_PROGRESS      = 2
STATE_COMPLETED        = 3
STATE_FAILED           = 4
STATE_CANCELLED        = 5
STATE_PICKUP_COMPLETED = 6
STATE_DELIVERING       = 7


# ─────────────────────────────────────────────────────────────────────────────
# Lightweight Stubs / Models for Phase 11 Nodes
# ─────────────────────────────────────────────────────────────────────────────

class MockPickupObservation:
    def __init__(self, obs_id="", robot_id="", task_id="", is_valid=False,
                 item_present=False, failure_reason="", confirmed_owner_id="",
                 lamport_clock=0):
        self.obs_id = obs_id
        self.robot_id = robot_id
        self.task_id = task_id
        self.is_valid = is_valid
        self.item_present = item_present
        self.failure_reason = failure_reason
        self.confirmed_owner_id = confirmed_owner_id
        self.lamport_clock = lamport_clock


class MockPickupValidator:
    """Implements Phase 11 pickup validator logic pure-Python."""
    def __init__(self, robot_id: str, crdt_store: LWWElementSet):
        self.robot_id = robot_id
        self.crdt_store = crdt_store
        self._seen_obs_ids: Set[str] = set()
        self.lamport_clock = 0
        self.published_responses: List[MockPickupObservation] = []

    def handle_observation(self, obs: MockPickupObservation) -> Optional[MockPickupObservation]:
        # Deduplication check
        if obs.obs_id:
            if obs.obs_id in self._seen_obs_ids:
                log(f'[{self.robot_id}] [PICKUP_VAL] Duplicate observation {obs.obs_id} ignored (idempotent).')
                return None
            self._seen_obs_ids.add(obs.obs_id)

        # Ignore if item is present, already valid, or from ourselves
        if obs.is_valid or obs.item_present or obs.confirmed_owner_id:
            return None
        if obs.robot_id == self.robot_id:
            return None

        log(f'[{self.robot_id}] [PICKUP_VAL] Pickup observation received: task={obs.task_id} '
            f'obs_id={obs.obs_id} from {obs.robot_id} (item absent). Running reconciliation...')

        confirmed_owner = self._find_confirmed_owner(obs.task_id, exclude_robot=obs.robot_id)

        self.lamport_clock += 1
        resp_obs_id = f"{self.robot_id}:{self.lamport_clock}"
        self._seen_obs_ids.add(resp_obs_id)

        resp = MockPickupObservation(
            obs_id=resp_obs_id,
            robot_id=self.robot_id,
            task_id=obs.task_id,
            lamport_clock=self.lamport_clock,
            item_present=False,
        )

        if confirmed_owner:
            resp.is_valid = True
            resp.confirmed_owner_id = confirmed_owner
            resp.failure_reason = ""
            log(f'[{self.robot_id}] [PICKUP_VAL] ✓ Reconciliation: task {obs.task_id} confirmed owned by '
                f'{confirmed_owner}. Notifying {obs.robot_id} to drop task.')
        else:
            resp.is_valid = False
            resp.confirmed_owner_id = ""
            resp.failure_reason = "no_confirmed_owner"
            log(f'[{self.robot_id}] [PICKUP_VAL] ✗ Reconciliation: no confirmed owner for task '
                f'{obs.task_id}. Notifying {obs.robot_id} to re-bid.')

        self.published_responses.append(resp)
        return resp

    def _find_confirmed_owner(self, task_id: str, exclude_robot: str) -> Optional[str]:
        entry = self.crdt_store.get(task_id)
        if not entry:
            return None
        state = entry.get('state')
        assigned = entry.get('assigned_robot_id')

        # Check CRDT confirmed physical states
        if state in (STATE_PICKUP_COMPLETED, STATE_DELIVERING, STATE_COMPLETED):
            if assigned and assigned != exclude_robot:
                return assigned
        if state in (STATE_ASSIGNED, STATE_IN_PROGRESS):
            if assigned and assigned != exclude_robot:
                return assigned
        return None


class MockBundleManager:
    """Implements Phase 11 bundle manager queue and invalidation logic."""
    def __init__(self, robot_id: str):
        self.robot_id = robot_id
        self.owned_tasks: Set[str] = set()
        self.completed_tasks: Set[str] = set()
        self.current_task: str = ""
        self.remaining_tasks: List[str] = []
        self.task_details: Dict[str, Dict] = {}
        self.nominal_speed = 0.5
        self.service_time = 2.0
        self.current_x = 0.0
        self.current_y = 0.0

    def add_task(self, task_id: str, pickup: Tuple[float, float], dropoff: Tuple[float, float], priority=1):
        self.task_details[task_id] = {
            'pickup': pickup,
            'dropoff': dropoff,
            'priority': priority,
            'state': STATE_AVAILABLE,
            'assigned_robot_id': '',
        }

    def claim_task(self, task_id: str):
        if task_id not in self.owned_tasks and task_id not in self.completed_tasks:
            self.owned_tasks.add(task_id)
            self.recompute_bundle()

    def handle_task_invalidated(self, task_id: str):
        if task_id not in self.owned_tasks and task_id not in self.remaining_tasks and self.current_task != task_id:
            return
        log(f'[{self.robot_id}] [BUNDLE] Task {task_id} invalidated — removed from bundle.')
        self.owned_tasks.discard(task_id)
        if self.current_task == task_id:
            self.current_task = ""
        if task_id in self.remaining_tasks:
            self.remaining_tasks.remove(task_id)
        self.recompute_bundle()

    def handle_task_completed(self, task_id: str):
        self.completed_tasks.add(task_id)
        self.owned_tasks.discard(task_id)
        if self.current_task == task_id:
            self.current_task = ""
        if task_id in self.remaining_tasks:
            self.remaining_tasks.remove(task_id)
        self.recompute_bundle()

    def recompute_bundle(self):
        unstarted = [
            t for t in self.owned_tasks
            if t != self.current_task and t not in self.completed_tasks
        ]
        start_x, start_y = self.current_x, self.current_y
        if self.current_task and self.current_task in self.task_details:
            start_x, start_y = self.task_details[self.current_task]['dropoff']

        best_rem, _, _ = optimize_task_sequence(
            start_x, start_y, unstarted, self.task_details, self.nominal_speed, self.service_time
        )
        self.remaining_tasks = best_rem
        if not self.current_task and self.remaining_tasks:
            self.current_task = self.remaining_tasks.pop(0)


class MockNav2Bridge:
    """Implements Phase 11 Nav2 execution state and abort logic."""
    def __init__(self, robot_id: str, bundle_mgr: MockBundleManager, crdt_store: LWWElementSet):
        self.robot_id = robot_id
        self.bundle_mgr = bundle_mgr
        self.crdt_store = crdt_store
        self.execution_state = TaskExecutionState.IDLE
        self.active_task_id: Optional[str] = None
        self._seen_obs_ids: Set[str] = set()
        self.lamport_clock = 0
        self.released_tasks: List[str] = []

    def start_task(self, task_id: str):
        self.active_task_id = task_id
        self.execution_state = TaskExecutionState.IN_PROGRESS
        log(f'[{self.robot_id}] Navigating to PICKUP for {task_id}')

    def on_pickup_arrival(self, item_present: bool):
        if item_present:
            self.execution_state = TaskExecutionState.PICKUP_COMPLETED
            self.lamport_clock += 1
            obs_id = f"{self.robot_id}:{self.lamport_clock}"
            self._seen_obs_ids.add(obs_id)
            log(f'[{self.robot_id}] ✓ Phase 11: Item confirmed present at pickup. Proceeding.')
            # Record in CRDT
            op = CrdtOp(
                op_id=obs_id,
                op_type=int(CrdtOpType.ADD_TASK),
                entity_id=self.active_task_id,
                origin_robot=self.robot_id,
                lamport=self.lamport_clock,
                payload={'task_id': self.active_task_id, 'state': STATE_PICKUP_COMPLETED,
                         'assigned_robot_id': self.robot_id}
            )
            self.crdt_store.apply(op)
            return MockPickupObservation(
                obs_id=obs_id, robot_id=self.robot_id, task_id=self.active_task_id,
                is_valid=True, item_present=True, confirmed_owner_id=self.robot_id,
                lamport_clock=self.lamport_clock
            )
        else:
            log(f'[{self.robot_id}] ✗ Phase 11: Item ABSENT at pickup for task {self.active_task_id}.')
            # Check local CRDT world view
            cached = self.crdt_store.get(self.active_task_id)
            if cached:
                st = cached.get('state')
                owner = cached.get('assigned_robot_id')
                if st in (STATE_PICKUP_COMPLETED, STATE_DELIVERING, STATE_COMPLETED) and owner and owner != self.robot_id:
                    log(f'[{self.robot_id}] [PICKUP_VAL] CRDT world view already confirms task '
                        f'{self.active_task_id} state={st} by {owner}. Dropping immediately.')
                    self.abort_active_task()
                    return None
            # Enter reconciling
            self.execution_state = TaskExecutionState.RECONCILING
            self.lamport_clock += 1
            obs_id = f"{self.robot_id}:{self.lamport_clock}"
            self._seen_obs_ids.add(obs_id)
            return MockPickupObservation(
                obs_id=obs_id, robot_id=self.robot_id, task_id=self.active_task_id,
                is_valid=False, item_present=False, failure_reason='item_absent_at_pickup',
                lamport_clock=self.lamport_clock
            )

    def abort_active_task(self):
        tid = self.active_task_id
        log(f'[{self.robot_id}] [NAV2] Aborting active task {tid}...')
        self.bundle_mgr.handle_task_invalidated(tid)
        self.active_task_id = None
        self.execution_state = TaskExecutionState.IDLE

    def release_task_ownership(self, task_id: str):
        log(f'[{self.robot_id}] Task {task_id} released. Returning to IDLE.')
        self.released_tasks.append(task_id)
        # Update CRDT to STATE_AVAILABLE
        self.lamport_clock += 1
        op = CrdtOp(
            op_id=f"{self.robot_id}:{self.lamport_clock}",
            op_type=int(CrdtOpType.ADD_TASK),
            entity_id=task_id,
            origin_robot=self.robot_id,
            lamport=self.lamport_clock,
            payload={'task_id': task_id, 'state': STATE_AVAILABLE, 'assigned_robot_id': ''}
        )
        self.crdt_store.apply(op)
        self.bundle_mgr.handle_task_invalidated(task_id)
        self.active_task_id = None
        self.execution_state = TaskExecutionState.IDLE


# ─────────────────────────────────────────────────────────────────────────────
# TESTS A – E
# ─────────────────────────────────────────────────────────────────────────────

def test_a_normal_pickup():
    log("\n=== TEST A: Normal pickup ===")
    crdt = LWWElementSet(is_task_set=True, log_callback=log)
    bm = MockBundleManager('robot1')
    nav = MockNav2Bridge('robot1', bm, crdt)

    bm.add_task('T1', pickup=(2.0, 3.0), dropoff=(5.0, 6.0))
    bm.claim_task('T1')
    assert bm.current_task == 'T1'

    nav.start_task('T1')
    assert nav.execution_state == TaskExecutionState.IN_PROGRESS

    # Arrive at pickup: item physically present
    obs = nav.on_pickup_arrival(item_present=True)
    assert obs is not None
    assert obs.is_valid is True
    assert obs.item_present is True
    assert obs.confirmed_owner_id == 'robot1'
    assert obs.obs_id.startswith('robot1:')
    assert nav.execution_state == TaskExecutionState.PICKUP_COMPLETED

    # Check CRDT registered state
    t1_crdt = crdt.get('T1')
    assert t1_crdt is not None
    assert t1_crdt['state'] == STATE_PICKUP_COMPLETED
    assert t1_crdt['assigned_robot_id'] == 'robot1'
    log("  PASS ✓ Normal pickup succeeded, transitions correct.")


def test_b_stale_late_join_pickup():
    log("\n=== TEST B: Stale late-join pickup ===")
    # R2 already completed T5 in CRDT
    crdt_shared = LWWElementSet(is_task_set=True, log_callback=log)
    op_completed = CrdtOp(
        op_id='robot2:10',
        op_type=int(CrdtOpType.ADD_TASK),
        entity_id='T5',
        origin_robot='robot2',
        lamport=10,
        payload={'task_id': 'T5', 'state': STATE_COMPLETED, 'assigned_robot_id': 'robot2'}
    )
    crdt_shared.apply(op_completed)

    # R4 late joins and mistakenly believes it owns T5
    bm_r4 = MockBundleManager('robot4')
    bm_r4.add_task('T5', pickup=(8.0, 2.0), dropoff=(10.0, 5.0))
    bm_r4.claim_task('T5')
    assert bm_r4.current_task == 'T5'

    nav_r4 = MockNav2Bridge('robot4', bm_r4, crdt_shared)
    nav_r4.start_task('T5')

    # R4 reaches P5, item is absent
    obs = nav_r4.on_pickup_arrival(item_present=False)
    # Since CRDT world view already shows T5 COMPLETED by robot2, R4 drops immediately
    assert obs is None
    assert nav_r4.execution_state == TaskExecutionState.IDLE
    assert 'T5' not in bm_r4.owned_tasks
    assert bm_r4.current_task == ""

    # Verify T5 in CRDT was NOT modified by R4
    t5_entry = crdt_shared.get('T5')
    assert t5_entry['state'] == STATE_COMPLETED
    assert t5_entry['assigned_robot_id'] == 'robot2'
    log("  PASS ✓ Stale late-join task T5 invalidated from bundle and not reallocated.")


def test_c_item_absent_no_confirmed_owner():
    log("\n=== TEST C: Item absent but no confirmed owner ===")
    crdt = LWWElementSet(is_task_set=True, log_callback=log)
    bm = MockBundleManager('robot1')
    bm.add_task('T3', pickup=(4.0, 1.0), dropoff=(9.0, 2.0))
    bm.claim_task('T3')

    nav = MockNav2Bridge('robot1', bm, crdt)
    nav.start_task('T3')

    # Reaches pickup, item absent
    obs = nav.on_pickup_arrival(item_present=False)
    assert obs is not None
    assert obs.is_valid is False
    assert obs.item_present is False
    assert nav.execution_state == TaskExecutionState.RECONCILING

    # Peer validator on robot2 processes observation
    val_r2 = MockPickupValidator('robot2', crdt)
    resp = val_r2.handle_observation(obs)
    assert resp is not None
    assert resp.is_valid is False
    assert resp.failure_reason == 'no_confirmed_owner'
    assert resp.confirmed_owner_id == ''

    # R1 receives response confirming no owner -> releases task for re-bid
    nav.release_task_ownership('T3')
    assert nav.execution_state == TaskExecutionState.IDLE
    assert 'T3' in nav.released_tasks

    # Verify task in CRDT is now STATE_AVAILABLE and unassigned for Max-Sum
    t3_crdt = crdt.get('T3')
    assert t3_crdt is not None
    assert t3_crdt['state'] == STATE_AVAILABLE
    assert t3_crdt['assigned_robot_id'] == ''
    log("  PASS ✓ Task released as AVAILABLE for Max-Sum re-bid.")


def test_d_duplicate_pickup_observation():
    log("\n=== TEST D: Duplicate pickup observation ===")
    crdt = LWWElementSet(is_task_set=True, log_callback=log)
    val = MockPickupValidator('robot2', crdt)

    obs = MockPickupObservation(
        obs_id='robot1:42',
        robot_id='robot1',
        task_id='T7',
        is_valid=False,
        item_present=False,
        failure_reason='item_absent_at_pickup',
        lamport_clock=42,
    )

    # First delivery
    resp1 = val.handle_observation(obs)
    assert resp1 is not None
    assert len(val.published_responses) == 1

    # Second delivery (duplicate)
    resp2 = val.handle_observation(obs)
    assert resp2 is None
    assert len(val.published_responses) == 1

    # Third delivery (duplicate)
    resp3 = val.handle_observation(obs)
    assert resp3 is None
    assert len(val.published_responses) == 1
    log("  PASS ✓ Duplicate observation correctly ignored (idempotent & deterministic).")


def test_e_current_task_protection():
    log("\n=== TEST E: Current task protection ===")
    crdt = LWWElementSet(is_task_set=True, log_callback=log)

    # R1 claims and is executing T1
    op_r1 = CrdtOp(
        op_id='robot1:5',
        op_type=int(CrdtOpType.ADD_TASK),
        entity_id='T1',
        origin_robot='robot1',
        lamport=5,
        payload={'task_id': 'T1', 'state': STATE_IN_PROGRESS, 'assigned_robot_id': 'robot1'}
    )
    res = crdt.apply(op_r1)
    assert res.accepted

    bm_r1 = MockBundleManager('robot1')
    bm_r1.add_task('T1', (1.0, 1.0), (2.0, 2.0))
    bm_r1.claim_task('T1')
    nav_r1 = MockNav2Bridge('robot1', bm_r1, crdt)
    nav_r1.start_task('T1')
    assert nav_r1.execution_state == TaskExecutionState.IN_PROGRESS

    # A peer (e.g. robot3) broadcasts stale lower-ranked state for T1 (STATE_AVAILABLE with clock=100)
    op_stale = CrdtOp(
        op_id='robot3:100',
        op_type=int(CrdtOpType.ADD_TASK),
        entity_id='T1',
        origin_robot='robot3',
        lamport=100,
        payload={'task_id': 'T1', 'state': STATE_AVAILABLE, 'assigned_robot_id': ''}
    )
    res_stale = crdt.apply(op_stale)
    # Physical lock must reject reverting active task state
    assert res_stale.physical_lock
    assert not res_stale.accepted
    assert crdt.get('T1')['state'] == STATE_IN_PROGRESS
    assert crdt.get('T1')['assigned_robot_id'] == 'robot1'

    # R1 execution state remains IN_PROGRESS and current_task remains T1
    assert nav_r1.execution_state == TaskExecutionState.IN_PROGRESS
    assert nav_r1.active_task_id == 'T1'
    log("  PASS ✓ Current task protected; physical lock rejected stale preemption.")


def run_all():
    tests = [
        test_a_normal_pickup,
        test_b_stale_late_join_pickup,
        test_c_item_absent_no_confirmed_owner,
        test_d_duplicate_pickup_observation,
        test_e_current_task_protection,
    ]
    passed = 0
    failed = 0
    for t in tests:
        try:
            t()
            passed += 1
        except Exception as e:
            failed += 1
            log(f"  FAIL ✗ {t.__name__}: {e}")
            import traceback
            traceback.print_exc()

    log("\n" + "═" * 60)
    log(f"  Results: {passed} passed, {failed} failed out of {len(tests)} tests")
    log("═" * 60 + "\n")
    if failed > 0:
        sys.exit(1)


if __name__ == '__main__':
    run_all()
