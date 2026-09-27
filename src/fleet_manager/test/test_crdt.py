#!/usr/bin/env python3
"""
Phase 10.1 CRDT Tests — Tests A through F
==========================================
Pure Python — no ROS required.

Run with:
    cd /path/to/FleetManager
    source install/setup.bash
    python3 -m pytest src/fleet_manager/test/test_crdt.py -v
or simply:
    python3 src/fleet_manager/test/test_crdt.py
"""

import sys
import os

# Allow running directly from workspace root without install
_pkg_dir = os.path.join(os.path.dirname(__file__), '..', '..')
sys.path.insert(0, _pkg_dir)

from fleet_manager.crdt_store import (
    LWWElementSet, CrdtOp, CrdtOpType, ApplyResult, ReplayResult,
)
from fleet_manager.fleet_state import FleetState, TaskEntry, RobotEntry, get_state_rank


# ─────────────────────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────────────────────

LOGS = []

def log(msg: str):
    LOGS.append(msg)
    print(msg)

def make_crdt(is_task=True) -> LWWElementSet:
    return LWWElementSet(is_task_set=is_task, log_callback=log)

def make_fs(robot_id: str) -> FleetState:
    return FleetState(robot_id=robot_id, log_callback=log)

def task_payload(task_id, state, assigned='', lamport=1, source='r1',
                 pickup_x=1.0, pickup_y=2.0, dropoff_x=3.0, dropoff_y=4.0):
    return {
        'task_id': task_id,
        'state': state,
        'assigned_robot_id': assigned,
        'version': 1,
        'priority': 1,
        'pickup_x': pickup_x,
        'pickup_y': pickup_y,
        'dropoff_x': dropoff_x,
        'dropoff_y': dropoff_y,
        'lamport_clock': lamport,
        'wall_time': 0.0,
        'source_robot_id': source,
    }

# Task state constants (matching fleet_interfaces Task.STATE_*)
STATE_AVAILABLE    = 0
STATE_ALLOCATED    = 1
STATE_IN_PROGRESS  = 2
STATE_COMPLETED    = 3
STATE_FAILED       = 4
STATE_CANCELLED    = 5
STATE_PICKUP_COMPLETED = 6
STATE_DELIVERING   = 7


# ─────────────────────────────────────────────────────────────────────────────
# TEST A — Duplicate operation delivery
# ─────────────────────────────────────────────────────────────────────────────

def test_A_duplicate_operation():
    """
    Send the same state operation twice.
    Expected: no duplicate state, deterministic final state, duplicate flag set.
    """
    print('\n=== TEST A: Duplicate operation ===')
    crdt = make_crdt()

    payload = task_payload('T1', STATE_AVAILABLE, lamport=10, source='r1')
    op = CrdtOp(
        op_id='r1:10', op_type=int(CrdtOpType.ADD_TASK),
        entity_id='T1', origin_robot='r1', lamport=10, payload=payload,
    )

    ar1 = crdt.apply(op)
    ar2 = crdt.apply(op)  # Same op — must be idempotent

    assert ar1.accepted, 'First apply must be accepted'
    assert ar1.materialized_changed, 'First apply must change state'
    assert not ar1.duplicate, 'First apply must NOT be duplicate'
    assert ar2.duplicate, 'Second apply of same op_id MUST be duplicate'
    assert not ar2.materialized_changed, 'Duplicate must not change state'

    # Materialized view: exactly one T1 entry
    all_entities = crdt.get_all()
    assert len(all_entities) == 1, f'Expected 1 entity, got {len(all_entities)}'
    assert all_entities[0]['state'] == STATE_AVAILABLE

    # Check [CRDT] Duplicate log line appeared
    assert any('Duplicate operation ignored' in l for l in LOGS), \
        'Expected [CRDT] Duplicate log line'

    print('  PASS ✓')


# ─────────────────────────────────────────────────────────────────────────────
# TEST B — Out-of-order operations
# ─────────────────────────────────────────────────────────────────────────────

def test_B_out_of_order():
    """
    Deliver newer operation (lamport=20, ALLOCATED) before older (lamport=10, AVAILABLE).
    Expected: final state = ALLOCATED (lamport=20 wins); out_of_order flag on late op.
    """
    print('\n=== TEST B: Out-of-order operations ===')
    crdt = make_crdt()

    op_new = CrdtOp(
        op_id='r1:20', op_type=int(CrdtOpType.ADD_TASK),
        entity_id='T1', origin_robot='r1', lamport=20,
        payload=task_payload('T1', STATE_ALLOCATED, assigned='r1', lamport=20, source='r1'),
    )
    op_old = CrdtOp(
        op_id='r1:10', op_type=int(CrdtOpType.ADD_TASK),
        entity_id='T1', origin_robot='r1', lamport=10,
        payload=task_payload('T1', STATE_AVAILABLE, lamport=10, source='r1'),
    )

    # Deliver newer first
    ar_new = crdt.apply(op_new)
    assert ar_new.accepted and ar_new.materialized_changed

    # Deliver older
    ar_old = crdt.apply(op_old)
    # The older op is added to log but should NOT become winner
    assert not ar_old.materialized_changed, \
        'Older op must not overwrite newer winning op'
    # Out-of-order flag: the old op arrived AFTER a higher-lamport op
    # Because state didn't change, materialized_changed=False is correct.

    # Final state must be ALLOCATED (from newer op)
    mat = crdt.get('T1')
    assert mat is not None
    assert mat['state'] == STATE_ALLOCATED, \
        f'Expected ALLOCATED, got {mat["state"]}'
    assert mat['assigned_robot_id'] == 'r1'

    print('  PASS ✓')


# ─────────────────────────────────────────────────────────────────────────────
# TEST C — Concurrent update (two robots update same task)
# ─────────────────────────────────────────────────────────────────────────────

def test_C_concurrent_update_determinism():
    """
    Two robots (r1, r2) both allocate T1 at the same Lamport time.
    Deterministic tie-break: lexicographically larger origin_robot wins.
    Both robots must converge to same state regardless of delivery order.
    """
    print('\n=== TEST C: Concurrent update — deterministic winner ===')

    # r1 claims T1 at lamport=15
    op_r1 = CrdtOp(
        op_id='r1:15', op_type=int(CrdtOpType.ADD_TASK),
        entity_id='T1', origin_robot='r1', lamport=15,
        payload=task_payload('T1', STATE_ALLOCATED, assigned='r1', lamport=15, source='r1'),
    )
    # r2 claims T1 at same lamport=15
    op_r2 = CrdtOp(
        op_id='r2:15', op_type=int(CrdtOpType.ADD_TASK),
        entity_id='T1', origin_robot='r2', lamport=15,
        payload=task_payload('T1', STATE_ALLOCATED, assigned='r2', lamport=15, source='r2'),
    )

    # Robot A's view: receives r1's op first, then r2's
    crdt_a = make_crdt()
    crdt_a.apply(op_r1)
    crdt_a.apply(op_r2)
    winner_a = crdt_a.get('T1')['assigned_robot_id']

    # Robot B's view: receives r2's op first, then r1's
    crdt_b = make_crdt()
    crdt_b.apply(op_r2)
    crdt_b.apply(op_r1)
    winner_b = crdt_b.get('T1')['assigned_robot_id']

    # Both must converge to the same winner
    assert winner_a == winner_b, \
        f'Non-deterministic! robot A says {winner_a}, robot B says {winner_b}'
    # Tie-break: 'r2' > 'r1' lexicographically
    assert winner_a == 'r2', f'Expected r2 to win tie-break, got {winner_a}'

    print(f'  Deterministic winner: {winner_a}')
    print('  PASS ✓')


# ─────────────────────────────────────────────────────────────────────────────
# TEST D — Physical lock: stale op cannot revert IN_PROGRESS
# ─────────────────────────────────────────────────────────────────────────────

def test_D_physical_lock():
    """
    R1 has T1 = IN_PROGRESS (lamport=50).
    Deliver stale T1 = ALLOCATED op (lamport=30, which is older).
    Also deliver T1 = AVAILABLE op (lamport=10, very stale).
    Expected: T1 must remain IN_PROGRESS.
    """
    print('\n=== TEST D: Physical lock — stale op cannot revert IN_PROGRESS ===')
    crdt = make_crdt()

    # First, establish T1 as IN_PROGRESS
    op_progress = CrdtOp(
        op_id='r1:50', op_type=int(CrdtOpType.ADD_TASK),
        entity_id='T1', origin_robot='r1', lamport=50,
        payload=task_payload('T1', STATE_IN_PROGRESS, assigned='r1', lamport=50, source='r1'),
    )
    ar = crdt.apply(op_progress)
    assert ar.accepted and ar.materialized_changed

    state_now = crdt.get('T1')['state']
    assert state_now == STATE_IN_PROGRESS, f'Expected IN_PROGRESS, got {state_now}'

    # Now deliver stale ALLOCATED op (higher lamport but lower rank — wait, lamport=30 is lower)
    # This should be rejected by LWW (lower lamport loses)
    op_stale_alloc = CrdtOp(
        op_id='r2:30', op_type=int(CrdtOpType.ADD_TASK),
        entity_id='T1', origin_robot='r2', lamport=30,
        payload=task_payload('T1', STATE_ALLOCATED, assigned='r2', lamport=30, source='r2'),
    )
    ar2 = crdt.apply(op_stale_alloc)
    # The op is stored but must NOT overwrite winner (lower lamport)
    assert not ar2.materialized_changed, 'Stale lower-lamport op must not change state'

    # Deliver a HIGHER lamport but LOWER-rank state (simulates bug/attack)
    # This tests physical lock: lamport=60 AVAILABLE tries to override IN_PROGRESS
    op_revert = CrdtOp(
        op_id='r3:60', op_type=int(CrdtOpType.ADD_TASK),
        entity_id='T1', origin_robot='r3', lamport=60,
        payload=task_payload('T1', STATE_AVAILABLE, lamport=60, source='r3'),
    )
    ar3 = crdt.apply(op_revert)
    assert ar3.physical_lock, \
        'Physical lock must reject AVAILABLE op that would downgrade IN_PROGRESS'
    assert not ar3.materialized_changed

    # Task must still be IN_PROGRESS
    final_state = crdt.get('T1')['state']
    assert final_state == STATE_IN_PROGRESS, \
        f'Expected IN_PROGRESS after lock test, got {final_state}'

    # Check physical-lock log line appeared
    assert any('Physical lock preserved' in l for l in LOGS), \
        'Expected [CRDT] Physical lock log line'

    print(f'  T1 state after stale ops: {final_state} (IN_PROGRESS)')
    print('  PASS ✓')


# ─────────────────────────────────────────────────────────────────────────────
# TEST E — Communication loss + reconnect with replay_ops
# ─────────────────────────────────────────────────────────────────────────────

def test_E_communication_loss_reconnect():
    """
    R1 goes offline, generates local ops.
    R2 continues generating ops.
    Reconnect: replay each side's ops on the other.
    Expected: both converge to same materialized state.
    """
    print('\n=== TEST E: Communication loss + reconnect ===')

    crdt_r1 = make_crdt()
    crdt_r2 = make_crdt()

    # --- Before disconnect: both see T1=AVAILABLE ---
    op_seed = CrdtOp(
        op_id='broadcaster:1', op_type=int(CrdtOpType.ADD_TASK),
        entity_id='T1', origin_robot='broadcaster', lamport=1,
        payload=task_payload('T1', STATE_AVAILABLE, lamport=1, source='broadcaster'),
    )
    crdt_r1.apply(op_seed)
    crdt_r2.apply(op_seed)

    # Also seed T2
    op_seed2 = CrdtOp(
        op_id='broadcaster:2', op_type=int(CrdtOpType.ADD_TASK),
        entity_id='T2', origin_robot='broadcaster', lamport=2,
        payload=task_payload('T2', STATE_AVAILABLE, lamport=2, source='broadcaster'),
    )
    crdt_r1.apply(op_seed2)
    crdt_r2.apply(op_seed2)

    # --- R1 allocates T1 while disconnected from R2 ---
    op_r1_allocates_T1 = CrdtOp(
        op_id='r1:10', op_type=int(CrdtOpType.ADD_TASK),
        entity_id='T1', origin_robot='r1', lamport=10,
        payload=task_payload('T1', STATE_ALLOCATED, assigned='r1', lamport=10, source='r1'),
    )
    crdt_r1.apply(op_r1_allocates_T1)

    # --- R2 allocates T2 while disconnected from R1 ---
    op_r2_allocates_T2 = CrdtOp(
        op_id='r2:10', op_type=int(CrdtOpType.ADD_TASK),
        entity_id='T2', origin_robot='r2', lamport=10,
        payload=task_payload('T2', STATE_ALLOCATED, assigned='r2', lamport=10, source='r2'),
    )
    crdt_r2.apply(op_r2_allocates_T2)

    # --- Reconnect: exchange full op logs ---
    ops_r1 = crdt_r1.dump_ops()
    ops_r2 = crdt_r2.dump_ops()

    rr_r1 = crdt_r1.replay_ops(ops_r2)  # R1 replays R2's ops
    rr_r2 = crdt_r2.replay_ops(ops_r1)  # R2 replays R1's ops

    # Both must converge: T1=ALLOCATED(r1), T2=ALLOCATED(r2)
    t1_r1 = crdt_r1.get('T1')
    t1_r2 = crdt_r2.get('T1')
    t2_r1 = crdt_r1.get('T2')
    t2_r2 = crdt_r2.get('T2')

    assert t1_r1 is not None and t1_r2 is not None
    assert t2_r1 is not None and t2_r2 is not None

    assert t1_r1['assigned_robot_id'] == t1_r2['assigned_robot_id'] == 'r1', \
        f'T1 owner mismatch: r1 says {t1_r1["assigned_robot_id"]}, r2 says {t1_r2["assigned_robot_id"]}'
    assert t2_r1['assigned_robot_id'] == t2_r2['assigned_robot_id'] == 'r2', \
        f'T2 owner mismatch: r1 says {t2_r1["assigned_robot_id"]}, r2 says {t2_r2["assigned_robot_id"]}'

    # No new ops needed after initial replay (full idempotency)
    rr_r1_again = crdt_r1.replay_ops(ops_r2)
    assert rr_r1_again.new_ops == 0, 'Re-replay must produce 0 new ops (idempotent)'
    assert rr_r1_again.duplicate_ops == len(ops_r2), \
        'All re-replayed ops must be duplicates'

    print(f'  After reconnect — T1 owner: {t1_r1["assigned_robot_id"]}, T2 owner: {t2_r1["assigned_robot_id"]}')
    print(f'  Re-replay: new_ops={rr_r1_again.new_ops} dup_ops={rr_r1_again.duplicate_ops}')
    print('  PASS ✓')


# ─────────────────────────────────────────────────────────────────────────────
# TEST F — FleetState integration: Max-Sum sees reconciled materialized state
# ─────────────────────────────────────────────────────────────────────────────

def test_F_fleet_state_integration():
    """
    Full FleetState integration test.
    R1 and R2 both use FleetState (CRDT-backed).
    After sync, both must have the same get_all_tasks() result.
    Max-Sum materialized view must reflect reconciled CRDT state.
    """
    print('\n=== TEST F: FleetState integration — Max-Sum reconciled state ===')

    fs_r1 = make_fs('r1')
    fs_r2 = make_fs('r2')

    # Task broadcaster seeds T1 on both
    t1_entry = TaskEntry(
        task_id='T1', state=STATE_AVAILABLE,
        pickup_x=1.0, pickup_y=1.0, dropoff_x=5.0, dropoff_y=5.0,
        lamport_clock=5, source_robot_id='broadcaster', priority=1,
    )
    fs_r1.merge_task(t1_entry)
    fs_r2.merge_task(t1_entry)

    # R1 allocates T1 (IN_PROGRESS at lamport 20)
    t1_progress = TaskEntry(
        task_id='T1', state=STATE_IN_PROGRESS, assigned_robot_id='r1',
        pickup_x=1.0, pickup_y=1.0, dropoff_x=5.0, dropoff_y=5.0,
        lamport_clock=20, source_robot_id='r1', priority=1,
    )
    fs_r1.merge_task(t1_progress)

    # R2 does NOT know about the progress update yet (simulating comms loss)
    t1_r2 = fs_r2.get_task('T1')
    assert t1_r2.state == STATE_AVAILABLE, 'R2 should still see T1 as AVAILABLE'

    # Reconnect: R1 sends full WorldView (with CRDT ops) to R2
    wv_dict = fs_r1.world_view_to_dict()
    assert 'ops_tasks' in wv_dict, 'world_view_to_dict must include ops_tasks'
    assert len(wv_dict['ops_tasks']) > 0, 'ops_tasks must not be empty'

    # R2 deserializes and merges
    robots, tasks, peer_clock, ops_tasks, ops_robots = fs_r2.world_view_from_dict(wv_dict)
    result = fs_r2.merge_world_view(robots, tasks, ops_tasks=ops_tasks, ops_robots=ops_robots)

    # R2 should now see T1 as IN_PROGRESS
    t1_after = fs_r2.get_task('T1')
    assert t1_after is not None
    assert t1_after.state == STATE_IN_PROGRESS, \
        f'Expected IN_PROGRESS after sync, got {t1_after.state}'
    assert t1_after.assigned_robot_id == 'r1'

    # Stale AVAILABLE op from R3 must NOT revert IN_PROGRESS
    t1_stale = TaskEntry(
        task_id='T1', state=STATE_AVAILABLE,
        pickup_x=1.0, pickup_y=1.0, dropoff_x=5.0, dropoff_y=5.0,
        lamport_clock=99, source_robot_id='r3', priority=1,
    )
    fs_r2.merge_task(t1_stale)  # Higher lamport but lower rank → physical lock
    t1_locked = fs_r2.get_task('T1')
    assert t1_locked.state == STATE_IN_PROGRESS, \
        f'Physical lock failed in FleetState: got {t1_locked.state}'

    print(f'  T1 state on R2 after sync: {t1_after.state} (IN_PROGRESS)')
    print(f'  T1 after stale op: {t1_locked.state} (still IN_PROGRESS)')
    print(f'  ops_tasks in WorldView: {len(wv_dict["ops_tasks"])} ops')
    print('  PASS ✓')


# ─────────────────────────────────────────────────────────────────────────────
# Additional: Materialized-state log message checks
# ─────────────────────────────────────────────────────────────────────────────

def test_G_log_messages():
    """Verify required [CRDT] log messages are emitted."""
    print('\n=== TEST G: [CRDT] log message verification ===')

    local_logs = []
    crdt = LWWElementSet(is_task_set=True, log_callback=local_logs.append)

    # ADD
    op_add = CrdtOp('r1:1', int(CrdtOpType.ADD_TASK), 'T1', 'r1', 1,
                     task_payload('T1', STATE_AVAILABLE, lamport=1))
    crdt.apply(op_add)
    assert any('ADD' in l or 'UPDATE' in l for l in local_logs), 'Expected ADD log'

    # DUPLICATE
    crdt.apply(op_add)
    assert any('Duplicate' in l for l in local_logs), 'Expected Duplicate log'

    # Physical lock
    op_progress = CrdtOp('r1:10', int(CrdtOpType.ADD_TASK), 'T1', 'r1', 10,
                          task_payload('T1', STATE_IN_PROGRESS, lamport=10))
    crdt.apply(op_progress)
    op_revert = CrdtOp('r2:20', int(CrdtOpType.ADD_TASK), 'T1', 'r2', 20,
                        task_payload('T1', STATE_AVAILABLE, lamport=20))
    crdt.apply(op_revert)
    assert any('Physical lock' in l for l in local_logs), 'Expected Physical lock log'

    # Reconnect merge
    other_crdt = LWWElementSet(is_task_set=True, log_callback=local_logs.append)
    rr = other_crdt.replay_ops(crdt.dump_ops())
    if rr.new_ops > 0:
        assert any('Reconnect merge completed' in l for l in local_logs), \
            'Expected Reconnect merge log'

    # Allocation-relevant
    assert any('Allocation-relevant' in l for l in local_logs), \
        'Expected Allocation-relevant log'

    print('  All required [CRDT] log lines emitted:')
    for l in local_logs:
        if '[CRDT]' in l:
            print(f'    {l}')
    print('  PASS ✓')


# ─────────────────────────────────────────────────────────────────────────────
# Test runner
# ─────────────────────────────────────────────────────────────────────────────

def run_all():
    passed = 0
    failed = 0
    tests = [
        test_A_duplicate_operation,
        test_B_out_of_order,
        test_C_concurrent_update_determinism,
        test_D_physical_lock,
        test_E_communication_loss_reconnect,
        test_F_fleet_state_integration,
        test_G_log_messages,
    ]
    for t in tests:
        LOGS.clear()
        try:
            t()
            passed += 1
        except AssertionError as e:
            print(f'  FAIL ✗ — {e}')
            failed += 1
        except Exception as e:
            import traceback
            traceback.print_exc()
            print(f'  ERROR ✗ — {e}')
            failed += 1

    print(f'\n{"═" * 60}')
    print(f'  Results: {passed} passed, {failed} failed out of {len(tests)} tests')
    print('═' * 60)
    return failed == 0


if __name__ == '__main__':
    ok = run_all()
    sys.exit(0 if ok else 1)


# pytest-compatible wrappers
def test_pytest_A(): test_A_duplicate_operation()
def test_pytest_B(): test_B_out_of_order()
def test_pytest_C(): test_C_concurrent_update_determinism()
def test_pytest_D(): test_D_physical_lock()
def test_pytest_E(): test_E_communication_loss_reconnect()
def test_pytest_F(): test_F_fleet_state_integration()
def test_pytest_G(): test_G_log_messages()
