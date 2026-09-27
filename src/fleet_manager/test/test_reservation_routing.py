#!/usr/bin/env python3
"""
Reservation Routing & Grant-Gating Tests
=========================================
Covers the aisle-reservation correctness fixes — the behaviors that decide
"robot reserves an aisle correctly" and "other robots react correctly when
they see an aisle is reserved":

  A. A RELEASED message delivered on the reserve handler (/fleet/reservations
     also carries releases) must be processed as a release — never re-granted
     to the releaser, never re-queued.
  B. A peer's GRANTED broadcast must be adopted as the current holder.
  C. Full request -> release -> promote round trip through the routed
     handlers (not just the direct internal calls).
  D. ConflictCoordinator blocks aisle entry while the grant is pending
     (no holder recorded yet) and while another robot holds the aisle.
  E. A robot whose plan never enters the aisle is not stopped and not
     blocked by aisle logic when the node bypasses reservations
     (respect_reservations=False) — passers-by keep moving.
  F. A robot already inside an aisle it does not hold is allowed to keep
     driving out (never hard-stopped inside a single-lane segment).

Run with:
    cd /home/mangal-devanshu/sih_ws/FleetManager
    python3 src/fleet_manager/test/test_reservation_routing.py
"""

import json
import os
import sys
import tempfile
import time

_pkg_dir = os.path.join(os.path.dirname(__file__), '..', '..')
sys.path.insert(0, _pkg_dir)

import rclpy
from rclpy.parameter import Parameter

from fleet_interfaces.msg import Reservation
from fleet_manager.reservation_manager import ReservationManager
from fleet_manager.conflict_resolver import (
    ConflictCoordinator,
    Vector2,
)

LOGS = []


def log(msg: str):
    LOGS.append(msg)
    print(msg)


TEST_AISLES = {
    'aisle_1': {
        'segment_id': 'aisle_1', 'x_min': 1.5, 'x_max': 4.0,
        'y_min': 0.0, 'y_max': 2.5, 'is_single_lane': True,
    },
    'aisle_2': {
        'segment_id': 'aisle_2', 'x_min': 1.5, 'x_max': 4.0,
        'y_min': -3.5, 'y_max': -1.0, 'is_single_lane': True,
    },
}

_NODE_COUNTER = 0


def _make_manager(robot_id: str) -> ReservationManager:
    """Builds a ReservationManager wired to a throwaway aisle config."""
    global _NODE_COUNTER
    _NODE_COUNTER += 1
    fd, cfg_path = tempfile.mkstemp(suffix='.json')
    with os.fdopen(fd, 'w') as f:
        json.dump(TEST_AISLES, f)
    node = ReservationManager(
        parameter_overrides=[
            Parameter('robot_id', Parameter.Type.STRING, robot_id),
            Parameter('aisle_config_file', Parameter.Type.STRING, cfg_path),
            # Never leak test aisle layouts into a live fleet graph: the
            # config topic is latched and authoritative fleet-wide.
            Parameter('publish_config_on_start', Parameter.Type.BOOL, False),
        ],
    )
    node._test_cfg_path = cfg_path
    return node


def _destroy_manager(node: ReservationManager):
    try:
        os.remove(node._test_cfg_path)
    except OSError:
        pass
    node.destroy_node()


def _make_res(res_id: str, robot_id: str, segment_id: str,
              state: int, lamport: int = 1, priority: int = 1,
              task_id: str = '') -> Reservation:
    res = Reservation()
    res.reservation_id = res_id
    res.robot_id = robot_id
    res.segment_id = segment_id
    res.task_id = task_id
    res.state = state
    res.lamport_clock = lamport
    res.priority = priority
    return res


# ─────────────────────────────────────────────────────────────────────────────
# TEST A: RELEASED on the reserve handler must not re-grant or re-queue
# ─────────────────────────────────────────────────────────────────────────────

def test_a_release_never_rerouted_as_request():
    log("\n" + "=" * 70)
    log("TEST A: RELEASED message delivered to reserve handler is routed as release")
    log("=" * 70)

    mgr = _make_manager('robot1')
    try:
        # robot1 holds aisle_1 (local request -> local grant)
        granted = mgr.request_aisle_reservation('aisle_1', task_id='T1', priority=1)
        assert granted is True
        assert mgr.aisles['aisle_1'].current_holder.robot_id == 'robot1'

        # robot2 requests remotely -> queued behind robot1
        req2 = _make_res('robot2:aisle_1:5', 'robot2', 'aisle_1',
                         Reservation.STATE_REQUESTED, lamport=5)
        mgr._handle_reserve_msg(req2)
        aisle = mgr.aisles['aisle_1']
        assert aisle.current_holder.robot_id == 'robot1'
        assert [q.robot_id for q in aisle.wait_queue] == ['robot2']

        # robot1 releases; robot2 gets promoted by the local arbitration
        mgr.release_aisle_reservation('aisle_1')
        assert aisle.current_holder.robot_id == 'robot2'
        assert len(aisle.wait_queue) == 0

        # REGRESSION: a copy of robot1's RELEASED message arrives on the
        # /fleet/reservations handler (which also receives plain requests).
        # Before the fix this re-ran the request pipeline and could re-grant
        # the aisle to robot1, hijacking it from robot2.
        rel_copy = _make_res('robot1:aisle_1:9', 'robot1', 'aisle_1',
                             Reservation.STATE_RELEASED, lamport=9)
        mgr._handle_reserve_msg(rel_copy)

        assert mgr.aisles['aisle_1'].current_holder is not None
        assert mgr.aisles['aisle_1'].current_holder.robot_id == 'robot2', \
            f"Release was re-processed as request! Holder: " \
            f"{mgr.aisles['aisle_1'].current_holder.robot_id}"
        assert all(q.robot_id != 'robot1' for q in mgr.aisles['aisle_1'].wait_queue), \
            "Releasing robot got re-queued by its own release"
        assert not mgr.is_reservation_granted('aisle_1')

        log("  PASS ✓ RELEASED routed as release; holding robot keeps the aisle.")
    finally:
        _destroy_manager(mgr)


# ─────────────────────────────────────────────────────────────────────────────
# TEST B: Peer GRANTED broadcast is adopted as the holder
# ─────────────────────────────────────────────────────────────────────────────

def test_b_peer_grant_sync():
    log("\n" + "=" * 70)
    log("TEST B: Peer GRANTED broadcast adopted as current holder")
    log("=" * 70)

    mgr = _make_manager('robot3')
    try:
        # Nobody holds aisle_2 yet; a GRANTED self-claim from robot1 arrives.
        grant = _make_res('robot1:aisle_2:4', 'robot1', 'aisle_2',
                          Reservation.STATE_GRANTED, lamport=4)
        mgr._handle_reserve_msg(grant)

        aisle = mgr.aisles['aisle_2']
        assert aisle.current_holder is not None, "Peer grant must set the holder"
        assert aisle.current_holder.robot_id == 'robot1'
        assert not mgr.is_reservation_granted('aisle_2'), \
            "Manager must record the peer holder, not claim it locally"

        # Divergent case: local mirror thinks robot4 holds it, but the
        # authoritative grant from robot5 wins (deterministic arbitration
        # already happened fleet-wide; grants are only broadcast by holders).
        divergent = _make_res('robot5:aisle_2:11', 'robot5', 'aisle_2',
                              Reservation.STATE_GRANTED, lamport=11)
        mgr._handle_reserve_msg(divergent)
        assert mgr.aisles['aisle_2'].current_holder.robot_id == 'robot5'

        # Idempotence: the same grant delivered again must not duplicate.
        before_queue = list(mgr.aisles['aisle_2'].wait_queue)
        mgr._handle_reserve_msg(divergent)
        assert mgr.aisles['aisle_2'].current_holder.robot_id == 'robot5'
        assert mgr.aisles['aisle_2'].wait_queue == before_queue

        log("  PASS ✓ Peer grants are adopted deterministically and idempotently.")
    finally:
        _destroy_manager(mgr)


# ─────────────────────────────────────────────────────────────────────────────
# TEST C: Full request -> release -> promote round trip via routed handlers
# ─────────────────────────────────────────────────────────────────────────────

def test_c_full_cycle_through_routed_handlers():
    log("\n" + "=" * 70)
    log("TEST C: Request -> release -> promotion via message handlers")
    log("=" * 70)

    mgr = _make_manager('robot1')
    try:
        # robot1 (local) requests; granted immediately
        assert mgr.request_aisle_reservation('aisle_1', task_id='T9', priority=1)
        assert mgr.is_reservation_granted('aisle_1')

        # remote requests from robot2 (higher priority) and robot3
        mgr._handle_reserve_msg(_make_res(
            'robot2:aisle_1:2', 'robot2', 'aisle_1',
            Reservation.STATE_REQUESTED, lamport=2, priority=3))
        mgr._handle_reserve_msg(_make_res(
            'robot3:aisle_1:3', 'robot3', 'aisle_1',
            Reservation.STATE_REQUESTED, lamport=3, priority=1))
        # robot2 has higher priority -> it is first in the wait queue
        assert [q.robot_id for q in mgr.aisles['aisle_1'].wait_queue] == ['robot2', 'robot3']

        # robot1 releases via the local API (a robot's own messages are
        # deduplicated on the wire; local release is the real code path).
        mgr.release_aisle_reservation('aisle_1')

        aisle = mgr.aisles['aisle_1']
        assert aisle.current_holder is not None
        assert aisle.current_holder.robot_id == 'robot2', \
            f"Higher-priority waiter must be promoted, got {aisle.current_holder.robot_id}"
        assert not mgr.is_reservation_granted('aisle_1')

        # robot2's grant self-claim then arrives and is consistent
        mgr._handle_reserve_msg(_make_res(
            'robot2:aisle_1:7', 'robot2', 'aisle_1',
            Reservation.STATE_GRANTED, lamport=7))
        assert mgr.aisles['aisle_1'].current_holder.robot_id == 'robot2'

        # robot2 releases; robot3 (next in queue) is promoted
        mgr._handle_release_msg(_make_res(
            'robot2:aisle_1:8', 'robot2', 'aisle_1',
            Reservation.STATE_RELEASED, lamport=8))
        assert mgr.aisles['aisle_1'].current_holder.robot_id == 'robot3'
        assert len(mgr.aisles['aisle_1'].wait_queue) == 0

        log("  PASS ✓ Deterministic request -> release -> promotion pipeline intact.")
    finally:
        _destroy_manager(mgr)


# ─────────────────────────────────────────────────────────────────────────────
# TEST D: Grant-gated aisle entry in the ConflictCoordinator
# ─────────────────────────────────────────────────────────────────────────────

def test_d_grant_gated_entry():
    log("\n" + "=" * 70)
    log("TEST D: Aisle entry requires a confirmed grant")
    log("=" * 70)

    coord = ConflictCoordinator('robot_1')

    # Robot approaching aisle_1 from the west side (inside margin of 0.8).
    pos = Vector2(1.2, 1.25)
    vel = Vector2(0.3, 0.0)
    pref = Vector2(0.3, 0.0)

    # Case 1: no holder recorded yet (grant still in flight) -> STOP outside.
    override, active, msg = coord.compute_override(
        my_pos=pos, my_vel=vel, nav2_pref_vel=pref,
        peer_states={}, active_reservation_holder=None, dt=0.1)
    log(f"[NO HOLDER] active={active} | {msg}")
    assert active, "Robot must not enter an aisle before its grant is confirmed"
    assert override is not None and override.length() == 0.0, "Must stop outside the aisle"
    assert 'Awaiting reservation grant' in msg

    # Case 2: aisle held by another robot -> STOP (existing behavior kept).
    override2, active2, msg2 = coord.compute_override(
        my_pos=pos, my_vel=vel, nav2_pref_vel=pref,
        peer_states={}, active_reservation_holder='robot_2', dt=0.1)
    log(f"[OTHER HOLDS] active={active2} | {msg2}")
    assert active2 and override2.length() == 0.0
    assert 'held by robot_2' in msg2

    # Case 3: grant confirmed for this robot -> Nav2 proceeds uninterrupted.
    override3, active3, msg3 = coord.compute_override(
        my_pos=pos, my_vel=vel, nav2_pref_vel=pref,
        peer_states={}, active_reservation_holder='robot_1', dt=0.1)
    log(f"[SELF HOLDS] active={active3} | {msg3}")
    assert not active3 and override3 is None, \
        "Once granted, the aisle must not stall the holding robot"

    log("  PASS ✓ Entry is gated on a confirmed grant, not a local guess.")


# ─────────────────────────────────────────────────────────────────────────────
# TEST E: Passers-by are not stopped by another robot's aisle reservation
# ─────────────────────────────────────────────────────────────────────────────

def test_e_passerby_not_blocked():
    log("\n" + "=" * 70)
    log("TEST E: Passer-by bypasses aisle logic (respect_reservations=False)")
    log("=" * 70)

    coord = ConflictCoordinator('robot_1')

    # Robot near an aisle entrance but plan does not enter it. Another robot
    # holds the aisle. With reservations bypassed, nothing should stop us.
    pos = Vector2(1.2, 1.25)
    vel = Vector2(0.0, 0.5)
    pref = Vector2(0.0, 0.5)

    override, active, msg = coord.compute_override(
        my_pos=pos, my_vel=vel, nav2_pref_vel=pref,
        peer_states={}, active_reservation_holder='robot_2', dt=0.1,
        respect_reservations=False)
    log(f"[PASSER-BY] active={active} | {msg}")
    assert not active and override is None, \
        "A robot whose plan never enters the aisle must not be stopped by it"

    log("  PASS ✓ Non-entering robots keep moving past held aisles.")


# ─────────────────────────────────────────────────────────────────────────────
# TEST F: Robot inside an aisle whose lease expired must STOP (safety)
# ─────────────────────────────────────────────────────────────────────────────

def test_f_inside_without_holder_stops():
    log("\n" + "=" * 70)
    log("TEST F: Robot inside aisle without grant must STOP (prevent collision)")
    log("=" * 70)

    coord = ConflictCoordinator('robot_1')

    # Position inside aisle_1 ([1.5,4.0] x [0.0,2.5]) without the grant.
    pos = Vector2(2.5, 1.25)
    vel = Vector2(0.3, 0.0)
    pref = vel

    # Another robot holds the aisle (our lease expired, it was promoted).
    override, active, msg = coord.compute_override(
        my_pos=pos, my_vel=vel, nav2_pref_vel=pref,
        peer_states={}, active_reservation_holder='robot_2', dt=0.1)
    log(f"[INSIDE, OTHER HOLDS] active={active} | {msg}")
    assert active and override is not None and override.length() == 0.0, \
        "Robot inside aisle without grant must STOP to avoid collision"
    assert 'Lease lost mid-traversal' in msg

    # No holder recorded at all (grant expired, nobody promoted yet) — also STOP.
    override2, active2, msg2 = coord.compute_override(
        my_pos=pos, my_vel=vel, nav2_pref_vel=pref,
        peer_states={}, active_reservation_holder=None, dt=0.1)
    log(f"[INSIDE, NO HOLDER] active={active2} | {msg2}")
    assert active2 and override2 is not None and override2.length() == 0.0, \
        "Robot inside aisle with no holder must STOP (lease lost)"
    assert 'Lease lost mid-traversal' in msg2

    log("  PASS ✓ Robots inside an aisle without a valid grant STOP safely.")


# ─────────────────────────────────────────────────────────────────────────────
# TEST G: ConflictResolver's local request reaches its ReservationManager
# ─────────────────────────────────────────────────────────────────────────────

def test_g_self_originated_request_is_arbitrated():
    log("\n" + "=" * 70)
    log("TEST G: Self-originated conflict-resolver request is locally arbitrated")
    log("=" * 70)

    mgr = _make_manager('robot1')
    try:
        # ConflictResolver publishes this request rather than calling the
        # manager API.  It therefore has not been pre-recorded in _seen_res_ids.
        # The manager must process it even though it bears its own robot id.
        req = _make_res('robot1:aisle_1:from_conflict_resolver', 'robot1',
                        'aisle_1', Reservation.STATE_REQUESTED, lamport=1)
        mgr._handle_reserve_msg(req)

        assert mgr.is_reservation_granted('aisle_1'), \
            'The local manager must grant a self-originated approach request'
        assert mgr.aisles['aisle_1'].current_holder.state == Reservation.STATE_GRANTED
        log("  PASS ✓ Local conflict-resolver request produces a confirmed grant.")
    finally:
        _destroy_manager(mgr)


# ─────────────────────────────────────────────────────────────────────────────
# TEST H: Long traversals renew; occupancy prevents unsafe promotion
# ─────────────────────────────────────────────────────────────────────────────

def test_h_renewal_and_occupancy_guard():
    log("\n" + "=" * 70)
    log("TEST H: Renewal protects a long traversal; occupancy gates promotion")
    log("=" * 70)

    mgr = _make_manager('robot1')
    try:
        assert mgr.request_aisle_reservation('aisle_1', task_id='T1')
        aisle = mgr.aisles['aisle_1']
        aisle.holder_start_time = time.monotonic() - 20.0

        # A holder heartbeat must refresh the lease rather than re-queue or
        # expire a robot that is making legitimate progress through an aisle.
        renewal = _make_res('robot1:aisle_1:renew:2', 'robot1', 'aisle_1',
                            Reservation.STATE_ACTIVE, lamport=2)
        mgr._handle_reserve_msg(renewal)
        assert time.monotonic() - aisle.holder_start_time < 1.0, 'renewal did not refresh lease'

        # Queue robot2, then simulate robot1 still physically in the segment.
        mgr._handle_reserve_msg(_make_res('robot2:aisle_1:3', 'robot2', 'aisle_1',
                                          Reservation.STATE_REQUESTED, lamport=3))
        mgr._robot_positions['robot1'] = (2.5, 1.0, time.monotonic())
        mgr.release_aisle_reservation('aisle_1')
        assert aisle.current_holder is None, 'occupied aisle must not promote on release'
        assert [q.robot_id for q in aisle.wait_queue] == ['robot2'], 'waiter must remain queued'

        # Once pose telemetry confirms the former holder has left, the queued
        # robot can be promoted by the watchdog.
        mgr._robot_positions['robot1'] = (10.0, 10.0, time.monotonic())
        mgr._watchdog_check()
        assert aisle.current_holder is not None, 'clear aisle must promote its waiter'
        assert aisle.current_holder.robot_id == 'robot2', 'wrong waiter promoted'
        log("  PASS ✓ Healthy holders renew; waiters enter only after aisle clearance.")
    finally:
        _destroy_manager(mgr)


# ─────────────────────────────────────────────────────────────────────────────
# Runner
# ─────────────────────────────────────────────────────────────────────────────

def _ensure_rclpy():
    if not rclpy.ok():
        rclpy.init()


try:
    import pytest

    @pytest.fixture(scope='module', autouse=True)
    def _rclpy_context():
        _ensure_rclpy()
        yield
except ImportError:
    pass


def main():
    _ensure_rclpy()
    tests = [
        test_a_release_never_rerouted_as_request,
        test_b_peer_grant_sync,
        test_c_full_cycle_through_routed_handlers,
        test_d_grant_gated_entry,
        test_e_passerby_not_blocked,
        test_f_inside_without_holder_stops,
    ]
    passed = failed = 0
    for t in tests:
        try:
            t()
            passed += 1
        except AssertionError as e:
            failed += 1
            log(f"  FAIL ✗ {t.__name__}: {e}")
        except Exception as e:
            failed += 1
            log(f"  ERROR ✗ {t.__name__}: {e!r}")
    log("\n" + "=" * 70)
    log(f"  Results: {passed} passed, {failed} failed out of {len(tests)} tests")
    log("=" * 70)
    if rclpy.ok():
        rclpy.shutdown()
    if failed:
        sys.exit(1)


if __name__ == '__main__':
    main()
