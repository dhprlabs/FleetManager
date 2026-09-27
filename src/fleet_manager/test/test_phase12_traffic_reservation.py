#!/usr/bin/env python3
"""
Phase 12 Tests — Single-Lane Aisle Segment Reservation
======================================================
Pure Python — no active ROS core required.

Tests:
  A. Two robots request same aisle (one granted, one waits)
  B. First robot exits (reservation released, waiting robot obtains access)
  C. Three robots contend (deterministic ordering, exactly one active reservation)
  D. Communication loss & recovery (stale reservation safely expires, no permanent deadlock)
  E. Multiple independent aisles (simultaneous non-conflicting reservations)

Run with:
    cd /home/mangal-devanshu/sih_ws/FleetManager
    python3 src/fleet_manager/test/test_phase12_traffic_reservation.py
"""

import sys
import os
import time
from typing import Dict, List, Optional, Set, Tuple

_pkg_dir = os.path.join(os.path.dirname(__file__), '..', '..')
sys.path.insert(0, _pkg_dir)

from fleet_manager.reservation_manager import (
    DEFAULT_AISLE_SEGMENTS,
    AisleState,
    reservation_sort_key,
)

LOGS: List[str] = []

def log(msg: str):
    LOGS.append(msg)
    print(msg)


# State constants (matching Reservation.STATE_*)
STATE_REQUESTED = 0
STATE_GRANTED   = 1
STATE_ACTIVE    = 2
STATE_RELEASED  = 3
STATE_WAITING   = 4
STATE_EXPIRED   = 5
STATE_DENIED    = 6


class TrafficReservation:
    """Pure-Python model of Reservation for test execution."""
    def __init__(self, reservation_id: str, robot_id: str, segment_id: str,
                 state: int = STATE_REQUESTED, priority: int = 1,
                 lamport_clock: int = 0, task_id: str = ""):
        self.reservation_id = reservation_id
        self.robot_id = robot_id
        self.segment_id = segment_id
        self.state = state
        self.priority = priority
        self.lamport_clock = lamport_clock
        self.task_id = task_id


class TrafficReservationCoordinator:
    """
    Decentralized traffic reservation coordinator implementing the Phase 12 logic.
    """
    def __init__(self, robot_id: str, safety_timeout_sec: float = 10.0):
        self.robot_id = robot_id
        self.safety_timeout_sec = safety_timeout_sec
        self.lamport_clock = 0
        self.aisles: Dict[str, AisleState] = {}
        for seg_id, spec in DEFAULT_AISLE_SEGMENTS.items():
            self.aisles[seg_id] = AisleState(
                seg_id, spec['x_min'], spec['x_max'], spec['y_min'], spec['y_max']
            )
        self.my_active_reservations: Dict[str, TrafficReservation] = {}
        self.published_events: List[TrafficReservation] = []

    def request_reservation(self, segment_id: str, requester_id: str,
                            priority: int = 1, lamport: int = 0, task_id: str = "") -> bool:
        if segment_id not in self.aisles:
            return False

        aisle = self.aisles[segment_id]
        self.lamport_clock = max(self.lamport_clock, lamport) + 1
        res_id = f"{requester_id}:{segment_id}:{self.lamport_clock}"

        res = TrafficReservation(
            reservation_id=res_id,
            robot_id=requester_id,
            segment_id=segment_id,
            state=STATE_REQUESTED,
            priority=priority,
            lamport_clock=self.lamport_clock,
            task_id=task_id,
        )

        # Case 1: Already held by requester
        if aisle.current_holder is not None and aisle.current_holder.robot_id == requester_id:
            res.state = STATE_GRANTED
            aisle.current_holder = res
            aisle.holder_start_time = time.monotonic()
            if requester_id == self.robot_id:
                self.my_active_reservations[segment_id] = res
            log(f'[{self.robot_id}] [TRAFFIC] Segment "{segment_id}" already held by {requester_id}.')
            return True

        # Case 2: Segment is free
        if aisle.current_holder is None:
            res.state = STATE_GRANTED
            aisle.current_holder = res
            aisle.holder_start_time = time.monotonic()
            if requester_id == self.robot_id:
                self.my_active_reservations[segment_id] = res
            log(f'[{self.robot_id}] [TRAFFIC] ✓ Segment "{segment_id}" GRANTED to {requester_id} '
                f'(priority={priority}, Lamport={res.lamport_clock}).')
            self.published_events.append(res)
            return True

        # Case 3: Segment occupied by another robot
        curr_holder = aisle.current_holder
        res.state = STATE_WAITING
        aisle.wait_queue = [q for q in aisle.wait_queue if q.robot_id != requester_id]
        aisle.wait_queue.append(res)
        aisle.wait_queue.sort(key=reservation_sort_key)

        queue_pos = [q.robot_id for q in aisle.wait_queue].index(requester_id) + 1
        log(f'[{self.robot_id}] [TRAFFIC] ✗ Segment "{segment_id}" DENIED to {requester_id} '
            f'(occupied by {curr_holder.robot_id}). Placed in WAITING queue (position {queue_pos}/{len(aisle.wait_queue)}).')
        self.published_events.append(res)
        return False

    def release_reservation(self, segment_id: str, releaser_id: str):
        if segment_id not in self.aisles:
            return

        aisle = self.aisles[segment_id]
        self.lamport_clock += 1

        if aisle.current_holder is not None and aisle.current_holder.robot_id == releaser_id:
            log(f'[{self.robot_id}] [TRAFFIC] Segment "{segment_id}" RELEASED by {releaser_id}.')
            aisle.current_holder = None
            if releaser_id == self.robot_id and segment_id in self.my_active_reservations:
                del self.my_active_reservations[segment_id]

            # Promote next waiter
            if aisle.wait_queue:
                next_res = aisle.wait_queue.pop(0)
                next_res.state = STATE_GRANTED
                aisle.current_holder = next_res
                aisle.holder_start_time = time.monotonic()
                log(f'[{self.robot_id}] [TRAFFIC] ★ Segment "{segment_id}" PROMOTED/GRANTED to waiting '
                    f'robot {next_res.robot_id} (priority={next_res.priority}, Lamport={next_res.lamport_clock}).')
                self.published_events.append(next_res)
        else:
            aisle.wait_queue = [q for q in aisle.wait_queue if q.robot_id != releaser_id]

    def watchdog_check(self, fake_elapsed: float = 0.0):
        """Checks for stale reservation expiry."""
        for seg_id, aisle in self.aisles.items():
            if aisle.current_holder is None:
                continue

            holder = aisle.current_holder
            if fake_elapsed >= self.safety_timeout_sec:
                log(f'[{self.robot_id}] [TRAFFIC] ⚠ Stale reservation detected for segment "{seg_id}" '
                    f'held by {holder.robot_id} (elapsed {fake_elapsed:.1f}s >= {self.safety_timeout_sec}s timeout). '
                    f'Expiring reservation to recover physical traffic.')
                if holder.robot_id == self.robot_id and seg_id in self.my_active_reservations:
                    del self.my_active_reservations[seg_id]

                aisle.current_holder = None
                if aisle.wait_queue:
                    next_res = aisle.wait_queue.pop(0)
                    next_res.state = STATE_GRANTED
                    aisle.current_holder = next_res
                    aisle.holder_start_time = time.monotonic()
                    log(f'[{self.robot_id}] [TRAFFIC] ★ Segment "{seg_id}" PROMOTED/GRANTED to waiting '
                        f'robot {next_res.robot_id} after stale timeout recovery.')
                    self.published_events.append(next_res)

    def is_granted(self, segment_id: str, robot_id: str) -> bool:
        aisle = self.aisles.get(segment_id)
        if not aisle:
            return False
        return (aisle.current_holder is not None and aisle.current_holder.robot_id == robot_id)


# ─────────────────────────────────────────────────────────────────────────────
# TESTS A – E
# ─────────────────────────────────────────────────────────────────────────────

def test_a_two_robots_request_same_aisle():
    log("\n=== TEST A: Two robots request same aisle ===")
    coord = TrafficReservationCoordinator('robot1')

    # Robot 1 requests aisle_1
    granted1 = coord.request_reservation('aisle_1', requester_id='robot1', priority=1, lamport=1)
    assert granted1 is True
    assert coord.is_granted('aisle_1', 'robot1') is True

    # Robot 2 requests aisle_1 while Robot 1 is inside
    granted2 = coord.request_reservation('aisle_1', requester_id='robot2', priority=1, lamport=2)
    assert granted2 is False
    assert coord.is_granted('aisle_1', 'robot2') is False

    # Verify state: exactly 1 holder, 1 waiter
    aisle = coord.aisles['aisle_1']
    assert aisle.current_holder.robot_id == 'robot1'
    assert len(aisle.wait_queue) == 1
    assert aisle.wait_queue[0].robot_id == 'robot2'
    assert aisle.wait_queue[0].state == STATE_WAITING
    log("  PASS ✓ Robot1 granted, Robot2 waits. No simultaneous dual ownership.")


def test_b_first_robot_exits_and_waiting_robot_acquires():
    log("\n=== TEST B: First robot exits & waiting robot obtains access ===")
    coord = TrafficReservationCoordinator('robot1')

    # Robot 1 has reservation, Robot 2 is waiting
    coord.request_reservation('aisle_1', requester_id='robot1', priority=1, lamport=1)
    coord.request_reservation('aisle_1', requester_id='robot2', priority=1, lamport=2)
    assert coord.is_granted('aisle_1', 'robot1') is True
    assert coord.is_granted('aisle_1', 'robot2') is False

    # Robot 1 exits and releases aisle_1
    coord.release_reservation('aisle_1', releaser_id='robot1')

    # Verification: Robot 2 automatically promoted to GRANTED
    assert coord.is_granted('aisle_1', 'robot1') is False
    assert coord.is_granted('aisle_1', 'robot2') is True
    assert len(coord.aisles['aisle_1'].wait_queue) == 0
    log("  PASS ✓ Reservation released by Robot1, Robot2 promoted to GRANTED.")


def test_c_three_robots_contend_deterministic_ordering():
    log("\n=== TEST C: Three robots contend — deterministic ordering ===")
    coord = TrafficReservationCoordinator('robot1')

    # Robot 1 arrives first (priority 1, Lamport 10)
    coord.request_reservation('aisle_1', requester_id='robot1', priority=1, lamport=10)
    assert coord.is_granted('aisle_1', 'robot1') is True

    # Robot 2 requests with priority 1, Lamport 20
    coord.request_reservation('aisle_1', requester_id='robot2', priority=1, lamport=20)

    # Robot 3 requests with HIGH priority 3, Lamport 25
    coord.request_reservation('aisle_1', requester_id='robot3', priority=3, lamport=25)

    aisle = coord.aisles['aisle_1']
    assert aisle.current_holder.robot_id == 'robot1'
    # Wait queue should be sorted deterministically: Robot 3 (pri=3) ahead of Robot 2 (pri=1)
    assert len(aisle.wait_queue) == 2
    assert aisle.wait_queue[0].robot_id == 'robot3'
    assert aisle.wait_queue[1].robot_id == 'robot2'

    # Robot 1 releases -> Robot 3 must get it (highest priority)
    coord.release_reservation('aisle_1', releaser_id='robot1')
    assert coord.is_granted('aisle_1', 'robot3') is True
    assert coord.is_granted('aisle_1', 'robot2') is False

    # Robot 3 releases -> Robot 2 gets it next
    coord.release_reservation('aisle_1', releaser_id='robot3')
    assert coord.is_granted('aisle_1', 'robot2') is True
    assert len(aisle.wait_queue) == 0
    log("  PASS ✓ Deterministic priority and Lamport ordering strictly respected.")


def test_d_communication_loss_and_stale_recovery():
    log("\n=== TEST D: Communication loss & recovery from stale reservation ===")
    coord = TrafficReservationCoordinator('robot1', safety_timeout_sec=5.0)

    # Robot 1 acquires aisle_1
    coord.request_reservation('aisle_1', requester_id='robot1', priority=1, lamport=1)
    # Robot 2 requests aisle_1 -> waits
    coord.request_reservation('aisle_1', requester_id='robot2', priority=1, lamport=2)

    assert coord.is_granted('aisle_1', 'robot1') is True
    assert coord.is_granted('aisle_1', 'robot2') is False

    # Simulate Robot 1 losing communication and disappearing for 6 seconds (> 5.0s timeout)
    coord.watchdog_check(fake_elapsed=6.0)

    # Robot 1's stale reservation must be expired, and Robot 2 promoted without deadlock
    assert coord.is_granted('aisle_1', 'robot1') is False
    assert coord.is_granted('aisle_1', 'robot2') is True
    log("  PASS ✓ Stale reservation recovered safely; no permanent deadlock.")


def test_e_multiple_independent_aisles():
    log("\n=== TEST E: Multiple independent aisles ===")
    coord = TrafficReservationCoordinator('robot1')

    # Three robots request three different independent narrow single-lane aisles
    g1 = coord.request_reservation('aisle_1', requester_id='robot1', priority=1, lamport=1)
    g2 = coord.request_reservation('aisle_2', requester_id='robot2', priority=1, lamport=2)
    g3 = coord.request_reservation('aisle_3', requester_id='robot3', priority=1, lamport=3)

    # All three must be granted simultaneously without mutual interference
    assert g1 is True
    assert g2 is True
    assert g3 is True
    assert coord.is_granted('aisle_1', 'robot1') is True
    assert coord.is_granted('aisle_2', 'robot2') is True
    assert coord.is_granted('aisle_3', 'robot3') is True
    log("  PASS ✓ Non-conflicting independent segments reserved simultaneously.")


def test_f_dynamic_aisle_configuration():
    log("\n" + "=" * 60)
    log("TEST F: Dynamic Aisle Configuration & Hot Reloading")
    log("=" * 60)

    import tempfile
    import json
    from fleet_manager.reservation_manager import ReservationManager

    with tempfile.NamedTemporaryFile(mode='w', suffix='.json', delete=False) as tf:
        temp_path = tf.name
        json.dump([
            {"segment_id": "aisle_custom_1", "x_min": 10.0, "x_max": 12.0, "y_min": 0.0, "y_max": 2.0, "is_single_lane": True},
            {"segment_id": "aisle_custom_2", "x_min": -5.0, "x_max": -2.0, "y_min": -3.0, "y_max": -1.0, "is_single_lane": True},
        ], tf)

    try:
        # 1. Test loading from config file
        specs = ReservationManager.load_aisle_specs(temp_path)
        assert len(specs) == 2
        assert "aisle_custom_1" in specs
        assert "aisle_custom_2" in specs
        assert specs["aisle_custom_1"]["x_min"] == 10.0

        # 2. Test saving to config file
        updated_specs = dict(specs)
        updated_specs["aisle_custom_3"] = {
            "segment_id": "aisle_custom_3",
            "x_min": 0.0, "x_max": 3.0, "y_min": 5.0, "y_max": 8.0,
            "is_single_lane": True
        }
        success = ReservationManager.save_aisle_specs(temp_path, updated_specs)
        assert success is True

        reloaded = ReservationManager.load_aisle_specs(temp_path)
        assert len(reloaded) == 3
        assert "aisle_custom_3" in reloaded
        log("  PASS ✓ Persistent JSON serialization and deserialization verified.")
    finally:
        if os.path.exists(temp_path):
            os.remove(temp_path)


def run_all():
    tests = [
        test_a_two_robots_request_same_aisle,
        test_b_first_robot_exits_and_waiting_robot_acquires,
        test_c_three_robots_contend_deterministic_ordering,
        test_d_communication_loss_and_stale_recovery,
        test_e_multiple_independent_aisles,
        test_f_dynamic_aisle_configuration,
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
