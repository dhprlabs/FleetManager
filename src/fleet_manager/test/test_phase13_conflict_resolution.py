#!/usr/bin/env python3
"""
Phase 13 Tests — Physical Conflict Resolution using PIBT with Dynamic Priority Aging and ORCA
=============================================================================================
Pure Python — no active ROS core or Gazebo required.

Tests:
  A. Open-space crossing: Two robots approach each other -> ORCA resolves collision, collision-free outcome.
  B. Single-lane aisle: Two robots approach -> ORCA disabled, reservation/PIBT resolves conflict.
  C. Starvation: R1 repeatedly waits while R2 crosses -> R1 priority ages, R1 eventually gains priority.
  D. Three-robot choke point: Deterministic resolution, no deadlock.
  E. Navigation integration: Nav2 continues normal execution, override only occurs during physical conflict.

Run with:
    cd /home/mangal-devanshu/sih_ws/FleetManager
    python3 src/fleet_manager/test/test_phase13_conflict_resolution.py
"""

import os
import sys
import math
import time
from typing import Dict, List, Tuple

_pkg_dir = os.path.join(os.path.dirname(__file__), '..', '..')
sys.path.insert(0, _pkg_dir)

from fleet_manager.conflict_resolver import (
    Vector2,
    ORCAEngine,
    PIBTEngine,
    PIBTAgent,
    PIBTNode,
    ConflictCoordinator,
    DEFAULT_AISLE_SEGMENTS,
)

LOGS: List[str] = []

def log(msg: str):
    LOGS.append(msg)
    print(msg)


# ─────────────────────────────────────────────────────────────────────────────
# TEST A: Open-space Crossing (Continuous ORCA Avoidance)
# ─────────────────────────────────────────────────────────────────────────────

def test_a_open_space_crossing() -> bool:
    log("\n" + "=" * 70)
    log("TEST A: Open-Space Crossing (ORCA Continuous Local Collision Avoidance)")
    log("=" * 70)

    # Robot 1 starts at (0.0, 4.0), heading South towards (0.0, -4.0) at 0.5 m/s
    pos1 = Vector2(0.0, 4.0)
    pref_vel1 = Vector2(0.0, -0.5)
    vel1 = Vector2(0.0, -0.5)

    # Robot 2 starts at (0.0, -4.0), heading North towards (0.0, 4.0) at 0.5 m/s
    pos2 = Vector2(0.0, -4.0)
    pref_vel2 = Vector2(0.0, 0.5)
    vel2 = Vector2(0.0, 0.5)

    coord1 = ConflictCoordinator("robot_1")
    coord2 = ConflictCoordinator("robot_2")

    min_distance = float('inf')
    dt = 0.1
    sim_time = 0.0
    max_sim_time = 16.0
    orca_activated_1 = False
    orca_activated_2 = False

    log(f"[SIM] Initial positions: R1={pos1}, R2={pos2}")
    log("[SIM] Head-on collision trajectory in open space...")

    while sim_time < max_sim_time:
        # Check coordinator for R1
        override1, active1, log1 = coord1.compute_override(
            my_pos=pos1, my_vel=vel1, nav2_pref_vel=pref_vel1,
            peer_states={"robot_2": (pos2, vel2)}, active_reservation_holder=None, dt=dt
        )
        if active1:
            orca_activated_1 = True
            vel1 = override1
        else:
            vel1 = pref_vel1

        # Check coordinator for R2
        override2, active2, log2 = coord2.compute_override(
            my_pos=pos2, my_vel=vel2, nav2_pref_vel=pref_vel2,
            peer_states={"robot_1": (pos1, vel1)}, active_reservation_holder=None, dt=dt
        )
        if active2:
            orca_activated_2 = True
            vel2 = override2
        else:
            vel2 = pref_vel2

        # Step simulation
        pos1 = pos1 + vel1 * dt
        pos2 = pos2 + vel2 * dt

        dist = (pos1 - pos2).length()
        if dist < min_distance:
            min_distance = dist

        # Log close encounters
        if 2.0 <= dist <= 3.0 and int(sim_time * 10) % 15 == 0:
            log(f"  t={sim_time:.1f}s | dist={dist:.2f}m | R1_vel={vel1} | R2_vel={vel2} | ORCA active: {active1}")

        # Re-aim preferred velocity towards original goal
        target1 = Vector2(0.0, -4.0)
        target2 = Vector2(0.0, 4.0)
        dir1 = (target1 - pos1)
        dir2 = (target2 - pos2)
        if dir1.length() > 0.2:
            pref_vel1 = dir1.normalized() * 0.5
        else:
            pref_vel1 = Vector2(0.0, 0.0)

        if dir2.length() > 0.2:
            pref_vel2 = dir2.normalized() * 0.5
        else:
            pref_vel2 = Vector2(0.0, 0.0)

        sim_time += dt

    log(f"[RESULT] Minimum distance between robots during encounter: {min_distance:.3f} m")
    log(f"[RESULT] ORCA activated for R1: {orca_activated_1}, for R2: {orca_activated_2}")
    log(f"[RESULT] Final positions: R1={pos1}, R2={pos2}")

    # Robot combined physical radius = 2 * 0.35 = 0.70 m.
    # Safe distance must remain >= 0.70 m without contact.
    assert orca_activated_1 and orca_activated_2, "ORCA should have activated during head-on encounter"
    assert min_distance >= 0.70, f"Collision occurred! min_dist {min_distance:.3f} < 0.70m"
    assert (pos1.y < 0.0 and pos2.y > 0.0), "Robots should have successfully crossed each other"

    log(">>> TEST A PASSED: ORCA successfully resolved open-space crossing with zero collision!\n")
    return True


# ─────────────────────────────────────────────────────────────────────────────
# TEST B: Single-Lane Aisle (ORCA Disabled, Reservation/PIBT Resolves)
# ─────────────────────────────────────────────────────────────────────────────

def test_b_single_lane_aisle() -> bool:
    log("=" * 70)
    log("TEST B: Single-Lane Aisle (ORCA Disabled, Reservation/PIBT Resolves Conflict)")
    log("=" * 70)

    # Aisle 1: x in [1.5, 4.0], y in [0.0, 2.5]
    aisle_spec = DEFAULT_AISLE_SEGMENTS['aisle_1']
    log(f"[CONFIG] Testing Aisle 1: bounds x=[{aisle_spec['x_min']}, {aisle_spec['x_max']}], "
        f"y=[{aisle_spec['y_min']}, {aisle_spec['y_max']}]")

    # Robot 1 has active reservation and is inside aisle traversing east
    pos1 = Vector2(2.5, 1.25)
    vel1 = Vector2(0.4, 0.0)
    pref1 = Vector2(0.4, 0.0)

    # Robot 2 is at eastern entrance wanting to enter heading west
    pos2 = Vector2(4.2, 1.25)
    vel2 = Vector2(-0.4, 0.0)
    pref2 = Vector2(-0.4, 0.0)

    coord1 = ConflictCoordinator("robot_1")
    coord2 = ConflictCoordinator("robot_2")

    # Verify coordinator detects single-lane aisle
    assert coord1.is_in_single_lane_aisle(pos1) == 'aisle_1', "Robot 1 should be detected inside aisle_1"
    assert coord2.is_approaching_aisle(pos2, pos2 + pref2) == 'aisle_1', "Robot 2 should be detected approaching aisle_1"

    # Robot 1 holds reservation
    override1, active1, log1 = coord1.compute_override(
        my_pos=pos1, my_vel=vel1, nav2_pref_vel=pref1,
        peer_states={"robot_2": (pos2, vel2)},
        active_reservation_holder="robot_1",
        dt=0.1
    )
    log(f"R1: {log1}")
    assert not active1, "Robot 1 holds reservation; override should NOT be active (Nav2 proceeds)"
    assert "ORCA disabled" in log1, "ORCA must be explicitly disabled inside single-lane aisle"

    # Robot 2 approaches without reservation
    override2, active2, log2 = coord2.compute_override(
        my_pos=pos2, my_vel=vel2, nav2_pref_vel=pref2,
        peer_states={"robot_1": (pos1, vel1)},
        active_reservation_holder="robot_1",
        dt=0.1
    )
    log(f"R2: {log2}")
    assert active2, "Robot 2 does not hold reservation; override MUST be active"
    assert override2.x == 0.0 and override2.y == 0.0, "Robot 2 must be commanded to STOP"
    assert "ORCA disabled" in log2, "ORCA must be disabled for Robot 2 approaching single-lane aisle"
    assert "choke wait active" in log2.lower(), "PIBT choke wait must be active for Robot 2"

    log(">>> TEST B PASSED: Single-lane aisle correctly disabled ORCA and enforced reservation/PIBT wait!\n")
    return True


# ─────────────────────────────────────────────────────────────────────────────
# TEST C: Starvation & Dynamic Priority Aging
# ─────────────────────────────────────────────────────────────────────────────

def test_c_starvation_and_priority_aging() -> bool:
    log("=" * 70)
    log("TEST C: Starvation & Dynamic Priority Aging at Choke Point")
    log("=" * 70)

    # Robot 1: Low base priority (1.0)
    # Robot 2: High base priority (3.0)
    pibt = PIBTEngine(aging_rate=0.5)
    r1 = pibt.register_agent("robot_1", base_priority=1.0)
    r2 = pibt.register_agent("robot_2", base_priority=3.0)

    log(f"[INIT] R1 initial: {r1}")
    log(f"[INIT] R2 initial: {r2}")
    assert r1.sort_key() > r2.sort_key(), "R2 must initially have higher priority than R1"

    # Simulate R1 repeatedly waiting at choke point while R2 contends
    dt = 1.0  # 1 second steps
    simulated_seconds = 0.0
    r1_gained_priority = False

    log("\n[SIM] R1 waiting at choke point while R2 requests passage...")
    for step in range(1, 8):
        simulated_seconds += dt
        r1.record_waiting(dt)

        log(f"  t={simulated_seconds:.1f}s | R1 eff_prio={r1.effective_priority:.2f} (wait={r1.wait_time:.1f}s) "
            f"vs R2 eff_prio={r2.effective_priority:.2f}")

        # Check if R1's effective priority surpasses R2
        if r1.sort_key() < r2.sort_key():  # Note: lower sort_key tuple means higher priority!
            r1_gained_priority = True
            log(f"  >>> Dynamic Aging Success at t={simulated_seconds:.1f}s! R1 effective priority ({r1.effective_priority:.2f}) "
                f"surpassed R2 ({r2.effective_priority:.2f})!")
            break

    assert r1_gained_priority, "R1 should have gained priority over R2 due to priority aging"

    # Verify reset after successful passage
    r1.reset_waiting()
    log(f"[RESET] R1 granted passage -> wait reset: {r1}")
    assert r1.wait_time == 0.0 and r1.effective_priority == 1.0, "R1 wait time and priority must reset after grant"

    log(">>> TEST C PASSED: Dynamic priority aging successfully eliminated starvation!\n")
    return True


# ─────────────────────────────────────────────────────────────────────────────
# TEST D: Three-Robot Choke Point (Deterministic Resolution, No Deadlock)
# ─────────────────────────────────────────────────────────────────────────────

def test_d_three_robot_choke_point() -> bool:
    log("=" * 70)
    log("TEST D: Three-Robot Choke Point (Deterministic PIBT Resolution, No Deadlock)")
    log("=" * 70)

    # Three robots R1, R2, R3 converge on single narrow choke point 'C'
    # Nodes: 'A' (West), 'B' (North), 'D' (East), 'C' (Choke point Center), 'E' (Exit)
    nodes = {
        'A': PIBTNode('A', 0.0, 1.0),
        'B': PIBTNode('B', 1.0, 2.0),
        'D': PIBTNode('D', 2.0, 1.0),
        'C': PIBTNode('C', 1.0, 1.0, is_choke_point=True),
        'E': PIBTNode('E', 1.0, 0.0)
    }

    pibt = PIBTEngine(aging_rate=0.2)
    # Equal base priorities to rigorously test deterministic tie-breaking
    r1 = pibt.register_agent("robot_1", base_priority=1.0)
    r2 = pibt.register_agent("robot_2", base_priority=1.0)
    r3 = pibt.register_agent("robot_3", base_priority=1.0)

    r1.current_node = 'A'
    r2.current_node = 'B'
    r3.current_node = 'D'

    # All three want to pass through C towards E
    targets = {
        'robot_1': ['C', 'A'],
        'robot_2': ['C', 'B'],
        'robot_3': ['C', 'D']
    }

    log("[INIT] Three robots with equal base priority (1.0) contending for choke point 'C'")
    log("  R1 at node A -> target [C, A]")
    log("  R2 at node B -> target [C, B]")
    log("  R3 at node D -> target [C, D]")

    # Step 1: Contention for C
    grants1 = pibt.step(nodes, targets, dt=0.5)
    log(f"[STEP 1] Grants: {grants1}")
    # Tie-breaking by robot_id: robot_1 should win C
    assert grants1.get('robot_1') == 'C', f"robot_1 should be granted C by deterministic tie-break, got {grants1}"
    assert grants1.get('robot_2') == 'B', "robot_2 should backtrack/hold at B"
    assert grants1.get('robot_3') == 'D', "robot_3 should backtrack/hold at D"

    # Step 2: R1 advances to Exit E. R2 and R3 contend for C.
    r1.current_node = 'C'
    targets['robot_1'] = ['E']
    grants2 = pibt.step(nodes, targets, dt=0.5)
    log(f"[STEP 2] Grants: {grants2}")
    assert grants2.get('robot_1') == 'E', "robot_1 should advance to E"
    assert grants2.get('robot_2') == 'C', f"robot_2 should win C over robot_3, got {grants2}"
    assert grants2.get('robot_3') == 'D', "robot_3 should wait at D"

    # Step 3: R2 advances to E. R3 takes C.
    r2.current_node = 'C'
    targets['robot_2'] = ['E']
    grants3 = pibt.step(nodes, targets, dt=0.5)
    log(f"[STEP 3] Grants: {grants3}")
    assert grants3.get('robot_2') == 'E', "robot_2 should advance to E"
    assert grants3.get('robot_3') == 'C', "robot_3 should finally be granted C"

    log(">>> TEST D PASSED: Three-robot choke point resolved deterministically with zero deadlock!\n")
    return True


# ─────────────────────────────────────────────────────────────────────────────
# TEST E: Navigation Integration (Nav2 Uninterrupted, Override Only on Conflict)
# ─────────────────────────────────────────────────────────────────────────────

def test_e_nav2_integration() -> bool:
    log("=" * 70)
    log("TEST E: Navigation Integration (Nav2 Uninterrupted, Override Only on Conflict)")
    log("=" * 70)

    coord = ConflictCoordinator("robot_1")

    # Situation 1: Normal open-space travel along Nav2 path, no peers
    pos1 = Vector2(-2.0, 5.0)
    vel1 = Vector2(0.5, 0.0)
    pref1 = Vector2(0.5, 0.0)

    override, is_active, explanation = coord.compute_override(
        my_pos=pos1, my_vel=vel1, nav2_pref_vel=pref1,
        peer_states={}, active_reservation_holder=None, dt=0.1
    )
    log(f"[CASE 1] Clear navigation: active={is_active} | {explanation}")
    assert not is_active, "Nav2 command should NOT be overridden when path is clear"
    assert override is None, "Override velocity should be None when clear"

    # Situation 2: Distant peer (> 4.5m away)
    override_dist, is_active_dist, explanation_dist = coord.compute_override(
        my_pos=pos1, my_vel=vel1, nav2_pref_vel=pref1,
        peer_states={"robot_2": (Vector2(4.0, 5.0), Vector2(-0.5, 0.0))},
        active_reservation_holder=None, dt=0.1
    )
    log(f"[CASE 2] Distant peer (6m away): active={is_active_dist} | {explanation_dist}")
    assert not is_active_dist, "Distant peer outside sensing radius must not trigger override"

    # Situation 3: Imminent open space collision within 1.5m
    override_conf, is_active_conf, explanation_conf = coord.compute_override(
        my_pos=pos1, my_vel=vel1, nav2_pref_vel=pref1,
        peer_states={"robot_2": (Vector2(-0.8, 5.0), Vector2(-0.5, 0.0))},
        active_reservation_holder=None, dt=0.1
    )
    log(f"[CASE 3] Imminent collision (1.2m away): active={is_active_conf} | {explanation_conf}")
    assert is_active_conf, "Imminent collision MUST trigger velocity override"
    assert override_conf is not None and (override_conf.y != 0.0 or override_conf.x != pref1.x), \
        "ORCA must alter velocity to avoid collision"

    # Situation 4: Nav2 is holding position beside another robot.  This must
    # not be reported as an avoidance activation or publish an override.
    override_idle, is_active_idle, explanation_idle = coord.compute_override(
        my_pos=pos1, my_vel=Vector2(0.0, 0.0), nav2_pref_vel=Vector2(0.0, 0.0),
        peer_states={"robot_2": (Vector2(-0.8, 5.0), Vector2(0.0, 0.0))},
        active_reservation_holder=None, dt=0.1
    )
    log(f"[CASE 4] Stationary Nav2 command: active={is_active_idle} | {explanation_idle}")
    assert not is_active_idle and override_idle is None, \
        "A stationary Nav2 command must not trigger ORCA avoidance"

    log(">>> TEST E PASSED: Nav2 commands proceed uninterrupted and override only triggers on conflict!\n")
    return True


# ─────────────────────────────────────────────────────────────────────────────
# Main Test Runner
# ─────────────────────────────────────────────────────────────────────────────

def main():
    log("=" * 70)
    log("RUNNING ALL PHASE 13 CONFLICT RESOLUTION TESTS")
    log("=" * 70)

    passes = []
    passes.append(("Test A: Open-space crossing (ORCA)", test_a_open_space_crossing()))
    passes.append(("Test B: Single-lane aisle (ORCA disabled, Reservation/PIBT)", test_b_single_lane_aisle()))
    passes.append(("Test C: Starvation & Priority Aging", test_c_starvation_and_priority_aging()))
    passes.append(("Test D: Three-robot choke point", test_d_three_robot_choke_point()))
    passes.append(("Test E: Navigation Integration & Override", test_e_nav2_integration()))

    log("\n" + "=" * 70)
    log("PHASE 13 TEST SUMMARY")
    log("=" * 70)
    all_passed = True
    for name, passed in passes:
        status = "PASSED [OK]" if passed else "FAILED [X]"
        log(f"  {name:60s}: {status}")
        if not passed:
            all_passed = False

    if all_passed:
        log("\n>>> ALL 5 PHASE 13 TESTS PASSED SUCCESSFULLY! <<<\n")
        sys.exit(0)
    else:
        log("\n>>> SOME TESTS FAILED! <<<\n")
        sys.exit(1)


if __name__ == '__main__':
    main()
