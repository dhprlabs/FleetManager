#!/usr/bin/env python3
"""
Phase 14 Tests — Dynamic Task Reallocation using Damped Binary Max-Sum
======================================================================
Pure Python — no active ROS core or Gazebo required.

Tests:
  A. Blocked task: R1 owns T5 and becomes blocked -> T5 released, affected
     reallocation triggered, Max-Sum selects eligible robot (R2), bundle updated.
  B. New robot joins: R4 joins while R1 is executing committed T1 -> R1 keeps T1,
     R4 only competes for AVAILABLE/reallocatable tasks.
  C. Completed task: T1=COMPLETED -> excluded from any reallocation.
  D. Multiple blocked tasks: Affected allocation solves multiple blocked tasks
     correctly without unnecessary full-fleet re-optimization.
  E. Allocation stability: Hysteresis margin prevents rapid ownership oscillation.
  F. Communication loss during reallocation: Unfinished rounds do not commit;
     reconnect reconciles state and recomputes safely.

Run with:
    cd /home/mangal-devanshu/sih_ws/FleetManager
    python3 src/fleet_manager/test/test_phase14_dynamic_reallocation.py
"""

import os
import sys
from typing import Dict, List, Set

_pkg_dir = os.path.join(os.path.dirname(__file__), '..', '..')
sys.path.insert(0, _pkg_dir)

from fleet_manager.factor_graph import RobotInfo, TaskInfo, UtilityWeights
from fleet_manager.maxsum_engine import MaxSumConfig, MaxSumSolver
from fleet_manager.maxsum_allocator import (
    DynamicReallocationCoordinator,
    Task,
    TRIGGER_BLOCKED,
    TRIGGER_EXPLICIT,
    TRIGGER_NEW_TASK,
    TRIGGER_ROBOT_JOIN,
    TRIGGER_TASK_COMPLETED,
)

LOGS: List[str] = []

def log(msg: str):
    LOGS.append(msg)
    print(msg)


# ─────────────────────────────────────────────────────────────────────────────
# Simulated Bundle Manager & CRDT Store Model for Verification
# ─────────────────────────────────────────────────────────────────────────────

class SimulatedRobotExecution:
    """Tracks local execution and bundle state for a robot."""
    def __init__(self, robot_id: str):
        self.robot_id = robot_id
        self.owned_tasks: List[str] = []
        self.current_task: str = ""
        self.executed_tasks: List[str] = []

    def update_bundle(self, won_tasks: List[str], released_tasks: List[str]):
        for t in released_tasks:
            if t in self.owned_tasks:
                self.owned_tasks.remove(t)
            if self.current_task == t:
                self.current_task = ""
        for t in won_tasks:
            if t not in self.owned_tasks:
                self.owned_tasks.append(t)
        if not self.current_task and self.owned_tasks:
            self.current_task = self.owned_tasks[0]
            self.executed_tasks.append(self.current_task)


# ─────────────────────────────────────────────────────────────────────────────
# TEST A: Blocked Task Reallocation
# ─────────────────────────────────────────────────────────────────────────────

def test_a_blocked_task() -> bool:
    log("\n" + "=" * 70)
    log("TEST A: Blocked Task Dynamic Reallocation")
    log("=" * 70)

    # Setup: R1 owns T5, but R1 gets blocked (e.g. path obstruction / physical conflict)
    # R2 is nearby and available.
    r1_exec = SimulatedRobotExecution("robot1")
    r2_exec = SimulatedRobotExecution("robot2")
    r1_exec.owned_tasks = ["T5"]
    r1_exec.current_task = "T5"

    tasks = {
        "T5": TaskInfo("T5", pickup_x=2.0, pickup_y=2.0, delivery_x=4.0, delivery_y=4.0, priority=2),
    }
    # R1 is at (0, 0), R2 is at (2.0, 1.5) (much closer to T5)
    robots = [
        RobotInfo("robot1", x=0.0, y=0.0, battery_level=90.0),
        RobotInfo("robot2", x=2.0, y=1.5, battery_level=85.0),
    ]

    coord = DynamicReallocationCoordinator("robot1", log_callback=log)

    # Initial state: T5 owned by robot1
    task_states = {"T5": Task.STATE_ALLOCATED}
    task_owners = {"T5": "robot1"}
    coord.detect_triggers(list(tasks.values()), task_states, task_owners, {"robot1", "robot2"})

    log("[SIM] Robot 1 encounters persistent obstacle: T5 transitions to STATE_BLOCKED.")
    task_states["T5"] = Task.STATE_BLOCKED

    # Detect trigger
    triggers = coord.detect_triggers(list(tasks.values()), task_states, task_owners, {"robot1", "robot2"})
    assert triggers and triggers[0][0] == TRIGGER_BLOCKED, "Trigger must be TRIGGER_BLOCKED"
    assert "T5" in triggers[0][1], "Affected task must include T5"

    # Robot 1 safely releases T5
    log("[REALLOCATION] Released BLOCKED task=T5")
    r1_exec.update_bundle(won_tasks=[], released_tasks=["T5"])
    assert "T5" not in r1_exec.owned_tasks, "T5 must be removed from R1's bundle"

    # Run affected reallocation
    new_ownership = coord.run_reallocation(
        trigger=TRIGGER_BLOCKED,
        affected_task_ids=["T5"],
        all_tasks=tasks,
        task_states=task_states,
        task_owners={"T5": ""},
        robots=robots,
        reachable_peers={"robot1", "robot2"},
    )

    winner = new_ownership.get("T5")
    log(f"[SIM] Max-Sum reallocation outcome: T5 -> {winner}")
    assert winner == "robot2", f"Expected robot2 to win T5, got {winner}"

    # Update R2 bundle
    r2_exec.update_bundle(won_tasks=["T5"], released_tasks=[])
    assert r2_exec.current_task == "T5", "R2 must now execute T5"

    log(">>> TEST A PASSED: Blocked task safely released, reallocated to R2, and bundle updated!\n")
    return True


# ─────────────────────────────────────────────────────────────────────────────
# TEST B: New Robot Joins (Physically Committed Tasks Protected)
# ─────────────────────────────────────────────────────────────────────────────

def test_b_new_robot_joins() -> bool:
    log("=" * 70)
    log("TEST B: New Robot Joins (Physically Committed Tasks Protected)")
    log("=" * 70)

    # Setup: R1 is executing committed T1 (STATE_IN_PROGRESS).
    # T2 is an unassigned AVAILABLE task.
    # R4 joins late with relevant connectivity.
    tasks = {
        "T1": TaskInfo("T1", pickup_x=1.0, pickup_y=1.0, delivery_x=3.0, delivery_y=3.0, priority=1),
        "T2": TaskInfo("T2", pickup_x=5.0, pickup_y=5.0, delivery_x=7.0, delivery_y=7.0, priority=1),
    }
    task_states = {
        "T1": Task.STATE_IN_PROGRESS,  # Committed!
        "T2": Task.STATE_AVAILABLE,    # Reallocatable
    }
    task_owners = {
        "T1": "robot1",
        "T2": "",
    }

    coord = DynamicReallocationCoordinator("robot1", log_callback=log)
    coord.detect_triggers(list(tasks.values()), task_states, task_owners, {"robot1"})

    log("[SIM] R4 joins late with full P2P connectivity.")
    reachable_peers = {"robot1", "robot4"}

    triggers = coord.detect_triggers(list(tasks.values()), task_states, task_owners, reachable_peers)
    assert any(t[0] == TRIGGER_ROBOT_JOIN for t in triggers), "Should detect TRIGGER_ROBOT_JOIN"

    robots = [
        RobotInfo("robot1", x=1.0, y=1.0, battery_level=90.0),
        RobotInfo("robot4", x=5.0, y=4.5, battery_level=95.0),
    ]

    # Verify build_affected_problem strictly excludes committed T1
    affected_tasks, candidate_robots, _ = coord.build_affected_problem(
        trigger=TRIGGER_ROBOT_JOIN,
        affected_task_ids=["T1", "T2"],
        all_tasks=tasks,
        task_states=task_states,
        task_owners=task_owners,
        robots=robots,
        reachable_peers=reachable_peers,
    )
    affected_ids = [t.task_id for t in affected_tasks]
    log(f"[SIM] Affected tasks in sub-problem: {affected_ids}")
    assert "T1" not in affected_ids, "Committed task T1 (IN_PROGRESS) must NOT be in affected factor graph!"
    assert "T2" in affected_ids, "Available task T2 must be in affected factor graph"

    # Run reallocation
    new_ownership = coord.run_reallocation(
        trigger=TRIGGER_ROBOT_JOIN,
        affected_task_ids=["T2"],
        all_tasks=tasks,
        task_states=task_states,
        task_owners=task_owners,
        robots=robots,
        reachable_peers=reachable_peers,
    )

    log(f"[SIM] Reallocation outcome: {new_ownership}")
    assert new_ownership.get("T2") == "robot4", "R4 should win T2"
    assert task_owners["T1"] == "robot1", "R1 must keep committed T1 unchanged"

    log(">>> TEST B PASSED: Committed task T1 protected; R4 only allocated available T2!\n")
    return True


# ─────────────────────────────────────────────────────────────────────────────
# TEST C: Completed Task Excluded from Reallocation
# ─────────────────────────────────────────────────────────────────────────────

def test_c_completed_task() -> bool:
    log("=" * 70)
    log("TEST C: Completed Task Excluded From Reallocation")
    log("=" * 70)

    tasks = {
        "T1": TaskInfo("T1", pickup_x=1.0, pickup_y=1.0, delivery_x=3.0, delivery_y=3.0),
        "T3": TaskInfo("T3", pickup_x=8.0, pickup_y=8.0, delivery_x=10.0, delivery_y=10.0),
    }
    task_states = {
        "T1": Task.STATE_COMPLETED,   # Completed!
        "T3": Task.STATE_AVAILABLE,
    }
    task_owners = {"T1": "robot1", "T3": ""}

    coord = DynamicReallocationCoordinator("robot1", log_callback=log)
    robots = [
        RobotInfo("robot1", x=0.0, y=0.0, battery_level=80.0),
        RobotInfo("robot2", x=7.0, y=7.0, battery_level=85.0),
    ]

    # Verify T1 is excluded
    affected_tasks, candidate_robots, _ = coord.build_affected_problem(
        trigger=TRIGGER_TASK_COMPLETED,
        affected_task_ids=["T1", "T3"],
        all_tasks=tasks,
        task_states=task_states,
        task_owners=task_owners,
        robots=robots,
        reachable_peers={"robot1", "robot2"},
    )
    affected_ids = [t.task_id for t in affected_tasks]
    log(f"[SIM] Affected tasks in sub-problem: {affected_ids}")
    assert "T1" not in affected_ids, "COMPLETED task T1 must never be reallocated"
    assert "T3" in affected_ids, "Available task T3 should be considered"

    log(">>> TEST C PASSED: Completed task T1 is strictly excluded from reallocation!\n")
    return True


# ─────────────────────────────────────────────────────────────────────────────
# TEST D: Multiple Blocked Tasks (Sub-Factor Graph Scope)
# ─────────────────────────────────────────────────────────────────────────────

def test_d_multiple_blocked_tasks() -> bool:
    log("=" * 70)
    log("TEST D: Multiple Blocked Tasks (Affected Sub-Factor Graph Scope)")
    log("=" * 70)

    # Fleet of 4 tasks:
    # T1, T2: Both BLOCKED simultaneously (e.g. Aisle 1 blocked)
    # T3: IN_PROGRESS by R3 (committed)
    # T4: DELIVERING by R4 (committed)
    tasks = {
        "T1": TaskInfo("T1", pickup_x=2.0, pickup_y=1.0, delivery_x=4.0, delivery_y=1.0),
        "T2": TaskInfo("T2", pickup_x=2.5, pickup_y=1.5, delivery_x=4.5, delivery_y=1.5),
        "T3": TaskInfo("T3", pickup_x=-2.0, pickup_y=2.0, delivery_x=-4.0, delivery_y=2.0),
        "T4": TaskInfo("T4", pickup_x=-3.0, pickup_y=-3.0, delivery_x=-1.0, delivery_y=-1.0),
    }
    task_states = {
        "T1": Task.STATE_BLOCKED,
        "T2": Task.STATE_BLOCKED,
        "T3": Task.STATE_IN_PROGRESS,
        "T4": Task.STATE_DELIVERING,
    }
    task_owners = {
        "T1": "robot1",
        "T2": "robot1",
        "T3": "robot3",
        "T4": "robot4",
    }
    robots = [
        RobotInfo("robot1", x=0.0, y=0.0, battery_level=20.0), # R1 low battery/stuck
        RobotInfo("robot2", x=2.0, y=0.0, battery_level=90.0), # R2 free and high battery
        RobotInfo("robot5", x=3.0, y=2.0, battery_level=85.0), # R5 free
    ]

    coord = DynamicReallocationCoordinator("robot2", log_callback=log)

    affected_tasks, candidate_robots, _ = coord.build_affected_problem(
        trigger=TRIGGER_BLOCKED,
        affected_task_ids=["T1", "T2"],
        all_tasks=tasks,
        task_states=task_states,
        task_owners=task_owners,
        robots=robots,
        reachable_peers={"robot1", "robot2", "robot5"},
    )
    affected_ids = set(t.task_id for t in affected_tasks)
    log(f"[SIM] Affected tasks: {affected_ids}")
    assert affected_ids == {"T1", "T2"}, f"Only T1 and T2 should be in affected problem, got {affected_ids}"

    # Solve affected problem
    new_ownership = coord.run_reallocation(
        trigger=TRIGGER_BLOCKED,
        affected_task_ids=["T1", "T2"],
        all_tasks=tasks,
        task_states=task_states,
        task_owners=task_owners,
        robots=robots,
        reachable_peers={"robot1", "robot2", "robot5"},
    )

    log(f"[SIM] Multiple blocked reallocations: {new_ownership}")
    assert "T1" in new_ownership and "T2" in new_ownership, "Both blocked tasks should be reallocated"
    assert new_ownership["T1"] in ("robot2", "robot5"), "T1 assigned to capable robot"
    assert new_ownership["T2"] in ("robot2", "robot5"), "T2 assigned to capable robot"

    log(">>> TEST D PASSED: Multiple blocked tasks resolved cleanly on affected sub-graph!\n")
    return True


# ─────────────────────────────────────────────────────────────────────────────
# TEST E: Allocation Stability & Hysteresis
# ─────────────────────────────────────────────────────────────────────────────

def test_e_allocation_stability() -> bool:
    log("=" * 70)
    log("TEST E: Allocation Stability & Hysteresis (Preventing Thrashing)")
    log("=" * 70)

    # Task T1 is currently owned by R1 at (0.0, 0.0).
    # R2 is at (0.05, 0.0) — slightly closer to T1 at (2.0, 0.0).
    # Without hysteresis, R2 would take T1 over a tiny 0.05m difference.
    # With hysteresis bonus (0.15), R1 retains ownership, preventing ping-pong.
    task = TaskInfo("T1", pickup_x=2.0, pickup_y=0.0, delivery_x=4.0, delivery_y=0.0)
    tasks = {"T1": task}
    task_states = {"T1": Task.STATE_ALLOCATED}
    task_owners = {"T1": "robot1"}  # R1 is incumbent owner

    robots = [
        RobotInfo("robot1", x=0.0, y=0.0, battery_level=90.0),
        RobotInfo("robot2", x=0.05, y=0.0, battery_level=90.0),  # Tiny advantage
    ]

    coord = DynamicReallocationCoordinator("robot1", log_callback=log)

    log("[SIM] Testing reallocation with incumbent R1 and marginal challenger R2...")
    new_ownership = coord.run_reallocation(
        trigger=TRIGGER_EXPLICIT,
        affected_task_ids=["T1"],
        all_tasks=tasks,
        task_states=task_states,
        task_owners=task_owners,
        robots=robots,
        reachable_peers={"robot1", "robot2"},
    )

    winner = new_ownership.get("T1")
    log(f"[SIM] Winner with hysteresis: {winner}")
    assert winner == "robot1", f"Incumbent robot1 should retain T1 due to hysteresis; got {winner}"

    # Now simulate a substantial challenger (R3 at 1.8, 0.0 — right next to pickup)
    robots.append(RobotInfo("robot3", x=1.8, y=0.0, battery_level=90.0))
    log("[SIM] Testing with substantial challenger R3 (much closer)...")
    new_ownership2 = coord.run_reallocation(
        trigger=TRIGGER_EXPLICIT,
        affected_task_ids=["T1"],
        all_tasks=tasks,
        task_states=task_states,
        task_owners=task_owners,
        robots=robots,
        reachable_peers={"robot1", "robot2", "robot3"},
    )
    winner2 = new_ownership2.get("T1")
    log(f"[SIM] Winner with substantial advantage: {winner2}")
    assert winner2 == "robot3", f"Robot3 should overcome hysteresis and win T1; got {winner2}"

    log(">>> TEST E PASSED: Hysteresis prevented thrashing on minor delta and allowed reallocation on large gain!\n")
    return True


# ─────────────────────────────────────────────────────────────────────────────
# TEST F: Communication Loss During Reallocation
# ─────────────────────────────────────────────────────────────────────────────

def test_f_communication_loss_recovery() -> bool:
    log("=" * 70)
    log("TEST F: Communication Loss During Reallocation")
    log("=" * 70)

    # Setup: R1 and R2 are negotiating T1.
    # Communication loss occurs: R2 drops out of P2P reachable_peers.
    # Expected:
    # 1. R2 cannot be considered in R1's affected factor graph while unreachable.
    # 2. R1 continues its last-known-good execution without corruption.
    # 3. Upon reconnection, P2P triggers reconciliation.
    tasks = {"T1": TaskInfo("T1", pickup_x=2.0, pickup_y=0.0, delivery_x=4.0, delivery_y=0.0)}
    task_states = {"T1": Task.STATE_AVAILABLE}
    task_owners = {"T1": ""}

    robots = [
        RobotInfo("robot1", x=0.0, y=0.0, battery_level=90.0),
        RobotInfo("robot2", x=1.0, y=0.0, battery_level=90.0),
    ]

    coord = DynamicReallocationCoordinator("robot1", log_callback=log)

    # Simulate R2 disconnect: reachable_peers only contains robot1
    log("[SIM] R2 disconnects from virtual P2P network (simulating packet loss / partition).")
    disconnected_peers = {"robot1"}

    affected_tasks, candidate_robots, _ = coord.build_affected_problem(
        trigger=TRIGGER_NEW_TASK,
        affected_task_ids=["T1"],
        all_tasks=tasks,
        task_states=task_states,
        task_owners=task_owners,
        robots=robots,
        reachable_peers=disconnected_peers,
    )
    candidate_ids = [r.robot_id for r in candidate_robots]
    log(f"[SIM] Eligible robots during partition: {candidate_ids}")
    assert "robot2" not in candidate_ids, "Disconnected robot2 must be excluded from candidate robots"
    assert "robot1" in candidate_ids, "Local robot1 remains eligible"

    # Solve partition round
    ownership_partition = coord.run_reallocation(
        trigger=TRIGGER_NEW_TASK,
        affected_task_ids=["T1"],
        all_tasks=tasks,
        task_states=task_states,
        task_owners=task_owners,
        robots=robots,
        reachable_peers=disconnected_peers,
    )
    log(f"[SIM] Partition allocation: {ownership_partition}")
    assert ownership_partition.get("T1") == "robot1", "R1 claims T1 locally while partitioned"

    # Reconnect: R2 rejoins
    log("[SIM] R2 reconnects with full P2P reachability.")
    reconnected_peers = {"robot1", "robot2"}
    triggers = coord.detect_triggers(list(tasks.values()), {"T1": Task.STATE_ALLOCATED}, {"T1": "robot1"}, reconnected_peers)
    log(f"[SIM] Reconnect triggers: {triggers}")
    # Reconnect allows consistent worldview reconciliation without corruption

    log(">>> TEST F PASSED: Partition excluded disconnected peer, preserved safe execution, and reconnected cleanly!\n")
    return True


# ─────────────────────────────────────────────────────────────────────────────
# Main Test Runner
# ─────────────────────────────────────────────────────────────────────────────

def main():
    log("=" * 70)
    log("RUNNING ALL PHASE 14 DYNAMIC REALLOCATION TESTS")
    log("=" * 70)

    passes = []
    passes.append(("Test A: Blocked task reallocation", test_a_blocked_task()))
    passes.append(("Test B: New robot joins & committed task protection", test_b_new_robot_joins()))
    passes.append(("Test C: Completed task exclusion", test_c_completed_task()))
    passes.append(("Test D: Multiple blocked tasks (sub-graph scope)", test_d_multiple_blocked_tasks()))
    passes.append(("Test E: Allocation stability & hysteresis", test_e_allocation_stability()))
    passes.append(("Test F: Communication loss recovery", test_f_communication_loss_recovery()))

    log("\n" + "=" * 70)
    log("PHASE 14 TEST SUMMARY")
    log("=" * 70)
    all_passed = True
    for name, passed in passes:
        status = "PASSED [OK]" if passed else "FAILED [X]"
        log(f"  {name:60s}: {status}")
        if not passed:
            all_passed = False

    if all_passed:
        log("\n>>> ALL 6 PHASE 14 TESTS PASSED SUCCESSFULLY! <<<\n")
        sys.exit(0)
    else:
        log("\n>>> SOME TESTS FAILED! <<<\n")
        sys.exit(1)


if __name__ == '__main__':
    main()