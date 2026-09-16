#!/usr/bin/env python3
"""
Damped Binary Max-Sum Engine
----------------------------
Implements Damped Binary (Decomposed) Max-Sum for decentralised multi-robot
task allocation.

Key differences from standard Max-Sum
--------------------------------------
* **Binary decomposition**: the multi-task factor graph is split into |T|
  independent binary sub-problems.  For each task j, every robot i has a
  binary variable x_{i,j} ∈ {0, 1} ("take task j" / "don't take task j").
  This avoids the O(|R| × |T|) domain and the O(|T|!) permutation explosion.

* **Scalar messages**: q_{i→j} and r_{j→i} are single scalars (net gain /
  opportunity cost), not full domain vectors.  This makes the update rules
  extremely cheap and easy to damp without numerical drift.

* **Damping**: new_msg = (1 - γ) * computed + γ * old_msg  (γ = damping_factor).

* **Convergence**: declared when max |Δmessage| < convergence_threshold for
  ``stable_iterations_required`` consecutive rounds, or max_iterations reached.

* **Deterministic tie-breaking**: argmax over robots sorted lexicographically.

* **THOP-style pruning** (Threshold-based Opportunity-cost Pruning):
  before iteration starts, compute the greedy utility of every (robot, task)
  pair and prune robot-task pairs whose utility is below
  ``max_utility_for_task - prune_threshold``.  This reduces message traffic
  and accelerates convergence.

Workflow
--------
1.  Compute raw utilities U[i][j] = calculate_robot_task_utility(robot_i, task_j).
2.  THOP pruning: build candidate sets C[j] ⊆ robots for each task j.
3.  Initialise q[i][j] = 0, r[j][i] = 0 for all (i,j) ∈ C.
4.  For each iteration:
    a. Update r_{j→i}:  r[j][i] = max_{k≠i, k∈C[j]} (U[k][j] + q[k][j])
                                  (opportunity cost: best competing bid)
       Damping: r[j][i] = (1-γ)*r_new + γ*r_old
    b. Update q_{i→j}:  q[i][j] = U[i][j] - max_{j'≠j} max(0, r[j'][i] + U[i][j'] - r[j'][i])
       Simplified:       q[i][j] = U[i][j] - best_alt_net_gain(i, j)
       Damping: q[i][j] = (1-γ)*q_new + γ*q_old
    c. Belief:  b[i][j] = U[i][j] + r[j][i]   (marginal gain of robot i taking task j)
    d. Assignment: for each task j, winner = argmax_{i∈C[j]} b[i][j]
                   only if b[winner][j] > 0  (otherwise task stays unassigned)
    e. Resolve conflicts: if robot i is winner of multiple tasks, keep the
       highest-belief one.
    f. Check convergence.
5.  Return ownership dict {task_id → robot_id}.
"""

import math
from typing import Dict, List, Optional, Tuple, Callable, Set
from dataclasses import dataclass

from fleet_manager.factor_graph import (
    RobotInfo,
    TaskInfo,
    UtilityWeights,
    calculate_robot_task_utility,
)


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

@dataclass
class MaxSumConfig:
    """Configuration parameters for Damped Binary Max-Sum solver."""
    max_iterations: int = 20             # Hard iteration cap
    convergence_threshold: float = 1e-2  # Max message delta to declare convergence
    damping_factor: float = 0.5          # γ ∈ [0, 1): higher → slower, more stable
    stable_iterations_required: int = 2  # Consecutive stable iters to stop early
    prune_threshold: float = 15.0        # THOP pruning margin (utility units)
    hysteresis_bonus: float = 0.15       # Stability hysteresis bonus for incumbent owner
    verbose: bool = True                 # Enable iteration logging


# ---------------------------------------------------------------------------
# Iteration Log (audit trail)
# ---------------------------------------------------------------------------

@dataclass
class MaxSumIterationLog:
    """Single record logged per iteration."""
    iteration: int
    sender: str
    receiver: str
    message_type: str
    task_id: str
    utility_value: float
    current_belief: float
    current_selected_owner: str


# ---------------------------------------------------------------------------
# Damped Binary Max-Sum Solver
# ---------------------------------------------------------------------------

class MaxSumSolver:
    """
    Damped Binary (Decomposed) Max-Sum solver.

    Operates on a fleet of robots and tasks directly — no FactorGraph object
    needed (the factor graph is implicit in the binary decomposition).
    """

    def __init__(
        self,
        robots: List[RobotInfo],
        tasks: List[TaskInfo],
        weights: Optional[UtilityWeights] = None,
        config: Optional[MaxSumConfig] = None,
        log_callback: Optional[Callable[[str], None]] = None,
        incumbents: Optional[Dict[str, str]] = None,
    ):
        self.robots = robots
        self.tasks = tasks
        self.weights = weights or UtilityWeights()
        self.config = config or MaxSumConfig()
        self.log_callback = log_callback or (lambda m: None)
        self.incumbents = incumbents or {}

        self.robot_ids: List[str] = [r.robot_id for r in robots]
        self.task_ids: List[str] = [t.task_id for t in tasks]
        self._robot_map: Dict[str, RobotInfo] = {r.robot_id: r for r in robots}
        self._task_map: Dict[str, TaskInfo] = {t.task_id: t for t in tasks}

        # Raw utility table U[robot_id][task_id]
        self.U: Dict[str, Dict[str, float]] = {}
        # Binary messages
        self.q: Dict[str, Dict[str, float]] = {}   # q[robot_id][task_id]
        self.r: Dict[str, Dict[str, float]] = {}   # r[task_id][robot_id]
        # Beliefs: b[robot_id][task_id]
        self.beliefs: Dict[str, Dict[str, float]] = {}
        # Current assignment: task_id -> robot_id
        self.assignment: Dict[str, str] = {}
        self.prev_assignment: Dict[str, str] = {}

        # THOP candidate sets: C[task_id] = set of robot_ids eligible
        self.candidates: Dict[str, Set[str]] = {}

        # Audit logs
        self.iteration_logs: List[MaxSumIterationLog] = []

        self._build_utilities()
        self._thop_pruning()
        self._init_messages()

    # -----------------------------------------------------------------------
    # Helpers
    # -----------------------------------------------------------------------

    def _log(self, text: str) -> None:
        if self.config.verbose:
            self.log_callback(text)

    def _build_utilities(self) -> None:
        """Compute U[robot][task] for all pairs (feasibility = -inf)."""
        for robot in self.robots:
            self.U[robot.robot_id] = {}
            for task in self.tasks:
                u, _ = calculate_robot_task_utility(robot, task, self.weights)
                # Hard feasibility: battery check
                if robot.battery_level < robot.battery_min_threshold:
                    u = float('-inf')
                elif self.incumbents and self.incumbents.get(task.task_id) == robot.robot_id:
                    # Incumbent stability bonus prevents allocation thrashing on tiny deltas
                    u += self.config.hysteresis_bonus
                self.U[robot.robot_id][task.task_id] = u

    def _thop_pruning(self) -> None:
        """
        THOP-style pruning: for each task j, keep only robots whose utility
        is within `prune_threshold` of the best robot for that task.
        Infeasible robots (-inf) are always pruned.
        """
        for task in self.tasks:
            tj = task.task_id
            utils = {
                rid: self.U[rid][tj]
                for rid in self.robot_ids
                if not math.isinf(self.U[rid][tj])
            }
            if not utils:
                self.candidates[tj] = set()
                continue
            best = max(utils.values())
            threshold = best - self.config.prune_threshold
            self.candidates[tj] = {
                rid for rid, u in utils.items() if u >= threshold
            }

        # Log pruning results
        for tj, cands in self.candidates.items():
            pruned = set(self.robot_ids) - cands
            if pruned:
                self._log(
                    f'[THOP] Task {tj}: pruned {sorted(pruned)}, '
                    f'kept {sorted(cands)}'
                )

    def _init_messages(self) -> None:
        """Initialise all q and r messages to 0."""
        for rid in self.robot_ids:
            self.q[rid] = {tj: 0.0 for tj in self.task_ids}
            self.beliefs[rid] = {tj: 0.0 for tj in self.task_ids}
        for tj in self.task_ids:
            self.r[tj] = {rid: 0.0 for rid in self.robot_ids}

        self.assignment = {tj: '' for tj in self.task_ids}
        self.prev_assignment = {tj: '' for tj in self.task_ids}

    # -----------------------------------------------------------------------
    # Binary Max-Sum Update Rules
    # -----------------------------------------------------------------------

    def _update_r(self) -> float:
        """
        Factor → Variable update (opportunity cost message).

        r_{j→i} = max_{k ∈ C[j], k ≠ i} (U[k][j] + q[k][j])

        This is the "best competing bid" for task j, excluding robot i.
        Damped: r_new = (1-γ)*r_computed + γ*r_old
        Returns max |Δr|.
        """
        gamma = self.config.damping_factor
        max_delta = 0.0

        for tj in self.task_ids:
            cands = self.candidates[tj]
            # Collect all bids for this task
            bids = {
                rid: self.U[rid][tj] + self.q[rid][tj]
                for rid in cands
            }

            for rid in self.robot_ids:
                # Best bid from OTHER candidates
                competing_bids = [v for k, v in bids.items() if k != rid]
                r_computed = max(competing_bids) if competing_bids else 0.0

                r_old = self.r[tj][rid]
                r_new = (1.0 - gamma) * r_computed + gamma * r_old
                delta = abs(r_new - r_old)
                if delta > max_delta:
                    max_delta = delta
                self.r[tj][rid] = r_new

        return max_delta

    def _update_q(self) -> float:
        """
        Variable → Factor update (net preference message).

        q_{i→j} = U[i][j] - max_{j' ≠ j} max(0, net_gain(i, j'))
        where net_gain(i, j') = U[i][j'] + r[j'][i]

        Intuition: robot i's preference for task j, accounting for the
        opportunity cost of giving up its best alternative task.
        Damped: q_new = (1-γ)*q_computed + γ*q_old
        Returns max |Δq|.
        """
        gamma = self.config.damping_factor
        max_delta = 0.0

        for rid in self.robot_ids:
            # Net gain for each task this robot is a candidate for
            net_gains = {}
            for tj in self.task_ids:
                if rid in self.candidates[tj]:
                    net_gains[tj] = self.U[rid][tj] + self.r[tj][rid]

            for tj in self.task_ids:
                if rid not in self.candidates.get(tj, set()):
                    # Not a candidate: pin q to a large negative
                    self.q[rid][tj] = -1e6
                    continue

                # Best alternative net gain (excluding task j)
                alt_gains = [g for t2, g in net_gains.items() if t2 != tj]
                best_alt = max(alt_gains) if alt_gains else 0.0
                best_alt = max(0.0, best_alt)  # Idle is always an option

                q_computed = self.U[rid][tj] - best_alt
                q_old = self.q[rid][tj]
                q_new = (1.0 - gamma) * q_computed + gamma * q_old
                delta = abs(q_new - q_old)
                if delta > max_delta:
                    max_delta = delta
                self.q[rid][tj] = q_new

        return max_delta

    def _update_beliefs(self) -> None:
        """
        Marginal belief: b[i][j] = U[i][j] - r[j][i]

        ``r[j][i]`` is the best competing utility/opportunity cost for task j.
        It must be subtracted from robot i's local utility.  Adding it is
        incorrect for the negative-cost utility convention and can make a
        farther robot beat the best candidate after damping.
        """
        for rid in self.robot_ids:
            for tj in self.task_ids:
                self.beliefs[rid][tj] = self.U[rid][tj] - self.r[tj][rid]

    def _extract_assignment(self) -> Dict[str, str]:
        """
        Maximum-utility, one-task-per-robot matching for a binary round.

        Cardinality is maximized first so every feasible connected robot
        receives work before a later sequential round fills bundles.  Ties are
        resolved deterministically by the task/robot identifiers.
        """
        task_ids = sorted(self.task_ids)
        robot_ids = sorted(self.robot_ids)
        best_count = -1
        best_score = float('-inf')
        best_canonical: Tuple[Tuple[str, str], ...] = ()
        best_assignment: Dict[str, str] = {}

        def search(index: int, used: Set[str], assignment: Dict[str, str], score: float):
            nonlocal best_count, best_score, best_canonical, best_assignment
            if index == len(task_ids):
                canonical = tuple(sorted(assignment.items()))
                count = len(assignment)
                if (count > best_count or
                        (count == best_count and score > best_score) or
                        (count == best_count and score == best_score and
                         (not best_canonical or canonical < best_canonical))):
                    best_count = count
                    best_score = score
                    best_canonical = canonical
                    best_assignment = dict(assignment)
                return

            task_id = task_ids[index]
            # Leave a task for a later sequential allocation round.
            search(index + 1, used, assignment, score)
            for robot_id in robot_ids:
                if robot_id in used or robot_id not in self.candidates.get(task_id, set()):
                    continue
                assignment[task_id] = robot_id
                used.add(robot_id)
                search(index + 1, used, assignment, score + self.U[robot_id][task_id])
                used.remove(robot_id)
                del assignment[task_id]

        search(0, set(), {}, 0.0)
        return best_assignment

    # -----------------------------------------------------------------------
    # Main Solve Loop
    # -----------------------------------------------------------------------

    def run(self) -> Dict:
        """
        Execute Damped Binary Max-Sum until convergence or iteration cap.

        Returns dict with:
          'converged'         : bool
          'iterations_run'    : int
          'ownership'         : {task_id: robot_id}
          'robot_assignments' : {robot_id: task_id | 'none'}
          'final_beliefs'     : {robot_id: {task_id: float}}
        """
        self._log('\n' + '═' * 78)
        self._log('  DAMPED BINARY MAX-SUM — TASK ALLOCATION')
        self._log(
            f'  Robots: {self.robot_ids} | Tasks: {self.task_ids} | '
            f'γ={self.config.damping_factor} | MaxIter={self.config.max_iterations} | '
            f'ε={self.config.convergence_threshold}'
        )
        self._log('═' * 78)

        converged = False
        stable_count = 0
        final_iter = 0

        for iteration in range(1, self.config.max_iterations + 1):
            final_iter = iteration
            self.prev_assignment = dict(self.assignment)

            # --- Step 1: r update (factor → variable) ---
            delta_r = self._update_r()

            # --- Step 2: q update (variable → factor) ---
            delta_q = self._update_q()

            # --- Step 3: Belief update ---
            self._update_beliefs()

            # --- Step 4: Extract assignment ---
            self.assignment = self._extract_assignment()

            max_delta = max(delta_r, delta_q)

            # --- Step 5: Convergence check ---
            assignments_stable = (self.assignment == self.prev_assignment)
            belief_converged = (max_delta < self.config.convergence_threshold)

            self._log(
                f'\n[ITER {iteration:02d}] max_delta={max_delta:.5f} | '
                f'belief_conv={belief_converged} | stable={assignments_stable}'
            )
            for tj in sorted(self.task_ids):
                winner = self.assignment.get(tj, '')
                bel = self.beliefs.get(winner, {}).get(tj, float('nan')) if winner else float('nan')
                cand_str = ', '.join(
                    f'{rid}:{self.beliefs[rid][tj]:.2f}'
                    for rid in sorted(self.candidates.get(tj, set()))
                )
                self._log(
                    f'  Task {tj}: winner={winner or "none"} '
                    f'belief={bel:.3f} | [{cand_str}]'
                )
                self.iteration_logs.append(MaxSumIterationLog(
                    iteration=iteration,
                    sender='MaxSumEngine',
                    receiver=winner or 'none',
                    message_type='BINARY_BELIEF',
                    task_id=tj,
                    utility_value=self.U.get(winner, {}).get(tj, float('nan')) if winner else float('nan'),
                    current_belief=bel,
                    current_selected_owner=winner or 'none',
                ))

            if belief_converged or assignments_stable:
                stable_count += 1
                if stable_count >= self.config.stable_iterations_required:
                    reason = (
                        f'Δ={max_delta:.5f} < ε={self.config.convergence_threshold}'
                        if belief_converged else 'assignments stabilised'
                    )
                    self._log(f'\n✓ Converged after {iteration} iterations ({reason})')
                    converged = True
                    break
            else:
                stable_count = 0

        if not converged:
            self._log(
                f'\n! Hit iteration limit ({self.config.max_iterations}). '
                f'Using best current assignment.'
            )

        # Build robot_assignments (robot_id → task_id | 'none')
        robot_assignments = {rid: 'none' for rid in self.robot_ids}
        for tj, rid in self.assignment.items():
            robot_assignments[rid] = tj

        self._log('\n  FINAL ASSIGNMENT:')
        for tj in sorted(self.task_ids):
            owner = self.assignment.get(tj, '')
            self._log(f'    {tj} ──► {owner or "UNASSIGNED"}')
        self._log('═' * 78 + '\n')

        return {
            'converged': converged,
            'iterations_run': final_iter,
            'ownership': dict(self.assignment),
            'robot_assignments': robot_assignments,
            'final_beliefs': {rid: dict(self.beliefs[rid]) for rid in self.robot_ids},
            'utilities': {rid: dict(self.U[rid]) for rid in self.robot_ids},
            'candidates': {tid: sorted(candidates) for tid, candidates in self.candidates.items()},
        }


# ---------------------------------------------------------------------------
# Multi-Round Allocation (called by MaxSumAllocator)
# ---------------------------------------------------------------------------

def run_multi_round_maxsum_allocation(
    robots: List[RobotInfo],
    tasks: List[TaskInfo],
    weights: Optional[UtilityWeights] = None,
    config: Optional[MaxSumConfig] = None,
    log_callback: Optional[Callable[[str], None]] = None,
    incumbents: Optional[Dict[str, str]] = None,
    round_callback: Optional[Callable[[Dict], None]] = None,
) -> Dict[str, str]:
    """
    Allocates all tasks using sequential Damped Binary Max-Sum rounds.

    Each round runs one full solver instance on unallocated tasks.
    Assigned tasks are removed from the pool; robot virtual positions and
    workloads are advanced for the next round.

    Returns task ownership: {task_id: robot_id}
    """
    logger = log_callback or print
    cfg = config or MaxSumConfig()
    w = weights or UtilityWeights()

    final_ownership: Dict[str, str] = {}
    unassigned: Set[str] = {t.task_id for t in tasks}
    task_map: Dict[str, TaskInfo] = {t.task_id: t for t in tasks}

    # Deep-copy robot states to track virtual progress
    active_robots = [
        RobotInfo(
            robot_id=r.robot_id,
            x=r.x,
            y=r.y,
            battery_level=r.battery_level,
            current_workload=r.current_workload,
            nominal_velocity=r.nominal_velocity,
            battery_min_threshold=r.battery_min_threshold,
        )
        for r in robots
    ]

    round_idx = 1
    logger('\n' + '█' * 78)
    logger(
        f'  MULTI-ROUND DAMPED BINARY MAX-SUM '
        f'({len(tasks)} tasks, {len(robots)} robots)'
    )
    logger('█' * 78)

    while unassigned and round_idx <= len(tasks):
        logger(f"\n{'═' * 28} ROUND {round_idx} {'═' * 28}")
        logger(f'  Unassigned: {sorted(unassigned)}')

        candidate_tasks = [task_map[tid] for tid in unassigned]

        solver = MaxSumSolver(
            robots=active_robots,
            tasks=candidate_tasks,
            weights=w,
            config=cfg,
            log_callback=logger,
            incumbents=incumbents,
        )
        result = solver.run()
        if round_callback:
            round_callback({
                'round': round_idx,
                'candidate_tasks': sorted(unassigned),
                'ownership': dict(result['ownership']),
                'iterations': result['iterations_run'],
                'converged': result['converged'],
                'utilities': result['utilities'],
                'beliefs': result['final_beliefs'],
            })

        allocated_this_round = 0
        for tj, owner in result['ownership'].items():
            if tj in unassigned and owner:
                final_ownership[tj] = owner
                unassigned.discard(tj)
                allocated_this_round += 1

                # Advance virtual robot state
                for r in active_robots:
                    if r.robot_id == owner:
                        r.current_workload += 1
                        t_obj = task_map[tj]
                        # Simple battery decay
                        d = (
                            math.hypot(t_obj.pickup_x - r.x, t_obj.pickup_y - r.y)
                            + math.hypot(
                                t_obj.delivery_x - t_obj.pickup_x,
                                t_obj.delivery_y - t_obj.pickup_y,
                            )
                        )
                        r.battery_level = max(0.0, r.battery_level - (d / 20.0) * 5.0)
                        r.x = t_obj.delivery_x
                        r.y = t_obj.delivery_y
                        break

        logger(f'  Round {round_idx}: allocated {allocated_this_round} tasks.')
        if allocated_this_round == 0:
            logger('  No progress — stopping early.')
            break
        round_idx += 1

    logger('\n' + '█' * 78)
    logger('  FINAL TASK OWNERSHIP:')
    for tid in sorted(final_ownership):
        logger(f'    {tid} ──► {final_ownership[tid]}')
    logger('█' * 78 + '\n')

    return final_ownership
