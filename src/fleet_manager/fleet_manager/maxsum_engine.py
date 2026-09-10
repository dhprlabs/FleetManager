#!/usr/bin/env python3
"""
Max-Sum Engine — Core Factor Graph Message Passing Algorithm
-------------------------------------------------------------
Implements the decentralized Max-Sum algorithm for multi-robot task allocation.

Workflow per round:
1. Initialize messages q_{v->f} and r_{f->v}
2. Compute local utilities U(i, t)
3. For each iteration:
   a. Compute Variable -> Factor messages (q_{v->f}) with damping and normalization
   b. Compute Factor -> Variable messages (r_{f->v}) via closed-form exclusivity
   c. Update marginal beliefs b_v(d)
   d. Check convergence (delta < threshold or stable assignment)
   e. Log iteration progress, sender/receiver, utility, and current beliefs
4. Iteration limit fallback if convergence is not reached
5. Extract final task ownership assignments
"""

import math
from typing import Dict, List, Any, Optional, Tuple, Callable
from dataclasses import dataclass, field

from fleet_manager.factor_graph import (
    RobotInfo,
    TaskInfo,
    UtilityWeights,
    VariableNode,
    TaskExclusivityFactor,
    RobotUtilityFactor,
    FactorGraph,
    VariableToFactorMessage,
    FactorToVariableMessage,
    calculate_robot_task_utility,
    build_task_allocation_factor_graph,
)


@dataclass
class MaxSumConfig:
    """Configuration parameters for Max-Sum solver."""
    max_iterations: int = 20            # Max iterations before fallback
    convergence_threshold: float = 1e-3 # Max belief difference for convergence
    damping_factor: float = 0.4        # Damping factor gamma in [0.0, 1.0)
    stable_iterations_required: int = 2 # Consecutive stable iterations to declare convergence
    verbose: bool = True               # Enable detailed debug logging


@dataclass
class MaxSumIterationLog:
    """Detailed log record for each iteration step."""
    iteration: int
    sender: str
    receiver: str
    message_type: str
    task_id: str
    utility_value: float
    current_belief: float
    current_selected_owner: str


class MaxSumSolver:
    """
    Decentralized Factor Graph Max-Sum solver.
    Operates on a FactorGraph instance with VariableNodes and FactorNodes.
    """
    def __init__(
        self,
        graph: FactorGraph,
        config: Optional[MaxSumConfig] = None,
        log_callback: Optional[Callable[[str], None]] = None
    ):
        self.graph = graph
        self.config = config or MaxSumConfig()
        self.log_callback = log_callback or (lambda msg: None)

        # Message stores:
        # q_messages[(var_name, factor_name)] = {val: utility}
        self.q_messages: Dict[Tuple[str, str], Dict[str, float]] = {}
        # r_messages[(factor_name, var_name)] = {val: utility}
        self.r_messages: Dict[Tuple[str, str], Dict[str, float]] = {}

        # Belief store: beliefs[var_name] = {val: total_utility}
        self.beliefs: Dict[str, Dict[str, float]] = {}
        self.previous_beliefs: Dict[str, Dict[str, float]] = {}

        # Current best assignment: {var_name: domain_val}
        self.current_assignment: Dict[str, str] = {}
        self.previous_assignment: Dict[str, str] = {}

        # Detailed step-by-step audit logs
        self.iteration_logs: List[MaxSumIterationLog] = []

        self._initialize_messages()

    def _log(self, text: str) -> None:
        if self.config.verbose:
            self.log_callback(text)

    def _initialize_messages(self) -> None:
        """Initializes all q and r messages to zero over variable domains."""
        for vname, var in self.graph.variables.items():
            for fname, factor in var.neighbors.items():
                self.q_messages[(vname, fname)] = {val: 0.0 for val in var.domain}
                self.r_messages[(fname, vname)] = {val: 0.0 for val in var.domain}

        for vname, var in self.graph.variables.items():
            self.beliefs[vname] = {val: 0.0 for val in var.domain}
            self.previous_beliefs[vname] = {val: 0.0 for val in var.domain}
            self.current_assignment[vname] = 'none'

    def compute_variable_to_factor(self, var_name: str, target_factor_name: str) -> Dict[str, float]:
        """
        Computes q_{v -> f}(x_v):
        q_{v -> f}(x_v) = alpha + U(v, x_v) + sum_{f' in N(v) \\ {f}} r_{f' -> v}(x_v)
        Applies damping: q = (1 - gamma)*q_new + gamma*q_old
        Applies mean normalization: alpha = - (1/|D|) sum q(x)
        """
        var = self.graph.variables[var_name]
        q_new: Dict[str, float] = {}

        # Local utility from connected RobotUtilityFactor (if present)
        unary_factor_name = f"U_{var_name}"
        unary_factor = self.graph.factors.get(unary_factor_name)

        for val in var.domain:
            # Base utility
            val_util = 0.0
            if unary_factor and target_factor_name != unary_factor_name:
                val_util = unary_factor.evaluate({var_name: val})
                if math.isinf(val_util) and val_util < 0:
                    val_util = -1e6

            # Sum of incoming messages from other factors
            incoming_sum = 0.0
            for fname in var.neighbors:
                if fname != target_factor_name and fname != unary_factor_name:
                    incoming_sum += self.r_messages.get((fname, var_name), {}).get(val, 0.0)

            raw_q = val_util + incoming_sum

            # Damping with previous q value
            prev_q = self.q_messages.get((var_name, target_factor_name), {}).get(val, 0.0)
            damped_q = (1.0 - self.config.damping_factor) * raw_q + self.config.damping_factor * prev_q
            q_new[val] = damped_q

        # Mean-normalization to keep values bounded
        vals = list(q_new.values())
        if vals:
            mean_val = sum(vals) / len(vals)
            for k in q_new:
                q_new[k] -= mean_val

        return q_new

    def compute_factor_to_variable(self, factor_name: str, target_var_name: str) -> Dict[str, float]:
        """
        Computes r_{f -> v}(x_v).
        Specialized closed-form formulation for TaskExclusivityFactor:
        - For x_v = T: sum_{k != v} max_{d' != T} q_{k->f}(d')
        - For x_v != T: sum_{k != v} max_{d' != T} q_{k->f}(d') + max(0, max_{j != v} [q_{j->f}(T) - max_{d' != T} q_{j->f}(d')])
        """
        factor = self.graph.factors[factor_name]
        var = self.graph.variables[target_var_name]
        r_new: Dict[str, float] = {}

        if isinstance(factor, TaskExclusivityFactor):
            task_id = factor.task_id
            reward = factor.reward
            other_vars = [v for v in factor.neighbors if v != target_var_name]

            # For each other robot k, compute max_{d' != task_id} q_{k->f}(d')
            max_non_t: Dict[str, float] = {}
            q_t: Dict[str, float] = {}

            for ov in other_vars:
                q_ov = self.q_messages.get((ov, factor_name), {})
                non_t_vals = [q_ov[d] for d in q_ov if d != task_id]
                max_non_t[ov] = max(non_t_vals) if non_t_vals else 0.0
                q_t[ov] = q_ov.get(task_id, -1e6)

            sum_max_non_t = sum(max_non_t.values())

            # Best gain if one other robot takes task_id: (q_t[j] + reward - max_non_t[j])
            max_gain = 0.0
            for ov in other_vars:
                gain = (q_t[ov] + reward) - max_non_t[ov]
                if gain > max_gain:
                    max_gain = gain

            # Populate r_new for each value in target variable's domain
            for val in var.domain:
                if val == task_id:
                    # Robot takes the task: earns reward, no other robot takes it
                    r_new[val] = sum_max_non_t + reward
                else:
                    # Robot does NOT take task: at most one other robot may take it
                    r_new[val] = sum_max_non_t + max_gain

        elif isinstance(factor, RobotUtilityFactor):
            # Unary factor: message to own variable is just the local utility
            for val in var.domain:
                r_new[val] = factor.evaluate({target_var_name: val})
        else:
            for val in var.domain:
                r_new[val] = 0.0

        # Mean-normalization on r message to keep values bounded
        r_vals = list(r_new.values())
        if r_vals:
            mean_r = sum(r_vals) / len(r_vals)
            for k in r_new:
                r_new[k] -= mean_r

        return r_new

    def update_beliefs(self) -> None:
        """
        Calculates marginal beliefs for each variable:
        b_v(x_v) = U(v, x_v) + sum_{f in N(v)} r_{f -> v}(x_v)
        """
        for vname, var in self.graph.variables.items():
            self.previous_beliefs[vname] = dict(self.beliefs[vname])
            unary_factor = self.graph.factors.get(f"U_{vname}")

            for val in var.domain:
                u_val = unary_factor.evaluate({vname: val}) if unary_factor else 0.0
                if math.isinf(u_val) and u_val < 0:
                    u_val = -1e6

                incoming_r = sum(
                    self.r_messages.get((fname, vname), {}).get(val, 0.0)
                    for fname in var.neighbors
                    if fname != f"U_{vname}"
                )
                self.beliefs[vname][val] = u_val + incoming_r

            # Normalize beliefs for numerical stability and convergence checking
            b_vals = list(self.beliefs[vname].values())
            if b_vals:
                mean_b = sum(b_vals) / len(b_vals)
                for k in self.beliefs[vname]:
                    self.beliefs[vname][k] -= mean_b

            # Determine best assignment: x_v* = argmax b_v(x)
            best_val = 'none'
            best_score = float('-inf')
            for val in sorted(var.domain, key=lambda x: (x == 'none', x)):
                score = self.beliefs[vname][val]
                if score > best_score:
                    best_score = score
                    best_val = val

            # Record previous assignment before updating
            self.previous_assignment[vname] = self.current_assignment.get(vname, 'none')
            self.current_assignment[vname] = best_val

    def check_convergence(self) -> Tuple[bool, float]:
        """
        Evaluates max belief delta: max_{v, d} |b_v^{(t)}(d) - b_v^{(t-1)}(d)|.
        Returns (has_converged, max_delta).
        """
        max_delta = 0.0
        for vname, cur_b in self.beliefs.items():
            prev_b = self.previous_beliefs.get(vname, {})
            for val, cur_val in cur_b.items():
                prev_val = prev_b.get(val, 0.0)
                delta = abs(cur_val - prev_val)
                if delta > max_delta:
                    max_delta = delta

        converged = max_delta < self.config.convergence_threshold
        return converged, max_delta

    def run(self) -> Dict[str, Any]:
        """
        Executes the Max-Sum message-passing iterations until convergence or max_iterations.
        Returns detailed summary and task ownership assignments.
        """
        self._log("\n" + "═" * 78)
        self._log("  MAX-SUM MESSAGE PASSING ALGORITHM EXECUTION")
        self._log(f"  Max Iterations: {self.config.max_iterations} | Convergence Threshold: {self.config.convergence_threshold}")
        self._log("═" * 78)

        converged = False
        consecutive_stable = 0
        final_iteration = 0

        for iteration in range(1, self.config.max_iterations + 1):
            final_iteration = iteration

            # -------------------------------------------------------------
            # Step 1: Variable -> Factor Messages (q_{v -> f})
            # -------------------------------------------------------------
            new_q: Dict[Tuple[str, str], Dict[str, float]] = {}
            for vname, var in self.graph.variables.items():
                for fname in var.neighbors:
                    q_val = self.compute_variable_to_factor(vname, fname)
                    new_q[(vname, fname)] = q_val

            self.q_messages.update(new_q)

            # -------------------------------------------------------------
            # Step 2: Factor -> Variable Messages (r_{f -> v})
            # -------------------------------------------------------------
            new_r: Dict[Tuple[str, str], Dict[str, float]] = {}
            for fname, factor in self.graph.factors.items():
                for vname in factor.neighbors:
                    r_val = self.compute_factor_to_variable(fname, vname)
                    new_r[(fname, vname)] = r_val

            self.r_messages.update(new_r)

            # -------------------------------------------------------------
            # Step 3: Update Beliefs & Select Current Assignment
            # -------------------------------------------------------------
            self.update_beliefs()

            # -------------------------------------------------------------
            # Step 4: Check Convergence
            # -------------------------------------------------------------
            is_conv, delta = self.check_convergence()

            # Log this iteration
            self._log(f"\n[ITERATION {iteration:02d}] Max Belief Delta: {delta:.6f}")
            for vname in sorted(self.graph.variables):
                assigned = self.current_assignment[vname]
                belief_val = self.beliefs[vname][assigned]
                # Log top candidate domain utilities
                dom_summary = ", ".join(
                    f"{d}:{self.beliefs[vname][d]:.2f}"
                    for d in sorted(self.graph.variables[vname].domain)
                )
                self._log(f"  • {vname:7s} -> Selected: {assigned:5s} | Belief={belief_val:8.3f} | [{dom_summary}]")

                # Store audit log record
                self.iteration_logs.append(MaxSumIterationLog(
                    iteration=iteration,
                    sender=vname,
                    receiver="FactorGraph",
                    message_type="BELIEF_UPDATE",
                    task_id=assigned,
                    utility_value=belief_val,
                    current_belief=belief_val,
                    current_selected_owner=f"{assigned} -> {vname}" if assigned != 'none' else "none"
                ))

            assignments_stable = (
                bool(self.previous_assignment) and
                all(self.current_assignment.get(k) == self.previous_assignment.get(k) for k in self.graph.variables)
            )

            if is_conv or (assignments_stable and iteration >= 3):
                consecutive_stable += 1
                if consecutive_stable >= self.config.stable_iterations_required:
                    converged = True
                    reason = f"belief delta = {delta:.6f} < {self.config.convergence_threshold}" if is_conv else "decision assignments stabilized"
                    self._log(f"\n✓ Converged after {iteration} iterations ({reason})")
                    break
            else:
                consecutive_stable = 0

        # Iteration limit fallback if not converged
        if not converged:
            self._log(f"\n! Iteration limit of {self.config.max_iterations} reached. Falling back to current best beliefs.")

        # Extract final task ownership
        # Map: task_id -> winning robot_id
        ownership: Dict[str, str] = {}
        for vname, assigned_task in self.current_assignment.items():
            if assigned_task != 'none':
                ownership[assigned_task] = vname

        return {
            'converged': converged,
            'iterations_run': final_iteration,
            'ownership': ownership,
            'robot_assignments': dict(self.current_assignment),
            'final_beliefs': {v: dict(b) for v, b in self.beliefs.items()},
            'total_utility': self.graph.evaluate_joint(self.current_assignment),
        }


# ---------------------------------------------------------------------------
# Sequential Multi-Task Max-Sum Allocator
# ---------------------------------------------------------------------------

def run_multi_round_maxsum_allocation(
    robots: List[RobotInfo],
    tasks: List[TaskInfo],
    weights: Optional[UtilityWeights] = None,
    config: Optional[MaxSumConfig] = None,
    log_callback: Optional[Callable[[str], None]] = None
) -> Dict[str, str]:
    """
    Allocates ALL tasks across the fleet using sequential Max-Sum rounds:
    - In each round, robots bid on unallocated tasks.
    - Assigned tasks are claimed and removed from the candidate pool.
    - Robot workloads L are incremented and positions updated for subsequent rounds.
    - Continues until all tasks are owned or no more tasks can be feasibly taken.

    Returns task ownership mapping:
    {'T1': 'robot1', 'T2': 'robot2', 'T3': 'robot3', 'T4': 'robot1', 'T5': 'robot3'}
    """
    logger = log_callback or print
    cfg = config or MaxSumConfig()

    final_task_ownership: Dict[str, str] = {}
    unassigned_task_ids = set(t.task_id for t in tasks)
    task_map = {t.task_id: t for t in tasks}

    # Deep-copy robot states to track virtual workload throughout allocation
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
    logger("\n" + "█" * 78)
    logger(f"  STARTING MULTI-ROUND MAX-SUM ALLOCATION ({len(tasks)} tasks, {len(robots)} robots)")
    logger("█" * 78)

    while unassigned_task_ids and round_idx <= len(tasks):
        logger(f"\n{'═' * 30} ROUND {round_idx} {'═' * 30}")
        logger(f"Unassigned Tasks: {sorted(unassigned_task_ids)}")

        # Build candidate task list for this round
        candidate_tasks = [task_map[tid] for tid in unassigned_task_ids]

        # Build factor graph for current round
        round_graph = build_task_allocation_factor_graph(active_robots, candidate_tasks, weights)

        # Run Max-Sum solver
        solver = MaxSumSolver(round_graph, cfg, log_callback=logger)
        result = solver.run()

        round_assignments = result['robot_assignments']
        allocated_in_this_round = 0

        for r_name, chosen_task in round_assignments.items():
            if chosen_task != 'none' and chosen_task in unassigned_task_ids:
                final_task_ownership[chosen_task] = r_name
                unassigned_task_ids.remove(chosen_task)
                allocated_in_this_round += 1

                # Update virtual robot state for next round (workload + end location at delivery)
                for r in active_robots:
                    if r.robot_id == r_name:
                        r.current_workload += 1
                        t_obj = task_map[chosen_task]
                        r.x = t_obj.delivery_x
                        r.y = t_obj.delivery_y
                        # Account for battery consumption
                        d = math.hypot(t_obj.pickup_x - r.x, t_obj.pickup_y - r.y) + \
                            math.hypot(t_obj.delivery_x - t_obj.pickup_x, t_obj.delivery_y - t_obj.pickup_y)
                        r.battery_level = max(0.0, r.battery_level - (d / 20.0) * 5.0)

        logger(f"Round {round_idx} allocated: {allocated_in_this_round} tasks.")
        if allocated_in_this_round == 0:
            logger("No further tasks could be allocated. Terminating rounds.")
            break

        round_idx += 1

    logger("\n" + "█" * 78)
    logger("  FINAL MAX-SUM TASK OWNERSHIP ALLOCATION:")
    for tid in sorted(final_task_ownership):
        logger(f"    {tid} ──► {final_task_ownership[tid]}")
    logger("█" * 78 + "\n")

    return final_task_ownership
