#!/usr/bin/env python3
"""
Factor Graph Representation for Decentralized Multi-Robot Task Allocation
--------------------------------------------------------------------------
Mathematical model replacing CBBA with Factor Graph Max-Sum.

Core Design Decisions:
1. Max-Sum determines task OWNERSHIP (who does what).
2. Bundle Manager determines task ORDERING (how each robot sequences its tasks).
3. Robot decision variable domain is linear in number of tasks:
   domain(R_i) = {'none', 'T1', 'T2', ..., 'Tm'}
   Avoids O(M!) permutation explosion.

Components:
- VariableNode: Represents robot decision variables x_i in D_i
- FactorNode: Base class for graph potential functions
- TaskExclusivityFactor: Enforces that each task is assigned to at most 1 robot
- RobotUtilityFactor: Evaluates U(i, t) = -(wd*D + wt*T + wb*B + wl*L)
- FeasibilityConstraint: Validates battery, range, and operational limits
- FactorGraph: Bipartite graph container with joint utility evaluation
- MaxSumMessageStructure: Mathematical specification of q_{v->f} and r_{f->v} messages
"""

import math
from typing import Dict, List, Any, Optional, Set, Tuple
from dataclasses import dataclass, field


# ---------------------------------------------------------------------------
# Task and Robot State Data Structures
# ---------------------------------------------------------------------------

@dataclass
class TaskInfo:
    """Task specification for utility and feasibility calculation."""
    task_id: str
    pickup_x: float
    pickup_y: float
    delivery_x: float
    delivery_y: float
    priority: int = 1
    service_time: float = 5.0  # seconds at pickup/delivery
    required_battery: float = 5.0  # % battery needed for execution


@dataclass
class RobotInfo:
    """Robot state snapshot for utility calculation."""
    robot_id: str
    x: float
    y: float
    battery_level: float = 100.0  # 0 to 100 %
    current_workload: int = 0      # number of tasks already queued in bundle
    nominal_velocity: float = 0.8  # m/s
    battery_min_threshold: float = 15.0  # minimum % below which robot cannot take tasks


@dataclass
class UtilityWeights:
    """Weights for the utility function U(i, t) = -(wd*D + wt*T + wb*B + wl*L)."""
    wd: float = 0.35   # Travel distance weight
    wt: float = 0.25   # Estimated completion time weight
    wb: float = 0.25   # Battery / energy deficit weight
    wl: float = 0.15   # Workload weight


# ---------------------------------------------------------------------------
# Utility Function
# ---------------------------------------------------------------------------

def calculate_robot_task_utility(
    robot: RobotInfo,
    task: TaskInfo,
    weights: Optional[UtilityWeights] = None
) -> Tuple[float, Dict[str, float]]:
    """
    Computes U(i, t) = -(wd*D + wt*T + wb*B + wl*L)
    Higher utility is better (negative cost formulation).

    :param robot: Current state of robot i
    :param task: Target task t
    :param weights: UtilityWeights (default applied if None)
    :return: (utility, component_breakdown_dict)
    """
    if weights is None:
        weights = UtilityWeights()

    # 1. Travel Distance D: robot -> pickup -> delivery
    d_to_pickup = math.hypot(task.pickup_x - robot.x, task.pickup_y - robot.y)
    d_pickup_to_delivery = math.hypot(task.delivery_x - task.pickup_x, task.delivery_y - task.pickup_y)
    total_distance = d_to_pickup + d_pickup_to_delivery

    # 2. Estimated Time T: transit time + service handling time
    transit_time = total_distance / max(0.1, robot.nominal_velocity)
    total_time = transit_time + task.service_time

    # 3. Battery / Energy Cost B:
    # Combines battery depletion penalty with low-battery sensitivity
    # Low battery increases cost exponentially as safety buffer shrinks
    battery_deficit = max(0.0, 100.0 - robot.battery_level)
    estimated_energy_consumption = (total_distance / 20.0) * 5.0  # ~5% per 20m
    total_battery_cost = battery_deficit * 0.5 + estimated_energy_consumption

    # 4. Current Workload L: tasks already in bundle
    workload_cost = float(robot.current_workload)

    # Combined negative cost utility
    cost = (
        weights.wd * total_distance +
        weights.wt * total_time +
        weights.wb * total_battery_cost +
        weights.wl * workload_cost
    )
    utility = -cost

    breakdown = {
        'distance_m': round(total_distance, 2),
        'time_s': round(total_time, 2),
        'battery_cost': round(total_battery_cost, 2),
        'workload': workload_cost,
        'cost': round(cost, 4),
        'utility': round(utility, 4),
    }

    return utility, breakdown


# ---------------------------------------------------------------------------
# Variable Node
# ---------------------------------------------------------------------------

class VariableNode:
    """
    Represents a decision variable in the factor graph.
    For task ownership, variable x_i belongs to robot R_i.
    Domain is the set of candidate tasks: {'none', 'T1', 'T2', ..., 'Tm'}.
    """
    def __init__(self, name: str, domain: Optional[List[str]] = None):
        self.name = name
        self.domain: List[str] = list(domain) if domain else ['none']
        if 'none' not in self.domain:
            self.domain.insert(0, 'none')
        self.neighbors: Dict[str, 'FactorNode'] = {}

    def add_neighbor(self, factor: 'FactorNode') -> None:
        self.neighbors[factor.name] = factor

    def remove_neighbor(self, factor_name: str) -> None:
        self.neighbors.pop(factor_name, None)

    def restrict_domain(self, allowed_values: Set[str]) -> None:
        """Removes values outside allowed set, keeping 'none' always valid."""
        allowed_with_none = set(allowed_values) | {'none'}
        self.domain = [v for v in self.domain if v in allowed_with_none]

    def __repr__(self) -> str:
        return f"VariableNode({self.name}, domain={self.domain}, deg={len(self.neighbors)})"


# ---------------------------------------------------------------------------
# Factor Node Base Class
# ---------------------------------------------------------------------------

class FactorNode:
    """
    Base class for factor nodes in the factor graph.
    Represents a local function f(x_{N(f)}) mapping assignments to real values.
    """
    def __init__(self, name: str):
        self.name = name
        self.neighbors: Dict[str, VariableNode] = {}

    def add_neighbor(self, var: VariableNode) -> None:
        self.neighbors[var.name] = var

    def remove_neighbor(self, var_name: str) -> None:
        self.neighbors.pop(var_name, None)

    def evaluate(self, assignment: Dict[str, str]) -> float:
        """
        Evaluates the factor potential given variable assignments.
        :param assignment: Dict mapping variable names to assigned domain values.
        :return: Utility (float). -inf on hard constraint violation.
        """
        raise NotImplementedError

    def __repr__(self) -> str:
        return f"FactorNode({self.name}, vars={list(self.neighbors.keys())})"


# ---------------------------------------------------------------------------
# Specialized Factor: Task Exclusivity Constraint
# ---------------------------------------------------------------------------

class TaskExclusivityFactor(FactorNode):
    """
    Enforces that task T_j is assigned to AT MOST ONE robot.
    Connected to all robot variables R_i that have T_j in their domain.

    F_{T_j}(x_1, ..., x_n):
      0.0   if sum_{i} I(x_i == T_j) <= 1
      -inf  if sum_{i} I(x_i == T_j) > 1  (Collision / conflict penalty)
    """
    PENALTY_INCOMPATIBLE = float('-inf')

    def __init__(self, task_id: str, reward: float = 0.0):
        super().__init__(name=f"F_{task_id}")
        self.task_id = task_id
        self.reward = reward

    def evaluate(self, assignment: Dict[str, str]) -> float:
        count = 0
        for var_name in self.neighbors:
            val = assignment.get(var_name, 'none')
            if val == self.task_id:
                count += 1

        if count > 1:
            return self.PENALTY_INCOMPATIBLE
        elif count == 1:
            return self.reward
        else:
            return 0.0  # Task unassigned is valid, no conflict


# ---------------------------------------------------------------------------
# Specialized Factor: Robot Utility & Feasibility
# ---------------------------------------------------------------------------

class RobotUtilityFactor(FactorNode):
    """
    Unary factor connected to robot variable R_i.
    Evaluates U(i, t) = -(wd*D + wt*T + wb*B + wl*L) for assigned task t.
    Enforces feasibility constraints (battery, reachability).
    """
    PENALTY_INFEASIBLE = float('-inf')

    def __init__(
        self,
        robot: RobotInfo,
        tasks: Dict[str, TaskInfo],
        weights: Optional[UtilityWeights] = None
    ):
        super().__init__(name=f"U_{robot.robot_id}")
        self.robot = robot
        self.tasks = tasks
        self.weights = weights or UtilityWeights()

    def is_feasible(self, task: TaskInfo) -> Tuple[bool, str]:
        """Checks hard feasibility constraints."""
        # 1. Battery check
        if self.robot.battery_level < self.robot.battery_min_threshold:
            return False, f"Battery {self.robot.battery_level:.1f}% below minimum threshold {self.robot.battery_min_threshold}%"
        if self.robot.battery_level < task.required_battery:
            return False, f"Battery {self.robot.battery_level:.1f}% insufficient for task requirement {task.required_battery}%"

        # 2. Maximum reachable distance check (e.g. 150m warehouse bound)
        d_pickup = math.hypot(task.pickup_x - self.robot.x, task.pickup_y - self.robot.y)
        if d_pickup > 150.0:
            return False, f"Task pickup distance {d_pickup:.1f}m exceeds maximum operation radius"

        return True, "OK"

    def evaluate(self, assignment: Dict[str, str]) -> float:
        val = assignment.get(self.robot.robot_id, 'none')
        if val == 'none':
            # Idle utility is 0 (or slight negative idle penalty)
            return 0.0

        task = self.tasks.get(val)
        if not task:
            return self.PENALTY_INFEASIBLE

        feasible, reason = self.is_feasible(task)
        if not feasible:
            return self.PENALTY_INFEASIBLE

        u, _ = calculate_robot_task_utility(self.robot, task, self.weights)
        return u


# ---------------------------------------------------------------------------
# Factor Graph Container
# ---------------------------------------------------------------------------

class FactorGraph:
    """
    Bipartite Factor Graph representing the distributed task allocation problem.
    Variables (Robots) <----> Factors (Task Exclusivity + Robot Utilities).
    """
    def __init__(self):
        self.variables: Dict[str, VariableNode] = {}
        self.factors: Dict[str, FactorNode] = {}

    def add_variable(self, var: VariableNode) -> None:
        self.variables[var.name] = var

    def add_factor(self, factor: FactorNode) -> None:
        self.factors[factor.name] = factor

    def connect(self, var_name: str, factor_name: str) -> None:
        """Creates a bidirectional edge between a Variable and a Factor."""
        if var_name not in self.variables:
            raise KeyError(f"Variable '{var_name}' not in graph.")
        if factor_name not in self.factors:
            raise KeyError(f"Factor '{factor_name}' not in graph.")

        var = self.variables[var_name]
        factor = self.factors[factor_name]
        var.add_neighbor(factor)
        factor.add_neighbor(var)

    def validate_bipartite(self) -> bool:
        """Verifies strictly bipartite connections and reciprocal neighbor sets."""
        for vname, var in self.variables.items():
            for fname, f in var.neighbors.items():
                if fname not in self.factors:
                    return False
                if vname not in f.neighbors:
                    return False
        for fname, f in self.factors.items():
            for vname, var in f.neighbors.items():
                if vname not in self.variables:
                    return False
                if fname not in var.neighbors:
                    return False
        return True

    def evaluate_joint(self, assignment: Dict[str, str]) -> float:
        """
        Evaluates the joint utility J(x) = sum_{f in Factors} f(x_{N(f)})
        """
        total = 0.0
        for factor in self.factors.values():
            val = factor.evaluate(assignment)
            if math.isinf(val) and val < 0:
                return float('-inf')
            total += val
        return total

    def summary(self) -> Dict[str, Any]:
        """Returns structured metadata about the graph."""
        return {
            'num_variables': len(self.variables),
            'num_factors': len(self.factors),
            'variables': {
                vname: {'domain': v.domain, 'degree': len(v.neighbors)}
                for vname, v in self.variables.items()
            },
            'factors': {
                fname: {'type': type(f).__name__, 'neighbors': list(f.neighbors.keys())}
                for fname, f in self.factors.items()
            }
        }

    def render_ascii(self) -> str:
        """Renders an ASCII visualization of the bipartite factor graph."""
        lines = []
        lines.append("╔═══════════════════════════════════════════════════════════════════════╗")
        lines.append("║                   FACTOR GRAPH TOPOLOGY                               ║")
        lines.append("╚═══════════════════════════════════════════════════════════════════════╝")
        lines.append("")
        lines.append("  VARIABLES (Robots):")
        for vname, v in sorted(self.variables.items()):
            domain_str = ', '.join(v.domain[:6]) + ('...' if len(v.domain) > 6 else '')
            lines.append(f"    ( {vname} )  domain: [{domain_str}]  degree={len(v.neighbors)}")

        lines.append("")
        lines.append("  FACTORS & CONNECTIONS:")
        for fname, f in sorted(self.factors.items()):
            ftype = "Exclusivity" if isinstance(f, TaskExclusivityFactor) else "Utility"
            var_list = ', '.join(sorted(f.neighbors.keys()))
            lines.append(f"    [ {fname} ] ({ftype:11s}) ─── connected to: {var_list}")

        lines.append("")
        return '\n'.join(lines)


# ---------------------------------------------------------------------------
# Max-Sum Message Structure Specification
# ---------------------------------------------------------------------------

@dataclass
class VariableToFactorMessage:
    """
    Message q_{v -> f}(x_v):
    Sent from Variable node v to Factor node f.
    Carries the accumulated utility for each candidate value in v's domain:
    q_{v -> f}(x_v) = alpha_{v -> f} + sum_{f' in N(v) \\ {f}} r_{f' -> v}(x_v)
    """
    sender_var: str
    target_factor: str
    utilities: Dict[str, float]  # domain_value -> utility
    iteration: int = 0

    def normalize(self) -> None:
        """
        Subtracts the mean utility to prevent values from diverging to infinity
        over cyclic factor graphs: alpha = - (1/|D|) sum q(x).
        """
        finite_vals = [u for u in self.utilities.values() if not math.isinf(u)]
        if finite_vals:
            mean_val = sum(finite_vals) / len(finite_vals)
            for k in self.utilities:
                if not math.isinf(self.utilities[k]):
                    self.utilities[k] -= mean_val

    def to_dict(self) -> Dict[str, Any]:
        return {
            'type': 'VAR_TO_FACTOR',
            'sender_var': self.sender_var,
            'target_factor': self.target_factor,
            'utilities': {k: (v if not math.isinf(v) else -1e9) for k, v in self.utilities.items()},
            'iteration': self.iteration
        }


@dataclass
class FactorToVariableMessage:
    """
    Message r_{f -> v}(x_v):
    Sent from Factor node f to Variable node v.
    Carries the marginal maximum utility for each state of v:
    r_{f -> v}(x_v) = max_{x_{N(f) \\ {v}}} [ f(x_{N(f)}) + sum_{v' in N(f) \\ {v}} q_{v' -> f}(x_v') ]
    """
    sender_factor: str
    target_var: str
    utilities: Dict[str, float]  # domain_value -> utility
    iteration: int = 0

    def to_dict(self) -> Dict[str, Any]:
        return {
            'type': 'FACTOR_TO_VAR',
            'sender_factor': self.sender_factor,
            'target_var': self.target_var,
            'utilities': {k: (v if not math.isinf(v) else -1e9) for k, v in self.utilities.items()},
            'iteration': self.iteration
        }


# ---------------------------------------------------------------------------
# Factory Helper: Build Complete Factor Graph
# ---------------------------------------------------------------------------

def build_task_allocation_factor_graph(
    robots: List[RobotInfo],
    tasks: List[TaskInfo],
    weights: Optional[UtilityWeights] = None
) -> FactorGraph:
    """
    Constructs the canonical bipartite Factor Graph for multi-robot task allocation:
    - 1 Variable node per robot (domain = ['none'] + all feasible task_ids)
    - 1 Exclusivity Factor per task (connected to all robots that can execute it)
    - 1 Utility Factor per robot (unary potential evaluating U(i, t))
    """
    graph = FactorGraph()
    task_dict = {t.task_id: t for t in tasks}

    # 1. Add Task Exclusivity Factors
    for task in tasks:
        task_reward = 50.0 * float(task.priority)
        exclusivity_factor = TaskExclusivityFactor(task.task_id, reward=task_reward)
        graph.add_factor(exclusivity_factor)

    # 2. Add Robot Variables and Utility Factors
    for robot in robots:
        # Determine candidate domain for this robot (filter out hard infeasibilities)
        candidate_tasks = []
        for task in tasks:
            # Quick preliminary check (battery threshold)
            if robot.battery_level >= robot.battery_min_threshold:
                candidate_tasks.append(task.task_id)

        var = VariableNode(robot.robot_id, ['none'] + candidate_tasks)
        graph.add_variable(var)

        # Unary Utility Factor
        utility_factor = RobotUtilityFactor(robot, task_dict, weights)
        graph.add_factor(utility_factor)
        graph.connect(var.name, utility_factor.name)

        # Connect variable to all task exclusivity factors in its domain
        for task_id in candidate_tasks:
            factor_name = f"F_{task_id}"
            if factor_name in graph.factors:
                graph.connect(var.name, factor_name)

    return graph
