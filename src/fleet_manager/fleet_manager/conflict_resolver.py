#!/usr/bin/env python3
"""
Conflict Resolver Node — Phase 13 Implementation
-------------------------------------------------
Runs identically on each robot.
Physical coordination layer combining:
  1. Priority Inheritance with Backtracking (PIBT) with dynamic priority aging for discrete conflict resolution.
  2. Optimal Reciprocal Collision Avoidance (ORCA) for continuous open-space multi-robot collision avoidance.
  3. Integration with Reservation Manager for narrow single-lane aisle segments.

CRITICAL RULES:
  - Max-Sum remains strictly for task ownership (Conflict Resolver never changes task ownership).
  - Bundle Manager remains responsible for local task ordering.
  - Reservation Manager remains responsible for single-lane aisle reservations.
  - Conflict Resolver never directly modifies CRDT task state.
  - Nav2 remains the primary navigation system.
  - Velocity override via /cmd_vel_override ONLY occurs when physical conflict or choke point wait exists.
  - ORCA is EXPLICITLY DISABLED inside single-lane aisle segments (which use Reservation + PIBT).
  - Dynamic priority aging eliminates starvation at choke points; tie-breaking is deterministic (robot_id).
"""

import json
import math
import time
from typing import Dict, List, Optional, Set, Tuple

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist, PoseStamped, Point
    from std_msgs.msg import String
    from fleet_interfaces.msg import Intent, Reservation, RobotState
    _ROS_AVAILABLE = True
except ImportError:
    _ROS_AVAILABLE = False
    # Mock base class for pure Python test execution
    class Node:
        def __init__(self, name: str):
            pass
    class Twist:
        def __init__(self):
            class Vec:
                def __init__(self): self.x, self.y, self.z = 0.0, 0.0, 0.0
            self.linear = Vec()
            self.angular = Vec()
    class Intent:
        pass
    class RobotState:
        pass
    class Reservation:
        STATE_REQUESTED = 0
        STATE_GRANTED = 1
        STATE_ACTIVE = 2
        STATE_RELEASED = 3
        STATE_WAITING = 4
        STATE_EXPIRED = 5
        STATE_DENIED = 6
    class String:
        def __init__(self): self.data = ''

try:
    from fleet_manager.reservation_manager import DEFAULT_AISLE_SEGMENTS
except ImportError:
    DEFAULT_AISLE_SEGMENTS = {
        'aisle_1': {'segment_id': 'aisle_1', 'x_min': 1.5, 'x_max': 4.0, 'y_min': 0.0, 'y_max': 2.5, 'is_single_lane': True},
        'aisle_2': {'segment_id': 'aisle_2', 'x_min': 1.5, 'x_max': 4.0, 'y_min': -3.5, 'y_max': -1.0, 'is_single_lane': True},
        'aisle_3': {'segment_id': 'aisle_3', 'x_min': -4.5, 'x_max': -2.0, 'y_min': -2.0, 'y_max': 2.0, 'is_single_lane': True},
    }

from fleet_manager.audit_log import audit_event


# ─────────────────────────────────────────────────────────────────────────────
# 2D Vector & ORCA Geometry Utilities
# ─────────────────────────────────────────────────────────────────────────────

class Vector2:
    __slots__ = ('x', 'y')

    def __init__(self, x: float = 0.0, y: float = 0.0):
        self.x = float(x)
        self.y = float(y)

    def __add__(self, other: 'Vector2') -> 'Vector2':
        return Vector2(self.x + other.x, self.y + other.y)

    def __sub__(self, other: 'Vector2') -> 'Vector2':
        return Vector2(self.x - other.x, self.y - other.y)

    def __mul__(self, scalar: float) -> 'Vector2':
        return Vector2(self.x * scalar, self.y * scalar)

    def __rmul__(self, scalar: float) -> 'Vector2':
        return Vector2(self.x * scalar, self.y * scalar)

    def __neg__(self) -> 'Vector2':
        return Vector2(-self.x, -self.y)

    def __truediv__(self, scalar: float) -> 'Vector2':
        if abs(scalar) < 1e-9:
            return Vector2(0.0, 0.0)
        return Vector2(self.x / scalar, self.y / scalar)

    def dot(self, other: 'Vector2') -> float:
        return self.x * other.x + self.y * other.y

    def length_sq(self) -> float:
        return self.x * self.x + self.y * self.y

    def length(self) -> float:
        return math.hypot(self.x, self.y)

    def normalized(self) -> 'Vector2':
        l = self.length()
        if l < 1e-9:
            return Vector2(0.0, 0.0)
        return Vector2(self.x / l, self.y / l)

    def det(self, other: 'Vector2') -> float:
        return self.x * other.y - self.y * other.x

    def __repr__(self) -> str:
        return f"Vec2({self.x:.3f}, {self.y:.3f})"


class HalfPlane:
    """Represents a linear constraint: (v - point) . normal >= 0"""
    __slots__ = ('point', 'normal')

    def __init__(self, point: Vector2, normal: Vector2):
        self.point = point
        self.normal = normal.normalized()


# ─────────────────────────────────────────────────────────────────────────────
# ORCA Continuous Collision Avoidance Engine
# ─────────────────────────────────────────────────────────────────────────────

class ORCAEngine:
    """
    Optimal Reciprocal Collision Avoidance for continuous 2D local collision avoidance.
    Assumptions: Appropriate for open/multi-robot spaces.
    NOTE: Must be disabled inside single-lane aisles.
    """
    def __init__(self, time_horizon: float = 3.0, robot_radius: float = 0.35,
                 safety_margin: float = 0.15, max_speed: float = 0.6):
        self.time_horizon = time_horizon
        self.robot_radius = robot_radius
        self.safety_margin = safety_margin
        self.combined_radius = (robot_radius + safety_margin) * 2.0
        self.max_speed = max_speed

    def compute_orca_halfplane(self, p_a: Vector2, v_a: Vector2,
                               p_b: Vector2, v_b: Vector2) -> Optional[HalfPlane]:
        """
        Computes the ORCA half-plane constraint for agent A induced by agent B.
        """
        rel_pos = p_b - p_a
        rel_vel = v_a - v_b
        dist_sq = rel_pos.length_sq()
        comb_radius = self.combined_radius
        comb_radius_sq = comb_radius * comb_radius

        tau = self.time_horizon
        inv_tau = 1.0 / tau

        if dist_sq > comb_radius_sq:
            # Not currently colliding
            w = rel_vel - (rel_pos * inv_tau)
            w_len_sq = w.length_sq()
            dot_product1 = w.dot(rel_pos)

            if dot_product1 < 0.0 and (dot_product1 * dot_product1) > (comb_radius_sq * w_len_sq):
                # Project on cut-off circle
                w_len = math.sqrt(w_len_sq)
                unit_w = w / w_len
                normal = unit_w
                u = unit_w * (comb_radius * inv_tau - w_len)

                # Symmetry breaking / Rule of the Road for head-on encounters:
                # If approaching head-on (rel_pos and rel_vel nearly collinear),
                # bias avoidance to the right (reciprocal port-to-port passing).
                cross = rel_pos.det(rel_vel)
                if abs(cross) < 0.3:
                    # Right perpendicular vector to rel_pos
                    perp = Vector2(-rel_pos.y, rel_pos.x).normalized()
                    bias = perp * (comb_radius * inv_tau * 0.8)
                    u = u + bias
                    normal = (normal + perp * 0.6).normalized()
            else:
                # Project on cone legs
                leg_len = math.sqrt(max(0.0, dist_sq - comb_radius_sq))
                if rel_pos.det(w) > 0.0:
                    # Left leg
                    normal = Vector2(
                        rel_pos.x * leg_len - rel_pos.y * comb_radius,
                        rel_pos.x * comb_radius + rel_pos.y * leg_len
                    ) / dist_sq
                else:
                    # Right leg
                    normal = -Vector2(
                        rel_pos.x * leg_len + rel_pos.y * comb_radius,
                        -rel_pos.x * comb_radius + rel_pos.y * leg_len
                    ) / dist_sq
                dot_product2 = rel_vel.dot(normal)
                u = (normal * dot_product2) - rel_vel
        else:
            # Penetration or within safety margin — emergency repulsive response
            inv_time_step = 1.0 / 0.1
            w = rel_vel - (rel_pos * inv_time_step)
            w_len = w.length()
            unit_w = w.normalized() if w_len > 1e-6 else Vector2(1.0, 0.0)
            normal = unit_w
            u = unit_w * (comb_radius * inv_time_step - w_len)

        # Agent A takes half the responsibility (reciprocal)
        half_u = u * 0.5
        point = v_a + half_u
        return HalfPlane(point, normal), u

    def solve_linear_program(self, planes: List[HalfPlane], pref_vel: Vector2) -> Vector2:
        """
        Finds velocity v with ||v|| <= max_speed that satisfies all half-plane constraints
        (v - plane.point) . plane.normal >= 0 and minimizes ||v - pref_vel||.
        """
        # Clamp preferred velocity to max speed
        if pref_vel.length() > self.max_speed:
            result = pref_vel.normalized() * self.max_speed
        else:
            result = Vector2(pref_vel.x, pref_vel.y)

        for i, plane in enumerate(planes):
            # Check if current result satisfies constraint (with small positive cushion)
            if (result - plane.point).dot(plane.normal) < 0.01:
                # Project onto line: (v - point) . normal = 0
                result = self._project_onto_line(planes[:i], plane, pref_vel)

        return result

    def _project_onto_line(self, other_planes: List[HalfPlane], line_plane: HalfPlane, pref_vel: Vector2) -> Vector2:
        """Projects optimal velocity along the 1D boundary of line_plane."""
        direction = Vector2(-line_plane.normal.y, line_plane.normal.x)
        t_center = (pref_vel - line_plane.point).dot(direction)
        cand_vel = line_plane.point + direction * t_center

        if cand_vel.length() > self.max_speed:
            cand_vel = cand_vel.normalized() * self.max_speed

        t_left = -float('inf')
        t_right = float('inf')

        for other in other_planes:
            numerator = (other.point - line_plane.point).dot(other.normal)
            denominator = direction.dot(other.normal)

            if abs(denominator) < 1e-9:
                if numerator > 0.0:
                    continue
                else:
                    return line_plane.point

            t = numerator / denominator
            if denominator > 0.0:
                t_left = max(t_left, t)
            else:
                t_right = min(t_right, t)

            if t_left > t_right:
                return cand_vel

        # Bound by max_speed circle along line
        a = direction.length_sq()
        b = 2.0 * line_plane.point.dot(direction)
        c = line_plane.point.length_sq() - self.max_speed * self.max_speed
        disc = b * b - 4.0 * a * c
        if disc >= 0.0:
            sqrt_disc = math.sqrt(disc)
            t_circ_min = (-b - sqrt_disc) / (2.0 * a)
            t_circ_max = (-b + sqrt_disc) / (2.0 * a)
            t_left = max(t_left, t_circ_min)
            t_right = min(t_right, t_circ_max)

        if t_left <= t_right:
            t_opt = max(t_left, min(t_right, t_center))
            return line_plane.point + direction * t_opt

        return cand_vel

    def compute_avoidance_velocity(self, robot_pos: Vector2, robot_vel: Vector2,
                                   pref_vel: Vector2, neighbors: List[Tuple[Vector2, Vector2]]) -> Tuple[Vector2, bool]:
        """
        Computes the collision-free ORCA velocity.
        Returns: (new_vel, conflict_detected)
        """
        planes: List[HalfPlane] = []
        conflict_detected = False

        for n_pos, n_vel in neighbors:
            rel_pos = n_pos - robot_pos
            rel_dist = rel_pos.length()
            if rel_dist > 4.5:
                continue

            # Check if moving towards each other
            rel_vel = robot_vel - n_vel
            is_closing = rel_pos.dot(rel_vel) > 0.0 or rel_dist < (self.combined_radius * 1.5)

            result = self.compute_orca_halfplane(robot_pos, robot_vel, n_pos, n_vel)
            if result is not None:
                hp, u = result
                # Active conflict if closing and correction vector u is non-zero
                if is_closing and (u.length() > 0.05 or rel_dist < (self.combined_radius * 1.5)):
                    conflict_detected = True
                planes.append(hp)

        if not conflict_detected or not planes:
            return pref_vel, False

        new_vel = self.solve_linear_program(planes, pref_vel)
        deviation = (new_vel - pref_vel).length()
        return new_vel, (deviation > 0.03 or conflict_detected)


# ─────────────────────────────────────────────────────────────────────────────
# PIBT Discrete Conflict Resolver with Dynamic Priority Aging
# ─────────────────────────────────────────────────────────────────────────────

class PIBTNode:
    """Represents a discrete position / waypoint."""
    def __init__(self, node_id: str, x: float, y: float, is_choke_point: bool = False):
        self.node_id = node_id
        self.x = x
        self.y = y
        self.is_choke_point = is_choke_point
        self.occupied_by: Optional[str] = None


class PIBTAgent:
    """Agent state for PIBT with dynamic priority aging."""
    def __init__(self, robot_id: str, base_priority: float = 1.0, aging_rate: float = 0.5):
        self.robot_id = robot_id
        self.base_priority = float(base_priority)
        self.aging_rate = float(aging_rate)
        self.wait_time: float = 0.0
        self.current_node: Optional[str] = None
        self.target_node: Optional[str] = None
        self.inherited_priority: Optional[float] = None

    @property
    def effective_priority(self) -> float:
        """
        Dynamic priority aging:
          effective_priority = base_priority + aging_rate * wait_time
        Prevents starvation of robots waiting at choke points.
        """
        base = self.inherited_priority if self.inherited_priority is not None else self.base_priority
        return base + (self.aging_rate * self.wait_time)

    def record_waiting(self, dt: float):
        """Increases wait time while yielding or waiting at choke point."""
        self.wait_time += dt

    def reset_waiting(self):
        """Resets wait time once the agent successfully moves or obtains access."""
        self.wait_time = 0.0
        self.inherited_priority = None

    def sort_key(self) -> Tuple[float, str]:
        """
        Deterministic tie-breaking:
          1. Higher effective priority comes first (negative for sorting).
          2. Lexicographical robot_id.
        """
        return (-self.effective_priority, self.robot_id)

    def __repr__(self) -> str:
        return (f"PIBTAgent({self.robot_id}: base={self.base_priority:.1f}, "
                f"wait={self.wait_time:.1f}s, eff_prio={self.effective_priority:.2f})")


class PIBTEngine:
    """
    Priority Inheritance with Backtracking (PIBT).
    Decides discrete movements at choke points and intersections deterministically.
    """
    def __init__(self, aging_rate: float = 0.5):
        self.agents: Dict[str, PIBTAgent] = {}
        self.aging_rate = aging_rate

    def register_agent(self, robot_id: str, base_priority: float = 1.0) -> PIBTAgent:
        if robot_id not in self.agents:
            self.agents[robot_id] = PIBTAgent(robot_id, base_priority, self.aging_rate)
        return self.agents[robot_id]

    def step(self, nodes: Dict[str, PIBTNode],
             agent_targets: Dict[str, List[str]],
             dt: float = 0.1) -> Dict[str, str]:
        """
        Performs one PIBT step for all registered agents.
        Returns: mapping of {robot_id: granted_node_id}
        """
        for node in nodes.values():
            node.occupied_by = None

        for r_id, agent in self.agents.items():
            if agent.current_node in nodes:
                nodes[agent.current_node].occupied_by = r_id

        sorted_agents = sorted(self.agents.values(), key=lambda a: a.sort_key())

        reserved_nodes: Dict[str, str] = {}
        grants: Dict[str, str] = {}
        resolved: Set[str] = set()

        def pibt_resolve(agent: PIBTAgent) -> bool:
            candidates = agent_targets.get(agent.robot_id, [])
            if not candidates and agent.current_node:
                candidates = [agent.current_node]

            for cand_node_id in candidates:
                if cand_node_id not in nodes:
                    continue

                if cand_node_id not in reserved_nodes:
                    curr_occ = nodes[cand_node_id].occupied_by
                    if curr_occ is None or curr_occ == agent.robot_id or curr_occ in resolved:
                        reserved_nodes[cand_node_id] = agent.robot_id
                        grants[agent.robot_id] = cand_node_id
                        resolved.add(agent.robot_id)
                        return True

                    other_agent = self.agents[curr_occ]
                    # Priority Inheritance: Can push if agent's effective priority is higher
                    if agent.effective_priority > other_agent.effective_priority:
                        other_agent.inherited_priority = max(
                            other_agent.inherited_priority or 0.0,
                            agent.effective_priority
                        )
                        reserved_nodes[cand_node_id] = agent.robot_id
                        if pibt_resolve(other_agent):
                            grants[agent.robot_id] = cand_node_id
                            resolved.add(agent.robot_id)
                            return True
                        else:
                            del reserved_nodes[cand_node_id]

            if agent.current_node and agent.current_node not in reserved_nodes:
                reserved_nodes[agent.current_node] = agent.robot_id
                grants[agent.robot_id] = agent.current_node
                resolved.add(agent.robot_id)
                return False

            return False

        for agent in sorted_agents:
            if agent.robot_id not in resolved:
                success = pibt_resolve(agent)
                if success and grants.get(agent.robot_id) != agent.current_node:
                    agent.reset_waiting()
                else:
                    agent.record_waiting(dt)

        return grants


# ─────────────────────────────────────────────────────────────────────────────
# Physical Conflict Coordinator (Combines ORCA, PIBT, and Reservation Manager)
# ─────────────────────────────────────────────────────────────────────────────

class ConflictCoordinator:
    """
    Coordinates local collision resolution:
      - Detects whether robots are in single-lane aisles.
      - Disables ORCA in single-lane aisles (Rule 6).
      - Uses Reservation Manager + PIBT for aisle access and choke points.
      - Uses ORCA for continuous avoidance in open space.
      - Computes /cmd_vel_override only when conflict/wait occurs (Rule 9).
    """
    def __init__(self, robot_id: str, aisle_segments: Optional[Dict[str, dict]] = None,
                 aging_rate: float = 0.5):
        self.robot_id = robot_id
        self.aisle_segments = aisle_segments or DEFAULT_AISLE_SEGMENTS
        self.orca = ORCAEngine(time_horizon=3.0, robot_radius=0.35, safety_margin=0.15, max_speed=0.6)
        self.pibt = PIBTEngine(aging_rate=aging_rate)
        self.pibt.register_agent(robot_id, base_priority=1.0)

        self.active_reservations: Dict[str, str] = {}
        self.is_waiting_at_choke: bool = False
        self.current_choke_segment: Optional[str] = None

    def get_aisle_at_point(self, x: float, y: float, margin: float = 0.0) -> Optional[str]:
        """Checks if a 2D point falls within any known single-lane aisle segment."""
        for seg_id, spec in self.aisle_segments.items():
            if (spec['x_min'] - margin <= x <= spec['x_max'] + margin and
                spec['y_min'] - margin <= y <= spec['y_max'] + margin):
                return seg_id
        return None

    def is_in_single_lane_aisle(self, pos: Vector2, margin: float = 0.0) -> Optional[str]:
        """Returns the segment_id if inside a single-lane aisle, else None."""
        return self.get_aisle_at_point(pos.x, pos.y, margin)

    def is_approaching_aisle(self, pos: Vector2, target: Vector2, threshold: float = 1.0) -> Optional[str]:
        """Detects if robot is within threshold of entering a single-lane aisle."""
        curr_aisle = self.get_aisle_at_point(pos.x, pos.y)
        if curr_aisle:
            return curr_aisle

        target_aisle = self.get_aisle_at_point(target.x, target.y)
        if target_aisle:
            return target_aisle

        for seg_id, spec in self.aisle_segments.items():
            dx = max(spec['x_min'] - pos.x, 0.0, pos.x - spec['x_max'])
            dy = max(spec['y_min'] - pos.y, 0.0, pos.y - spec['y_max'])
            dist = math.hypot(dx, dy)
            if dist < threshold:
                return seg_id
        return None

    def compute_override(self,
                         my_pos: Vector2,
                         my_vel: Vector2,
                         nav2_pref_vel: Vector2,
                         peer_states: Dict[str, Tuple[Vector2, Vector2]],
                         active_reservation_holder: Optional[str] = None,
                         dt: float = 0.1) -> Tuple[Optional[Vector2], bool, str]:
        """
        Main decision loop for physical conflict resolution.
        Returns:
          (override_vel, is_override_active, explanation_log)
        """
        my_agent = self.pibt.register_agent(self.robot_id)

        # ─────────────────────────────────────────────────────────────────────
        # Step 1: Check Single-Lane Aisle & Choke Points
        # ─────────────────────────────────────────────────────────────────────
        aisle_id = self.is_in_single_lane_aisle(my_pos, margin=0.1)
        approaching_aisle = aisle_id or self.is_approaching_aisle(
            my_pos,
            my_pos + nav2_pref_vel * 2.0,
            threshold=0.8
        )

        if approaching_aisle:
            # RULE 6: ORCA is EXPLICITLY DISABLED inside/near single-lane aisles.
            # Coordination relies strictly on Reservation Manager + PIBT.
            holder = active_reservation_holder

            if holder == self.robot_id:
                self.is_waiting_at_choke = False
                my_agent.reset_waiting()
                return None, False, f"[{self.robot_id}] Aisle '{approaching_aisle}' reserved by self. ORCA disabled. Nav2 executes uninterrupted."

            elif holder is not None and holder != self.robot_id:
                self.is_waiting_at_choke = True
                self.current_choke_segment = approaching_aisle
                my_agent.record_waiting(dt)

                override_vel = Vector2(0.0, 0.0)
                log_msg = (f"[{self.robot_id}] Aisle '{approaching_aisle}' held by {holder}. "
                           f"ORCA disabled. PIBT choke wait active. Priority aging: "
                           f"wait={my_agent.wait_time:.1f}s -> eff_prio={my_agent.effective_priority:.2f}. "
                           f"Velocity override: STOP.")
                return override_vel, True, log_msg

            else:
                # No active holder yet: PIBT discrete ordering resolves contention
                competing_peers = []
                for p_id, (p_pos, _) in peer_states.items():
                    if self.get_aisle_at_point(p_pos.x, p_pos.y, margin=1.0) == approaching_aisle:
                        peer_agent = self.pibt.register_agent(p_id)
                        competing_peers.append(peer_agent)

                my_sort = my_agent.sort_key()
                we_have_priority = True
                for p_agent in competing_peers:
                    if p_agent.sort_key() < my_sort:
                        we_have_priority = False
                        break

                if we_have_priority:
                    self.is_waiting_at_choke = False
                    my_agent.reset_waiting()
                    return None, False, (f"[{self.robot_id}] Contending for aisle '{approaching_aisle}': "
                                         f"Highest PIBT priority ({my_agent.effective_priority:.2f}). "
                                         f"Proceeding to enter reservation. ORCA disabled.")
                else:
                    self.is_waiting_at_choke = True
                    my_agent.record_waiting(dt)
                    override_vel = Vector2(0.0, 0.0)
                    return override_vel, True, (f"[{self.robot_id}] Contending for aisle '{approaching_aisle}': "
                                                f"Yielding to higher priority peer. Wait time {my_agent.wait_time:.1f}s. "
                                                f"Velocity override: STOP.")

        # ─────────────────────────────────────────────────────────────────────
        # Step 2: Open Space — Continuous ORCA Local Collision Avoidance
        # ─────────────────────────────────────────────────────────────────────
        # Nav2 has no active translational command.  A stationary robot must not
        # create an ORCA avoidance activation merely because another robot is
        # nearby; the command mux will continue forwarding the zero command.
        if nav2_pref_vel.length() <= 0.01:
            my_agent.reset_waiting()
            return None, False, f"[{self.robot_id}] Nav2 stationary. ORCA inactive."

        open_space_neighbors: List[Tuple[Vector2, Vector2]] = []
        for p_id, (p_pos, p_vel) in peer_states.items():
            if not self.is_in_single_lane_aisle(p_pos, margin=0.0):
                open_space_neighbors.append((p_pos, p_vel))

        if not open_space_neighbors:
            my_agent.reset_waiting()
            return None, False, f"[{self.robot_id}] Open space clear. Nav2 runs uninterrupted."

        safe_vel, conflict_detected = self.orca.compute_avoidance_velocity(
            my_pos, my_vel, nav2_pref_vel, open_space_neighbors
        )

        if conflict_detected:
            log_msg = (f"[{self.robot_id}] Open-space conflict detected! ORCA active. "
                       f"Adjusted vel: pref={nav2_pref_vel} -> orca={safe_vel}. Override ACTIVE.")
            return safe_vel, True, log_msg
        else:
            return None, False, f"[{self.robot_id}] Open space: ORCA active, no collision course. Nav2 clear."


# ─────────────────────────────────────────────────────────────────────────────
# ROS 2 Node Implementation
# ─────────────────────────────────────────────────────────────────────────────

class ConflictResolver(Node):
    """
    ROS 2 Conflict Resolver Node.
    Publishes to /cmd_vel_override when physical conflicts exist.
    """
    def __init__(self):
        super().__init__('conflict_resolver')

        default_id = self.get_namespace().strip('/') or 'robot_1'
        self.declare_parameter('robot_id', default_id)
        self.robot_id = self.get_parameter('robot_id').get_parameter_value().string_value

        self.declare_parameter('aging_rate', 0.5)
        aging_rate = self.get_parameter('aging_rate').get_parameter_value().double_value

        self.coordinator = ConflictCoordinator(robot_id=self.robot_id, aging_rate=aging_rate)

        self.peer_states: Dict[str, Tuple[Vector2, Vector2]] = {}
        self.my_pos = Vector2(0.0, 0.0)
        self.my_vel = Vector2(0.0, 0.0)
        self.nav2_pref_vel = Vector2(0.0, 0.0)
        self.active_reservations: Dict[str, str] = {}
        self._requested_aisles: Set[str] = set()
        self._reservation_clock = 0
        self._last_traffic_state = ''

        self.intent_sub = self.create_subscription(
            Intent, '/fleet/intents', self.handle_intent, 10
        )
        self.reservation_sub = self.create_subscription(
            Reservation, '/fleet/reservations', self.handle_reservation, 10
        )
        self.robot_state_sub = self.create_subscription(
            RobotState, '/fleet/robot_states', self.handle_robot_state, 10
        )
        self.nav2_cmd_sub = self.create_subscription(
            Twist, 'cmd_vel_nav', self.handle_nav2_cmd, 10
        )
        self.aisle_config_sub = self.create_subscription(
            String, '/fleet/aisle_config', self.handle_aisle_config, 10
        )

        # Nav2 is remapped to cmd_vel_nav.  This node is the command mux: it
        # forwards Nav2 while clear and substitutes ORCA/PIBT commands only
        # when physical coordination requires it.
        self.drive_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.reservation_request_pub = self.create_publisher(Reservation, '/traffic/reserve', 10)
        self.traffic_event_pub = self.create_publisher(String, '/fleet/traffic_events', 50)
        self.override_pub = self.create_publisher(Twist, '/cmd_vel_override', 10)
        self.local_override_pub = self.create_publisher(Twist, 'cmd_vel_override', 10)

        self.last_step_time = time.time()
        self.timer = self.create_timer(0.1, self.conflict_check)
        self.get_logger().info(f'[{self.robot_id}] Conflict Resolver initialized (PIBT + Aging + ORCA).')

    def handle_intent(self, msg: Intent):
        pass

    def handle_reservation(self, msg: Reservation):
        if msg.state in (Reservation.STATE_GRANTED, Reservation.STATE_ACTIVE):
            self.active_reservations[msg.segment_id] = msg.robot_id
        elif msg.state in (Reservation.STATE_RELEASED, Reservation.STATE_EXPIRED, Reservation.STATE_DENIED):
            if self.active_reservations.get(msg.segment_id) == msg.robot_id:
                del self.active_reservations[msg.segment_id]

    def handle_aisle_config(self, msg: String):
        try:
            payload = json.loads(msg.data)
            aisles_raw = payload.get('aisles', [])
            new_segments = {}
            for item in aisles_raw:
                seg_id = item.get('segment_id') or item.get('id')
                if not seg_id:
                    continue
                new_segments[seg_id] = {
                    'segment_id': seg_id,
                    'x_min': float(item['x_min']),
                    'x_max': float(item['x_max']),
                    'y_min': float(item['y_min']),
                    'y_max': float(item['y_max']),
                    'is_single_lane': bool(item.get('is_single_lane', True)),
                }
            if new_segments:
                self.coordinator.aisle_segments = new_segments
                self.get_logger().info(f"[{self.robot_id}] Updated aisle segments configuration ({len(new_segments)} aisles).")
        except Exception as e:
            self.get_logger().warn(f"[{self.robot_id}] Failed to parse aisle config message: {e}")

    def handle_robot_state(self, msg: RobotState):
        pos = Vector2(msg.current_pose.pose.position.x, msg.current_pose.pose.position.y)
        vel = Vector2(msg.current_velocity.linear.x, msg.current_velocity.linear.y)
        if msg.robot_id == self.robot_id:
            self.my_pos = pos
            self.my_vel = vel
        else:
            self.peer_states[msg.robot_id] = (pos, vel)

    def handle_nav2_cmd(self, msg: Twist):
        self.nav2_pref_vel = Vector2(msg.linear.x, msg.linear.y)

    def _request_approaching_aisle(self, segment_id: str):
        """Request each approach once; ReservationManager owns arbitration."""
        if segment_id in self._requested_aisles:
            return
        self._requested_aisles.add(segment_id)
        self._reservation_clock += 1
        request = Reservation()
        request.reservation_id = f'{self.robot_id}:{segment_id}:{self._reservation_clock}'
        request.robot_id = self.robot_id
        request.segment_id = segment_id
        request.state = Reservation.STATE_REQUESTED
        request.lamport_clock = self._reservation_clock
        request.priority = 1
        request.request_time = self.get_clock().now().to_msg()
        self.reservation_request_pub.publish(request)
        self._publish_traffic_event('reservation_requested', segment_id=segment_id)
        self.get_logger().info(
            f'[{self.robot_id}] [TRAFFIC] Requested reservation for {segment_id}.'
        )

    def _publish_traffic_event(self, event: str, **details):
        record = audit_event(self.get_logger(), 'conflict_resolver', event, self.robot_id, **details)
        message = String()
        message.data = json.dumps(record, separators=(',', ':'), default=str)
        self.traffic_event_pub.publish(message)

    def conflict_check(self):
        now = time.time()
        dt = max(0.01, min(0.5, now - self.last_step_time))
        self.last_step_time = now

        aisle_id = self.coordinator.is_in_single_lane_aisle(self.my_pos)
        approaching_aisle = aisle_id or self.coordinator.is_approaching_aisle(
            self.my_pos, self.my_pos + self.nav2_pref_vel * 2.0, threshold=0.8
        )
        if approaching_aisle:
            self._request_approaching_aisle(approaching_aisle)
        else:
            # A later approach may be a new traversal and needs a fresh grant.
            self._requested_aisles.clear()
        holder = self.active_reservations.get(approaching_aisle) if approaching_aisle else None

        override_vel, is_active, log_msg = self.coordinator.compute_override(
            my_pos=self.my_pos,
            my_vel=self.my_vel,
            nav2_pref_vel=self.nav2_pref_vel,
            peer_states=self.peer_states,
            active_reservation_holder=holder,
            dt=dt
        )

        selected_vel = override_vel if is_active and override_vel is not None else self.nav2_pref_vel
        twist_msg = Twist()
        twist_msg.linear.x = float(selected_vel.x)
        twist_msg.linear.y = float(selected_vel.y)
        twist_msg.angular.z = 0.0
        self.drive_pub.publish(twist_msg)
        if is_active and override_vel is not None:
            self.override_pub.publish(twist_msg)
            self.local_override_pub.publish(twist_msg)
            self.get_logger().info(log_msg)
            event = 'orca_avoidance' if 'Open-space conflict' in log_msg else 'pibt_wait'
            if event != self._last_traffic_state:
                self._publish_traffic_event(event, segment_id=approaching_aisle or '', detail=log_msg)
            self._last_traffic_state = event
        else:
            self._last_traffic_state = ''


def main(args=None):
    rclpy.init(args=args)
    node = ConflictResolver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
