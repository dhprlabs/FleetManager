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
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from geometry_msgs.msg import Twist, PoseStamped, Point
    from nav_msgs.msg import Path
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
    from fleet_manager.reservation_manager import DEFAULT_AISLE_SEGMENTS, ReservationManager
except ImportError:
    ReservationManager = None
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
    def __init__(self, time_horizon: float = 5.0, robot_radius: float = 0.35,
                 safety_margin: float = 0.25, max_speed: float = 0.6):
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
            # Check if current result satisfies constraint (with larger cushion for safety)
            if (result - plane.point).dot(plane.normal) < 0.05:
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
        emergency_stop = False

        for n_pos, n_vel in neighbors:
            rel_pos = n_pos - robot_pos
            rel_dist = rel_pos.length()
            if rel_dist > 6.0:
                continue

            # ── Emergency stop: within physical collision distance ─────────────
            # Triggered when robots are within combined_radius*1.1 regardless of
            # velocity direction.  This catches the same-direction trailing case
            # that standard ORCA cannot resolve (zero relative velocity → zero u).
            if rel_dist < self.combined_radius * 1.1:
                emergency_stop = True
                conflict_detected = True
                continue

            # ── Same-direction proximity bias ─────────────────────────────────
            # When two robots move in the same direction at similar speed,
            # the relative velocity ≈ 0, so ORCA computes zero avoidance.
            # Inject a right-lateral bias into the preferred velocity so that
            # the ORCA constraint has something to work against.
            pref_len = pref_vel.length()
            rel_vel_direct = robot_vel - n_vel
            same_dir_closing = (rel_dist < self.combined_radius * 3.0
                                 and rel_vel_direct.length() < 0.08
                                 and pref_len > 0.02)
            if same_dir_closing:
                # Perpendicular-right of current motion (port-starboard rule)
                perp = Vector2(-pref_vel.y, pref_vel.x).normalized()
                bias_strength = max(0.2, pref_len * 0.6)
                pref_vel = pref_vel + perp * bias_strength
                if pref_vel.length() > self.max_speed:
                    pref_vel = pref_vel.normalized() * self.max_speed
                conflict_detected = True

            # Check if moving towards each other
            rel_vel = robot_vel - n_vel
            is_closing = rel_pos.dot(rel_vel) > 0.0 or rel_dist < (self.combined_radius * 2.0)

            result = self.compute_orca_halfplane(robot_pos, robot_vel, n_pos, n_vel)
            if result is not None:
                hp, u = result
                # Active conflict if closing and correction vector u is non-zero
                if is_closing and (u.length() > 0.03 or rel_dist < (self.combined_radius * 2.0)):
                    conflict_detected = True
                planes.append(hp)

        # Emergency stop overrides all: return zero velocity immediately
        if emergency_stop:
            return Vector2(0.0, 0.0), True

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
        self.orca = ORCAEngine(time_horizon=5.0, robot_radius=0.35, safety_margin=0.25, max_speed=0.6)
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
                target_dx = max(spec['x_min'] - target.x, 0.0, target.x - spec['x_max'])
                target_dy = max(spec['y_min'] - target.y, 0.0, target.y - spec['y_max'])
                target_dist = math.hypot(target_dx, target_dy)
                # Only consider approaching if the robot is actually moving CLOSER to the aisle
                # (ignores robots that are moving away, moving parallel, or stationary/turning)
                if target_dist >= dist - 0.05:
                    continue
                return seg_id
        return None

    def compute_override(self,
                         my_pos: Vector2,
                         my_vel: Vector2,
                         nav2_pref_vel: Vector2,
                         peer_states: Dict[str, Tuple[Vector2, Vector2]],
                         active_reservation_holder: Optional[str] = None,
                         dt: float = 0.1,
                         respect_reservations: bool = True) -> Tuple[Optional[Vector2], bool, str]:
        """
        Main decision loop for physical conflict resolution.
        Returns:
          (override_vel, is_override_active, explanation_log)

        respect_reservations=False bypasses all aisle logic (used by the node
        when the robot is merely passing near an aisle its plan never enters);
        open-space ORCA then handles collision avoidance instead.
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

        if respect_reservations and approaching_aisle:
            # RULE 6: ORCA is EXPLICITLY DISABLED inside/near single-lane aisles.
            # Coordination relies strictly on Reservation Manager + PIBT.
            holder = active_reservation_holder
            inside_aisle = aisle_id is not None

            if holder == self.robot_id:
                self.is_waiting_at_choke = False
                my_agent.reset_waiting()
                return None, False, f"[{self.robot_id}] Aisle '{approaching_aisle}' reserved by self. ORCA disabled. Nav2 executes uninterrupted."

            elif inside_aisle:
                if holder is not None and holder != self.robot_id:
                    # An actual other robot holds this aisle — stop to avoid collision
                    self.is_waiting_at_choke = True
                    self.current_choke_segment = aisle_id
                    my_agent.record_waiting(dt)
                    override_vel = Vector2(0.0, 0.0)
                    return override_vel, True, (f"[{self.robot_id}] Inside aisle '{aisle_id}' held by {holder} — "
                                                f"STOP to avoid collision. PIBT wait active.")
                else:
                    # Inside aisle with no recorded holder: allow robot to proceed to exit the aisle cleanly
                    self.is_waiting_at_choke = False
                    my_agent.reset_waiting()
                    return None, False, f"[{self.robot_id}] Inside aisle '{aisle_id}' (no conflicting holder). Proceeding to exit."

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
                # No known holder: never enter a single-lane aisle before our
                # reservation grant arrives over /fleet/reservations.  The
                # ReservationManager is the single arbitration authority — a
                # local PIBT guess cannot replace a confirmed grant.
                self.is_waiting_at_choke = True
                self.current_choke_segment = approaching_aisle
                my_agent.record_waiting(dt)
                override_vel = Vector2(0.0, 0.0)
                return override_vel, True, (f"[{self.robot_id}] Awaiting reservation grant for aisle "
                                            f"'{approaching_aisle}' (no holder recorded yet). "
                                            f"ORCA disabled. Velocity override: STOP.")

        # ─────────────────────────────────────────────────────────────────────
        # Step 2: Open Space — Nav2 Local Planner (DWB + Costmap) handles dynamic avoidance
        # ─────────────────────────────────────────────────────────────────────
        # In open warehouse space, Nav2's local planner uses LiDAR (VoxelLayer) and
        # DWB BaseObstacle critic to steer differential-drive robots around each other
        # smoothly without artificial sideways velocity commands.
        my_agent.reset_waiting()
        return None, False, f"[{self.robot_id}] Open space clear of choke points. Nav2 DWB local planner active."


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

        # Begin with exactly the layout used by ReservationManager.  Dynamic
        # /fleet/aisle_config updates can still replace this, but startup must
        # not depend on receiving a configuration message after subscribing.
        self.declare_parameter(
            'aisle_config_file',
            '/home/mangal-devanshu/sih_ws/FleetManager/src/fleet_manager/config/aisle_segments.json'
        )
        aisle_config_file = self.get_parameter(
            'aisle_config_file'
        ).get_parameter_value().string_value
        aisle_segments = (
            ReservationManager.load_aisle_specs(aisle_config_file)
            if ReservationManager is not None else dict(DEFAULT_AISLE_SEGMENTS)
        )
        self.coordinator = ConflictCoordinator(
            robot_id=self.robot_id,
            aisle_segments=aisle_segments,
            aging_rate=aging_rate,
        )

        self.peer_states: Dict[str, Tuple[Vector2, Vector2]] = {}
        self.my_pos = Vector2(0.0, 0.0)
        self.my_vel = Vector2(0.0, 0.0)
        self.nav2_pref_vel = Vector2(0.0, 0.0)
        self.nav2_pref_angular_z: float = 0.0
        self.active_reservations: Dict[str, str] = {}
        self._requested_aisles: Set[str] = set()
        self._reservation_clock = 0
        self._last_reservation_renewal: Dict[str, float] = {}
        self._aisle_release_times: Dict[str, float] = {}
        self.declare_parameter('reservation_renewal_sec', 0.5)
        self.reservation_renewal_sec = self.get_parameter(
            'reservation_renewal_sec').get_parameter_value().double_value
        self._last_traffic_state = ''
        # Guard: do not attempt aisle reservations until at least one Nav2
        # plan has been received.  Without this, proximity checks fire on
        # startup / between goals and create phantom reservations.
        self._plan_received: bool = False

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
        # ReservationManager publishes the authoritative aisle map once at
        # startup, before this node is normally constructed.  This must be a
        # transient-local subscription or the resolver retains its stale
        # fallback layout and never detects the configured aisle on approach.
        aisle_config_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.aisle_config_sub = self.create_subscription(
            String, '/fleet/aisle_config', self.handle_aisle_config,
            aisle_config_qos
        )
        # Fix 2: Subscribe to this robot's Nav2 global plan to do path-based
        # aisle reservation instead of proximity-only.
        self._nav_plan_waypoints: List[Tuple[float, float]] = []
        if _ROS_AVAILABLE:
            self.plan_sub = self.create_subscription(
                Path, f'/{self.robot_id}/plan', self.handle_plan, 5
            )

        # Nav2 is remapped to cmd_vel_nav.  This node is the command mux: it
        # forwards Nav2 while clear and substitutes ORCA/PIBT commands only
        # when physical coordination requires it.
        self.drive_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.reservation_request_pub = self.create_publisher(Reservation, '/traffic/reserve', 10)
        self.fleet_res_pub = self.create_publisher(Reservation, '/fleet/reservations', 10)
        self.reservation_release_pub = self.create_publisher(Reservation, '/traffic/release', 10)
        self.fleet_release_pub = self.create_publisher(Reservation, '/fleet/reservations', 10)
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
            raw_aisles = payload.get('aisles', payload)
            new_segments = {}

            if isinstance(raw_aisles, dict):
                # Dict format: {segment_id: {x_min, x_max, y_min, y_max, ...}}
                for seg_id, spec in raw_aisles.items():
                    new_segments[seg_id] = {
                        'segment_id': seg_id,
                        'x_min': float(spec.get('x_min', 0.0)),
                        'x_max': float(spec.get('x_max', 0.0)),
                        'y_min': float(spec.get('y_min', 0.0)),
                        'y_max': float(spec.get('y_max', 0.0)),
                        'is_single_lane': bool(spec.get('is_single_lane', True)),
                    }
            elif isinstance(raw_aisles, list):
                # List format: [{segment_id, x_min, x_max, y_min, y_max, ...}, ...]
                for item in raw_aisles:
                    seg_id = item.get('segment_id') or item.get('id')
                    if not seg_id:
                        continue
                    new_segments[seg_id] = {
                        'segment_id': seg_id,
                        'x_min': float(item.get('x_min', 0.0)),
                        'x_max': float(item.get('x_max', 0.0)),
                        'y_min': float(item.get('y_min', 0.0)),
                        'y_max': float(item.get('y_max', 0.0)),
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
            if msg.status == RobotState.STATUS_IDLE and not msg.current_task_id:
                self._nav_plan_waypoints.clear()
                self._plan_received = False
        else:
            self.peer_states[msg.robot_id] = (pos, vel)

    def handle_nav2_cmd(self, msg: Twist):
        self.nav2_pref_vel = Vector2(msg.linear.x, msg.linear.y)
        self.nav2_pref_angular_z = float(msg.angular.z)

    def handle_plan(self, msg: 'Path'):
        """Cache a Nav2 route and reserve its first single-lane aisle early."""
        self._nav_plan_waypoints = [
            (p.pose.position.x, p.pose.position.y)
            for p in (msg.poses or [])
        ]
        if self._nav_plan_waypoints:
            self._plan_received = True

            # Evict any previously-requested aisles the new route no longer passes
            # through.  This covers the pickup→delivery transition: the delivery
            # route may not need the same aisle as the pickup route, so we release
            # the stale reservation request so the robot isn't stuck waiting for a
            # grant it no longer needs.
            #
            # IMPORTANT: if the robot is currently INSIDE the aisle (aisle_id is
            # set), do NOT release the reservation even if the new plan omits it.
            # The robot still needs the reservation to physically exit; the
            # auto-release in conflict_check will fire once it is clear.
            current_aisle = self.coordinator.is_in_single_lane_aisle(self.my_pos, margin=0.1)
            for seg_id in list(self._requested_aisles):
                if not self._plan_passes_through_aisle(seg_id):
                    if current_aisle == seg_id:
                        # Still physically inside — keep the reservation.
                        self.get_logger().info(
                            f'[{self.robot_id}] [PLAN] New route no longer uses {seg_id} '
                            f'but robot is still inside — holding reservation until exit.'
                        )
                        continue
                    self.get_logger().info(
                        f'[{self.robot_id}] [PLAN] New route no longer uses {seg_id} — '
                        f'clearing stale reservation request.'
                    )
                    self._requested_aisles.discard(seg_id)
                    # Release the held reservation only if we are clear of the aisle.
                    if self.active_reservations.get(seg_id) == self.robot_id:
                        self._release_aisle_reservation(seg_id)

            # A route, rather than just proximity to an entrance, tells us
            # which choke point the robot is committed to using.  Claim the
            # first aisle on that route now; later aisles remain available
            # until the robot approaches them, avoiding speculative locks.
            for x, y in self._nav_plan_waypoints:
                segment_id = self.coordinator.get_aisle_at_point(x, y)
                if segment_id:
                    self._request_approaching_aisle(segment_id)
                    break


    def _plan_passes_through_aisle(self, seg_id: str, inflation: float = 0.0) -> bool:
        """
        Returns True if any upcoming waypoint in the cached Nav2 plan falls inside the
        given aisle segment.
        Returns False when no plan has been received yet or when remaining path does
        not enter the aisle.
        """
        if not self._plan_received or not self._nav_plan_waypoints:
            # No plan available yet — refuse reservation to avoid ghost bookings.
            return False
        spec = self.coordinator.aisle_segments.get(seg_id)
        if spec is None:
            return True
        x_min = spec['x_min'] - inflation
        x_max = spec['x_max'] + inflation
        y_min = spec['y_min'] - inflation
        y_max = spec['y_max'] + inflation

        # Find index of closest waypoint to current position to only check remaining path
        closest_idx = 0
        min_dist_sq = float('inf')
        for i, (wx, wy) in enumerate(self._nav_plan_waypoints):
            d2 = (wx - self.my_pos.x) ** 2 + (wy - self.my_pos.y) ** 2
            if d2 < min_dist_sq:
                min_dist_sq = d2
                closest_idx = i

        for (wx, wy) in self._nav_plan_waypoints[closest_idx:]:
            if x_min <= wx <= x_max and y_min <= wy <= y_max:
                return True
        return False

    def _request_approaching_aisle(self, segment_id: str):
        """Request each approach once; ReservationManager owns arbitration."""
        if segment_id in self._requested_aisles:
            return
        # If this robot recently released this aisle (within 2s), don't immediately re-request
        # unless its upcoming plan clearly passes through the aisle
        if time.time() - self._aisle_release_times.get(segment_id, 0.0) < 2.0:
            if not self._plan_passes_through_aisle(segment_id):
                return
        # Gate on the Nav2 plan once one exists.  Before the first plan
        # arrives we still request so the grant can land before the robot
        # reaches the aisle (the robot waits outside until granted anyway).
        if self._plan_received and not self._plan_passes_through_aisle(segment_id):
            self.get_logger().debug(
                f'[{self.robot_id}] [TRAFFIC] Skipping reservation for {segment_id}: '
                f'Nav2 plan does not pass through this aisle.'
            )
            return
        self._requested_aisles.add(segment_id)
        self._reservation_clock += 1
        spec = self.coordinator.aisle_segments.get(segment_id, {})
        x_min = spec.get('x_min', 0.0)
        x_max = spec.get('x_max', 0.0)
        y_min = spec.get('y_min', 0.0)
        y_max = spec.get('y_max', 0.0)
        request = Reservation()
        request.reservation_id = f'{self.robot_id}:{segment_id}:{self._reservation_clock}'
        request.robot_id = self.robot_id
        request.segment_id = segment_id
        request.task_id = ''  # filled by nav2_bridge when task starts
        request.state = Reservation.STATE_REQUESTED
        request.lamport_clock = self._reservation_clock
        request.priority = 1
        request.request_time = self.get_clock().now().to_msg()
        request.start_time = self.get_clock().now().to_msg()
        request.end_time = self.get_clock().now().to_msg()
        request.expiry_time = self.get_clock().now().to_msg()
        request.zone_min.x = float(x_min)
        request.zone_min.y = float(y_min)
        request.zone_min.z = 0.0
        request.zone_max.x = float(x_max)
        request.zone_max.y = float(y_max)
        request.zone_max.z = 0.0
        self.reservation_request_pub.publish(request)
        self.fleet_res_pub.publish(request)
        self._publish_traffic_event('reservation_requested', segment_id=segment_id)
        self.get_logger().info(
            f'[{self.robot_id}] [TRAFFIC] Requested reservation for {segment_id}.'
        )

    def _release_aisle_reservation(self, segment_id: str):
        """Release reservation for an aisle segment."""
        self._reservation_clock += 1
        spec = self.coordinator.aisle_segments.get(segment_id, {})
        x_min = spec.get('x_min', 0.0)
        x_max = spec.get('x_max', 0.0)
        y_min = spec.get('y_min', 0.0)
        y_max = spec.get('y_max', 0.0)
        release = Reservation()
        release.reservation_id = f'{self.robot_id}:{segment_id}:release:{self._reservation_clock}'
        release.robot_id = self.robot_id
        release.segment_id = segment_id
        release.state = Reservation.STATE_RELEASED
        release.lamport_clock = self._reservation_clock
        release.zone_min.x = float(x_min)
        release.zone_min.y = float(y_min)
        release.zone_min.z = 0.0
        release.zone_max.x = float(x_max)
        release.zone_max.y = float(y_max)
        release.zone_max.z = 0.0
        self.reservation_release_pub.publish(release)
        self.fleet_release_pub.publish(release)
        self._publish_traffic_event('reservation_released', segment_id=segment_id)
        self.get_logger().info(
            f'[{self.robot_id}] [TRAFFIC] Released reservation for {segment_id}.'
        )
        # Clear from requested set so it can be re-requested if needed
        self._requested_aisles.discard(segment_id)
        self._last_reservation_renewal.pop(segment_id, None)
        self._aisle_release_times[segment_id] = time.time()

    def _renew_aisle_reservation(self, segment_id: str, now: float):
        """Keep a granted aisle lease alive while the holder is traversing it.

        Uses time.monotonic() for the renewal interval to match the watchdog's
        time base in the reservation manager.
        """
        if self.active_reservations.get(segment_id) != self.robot_id:
            return
        # Use monotonic time for renewal interval to match watchdog time base
        mono_now = time.monotonic()
        if mono_now - self._last_reservation_renewal.get(segment_id, 0.0) < self.reservation_renewal_sec:
            return

        self._reservation_clock += 1
        renewal = Reservation()
        renewal.reservation_id = f'{self.robot_id}:{segment_id}:renew:{self._reservation_clock}'
        renewal.robot_id = self.robot_id
        renewal.segment_id = segment_id
        renewal.state = Reservation.STATE_ACTIVE
        renewal.lamport_clock = self._reservation_clock
        renewal.priority = 1
        renewal.request_time = self.get_clock().now().to_msg()
        self.reservation_request_pub.publish(renewal)
        self.fleet_res_pub.publish(renewal)
        self._last_reservation_renewal[segment_id] = mono_now
        self._publish_traffic_event('reservation_renewed', segment_id=segment_id)

    def _publish_traffic_event(self, event: str, **details):
        record = audit_event(self.get_logger(), 'conflict_resolver', event, self.robot_id, **details)
        message = String()
        message.data = json.dumps(record, separators=(',', ':'), default=str)
        self.traffic_event_pub.publish(message)

    def conflict_check(self):
        now = time.time()
        dt = max(0.01, min(0.5, now - self.last_step_time))
        self.last_step_time = now

        # ─────────────────────────────────────────────────────────────────────
        # SESSION LOGGING: Track robot position, aisle state, and velocities
        # ─────────────────────────────────────────────────────────────────────
        aisle_id = self.coordinator.is_in_single_lane_aisle(self.my_pos, margin=0.1)
        approaching_aisle = aisle_id or self.coordinator.is_approaching_aisle(
            self.my_pos, self.my_pos + self.nav2_pref_vel * 2.0, threshold=0.8
        )

        # If outside the aisle and recently released it, ignore spurious approach detection
        # unless our plan genuinely enters deep into the aisle.
        if aisle_id is None and approaching_aisle is not None:
            recently_released = (now - self._aisle_release_times.get(approaching_aisle, 0.0)) < 5.0
            if recently_released and not self._plan_passes_through_aisle(approaching_aisle, inflation=0.0):
                approaching_aisle = None

        # Plan-awareness: does our Nav2 plan actually enter the detected aisle?
        # A robot merely passing NEAR an aisle its plan never enters must
        # neither reserve it nor stop because another robot holds it.
        plan_enters_aisle = (
            self._plan_passes_through_aisle(approaching_aisle, inflation=0.0)
            if approaching_aisle else False
        )
        # Respect reservation logic when: inside the aisle, or the plan enters it.
        # A robot outside the aisle without a plan entering it NEVER stops for reservations.
        respect_reservations = (
            approaching_aisle is not None
            and (aisle_id is not None or plan_enters_aisle)
        )

        if aisle_id:
            if self.active_reservations.get(aisle_id) == self.robot_id:
                self._renew_aisle_reservation(aisle_id, now)
            else:
                self._request_approaching_aisle(aisle_id)


        
        # Log position and aisle state changes
        if not hasattr(self, '_last_aisle_state'):
            self._last_aisle_state = None
            self._last_position_log = 0
            self._last_in_aisle = None
        
        # Log position every 2 seconds or on aisle state change
        position_changed = (now - self._last_position_log) > 2.0
        aisle_state_changed = approaching_aisle != self._last_aisle_state
        
        # Auto-release reservation when exiting an aisle
        if self._last_in_aisle is not None and aisle_id != self._last_in_aisle:
            if self.active_reservations.get(self._last_in_aisle) == self.robot_id:
                self._release_aisle_reservation(self._last_in_aisle)
        
        if position_changed or aisle_state_changed:
            self._last_position_log = now
            self.get_logger().info(
                f'[{self.robot_id}] [SESSION] pos=({self.my_pos.x:.2f},{self.my_pos.y:.2f}) '
                f'vel=({self.my_vel.x:.2f},{self.my_vel.y:.2f}) '
                f'pref_vel=({self.nav2_pref_vel.x:.2f},{self.nav2_pref_vel.y:.2f}) '
                f'in_aisle={aisle_id} approaching={approaching_aisle} '
                f'active_reservations={dict(self.active_reservations)}'
            )
            if aisle_state_changed:
                self._publish_traffic_event('aisle_state_change',
                    segment_id=approaching_aisle or '',
                    in_aisle=aisle_id,
                    approaching=approaching_aisle,
                    pos_x=round(self.my_pos.x, 2),
                    pos_y=round(self.my_pos.y, 2)
                )
            self._last_aisle_state = approaching_aisle
        
        self._last_in_aisle = aisle_id

        # Skip conflict resolution when the robot is genuinely stationary AND
        # not near any single-lane aisle: both Nav2 command and observed
        # velocity below noise floor, with no aisle in play. This prevents
        # ORCA from activating while robots idle at the dock.
        #
        # Critically, this must NOT trigger while approaching/inside an
        # aisle: a robot waiting for a reservation grant, or holding for PIBT,
        # is *expected* to sit at velocity 0 — that used to make this branch
        # fire every cycle once it settled, which returned before
        # _request_approaching_aisle() (so a reservation might never even be
        # requested) and before compute_override() (so the code never
        # rechecked whether a grant had since arrived and it was safe to
        # move). The robot would then freeze permanently at the first
        # chokepoint it ever stopped at, regardless of later reservation
        # state — exactly the case where two robots end up physically
        # sharing an aisle with no active_reservations recorded for either.
        if approaching_aisle is None and self.nav2_pref_vel.length() < 0.02 and self.my_vel.length() < 0.02:
            if self._last_traffic_state:
                self.get_logger().info(
                    f'[{self.robot_id}] [SESSION] Robot stationary — conflict resolution paused.'
                )
                self._last_traffic_state = ''
            # Still forward the zero command so the robot does not drift
            twist_msg = Twist()
            self.drive_pub.publish(twist_msg)
            return

        if respect_reservations:
            self._request_approaching_aisle(approaching_aisle)
        else:
            # A later approach may be a new traversal and needs a fresh grant.
            self._requested_aisles.clear()
        holder = (self.active_reservations.get(approaching_aisle)
                  if respect_reservations and approaching_aisle else None)

        override_vel, is_active, log_msg = self.coordinator.compute_override(
            my_pos=self.my_pos,
            my_vel=self.my_vel,
            nav2_pref_vel=self.nav2_pref_vel,
            peer_states=self.peer_states,
            active_reservation_holder=holder,
            dt=dt,
            respect_reservations=respect_reservations
        )

        selected_vel = override_vel if is_active and override_vel is not None else self.nav2_pref_vel
        twist_msg = Twist()
        twist_msg.linear.x = float(selected_vel.x)
        # Differential drive robots have zero lateral velocity
        twist_msg.linear.y = 0.0
        # Forward Nav2 angular velocity unless override requires a full stop (0.0)
        twist_msg.angular.z = 0.0 if (is_active and override_vel is not None) else self.nav2_pref_angular_z
        self.drive_pub.publish(twist_msg)
        if is_active and override_vel is not None:
            self.override_pub.publish(twist_msg)
            self.local_override_pub.publish(twist_msg)
            event = 'orca_avoidance' if 'Open-space conflict' in log_msg else 'pibt_wait'
            if event != self._last_traffic_state:
                self.get_logger().info(log_msg)
                self._publish_traffic_event(event, segment_id=approaching_aisle or '', detail=log_msg)
            else:
                self.get_logger().debug(log_msg)
            self._last_traffic_state = event
        else:
            if self._last_traffic_state:
                self.get_logger().info(f"[{self.robot_id}] Physical conflict resolved. Resuming normal Nav2 operation.")
                self._publish_traffic_event('conflict_resolved', segment_id=approaching_aisle or '', detail='clear')
            self._last_traffic_state = ''


def main(args=None):
    rclpy.init(args=args)
    node = ConflictResolver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == '__main__':
    main()
