#!/usr/bin/env python3
"""
Fleet State — Shared Distributed World View Library
-----------------------------------------------------
Pure-Python, no ROS dependencies. Implements:
  - Lamport logical clock
  - LWW (Last-Write-Wins) deterministic state merging for robots and tasks
  - Full world-view serialization / deserialization (JSON ↔ ROS messages)

Used by state_manager, task_manager, and future coordination modules.
"""

import json
import time
from dataclasses import dataclass, field, asdict
from typing import Dict, List, Optional


# ─────────────────────────────────────────────────────────────────────────────
# Data Entries
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class RobotEntry:
    """In-memory representation of a known robot's state."""
    robot_id: str
    # Pose components (flat, to avoid ROS msg dependency)
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    qx: float = 0.0
    qy: float = 0.0
    qz: float = 0.0
    qw: float = 1.0
    # Velocity
    vx: float = 0.0
    vy: float = 0.0
    # Status
    battery: float = 100.0
    status: int = 0          # RobotState.STATUS_*
    task_id: str = ''
    task_state: str = ''
    # Destination
    dest_x: float = 0.0
    dest_y: float = 0.0
    # Lamport metadata
    lamport_clock: int = 0
    wall_time: float = 0.0   # Unix timestamp
    source_robot_id: str = ''


@dataclass
class TaskEntry:
    """In-memory representation of a known task's state."""
    task_id: str
    state: int = 0           # Task.STATE_*
    assigned_robot_id: str = ''
    version: int = 0
    priority: int = 1
    pickup_x: float = 0.0
    pickup_y: float = 0.0
    dropoff_x: float = 0.0
    dropoff_y: float = 0.0
    # Lamport metadata
    lamport_clock: int = 0
    wall_time: float = 0.0
    source_robot_id: str = ''


# ─────────────────────────────────────────────────────────────────────────────
# Fleet State Store
# ─────────────────────────────────────────────────────────────────────────────

class FleetState:
    """
    Distributed World View Store with Lamport Clock and LWW Merging.

    LWW Rule:
      - On conflict (same robot_id or task_id):
          1. Keep the entry with higher lamport_clock.
          2. Tie-break: keep entry with lexicographically larger source_robot_id (deterministic).
    """

    def __init__(self, robot_id: str):
        self.robot_id = robot_id
        self._lamport: int = 0

        # Known robot states: robot_id -> RobotEntry
        self._robots: Dict[str, RobotEntry] = {}

        # Known task states: task_id -> TaskEntry
        self._tasks: Dict[str, TaskEntry] = {}

    # ── Lamport Clock ────────────────────────────────────────────────────────

    def tick(self) -> int:
        """Increment the Lamport clock for a local event. Returns new value."""
        self._lamport += 1
        return self._lamport

    def advance(self, received: int) -> int:
        """Advance clock upon receiving a message: L = max(L, received) + 1."""
        self._lamport = max(self._lamport, received) + 1
        return self._lamport

    @property
    def clock(self) -> int:
        return self._lamport

    # ── LWW Merge Helpers ────────────────────────────────────────────────────

    @staticmethod
    def _lww_wins(incoming_clock: int, incoming_source: str,
                  existing_clock: int, existing_source: str) -> bool:
        """Returns True if incoming entry wins over existing entry."""
        if incoming_clock > existing_clock:
            return True
        if incoming_clock == existing_clock:
            return incoming_source > existing_source  # deterministic tie-break
        return False

    # ── Robot State Merge ─────────────────────────────────────────────────────

    def merge_robot(self, entry: RobotEntry) -> bool:
        """
        Merge a received robot state entry (LWW).
        Returns True if the entry changed local state.
        """
        existing = self._robots.get(entry.robot_id)
        if existing is None or self._lww_wins(
            entry.lamport_clock, entry.source_robot_id,
            existing.lamport_clock, existing.source_robot_id
        ):
            self._robots[entry.robot_id] = entry
            self.advance(entry.lamport_clock)
            return True
        return False

    def update_own_robot(self, entry: RobotEntry) -> RobotEntry:
        """
        Update own robot state, automatically ticking clock.
        """
        self.tick()
        entry.lamport_clock = self._lamport
        entry.wall_time = time.time()
        entry.source_robot_id = self.robot_id
        self._robots[self.robot_id] = entry
        return entry

    # ── Task State Merge ──────────────────────────────────────────────────────

    def merge_task(self, entry: TaskEntry) -> bool:
        """
        Merge a received task state entry (LWW + monotonic state guard).

        Monotonicity rule: task lifecycle state is irreversible.
        A task at state N (e.g. IN_PROGRESS=2) must NEVER be overwritten
        by an incoming entry with state < N (e.g. AVAILABLE=0), regardless
        of Lamport clock. This prevents late-joining robots with stale world
        views from resetting already-progressed tasks.

        Returns True if the entry changed local state.
        """
        existing = self._tasks.get(entry.task_id)
        if existing is None:
            self._tasks[entry.task_id] = entry
            self.advance(entry.lamport_clock)
            return True

        # Monotonic state guard: only allow state transitions that move
        # forward in the lifecycle (higher state value wins unconditionally)
        if entry.state > existing.state:
            self._tasks[entry.task_id] = entry
            self.advance(entry.lamport_clock)
            return True

        # Same state level: fall back to standard LWW clock comparison
        if entry.state == existing.state and self._lww_wins(
            entry.lamport_clock, entry.source_robot_id,
            existing.lamport_clock, existing.source_robot_id
        ):
            self._tasks[entry.task_id] = entry
            self.advance(entry.lamport_clock)
            return True

        # Incoming state is lower than existing — silently discard
        return False

    def update_own_task(self, entry: TaskEntry) -> TaskEntry:
        """
        Update a task state where this robot is the authority (e.g. executor).
        Automatically ticks clock.
        """
        self.tick()
        entry.lamport_clock = self._lamport
        entry.wall_time = time.time()
        entry.source_robot_id = self.robot_id
        self._tasks[entry.task_id] = entry
        return entry

    # ── World View Bulk Merge ─────────────────────────────────────────────────

    def merge_world_view(self, robots: List[RobotEntry], tasks: List[TaskEntry]) -> dict:
        """
        Merge a bulk world-view snapshot (used during reconnection).
        Returns a summary dict of changes made.
        """
        robot_updates = 0
        task_updates = 0
        for r in robots:
            if r.robot_id != self.robot_id:  # Never overwrite own state from peer
                if self.merge_robot(r):
                    robot_updates += 1
        for t in tasks:
            if self.merge_task(t):
                task_updates += 1
        return {'robot_updates': robot_updates, 'task_updates': task_updates}

    # ── Getters ───────────────────────────────────────────────────────────────

    def get_robot(self, robot_id: str) -> Optional[RobotEntry]:
        return self._robots.get(robot_id)

    def get_own_state(self) -> Optional[RobotEntry]:
        return self._robots.get(self.robot_id)

    def get_all_robots(self) -> List[RobotEntry]:
        return list(self._robots.values())

    def get_peer_robots(self) -> List[RobotEntry]:
        return [r for rid, r in self._robots.items() if rid != self.robot_id]

    def get_task(self, task_id: str) -> Optional[TaskEntry]:
        return self._tasks.get(task_id)

    def get_all_tasks(self) -> List[TaskEntry]:
        return list(self._tasks.values())

    def known_robot_ids(self) -> List[str]:
        return list(self._robots.keys())

    def known_task_ids(self) -> List[str]:
        return list(self._tasks.keys())

    # ── Serialization (JSON ↔ dict for P2P payload) ───────────────────────────

    def robot_entry_to_dict(self, e: RobotEntry) -> dict:
        return asdict(e)

    def task_entry_to_dict(self, e: TaskEntry) -> dict:
        return asdict(e)

    def robot_entry_from_dict(self, d: dict) -> RobotEntry:
        return RobotEntry(**{k: v for k, v in d.items() if k in RobotEntry.__dataclass_fields__})

    def task_entry_from_dict(self, d: dict) -> TaskEntry:
        return TaskEntry(**{k: v for k, v in d.items() if k in TaskEntry.__dataclass_fields__})

    def world_view_to_dict(self) -> dict:
        return {
            'robot_id': self.robot_id,
            'lamport_clock': self._lamport,
            'robots': [self.robot_entry_to_dict(r) for r in self.get_all_robots()],
            'tasks': [self.task_entry_to_dict(t) for t in self.get_all_tasks()],
        }

    def world_view_from_dict(self, d: dict) -> tuple:
        robots = [self.robot_entry_from_dict(r) for r in d.get('robots', [])]
        tasks = [self.task_entry_from_dict(t) for t in d.get('tasks', [])]
        return robots, tasks, d.get('lamport_clock', 0)

    # ── ROS Message Converters ────────────────────────────────────────────────

    def robot_entry_from_ros(self, ros_entry) -> RobotEntry:
        """Convert fleet_interfaces/msg/RobotWorldEntry to RobotEntry."""
        p = ros_entry.current_pose.pose
        v = ros_entry.current_velocity
        d = ros_entry.destination.pose
        return RobotEntry(
            robot_id=ros_entry.robot_id,
            x=p.position.x, y=p.position.y, z=p.position.z,
            qx=p.orientation.x, qy=p.orientation.y,
            qz=p.orientation.z, qw=p.orientation.w,
            vx=v.linear.x, vy=v.linear.y,
            battery=float(ros_entry.battery_level),
            status=ros_entry.status,
            task_id=ros_entry.current_task_id,
            task_state=ros_entry.current_task_state,
            dest_x=d.position.x, dest_y=d.position.y,
            lamport_clock=ros_entry.lamport_clock,
            wall_time=ros_entry.wall_time.sec + ros_entry.wall_time.nanosec * 1e-9,
            source_robot_id=ros_entry.source_robot_id,
        )

    def task_entry_from_task_ros(self, ros_task) -> TaskEntry:
        """Convert fleet_interfaces/msg/Task to TaskEntry."""
        return TaskEntry(
            task_id=ros_task.task_id,
            state=ros_task.state,
            assigned_robot_id=ros_task.assigned_robot_id,
            version=ros_task.version,
            priority=ros_task.priority,
            pickup_x=ros_task.pickup_pose.pose.position.x,
            pickup_y=ros_task.pickup_pose.pose.position.y,
            dropoff_x=ros_task.dropoff_pose.pose.position.x,
            dropoff_y=ros_task.dropoff_pose.pose.position.y,
            lamport_clock=0,
            wall_time=time.time(),
            source_robot_id='task_broadcaster',
        )

    def __repr__(self):
        return (
            f'FleetState(robot_id={self.robot_id!r}, clock={self._lamport}, '
            f'robots={list(self._robots.keys())}, tasks={list(self._tasks.keys())})'
        )
