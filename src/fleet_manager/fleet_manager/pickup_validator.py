#!/usr/bin/env python3
"""
Pickup Validator Node — Phase 11 Implementation
-----------------------------------------------
Runs identically on each robot.

Listens to /fleet/pickup_observations and /tasks/pickup_observation
for PICKUP_OBSERVATION events where a robot arrived at a pickup location
but found the physical item absent.

Reconciliation logic:
  1. Deduplicate observations using unique obs_id.
  2. Query local WorldView / CRDT state cache for confirmed ownership of the task.
  3. If a peer robot is in PICKUP_COMPLETED, DELIVERING, or COMPLETED (or active state)
     for the task → confirmed other owner. Respond with is_valid=True + confirmed_owner_id.
  4. If no confirmed owner is found → task state is stale. Respond with
     is_valid=False and failure_reason='no_confirmed_owner' (reporter should re-bid the task).

The validator does NOT change task state directly; it publishes a resolved
PickupObservation that the reporting robot (and fleet) acts on.
"""

import json
from typing import Dict, Optional, Set

import rclpy
from rclpy.node import Node

from fleet_interfaces.msg import (
    P2PMessage,
    PickupObservation,
    Task,
    WorldView,
)
from fleet_manager.p2p_client import P2PClient


# Task states that indicate a robot is actively executing or completed the task
_ACTIVE_STATES = {
    Task.STATE_ASSIGNED,
    Task.STATE_IN_PROGRESS,
    Task.STATE_PICKUP_COMPLETED,
    Task.STATE_DELIVERING,
    Task.STATE_COMPLETED,
}


class PickupValidator(Node):
    def __init__(self):
        super().__init__('pickup_validator')

        default_id = self.get_namespace().strip('/') or 'robot_1'
        self.declare_parameter('robot_id', default_id)
        self.robot_id = self.get_parameter('robot_id').get_parameter_value().string_value

        # World view cache: task_id -> (state, assigned_robot_id)
        self._task_state_cache: Dict[str, tuple] = {}
        self._seen_obs_ids: Set[str] = set()
        self.lamport_clock: int = 0

        # P2P client
        try:
            self.p2p = P2PClient(node=self)
            self.p2p.register_handler('PICKUP_OBSERVATION', self._on_p2p_pickup_obs)
        except Exception as e:
            self.p2p = None
            self.get_logger().warn(f'[{self.robot_id}] [PICKUP_VAL] P2PClient init skipped: {e}')

        # ── Subscriptions ──────────────────────────────────────────────────
        self.world_view_sub = self.create_subscription(
            WorldView, 'world_view', self._handle_world_view, 10
        )
        self.pickup_obs_sub = self.create_subscription(
            PickupObservation, '/fleet/pickup_observations',
            self._handle_pickup_observation, 10
        )
        self.tasks_pickup_obs_sub = self.create_subscription(
            PickupObservation, '/tasks/pickup_observation',
            self._handle_pickup_observation, 10
        )
        self.task_events_sub = self.create_subscription(
            Task, '/fleet/task_events', self._handle_task_event, 10
        )

        # ── Publishers ─────────────────────────────────────────────────────
        self.obs_response_pub = self.create_publisher(
            PickupObservation, '/fleet/pickup_observations', 10
        )
        self.tasks_obs_response_pub = self.create_publisher(
            PickupObservation, '/tasks/pickup_observation', 10
        )

        self.get_logger().info(
            f'[{self.robot_id}] [PICKUP_VAL] Pickup Validator online — monitoring pickup observations'
        )

    # ──────────────────────────────────────────────────────────────────────────
    # State Cache Handlers
    # ──────────────────────────────────────────────────────────────────────────

    def _handle_world_view(self, msg: WorldView):
        """Updates task state cache from the current CRDT world view."""
        self.lamport_clock = max(self.lamport_clock, msg.lamport_clock)
        for t in msg.task_states:
            self._task_state_cache[t.task_id] = (t.state, t.assigned_robot_id)

    def _handle_task_event(self, t: Task):
        """Updates task state cache from task events."""
        self._task_state_cache[t.task_id] = (t.state, t.assigned_robot_id)

    def _on_p2p_pickup_obs(self, msg: P2PMessage):
        """Handles PICKUP_OBSERVATION incoming over P2P virtual network."""
        try:
            d = json.loads(msg.payload)
            obs = PickupObservation()
            obs.obs_id = d.get('obs_id', '')
            obs.robot_id = d.get('robot_id', msg.source_robot_id)
            obs.task_id = d.get('task_id', '')
            obs.is_valid = d.get('is_valid', False)
            obs.item_present = d.get('item_present', False)
            obs.failure_reason = d.get('failure_reason', '')
            obs.confirmed_owner_id = d.get('confirmed_owner_id', '')
            obs.lamport_clock = d.get('lamport_clock', 0)
            self._handle_pickup_observation(obs)
        except Exception as e:
            self.get_logger().warn(f'[{self.robot_id}] [PICKUP_VAL] Error parsing P2P PICKUP_OBSERVATION: {e}')

    # ──────────────────────────────────────────────────────────────────────────
    # Reconciliation Logic
    # ──────────────────────────────────────────────────────────────────────────

    def _handle_pickup_observation(self, obs: PickupObservation):
        """
        Processes an incoming pickup observation where is_valid=False (or item_present=False).
        Ignores:
          - Observations already seen (idempotency deduplication by obs_id)
          - Observations already resolved (is_valid=True)
          - Observations confirming item is present (item_present=True)
          - Observations published by ourselves
          - Observations with a confirmed_owner_id already set
        """
        if obs.obs_id:
            if obs.obs_id in self._seen_obs_ids:
                self.get_logger().debug(
                    f'[{self.robot_id}] [PICKUP_VAL] Duplicate observation {obs.obs_id} ignored (idempotent).'
                )
                return
            self._seen_obs_ids.add(obs.obs_id)

        # Already resolved or not a failure report
        if obs.is_valid or obs.item_present or obs.confirmed_owner_id:
            return

        # Don't process our own unresolved observation — let peers resolve it
        if obs.robot_id == self.robot_id:
            return

        task_id = obs.task_id
        self.get_logger().info(
            f'[{self.robot_id}] [PICKUP_VAL] Pickup observation received: task={task_id} '
            f'obs_id={obs.obs_id} from {obs.robot_id} (item absent). Running reconciliation...'
        )

        confirmed_owner = self._find_confirmed_owner(task_id, exclude_robot=obs.robot_id)

        self.lamport_clock += 1
        resp_obs_id = f"{self.robot_id}:{self.lamport_clock}"
        self._seen_obs_ids.add(resp_obs_id)

        response = PickupObservation()
        response.obs_id = resp_obs_id
        response.task_id = task_id
        response.robot_id = self.robot_id  # responder
        response.observed_pose = obs.observed_pose
        response.lamport_clock = int(self.lamport_clock)
        response.item_present = False
        response.timestamp = self.get_clock().now().to_msg()

        if confirmed_owner:
            # Another robot is actively working or completed this task
            response.is_valid = True
            response.confirmed_owner_id = confirmed_owner
            response.failure_reason = ''
            self.get_logger().info(
                f'[{self.robot_id}] [PICKUP_VAL] ✓ Reconciliation: task {task_id} confirmed owned by '
                f'{confirmed_owner}. Notifying {obs.robot_id} to drop task.'
            )
        else:
            # No confirmed owner — task state is stale, re-bid is appropriate
            response.is_valid = False
            response.confirmed_owner_id = ''
            response.failure_reason = 'no_confirmed_owner'
            self.get_logger().warn(
                f'[{self.robot_id}] [PICKUP_VAL] ✗ Reconciliation: no confirmed owner for task '
                f'{task_id}. Notifying {obs.robot_id} to re-bid.'
            )

        self.obs_response_pub.publish(response)
        self.tasks_obs_response_pub.publish(response)

        if self.p2p:
            self.p2p.broadcast('PICKUP_OBSERVATION', payload={
                'obs_id': response.obs_id,
                'robot_id': response.robot_id,
                'task_id': response.task_id,
                'is_valid': response.is_valid,
                'item_present': response.item_present,
                'failure_reason': response.failure_reason,
                'confirmed_owner_id': response.confirmed_owner_id,
                'lamport_clock': response.lamport_clock,
                'timestamp': self.get_clock().now().nanoseconds,
            })

    def _find_confirmed_owner(self, task_id: str, exclude_robot: str) -> Optional[str]:
        """
        Checks the local task state cache (CRDT-backed) for a robot that has
        physically confirmed or actively claimed task_id.
        Returns the confirmed owner robot_id, or None if not found.
        """
        cached = self._task_state_cache.get(task_id)
        if cached is None:
            return None

        state, assigned_robot = cached

        # Check CRDT confirmed physical states first
        if state in (Task.STATE_PICKUP_COMPLETED, Task.STATE_DELIVERING, Task.STATE_COMPLETED):
            if assigned_robot and assigned_robot != exclude_robot:
                return assigned_robot

        # Check active assignment states
        if state in _ACTIVE_STATES and assigned_robot and assigned_robot != exclude_robot:
            return assigned_robot

        return None


def main(args=None):
    rclpy.init(args=args)
    node = PickupValidator()
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
