#!/usr/bin/env python3
"""
Pickup Validator Node — Phase 9 Implementation
-----------------------------------------------
Runs identically on each robot.

Listens to /fleet/pickup_observations for PICKUP_OBSERVATION events where a
robot arrived at a pickup location but found the physical item absent.

Reconciliation logic:
  1. Query local WorldView for confirmed ownership of the task.
  2. If a peer robot is IN_PROGRESS, PICKUP_COMPLETED, or DELIVERING for the
     task → confirmed other owner. Respond with is_valid=True + confirmed_owner_id.
  3. If no confirmed owner is found → task state is stale. Respond with
     is_valid=False (reporter should re-bid the task).

The validator does NOT change task state itself; it only publishes a resolved
PickupObservation that the nav2_bridge on the reporting robot acts on.
"""

import rclpy
from rclpy.node import Node
from typing import Dict, Optional

from fleet_interfaces.msg import (
    PickupObservation,
    Task,
    WorldView,
)


# Task states that indicate a robot is actively executing the task
_ACTIVE_STATES = {
    Task.STATE_ASSIGNED,
    Task.STATE_IN_PROGRESS,
    Task.STATE_PICKUP_COMPLETED,
    Task.STATE_DELIVERING,
}


class PickupValidator(Node):
    def __init__(self):
        super().__init__('pickup_validator')

        default_id = self.get_namespace().strip('/') or 'robot_1'
        self.declare_parameter('robot_id', default_id)
        self.robot_id = self.get_parameter('robot_id').get_parameter_value().string_value

        # World view cache: task_id -> (state, assigned_robot_id)
        self._task_state_cache: Dict[str, tuple] = {}

        # ── Subscriptions ──────────────────────────────────────────────────
        self.world_view_sub = self.create_subscription(
            WorldView, 'world_view', self._handle_world_view, 10
        )
        self.pickup_obs_sub = self.create_subscription(
            PickupObservation, '/fleet/pickup_observations',
            self._handle_pickup_observation, 10
        )
        self.task_events_sub = self.create_subscription(
            Task, '/fleet/task_events', self._handle_task_event, 10
        )

        # ── Publishers ─────────────────────────────────────────────────────
        self.obs_response_pub = self.create_publisher(
            PickupObservation, '/fleet/pickup_observations', 10
        )

        self.get_logger().info(
            f'[{self.robot_id}] Pickup Validator online — monitoring /fleet/pickup_observations'
        )

    # ──────────────────────────────────────────────────────────────────────────
    # State Cache Handlers
    # ──────────────────────────────────────────────────────────────────────────

    def _handle_world_view(self, msg: WorldView):
        """Updates task state cache from the current world view."""
        for t in msg.task_states:
            self._task_state_cache[t.task_id] = (t.state, t.assigned_robot_id)

    def _handle_task_event(self, t: Task):
        """Updates task state cache from task events."""
        self._task_state_cache[t.task_id] = (t.state, t.assigned_robot_id)

    # ──────────────────────────────────────────────────────────────────────────
    # Reconciliation Logic
    # ──────────────────────────────────────────────────────────────────────────

    def _handle_pickup_observation(self, obs: PickupObservation):
        """
        Processes an incoming pickup observation where is_valid=False.
        Ignores:
          - Observations already resolved (is_valid=True)
          - Observations published by ourselves (we are the reporter, not the resolver)
          - Observations with a confirmed_owner_id already set (already resolved)
        """
        # Already resolved or not a failure report
        if obs.is_valid or obs.confirmed_owner_id:
            return

        # Don't process our own unresolved observation — let peers resolve it
        if obs.robot_id == self.robot_id:
            return

        task_id = obs.task_id
        self.get_logger().info(
            f'[{self.robot_id}] Pickup observation received: task={task_id} '
            f'from {obs.robot_id} (item absent). Running reconciliation...'
        )

        confirmed_owner = self._find_confirmed_owner(task_id, exclude_robot=obs.robot_id)

        response = PickupObservation()
        response.task_id = task_id
        response.robot_id = self.robot_id  # responder
        response.observed_pose = obs.observed_pose
        response.timestamp = self.get_clock().now().to_msg()

        if confirmed_owner:
            # Another robot is actively working this task
            response.is_valid = True
            response.confirmed_owner_id = confirmed_owner
            response.failure_reason = ''
            self.get_logger().info(
                f'[{self.robot_id}] ✓ Reconciliation: task {task_id} confirmed owned by '
                f'{confirmed_owner}. Notifying {obs.robot_id} to drop task.'
            )
        else:
            # No confirmed owner — task state is stale, re-bid is appropriate
            response.is_valid = False
            response.confirmed_owner_id = ''
            response.failure_reason = 'no_confirmed_owner'
            self.get_logger().warn(
                f'[{self.robot_id}] ✗ Reconciliation: no confirmed owner for task '
                f'{task_id}. Notifying {obs.robot_id} to re-bid.'
            )

        self.obs_response_pub.publish(response)

    def _find_confirmed_owner(self, task_id: str, exclude_robot: str) -> Optional[str]:
        """
        Checks the local task state cache for a robot that is actively executing task_id.
        Returns the confirmed owner robot_id, or None if not found.
        """
        cached = self._task_state_cache.get(task_id)
        if cached is None:
            return None

        state, assigned_robot = cached

        # Active state + assigned to a different robot = confirmed owner
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
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
