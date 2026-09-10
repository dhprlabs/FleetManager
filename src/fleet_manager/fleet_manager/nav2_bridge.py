#!/usr/bin/env python3
"""
Nav2 Bridge Node
----------------
Runs identically on each robot.
Translates allocated task waypoints into Nav2 action goals (NavigateToPose)
and coordinates execution between task pickup and delivery locations.
"""

import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

from fleet_interfaces.msg import Task


class Nav2Bridge(Node):
    def __init__(self):
        super().__init__('nav2_bridge')

        default_id = self.get_namespace().strip('/') or 'robot_1'
        self.declare_parameter('robot_id', default_id)
        self.declare_parameter('enable_nav2', True)

        self.robot_id = self.get_parameter('robot_id').get_parameter_value().string_value
        self.enable_nav2 = self.get_parameter('enable_nav2').get_parameter_value().bool_value

        # Nav2 Action Client scoped to this robot's namespace
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.current_goal_handle = None
        self.active_task: Optional[Task] = None
        self.task_phase = 'IDLE'  # 'IDLE', 'NAV_TO_PICKUP', 'AT_PICKUP', 'NAV_TO_DROPOFF', 'COMPLETED'

        # Subscriptions
        self.assigned_task_sub = self.create_subscription(
            Task, 'assigned_task', self.handle_assigned_task, 10
        )

        self.get_logger().info(
            f'[{self.robot_id}] Nav2 Bridge proxy initialized. Action server target: /{self.robot_id}/navigate_to_pose'
        )

    def handle_assigned_task(self, task: Task):
        """Called when TaskManager assigns a task to this robot."""
        if self.active_task and self.active_task.task_id == task.task_id:
            return  # Already executing

        self.active_task = task
        p_x = task.pickup_pose.pose.position.x
        p_y = task.pickup_pose.pose.position.y
        d_x = task.dropoff_pose.pose.position.x
        d_y = task.dropoff_pose.pose.position.y

        self.get_logger().info(
            f'[{self.robot_id}] ★ Nav2 Bridge received assigned task {task.task_id}: '
            f'Pickup=({p_x:.1f}, {p_y:.1f}) ──► Dropoff=({d_x:.1f}, {d_y:.1f})'
        )

        self._start_task_execution(task)

    def _start_task_execution(self, task: Task):
        """Dispatches navigation goal to pickup location."""
        self.task_phase = 'NAV_TO_PICKUP'
        self.get_logger().info(f'[{self.robot_id}] Navigating to PICKUP location for {task.task_id}...')
        self._send_nav_goal(task.pickup_pose, on_success=self._on_pickup_reached)

    def _on_pickup_reached(self):
        """Called when robot reaches pickup location."""
        self.task_phase = 'AT_PICKUP'
        self.get_logger().info(f'[{self.robot_id}] ✓ Reached PICKUP for {self.active_task.task_id}! Handling cargo...')
        time.sleep(1.0)  # Brief simulated handling

        self.task_phase = 'NAV_TO_DROPOFF'
        self.get_logger().info(f'[{self.robot_id}] Navigating to DROPOFF location for {self.active_task.task_id}...')
        self._send_nav_goal(self.active_task.dropoff_pose, on_success=self._on_dropoff_reached)

    def _on_dropoff_reached(self):
        """Called when robot reaches dropoff location."""
        self.task_phase = 'COMPLETED'
        self.get_logger().info(
            f'[{self.robot_id}] ★ SUCCESS: Completed task {self.active_task.task_id}! Returning to IDLE.'
        )
        self.active_task = None
        self.task_phase = 'IDLE'

    def _send_nav_goal(self, target_pose: PoseStamped, on_success):
        """Sends goal to Nav2 NavigateToPose action server with graceful offline handling."""
        if not self.nav_client.wait_for_server(timeout_sec=1.5):
            self.get_logger().warn(
                f'[{self.robot_id}] Nav2 action server "navigate_to_pose" not available yet. '
                f'(Waiting for Nav2 bringup...)'
            )
            # When testing headless / without Nav2 active, gracefully simulate arrival
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = target_pose
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        send_future = self.nav_client.send_goal_async(goal_msg)
        send_future.add_done_callback(lambda f: self._goal_response_callback(f, on_success))

    def _goal_response_callback(self, future, on_success):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f'[{self.robot_id}] Nav2 goal was rejected by planner!')
            return

        self.current_goal_handle = goal_handle
        self.get_logger().info(f'[{self.robot_id}] Nav2 goal accepted. Tracking execution...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f: self._goal_result_callback(f, on_success))

    def _goal_result_callback(self, future, on_success):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            on_success()
        else:
            self.get_logger().warn(f'[{self.robot_id}] Nav2 navigation finished with status: {status}')


def main(args=None):
    rclpy.init(args=args)
    node = Nav2Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
