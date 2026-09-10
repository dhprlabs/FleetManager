#!/usr/bin/env python3
"""
Bundle Manager Node
-------------------
Runs identically on each robot.
Manages the robot's bundle/sequence of allocated tasks and insertion costs.
"""

import rclpy
from rclpy.node import Node

from fleet_interfaces.msg import Bundle, TaskOwnership


class BundleManager(Node):
    def __init__(self):
        super().__init__('bundle_manager')

        default_id = self.get_namespace().strip('/') or 'robot_1'
        self.declare_parameter('robot_id', default_id)
        self.robot_id = self.get_parameter('robot_id').get_parameter_value().string_value

        self.bundle = []  # Ordered list of task_ids
        self.bundle_pub = self.create_publisher(Bundle, 'bundle', 10)
        self.fleet_bundle_pub = self.create_publisher(Bundle, '/fleet/bundles', 10)

        # Subscribe to task ownership to track won tasks
        self.ownership_sub = self.create_subscription(
            TaskOwnership, 'task_ownership', self._handle_ownership, 10
        )

        self.timer = self.create_timer(1.0, self.publish_bundle)
        self.get_logger().info(f'[{self.robot_id}] Bundle Manager initialized.')

    def _handle_ownership(self, msg: TaskOwnership):
        if msg.robot_id == self.robot_id and msg.task_id not in self.bundle:
            self.bundle.append(msg.task_id)
            self.get_logger().info(f'[{self.robot_id}] Added {msg.task_id} to bundle: {self.bundle}')
            self.publish_bundle()

    def publish_bundle(self):
        msg = Bundle()
        msg.robot_id = self.robot_id
        msg.task_ids = list(self.bundle)
        msg.total_estimated_cost = 0.0
        msg.timestamp = self.get_clock().now().to_msg()
        self.bundle_pub.publish(msg)
        self.fleet_bundle_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BundleManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
