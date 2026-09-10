#!/usr/bin/env python3
"""
Pickup Validator Node
---------------------
Runs identically on each robot.
Verifies pickup observation correctness upon reaching target stations.
"""

import rclpy
from rclpy.node import Node

from fleet_interfaces.msg import PickupObservation, Task


class PickupValidator(Node):
    def __init__(self):
        super().__init__('pickup_validator')

        default_id = self.get_namespace().strip('/') or 'robot_1'
        self.declare_parameter('robot_id', default_id)
        self.robot_id = self.get_parameter('robot_id').get_parameter_value().string_value

        self.observation_pub = self.create_publisher(PickupObservation, 'pickup_observation', 10)
        self.task_sub = self.create_subscription(Task, 'assigned_task', self.handle_assigned_task, 10)

        self.timer = self.create_timer(1.0, self.status_check)
        self.get_logger().info(f'[{self.robot_id}] Pickup Validator initialized.')

    def handle_assigned_task(self, task: Task):
        pass

    def status_check(self):
        pass


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
