#!/usr/bin/env python3
"""
Intent Broadcaster Node
-----------------------
Runs identically on each robot.
Broadcasts motion intents, target poses, and trajectory predictions
to coordinate with surrounding robots.
"""

import rclpy
from rclpy.node import Node

from fleet_interfaces.msg import Intent


class IntentBroadcaster(Node):
    def __init__(self):
        super().__init__('intent_broadcaster')

        default_id = self.get_namespace().strip('/') or 'robot_1'
        self.declare_parameter('robot_id', default_id)
        self.robot_id = self.get_parameter('robot_id').get_parameter_value().string_value

        self.intent_pub = self.create_publisher(Intent, 'intent', 10)
        self.fleet_intent_pub = self.create_publisher(Intent, '/fleet/intents', 10)

        self.timer = self.create_timer(1.0, self.publish_intent)
        self.get_logger().info(f'[{self.robot_id}] Intent Broadcaster initialized.')

    def publish_intent(self):
        msg = Intent()
        msg.robot_id = self.robot_id
        msg.task_id = ''
        msg.expected_arrival_time = self.get_clock().now().to_msg()
        self.intent_pub.publish(msg)
        self.fleet_intent_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = IntentBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
