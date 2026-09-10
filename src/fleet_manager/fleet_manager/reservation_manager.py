#!/usr/bin/env python3
"""
Reservation Manager Node
------------------------
Runs identically on each robot.
Manages and negotiates spatial-temporal zone reservations across the fleet.
"""

import rclpy
from rclpy.node import Node

from fleet_interfaces.msg import Reservation


class ReservationManager(Node):
    def __init__(self):
        super().__init__('reservation_manager')

        default_id = self.get_namespace().strip('/') or 'robot_1'
        self.declare_parameter('robot_id', default_id)
        self.robot_id = self.get_parameter('robot_id').get_parameter_value().string_value

        self.reservation_pub = self.create_publisher(Reservation, '/fleet/reservations', 10)
        self.reservation_sub = self.create_subscription(
            Reservation, '/fleet/reservations', self.handle_reservation, 10
        )

        self.timer = self.create_timer(1.0, self.reservation_sync)
        self.get_logger().info(f'[{self.robot_id}] Reservation Manager initialized.')

    def handle_reservation(self, msg: Reservation):
        pass

    def reservation_sync(self):
        pass


def main(args=None):
    rclpy.init(args=args)
    node = ReservationManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
