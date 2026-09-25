#!/usr/bin/env python3
"""
Peer Obstacle Publisher — Simulation Dynamic Obstacle Layer
-----------------------------------------------------------
Runs on each robot. Reads peer robot positions from /fleet/robot_states
and publishes a synthetic LaserScan on 'peer_obstacles_scan' (namespaced)
so Nav2's local and global costmaps treat peer robots as real obstacles.

This is a simulation-only node. In a real deployment the actual LiDAR
would see other robots. Here we inject them directly into the costmap.

The synthetic scan places 'hit' beams around each peer robot's footprint
radius sampled at 3-degree resolution over 360 degrees.
Nav2's VoxelLayer / ObstacleLayer processes the scan exactly like a real
LiDAR, inflating around the hits via the InflationLayer.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
from fleet_interfaces.msg import RobotState

# Footprint radius to inflate around each peer robot centre (metres)
PEER_ROBOT_RADIUS = 0.28
# Scan angular resolution (radians)
SCAN_ANGLE_STEP = math.radians(1.0)
SCAN_ANGLE_MIN = -math.pi
SCAN_ANGLE_MAX = math.pi
NUM_RANGES = int(round((SCAN_ANGLE_MAX - SCAN_ANGLE_MIN) / SCAN_ANGLE_STEP)) + 1
# Maximum range reported by this virtual sensor
MAX_RANGE = 10.0
# Only inject obstacles for peers within this radius
MAX_PEER_RANGE = 8.0


class PeerObstaclePublisher(Node):
    """Publishes peer robot positions as a virtual LaserScan for Nav2 costmaps."""

    def __init__(self):
        super().__init__('peer_obstacle_publisher')

        default_id = self.get_namespace().strip('/') or 'robot_1'
        self.declare_parameter('robot_id', default_id)
        self.robot_id = self.get_parameter('robot_id').get_parameter_value().string_value

        self._peer_positions: dict = {}
        self._own_x: float = 0.0
        self._own_y: float = 0.0

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.robot_state_sub = self.create_subscription(
            RobotState, '/fleet/robot_states', self._handle_robot_state, 10
        )

        self.scan_pub = self.create_publisher(
            LaserScan, 'peer_obstacles_scan', sensor_qos
        )

        # Publish at 10 Hz
        self.timer = self.create_timer(0.1, self._publish_scan)

        self.get_logger().info(
            f'[{self.robot_id}] PeerObstaclePublisher online — '
            f'publishing peer robots to peer_obstacles_scan'
        )

    def _handle_robot_state(self, msg: RobotState):
        x = msg.current_pose.pose.position.x
        y = msg.current_pose.pose.position.y
        if msg.robot_id == self.robot_id:
            self._own_x = x
            self._own_y = y
        else:
            self._peer_positions[msg.robot_id] = (x, y)

    def _publish_scan(self):
        """Build and publish a virtual LaserScan encoding all peer robot positions."""
        ranges = [MAX_RANGE] * NUM_RANGES

        for _peer_id, (px, py) in self._peer_positions.items():
            dx = px - self._own_x
            dy = py - self._own_y
            dist_to_peer = math.hypot(dx, dy)

            if dist_to_peer > MAX_PEER_RANGE or dist_to_peer < 0.05:
                continue

            # Place hits on a circle of radius PEER_ROBOT_RADIUS around peer centre
            for angle_deg in range(0, 360, 3):
                edge_angle = math.radians(angle_deg)
                edge_x = px + PEER_ROBOT_RADIUS * math.cos(edge_angle)
                edge_y = py + PEER_ROBOT_RADIUS * math.sin(edge_angle)

                rdx = edge_x - self._own_x
                rdy = edge_y - self._own_y
                r_dist = math.hypot(rdx, rdy)
                if r_dist < 0.01:
                    continue

                r_angle = math.atan2(rdy, rdx)
                idx = int(round((r_angle - SCAN_ANGLE_MIN) / SCAN_ANGLE_STEP))
                idx = max(0, min(NUM_RANGES - 1, idx))

                if r_dist < ranges[idx]:
                    ranges[idx] = float(r_dist)

        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = f'{self.robot_id}/base_footprint'
        scan.angle_min = SCAN_ANGLE_MIN
        scan.angle_max = SCAN_ANGLE_MAX
        scan.angle_increment = SCAN_ANGLE_STEP
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = 0.05
        scan.range_max = MAX_RANGE
        scan.ranges = ranges
        scan.intensities = []

        self.scan_pub.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = PeerObstaclePublisher()
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
