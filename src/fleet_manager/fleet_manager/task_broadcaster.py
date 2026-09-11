#!/usr/bin/env python3
"""
Task Broadcaster Node
--------------------
Central task source for the decentralized warehouse multi-robot system.
Creates tasks (T1, T2, etc.) and publishes them to the fleet.
All tasks initially have state 'AVAILABLE'.
The broadcaster NEVER assigns tasks to robots; robots decide ownership autonomously.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

import json
from geometry_msgs.msg import PoseStamped
from fleet_interfaces.msg import Task, TaskPool, P2PMessage
from fleet_interfaces.srv import CreateTask, GenerateSampleTasks


# Lifecycle progress rank for monotonic state tracking
TASK_STATE_RANK = {
    Task.STATE_AVAILABLE: 0,
    Task.STATE_ALLOCATED: 1,
    Task.STATE_ASSIGNED: 1,
    Task.STATE_IN_PROGRESS: 2,
    Task.STATE_PICKUP_COMPLETED: 3,
    Task.STATE_DELIVERING: 4,
    Task.STATE_COMPLETED: 5,
    Task.STATE_FAILED: 5,
    Task.STATE_CANCELLED: 5,
}


def _get_state_rank(state: int) -> int:
    return TASK_STATE_RANK.get(state, int(state))


class TaskBroadcaster(Node):
    def __init__(self):
        super().__init__('task_broadcaster')

        # Parameters
        self.declare_parameter('publish_rate', 1.0)
        self.declare_parameter('auto_generate_sample_tasks', False)
        self.declare_parameter('warehouse_frame', 'map')
        self.declare_parameter('dock_x', 5.0)
        self.declare_parameter('dock_y', 12.0)
        self.declare_parameter('broadcast_radius', 6.0)

        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.auto_generate = self.get_parameter('auto_generate_sample_tasks').get_parameter_value().bool_value
        self.frame_id = self.get_parameter('warehouse_frame').get_parameter_value().string_value
        self.dock_x = self.get_parameter('dock_x').get_parameter_value().double_value
        self.dock_y = self.get_parameter('dock_y').get_parameter_value().double_value
        self.broadcast_radius = self.get_parameter('broadcast_radius').get_parameter_value().double_value

        # Task repository: task_id -> Task
        self.tasks = {}

        # QoS Profiles
        latching_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # Publishers
        self.task_pool_pub = self.create_publisher(TaskPool, '/fleet/task_pool', latching_qos)
        self.task_event_pub = self.create_publisher(Task, '/fleet/task_events', 10)
        # Wireless medium publisher for P2P-constrained dock transmission
        self.wireless_packet_pub = self.create_publisher(P2PMessage, '/fleet/wireless_packets', 10)
        # Dock station pose publisher for fleet visualizer and web frontend
        self.dock_pose_pub = self.create_publisher(PoseStamped, '/fleet/dock_pose', latching_qos)

        # Subscriptions
        self.task_event_sub = self.create_subscription(
            Task, '/fleet/task_events', self.handle_task_event, 10
        )
        self.batch_task_sub = self.create_subscription(
            TaskPool, '/fleet/broadcast_tasks', self.handle_broadcast_tasks, 10
        )

        # Services
        self.create_task_srv = self.create_service(
            CreateTask, '/fleet/create_task', self.handle_create_task
        )
        self.gen_sample_srv = self.create_service(
            GenerateSampleTasks, '/fleet/generate_sample_tasks', self.handle_generate_sample_tasks
        )

        # Predefined warehouse tasks
        if self.auto_generate:
            self.generate_sample_tasks(count=5)

        # Periodic timer for task pool broadcast
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_task_pool)

        self.get_logger().info(
            f'Task Broadcaster online at Dock Station ({self.dock_x:.1f}, {self.dock_y:.1f}) | '
            f'Radius: {self.broadcast_radius:.1f}m | Rate: {self.publish_rate}Hz'
        )

    def _create_pose(self, x: float, y: float, z: float = 0.0, yaw_w: float = 1.0) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self.frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)
        pose.pose.orientation.w = float(yaw_w)
        return pose

    def create_task_internal(self, task_id: str, pickup_x: float, pickup_y: float,
                             dropoff_x: float, dropoff_y: float, priority: int = 1) -> Task:
        task = Task()
        task.task_id = task_id
        task.pickup_pose = self._create_pose(pickup_x, pickup_y)
        task.dropoff_pose = self._create_pose(dropoff_x, dropoff_y)
        task.priority = priority
        task.creation_time = self.get_clock().now().to_msg()
        task.version = 1
        task.state = Task.STATE_AVAILABLE  # Explicitly AVAILABLE
        task.assigned_robot_id = ''        # Broadcaster NEVER assigns ownership

        self.tasks[task_id] = task

        # Logging checkpoint requirement
        self.get_logger().info(
            f'Created {task.task_id} (AVAILABLE) | Priority: {task.priority} | '
            f'Pickup: ({pickup_x:.1f}, {pickup_y:.1f}) -> Dropoff: ({dropoff_x:.1f}, {dropoff_y:.1f})'
        )

        # Publish immediate event
        self.task_event_pub.publish(task)
        return task

    def generate_sample_tasks(self, count: int = 5):
        """Generates predefined warehouse tasks T1 through T5."""
        sample_locations = [
            # T1
            {'id': 'T1', 'p': (2.0, 1.0), 'd': (-2.0, -1.0), 'pri': 2},
            # T2
            {'id': 'T2', 'p': (3.5, 1.0), 'd': (-3.5, -1.0), 'pri': 1},
            # T3
            {'id': 'T3', 'p': (2.0, -2.5), 'd': (-2.0, 2.5), 'pri': 3},
            # T4
            {'id': 'T4', 'p': (4.0, -2.5), 'd': (-4.0, 2.5), 'pri': 1},
            # T5
            {'id': 'T5', 'p': (1.0, 3.0), 'd': (-1.0, -3.0), 'pri': 2},
        ]

        created_ids = []
        for i in range(min(count, len(sample_locations))):
            spec = sample_locations[i]
            t = self.create_task_internal(
                task_id=spec['id'],
                pickup_x=spec['p'][0],
                pickup_y=spec['p'][1],
                dropoff_x=spec['d'][0],
                dropoff_y=spec['d'][1],
                priority=spec['pri']
            )
            created_ids.append(t.task_id)

        # Update pool publication immediately
        self.publish_task_pool()
        return created_ids

    def publish_task_pool(self):
        pool_msg = TaskPool()
        pool_msg.header.stamp = self.get_clock().now().to_msg()
        pool_msg.header.frame_id = self.frame_id
        pool_msg.tasks = list(self.tasks.values())
        self.task_pool_pub.publish(pool_msg)

        # Publish dock station pose
        dock_pose = self._create_pose(self.dock_x, self.dock_y)
        self.dock_pose_pub.publish(dock_pose)

        # Broadcast over simulated wireless medium from dock station location
        p2p_msg = P2PMessage()
        p2p_msg.source_robot_id = 'dock_station'
        p2p_msg.target_robot_id = '*'
        p2p_msg.message_type = 'TASK_POOL'
        p2p_msg.timestamp = pool_msg.header.stamp
        p2p_msg.sender_pose.header.frame_id = self.frame_id
        p2p_msg.sender_pose.header.stamp = pool_msg.header.stamp
        p2p_msg.sender_pose.pose.position.x = float(self.dock_x)
        p2p_msg.sender_pose.pose.position.y = float(self.dock_y)
        p2p_msg.sender_pose.pose.orientation.w = 1.0

        tasks_payload = []
        for t in self.tasks.values():
            tasks_payload.append({
                'task_id': t.task_id,
                'state': int(t.state),
                'priority': int(t.priority),
                'pickup_x': t.pickup_pose.pose.position.x,
                'pickup_y': t.pickup_pose.pose.position.y,
                'dropoff_x': t.dropoff_pose.pose.position.x,
                'dropoff_y': t.dropoff_pose.pose.position.y,
                'assigned_robot_id': t.assigned_robot_id,
                'version': int(t.version),
            })
        p2p_msg.payload = json.dumps({'tasks': tasks_payload, 'dock_x': self.dock_x, 'dock_y': self.dock_y})
        self.wireless_packet_pub.publish(p2p_msg)

    def handle_create_task(self, request, response):
        task = request.task
        if not task.task_id:
            response.success = False
            response.message = "Task ID cannot be empty"
            return response

        # Force state to AVAILABLE and clear any assignment
        task.state = Task.STATE_AVAILABLE
        task.assigned_robot_id = ''
        task.creation_time = self.get_clock().now().to_msg()
        task.version = 1

        self.tasks[task.task_id] = task
        self.get_logger().info(f'Created {task.task_id} (AVAILABLE) via service')
        self.task_event_pub.publish(task)
        self.publish_task_pool()

        response.success = True
        response.message = f"Task {task.task_id} successfully broadcasted as AVAILABLE"
        return response

    def handle_broadcast_tasks(self, msg: TaskPool):
        """Receives a batch of tasks from frontend and broadcasts them to the fleet."""
        if not msg.tasks:
            self.get_logger().warn('Received empty task batch on /fleet/broadcast_tasks')
            return

        now = self.get_clock().now().to_msg()
        added_ids = []
        for task in msg.tasks:
            if not task.task_id:
                task.task_id = f'T{len(self.tasks) + 1}'

            task.state = Task.STATE_AVAILABLE
            task.assigned_robot_id = ''
            task.creation_time = now
            task.version = 1
            if task.priority <= 0:
                task.priority = 1

            self.tasks[task.task_id] = task
            self.task_event_pub.publish(task)
            added_ids.append(task.task_id)
            self.get_logger().info(
                f'Batch task added: {task.task_id} (AVAILABLE) | '
                f'Pickup: ({task.pickup_pose.pose.position.x:.2f}, {task.pickup_pose.pose.position.y:.2f}) -> '
                f'Dropoff: ({task.dropoff_pose.pose.position.x:.2f}, {task.dropoff_pose.pose.position.y:.2f})'
            )

        self.get_logger().info(f'Broadcasted batch of {len(added_ids)} tasks to fleet: {added_ids}')
        self.publish_task_pool()

    def handle_task_event(self, task: Task):
        """Updates internal task state so periodic broadcasts maintain live states."""
        existing = self.tasks.get(task.task_id)
        if existing is None or _get_state_rank(task.state) >= _get_state_rank(existing.state):
            self.tasks[task.task_id] = task

    def handle_generate_sample_tasks(self, request, response):
        count = request.count if request.count > 0 else 5
        created = self.generate_sample_tasks(count)
        response.success = True
        response.task_ids = created
        response.message = f"Generated {len(created)} tasks"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = TaskBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
