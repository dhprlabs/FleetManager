#!/usr/bin/env python3
"""
Fleet Visualizer Node
---------------------
Publishes RViz MarkerArrays to visualize the fleet task graph:

  /fleet/task_markers  —  MarkerArray

Marker layout per task:
  • Pickup  sphere  (large, color-coded by state)
  • Pickup  text    (task ID + "P")
  • Dropoff sphere  (large, color-coded by state)
  • Dropoff text    (task ID + "D")
  • Arrow  connecting pickup ──► dropoff

Color coding by task state:
  AVAILABLE         → white
  ALLOCATED/ASSIGNED→ cyan
  IN_PROGRESS       → yellow
  PICKUP_COMPLETED  → orange
  DELIVERING        → blue
  COMPLETED         → dim grey (transparent)
  FAILED/CANCELLED  → red

Subscribes to:
  /fleet/task_pool    (TaskPool  — latched, initial seed)
  /fleet/task_events  (Task      — live updates)
  /fleet/world_views  (WorldView — per-robot world views for cross-check)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

from fleet_interfaces.msg import Task, TaskPool, WorldView


# ── State → colour mapping ─────────────────────────────────────────────────
def _state_color(state: int) -> ColorRGBA:
    c = ColorRGBA()
    c.a = 1.0
    if state == Task.STATE_AVAILABLE:       # white
        c.r, c.g, c.b = 1.0, 1.0, 1.0
    elif state in (Task.STATE_ALLOCATED,
                   Task.STATE_ASSIGNED):    # cyan
        c.r, c.g, c.b = 0.0, 1.0, 1.0
    elif state == Task.STATE_IN_PROGRESS:   # yellow
        c.r, c.g, c.b = 1.0, 0.9, 0.0
    elif state == Task.STATE_PICKUP_COMPLETED:  # orange
        c.r, c.g, c.b = 1.0, 0.5, 0.0
    elif state == Task.STATE_DELIVERING:    # sky blue
        c.r, c.g, c.b = 0.2, 0.6, 1.0
    elif state == Task.STATE_COMPLETED:     # dim grey, semi-transparent
        c.r, c.g, c.b = 0.4, 0.4, 0.4
        c.a = 0.35
    else:                                   # FAILED / CANCELLED — red
        c.r, c.g, c.b = 1.0, 0.1, 0.1
    return c


# Monotonic lifecycle rank mapping
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


class FleetVisualizer(Node):
    def __init__(self):
        super().__init__('fleet_visualizer')

        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('sphere_scale', 0.35)
        self.declare_parameter('text_scale', 0.25)
        self.declare_parameter('arrow_shaft_diameter', 0.06)
        self.declare_parameter('arrow_head_diameter', 0.15)
        self.declare_parameter('publish_rate', 2.0)

        self.frame_id         = self.get_parameter('frame_id').get_parameter_value().string_value
        self.sphere_scale     = self.get_parameter('sphere_scale').get_parameter_value().double_value
        self.text_scale       = self.get_parameter('text_scale').get_parameter_value().double_value
        self.arrow_shaft_d    = self.get_parameter('arrow_shaft_diameter').get_parameter_value().double_value
        self.arrow_head_d     = self.get_parameter('arrow_head_diameter').get_parameter_value().double_value
        self.publish_rate     = self.get_parameter('publish_rate').get_parameter_value().double_value

        # In-memory store: task_id -> Task
        self._tasks: dict[str, Task] = {}

        # ── QoS ────────────────────────────────────────────────────────────
        latching_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # ── Subscriptions ──────────────────────────────────────────────────
        self.create_subscription(
            TaskPool, '/fleet/task_pool', self._on_task_pool, latching_qos
        )
        self.create_subscription(
            Task, '/fleet/task_events', self._on_task_event, 10
        )
        self.create_subscription(
            WorldView, '/fleet/world_views', self._on_world_view, 10
        )

        # ── Publisher ──────────────────────────────────────────────────────
        self._marker_pub = self.create_publisher(MarkerArray, '/fleet/task_markers', 10)

        # ── Timer ──────────────────────────────────────────────────────────
        self.create_timer(1.0 / self.publish_rate, self._publish_markers)

        self.get_logger().info(
            f'Fleet Visualizer running at {self.publish_rate:.1f} Hz | Frame: {self.frame_id}'
        )

    # ── Subscription handlers ──────────────────────────────────────────────

    def _on_task_pool(self, msg: TaskPool):
        for t in msg.tasks:
            if t.task_id not in self._tasks:
                self._tasks[t.task_id] = t
        self._publish_markers()

    def _on_task_event(self, t: Task):
        existing = self._tasks.get(t.task_id)
        # Monotonic guard: never downgrade state from event stream
        if existing is None or _get_state_rank(t.state) >= _get_state_rank(existing.state):
            self._tasks[t.task_id] = t
        self._publish_markers()

    def _on_world_view(self, msg: WorldView):
        updated = False
        for te in msg.task_states:
            existing = self._tasks.get(te.task_id)
            # Build a Task-like object from TaskWorldEntry fields
            t = Task()
            t.task_id        = te.task_id
            t.state          = te.state
            t.assigned_robot_id = te.assigned_robot_id
            t.priority       = te.priority
            t.pickup_pose    = te.pickup_pose
            t.dropoff_pose   = te.dropoff_pose

            if existing is None or _get_state_rank(t.state) >= _get_state_rank(existing.state):
                self._tasks[t.task_id] = t
                updated = True
        if updated:
            self._publish_markers()

    # ── Marker generation ──────────────────────────────────────────────────

    def _publish_markers(self):
        now = self.get_clock().now().to_msg()
        array = MarkerArray()
        marker_id = 0

        # First publish a DELETE_ALL to clear stale markers
        delete_all = Marker()
        delete_all.header.frame_id = self.frame_id
        delete_all.header.stamp = now
        delete_all.ns = 'fleet_tasks'
        delete_all.id = 0
        delete_all.action = Marker.DELETEALL
        array.markers.append(delete_all)

        for task_id, task in self._tasks.items():
            color = _state_color(task.state)
            px = task.pickup_pose.pose.position.x
            py = task.pickup_pose.pose.position.y
            dx = task.dropoff_pose.pose.position.x
            dy = task.dropoff_pose.pose.position.y

            # ── 1. Pickup sphere ─────────────────────────────────────────
            m = Marker()
            m.header.frame_id = self.frame_id
            m.header.stamp = now
            m.ns = 'fleet_tasks'
            m.id = marker_id; marker_id += 1
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = px
            m.pose.position.y = py
            m.pose.position.z = 0.15
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = self.sphere_scale
            m.color = color
            m.lifetime.sec = 3
            array.markers.append(m)

            # ── 2. Pickup label ──────────────────────────────────────────
            lbl = Marker()
            lbl.header.frame_id = self.frame_id
            lbl.header.stamp = now
            lbl.ns = 'fleet_tasks'
            lbl.id = marker_id; marker_id += 1
            lbl.type = Marker.TEXT_VIEW_FACING
            lbl.action = Marker.ADD
            lbl.pose.position.x = px
            lbl.pose.position.y = py
            lbl.pose.position.z = 0.55
            lbl.pose.orientation.w = 1.0
            lbl.scale.z = self.text_scale
            lbl.color.r = lbl.color.g = lbl.color.b = 1.0
            lbl.color.a = 1.0
            owner = f'\n→{task.assigned_robot_id}' if task.assigned_robot_id else ''
            lbl.text = f'{task_id} ▲{owner}'
            lbl.lifetime.sec = 3
            array.markers.append(lbl)

            # ── 3. Dropoff sphere ────────────────────────────────────────
            dm = Marker()
            dm.header.frame_id = self.frame_id
            dm.header.stamp = now
            dm.ns = 'fleet_tasks'
            dm.id = marker_id; marker_id += 1
            dm.type = Marker.CUBE
            dm.action = Marker.ADD
            dm.pose.position.x = dx
            dm.pose.position.y = dy
            dm.pose.position.z = 0.15
            dm.pose.orientation.w = 1.0
            dm.scale.x = dm.scale.y = dm.scale.z = self.sphere_scale
            dm.color = color
            dm.color.a = max(0.35, color.a * 0.75)  # slightly transparent vs pickup
            dm.lifetime.sec = 3
            array.markers.append(dm)

            # ── 4. Dropoff label ─────────────────────────────────────────
            dl = Marker()
            dl.header.frame_id = self.frame_id
            dl.header.stamp = now
            dl.ns = 'fleet_tasks'
            dl.id = marker_id; marker_id += 1
            dl.type = Marker.TEXT_VIEW_FACING
            dl.action = Marker.ADD
            dl.pose.position.x = dx
            dl.pose.position.y = dy
            dl.pose.position.z = 0.55
            dl.pose.orientation.w = 1.0
            dl.scale.z = self.text_scale
            dl.color.r = dl.color.g = dl.color.b = 1.0
            dl.color.a = 1.0
            dl.text = f'{task_id} ■'
            dl.lifetime.sec = 3
            array.markers.append(dl)

            # ── 5. Arrow pickup ──► dropoff ──────────────────────────────
            arr = Marker()
            arr.header.frame_id = self.frame_id
            arr.header.stamp = now
            arr.ns = 'fleet_tasks'
            arr.id = marker_id; marker_id += 1
            arr.type = Marker.ARROW
            arr.action = Marker.ADD
            start = Point(); start.x = px; start.y = py; start.z = 0.15
            end   = Point(); end.x = dx;   end.y = dy;   end.z = 0.15
            arr.points = [start, end]
            arr.scale.x = self.arrow_shaft_d   # shaft diameter
            arr.scale.y = self.arrow_head_d    # head diameter
            arr.scale.z = 0.0
            arr.color = color
            arr.color.a = max(0.25, color.a * 0.6)
            arr.lifetime.sec = 3
            array.markers.append(arr)

        self._marker_pub.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = FleetVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
