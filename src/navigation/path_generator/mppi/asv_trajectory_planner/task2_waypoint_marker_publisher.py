#!/usr/bin/env python3
"""Render Task 2's two GPS-reference poses as persistent 3D markers."""

import rclpy
from geometry_msgs.msg import Point, PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from visualization_msgs.msg import Marker, MarkerArray


class Task2WaypointMarkerPublisher(Node):
    """Turn /waypoint1_pose and /waypoint2_pose into one Foxglove marker topic."""

    def __init__(self):
        super().__init__("task2_waypoint_marker_publisher")
        self.declare_parameter("waypoint1_topic", "/waypoint1_pose")
        self.declare_parameter("waypoint2_topic", "/waypoint2_pose")
        self.declare_parameter("marker_topic", "/task2/gps_waypoint_markers")
        self.declare_parameter("start_label", "GPS 5 - Start")
        self.declare_parameter("goal_label", "GPS 6 - Goal")
        self._waypoints = [None, None]
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._publisher = self.create_publisher(
            MarkerArray, self.get_parameter("marker_topic").value, qos)
        self.create_subscription(
            PoseStamped, self.get_parameter("waypoint1_topic").value,
            lambda message: self._on_waypoint(0, message), 10)
        self.create_subscription(
            PoseStamped, self.get_parameter("waypoint2_topic").value,
            lambda message: self._on_waypoint(1, message), 10)

    def _on_waypoint(self, index, message):
        self._waypoints[index] = message
        if all(self._waypoints):
            self._publish()

    @staticmethod
    def _marker(message, marker_id, marker_type, color, scale):
        marker = Marker()
        marker.header = message.header
        marker.ns = "task2_gps_waypoints"
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x, marker.scale.y, marker.scale.z = scale
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        return marker

    def _publish(self):
        start, goal = self._waypoints
        if start.header.frame_id != goal.header.frame_id:
            self.get_logger().warning("Task 2 waypoint frames differ; waiting for matching frames.")
            return
        markers = MarkerArray()
        for index, (message, label, color) in enumerate((
                (start, self.get_parameter("start_label").value, (1.0, 0.75, 0.05, 1.0)),
                (goal, self.get_parameter("goal_label").value, (0.1, 0.8, 1.0, 1.0)))):
            sphere = self._marker(message, index * 2, Marker.SPHERE, color, (0.8, 0.8, 0.25))
            sphere.pose.position = message.pose.position
            markers.markers.append(sphere)
            text = self._marker(message, index * 2 + 1, Marker.TEXT_VIEW_FACING, (1.0, 1.0, 1.0, 1.0), (0.0, 0.0, 0.55))
            text.pose.position = message.pose.position
            text.pose.position.z += 0.8
            text.text = label
            markers.markers.append(text)
        route = self._marker(start, 4, Marker.LINE_STRIP, (0.95, 0.95, 0.95, 0.8), (0.06, 0.0, 0.0))
        route.points = [
            Point(x=start.pose.position.x, y=start.pose.position.y, z=start.pose.position.z + 0.03),
            Point(x=goal.pose.position.x, y=goal.pose.position.y, z=goal.pose.position.z + 0.03),
        ]
        markers.markers.append(route)
        self._publisher.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = Task2WaypointMarkerPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
