"""Publish the selected local waypoint YAML as geographic display markers."""

from pathlib import Path
import re

from ament_index_python.packages import get_package_share_directory
from njord_interfaces.msg import MissionStatus
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from visualization_msgs.msg import Marker, MarkerArray
import yaml


_CONFIGS = {
    "task1": ("task1_waypoints.yaml", "task1_config"),
    "task1_skip_1_1": ("task1_skip_1_1_waypoints.yaml", "task1_skip_1_1_config"),
    "task1_follow": ("task1_follow_waypoints.yaml", "task1_follow_config"),
    "task2": ("task2_waypoints.yaml", "task2_config"),
    "task3_1": ("task3_waypoints.yaml", "task3_1_config"),
    "task3_2": ("task3_waypoints.yaml", "task3_2_config"),
}


def _coordinate(value):
    if isinstance(value, (int, float)):
        return float(value)
    match = re.fullmatch(r"\s*(\d+)°(\d+)[′']([0-9.]+)[″\"]\s*([NSEW])\s*", str(value))
    if not match:
        raise ValueError(f"invalid coordinate {value!r}")
    result = int(match.group(1)) + int(match.group(2)) / 60 + float(match.group(3)) / 3600
    return -result if match.group(4) in "SW" else result


class GroundWaypointGeoPublisher(Node):
    """Ground-only WGS84 marker source for Foxglove waypoint inspection.

    When ``task_type`` is set, this node is entirely independent of Mission
    Manager and continuously displays that task's YAML route.  An empty value
    retains the ground-PC bringup behaviour of following ``/mission/status``.
    """

    def __init__(self):
        super().__init__("ground_waypoint_geo_publisher")
        self.declare_parameter("marker_topic", "/ground_waypoint_markers")
        self.declare_parameter("publish_rate_hz", 1.0)
        self.declare_parameter("task_type", "")
        self.waypoints = []
        self.active_task = ""
        self.publisher = self.create_publisher(
            MarkerArray,
            self.get_parameter("marker_topic").value,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        rate = float(self.get_parameter("publish_rate_hz").value)
        if rate <= 0.0:
            raise ValueError("publish_rate_hz must be positive")
        selected_task = str(self.get_parameter("task_type").value).strip()
        self.status_sub = None
        if selected_task:
            self._select_task(selected_task)
        else:
            status_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.status_sub = self.create_subscription(
                MissionStatus, "/mission/status", self._on_status, status_qos)
        self.timer = self.create_timer(1.0 / rate, self._publish)

    def _on_status(self, message):
        """Load the YAML selected by the task command sent from ground."""
        self._select_task(message.task_id.strip())

    def _select_task(self, task_type):
        """Load a configured route without creating navigation goals."""
        if task_type == self.active_task:
            return
        self.active_task = task_type
        self.waypoints = []
        if task_type not in _CONFIGS:
            self.get_logger().info("No active task waypoint display")
            self._publish()
            return
        filename, key = _CONFIGS[task_type]
        config_path = Path(get_package_share_directory("waypoint_publisher")) / "config" / filename
        with config_path.open() as stream:
            self.waypoints = yaml.safe_load(stream)[key].get("waypoints", [])
        self.get_logger().info(f"Showing local YAML waypoints for {task_type}")
        self._publish()

    def _publish(self):
        markers = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        reset = Marker()
        reset.action = Marker.DELETEALL
        markers.markers.append(reset)
        for index, waypoint in enumerate(self.waypoints):
            if "latitude" not in waypoint or "longitude" not in waypoint:
                continue
            marker = Marker()
            marker.header.frame_id, marker.header.stamp = "wgs84", stamp
            # The namespace carries display metadata only.  The Foxglove
            # extension uses it to show which Mission Manager task selected
            # this route while retaining the same WGS84 marker contract.
            marker.ns, marker.id = f"ground_waypoint_wgs84/{self.active_task}", index
            marker.type, marker.action = Marker.SPHERE, Marker.ADD
            # This display-only contract uses x=longitude and y=latitude.
            marker.pose.position.x = _coordinate(waypoint["longitude"])
            marker.pose.position.y = _coordinate(waypoint["latitude"])
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = marker.scale.z = 1.0
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.0, 0.9, 1.0, 1.0
            marker.text = str(waypoint.get("competition_id", waypoint.get("id", index + 1)))
            markers.markers.append(marker)
        self.publisher.publish(markers)


def main():
    rclpy.init()
    node = GroundWaypointGeoPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
