"""Publish the selected local waypoint YAML as geographic display markers."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
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


class GroundWaypointGeoPublisher(Node):
    """Ground-only WGS84 marker source; no vessel waypoint topic is consumed."""

    def __init__(self):
        super().__init__("ground_waypoint_geo_publisher")
        self.declare_parameter("task_type", "task1")
        self.declare_parameter("marker_topic", "/ground_waypoint_markers")
        self.declare_parameter("publish_rate_hz", 1.0)
        task_type = self.get_parameter("task_type").value
        if task_type not in _CONFIGS:
            raise ValueError(f"unknown task_type '{task_type}'")
        filename, key = _CONFIGS[task_type]
        config_path = Path(get_package_share_directory("waypoint_publisher")) / "config" / filename
        with config_path.open() as stream:
            self.waypoints = yaml.safe_load(stream)[key].get("waypoints", [])
        self.publisher = self.create_publisher(
            MarkerArray,
            self.get_parameter("marker_topic").value,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        rate = float(self.get_parameter("publish_rate_hz").value)
        if rate <= 0.0:
            raise ValueError("publish_rate_hz must be positive")
        self._publish()
        self.timer = self.create_timer(1.0 / rate, self._publish)

    def _publish(self):
        markers = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        for index, waypoint in enumerate(self.waypoints):
            if "latitude" not in waypoint or "longitude" not in waypoint:
                continue
            marker = Marker()
            marker.header.frame_id, marker.header.stamp = "wgs84", stamp
            marker.ns, marker.id = "ground_waypoint_wgs84", index
            marker.type, marker.action = Marker.SPHERE, Marker.ADD
            # This display-only contract uses x=longitude and y=latitude.
            marker.pose.position.x = float(waypoint["longitude"])
            marker.pose.position.y = float(waypoint["latitude"])
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
