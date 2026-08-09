#!/usr/bin/env python3

"""Task2 waypoint pose publisher for the REAL vessel stack.

Why this node exists (waypoint-source decision):
    The waypoint_publisher package delivers Task2 waypoints only as a
    NavigateThroughPoses action goal plus /task_waypoints // /plan Path
    topics containing ALL configured waypoints (start, gates, goal). The
    MPPI planner_node instead needs exactly two PoseStamped topics
    (/waypoint1_pose, /waypoint2_pose) defining the GPS reference line, and
    waypoint_publisher offers no parameter or remap that can produce them.
    The sim-only task2_gps_waypoint_publisher relies on /sim/task2_gps_markers,
    which does not exist on the real vessel. Hence this minimal node.

Conversion approach:
    waypoint_publisher performs no runtime GPS->map conversion: its
    _build_poses_from_config() reads map-frame x/y directly from
    share/waypoint_publisher/config/task2_waypoints.yaml (the lat/lon
    entries are reference metadata; the conversion to the map frame is
    precomputed offline into the x/y fields). This node loads the SAME
    shared YAML at runtime and reuses the same read-x/y approach, selecting
    the 'start'-type waypoint as waypoint1 and the 'goal'-type waypoint as
    waypoint2 (fallback: first / last entries).
"""

import math

import rclpy
from rclpy.node import Node

import yaml
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from geometry_msgs.msg import PoseStamped
from geographic_msgs.msg import GeoPoint
from robot_localization.srv import FromLL


class Task2WaypointPosePublisher(Node):
    """
    share/waypoint_publisher/config/task2_waypoints.yaml を読み込み，
    start / goal のwaypointを /waypoint1_pose, /waypoint2_pose として
    周期的(既定2Hz)にpublishするノード．
    """

    def __init__(self):
        super().__init__("task2_waypoint_pose_publisher")

        self.declare_parameter("config_package", "waypoint_publisher")
        self.declare_parameter("config_file", "task2_waypoints.yaml")
        self.declare_parameter("config_key", "task2_config")
        self.declare_parameter("waypoint1_topic", "/waypoint1_pose")
        self.declare_parameter("waypoint2_topic", "/waypoint2_pose")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("publish_frequency", 2.0)

        config_package = self.get_parameter("config_package").value
        config_file = self.get_parameter("config_file").value
        config_key = self.get_parameter("config_key").value

        self.waypoint1_topic = self.get_parameter("waypoint1_topic").value
        self.waypoint2_topic = self.get_parameter("waypoint2_topic").value
        self.frame_id = self.get_parameter("frame_id").value

        publish_frequency = self.get_parameter("publish_frequency").value
        timer_period = 1.0 / max(publish_frequency, 1e-6)

        self.coordinate_mode, self.origin, self.start_wp, self.goal_wp = self._load_waypoints(
            config_package=config_package,
            config_file=config_file,
            config_key=config_key,
        )

        self.wp1_pub = self.create_publisher(PoseStamped, self.waypoint1_topic, 10)
        self.wp2_pub = self.create_publisher(PoseStamped, self.waypoint2_topic, 10)
        self.from_ll_client = self.create_client(FromLL, "/fromLL")
        self.waypoint1_xy = None
        self.waypoint2_xy = None
        self.projection_futures = None

        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.create_timer(0.2, self._resolve_coordinates)

        self.get_logger().info("task2_waypoint_pose_publisher started.")
        self.get_logger().info(
            f"Config   : {config_package}/config/{config_file} [{config_key}]"
        )
        self.get_logger().info(
            f"waypoint1: {self.waypoint1_xy} -> {self.waypoint1_topic}"
        )
        self.get_logger().info(
            f"waypoint2: {self.waypoint2_xy} -> {self.waypoint2_topic}"
        )

    def _load_waypoints(self, config_package, config_file, config_key):
        """
        waypoint_publisherと同じ共有YAMLを読み、start/goalの(x, y)を返す。

        waypoint_publisher._build_poses_from_config()と同様に、
        map基準のx/yフィールドをそのまま使用する(GPS lat/lonはメタデータ)。
        """
        config_path = (
            Path(get_package_share_directory(config_package))
            / "config"
            / config_file
        )

        if not config_path.exists():
            raise FileNotFoundError(f"Config file not found: {config_path}")

        with open(config_path, "r") as f:
            full_config = yaml.safe_load(f)

        config = full_config[config_key]
        waypoints = config.get("waypoints", [])

        if len(waypoints) < 2:
            raise ValueError(
                f"Need at least 2 waypoints in {config_path}, got {len(waypoints)}"
            )

        start_wp = next(
            (wp for wp in waypoints if wp.get("type") == "start"),
            waypoints[0],
        )
        goal_wp = next(
            (wp for wp in waypoints if wp.get("type") == "goal"),
            waypoints[-1],
        )

        mode = config.get("coordinate_mode", "map_xy")
        if mode not in ("map_xy", "origin_relative_xy", "geodetic"):
            raise ValueError("coordinate_mode must be map_xy, origin_relative_xy, or geodetic")
        return mode, config.get("origin"), start_wp, goal_wp

    @staticmethod
    def _request(point):
        request = FromLL.Request()
        request.ll_point = GeoPoint(
            latitude=float(point["latitude"]), longitude=float(point["longitude"]),
            altitude=float(point.get("altitude", 0.0)))
        return request

    def _resolve_coordinates(self):
        if self.waypoint1_xy is not None:
            return
        if self.coordinate_mode == "map_xy":
            self.waypoint1_xy = (float(self.start_wp["x"]), float(self.start_wp["y"]))
            self.waypoint2_xy = (float(self.goal_wp["x"]), float(self.goal_wp["y"]))
            return
        if not self.from_ll_client.service_is_ready():
            self.get_logger().warning("Waiting for /fromLL to resolve Task2 GPS waypoints.", throttle_duration_sec=2.0)
            return
        if self.projection_futures is None:
            try:
                points = ([self.origin] if self.coordinate_mode == "origin_relative_xy"
                          else [self.start_wp, self.goal_wp])
                if any(not isinstance(point, dict) or "latitude" not in point or "longitude" not in point for point in points):
                    raise ValueError(f"{self.coordinate_mode} requires the documented latitude/longitude fields")
                self.projection_futures = [self.from_ll_client.call_async(self._request(point)) for point in points]
            except Exception as error:
                self.get_logger().error(f"Invalid Task2 waypoint GPS configuration: {error}")
            return
        if not all(future.done() for future in self.projection_futures):
            return
        try:
            positions = [(future.result().map_point.x, future.result().map_point.y)
                         for future in self.projection_futures]
            if self.coordinate_mode == "origin_relative_xy":
                ox, oy = positions[0]
                self.waypoint1_xy = (ox + float(self.start_wp["x"]), oy + float(self.start_wp["y"]))
                self.waypoint2_xy = (ox + float(self.goal_wp["x"]), oy + float(self.goal_wp["y"]))
            else:
                self.waypoint1_xy, self.waypoint2_xy = positions
            self.get_logger().info(f"Resolved Task2 {self.coordinate_mode} waypoints in map frame.")
        except Exception as error:
            self.projection_futures = None
            self.get_logger().error(f"Task2 GPS waypoint projection failed: {error}")

    def timer_callback(self):
        if self.waypoint1_xy is None or self.waypoint2_xy is None:
            return
        x1, y1 = self.waypoint1_xy
        x2, y2 = self.waypoint2_xy

        now = self.get_clock().now().to_msg()

        wp1_msg = self._make_pose_stamped(
            x=x1, y=y1, next_x=x2, next_y=y2, stamp=now
        )
        wp2_msg = self._make_pose_stamped(
            x=x2, y=y2, next_x=x1, next_y=y1, stamp=now
        )

        self.wp1_pub.publish(wp1_msg)
        self.wp2_pub.publish(wp2_msg)

        self.get_logger().info(
            f"Published Task2 waypoints: wp1=({x1:.2f}, {y1:.2f}), "
            f"wp2=({x2:.2f}, {y2:.2f})",
            throttle_duration_sec=10.0,
        )

    def _make_pose_stamped(self, x, y, next_x, next_y, stamp):
        # task2_gps_waypoint_publisher.make_pose_stamped と同じ向き付け:
        # 相手waypointへ向かうyawを設定する。
        yaw = math.atan2(next_y - y, next_x - x)

        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id

        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 0.0

        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)

        return msg


def main(args=None):
    rclpy.init(args=args)

    node = Task2WaypointPosePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
