#!/usr/bin/env python3
"""Select stationary Task 2 buoys from the shared LiDAR track stream.

The route GPS5 -> GPS6 defines the colours geometrically: a stationary track
near +2.5 m (left) is red, and one near -2.5 m (right) is green.  This avoids
the old buoy-specific detector while retaining the tracker confirmation and
TF/ego-motion handling used for other vessels.
"""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

from njord_interfaces.msg import BuoyDetection, BuoyDetectionArray

try:
    from tf2_ros import TransformException
except ImportError:
    from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
    TransformException = (LookupException, ConnectivityException, ExtrapolationException)

from ship_perception_msgs.msg import TrackedObjectArray
from task2_perception import buoy_glue, cloud_ops, tracking_glue
from task2_perception.tracking_glue import SelectionParams, Track


def _yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class BuoySelectorNode(Node):
    def __init__(self):
        super().__init__("task2_buoy_selector")
        self.declare_parameters("", [
            ("tracked_objects_topic", "/tracked_objects"),
            ("ego_odom_topic", "/task2/ego_odom"),
            ("output_topic", "/task2/buoy_detections"),
            ("legacy_point_topic", "/buoy_detections"),
            ("map_frame", "map"), ("base_frame", "base_link"),
            ("waypoint_start_topic", "/waypoint1_pose"),
            ("waypoint_end_topic", "/waypoint2_pose"),
            ("expected_lateral_offset_m", 2.5),
            ("lateral_tolerance_m", 1.0),
            ("start_margin_m", 2.0), ("end_margin_m", 2.0),
            ("confirmed_only", True), ("min_hit_count", 5),
            ("max_distance_m", 40.0), ("min_point_count", 0),
            ("stale_timeout_sec", 2.0),
            ("stationary_speed_max_mps", 0.35),
            ("publish_rate_hz", 5.0),
        ])
        p = lambda name: self.get_parameter(name).value
        self.map_frame, self.base_frame = str(p("map_frame")), str(p("base_frame"))
        self.expected_offset_m, self.lateral_tolerance_m = float(p("expected_lateral_offset_m")), float(p("lateral_tolerance_m"))
        self.start_margin_m, self.end_margin_m = float(p("start_margin_m")), float(p("end_margin_m"))
        self.stationary_speed_max_mps = float(p("stationary_speed_max_mps"))
        if self.expected_offset_m <= 0.0 or self.lateral_tolerance_m < 0.0 or self.stationary_speed_max_mps < 0.0:
            raise ValueError("Buoy side-line offset/tolerance/speed parameters must be non-negative (offset > 0)")
        self.selection_params = SelectionParams(
            confirmed_only=bool(p("confirmed_only")), max_distance_m=float(p("max_distance_m")),
            min_length_m=0.0, max_length_m=float("inf"), min_width_m=0.0,
            max_width_m=float("inf"), min_height_m=0.0, max_height_m=float("inf"),
            min_point_count=int(p("min_point_count")), min_hit_count=int(p("min_hit_count")),
            stale_timeout_sec=float(p("stale_timeout_sec")))
        self.tracks, self.start_map, self.end_map = [], None, None
        self.ego_vel_base, self.ego_yaw_rate = np.zeros(3), 0.0
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.array_pub = self.create_publisher(BuoyDetectionArray, str(p("output_topic")), 10)
        self.point_pub = self.create_publisher(PointStamped, str(p("legacy_point_topic")), 10)
        self.create_subscription(TrackedObjectArray, str(p("tracked_objects_topic")), self.tracks_callback, 10)
        self.create_subscription(Odometry, str(p("ego_odom_topic")), self.ego_callback, 10)
        self.create_subscription(PoseStamped, str(p("waypoint_start_topic")), lambda msg: self.endpoint_callback(msg, True), 10)
        self.create_subscription(PoseStamped, str(p("waypoint_end_topic")), lambda msg: self.endpoint_callback(msg, False), 10)
        self.timer = self.create_timer(1.0 / max(float(p("publish_rate_hz")), 1e-6), self.timer_callback)

    def tracks_callback(self, msg):
        self.tracks = [Track(
            object_id=int(o.object_id), position=np.array([o.state.pose.pose.position.x, o.state.pose.pose.position.y, o.state.pose.pose.position.z]),
            yaw=_yaw_from_quaternion(o.state.pose.pose.orientation), velocity_body=np.array([o.state.twist.twist.linear.x, o.state.twist.twist.linear.y, o.state.twist.twist.linear.z]),
            yaw_rate=float(o.state.twist.twist.angular.z), dimensions=np.array([o.dimensions.x, o.dimensions.y, o.dimensions.z]),
            point_count=int(o.point_count), track_state=int(o.track_state), stamp_sec=o.header.stamp.sec + o.header.stamp.nanosec * 1e-9,
            hit_count=int(o.hit_count), miss_count=int(o.miss_count)) for o in msg.objects]

    def ego_callback(self, msg):
        v = msg.twist.twist.linear
        self.ego_vel_base = np.array([v.x, v.y, v.z])
        self.ego_yaw_rate = float(msg.twist.twist.angular.z)

    def endpoint_callback(self, msg, is_start):
        if msg.header.frame_id and msg.header.frame_id != self.map_frame:
            self.get_logger().warning("Ignoring buoy-route waypoint outside map frame", throttle_duration_sec=2.0)
            return
        if is_start:
            self.start_map = np.array([msg.pose.position.x, msg.pose.position.y])
        else:
            self.end_map = np.array([msg.pose.position.x, msg.pose.position.y])

    def timer_callback(self):
        if self.start_map is None or self.end_map is None:
            return
        now = self.get_clock().now()
        detections = BuoyDetectionArray()
        detections.header.stamp = now.to_msg()
        detections.header.frame_id = self.map_frame
        for track in tracking_glue.select_opponent(self.tracks, now.nanoseconds * 1e-9, params=self.selection_params):
            try:
                tf = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, Time(seconds=track.stamp_sec))
            except TransformException:
                continue
            q, t = tf.transform.rotation, tf.transform.translation
            transform = cloud_ops.make_transform(cloud_ops.quaternion_to_rotation_matrix(q.x, q.y, q.z, q.w), [t.x, t.y, t.z])
            absolute_base = tracking_glue.ego_compensate(track.velocity_base, self.ego_vel_base, self.ego_yaw_rate, track.position)
            position_map, velocity_map = tracking_glue.to_map_frame(track.position, absolute_base, transform)
            colour = buoy_glue.classify_route_side(position_map, self.start_map, self.end_map, self.expected_offset_m, self.lateral_tolerance_m, self.start_margin_m, self.end_margin_m)
            if colour is None or not buoy_glue.is_stationary(velocity_map, self.stationary_speed_max_mps):
                continue
            detection = BuoyDetection()
            detection.class_id = BuoyDetection.CLASS_GREEN if colour == buoy_glue.GREEN else BuoyDetection.CLASS_RED
            detection.confidence = min(1.0, float(track.hit_count) / max(1, int(self.selection_params.min_hit_count)))
            detection.position.x, detection.position.y, detection.position.z = map(float, position_map)
            detection.position_source = BuoyDetection.POSITION_LIDAR_FUSED
            detections.detections.append(detection)
            point = PointStamped()
            point.header = detections.header
            point.point = detection.position
            self.point_pub.publish(point)
        self.array_pub.publish(detections)


def main(args=None):
    rclpy.init(args=args)
    node = BuoySelectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
