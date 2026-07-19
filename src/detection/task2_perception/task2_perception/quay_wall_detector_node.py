#!/usr/bin/env python3
"""Quay wall detector for Task 2 (node name: quay_wall_detector).

Subscribes /pcl/nonground (PointCloud2, base_link — output of the
pcl_segmentation ground remover) and extracts wall line segments via the
pure-numpy wall_fit module with temporal confirmation. Publishes:

  * /quay_wall/points  (PointCloud2, base_link) confirmed wall inlier points
    — this is the Nav2 obstacle-layer feed ("quay" observation source).
  * /quay_wall/markers (MarkerArray LINE_LIST, gated by publish_wall_markers)
  * /quay_wall/costmap (OccupancyGrid in the map frame via TF, segments
    inflated by quay_safety_margin_m, gated by publish_wall_costmap)

Note: enable_mppi_quay_cost belongs to the planner package, not here.
"""

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time

from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

try:
    from tf2_ros import TransformException
except ImportError:
    from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
    TransformException = (LookupException, ConnectivityException, ExtrapolationException)

from task2_perception import cloud_ops, wall_fit


def _cloud_to_xyz(msg: PointCloud2) -> np.ndarray:
    data = point_cloud2.read_points(
        msg, field_names=("x", "y", "z"), skip_nans=True)
    arr = np.asarray(data)
    if arr.dtype.names:
        return np.column_stack(
            [np.asarray(arr[n], dtype=np.float64) for n in ("x", "y", "z")])
    out = np.asarray([list(p) for p in data], dtype=np.float64)
    return out.reshape(-1, 3)


class QuayWallDetectorNode(Node):

    def __init__(self):
        super().__init__("quay_wall_detector")

        self.declare_parameters("", [
            ("input_topic", "/pcl/nonground"),
            ("map_frame", "map"),
            ("base_frame", "base_link"),
            ("wall_min_points", 30),
            ("wall_min_length_m", 2.0),
            ("wall_max_distance_m", 40.0),
            ("wall_line_distance_threshold_m", 0.15),
            ("wall_normal_z_max", 0.3),
            ("wall_temporal_confirmations", 3),
            ("wall_timeout_sec", 2.0),
            ("quay_safety_margin_m", 5.0),
            ("publish_wall_markers", True),
            ("publish_wall_costmap", True),
            ("costmap_resolution_m", 0.5),
            ("costmap_size_m", 100.0),
        ])
        gp = lambda name: self.get_parameter(name).value  # noqa: E731

        self.map_frame = str(gp("map_frame"))
        self.base_frame = str(gp("base_frame"))
        self.min_points = int(gp("wall_min_points"))
        self.min_length = float(gp("wall_min_length_m"))
        self.max_distance = float(gp("wall_max_distance_m"))
        self.line_dist_thresh = float(gp("wall_line_distance_threshold_m"))
        self.normal_z_max = float(gp("wall_normal_z_max"))
        self.safety_margin = float(gp("quay_safety_margin_m"))
        self.publish_markers = bool(gp("publish_wall_markers"))
        self.publish_costmap = bool(gp("publish_wall_costmap"))
        self.costmap_resolution = float(gp("costmap_resolution_m"))
        self.costmap_size = float(gp("costmap_size_m"))

        self.tracker = wall_fit.WallTracker(
            confirmations=int(gp("wall_temporal_confirmations")),
            timeout_sec=float(gp("wall_timeout_sec")))
        self.rng = np.random.default_rng(0)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pub_points = self.create_publisher(
            PointCloud2, "/quay_wall/points", 10)
        self.pub_markers = self.create_publisher(
            MarkerArray, "/quay_wall/markers", 10) if self.publish_markers else None
        self.pub_costmap = self.create_publisher(
            OccupancyGrid, "/quay_wall/costmap", 10) if self.publish_costmap else None

        self.create_subscription(
            PointCloud2, str(gp("input_topic")), self.cloud_callback,
            qos_profile_sensor_data)
        self.get_logger().info(
            f"quay_wall_detector: {gp('input_topic')} -> /quay_wall/* "
            f"(margin={self.safety_margin} m)")

    # ------------------------------------------------------------------
    def _segment_is_vertical(self, seg: wall_fit.WallSegment) -> bool:
        """Plane-normal gate: the smallest-variance direction of the inlier
        points must be near-horizontal (|normal_z| <= wall_normal_z_max) for a
        vertical wall face."""
        pts = seg.inlier_points
        if pts.shape[0] < 3:
            return True  # not enough data to judge; keep
        centered = pts - pts.mean(axis=0)
        # Guard: SVD of a degenerate set can fail; treat as non-vertical.
        try:
            _, _, vt = np.linalg.svd(centered, full_matrices=False)
        except np.linalg.LinAlgError:
            return False
        normal = vt[-1]
        return abs(float(normal[2])) <= self.normal_z_max

    def cloud_callback(self, msg: PointCloud2):
        xyz = _cloud_to_xyz(msg)
        stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        segments = []
        if xyz.shape[0] >= self.min_points:
            candidates = wall_fit.extract_wall_candidates(xyz)
            segments = wall_fit.fit_lines_ransac(
                xyz,
                distance_threshold_m=self.line_dist_thresh,
                min_points=self.min_points,
                min_length_m=self.min_length,
                max_distance_m=self.max_distance,
                candidate_mask=candidates,
                rng=self.rng)
            segments = [s for s in segments if self._segment_is_vertical(s)]

        confirmed = self.tracker.update(segments, stamp_sec)

        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = msg.header.frame_id or self.base_frame

        # Confirmed wall inlier points (base_link) — Nav2 obstacle-layer feed.
        if confirmed:
            wall_pts = np.vstack([s.inlier_points for s in confirmed])
        else:
            wall_pts = np.zeros((0, 3))
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        self.pub_points.publish(
            point_cloud2.create_cloud(header, fields, wall_pts.astype(np.float32)))

        if self.pub_markers is not None:
            self.pub_markers.publish(self._make_markers(header, confirmed))
        if self.pub_costmap is not None and confirmed:
            grid = self._make_costmap(msg.header.stamp, confirmed)
            if grid is not None:
                self.pub_costmap.publish(grid)

    # ------------------------------------------------------------------
    def _make_markers(self, header: Header, segments) -> MarkerArray:
        arr = MarkerArray()
        marker = Marker()
        marker.header = header
        marker.ns = "quay_wall"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.2
        marker.color.r = 1.0
        marker.color.g = 0.3
        marker.color.a = 1.0
        marker.lifetime = Duration(seconds=1.0).to_msg()
        for seg in segments:
            marker.points.append(
                Point(x=float(seg.p0[0]), y=float(seg.p0[1]), z=0.0))
            marker.points.append(
                Point(x=float(seg.p1[0]), y=float(seg.p1[1]), z=0.0))
        arr.markers.append(marker)
        return arr

    def _make_costmap(self, stamp, segments) -> OccupancyGrid | None:
        try:
            tf_msg = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, Time())
        except TransformException as e:
            self.get_logger().warning(
                f"TF {self.base_frame} -> {self.map_frame} unavailable, "
                f"skipping quay costmap: {e}", throttle_duration_sec=2.0)
            return None
        q = tf_msg.transform.rotation
        t = tf_msg.transform.translation
        rot2 = cloud_ops.quaternion_to_rotation_matrix(q.x, q.y, q.z, q.w)[:2, :2]
        center = np.array([t.x, t.y])

        # Grid-local endpoints: the grid is axis-aligned with the map frame
        # and centered on the current base position, so grid-local
        # = (map-frame point) - center = rot2 @ p_base.
        endpoint_pairs = [(rot2 @ seg.p0, rot2 @ seg.p1) for seg in segments]
        grid_data = wall_fit.segments_to_occupancy(
            endpoint_pairs,
            margin_m=self.safety_margin,
            resolution_m=self.costmap_resolution,
            size_m=self.costmap_size)

        grid = OccupancyGrid()
        grid.header.stamp = stamp
        grid.header.frame_id = self.map_frame
        grid.info.resolution = self.costmap_resolution
        grid.info.width = grid_data.shape[1]
        grid.info.height = grid_data.shape[0]
        grid.info.origin.position.x = float(center[0] - self.costmap_size / 2.0)
        grid.info.origin.position.y = float(center[1] - self.costmap_size / 2.0)
        grid.info.origin.orientation.w = 1.0
        grid.data = grid_data.ravel().tolist()
        return grid


def main(args=None):
    rclpy.init(args=args)
    node = QuayWallDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
