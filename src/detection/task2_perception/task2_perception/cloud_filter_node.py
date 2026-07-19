#!/usr/bin/env python3
"""Task 2 cloud filter node (node name: task2_cloud_filter).

/livox/lidar (livox_frame) -> [optional emergency manual pre-rotation]
-> TF to base_link -> nonfinite / range / self crop-box / object height band
/ water removal (z band + guarded RANSAC) / optional voxel
-> /task2/points_filtered (frame = base_link, xyz + intensity).

The upside-down LiDAR correction lives ONLY in the URDF (lidar_joint,
roll = pi). This node just looks the transform up via TF. The
lidar_inverted / lidar_*_deg parameters are an emergency manual pre-rotation
applied BEFORE TF and default to a no-op; a startup warning fires when they
are active because the URDF TF is the authoritative inversion correction.

Voxel downsampling defaults OFF and accumulation to 1 frame because the
downstream pcl_preprocessing node (pcl_segmentation submodule) voxelizes and
accumulates itself.
"""

from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time

from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformListener

try:
    from tf2_ros import TransformException
except ImportError:  # older tf2_ros
    from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
    TransformException = (LookupException, ConnectivityException, ExtrapolationException)

from task2_perception import cloud_ops


def cloud_to_array(msg: PointCloud2) -> np.ndarray:
    """PointCloud2 -> (N, 4) float array [x, y, z, intensity].

    Missing intensity fields are filled with zeros so the pipeline shape is
    stable.
    """
    field_names = [f.name for f in msg.fields]
    want = ["x", "y", "z"]
    has_intensity = "intensity" in field_names
    if has_intensity:
        want.append("intensity")
    data = point_cloud2.read_points(msg, field_names=want, skip_nans=False)
    arr = np.asarray(data)
    if arr.dtype.names:  # structured array (Humble and later)
        stacked = np.column_stack(
            [np.asarray(arr[n], dtype=np.float64) for n in want])
    else:  # generator fallback (older sensor_msgs_py)
        stacked = np.asarray([list(p) for p in data], dtype=np.float64)
        if stacked.size == 0:
            stacked = stacked.reshape(0, len(want))
    if not has_intensity:
        stacked = np.column_stack(
            [stacked, np.zeros(stacked.shape[0], dtype=np.float64)])
    return stacked


def array_to_cloud(header: Header, points: np.ndarray) -> PointCloud2:
    """(N, 4) [x, y, z, intensity] -> PointCloud2."""
    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12,
                   datatype=PointField.FLOAT32, count=1),
    ]
    return point_cloud2.create_cloud(
        header, fields, np.asarray(points, dtype=np.float32))


def transform_msg_to_matrix(tf_msg) -> np.ndarray:
    q = tf_msg.transform.rotation
    t = tf_msg.transform.translation
    rot = cloud_ops.quaternion_to_rotation_matrix(q.x, q.y, q.z, q.w)
    return cloud_ops.make_transform(rot, [t.x, t.y, t.z])


class CloudFilterNode(Node):

    def __init__(self):
        super().__init__("task2_cloud_filter")

        self.declare_parameters("", [
            ("input_topic", "/livox/lidar"),
            ("output_topic", "/task2/points_filtered"),
            ("output_frame", "base_link"),
            ("min_range_m", 0.5),
            ("max_range_m", 60.0),
            ("voxel_leaf_size_m", 0.0),
            ("accumulation_frames", 1),
            ("lidar_inverted", False),
            ("lidar_roll_deg", 0.0),
            ("lidar_pitch_deg", 0.0),
            ("lidar_yaw_deg", 0.0),
            ("waterline_z_m", 0.0),
            ("water_remove_min_z_m", -0.3),
            ("water_remove_max_z_m", 0.15),
            ("water_plane_distance_threshold_m", 0.05),
            ("water_plane_normal_z_min", 0.9),
            ("water_plane_max_height_error_m", 0.3),
            ("use_water_plane_ransac", True),
            ("self_crop_min_x", -1.2),
            ("self_crop_max_x", 1.2),
            ("self_crop_min_y", -0.8),
            ("self_crop_max_y", 0.8),
            ("self_crop_min_z", -0.5),
            ("self_crop_max_z", 1.5),
            ("object_min_z_m", -0.2),
            ("object_max_z_m", 4.0),
            ("publish_debug", False),
        ])
        gp = lambda name: self.get_parameter(name).value  # noqa: E731

        self.output_frame = str(gp("output_frame"))
        self.min_range = float(gp("min_range_m"))
        self.max_range = float(gp("max_range_m"))
        self.voxel_leaf = float(gp("voxel_leaf_size_m"))
        self.lidar_inverted = bool(gp("lidar_inverted"))
        roll_deg = float(gp("lidar_roll_deg"))
        pitch_deg = float(gp("lidar_pitch_deg"))
        yaw_deg = float(gp("lidar_yaw_deg"))
        if self.lidar_inverted:
            roll_deg += 180.0
        self.manual_prerotation_active = (
            abs(roll_deg) > 1e-9 or abs(pitch_deg) > 1e-9 or abs(yaw_deg) > 1e-9)
        self.pre_transform = cloud_ops.make_transform(
            cloud_ops.rpy_to_rotation_matrix(
                np.deg2rad(roll_deg), np.deg2rad(pitch_deg), np.deg2rad(yaw_deg)),
            np.zeros(3))
        if self.manual_prerotation_active:
            self.get_logger().warning(
                "Manual LiDAR pre-rotation is ACTIVE "
                f"(inverted={self.lidar_inverted}, rpy_deg=({roll_deg}, "
                f"{pitch_deg}, {yaw_deg})). URDF TF is the authoritative "
                "inversion correction — do not double-apply.")

        self.waterline_z = float(gp("waterline_z_m"))
        self.water_band_min = float(gp("water_remove_min_z_m"))
        self.water_band_max = float(gp("water_remove_max_z_m"))
        self.water_dist_thresh = float(gp("water_plane_distance_threshold_m"))
        self.water_normal_z_min = float(gp("water_plane_normal_z_min"))
        self.water_max_height_err = float(gp("water_plane_max_height_error_m"))
        self.use_water_ransac = bool(gp("use_water_plane_ransac"))
        self.self_min = np.array([float(gp("self_crop_min_x")),
                                  float(gp("self_crop_min_y")),
                                  float(gp("self_crop_min_z"))])
        self.self_max = np.array([float(gp("self_crop_max_x")),
                                  float(gp("self_crop_max_y")),
                                  float(gp("self_crop_max_z"))])
        self.object_min_z = float(gp("object_min_z_m"))
        self.object_max_z = float(gp("object_max_z_m"))
        self.publish_debug = bool(gp("publish_debug"))

        self.accumulator = deque(maxlen=max(1, int(gp("accumulation_frames"))))
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.rng = np.random.default_rng(0)

        self.pub = self.create_publisher(PointCloud2, str(gp("output_topic")), 10)
        if self.publish_debug:
            self.pub_raw = self.create_publisher(
                PointCloud2, "/task2/debug/raw_transformed", 1)
            self.pub_self = self.create_publisher(
                PointCloud2, "/task2/debug/self_filtered", 1)
            self.pub_water = self.create_publisher(
                PointCloud2, "/task2/debug/water_removed", 1)
            self.pub_rejected = self.create_publisher(
                PointCloud2, "/task2/debug/rejected_water", 1)

        self.sub = self.create_subscription(
            PointCloud2, str(gp("input_topic")), self.cloud_callback,
            qos_profile_sensor_data)

        self.get_logger().info(
            f"task2_cloud_filter: {gp('input_topic')} -> {gp('output_topic')} "
            f"(frame={self.output_frame}, waterline_z={self.waterline_z} m)")

    # ------------------------------------------------------------------
    def _lookup(self, target: str, source: str, stamp) -> np.ndarray | None:
        try:
            tf_msg = self.tf_buffer.lookup_transform(target, source, stamp)
        except TransformException:
            try:  # fall back to the latest available transform
                tf_msg = self.tf_buffer.lookup_transform(target, source, Time())
                self.get_logger().warning(
                    f"TF {source} -> {target} at cloud stamp unavailable; "
                    "using the latest transform.", throttle_duration_sec=5.0)
            except TransformException as e:
                self.get_logger().warning(
                    f"TF {source} -> {target} unavailable, dropping cloud: {e}",
                    throttle_duration_sec=2.0)
                return None
        return transform_msg_to_matrix(tf_msg)

    def cloud_callback(self, msg: PointCloud2):
        transform = self._lookup(
            self.output_frame, msg.header.frame_id, msg.header.stamp)
        if transform is None:
            return

        points = cloud_to_array(msg)
        # NaN/Inf removal first so the transforms never propagate garbage.
        points = cloud_ops.remove_nonfinite(points)
        if points.shape[0] == 0:
            return

        # Emergency manual pre-rotation (BEFORE TF); no-op by default.
        if self.manual_prerotation_active:
            points = cloud_ops.apply_transform(points, self.pre_transform)
        points = cloud_ops.apply_transform(points, transform)

        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = self.output_frame

        if self.publish_debug:
            self.pub_raw.publish(array_to_cloud(header, points))

        # Range band (measured from base_link; the small lidar mount offset is
        # negligible against the 60 m working range).
        points = cloud_ops.range_filter(points, self.min_range, self.max_range)
        # Self crop: remove own hull / deck / mast returns.
        points = cloud_ops.crop_box_remove(points, self.self_min, self.self_max)
        if self.publish_debug:
            self.pub_self.publish(array_to_cloud(header, points))
        # Object height band keep.
        points = cloud_ops.crop_box_keep(
            points,
            [-np.inf, -np.inf, self.object_min_z],
            [np.inf, np.inf, self.object_max_z])
        # Water removal: z band around the waterline + guarded RANSAC plane.
        points, rejected_water = cloud_ops.water_removal(
            points,
            waterline_z_m=self.waterline_z,
            band_min_m=self.water_band_min,
            band_max_m=self.water_band_max,
            use_ransac=self.use_water_ransac,
            distance_threshold_m=self.water_dist_thresh,
            normal_z_min=self.water_normal_z_min,
            max_height_error_m=self.water_max_height_err,
            rng=self.rng)
        if self.publish_debug:
            self.pub_water.publish(array_to_cloud(header, points))
            self.pub_rejected.publish(array_to_cloud(header, rejected_water))

        # Optional voxel downsampling (default off: the downstream submodule
        # pcl_preprocessing voxelizes at 0.1 m itself).
        points = cloud_ops.voxel_downsample(points, self.voxel_leaf)

        # Optional short accumulation (default 1 = passthrough; the submodule
        # accumulates 3 frames downstream).
        self.accumulator.append(points)
        if self.accumulator.maxlen > 1:
            points = np.vstack(list(self.accumulator))

        self.pub.publish(array_to_cloud(header, points))


def main(args=None):
    rclpy.init(args=args)
    node = CloudFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
