"""Convert fused cardinal-marker detections into Nav2 virtual obstacles."""

import math
import struct

from geometry_msgs.msg import PointStamped
from njord_interfaces.msg import BuoyDetection, BuoyDetectionArray
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import Buffer, TransformException, TransformListener
import tf2_geometry_msgs  # noqa: F401 - registers PointStamped conversions.

from buoy_obstacle_publisher.cardinal_wall_geometry import CARDINAL_DIRECTIONS, wall_points


CARDINAL_CLASSES = set(CARDINAL_DIRECTIONS)


class CardinalWallPublisher(Node):
    """Publish persistent map-frame walls for confirmed fused cardinal markers."""

    def __init__(self):
        super().__init__('cardinal_wall_publisher')
        self.declare_parameter('detection_topic', '/buoy_detections_3d')
        self.declare_parameter('output_topic', '/virtual_obstacles')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('course_bounds', [-100.0, 100.0, -100.0, 100.0])
        self.declare_parameter('wall_width_m', 0.2)
        self.declare_parameter('point_spacing_m', 0.1)
        self.declare_parameter('marker_merge_radius_m', 2.0)
        self.declare_parameter('confirmations_required', 2)
        self.declare_parameter('publish_rate_hz', 2.0)

        self.map_frame = self.get_parameter('map_frame').value
        self.bounds = list(self.get_parameter('course_bounds').value)
        if len(self.bounds) != 4 or self.bounds[0] >= self.bounds[1] or self.bounds[2] >= self.bounds[3]:
            raise ValueError('course_bounds must be [min_x, max_x, min_y, max_y]')
        self.wall_width = max(0.01, float(self.get_parameter('wall_width_m').value))
        self.spacing = max(0.02, float(self.get_parameter('point_spacing_m').value))
        self.merge_radius = max(0.05, float(self.get_parameter('marker_merge_radius_m').value))
        self.required_confirmations = max(1, int(self.get_parameter('confirmations_required').value))
        self.tracks = []

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(PointCloud2, self.get_parameter('output_topic').value, 10)
        self.create_subscription(
            BuoyDetectionArray, self.get_parameter('detection_topic').value, self._on_detections, 10)
        rate = max(0.2, float(self.get_parameter('publish_rate_hz').value))
        self.create_timer(1.0 / rate, self.publish_walls)

    def _on_detections(self, msg):
        for detection in msg.detections:
            if detection.class_id not in CARDINAL_CLASSES:
                continue
            if detection.position_source == BuoyDetection.POSITION_NONE:
                continue
            if not math.isfinite(detection.position.x) or not math.isfinite(detection.position.y):
                continue
            point = PointStamped()
            point.header = msg.header
            point.point = detection.position
            try:
                mapped = self.tf_buffer.transform(point, self.map_frame, timeout=Duration(seconds=0.1))
            except TransformException as error:
                self.get_logger().debug(f'Cannot transform cardinal detection: {error}')
                continue
            self._record_detection(mapped.point.x, mapped.point.y, detection.class_id)

    def _record_detection(self, x, y, class_id):
        nearest = None
        nearest_distance = self.merge_radius
        for track in self.tracks:
            distance = math.hypot(track['x'] - x, track['y'] - y)
            if distance <= nearest_distance:
                nearest = track
                nearest_distance = distance
        if nearest is None:
            nearest = {'x': x, 'y': y, 'candidate': class_id, 'count': 1, 'class_id': None}
            self.tracks.append(nearest)
        elif nearest['class_id'] is None:
            if nearest['candidate'] == class_id:
                nearest['count'] += 1
            else:
                nearest['candidate'] = class_id
                nearest['count'] = 1
        if nearest['class_id'] is None and nearest['count'] >= self.required_confirmations:
            nearest['class_id'] = nearest['candidate']
            self.get_logger().info(
                f"Confirmed cardinal marker {nearest['class_id']} at ({nearest['x']:.2f}, {nearest['y']:.2f})")

    def publish_walls(self):
        points = []
        for track in self.tracks:
            if track['class_id'] is not None:
                points.extend(wall_points(
                    self.bounds, self.wall_width, self.spacing,
                    track['x'], track['y'], track['class_id']))
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.height = 1
        msg.width = len(points)
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        msg.data = b''.join(struct.pack('fff', *point) for point in points)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CardinalWallPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
