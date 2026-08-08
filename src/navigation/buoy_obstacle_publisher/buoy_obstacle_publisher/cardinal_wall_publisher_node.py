"""Convert fused cardinal-marker detections into Nav2 virtual obstacles."""

import math
import struct

from geometry_msgs.msg import Point, PointStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from njord_interfaces.msg import BuoyDetection, BuoyDetectionArray
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import Buffer, TransformException, TransformListener
import tf2_geometry_msgs  # noqa: F401 - registers PointStamped conversions.
from visualization_msgs.msg import Marker, MarkerArray

from buoy_obstacle_publisher.cardinal_wall_geometry import (
    CARDINAL_DIRECTIONS,
    is_behind_retirement_frontier,
    wall_points,
)


WALL_CLASSES = set(CARDINAL_DIRECTIONS) | {BuoyDetection.CLASS_GREEN, BuoyDetection.CLASS_RED}
CLASS_LABELS = {
    BuoyDetection.CLASS_GREEN: 'GREEN / STARBOARD',
    BuoyDetection.CLASS_RED: 'RED / PORT',
    BuoyDetection.CLASS_NORTH: 'N',
    BuoyDetection.CLASS_EAST: 'E',
    BuoyDetection.CLASS_SOUTH: 'S',
    BuoyDetection.CLASS_WEST: 'W',
}
CLASS_COLOURS = {
    BuoyDetection.CLASS_GREEN: (0.0, 0.53, 0.28),
    BuoyDetection.CLASS_RED: (0.80, 0.03, 0.12),
    BuoyDetection.CLASS_NORTH: (1.0, 0.78, 0.0),
    BuoyDetection.CLASS_EAST: (1.0, 0.78, 0.0),
    BuoyDetection.CLASS_SOUTH: (1.0, 0.78, 0.0),
    BuoyDetection.CLASS_WEST: (1.0, 0.78, 0.0),
}


class CardinalWallPublisher(Node):
    """Publish persistent map-frame walls for confirmed fused cardinal markers."""

    def __init__(self):
        super().__init__('cardinal_wall_publisher')
        self.declare_parameter('detection_topic', '/buoy_detections_3d')
        self.declare_parameter('output_topic', '/virtual_obstacles')
        self.declare_parameter('marker_topic', '/virtual_obstacle_markers')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('course_bounds', [-100.0, 100.0, -100.0, 100.0])
        self.declare_parameter('wall_width_m', 0.2)
        self.declare_parameter('point_spacing_m', 0.1)
        self.declare_parameter('marker_merge_radius_m', 2.0)
        self.declare_parameter('confirmations_required', 2)
        self.declare_parameter('publish_rate_hz', 2.0)
        self.declare_parameter('course_heading_rad', 0.0)
        self.declare_parameter('true_north_heading_topic', '')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('true_north_yaw_rad', math.pi / 2.0)
        self.declare_parameter('retirement_course_heading_rad', float('nan'))
        self.declare_parameter('retirement_frontier_topic', '')
        self.declare_parameter('retirement_margin_m', 0.5)

        self.map_frame = self.get_parameter('map_frame').value
        self.bounds = list(self.get_parameter('course_bounds').value)
        if len(self.bounds) != 4 or self.bounds[0] >= self.bounds[1] or self.bounds[2] >= self.bounds[3]:
            raise ValueError('course_bounds must be [min_x, max_x, min_y, max_y]')
        self.wall_width = max(0.01, float(self.get_parameter('wall_width_m').value))
        self.spacing = max(0.02, float(self.get_parameter('point_spacing_m').value))
        self.merge_radius = max(0.05, float(self.get_parameter('marker_merge_radius_m').value))
        self.required_confirmations = max(1, int(self.get_parameter('confirmations_required').value))
        self.course_heading_rad = float(self.get_parameter('course_heading_rad').value)
        retirement_heading = float(self.get_parameter('retirement_course_heading_rad').value)
        self.retirement_course_heading_rad = (
            self.course_heading_rad if not math.isfinite(retirement_heading) else retirement_heading)
        self.true_north_yaw_rad = float(self.get_parameter('true_north_yaw_rad').value)
        self.base_frame_id = self.get_parameter('base_frame_id').value
        self.retirement_margin = max(0.0, float(self.get_parameter('retirement_margin_m').value))
        self.tracks = []

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(PointCloud2, self.get_parameter('output_topic').value, 10)
        self.marker_pub = self.create_publisher(MarkerArray, self.get_parameter('marker_topic').value, 10)
        self.create_subscription(
            BuoyDetectionArray, self.get_parameter('detection_topic').value, self._on_detections, 10)
        retirement_topic = self.get_parameter('retirement_frontier_topic').value
        if retirement_topic:
            self.create_subscription(PointStamped, retirement_topic, self._on_retirement_frontier, 10)
        true_north_topic = self.get_parameter('true_north_heading_topic').value
        if true_north_topic:
            self.create_subscription(PoseWithCovarianceStamped, true_north_topic, self._on_true_north_heading, 10)
        rate = max(0.2, float(self.get_parameter('publish_rate_hz').value))
        self.create_timer(1.0 / rate, self.publish_walls)

    def _on_detections(self, msg):
        for detection in msg.detections:
            if detection.class_id not in WALL_CLASSES:
                continue
            if detection.position_source == BuoyDetection.POSITION_NONE:
                continue
            if not math.isfinite(detection.position.x) or not math.isfinite(detection.position.y):
                continue
            point = PointStamped()
            point.header.frame_id = msg.header.frame_id
            # Request the latest available transform (Time() == time zero)
            # instead of msg.header.stamp. This callback runs on the same
            # single-threaded executor as the TF listener, so a lookup for
            # an exact recent stamp can never be satisfied: the /tf message
            # that would complete it can't be processed until this callback
            # returns, and the wait always times out as "extrapolation into
            # the future" even though the requested time is only
            # milliseconds ahead. Static cardinal markers don't need
            # sub-frame timing precision, so the latest transform is fine.
            point.header.stamp = Time().to_msg()
            point.point = detection.position
            try:
                mapped = self.tf_buffer.transform(point, self.map_frame, timeout=Duration(seconds=0.2))
            except TransformException as error:
                self.get_logger().debug(f'Cannot transform cardinal detection: {error}')
                continue
            self._record_detection(mapped.point.x, mapped.point.y, detection.class_id)

    def _on_retirement_frontier(self, msg):
        """Disable walls behind a reached waypoint while retaining their tracks."""
        point = msg
        if msg.header.frame_id and msg.header.frame_id != self.map_frame:
            try:
                point = self.tf_buffer.transform(msg, self.map_frame, timeout=Duration(seconds=0.2))
            except TransformException as error:
                self.get_logger().debug(f'Cannot transform retirement frontier: {error}')
                return
        retired = 0
        for track in self.tracks:
            behind_frontier = is_behind_retirement_frontier(
                track['x'], track['y'], point.point.x, point.point.y,
                self.retirement_course_heading_rad, self.retirement_margin)
            if behind_frontier and track.get('wall_active', True):
                track['wall_active'] = False
                retired += 1
        if retired:
            self.get_logger().info(
                f'Disabled {retired} passed-buoy virtual wall(s); detection tracks are retained')

    def _on_true_north_heading(self, msg):
        """Estimate map-frame true north from UM982 dual-antenna heading."""
        q = msg.pose.pose.orientation
        compass_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame_id, Time(), timeout=Duration(seconds=0.2))
        except TransformException as error:
            self.get_logger().debug(f'Cannot estimate true north from GNSS heading: {error}')
            return
        q_map = transform.transform.rotation
        map_body_yaw = math.atan2(
            2.0 * (q_map.w * q_map.z + q_map.x * q_map.y),
            1.0 - 2.0 * (q_map.y * q_map.y + q_map.z * q_map.z))
        # UM982 publishes body heading in ENU (east=0, true north=pi/2).
        # The difference aligns that true-north axis with the map frame.
        self.true_north_yaw_rad = math.atan2(
            math.sin(map_body_yaw + math.pi / 2.0 - compass_yaw),
            math.cos(map_body_yaw + math.pi / 2.0 - compass_yaw))

    def _record_detection(self, x, y, class_id):
        nearest = None
        nearest_distance = self.merge_radius
        for track in self.tracks:
            distance = math.hypot(track['x'] - x, track['y'] - y)
            if distance <= nearest_distance:
                nearest = track
                nearest_distance = distance
        if nearest is None:
            nearest = {
                'x': x, 'y': y, 'candidate': class_id, 'count': 1,
                'class_id': None, 'wall_active': True, 'true_north_yaw_rad': None,
            }
            self.tracks.append(nearest)
        elif nearest['class_id'] is None:
            if nearest['candidate'] == class_id:
                nearest['count'] += 1
            else:
                nearest['candidate'] = class_id
                nearest['count'] = 1
        if nearest['class_id'] is None and nearest['count'] >= self.required_confirmations:
            nearest['class_id'] = nearest['candidate']
            nearest['true_north_yaw_rad'] = self.true_north_yaw_rad
            self.get_logger().info(
                f"Confirmed cardinal marker {nearest['class_id']} at ({nearest['x']:.2f}, {nearest['y']:.2f})")

    def publish_walls(self):
        points = []
        for track in self.tracks:
            if track['class_id'] is not None and track.get('wall_active', True):
                points.extend(wall_points(
                    self.bounds, self.wall_width, self.spacing,
                    track['x'], track['y'], track['class_id'],
                    track.get('true_north_yaw_rad', self.true_north_yaw_rad)))
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
        self._publish_detection_markers(msg.header.stamp)

    def _publish_detection_markers(self, stamp):
        """Visualize every confirmed buoy in the map frame.

        This is intentionally driven by the same fused tracks as the virtual
        walls, so Foxglove/RViz shows the class that planning is actually
        using on the vessel, rather than a separate simulator-only annotation.
        """
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        for index, track in enumerate(self.tracks):
            class_id = track.get('class_id')
            if class_id is None:
                continue
            colour = CLASS_COLOURS.get(class_id, (1.0, 1.0, 1.0))
            label = CLASS_LABELS.get(class_id, str(class_id))
            if class_id in CARDINAL_DIRECTIONS:
                label = f'{label} / CARDINAL'
            if not track.get('wall_active', True):
                label = f'{label} / WALL RETIRED'

            if track.get('wall_active', True):
                # /virtual_obstacles itself is PointCloud2, which is not
                # shown by default in Foxglove/RViz. Mirror the exact wall
                # samples as a thick magenta POINTS marker so the obstacle
                # Nav2 receives is always visible without extra display setup.
                wall = Marker()
                wall.header.frame_id = self.map_frame
                wall.header.stamp = stamp
                wall.ns = 'virtual_obstacle_wall'
                wall.id = index
                wall.type = Marker.POINTS
                wall.action = Marker.ADD
                wall.pose.orientation.w = 1.0
                wall.scale.x = self.wall_width + 0.10
                wall.scale.y = self.wall_width + 0.10
                wall.color.r = 1.0
                wall.color.g = 0.0
                wall.color.b = 1.0
                wall.color.a = 0.85
                for x, y, z in wall_points(
                        self.bounds, self.wall_width, self.spacing,
                        track['x'], track['y'], class_id,
                        track.get('true_north_yaw_rad', self.true_north_yaw_rad)):
                    wall.points.append(Point(x=x, y=y, z=z + 0.08))
                markers.markers.append(wall)

            body = Marker()
            body.header.frame_id = self.map_frame
            body.header.stamp = stamp
            body.ns = 'detected_buoy_body'
            body.id = index
            body.type = Marker.CYLINDER
            body.action = Marker.ADD
            body.pose.position.x = track['x']
            body.pose.position.y = track['y']
            body.pose.position.z = 0.35
            body.pose.orientation.w = 1.0
            body.scale.x = 0.35
            body.scale.y = 0.35
            body.scale.z = 0.70
            body.color.r, body.color.g, body.color.b = colour
            body.color.a = 0.90
            markers.markers.append(body)

            text = Marker()
            text.header.frame_id = self.map_frame
            text.header.stamp = stamp
            text.ns = 'detected_buoy_label'
            text.id = index
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = track['x']
            text.pose.position.y = track['y']
            text.pose.position.z = 0.90
            text.pose.orientation.w = 1.0
            text.scale.z = 0.30
            text.color.r, text.color.g, text.color.b = colour
            text.color.a = 1.0
            text.text = label
            markers.markers.append(text)
        self.marker_pub.publish(markers)


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
