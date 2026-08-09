"""Convert fused cardinal-marker detections into Nav2 virtual obstacles."""

import json
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
from std_msgs.msg import Bool
from std_msgs.msg import Float64
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
CARDINAL_CLASS_BY_LABEL = {
    'N': BuoyDetection.CLASS_NORTH,
    'E': BuoyDetection.CLASS_EAST,
    'S': BuoyDetection.CLASS_SOUTH,
    'W': BuoyDetection.CLASS_WEST,
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
        self.declare_parameter('true_north_confirmations_required', 10)
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('true_north_yaw_rad', math.pi / 2.0)
        self.declare_parameter('retirement_course_heading_rad', float('nan'))
        self.declare_parameter('retirement_frontier_topic', '')
        self.declare_parameter('retirement_margin_m', 0.5)
        self.declare_parameter('retirement_confirmations_required', 3)
        self.declare_parameter('retire_passed_cardinal_walls_from_base_pose', False)
        self.declare_parameter('retirement_heading_topic', '')
        self.declare_parameter('wall_enable_topic', '')
        # Simulation-only ground-truth preview. It is visualization-only and
        # is deliberately never included in /virtual_obstacles.
        self.declare_parameter('preview_buoy_positions', '[]')
        self.declare_parameter('preview_buoy_marks', '[]')

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
        self.true_north_confirmations_required = max(
            1, int(self.get_parameter('true_north_confirmations_required').value))
        self.base_frame_id = self.get_parameter('base_frame_id').value
        self.retirement_margin = max(0.0, float(self.get_parameter('retirement_margin_m').value))
        self.retirement_confirmations_required = max(
            1, int(self.get_parameter('retirement_confirmations_required').value))
        self.retire_passed_cardinal_walls_from_base_pose = self.get_parameter(
            'retire_passed_cardinal_walls_from_base_pose').value
        self.tracks = []
        self.preview_tracks = self._load_preview_tracks(
            self.get_parameter('preview_buoy_positions').value,
            self.get_parameter('preview_buoy_marks').value)
        wall_enable_topic = self.get_parameter('wall_enable_topic').value
        true_north_topic = self.get_parameter('true_north_heading_topic').value
        # Deployments without a task-stage signal retain the legacy behavior.
        self.walls_enabled = not bool(wall_enable_topic)
        # A configured GNSS heading source is authoritative.  Do not freeze
        # the default map +Y value into a cardinal track before its first
        # map-frame heading estimate is available.
        self.has_true_north_heading = not bool(true_north_topic)
        self._true_north_samples = 0
        self._true_north_sin_sum = 0.0
        self._true_north_cos_sum = 0.0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(PointCloud2, self.get_parameter('output_topic').value, 10)
        self.marker_pub = self.create_publisher(MarkerArray, self.get_parameter('marker_topic').value, 10)
        self.create_subscription(
            BuoyDetectionArray, self.get_parameter('detection_topic').value, self._on_detections, 10)
        retirement_topic = self.get_parameter('retirement_frontier_topic').value
        if retirement_topic:
            self.create_subscription(PointStamped, retirement_topic, self._on_retirement_frontier, 10)
        if wall_enable_topic:
            self.create_subscription(Bool, wall_enable_topic, self._on_wall_enable, 10)
        heading_topic = self.get_parameter('retirement_heading_topic').value
        if heading_topic:
            self.create_subscription(Float64, heading_topic, self._on_retirement_heading, 10)
        if true_north_topic:
            self.create_subscription(PoseWithCovarianceStamped, true_north_topic, self._on_true_north_heading, 10)
        rate = max(0.2, float(self.get_parameter('publish_rate_hz').value))
        self.create_timer(1.0 / rate, self.publish_walls)

    @staticmethod
    def _load_preview_tracks(positions_value, marks_value):
        """Read optional simulator ground truth without affecting planning."""
        try:
            positions = json.loads(positions_value) if isinstance(positions_value, str) else positions_value
            marks = json.loads(marks_value) if isinstance(marks_value, str) else marks_value
        except (TypeError, json.JSONDecodeError):
            return []
        if not isinstance(positions, list) or not isinstance(marks, list):
            return []
        preview = []
        for index, position in enumerate(positions):
            if not isinstance(position, (list, tuple)) or len(position) < 2:
                continue
            class_id = CARDINAL_CLASS_BY_LABEL.get(str(marks[index]).upper()) if index < len(marks) else None
            if class_id is None:
                continue
            try:
                preview.append((float(position[0]), float(position[1]), class_id))
            except (TypeError, ValueError):
                continue
        return preview

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

    def _on_wall_enable(self, msg):
        """Gate planning walls by the Task1 GPS3 completion signal."""
        if msg.data and not self.walls_enabled:
            self.walls_enabled = True
            self.get_logger().info('GPS3 reached: enabling confirmed buoy virtual walls')
        elif not msg.data and self.walls_enabled:
            # A new Mission Manager execution starts with false.  Do not let
            # confirmed positions from a previous run become obstacles in the
            # next run before its GPS3 gate is crossed.
            self.walls_enabled = False
            self.tracks.clear()
            self.get_logger().info('Task1 wall gate reset: cleared prior virtual-wall tracks')

    def _on_retirement_heading(self, msg):
        """Accept the map-frame GPS3->4 heading calculated from route points."""
        if math.isfinite(msg.data):
            self.retirement_course_heading_rad = msg.data

    def _on_retirement_frontier(self, msg):
        """Disable walls behind a reached waypoint while retaining their tracks."""
        if not self.walls_enabled:
            return
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
        sample = math.atan2(
            math.sin(map_body_yaw + math.pi / 2.0 - compass_yaw),
            math.cos(map_body_yaw + math.pi / 2.0 - compass_yaw))
        if not self.has_true_north_heading:
            # GPS1->GPS3 is the settling interval.  Circular averaging avoids
            # both angle-wrap errors and freezing a single noisy GNSS sample
            # into every cardinal wall for the rest of the task.
            self._true_north_sin_sum += math.sin(sample)
            self._true_north_cos_sum += math.cos(sample)
            self._true_north_samples += 1
            if self._true_north_samples < self.true_north_confirmations_required:
                return
            self.true_north_yaw_rad = math.atan2(
                self._true_north_sin_sum, self._true_north_cos_sum)
            self.has_true_north_heading = True
            self.get_logger().info(
                'Locked map-frame true-north heading from '
                f'{self._true_north_samples} GNSS samples; cardinal walls may now confirm')

    def _record_detection(self, x, y, class_id):
        if class_id in CARDINAL_DIRECTIONS and not self.has_true_north_heading:
            # Cardinal N/E/S/W sides depend on true north.  Red/green walls
            # use the GPS3->GPS4 route heading and remain independent.
            return
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
                'passed_samples': 0,
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
        # GPS3 is the start of the cardinal course.  Before that gate, the
        # boat position must not retire future course walls.
        if self.walls_enabled:
            self._retire_passed_cardinal_walls_from_base_pose()
        points = []
        for track in self.tracks:
            if (self.walls_enabled and track['class_id'] is not None
                    and track.get('wall_active', True)):
                wall_heading = self._wall_heading(track['class_id'], track)
                points.extend(wall_points(
                    self.bounds, self.wall_width, self.spacing,
                    track['x'], track['y'], track['class_id'],
                    wall_heading))
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

    def _wall_heading(self, class_id, track=None):
        """Return the reference direction used by planning and visualization."""
        if class_id in CARDINAL_DIRECTIONS:
            return (track or {}).get('true_north_yaw_rad', self.true_north_yaw_rad)
        # Lateral buoy sides are port/starboard relative to GPS3 -> GPS4.
        return self.retirement_course_heading_rad

    def _retire_passed_cardinal_walls_from_base_pose(self):
        """Retire each buoy wall once the hull crosses its parallel plane.

        The plane is perpendicular to the GPS3->4 course direction.  This
        preserves walls for upcoming marks while removing only a passed buoy,
        even when the boat takes a lateral avoidance path.
        """
        if not self.walls_enabled or not self.retire_passed_cardinal_walls_from_base_pose:
            return
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame_id, Time(), timeout=Duration(seconds=0.05))
        except TransformException:
            return
        boat_x = transform.transform.translation.x
        boat_y = transform.transform.translation.y
        forward_x = math.cos(self.retirement_course_heading_rad)
        forward_y = math.sin(self.retirement_course_heading_rad)
        retired = 0
        for track in self.tracks:
            if not track.get('wall_active', True):
                continue
            passed_distance = ((boat_x - track['x']) * forward_x +
                               (boat_y - track['y']) * forward_y)
            if passed_distance > self.retirement_margin:
                track['passed_samples'] = track.get('passed_samples', 0) + 1
                if track['passed_samples'] >= self.retirement_confirmations_required:
                    track['wall_active'] = False
                    retired += 1
            else:
                # A one-cycle TF/localization jump must not retire a wall.
                track['passed_samples'] = 0
        if retired:
            self.get_logger().info(
                f'Disabled {retired} passed-buoy wall(s) after '
                f'{self.retirement_confirmations_required} confirmations')

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
        for index, (x, y, class_id) in enumerate(self.preview_tracks):
            # A confirmed track replaces its nearby ground-truth preview.
            confirmed = any(
                track.get('class_id') == class_id
                and math.hypot(track['x'] - x, track['y'] - y) <= self.merge_radius
                for track in self.tracks)
            if confirmed:
                continue

            if self.walls_enabled:
                preview_wall = Marker()
                preview_wall.header.frame_id = self.map_frame
                preview_wall.header.stamp = stamp
                preview_wall.ns = 'preview_virtual_obstacle_wall'
                preview_wall.id = index
                preview_wall.type = Marker.POINTS
                preview_wall.action = Marker.ADD
                preview_wall.pose.orientation.w = 1.0
                preview_wall.scale.x = self.wall_width + 0.04
                preview_wall.scale.y = self.wall_width + 0.04
                preview_wall.color.r = 0.65
                preview_wall.color.g = 0.20
                preview_wall.color.b = 0.85
                preview_wall.color.a = 0.22
                for px, py, pz in wall_points(
                        self.bounds, self.wall_width, self.spacing, x, y,
                        class_id, self._wall_heading(class_id)):
                    preview_wall.points.append(Point(x=px, y=py, z=pz + 0.04))
                markers.markers.append(preview_wall)

            preview_text = Marker()
            preview_text.header.frame_id = self.map_frame
            preview_text.header.stamp = stamp
            preview_text.ns = 'preview_cardinal_buoy_label'
            preview_text.id = index
            preview_text.type = Marker.TEXT_VIEW_FACING
            preview_text.action = Marker.ADD
            preview_text.pose.position.x = x
            preview_text.pose.position.y = y
            preview_text.pose.position.z = 1.10
            preview_text.pose.orientation.w = 1.0
            preview_text.scale.z = 0.30
            preview_text.color.r = 0.85
            preview_text.color.g = 0.85
            preview_text.color.b = 0.85
            preview_text.color.a = 0.55
            state = 'NOT DETECTED' if self.walls_enabled else 'WAITING GPS3'
            preview_text.text = f'{CLASS_LABELS[class_id]} / {state}'
            markers.markers.append(preview_text)

        for index, track in enumerate(self.tracks):
            class_id = track.get('class_id')
            if class_id is None:
                continue
            colour = CLASS_COLOURS.get(class_id, (1.0, 1.0, 1.0))
            label = CLASS_LABELS.get(class_id, str(class_id))
            if class_id in CARDINAL_DIRECTIONS:
                label = f'{label} / CARDINAL'
            if not self.walls_enabled:
                label = f'{label} / WAITING GPS3'
            if not track.get('wall_active', True):
                label = f'{label} / WALL RETIRED'

            if self.walls_enabled and track.get('wall_active', True):
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
                        self._wall_heading(class_id, track)):
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
