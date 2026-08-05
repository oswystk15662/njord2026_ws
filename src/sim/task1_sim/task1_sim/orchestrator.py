import json
import math
import random
import struct

from geometry_msgs.msg import Point
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Bool, ColorRGBA, String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray


CARDINAL_MARKS = ("N", "E", "S", "W")


def make_pointcloud2(frame_id: str, points: list[list[float]], stamp):
    msg = PointCloud2()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    msg.height = 1
    msg.width = len(points)
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = msg.point_step * len(points)
    msg.is_dense = True
    msg.data = b"".join(struct.pack("fff", p[0], p[1], p[2]) for p in points)
    return msg


def parse_xy_json(json_str: str) -> list[list[float]]:
    if not json_str:
        return []
    try:
        parsed = json.loads(json_str)
    except json.JSONDecodeError:
        return []

    points = []
    for point in parsed:
        if isinstance(point, list) and len(point) >= 2:
            points.append([float(point[0]), float(point[1])])
    return points


def parse_marks_json(json_str: str) -> list[str]:
    """Parse a JSON array of cardinal-mark letters, e.g. '["S", "N", "S"]'."""
    if not json_str:
        return []
    try:
        parsed = json.loads(json_str)
    except json.JSONDecodeError:
        return []
    if not isinstance(parsed, list):
        return []
    marks = []
    for value in parsed:
        if isinstance(value, str):
            marks.append(value.strip().upper())
    return marks


def sample_line(start: list[float], end: list[float], spacing: float) -> list[list[float]]:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    distance = math.hypot(dx, dy)
    steps = max(1, int(distance / max(spacing, 0.05)))
    points = []
    for i in range(steps + 1):
        t = i / float(steps)
        points.append([start[0] + dx * t, start[1] + dy * t, 0.0])
    return points


class Task1Orchestrator(Node):
    def __init__(self):
        super().__init__("task1_orchestrator")

        self.declare_parameter("frame_id", "map")
        self.declare_parameter("gps_checkpoint_xy", "[[0.0, 0.0], [50.0, 0.0], [50.0, -25.0], [0.0, -25.0]]")
        self.declare_parameter("waypoint1_xy", "[]")
        self.declare_parameter("waypoint2_xy", "[]")
        self.declare_parameter("buoy_position_xy", "[[28.0, -25.0], [18.0, -25.0], [11.0, -25.0]]")
        self.declare_parameter("buoy_marks", "[\"S\", \"N\", \"S\"]")
        self.declare_parameter("wall_radius", 2.5)
        self.declare_parameter("wall_points", 24)
        self.declare_parameter("goal_radius", 2.0)
        self.declare_parameter("waypoint_reach_radius", 2.0)
        self.declare_parameter("avoidance_eval_radius", 8.0)
        self.declare_parameter("avoidance_margin", 0.5)
        self.declare_parameter("forced_mark", "")
        self.declare_parameter("course_bounds", [-5.0, 55.0, -40.0, 35.0])
        self.declare_parameter("center_line", [0.0, 40.0, -10.0, 0.05])
        self.declare_parameter("pre_inference_block", "[]")
        self.declare_parameter("obstacle_spacing", 0.25)
        self.declare_parameter("boundary_marker_step", 5.0)
        self.declare_parameter("seed", 2026)
        self.declare_parameter("publish_rate_hz", 2.0)

        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self.gps_checkpoints = parse_xy_json(
            self.get_parameter("gps_checkpoint_xy").get_parameter_value().string_value
        )
        if len(self.gps_checkpoints) < 2:
            self.gps_checkpoints = [[0.0, 0.0], [25.0, 0.0]]

        self.waypoint1_xy = parse_xy_json(self.get_parameter("waypoint1_xy").get_parameter_value().string_value)
        self.waypoint2_xy = parse_xy_json(self.get_parameter("waypoint2_xy").get_parameter_value().string_value)
        self.buoy_positions = parse_xy_json(
            self.get_parameter("buoy_position_xy").get_parameter_value().string_value
        )
        if not self.buoy_positions:
            self.buoy_positions = [[10.0, 0.0]]

        self.wall_radius = self.get_parameter("wall_radius").get_parameter_value().double_value
        self.wall_points = max(6, self.get_parameter("wall_points").get_parameter_value().integer_value)
        self.goal_radius = self.get_parameter("goal_radius").get_parameter_value().double_value
        self.waypoint_reach_radius = self.get_parameter("waypoint_reach_radius").get_parameter_value().double_value
        self.avoidance_eval_radius = self.get_parameter("avoidance_eval_radius").get_parameter_value().double_value
        self.avoidance_margin = self.get_parameter("avoidance_margin").get_parameter_value().double_value
        self.forced_mark = self.get_parameter("forced_mark").get_parameter_value().string_value.strip().upper()

        # Per-marker cardinal orientation, aligned index-wise with
        # buoy_positions. forced_mark (if set) overrides every marker so
        # existing single-mark test tooling keeps working.
        if self.forced_mark in CARDINAL_MARKS:
            self.buoy_marks = [self.forced_mark for _ in self.buoy_positions]
        else:
            buoy_marks_raw = parse_marks_json(self.get_parameter("buoy_marks").get_parameter_value().string_value)
            self.buoy_marks = self._align_marks(buoy_marks_raw, len(self.buoy_positions))

        self.course_bounds = list(self.get_parameter("course_bounds").get_parameter_value().double_array_value)
        self.center_line = list(self.get_parameter("center_line").get_parameter_value().double_array_value)
        self.pre_inference_block = self._parse_rect_json(
            self.get_parameter("pre_inference_block").get_parameter_value().string_value
        )
        self.obstacle_spacing = max(0.05, self.get_parameter("obstacle_spacing").get_parameter_value().double_value)
        self.boundary_marker_step = max(
            0.5,
            self.get_parameter("boundary_marker_step").get_parameter_value().double_value,
        )
        seed = self.get_parameter("seed").get_parameter_value().integer_value
        self.publish_rate_hz = self.get_parameter("publish_rate_hz").get_parameter_value().double_value

        self.required_waypoints = self._build_required_waypoints()
        self.next_waypoint_idx = 0
        self.random = random.Random(seed)
        self.inference_done = False
        self.goal_announced = False
        self.avoidance_failed = False
        self.avoidance_results = []
        self._init_buoy_states()

        transient_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_start = self.create_publisher(Bool, "/sim/start", transient_qos)
        self.pub_goal = self.create_publisher(Bool, "/sim/goal_reached", transient_qos)
        self.pub_cardinal = self.create_publisher(String, "/sim/cardinal_mark", 10)
        # Fixed simulator geometry is distinct from perception-derived
        # cardinal walls, which own /virtual_obstacles.
        self.pub_sim_obstacles = self.create_publisher(PointCloud2, "/sim_obstacles", 10)
        self.pub_status = self.create_publisher(String, "/sim/task1_status", 10)
        self.pub_boundary_markers = self.create_publisher(MarkerArray, "/sim/boundary_markers", transient_qos)
        self.pub_cardinal_markers = self.create_publisher(
            MarkerArray,
            "/sim/cardinal_mark_markers",
            transient_qos,
        )

        self.sub_odom = self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.srv_infer = self.create_service(Trigger, "/yolo/start_inference", self.on_start_inference)

        self.publish_start()
        self.publish_sim_obstacles()
        self.publish_boundary_markers()
        self.publish_cardinal_markers()

        period = 1.0 / max(0.2, self.publish_rate_hz)
        self.timer = self.create_timer(period, self.on_timer)

    def _build_required_waypoints(self) -> list[list[float]]:
        required = []
        required.extend(self.waypoint1_xy)
        if len(self.gps_checkpoints) > 2:
            required.extend(self.gps_checkpoints[1:-1])
        required.extend(self.waypoint2_xy)
        return required

    def _align_marks(self, marks: list[str], count: int) -> list[str]:
        """Pad/truncate a parsed buoy_marks list to match buoy_positions,
        defaulting missing entries to "N" and dropping invalid letters."""
        aligned = []
        for i in range(count):
            mark = marks[i] if i < len(marks) else "N"
            aligned.append(mark if mark in CARDINAL_MARKS else "N")
        return aligned

    def _init_buoy_states(self):
        self.buoy_states = []
        for idx, buoy in enumerate(self.buoy_positions):
            self.buoy_states.append({
                "xy": buoy,
                "mark": self.buoy_marks[idx] if idx < len(self.buoy_marks) else "N",
                "in_zone": False,
                "evaluated": False,
                "closest_dist": float("inf"),
                "signed_at_closest": 0.0,
            })

    def _publish_status(self, text: str):
        msg = String()
        msg.data = text
        self.pub_status.publish(msg)
        self.get_logger().info(f"task1_status: {text}")

    def publish_start(self):
        start_msg = Bool()
        start_msg.data = True
        self.pub_start.publish(start_msg)
        self._publish_status("start_published")

    def on_start_inference(self, _request, response):
        # Marker orientation is fixed by course design (buoy_marks /
        # forced_mark), not chosen randomly here -- classification is simply
        # confirmed once inference runs.
        self.inference_done = True
        self._init_buoy_states()
        self.avoidance_failed = False
        self.avoidance_results = []
        self.publish_sim_obstacles()
        self.publish_cardinal_markers()

        response.success = True
        response.message = ",".join(self.buoy_marks)
        self._publish_status(f"marks={','.join(self.buoy_marks)}")
        return response

    def _signed_side_value(self, mark: str, x: float, y: float, bx: float, by: float) -> float:
        if mark == "N":
            return y - by
        if mark == "S":
            return by - y
        if mark == "E":
            return x - bx
        return bx - x

    def _append_course_boundary(self, points: list[list[float]]):
        min_x, max_x, min_y, max_y = self.course_bounds
        points.extend(sample_line([min_x, min_y], [max_x, min_y], self.obstacle_spacing))
        points.extend(sample_line([max_x, min_y], [max_x, max_y], self.obstacle_spacing))
        points.extend(sample_line([max_x, max_y], [min_x, max_y], self.obstacle_spacing))
        points.extend(sample_line([min_x, max_y], [min_x, min_y], self.obstacle_spacing))

    def _append_center_line(self, points: list[list[float]]):
        start_x, end_x, y, half_width = self.center_line
        y_offsets = [0.0]
        if half_width > 0.0:
            y_offsets.extend([-half_width, half_width])
        for y_offset in y_offsets:
            points.extend(sample_line([start_x, y + y_offset], [end_x, y + y_offset], self.obstacle_spacing))

    def _append_rect_fill(self, points: list[list[float]], rect: list[float]):
        if len(rect) != 4:
            return
        min_x, max_x, min_y, max_y = rect
        y = min_y
        while y <= max_y + 1.0e-6:
            x = min_x
            while x <= max_x + 1.0e-6:
                points.append([x, y, 0.0])
                x += self.obstacle_spacing
            y += self.obstacle_spacing

    def _parse_rect_json(self, json_str: str) -> list[float]:
        if not json_str:
            return []
        try:
            parsed = json.loads(json_str)
        except json.JSONDecodeError:
            return []
        if not isinstance(parsed, list) or len(parsed) != 4:
            return []
        try:
            return [float(value) for value in parsed]
        except (TypeError, ValueError):
            return []

    def build_sim_obstacle_points(self) -> list[list[float]]:
        points = []
        # The course boundary is a visual aid, not a physical wall. Including
        # it in /sim_obstacles split the global costmap at y=15 and made the
        # shared waypoint route impossible to plan through.
        self._append_center_line(points)
        return points

    def publish_sim_obstacles(self):
        stamp = self.get_clock().now().to_msg()
        points = self.build_sim_obstacle_points()
        self.pub_sim_obstacles.publish(make_pointcloud2(self.frame_id, points, stamp))

    def publish_boundary_markers(self):
        min_x, max_x, min_y, max_y = self.course_bounds
        marker_array = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        marker_id = 0

        def color(r, g, b, a=1.0):
            msg = ColorRGBA()
            msg.r = r
            msg.g = g
            msg.b = b
            msg.a = a
            return msg

        def add_pole(x, y, rgba):
            nonlocal marker_id
            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.header.stamp = stamp
            marker.ns = "task1_boundary"
            marker.id = marker_id
            marker_id += 1
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = float(x)
            marker.pose.position.y = float(y)
            marker.pose.position.z = 1.5
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 3.0
            marker.color = rgba
            marker_array.markers.append(marker)

        x = min_x
        while x <= max_x + 1.0e-6:
            add_pole(x, max_y, color(1.0, 0.1, 0.1))
            add_pole(x, min_y, color(0.1, 0.3, 1.0))
            x += self.boundary_marker_step

        y = min_y + self.boundary_marker_step
        while y < max_y - 1.0e-6:
            add_pole(min_x, y, color(0.1, 0.9, 0.2))
            add_pole(max_x, y, color(1.0, 0.85, 0.0))
            y += self.boundary_marker_step

        for bx, by in self.buoy_positions:
            add_pole(bx, by, color(0.0, 0.0, 0.0))

        self.pub_boundary_markers.publish(marker_array)

    def publish_cardinal_markers(self):
        marker_array = MarkerArray()
        clear_marker = Marker()
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)

        stamp = self.get_clock().now().to_msg()
        arrow_by_mark = {
            "N": (0.0, 1.0),
            "E": (1.0, 0.0),
            "S": (0.0, -1.0),
            "W": (-1.0, 0.0),
        }

        for idx, (bx, by) in enumerate(self.buoy_positions):
            mark = self.buoy_marks[idx] if idx < len(self.buoy_marks) else "N"
            dx, dy = arrow_by_mark.get(mark, (0.0, 1.0))
            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.header.stamp = stamp
            marker.ns = "task1_cardinal_arrow"
            marker.id = idx
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0

            start = Point()
            start.x = float(bx)
            start.y = float(by)
            start.z = 1.0
            end = Point()
            end.x = float(bx + dx * 3.0)
            end.y = float(by + dy * 3.0)
            end.z = 1.0
            marker.points = [start, end]

            marker.scale.x = 0.25
            marker.scale.y = 0.75
            marker.scale.z = 0.75
            marker.color.r = 1.0
            marker.color.g = 0.85
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker_array.markers.append(marker)

        self.pub_cardinal_markers.publish(marker_array)

    def on_odom(self, msg: Odometry):
        if self.goal_announced:
            return

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        if self.next_waypoint_idx < len(self.required_waypoints):
            wx, wy = self.required_waypoints[self.next_waypoint_idx]
            if math.hypot(x - wx, y - wy) <= self.waypoint_reach_radius:
                self.next_waypoint_idx += 1
                self._publish_status(f"waypoint_reached={self.next_waypoint_idx}/{len(self.required_waypoints)}")

        if self.inference_done:
            self._evaluate_buoy_avoidance(x, y)

        final_goal = self.gps_checkpoints[-1]
        all_waypoints_reached = self.next_waypoint_idx >= len(self.required_waypoints)
        all_buoys_evaluated = self.inference_done and all(state["evaluated"] for state in self.buoy_states)
        at_goal = math.hypot(x - final_goal[0], y - final_goal[1]) <= self.goal_radius

        if at_goal and all_waypoints_reached and all_buoys_evaluated and not self.avoidance_failed:
            done = Bool()
            done.data = True
            self.pub_goal.publish(done)
            self.goal_announced = True
            self.get_logger().info("Goal area reached with Task1 constraints")
            self._publish_status("goal_reached_with_task1_constraints")

    def _evaluate_buoy_avoidance(self, x: float, y: float):
        for idx, state in enumerate(self.buoy_states):
            if state["evaluated"]:
                continue

            bx, by = state["xy"]
            distance = math.hypot(x - bx, y - by)
            if distance <= self.avoidance_eval_radius:
                state["in_zone"] = True
                signed = self._signed_side_value(state["mark"], x, y, bx, by)
                if distance < state["closest_dist"]:
                    state["closest_dist"] = distance
                    state["signed_at_closest"] = signed
            elif state["in_zone"]:
                state["evaluated"] = True
                passed = state["signed_at_closest"] >= self.avoidance_margin
                if not passed:
                    self.avoidance_failed = True
                self.avoidance_results.append((idx, passed, state["signed_at_closest"]))
                self._publish_status(
                    f"avoidance_buoy={idx},pass={str(passed).lower()},signed={state['signed_at_closest']:.2f}"
                )

    def on_timer(self):
        # Published as a JSON array so each marker (buoy_positions[i]) keeps
        # its own cardinal orientation (buoy_marks[i]) instead of sharing one
        # mark across the whole course.
        cardinal = String()
        cardinal.data = json.dumps(self.buoy_marks)
        self.pub_cardinal.publish(cardinal)
        self.publish_sim_obstacles()
        self.publish_boundary_markers()
        self.publish_cardinal_markers()


def main(args=None):
    rclpy.init(args=args)
    node = Task1Orchestrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
