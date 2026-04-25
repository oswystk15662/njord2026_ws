import math
import random
import struct

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


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
    msg.row_step = 12 * len(points)
    msg.is_dense = True
    msg.data = b"".join(struct.pack("fff", p[0], p[1], p[2]) for p in points)
    return msg


class Task1Orchestrator(Node):
    def __init__(self):
        super().__init__("task1_orchestrator")

        self.declare_parameter("frame_id", "map")
        self.declare_parameter("buoy_xy", [10.0, 0.0])
        self.declare_parameter("buoy_position_xy", [10.0, 0.0])
        self.declare_parameter("bouy_position_xy", [10.0, 0.0])
        self.declare_parameter("wall_radius", 2.5)
        self.declare_parameter("wall_points", 24)
        self.declare_parameter("gps_checkpoint_xy", [0.0, 0.0, 50.0, 0.0, 50.0, -25.0, 0.0, -25.0])
        self.declare_parameter("waypoint1_xy", [10.0, 5.0, 40.0, 0.0])
        self.declare_parameter("waypoint2_xy", [24.0, -35.0, 8.0, -35.0])
        self.declare_parameter("goal_radius", 2.0)
        self.declare_parameter("waypoint_reach_radius", 2.0)
        self.declare_parameter("avoidance_eval_radius", 8.0)
        self.declare_parameter("avoidance_margin", 0.5)
        self.declare_parameter("forced_mark", "")
        self.declare_parameter("seed", 2026)
        self.declare_parameter("publish_rate_hz", 2.0)

        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self.buoy_xy = list(self.get_parameter("buoy_xy").get_parameter_value().double_array_value)
        self.wall_radius = self.get_parameter("wall_radius").get_parameter_value().double_value
        self.wall_points = max(6, self.get_parameter("wall_points").get_parameter_value().integer_value)
        self.gps_checkpoint_xy = self._parse_xy_list(
            self.get_parameter("gps_checkpoint_xy").get_parameter_value().double_array_value
        )
        if len(self.gps_checkpoint_xy) == 0:
            self.gps_checkpoint_xy = [[0.0, 0.0], [25.0, 0.0]]
        self.waypoint1_xy = self._parse_xy_list(
            self.get_parameter("waypoint1_xy").get_parameter_value().double_array_value
        )
        self.waypoint2_xy = self._parse_xy_list(
            self.get_parameter("waypoint2_xy").get_parameter_value().double_array_value
        )
        self.goal_radius = self.get_parameter("goal_radius").get_parameter_value().double_value
        self.waypoint_reach_radius = self.get_parameter("waypoint_reach_radius").get_parameter_value().double_value
        self.avoidance_eval_radius = self.get_parameter("avoidance_eval_radius").get_parameter_value().double_value
        self.avoidance_margin = self.get_parameter("avoidance_margin").get_parameter_value().double_value
        self.forced_mark = self.get_parameter("forced_mark").get_parameter_value().string_value.strip().upper()
        seed = self.get_parameter("seed").get_parameter_value().integer_value
        self.publish_rate_hz = self.get_parameter("publish_rate_hz").get_parameter_value().double_value

        raw_buoy_positions = []
        buoy_position_xy = self.get_parameter("buoy_position_xy").get_parameter_value().double_array_value
        bouy_position_xy = self.get_parameter("bouy_position_xy").get_parameter_value().double_array_value
        if len(buoy_position_xy) > 0:
            raw_buoy_positions = buoy_position_xy
        elif len(bouy_position_xy) > 0:
            raw_buoy_positions = bouy_position_xy
        if len(raw_buoy_positions) >= 2 and len(raw_buoy_positions) % 2 == 0:
            self.buoy_positions = []
            for i in range(0, len(raw_buoy_positions), 2):
                self.buoy_positions.append([float(raw_buoy_positions[i]), float(raw_buoy_positions[i + 1])])
        else:
            self.buoy_positions = [[self.buoy_xy[0], self.buoy_xy[1]]]

        if len(self.waypoint1_xy) == 0 and len(self.waypoint2_xy) == 0:
            self.waypoints = [
                [self.gps_checkpoint_xy[-1][0], self.gps_checkpoint_xy[-1][1]]
            ]
        else:
            self.waypoints = self.waypoint1_xy + self.waypoint2_xy
        self.next_waypoint_idx = 0
        self.reached_waypoints = []

        self.random = random.Random(seed)
        self.current_mark = "N"
        self.goal_announced = False
        self.avoidance_failed = False
        self.avoidance_results = []
        self._init_buoy_states()

        transient_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_start = self.create_publisher(Bool, "/sim/start", transient_qos)
        self.pub_goal = self.create_publisher(Bool, "/sim/goal_reached", transient_qos)
        self.pub_goal_pose = self.create_publisher(PoseStamped, "/goal_pose", transient_qos)

        self.pub_cardinal = self.create_publisher(String, "/sim/cardinal_mark", 10)
        self.pub_virtual_obstacles = self.create_publisher(PointCloud2, "/virtual_obstacles", 10)
        self.pub_status = self.create_publisher(String, "/sim/task1_status", 10)

        self.sub_odom = self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.srv_infer = self.create_service(Trigger, "/yolo/start_inference", self.on_start_inference)

        self.publish_start_and_goal()
        self.publish_virtual_wall()

        period = 1.0 / max(0.2, self.publish_rate_hz)
        self.timer = self.create_timer(period, self.on_timer)

    def _init_buoy_states(self):
        self.buoy_states = []
        for buoy in self.buoy_positions:
            self.buoy_states.append({
                "xy": buoy,
                "in_zone": False,
                "evaluated": False,
                "closest_dist": float("inf"),
                "signed_at_closest": 0.0,
            })

    @staticmethod
    def _parse_xy_list(param_value):
        if len(param_value) % 2 != 0:
            return []
        out = []
        for i in range(0, len(param_value), 2):
            out.append([float(param_value[i]), float(param_value[i + 1])])
        return out

    def _signed_side_value(self, mark, x, y, bx, by):
        if mark == "N":
            return y - by
        if mark == "S":
            return by - y
        if mark == "E":
            return x - bx
        return bx - x

    def _publish_status(self, text):
        msg = String()
        msg.data = text
        self.pub_status.publish(msg)
        self.get_logger().info(f"task1_status: {text}")

    def publish_start_and_goal(self):
        start_msg = Bool()
        start_msg.data = True
        self.pub_start.publish(start_msg)

        final_goal = self.gps_checkpoint_xy[-1]
        goal = PoseStamped()
        goal.header.frame_id = self.frame_id
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = final_goal[0]
        goal.pose.position.y = final_goal[1]
        goal.pose.orientation.w = 1.0
        self.pub_goal_pose.publish(goal)
        self._publish_status("start_published")

    def on_start_inference(self, _request, response):
        if self.forced_mark in ["N", "E", "W", "S"]:
            self.current_mark = self.forced_mark
        else:
            self.current_mark = self.random.choice(["N", "E", "W", "S"])
        self._init_buoy_states()
        self.avoidance_failed = False
        self.avoidance_results = []
        self.publish_virtual_wall()
        response.success = True
        response.message = self.current_mark
        self.get_logger().info(f"YOLO stub returned cardinal mark: {self.current_mark}")
        self._publish_status(f"mark={self.current_mark}")
        return response

    def build_wall_points_for_buoy(self, bx, by):
        points = []

        if self.current_mark == "N":
            start, end = math.pi, 2.0 * math.pi
        elif self.current_mark == "S":
            start, end = 0.0, math.pi
        elif self.current_mark == "E":
            start, end = math.pi / 2.0, 3.0 * math.pi / 2.0
        else:  # W
            start, end = -math.pi / 2.0, math.pi / 2.0

        for i in range(self.wall_points + 1):
            t = i / float(self.wall_points)
            angle = start + (end - start) * t
            points.append([
                bx + self.wall_radius * math.cos(angle),
                by + self.wall_radius * math.sin(angle),
                0.0,
            ])
        return points

    def build_wall_points(self):
        all_points = []
        for buoy in self.buoy_positions:
            all_points.extend(self.build_wall_points_for_buoy(buoy[0], buoy[1]))
        return all_points

    def publish_virtual_wall(self):
        msg = make_pointcloud2(self.frame_id, self.build_wall_points(), self.get_clock().now().to_msg())
        self.pub_virtual_obstacles.publish(msg)

    def on_odom(self, msg: Odometry):
        if self.goal_announced:
            return

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        if self.next_waypoint_idx < len(self.waypoints):
            wx, wy = self.waypoints[self.next_waypoint_idx]
            if math.hypot(x - wx, y - wy) <= self.waypoint_reach_radius:
                self.reached_waypoints.append(self.next_waypoint_idx)
                self.next_waypoint_idx += 1
                self._publish_status(f"waypoint_reached={self.next_waypoint_idx}/{len(self.waypoints)}")

        for idx, buoy_state in enumerate(self.buoy_states):
            if buoy_state["evaluated"]:
                continue
            bx, by = buoy_state["xy"]
            d = math.hypot(x - bx, y - by)

            if d <= self.avoidance_eval_radius:
                buoy_state["in_zone"] = True
                signed = self._signed_side_value(self.current_mark, x, y, bx, by)
                if d < buoy_state["closest_dist"]:
                    buoy_state["closest_dist"] = d
                    buoy_state["signed_at_closest"] = signed
            elif buoy_state["in_zone"]:
                buoy_state["evaluated"] = True
                passed = buoy_state["signed_at_closest"] >= self.avoidance_margin
                self.avoidance_results.append((idx, passed, buoy_state["signed_at_closest"]))
                if not passed:
                    self.avoidance_failed = True
                self._publish_status(
                    f"avoidance_buoy={idx},pass={str(passed).lower()},signed={buoy_state['signed_at_closest']:.2f}"
                )

        final_goal = self.gps_checkpoint_xy[-1]
        dx = x - final_goal[0]
        dy = y - final_goal[1]
        all_waypoints_reached = self.next_waypoint_idx >= len(self.waypoints)
        all_buoy_evaluated = all(s["evaluated"] for s in self.buoy_states)

        if math.hypot(dx, dy) <= self.goal_radius and all_waypoints_reached and all_buoy_evaluated and not self.avoidance_failed:
            done = Bool()
            done.data = True
            self.pub_goal.publish(done)
            self.goal_announced = True
            self.get_logger().info("Goal area reached")
            self._publish_status("goal_reached_with_task1_constraints")

    def on_timer(self):
        cardinal = String()
        cardinal.data = self.current_mark
        self.pub_cardinal.publish(cardinal)
        self.publish_virtual_wall()


def main(args=None):
    rclpy.init(args=args)
    node = Task1Orchestrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
