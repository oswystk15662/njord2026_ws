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
        self.declare_parameter("wall_radius", 2.5)
        self.declare_parameter("wall_points", 24)
        self.declare_parameter("goal_xy", [25.0, 0.0])
        self.declare_parameter("goal_radius", 2.0)
        self.declare_parameter("seed", 2026)
        self.declare_parameter("publish_rate_hz", 2.0)

        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self.buoy_xy = list(self.get_parameter("buoy_xy").get_parameter_value().double_array_value)
        self.wall_radius = self.get_parameter("wall_radius").get_parameter_value().double_value
        self.wall_points = max(6, self.get_parameter("wall_points").get_parameter_value().integer_value)
        self.goal_xy = list(self.get_parameter("goal_xy").get_parameter_value().double_array_value)
        self.goal_radius = self.get_parameter("goal_radius").get_parameter_value().double_value
        seed = self.get_parameter("seed").get_parameter_value().integer_value
        self.publish_rate_hz = self.get_parameter("publish_rate_hz").get_parameter_value().double_value

        self.random = random.Random(seed)
        self.current_mark = "N"
        self.goal_announced = False

        transient_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_start = self.create_publisher(Bool, "/sim/start", transient_qos)
        self.pub_goal = self.create_publisher(Bool, "/sim/goal_reached", transient_qos)
        self.pub_goal_pose = self.create_publisher(PoseStamped, "/goal_pose", transient_qos)

        self.pub_cardinal = self.create_publisher(String, "/sim/cardinal_mark", 10)
        self.pub_virtual_obstacles = self.create_publisher(PointCloud2, "/virtual_obstacles", 10)

        self.sub_odom = self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.srv_infer = self.create_service(Trigger, "/yolo/start_inference", self.on_start_inference)

        self.publish_start_and_goal()
        self.publish_virtual_wall()

        period = 1.0 / max(0.2, self.publish_rate_hz)
        self.timer = self.create_timer(period, self.on_timer)

    def publish_start_and_goal(self):
        start_msg = Bool()
        start_msg.data = True
        self.pub_start.publish(start_msg)

        goal = PoseStamped()
        goal.header.frame_id = self.frame_id
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = self.goal_xy[0]
        goal.pose.position.y = self.goal_xy[1]
        goal.pose.orientation.w = 1.0
        self.pub_goal_pose.publish(goal)

    def on_start_inference(self, _request, response):
        self.current_mark = self.random.choice(["N", "E", "W", "S"])
        self.publish_virtual_wall()
        response.success = True
        response.message = self.current_mark
        self.get_logger().info(f"YOLO stub returned cardinal mark: {self.current_mark}")
        return response

    def build_wall_points(self):
        bx, by = self.buoy_xy[0], self.buoy_xy[1]
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

    def publish_virtual_wall(self):
        msg = make_pointcloud2(self.frame_id, self.build_wall_points(), self.get_clock().now().to_msg())
        self.pub_virtual_obstacles.publish(msg)

    def on_odom(self, msg: Odometry):
        if self.goal_announced:
            return
        dx = msg.pose.pose.position.x - self.goal_xy[0]
        dy = msg.pose.pose.position.y - self.goal_xy[1]
        if math.hypot(dx, dy) <= self.goal_radius:
            done = Bool()
            done.data = True
            self.pub_goal.publish(done)
            self.goal_announced = True
            self.get_logger().info("Goal area reached")

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
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        try:
            rclpy.try_shutdown()
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
