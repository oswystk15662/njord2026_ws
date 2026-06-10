import math

import rclpy
from nav_msgs.msg import Odometry, Path as PathMsg
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class PurePursuitFollowerNode(Node):
    def __init__(self):
        super().__init__("path_follower_node")

        self.declare_parameter("topic_plan", "/plan")
        self.declare_parameter("topic_odom", "/odom")
        self.declare_parameter("topic_thruster_command", "/thruster_command")

        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("lookahead_distance", 2.0)
        self.declare_parameter("goal_tolerance", 1.0)

        self.declare_parameter("base_duty", 350)
        self.declare_parameter("max_duty", 700)
        self.declare_parameter("turn_gain", 450.0)
        self.declare_parameter("slowdown_distance", 5.0)

        self.topic_plan = self.get_parameter("topic_plan").get_parameter_value().string_value
        self.topic_odom = self.get_parameter("topic_odom").get_parameter_value().string_value
        self.topic_thruster_command = (
            self.get_parameter("topic_thruster_command").get_parameter_value().string_value
        )

        self.control_rate_hz = (
            self.get_parameter("control_rate_hz").get_parameter_value().double_value
        )
        self.lookahead_distance = (
            self.get_parameter("lookahead_distance").get_parameter_value().double_value
        )
        self.goal_tolerance = (
            self.get_parameter("goal_tolerance").get_parameter_value().double_value
        )

        self.base_duty = self.get_parameter("base_duty").get_parameter_value().integer_value
        self.max_duty = self.get_parameter("max_duty").get_parameter_value().integer_value
        self.turn_gain = self.get_parameter("turn_gain").get_parameter_value().double_value
        self.slowdown_distance = (
            self.get_parameter("slowdown_distance").get_parameter_value().double_value
        )

        self.path_points = []
        self.current_pose = None
        self.goal_reached = False

        self.sub_plan = self.create_subscription(
            PathMsg,
            self.topic_plan,
            self.on_plan,
            10,
        )

        self.sub_odom = self.create_subscription(
            Odometry,
            self.topic_odom,
            self.on_odom,
            30,
        )

        self.pub_thruster = self.create_publisher(
            Int16MultiArray,
            self.topic_thruster_command,
            10,
        )

        period = 1.0 / max(1.0, self.control_rate_hz)
        self.timer = self.create_timer(period, self.on_control)

        self.get_logger().info(
            "Path follower started. "
            f"plan={self.topic_plan}, odom={self.topic_odom}, "
            f"cmd={self.topic_thruster_command}"
        )

    def on_plan(self, msg: PathMsg):
        self.path_points = [
            [pose.pose.position.x, pose.pose.position.y]
            for pose in msg.poses
        ]

        self.goal_reached = False

        self.get_logger().info(
            f"Received plan with {len(self.path_points)} points."
        )

    def on_odom(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)

        u = msg.twist.twist.linear.x

        self.current_pose = [x, y, yaw, u]

    def publish_thruster(self, left_duty: int, right_duty: int):
        msg = Int16MultiArray()
        msg.data = [int(left_duty), int(right_duty)]
        self.pub_thruster.publish(msg)

    def stop(self):
        self.publish_thruster(0, 0)

    def find_nearest_index(self, x: float, y: float) -> int:
        nearest_idx = 0
        nearest_dist = float("inf")

        for i, (px, py) in enumerate(self.path_points):
            dist = math.hypot(px - x, py - y)
            if dist < nearest_dist:
                nearest_dist = dist
                nearest_idx = i

        return nearest_idx

    def find_lookahead_point(self, x: float, y: float):
        if len(self.path_points) == 0:
            return None

        nearest_idx = self.find_nearest_index(x, y)

        for i in range(nearest_idx, len(self.path_points)):
            px, py = self.path_points[i]
            dist = math.hypot(px - x, py - y)

            if dist >= self.lookahead_distance:
                return px, py

        return self.path_points[-1]

    def compute_duty_command(self, x: float, y: float, yaw: float):
        goal_x, goal_y = self.path_points[-1]
        goal_dist = math.hypot(goal_x - x, goal_y - y)

        if goal_dist <= self.goal_tolerance:
            self.goal_reached = True
            return 0, 0

        target = self.find_lookahead_point(x, y)

        if target is None:
            return 0, 0

        target_x, target_y = target

        desired_yaw = math.atan2(target_y - y, target_x - x)
        heading_error = normalize_angle(desired_yaw - yaw)

        # ゴールに近づくと前進dutyを小さくする
        if self.slowdown_distance > 1.0e-6:
            speed_scale = min(1.0, max(0.25, goal_dist / self.slowdown_distance))
        else:
            speed_scale = 1.0

        forward = float(self.base_duty) * speed_scale
        turn = self.turn_gain * heading_error

        left = forward - turn
        right = forward + turn

        left = max(-self.max_duty, min(self.max_duty, left))
        right = max(-self.max_duty, min(self.max_duty, right))

        return int(left), int(right)

    def on_control(self):
        if len(self.path_points) < 2:
            self.stop()
            return

        if self.current_pose is None:
            self.stop()
            return

        if self.goal_reached:
            self.stop()
            return

        x, y, yaw, _u = self.current_pose

        left, right = self.compute_duty_command(x, y, yaw)
        self.publish_thruster(left, right)


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitFollowerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()