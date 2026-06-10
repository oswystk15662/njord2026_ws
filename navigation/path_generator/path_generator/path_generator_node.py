import math

import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Path as PathMsg
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from .catmull_rom import generate_catmull_rom_path


def yaw_to_quaternion(yaw):
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class PathGeneratorNode(Node):
    def __init__(self):
        super().__init__("path_generator_node")

        self.declare_parameter("frame_id", "map")
        self.declare_parameter("topic_waypoints", "/sim/waypoints")
        self.declare_parameter("topic_plan", "/plan")
        self.declare_parameter("sample_interval", 0.5)

        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        topic_waypoints = self.get_parameter("topic_waypoints").get_parameter_value().string_value
        topic_plan = self.get_parameter("topic_plan").get_parameter_value().string_value
        self.sample_interval = self.get_parameter("sample_interval").get_parameter_value().double_value

        transient_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.pub_plan = self.create_publisher(PathMsg, topic_plan, transient_qos)

        self.sub_waypoints = self.create_subscription(
            PathMsg,
            topic_waypoints,
            self.on_waypoints,
            10,
        )

        self.get_logger().info(
            f"Path generator started: {topic_waypoints} -> {topic_plan}"
        )

    def on_waypoints(self, msg):
        waypoints = [
            [pose.pose.position.x, pose.pose.position.y]
            for pose in msg.poses
        ]

        if len(waypoints) < 2:
            self.get_logger().warn("Received less than 2 waypoints.")
            return

        smooth_points = generate_catmull_rom_path(
            waypoints,
            self.sample_interval,
        )

        plan = self.points_to_path_msg(smooth_points)
        self.pub_plan.publish(plan)

    def points_to_path_msg(self, points):
        msg = PathMsg()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()

        for i, point in enumerate(points):
            x, y = point

            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.0

            if i < len(points) - 1:
                nx, ny = points[i + 1]
                yaw = math.atan2(ny - y, nx - x)
            elif i > 0:
                px, py = points[i - 1]
                yaw = math.atan2(y - py, x - px)
            else:
                yaw = 0.0

            pose.pose.orientation = yaw_to_quaternion(yaw)
            msg.poses.append(pose)

        return msg


def main(args=None):
    rclpy.init(args=args)
    node = PathGeneratorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()