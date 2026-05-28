import math

import rclpy
from geometry_msgs.msg import Point32, PolygonStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster


class Task3Orchestrator(Node):
    def __init__(self):
        super().__init__("task3_orchestrator")

        self.declare_parameter("frame_id", "map")
        self.declare_parameter("red_center", [18.0, 2.0])
        self.declare_parameter("green_center", [18.0, -2.0])
        self.declare_parameter("motion_amp", 0.5)
        self.declare_parameter("motion_freq_hz", 0.1)
        self.declare_parameter("dock_pose", [35.0, 0.0, 0.0])
        self.declare_parameter("dock_size_xy", [5.0, 2.0])
        self.declare_parameter("goal_xy", [35.0, 0.0])
        self.declare_parameter("goal_radius", 1.5)
        self.declare_parameter("publish_rate_hz", 10.0)

        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self.red_center = list(self.get_parameter("red_center").get_parameter_value().double_array_value)
        self.green_center = list(self.get_parameter("green_center").get_parameter_value().double_array_value)
        self.motion_amp = self.get_parameter("motion_amp").get_parameter_value().double_value
        self.motion_freq_hz = self.get_parameter("motion_freq_hz").get_parameter_value().double_value
        self.dock_pose = list(self.get_parameter("dock_pose").get_parameter_value().double_array_value)
        self.dock_size_xy = list(self.get_parameter("dock_size_xy").get_parameter_value().double_array_value)
        self.goal_xy = list(self.get_parameter("goal_xy").get_parameter_value().double_array_value)
        self.goal_radius = self.get_parameter("goal_radius").get_parameter_value().double_value
        self.publish_rate_hz = self.get_parameter("publish_rate_hz").get_parameter_value().double_value

        transient_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_start = self.create_publisher(Bool, "/sim/start", transient_qos)
        self.pub_goal = self.create_publisher(Bool, "/sim/goal_reached", transient_qos)
        self.pub_dock_projection = self.create_publisher(PolygonStamped, "/sim/dock_projection", 10)

        self.sub_odom = self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.goal_announced = False
        self.publish_start_and_goal()

        period = 1.0 / max(1.0, self.publish_rate_hz)
        self.timer = self.create_timer(period, self.on_timer)

    def publish_start_and_goal(self):
        start_msg = Bool()
        start_msg.data = True
        self.pub_start.publish(start_msg)

    def publish_tf(self, name: str, x: float, y: float, yaw: float = 0.0):
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = self.frame_id
        tf.child_frame_id = name
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.translation.z = 0.0
        tf.transform.rotation.z = math.sin(yaw / 2.0)
        tf.transform.rotation.w = math.cos(yaw / 2.0)
        self.tf_broadcaster.sendTransform(tf)

    def publish_dock_projection(self):
        x, y, yaw = self.dock_pose
        sx, sy = self.dock_size_xy
        corners_local = [
            (-sx / 2.0, -sy / 2.0),
            (sx / 2.0, -sy / 2.0),
            (sx / 2.0, sy / 2.0),
            (-sx / 2.0, sy / 2.0),
        ]
        c = math.cos(yaw)
        s = math.sin(yaw)

        poly = PolygonStamped()
        poly.header.stamp = self.get_clock().now().to_msg()
        poly.header.frame_id = self.frame_id
        for lx, ly in corners_local:
            p = Point32()
            p.x = float(x + c * lx - s * ly)
            p.y = float(y + s * lx + c * ly)
            p.z = 0.0
            poly.polygon.points.append(p)
        self.pub_dock_projection.publish(poly)

    def on_timer(self):
        t = self.get_clock().now().nanoseconds * 1.0e-9
        phase = 2.0 * math.pi * self.motion_freq_hz * t

        red_x = self.red_center[0] + self.motion_amp * math.sin(phase)
        red_y = self.red_center[1] + 0.3 * self.motion_amp * math.cos(phase)

        green_x = self.green_center[0] + self.motion_amp * math.sin(phase + math.pi / 2.0)
        green_y = self.green_center[1] + 0.3 * self.motion_amp * math.cos(phase + math.pi / 2.0)

        self.publish_tf("red_buoy", red_x, red_y)
        self.publish_tf("green_buoy", green_x, green_y)
        self.publish_tf("dock", self.dock_pose[0], self.dock_pose[1], self.dock_pose[2])
        self.publish_dock_projection()

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


def main(args=None):
    rclpy.init(args=args)
    node = Task3Orchestrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
