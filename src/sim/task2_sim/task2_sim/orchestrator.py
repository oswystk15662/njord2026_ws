import math

import rclpy
from geometry_msgs.msg import Point, Point32, PolygonStamped, PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray


class Task2Orchestrator(Node):
    def __init__(self):
        super().__init__("task2_orchestrator")

        self.declare_parameter("frame_id", "map")
        self.declare_parameter("red_center", [8.0, 2.0])
        self.declare_parameter("green_center", [8.0, -2.0])
        self.declare_parameter("buoy_x_positions", [10.0, 15.0, 20.0, 40.0, 45.0, 50.0])
        self.declare_parameter("red_y", 2.5)
        self.declare_parameter("green_y", -2.5)
        self.declare_parameter("buoy_radius", 0.2)
        self.declare_parameter("buoy_motion_amp", 0.0)
        self.declare_parameter("motion_amp", 0.7)
        self.declare_parameter("motion_freq_hz", 0.15)
        self.declare_parameter("goal_xy", [30.0, 0.0])
        self.declare_parameter("goal_radius", 2.0)
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("projection_xywh", [15.0, 0.0, 8.0, 5.0])

        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self.red_center = list(self.get_parameter("red_center").get_parameter_value().double_array_value)
        self.green_center = list(self.get_parameter("green_center").get_parameter_value().double_array_value)
        self.buoy_x_positions = list(self.get_parameter("buoy_x_positions").get_parameter_value().double_array_value)
        self.red_y = self.get_parameter("red_y").get_parameter_value().double_value
        self.green_y = self.get_parameter("green_y").get_parameter_value().double_value
        self.buoy_radius = self.get_parameter("buoy_radius").get_parameter_value().double_value
        self.buoy_motion_amp = self.get_parameter("buoy_motion_amp").get_parameter_value().double_value
        self.motion_amp = self.get_parameter("motion_amp").get_parameter_value().double_value
        self.motion_freq_hz = self.get_parameter("motion_freq_hz").get_parameter_value().double_value
        self.goal_xy = list(self.get_parameter("goal_xy").get_parameter_value().double_array_value)
        self.goal_radius = self.get_parameter("goal_radius").get_parameter_value().double_value
        self.publish_rate_hz = self.get_parameter("publish_rate_hz").get_parameter_value().double_value
        self.projection_xywh = list(self.get_parameter("projection_xywh").get_parameter_value().double_array_value)

        transient_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_start = self.create_publisher(Bool, "/sim/start", transient_qos)
        self.pub_goal = self.create_publisher(Bool, "/sim/goal_reached", transient_qos)
        self.pub_goal_pose = self.create_publisher(PoseStamped, "/goal_pose", transient_qos)
        self.pub_projection = self.create_publisher(PolygonStamped, "/sim/marker_vessel_projection", 10)
        self.pub_buoy_markers = self.create_publisher(MarkerArray, "/sim/task2_buoy_markers", 10)

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

        goal = PoseStamped()
        goal.header.frame_id = self.frame_id
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = self.goal_xy[0]
        goal.pose.position.y = self.goal_xy[1]
        goal.pose.orientation.w = 1.0
        self.pub_goal_pose.publish(goal)

    def publish_buoy_tf(self, name: str, x: float, y: float):
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = self.frame_id
        tf.child_frame_id = name
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.translation.z = 0.0
        tf.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(tf)

    def publish_buoy_markers(self, red_positions, green_positions):
        stamp = self.get_clock().now().to_msg()
        markers = MarkerArray()

        for marker_id, (namespace, positions, color) in enumerate([
            ("task2_red_buoys", red_positions, (1.0, 0.05, 0.05, 1.0)),
            ("task2_green_buoys", green_positions, (0.05, 0.8, 0.1, 1.0)),
        ]):
            marker = Marker()
            marker.header.stamp = stamp
            marker.header.frame_id = self.frame_id
            marker.ns = namespace
            marker.id = marker_id
            marker.type = Marker.SPHERE_LIST
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = 2.0 * self.buoy_radius
            marker.scale.y = 2.0 * self.buoy_radius
            marker.scale.z = 0.4
            marker.color.r = color[0]
            marker.color.g = color[1]
            marker.color.b = color[2]
            marker.color.a = color[3]

            for x, y in positions:
                p = Point()
                p.x = float(x)
                p.y = float(y)
                p.z = 0.0
                marker.points.append(p)
            markers.markers.append(marker)

        self.pub_buoy_markers.publish(markers)

    def publish_projection(self):
        cx, cy, w, h = self.projection_xywh
        poly = PolygonStamped()
        poly.header.stamp = self.get_clock().now().to_msg()
        poly.header.frame_id = self.frame_id
        corners = [
            (cx - w / 2.0, cy - h / 2.0),
            (cx + w / 2.0, cy - h / 2.0),
            (cx + w / 2.0, cy + h / 2.0),
            (cx - w / 2.0, cy + h / 2.0),
        ]
        for x, y in corners:
            p = Point32()
            p.x = float(x)
            p.y = float(y)
            p.z = 0.0
            poly.polygon.points.append(p)
        self.pub_projection.publish(poly)

    def on_timer(self):
        t = self.get_clock().now().nanoseconds * 1.0e-9
        red_positions = []
        green_positions = []

        for idx, base_x in enumerate(self.buoy_x_positions):
            phase = 2.0 * math.pi * self.motion_freq_hz * t + idx * math.pi / 3.0
            red_x = base_x + self.buoy_motion_amp * math.sin(phase)
            red_y = self.red_y + 0.3 * self.buoy_motion_amp * math.cos(phase)
            green_x = base_x + self.buoy_motion_amp * math.sin(phase + math.pi / 2.0)
            green_y = self.green_y + 0.3 * self.buoy_motion_amp * math.cos(phase + math.pi / 2.0)

            suffix = str(int(base_x)) if float(base_x).is_integer() else f"{base_x:.1f}".replace(".", "_")
            self.publish_buoy_tf(f"red_buoy_x{suffix}", red_x, red_y)
            self.publish_buoy_tf(f"green_buoy_x{suffix}", green_x, green_y)
            red_positions.append((red_x, red_y))
            green_positions.append((green_x, green_y))

            if idx == 0:
                self.publish_buoy_tf("red_buoy", red_x, red_y)
                self.publish_buoy_tf("green_buoy", green_x, green_y)

        self.publish_buoy_markers(red_positions, green_positions)
        self.publish_projection()

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
    node = Task2Orchestrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
