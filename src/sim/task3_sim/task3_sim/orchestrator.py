import math
import struct

import rclpy
from geometry_msgs.msg import Point32, PolygonStamped, TransformStamped
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


class Task3Orchestrator(Node):
    def __init__(self):
        super().__init__("task3_orchestrator")

        # Declare parameters
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("task_type", "task3_1")
        self.declare_parameter("motion_amp", 0.3)
        self.declare_parameter("motion_freq_hz", 0.1)
        self.declare_parameter("dock_pose", [15.0, 0.0, 0.0])
        self.declare_parameter("dock_size_xy", [2.0, 4.13])
        self.declare_parameter("goal_xy", [13.0, 11.0])
        self.declare_parameter("goal_radius", 1.5)
        self.declare_parameter("publish_rate_hz", 10.0)

        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self.task_type = self.get_parameter("task_type").get_parameter_value().string_value
        self.motion_amp = self.get_parameter("motion_amp").get_parameter_value().double_value
        self.motion_freq_hz = self.get_parameter("motion_freq_hz").get_parameter_value().double_value
        self.goal_radius = self.get_parameter("goal_radius").get_parameter_value().double_value
        self.publish_rate_hz = self.get_parameter("publish_rate_hz").get_parameter_value().double_value

        # Dynamically override parameters based on task type to form a point-symmetric virtual field
        if self.task_type == "task3_2":
            self.dock_pose = [-15.0, 0.0, 3.14159]
            self.dock_size_xy = [2.0, 4.13]
            self.goal_xy = [-13.0, -11.0]
        else:  # task3_1 default
            self.dock_pose = [15.0, 0.0, 0.0]
            self.dock_size_xy = [2.0, 4.13]
            self.goal_xy = [13.0, 11.0]

        # Always import BOTH buoy sets — full field is always visible
        from .buoy_config import BUOY_DEFINITIONS_3_1, BUOY_DEFINITIONS_3_2
        self.buoy_definitions = BUOY_DEFINITIONS_3_1 + BUOY_DEFINITIONS_3_2

        transient_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_start = self.create_publisher(Bool, "/sim/start", transient_qos)
        self.pub_goal = self.create_publisher(Bool, "/sim/goal_reached", transient_qos)
        self.pub_dock_projection = self.create_publisher(PolygonStamped, "/sim/dock_projection", 10)
        self.pub_buoy_markers = self.create_publisher(MarkerArray, "/sim/buoy_markers", 10)

        # Pointcloud publisher and TF listener
        self.pub_pointcloud = self.create_publisher(PointCloud2, "/pointcloud", 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.sub_odom = self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.goal_announced = False
        self.publish_start_and_goal()

        # Pre-generate static dock points in the global map frame
        self.static_map_points = self.generate_field_points()

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

    def publish_buoy_markers(self, buoy_positions):
        markers = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        for marker_id, buoy in enumerate(buoy_positions):
            marker = Marker()
            marker.header.stamp = stamp
            marker.header.frame_id = self.frame_id
            marker.ns = "task3_buoys"
            marker.id = marker_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = buoy["x"]
            marker.pose.position.y = buoy["y"]
            marker.pose.position.z = 0.45
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.9
            marker.scale.y = 0.9
            marker.scale.z = 0.9
            marker.color.a = 0.9
            if "red" in buoy["name"]:
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
            elif "green" in buoy["name"]:
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            else:
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 1.0
            markers.markers.append(marker)

        self.pub_buoy_markers.publish(markers)

    def generate_field_points(self):
        pts = []
        # 1. Task 3.1 Dock (Normal Dock) - shifted 2m deeper (x: 14.0 to 16.0)
        # Left prong
        pts.extend(self.line_points(14.0, 2.195, 16.0, 2.195, 0.05))
        pts.extend(self.line_points(14.0, 2.065, 16.0, 2.065, 0.05))
        pts.extend(self.line_points(14.0, 2.065, 14.0, 2.195, 0.05))
        # Right prong
        pts.extend(self.line_points(14.0, -2.195, 16.0, -2.195, 0.05))
        pts.extend(self.line_points(14.0, -2.065, 16.0, -2.065, 0.05))
        pts.extend(self.line_points(14.0, -2.065, 14.0, -2.195, 0.05))
        # Center prong
        pts.extend(self.line_points(14.0, 0.065, 16.0, 0.065, 0.05))
        pts.extend(self.line_points(14.0, -0.065, 16.0, -0.065, 0.05))
        pts.extend(self.line_points(14.0, -0.065, 14.0, 0.065, 0.05))
        # Back walls
        pts.extend(self.line_points(16.0, 0.065, 16.0, 2.065, 0.05))
        pts.extend(self.line_points(16.0, -2.065, 16.0, -0.065, 0.05))

        # 2. Task 3.2 Dock (Parallel Dock) - shifted 2m deeper (x: -14.0 to -16.0)
        # Left prong
        pts.extend(self.line_points(-14.0, 2.195, -16.0, 2.195, 0.05))
        pts.extend(self.line_points(-14.0, 2.065, -16.0, 2.065, 0.05))
        pts.extend(self.line_points(-14.0, 2.065, -14.0, 2.195, 0.05))
        # Right prong
        pts.extend(self.line_points(-14.0, -2.195, -16.0, -2.195, 0.05))
        pts.extend(self.line_points(-14.0, -2.065, -16.0, -2.065, 0.05))
        pts.extend(self.line_points(-14.0, -2.065, -14.0, -2.195, 0.05))
        # Back wall (no center prong)
        pts.extend(self.line_points(-16.0, -2.065, -16.0, 2.065, 0.05))

        # Unique points (deduplicate within 1mm)
        unique_pts = []
        seen = set()
        for p in pts:
            key = (round(p[0], 3), round(p[1], 3))
            if key not in seen:
                seen.add(key)
                unique_pts.append(p)
        return unique_pts

    def line_points(self, x1, y1, x2, y2, step):
        points = []
        dx = x2 - x1
        dy = y2 - y1
        dist = math.hypot(dx, dy)
        if dist < 1e-6:
            return [[x1, y1, 0.0]]
        num_steps = int(math.ceil(dist / step))
        for i in range(num_steps + 1):
            t = i / num_steps
            points.append([x1 + t * dx, y1 + t * dy, 0.0])
        return points

    def rotate_vector(self, x, y, z, qx, qy, qz, qw):
        # Rotate (x, y, z) by quaternion (qx, qy, qz, qw)
        c1x = qy * z - qz * y
        c1y = qz * x - qx * z
        c1z = qx * y - qy * x

        v1x = c1x + qw * x
        v1y = c1y + qw * y
        v1z = c1z + qw * z

        c2x = qy * v1z - qz * v1y
        c2y = qz * v1x - qx * v1z
        c2z = qx * v1y - qy * v1x

        rx = x + 2.0 * c2x
        ry = y + 2.0 * c2y
        rz = z + 2.0 * c2z
        return rx, ry, rz

    def create_pointcloud2(self, points):
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"

        msg.height = 1
        msg.width = len(points)

        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12 * len(points)
        msg.is_dense = True

        buffer = []
        for p in points:
            buffer.append(struct.pack('fff', p[0], p[1], p[2]))

        msg.data = b''.join(buffer)
        return msg

    def on_timer(self):
        t = self.get_clock().now().nanoseconds * 1.0e-9

        # Generate dynamic buoy positions and broadcast TFs
        buoy_pts = []
        buoy_positions = []
        s = 0.05  # 5cm square
        for buoy in self.buoy_definitions:
            phase = 2.0 * math.pi * self.motion_freq_hz * t + buoy['phase_offset']
            x_dyn = buoy['center'][0] + self.motion_amp * math.sin(phase)
            y_dyn = buoy['center'][1] + self.motion_amp * math.cos(phase)

            # Broadcast buoy TF
            self.publish_tf(buoy['name'], x_dyn, y_dyn)
            buoy_positions.append({"name": buoy["name"], "x": x_dyn, "y": y_dyn})

            # Generate 5cm-squared points around the dynamic center
            buoy_pts.extend(self.line_points(x_dyn - s/2, y_dyn + s/2, x_dyn + s/2, y_dyn + s/2, 0.01))
            buoy_pts.extend(self.line_points(x_dyn - s/2, y_dyn - s/2, x_dyn + s/2, y_dyn - s/2, 0.01))
            buoy_pts.extend(self.line_points(x_dyn - s/2, y_dyn - s/2, x_dyn - s/2, y_dyn + s/2, 0.01))
            buoy_pts.extend(self.line_points(x_dyn + s/2, y_dyn - s/2, x_dyn + s/2, y_dyn + s/2, 0.01))

        # Broadcast dock TF and projection
        self.publish_tf("dock", self.dock_pose[0], self.dock_pose[1], self.dock_pose[2])
        self.publish_dock_projection()
        self.publish_buoy_markers(buoy_positions)

        # Combine static dock and dynamic buoy points
        all_map_pts = self.static_map_points + buoy_pts

        # Transform and publish pointcloud
        try:
            trans = self.tf_buffer.lookup_transform("base_link", "map", rclpy.time.Time())
            tx = trans.transform.translation.x
            ty = trans.transform.translation.y
            tz = trans.transform.translation.z
            qx = trans.transform.rotation.x
            qy = trans.transform.rotation.y
            qz = trans.transform.rotation.z
            qw = trans.transform.rotation.w

            transformed_pts = []
            for px, py, pz in all_map_pts:
                rx, ry, rz = self.rotate_vector(px, py, pz, qx, qy, qz, qw)
                transformed_pts.append([rx + tx, ry + ty, rz + tz])

            pc_msg = self.create_pointcloud2(transformed_pts)
            self.pub_pointcloud.publish(pc_msg)
        except Exception:
            pass

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
