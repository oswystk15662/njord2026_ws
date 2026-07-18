#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry, Path

from tf2_ros import Buffer, TransformListener

try:
    from tf2_ros import TransformException
except ImportError:
    from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
    TransformException = (LookupException, ConnectivityException, ExtrapolationException)

from asv_trajectory_planner.trajectory_generator import TrajectoryGenerator


class PlannerNode(Node):
    """
    自船Odometry，他船TF，他船TwistStamped，2つのウェイポイントを受け取り，
    自船が追従すべきPathをpublishするノード．

    Subscribe:
        /own_ship/odom
        /other_ship/twist
        /waypoint1_pose
        /waypoint2_pose

    TF:
        base_link -> other_ship_base_link

    Publish:
        /planned_path
    """

    def __init__(self):
        super().__init__("planner_node")

        # Topics
        self.declare_parameter("own_odom_topic", "/own_ship/odom")
        self.declare_parameter("other_ship_twist_topic", "/other_ship/twist")
        self.declare_parameter("waypoint1_topic", "/waypoint1_pose")
        self.declare_parameter("waypoint2_topic", "/waypoint2_pose")
        self.declare_parameter("path_topic", "/planned_path")

        # TF frames
        self.declare_parameter("own_frame", "base_link")
        self.declare_parameter("other_ship_frame", "other_ship_base_link")

        # Planner parameters
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("planning_frequency", 2.0)
        self.declare_parameter("point_spacing", 0.5)
        self.declare_parameter("avoid_radius", 2.0)
        self.declare_parameter("avoid_offset", 3.0)
        self.declare_parameter("require_other_ship", True)
        self.declare_parameter("other_twist_is_relative", True)
        self.declare_parameter("reconnect_line_distance_m", 1.0)
        self.declare_parameter("reconnect_ahead_length_m", 8.0)
        self.declare_parameter("straight_path_spacing_m", 2.0)
        self.declare_parameter("straight_path_length_m", 60.0)
        self.declare_parameter("mppi_smoothing_window", 5)
        self.declare_parameter("opponent_use_distance_m", 20.0)
        self.declare_parameter("opponent_passed_margin_m", 2.0)

        self.own_odom_topic = self.get_parameter("own_odom_topic").value
        self.other_ship_twist_topic = self.get_parameter("other_ship_twist_topic").value
        self.waypoint1_topic = self.get_parameter("waypoint1_topic").value
        self.waypoint2_topic = self.get_parameter("waypoint2_topic").value
        self.path_topic = self.get_parameter("path_topic").value

        self.own_frame = self.get_parameter("own_frame").value
        self.other_ship_frame = self.get_parameter("other_ship_frame").value

        self.frame_id = self.get_parameter("frame_id").value
        planning_frequency = self.get_parameter("planning_frequency").value
        point_spacing = self.get_parameter("point_spacing").value
        avoid_radius = self.get_parameter("avoid_radius").value
        avoid_offset = self.get_parameter("avoid_offset").value
        self.require_other_ship = self.get_parameter("require_other_ship").value
        other_twist_is_relative = self.get_parameter("other_twist_is_relative").value
        reconnect_line_distance_m = self.get_parameter("reconnect_line_distance_m").value
        reconnect_ahead_length_m = self.get_parameter("reconnect_ahead_length_m").value
        straight_path_spacing_m = self.get_parameter("straight_path_spacing_m").value
        straight_path_length_m = self.get_parameter("straight_path_length_m").value
        mppi_smoothing_window = self.get_parameter("mppi_smoothing_window").value
        opponent_use_distance_m = self.get_parameter("opponent_use_distance_m").value
        opponent_passed_margin_m = self.get_parameter("opponent_passed_margin_m").value

        self.trajectory_generator = TrajectoryGenerator(
            frame_id=self.frame_id,
            point_spacing=point_spacing,
            avoid_radius=avoid_radius,
            avoid_offset=avoid_offset,
            other_twist_is_relative=other_twist_is_relative,
            opponent_use_distance_m=opponent_use_distance_m,
            opponent_passed_margin_m=opponent_passed_margin_m,
            reconnect_line_distance_m=reconnect_line_distance_m,
            reconnect_ahead_length_m=reconnect_ahead_length_m,
            straight_path_spacing_m=straight_path_spacing_m,
            straight_path_length_m=straight_path_length_m,
            mppi_smoothing_window=mppi_smoothing_window,
        )

        self.latest_own_odom = None
        self.latest_other_ship_twist = None
        self.latest_waypoint1_pose = None
        self.latest_waypoint2_pose = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.own_odom_sub = self.create_subscription(
            Odometry,
            self.own_odom_topic,
            self.own_odom_callback,
            10,
        )

        self.other_ship_twist_sub = self.create_subscription(
            TwistStamped,
            self.other_ship_twist_topic,
            self.other_ship_twist_callback,
            10,
        )

        self.waypoint1_sub = self.create_subscription(
            PoseStamped,
            self.waypoint1_topic,
            self.waypoint1_callback,
            10,
        )

        self.waypoint2_sub = self.create_subscription(
            PoseStamped,
            self.waypoint2_topic,
            self.waypoint2_callback,
            10,
        )

        self.path_pub = self.create_publisher(
            Path,
            self.path_topic,
            10,
        )

        timer_period = 1.0 / max(planning_frequency, 1e-6)
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info("planner_node started.")
        self.get_logger().info(f"Subscribe own_odom        : {self.own_odom_topic}")
        self.get_logger().info(f"Subscribe other_ship_twist: {self.other_ship_twist_topic}")
        self.get_logger().info(f"Subscribe waypoint1      : {self.waypoint1_topic}")
        self.get_logger().info(f"Subscribe waypoint2      : {self.waypoint2_topic}")
        self.get_logger().info(f"TF own_frame             : {self.own_frame}")
        self.get_logger().info(f"TF other_ship_frame      : {self.other_ship_frame}")
        self.get_logger().info(f"Publish planned_path     : {self.path_topic}")
        self.get_logger().info(f"Opponent use distance    : {opponent_use_distance_m} m")
        self.get_logger().info(f"Opponent passed margin   : {opponent_passed_margin_m} m")
        self.get_logger().info(f"Reconnect line distance  : {reconnect_line_distance_m} m")
        self.get_logger().info(f"Reconnect ahead length   : {reconnect_ahead_length_m} m")
        self.get_logger().info(f"Opponent use distance    : {opponent_use_distance_m} m")
        self.get_logger().info(f"Opponent passed margin   : {opponent_passed_margin_m} m")

    def own_odom_callback(self, msg: Odometry):
        self.latest_own_odom = msg

    def other_ship_twist_callback(self, msg: TwistStamped):
        self.latest_other_ship_twist = msg

    def waypoint1_callback(self, msg: PoseStamped):
        self.latest_waypoint1_pose = msg

    def waypoint2_callback(self, msg: PoseStamped):
        self.latest_waypoint2_pose = msg

    def timer_callback(self):
        if self.latest_own_odom is None:
            self.get_logger().debug("Waiting for own_odom...")
            return

        if self.latest_waypoint1_pose is None:
            self.get_logger().debug("Waiting for waypoint1_pose...")
            return

        if self.latest_waypoint2_pose is None:
            self.get_logger().debug("Waiting for waypoint2_pose...")
            return

        other_transform = None

        if self.latest_other_ship_twist is None:
            if self.require_other_ship:
                self.get_logger().debug("Waiting for other_ship_twist...")
                return
        else:
            try:
                tf_msg = self.tf_buffer.lookup_transform(
                    self.own_frame,
                    self.other_ship_frame,
                    Time(),
                )
                other_transform = tf_msg.transform

            except TransformException as e:
                if self.require_other_ship:
                    self.get_logger().warn(
                        f"Cannot lookup TF {self.own_frame} -> "
                        f"{self.other_ship_frame}: {e}",
                        throttle_duration_sec=2.0,
                    )
                    return

                other_transform = None

        path = self.trajectory_generator.generate(
            own_odom=self.latest_own_odom,
            other_transform=other_transform,
            other_twist=self.latest_other_ship_twist,
            waypoint1_pose=self.latest_waypoint1_pose,
            waypoint2_pose=self.latest_waypoint2_pose,
        )

        now = self.get_clock().now().to_msg()
        path.header.stamp = now
        path.header.frame_id = self.frame_id

        for pose in path.poses:
            pose.header.stamp = now
            pose.header.frame_id = self.frame_id

        self.path_pub.publish(path)

        self.get_logger().info(
            f"Published path: {len(path.poses)} poses",
            throttle_duration_sec=2.0,
        )


def main(args=None):
    rclpy.init(args=args)

    node = PlannerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
