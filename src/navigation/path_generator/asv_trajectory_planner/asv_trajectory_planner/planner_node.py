#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path

from asv_trajectory_planner.trajectory_generator import TrajectoryGenerator


class PlannerNode(Node):
    """
    自船Odometry，他船Odometry，ゴールPoseを受け取り，
    自船が追従すべきPathをpublishするノード．

    Subscribe:
        /own_ship/odom
        /other_ship/odom
        /goal_pose

    Publish:
        /planned_path
    """

    def __init__(self):
        super().__init__("planner_node")

        # Topics
        self.declare_parameter("own_odom_topic", "/own_ship/odom")
        self.declare_parameter("other_ship_odom_topic", "/other_ship/odom")
        self.declare_parameter("goal_pose_topic", "/goal_pose")
        self.declare_parameter("path_topic", "/planned_path")

        # Planner parameters
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("planning_frequency", 2.0)
        self.declare_parameter("point_spacing", 0.5)
        self.declare_parameter("avoid_radius", 2.0)
        self.declare_parameter("avoid_offset", 3.0)
        self.declare_parameter("require_other_ship", True)

        self.own_odom_topic = self.get_parameter("own_odom_topic").value
        self.other_ship_odom_topic = self.get_parameter("other_ship_odom_topic").value
        self.goal_pose_topic = self.get_parameter("goal_pose_topic").value
        self.path_topic = self.get_parameter("path_topic").value

        self.frame_id = self.get_parameter("frame_id").value
        planning_frequency = self.get_parameter("planning_frequency").value
        point_spacing = self.get_parameter("point_spacing").value
        avoid_radius = self.get_parameter("avoid_radius").value
        avoid_offset = self.get_parameter("avoid_offset").value
        self.require_other_ship = self.get_parameter("require_other_ship").value

        self.trajectory_generator = TrajectoryGenerator(
            frame_id=self.frame_id,
            point_spacing=point_spacing,
            avoid_radius=avoid_radius,
            avoid_offset=avoid_offset,
        )

        self.latest_own_odom = None
        self.latest_other_ship_odom = None
        self.latest_goal_pose = None

        self.own_odom_sub = self.create_subscription(
            Odometry,
            self.own_odom_topic,
            self.own_odom_callback,
            10,
        )

        self.other_ship_odom_sub = self.create_subscription(
            Odometry,
            self.other_ship_odom_topic,
            self.other_ship_odom_callback,
            10,
        )

        self.goal_pose_sub = self.create_subscription(
            PoseStamped,
            self.goal_pose_topic,
            self.goal_pose_callback,
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
        self.get_logger().info(f"Subscribe own_odom       : {self.own_odom_topic}")
        self.get_logger().info(f"Subscribe other_ship_odom: {self.other_ship_odom_topic}")
        self.get_logger().info(f"Subscribe goal_pose      : {self.goal_pose_topic}")
        self.get_logger().info(f"Publish planned_path     : {self.path_topic}")

    def own_odom_callback(self, msg: Odometry):
        self.latest_own_odom = msg

    def other_ship_odom_callback(self, msg: Odometry):
        self.latest_other_ship_odom = msg

    def goal_pose_callback(self, msg: PoseStamped):
        self.latest_goal_pose = msg

    def timer_callback(self):
        if self.latest_own_odom is None:
            self.get_logger().debug("Waiting for own_odom...")
            return

        if self.latest_goal_pose is None:
            self.get_logger().debug("Waiting for goal_pose...")
            return

        if self.require_other_ship and self.latest_other_ship_odom is None:
            self.get_logger().debug("Waiting for other_ship_odom...")
            return

        path = self.trajectory_generator.generate(
            own_odom=self.latest_own_odom,
            other_ship_odom=self.latest_other_ship_odom,
            goal_pose=self.latest_goal_pose,
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