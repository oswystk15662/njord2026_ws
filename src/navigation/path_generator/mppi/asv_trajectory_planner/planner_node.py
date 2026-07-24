#!/usr/bin/env python3

import copy
import math

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
from asv_trajectory_planner.frame_transform import (
    transform_planar_pose,
    yaw_from_quaternion,
)


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
        self.declare_parameter("other_ship_timeout_sec", 2.0)
        self.declare_parameter("other_twist_is_relative", True)
        self.declare_parameter("reconnect_line_distance_m", 1.0)
        self.declare_parameter("reconnect_ahead_length_m", 8.0)
        self.declare_parameter("straight_path_spacing_m", 2.0)
        self.declare_parameter("straight_path_length_m", 60.0)
        self.declare_parameter("mppi_smoothing_window", 5)
        self.declare_parameter("opponent_use_distance_m", 20.0)
        self.declare_parameter("opponent_passed_margin_m", 2.0)

        # ------------------------------------------------------------
        # MPPI hyperparameters.
        # Defaults equal the values that were hardcoded in
        # asv_trajectory_planner/mppi_torch.py, so leaving these untouched
        # keeps the planner behaviorally identical to the previous version.
        # See config/mppi_params.yaml for the parameter -> code mapping.
        # ------------------------------------------------------------
        self.declare_parameter("mppi.horizon", 225)
        self.declare_parameter("mppi.dt", 0.1)
        self.declare_parameter("mppi.num_samples", 5000)
        self.declare_parameter("mppi.lambda", 12.0)
        self.declare_parameter("mppi.control_noise_sigma", [35.0, 0.0])
        self.declare_parameter("mppi.target_speed", 2.0 * 1852.0 / 3600.0)
        self.declare_parameter("mppi.path_cost_weight", 150.0)
        # collision cost is a (min, max) scaling pair in the code
        # (col_cost_min / col_cost_max), not a single weight.
        self.declare_parameter("mppi.collision_cost_min", 10.0)
        self.declare_parameter("mppi.collision_cost_max", 50.0)
        self.declare_parameter("mppi.gate_cost_weight", 3.0)
        self.declare_parameter("mppi.buoy_cost_weight", 120.0)
        self.declare_parameter("mppi.speed_cost_weight", 0.1)
        self.declare_parameter("mppi.control_cost_weight", 0.2)
        # "safe distance" in the code is the CRM bumper ellipse, expressed as
        # per-side gains in multiples of LOA, plus buoy/gate geometry.
        self.declare_parameter("mppi.loa", 2.0)
        self.declare_parameter("mppi.safe_distance_right_loa", 3.2)
        self.declare_parameter("mppi.safe_distance_left_loa", 1.6)
        self.declare_parameter("mppi.safe_distance_fore_loa", 6.4)
        self.declare_parameter("mppi.safe_distance_aft_loa", 1.6)
        self.declare_parameter("mppi.gate_half_width_m", 4.0)
        self.declare_parameter("mppi.buoy_margin_m", 1.0)
        self.declare_parameter("mppi.buoy_longitudinal_sigma_m", 8.0)

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
        self.other_ship_timeout_sec = float(
            self.get_parameter("other_ship_timeout_sec").value
        )
        other_twist_is_relative = self.get_parameter("other_twist_is_relative").value
        reconnect_line_distance_m = self.get_parameter("reconnect_line_distance_m").value
        reconnect_ahead_length_m = self.get_parameter("reconnect_ahead_length_m").value
        straight_path_spacing_m = self.get_parameter("straight_path_spacing_m").value
        straight_path_length_m = self.get_parameter("straight_path_length_m").value
        mppi_smoothing_window = self.get_parameter("mppi_smoothing_window").value
        opponent_use_distance_m = self.get_parameter("opponent_use_distance_m").value
        opponent_passed_margin_m = self.get_parameter("opponent_passed_margin_m").value

        # Keyword names match MPPIPlanner's constructor arguments.
        mppi_params = {
            "horizon": int(self.get_parameter("mppi.horizon").value),
            "dt": float(self.get_parameter("mppi.dt").value),
            "num_samples": int(self.get_parameter("mppi.num_samples").value),
            "lambda_": float(self.get_parameter("mppi.lambda").value),
            "control_noise_sigma": [
                float(v) for v in self.get_parameter("mppi.control_noise_sigma").value
            ],
            "target_speed": float(self.get_parameter("mppi.target_speed").value),
            "path_cost_weight": float(self.get_parameter("mppi.path_cost_weight").value),
            "collision_cost_min": float(self.get_parameter("mppi.collision_cost_min").value),
            "collision_cost_max": float(self.get_parameter("mppi.collision_cost_max").value),
            "gate_cost_weight": float(self.get_parameter("mppi.gate_cost_weight").value),
            "buoy_cost_weight": float(self.get_parameter("mppi.buoy_cost_weight").value),
            "speed_cost_weight": float(self.get_parameter("mppi.speed_cost_weight").value),
            "control_cost_weight": float(self.get_parameter("mppi.control_cost_weight").value),
            "loa": float(self.get_parameter("mppi.loa").value),
            "safe_distance_right_loa": float(
                self.get_parameter("mppi.safe_distance_right_loa").value
            ),
            "safe_distance_left_loa": float(
                self.get_parameter("mppi.safe_distance_left_loa").value
            ),
            "safe_distance_fore_loa": float(
                self.get_parameter("mppi.safe_distance_fore_loa").value
            ),
            "safe_distance_aft_loa": float(
                self.get_parameter("mppi.safe_distance_aft_loa").value
            ),
            "gate_half_width_m": float(self.get_parameter("mppi.gate_half_width_m").value),
            "buoy_margin_m": float(self.get_parameter("mppi.buoy_margin_m").value),
            "buoy_longitudinal_sigma_m": float(
                self.get_parameter("mppi.buoy_longitudinal_sigma_m").value
            ),
        }

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
            mppi_params=mppi_params,
        )

        self.latest_own_odom = None
        self.latest_other_ship_twist = None
        self.latest_other_ship_received_at = None
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

    def own_odom_callback(self, msg: Odometry):
        self.latest_own_odom = msg

    def other_ship_twist_callback(self, msg: TwistStamped):
        self.latest_other_ship_twist = msg
        self.latest_other_ship_received_at = self.get_clock().now()

    def waypoint1_callback(self, msg: PoseStamped):
        self.latest_waypoint1_pose = msg

    def waypoint2_callback(self, msg: PoseStamped):
        self.latest_waypoint2_pose = msg

    def own_odom_in_planning_frame(self):
        """Return own odometry with its pose expressed in ``frame_id``.

        nav_msgs/Odometry pose is expressed in header.frame_id, while its
        twist is expressed in child_frame_id.  Only the pose is transformed;
        retaining the twist is therefore intentional.
        """
        source_frame = self.latest_own_odom.header.frame_id
        if not source_frame or source_frame == self.frame_id:
            return self.latest_own_odom

        try:
            tf_msg = self.tf_buffer.lookup_transform(
                self.frame_id,
                source_frame,
                Time(),
            )
        except TransformException as error:
            self.get_logger().warning(
                f"Cannot transform own odometry {source_frame} -> "
                f"{self.frame_id}: {error}",
                throttle_duration_sec=2.0,
            )
            return None

        pose = self.latest_own_odom.pose.pose
        transform = tf_msg.transform
        pose_yaw = yaw_from_quaternion(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        transform_yaw = yaw_from_quaternion(
            transform.rotation.x,
            transform.rotation.y,
            transform.rotation.z,
            transform.rotation.w,
        )
        x, y, yaw = transform_planar_pose(
            pose.position.x,
            pose.position.y,
            pose_yaw,
            transform.translation.x,
            transform.translation.y,
            transform_yaw,
        )

        transformed = copy.deepcopy(self.latest_own_odom)
        transformed.header.frame_id = self.frame_id
        transformed.pose.pose.position.x = x
        transformed.pose.pose.position.y = y
        transformed.pose.pose.position.z += transform.translation.z
        transformed.pose.pose.orientation.x = 0.0
        transformed.pose.pose.orientation.y = 0.0
        transformed.pose.pose.orientation.z = math.sin(yaw * 0.5)
        transformed.pose.pose.orientation.w = math.cos(yaw * 0.5)
        return transformed

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
        other_twist = self.latest_other_ship_twist

        if self.latest_other_ship_received_at is not None:
            age_sec = (
                self.get_clock().now() - self.latest_other_ship_received_at
            ).nanoseconds * 1e-9
            if age_sec > self.other_ship_timeout_sec:
                other_twist = None
                self.get_logger().warning(
                    f"Opponent update is stale ({age_sec:.1f} s); "
                    "planning without the opponent.",
                    throttle_duration_sec=2.0,
                )

        if other_twist is None:
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

        own_odom = self.own_odom_in_planning_frame()
        if own_odom is None:
            return

        path = self.trajectory_generator.generate(
            own_odom=own_odom,
            other_transform=other_transform,
            other_twist=other_twist,
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
