"""Real-vessel Task2 MPPI planner launch.

Starts ONLY the nodes needed on the real vessel:
    - task2_waypoint_pose_publisher (waypoint source from the shared
      waypoint_publisher config; see that node's docstring for the
      waypoint-source decision)
    - planner_node (MPPI, parameters from config/mppi_params.yaml)
    - path_pruner_node
    - follow_path_client_node

NO simulation bridge nodes are started here (task2_gps_waypoint_publisher and
opponent_twist_from_tf_node are sim-only; on the real vessel /other_ship/twist
and the map->opponent_vessel TF come from the task2_perception /
ship_perception_bringup pipeline).

Topic wiring intentionally mirrors planner_with_follow_path.launch.py:
planner_node publishes directly to /planned_path_pruned (path_pruner_node is
kept running but effectively bypassed), preserving the wiring validated in
simulation.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    own_odom_topic = LaunchConfiguration("own_odom_topic")

    mppi_params_file = PathJoinSubstitution(
        [
            FindPackageShare("asv_trajectory_planner"),
            "config",
            "mppi_params.yaml",
        ]
    )

    task2_waypoint_pose_publisher = Node(
        package="asv_trajectory_planner",
        executable="task2_waypoint_pose_publisher",
        name="task2_waypoint_pose_publisher",
        output="screen",
        parameters=[
            {
                "config_package": "waypoint_publisher",
                "config_file": "task2_waypoints.yaml",
                "config_key": "task2_config",
                "waypoint1_topic": "/waypoint1_pose",
                "waypoint2_topic": "/waypoint2_pose",
                "frame_id": "map",
                "publish_frequency": 2.0,
            }
        ],
    )

    planner_node = Node(
        package="asv_trajectory_planner",
        executable="planner_node",
        name="planner_node",
        output="screen",
        parameters=[
            # MPPI hyperparameters (defaults = historical hardcoded values).
            mppi_params_file,
            # Same planner params as planner_with_follow_path.launch.py,
            # except own_odom_topic which is configurable for the real stack.
            {
                "own_odom_topic": own_odom_topic,
                "other_ship_twist_topic": "/other_ship/twist",
                "waypoint1_topic": "/waypoint1_pose",
                "waypoint2_topic": "/waypoint2_pose",
                "path_topic": "/planned_path_pruned",

                "own_frame": "base_link",
                "other_ship_frame": "opponent_vessel",

                "frame_id": "map",
                "planning_frequency": 2.0,
                "point_spacing": 0.5,
                "avoid_radius": 2.0,
                "avoid_offset": 3.0,

                # 相手船がいれば見る。いなければ相手船なしでPathを出す。
                "require_other_ship": False,
                "other_ship_timeout_sec": 2.0,

                # /other_ship/twist は map基準で出す
                "other_twist_is_relative": False,
                "opponent_use_distance_m": 20.0,
                "opponent_passed_margin_m": 10.0,
                "reconnect_line_distance_m": 1.0,
                "reconnect_ahead_length_m": 5.0,
                "straight_path_spacing_m": 2.0,
                "straight_path_length_m": 60.0,
                "mppi_smoothing_window": 3,
            },
        ],
    )

    path_pruner_node = Node(
        package="asv_trajectory_planner",
        executable="path_pruner_node",
        name="path_pruner_node",
        output="screen",
        parameters=[
            {
                "input_path_topic": "/planned_path",
                "output_path_topic": "/planned_path_pruned",
                "skip_points_after_closest": 0,
                "prepend_current_pose": True,
                "min_output_points": 3,
                "min_point_spacing_m": 0.5,
                "odom_topic": own_odom_topic,
            }
        ],
    )

    follow_path_client_node = Node(
        package="asv_trajectory_planner",
        executable="follow_path_client_node",
        name="follow_path_client_node",
        output="screen",
        parameters=[
            {
                "path_topic": "/planned_path_pruned",
                "action_name": "/follow_path",
                "controller_id": "FollowPath",
                "goal_checker_id": "general_goal_checker",
                "progress_checker_id": "progress_checker",

                # goalを送り直しすぎるとNav2がabortしやすいので抑制
                "send_frequency": 1.0,
                "enable_replanning": True,
            }
        ],
    )

    return LaunchDescription(
        [
            # GLIM publishes /odom on the real vessel (same topic as sim).
            DeclareLaunchArgument("own_odom_topic", default_value="/odom"),
            task2_waypoint_pose_publisher,
            planner_node,
            path_pruner_node,
            follow_path_client_node,
        ]
    )
