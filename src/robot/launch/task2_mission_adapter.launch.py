"""Persistent Task 2 perception and MPPI adapter.

Hardware and the final command path remain owned by role bringup.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _include(package_name, launch_file, arguments=None):
    return GroupAction(
        scoped=True,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare(package_name), "launch", launch_file]
                    )
                ),
                launch_arguments=(arguments or {}).items(),
            )
        ],
    )


def generate_launch_description():
    own_odom_topic = LaunchConfiguration("own_odom_topic")
    enable_ship_tracking = LaunchConfiguration("enable_ship_tracking")
    motion_mode = LaunchConfiguration("opponent_motion_mode")
    perception = _include(
        "task2_perception",
        "task2_perception.launch.py",
        {
            "enable_cloud_filter": "true",
            "enable_opponent_selector": enable_ship_tracking,
            "publish_self_marker": "false",
            "ego_odom_topic": own_odom_topic,
            "motion_filter_mode": motion_mode,
        },
    )
    preprocessing = _include(
        "pcl_preprocessing",
        "preprocessing.launch.py",
        {
            "config_file": PathJoinSubstitution(
                [FindPackageShare("task2_perception"), "config", "task2_params.yaml"]),
            "input_topic": "/task2/points_filtered",
            "output_topic": "/pcl/preprocessed",
        },
    )
    segmentation = _include(
        "pcl_segmentation",
        "segmentation.launch.py",
        {
            "config_file": PathJoinSubstitution(
                [FindPackageShare("task2_perception"), "config", "task2_params.yaml"]),
            "use_color": "false",
        },
    )
    tracking = _include(
        "ship_tracking",
        "tracker.launch.py",
        {
            "config_file": PathJoinSubstitution(
                [FindPackageShare("task2_perception"), "config", "task2_params.yaml"]),
            "ego_odom_topic": own_odom_topic,
        },
    )
    mppi = _include(
        "asv_trajectory_planner",
        "planner_real.launch.py",
        {"own_odom_topic": own_odom_topic, "mission_gate_required": "true"},
    )
    ready = Node(
        package="asv_trajectory_planner",
        executable="task2_autonomy_ready_node",
        name="task2_autonomy_ready",
        output="screen",
        parameters=[
            {
                "path_topic": "/planned_path_pruned",
                "action_name": "/follow_path",
                "path_timeout_sec": 2.0,
            }
        ],
    )
    return LaunchDescription([
        DeclareLaunchArgument("own_odom_topic", default_value="/odometry/filtered/global"),
        DeclareLaunchArgument("enable_ship_tracking", default_value="true"),
        DeclareLaunchArgument(
            "opponent_motion_mode",
            default_value="straight_line",
            choices=["standard", "straight_line"],
        ),
        perception,
        GroupAction(
            condition=IfCondition(enable_ship_tracking),
            actions=[preprocessing, segmentation, tracking],
        ),
        mppi,
        ready,
    ])
