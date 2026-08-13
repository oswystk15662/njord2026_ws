"""Replay a Task 2 rosbag through the other-vessel perception pipeline.

This launch is perception-only: it does not start propulsion, navigation,
cameras, or physical sensors.  It supplies the TFs required by the recorded
LiDAR, then starts the Task 2 filter, clustering/tracking, and selector.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _include(package_name, launch_file, arguments):
    return GroupAction(
        scoped=True,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare(package_name), "launch", launch_file
                    ])
                ),
                launch_arguments=arguments.items(),
            )
        ],
    )


def _bag_player(context):
    """Resolve launch arguments into a non-interactive rosbag command."""
    storage_id = LaunchConfiguration("storage_id").perform(context)
    bag_path = LaunchConfiguration("bag_path").perform(context)
    playback_rate = LaunchConfiguration("playback_rate").perform(context)
    loop = LaunchConfiguration("loop").perform(context).lower() == "true"
    qos_profile = LaunchConfiguration("qos_profile").perform(context)

    # Auto-detection supports both MCAP and SQLite rosbag2 recordings.
    command = ["ros2", "bag", "play"]
    if storage_id:
        command.extend(["-s", storage_id])
    command.extend([
        bag_path,
        "--clock", "--rate", playback_rate,
        # Launch has no interactive TTY; this avoids rosbag's tcgetattr exit.
        "--disable-keyboard-controls",
    ])
    if loop:
        command.append("--loop")
    if qos_profile:
        command.extend(["--qos-profile-overrides-path", qos_profile])
    return [ExecuteProcess(cmd=command, output="screen")]


def generate_launch_description():
    tf_fallback = Node(
        package="robot",
        executable="bag_odometry_tf_fallback.py",
        name="bag_odometry_tf_fallback",
        output="screen",
        parameters=[{
            "ego_odom_topic": LaunchConfiguration("ego_odom_topic"),
            "odom_frame": "odom",
            "base_frame": "base_link",
        }],
    )

    perception = _include(
        "task2_perception", "task2_perception.launch.py",
        {
            "enable_cloud_filter": "true",
            "enable_opponent_selector": "true",
            "use_sim_time": "true",
            "ego_odom_topic": LaunchConfiguration("ego_odom_topic"),
            "map_frame": "odom",
            "base_frame": "base_link",
        },
    )
    tracking = _include(
        "ship_perception_bringup", "classical_pipeline.launch.py",
        {
            "lidar_topic": "/task2/points_filtered",
            "ego_odom_topic": LaunchConfiguration("ego_odom_topic"),
            "preprocessing_config_file": PathJoinSubstitution([
                FindPackageShare("task2_perception"), "config", "task2_params.yaml"
            ]),
            "segmentation_config_file": PathJoinSubstitution([
                FindPackageShare("task2_perception"), "config", "task2_params.yaml"
            ]),
            "tracker_config_file": PathJoinSubstitution([
                FindPackageShare("task2_perception"), "config", "task2_params.yaml"
            ]),
        },
    )
    waypoint_markers = Node(
        package="asv_trajectory_planner",
        executable="task2_waypoint_marker_publisher",
        name="task2_waypoint_marker_publisher",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "bag_path",
            description="rosbag2 directory or MCAP file to replay",
        ),
        DeclareLaunchArgument(
            "storage_id", default_value="",
            description="Optional rosbag storage plugin; empty auto-detects.",
        ),
        DeclareLaunchArgument("playback_rate", default_value="1.0"),
        DeclareLaunchArgument(
            "ego_odom_topic", default_value="/odometry/filtered/local",
            description="Own-vessel odometry used for tracking and TF fallback.",
        ),
        DeclareLaunchArgument(
            "loop", default_value="true",
            description="Repeat the recording after it ends.",
        ),
        DeclareLaunchArgument(
            "qos_profile",
            default_value=PathJoinSubstitution([
                FindPackageShare("robot"), "config", "collision_avoidance_qos.yaml"
            ]),
            description="rosbag QoS override YAML; set empty to disable.",
        ),
        tf_fallback,
        perception,
        tracking,
        waypoint_markers,
        OpaqueFunction(function=_bag_player),
    ])
