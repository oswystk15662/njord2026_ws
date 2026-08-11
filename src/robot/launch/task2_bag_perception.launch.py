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
    ego_odom_topic = "/task2/ego_odom"
    odom_selector = Node(
        package="robot",
        executable="bag_odometry_selector.py",
        name="bag_odometry_selector",
        output="screen",
        parameters=[LaunchConfiguration("params_file"), {"use_sim_time": True}],
    )
    tf_fallback = Node(
        package="robot",
        executable="bag_odometry_tf_fallback.py",
        name="bag_odometry_tf_fallback",
        output="screen",
        parameters=[{
            "ego_odom_topic": ego_odom_topic,
            "odom_frame": "odom",
            "base_frame": "base_link",
        }],
    )

    perception = _include(
        "task2_perception", "task2_perception.launch.py",
        {
            "enable_cloud_filter": "true",
            "enable_opponent_selector": "true",
            "params_file": LaunchConfiguration("params_file"),
            "use_sim_time": "true",
            "ego_odom_topic": ego_odom_topic,
            "map_frame": "odom",
            "base_frame": "base_link",
        },
    )
    tracking = GroupAction(scoped=True, actions=[
        _include("pcl_preprocessing", "preprocessing.launch.py", {
            "config_file": LaunchConfiguration("params_file"),
            "input_topic": "/task2/points_filtered", "output_topic": "/pcl/preprocessed",
        }),
        _include("pcl_segmentation", "segmentation.launch.py", {
            "config_file": LaunchConfiguration("params_file"), "use_color": "false",
        }),
        Node(
            package="ship_tracking", executable="ship_tracker_node", name="ship_tracker_node",
            output="screen", parameters=[LaunchConfiguration("params_file")],
            remappings=[
                ("input/cluster_observations", "/pcl/cluster_observations"),
                ("input/ego_odometry", ego_odom_topic),
                ("output/tracked_objects", "/tracked_objects"),
                ("output/tracked_objects/markers", "/tracked_objects/markers"),
            ],
        ),
    ])

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
            "params_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("task2_perception"), "config", "task2_params.yaml"
            ]),
            description="Single Task 2 perception/opponent tuning YAML.",
        ),
        odom_selector,
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
        OpaqueFunction(function=_bag_player),
    ])
