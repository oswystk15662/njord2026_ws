"""Jetson half of Task 2: LiDAR perception, tracking and MPPI planning."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _include(package, launch_file, arguments):
    return GroupAction(scoped=True, actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare(package), "launch", launch_file])),
        launch_arguments=arguments.items(),
    )])


def generate_launch_description():
    odom = LaunchConfiguration("own_odom_topic")
    tracking = LaunchConfiguration("enable_ship_tracking")
    params = LaunchConfiguration("params_file")
    perception = _include("task2_perception", "task2_perception.launch.py", {
        "enable_cloud_filter": "true", "enable_opponent_selector": tracking,
        "publish_self_marker": "false", "ego_odom_topic": odom,
        "params_file": params,
    })
    preprocessing = _include("pcl_preprocessing", "preprocessing.launch.py", {
        "config_file": params, "input_topic": "/task2/points_filtered", "output_topic": "/pcl/preprocessed",
    })
    segmentation = _include("pcl_segmentation", "segmentation.launch.py", {
        "config_file": params, "use_color": "false",
    })
    ship_tracking = Node(
        package="ship_tracking", executable="ship_tracker_node", name="ship_tracker_node",
        output="screen", parameters=[params],
        remappings=[
            ("input/cluster_observations", "/pcl/cluster_observations"),
            ("input/ego_odometry", odom),
            ("output/tracked_objects", "/tracked_objects"),
            ("output/tracked_objects/markers", "/tracked_objects/markers"),
        ],
    )
    planner = _include("asv_trajectory_planner", "planner_real.launch.py", {
        "own_odom_topic": odom, "mission_gate_required": "false",
        "task2_params_file": params,
        "start_follow_path_client": "false", "start_waypoint_pose_publisher": "false",
    })
    safety_points = Node(
        package="task2_perception", executable="task2_cloud_filter_node",
        name="task2_safety_cloud_filter", output="screen", parameters=[params],
    )
    return LaunchDescription([
        DeclareLaunchArgument("own_odom_topic", default_value="/odometry/filtered/global"),
        DeclareLaunchArgument("enable_ship_tracking", default_value="true"),
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("task2_perception"), "config", "task2_params.yaml"]),
            description="Single Task 2 perception/opponent tuning YAML.",
        ),
        perception,
        GroupAction(condition=IfCondition(tracking), actions=[preprocessing, segmentation, ship_tracking]),
        planner, safety_points,
    ])
