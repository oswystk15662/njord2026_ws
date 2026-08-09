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
    params = PathJoinSubstitution([FindPackageShare("task2_perception"), "config", "task2_params.yaml"])
    perception = _include("task2_perception", "task2_perception.launch.py", {
        "enable_cloud_filter": "true", "enable_opponent_selector": tracking,
        "publish_self_marker": "false", "ego_odom_topic": odom,
        "motion_filter_mode": "straight_line",
    })
    preprocessing = _include("pcl_preprocessing", "preprocessing.launch.py", {
        "config_file": params, "input_topic": "/task2/points_filtered", "output_topic": "/pcl/preprocessed",
    })
    segmentation = _include("pcl_segmentation", "segmentation.launch.py", {
        "config_file": params, "use_color": "false",
    })
    ship_tracking = _include("ship_tracking", "tracker.launch.py", {
        "config_file": params, "ego_odom_topic": odom,
    })
    planner = _include("asv_trajectory_planner", "planner_real.launch.py", {
        "own_odom_topic": odom, "mission_gate_required": "false",
        "start_follow_path_client": "false", "start_waypoint_pose_publisher": "false",
    })
    safety_points = Node(
        package="task2_perception", executable="task2_cloud_filter_node",
        name="task2_safety_points", output="screen",
        parameters=[{
            "input_topic": "/livox/lidar", "output_topic": "/task2/safety_points",
            "visual_output_topic": "/task2/safety_points_visual", "output_frame": "base_link",
            "min_range_m": 0.5, "max_range_m": 15.0, "voxel_leaf_size_m": 0.30,
            "accumulation_frames": 1, "process_rate_hz": 5.0,
            "publish_visual_z_mirror": False, "publish_self_marker": False,
            "publish_debug": False,
        }],
    )
    return LaunchDescription([
        DeclareLaunchArgument("own_odom_topic", default_value="/odometry/filtered/global"),
        DeclareLaunchArgument("enable_ship_tracking", default_value="true"),
        perception,
        GroupAction(condition=IfCondition(tracking), actions=[preprocessing, segmentation, ship_tracking]),
        planner, safety_points,
    ])
