"""miniPC land-cart bringup for a scaled, rotated Task 4 route."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _include(package, launch_file, arguments):
    return GroupAction(scoped=True, actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare(package), "launch", launch_file])),
        launch_arguments=arguments.items(),
    )])


def generate_launch_description():
    transform_args = {
        "active_nav2_profile": "task3",
        "waypoint_transform_enabled": "true",
        "waypoint_transform_anchor_x": LaunchConfiguration("anchor_x"),
        "waypoint_transform_anchor_y": LaunchConfiguration("anchor_y"),
        "waypoint_transform_rotation_rad": LaunchConfiguration("rotation_rad"),
        "waypoint_transform_scale": LaunchConfiguration("scale"),
    }
    return LaunchDescription([
        LogInfo(msg="Cart-test stack ready; start Task 4 explicitly with njord-critical task start task4."),
        DeclareLaunchArgument("anchor_x", default_value="0.0"),
        DeclareLaunchArgument("anchor_y", default_value="0.0"),
        DeclareLaunchArgument("rotation_rad", default_value="0.0"),
        DeclareLaunchArgument("scale", default_value="0.1"),
        _include("robot", "minipc_bringup.launch.py", {
            "enable_mission_manager": "false", "enable_nav2": "false", "active_nav2_profile": "task3",
        }),
        _include("mission_manager", "mission.launch.py", transform_args),
    ])
