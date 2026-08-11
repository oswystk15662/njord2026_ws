"""Persistent Mission Manager; it never submits a task at startup."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("active_nav2_profile", default_value=""),
        DeclareLaunchArgument(
            "ground_link_return_monitor_log_level", default_value="info"
        ),
        Node(package="mission_manager", executable="runtime_manager_node", name="runtime_manager", output="screen"),
        Node(package="mission_manager", executable="operator_dispatcher_node", name="operator_dispatcher", output="screen"),
        Node(
            package="mission_manager",
            executable="ground_link_return_monitor_node",
            name="ground_link_return_monitor",
            output="screen",
            arguments=[
                "--ros-args",
                "--log-level",
                LaunchConfiguration("ground_link_return_monitor_log_level"),
            ],
        ),
        Node(
            package="mission_manager",
            executable="mission_manager_node",
            name="mission_manager",
            output="screen",
            parameters=[{"active_nav2_profile": LaunchConfiguration("active_nav2_profile")}],
        ),
    ])
