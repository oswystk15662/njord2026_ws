"""Persistent Mission Manager; it never submits a task at startup."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("active_nav2_profile", default_value=""),
        Node(package="mission_manager", executable="runtime_manager_node", name="runtime_manager", output="screen"),
        Node(package="mission_manager", executable="operator_dispatcher_node", name="operator_dispatcher", output="screen"),
        Node(
            package="mission_manager",
            executable="ground_link_return_monitor_node",
            name="ground_link_return_monitor",
            output="screen",
        ),
        Node(
            package="mission_manager",
            executable="task2_readiness_adapter_node",
            name="task2_readiness_adapter",
            output="screen",
            condition=IfCondition(PythonExpression(["'", LaunchConfiguration("active_nav2_profile"), "' == 'task2'"])),
        ),
        Node(
            package="mission_manager",
            executable="mission_manager_node",
            name="mission_manager",
            output="screen",
            parameters=[{"active_nav2_profile": LaunchConfiguration("active_nav2_profile")}],
        ),
    ])
