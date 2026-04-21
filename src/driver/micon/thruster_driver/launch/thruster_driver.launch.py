from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory("thruster_driver"),
        "config",
        "config.yaml",
    )

    config_file = LaunchConfiguration("config_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config,
                description="Path to thruster_driver config yaml",
            ),
            Node(
                package="thruster_driver",
                executable="thruster_driver_node",
                name="thruster_driver_node",
                output="screen",
                parameters=[config_file],
            ),
        ]
    )
