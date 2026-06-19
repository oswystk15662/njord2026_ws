from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory("thruster_driver"),
        "config",
        "config.yaml",
    )
    default_robot_description = os.path.join(
        get_package_share_directory("robot"),
        "urdf",
        "robot.urdf_modified.urdf",
    )

    config_file = LaunchConfiguration("config_file")
    robot_description_file = LaunchConfiguration("robot_description_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config,
                description="Path to thruster_driver config yaml",
            ),
            DeclareLaunchArgument(
                "robot_description_file",
                default_value=default_robot_description,
                description="Path to robot URDF containing fixed thruster poses",
            ),
            Node(
                package="thruster_driver",
                executable="thruster_driver_node",
                name="thruster_driver_node",
                output="screen",
                parameters=[
                    config_file,
                    {
                        "robot_description": ParameterValue(
                            Command(["cat ", robot_description_file]),
                            value_type=str,
                        ),
                    },
                ],
            ),
        ]
    )
