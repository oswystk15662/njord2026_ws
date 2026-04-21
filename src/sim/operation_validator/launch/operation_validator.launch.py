import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("operation_validator")
    default_config = os.path.join(pkg_share, "config", "validator_params.yaml")

    config_arg = DeclareLaunchArgument("params", default_value=default_config)

    node = Node(
        package="operation_validator",
        executable="operation_validator_node",
        name="operation_validator_node",
        parameters=[LaunchConfiguration("params")],
        output="screen",
    )

    return LaunchDescription([config_arg, node])
