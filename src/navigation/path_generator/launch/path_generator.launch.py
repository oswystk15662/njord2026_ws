import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("path_generator")
    default_params = os.path.join(pkg_share, "config", "path_generator_params.yaml")

    params_arg = DeclareLaunchArgument(
        "params",
        default_value=default_params,
    )

    node = Node(
        package="path_generator",
        executable="path_generator_node",
        name="path_generator_node",
        parameters=[LaunchConfiguration("params")],
        output="screen",
    )

    return LaunchDescription([params_arg, node])