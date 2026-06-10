import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("path_follower")
    default_params = os.path.join(pkg_share, "config", "path_follower_params.yaml")

    params_arg = DeclareLaunchArgument(
        "params",
        default_value=default_params,
        description="Path to path_follower parameter yaml",
    )

    node = Node(
        package="path_follower",
        executable="path_follower_node",
        name="path_follower_node",
        parameters=[LaunchConfiguration("params")],
        output="screen",
    )

    return LaunchDescription([
        params_arg,
        node,
    ])