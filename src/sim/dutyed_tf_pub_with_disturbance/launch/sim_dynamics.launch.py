import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("dutyed_tf_pub_with_disturbance")
    default_config = os.path.join(pkg_share, "config", "node_config.yaml")

    config_arg = DeclareLaunchArgument(
        "config",
        default_value=default_config,
        description="Path to node parameter yaml",
    )

    node = Node(
        package="dutyed_tf_pub_with_disturbance",
        executable="dutyed_tf_pub_with_disturbance_node",
        name="dutyed_tf_pub_with_disturbance_node",
        parameters=[LaunchConfiguration("config")],
        output="screen",
    )

    return LaunchDescription([config_arg, node])
