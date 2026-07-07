import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("catmull_rom_path_smoother")
    default_params = os.path.join(pkg_share, "config", "catmull_rom_params.yaml")

    params_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params,
        description="Path to Catmull-Rom smoother parameter file",
    )

    smoother_node = Node(
        package="catmull_rom_path_smoother",
        executable="catmull_rom_path_smoother_node",
        name="catmull_rom_path_smoother",
        output="screen",
        parameters=[LaunchConfiguration("params_file")],
    )

    return LaunchDescription([params_arg, smoother_node])
