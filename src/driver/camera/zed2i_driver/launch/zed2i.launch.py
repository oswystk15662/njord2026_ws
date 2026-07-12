import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    mode = LaunchConfiguration("mode").perform(context)
    if mode not in ("cpu", "sdk"):
        raise RuntimeError("mode must be either 'cpu' or 'sdk'")

    executable = "zed2i_cpu_node" if mode == "cpu" else "zed2i_sdk_node"
    return [
        Node(
            package="zed2i_driver",
            executable=executable,
            name="zed2i",
            namespace=LaunchConfiguration("namespace"),
            output="screen",
            parameters=[
                LaunchConfiguration("params_file"),
            ],
        )
    ]


def generate_launch_description():
    default_params_file = os.path.join(
        get_package_share_directory("zed2i_driver"), "config", "zed2i_jetson_orin_nano.yaml"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("mode", default_value="sdk"),
            DeclareLaunchArgument("namespace", default_value="/zed2i"),
            DeclareLaunchArgument("params_file", default_value=default_params_file),
            OpaqueFunction(function=launch_setup),
        ]
    )
