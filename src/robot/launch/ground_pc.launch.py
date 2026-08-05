import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    zed2i_share = FindPackageShare("zed2i_driver")
    zed2i_share_path = get_package_share_directory("zed2i_driver")

    ground_video_receiver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(zed2i_share_path, "launch", "ground_video_receiver.launch.py")
        ),
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
        emulate_tty=True,
    )

    foxglove_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("foxglove_bridge"), "launch", "foxglove_bridge.launch.py")
        ),
    )

    return LaunchDescription(
        [
            joy_node,
            ground_video_receiver_launch,
            foxglove_bridge_launch,
        ]
    )
