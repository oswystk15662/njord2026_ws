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

    back_cam_jpeg_receiver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(zed2i_share_path, "launch", "ground_video_receiver.launch.py")
        ),
        launch_arguments={
            "port": "5602",
            "topic": "/ground_video/back_cam_jpeg/compressed",
        }.items(),
    )

    back_cam_h26x_receiver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(zed2i_share_path, "launch", "ground_h26x_receiver.launch.py")
        ),
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
        emulate_tty=True,
    )

    ground_station_heartbeat = Node(
        package="simple_manual",
        executable="ground_station_heartbeat_node",
        name="ground_station_heartbeat",
        output="screen",
        parameters=[{"topic": "/heartbeat/ground_station", "period_sec": 1.0}],
    )

    foxglove_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("foxglove_bridge"), "launch", "foxglove_bridge.launch.py")
        ),
    )

    return LaunchDescription(
        [
            joy_node,
            ground_station_heartbeat,
            ground_video_receiver_launch,
            back_cam_h26x_receiver_launch,
            back_cam_jpeg_receiver_launch,
            foxglove_bridge_launch,
        ]
    )
