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
    front_video_port = LaunchConfiguration("front_video_port")
    front_video_topic = LaunchConfiguration("front_video_topic")
    back_video_port = LaunchConfiguration("back_video_port")
    back_video_codec = LaunchConfiguration("back_video_codec")
    back_video_topic = LaunchConfiguration("back_video_topic")

    ground_video_receiver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(zed2i_share_path, "launch", "ground_video_receiver.launch.py")
        ),
        launch_arguments={
            "port": front_video_port,
            "topic": front_video_topic,
        }.items(),
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
        launch_arguments={
            "port": back_video_port,
            "codec": back_video_codec,
            "topic": back_video_topic,
        }.items(),
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
            # The two receivers ingest different RTP streams and must keep
            # separate UDP ports and ROS topics.
            DeclareLaunchArgument("front_video_port", default_value="5600"),
            DeclareLaunchArgument(
                "front_video_topic", default_value="/ground_video/image/compressed"
            ),
            DeclareLaunchArgument("back_video_port", default_value="5601"),
            DeclareLaunchArgument(
                "back_video_codec", default_value="h264", choices=["h264", "h265"]
            ),
            DeclareLaunchArgument(
                "back_video_topic", default_value="/ground_video/back_cam/image_raw"
            ),
            joy_node,
            ground_station_heartbeat,
            ground_video_receiver_launch,
            back_cam_h26x_receiver_launch,
            back_cam_jpeg_receiver_launch,
            foxglove_bridge_launch,
        ]
    )
