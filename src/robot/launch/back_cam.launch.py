from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("video_device", default_value="/dev/video0"),
            DeclareLaunchArgument("image_width", default_value="640"),
            DeclareLaunchArgument("image_height", default_value="480"),
            DeclareLaunchArgument("framerate", default_value="30.0"),
            DeclareLaunchArgument("pixel_format", default_value="mjpeg2rgb"),
            Node(
                package="usb_cam",
                executable="usb_cam_node_exe",
                name="back_cam",
                namespace="back_cam",
                output="screen",
                parameters=[
                    {
                        "video_device": LaunchConfiguration("video_device"),
                        "image_width": LaunchConfiguration("image_width"),
                        "image_height": LaunchConfiguration("image_height"),
                        "framerate": LaunchConfiguration("framerate"),
                        "pixel_format": LaunchConfiguration("pixel_format"),
                        "camera_name": "back_cam",
                        "frame_id": "back_cam_link",
                    }
                ],
            ),
        ]
    )
