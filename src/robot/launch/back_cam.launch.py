import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    # usb_cam_node_exe naively prepends "/dev/" to the by-id symlink's
    # relative target (e.g. "../../video2"), producing a broken path like
    # "/dev/../../video2". Resolve the by-id path to the real /dev/videoN
    # here at launch time so usb_cam always receives a real device path,
    # while the by-id path stays the stable source of truth for which
    # physical camera to use (survives /dev/videoN renumbering on
    # reconnect/reboot without needing udev/sudo).
    video_device = os.path.realpath(LaunchConfiguration("video_device").perform(context))

    return [
        Node(
            package="usb_cam",
            executable="usb_cam_node_exe",
            name="back_cam",
            namespace="back_cam",
            output="screen",
            parameters=[
                {
                    "video_device": video_device,
                    "image_width": LaunchConfiguration("image_width"),
                    "image_height": LaunchConfiguration("image_height"),
                    "framerate": LaunchConfiguration("framerate"),
                    "pixel_format": LaunchConfiguration("pixel_format"),
                    "camera_name": "back_cam",
                    "frame_id": "back_cam_link",
                    "brightness": 0,
                    "contrast": 5,
                    "gain": 0,
                    "autoexposure": False,
                    "exposure": 1,
                }
            ],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            # /dev/videoN indices shift on reconnect/reboot; use the stable
            # by-id path for the Adesso CyberTrack H7 (back_cam).
            DeclareLaunchArgument(
                "video_device",
                default_value="/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._Adesso_CyberTrack_H7_SN0001-video-index0",
            ),
            DeclareLaunchArgument("image_width", default_value="640"),
            DeclareLaunchArgument("image_height", default_value="480"),
            DeclareLaunchArgument("framerate", default_value="30.0"),
            DeclareLaunchArgument("pixel_format", default_value="mjpeg2rgb"),
            OpaqueFunction(function=launch_setup),
        ]
    )
