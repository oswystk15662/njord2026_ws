import os

from ament_index_python.packages import get_package_share_directory
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
                LaunchConfiguration("params_file"),
                {
                    "video_device": video_device,
                }
            ],
        )
    ]


def generate_launch_description():
    default_params_file = os.path.join(
        get_package_share_directory("robot"), "config", "back_cam.yaml"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params_file,
                description="Path to the usb_cam ROS parameter YAML file.",
            ),
            # /dev/videoN indices shift on reconnect/reboot; use the stable
            # by-id path for the Adesso CyberTrack H7 (back_cam).
            DeclareLaunchArgument(
                "video_device",
                default_value="/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._Adesso_CyberTrack_H7_SN0001-video-index0",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
