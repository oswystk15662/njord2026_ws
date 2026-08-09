import os
import subprocess
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def configure_v4l2_controls(video_device, params):
    """Apply controls whose names differ between usb_cam and V4L2."""
    if "autoexposure" not in params:
        return

    auto_exposure = 3 if params["autoexposure"] else 1
    commands = [
        [
            "v4l2-ctl",
            f"--device={video_device}",
            f"--set-ctrl=auto_exposure={auto_exposure}",
        ]
    ]
    if not params["autoexposure"] and "exposure" in params:
        commands.extend(
            [
                [
                    "v4l2-ctl",
                    f"--device={video_device}",
                    "--set-ctrl=exposure_dynamic_framerate=0",
                ],
                [
                    "v4l2-ctl",
                    f"--device={video_device}",
                    f"--set-ctrl=exposure_time_absolute={params['exposure']}",
                ],
            ]
        )

    for command in commands:
        subprocess.run(command, check=True)


def launch_setup(context, *args, **kwargs):
    # usb_cam_node_exe naively prepends "/dev/" to the by-id symlink's
    # relative target (e.g. "../../video2"), producing a broken path like
    # "/dev/../../video2". Resolve the by-id path to the real /dev/videoN
    # here at launch time so usb_cam always receives a real device path,
    # while the by-id path stays the stable source of truth for which
    # physical camera to use (survives /dev/videoN renumbering on
    # reconnect/reboot without needing udev/sudo).
    params_file = LaunchConfiguration("params_file").perform(context)
    with open(params_file, encoding="utf-8") as file:
        params = yaml.safe_load(file)["/back_cam/back_cam"]["ros__parameters"]

    # A video_device launch argument is useful for one-off tests. Otherwise,
    # take the device path from the selected parameter file.
    video_device_arg = LaunchConfiguration("video_device").perform(context)
    video_device = os.path.realpath(video_device_arg or params["video_device"])
    configure_v4l2_controls(video_device, params)

    return [
        Node(
            package="usb_cam",
            executable="usb_cam_node_exe",
            name="back_cam",
            namespace="back_cam",
            output="screen",
            parameters=[
                params_file,
                {
                    "video_device": video_device,
                }
            ],
        )
    ]


def generate_launch_description():
    # This relative path works both from the source tree and after ament
    # installs launch/ and config/ as siblings under share/robot/.
    default_params_file = os.path.abspath(
        os.path.join(os.path.dirname(__file__), "..", "config", "back_cam.yaml")
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
                default_value="",
                description="Optional device-path override for the selected parameter file.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
