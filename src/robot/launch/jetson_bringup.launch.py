"""Jetson-side bringup: MID360S LiDAR + GLIM + buoy detection, and ZED 2i.

Jetson Orin Nano Super hosts CUDA/TensorRT/ZED SDK and owns the USB ZED 2i
camera and the Ethernet-attached Livox MID360S. Nothing else (no TF
publisher, no EKF, no thrusters/micon, no joy) belongs in this file: those
live on the miniPC side (see minipc_bringup.launch.py).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def include_launch(package_name, path_parts, condition, launch_arguments=None):
    # Keep included launch files' generic argument names (e.g. camera_resolution)
    # scoped to this include so they cannot leak into sibling includes.
    return GroupAction(
        scoped=True,
        condition=condition,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare(package_name)] + path_parts)
                ),
                launch_arguments=(launch_arguments or {}).items(),
            )
        ],
    )


def generate_launch_description():
    default_engine = os.path.join(
        get_package_share_directory("robot"), "config", "yolo_model", "best.engine"
    )

    lidar_model = LaunchConfiguration("lidar_model")
    enable_mid360 = LaunchConfiguration("enable_mid360")
    enable_zed2i = LaunchConfiguration("enable_zed2i")
    enable_glim = LaunchConfiguration("enable_glim")
    glim_backend = LaunchConfiguration("glim_backend")
    enable_pcl_buoy_detection = LaunchConfiguration("enable_pcl_buoy_detection")
    enable_gpu_perception = LaunchConfiguration("enable_gpu_perception")
    engine_path = LaunchConfiguration("engine_path")
    camera_resolution = LaunchConfiguration("camera_resolution")
    camera_framerate = LaunchConfiguration("camera_framerate")
    enable_ground_video = LaunchConfiguration("enable_ground_video")
    ground_video_host = LaunchConfiguration("ground_video_host")
    ground_video_port = LaunchConfiguration("ground_video_port")
    ground_video_width = LaunchConfiguration("ground_video_width")
    ground_video_height = LaunchConfiguration("ground_video_height")
    ground_video_fps = LaunchConfiguration("ground_video_fps")

    mid360_launch = include_launch(
        "robot",
        ["launch", "lidar.launch.py"],
        IfCondition(enable_mid360),
        {
            "lidar_model": lidar_model,
            "enable_buoy_detection": enable_pcl_buoy_detection,
            "enable_glim": enable_glim,
            "glim_backend": glim_backend,
        },
    )

    zed2i_launch = include_launch(
        "zed2i_driver",
        ["launch", "zed2i.launch.py"],
        IfCondition(enable_zed2i),
        {
            "mode": "sdk",
            "enable_gpu_perception": enable_gpu_perception,
            "engine_path": engine_path,
            "camera_resolution": camera_resolution,
            "framerate": camera_framerate,
            "enable_ground_video": enable_ground_video,
            "ground_video_host": ground_video_host,
            "ground_video_port": ground_video_port,
            "ground_video_width": ground_video_width,
            "ground_video_height": ground_video_height,
            "ground_video_fps": ground_video_fps,
        },
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "lidar_model", default_value="mid360s", choices=["mid360", "mid360s"]
            ),
            DeclareLaunchArgument("enable_mid360", default_value="true"),
            DeclareLaunchArgument("enable_zed2i", default_value="true"),
            DeclareLaunchArgument(
                "enable_glim",
                default_value="true",
                description="Load GLIM into the Livox component container",
            ),
            DeclareLaunchArgument(
                "glim_backend",
                default_value="gpu",
                choices=["gpu", "cpu"],
                description="Select the GLIM config directory (glim_config for gpu, glim_config_cpu for cpu)",
            ),
            DeclareLaunchArgument("enable_pcl_buoy_detection", default_value="true"),
            DeclareLaunchArgument("enable_gpu_perception", default_value="true"),
            DeclareLaunchArgument("engine_path", default_value=default_engine),
            DeclareLaunchArgument(
                "camera_resolution",
                default_value="HD720",
                description="ZED camera resolution: HD2K, HD1080, HD720, or VGA",
            ),
            DeclareLaunchArgument("camera_framerate", default_value="15"),
            DeclareLaunchArgument("enable_ground_video", default_value="false"),
            DeclareLaunchArgument(
                "ground_video_host",
                default_value="osw-Stealth-14-AI-Studio-A1VGG.local",
            ),
            DeclareLaunchArgument("ground_video_port", default_value="5600"),
            DeclareLaunchArgument("ground_video_width", default_value="480"),
            DeclareLaunchArgument("ground_video_height", default_value="360"),
            DeclareLaunchArgument("ground_video_fps", default_value="4.0"),
            # Staged startup, carried over from manual_control.launch.py. On a
            # dedicated Jetson there is far less contention than in the old
            # single-machine setup, so both default to 0.0 (start immediately);
            # standalone_bringup.launch.py passes the original 18.0/20.0 values
            # to reproduce the pre-split behaviour.
            DeclareLaunchArgument(
                "lidar_start_delay",
                default_value="0.0",
                description="Seconds to wait before starting the MID360 container",
            ),
            DeclareLaunchArgument(
                "camera_start_delay",
                default_value="0.0",
                description="Seconds to wait before starting the ZED 2i driver",
            ),
            TimerAction(
                period=LaunchConfiguration("lidar_start_delay"),
                actions=[mid360_launch],
            ),
            TimerAction(
                period=LaunchConfiguration("camera_start_delay"),
                actions=[zed2i_launch],
            ),
        ]
    )
