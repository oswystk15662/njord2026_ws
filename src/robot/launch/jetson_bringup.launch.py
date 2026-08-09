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
from launch_ros.actions import Node


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
    glim_headless = LaunchConfiguration("glim_headless")
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
    heartbeat_monitor_zed2i = LaunchConfiguration("heartbeat_monitor_zed2i")
    heartbeat_monitor_lidar = LaunchConfiguration("heartbeat_monitor_lidar")
    enable_task1_safety_points = LaunchConfiguration("enable_task1_safety_points")

    mid360_launch = include_launch(
        "robot",
        ["launch", "lidar.launch.py"],
        IfCondition(enable_mid360),
        {
            "lidar_model": lidar_model,
            "enable_buoy_detection": enable_pcl_buoy_detection,
            "enable_glim": enable_glim,
            "glim_headless": glim_headless,
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

    heartbeat_launch = include_launch(
        "robot",
        ["launch", "heartbeat.launch.py"],
        IfCondition(LaunchConfiguration("enable_heartbeats")),
        {
            "role": "jetson",
            "heartbeat_monitor_zed2i": heartbeat_monitor_zed2i,
            "heartbeat_monitor_lidar": heartbeat_monitor_lidar,
        },
    )

    # Do not forward raw depth or LiDAR clouds to the miniPC. This compact,
    # base_link-frame cloud is generated from the ZED2i depth point cloud and
    # is the only near-field obstacle feed sent across the link for Task 1.
    task1_safety_points = Node(
        package="task2_perception",
        executable="task2_cloud_filter_node",
        name="task1_safety_points",
        output="screen",
        parameters=[{
            "input_topic": "/zed2i/points",
            "output_topic": "/task1/safety_points",
            "visual_output_topic": "/task1/safety_points_visual",
            "publish_visual_z_mirror": False,
            "output_frame": "base_link",
            "min_range_m": 0.5,
            "max_range_m": 8.0,
            "voxel_leaf_size_m": 0.25,
            "accumulation_frames": 1,
            "process_rate_hz": 10.0,
            "waterline_z_m": 0.0,
            "water_remove_min_z_m": -0.3,
            "water_remove_max_z_m": 0.15,
            "use_water_plane_ransac": False,
            "object_min_z_m": -0.5,
            "object_max_z_m": 2.0,
            "publish_self_marker": False,
            "publish_debug": False,
        }],
        condition=IfCondition(enable_task1_safety_points),
    )

    networking_launch = include_launch(
        "robot",
        ["launch", "networking.launch.py"],
        None,
        {
            "role": "jetson",
            "enable_zenoh_bridge": LaunchConfiguration("enable_zenoh_bridge"),
            "enable_critical_link": "false",
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
                default_value="false",
                description="Load GLIM into the Livox component container",
            ),
            DeclareLaunchArgument(
                "glim_headless",
                default_value="true",
                description="Use GLIM's headless configuration without GUI viewer extensions.",
            ),
            DeclareLaunchArgument(
                "glim_backend",
                default_value="gpu",
                choices=["gpu", "cpu"],
                description="Select the GLIM config directory "
                "(glim_config for gpu, glim_config_cpu for cpu)",
            ),
            DeclareLaunchArgument(
                "enable_pcl_buoy_detection",
                default_value="false",
                description="Standalone LiDAR-only detector for explicit fallback testing. "
                "Normal operation uses ZED detection with its in-node LiDAR depth fallback.",
            ),
            DeclareLaunchArgument("enable_gpu_perception", default_value="true"),
            DeclareLaunchArgument("engine_path", default_value=default_engine),
            DeclareLaunchArgument(
                "camera_resolution",
                default_value="HD720",
                description="ZED camera resolution: HD2K, HD1080, HD720, or VGA",
            ),
            DeclareLaunchArgument("camera_framerate", default_value="15"),
            DeclareLaunchArgument("enable_ground_video", default_value="true"),
            DeclareLaunchArgument(
                "ground_video_host",
                default_value="osw-Stealth-14-AI-Studio-A1VGG.local",
            ),
            DeclareLaunchArgument("ground_video_port", default_value="5600"),
            DeclareLaunchArgument("ground_video_width", default_value="360"),
            DeclareLaunchArgument("ground_video_height", default_value="240"),
            DeclareLaunchArgument("ground_video_fps", default_value="3.0"),
            DeclareLaunchArgument("enable_heartbeats", default_value="true"),
            DeclareLaunchArgument(
                "enable_task1_safety_points",
                default_value="true",
                description=(
                    "Publish compact near-field LiDAR safety points for the "
                    "miniPC Task 1 collision monitor."
                ),
            ),
            DeclareLaunchArgument(
                "enable_zenoh_bridge",
                default_value="true",
                description="Start the Jetson zenoh-bridge-ros2dds process.",
            ),
            DeclareLaunchArgument("heartbeat_monitor_zed2i", default_value="true"),
            DeclareLaunchArgument("heartbeat_monitor_lidar", default_value="true"),
            # Staged startup, carried over from manual_control.launch.py. On a
            # dedicated Jetson there is far less contention than in the old
            # single-machine setup, but starting the ZED/TensorRT stack after
            # the Livox container avoids simultaneous device/model startup;
            # standalone_bringup.launch.py passes the original 18.0/20.0 values
            # to reproduce the pre-split behaviour.
            DeclareLaunchArgument(
                "lidar_start_delay",
                default_value="0.0",
                description="Seconds to wait before starting the MID360 container",
            ),
            DeclareLaunchArgument(
                "camera_start_delay",
                default_value="5.0",
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
            task1_safety_points,
            heartbeat_launch,
            networking_launch,
        ]
    )
