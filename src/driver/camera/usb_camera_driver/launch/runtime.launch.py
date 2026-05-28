#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Runtime camera launch file for Task1.
Streams camera feed without calibration overlay.
Publishes to /camera/image_raw for YOLO detection.
"""

import launch
import launch_ros.actions
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    TextSubstitution,
)
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Generate runtime camera launch description."""
    
    mjpg_streamer_dir = "/home/osw/Download/mjpg-streamer/mjpg-streamer-experimental"

    # LD_LIBRARY_PATHを設定
    env = {
        "LD_LIBRARY_PATH": f"{mjpg_streamer_dir}:{EnvironmentVariable('LD_LIBRARY_PATH')}",
    }

    # HTTP streamer for left camera only
    output_plugin_args_left = TextSubstitution(text="output_http.so -w ./www -p 8080")
    input_plugin_args_left = TextSubstitution(
        text="input_uvc.so -d /dev/camera_c270_stream_left -r 640x480 -fps 30"
    )

    mjpg_streamer_left = ExecuteProcess(
        cmd=[
            f"{mjpg_streamer_dir}/mjpg_streamer",
            "-o",
            output_plugin_args_left,
            "-i",
            input_plugin_args_left,
        ],
        cwd=mjpg_streamer_dir,
        env=env,
        output="screen",
    )

    # Camera driver arguments
    image_width_arg = DeclareLaunchArgument(
        "image_width", default_value="640", description="Width of the captured image"
    )
    image_height_arg = DeclareLaunchArgument(
        "image_height", default_value="480", description="Height of the captured image"
    )
    framerate_arg = DeclareLaunchArgument(
        "framerate",
        default_value="30",
        description="Framerate of the captured image (fps)",
    )

    camera_device_arg = DeclareLaunchArgument(
        "camera_device",
        default_value="http://localhost:8080/?action=stream",
        description="Path or URL to the camera device",
    )
    
    camera_frame_id_arg = DeclareLaunchArgument(
        "camera_frame_id",
        default_value="camera_link",
        description="Frame ID for the camera (for TF)",
    )

    # Composable container with rpi_camera_driver (if available)
    composable_container = ComposableNodeContainer(
        name="camera_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="rpi_camera_driver",
                plugin="RPiCameraDriverNode",
                name="camera_driver",
                parameters=[
                    {"camera_device": LaunchConfiguration("camera_device")},
                    {"image_width": LaunchConfiguration("image_width")},
                    {"image_height": LaunchConfiguration("image_height")},
                    {"framerate": LaunchConfiguration("framerate")},
                    {"camera_frame_id": LaunchConfiguration("camera_frame_id")},
                    {"image_topic_name": "camera/image_raw"},
                    {"camera_info_topic_name": "camera/camera_info"},
                ],
            ),
        ],
        output="screen",
    )

    return launch.LaunchDescription([
        image_width_arg,
        image_height_arg,
        framerate_arg,
        camera_device_arg,
        camera_frame_id_arg,
        mjpg_streamer_left,
        composable_container,
    ])
