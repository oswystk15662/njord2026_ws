from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    mode = LaunchConfiguration("mode").perform(context)
    if mode not in ("cpu", "sdk"):
        raise RuntimeError("mode must be either 'cpu' or 'sdk'")

    executable = "zed2i_cpu_node" if mode == "cpu" else "zed2i_sdk_node"
    return [
        Node(
            package="zed2i_driver",
            executable=executable,
            name="zed2i",
            namespace=LaunchConfiguration("namespace"),
            output="screen",
            parameters=[
                {
                    "left_device": LaunchConfiguration("left_device"),
                    "right_device": LaunchConfiguration("right_device"),
                    "left_frame_id": LaunchConfiguration("left_frame_id"),
                    "right_frame_id": LaunchConfiguration("right_frame_id"),
                    "depth_frame_id": LaunchConfiguration("depth_frame_id"),
                    "image_width": LaunchConfiguration("image_width"),
                    "image_height": LaunchConfiguration("image_height"),
                    "framerate": LaunchConfiguration("framerate"),
                    "baseline_m": LaunchConfiguration("baseline_m"),
                    "fx": LaunchConfiguration("fx"),
                    "fy": LaunchConfiguration("fy"),
                    "cx": LaunchConfiguration("cx"),
                    "cy": LaunchConfiguration("cy"),
                    "depth_min_m": LaunchConfiguration("depth_min_m"),
                    "depth_max_m": LaunchConfiguration("depth_max_m"),
                    "publish_pointcloud": LaunchConfiguration("publish_pointcloud"),
                    "pointcloud_stride": LaunchConfiguration("pointcloud_stride"),
                    "num_disparities": LaunchConfiguration("num_disparities"),
                    "block_size": LaunchConfiguration("block_size"),
                }
            ],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("mode", default_value="cpu"),
            DeclareLaunchArgument("namespace", default_value="/zed2i"),
            DeclareLaunchArgument("left_device", default_value="/dev/video0"),
            DeclareLaunchArgument("right_device", default_value="/dev/video1"),
            DeclareLaunchArgument("left_frame_id", default_value="zed2i_left_camera_frame"),
            DeclareLaunchArgument("right_frame_id", default_value="zed2i_right_camera_frame"),
            DeclareLaunchArgument("depth_frame_id", default_value="zed2i_left_camera_frame"),
            DeclareLaunchArgument("image_width", default_value="1280"),
            DeclareLaunchArgument("image_height", default_value="720"),
            DeclareLaunchArgument("framerate", default_value="15"),
            DeclareLaunchArgument("baseline_m", default_value="0.12"),
            DeclareLaunchArgument("fx", default_value="1280.0"),
            DeclareLaunchArgument("fy", default_value="1280.0"),
            DeclareLaunchArgument("cx", default_value="640.0"),
            DeclareLaunchArgument("cy", default_value="360.0"),
            DeclareLaunchArgument("depth_min_m", default_value="0.3"),
            DeclareLaunchArgument("depth_max_m", default_value="20.0"),
            DeclareLaunchArgument("publish_pointcloud", default_value="true"),
            DeclareLaunchArgument("pointcloud_stride", default_value="2"),
            DeclareLaunchArgument("num_disparities", default_value="128"),
            DeclareLaunchArgument("block_size", default_value="5"),
            OpaqueFunction(function=launch_setup),
        ]
    )
