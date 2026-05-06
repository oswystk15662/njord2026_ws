from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    camera_device_arg = DeclareLaunchArgument(
        "camera_device",
        default_value="/dev/video0",
        description="Camera device path",
    )
    image_width_arg = DeclareLaunchArgument(
        "image_width",
        default_value="640",
        description="Capture width",
    )
    image_height_arg = DeclareLaunchArgument(
        "image_height",
        default_value="480",
        description="Capture height",
    )
    framerate_arg = DeclareLaunchArgument(
        "framerate",
        default_value="30",
        description="Capture framerate",
    )
    camera_frame_id_arg = DeclareLaunchArgument(
        "camera_frame_id",
        default_value="camera_link",
        description="Frame ID for published messages",
    )
    compressed_topic_arg = DeclareLaunchArgument(
        "compressed_image_topic_name",
        default_value="image_raw",
        description="Compressed image topic name",
    )
    camera_info_topic_arg = DeclareLaunchArgument(
        "camera_info_topic_name",
        default_value="camera_info",
        description="Camera info topic name",
    )

    container = ComposableNodeContainer(
        name="usb_camera_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        output="screen",
        composable_node_descriptions=[
            ComposableNode(
                package="usb_camera_driver",
                plugin="USBCameraDriverNode",
                name="usb_camera_driver_node",
                parameters=[
                    {
                        "camera_device": LaunchConfiguration("camera_device"),
                        "image_width": LaunchConfiguration("image_width"),
                        "image_height": LaunchConfiguration("image_height"),
                        "framerate": LaunchConfiguration("framerate"),
                        "camera_frame_id": LaunchConfiguration("camera_frame_id"),
                        "compressed_image_topic_name": LaunchConfiguration(
                            "compressed_image_topic_name"
                        ),
                        "camera_info_topic_name": LaunchConfiguration(
                            "camera_info_topic_name"
                        ),
                    }
                ],
            )
        ],
    )

    return LaunchDescription(
        [
            camera_device_arg,
            image_width_arg,
            image_height_arg,
            framerate_arg,
            camera_frame_id_arg,
            compressed_topic_arg,
            camera_info_topic_arg,
            container,
        ]
    )
