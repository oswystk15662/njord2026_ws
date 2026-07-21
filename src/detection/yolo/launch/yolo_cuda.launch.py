import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_name = 'yolo'
    pkg_share = get_package_share_directory(pkg_name)

    default_model_path = os.path.join(pkg_share, 'config', 'best.pt')

    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value=default_model_path,
        description='Absolute path to the YOLO model file (.pt)'
    )

    device_arg = DeclareLaunchArgument(
        'device',
        default_value='cuda:0',
        description='Computation device (default: cuda:0)'
    )

    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic',
        default_value='/camera/image_raw',
        description='Image topic consumed by the YOLO detector',
    )

    yolo_node = Node(
        package=pkg_name,
        executable='yolo_cuda_node',
        name='yolo_detector_cuda',
        output='screen',
        parameters=[{
            'model_path': LaunchConfiguration('model_path'),
            'device': LaunchConfiguration('device'),
            'camera_topic': LaunchConfiguration('camera_topic')
        }]
    )

    return LaunchDescription([
        model_path_arg,
        device_arg,
        camera_topic_arg,
        yolo_node,
    ])
