import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Legacy CUDA launcher kept as a thin compatibility wrapper.

    New deployments should use yolo.launch.py with yolo_params.yaml. This
    entry point preserves the existing command used by older bringup scripts.
    """
    pkg_share = get_package_share_directory('yolo')
    default_model_path = os.path.join(pkg_share, 'config', 'best.pt')

    model_path_arg = DeclareLaunchArgument(
        'model_path', default_value=default_model_path)
    device_arg = DeclareLaunchArgument('device', default_value='cuda:0')
    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic', default_value='/camera/image_raw')
    node_name_arg = DeclareLaunchArgument(
        'node_name', default_value='yolo_detector_cuda')
    namespace_arg = DeclareLaunchArgument('namespace', default_value='')
    enable_roi_arg = DeclareLaunchArgument('enable_roi', default_value='true')
    enable_virtual_wall_arg = DeclareLaunchArgument(
        'enable_virtual_wall', default_value='false')

    yolo_node = Node(
        package='yolo',
        executable='yolo_cuda_node',
        name=LaunchConfiguration('node_name'),
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[{
            'model_path': LaunchConfiguration('model_path'),
            'device': LaunchConfiguration('device'),
            'camera_topic': LaunchConfiguration('camera_topic'),
            'enable_roi': LaunchConfiguration('enable_roi'),
            'enable_virtual_wall': LaunchConfiguration('enable_virtual_wall'),
        }],
    )

    return LaunchDescription([
        model_path_arg,
        device_arg,
        camera_topic_arg,
        node_name_arg,
        namespace_arg,
        enable_roi_arg,
        enable_virtual_wall_arg,
        yolo_node,
    ])
