import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'njord_perception'
    pkg_share = get_package_share_directory(pkg_name)

    # デフォルトのモデルパス
    default_model_path = os.path.join(pkg_share, 'config', 'best.pt')

    # Launch引数の宣言
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value=default_model_path,
        description='Absolute path to the YOLO model file (.pt)'
    )
    
    device_arg = DeclareLaunchArgument(
        'device',
        default_value='cpu',
        description='Computation device (cpu or cuda:0)'
    )

    # ノードの定義
    yolo_node = Node(
        package=pkg_name,
        executable='yolo_node',
        name='yolo_detector',
        output='screen',
        parameters=[{
            'model_path': LaunchConfiguration('model_path'),
            'device': LaunchConfiguration('device'),
            'camera_topic': '/camera/image_raw'
        }]
    )

    return LaunchDescription([
        model_path_arg,
        device_arg,
        yolo_node
    ])