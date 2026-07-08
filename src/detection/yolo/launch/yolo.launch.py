import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_name = 'yolo'
    pkg_share = get_package_share_directory(pkg_name)

    default_config_path = os.path.join(pkg_share, 'config', 'yolo_params.yaml')

    # 起動時に読み込む YAML。
    # device を "cuda:0" にすれば GPU、"cpu" にすれば CPU で動く。
    # model_path、camera_topic、ROI、debug_image の publish 周期も YAML 側で調整する。
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_path,
        description='Path to YOLO parameter YAML'
    )

    # yolo_node だけを使う。
    # GPU/CPU の切り替えは executable を分けず、YAML の device で行う。
    yolo_node = Node(
        package=pkg_name,
        executable='yolo_node',
        name='yolo_detector',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
        ]
    )

    return LaunchDescription([
        config_file_arg,
        yolo_node,
    ])
