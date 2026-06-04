import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('path_marker_visualizer'),
        'config',
        'full_path_config.yaml'
    )

    full_path_publisher_node = Node(
        package='path_marker_visualizer',
        executable='full_path_publisher',
        name='full_path_publisher',
        output='screen',
        parameters=[config_file]
    )

    return LaunchDescription([
        full_path_publisher_node
    ])
