import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_name = 'drogger_bt'
    share_dir = get_package_share_directory(pkg_name)
    config_path = os.path.join(share_dir, 'config', 'config.yaml')

    return LaunchDescription([
        Node(
            package=pkg_name,
            executable='drogger_driver_node',
            name='drogger_driver',
            output='screen',
            parameters=[config_path]
        )
    ])