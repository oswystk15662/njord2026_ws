from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('drogger_wired_flex')
    default_params_file = os.path.join(pkg_share, 'config', 'params_rzs_d01_usb.yaml')
    params_file = LaunchConfiguration('config_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=default_params_file,
            description='Path to YAML parameter file',
        ),
        Node(
            package='drogger_wired_flex',
            executable='drogger_wired_flex_node',
            name='drogger_wired_flex',
            output='screen',
            parameters=[params_file],
        )
    ])
