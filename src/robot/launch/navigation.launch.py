import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('your_package_name')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # さっき作ったパラメータファイルのパス
    params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    
    return LaunchDescription([
        # Nav2 Bringup (map_serverなしで起動)
        # GPS走行の場合、map_serverは使わないか、ダミーを使う構成になります。
        # ここでは単純化のため navigation_launch.py を呼び出します。
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'params_file': params_file,
                'use_sim_time': 'false',
                'autostart': 'true',
            }.items()
        ),
    ])