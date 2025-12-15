import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    
    # 自分のconfigファイルのパス
    # ※パッケージ名は適宜修正してください (例: njord_bringup)
    params_file = os.path.join(
        get_package_share_directory('robot'), 
        'config', 
        'nav2_mintest.yaml'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'params_file': params_file,
                'use_sim_time': 'false',  # シミュレータなのでTrue, foxgloveはFalse
                'autostart': 'true'      # 自動でActiveにする
            }.items()
        )
    ])