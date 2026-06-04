import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    default_params_file = os.path.join(
        get_package_share_directory('robot'),
        'config',
        'nav2_params.yaml'
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the Nav2 parameters file'
    )

    return LaunchDescription([
        params_file_arg,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'params_file': LaunchConfiguration('params_file'),
                'use_sim_time': 'false',
                'autostart': 'true'
            }.items()
        )
    ])