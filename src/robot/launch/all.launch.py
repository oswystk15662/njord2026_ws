from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    
    localization_launch_file = os.path.join(
        FindPackageShare('robot').find('robot'),
        'launch',
        'localization.launch.py'
    )

    # Includeしてパラメータを上書き
    localization_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(localization_launch_file),
    )


    navigation_launch_file = os.path.join(
        FindPackageShare('robot').find('robot'),
        'launch',
        'navigation.launch.py'
    )
    navigation_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            navigation_launch_file
        ),
    )

    return LaunchDescription([
        localization_nodes,
        navigation_nodes,
    ])