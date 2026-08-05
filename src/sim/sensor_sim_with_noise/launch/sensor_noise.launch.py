import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('sensor_sim_with_noise')
    config_file = os.path.join(pkg_share, 'config', 'config.yaml')
    fix_topic_arg = DeclareLaunchArgument(
        'fix_topic',
        default_value='/gps/fix',
        description='Output topic for the simulated NavSatFix message',
    )

    gnss_sim_node = Node(
        package='sensor_sim_with_noise',
        executable='gnss_noise_simulator',
        name='gnss_noise_simulator',
        parameters=[config_file],
        remappings=[('/gps/fix', LaunchConfiguration('fix_topic'))],
        output='screen'
    )

    imu_sim_node = Node(
        package='sensor_sim_with_noise',
        executable='imu_noise_simulator',
        name='imu_noise_simulator',
        parameters=[config_file],
        output='screen'
    )

    um982_odom_sim_node = Node(
        package='sensor_sim_with_noise',
        executable='um982_odometry_simulator',
        name='um982_odometry_simulator',
        parameters=[config_file],
        output='screen'
    )

    return LaunchDescription([
        fix_topic_arg,
        gnss_sim_node,
        imu_sim_node,
        um982_odom_sim_node
    ])
