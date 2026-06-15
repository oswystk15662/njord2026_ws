from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('buoy_obstacle_publisher'),
        'config',
        'buoy_obstacle_publisher.yaml'
    )
    node = Node(
        package='buoy_obstacle_publisher',
        executable='buoy_obstacle_publisher',
        name='buoy_obstacle_publisher',
        parameters=[config],
        output='screen',
    )
    return LaunchDescription([node])
