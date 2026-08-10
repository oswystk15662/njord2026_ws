"""Replay ZED image/depth rosbag data through the buoy position checker."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('model_path', default_value=''),
        DeclareLaunchArgument('device', default_value='cpu'),
        DeclareLaunchArgument('image_topic', default_value='/zed/zed_node/left/image_rect_color'),
        DeclareLaunchArgument('depth_topic', default_value='/zed/zed_node/depth/depth_registered'),
        DeclareLaunchArgument('camera_info_topic', default_value='/zed/zed_node/left/camera_info'),
        Node(package='yolo', executable='bag_depth_position_node', name='bag_depth_position_checker', output='screen',
             parameters=[{'model_path': LaunchConfiguration('model_path'), 'device': LaunchConfiguration('device'),
                          'image_topic': LaunchConfiguration('image_topic'), 'depth_topic': LaunchConfiguration('depth_topic'),
                          'camera_info_topic': LaunchConfiguration('camera_info_topic')}]),
    ])
