#!/usr/bin/env python3
"""
Launch file for the Micon ESP32 Agent with YAML configuration.
This launch file starts the micro-ROS agent using configuration from a YAML file.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description."""

    package_dir = get_package_share_directory('micon_agent_esp32')
    config_path = os.path.join(package_dir, 'config', 'micon_esp32_config.yaml')

    # Declare launch arguments
    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=config_path,
        description='Path to the configuration YAML file'
    )

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='micon',
        description='Namespace for the agent topics'
    )

    # Create the node with config file
    micon_agent_node = Node(
        package='micon_agent_esp32',
        executable='micon_esp32_agent',
        name='micon_esp32_agent',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[LaunchConfiguration('config_file')]
    )

    return LaunchDescription([
        config_arg,
        namespace_arg,
        micon_agent_node,
    ])
