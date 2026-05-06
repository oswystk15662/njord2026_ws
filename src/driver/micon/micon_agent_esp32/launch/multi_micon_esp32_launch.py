#!/usr/bin/env python3
"""
Launch file for multiple Micon ESP32 Agents.
This allows launching multiple agents for different ESP32 devices.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate launch description for multiple agents."""

    # Agent 1
    agent1_port_arg = DeclareLaunchArgument(
        'agent1_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for ESP32 Agent 1'
    )

    agent1_node = Node(
        package='micon_agent_esp32',
        executable='micon_esp32_agent',
        name='micon_esp32_agent_1',
        namespace='micon_1',
        output='screen',
        parameters=[
            {
                'port': LaunchConfiguration('agent1_port'),
                'baudrate': 115200,
                'enable_imu': False,
                'enable_sensors': True,
                'publish_rate': 10,
            }
        ]
    )

    # Agent 2
    agent2_port_arg = DeclareLaunchArgument(
        'agent2_port',
        default_value='/dev/ttyUSB1',
        description='Serial port for ESP32 Agent 2'
    )

    agent2_node = Node(
        package='micon_agent_esp32',
        executable='micon_esp32_agent',
        name='micon_esp32_agent_2',
        namespace='micon_2',
        output='screen',
        parameters=[
            {
                'port': LaunchConfiguration('agent2_port'),
                'baudrate': 115200,
                'enable_imu': False,
                'enable_sensors': True,
                'publish_rate': 10,
            }
        ]
    )

    return LaunchDescription([
        agent1_port_arg,
        agent2_port_arg,
        agent1_node,
        agent2_node,
    ])
