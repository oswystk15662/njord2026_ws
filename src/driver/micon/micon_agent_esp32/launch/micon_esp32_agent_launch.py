#!/usr/bin/env python3
"""
Launch file for the Micon ESP32 Agent.
This launch file starts the micro-ROS agent for ESP32 communication.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate launch description."""

    # Declare launch arguments
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyUSB0',
        description='Serial port for ESP32 connection'
    )

    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='115200',
        description='Serial port baudrate'
    )

    enable_imu_arg = DeclareLaunchArgument(
        'enable_imu',
        default_value='false',
        description='Enable IMU publishing'
    )

    enable_sensors_arg = DeclareLaunchArgument(
        'enable_sensors',
        default_value='true',
        description='Enable general sensor publishing'
    )

    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='10',
        description='Publishing rate in Hz'
    )

    # Create the node
    micon_agent_node = Node(
        package='micon_agent_esp32',
        executable='micon_esp32_agent',
        name='micon_esp32_agent',
        output='screen',
        parameters=[
            {
                'port': LaunchConfiguration('port'),
                'baudrate': LaunchConfiguration('baudrate'),
                'enable_imu': LaunchConfiguration('enable_imu'),
                'enable_sensors': LaunchConfiguration('enable_sensors'),
                'publish_rate': LaunchConfiguration('publish_rate'),
            }
        ]
    )

    return LaunchDescription([
        port_arg,
        baudrate_arg,
        enable_imu_arg,
        enable_sensors_arg,
        publish_rate_arg,
        micon_agent_node,
    ])
