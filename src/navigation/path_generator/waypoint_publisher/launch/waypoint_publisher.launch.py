#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Launch file for waypoint_publisher node
Supports task1, task2, task3_1, and task3_2 modes
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """Generate launch description for waypoint_publisher"""
    
    # Declare arguments
    task_type_arg = DeclareLaunchArgument(
        'task_type',
        default_value='task1',
        description='Task type: task1, task1_follow, task2, task3_1, or task3_2'
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='map',
        description='Frame ID for waypoints'
    )
    
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate_hz',
        default_value='2.0',
        description='Publish rate in Hz'
    )
    
    # Create node
    waypoint_publisher_node = Node(
        package='waypoint_publisher',
        executable='waypoint_publisher_node',
        name='waypoint_publisher',
        parameters=[
            {
                'task_type': LaunchConfiguration('task_type'),
                'frame_id': LaunchConfiguration('frame_id'),
                'publish_rate_hz': LaunchConfiguration('publish_rate_hz'),
            }
        ],
        output='screen',
        emulate_tty=True,
    )
    
    # Create launch description
    ld = LaunchDescription([
        task_type_arg,
        frame_id_arg,
        publish_rate_arg,
        waypoint_publisher_node,
    ])
    
    return ld
