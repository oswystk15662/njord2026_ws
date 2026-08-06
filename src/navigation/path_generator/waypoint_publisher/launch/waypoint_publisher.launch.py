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
from launch_ros.parameter_descriptions import ParameterValue

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

    dynamic_gate_arg = DeclareLaunchArgument(
        'use_dynamic_gate_midpoints',
        default_value='true',
        description='Use live red/green buoy TF midpoints for Task3 gate waypoints'
    )

    full_sequence_arg = DeclareLaunchArgument(
        'run_full_sequence',
        default_value='false',
        description='For task3_1, continue through task3_2 and finish at GPS10'
    )

    waypoint_marker_topic_arg = DeclareLaunchArgument(
        'waypoint_marker_topic',
        default_value='/waypoint_markers',
        description='MarkerArray topic used to visualize the configured waypoint sequence'
    )

    nav2_goal_tolerance_arg = DeclareLaunchArgument(
        'nav2_goal_tolerance_m',
        default_value='1.0',
        description='Nav2 XY reach tolerance rendered around each waypoint in metres'
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
                'use_dynamic_gate_midpoints': ParameterValue(
                    LaunchConfiguration('use_dynamic_gate_midpoints'),
                    value_type=bool,
                ),
                'run_full_sequence': ParameterValue(
                    LaunchConfiguration('run_full_sequence'),
                    value_type=bool,
                ),
                'waypoint_marker_topic': LaunchConfiguration('waypoint_marker_topic'),
                'nav2_goal_tolerance_m': ParameterValue(
                    LaunchConfiguration('nav2_goal_tolerance_m'),
                    value_type=float,
                ),
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
        dynamic_gate_arg,
        full_sequence_arg,
        waypoint_marker_topic_arg,
        nav2_goal_tolerance_arg,
        waypoint_publisher_node,
    ])
    
    return ld
