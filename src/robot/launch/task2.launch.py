"""Real-vessel Task 2 bringup: manual hardware, Nav2, and task waypoints."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _include(package, launch_file, arguments=None):
    """Include a launch file in its own argument scope."""
    return GroupAction(
        scoped=True,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare(package), 'launch', launch_file])
                ),
                launch_arguments=(arguments or {}).items(),
            )
        ],
    )


def generate_launch_description():
    robot_share = get_package_share_directory('robot')
    manual = _include(
        'simple_manual',
        'manual_control.launch.py',
        {
            'serial_port': LaunchConfiguration('serial_port'),
            'baud': LaunchConfiguration('baud'),
            'um982_port': LaunchConfiguration('um982_port'),
            'auto_topic': '/cmd_vel_smoothed',
        },
    )
    nav2 = _include(
        'robot',
        'nav2.launch.py',
        {
            'params_file': os.path.join(robot_share, 'config', 'nav2_params_task2.yaml'),
            'enable_diagnostics': LaunchConfiguration('enable_nav2_diagnostics'),
        },
    )
    waypoints = _include(
        'waypoint_publisher',
        'waypoint_publisher.launch.py',
        {'task_type': 'task2', 'frame_id': 'map', 'publish_rate_hz': '2.0'},
    )
    autonomy_supervisor = Node(
        package='diagnostic_monitors',
        executable='autonomy_supervisor_node',
        name='autonomy_supervisor',
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port',
            default_value=(
                '/dev/serial/by-id/'
                'usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_'
                'c82421728a9aef118808b29061ce3355-if00-port0'
            ),
        ),
        DeclareLaunchArgument('baud', default_value='115200'),
        DeclareLaunchArgument(
            'um982_port',
            default_value='/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0',
        ),
        DeclareLaunchArgument('enable_nav2_diagnostics', default_value='true'),
        DeclareLaunchArgument('nav2_start_delay', default_value='35.0'),
        DeclareLaunchArgument('waypoint_start_delay', default_value='45.0'),
        manual,
        autonomy_supervisor,
        TimerAction(period=LaunchConfiguration('nav2_start_delay'), actions=[nav2]),
        TimerAction(period=LaunchConfiguration('waypoint_start_delay'), actions=[waypoints]),
    ])
