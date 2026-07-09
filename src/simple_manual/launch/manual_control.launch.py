from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

import os


def generate_launch_description():
    serial_port = LaunchConfiguration('serial_port')
    baud = LaunchConfiguration('baud')
    launch_gui = LaunchConfiguration('launch_gui')
    thruster_launch = os.path.join(
        get_package_share_directory('thruster_driver'),
        'launch',
        'thruster_driver.launch.py',
    )

    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('baud', default_value='115200'),
        DeclareLaunchArgument('launch_gui', default_value='true'),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            additional_env={'SDL_JOYSTICK_DEVICE': '/dev/input/js0'},
        ),
        Node(
            package='simple_manual',
            executable='joy_converter_node',
            name='joy_converter',
            output='screen',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(thruster_launch),
        ),
        Node(
            package='micon_driver_fd',
            executable='serial_writer',
            name='serial_writer',
            parameters=[{'serial_port': serial_port, 'baud': baud}],
            output='screen',
        ),
        #Node(
        #    package='rqt_reconfigure',
        #    executable='rqt_reconfigure',
        #    name='manual_control_parameters',
        #    condition=IfCondition(launch_gui),
        #    output='screen',
        #),
    ])
