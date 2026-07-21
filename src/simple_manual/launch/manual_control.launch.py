"""Bring up the real vessel for joystick control without navigation."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def include_launch(package_name, path_parts, condition=None, launch_arguments=None):
    """Include a launch file without leaking its generic launch arguments."""
    return GroupAction(
        scoped=True,
        condition=condition,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare(package_name)] + path_parts)
                ),
                launch_arguments=(launch_arguments or {}).items(),
            )
        ],
    )


def generate_launch_description():
    enable_detection = LaunchConfiguration('enable_detection')

    serial_port = LaunchConfiguration('serial_port')
    baud = LaunchConfiguration('baud')
    um982_transport = LaunchConfiguration('um982_transport')
    um982_port = LaunchConfiguration('um982_port')
    enable_um982_rtk = LaunchConfiguration('enable_um982_rtk')
    drogger_rzs_port = LaunchConfiguration('drogger_rzs_port')
    drogger_rzs_baud = LaunchConfiguration('drogger_rzs_baud')
    drogger_rzs_fix_topic = LaunchConfiguration('drogger_rzs_fix_topic')

    # real_bringup owns the real sensor drivers and localization. Cameras and
    # WIT IMU are deliberately disabled: the manual vessel configuration is
    # MID360S + UM982 + Spatial + Drogger. Thrusters are launched below so the
    # joystick command path has one, unambiguous owner. Navigation stays off.
    sensors_and_localization = include_launch(
        'robot',
        ['launch', 'real_bringup.launch.py'],
        launch_arguments={
            'enable_mid360': 'true',
            'lidar_model': 'mid360s',
            'enable_zed2i': 'false',
            'camera_resolution': 'VGA',
            'enable_back_cam': 'false',
            'enable_um982': 'true',
            'enable_drogger_rzs': 'true',
            'enable_imu': 'false',
            'enable_localization': 'true',
            'enable_thruster': 'false',
            'enable_nav2': 'false',
            'um982_transport': um982_transport,
            'um982_port': um982_port,
            'enable_um982_rtk': enable_um982_rtk,
            'drogger_rzs_port': drogger_rzs_port,
            'drogger_rzs_baud': drogger_rzs_baud,
            'drogger_rzs_fix_topic': drogger_rzs_fix_topic,
        },
    )

    # Complete LiDAR-only detection chain:
    # preprocessing -> segmentation/clustering -> object tracking.
    detection = include_launch(
        'ship_perception_bringup',
        ['launch', 'classical_pipeline.launch.py'],
        condition=IfCondition(enable_detection),
        launch_arguments={
            'lidar_topic': '/livox/lidar',
            'ego_odom_topic': '/odometry/filtered/local',
        },
    )

    thruster_driver = include_launch(
        'thruster_driver',
        ['launch', 'thruster_driver.launch.py'],
        launch_arguments={'stop_on_feedback_timeout': 'false'},
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'serial_port',
                default_value=(
                    '/dev/serial/by-id/'
                    'usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_'
                    'c82421728a9aef118808b29061ce3355-if00-port0'
                ),
                description='Serial device connected to micon_driver_fd',
            ),
            DeclareLaunchArgument('baud', default_value='115200'),
            DeclareLaunchArgument('enable_detection', default_value='true'),
            DeclareLaunchArgument('um982_transport', default_value='uart'),
            DeclareLaunchArgument(
                'um982_port',
                default_value='/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0',
            ),
            DeclareLaunchArgument('enable_um982_rtk', default_value='false'),
            DeclareLaunchArgument(
                'drogger_rzs_port',
                default_value=(
                    '/dev/serial/by-id/'
                    'usb-Prolific_Technology_Inc._USB-Serial_Controller_'
                    'ACCQg146B12-if00-port0'
                ),
            ),
            DeclareLaunchArgument('drogger_rzs_baud', default_value='115200'),
            DeclareLaunchArgument('drogger_rzs_fix_topic', default_value='/gnss/fix'),
            sensors_and_localization,
            detection,
            Node(
                package='simple_manual',
                executable='joy_converter_node',
                name='joy_converter',
                output='screen',
                parameters=[
                    {
                        'button.yaw_positive': 4,
                        'button.yaw_negative': 5,
                        'scale.linear_x': 1.0,
                        'scale.linear_y': 1.0,
                        'scale.angular_z': 1.0,
                    }
                ],
            ),
            thruster_driver,
            Node(
                package='micon_driver_fd',
                executable='serial_writer',
                name='serial_writer',
                parameters=[
                    {
                        'serial_port': serial_port,
                        'baud': baud,
                        'command_topic': '/thruster_command',
                        'use_sim_time': False,
                    }
                ],
                output='screen',
            ),
        ]
    )
