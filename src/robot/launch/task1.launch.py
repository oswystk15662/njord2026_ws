"""Real-vessel Task 1 bringup: manual hardware, Nav2, and task waypoints."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
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
    robot_description = Command([
        FindExecutable(name='xacro'),
        ' ',
        PathJoinSubstitution([FindPackageShare('robot'), 'urdf', 'robot.urdf.xacro']),
    ])

    manual = _include(
        'simple_manual',
        'manual_control.launch.py',
        {
            'serial_port': LaunchConfiguration('serial_port'),
            'baud': LaunchConfiguration('baud'),
            'um982_port': LaunchConfiguration('um982_port'),
            'lidar_start_delay': LaunchConfiguration('lidar_start_delay'),
            'perception_start_delay': LaunchConfiguration('perception_start_delay'),
            'localization_start_delay': LaunchConfiguration('localization_start_delay'),
        },
    )
    nav2 = _include(
        'robot',
        'nav2.launch.py',
        {
            'enable_diagnostics': LaunchConfiguration('enable_nav2_diagnostics'),
            # Task 1 currently navigates from UM982 odometry without LiDAR-based
            # collision monitoring. Keep the Nav2 cmd_vel relay alive, but disable
            # its LiDAR source and collision actions.
            'enable_collision_monitor': 'false',
        },
    )
    waypoints = _include(
        'waypoint_publisher',
        'waypoint_publisher.launch.py',
        {'task_type': 'task1', 'frame_id': 'odom', 'publish_rate_hz': '2.0'},
    )
    um982_driver = Node(
        package='um982_driver',
        executable='um982_driver_node',
        name='um982_driver',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'uart_or_tcp': 'uart',
            'GNSS_SerialPort': LaunchConfiguration('um982_port'),
            'GNSS_Baudrate': 115200,
            'FIX_FREQ': 20,
            'HEADING_FREQ': 20,
            'GNSS_RTK_Enable': False,
            'Heading_FrameID': 'odom',
            'log_file_name': '',
            'publish_feedback_odometry': True,
        }],
    )
    um982_feedback = _include(
        'um982_feedback_filter',
        'um982_feedback.launch.py',
        {
            'feedback_mode': 'ekf',
            'raw_topic': '/odometry/feedback',
            'output_topic': '/odometry/filtered/local',
        },
    )
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(robot_description, value_type=str),
        }],
    )
    um982_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='um982_static_tf_pub',
        output='screen',
        arguments=[
            '--x', '0.0', '--y', '0.0', '--z', '0.0',
            '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
            '--frame-id', 'base_link', '--child-frame-id', 'um982_link',
        ],
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
        DeclareLaunchArgument(
            'lidar_start_delay',
            default_value='18.0',
            description='Seconds to wait before starting the Livox LiDAR.',
        ),
        DeclareLaunchArgument('localization_start_delay', default_value='25.0'),
        DeclareLaunchArgument('perception_start_delay', default_value='30.0'),
        DeclareLaunchArgument('nav2_start_delay', default_value='35.0'),
        DeclareLaunchArgument('waypoint_start_delay', default_value='45.0'),
        manual,
        um982_driver,
        um982_feedback,
        robot_state_publisher,
        um982_static_tf,
        TimerAction(period=LaunchConfiguration('nav2_start_delay'), actions=[nav2]),
        TimerAction(period=LaunchConfiguration('waypoint_start_delay'), actions=[waypoints]),
    ])
