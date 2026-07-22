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
    enable_yolo = LaunchConfiguration('enable_yolo')
    yolo_device = LaunchConfiguration('yolo_device')
    enable_back_yolo = LaunchConfiguration('enable_back_yolo')
    back_yolo_device = LaunchConfiguration('back_yolo_device')
    enable_ground_video = LaunchConfiguration('enable_ground_video')
    ground_video_host = LaunchConfiguration('ground_video_host')
    ground_video_port = LaunchConfiguration('ground_video_port')
    ground_video_fps = LaunchConfiguration('ground_video_fps')
    enable_back_ground_video = LaunchConfiguration('enable_back_ground_video')
    back_ground_video_host = LaunchConfiguration('back_ground_video_host')
    back_ground_video_port = LaunchConfiguration('back_ground_video_port')
    back_ground_video_fps = LaunchConfiguration('back_ground_video_fps')

    serial_port = LaunchConfiguration('serial_port')
    baud = LaunchConfiguration('baud')
    um982_protocol = LaunchConfiguration('um982_protocol')
    um982_port = LaunchConfiguration('um982_port')
    enable_um982_rtk = LaunchConfiguration('enable_um982_rtk')
    drogger_rzs_port = LaunchConfiguration('drogger_rzs_port')
    drogger_rzs_baud = LaunchConfiguration('drogger_rzs_baud')
    drogger_rzs_fix_topic = LaunchConfiguration('drogger_rzs_fix_topic')

    # real_bringup owns the real sensor drivers and localization. ZED2i is
    # enabled as the input camera for YOLO; WIT IMU remains disabled. Thrusters
    # are launched below so the joystick command path has one, unambiguous
    # owner. Navigation stays off.
    sensors_and_localization = include_launch(
        'robot',
        ['launch', 'real_bringup.launch.py'],
        launch_arguments={
            'enable_mid360': 'true',
            'lidar_model': 'mid360s',
            'enable_zed2i': 'true',
            'camera_resolution': 'VGA',
            'camera_framerate': '30',
            'enable_ground_video': enable_ground_video,
            'ground_video_host': ground_video_host,
            'ground_video_port': ground_video_port,
            'ground_video_fps': ground_video_fps,
            'enable_back_cam': 'true',
            'enable_um982': 'true',
            'enable_drogger_rzs': 'true',
            'enable_imu': 'false',
            'enable_localization': 'true',
            'um982_protocol': um982_protocol,
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

    yolo = include_launch(
        'yolo',
        ['launch', 'yolo_cuda.launch.py'],
        condition=IfCondition(enable_yolo),
        launch_arguments={
            'model_path': PathJoinSubstitution(
                [FindPackageShare('robot'), 'config', 'yolo_model', 'best.engine']
            ),
            'device': yolo_device,
            'camera_topic': '/zed2i/left/image_rect',
        },
    )

    # Rear-camera detections are deliberately namespaced and do not publish
    # forward-camera ROI/virtual-obstacle data used by navigation.
    back_yolo = include_launch(
        'yolo',
        ['launch', 'yolo_cuda.launch.py'],
        condition=IfCondition(enable_back_yolo),
        launch_arguments={
            'model_path': PathJoinSubstitution(
                [FindPackageShare('robot'), 'config', 'yolo_model', 'best.engine']
            ),
            'device': back_yolo_device,
            'camera_topic': '/back_cam/image_raw',
            'node_name': 'back_yolo_detector_cuda',
            'namespace': 'yolo_back',
            'enable_roi': 'false',
            'enable_virtual_wall': 'false',
            'enable_debug_image': 'true',
        },
    )

    # Keep a second RTP/JPEG stream separate from the ZED stream (port 5600),
    # so both can be displayed concurrently on the ground PC.
    back_ground_video = Node(
        package='zed2i_driver',
        executable='back_cam_ground_video_streamer',
        name='back_cam_ground_video_streamer',
        output='screen',
        parameters=[{
            'image_topic': '/yolo_back/yolo/debug_image',
            'host': back_ground_video_host,
            'port': back_ground_video_port,
            'fps': back_ground_video_fps,
        }],
        condition=IfCondition(enable_back_ground_video),
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
            DeclareLaunchArgument('enable_yolo', default_value='true'),
            DeclareLaunchArgument('yolo_device', default_value='cuda:0'),
            DeclareLaunchArgument('enable_back_yolo', default_value='true'),
            DeclareLaunchArgument('back_yolo_device', default_value='cuda:0'),
            DeclareLaunchArgument('enable_ground_video', default_value='true'),
            DeclareLaunchArgument('ground_video_host', default_value='osw-Stealth-14-AI-Studio-A1VGG'),
            DeclareLaunchArgument('ground_video_port', default_value='5600'),
            DeclareLaunchArgument('ground_video_fps', default_value='5.0'),
            DeclareLaunchArgument('enable_back_ground_video', default_value='true'),
            DeclareLaunchArgument(
                'back_ground_video_host', default_value='osw-Stealth-14-AI-Studio-A1VGG'
            ),
            DeclareLaunchArgument('back_ground_video_port', default_value='5601'),
            DeclareLaunchArgument('back_ground_video_fps', default_value='5.0'),
            DeclareLaunchArgument('um982_protocol', default_value='uart'),
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
            # detection,
            yolo,
            back_yolo,
            back_ground_video,
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
