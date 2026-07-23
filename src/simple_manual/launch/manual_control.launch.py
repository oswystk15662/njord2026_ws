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

    joy_converter = Node(
        package='simple_manual',
        executable='joy_converter_node',
        name='joy_converter',
        output='screen',
        parameters=[
            PathJoinSubstitution(
                [FindPackageShare('simple_manual'), 'config', 'joy_converter.yaml']
            )
        ],
    )

    micon_driver_fd_node = Node(
        package='micon_driver_fd',
        executable='serial_writer',
        name='serial_writer',
        parameters=[
            {
                'serial_port': '/dev/serial/by-id/'
                    'usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_'
                    'c82421728a9aef118808b29061ce3355-if00-port0',
                'baud': '115200',
                'command_topic': '/thruster_command',
                'use_sim_time': False,
            }
        ],
        output='screen',
    )

    thruster_driver = include_launch(
        'thruster_driver',
        ['launch', 'thruster_driver.launch.py'],
        launch_arguments={
            'use_velocity_feedback': 'false',
            'stop_on_feedback_timeout': 'false',
        },
    )

    # Do not include real_bringup here: it also starts the thruster serial
    # driver, which gives manual_control a second owner of the Micon UART.
    # Keep sensor and localization ownership explicit instead.
    sensors_launch = include_launch(
        'robot',
        ['launch', 'sensors.launch.py'],
        launch_arguments={
            'enable_gpu_perception': 'true',
            'engine_path': os.path.join(
                get_package_share_directory("robot"), "config", "yolo_model", "best.engine"
            ),
        },
    )

    localization_launch = include_launch(
        'robot',
        ['launch', 'localization.launch.py'],
    )

    # Complete LiDAR-only detection chain:
    # preprocessing -> segmentation/clustering -> object tracking.
    ship_detection = include_launch(
        'ship_perception_bringup',
        ['launch', 'classical_pipeline.launch.py'],
        condition=IfCondition(enable_detection),
        launch_arguments={
            'lidar_topic': '/livox/lidar',
            'ego_odom_topic': '/odometry/filtered/local',
        },
    )

    # if you really want zero copy, don't launch this node.
    # instead turn on enable_gpu_perception in the zed2i_launch.
    yolo = include_launch(
        'yolo',
        ['launch', 'yolo_cuda.launch.py'],
        condition=IfCondition(enable_yolo),
        launch_arguments={
            'model_path': PathJoinSubstitution(
                [FindPackageShare('robot'), 'config', 'yolo_model', 'best.engine']
            ),
            'device': "cuda:0",
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
            'device': "cuda:0",
            'camera_topic': '/back_cam/image_raw',
            'node_name': 'back_yolo_detector_cuda',
            'namespace': 'yolo_back',
            'enable_roi': 'false',
            'enable_virtual_wall': 'false',
            'enable_debug_image': 'false',
        },
    )

    # Keep a second RTP/JPEG stream separate from the ZED stream (port 5600),
    # so both can be displayed concurrently on the ground PC.
    ground_video = Node(
        package='zed2i_driver',
        executable='ground_video_streamer',
        name='ground_video_streamer',
        output='screen',
        parameters=[{
            'image_topic': '/zed2i/left/image_rect_color',
            'host': "osw-Stealth-14-AI-Studio-A1VGG",
            'port': "5600",
            'fps':  "5.0",
        }],
    )
    back_ground_video = Node(
        package='zed2i_driver',
        executable='back_cam_ground_video_streamer',
        name='back_cam_ground_video_streamer',
        output='screen',
        parameters=[{
            'image_topic': '/yolo_back/yolo/debug_image',
            'host': "osw-Stealth-14-AI-Studio-A1VGG",
            'port': "5601",
            'fps': "5.0",
        }],
    )

    return LaunchDescription(
        [
            # Forward perception runs inside zed2i_sdk_node. Keep the legacy
            # Python node opt-in only so it cannot duplicate GPU inference or
            # publish competing virtual walls.
        
            joy_converter,
            micon_driver_fd_node,
            thruster_driver,

            sensors_launch,
            localization_launch,

            # ship_detection,
            # yolo,
            # back_yolo,

            # gournd_video,
            # back_ground_video,        
        ]
    )
