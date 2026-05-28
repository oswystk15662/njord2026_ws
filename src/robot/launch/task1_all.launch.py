#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Task1 All Bringup Launch File.

Starts the complete Task1 system for the real robot:
- Driver Layer: IMU, Microcontroller (micon), Thruster, Camera
- Localization: GNSS, GLIM, EKF, Robot State Publisher
- Detection: YOLO CUDA, PCL Buoy Detection
- Navigation: Nav2 with Waypoint Publisher

Default Topics:
  - Camera: /camera/image_raw
  - Lidar: /livox/lidar
  - Odometry: /odometry/filtered/global
  - Navigation Goal: /goal_pose
  - Buoy Detections: /buoy_detections

Usage:
  ros2 launch robot task1_all.launch.py
  ros2 launch robot task1_all.launch.py enable_camera:=true enable_yolo:=true enable_pcl:=true
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for Task1 all bringup."""

    # Timing configuration
    driver_delay = LaunchConfiguration('driver_delay')
    localization_delay = LaunchConfiguration('localization_delay')
    detection_delay = LaunchConfiguration('detection_delay')
    navigation_delay = LaunchConfiguration('navigation_delay')

    # Declare launch arguments
    # Timing arguments
    driver_delay_arg = DeclareLaunchArgument(
        'driver_delay',
        default_value='0.0',
        description='Delay before launching driver layer (seconds)'
    )
    
    localization_delay_arg = DeclareLaunchArgument(
        'localization_delay',
        default_value='1.0',
        description='Delay before launching localization layer (seconds)'
    )
    
    detection_delay_arg = DeclareLaunchArgument(
        'detection_delay',
        default_value='3.0',
        description='Delay before launching detection layer (seconds)'
    )
    
    navigation_delay_arg = DeclareLaunchArgument(
        'navigation_delay',
        default_value='5.0',
        description='Delay before launching navigation layer (seconds)'
    )

    # Declare launch arguments
    # Driver arguments
    micon_port_arg = DeclareLaunchArgument(
        'micon_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for microcontroller (ESP32)'
    )
    
    micon_baudrate_arg = DeclareLaunchArgument(
        'micon_baudrate',
        default_value='115200',
        description='Baudrate for microcontroller serial connection'
    )
    
    imu_port_arg = DeclareLaunchArgument(
        'imu_port',
        default_value='/dev/ttyUSB1',
        description='Serial port for WIT IMU'
    )
    
    camera_device_arg = DeclareLaunchArgument(
        'camera_device',
        default_value='http://localhost:8080/?action=stream',
        description='Camera device URL or path'
    )
    
    # Detection arguments
    yolo_device_arg = DeclareLaunchArgument(
        'yolo_device',
        default_value='cuda:0',
        description='CUDA device for YOLO'
    )
    
    # Navigation arguments
    task_type_arg = DeclareLaunchArgument(
        'task_type',
        default_value='task1',
        description='Task type for waypoint publisher'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time (should be false for real robot)'
    )

    # ========== DRIVER LAYER ==========
    
    # WIT IMU launch
    wit_imu = Node(
        package='wit_node',
        executable='wit_node',
        name='wit_imu_node',
        parameters=[{
            'port': LaunchConfiguration('imu_port'),
            'baud_rate': '115200',
            'frame_id': 'imu_link',
            'publish_hz': 10.0
        }],
        output='screen'
    )
    
    # Microcontroller (ESP32) Agent launch
    micon_esp32_agent = Node(
        package='micon_agent_esp32',
        executable='micon_esp32_agent',
        name='micon_esp32_agent',
        parameters=[{
            'port': LaunchConfiguration('micon_port'),
            'baudrate': LaunchConfiguration('micon_baudrate'),
            'enable_imu': False,
            'enable_sensors': True,
            'publish_rate': 10
        }],
        output='screen'
    )
    
    # Thruster driver launch
    thruster_driver_config = os.path.join(
        get_package_share_directory('thruster_driver'),
        'config',
        'config.yaml'
    )
    
    thruster_driver = Node(
        package='thruster_driver',
        executable='thruster_driver_node',
        name='thruster_driver_node',
        parameters=[thruster_driver_config],
        output='screen'
    )
    
    # Runtime Camera launch
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('usb_camera_driver'),
                'launch',
                'runtime.launch.py'
            )
        ),
        launch_arguments={
            'camera_device': LaunchConfiguration('camera_device'),
        }.items()
    )

    # ========== DRIVER LAYER (with delay) ==========
    driver_layer_timer = TimerAction(
        period=driver_delay,
        actions=[
            wit_imu,
            micon_esp32_agent,
            thruster_driver,
            camera_launch,
        ]
    )

    # ========== LOCALIZATION LAYER ==========
    
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('robot'),
                'launch',
                'localization.launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items()
    )

    # Localization layer with delay
    localization_layer_timer = TimerAction(
        period=localization_delay,
        actions=[localization_launch]
    )

    # ========== DETECTION LAYER ==========
    
    # YOLO CUDA detection
    yolo_detection = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('yolo'),
                'launch',
                'yolo_cuda.launch.py'
            )
        ),
        launch_arguments={
            'device': LaunchConfiguration('yolo_device'),
        }.items()
    )
    
    # PCL Buoy Detection
    pcl_detection = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('pcl_det'),
                'launch',
                'pcl_bouy_det.launch.py'
            )
        ),
        launch_arguments={
            'input_topic': '/livox/lidar',
            'output_topic': '/buoy_detections',
            'frame_id': 'base_link',
        }.items()
    )

    # Detection layer with delay
    detection_layer_timer = TimerAction(
        period=detection_delay,
        actions=[
            yolo_detection,
            pcl_detection,
        ]
    )

    # ========== NAVIGATION LAYER ==========
    
    # Nav2 navigation
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('robot'),
                'launch',
                'nav2.launch.py'
            )
        )
    )
    
    # Waypoint Publisher
    waypoint_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('waypoint_publisher'),
                'launch',
                'waypoint_publisher.launch.py'
            )
        ),
        launch_arguments={
            'task_type': LaunchConfiguration('task_type'),
            'frame_id': 'map',
            'publish_rate_hz': '2.0',
        }.items()
    )

    # Navigation layer with delay
    navigation_layer_timer = TimerAction(
        period=navigation_delay,
        actions=[
            nav2_launch,
            waypoint_publisher,
        ]
    )

    # Startup message
    startup_message = LogInfo(msg='========== Task1 All Bringup Started ==========')

    # Compose launch description
    ld = LaunchDescription([
        # Startup message
        startup_message,
        
        # Declare timing arguments
        driver_delay_arg,
        localization_delay_arg,
        detection_delay_arg,
        navigation_delay_arg,
        
        # Declare launch arguments
        micon_port_arg,
        micon_baudrate_arg,
        imu_port_arg,
        camera_device_arg,
        yolo_device_arg,
        task_type_arg,
        use_sim_time_arg,
        
        # Driver layer (with delay)
        driver_layer_timer,
        
        # Localization layer (with delay)
        localization_layer_timer,
        
        # Detection layer (with delay)
        detection_layer_timer,
        
        # Navigation layer (with delay)
        navigation_layer_timer,
    ])

    return ld
