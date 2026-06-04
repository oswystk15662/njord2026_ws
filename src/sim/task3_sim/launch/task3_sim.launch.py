import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_task3_sim  = get_package_share_directory("task3_sim")
    pkg_task1_sim  = get_package_share_directory("task1_sim")
    pkg_robot      = get_package_share_directory("robot")
    pkg_dutyed     = get_package_share_directory("dutyed_tf_pub_with_disturbance")
    pkg_waypoint   = get_package_share_directory("waypoint_publisher")
    pkg_kinematics = get_package_share_directory("kinematics")
    pkg_buoy_pub   = get_package_share_directory("buoy_obstacle_publisher")

    # ── Launch arguments ──────────────────────────────────────────────────────
    driver_delay_arg = DeclareLaunchArgument(
        'driver_delay', default_value='0.0',
        description='Delay before launching driver layer')
    navigation_delay_arg = DeclareLaunchArgument(
        'navigation_delay', default_value='3.0',
        description='Delay before launching navigation layer')
    task_type_arg = DeclareLaunchArgument(
        'task_type', default_value='task3_1',
        description='Task type: task3_1 or task3_2')

    driver_delay    = LaunchConfiguration('driver_delay')
    navigation_delay = LaunchConfiguration('navigation_delay')
    task_type       = LaunchConfiguration('task_type')

    # ── DRIVER LAYER ──────────────────────────────────────────────────────────
    # Physics simulation (odom ground truth, TF disabled — EKF handles odom→base_link)
    sim_dynamics = Node(
        package="dutyed_tf_pub_with_disturbance",
        executable="dutyed_tf_pub_with_disturbance_node",
        name="dutyed_tf_pub_with_disturbance_node",
        parameters=[
            os.path.join(pkg_dutyed, "config", "node_config.yaml"),
            {"publish_tf": False}   # EKF publishes odom→base_link
        ],
        output="screen"
    )

    # Sensor noise simulator: /odom → /wit/imu + /gps/fix + /sensor/gnss/compass/raw
    pkg_sensor_noise = get_package_share_directory("sensor_sim_with_noise")
    sensor_noise_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_sensor_noise, "launch", "sensor_noise.launch.py")))

    # Task3 orchestrator: buoys (3.1 + 3.2), dock geometry, PointCloud2, TFs
    config = os.path.join(pkg_task3_sim, "config", "task3_params.yaml")
    task3_orchestrator = Node(
        package="task3_sim",
        executable="task3_orchestrator",
        name="task3_orchestrator",
        parameters=[config, {"task_type": task_type}],
        output="screen",
    )

    # Kinematics: cmd_vel → /thruster_command (Int16MultiArray)
    kinematics_node = Node(
        package="kinematics",
        executable="kinematics_node",
        name="kinematics_node",
        parameters=[os.path.join(pkg_kinematics, "config", "config.yaml")],
        output="screen",
    )

    # Robot state publisher: publishes base_link→{imu_link, livox_frame, gnss_link, …} TFs from URDF
    # NOTE: uses .urdf.xacro — must be pre-processed. task3_sim uses the pre-generated .urdf
    robot_description_file = os.path.join(pkg_robot, 'urdf', 'robot.urdf_modified.urdf')
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': open(robot_description_file, 'r').read(),
            'use_sim_time': False
        }]
    )

    # EKF local: /odom + /wit/imu → odometry/filtered/local  (odom→base_link TF)
    local_ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_local',
        output='screen',
        parameters=[os.path.join(pkg_robot, 'config', 'ekf_local.yaml')],
        remappings=[('odometry/filtered', 'odometry/filtered/local')]
    )

    # EKF global: odometry/filtered/local + /odometry/gps → map→odom TF
    global_ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_global',
        output='screen',
        parameters=[os.path.join(pkg_robot, 'config', 'ekf_global.yaml')],
        remappings=[('odometry/filtered', 'odometry/filtered/global')]
    )

    # NavSat transform: /gps/fix + /wit/imu → /odometry/gps
    navsat_transform_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[{
            'frequency': 10.0,
            'magnetic_declination_radians': 0.0,
            'yaw_offset': 0.0,
            'zero_altitude': True,
            'broadcast_utm_transform': False,  # Disable: UTM coords fly millions of meters in RViz
            'publish_filtered_gps': True,
            'use_odometry_yaw': False,
            'wait_for_datum': False,
        }],
        remappings=[
            ('imu', '/wit/imu'),
            ('gps/fix', '/gps/fix'),
            ('odometry/filtered', 'odometry/filtered/local')
        ]
    )

    # Field boundary costmap: latched OccupancyGrid — outside 40m×40m square = LETHAL
    field_boundary_node = Node(
        package='buoy_obstacle_publisher',
        executable='field_boundary_publisher',
        name='field_boundary_publisher',
        parameters=[{
            'map_frame': 'map',
            'resolution': 0.2,
            'map_size_m': 80.0,
            'field_size_m': 40.0,
            'field_center_x': 0.0,
            'field_center_y': 0.0,
            'boundary_cost': 100,
        }],
        output='screen',
    )

    # Buoy obstacle publisher: buoy TFs → /buoy_costmap (OccupancyGrid)
    buoy_obstacle_node = Node(
        package='buoy_obstacle_publisher',
        executable='buoy_obstacle_publisher',
        name='buoy_obstacle_publisher',
        parameters=[os.path.join(pkg_buoy_pub, 'config', 'buoy_obstacle_publisher.yaml')],
        output='screen',
    )

    driver_layer_timer = TimerAction(
        period=driver_delay,
        actions=[
            sim_dynamics,
            sensor_noise_launch,
            task3_orchestrator,
            kinematics_node,
            robot_state_pub_node,
            local_ekf_node,
            global_ekf_node,
            navsat_transform_node,
            field_boundary_node,
            buoy_obstacle_node,
        ]
    )

    # ── NAVIGATION LAYER ─────────────────────────────────────────────────────
    # nav2.launch.py parametrized with task3-specific nav2 params
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robot, "launch", "nav2.launch.py")
        ),
        launch_arguments={
            'params_file': os.path.join(pkg_robot, 'config', 'nav2_params_task3.yaml'),
        }.items()
    )

    waypoint_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_waypoint, "launch", "waypoint_publisher.launch.py")),
        launch_arguments={
            'task_type': task_type,
            'frame_id': 'map',
            'publish_rate_hz': '2.0',
        }.items()
    )

    navigation_layer_timer = TimerAction(
        period=navigation_delay,
        actions=[nav2_launch, waypoint_publisher]
    )

    startup_message = LogInfo(msg='========== Task3 Sim Bringup Started ==========')

    return LaunchDescription([
        startup_message,
        driver_delay_arg,
        navigation_delay_arg,
        task_type_arg,
        driver_layer_timer,
        navigation_layer_timer,
    ])
