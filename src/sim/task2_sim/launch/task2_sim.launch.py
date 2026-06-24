import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_task2 = get_package_share_directory("task2_sim")
    pkg_robot = get_package_share_directory("robot")
    pkg_dutyed = get_package_share_directory("dutyed_tf_pub_with_disturbance")
    pkg_sensor_noise = get_package_share_directory("sensor_sim_with_noise")
    pkg_waypoint = get_package_share_directory("waypoint_publisher")
    pkg_thruster = get_package_share_directory("thruster_driver")

    config = os.path.join(pkg_task2, "config", "task2_params.yaml")
    opponent_config = os.path.join(pkg_task2, "config", "task2_opponent_sim.yaml")
    robot_description_file = os.path.join(pkg_robot, "urdf", "robot.urdf_modified.urdf")
    robot_description = open(robot_description_file, "r").read()

    use_dynamics = DeclareLaunchArgument("use_dynamics", default_value="true")
    use_nav2 = DeclareLaunchArgument("use_nav2", default_value="true")
    use_waypoints = DeclareLaunchArgument("use_waypoints", default_value="true")
    use_mppi = DeclareLaunchArgument("use_mppi", default_value="true")

    params_arg = DeclareLaunchArgument("params", default_value=config)
    opponent_params_arg = DeclareLaunchArgument("opponent_params", default_value=opponent_config)

    driver_delay_arg = DeclareLaunchArgument(
        "driver_delay",
        default_value="0.0",
        description="Delay before launching simulation sensor/EKF layer",
    )
    nav2_delay_arg = DeclareLaunchArgument(
        "nav2_delay",
        default_value="5.0",
        description="Delay before launching Nav2",
    )
    goal_delay_arg = DeclareLaunchArgument(
        "goal_delay",
        default_value="8.0",
        description="Delay before launching waypoint_publisher",
    )
    mppi_delay_arg = DeclareLaunchArgument(
        "mppi_delay",
        default_value="2.0",
        description="Delay before launching MPPI planner node",
    )

    driver_delay = LaunchConfiguration("driver_delay")
    nav2_delay = LaunchConfiguration("nav2_delay")
    goal_delay = LaunchConfiguration("goal_delay")
    mppi_delay = LaunchConfiguration("mppi_delay")

    dynamics = Node(
        package="dutyed_tf_pub_with_disturbance",
        executable="dutyed_tf_pub_with_disturbance_node",
        name="dutyed_tf_pub_with_disturbance_node",
        parameters=[
            os.path.join(pkg_dutyed, "config", "node_config.yaml"),
            {"publish_tf": True},
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_dynamics")),
    )

    sensor_noise_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_sensor_noise, "launch", "sensor_noise.launch.py")
        )
    )

    task2_orchestrator = Node(
        package="task2_sim",
        executable="task2_orchestrator",
        name="task2_orchestrator",
        parameters=[LaunchConfiguration("params")],
        output="screen",
    )

    opponent_vessel = Node(
        package="task2_sim",
        executable="opponent_vessel_node",
        name="opponent_vessel_node",
        parameters=[LaunchConfiguration("opponent_params")],
        output="screen",
    )

    ideal_lidar = Node(
        package="task2_sim",
        executable="ideal_lidar_pointcloud_node",
        name="ideal_lidar_pointcloud_node",
        parameters=[LaunchConfiguration("opponent_params")],
        output="screen",
    )

    thruster_driver_node = Node(
        package="thruster_driver",
        executable="thruster_driver_node",
        name="thruster_driver_node",
        parameters=[
            os.path.join(pkg_thruster, "config", "config.yaml"),
            {
                "robot_description": robot_description,
                "transport_mode": "sim",
                "control.dob.enable": False,
            },
        ],
        output="screen",
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": False,
            }
        ],
    )

    local_ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node_local",
        output="screen",
        parameters=[
            os.path.join(pkg_robot, "config", "ekf_local.yaml"),
            {"publish_tf": False},
        ],
        remappings=[("odometry/filtered", "odometry/filtered/local")],
    )

    global_ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node_global",
        output="screen",
        parameters=[
            os.path.join(pkg_robot, "config", "ekf_global.yaml"),
            {"publish_tf": False},
        ],
        remappings=[("odometry/filtered", "odometry/filtered/global")],
    )

    navsat_transform_node = Node(
        package="robot_localization",
        executable="navsat_transform_node",
        name="navsat_transform_node",
        output="screen",
        parameters=[
            {
                "frequency": 10.0,
                "magnetic_declination_radians": 0.0,
                "yaw_offset": 0.0,
                "zero_altitude": True,
                "broadcast_utm_transform": False,
                "publish_filtered_gps": True,
                "use_odometry_yaw": False,
                "wait_for_datum": False,
            }
        ],
        remappings=[
            ("imu", "/wit/imu"),
            ("gps/fix", "/gps/fix"),
            ("odometry/filtered", "odometry/filtered/local"),
        ],
    )

    mppi_planner = Node(
        package="asv_trajectory_planner",
        executable="planner_node",
        name="mppi_planner_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_mppi")),
    )

    sensor_layer_timer = TimerAction(
        period=driver_delay,
        actions=[
            dynamics,
            sensor_noise_launch,
            task2_orchestrator,
            opponent_vessel,
            ideal_lidar,
            thruster_driver_node,
            robot_state_pub_node,
            local_ekf_node,
            global_ekf_node,
            navsat_transform_node,
        ],
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robot, "launch", "nav2.launch.py")
        ),
        launch_arguments={
            "params_file": os.path.join(pkg_robot, "config", "nav2_params_task2.yaml"),
            "enable_diagnostics": "false",
        }.items(),
        condition=IfCondition(LaunchConfiguration("use_nav2")),
    )

    nav2_layer_timer = TimerAction(
        period=nav2_delay,
        actions=[nav2_launch],
    )

    waypoint_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_waypoint, "launch", "waypoint_publisher.launch.py")
        ),
        launch_arguments={
            "task_type": "task2",
            "frame_id": "map",
            "publish_rate_hz": "2.0",
        }.items(),
        condition=IfCondition(LaunchConfiguration("use_waypoints")),
    )

    goal_layer_timer = TimerAction(
        period=goal_delay,
        actions=[waypoint_publisher],
    )

    mppi_layer_timer = TimerAction(
        period=mppi_delay,
        actions=[mppi_planner],
    )

    return LaunchDescription(
        [
            LogInfo(msg="========== Task2 Collision Avoidance Sim Bringup Started =========="),
            use_dynamics,
            use_nav2,
            use_waypoints,
            use_mppi,
            params_arg,
            opponent_params_arg,
            driver_delay_arg,
            nav2_delay_arg,
            goal_delay_arg,
            mppi_delay_arg,
            sensor_layer_timer,
            mppi_layer_timer,
            nav2_layer_timer,
            goal_layer_timer,
        ]
    )