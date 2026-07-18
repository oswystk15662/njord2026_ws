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
    pkg_asv_planner = get_package_share_directory("asv_trajectory_planner")

    config = os.path.join(pkg_task2, "config", "task2_params.yaml")
    opponent_config = os.path.join(pkg_task2, "config", "task2_opponent_sim.yaml")
    robot_description_file = os.path.join(pkg_robot, "urdf", "robot.urdf_modified.urdf")
    robot_description = open(robot_description_file, "r").read()

    use_dynamics = DeclareLaunchArgument("use_dynamics", default_value="true")
    use_nav2 = DeclareLaunchArgument("use_nav2", default_value="true")
    use_waypoints = DeclareLaunchArgument("use_waypoints", default_value="false")
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
        default_value="2.0",
        description="Delay before launching Nav2",
    )
    goal_delay_arg = DeclareLaunchArgument(
        "goal_delay",
        default_value="3.0",
        description="Delay before launching goal/planner layer",
    )

    opponent_delay_arg = DeclareLaunchArgument(
        "opponent_delay",
        default_value="10.0",
        description="Delay before launching opponent vessel layer",
    )

    driver_delay = LaunchConfiguration("driver_delay")
    nav2_delay = LaunchConfiguration("nav2_delay")
    goal_delay = LaunchConfiguration("goal_delay")
    opponent_delay = LaunchConfiguration("opponent_delay")

    dynamics = Node(
        package="dutyed_tf_pub_with_disturbance",
        executable="dutyed_tf_pub_with_disturbance_node",
        name="dutyed_tf_pub_with_disturbance_node",
        parameters=[
            os.path.join(pkg_dutyed, "config", "node_config.yaml"),
            {
                "publish_tf": True,
                # Sim-only: duty counts come from sim_thruster_command_adapter,
                # not from the real Float32MultiArray /thruster_command.
                "topic_thruster_command": "/sim/thruster_duty",
            },
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_dynamics")),
    )

    # Sim-only bridge: /thruster_command (Float32MultiArray, N) -> duty counts.
    # Keeps the real chain thruster_driver -> /thruster_command untouched.
    sim_thruster_adapter = Node(
        package="task2_sim",
        executable="sim_thruster_command_adapter",
        name="sim_thruster_command_adapter",
        parameters=[{
            "input_topic": "/thruster_command",
            "output_topic": "/sim/thruster_duty",
            "force_per_duty": [40.0, 40.0, 40.0, 40.0],
            "duty_resolution": 1000,
        }],
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
                "control.dob.enable": False,
            },
        ],
        output="screen",
        remappings=[
            ('/cmd_vel', '/cmd_vel_thruster'),
            ('cmd_vel', '/cmd_vel_thruster'),
        ]
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": False,
        }],
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
        parameters=[{
            "frequency": 10.0,
            "magnetic_declination_radians": 0.0,
            "yaw_offset": 0.0,
            "zero_altitude": True,
            "broadcast_utm_transform": False,
            "publish_filtered_gps": True,
            "use_odometry_yaw": False,
            "wait_for_datum": False,
        }],
        remappings=[
            ("imu", "/wit/imu"),
            ("gps/fix", "/gps/fix"),
            ("odometry/filtered", "odometry/filtered/local"),
        ],
    )

    sensor_layer_timer = TimerAction(
        period=driver_delay,
        actions=[
            dynamics,
            sim_thruster_adapter,
            sensor_noise_launch,
            task2_orchestrator,
            thruster_driver_node,
            robot_state_pub_node,
            local_ekf_node,
            global_ekf_node,
            navsat_transform_node,
        ],
    )

    # Include the Task 2 Nav2 launch directly so robot/nav2.launch.py can stay
    # on the standard nav2_bringup navigation_launch.py for the real vessel.
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robot, "launch", "navigation_launch_task2.py")
        ),
        launch_arguments={
            "params_file": os.path.join(pkg_robot, "config", "nav2_params_task2.yaml"),
            "use_sim_time": "false",
            "autostart": "true",
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

    mppi_planner_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_asv_planner,
                "launch",
                "planner_with_follow_path.launch.py",
            )
        ),
        condition=IfCondition(LaunchConfiguration("use_mppi")),
    )

    mppi_layer_timer = TimerAction(
        period=goal_delay,
        actions=[mppi_planner_launch],
    )

    opponent_layer_timer = TimerAction(
        period=opponent_delay,
        actions=[
            opponent_vessel,
            ideal_lidar,
        ],
    )

    return LaunchDescription([
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
        opponent_delay_arg,
        sensor_layer_timer,
        nav2_layer_timer,
        goal_layer_timer,
        mppi_layer_timer,
        opponent_layer_timer,
    ])
