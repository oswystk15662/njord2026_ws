import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def include_launch(package_name, relative_path, condition, launch_arguments=None):
    package_share = get_package_share_directory(package_name)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(package_share, *relative_path)),
        condition=condition,
        launch_arguments=(launch_arguments or {}).items(),
    )


def generate_launch_description():
    pkg_share = get_package_share_directory("task1_sim")
    pkg_robot = get_package_share_directory("robot")
    pkg_dutyed = get_package_share_directory("dutyed_tf_pub_with_disturbance")
    pkg_sensor_noise = get_package_share_directory("sensor_sim_with_noise")
    pkg_thruster = get_package_share_directory("thruster_driver")

    config = os.path.join(pkg_share, "config", "task1_params.yaml")
    nav2_params = os.path.join(pkg_share, "config", "task1_nav2_params.yaml")
    nav_through_poses_bt_xml = os.path.join(
        pkg_share,
        "behavior_trees",
        "navigate_through_poses_w_replanning_and_recovery.xml",
    )
    nav_to_pose_bt_xml = os.path.join(
        get_package_share_directory("nav2_bt_navigator"),
        "behavior_trees",
        "navigate_to_pose_w_replanning_and_recovery.xml",
    )
    robot_description_file = os.path.join(pkg_robot, "urdf", "robot.urdf_modified.urdf")
    robot_description = open(robot_description_file, "r").read()

    # Startup timing (mirrors task2/task3):
    #   t=0.0  SENSOR layer: sim_dynamics, sensor_noise, thruster_driver, orchestrator, EKF, navsat
    #   t=5.0  NAV2 layer:   Nav2 bringup (needs SimNode TF + filtered odometry)
    #   t=8.0  GOAL layer:   waypoint_publisher (needs NavigateThroughPoses action server)
    use_dynamics_arg = DeclareLaunchArgument("use_dynamics", default_value="true")
    use_nav2_arg = DeclareLaunchArgument("use_nav2", default_value="true")
    use_thruster_driver_arg = DeclareLaunchArgument("use_thruster_driver", default_value="true")
    use_waypoints_arg = DeclareLaunchArgument("use_waypoints", default_value="true")
    use_validator_arg = DeclareLaunchArgument("use_validator", default_value="true")
    use_sensor_noise_arg = DeclareLaunchArgument("use_sensor_noise", default_value="true")
    use_local_ekf_arg = DeclareLaunchArgument("use_local_ekf", default_value="true")
    use_global_ekf_arg = DeclareLaunchArgument("use_global_ekf", default_value="true")
    use_navsat_arg = DeclareLaunchArgument("use_navsat", default_value="true")
    task_type_arg = DeclareLaunchArgument(
        "task_type",
        default_value="task1",
        description="Waypoint set: 'task1' (competition scenario) or 'task1_follow' (lawnmower survey)",
    )
    driver_delay_arg = DeclareLaunchArgument(
        "driver_delay",
        default_value="0.0",
        description="Delay before launching Task1 sensor/dynamics/EKF layer",
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
    params_arg = DeclareLaunchArgument("params", default_value=config)
    nav2_params_arg = DeclareLaunchArgument("nav2_params", default_value=nav2_params)

    # ── SENSOR / PHYSICS LAYER (t=0) ─────────────────────────────────────────
    # SimNode is the sole simulation TF authority (publish_tf=True). EKF/navsat
    # run with publish_tf=False so they only produce filtered odometry topics.
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
        ),
        condition=IfCondition(LaunchConfiguration("use_sensor_noise")),
    )

    # Thruster driver: cmd_vel -> /thruster_command with P + DOB velocity control.
    # Replaces the deprecated open-loop `kinematics` node.
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
        condition=IfCondition(LaunchConfiguration("use_thruster_driver")),
    )

    # Keep the simulation on the vessel command path.  Nav2's collision
    # monitor publishes /cmd_vel_auto; twist_mux is the sole owner of /cmd_vel.
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        name="twist_mux",
        parameters=[
            os.path.join(pkg_robot, "config", "twist_mux.yaml"),
            {"topics.navigation.topic": "/cmd_vel_auto"},
        ],
        remappings=[("cmd_vel_out", "/cmd_vel")],
        output="screen",
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
            os.path.join(pkg_share, "config", "task1_ekf_local.yaml"),
            {"publish_tf": False},
        ],
        remappings=[("odometry/filtered", "odometry/filtered/local")],
        condition=IfCondition(LaunchConfiguration("use_local_ekf")),
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
        condition=IfCondition(LaunchConfiguration("use_global_ekf")),
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
        condition=IfCondition(LaunchConfiguration("use_navsat")),
    )

    orchestrator = Node(
        package="task1_sim",
        executable="task1_orchestrator",
        name="task1_orchestrator",
        parameters=[LaunchConfiguration("params")],
        output="screen",
    )

    validator = include_launch(
        "operation_validator",
        ["launch", "operation_validator.launch.py"],
        IfCondition(LaunchConfiguration("use_validator")),
    )

    cardinal_walls = Node(
        package="buoy_obstacle_publisher",
        executable="cardinal_wall_publisher",
        name="cardinal_wall_publisher",
        parameters=[{
            "detection_topic": "/buoy_detections_3d",
            "output_topic": "/virtual_obstacles",
            "map_frame": "map",
            "course_bounds": [-5.0, 55.0, -40.0, 15.0],
            "wall_width_m": 0.2,
            "point_spacing_m": 0.05,
            "confirmations_required": 2,
        }],
        output="screen",
    )

    sensor_layer_timer = TimerAction(
        period=LaunchConfiguration("driver_delay"),
        actions=[
            dynamics,
            sensor_noise_launch,
            thruster_driver_node,
            twist_mux,
            robot_state_pub_node,
            local_ekf_node,
            global_ekf_node,
            navsat_transform_node,
            validator,
            orchestrator,
            cardinal_walls,
        ],
    )

    # ── NAV2 LAYER (t=5s) ────────────────────────────────────────────────────
    configured_nav2_params = RewrittenYaml(
        source_file=LaunchConfiguration("nav2_params"),
        root_key=None,
        param_rewrites={
            "bt_navigator.ros__parameters.default_nav_through_poses_bt_xml": nav_through_poses_bt_xml,
            "bt_navigator.ros__parameters.default_nav_to_pose_bt_xml": nav_to_pose_bt_xml,
        },
        convert_types=True,
    )

    nav2 = include_launch(
        "nav2_bringup",
        ["launch", "navigation_launch.py"],
        IfCondition(LaunchConfiguration("use_nav2")),
        {
            "params_file": configured_nav2_params,
            "use_sim_time": "false",
            "autostart": "true",
        },
    )

    nav2_layer_timer = TimerAction(
        period=LaunchConfiguration("nav2_delay"),
        actions=[nav2],
    )

    # ── GOAL LAYER (t=8s) ────────────────────────────────────────────────────
    waypoints = include_launch(
        "waypoint_publisher",
        ["launch", "waypoint_publisher.launch.py"],
        IfCondition(LaunchConfiguration("use_waypoints")),
        {
            "task_type": LaunchConfiguration("task_type"),
            "frame_id": "map",
            "publish_rate_hz": "2.0",
        },
    )

    goal_layer_timer = TimerAction(
        period=LaunchConfiguration("goal_delay"),
        actions=[waypoints],
    )

    startup_message = LogInfo(msg="========== Task1 Sim Bringup Started ==========")

    return LaunchDescription([
        startup_message,
        use_dynamics_arg,
        use_nav2_arg,
        use_thruster_driver_arg,
        use_waypoints_arg,
        use_validator_arg,
        use_sensor_noise_arg,
        use_local_ekf_arg,
        use_global_ekf_arg,
        use_navsat_arg,
        task_type_arg,
        driver_delay_arg,
        nav2_delay_arg,
        goal_delay_arg,
        params_arg,
        nav2_params_arg,
        sensor_layer_timer,
        nav2_layer_timer,
        goal_layer_timer,
    ])
