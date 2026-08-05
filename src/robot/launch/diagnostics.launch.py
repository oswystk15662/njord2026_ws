from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def _monitor(
    monitor_name,
    topic,
    topic_type,
    mode,
    expected_frequency,
    minimum_frequency,
    timeout,
    stale_timeout,
    qos_reliability="best_effort",
):
    diagnostic_name = monitor_name
    if monitor_name == "local_filtered_odom":
        diagnostic_name = "odom_local"
    elif monitor_name == "global_filtered_odom":
        diagnostic_name = "odom_global"
    elif monitor_name == "thruster_command":
        diagnostic_name = "thruster"

    return {
        "monitor_name": monitor_name,
        "diagnostic_name": diagnostic_name,
        "topic": topic,
        "topic_type": topic_type,
        "mode": mode,
        "expected_frequency": expected_frequency,
        "minimum_frequency": minimum_frequency,
        "timeout": timeout,
        "stale_timeout": stale_timeout,
        "window_size": 50,
        "qos_depth": 10,
        "qos_reliability": qos_reliability,
    }


PROFILES = {
    "localization": [
        # This profile is used by the miniPC role.  GLIM runs on the Jetson
        # and the Spatial IMU is optional, so neither topic is a required
        # miniPC heartbeat.  The local EKF subscribes to /livox/imu received
        # from the Jetson; its resulting local odometry is monitored below.
        _monitor(
            "gps_fix",
            "/sensor/vehicle_gnss/fix/raw",
            "sensor_msgs/msg/NavSatFix",
            "required_frequency",
            20.0,
            10.0,
            1.0,
            3.0,
        ),
        _monitor(
            "local_filtered_odom",
            "/odometry/filtered/local",
            "nav_msgs/msg/Odometry",
            "required_frequency",
            30.0,
            5.0,
            1.0,
            4.0,
        ),
        _monitor(
            "global_filtered_odom",
            "/odometry/filtered/global",
            "nav_msgs/msg/Odometry",
            "required_frequency",
            30.0,
            5.0,
            1.0,
            4.0,
        ),
    ],
    "nav2": [
        _monitor(
            "cmd_vel",
            "/cmd_vel",
            "geometry_msgs/msg/Twist",
            "heartbeat_only",
            10.0,
            1.0,
            2.0,
            8.0,
            "reliable",
        ),
        _monitor(
            "global_filtered_odom",
            "/odometry/filtered/global",
            "nav_msgs/msg/Odometry",
            "required_frequency",
            30.0,
            5.0,
            1.0,
            4.0,
        ),
    ],
    "task3": [
        _monitor(
            "sim_odom",
            "/odom",
            "nav_msgs/msg/Odometry",
            "required_frequency",
            50.0,
            20.0,
            0.5,
            2.0,
        ),
        _monitor(
            "imu",
            "/wit/imu",
            "sensor_msgs/msg/Imu",
            "required_frequency",
            20.0,
            10.0,
            0.5,
            2.0,
        ),
        _monitor(
            "gps_fix",
            "/gps/fix",
            "sensor_msgs/msg/NavSatFix",
            "required_frequency",
            10.0,
            2.0,
            1.0,
            3.0,
        ),
        _monitor(
            "local_filtered_odom",
            "/odometry/filtered/local",
            "nav_msgs/msg/Odometry",
            "required_frequency",
            30.0,
            5.0,
            1.0,
            4.0,
        ),
        _monitor(
            "global_filtered_odom",
            "/odometry/filtered/global",
            "nav_msgs/msg/Odometry",
            "required_frequency",
            30.0,
            5.0,
            1.0,
            4.0,
        ),
        _monitor(
            "cmd_vel",
            "/cmd_vel",
            "geometry_msgs/msg/Twist",
            "heartbeat_only",
            10.0,
            1.0,
            2.0,
            8.0,
            "reliable",
        ),
        _monitor(
            "thruster_command",
            "/thruster_command",
            "std_msgs/msg/Int16MultiArray",
            "heartbeat_only",
            10.0,
            1.0,
            2.0,
            8.0,
            "reliable",
        ),
        _monitor(
            "buoy_costmap",
            "/buoy_costmap",
            "nav_msgs/msg/OccupancyGrid",
            "required_frequency",
            5.0,
            1.0,
            2.0,
            6.0,
        ),
    ],
}


def _launch_setup(context, *args, **kwargs):
    profile = LaunchConfiguration("profile").perform(context)
    monitors = PROFILES.get(profile)
    if monitors is None:
        raise RuntimeError(
            f"Unknown diagnostics profile '{profile}'. "
            f"Available profiles: {', '.join(sorted(PROFILES))}"
        )

    components = []
    for monitor in monitors:
        components.append(
            ComposableNode(
                package="diagnostic_monitors",
                plugin="njord::diagnostic_monitors::TopicHeartbeatMonitor",
                name=monitor["diagnostic_name"],
                parameters=[monitor],
            )
        )

    return [
        ComposableNodeContainer(
            name=f"{profile}_diag",
            namespace="",
            package="rclcpp_components",
            executable="component_container_mt",
            composable_node_descriptions=components,
            output="screen",
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "profile",
                default_value="localization",
                description="Diagnostic monitor profile: localization, nav2, or task3",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
