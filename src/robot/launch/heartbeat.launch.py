"""Hierarchical health-derived heartbeat tree for vessel processes."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _gate(name, inputs, output, timeout=2.0, grace=15.0):
    return Node(
        package="diagnostic_monitors",
        executable="heartbeat_aggregator_node",
        name=name,
        output="screen",
        parameters=[
            {
                "input_topics": [topic for topic, _ in inputs],
                "input_types": [topic_type for _, topic_type in inputs],
                "output_topic": output,
                "input_timeout_sec": timeout,
                "startup_grace_sec": grace,
                "publish_period_sec": 0.5,
            }
        ],
    )


def _launch_setup(context, *args, **kwargs):
    role = LaunchConfiguration("role").perform(context)
    def enabled(argument):
        value = LaunchConfiguration(argument).perform(context).strip().lower()
        return value in {"1", "true", "yes", "on"}

    monitor_zed2i = enabled("heartbeat_monitor_zed2i")
    monitor_lidar = enabled("heartbeat_monitor_lidar")
    monitor_ekf_local = enabled("heartbeat_monitor_ekf_local")
    monitor_gnss_compass = enabled("heartbeat_monitor_gnss_compass")
    # An intentionally disabled EKF has no odometry publisher.  Do not make
    # that absence unhealthy merely because its heartbeat monitor was enabled.
    monitor_ekf_global = (
        enabled("heartbeat_monitor_ekf_global") and enabled("enable_global_ekf")
    )

    if role == "jetson":
        gates = []
        if monitor_zed2i:
            gates.append(_gate(
                "heartbeat_driver_camera_front",
                [("/zed2i/left/image_rect", "sensor_msgs/msg/Image")],
                "/heartbeat/driver/camera/front",
                timeout=1.0,
            ))
        if monitor_lidar:
            gates.append(_gate(
                "heartbeat_driver_lidar",
                [
                    ("/livox/lidar", "sensor_msgs/msg/PointCloud2"),
                    ("/livox/imu", "sensor_msgs/msg/Imu"),
                ],
                "/heartbeat/driver/lidar",
                timeout=1.0,
            ))
        return gates

    if role == "minipc":
        gnss_inputs = [
            ("/sensor/vehicle_gnss/fix/raw", "sensor_msgs/msg/NavSatFix"),
        ]
        if monitor_gnss_compass:
            gnss_inputs.append(
                (
                    "/sensor/vehicle_gnss/compass/raw",
                    "geometry_msgs/msg/PoseWithCovarianceStamped",
                )
            )

        leaves = [
            _gate(
                "heartbeat_driver_camera_back",
                [("/back_cam/image_raw", "sensor_msgs/msg/Image")],
                "/heartbeat/driver/camera/back",
                timeout=1.0,
            ),
            _gate(
                "heartbeat_driver_gnss",
                gnss_inputs,
                "/heartbeat/driver/gnss",
                timeout=1.0,
            ),
            _gate(
                "heartbeat_driver_micon",
                [("/micon/bms_cells", "std_msgs/msg/Float32MultiArray")],
                "/heartbeat/driver/micon",
                timeout=3.0,
            ),
        ]
        if monitor_ekf_local:
            leaves.append(_gate(
                "heartbeat_localization_local",
                [("/odometry/filtered/local", "nav_msgs/msg/Odometry")],
                "/heartbeat/localization/local",
                timeout=1.0,
            ))
        if monitor_ekf_global:
            leaves.append(_gate(
                "heartbeat_localization_global",
                [("/odometry/filtered/global", "nav_msgs/msg/Odometry")],
                "/heartbeat/localization/global",
                timeout=1.0,
            ))
        camera_inputs = [("/heartbeat/driver/camera/back", "std_msgs/msg/Empty")]
        driver_inputs = [
            ("/heartbeat/driver/gnss", "std_msgs/msg/Empty"),
            ("/heartbeat/driver/micon", "std_msgs/msg/Empty"),
        ]
        localization_inputs = []
        if monitor_zed2i:
            camera_inputs.insert(0, ("/heartbeat/driver/camera/front", "std_msgs/msg/Empty"))
        if monitor_lidar:
            driver_inputs.insert(0, ("/heartbeat/driver/lidar", "std_msgs/msg/Empty"))
        if monitor_ekf_local:
            localization_inputs.append(("/heartbeat/localization/local", "std_msgs/msg/Empty"))
        if monitor_ekf_global:
            localization_inputs.append(("/heartbeat/localization/global", "std_msgs/msg/Empty"))

        groups = [
            _gate(
                "heartbeat_driver_camera",
                camera_inputs,
                "/heartbeat/driver/camera",
                timeout=1.5,
            ),
            _gate(
                "heartbeat_driver",
                [
                    ("/heartbeat/driver/camera", "std_msgs/msg/Empty"),
                ] + driver_inputs,
                "/heartbeat/driver",
                timeout=1.5,
            ),
        ]
        if localization_inputs:
            groups.append(
                _gate(
                    "heartbeat_localization",
                    localization_inputs,
                    "/heartbeat/localization",
                    timeout=1.5,
                )
            )
        return leaves + groups

    raise RuntimeError(f"Unknown heartbeat role: {role}")


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "role", choices=["jetson", "minipc"], default_value="minipc"
            ),
            DeclareLaunchArgument("heartbeat_monitor_zed2i", default_value="true"),
            DeclareLaunchArgument("heartbeat_monitor_lidar", default_value="true"),
            DeclareLaunchArgument("heartbeat_monitor_ekf_local", default_value="true"),
            DeclareLaunchArgument("heartbeat_monitor_ekf_global", default_value="true"),
            DeclareLaunchArgument(
                "heartbeat_monitor_gnss_compass",
                default_value="false",
                description="Include UM982 compass output in the GNSS heartbeat.",
            ),
            DeclareLaunchArgument("enable_global_ekf", default_value="true"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
