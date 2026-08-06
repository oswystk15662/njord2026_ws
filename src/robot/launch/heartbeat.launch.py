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

    if role == "jetson":
        return [
            _gate(
                "heartbeat_driver_camera_front",
                [("/zed2i/left/image_rect", "sensor_msgs/msg/Image")],
                "/heartbeat/driver/camera/front",
                timeout=1.0,
            ),
            _gate(
                "heartbeat_driver_lidar",
                [
                    ("/livox/lidar", "sensor_msgs/msg/PointCloud2"),
                    ("/livox/imu", "sensor_msgs/msg/Imu"),
                ],
                "/heartbeat/driver/lidar",
                timeout=1.0,
            ),
        ]

    if role == "minipc":
        leaves = [
            _gate(
                "heartbeat_driver_camera_back",
                [("/back_cam/image_raw", "sensor_msgs/msg/Image")],
                "/heartbeat/driver/camera/back",
                timeout=1.0,
            ),
            _gate(
                "heartbeat_driver_gnss",
                [
                    ("/sensor/vehicle_gnss/fix/raw", "sensor_msgs/msg/NavSatFix"),
                    (
                        "/sensor/vehicle_gnss/compass/raw",
                        "geometry_msgs/msg/PoseWithCovarianceStamped",
                    ),
                ],
                "/heartbeat/driver/gnss",
                timeout=1.0,
            ),
            _gate(
                "heartbeat_driver_micon",
                [("/micon/bms_cells", "std_msgs/msg/Float32MultiArray")],
                "/heartbeat/driver/micon",
                timeout=3.0,
            ),
            _gate(
                "heartbeat_localization_local",
                [("/odometry/filtered/local", "nav_msgs/msg/Odometry")],
                "/heartbeat/localization/local",
                timeout=1.0,
            ),
            _gate(
                "heartbeat_localization_global",
                [("/odometry/filtered/global", "nav_msgs/msg/Odometry")],
                "/heartbeat/localization/global",
                timeout=1.0,
            ),
        ]
        groups = [
            _gate(
                "heartbeat_driver_camera",
                [
                    ("/heartbeat/driver/camera/front", "std_msgs/msg/Empty"),
                    ("/heartbeat/driver/camera/back", "std_msgs/msg/Empty"),
                ],
                "/heartbeat/driver/camera",
                timeout=1.5,
            ),
            _gate(
                "heartbeat_driver",
                [
                    ("/heartbeat/driver/camera", "std_msgs/msg/Empty"),
                    ("/heartbeat/driver/lidar", "std_msgs/msg/Empty"),
                    ("/heartbeat/driver/gnss", "std_msgs/msg/Empty"),
                    ("/heartbeat/driver/micon", "std_msgs/msg/Empty"),
                ],
                "/heartbeat/driver",
                timeout=1.5,
            ),
            _gate(
                "heartbeat_localization",
                [
                    ("/heartbeat/localization/local", "std_msgs/msg/Empty"),
                    ("/heartbeat/localization/global", "std_msgs/msg/Empty"),
                ],
                "/heartbeat/localization",
                timeout=1.5,
            ),
        ]
        return leaves + groups

    raise RuntimeError(f"Unknown heartbeat role: {role}")


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "role", choices=["jetson", "minipc"], default_value="minipc"
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
