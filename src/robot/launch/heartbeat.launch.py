"""Hierarchical health-derived heartbeat tree for vessel processes."""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
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


def _minipc_gates_from_config():
    """Build the miniPC heartbeat tree from its readable YAML inventory."""
    config_path = os.path.join(
        get_package_share_directory("diagnostic_monitors"),
        "config",
        "minipc_heartbeat.yaml",
    )
    with open(config_path, encoding="utf-8") as config_file:
        config = yaml.safe_load(config_file) or {}

    gates = config.get("minipc_heartbeat")
    if not isinstance(gates, list):
        raise RuntimeError(f"{config_path} must contain a 'minipc_heartbeat' list.")

    result = []
    for gate in gates:
        try:
            inputs = [
                (input_config["topic"], input_config["topic_type"])
                for input_config in gate["inputs"]
            ]
            result.append(
                _gate(
                    gate["name"],
                    inputs,
                    gate["output_topic"],
                    timeout=gate.get("timeout", 2.0),
                    grace=gate.get("startup_grace_sec", 15.0),
                )
            )
        except (KeyError, TypeError) as error:
            raise RuntimeError(f"Invalid heartbeat entry in {config_path}: {gate!r}") from error
    return result


def _launch_setup(context, *args, **kwargs):
    role = LaunchConfiguration("role").perform(context)
    if role == "minipc":
        # The miniPC owns the canonical vessel health summary.  Jetson signals
        # reach this graph through the existing ROS/Zenoh transport; a Jetson
        # local aggregator must use a remapped topic until a role-aware merger
        # is introduced.
        return _minipc_gates_from_config() + [
            Node(
                package="diagnostic_monitors",
                executable="health_state_aggregator_node",
                name="health_state_aggregator",
                output="screen",
                parameters=[
                    {
                        "input_topic": "/health/signals",
                        "output_topic": "/health/state",
                        "signal_timeout_sec": 10.0,
                        "publish_period_sec": 1.0,
                    }
                ],
            )
        ]

    def enabled(argument):
        value = LaunchConfiguration(argument).perform(context).strip().lower()
        return value in {"1", "true", "yes", "on"}

    monitor_zed2i = enabled("heartbeat_monitor_zed2i")
    monitor_lidar = enabled("heartbeat_monitor_lidar")
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

    raise RuntimeError(f"Unknown heartbeat role: {role}")


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "role", choices=["jetson", "minipc"], default_value="minipc"
            ),
            DeclareLaunchArgument("heartbeat_monitor_zed2i", default_value="true"),
            DeclareLaunchArgument("heartbeat_monitor_lidar", default_value="true"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
