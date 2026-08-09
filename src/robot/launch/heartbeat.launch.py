"""Start the validated, role-owned topic heartbeat and health graph.

The role inventory is the source of truth for observation.  Safety policy is
deliberately kept in control_manager; this launch file only turns configured
topics into named HealthSignal messages and aggregates them into HealthState.
"""

from collections.abc import Mapping
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


_ROLES = ("minipc", "jetson", "groundpc")


def _load_inventory(role):
    config_path = os.path.join(
        get_package_share_directory("diagnostic_monitors"),
        "config",
        "heartbeat",
        f"{role}.yaml",
    )
    try:
        with open(config_path, encoding="utf-8") as config_file:
            config = yaml.safe_load(config_file)
    except (OSError, yaml.YAMLError) as error:
        raise RuntimeError(f"Cannot load heartbeat inventory {config_path}: {error}") from error

    if not isinstance(config, Mapping) or config.get("role") != role:
        raise RuntimeError(f"{config_path} must declare role={role!r}")
    signals = config.get("signals")
    if not isinstance(signals, Mapping) or not signals:
        raise RuntimeError(f"{config_path} must contain a non-empty 'signals' mapping")

    validated = []
    for name, signal in signals.items():
        if not isinstance(name, str) or not name.strip() or not isinstance(signal, Mapping):
            raise RuntimeError(f"Invalid heartbeat signal in {config_path}: {name!r}")
        required = ("topic", "type", "timeout_sec", "expected_frequency_hz")
        missing = [key for key in required if key not in signal]
        if missing:
            raise RuntimeError(f"Signal {name!r} in {config_path} misses {missing}")
        topic = signal["topic"]
        topic_type = signal["type"]
        if not isinstance(topic, str) or not topic.startswith("/"):
            raise RuntimeError(f"Signal {name!r} topic must be an absolute ROS topic")
        if not isinstance(topic_type, str) or "/" not in topic_type:
            raise RuntimeError(f"Signal {name!r} type must look like package/msg/Type")
        try:
            timeout = float(signal["timeout_sec"])
            expected = float(signal["expected_frequency_hz"])
        except (TypeError, ValueError) as error:
            raise RuntimeError(f"Signal {name!r} has invalid numeric values") from error
        if timeout <= 0.0 or expected < 0.0:
            raise RuntimeError(f"Signal {name!r} has invalid timeout/frequency")
        enabled = signal.get("enabled", True)
        if not isinstance(enabled, bool):
            raise RuntimeError(f"Signal {name!r} enabled must be boolean")
        if enabled:
            validated.append((name, topic, topic_type, timeout, expected))
    return validated


def _monitor(role, name, topic, topic_type, timeout, expected):
    # Keep signal names role-qualified on the transport so a Jetson/ground-PC
    # graph cannot accidentally overwrite a miniPC signal with the same name.
    return Node(
        package="diagnostic_monitors",
        executable="topic_heartbeat_monitor_node",
        name=f"heartbeat_{role}_{name}",
        output="screen",
        parameters=[
            {
                "monitor_name": name,
                "topic": topic,
                "topic_type": topic_type,
                "mode": "required_frequency",
                "expected_frequency": expected,
                "minimum_frequency": expected,
                "timeout": timeout,
                "stale_timeout": max(timeout * 3.0, timeout + 1.0),
                "health_signal_topic": f"/health/signals/{role}",
            }
        ],
    )


def _launch_setup(context, *args, **kwargs):
    del args, kwargs
    role = LaunchConfiguration("role").perform(context).strip().lower()
    if role not in _ROLES:
        raise RuntimeError(f"Unknown heartbeat role: {role!r}")

    signals = _load_inventory(role)
    # These arguments remain accepted for compatibility with existing Jetson
    # bringup files.  The inventory still records all signals; disabling a
    # legacy branch only suppresses its monitor process for that launch.
    monitor_zed2i = LaunchConfiguration("heartbeat_monitor_zed2i").perform(context)
    monitor_lidar = LaunchConfiguration("heartbeat_monitor_lidar").perform(context)
    disabled = set()
    if monitor_zed2i.strip().lower() in {"0", "false", "no", "off"}:
        disabled.add("front_camera")
    if monitor_lidar.strip().lower() in {"0", "false", "no", "off"}:
        disabled.update({"lidar_points", "lidar_imu"})

    nodes = [
        _monitor(role, name, topic, topic_type, timeout, expected)
        for name, topic, topic_type, timeout, expected in signals
        if name not in disabled
    ]
    # miniPC is the canonical vessel health owner.  Role-local summaries on
    # Jetson and ground-PC remain observable without creating competing
    # publishers of /health/state.
    health_topic = "/health/state" if role == "minipc" else f"/health/state/{role}"
    nodes.append(
        Node(
            package="diagnostic_monitors",
            executable="health_state_aggregator_node",
            name=f"health_state_aggregator_{role}",
            output="screen",
            parameters=[
                {
                    "input_topic": f"/health/signals/{role}",
                    "output_topic": health_topic,
                    "signal_timeout_sec": 10.0,
                    "publish_period_sec": 1.0,
                }
            ],
        )
    )
    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("role", choices=list(_ROLES), default_value="minipc"),
            DeclareLaunchArgument("heartbeat_monitor_zed2i", default_value="true"),
            DeclareLaunchArgument("heartbeat_monitor_lidar", default_value="true"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
