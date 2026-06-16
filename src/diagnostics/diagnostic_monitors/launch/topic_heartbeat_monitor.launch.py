from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    args = [
        DeclareLaunchArgument("monitor_name", default_value="topic"),
        DeclareLaunchArgument("topic", default_value="/diagnostics_target"),
        DeclareLaunchArgument("topic_type", default_value="std_msgs/msg/String"),
        DeclareLaunchArgument("mode", default_value="required_frequency"),
        DeclareLaunchArgument("expected_frequency", default_value="1.0"),
        DeclareLaunchArgument("minimum_frequency", default_value="0.1"),
        DeclareLaunchArgument("timeout", default_value="5.0"),
        DeclareLaunchArgument("stale_timeout", default_value="15.0"),
        DeclareLaunchArgument("window_size", default_value="50"),
        DeclareLaunchArgument("qos_reliability", default_value="best_effort"),
        DeclareLaunchArgument("qos_depth", default_value="10"),
    ]

    monitor = ComposableNode(
        package="diagnostic_monitors",
        plugin="njord::diagnostic_monitors::TopicHeartbeatMonitor",
        name="topic_heartbeat_monitor",
        parameters=[
            {
                "monitor_name": LaunchConfiguration("monitor_name"),
                "topic": LaunchConfiguration("topic"),
                "topic_type": LaunchConfiguration("topic_type"),
                "mode": LaunchConfiguration("mode"),
                "expected_frequency": ParameterValue(
                    LaunchConfiguration("expected_frequency"), value_type=float
                ),
                "minimum_frequency": ParameterValue(
                    LaunchConfiguration("minimum_frequency"), value_type=float
                ),
                "timeout": ParameterValue(
                    LaunchConfiguration("timeout"), value_type=float
                ),
                "stale_timeout": ParameterValue(
                    LaunchConfiguration("stale_timeout"), value_type=float
                ),
                "window_size": ParameterValue(
                    LaunchConfiguration("window_size"), value_type=int
                ),
                "qos_reliability": LaunchConfiguration("qos_reliability"),
                "qos_depth": ParameterValue(
                    LaunchConfiguration("qos_depth"), value_type=int
                ),
            }
        ],
    )

    container = ComposableNodeContainer(
        name="topic_heartbeat_monitor_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[monitor],
        output="screen",
    )

    return LaunchDescription(args + [container])
