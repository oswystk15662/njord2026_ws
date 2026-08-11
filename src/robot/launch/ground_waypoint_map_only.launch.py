"""Ground-PC route inspection without GNSS, navigation, or vessel control."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Start only Foxglove and the WGS84 waypoint-marker publisher.

    This launch intentionally has no GNSS subscriber, Nav2 client, mission
    client, critical-link, video receiver, or propulsion-related node.  It is
    suitable for inspecting a route on a laptop with no vessel connected.
    """
    task_type = LaunchConfiguration("task_type")
    marker_topic = LaunchConfiguration("marker_topic")
    enable_foxglove_bridge = LaunchConfiguration("enable_foxglove_bridge")

    waypoint_map = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("waypoint_publisher"), "launch", "waypoint_map.launch.py"]
            )
        ),
        launch_arguments={
            "task_type": task_type,
            "marker_topic": marker_topic,
        }.items(),
    )

    foxglove_bridge = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("foxglove_bridge"), "launch", "foxglove_bridge_launch.xml"]
            )
        ),
        condition=IfCondition(enable_foxglove_bridge),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "task_type",
                default_value="task4",
                choices=[
                    "task1", "task1_skip_1_1", "task1_follow", "task2",
                    "task3_1", "task3_2", "task4",
                ],
                description="Surveyed route to show; this launch never starts a mission.",
            ),
            DeclareLaunchArgument(
                "marker_topic",
                default_value="/ground_waypoint_markers",
            ),
            DeclareLaunchArgument(
                "enable_foxglove_bridge",
                default_value="true",
                description="Expose the display-only marker topic to Foxglove.",
            ),
            waypoint_map,
            foxglove_bridge,
        ]
    )
