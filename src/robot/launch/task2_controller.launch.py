"""miniPC-side Task 2 controller and independent near-obstacle stop.

Start this after the miniPC manual-control stack is healthy.  The Jetson
launches perception, tracking and MPPI; it bridges only /planned_path_pruned
and the downsampled /task2/safety_points cloud.  This launch keeps the Nav2
FollowPath action, velocity smoothing, Collision Monitor and the command
arbiter input on the miniPC, next to the actuator safety boundary.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _include(package_name, launch_file, arguments=None):
    return GroupAction(
        scoped=True,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare(package_name), "launch", launch_file])
                ),
                launch_arguments=(arguments or {}).items(),
            )
        ],
    )


def generate_launch_description():
    nav2 = _include(
        "robot",
        "navigation_launch_task2.py",
        {
            "params_file": LaunchConfiguration("params_file"),
            "use_sim_time": "false",
            "autostart": "true",
            "auto_cmd_vel_topic": "/cmd_vel_nav",
        },
    )
    follow_path = Node(
        package="asv_trajectory_planner",
        executable="follow_path_client_node",
        name="follow_path_client_node",
        output="screen",
        parameters=[{
            "path_topic": "/planned_path_pruned",
            "action_name": "/follow_path",
            "controller_id": "FollowPath",
            "goal_checker_id": "general_goal_checker",
            "send_frequency": 1.0,
            "enable_replanning": True,
        }],
    )
    autonomy_ready = Node(
        package="asv_trajectory_planner",
        executable="task2_autonomy_ready_node",
        name="task2_autonomy_ready",
        output="screen",
        parameters=[{
            "path_topic": "/planned_path_pruned",
            "action_name": "/follow_path",
            "path_timeout_sec": 2.0,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("robot"), "config", "nav2_params_task2_humble.yaml"]),
            description="Humble Nav2 parameters, including the Collision Monitor.",
        ),
        nav2,
        follow_path,
        autonomy_ready,
    ])
