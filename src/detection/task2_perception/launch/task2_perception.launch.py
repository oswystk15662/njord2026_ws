"""Task 2 perception launch.

Starts the two task2_perception nodes with task2_perception_params.yaml.

NOTE: the pcl_segmentation submodule pipeline (classical_pipeline.launch.py
with lidar_topic:=/task2/points_filtered and
ego_odom_topic:=/odometry/filtered/local) is intentionally NOT included here;
the real-robot integration launch owns that composition.

Arguments (all default true):
  enable_cloud_filter       start task2_cloud_filter
  enable_opponent_selector  start opponent_selector
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = PathJoinSubstitution(
        [FindPackageShare("task2_perception"), "config",
         "task2_perception_params.yaml"])

    return LaunchDescription([
        DeclareLaunchArgument(
            "enable_cloud_filter", default_value="true",
            description="Start task2_cloud_filter (/livox/lidar -> "
                        "/task2/points_filtered)"),
        DeclareLaunchArgument(
            "enable_opponent_selector", default_value="true",
            description="Start opponent_selector (/tracked_objects -> "
                        "/other_ship/twist + opponent TF)"),

        Node(
            package="task2_perception",
            executable="task2_cloud_filter_node",
            name="task2_cloud_filter",
            output="screen",
            parameters=[params_file],
            condition=IfCondition(LaunchConfiguration("enable_cloud_filter")),
        ),
        Node(
            package="task2_perception",
            executable="opponent_selector_node",
            name="opponent_selector",
            output="screen",
            parameters=[params_file],
            condition=IfCondition(
                LaunchConfiguration("enable_opponent_selector")),
        ),
    ])
