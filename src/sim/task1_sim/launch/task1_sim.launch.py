import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("task1_sim")
    dutyed_share = get_package_share_directory("dutyed_tf_pub_with_disturbance")

    config = os.path.join(pkg_share, "config", "task1_params.yaml")
    dutyed_launch = os.path.join(dutyed_share, "launch", "sim_dynamics.launch.py")

    use_dynamics = DeclareLaunchArgument("use_dynamics", default_value="true")
    params_arg = DeclareLaunchArgument("params", default_value=config)

    dynamics = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(dutyed_launch),
        condition=IfCondition(LaunchConfiguration("use_dynamics")),
    )

    node = Node(
        package="task1_sim",
        executable="task1_orchestrator",
        name="task1_orchestrator",
        parameters=[LaunchConfiguration("params")],
        output="screen",
    )

    return LaunchDescription([use_dynamics, params_arg, dynamics, node])
