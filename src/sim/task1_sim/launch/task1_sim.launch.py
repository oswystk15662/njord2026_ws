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

    robot_share = get_package_share_directory("robot")
    nav2_launch = os.path.join(robot_share, "launch", "nav2_mintest.launch.py")
    use_nav2 = DeclareLaunchArgument("use_nav2", default_value="true")
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch),
        condition=IfCondition(LaunchConfiguration("use_nav2")),
    )

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        output="screen",
    )

    map_yaml_file = os.path.join(pkg_share, "maps", "task1_map.yaml")

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_yaml_file}]
    )

    lifecycle_manager_map = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{'use_sim_time': False},
                    {'autostart': True},
                    {'node_names': ['map_server']}]
    )

    return LaunchDescription([use_dynamics, params_arg, use_nav2, dynamics, nav2, static_tf, map_server, lifecycle_manager_map, node])
