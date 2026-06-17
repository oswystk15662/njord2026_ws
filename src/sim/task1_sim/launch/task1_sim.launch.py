import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def include_launch(package_name, relative_path, condition, launch_arguments=None):
    package_share = get_package_share_directory(package_name)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(package_share, *relative_path)),
        condition=condition,
        launch_arguments=(launch_arguments or {}).items(),
    )


def generate_launch_description():
    pkg_share = get_package_share_directory("task1_sim")
    config = os.path.join(pkg_share, "config", "task1_params.yaml")
    nav2_params = os.path.join(pkg_share, "config", "task1_nav2_params.yaml")

    use_dynamics_arg = DeclareLaunchArgument("use_dynamics", default_value="true")
    use_nav2_arg = DeclareLaunchArgument("use_nav2", default_value="true")
    use_waypoints_arg = DeclareLaunchArgument("use_waypoints", default_value="true")
    use_validator_arg = DeclareLaunchArgument("use_validator", default_value="true")
    params_arg = DeclareLaunchArgument("params", default_value=config)
    nav2_params_arg = DeclareLaunchArgument("nav2_params", default_value=nav2_params)

    dynamics = include_launch(
        "dutyed_tf_pub_with_disturbance",
        ["launch", "sim_dynamics.launch.py"],
        IfCondition(LaunchConfiguration("use_dynamics")),
    )

    nav2 = include_launch(
        "nav2_bringup",
        ["launch", "navigation_launch.py"],
        IfCondition(LaunchConfiguration("use_nav2")),
        {
            "params_file": LaunchConfiguration("nav2_params"),
            "use_sim_time": "false",
            "autostart": "true",
        },
    )

    waypoints = include_launch(
        "waypoint_publisher",
        ["launch", "waypoint_publisher.launch.py"],
        IfCondition(LaunchConfiguration("use_waypoints")),
        {
            "task_type": "task1",
            "frame_id": "map",
            "publish_rate_hz": "2.0",
        },
    )

    validator = include_launch(
        "operation_validator",
        ["launch", "operation_validator.launch.py"],
        IfCondition(LaunchConfiguration("use_validator")),
    )

    orchestrator = Node(
        package="task1_sim",
        executable="task1_orchestrator",
        name="task1_orchestrator",
        parameters=[LaunchConfiguration("params")],
        output="screen",
    )

    return LaunchDescription([
        use_dynamics_arg,
        use_nav2_arg,
        use_waypoints_arg,
        use_validator_arg,
        params_arg,
        nav2_params_arg,
        dynamics,
        nav2,
        validator,
        waypoints,
        orchestrator,
    ])
