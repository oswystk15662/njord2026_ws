import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, TimerAction
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
    driver_delay_arg = DeclareLaunchArgument(
        "driver_delay",
        default_value="0.0",
        description="Delay before launching Task1 dynamics/orchestrator/validator layer",
    )
    nav2_delay_arg = DeclareLaunchArgument(
        "nav2_delay",
        default_value="5.0",
        description="Delay before launching Nav2",
    )
    goal_delay_arg = DeclareLaunchArgument(
        "goal_delay",
        default_value="8.0",
        description="Delay before launching waypoint_publisher",
    )
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

    sensor_layer_timer = TimerAction(
        period=LaunchConfiguration("driver_delay"),
        actions=[
            dynamics,
            validator,
            orchestrator,
        ],
    )

    nav2_layer_timer = TimerAction(
        period=LaunchConfiguration("nav2_delay"),
        actions=[nav2],
    )

    goal_layer_timer = TimerAction(
        period=LaunchConfiguration("goal_delay"),
        actions=[waypoints],
    )

    startup_message = LogInfo(msg="========== Task1 Sim Bringup Started ==========")

    return LaunchDescription([
        startup_message,
        use_dynamics_arg,
        use_nav2_arg,
        use_waypoints_arg,
        use_validator_arg,
        driver_delay_arg,
        nav2_delay_arg,
        goal_delay_arg,
        params_arg,
        nav2_params_arg,
        sensor_layer_timer,
        nav2_layer_timer,
        goal_layer_timer,
    ])
