"""Real-vessel Task 2 (collision avoidance) entry point.

Composes the existing bringup/perception/planning launch files; nothing is
copy-pasted from them:

    real_bringup.launch.py        sensors, GLIM/EKF localization, (thrusters)
    yolo11s.launch.py             camera buoy detection        [enable_yolo]
    task2_perception.launch.py    cloud filter / opponent selector
                                                               [enable_lidar]
    classical_pipeline.launch.py  ship tracking (/other_ship/twist,
                                  map->opponent_vessel TF)     [enable_ship_tracking]
    planner_real.launch.py        MPPI planner + waypoint source
                                  (asv_trajectory_planner)     [enable_mppi]
    navigation_launch_task2.py    Task2 Nav2 stack with
                                  nav2_params_task2_jazzy.yaml [enable_nav2]
    diagnostics.launch.py         topic heartbeat monitors     [enable_debug_topics]
                                  (real_bringup does NOT launch diagnostics,
                                  so there is no double launch)

SAFETY / DRY-RUN GATING
=======================
Defaults are SAFE: dry_run:=true and send_thruster_commands:=false.

real_bringup's ``enable_thruster`` (which gates BOTH thruster_driver AND the
micon serial_writer) receives true ONLY when ALL THREE of the following hold:

    enable_thrusters == 'true'
    dry_run == 'false'
    send_thruster_commands == 'true'

In a dry run, Nav2 + MPPI run normally and /cmd_vel_nav (output of the
Nav2 velocity_smoother) is fully observable, but thruster_driver and
serial_writer are never started, so no /thruster_command is produced and no
bytes reach the ESP32. That is the safety boundary: to actually actuate you
must explicitly pass dry_run:=false AND send_thruster_commands:=true (with
enable_thrusters left true).

WARNING - SIM-ONLY NODES MUST NOT BE LAUNCHED HERE
==================================================
The following six executables are simulation-only and must never appear in
this real-vessel launch: opponent_twist_from_tf_node,
task2_gps_waypoint_publisher, opponent_vessel_node,
ideal_lidar_pointcloud_node, dutyed_tf_pub_with_disturbance,
sim_thruster_command_adapter. On the real vessel /other_ship/twist and the
map->opponent_vessel TF come from the perception pipeline, and the waypoints
come from task2_waypoint_pose_publisher (started by planner_real.launch.py).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.substitutions import FindPackageShare


def include_launch(package_name, path_parts, condition=None, launch_arguments=None):
    # Scoped include, mirroring real_bringup.launch.py: keeps generic argument
    # names local to each included launch file.
    return GroupAction(
        scoped=True,
        condition=condition,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare(package_name)] + path_parts)
                ),
                launch_arguments=(launch_arguments or {}).items(),
            )
        ],
    )


def generate_launch_description():
    enable_camera = LaunchConfiguration("enable_camera")
    enable_yolo = LaunchConfiguration("enable_yolo")
    enable_lidar = LaunchConfiguration("enable_lidar")
    enable_buoy_perception = LaunchConfiguration("enable_buoy_perception")
    enable_ship_tracking = LaunchConfiguration("enable_ship_tracking")
    enable_mppi = LaunchConfiguration("enable_mppi")
    enable_nav2 = LaunchConfiguration("enable_nav2")
    enable_thrusters = LaunchConfiguration("enable_thrusters")
    enable_debug_topics = LaunchConfiguration("enable_debug_topics")

    lidar_model = LaunchConfiguration("lidar_model")
    yolo_backend = LaunchConfiguration("yolo_backend")
    yolo_model_path = LaunchConfiguration("yolo_model_path")
    use_tensorrt = LaunchConfiguration("use_tensorrt")

    dry_run = LaunchConfiguration("dry_run")
    send_thruster_commands = LaunchConfiguration("send_thruster_commands")

    # ------------------------------------------------------------------
    # SAFETY BOUNDARY: thrusters are enabled only when all three gates
    # agree. Anything else keeps thruster_driver + serial_writer down.
    # ------------------------------------------------------------------
    thruster_allowed = PythonExpression(
        [
            "'",
            enable_thrusters,
            "' == 'true' and '",
            dry_run,
            "' == 'false' and '",
            send_thruster_commands,
            "' == 'true'",
        ]
    )

    # use_tensorrt=='true' forces the tensorrt backend; otherwise the
    # yolo_backend argument value is used as-is.
    yolo_backend_resolved = PythonExpression(
        [
            "'tensorrt' if '",
            use_tensorrt,
            "' == 'true' else '",
            yolo_backend,
            "'",
        ]
    )

    real_bringup = include_launch(
        "robot",
        ["launch", "real_bringup.launch.py"],
        launch_arguments={
            # Nav2 from real_bringup is ALWAYS disabled here; the Task2 Nav2
            # stack (navigation_launch_task2.py + nav2_params_task2_jazzy.yaml) is
            # launched below, gated by enable_nav2.
            "enable_nav2": "false",
            "enable_mid360": enable_lidar,
            "lidar_model": lidar_model,
            "enable_zed2i": enable_camera,
            "enable_back_cam": "false",
            "enable_localization": "true",
            "enable_pcl_buoy_detection": enable_buoy_perception,
            "enable_thruster": thruster_allowed,
        },
    )

    yolo_launch = include_launch(
        "yolo",
        ["launch", "yolo11s.launch.py"],
        condition=IfCondition(enable_yolo),
        launch_arguments={
            "yolo_model_path": yolo_model_path,
            "backend": yolo_backend_resolved,
        },
    )

    task2_perception_launch = include_launch(
        "task2_perception",
        ["launch", "task2_perception.launch.py"],
        condition=IfCondition(enable_lidar),
        launch_arguments={
            "enable_cloud_filter": "true",
            "enable_opponent_selector": enable_ship_tracking,
            # Self-vessel Marker is a Foxglove/RViz aid for bag debugging;
            # do not publish it in the real-vessel runtime.
            "publish_self_marker": "false",
        },
    )

    ship_tracking_launch = include_launch(
        "ship_perception_bringup",
        ["launch", "classical_pipeline.launch.py"],
        condition=IfCondition(enable_ship_tracking),
        launch_arguments={
            "lidar_topic": "/task2/points_filtered",
            "ego_odom_topic": "/odometry/filtered/local",
        },
    )

    # Waypoint source (task2_waypoint_pose_publisher) and the MPPI planner
    # chain are both started by planner_real.launch.py, gated by enable_mppi.
    mppi_launch = include_launch(
        "asv_trajectory_planner",
        ["launch", "planner_real.launch.py"],
        condition=IfCondition(enable_mppi),
    )

    nav2_task2_launch = include_launch(
        "robot",
        ["launch", "navigation_launch_task2.py"],
        condition=IfCondition(enable_nav2),
        launch_arguments={
            "params_file": PathJoinSubstitution(
                [
                    FindPackageShare("robot"),
                    "config",
                    "nav2_params_task2_jazzy.yaml",
                ]
            ),
            "use_sim_time": "false",
            "autostart": "true",
            "auto_cmd_vel_topic": "/cmd_vel_nav",
        },
    )

    # real_bringup.launch.py does not include diagnostics.launch.py, so this
    # is the only diagnostics launch (no double launch). Extra debug topics
    # are gated by enable_debug_topics.
    diagnostics_launch = include_launch(
        "robot",
        ["launch", "diagnostics.launch.py"],
        condition=IfCondition(enable_debug_topics),
        launch_arguments={
            "profile": "localization",
        },
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("enable_camera", default_value="true"),
            DeclareLaunchArgument("enable_yolo", default_value="true"),
            DeclareLaunchArgument("enable_lidar", default_value="true"),
            DeclareLaunchArgument(
                "enable_buoy_perception", default_value="true",
                description="Start YOLO-ROI + LiDAR buoy position detection for MPPI."),
            DeclareLaunchArgument("enable_ship_tracking", default_value="true"),
            DeclareLaunchArgument("enable_mppi", default_value="true"),
            DeclareLaunchArgument("enable_nav2", default_value="true"),
            DeclareLaunchArgument("enable_thrusters", default_value="true"),
            DeclareLaunchArgument("enable_debug_topics", default_value="false"),
            DeclareLaunchArgument(
                "lidar_model",
                default_value="mid360s",
                choices=["mid360", "mid360s"],
            ),
            DeclareLaunchArgument(
                "yolo_backend",
                default_value="pytorch",
                description="YOLO backend when use_tensorrt is false; "
                "yolo11_node accepts 'pytorch' (.pt) or 'tensorrt' (.engine)",
            ),
            DeclareLaunchArgument(
                "yolo_model_path",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("yolo"), "config", "best.pt"]
                ),
                description="Path to YOLO weights; override with the Jetson "
                "TensorRT engine / yolo11s weights as appropriate",
            ),
            DeclareLaunchArgument(
                "use_tensorrt",
                default_value="false",
                description="true forces yolo backend to 'tensorrt'",
            ),
            DeclareLaunchArgument(
                "use_int8",
                default_value="false",
                description="Reserved passthrough for INT8 quantized "
                "inference; not consumed yet",
            ),
            # SAFE defaults: dry run, no thruster commands.
            DeclareLaunchArgument(
                "dry_run",
                default_value="true",
                description="true = never start thruster_driver/serial_writer",
            ),
            DeclareLaunchArgument(
                "send_thruster_commands",
                default_value="false",
                description="Must be explicitly set true (with dry_run:=false) "
                "to actuate thrusters",
            ),
            real_bringup,
            yolo_launch,
            task2_perception_launch,
            ship_tracking_launch,
            mppi_launch,
            nav2_task2_launch,
            diagnostics_launch,
        ]
    )
