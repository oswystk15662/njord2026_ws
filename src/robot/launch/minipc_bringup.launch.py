"""miniPC-side bringup: everything except MID360S/GLIM/ZED 2i.

x86_64 Ubuntu 22.04 / ROS 2 Humble, no CUDA / no ZED SDK. Owns every USB
serial device (micon, UM982, Drogger, WIT IMU, joy pad), localization
(robot_state_publisher, TF, both EKFs, navsat_transform via
localization.launch.py), thrusters, and the operator-facing links
(foxglove bridge). GLIM/MID360S/ZED 2i live on the Jetson side
(jetson_bringup.launch.py) and must not be duplicated here.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def include_launch(package_name, path_parts, condition, launch_arguments=None):
    # Included launch files commonly declare generic argument names (e.g.
    # serial_port, config_file). Keep those configurations local to the
    # include so one driver's launch args cannot leak into another's.
    return GroupAction(
        scoped=True,
        condition=condition,
        actions=[
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare(package_name)] + path_parts)
                ),
                launch_arguments=(launch_arguments or {}).items(),
            )
        ],
    )


def generate_launch_description():
    default_thruster_config = os.path.join(
        get_package_share_directory("thruster_driver"), "config", "config.yaml"
    )
    default_thruster_urdf = os.path.join(
        get_package_share_directory("robot"), "urdf", "robot.urdf_modified.urdf"
    )

    serial_port = LaunchConfiguration("serial_port")
    baud = LaunchConfiguration("baud")
    um982_port = LaunchConfiguration("um982_port")
    um982_protocol = LaunchConfiguration("um982_protocol")
    enable_um982_rtk = LaunchConfiguration("enable_um982_rtk")
    um982_feedback_mode = LaunchConfiguration("um982_feedback_mode")
    drogger_rzs_port = LaunchConfiguration("drogger_rzs_port")
    drogger_rzs_baud = LaunchConfiguration("drogger_rzs_baud")
    imu_port = LaunchConfiguration("imu_port")
    imu_baud = LaunchConfiguration("imu_baud")
    enable_um982 = LaunchConfiguration("enable_um982")
    enable_drogger_rzs = LaunchConfiguration("enable_drogger_rzs")
    enable_imu = LaunchConfiguration("enable_imu")
    enable_localization = LaunchConfiguration("enable_localization")
    enable_thruster = LaunchConfiguration("enable_thruster")
    enable_joy = LaunchConfiguration("enable_joy")
    enable_alert_lamp = LaunchConfiguration("enable_alert_lamp")
    enable_bms = LaunchConfiguration("enable_bms")
    enable_buoy_costmap = LaunchConfiguration("enable_buoy_costmap")
    enable_foxglove = LaunchConfiguration("enable_foxglove")
    enable_back_cam = LaunchConfiguration("enable_back_cam")
    enable_nav2 = LaunchConfiguration("enable_nav2")
    enable_diagnostics = LaunchConfiguration("enable_diagnostics")
    thruster_config_file = LaunchConfiguration("thruster_config_file")
    thruster_robot_description_file = LaunchConfiguration("thruster_robot_description_file")

    # robot_state_publisher and base_link->um982_link static TF are started by
    # localization.launch.py. Do not start them again here.
    localization_launch = include_launch(
        "robot",
        ["launch", "localization.launch.py"],
        IfCondition(enable_localization),
        {
            "enable_glim": "false",
            "enable_local_ekf": "true",
            "enable_global_ekf": "true",
            "enable_navsat_transform": "true",
            "enable_diagnostics": enable_diagnostics,
        },
    )

    um982_launch = include_launch(
        "um982_driver",
        ["launch", "um982.launch.py"],
        IfCondition(enable_um982),
        {
            "uart_or_tcp": um982_protocol,
            "gnss_port": um982_port,
            "rtk_enable": enable_um982_rtk,
        },
    )

    um982_feedback_launch = include_launch(
        "um982_feedback_filter",
        ["launch", "um982_feedback.launch.py"],
        IfCondition(enable_um982),
        {
            "feedback_mode": um982_feedback_mode,
            "raw_topic": "/odometry/feedback",
            "output_topic": "/odometry/filtered/local",
        },
    )

    drogger_launch = include_launch(
        "drogger_wired_flex",
        ["launch", "driver.launch.py"],
        IfCondition(enable_drogger_rzs),
        {
            "serial_port": drogger_rzs_port,
            "serial_baudrate": drogger_rzs_baud,
        },
    )

    imu_node = Node(
        package="witmotion_imu_driver",
        executable="witmotion_standard_protocol_driver_node",
        name="witmotion_standard_protocol_driver",
        output="screen",
        parameters=[
            {
                "device_port_name": imu_port,
                "serial_baud_rate": imu_baud,
                "frame_id": "imu_link",
                "publish_frequency": 10.0,
            }
        ],
        remappings=[("~/imu/raw", "/wit/imu")],
        condition=IfCondition(enable_imu),
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(enable_joy),
    )

    joy_converter = Node(
        package="simple_manual",
        executable="joy_converter_node",
        name="joy_converter",
        output="screen",
        parameters=[
            PathJoinSubstitution(
                [FindPackageShare("simple_manual"), "config", "joy_converter.yaml"]
            )
        ],
    )

    command_arbiter = Node(
        package="simple_manual",
        executable="command_arbiter_node",
        name="command_arbiter",
        output="screen",
        parameters=[{"initial_mode": "manual"}],
    )

    thruster_launch = include_launch(
        "thruster_driver",
        ["launch", "thruster_driver.launch.py"],
        IfCondition(enable_thruster),
        {
            "config_file": thruster_config_file,
            "robot_description_file": thruster_robot_description_file,
        },
    )

    micon_driver = Node(
        package="micon_driver_fd",
        executable="serial_writer",
        name="serial_writer",
        output="screen",
        parameters=[
            {
                "serial_port": serial_port,
                "baud": baud,
                "command_topic": "/thruster_command",
                "use_sim_time": False,
            }
        ],
        condition=IfCondition(enable_thruster),
    )

    bms_launch = include_launch(
        "bms",
        ["launch", "bms.launch.py"],
        IfCondition(enable_bms),
    )

    alert_lamp_launch = include_launch(
        "alert_lamp",
        ["launch", "alert_lamp.launch.py"],
        IfCondition(enable_alert_lamp),
    )

    buoy_obstacle_launch = include_launch(
        "buoy_obstacle_publisher",
        ["launch", "buoy_obstacle_publisher.launch.py"],
        IfCondition(enable_buoy_costmap),
    )

    foxglove_bridge_launch = include_launch(
        "foxglove_bridge",
        ["launch", "foxglove_bridge_launch.xml"],
        IfCondition(enable_foxglove),
    )

    back_cam_launch = include_launch(
        "robot",
        ["launch", "back_cam.launch.py"],
        IfCondition(enable_back_cam),
    )

    nav2_launch = include_launch(
        "robot",
        ["launch", "nav2.launch.py"],
        IfCondition(enable_nav2),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "serial_port",
                default_value=(
                    "/dev/serial/by-id/"
                    "usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_"
                    "c82421728a9aef118808b29061ce3355-if00-port0"
                ),
            ),
            DeclareLaunchArgument("baud", default_value="115200"),
            DeclareLaunchArgument(
                "um982_port",
                default_value="/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0",
            ),
            DeclareLaunchArgument("um982_protocol", default_value="uart"),
            DeclareLaunchArgument("enable_um982_rtk", default_value="false"),
            DeclareLaunchArgument(
                "um982_feedback_mode", default_value="ekf", choices=["window", "ekf"]
            ),
            DeclareLaunchArgument(
                "drogger_rzs_port",
                default_value=(
                    "/dev/serial/by-id/"
                    "usb-Prolific_Technology_Inc._USB-Serial_Controller_"
                    "ACCQg146B12-if00-port0"
                ),
            ),
            DeclareLaunchArgument("drogger_rzs_baud", default_value="115200"),
            DeclareLaunchArgument("imu_port", default_value="/dev/ttyUSB0"),
            DeclareLaunchArgument("imu_baud", default_value="9600"),
            DeclareLaunchArgument("enable_um982", default_value="true"),
            DeclareLaunchArgument("enable_drogger_rzs", default_value="true"),
            DeclareLaunchArgument("enable_imu", default_value="false"),
            DeclareLaunchArgument("enable_localization", default_value="true"),
            DeclareLaunchArgument("enable_thruster", default_value="true"),
            DeclareLaunchArgument("enable_joy", default_value="true"),
            DeclareLaunchArgument("enable_alert_lamp", default_value="true"),
            DeclareLaunchArgument("enable_bms", default_value="true"),
            DeclareLaunchArgument("enable_buoy_costmap", default_value="true"),
            DeclareLaunchArgument("enable_foxglove", default_value="true"),
            DeclareLaunchArgument(
                "enable_back_cam",
                default_value="true",
                description="Start the miniPC-connected rear USB camera.",
            ),
            DeclareLaunchArgument(
                "enable_nav2",
                default_value="false",
                description="task1/2/3 launch files start Nav2 themselves with a "
                "task-specific params file; leave this false in that case.",
            ),
            DeclareLaunchArgument("enable_diagnostics", default_value="true"),
            DeclareLaunchArgument("thruster_config_file", default_value=default_thruster_config),
            DeclareLaunchArgument(
                "thruster_robot_description_file", default_value=default_thruster_urdf
            ),
            localization_launch,
            um982_launch,
            um982_feedback_launch,
            # drogger_launch,
            # imu_node,
            # joy_node,
            joy_converter,
            command_arbiter,
            thruster_launch,
            micon_driver,
            bms_launch,
            alert_lamp_launch,
            buoy_obstacle_launch,
            # foxglove_bridge_launch,
            back_cam_launch,
            nav2_launch,
        ]
    )
