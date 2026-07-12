from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def include_launch(package_name, path_parts, condition, launch_arguments=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package_name)] + path_parts)
        ),
        condition=condition,
        launch_arguments=(launch_arguments or {}).items(),
    )


def generate_launch_description():
    enable_mid360 = LaunchConfiguration("enable_mid360")
    enable_zed2i = LaunchConfiguration("enable_zed2i")
    enable_back_cam = LaunchConfiguration("enable_back_cam")
    enable_um982 = LaunchConfiguration("enable_um982")
    enable_imu = LaunchConfiguration("enable_imu")
    enable_localization = LaunchConfiguration("enable_localization")
    enable_thruster = LaunchConfiguration("enable_thruster")
    enable_nav2 = LaunchConfiguration("enable_nav2")

    serial_port = LaunchConfiguration("serial_port")
    baud = LaunchConfiguration("baud")
    device = LaunchConfiguration("device")

    mid360_launch = include_launch(
        "livox_ros_driver2",
        ["launch_ROS2", "msg_MID360_launch.py"],
        IfCondition(enable_mid360),
    )

    zed2i_launch = include_launch(
        "zed2i_driver",
        ["launch", "zed2i.launch.py"],
        IfCondition(enable_zed2i),
        {"mode": "sdk"},
    )

    back_cam_launch = include_launch(
        "robot",
        ["launch", "back_cam.launch.py"],
        IfCondition(enable_back_cam),
        {"video_device": device},
    )

    um982_launch = include_launch(
        "um982_driver",
        ["launch", "um982.launch.py"],
        IfCondition(enable_um982),
    )

    imu_launch = include_launch(
        "wit_node",
        ["launch", "wit.launch.py"],
        IfCondition(enable_imu),
    )

    localization_launch = include_launch(
        "robot",
        ["launch", "localization.launch.py"],
        IfCondition(enable_localization),
    )

    thruster_launch = include_launch(
        "thruster_driver",
        ["launch", "thruster_driver.launch.py"],
        IfCondition(enable_thruster),
        {"stop_on_feedback_timeout": "true"},
    )

    # TODO: micon_driver_fd currently has no standalone launch file; keep the
    # executable and node names here because manual_control.launch.py depends on them.
    serial_writer = Node(
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
        IfCondition(enable_thruster),
    )

    nav2_launch = include_launch(
        "robot",
        ["launch", "nav2.launch.py"],
        IfCondition(enable_nav2),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("enable_mid360", default_value="true"),
            DeclareLaunchArgument("enable_zed2i", default_value="true"),
            DeclareLaunchArgument("enable_back_cam", default_value="true"),
            DeclareLaunchArgument("enable_um982", default_value="true"),
            DeclareLaunchArgument("enable_imu", default_value="true"),
            DeclareLaunchArgument("enable_localization", default_value="true"),
            DeclareLaunchArgument("enable_thruster", default_value="true"),
            DeclareLaunchArgument("enable_nav2", default_value="false"),
            DeclareLaunchArgument("serial_port", default_value="/dev/ttyUSB1"),
            DeclareLaunchArgument("baud", default_value="115200"),
            DeclareLaunchArgument(
                "device",
                default_value="/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._Adesso_CyberTrack_H7_SN0001-video-index0",
            ),
            mid360_launch,
            zed2i_launch,
            back_cam_launch,
            um982_launch,
            imu_launch,
            localization_launch,
            thruster_launch,
            serial_writer,
            bms_launch,
            nav2_launch,
        ]
    )
