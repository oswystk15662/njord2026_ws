from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def include_launch(package_name, path_parts, condition, launch_arguments=None):
    # Included launch files commonly declare generic argument names such as
    # ``serial_port``. Keep those configurations local to the include so a
    # GNSS launch cannot overwrite the Micon serial port (or vice versa).
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
    mid360_launch = include_launch(
        "robot",
        ["launch", "lidar.launch.py"],
        True,
        {
            "lidar_model": "mid360s",
            "enable_buoy_detection": True,
        },
    )

    zed2i_launch = include_launch(
        "zed2i_driver",
        ["launch", "zed2i.launch.py"],
        True,
        {
            "mode": "sdk",
            "camera_resolution": "HD720",
            "framerate": 15,
            "enable_ground_video": True,
            "ground_video_host": "",
            "ground_video_port": "5600",
            "ground_video_fps": "5.0",
        },
    )

    back_cam_launch = include_launch(
        "robot",
        ["launch", "back_cam.launch.py"],
        True,
        {"video_device": "/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._Adesso_CyberTrack_H7_SN0001-video-index0"},
    )

    um982_launch = include_launch(
        "um982_driver",
        ["launch", "um982.launch.py"],
        True,
        {
            "uart_or_tcp": "uart",
            "gnss_port": "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0",
            "rtk_enable": True,
        },
    )

    drogger_rzs_launch = include_launch(
        "drogger_wired_flex",
        ["launch", "driver.launch.py"],
        True,
        {
            "serial_port": "/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller_ACCQg146B12-if00-port0",
            "serial_baudrate": 115200,
            "fix_topic": "/gnss/fix",
        },
    )

    imu_node = Node(
        package="witmotion_imu_driver",
        executable="witmotion_standard_protocol_driver_node",
        name="witmotion_standard_protocol_driver",
        output="screen",
        parameters=[
            {
                "device_port_name": "/dev/ttyUSB0",
                "serial_baud_rate": 115200,
                "frame_id": "wit_imu_link",
                "publish_frequency": 10.0,
            }
        ],
        remappings=[("~/imu/raw", "/wit/imu")],
        condition=True,
    )

    return LaunchDescription(
        [
            mid360_launch,
            zed2i_launch,
            back_cam_launch,
            um982_launch,
            drogger_rzs_launch,
            imu_node,
        ]
    )
