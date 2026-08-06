"""Replay-time UM982 global-localization pipeline.

Start this launch before playing a bag with ``ros2 bag play <bag> --clock``.
It intentionally starts no hardware drivers: the bag supplies
``/odometry/filtered/local`` and ``/sensor/vehicle_gnss/fix/raw``.  The
navsat transform emits map-frame GNSS odometry and the global EKF publishes
the corresponding ``map -> odom`` transform.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    robot_share = FindPackageShare("robot")

    navsat_transform_node = Node(
        package="robot_localization",
        executable="navsat_transform_node",
        name="navsat_transform_node",
        output="screen",
        arguments=["--ros-args", "--log-level", "ERROR"],
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "world_frame": "map",
                "frequency": 10.0,
                "magnetic_declination_radians": 0.0,
                "yaw_offset": 0.0,
                "zero_altitude": True,
                "broadcast_utm_transform": False,
                "publish_filtered_gps": True,
                "use_odometry_yaw": True,
                "wait_for_datum": False,
            }
        ],
        remappings=[
            ("gps/fix", "/sensor/vehicle_gnss/fix/raw"),
            ("odometry/filtered", "/odometry/filtered/local"),
            ("odometry/gps", "/odometry/gps/um982"),
        ],
    )

    global_ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node_global",
        output="screen",
        parameters=[
            PathJoinSubstitution([robot_share, "config", "ekf_global.yaml"]),
            {"use_sim_time": use_sim_time},
        ],
        remappings=[("odometry/filtered", "/odometry/filtered/global")],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use the rosbag /clock topic.",
            ),
            navsat_transform_node,
            global_ekf_node,
        ]
    )
