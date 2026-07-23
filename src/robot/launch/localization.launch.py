import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_share = FindPackageShare("robot")
    robot_share_path = get_package_share_directory("robot")

    enable_diagnostics_arg = DeclareLaunchArgument(
        "enable_diagnostics",
        default_value="true",
        # description="Launch generic topic heartbeat diagnostics for localization topics",
    )

    robot_description = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            PathJoinSubstitution([robot_share, "urdf", "robot.urdf.xacro"]),
        ]
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": ParameterValue(
                    robot_description,
                    value_type=str,
                )
            }
        ],
    )

    # CPUリソースが枯渇していた場合、pointcloud2に変換するのがボトルネックになるので、
    # 取れた点群を直に転送できるように、LiDAR Driver側でComposableNode起動？してください
    glim_node = Node(
        package="glim_ros",
        executable="glim_ros_node",
        name="glim_ros_node",
        output="screen",
        parameters=[
            {
                "config_path": PathJoinSubstitution(
                    [FindPackageShare("robot"), "config", "glim_config"]
                ),
                "use_sim_time": False,
            }
        ],
        remappings=[("/glim_ros_node/odom", "/odom")],
    )

    spatial_navsat_transform_node = Node(
        package="robot_localization",
        executable="navsat_transform_node",
        name="spatial_navsat_transform_node",
        output="screen",
        parameters=[
            {
                "frequency": 10.0,
                "magnetic_declination_radians": 0.0,
                "yaw_offset": 0.0,
                "zero_altitude": True,
                "broadcast_utm_transform": False,
                "publish_filtered_gps": False,
                "use_odometry_yaw": False,
                "wait_for_datum": False,
            }
        ],
        remappings=[
            ("imu", "/adnav_driver/imu"),
            ("gps/fix", "/adnav_driver/nav_sat_fix"),
            ("odometry/filtered", "odometry/filtered/local"),
            ("odometry/gps", "/odometry/gps/spatial"),
        ],
    )

    # The UM982 driver is launched separately because its transport settings
    # are deployment-specific.
    um982_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="um982_static_tf_pub",
        output="screen",
        arguments=[
            "--x", "0.0", "--y", "0.0", "--z", "0.0",
            "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0",
            "--frame-id", "base_link", "--child-frame-id", "um982_link",
        ],
    )

    local_ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node_local",
        output="screen",
        parameters=[
            PathJoinSubstitution([robot_share, "config", "ekf_local.yaml"])
        ],
        remappings=[("odometry/filtered", "odometry/filtered/local")],
    )

    global_ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node_global",
        output="screen",
        parameters=[
            PathJoinSubstitution([robot_share, "config", "ekf_global.yaml"])
        ],
        remappings=[("odometry/filtered", "odometry/filtered/global")],
    )

    navsat_transform_node = Node(
        package="robot_localization",
        executable="navsat_transform_node",
        name="navsat_transform_node",
        output="screen",
        # Keep the noisy datum/initialization messages local to this node.
        # Other nodes keep the launch-wide default log level.
        arguments=["--ros-args", "--log-level", "ERROR"],
        parameters=[
            {
                "frequency": 10.0,
                "magnetic_declination_radians": 0.0,
                "yaw_offset": 0.0,
                "zero_altitude": True,
                "broadcast_utm_transform": True,
                "publish_filtered_gps": True,
                "use_odometry_yaw": True,
                "wait_for_datum": False,
            }
        ],
        remappings=[
            ("gps/fix", "/sensor/vehicle_gnss/fix/raw"),
            ("odometry/filtered", "odometry/filtered/local"),
        ],
    )

    diagnostics_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_share_path, "launch", "diagnostics.launch.py")
        ),
        launch_arguments={"profile": "localization"}.items(),
        condition=IfCondition(LaunchConfiguration("enable_diagnostics")),
    )

    return LaunchDescription(
        [
            enable_diagnostics_arg,
            robot_state_publisher,
            # glim_node,
            # spatial_navsat_transform_node,
            um982_static_tf,
            local_ekf_node,
            global_ekf_node,
            navsat_transform_node,
            diagnostics_launch,
        ]
    )
