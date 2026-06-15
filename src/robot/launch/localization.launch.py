from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_share = FindPackageShare("robot")

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
        parameters=[{"robot_description": robot_description}],
    )

    # The UM982 driver is launched separately because its transport settings
    # are deployment-specific.
    um982_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="um982_static_tf_pub",
        output="screen",
        arguments=[
            "--x",
            "0.0",
            "--y",
            "0.0",
            "--z",
            "0.0",
            "--roll",
            "0.0",
            "--pitch",
            "0.0",
            "--yaw",
            "0.0",
            "--frame-id",
            "base_link",
            "--child-frame-id",
            "um982_link",
        ],
    )

    glim_node = Node(
        package="glim_ros",
        executable="glim_node",
        name="glim_node",
        output="screen",
        parameters=[
            {
                "config_path": PathJoinSubstitution(
                    [robot_share, "config", "glim_config"]
                ),
                "use_sim_time": False,
            }
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
        parameters=[
            {
                "frequency": 10.0,
                "magnetic_declination_radians": 0.0,
                "yaw_offset": 0.0,
                "zero_altitude": True,
                "broadcast_utm_transform": True,
                "publish_filtered_gps": True,
                "use_odometry_yaw": False,
                "wait_for_datum": False,
            }
        ],
        remappings=[
            ("imu", "/wit/imu"),
            ("gps/fix", "/gps/fix"),
            ("odometry/filtered", "odometry/filtered/local"),
        ],
    )

    return LaunchDescription(
        [
            robot_state_publisher,
            um982_static_tf,
            glim_node,
            local_ekf_node,
            global_ekf_node,
            navsat_transform_node,
        ]
    )
