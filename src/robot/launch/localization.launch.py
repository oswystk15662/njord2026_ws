from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # um982_driverのlaunchファイルのパスを取得
    um982_launch_file = os.path.join(
        FindPackageShare('um982_driver').find('um982_driver'),
        'launch',
        'um982_driver.launch.py'
    )

    # Includeしてパラメータを上書き
    um982_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(um982_launch_file),
        launch_arguments={
            'uart_or_tcp': 'tcp',           # モード指定
            'tcp_ip': '192.168.1.50',       # IPアドレス変更
            'heading_frame_id': 'base_link' # フレームID変更
        }.items()
    )

    um982_static_tf_pub_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='um982_static_tf_pub',
        output='screen',
        parameters=[{
            'x': 0.0,
            'y': 0.0,
            'z': 0.0,
            'roll': 0.0,
            'pitch': 0.0,
            'yaw': 0.0,
            'frame_id': 'base_link',
            'child_frame_id': 'um982_link'
        }]
    )

    drogger_launch = os.path.join(
        FindPackageShare('drogger_wired_flex').find('drogger_wired_flex'),
        'launch',
        'driver.launch.py'
    )
    drogger_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(drogger_launch),
    )

    mid360_launch = os.path.join(
        FindPackageShare('livox_ros_driver2').find('livox_ros_driver2'),
        'launch',
        'rviz_MID360.launch.py'
    )
    mid360_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mid360_launch),
    )

    glim_config_path = os.path.join(
        FindPackageShare('robot').find('robot'),
        'config',
        'glim_config'
    )

    glim_node = Node(
        package='glim_ros',
        executable='glim_node',
        name='glim_node',
        output='screen',
        parameters=[{
            'config_path': glim_config_path,
            'use_sim_time': False
        }]
    )

    local_ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_local',
        output='screen',
        parameters=[os.path.join(
            FindPackageShare('robot').find('robot'),
            'config',
            'ekf_local.yaml'
        )],
        remappings=[('odometry/filtered', 'odometry/filtered/local')]
    )

    global_ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_global',
        output='screen',
        parameters=[os.path.join(
            FindPackageShare('robot').find('robot'),
            'config',
            'ekf_global.yaml'
        )],
        remappings=[('odometry/filtered', 'odometry/filtered/global')]
    )

    navsat_transform_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[{
            'frequency': 10.0,
            'magnetic_declination_radians': 0.0,
            'yaw_offset': 0.0,
            'zero_altitude': True,
            'broadcast_utm_transform': True,
            'publish_filtered_gps': True,
            'use_odometry_yaw': False,
            'wait_for_datum': False,
        }],
        remappings=[
            ('imu/data', '/wit/imu'),
            ('gps/fix', '/gps/fix'),
            ('odometry/filtered', 'odometry/filtered/local')
        ]
    )

    # robot_state_publisher: URDF経由でbase_link -> sensors のTFを配信
    robot_description_file = os.path.join(
        get_package_share_directory('robot'),
        'urdf',
        'robot.urdf.xacro'
    )
    
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': open(robot_description_file, 'r').read(),
            'use_sim_time': False
        }]
    )

    return LaunchDescription([
        # TF publisher: base_link -> sensors (URDF経由)
        robot_state_pub_node,
        # Localization nodes
        um982_static_tf_pub_node,
        drogger_driver,
        mid360_driver,
        glim_node,
        local_ekf_node,
        global_ekf_node,
        # GPS integration
        navsat_transform_node,
    ])