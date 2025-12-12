from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

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

    local_ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_local',
        output='screen',
        parameters=[os.path.join(
            FindPackageShare('robot').find('robot'),
            'config',
            'ekf.yaml'
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
            'ekf.yaml'
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
            'magnetic_declination_radians': 0.0, # 地域の磁気偏角を設定
            'yaw_offset': 0.0,                   # IMUが真東を向いて0なら0。真北なら1.57(pi/2)
            'zero_altitude': True,
            'broadcast_utm_transform': True,
            'publish_filtered_gps': True,
            'use_odometry_yaw': False,
            'wait_for_datum': False,
        }],
        remappings=[
            ('imu/data', '/wit/imu'),          # 方位基準に使うIMU (またはDual AntennaのHeading)
            ('gps/fix', '/gps/fix'),           # GNSSドライバのトピック
            ('odometry/filtered', 'odometry/filtered/local') # 初期位置計算用にLocal EKFの結果が必要
        ]
    )

    static_transform_publisher_launch_file = os.path.join(
        FindPackageShare('robot').find('robot'),
        'launch',
        'static_transform_publisher.launch.py'
    )
    static_transform_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            static_transform_publisher_launch_file
        ),
        launch_arguments={
            'x': '0.0',
            'y': '0.0',
            'z': '0.0',
            'roll': '0.0',
            'pitch': '0.0',
            'yaw': '0.0',
            'frame_id': 'base_link',
            'child_frame_id': 'um982_link'
        }.items()
    )

    return LaunchDescription([
        um982_driver
    ])