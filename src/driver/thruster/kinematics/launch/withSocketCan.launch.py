import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('kinematics')
    
    # デフォルトのConfigパス
    default_config_path = os.path.join(pkg_share, 'config', 'config.yaml')

    # Launch引数: 外部から設定ファイルパスを指定可能にする
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_path,
        description='Path to the kinematics config file'
    )

    # ノード定義
    kinematics_node = Node(
        package='kinematics',
        executable='kinematics_node',
        name='kinematics_node',
        output='screen',
        # Launch引数で指定されたパスのファイルをパラメータとして読み込む
        parameters=[LaunchConfiguration('config_file')]
    )

    return LaunchDescription([
        config_file_arg,
        kinematics_node
    ])