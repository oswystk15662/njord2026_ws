import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # --- 引数の宣言 (デフォルト値はC++コードに合わせつつ、ここで変更可能にする) ---
    
    # 通信モード: 'uart' or 'tcp'
    arg_mode = DeclareLaunchArgument(
        'uart_or_tcp',
        default_value='tcp',
        description='Connection mode: "uart" or "tcp"'
    )

    # シリアル接続設定
    arg_port = DeclareLaunchArgument(
        'gnss_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for GNSS (e.g., /dev/ttyUSB0)'
    )
    arg_baud = DeclareLaunchArgument(
        'gnss_baudrate',
        default_value='115200',
        description='Baudrate for serial connection'
    )

    # TCP接続設定
    arg_ip = DeclareLaunchArgument(
        'tcp_ip',
        default_value='192.168.0.126',
        description='IP address for TCP connection'
    )
    arg_tcp_port = DeclareLaunchArgument(
        'tcp_port',
        default_value='23',
        description='Port number for TCP connection'
    )

    # GNSS設定
    arg_fix_freq = DeclareLaunchArgument(
        'fix_freq',
        default_value='20',
        description='Frequency for NavSatFix publishing (Hz)'
    )
    arg_heading_freq = DeclareLaunchArgument(
        'heading_freq',
        default_value='20',
        description='Frequency for Heading publishing (Hz)'
    )
    arg_rtk = DeclareLaunchArgument(
        'rtk_enable',
        default_value='true',
        description='Enable RTK (NTRIP client)'
    )
    arg_frame_id = DeclareLaunchArgument(
        'heading_frame_id',
        default_value='odom',
        description='Frame ID for the heading message'
    )

    # ログ設定
    arg_log = DeclareLaunchArgument(
        'log_file_name',
        default_value='',
        description='Path to save raw GNSS log (leave empty to disable)'
    )

    # --- ノードの定義 ---
    um982_node = Node(
        package='um982_driver',
        executable='um982_driver_node',
        name='um982_driver',
        output='screen',
        emulate_tty=True,
        parameters=[{
            # C++側のパラメータ名 : LaunchConfiguration(引数名)
            'uart_or_tcp': LaunchConfiguration('uart_or_tcp'),
            'GNSS_SerialPort': LaunchConfiguration('gnss_port'),
            'GNSS_Baudrate': LaunchConfiguration('gnss_baudrate'),
            'tcp_ip': LaunchConfiguration('tcp_ip'),
            'tcp_port': LaunchConfiguration('tcp_port'),
            'FIX_FREQ': LaunchConfiguration('fix_freq'),
            'HEADING_FREQ': LaunchConfiguration('heading_freq'),
            'GNSS_RTK_Enable': LaunchConfiguration('rtk_enable'),
            'Heading_FrameID': LaunchConfiguration('heading_frame_id'),
            'log_file_name': LaunchConfiguration('log_file_name'),
        }]
    )

    return LaunchDescription([
        arg_mode,
        arg_port,
        arg_baud,
        arg_ip,
        arg_tcp_port,
        arg_fix_freq,
        arg_heading_freq,
        arg_rtk,
        arg_frame_id,
        arg_log,
        um982_node
    ])