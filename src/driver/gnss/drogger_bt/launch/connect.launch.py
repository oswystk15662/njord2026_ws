import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo


def generate_launch_description():
    pkg_name = 'drogger_bt'
    share_dir = get_package_share_directory(pkg_name)
    config_path = os.path.join(share_dir, 'config', 'config.yaml')
    script_path = os.path.join(share_dir, 'scripts', 'connect.sh')

    mac_addr = ''
    rfcomm_id = '0'
    channel = '1'

    try:
        with open(config_path, 'r') as file_handle:
            config_dict = yaml.safe_load(file_handle)
            node_params = list(config_dict.values())[0]['ros__parameters']

            mac_addr = node_params.get('bt_mac_address', '')
            rfcomm_id = str(node_params.get('rfcomm_id', 0))
            channel = str(node_params.get('rfcomm_channel', 1))
    except Exception as exc:
        return LaunchDescription([LogInfo(msg=f'Failed to read config file: {exc}')])

    setup_bt_cmd = ExecuteProcess(
        cmd=['sudo', 'bash', script_path, mac_addr, rfcomm_id, channel],
        output='screen'
    )

    return LaunchDescription([setup_bt_cmd])