import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, LogInfo
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'drogger_bt'
    share_dir = get_package_share_directory(pkg_name)
    config_path = os.path.join(share_dir, 'config', 'config.yaml')
    script_path = os.path.join(share_dir, 'scripts', 'connect.sh')

    # Read YAML manually to extract params for the shell script
    mac_addr = ""
    rfcomm_id = "0"
    channel = "1"
    
    try:
        with open(config_path, 'r') as f:
            config_dict = yaml.safe_load(f)
            # Assuming standard ros2 param structure: /**/ros__parameters/...
            # We take the first key (usually /**)
            node_params = list(config_dict.values())[0]['ros__parameters']
            
            mac_addr = node_params.get('bt_mac_address', '')
            rfcomm_id = str(node_params.get('rfcomm_id', 0))
            channel = str(node_params.get('rfcomm_channel', 1))
    except Exception as e:
        return LaunchDescription([LogInfo(msg=f"Failed to read config file: {e}")])

    # 1. Setup Script Process
    # We use 'bash' to execute the script
    setup_bt_cmd = ExecuteProcess(
        cmd=['bash', script_path, mac_addr, rfcomm_id, channel],
        output='screen'
    )

    # 2. Driver Node
    driver_node = Node(
        package=pkg_name,
        executable='drogger_bt',
        name='drogger_driver',
        output='screen',
        parameters=[config_path]
    )

    # 3. Sequencing: Run node after script finishes
    return LaunchDescription([
        setup_bt_cmd,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=setup_bt_cmd,
                on_exit=[driver_node]
            )
        )
    ])