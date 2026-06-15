import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('natural_cubic_spline')
    
    # Path to test parameters
    default_params_file = os.path.join(pkg_share, 'config', 'test_nav2_params.yaml')
    
    params_file = LaunchConfiguration('params_file')
    
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the ROS2 parameters file to use for all launched nodes'
    )
    
    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file]
    )
    
    smoother_server_node = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[params_file]
    )
    
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_test',
        output='screen',
        parameters=[
            {'use_sim_time': False},
            {'autostart': True},
            {'node_names': ['planner_server', 'smoother_server']}
        ]
    )

    static_tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen'
    )

    static_tf_odom_baselink = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_odom_baselink',
        arguments=['-5', '0', '0', '0', '0', '0', 'odom', 'base_link'],
        output='screen'
    )
    
    ld = LaunchDescription()
    ld.add_action(declare_params_file_cmd)
    ld.add_action(planner_server_node)
    ld.add_action(smoother_server_node)
    ld.add_action(lifecycle_manager_node)
    ld.add_action(static_tf_map_odom)
    ld.add_action(static_tf_odom_baselink)
    
    return ld
