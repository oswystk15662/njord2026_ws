import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_robot = get_package_share_directory('robot')

    default_params_file = os.path.join(
        pkg_robot,
        'config',
        'nav2_params.yaml'
    )
    default_nav_to_pose_bt_xml = os.path.join(
        pkg_robot, 'config', 'navigate_to_pose_w_replanning_and_recovery.xml'
    )
    default_nav_through_poses_bt_xml = os.path.join(
        pkg_robot, 'config', 'navigate_through_poses_w_replanning_and_recovery.xml'
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the Nav2 parameters file'
    )
    nav_to_pose_bt_xml_arg = DeclareLaunchArgument(
        'nav_to_pose_bt_xml',
        default_value=default_nav_to_pose_bt_xml,
        description='Full path to the NavigateToPose behavior tree XML'
    )
    nav_through_poses_bt_xml_arg = DeclareLaunchArgument(
        'nav_through_poses_bt_xml',
        default_value=default_nav_through_poses_bt_xml,
        description='Full path to the NavigateThroughPoses behavior tree XML'
    )

    configured_params = RewrittenYaml(
        source_file=LaunchConfiguration('params_file'),
        root_key=None,
        param_rewrites={
            'bt_navigator.ros__parameters.default_nav_to_pose_bt_xml':
                LaunchConfiguration('nav_to_pose_bt_xml'),
            'bt_navigator.ros__parameters.default_nav_through_poses_bt_xml':
                LaunchConfiguration('nav_through_poses_bt_xml'),
        },
        convert_types=True,
    )

    return LaunchDescription([
        params_file_arg,
        nav_to_pose_bt_xml_arg,
        nav_through_poses_bt_xml_arg,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'params_file': configured_params,
                'use_sim_time': 'false',
                'autostart': 'true'
            }.items()
        )
    ])
