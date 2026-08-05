import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml


def _load_replanning_frequency(params_file):
    with open(params_file, 'r') as f:
        params = yaml.safe_load(f) or {}
    bt_params = params.get('bt_navigator', {}).get('ros__parameters', {})
    return float(bt_params.get('replanning_frequency', 1.0))


def _write_configured_bt_xml(source_file, replanning_frequency):
    with open(source_file, 'r') as f:
        xml = f.read()
    xml = xml.replace('__REPLANNING_FREQUENCY__', f'{replanning_frequency:.6g}')
    tmp = tempfile.NamedTemporaryFile(
        mode='w',
        prefix='nav2_bt_',
        suffix='.xml',
        delete=False,
    )
    with tmp:
        tmp.write(xml)
    return tmp.name


def _launch_nav2(context, pkg_robot):
    params_file = LaunchConfiguration('params_file').perform(context)
    nav_to_pose_bt_xml = LaunchConfiguration('nav_to_pose_bt_xml').perform(context)
    nav_through_poses_bt_xml = LaunchConfiguration('nav_through_poses_bt_xml').perform(context)

    replanning_frequency = _load_replanning_frequency(params_file)
    configured_nav_to_pose_bt_xml = _write_configured_bt_xml(
        nav_to_pose_bt_xml,
        replanning_frequency,
    )
    configured_nav_through_poses_bt_xml = _write_configured_bt_xml(
        nav_through_poses_bt_xml,
        replanning_frequency,
    )

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=None,
        param_rewrites={
            'bt_navigator.ros__parameters.default_nav_to_pose_bt_xml':
                configured_nav_to_pose_bt_xml,
            'bt_navigator.ros__parameters.default_nav_through_poses_bt_xml':
                configured_nav_through_poses_bt_xml,
        },
        convert_types=True,
    )

    diagnostics_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robot, 'launch', 'diagnostics.launch.py')
        ),
        launch_arguments={'profile': 'nav2'}.items(),
        condition=IfCondition(LaunchConfiguration('enable_diagnostics')),
    )

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_robot, 'launch', 'navigation.launch.py')
            ),
            launch_arguments={
                'params_file': configured_params,
                'use_sim_time': 'false',
                'autostart': 'true'
            }.items()
        ),
        diagnostics_launch,
    ]


def generate_launch_description():
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
    enable_diagnostics_arg = DeclareLaunchArgument(
        'enable_diagnostics',
        default_value='true',
        description='Launch generic topic heartbeat diagnostics for Nav2 topics'
    )

    return LaunchDescription([
        params_file_arg,
        nav_to_pose_bt_xml_arg,
        nav_through_poses_bt_xml_arg,
        enable_diagnostics_arg,
        OpaqueFunction(function=_launch_nav2, args=[pkg_robot]),
    ])
