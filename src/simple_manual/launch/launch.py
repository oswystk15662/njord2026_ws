from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    container = ComposableNodeContainer(
        name='simple_manual_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='simple_manual',
                plugin='simple_manual::JoyConverter',
                name='joy_converter'
            ),
            ComposableNode(
                package='simple_manual',
                plugin='simple_manual::SerialWriter',
                name='serial_writer'
            ),
        ],
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(container)
    return ld
