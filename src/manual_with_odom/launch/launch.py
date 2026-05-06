from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    container = ComposableNodeContainer(
        name='manual_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='manual_with_odom',
                plugin='manual_with_odom::JoyConverter',
                name='joy_converter'
            ),
            ComposableNode(
                package='manual_with_odom',
                plugin='manual_with_odom::PIDController',
                name='pid_force_controller'
            ),
        ],
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(container)
    return ld
