import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def launch_setup(context, *args, **kwargs):
    mode = LaunchConfiguration("mode").perform(context)
    if mode not in ("cpu", "sdk"):
        raise RuntimeError("mode must be either 'cpu' or 'sdk'")

    plugin = (
        "zed2i_driver::CpuStereoNode"
        if mode == "cpu"
        else "zed2i_driver::SdkNode"
    )
    return [
        ComposableNodeContainer(
            package="rclcpp_components",
            executable="component_container_mt",
            name="zed2i_container",
            namespace="/",
            output="screen",
            composable_node_descriptions=[
                ComposableNode(
                    package="zed2i_driver",
                    plugin=plugin,
                    name="zed2i",
                    namespace=LaunchConfiguration("namespace"),
                    parameters=[
                        LaunchConfiguration("params_file"),
                        {
                            "enable_gpu_perception": LaunchConfiguration("enable_gpu_perception"),
                            "engine_path": LaunchConfiguration("engine_path"),
                        },
                    ],
                    extra_arguments=[{"use_intra_process_comms": True}],
                )
            ],
        )
    ]


def generate_launch_description():
    default_params_file = os.path.join(
        get_package_share_directory("zed2i_driver"), "config", "zed2i_jetson_orin_nano.yaml"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("mode", default_value="sdk"),
            DeclareLaunchArgument("namespace", default_value="/zed2i"),
            DeclareLaunchArgument("params_file", default_value=default_params_file),
            DeclareLaunchArgument("enable_gpu_perception", default_value="false"),
            DeclareLaunchArgument("engine_path", default_value=""),
            OpaqueFunction(function=launch_setup),
        ]
    )
