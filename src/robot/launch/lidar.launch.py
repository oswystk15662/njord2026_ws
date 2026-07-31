# Livox LiDAR 起動。config の canonical は robot/config/livox/。
# vendored の config/launch は bringup では未使用。
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare

# Livox ROS driver parameters, transcribed from
# livox_ros_driver2/launch_ROS2/msg_MID360_launch.py.
XFER_FORMAT = 0  # 0-Pointcloud2(PointXYZRTL), 1-customized pointcloud format
MULTI_TOPIC = 0  # 0-All LiDARs share the same topic, 1-One LiDAR one topic
DATA_SRC = 0  # 0-lidar, others-Invalid data src
PUBLISH_FREQ = 10.0  # frequency of publish, 5.0, 10.0, 20.0, 50.0, etc.
OUTPUT_DATA_TYPE = 0
FRAME_ID = "livox_frame"  # glim/TF が依存。必須。
LVX_FILE_PATH = "/home/livox/livox_test.lvx"
CMDLINE_INPUT_BD_CODE = "livox0000000001"

LIDAR_MODEL_TO_CONFIG_FILE = {
    "mid360": "MID360_config.json",
    "mid360s": "MID360S_config.json",
}


def launch_setup(context, *args, **kwargs):
    lidar_model = LaunchConfiguration("lidar_model").perform(context)
    config_file = LIDAR_MODEL_TO_CONFIG_FILE[lidar_model]
    enable_buoy_detection = (
        LaunchConfiguration("enable_buoy_detection").perform(context).lower()
        in ("true", "1", "yes", "on")
    )
    enable_glim = (
        LaunchConfiguration("enable_glim").perform(context).lower()
        in ("true", "1", "yes", "on")
    )
    glim_backend = LaunchConfiguration("glim_backend").perform(context)
    glim_config_dir_name = "glim_config" if glim_backend == "gpu" else "glim_config_cpu"

    user_config_path = PathJoinSubstitution(
        [FindPackageShare("robot"), "config", "livox", config_file]
    )

    components = [
        ComposableNode(
            package="livox_ros_driver2",
            plugin="livox_ros::DriverNode",
            name="livox_lidar_publisher",
            parameters=[
                {
                    "xfer_format": XFER_FORMAT,
                    "multi_topic": MULTI_TOPIC,
                    "data_src": DATA_SRC,
                    "publish_freq": PUBLISH_FREQ,
                    "output_data_type": OUTPUT_DATA_TYPE,
                    "frame_id": FRAME_ID,
                    "lvx_file_path": LVX_FILE_PATH,
                    "user_config_path": user_config_path,
                    "cmdline_input_bd_code": CMDLINE_INPUT_BD_CODE,
                }
            ],
            extra_arguments=[{"use_intra_process_comms": True}],
        )
    ]

    if enable_buoy_detection:
        components.append(
            ComposableNode(
                package="pcl_det",
                plugin="pcl_det::PclBuoyDetectionNode",
                name="pcl_bouy_det_node",
                parameters=[
                    {
                        "input_topic": "/livox/lidar",
                        "roi_topic": LaunchConfiguration("roi_topic"),
                        "output_topic": LaunchConfiguration("output_topic"),
                        "frame_id": LaunchConfiguration("detection_frame_id"),
                    }
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            )
        )

    # GLIM is a registered rclcpp component (glim::GlimROS).  Keeping it in
    # the Livox container lets its PointCloud2 subscription use ROS 2
    # intra-process transport instead of serializing the cloud through DDS.
    # It still receives IMU over the normal topic path because that publisher
    # is not part of this container.
    if enable_glim:
        components.append(
            ComposableNode(
                package="glim_ros",
                plugin="glim::GlimROS",
                name="glim_node",
                parameters=[
                    {
                        "config_path": PathJoinSubstitution(
                            [FindPackageShare("robot"), "config", glim_config_dir_name]
                        ),
                        "use_sim_time": False,
                    }
                ],
                remappings=[("/glim_node/odom", "/odom")],
                extra_arguments=[{"use_intra_process_comms": True}],
            )
        )

    sensor_container = ComposableNodeContainer(
        package="rclcpp_components",
        executable="component_container_mt",
        name="livox_perception_container",
        namespace="/",
        output="screen",
        composable_node_descriptions=components,
    )

    return [sensor_container]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "lidar_model",
                default_value="mid360s",
                choices=["mid360", "mid360s"],
            ),
            DeclareLaunchArgument("enable_buoy_detection", default_value="false"),
            DeclareLaunchArgument(
                "enable_glim",
                default_value="true",
                description="Load GLIM into the Livox component container",
            ),
            DeclareLaunchArgument(
                "glim_backend",
                default_value="gpu",
                choices=["gpu", "cpu"],
                description="Select the GLIM config directory (glim_config for gpu, glim_config_cpu for cpu)",
            ),
            DeclareLaunchArgument("roi_topic", default_value="/buoy_roi"),
            DeclareLaunchArgument("output_topic", default_value="/buoy_detections"),
            DeclareLaunchArgument("detection_frame_id", default_value="base_link"),
            OpaqueFunction(function=launch_setup),
        ]
    )
