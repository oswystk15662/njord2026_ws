# Livox LiDAR 起動。config の canonical は robot/config/livox/。
# vendored の config/launch は bringup では未使用。
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
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

    user_config_path = PathJoinSubstitution(
        [FindPackageShare("robot"), "config", "livox", config_file]
    )

    livox_driver = Node(
        package="livox_ros_driver2",
        executable="livox_ros_driver2_node",
        name="livox_lidar_publisher",
        output="screen",
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
    )

    return [livox_driver]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "lidar_model",
                default_value="mid360",
                choices=["mid360", "mid360s"],
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
