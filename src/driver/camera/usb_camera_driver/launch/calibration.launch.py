import launch
import launch_ros.actions
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    TextSubstitution,
)
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    mjpg_streamer_dir = "/home/osw/Download/mjpg-streamer/mjpg-streamer-experimental"

    # LD_LIBRARY_PATHを設定する
    env = {
        "LD_LIBRARY_PATH": f"{mjpg_streamer_dir}:{EnvironmentVariable('LD_LIBRARY_PATH')}",
    }

    # -o オプションに渡す引数を構築
    # output_plugin_args_left = TextSubstitution(text=f"output_file.so -f /home/osw/water2025/tmp/left")
    # output_plugin_args_right = TextSubstitution(text=f"output_file.so -f /home/osw/water2025/tmp/right")
    output_plugin_args_left = TextSubstitution(text="output_http.so -w ./www -p 8080")
    output_plugin_args_right = TextSubstitution(text="output_http.so -w ./www -p 7070")

    # -i オプションに渡す引数を構築
    input_plugin_args_left = TextSubstitution(
        text="input_uvc.so -d /dev/camera_c270_stream_left -r 640x480 -fps 30"
    )
    input_plugin_args_right = TextSubstitution(
        text="input_uvc.so -d /dev/camera_c270_stream_right -r 640x480 -fps 30"
    )

    mjpg_streamer_left = ExecuteProcess(
        cmd=[
            f"{mjpg_streamer_dir}/mjpg_streamer",
            "-o",
            output_plugin_args_left,
            "-i",
            input_plugin_args_left,
        ],
        cwd=mjpg_streamer_dir,
        env=env,
        output="screen",
    )

    mjpg_streamer_right = ExecuteProcess(
        cmd=[
            f"{mjpg_streamer_dir}/mjpg_streamer",
            "-o",
            output_plugin_args_right,
            "-i",
            input_plugin_args_right,
        ],
        cwd=mjpg_streamer_dir,
        env=env,
        output="screen",
    )

    image_width_arg = DeclareLaunchArgument(
        "image_width", default_value="640", description="Width of the captured image"
    )
    image_height_arg = DeclareLaunchArgument(
        "image_height", default_value="480", description="Height of the captured image"
    )
    framerate_arg = DeclareLaunchArgument(
        "framerate",
        default_value="30",
        description="Framerate of the captured image (fps)",
    )

    # カメラ0のパラメータ
    # Parameters for Camera 0
    camera_device_left_arg = DeclareLaunchArgument(
        "camera_left",
        # default_value="/dev/camera_c270_stream_left",
        default_value="http://localhost:8080/?action=stream",
        description="Path to the camera device for camera 0 (e.g., /dev/video0)",
    )
    camera_frame_id_left_arg = DeclareLaunchArgument(
        "camera_frame_id_left",
        default_value="camera_link_left",
        description="Frame ID for camera 0 (for TF)",
    )
    compressed_image_topic_name_left_arg = DeclareLaunchArgument(
        "image_topic_name_left",
        # default_value="camera_left/image_raw",
        default_value="left",
        description="Name of the image topic for camera 0 to publish",
    )
    camera_info_topic_name_left_arg = DeclareLaunchArgument(
        "camera_info_topic_name_left",
        # default_value="camera_left/camera_info",
        default_value="left_camera",
        description="Name of the camera info topic for camera 0 to publish",
    )

    # カメラ1のパラメータ
    # Parameters for Camera 1
    camera_device_right_arg = DeclareLaunchArgument(
        "camera_right",
        default_value="http://localhost:7070/?action=stream",
        description="Path to the camera device for camera 1 (e.g., /dev/video1)",
    )
    camera_frame_id_right_arg = DeclareLaunchArgument(
        "camera_frame_id_right",
        default_value="camera_link_right",
        description="Frame ID for camera 1 (for TF)",
    )
    compressed_image_topic_name_right_arg = DeclareLaunchArgument(
        "image_topic_name_right",
        # default_value="camera_right/image_raw",
        default_value="right",
        description="Name of the image topic for camera 1 to publish",
    )
    camera_info_topic_name_right_arg = DeclareLaunchArgument(
        "camera_info_topic_name_right",
        # default_value="camera_right/camera_info",
        default_value="right_camera",
        description="Name of the camera info topic for camera 1 to publish",
    )

    # --- コンポーザブルノードコンテナとカメラドライバーノードの定義 ---
    # --- Define Composable Node Container and Camera Driver Nodes ---
    composable_camera_nodes = [
        # ComposableNode(
        #     package="rpi_camera_driver",
        #     plugin="RPiCameraDriverNode",  # C++ノードのクラス名
        #     # C++ node class name
        #     name="rpi_camera_driver_node_left",
        #     parameters=[
        #         {"camera_device": LaunchConfiguration("camera_left")},
        #         {"image_width": LaunchConfiguration("image_width")},
        #         {"image_height": LaunchConfiguration("image_height")},
        #         {"framerate": LaunchConfiguration("framerate")},
        #         {"camera_frame_id": LaunchConfiguration("camera_frame_id_left")},
        #         {"image_topic_name": LaunchConfiguration("image_topic_name_left")},
        #         {
        #             "camera_info_topic_name": LaunchConfiguration(
        #                 "camera_info_topic_name_left"
        #             )
        #         },
        #     ],
        # ),
        # ComposableNode(
        #     package="rpi_camera_driver",
        #     plugin="RPiCameraDriverNode",  # 同じクラスで2台目のカメラ
        #     # Same class for the second camera
        #     name="rpi_camera_driver_node_right",
        #     parameters=[
        #         {"camera_device": LaunchConfiguration("camera_right")},
        #         {"image_width": LaunchConfiguration("image_width")},
        #         {"image_height": LaunchConfiguration("image_height")},
        #         {"framerate": LaunchConfiguration("framerate")},
        #         {"camera_frame_id": LaunchConfiguration("camera_frame_id_right")},
        #         {"image_topic_name": LaunchConfiguration("image_topic_name_right")},
        #         {
        #             "camera_info_topic_name": LaunchConfiguration(
        #                 "camera_info_topic_name_right"
        #             )
        #         },
        #     ],
        # ),
        
        # mjpg
        ComposableNode(
            package="rpi_camera_driver",
            plugin="RPiCameraDriverMJPGNode",
            # Same class for the second camera
            name="rpi_camera_driver_node_left",
            parameters=[
                {"camera_device": LaunchConfiguration("camera_left")},
                {"image_width": LaunchConfiguration("image_width")},
                {"image_height": LaunchConfiguration("image_height")},
                {"framerate": LaunchConfiguration("framerate")},
                {"camera_frame_id": LaunchConfiguration("camera_frame_id_left")},
                {"image_topic_name": LaunchConfiguration("image_topic_name_left")},
                {
                    "camera_info_topic_name": LaunchConfiguration(
                        "camera_info_topic_name_left"
                    )
                },
            ],
        ),
        ComposableNode(
            package="rpi_camera_driver",
            plugin="RPiCameraDriverMJPGNode",  # 同じクラスで2台目のカメラ
            # Same class for the second camera
            name="rpi_camera_driver_node_right",
            parameters=[
                {"camera_device": LaunchConfiguration("camera_right")},
                {"image_width": LaunchConfiguration("image_width")},
                {"image_height": LaunchConfiguration("image_height")},
                {"framerate": LaunchConfiguration("framerate")},
                {"camera_frame_id": LaunchConfiguration("camera_frame_id_right")},
                {"image_topic_name": LaunchConfiguration("image_topic_name_right")},
                {
                    "camera_info_topic_name": LaunchConfiguration(
                        "camera_info_topic_name_right"
                    )
                },
            ],
        ),

    ]

    # コンテナノードの定義
    # Define the container node
    camera_container = ComposableNodeContainer(
        name="camera_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=composable_camera_nodes,
        output="screen",
    )

    # camera calibrator setting

    # ChArUcoボードのパラメータをLaunchArgumentsとして定義
    squares_x_arg = DeclareLaunchArgument("squares_x", default_value="7")
    squares_y_arg = DeclareLaunchArgument("squares_y", default_value="5")
    square_len_arg = DeclareLaunchArgument("square_len", default_value="0.025")
    marker_len_arg = DeclareLaunchArgument("charuco_marker_size", default_value="0.015")
    aruco_dict_arg = DeclareLaunchArgument("aruco_dict", default_value="6x6_250")

    # LaunchConfigurationオブジェクトを取得
    squares_x = LaunchConfiguration("squares_x")
    squares_y = LaunchConfiguration("squares_y")
    square_len = LaunchConfiguration("square_len")
    marker_len = LaunchConfiguration("charuco_marker_size")
    aruco_dict = LaunchConfiguration("aruco_dict")

    # `--size` 引数を LaunchConfiguration のリストとして構築
    size_arg = [squares_x, "x", squares_y]

    return launch.LaunchDescription(
        [
            # mjpg
            mjpg_streamer_left,
            mjpg_streamer_right,
            # other
            image_width_arg,
            image_height_arg,
            framerate_arg,
            camera_device_left_arg,
            camera_frame_id_left_arg,
            compressed_image_topic_name_left_arg,
            camera_info_topic_name_left_arg,
            camera_device_right_arg,
            camera_frame_id_right_arg,
            compressed_image_topic_name_right_arg,
            camera_info_topic_name_right_arg,
            camera_container,  # ここにコンテナノードのみを配置
            squares_x_arg,
            squares_y_arg,
            square_len_arg,
            marker_len_arg,
            aruco_dict_arg,
            launch_ros.actions.Node(
                package="camera_calibration",
                executable="cameracalibrator",
                name="cameracalibrator",
                output="screen",
                arguments=[
                    "--pattern",
                    "charuco",
                    "--size",
                    size_arg,
                    "--square",
                    square_len,
                    "--aruco_dict",
                    aruco_dict,
                    "--charuco_marker_size",
                    marker_len,
                    "--mode",
                    "stereo",
                    # "left:=/camera_left/image_raw",  # <-- トピック名引数として渡す
                    # "left_camera:=/camera_left/camera_info",
                    # "right:=/camera_right/image_raw",  # <-- トピック名引数として渡す
                    # "right_camera:=/camera_right/camera_info"
                ],
                # remappings=[
                #     # リマッピングのキーと値が文字列として確実に渡されるように修正
                #     ("left", "/camera_left/image_raw"),
                #     ("left_camera", "/camera_left/camera_info"),
                #     ("right", "/camera_right/image_raw"),
                #     ("right_camera", "/camera_right/camera_info"),
                # ],
            ),
        ]
    )