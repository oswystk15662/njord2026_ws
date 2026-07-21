"""Receive the ZED2i RTP/JPEG ground-video stream with low latency."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    port = LaunchConfiguration('port')
    jitter_latency_ms = LaunchConfiguration('jitter_latency_ms')
    return LaunchDescription(
        [
            DeclareLaunchArgument('port', default_value='5600'),
            DeclareLaunchArgument('jitter_latency_ms', default_value='50'),
            ExecuteProcess(
                cmd=[
                    'gst-launch-1.0', '-e',
                    'udpsrc', PythonExpression(["'port=' + str(", port, ")"]),
                    'caps=application/x-rtp,media=video,encoding-name=JPEG,payload=26',
                    '!', 'rtpjitterbuffer',
                    PythonExpression(["'latency=' + str(", jitter_latency_ms, ")"]),
                    'drop-on-latency=true',
                    '!', 'rtpjpegdepay', '!', 'jpegdec', '!', 'autovideosink', 'sync=false',
                ],
                output='screen',
            ),
        ]
    )
