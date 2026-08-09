"""Publish Task 2 perception readiness from the filtered LiDAR cloud."""

from __future__ import annotations

from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool

from .readiness_evidence import filtered_cloud_ready


_STATUS_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


def _stamp_seconds(stamp) -> Optional[float]:
    seconds = float(stamp.sec) + float(stamp.nanosec) * 1e-9
    return seconds if seconds > 0.0 else None


class Task2ReadinessAdapter(Node):
    def __init__(self) -> None:
        super().__init__("task2_readiness_adapter")
        self.declare_parameter("filtered_cloud_topic", "/task2/points_filtered")
        self.declare_parameter("timeout_sec", 1.0)
        self.declare_parameter("publish_period_sec", 0.1)
        self._observed_at: Optional[float] = None
        self._header_stamp: Optional[float] = None
        self._header_frame = ""
        self._publisher = self.create_publisher(
            Bool, "/mission/readiness/buoy_perception", _STATUS_QOS
        )
        self._subscription = self.create_subscription(
            PointCloud2,
            str(self.get_parameter("filtered_cloud_topic").value),
            self._on_cloud,
            10,
        )
        period = float(self.get_parameter("publish_period_sec").value)
        if period <= 0.0:
            raise ValueError("publish_period_sec must be positive")
        self._timer = self.create_timer(period, self._publish)
        self._publish()

    def _now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_cloud(self, message: PointCloud2) -> None:
        self._observed_at = self._now_seconds()
        self._header_stamp = _stamp_seconds(message.header.stamp)
        self._header_frame = message.header.frame_id

    def _publish(self) -> None:
        self._publisher.publish(Bool(data=filtered_cloud_ready(
            self._observed_at,
            self._now_seconds(),
            float(self.get_parameter("timeout_sec").value),
            self._header_stamp,
            self._header_frame,
        )))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Task2ReadinessAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
