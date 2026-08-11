#!/usr/bin/env python3
"""Use bag TF when available, otherwise relay ego odometry as TF.

The decision is deliberately made after playback starts.  A recorded dynamic
``odom -> base_link`` transform is authoritative.  Only when it is absent do
we derive that transform from the selected Odometry message, preserving the
recorded pose rather than pinning the vessel at the origin.
"""

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


def _normalise_frame(frame: str) -> str:
    return frame.lstrip("/")


class BagOdometryTfFallback(Node):
    def __init__(self):
        super().__init__("bag_odometry_tf_fallback")
        self.declare_parameter("ego_odom_topic", "/odometry/filtered/local")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("fallback_delay_sec", 2.0)
        self.declare_parameter("fallback_livox_x", 0.5)
        self.declare_parameter("fallback_livox_y", 0.0)
        self.declare_parameter("fallback_livox_z", 0.8)
        self.declare_parameter("fallback_livox_roll", 3.141592653589793)

        self.odom_frame = _normalise_frame(
            self.get_parameter("odom_frame").value)
        self.base_frame = _normalise_frame(
            self.get_parameter("base_frame").value)
        self.dynamic_tf_seen = False
        self.livox_tf_seen = False
        self.fallback_active = False
        self._latest_odom = None

        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.create_subscription(TFMessage, "/tf", self._tf_callback, 100)
        static_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(TFMessage, "/tf_static", self._static_callback,
                                 static_qos)
        self.create_subscription(
            Odometry, self.get_parameter("ego_odom_topic").value,
            self._odom_callback, 10)
        self._decision_timer = self.create_timer(
            self.get_parameter("fallback_delay_sec").value,
            self._decide_fallback)

    def _matches_odom_base(self, transform: TransformStamped) -> bool:
        return (_normalise_frame(transform.header.frame_id) == self.odom_frame
                and _normalise_frame(transform.child_frame_id) == self.base_frame)

    def _tf_callback(self, message: TFMessage):
        if any(self._matches_odom_base(transform)
               for transform in message.transforms):
            self.dynamic_tf_seen = True

    def _static_callback(self, message: TFMessage):
        if any(_normalise_frame(transform.header.frame_id) == self.base_frame
               and _normalise_frame(transform.child_frame_id) == "livox_frame"
               for transform in message.transforms):
            self.livox_tf_seen = True

    def _odom_callback(self, message: Odometry):
        self._latest_odom = message
        if self.fallback_active:
            self._publish_odom_transform(message)

    def _decide_fallback(self):
        if self.dynamic_tf_seen:
            self.get_logger().info(
                "Using recorded dynamic odom -> base_link TF; "
                "odometry TF fallback is disabled.")
            self.destroy_timer(self._decision_timer)
            return

        self.fallback_active = True
        self.get_logger().warning(
            "No recorded dynamic odom -> base_link TF found; relaying "
            "the selected odometry pose as dynamic TF.")
        if self._latest_odom is not None:
            self._publish_odom_transform(self._latest_odom)
        if not self.livox_tf_seen:
            self._publish_livox_static_transform()
        self.destroy_timer(self._decision_timer)

    def _publish_odom_transform(self, message: Odometry):
        transform = TransformStamped()
        transform.header = message.header
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = message.pose.pose.position.x
        transform.transform.translation.y = message.pose.pose.position.y
        transform.transform.translation.z = message.pose.pose.position.z
        transform.transform.rotation = message.pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)

    def _publish_livox_static_transform(self):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.base_frame
        transform.child_frame_id = "livox_frame"
        transform.transform.translation.x = self.get_parameter(
            "fallback_livox_x").value
        transform.transform.translation.y = self.get_parameter(
            "fallback_livox_y").value
        transform.transform.translation.z = self.get_parameter(
            "fallback_livox_z").value
        transform.transform.rotation.x = 1.0
        transform.transform.rotation.w = 0.0
        self.static_broadcaster.sendTransform(transform)


def main():
    rclpy.init()
    node = BagOdometryTfFallback()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
