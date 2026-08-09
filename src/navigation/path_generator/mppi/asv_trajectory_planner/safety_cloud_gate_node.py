#!/usr/bin/env python3
"""Fail-safe final gate for the Jetson-provided Task 2 safety cloud.

Collision Monitor rejects obstacle points, but an unavailable source is merely
empty to it.  This node therefore runs on miniPC after Collision Monitor and
publishes zero velocity unless both the safety cloud and its input command are
fresh.  It deliberately gates only the automatic path, not manual control or
the physical emergency-stop chain.
"""

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import PointCloud2


class SafetyCloudGate(Node):
    """Pass a command only while the local safety-cloud is fresh."""

    def __init__(self):
        """Configure topic endpoints and the two freshness timers."""
        super().__init__("task2_safety_cloud_gate")
        self.declare_parameter("safety_topic", "/task2/safety_points")
        self.declare_parameter(
            "cmd_vel_in_topic", "/cmd_vel_collision_checked")
        self.declare_parameter("cmd_vel_out_topic", "/cmd_vel_nav")
        self.declare_parameter("safety_timeout_sec", 1.0)
        self.declare_parameter("command_timeout_sec", 0.5)
        self.declare_parameter("publish_rate_hz", 20.0)

        safety_topic = self.get_parameter("safety_topic").value
        cmd_vel_in_topic = self.get_parameter("cmd_vel_in_topic").value
        cmd_vel_out_topic = self.get_parameter("cmd_vel_out_topic").value
        self.safety_timeout_sec = float(
            self.get_parameter("safety_timeout_sec").value)
        self.command_timeout_sec = float(
            self.get_parameter("command_timeout_sec").value)
        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)

        self.last_safety_at = None
        self.last_command_at = None
        self.last_command = Twist()
        self.safety_sub = self.create_subscription(
            PointCloud2, safety_topic, self.safety_callback, 10)
        self.command_sub = self.create_subscription(
            Twist, cmd_vel_in_topic, self.command_callback, 10)
        self.command_pub = self.create_publisher(Twist, cmd_vel_out_topic, 10)
        self.timer = self.create_timer(
            1.0 / max(publish_rate_hz, 1.0), self.publish_callback)

        self.get_logger().info(
            "Task 2 safety gate: %s + %s -> %s (timeouts %.2f s / %.2f s)" % (
                safety_topic, cmd_vel_in_topic, cmd_vel_out_topic,
                self.safety_timeout_sec, self.command_timeout_sec))

    def safety_callback(self, _message):
        """Record receipt of a safety observation, including an empty cloud."""
        # An empty PointCloud2 is a valid, fresh observation; stale transport
        # is what must inhibit automatic motion.
        self.last_safety_at = time.monotonic()

    def command_callback(self, message):
        """Store the latest command from Collision Monitor."""
        self.last_command = message
        self.last_command_at = time.monotonic()

    @staticmethod
    def is_fresh(stamp, timeout_sec, now):
        """Return whether a receipt stamp is within its timeout."""
        return stamp is not None and now - stamp <= timeout_sec

    def publish_callback(self):
        """Publish a command only when both safety inputs are current."""
        now = time.monotonic()
        safety_fresh = self.is_fresh(
            self.last_safety_at, self.safety_timeout_sec, now)
        command_fresh = self.is_fresh(
            self.last_command_at, self.command_timeout_sec, now)
        if safety_fresh and command_fresh:
            self.command_pub.publish(self.last_command)
        else:
            self.command_pub.publish(Twist())


def main(args=None):
    """Run the safety-cloud gate node."""
    rclpy.init(args=args)
    node = SafetyCloudGate()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
