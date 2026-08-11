#!/usr/bin/env python3
"""Select one usable ego odometry stream while replaying a rosbag.

All Task 2 perception consumers receive the selected stream on one stable
topic.  This avoids accidentally using different odometry sources for the
tracker, velocity compensation, and TF fallback when a recording contains
several localization outputs.
"""

from nav_msgs.msg import Odometry
from std_msgs.msg import String

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile


DEFAULT_CANDIDATE_TOPICS = [
    "/odometry/filtered/local",
    "/odom",
    "/odometry/filtered/global",
]


def choose_preferred_topic(candidate_topics, available_topics):
    """Return the first configured candidate which has supplied odometry."""
    return next((topic for topic in candidate_topics if topic in available_topics), None)


class Task2OdometrySelector(Node):
    def __init__(self):
        super().__init__("task2_odometry_selector")
        self.declare_parameter("candidate_topics", DEFAULT_CANDIDATE_TOPICS)
        self.declare_parameter("output_topic", "/task2/ego_odom")
        # Wait briefly after the first received sample so the preferred local
        # EKF stream wins even if /odom happens to be played first.
        self.declare_parameter("selection_window_sec", 1.0)

        self.candidate_topics = list(self.get_parameter("candidate_topics").value)
        self.selection_window_sec = max(
            0.0, float(self.get_parameter("selection_window_sec").value))
        if not self.candidate_topics:
            raise ValueError("candidate_topics must not be empty")

        self.available_topics = set()
        self.latest_messages = {}
        self.selected_topic = None
        self.selection_deadline_ns = None
        self.publisher = self.create_publisher(
            Odometry, str(self.get_parameter("output_topic").value), 10)
        source_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.source_publisher = self.create_publisher(
            String, "/task2/ego_odom_source", source_qos)
        self.subscriptions = [
            self.create_subscription(
                Odometry, topic,
                lambda message, source=topic: self._odom_callback(message, source),
                10)
            for topic in self.candidate_topics
        ]
        self.timer = self.create_timer(0.05, self._select_when_ready)
        self.get_logger().info(
            "Task 2 rosbag odometry priority: " + " -> ".join(self.candidate_topics))

    def _odom_callback(self, message, source):
        self.available_topics.add(source)
        self.latest_messages[source] = message
        if self.selection_deadline_ns is None:
            self.selection_deadline_ns = (
                self.get_clock().now().nanoseconds +
                int(self.selection_window_sec * 1e9))
        if self.selected_topic == source:
            self.publisher.publish(message)

    def _select_when_ready(self):
        if self.selected_topic is not None or self.selection_deadline_ns is None:
            return
        if self.get_clock().now().nanoseconds < self.selection_deadline_ns:
            return
        self.selected_topic = choose_preferred_topic(
            self.candidate_topics, self.available_topics)
        if self.selected_topic is None:
            return
        source = String()
        source.data = self.selected_topic
        self.source_publisher.publish(source)
        self.publisher.publish(self.latest_messages[self.selected_topic])
        self.get_logger().info(
            f"Selected Task 2 rosbag odometry: {self.selected_topic}")


def main():
    rclpy.init()
    node = Task2OdometrySelector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
