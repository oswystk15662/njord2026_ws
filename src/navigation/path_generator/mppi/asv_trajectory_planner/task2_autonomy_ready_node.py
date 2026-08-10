#!/usr/bin/env python3
"""Publish the sole Task 2 readiness state for FollowPath-only bringup.

Task 2 keeps the generic ``autonomy_supervisor`` disabled, so this node owns
``/autonomy/ready`` while the persistent Mission/Control graph is active.
"""

from __future__ import annotations

import rclpy
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool


class Task2AutonomyReadyNode(Node):
    def __init__(self) -> None:
        super().__init__("task2_autonomy_ready")

        self.declare_parameter("path_topic", "/planned_path_pruned")
        self.declare_parameter("action_name", "/follow_path")
        self.declare_parameter("ready_topic", "/autonomy/ready")
        self.declare_parameter("path_timeout_sec", 2.0)
        self.declare_parameter("publish_frequency_hz", 5.0)
        self.declare_parameter("collision_monitor_get_state_service", "/collision_monitor/get_state")

        path_topic = str(self.get_parameter("path_topic").value)
        action_name = str(self.get_parameter("action_name").value)
        ready_topic = str(self.get_parameter("ready_topic").value)
        self.path_timeout_sec = float(self.get_parameter("path_timeout_sec").value)
        frequency = float(self.get_parameter("publish_frequency_hz").value)
        collision_state_service = str(
            self.get_parameter("collision_monitor_get_state_service").value
        )

        self.last_valid_path_time = None
        self.collision_monitor_active = False
        self._collision_state_future = None
        self.path_sub = self.create_subscription(Path, path_topic, self._path_callback, 10)
        self.ready_pub = self.create_publisher(Bool, ready_topic, 10)
        self.follow_path_client = ActionClient(self, FollowPath, action_name)
        self.collision_state_client = self.create_client(GetState, collision_state_service)
        self.timer = self.create_timer(1.0 / max(frequency, 1e-3), self._publish_ready)

        self.get_logger().info(
            f"Task2 readiness requires {path_topic}, FollowPath action {action_name}, "
            "and an ACTIVE collision monitor"
        )

    def _path_callback(self, msg: Path) -> None:
        if len(msg.poses) >= 2:
            self.last_valid_path_time = self.get_clock().now()

    def _publish_ready(self) -> None:
        self._refresh_collision_monitor_state()
        path_fresh = False
        if self.last_valid_path_time is not None:
            age_sec = (self.get_clock().now() - self.last_valid_path_time).nanoseconds * 1e-9
            path_fresh = age_sec <= self.path_timeout_sec

        ready = (
            path_fresh
            and self.follow_path_client.server_is_ready()
            and self.collision_monitor_active
        )
        self.ready_pub.publish(Bool(data=ready))

    def _refresh_collision_monitor_state(self) -> None:
        if self._collision_state_future is None:
            if not self.collision_state_client.service_is_ready():
                self.collision_monitor_active = False
                return
            self._collision_state_future = self.collision_state_client.call_async(GetState.Request())
            return
        if not self._collision_state_future.done():
            return
        try:
            state = self._collision_state_future.result().current_state
            self.collision_monitor_active = state.id == State.PRIMARY_STATE_ACTIVE
        except Exception as error:
            self.collision_monitor_active = False
            self.get_logger().warning(
                f"Cannot read Collision Monitor lifecycle state: {error}",
                throttle_duration_sec=2.0,
            )
        finally:
            self._collision_state_future = None


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Task2AutonomyReadyNode()
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
