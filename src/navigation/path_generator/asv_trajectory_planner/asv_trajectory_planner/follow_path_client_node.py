#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav_msgs.msg import Path
from nav2_msgs.action import FollowPath


class FollowPathClientNode(Node):
    """
    /planned_pathをNav2のFollowPath actionに送るノード．

    Subscribe:
        /planned_path

    Action Client:
        /follow_path
    """

    def __init__(self):
        super().__init__("follow_path_client_node")

        self.declare_parameter("path_topic", "/planned_path")
        self.declare_parameter("action_name", "follow_path")

        # Nav2側のcontroller plugin ID
        # Nav2のデフォルト設定では "FollowPath" であることが多い
        self.declare_parameter("controller_id", "FollowPath")
        self.declare_parameter("goal_checker_id", "general_goal_checker")
        self.declare_parameter("progress_checker_id", "progress_checker")

        # PathをNav2へ送る周期
        self.declare_parameter("send_frequency", 1.0)

        # Trueにすると，新しいPathを定期的に送り直す
        self.declare_parameter("enable_replanning", True)

        self.path_topic = self.get_parameter("path_topic").value
        self.action_name = self.get_parameter("action_name").value
        self.controller_id = self.get_parameter("controller_id").value
        self.goal_checker_id = self.get_parameter("goal_checker_id").value
        self.progress_checker_id = self.get_parameter("progress_checker_id").value
        self.enable_replanning = self.get_parameter("enable_replanning").value

        send_frequency = self.get_parameter("send_frequency").value
        timer_period = 1.0 / max(send_frequency, 1e-6)

        self.latest_path = None
        self.goal_active = False

        self.path_sub = self.create_subscription(
            Path,
            self.path_topic,
            self.path_callback,
            10,
        )

        self.action_client = ActionClient(
            self,
            FollowPath,
            self.action_name,
        )

        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info("follow_path_client_node started.")
        self.get_logger().info(f"Subscribe path: {self.path_topic}")
        self.get_logger().info(f"Action name   : {self.action_name}")
        self.get_logger().info(f"Controller ID : {self.controller_id}")

    def path_callback(self, msg: Path):
        if len(msg.poses) == 0:
            self.get_logger().warn("Received empty path. Ignored.")
            return

        self.latest_path = msg

    def timer_callback(self):
        if self.latest_path is None:
            return

        if self.goal_active and not self.enable_replanning:
            return

        if not self.action_client.server_is_ready():
            self.get_logger().warn(
                f"Waiting for FollowPath action server: {self.action_name}",
                throttle_duration_sec=2.0,
            )
            return

        self.send_follow_path_goal(self.latest_path)

    def send_follow_path_goal(self, path: Path):
        goal_msg = FollowPath.Goal()
        goal_msg.path = path
        goal_msg.controller_id = self.controller_id
        goal_msg.goal_checker_id = self.goal_checker_id
        goal_msg.progress_checker_id = self.progress_checker_id

        self.get_logger().info(
            f"Sending FollowPath goal: {len(path.poses)} poses",
            throttle_duration_sec=2.0,
        )

        send_goal_future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback,
        )

        send_goal_future.add_done_callback(self.goal_response_callback)
        self.goal_active = True

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warn("FollowPath goal was rejected.")
            self.goal_active = False
            return

        self.get_logger().info("FollowPath goal was accepted.")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback

        # 必要ならfeedbackの中身を確認する
        # ここではログを出しすぎないようにdebugにしている
        self.get_logger().debug(f"FollowPath feedback: {feedback}")

    def result_callback(self, future):
        result = future.result().result

        self.get_logger().info(
            f"FollowPath finished. Error code: {result.error_code}"
        )

        self.goal_active = False


def main(args=None):
    rclpy.init(args=args)

    node = FollowPathClientNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()