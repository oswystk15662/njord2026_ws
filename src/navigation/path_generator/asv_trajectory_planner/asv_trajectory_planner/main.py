#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


class TrajectoryGenerator:
    """
    他船位置からtrajectoryを生成するクラス

    現時点では例として，
    他船の現在位置からx方向に一定間隔で点を並べたPathを生成する．

    後でこの中身を，
    ・速度推定
    ・スプライン補間
    ・MPPI
    ・衝突回避用の予測軌道生成
    などに置き換えればよい．
    """

    def __init__(self, prediction_horizon=5.0, dt=0.5, assumed_speed=0.5):
        self.prediction_horizon = prediction_horizon
        self.dt = dt
        self.assumed_speed = assumed_speed

    def generate(self, other_ship_pose: PoseStamped) -> Path:
        """
        Parameters
        ----------
        other_ship_pose : PoseStamped
            他船の現在位置

        Returns
        -------
        Path
            生成されたtrajectory
        """

        path = Path()
        path.header = other_ship_pose.header

        x0 = other_ship_pose.pose.position.x
        y0 = other_ship_pose.pose.position.y
        z0 = other_ship_pose.pose.position.z

        num_steps = int(self.prediction_horizon / self.dt)

        for i in range(num_steps + 1):
            t = i * self.dt

            pose = PoseStamped()
            pose.header = other_ship_pose.header

            # 仮の軌道生成
            # ここではx方向に assumed_speed [m/s] で進むと仮定
            pose.pose.position.x = x0 + self.assumed_speed * t
            pose.pose.position.y = y0
            pose.pose.position.z = z0

            # 姿勢は入力をそのまま使う
            pose.pose.orientation = other_ship_pose.pose.orientation

            path.poses.append(pose)

        return path


class TrajectoryNode(Node):
    """
    他船位置をsubscribeし，trajectoryをpublishするROS 2ノード
    """

    def __init__(self):
        super().__init__("trajectory_node")

        self.declare_parameter("input_topic", "/other_ship/pose")
        self.declare_parameter("output_topic", "/other_ship/trajectory")
        self.declare_parameter("prediction_horizon", 5.0)
        self.declare_parameter("dt", 0.5)
        self.declare_parameter("assumed_speed", 0.5)

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        prediction_horizon = self.get_parameter("prediction_horizon").value
        dt = self.get_parameter("dt").value
        assumed_speed = self.get_parameter("assumed_speed").value

        self.trajectory_generator = TrajectoryGenerator(
            prediction_horizon=prediction_horizon,
            dt=dt,
            assumed_speed=assumed_speed,
        )

        self.subscription = self.create_subscription(
            PoseStamped,
            input_topic,
            self.other_ship_pose_callback,
            10,
        )

        self.publisher = self.create_publisher(
            Path,
            output_topic,
            10,
        )

        self.get_logger().info("trajectory_node has started.")
        self.get_logger().info(f"Subscribe: {input_topic}")
        self.get_logger().info(f"Publish  : {output_topic}")

    def other_ship_pose_callback(self, msg: PoseStamped):
        trajectory = self.trajectory_generator.generate(msg)

        # 出力時刻を現在時刻に更新
        trajectory.header.stamp = self.get_clock().now().to_msg()

        for pose in trajectory.poses:
            pose.header.stamp = trajectory.header.stamp

        self.publisher.publish(trajectory)

        self.get_logger().info(
            f"Published trajectory with {len(trajectory.poses)} poses."
        )


def main(args=None):
    rclpy.init(args=args)

    node = TrajectoryNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()