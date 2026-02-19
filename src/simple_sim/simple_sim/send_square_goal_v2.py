import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import time

class SquareNavigator(Node):
    def __init__(self):
        super().__init__('square_navigator')
        self.navigator = BasicNavigator()
        
        # 4m四方の四角形（少し小さくしました）
        self.corners = [
            (5.0, 0.0),
            (5.0, 5.0),
            (0.0, 5.0),
            (0.0, 0.0)
        ]

    def start_navigation(self):
        self.get_logger().info("Nav2の起動を待機中...")

        # ★【重要修正ポイント】
        # AMCL(地図マッピング)を使っていないため、localizer待機を無効化(None)します。
        # これを指定しないと、存在しないamclノードを永遠に待ち続けてしまいます。
        # self.navigator.waitUntilNav2Active(localizer=None)
        self.navigator.waitUntilNav2Active(localizer='controller_server')
        import time
        time.sleep(3.0)
        
        self.get_logger().info("Nav2がアクティブになりました。走行を開始します。")

        # ゴールリストの作成
        goal_poses = self._create_goal_poses()
        
        # 指令送信 (Navigate Through Poses)
        self.navigator.goThroughPoses(goal_poses)

        # 監視ループ開始
        self._monitor_progress()

    def _create_goal_poses(self):
        goal_poses = []
        frame_id = 'odom'
        now = self.navigator.get_clock().now().to_msg()

        for pt in self.corners:
            pose = PoseStamped()
            pose.header.frame_id = frame_id
            pose.header.stamp = now
            pose.pose.position.x = pt[0]
            pose.pose.position.y = pt[1]
            pose.pose.orientation.w = 1.0
            goal_poses.append(pose)
        return goal_poses

    def _monitor_progress(self):
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                # 距離表示（オプション）
                # print(f'残り距離: {feedback.distance_remaining:.2f} m', end='\r')
                pass
            time.sleep(0.1)

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info("ゴールに到達しました！")
        elif result == TaskResult.CANCELED:
            self.get_logger().warn("タスクがキャンセルされました")
        elif result == TaskResult.FAILED:
            self.get_logger().error("タスクが失敗しました")

def main():
    rclpy.init()
    node = SquareNavigator()
    try:
        node.start_navigation()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()