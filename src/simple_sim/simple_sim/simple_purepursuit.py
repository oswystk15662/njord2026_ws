import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from tf_transformations import euler_from_quaternion

class SimplePurePursuit(Node):
    def __init__(self):
        super().__init__('simple_pure_pursuit')

        # --- パラメータ ---
        self.lookahead_dist = 0.5  # [m]
        self.target_speed = 0.5    # [m/s]
        self.k_p = 1.5             # 旋回ゲイン

        # --- 変数 ---
        self.cx = 0.0
        self.cy = 0.0
        self.cyaw = 0.0
        self.path = None
        
        # ★追加: 前回のインデックスを覚えておく
        self.old_nearest_point_index = None

        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.sub_path = self.create_subscription(Path, '/plan', self.path_callback, 10)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.timer = self.create_timer(0.05, self.control_loop)

    def odom_callback(self, msg):
        self.cx = msg.pose.pose.position.x
        self.cy = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        (_, _, self.cyaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])

    def path_callback(self, msg):
        self.path = msg
        # パスが新しくなったらインデックスをリセット
        self.old_nearest_point_index = None
        self.get_logger().info(f"Received Path with {len(msg.poses)} points.")

    def control_loop(self):
        if self.path is None or len(self.path.poses) == 0:
            return

        # 1. 目標点を探す (修正版)
        target_idx, target_x, target_y = self.search_target_index()
        
        if target_idx is None:
            # ゴールした、または見失った
            cmd = Twist()
            self.pub_cmd.publish(cmd)
            return

        # 2. 方角計算
        angle_to_target = math.atan2(target_y - self.cy, target_x - self.cx)
        
        # 3. 誤差計算
        alpha = angle_to_target - self.cyaw
        while alpha > math.pi: alpha -= 2.0 * math.pi
        while alpha < -math.pi: alpha += 2.0 * math.pi

        # 4. 指令
        twist = Twist()
        twist.linear.x = self.target_speed
        twist.angular.z = alpha * self.k_p
        
        # リミッター (少し緩めました)
        twist.angular.z = max(min(twist.angular.z, 1.5), -1.5)

        self.pub_cmd.publish(twist)

    def search_target_index(self):
        # --- A. 現在地に一番近い点を探す (前回より後ろは見ない) ---
        if self.old_nearest_point_index is None:
            # 初回は全探索
            search_start_idx = 0
        else:
            # 2回目以降は、前回見つけた場所から少しだけ先を探す
            search_start_idx = self.old_nearest_point_index

        min_dist = float('inf')
        nearest_idx = -1

        # パスの最後まで探索
        for i in range(search_start_idx, len(self.path.poses)):
            px = self.path.poses[i].pose.position.x
            py = self.path.poses[i].pose.position.y
            dist = math.sqrt((px - self.cx)**2 + (py - self.cy)**2)
            
            if dist < min_dist:
                min_dist = dist
                nearest_idx = i
            else:
                # 距離が増え始めたら、そこが極小値（一番近い点）なので探索終了していい
                # (パスがループしていない前提)
                pass

        self.old_nearest_point_index = nearest_idx

        # --- B. 一番近い点から前方に進んで、Lookahead距離以上離れた点を探す ---
        target_idx = -1
        for i in range(nearest_idx, len(self.path.poses)):
            px = self.path.poses[i].pose.position.x
            py = self.path.poses[i].pose.position.y
            dist = math.sqrt((px - self.cx)**2 + (py - self.cy)**2)

            if dist > self.lookahead_dist:
                target_idx = i
                return target_idx, px, py
        
        # 最後まで見つからなかった = ゴールが近い
        # 最後の点を返す
        last_idx = len(self.path.poses) - 1
        return last_idx, self.path.poses[last_idx].pose.position.x, self.path.poses[last_idx].pose.position.y

def main():
    rclpy.init()
    node = SimplePurePursuit()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()