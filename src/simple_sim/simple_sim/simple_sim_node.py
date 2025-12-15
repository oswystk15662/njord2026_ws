import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped, PoseStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import tf_transformations # sudo apt install ros-humble-tf-transformations 必要

class SimpleSimNode(Node):
    def __init__(self):
        super().__init__('simple_sim_node')

        # --- パラメータ ---
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0  # Yaw angle (rad)
        self.vx = 0.0
        self.vth = 0.0
        
        self.last_time = self.get_clock().now()

        # --- 通信設定 ---
        # 1. 指令を受け取る
        self.sub_cmd = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        
        # 2. 現在地(Odom)とTFを配信する
        self.pub_odom = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # 3. 周期実行 (50Hz)
        self.create_timer(0.02, self.update)

        self.get_logger().info("Simple Simulator Started. Ready to move!")

    def cmd_callback(self, msg):
        # 簡易モデル: 指令値がそのまま速度になると仮定（慣性なし）
        self.vx = msg.linear.x
        self.vth = msg.angular.z

    def update(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # --- 運動学モデル (等速運動) ---
        delta_x = (self.vx * math.cos(self.th)) * dt
        delta_y = (self.vx * math.sin(self.th)) * dt
        delta_th = self.vth * dt

        self.x += delta_x
        self.y += delta_y
        self.th += delta_th

        # --- Quaternion変換 ---
        q = tf_transformations.quaternion_from_euler(0, 0, self.th)

        # --- 1. Publish TF (odom -> base_link) ---
        # これがないとNav2は自分の位置がわからない
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)

        # --- 2. Publish Odometry ---
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        # Pose
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = t.transform.rotation

        # Twist (速度情報)
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.angular.z = self.vth

        self.pub_odom.publish(odom)

def main():
    rclpy.init()
    node = SimpleSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()