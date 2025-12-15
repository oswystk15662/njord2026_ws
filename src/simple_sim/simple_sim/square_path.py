import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import math

class TestPathPub(Node):
    def __init__(self):
        super().__init__('test_path_pub')
        self.pub = self.create_publisher(Path, '/plan', 10)
        self.timer = self.create_timer(1.0, self.publish_path)

    def publish_path(self):
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"

        # 10m四方の四角形を通るウェイポイント（四隅）
        corners = [
            (0.0, 0.0),
            (5.0, 0.0),
            (5.0, 5.0),
            (0.0, 5.0),
            (0.0, 0.0)
        ]

        # ★修正ポイント: 角と角の間を細かく埋める (補間)
        resolution = 0.1  # 10cmごとに点を打つ
        
        for i in range(len(corners) - 1):
            start = corners[i]
            end = corners[i+1]
            
            dist = math.sqrt((end[0]-start[0])**2 + (end[1]-start[1])**2)
            num_points = int(dist / resolution)
            
            for j in range(num_points):
                # 線形補間
                ratio = j / num_points
                x = start[0] + (end[0] - start[0]) * ratio
                y = start[1] + (end[1] - start[1]) * ratio
                
                pose = PoseStamped()
                pose.header = msg.header
                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.orientation.w = 1.0
                msg.poses.append(pose)

        # 最後の点も追加
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose.position.x = corners[-1][0]
        pose.pose.position.y = corners[-1][1]
        pose.pose.orientation.w = 1.0
        msg.poses.append(pose)

        self.pub.publish(msg)
        self.get_logger().info(f"Published Dense Path with {len(msg.poses)} points!")
        self.timer.cancel()

def main():
    rclpy.init()
    node = TestPathPub()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()