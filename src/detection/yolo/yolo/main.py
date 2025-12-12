import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import numpy as np
import struct
import math
from ament_index_python.packages import get_package_share_directory # パス解決用

class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector')

        # --- パス解決ロジック ---
        # 1. パッケージのインストール先ディレクトリを取得
        pkg_share = get_package_share_directory('njord_perception')
        
        # 2. デフォルトのモデルパスを作成 (install/share/njord_perception/config/best.pt)
        default_model_path = os.path.join(pkg_share, 'config', 'best.pt')

        # 3. パラメータ宣言 (デフォルト値を設定)
        self.declare_parameter('model_path', default_model_path)
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('camera_topic', '/camera/image_raw')
        
        # 4. パラメータ取得
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        device = self.get_parameter('device').get_parameter_value().string_value
        cam_topic = self.get_parameter('camera_topic').get_parameter_value().string_value

        self.get_logger().info(f'Loading YOLO model from: {model_path}')

        # YOLOモデルのロード
        try:
            self.model = YOLO(model_path)
            self.model.to(device)
        except Exception as e:
            self.get_logger().error(f'Failed to load model from {model_path}: {e}')
            # フォールバック (必要なら)
            # self.model = YOLO("yolov8n.pt") 
            raise e # 起動失敗させる

        # ROS通信設定
        self.sub_img = self.create_subscription(
            Image, cam_topic, self.image_callback, 10)
        
        # デバッグ用画像出力
        self.pub_debug_img = self.create_publisher(Image, 'yolo/debug_image', 10)
        
        # Step 3: Nav2のCostmapに反映させるための仮想障害物（点群）
        self.pub_virtual_wall = self.create_publisher(PointCloud2, '/virtual_obstacles', 10)

        self.bridge = CvBridge()
        self.get_logger().info('YoloDetectorNode Initialized.')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'CvBridge Error: {e}')
            return

        # --- YOLO推論 ---
        results = self.model.predict(cv_image, verbose=False)
        result = results[0]
        
        virtual_obstacles = [] # 生成する点のリスト [[x, y, z], ...]

        # 検出結果のループ
        for box in result.boxes:
            class_id = int(box.cls[0])
            label = result.names[class_id]
            conf = float(box.conf[0])
            xyxy = box.xyxy[0].cpu().numpy() # [x1, y1, x2, y2]
            
            # 中心座標
            cx = int((xyxy[0] + xyxy[2]) / 2)
            cy = int((xyxy[1] + xyxy[3]) / 2)

            # --- 距離・位置推定ロジック (簡易版) ---
            # 本来はここで main_yolo.py のようなステレオ/LiDARフュージョンを行う
            # 一旦、仮の距離として「バウンディングボックスの大きさ」や「Y座標」から推定
            estimated_dist = 5.0 # [m] (仮置き)
            estimated_angle = -math.atan2(cx - msg.width/2, (msg.width/2) / math.tan(math.radians(45))) # [rad] (FOV90度仮定)

            # ロボット座標系(base_link)でのブイの位置 (x:前, y:左)
            buoy_x = estimated_dist * math.cos(estimated_angle)
            buoy_y = estimated_dist * math.sin(estimated_angle)

            # --- Step 3: 方位標識ロジック ---
            # 方位標識の種類に応じて「通ってはいけない側」に壁を作る
            wall_points = self.generate_virtual_wall(label, buoy_x, buoy_y)
            if wall_points:
                virtual_obstacles.extend(wall_points)
                
                # デバッグ描画: 壁の方向へ線を引く
                cv2.line(cv_image, (cx, cy), (cx, cy+50), (0, 0, 255), 3)

            # デバッグ描画: BBox
            cv2.rectangle(cv_image, (int(xyxy[0]), int(xyxy[1])), (int(xyxy[2]), int(xyxy[3])), (0, 255, 0), 2)
            cv2.putText(cv_image, f'{label} {conf:.2f}', (int(xyxy[0]), int(xyxy[1])-10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # 仮想壁（点群）のPublish
        if virtual_obstacles:
            pc_msg = self.create_pointcloud2(virtual_obstacles)
            self.pub_virtual_wall.publish(pc_msg)
        else:
            # 障害物がない時は空のデータを送る（前の壁を消すため）
            pc_msg = self.create_pointcloud2([])
            self.pub_virtual_wall.publish(pc_msg)

        # デバッグ画像のPublish
        self.pub_debug_img.publish(self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8'))

    def generate_virtual_wall(self, label, bx, by):
        """
        ブイの種類に応じて、危険領域（壁）の点群を生成する
        bx, by: ロボットから見たブイの位置 (m)
        """
        points = []
        radius = 2.0 # ブイから半径何mを壁にするか
        density = 10 # 点の密度

        # 北方位標識 (North Cardinal) -> 北側が安全 = 南側(手前側)に通れない壁を作る
        if 'north' in label.lower():
            # ブイの南側(ロボット側)に半円状の壁
            # 方位: 南 = -90度(右) 〜 +90度(左) ... 座標系に合わせて調整が必要
            # ここではシンプルに「ブイを中心に、Y軸マイナス方向(南とする)に壁」と仮定
            # ※ 本来は「絶対方位」が必要ですが、カメラ画像だけだと「画面の下＝南」とは限らないため、
            #    GNSS/CompassのHeadingと組み合わせて「世界座標の南」を計算するのがベストです。
            #    ここでは簡易的に「ブイの手前」を塞ぎます。
            for i in range(density):
                angle = math.pi + (math.pi * i / density) # 半円
                px = bx + radius * math.cos(angle)
                py = by + radius * math.sin(angle)
                points.append([px, py, 0.0])

        elif 'red' in label.lower():
            # 赤ブイ -> 欧州方式(IALA A)では「左舷側に見て通る」= ブイの右側が危険
            # 右側に壁を作る
            for i in range(density):
                px = bx
                py = by - (i * 0.5) # 右(yマイナス)へ伸ばす
                points.append([px, py, 0.0])
        
        # ... 他の緑ブイ、南標識なども同様に追加

        return points

    def create_pointcloud2(self, points):
        """
        [[x,y,z], ...] のリストから PointCloud2 メッセージを作成する
        """
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link" # ロボット基準で壁を置く

        msg.height = 1
        msg.width = len(points)
        
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12 * len(points)
        msg.is_dense = True

        buffer = []
        for p in points:
            buffer.append(struct.pack('fff', p[0], p[1], p[2]))
        
        msg.data = b''.join(buffer)
        return msg

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()