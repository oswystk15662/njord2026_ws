import os
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, PointCloud2, PointField
from njord_interfaces.msg import BuoyRoi
import cv2
import numpy as np
import struct
import math
from ament_index_python.packages import get_package_share_directory # パス解決用

try:
    from cv_bridge import CvBridge
    _cv_bridge_import_error = None
except Exception as exc:
    CvBridge = None
    _cv_bridge_import_error = exc

try:
    from ultralytics import YOLO
    _ultralytics_import_error = None
except Exception as exc:
    YOLO = None
    _ultralytics_import_error = exc


class _LatestFrameBuffer:
    """Single-slot blocking buffer that replaces stale, unprocessed frames."""

    def __init__(self):
        self._condition = threading.Condition()
        self._latest = None
        self._stopping = False

    def put(self, frame):
        with self._condition:
            if self._stopping:
                return False
            self._latest = frame
            self._condition.notify()
            return True

    def take(self):
        with self._condition:
            self._condition.wait_for(
                lambda: self._stopping or self._latest is not None
            )
            if self._stopping:
                return None
            frame = self._latest
            self._latest = None
            return frame

    def stop(self):
        with self._condition:
            self._stopping = True
            self._latest = None
            self._condition.notify_all()


class YoloDetectorNode(Node):
    def __init__(self, node_name='yolo_detector', device_default='cpu'):
        super().__init__(node_name)

        self._validate_runtime_dependencies()

        # --- パス解決ロジック ---
        # 1. デフォルトのモデルパスを解決
        default_model_path = self._resolve_default_model_path()

        # 3. パラメータ宣言 (デフォルト値を設定)
        self.declare_parameter('model_path', default_model_path)
        self.declare_parameter('device', device_default)
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('enable_virtual_wall', False)
        self.declare_parameter('enable_roi', True)
        self.declare_parameter('roi_topic', '/buoy_roi')
        self.declare_parameter('roi_frame_id', 'base_link')
        self.declare_parameter('roi_range_predict', 5.0)
        self.declare_parameter('roi_range_half', 2.0)
        self.declare_parameter('camera_fov_deg', 90.0)
        self.declare_parameter('roi_theta_min_deg', 2.0)
        
        # 4. パラメータ取得
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        device = self.get_parameter('device').get_parameter_value().string_value
        cam_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        self.enable_virtual_wall = self.get_parameter('enable_virtual_wall').get_parameter_value().bool_value
        self.enable_roi = self.get_parameter('enable_roi').get_parameter_value().bool_value
        self.roi_topic = self.get_parameter('roi_topic').get_parameter_value().string_value
        self.roi_frame_id = self.get_parameter('roi_frame_id').get_parameter_value().string_value
        self.roi_range_predict = self.get_parameter('roi_range_predict').get_parameter_value().double_value
        self.roi_range_half = self.get_parameter('roi_range_half').get_parameter_value().double_value
        self.camera_fov_deg = self.get_parameter('camera_fov_deg').get_parameter_value().double_value
        self.roi_theta_min_deg = self.get_parameter('roi_theta_min_deg').get_parameter_value().double_value

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

        self._frames = _LatestFrameBuffer()

        # ROS通信設定
        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.sub_img = self.create_subscription(
            Image, cam_topic, self.image_callback, image_qos)
        
        # デバッグ用画像出力
        self.pub_debug_img = self.create_publisher(Image, 'yolo/debug_image', 10)
        
        # Step 3: Nav2のCostmapに反映させるための仮想障害物（点群）
        self.pub_virtual_wall = self.create_publisher(PointCloud2, '/virtual_obstacles', 10)

        self.pub_roi = self.create_publisher(BuoyRoi, self.roi_topic, 10)

        self.bridge = CvBridge()
        self._inference_thread = threading.Thread(
            target=self._inference_loop,
            name=f'{node_name}_inference',
            daemon=True,
        )
        self._inference_thread.start()
        self.get_logger().info('YoloDetectorNode Initialized.')

    def _validate_runtime_dependencies(self):
        if _cv_bridge_import_error is not None:
            raise RuntimeError(
                f'Failed to import cv_bridge: {_cv_bridge_import_error}. '
                'Use system ROS environment for cv_bridge and keep numpy<2.0.'
            )

        if _ultralytics_import_error is not None:
            raise RuntimeError(
                f'Failed to import ultralytics: {_ultralytics_import_error}. '
                'Activate YOLO venv before launching this node.'
            )

        self.get_logger().info(
            f'Runtime versions: numpy={np.__version__}, opencv={cv2.__version__}'
        )

        try:
            np_major = int(np.__version__.split('.')[0])
        except Exception:
            np_major = 0

        if np_major >= 2:
            self.get_logger().warn(
                'Detected numpy>=2.0. cv_bridge and ROS Python packages may break. '
                'Use Jetson YOLO venv with pinned numpy 1.x.'
            )

    def _resolve_default_model_path(self):
        try:
            pkg_share = get_package_share_directory('yolo')
            model_path = os.path.join(pkg_share, 'config', 'best.pt')
            if os.path.exists(model_path):
                return model_path
        except Exception:
            pass

        self.get_logger().warn(
            'Could not resolve installed model path from package share. '
            'Falling back to yolov8n.pt.'
        )
        return 'yolov8n.pt'

    def image_callback(self, msg):
        if not self.enable_virtual_wall and not self.enable_roi:
            return

        # Replace an unprocessed frame instead of queueing stale camera data.
        self._frames.put(msg)

    def _inference_loop(self):
        while True:
            msg = self._frames.take()
            if msg is None:
                return
            try:
                self._process_image(msg)
            except Exception as exc:
                self.get_logger().error(f'YOLO inference failed: {exc}')

    def _process_image(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'CvBridge Error: {e}')
            return

        # --- YOLO推論 ---
        results = self.model.predict(cv_image, verbose=False)
        result = results[0]
        
        virtual_obstacles = [] # 生成する点のリスト [[x, y, z], ...]
        best_roi = None
        best_conf = 0.0
        fov_rad = math.radians(self.camera_fov_deg)
        min_theta_rad = math.radians(self.roi_theta_min_deg)
        publish_debug = self.pub_debug_img.get_subscription_count() > 0

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
            estimated_dist = self.roi_range_predict # [m] (仮置き)
            estimated_angle = -math.atan2(
                cx - msg.width / 2,
                (msg.width / 2) / math.tan(fov_rad / 2)
            )

            # ロボット座標系(base_link)でのブイの位置 (x:前, y:左)
            buoy_x = estimated_dist * math.cos(estimated_angle)
            buoy_y = estimated_dist * math.sin(estimated_angle)

            # --- Step 3: 方位標識ロジック ---
            # 方位標識の種類に応じて「通ってはいけない側」に壁を作る
            if self.enable_virtual_wall:
                wall_points = self.generate_virtual_wall(label, buoy_x, buoy_y)
                if wall_points:
                    virtual_obstacles.extend(wall_points)

                    # デバッグ描画: 壁の方向へ線を引く
                    if publish_debug:
                        cv2.line(cv_image, (cx, cy), (cx, cy+50), (0, 0, 255), 3)

            if self.enable_roi and conf > best_conf:
                bbox_width = max(1.0, float(xyxy[2] - xyxy[0]))
                theta_range = max(
                    min_theta_rad,
                    (bbox_width / msg.width) * fov_rad * 0.5
                )
                best_roi = BuoyRoi()
                best_roi.header.stamp = msg.header.stamp
                best_roi.header.frame_id = self.roi_frame_id
                best_roi.r_predict = float(estimated_dist)
                best_roi.r_range = float(self.roi_range_half)
                best_roi.theta_predict = float(estimated_angle)
                best_roi.theta_range = float(theta_range)
                best_conf = conf

            # デバッグ描画: BBox
            if publish_debug:
                cv2.rectangle(
                    cv_image,
                    (int(xyxy[0]), int(xyxy[1])),
                    (int(xyxy[2]), int(xyxy[3])),
                    (0, 255, 0),
                    2,
                )
                cv2.putText(
                    cv_image,
                    f'{label} {conf:.2f}',
                    (int(xyxy[0]), int(xyxy[1])-10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    2,
                )

        # 仮想壁（点群）のPublish
        if self.enable_virtual_wall:
            if virtual_obstacles:
                pc_msg = self.create_pointcloud2(virtual_obstacles)
                self.pub_virtual_wall.publish(pc_msg)
            else:
                # 障害物がない時は空のデータを送る（前の壁を消すため）
                pc_msg = self.create_pointcloud2([])
                self.pub_virtual_wall.publish(pc_msg)

        if self.enable_roi and best_roi is not None:
            self.pub_roi.publish(best_roi)

        # デバッグ画像のPublish
        if publish_debug:
            self.pub_debug_img.publish(self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8'))

    def destroy_node(self):
        self._frames.stop()
        self._inference_thread.join()
        return super().destroy_node()

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

def run_node(args=None, node_name='yolo_detector', device_default='cpu'):
    rclpy.init(args=args)
    node = YoloDetectorNode(node_name=node_name, device_default=device_default)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


def main(args=None):
    run_node(args=args, node_name='yolo_detector', device_default='cpu')


def cuda_main(args=None):
    run_node(args=args, node_name='yolo_detector_cuda', device_default='cuda:0')

if __name__ == '__main__':
    main()
