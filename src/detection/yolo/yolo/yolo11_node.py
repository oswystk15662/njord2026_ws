"""YOLO11s detector node (standard Ultralytics only).

既存の main.py (YOLOv10 fork ベース) とは独立した並行実装。
- 標準 Ultralytics の `YOLO` クラスのみ使用 (sys.path 挿入・ベンダリング禁止)
- 推論レート制限 / debug 画像レート制限 (fusion 版から移植)
- ROI クロップ (0.0-1.0 正規化、bbox は元画像座標へ復元)
- 出力: Detection2DArray (/yolo/detections), BuoyRoi (/buoy_roi), debug 画像
- HSV 色推定 (fusion 版から移植、クラス名依存文字列はパラメータ化)
- `.engine` パスを渡すと Ultralytics の TensorRT バックエンドで動く
"""

import math
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from sensor_msgs.msg import CameraInfo, Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from njord_interfaces.msg import BuoyRoi
from ament_index_python.packages import get_package_share_directory

import cv2
import numpy as np

# ultralytics が /home 以外へ設定を書こうとして失敗しないように tmp を指定
os.environ.setdefault('MPLCONFIGDIR', os.path.join('/tmp', 'njord_yolo_matplotlib'))
os.environ.setdefault('YOLO_CONFIG_DIR', os.path.join('/tmp', 'njord_yolo_ultralytics'))

try:
    from cv_bridge import CvBridge
    _cv_bridge_import_error = None
except Exception as exc:  # noqa: BLE001
    CvBridge = None
    _cv_bridge_import_error = exc

try:
    from ultralytics import YOLO
    _ultralytics_import_error = None
except Exception as exc:  # noqa: BLE001
    YOLO = None
    _ultralytics_import_error = exc


class Yolo11DetectorNode(Node):
    """YOLO11s 推論ノード。既存 yolo_node と並行して追加された新実装。"""

    def __init__(self, node_name='yolo11_detector'):
        super().__init__(node_name)

        self._validate_runtime_dependencies()
        self._declare_parameters()
        self._read_parameters()

        model_path = self._resolve_model_path()
        self._load_model(model_path)

        # ROS 通信設定
        self.sub_img = self.create_subscription(
            Image, self.camera_topic, self.image_callback, qos_profile_sensor_data)
        self.sub_camera_info = self.create_subscription(
            CameraInfo, self.camera_info_topic, self.camera_info_callback,
            qos_profile_sensor_data)

        self.pub_detections = self.create_publisher(
            Detection2DArray, self.detections_topic, 10)
        self.pub_roi = self.create_publisher(BuoyRoi, self.buoy_roi_topic, 10)
        self.pub_debug_img = self.create_publisher(Image, 'yolo/debug_image', 10)

        self.bridge = CvBridge()
        self.camera_info = None
        self._camera_info_logged = False

        self.min_inference_interval = (
            1.0 / self.inference_hz if self.inference_hz > 0.0 else 0.0)
        self.min_debug_image_interval = (
            1.0 / self.debug_image_hz if self.debug_image_hz > 0.0 else 0.0)
        self.last_inference_time = 0.0
        self.last_debug_image_time = 0.0

        self.get_logger().info(
            f'Yolo11DetectorNode initialized. backend={self.backend}, '
            f'model={model_path}, device={self.device}, imgsz={self.imgsz}, '
            f'roi_crop={"on" if self.use_roi_crop else "off"} '
            f'x[{self.roi_x_min:.2f}, {self.roi_x_max:.2f}] '
            f'y[{self.roi_y_min:.2f}, {self.roi_y_max:.2f}], '
            f'inference_hz={self.inference_hz}, debug_image_hz={self.debug_image_hz}'
        )

    # ------------------------------------------------------------------
    # setup helpers
    # ------------------------------------------------------------------
    def _validate_runtime_dependencies(self):
        if _cv_bridge_import_error is not None:
            raise RuntimeError(
                f'Failed to import cv_bridge: {_cv_bridge_import_error}. '
                'Use system ROS environment for cv_bridge and keep numpy<2.0.'
            )

        if _ultralytics_import_error is not None:
            raise RuntimeError(
                f'Failed to import ultralytics: {_ultralytics_import_error}. '
                'Activate the YOLO venv (see README_jetson_yolo11.md) before '
                'launching this node.'
            )

        self.get_logger().info(
            f'Runtime versions: numpy={np.__version__}, opencv={cv2.__version__}'
        )

        try:
            np_major = int(np.__version__.split('.')[0])
        except Exception:  # noqa: BLE001
            np_major = 0
        if np_major >= 2:
            self.get_logger().warn(
                'Detected numpy>=2.0. cv_bridge and ROS Python packages may break. '
                'Use the Jetson YOLO venv with pinned numpy 1.26.x.'
            )

    def _declare_parameters(self):
        # Model / inference
        self.declare_parameter('model_path', '')
        self.declare_parameter('device', 'cuda:0')
        self.declare_parameter('imgsz', 640)
        self.declare_parameter('conf_threshold', 0.25)
        self.declare_parameter('iou_threshold', 0.45)
        self.declare_parameter('max_det', 20)
        self.declare_parameter(
            'classes', [],
            ParameterDescriptor(
                type=ParameterType.PARAMETER_INTEGER_ARRAY,
                description='Class id filter for inference. Empty = all classes.'))
        self.declare_parameter('use_half', True)
        # backend は記録用 ("pytorch" | "tensorrt")。実際の切替は model_path の拡張子
        # (.pt → PyTorch, .engine → TensorRT) で Ultralytics が自動判別する。
        self.declare_parameter('backend', 'pytorch')

        # Rate limit
        self.declare_parameter('inference_hz', 5.0)
        self.declare_parameter('debug_image_hz', 2.0)
        self.declare_parameter('publish_debug_image', True)

        # ROI crop (0.0-1.0 normalized)
        self.declare_parameter('use_roi_crop', False)
        self.declare_parameter('roi_x_min', 0.0)
        self.declare_parameter('roi_x_max', 1.0)
        self.declare_parameter('roi_y_min', 0.0)
        self.declare_parameter('roi_y_max', 1.0)
        self.declare_parameter('draw_roi_rect', True)

        # Topics
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('detections_topic', '/yolo/detections')
        self.declare_parameter('buoy_roi_topic', '/buoy_roi')

        # BuoyRoi (現行 main.py の値を踏襲)
        self.declare_parameter('roi_frame_id', 'base_link')
        self.declare_parameter('fixed_range_m', 5.0)
        self.declare_parameter('roi_range_half', 2.0)
        self.declare_parameter('camera_fov_deg', 90.0)
        self.declare_parameter('roi_theta_min_deg', 2.0)

        # Color estimation (fusion 版の文字列依存をパラメータ化)
        self.declare_parameter('enable_color_estimation', True)
        self.declare_parameter(
            'class_names', [''],
            ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING_ARRAY,
                description='Optional class-name override by id. Empty = model names.'))
        self.declare_parameter(
            'color_class_map',
            ['red', 'green', 'yellow', 'black', 'white', 'blue', 'orange'],
            ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING_ARRAY,
                description='Color keywords; a class name containing one of these '
                            'is assigned that color without HSV estimation.'))

    def _read_parameters(self):
        gp = self.get_parameter
        self.model_path_param = gp('model_path').get_parameter_value().string_value
        self.device = gp('device').get_parameter_value().string_value
        self.imgsz = gp('imgsz').get_parameter_value().integer_value
        self.conf_threshold = gp('conf_threshold').get_parameter_value().double_value
        self.iou_threshold = gp('iou_threshold').get_parameter_value().double_value
        self.max_det = gp('max_det').get_parameter_value().integer_value
        self.classes = list(gp('classes').get_parameter_value().integer_array_value)
        self.use_half = gp('use_half').get_parameter_value().bool_value
        self.backend = gp('backend').get_parameter_value().string_value

        self.inference_hz = gp('inference_hz').get_parameter_value().double_value
        self.debug_image_hz = gp('debug_image_hz').get_parameter_value().double_value
        self.publish_debug_image = gp('publish_debug_image').get_parameter_value().bool_value

        self.use_roi_crop = gp('use_roi_crop').get_parameter_value().bool_value
        self.roi_x_min = gp('roi_x_min').get_parameter_value().double_value
        self.roi_x_max = gp('roi_x_max').get_parameter_value().double_value
        self.roi_y_min = gp('roi_y_min').get_parameter_value().double_value
        self.roi_y_max = gp('roi_y_max').get_parameter_value().double_value
        self.draw_roi_rect = gp('draw_roi_rect').get_parameter_value().bool_value

        self.camera_topic = gp('camera_topic').get_parameter_value().string_value
        self.camera_info_topic = gp('camera_info_topic').get_parameter_value().string_value
        self.detections_topic = gp('detections_topic').get_parameter_value().string_value
        self.buoy_roi_topic = gp('buoy_roi_topic').get_parameter_value().string_value

        self.roi_frame_id = gp('roi_frame_id').get_parameter_value().string_value
        self.fixed_range_m = gp('fixed_range_m').get_parameter_value().double_value
        self.roi_range_half = gp('roi_range_half').get_parameter_value().double_value
        self.camera_fov_deg = gp('camera_fov_deg').get_parameter_value().double_value
        self.roi_theta_min_deg = gp('roi_theta_min_deg').get_parameter_value().double_value

        self.enable_color_estimation = gp('enable_color_estimation').get_parameter_value().bool_value
        class_names = list(gp('class_names').get_parameter_value().string_array_value)
        # [''] はプレースホルダ (未指定) 扱い
        self.class_names = [n for n in class_names if n]
        self.color_class_map = [
            c.lower() for c in gp('color_class_map').get_parameter_value().string_array_value if c]

    def _resolve_model_path(self):
        """model_path パラメータを解決する。空文字なら <pkg share>/config/yolo11s.pt。"""
        model_path = self.model_path_param
        if not model_path:
            try:
                pkg_share = get_package_share_directory('yolo')
                model_path = os.path.join(pkg_share, 'config', 'yolo11s.pt')
            except Exception as exc:  # noqa: BLE001
                raise RuntimeError(
                    'model_path is empty and the yolo package share directory could '
                    f'not be resolved: {exc}'
                )

        model_path = os.path.expanduser(model_path)
        if not os.path.exists(model_path):
            raise FileNotFoundError(
                f'YOLO11 model file not found: {model_path}. '
                'The weight file is NOT shipped in this repository. '
                'Place the trained yolo11s weights (.pt) or TensorRT engine '
                '(.engine) at this path, or set the model_path parameter / '
                'launch argument to an existing file.'
            )
        return model_path

    def _load_model(self, model_path):
        self.is_tensorrt_engine = model_path.endswith('.engine')
        self.get_logger().info(
            f'Loading YOLO11 model from: {model_path} '
            f'({"TensorRT engine" if self.is_tensorrt_engine else "PyTorch weights"})'
        )
        try:
            # .engine の場合も Ultralytics YOLO() がそのまま TensorRT backend で読む
            self.model = YOLO(model_path)
            if not self.is_tensorrt_engine:
                self.model.to(self.device)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'Failed to load model from {model_path}: {exc}')
            raise

        if self.is_tensorrt_engine and self.backend != 'tensorrt':
            self.get_logger().warn(
                'model_path is a .engine file but backend parameter is '
                f'"{self.backend}". Treating as tensorrt.'
            )
            self.backend = 'tensorrt'

    # ------------------------------------------------------------------
    # callbacks
    # ------------------------------------------------------------------
    def camera_info_callback(self, msg):
        self.camera_info = msg
        if not self._camera_info_logged:
            self._camera_info_logged = True
            self.get_logger().info(
                f'Received CameraInfo: {msg.width}x{msg.height} frame_id={msg.header.frame_id}'
            )

    def image_callback(self, msg):
        now = time.monotonic()
        if (self.min_inference_interval > 0.0
                and now - self.last_inference_time < self.min_inference_interval):
            return
        self.last_inference_time = now

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'CvBridge Error: {exc}')
            return

        roi_image, offset_x, offset_y, roi_rect = self.crop_image_roi(cv_image)

        predict_kwargs = {
            'imgsz': self.imgsz,
            'conf': self.conf_threshold,
            'iou': self.iou_threshold,
            'max_det': self.max_det,
            'verbose': False,
        }
        if self.classes:
            predict_kwargs['classes'] = self.classes
        if not self.is_tensorrt_engine:
            # engine は build 時の精度 (FP16/INT8) が焼き込まれているので half 指定不要
            predict_kwargs['half'] = self.use_half and 'cuda' in self.device

        try:
            results = self.model.predict(roi_image, **predict_kwargs)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'YOLO inference failed: {exc}')
            return
        result = results[0]

        # debug 画像を publish する周期のときだけ描画する (無駄なコピー/描画を避ける)
        debug_due = self.should_publish_debug_image(now)

        detection_array = Detection2DArray()
        detection_array.header = msg.header

        best_roi = None
        best_conf = 0.0
        fov_rad = math.radians(self.camera_fov_deg)
        min_theta_rad = math.radians(self.roi_theta_min_deg)

        for box in result.boxes:
            class_id = int(box.cls[0])
            label = self.lookup_class_name(result, class_id)
            conf = float(box.conf[0])
            xyxy = box.xyxy[0].cpu().numpy()  # ROI 内座標 [x1, y1, x2, y2]
            xyxy = self.shift_bbox_to_full_image(xyxy, offset_x, offset_y)
            color_name = self.estimate_detection_color(cv_image, xyxy, label)

            detection_array.detections.append(
                self.create_detection_msg(msg.header, xyxy, label, conf, color_name))

            # --- BuoyRoi: 画素中心 + FOV から方位角、距離は固定値 (現行 main.py 移植) ---
            cx = (xyxy[0] + xyxy[2]) * 0.5
            estimated_angle = -math.atan2(
                cx - msg.width / 2,
                (msg.width / 2) / math.tan(fov_rad / 2)
            )
            if conf > best_conf:
                bbox_width = max(1.0, float(xyxy[2] - xyxy[0]))
                theta_range = max(
                    min_theta_rad,
                    (bbox_width / msg.width) * fov_rad * 0.5
                )
                best_roi = BuoyRoi()
                best_roi.header.stamp = msg.header.stamp
                best_roi.header.frame_id = self.roi_frame_id
                best_roi.r_predict = float(self.fixed_range_m)
                best_roi.r_range = float(self.roi_range_half)
                best_roi.theta_predict = float(estimated_angle)
                best_roi.theta_range = float(theta_range)
                best_conf = conf

            if debug_due:
                x1, y1, x2, y2 = [int(round(v)) for v in xyxy]
                cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(cv_image, f'{label} {color_name} {conf:.2f}',
                            (x1, max(0, y1 - 10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        self.pub_detections.publish(detection_array)
        if best_roi is not None:
            self.pub_roi.publish(best_roi)

        if debug_due:
            if self.use_roi_crop and self.draw_roi_rect:
                x1, y1, x2, y2 = roi_rect
                cv2.rectangle(cv_image, (x1, y1), (x2, y2), (255, 180, 0), 2)
            self.last_debug_image_time = now
            debug_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            debug_msg.header = msg.header
            self.pub_debug_img.publish(debug_msg)

    # ------------------------------------------------------------------
    # logic helpers (fusion 版から移植)
    # ------------------------------------------------------------------
    def should_publish_debug_image(self, now):
        if not self.publish_debug_image:
            return False
        if self.min_debug_image_interval <= 0.0:
            return True
        return now - self.last_debug_image_time >= self.min_debug_image_interval

    def crop_image_roi(self, image):
        """画像内の監視領域 ROI を切り出す (比率指定 0.0-1.0)。

        戻り値:
          roi_image: YOLO に渡す画像 (crop 無効時は元画像そのもの)
          offset_x/y: ROI 左上の元画像座標
          roi_rect: debug_image に描くための (x1, y1, x2, y2)
        """
        h, w = image.shape[:2]
        if not self.use_roi_crop:
            return image, 0, 0, (0, 0, w - 1, h - 1)

        x_min = self.clamp(self.roi_x_min, 0.0, 1.0)
        x_max = self.clamp(self.roi_x_max, 0.0, 1.0)
        y_min = self.clamp(self.roi_y_min, 0.0, 1.0)
        y_max = self.clamp(self.roi_y_max, 0.0, 1.0)

        x1 = int(round(w * min(x_min, x_max)))
        x2 = int(round(w * max(x_min, x_max)))
        y1 = int(round(h * min(y_min, y_max)))
        y2 = int(round(h * max(y_min, y_max)))

        x1 = max(0, min(w - 1, x1))
        x2 = max(x1 + 1, min(w, x2))
        y1 = max(0, min(h - 1, y1))
        y2 = max(y1 + 1, min(h, y2))

        # numpy view のままだと非連続になるので、crop 時のみ連続配列にする
        roi = np.ascontiguousarray(image[y1:y2, x1:x2])
        return roi, x1, y1, (x1, y1, x2 - 1, y2 - 1)

    @staticmethod
    def shift_bbox_to_full_image(xyxy, offset_x, offset_y):
        """ROI crop 内の bbox 座標を元画像全体の座標へ戻す。"""
        shifted = np.asarray(xyxy, dtype=float).copy()
        shifted[0] += offset_x
        shifted[2] += offset_x
        shifted[1] += offset_y
        shifted[3] += offset_y
        return shifted

    @staticmethod
    def clamp(value, min_value, max_value):
        return max(min_value, min(max_value, float(value)))

    def lookup_class_name(self, result, class_id):
        if 0 <= class_id < len(self.class_names):
            return self.class_names[class_id]
        try:
            return result.names[class_id]
        except Exception:  # noqa: BLE001
            return str(class_id)

    def create_detection_msg(self, header, xyxy, label, confidence, color_name):
        """bbox/class/confidence/color を vision_msgs/Detection2D に詰める。

        color は標準 msg に専用 field がないため、暫定的に detection.id に入れる
        (fusion 側は detection.results[0].hypothesis.class_id と detection.id を読む)。
        """
        x1, y1, x2, y2 = [float(v) for v in xyxy]
        width = max(1.0, x2 - x1)
        height = max(1.0, y2 - y1)

        detection = Detection2D()
        detection.header = header
        detection.id = color_name
        detection.bbox.center.position.x = x1 + width * 0.5
        detection.bbox.center.position.y = y1 + height * 0.5
        detection.bbox.center.theta = 0.0
        detection.bbox.size_x = width
        detection.bbox.size_y = height

        hypothesis = ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id = str(label)
        hypothesis.hypothesis.score = float(confidence)
        detection.results.append(hypothesis)
        return detection

    def estimate_detection_color(self, image_bgr, xyxy, label):
        """bbox 内の HSV から見た目色を推定する (fusion 版から移植)。

        クラス名が color_class_map のいずれかの色名を含む場合は、その色を優先する。
        """
        label_lower = str(label).lower()
        for name in self.color_class_map:
            if name in label_lower:
                return name

        if not self.enable_color_estimation:
            return 'unknown'

        h, w = image_bgr.shape[:2]
        x1, y1, x2, y2 = [int(round(v)) for v in xyxy]
        x1 = max(0, min(w - 1, x1))
        x2 = max(0, min(w, x2))
        y1 = max(0, min(h - 1, y1))
        y2 = max(0, min(h, y2))

        if x2 <= x1 or y2 <= y1:
            return 'unknown'

        crop = image_bgr[y1:y2, x1:x2]
        if crop.size == 0:
            return 'unknown'

        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
        saturation = hsv[:, :, 1]
        value = hsv[:, :, 2]

        valid = (saturation > 50) & (value > 50)
        if np.count_nonzero(valid) < 10:
            mean_v = float(np.mean(value))
            return 'white' if mean_v > 160 else 'black'

        hue = hsv[:, :, 0][valid]
        mean_hue = float(np.median(hue))

        if mean_hue < 10 or mean_hue > 170:
            return 'red'
        if 10 <= mean_hue < 25:
            return 'orange'
        if 25 <= mean_hue < 40:
            return 'yellow'
        if 40 <= mean_hue < 90:
            return 'green'
        if 90 <= mean_hue < 135:
            return 'blue'
        return 'unknown'


def main(args=None):
    rclpy.init(args=args)
    node = Yolo11DetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
