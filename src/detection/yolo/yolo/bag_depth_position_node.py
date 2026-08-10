"""Offline buoy-position checker for ZED rosbag replay.

This intentionally publishes a separate topic from the live Task 1 detector.
It runs the configured YOLO model on a replayed image, samples the central
part of each box in the replayed ZED depth image, and publishes the resulting
camera-frame 3-D point as a BuoyDetectionArray.
"""

import math
import os

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from njord_interfaces.msg import BuoyDetection, BuoyDetectionArray
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

try:
    from ultralytics import YOLO
except ImportError as exc:  # Clear startup error when the Jetson venv is absent.
    YOLO = None
    _ULTRALYTICS_ERROR = exc
else:
    _ULTRALYTICS_ERROR = None


class BagDepthPositionNode(Node):
    def __init__(self):
        super().__init__('bag_depth_position_checker')
        share = get_package_share_directory('yolo')
        self.declare_parameter('model_path', os.path.join(share, 'config', 'best.pt'))
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('image_topic', '/zed/zed_node/left/image_rect_color')
        self.declare_parameter('depth_topic', '/zed/zed_node/depth/depth_registered')
        self.declare_parameter('camera_info_topic', '/zed/zed_node/left/camera_info')
        self.declare_parameter('output_topic', '/bag_replay/buoy_detections_3d')
        self.declare_parameter('debug_image_topic', '/bag_replay/buoy_debug_image')
        self.declare_parameter('confidence_threshold', 0.25)
        self.declare_parameter('depth_center_ratio', 0.5)
        self.declare_parameter('min_valid_depth_samples', 16)
        self.declare_parameter('depth_min_m', 0.3)
        self.declare_parameter('depth_max_m', 20.0)

        if YOLO is None:
            raise RuntimeError(f'ultralytics is unavailable: {_ULTRALYTICS_ERROR}')
        self.bridge = CvBridge()
        self.depth = None
        self.camera_info = None
        model_path = self.get_parameter('model_path').value or os.path.join(share, 'config', 'best.pt')
        self.model = YOLO(model_path)
        device = self.get_parameter('device').value
        if not str(model_path).endswith('.engine'):
            self.model.to(device)
        self.output = self.create_publisher(BuoyDetectionArray, self.get_parameter('output_topic').value, 10)
        self.debug = self.create_publisher(Image, self.get_parameter('debug_image_topic').value, 10)
        self.create_subscription(Image, self.get_parameter('depth_topic').value, self._depth_cb, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, self.get_parameter('camera_info_topic').value, self._info_cb, qos_profile_sensor_data)
        self.create_subscription(Image, self.get_parameter('image_topic').value, self._image_cb, qos_profile_sensor_data)

    def _depth_cb(self, msg):
        try:
            self.depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough').astype(np.float32, copy=False)
        except Exception as exc:
            self.get_logger().warn(f'Ignoring depth image: {exc}')

    def _info_cb(self, msg):
        self.camera_info = msg

    def _image_cb(self, msg):
        if self.depth is None or self.camera_info is None:
            return
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            result = self.model(image, verbose=False, conf=float(self.get_parameter('confidence_threshold').value))[0]
        except Exception as exc:
            self.get_logger().error(f'YOLO inference failed: {exc}')
            return
        out = BuoyDetectionArray()
        annotated = image.copy()
        k = self.camera_info.k
        fx, fy, cx, cy = k[0], k[4], k[2], k[5]
        ratio = float(self.get_parameter('depth_center_ratio').value)
        min_samples = int(self.get_parameter('min_valid_depth_samples').value)
        dmin, dmax = float(self.get_parameter('depth_min_m').value), float(self.get_parameter('depth_max_m').value)
        h, w = self.depth.shape[:2]
        for box in result.boxes:
            x1, y1, x2, y2 = map(float, box.xyxy[0].tolist())
            u, v = (x1 + x2) * .5, (y1 + y2) * .5
            rw, rh = max(1, int((x2 - x1) * ratio)), max(1, int((y2 - y1) * ratio))
            xa, xb = max(0, int(u - rw / 2)), min(w, int(u + rw / 2))
            ya, yb = max(0, int(v - rh / 2)), min(h, int(v + rh / 2))
            values = self.depth[ya:yb, xa:xb]
            values = values[np.isfinite(values) & (values >= dmin) & (values <= dmax)]
            det = BuoyDetection()
            det.class_id = int(box.cls[0])
            det.confidence = float(box.conf[0])
            det.header = msg.header
            if values.size >= min_samples and fx > 0.0 and fy > 0.0:
                z = float(np.median(values))
                det.position.x, det.position.y, det.position.z = (u - cx) * z / fx, (v - cy) * z / fy, z
                det.position_source = BuoyDetection.POSITION_ZED_DEPTH
                cv2.putText(annotated, f'{z:.2f}m', (int(x1), max(0, int(y1)-5)), cv2.FONT_HERSHEY_SIMPLEX, .5, (0,255,0), 1)
            else:
                det.position.x = det.position.y = det.position.z = math.nan
                det.position_source = BuoyDetection.POSITION_NONE
            out.detections.append(det)
            cv2.rectangle(annotated, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
        self.output.publish(out)
        self.debug.publish(self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8'))


def main():
    rclpy.init()
    node = BagDepthPositionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
