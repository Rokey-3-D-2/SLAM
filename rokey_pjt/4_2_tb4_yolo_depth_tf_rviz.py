import rclpy
from rclpy.node import Node
from rclpy.time import Time as RclTime
from rclpy.executors import MultiThreadedExecutor
import rclpy.time
from rokey_pjt_interfaces.msg import AnomalyReport
from sensor_msgs.msg import Image, CameraInfo
from rokey_pjt_interfaces.srv import TransformPoint
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge
import tf2_ros
import tf2_geometry_msgs
import numpy as np
import cv2
import threading
import time
import os
import sys
from ultralytics import YOLO
import torch

# === 상수 정의 ===
MODEL_PATH = '/home/rokey/rokey_ws/model/best.pt'
# TARGET_CLASS_ID = 0
INFERENCE_PERIOD_SEC = 1.0 / 20  # 20Hz 추론 주기
INIT_LOADING_TIME = 2.0

BASE_LINK = 'base_link'
XYZ = ['x', 'y', 'z']
make_xyz_dict = lambda x: {k:v for k, v in zip(XYZ, x)}

# Topic
RGB_TOPIC = '/robot1/oakd/rgb/preview/image_raw'
DEPTH_TOPIC = '/robot1/oakd/stereo/image_raw'
CAMERA_INFO_TOPIC = '/robot1/oakd/stereo/camera_info'
MARKER_TOPIC = '/robot1/detected_objects_marker'
ERROR_TOPIC = '/robot1/error_detected'

# Service
TF_SERVICE = '/robot1/transform_to_map'

class YoloDepthToMap(Node):
    def __init__(self):
        super().__init__('yolo_depth_to_map')
        self.get_logger().info("[1/6] 노드 초기화 시작...")

        if not os.path.exists(MODEL_PATH):
            self.get_logger().error(f"Model file not found: {MODEL_PATH}")
            sys.exit(1)

        self.model = YOLO(MODEL_PATH)
        self.model.to('cuda' if torch.cuda.is_available() else 'cpu')
        self.get_logger().info(f"[2/6] YOLO 모델 로드 완료 (GPU 사용: {torch.cuda.is_available()})")

        self.bridge = CvBridge()
        self.classNames = getattr(self.model, 'names', [])

        self.K = None
        self.latest_rgb = self.latest_depth = self.latest_rgb_msg = None
        self.overlay_info = []
        self.display_rgb = None
        self.lock = threading.Lock()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        time.sleep(INIT_LOADING_TIME)
        self.get_logger().info("[3/6] TF2 Transform Listener 초기화 완료")

        self.create_subscription(Image, RGB_TOPIC, self.rgb_callback, 1)
        self.create_subscription(Image, DEPTH_TOPIC, self.depth_callback, 1)
        self.create_subscription(CameraInfo, CAMERA_INFO_TOPIC, self.camera_info_callback, 1)
        self.get_logger().info(f"[4/6] 토픽 구독 완료:\n  RGB: {RGB_TOPIC}\n  Depth: {DEPTH_TOPIC}\n  CameraInfo: {CAMERA_INFO_TOPIC}")

        self.marker_pub = self.create_publisher(Marker, MARKER_TOPIC, 10)
        self.marker_id = 0
        self.error_pub = self.create_publisher(AnomalyReport, ERROR_TOPIC, 10)
        self.get_logger().info(f"[5/6] 퍼블리셔 설정 완료\n  MAKER: {MARKER_TOPIC}\n  ERROR: {ERROR_TOPIC}")

        self.tf_service = self.create_client(TransformPoint, TF_SERVICE)
        self.get_logger().info(f"[6/6] 서비스 설정 완료: {TF_SERVICE}")

        self.create_timer(INFERENCE_PERIOD_SEC, self.inference_callback)

    def camera_info_callback(self, msg):
        if self.K is None:
            self.K = np.array(msg.k).reshape(3, 3)
            self.get_logger().info(f"CameraInfo 수신: fx={self.K[0,0]:.2f}, fy={self.K[1,1]:.2f}, cx={self.K[0,2]:.2f}, cy={self.K[1,2]:.2f}")

    def rgb_callback(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            with self.lock:
                self.latest_rgb = img.copy()
                self.latest_rgb_msg = msg
                self.display_rgb = img.copy()
        except Exception as e:
            self.get_logger().error(f"RGB conversion error: {e}")

    def depth_callback(self, msg):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
            with self.lock:
                self.latest_depth = depth
        except Exception as e:
            self.get_logger().error(f"Depth conversion error: {e}")

    def set_point(self, frame_id, x, y, z):
        point = PointStamped()
        point.header.frame_id = frame_id
        point.header.stamp = rclpy.time.Time().to_msg()
        point.point.x, point.point.y, point.point.z = x, y, z
        return point
    
    def transform_to_map(self, point: PointStamped, class_name: str):
        try:
            map = self.tf_buffer.transform(point, 'map', timeout=rclpy.duration.Duration(seconds=1.0))
            x, y, z = map.point.x, map.point.y, map.point.z
            self.get_logger().info(f"[TF] {class_name} → map: (x={x:.2f}, y={y:.2f}, z={z:.2f})")
            return x, y, z
        except Exception as e:
            self.get_logger().warn(f"[TF] class={class_name} 변환 실패: {e}")
            return float('nan'), float('nan'), float('nan')

    def publish_marker(self, x, y, z, label):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'detected_objects'
        marker.id = self.marker_id
        marker.text = label
        self.marker_id += 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.2
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.lifetime.sec = 3
        self.marker_pub.publish(marker)
    
    def publish_error(self, base:dict, obj:dict, label:str, img):
        log = f"Error Detected : {label}"
        img_msg = self.bridge.cv2_to_imgmsg(img, 'bgr8')

        # 메시지 구성
        msg = AnomalyReport()
        msg.anomaly_label = label
        msg.description = log

        msg.robot_x = base.get("x", 0.0)
        msg.robot_y = base.get("y", 0.0)
        msg.robot_z = base.get("z", 0.0)

        msg.object_x = obj.get("x", 0.0)
        msg.object_y = obj.get("y", 0.0)
        msg.object_z = obj.get("z", 0.0)

        msg.timestamp = RclTime().to_msg()
        msg.image = img_msg

        self.error_pub.publish(msg)

    def inference_callback(self):
        with self.lock:
            rgb, depth, K, rgb_msg = self.latest_rgb, self.latest_depth, self.K, self.latest_rgb_msg

        if any(v is None for v in (rgb, depth, K, rgb_msg)):
            return

        results = self.model(rgb)
        overlay_info = []

        for result in results:
            if result.boxes is None:
                continue
            for box in result.boxes:
                cls = int(box.cls[0])
                # if cls != TARGET_CLASS_ID:
                #     continue

                u, v = map(int, box.xywh[0][:2].cpu().numpy())
                if not (0 <= v < depth.shape[0] and 0 <= u < depth.shape[1]):
                    continue

                z = float(depth[v, u]) / 1000.0
                if z <= 0.05 or np.isnan(z):
                    continue

                fx, fy = K[0, 0], K[1, 1]
                cx, cy = K[0, 2], K[1, 2]
                x = (u - cx) * z / fx
                y = (v - cy) * z / fy

                x1, y1, x2, y2 = map(int, box.xyxy[0].cpu().numpy())
                conf = float(box.conf[0])
                label = self.classNames[cls] if cls < len(self.classNames) else f'class_{cls}'

                # object 기준 포인트 생성
                obj = self.set_point(rgb_msg.header.frame_id, x, y, z)
                obj_x, obj_y, obj_z = self.transform_to_map(obj, label)
                if not np.isnan(obj_x):
                    self.publish_marker(obj_x, obj_y, obj_z, label)

                if 'err' in label:
                    # base_link 기준 포인트 생성
                    base = self.set_point(BASE_LINK, 0.0, 0.0, 0.0)
                    base_x, base_y, base_z = self.transform_to_map(base, BASE_LINK)
                    
                    obj = make_xyz_dict([obj_x, obj_y, obj_z])
                    base = make_xyz_dict([base_x, base_y, base_z])
                    self.get_logger().info(base)
                    img = self.display_rgb
                    self.publish_error(base, obj, label, img)

                overlay_info.append({
                    "label": label,
                    "conf": conf,
                    "center": (u, v),
                    "bbox": (x1, y1, x2, y2),
                    "depth": z
                })

        with self.lock:
            self.overlay_info = overlay_info

import sys
def main():
    rclpy.init()
    node = YoloDepthToMap()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    try:
        while rclpy.ok():
            with node.lock:
                frame = node.display_rgb.copy() if node.display_rgb is not None else None
                overlay_info = node.overlay_info.copy()

            if frame is not None:
                for obj in overlay_info:
                    if obj["conf"] < 0.75:
                        continue
                    u, v = obj["center"]
                    x1, y1, x2, y2 = obj["bbox"]
                    label = obj["label"]
                    conf = obj["conf"]
                    z = obj["depth"]
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.circle(frame, (u, v), 4, (0, 0, 255), -1)
                    cv2.putText(frame, f"{label} {conf:.2f} {z:.2f}m", (x1, y1 - 5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

                display_img = cv2.resize(frame, (frame.shape[1]*3, frame.shape[0]*3))
                cv2.imshow("YOLO + Depth + TF", display_img)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                node.get_logger().info("Shutdown requested by user.")
                break
            time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()
        print("Shutdown complete.")
        sys.exit(0)

if __name__ == '__main__':
    main()
