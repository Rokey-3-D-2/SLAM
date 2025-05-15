import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped

from ultralytics import YOLO

import numpy as np

import cv2
from cv_bridge import CvBridge

import tf2_ros
import tf2_geometry_msgs  # 꼭 필요

import math, os, sys, threading


# ========================
# 상수 정의
# ========================
MODEL_PATH = "/home/we/rokey_ws/model/best.pt"  # YOLO 모델 경로 경로
IMAGE_TOPIC = "/robot1/oakd/rgb/preview/image_raw"  # 구독할 이미지 토픽 이름
DEPTH_TOPIC = "/robot1/oakd/stereo/image_raw"  # Depth 이미지 토픽
CAMERA_INFO_TOPIC = "/robot1/oakd/stereo/camera_info"  # CameraInfo 토픽

# TARGET_CLASS_ID = 10


# ========================
# YOLO + Depth + TF 노드
# ========================
class YoloDepthTfNode(Node):

    def __init__(self):
        super().__init__("yolo_depth_tf_node")
        self.get_logger().info("YOLO + Depth + TF 출력 노드 시작")

        # load yolo model
        if not os.path.exists(MODEL_PATH):
            self.get_logger().error(f"Model not found: {MODEL_PATH}")
            sys.exit(1)
        self.model = YOLO(MODEL_PATH)
        self.class_names = getattr(self.model, "name", [])  # class names
        self.get_logger().info(f"class names :", self.class_names)

        # OpenCV setting
        self.window_name = "YOLO_Depth_TF"  # OpenCV 창 이름
        self.bridge = CvBridge()  # Topic msg → OpenCV
        self.K = None  # Depth 카메라 내부 파라미터 행렬 [3, 3]
        self.depth_image = None
        self.rgb_image = None
        self.distance_m = None
        self.lock = threading.Lock()  # thread lock

        # TF Buffer와 Listener 준비
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscribers
        self.create_subscription(
            CameraInfo, CAMERA_INFO_TOPIC, self.info_callback, 10
        )  # info subscriber
        self.create_subscription(
            Image, DEPTH_TOPIC, self.depth_callback, 10
        )  # depth subscriber
        self.create_subscription(
            Image, IMAGE_TOPIC, self.rgb_callback, 10
        )  # rgb subscriber

        # YOLO + 거리 출력 루프 실행
        threading.Thread(target=self.processing_loop, daemon=True).start()

        # 5초 후에 변환 시작
        self.get_logger().info("TF Tree 안정화 시작. 5초 후 변환 시작합니다.")
        self.start_timer = self.create_timer(5.0, self.start_transform)

    def info_callback(self, msg):
        if self.K is None:
            self.K = np.array(msg.k).reshape(3, 3)
            self.get_logger().info(
                f"CameraInfo received: fx={self.K[0,0]:.2f}, fy={self.K[1,1]:.2f}, cx={self.K[0,2]:.2f}, cy={self.K[1,2]:.2f}"
            )

    def depth_callback(self, msg):
        with self.lock:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, "passthrough")

    def rgb_callback(self, msg):
        with self.lock:
            self.rgb_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def start_transform(self):
        self.get_logger().info("TF Tree 안정화 완료. 변환 시작합니다.")

        # 주기적 변환 타이머 등록
        self.transform_timer = self.create_timer(2.0, self.timer_callback)

        # 시작 타이머 중지 (한 번만 실행)
        self.start_timer.cancel()

    def timer_callback(self):
        if self.distance_m == None:
            self.get_logger().info("Distance is None")
            return

        try:
            # base_link 기준 포인트 생성
            point_base = PointStamped()
            point_base.header.stamp = rclpy.time.Time().to_msg()
            point_base.header.frame_id = "base_link"
            point_base.point.x = self.distance_m
            point_base.point.y = 0.0
            point_base.point.z = 0.0

            # base_link → map 변환
            try:
                point_map = self.tf_buffer.transform(
                    point_base, "map", timeout=rclpy.duration.Duration(seconds=0.5)
                )
                self.get_logger().info(
                    f"[TF] image → map ({point_map.point.x:.2f}, {point_map.point.y:.2f}, {point_map.point.z:.2f})"
                )
            except Exception as e:
                self.get_logger().warn(f"TF transform to map failed: {e}")

        except Exception as e:
            self.get_logger().warn(f"Unexpected error: {e}")

    def processing_loop(self):
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)

        while rclpy.ok():
            with self.lock:
                if self.rgb_image is None or self.depth_image is None or self.K is None:
                    continue
                rgb = self.rgb_image.copy()
                depth = self.depth_image.copy()
            self.distance_m = None

            # model inference
            results = self.model(rgb, stream=True)

            for r in results:
                for box in r.boxes:
                    cls = int(box.cls[0])
                    # if cls < TARGET_CLASS_ID: continue

                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    u, v = (x1 + x2) // 2, (y1 + y2) // 2  # 중심 좌표

                    if not (0 <= u < depth.shape[1] and 0 <= v < depth.shape[0]):
                        continue

                    conf = math.ceil(box.conf[0] * 100) / 100  # score
                    label = (
                        self.class_names[cls]
                        if cls < len(self.class_names)
                        else f"class_{cls}"
                    )

                    # 거리 계산 (mm → m)
                    val = depth[v, u]
                    if depth.dtype == np.uint16:
                        distance_m = val / 1000.0
                    else:
                        distance_m = float(val)
                    self.distance_m = distance_m

                    # self.get_logger().info(f"{label} at ({u},{v}) → {distance_m:.2f}m")

                    # RGB 이미지 위 시각화
                    cv2.rectangle(
                        rgb, (x1, y1), (x2, y2), (0, 255, 0), 2
                    )  # object detection
                    cv2.circle(rgb, (u, v), 4, (0, 0, 255), -1)  # depth
                    cv2.putText(
                        rgb,
                        f"{label}: {conf}, {distance_m:.2f}m",
                        (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.8,
                        (255, 0, 0),
                        2,
                    )

            # 시각화를 위해 2배 확대 후 OpenCV로 표시
            display_img = cv2.resize(rgb, (rgb.shape[1] * 2, rgb.shape[0] * 2))
            cv2.imshow(self.window_name, display_img)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                self.get_logger().info("Q pressed. Shutting down...")
                break


# ========================
# 메인 함수
# ========================
def main():
    rclpy.init()
    node = YoloDepthTfNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
