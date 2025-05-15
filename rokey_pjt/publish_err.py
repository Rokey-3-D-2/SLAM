import rclpy
from rclpy.node import Node
from rokey_pjt_interfaces.msg import AnomalyReport  # ← 메시지 패키지명 수정
from builtin_interfaces.msg import Time
from sensor_msgs.msg import Image
import numpy as np
import cv2
from cv_bridge import CvBridge


class AnomalyReportPublisher(Node):
    def __init__(self):
        super().__init__("anomaly_report_publisher")
        self.publisher = self.create_publisher(
            AnomalyReport, "/robot1/error_detected", 10
        )
        self.bridge = CvBridge()

        # 반복 퍼블리시: 1초마다
        self.create_timer(1.0, self.publish_anomaly)

    def publish_anomaly(self):
        # 흑백 테스트 이미지 생성
        img = np.zeros((240, 320, 3), dtype=np.uint8)
        cv2.putText(
            img, "Anomaly", (50, 120), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2
        )

        ros_img = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")

        msg = AnomalyReport()
        msg.anomaly_label = "스크래치"
        msg.description = "Error Detected : 스크래치"

        msg.robot_x = 1.2
        msg.robot_y = 2.3
        msg.robot_z = 0.0

        msg.object_x = 3.4
        msg.object_y = 4.5
        msg.object_z = 0.0

        msg.timestamp = self.get_clock().now().to_msg()
        msg.image = ros_img

        self.get_logger().info("📤 이상 감지 메시지 퍼블리시!")
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = AnomalyReportPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
