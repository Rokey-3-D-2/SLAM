import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import cv2

# 사용자 정의 메시지 import
# from rokey_pjt.msg import Anomalyreport
from rokey_pjt.msg import Anomalyreport


class AnomalySubscriber(Node):
    def __init__(self):
        super().__init__("anomaly_report_subscriber")
        self.subscription = self.create_subscription(
            Anomalyreport, "anomaly_report", self.listener_callback, 10
        )
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        # 텍스트 출력
        self.get_logger().info(f"📌 감지 라벨: {msg.anomaly_label}")
        self.get_logger().info(f"📝 설명: {msg.description}")
        self.get_logger().info(
            f"🤖 로봇 위치: ({msg.robot_x}, {msg.robot_y}, {msg.robot_z})"
        )
        self.get_logger().info(
            f"📦 객체 위치: ({msg.object_x}, {msg.object_y}, {msg.object_z})"
        )
        self.get_logger().info(
            f"⏱️ 타임스탬프: {msg.timestamp.sec}.{msg.timestamp.nanosec}"
        )

        # 이미지 처리 (OpenCV로 표시)
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg.image, desired_encoding="bgr8")
            cv2.imshow("Anomaly Image", cv_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"[이미지 처리 오류] {e}")


def main(args=None):
    rclpy.init(args=args)
    node = AnomalySubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
