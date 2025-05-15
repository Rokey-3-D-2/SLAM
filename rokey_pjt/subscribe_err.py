import rclpy
from rclpy.node import Node
from rokey_pjt_interfaces.msg import (
    AnomalyReport,
)  # ← 실제 메시지가 정의된 패키지 이름으로 변경
from sensor_msgs.msg import Image
from builtin_interfaces.msg import Time


class AnomalyReportSubscriber(Node):
    def __init__(self):
        super().__init__("anomaly_report_subscriber")
        self.subscription = self.create_subscription(
            AnomalyReport, "/robot1/error_detected", self.listener_callback, 10
        )

    def listener_callback(self, msg):
        self.get_logger().info("📡 이상 감지 메시지 수신:")
        self.get_logger().info(f" - 종류: {msg.anomaly_label}")
        self.get_logger().info(f" - 설명: {msg.description}")
        self.get_logger().info(
            f" - 로봇 위치: ({msg.robot_x}, {msg.robot_y}, {msg.robot_z})"
        )
        self.get_logger().info(
            f" - 객체 위치: ({msg.object_x}, {msg.object_y}, {msg.object_z})"
        )
        self.get_logger().info(
            f" - 타임스탬프: {msg.timestamp.sec}.{msg.timestamp.nanosec}"
        )
        self.get_logger().info(
            f" - 이미지 포함됨: {'Yes' if isinstance(msg.image, Image) else 'No'}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = AnomalyReportSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
