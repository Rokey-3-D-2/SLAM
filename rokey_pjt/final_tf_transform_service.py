import rclpy
from rclpy.node import Node
from rokey_pjt_interfaces.srv import TransformPoint
import tf2_ros
import tf2_geometry_msgs


class TFTransformService(Node):
    def __init__(self):
        super().__init__("tf_transform_service")
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.srv = self.create_service(
            TransformPoint, "/robot1/transform_to_map", self.callback
        )

    def callback(self, request, response):
        try:
            transformed = self.tf_buffer.transform(
                request.point, "map", rclpy.duration.Duration(seconds=0.5)
            )
            response.transformed_point = transformed
            response.label = request.label
            response.success = True
            response.error_message = ""

            # self.get_logger().info(f"[TF] 변환 성공: {request.label}")
            self.get_logger().info(
                f"[TF] {request.label} → map: (x={request.x:.2f}, y={request.y:.2f}, z={request.z:.2f})"
            )
        except Exception as e:
            response.success = False
            response.error_message = str(e)
            self.get_logger().warn(f"[TF] 변환 실패: {e}")
        return response


def main():
    rclpy.init()
    node = TFTransformService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
