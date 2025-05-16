import rclpy
from rclpy.node import Node
from rokey_pjt_interfaces.srv import TransformPoint
import tf2_ros
import tf2_geometry_msgs
import traceback


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
                request.input, "map", rclpy.duration.Duration(seconds=0.5)
            )

            response.x = transformed.point.x
            response.y = transformed.point.y
            response.z = transformed.point.z
            response.success = True
            response.error_msg = f"[TF] {request.label} → map: (x={response.x:.2f}, y={response.y:.2f}, z={response.z:.2f})"

            self.get_logger().info(response.error_msg)

        except Exception as e:
            response.x = 0.0
            response.y = 0.0
            response.z = 0.0
            response.success = False
            response.error_msg = f"[TF ERROR] {str(e)}"
            
            self.get_logger().warn(traceback.format_exc())
            self.get_logger().warn(response.error_msg)

        finally:
            return response


def main():
    rclpy.init()
    node = TFTransformService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
