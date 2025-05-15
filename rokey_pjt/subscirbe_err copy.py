import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import tkinter as tk
from PIL import Image, ImageTk
import os

from sensor_msgs.msg import Image as RosImage
from cv_bridge import CvBridge
import cv2


class ImageMarkerNode(Node):
    def __init__(self, canvas, marker_callbacks, camera_label):
        super().__init__("image_marker_node")

        self.canvas = canvas
        self.marker_callbacks = marker_callbacks
        self.camera_label = camera_label
        self.bridge = CvBridge()

        # 첫 번째 마커 (빨간색)
        self.create_subscription(Point, "marker_point", self.point1_callback, 10)

        # 두 번째 마커 (파란색)
        self.create_subscription(Point, "robot_point", self.point2_callback, 10)

        # 카메라 영상 구독
        self.create_subscription(
            RosImage, "/robot1/oakd/rgb/preview/image_raw", self.camera_callback, 10
        )

    def point1_callback(self, msg):
        x, y = msg.x, msg.y
        self.get_logger().info(f"[marker_point] ({x}, {y})")
        self.marker_callbacks["red"](x, y)

    def point2_callback(self, msg):
        x, y = msg.x, msg.y
        self.get_logger().info(f"[robot_point] ({x}, {y})")
        self.marker_callbacks["blue"](x, y)

    def camera_callback(self, msg):
        try:
            # ROS 이미지 → OpenCV 이미지 (BGR)
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            # BGR → RGB
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            # OpenCV → PIL → Tkinter-compatible
            img_pil = Image.fromarray(cv_image).resize((320, 240))  # 영상 크기 조절
            tk_image = ImageTk.PhotoImage(img_pil)
            # Tkinter 라벨에 적용
            self.camera_label.config(image=tk_image)
            self.camera_label.image = tk_image  # 🔐 참조 유지
        except Exception as e:
            self.get_logger().error(f"[camera_callback error] {e}")


def main():
    rclpy.init()

    root = tk.Tk()
    root.title("ROS2 Image + Camera Viewer")

    # 프레임 나누기
    frame = tk.Frame(root)
    frame.pack()

    # =================== 맵 이미지 표시 ====================
    image_path = os.path.expanduser("~/rokey_ws/maps/first_map.pgm")
    image = Image.open(image_path).convert("RGB")
    scale_factor = 8
    resized_image = image.resize(
        (image.width * scale_factor, image.height * scale_factor), Image.NEAREST
    )
    tk_image = ImageTk.PhotoImage(resized_image)

    canvas = tk.Canvas(frame, width=resized_image.width, height=resized_image.height)
    canvas.pack(side=tk.LEFT)
    canvas.create_image(0, 0, anchor=tk.NW, image=tk_image)

    # =================== 카메라 영상 표시 ====================
    camera_label = tk.Label(frame)
    camera_label.pack(side=tk.RIGHT, padx=10)

    # 마커 관리
    markers = {"red": {"id": None}, "blue": {"id": None}}

    def draw_marker(color):
        def _draw(x, y):
            radius = 8
            x *= scale_factor
            y *= scale_factor
            if markers[color]["id"]:
                canvas.delete(markers[color]["id"])
            markers[color]["id"] = canvas.create_oval(
                x - radius, y - radius, x + radius, y + radius, fill=color
            )

        return _draw

    # 노드 실행
    node = ImageMarkerNode(
        canvas, {"red": draw_marker("red"), "blue": draw_marker("blue")}, camera_label
    )

    def tk_spin():
        rclpy.spin_once(node, timeout_sec=0.1)
        root.after(50, tk_spin)

    root.after(50, tk_spin)
    root.mainloop()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
