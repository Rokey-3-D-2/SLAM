import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rokey_pjt_interfaces.msg import AnomalyReport
import cv2
from cv_bridge import CvBridge
from PIL import Image as PILImage, ImageTk
import tkinter as tk
import threading


class MapMarkerNode(Node):
    def __init__(self):
        super().__init__("map_marker_node")

        # 맵 이미지 불러오기
        map_path = "/home/we/rokey_ws/maps/first_map.pgm"
        raw_map = cv2.imread(map_path, cv2.IMREAD_GRAYSCALE)
        if raw_map is None:
            self.get_logger().error(f"맵 이미지 로드 실패: {map_path}")
            return

        self.scale_factor = 8
        self.map_image = cv2.resize(
            raw_map,
            (0, 0),
            fx=self.scale_factor,
            fy=self.scale_factor,
            interpolation=cv2.INTER_NEAREST,
        )

        # 위치 저장
        self.robot_pos = None
        self.anomaly_positions = []
        self.latest_rgb = None

        # 이미지 디코더
        self.bridge = CvBridge()

        # 구독자 등록
        self.create_subscription(
            AnomalyReport,
            "/robot1/error_detected",
            self.anomaly_callback,
            10,
        )
        self.create_subscription(
            Image,
            "/robot1/oakd/rgb/preview/image_raw",
            self.rgb_callback,
            10,
        )

        # GUI 쓰레드 실행
        threading.Thread(target=self.init_gui, daemon=True).start()

    def anomaly_callback(self, msg):
        self.get_logger().info(
            f"객체 좌표: x={msg.object_x:.2f}, y={msg.object_y:.2f}, z={msg.object_z:.2f}"
        )
        ox = msg.object_x * self.scale_factor
        oy = msg.object_y * self.scale_factor
        self.anomaly_positions.append((ox, oy))

        rx = msg.robot_x * self.scale_factor
        ry = msg.robot_y * self.scale_factor
        self.robot_pos = (rx, ry)

        self.get_logger().info(
            f"이상 감지: {msg.anomaly_label} | 객체=({ox:.1f},{oy:.1f}) 로봇=({rx:.1f},{ry:.1f})"
        )

    def rgb_callback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            cv_img_resized = cv2.resize(cv_img, (320, 240))
            rgb_img = cv2.cvtColor(cv_img_resized, cv2.COLOR_BGR2RGB)
            self.latest_rgb = PILImage.fromarray(rgb_img)
        except Exception as e:
            self.get_logger().warn(f"카메라 이미지 변환 실패: {e}")

    def init_gui(self):
        self.root = tk.Tk()
        self.root.title("맵 + 카메라 이미지 뷰어")

        self.main_frame = tk.Frame(self.root)
        self.main_frame.pack()

        # 맵 캔버스 (왼쪽)
        h, w = self.map_image.shape
        self.canvas_map = tk.Canvas(self.main_frame, width=w, height=h)
        self.canvas_map.grid(row=0, column=0)

        pil_map = PILImage.fromarray(self.map_image)
        self.tk_map_img = ImageTk.PhotoImage(image=pil_map)
        self.canvas_map.create_image(0, 0, anchor=tk.NW, image=self.tk_map_img)

        # 카메라 이미지 캔버스 (오른쪽)
        self.canvas_rgb = tk.Label(self.main_frame)
        self.canvas_rgb.grid(row=0, column=1)

        self.update_loop()
        self.root.mainloop()

    def update_loop(self):
        self.canvas_map.delete("marker")

        # 로봇 마커 (빨간색)
        if self.robot_pos:
            x, y = self.robot_pos
            self.draw_circle(self.canvas_map, x, y, 10, "red")

        # 이상 마커 (보라색)
        for x, y in self.anomaly_positions:
            self.draw_circle(self.canvas_map, x, y, 8, "purple")

        # 카메라 이미지 업데이트
        if self.latest_rgb:
            tk_rgb_img = ImageTk.PhotoImage(self.latest_rgb)
            self.canvas_rgb.configure(image=tk_rgb_img)
            self.canvas_rgb.image = tk_rgb_img  # prevent garbage collection

        self.root.after(100, self.update_loop)

    def draw_circle(self, canvas, x, y, r, color):
        canvas.create_oval(x - r, y - r, x + r, y + r, fill=color, tags="marker")


def main(args=None):
    rclpy.init(args=args)
    node = MapMarkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
