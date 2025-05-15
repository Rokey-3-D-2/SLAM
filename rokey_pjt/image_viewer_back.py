import rclpy
from rclpy.node import Node
from rokey_pjt_interfaces.msg import AnomalyReport
import cv2
from PIL import Image, ImageTk
import tkinter as tk
import threading


class MapMarkerNode(Node):
    def __init__(self):
        super().__init__("map_marker_node")

        # 맵 로드 및 확대
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

        # 구독자: AnomalyReport 하나만
        self.create_subscription(
            AnomalyReport,
            "/robot1/error_detected",
            self.anomaly_callback,
            10,
        )

        # GUI 쓰레드 실행
        threading.Thread(target=self.init_gui, daemon=True).start()

    def anomaly_callback(self, msg):
        # 이상 위치 마커 추가
        ox = msg.object_x * self.scale_factor
        oy = msg.object_y * self.scale_factor
        self.anomaly_positions.append((ox, oy))

        # 로봇 위치 갱신
        rx = msg.robot_x * self.scale_factor
        ry = msg.robot_y * self.scale_factor
        self.robot_pos = (rx, ry)

        self.get_logger().info(
            f"이상 감지: {msg.anomaly_label} | 객체=({ox:.1f},{oy:.1f}) 로봇=({rx:.1f},{ry:.1f})"
        )

    def init_gui(self):
        self.root = tk.Tk()
        self.root.title("확대 맵 - 로봇 및 이상 위치 표시")

        height, width = self.map_image.shape
        self.canvas = tk.Canvas(self.root, width=width, height=height)
        self.canvas.pack()

        pil_img = Image.fromarray(self.map_image)
        self.tk_img = ImageTk.PhotoImage(image=pil_img)
        self.canvas.create_image(0, 0, anchor=tk.NW, image=self.tk_img)

        self.update_loop()
        self.root.mainloop()

    def update_loop(self):
        self.canvas.delete("marker")

        # 로봇 위치 (빨간색)
        if self.robot_pos:
            x, y = self.robot_pos
            self.draw_circle(x, y, 10, "red")

        # 이상 위치들 (보라색)
        for x, y in self.anomaly_positions:
            self.draw_circle(x, y, 8, "purple")

        self.root.after(100, self.update_loop)

    def draw_circle(self, x, y, radius, color):
        self.canvas.create_oval(
            x - radius, y - radius, x + radius, y + radius, fill=color, tags="marker"
        )


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
