import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray
import cv2
from PIL import Image, ImageTk
import tkinter as tk
import threading


class MapMarkerNode(Node):
    def __init__(self):
        super().__init__("map_marker_node")

        # 맵 이미지 로딩 및 확대
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

        # ROS2 구독자
        self.create_subscription(Point, "/robot_pose", self.robot_callback, 10)
        self.create_subscription(
            Float32MultiArray, "/obstacles", self.obstacle_callback, 10
        )

        self.robot_pos = None
        self.obstacles = []

        # GUI 시작
        threading.Thread(target=self.init_gui, daemon=True).start()

    def robot_callback(self, msg):
        self.robot_pos = (msg.x * self.scale_factor, msg.y * self.scale_factor)

    def obstacle_callback(self, msg):
        self.obstacles = [
            (msg.data[i] * self.scale_factor, msg.data[i + 1] * self.scale_factor)
            for i in range(0, len(msg.data), 2)
        ]

    def init_gui(self):
        self.root = tk.Tk()
        self.root.title("확대된 맵 - 로봇 및 장애물 위치 표시")

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

        # 로봇 위치 마킹 (빨간색)
        if self.robot_pos:
            x, y = self.robot_pos
            self.draw_circle(x, y, 6, "red")

        # 장애물 마킹 (파란색)
        for obs_x, obs_y in self.obstacles:
            self.draw_circle(obs_x, obs_y, 4, "blue")

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
