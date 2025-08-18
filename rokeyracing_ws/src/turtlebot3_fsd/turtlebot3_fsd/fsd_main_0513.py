import sys
import rclpy
from rclpy.node import Node

from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QGraphicsView, QGraphicsScene, QGraphicsPixmapItem,
    QLabel, QPushButton, QGridLayout, QTextEdit, QProgressBar
)
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import QTimer

from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist

import cv2
import numpy as np
from std_msgs.msg import Float32


class ROS2ControlGUI(Node, QWidget):
    def __init__(self):
        rclpy.init()
        Node.__init__(self, 'gui_cmdvel_compressed')
        QWidget.__init__(self)

        self.setWindowTitle("ROS2 GUI - Camera + Lane + Move Buttons")
        self.setFixedSize(1300, 900)

        # --- 속도 설정 변수 ---
        self.linear_speed = 0.05
        self.angular_speed = 0.0

        # --- Views ---
        self.camera_view = self.create_view("Camera View")
        self.lane_view = self.create_view("Lane Detection View")

        # --- ROS 설정 ---
        self.create_subscription(CompressedImage, '/camera/image_raw/compressed', self.camera_callback, 10)
        self.create_subscription(CompressedImage, '/detect/image_lane/compressed', self.lane_callback, 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.max_vel_publisher_ = self.create_publisher(Float32, '/max_vel', 10)


        # --- Layout ---
        main_layout = QVBoxLayout()
        top_layout = QHBoxLayout()
        top_layout.addLayout(self.camera_view['layout'], 1)
        top_layout.addLayout(self.lane_view['layout'], 2)
        main_layout.addLayout(top_layout)

        # --- 버튼과 속도 설정 UI ---
        main_layout.addLayout(self.create_button_grid())

        # --- 속도 시각화 ---
        self.cmd_label = QLabel("Linear: 0.00, Angular: 0.00")
        self.cmd_label.setStyleSheet("font-size: 16px; font-weight: bold;")
        main_layout.addWidget(self.cmd_label)

        self.linear_bar = QProgressBar()
        self.linear_bar.setRange(-100, 100)
        self.linear_bar.setFormat("Linear Speed: %v")
        self.linear_bar.setValue(0)

        self.angular_bar = QProgressBar()
        self.angular_bar.setRange(-100, 100)
        self.angular_bar.setFormat("Angular Z: %v")
        self.angular_bar.setValue(0)

        main_layout.addWidget(self.linear_bar)
        main_layout.addWidget(self.angular_bar)

        # --- 로그 출력 ---
        main_layout.addWidget(QLabel("📜 cmd_vel History"))
        self.cmd_log = QTextEdit()
        self.cmd_log.setReadOnly(True)
        self.cmd_log.setFixedHeight(150)
        main_layout.addWidget(self.cmd_log)

        # --- 속도 설정 표시 ---
        self.speed_label = QLabel(f"🔧 Speed - Linear: {self.linear_speed:.1f}, Angular: {self.angular_speed:.1f}")
        main_layout.addWidget(self.speed_label)

        self.setLayout(main_layout)

        # --- Timer ---
        self.timer = QTimer()
        self.timer.timeout.connect(self.spin_ros)
        self.timer.start(50)

    def spin_ros(self):
        rclpy.spin_once(self, timeout_sec=0.01)

    def create_view(self, label_text):
        view = QGraphicsView()
        scene = QGraphicsScene()
        item = QGraphicsPixmapItem()
        scene.addItem(item)
        view.setScene(scene)

        label = QLabel(label_text)
        layout = QVBoxLayout()
        layout.addWidget(label)
        layout.addWidget(view)

        return {'view': view, 'scene': scene, 'item': item, 'layout': layout}

    def update_image(self, item, img):
        rgb_img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_img.shape
        bytes_per_line = ch * w
        qt_image = QImage(rgb_img.data, w, h, bytes_per_line, QImage.Format_RGB888)
        item.setPixmap(QPixmap.fromImage(qt_image))

    def camera_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        img_small = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        h, w, _ = img_small.shape
        center_x, center_y = w // 2, h // 2
        cv2.line(img_small, (center_x - 20, center_y), (center_x + 20, center_y), (0, 255, 0), 1)
        cv2.line(img_small, (center_x, center_y - 20), (center_x, center_y + 20), (0, 255, 0), 1)
        cv2.rectangle(img_small, (w//4, int(h*0.3)), (w//4 + w//2, int(h*0.3)+80), (0, 0, 255), 2)

        self.update_image(self.camera_view['item'], img_small)

    def lane_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        img_small = cv2.resize(img, (0, 0), fx=0.5, fy=0.5)
        self.update_image(self.lane_view['item'], img_small)

    def create_button_grid(self):
        grid = QGridLayout()

        # 방향 버튼
        btn_up = QPushButton("▲")
        btn_down = QPushButton("▼")
        btn_left = QPushButton("◀")
        btn_right = QPushButton("▶")
        btn_stop = QPushButton("■ 정지")

        # 속도 조절 버튼
        btn_faster = QPushButton("Speed +")
        btn_slower = QPushButton("Speed -")

        btn_up.clicked.connect(lambda: self.publish_cmd(self.linear_speed, 0.0))
        btn_down.clicked.connect(lambda: self.publish_cmd(-self.linear_speed, 0.0))
        btn_left.clicked.connect(lambda: self.publish_cmd(0.0, self.angular_speed))
        btn_right.clicked.connect(lambda: self.publish_cmd(0.0, -self.angular_speed))
        btn_stop.clicked.connect(lambda: self.publish_cmd(0.0, 0.0))

        btn_faster.clicked.connect(self.increase_speed)
        btn_slower.clicked.connect(self.decrease_speed)

        grid.addWidget(btn_up, 0, 1)
        grid.addWidget(btn_left, 1, 0)
        grid.addWidget(btn_stop, 1, 1)
        grid.addWidget(btn_right, 1, 2)
        grid.addWidget(btn_down, 2, 1)

        grid.addWidget(btn_faster, 3, 0)
        grid.addWidget(btn_slower, 3, 2)

        return grid

    def increase_speed(self):
        self.linear_speed += 0.01
        self.angular_speed += 0.01
        self.update_speed_label()

    def decrease_speed(self):
        self.linear_speed = max(0.01, self.linear_speed - 0.01)
        self.angular_speed = max(0.01, self.angular_speed - 0.01)
        self.update_speed_label()

    def update_speed_label(self):
        self.speed_label.setText(f"🔧 Speed - Linear: {self.linear_speed:.2f}, Angular: {self.angular_speed:.2f}")
        msg = Float32()
        msg.data = self.linear_speed
        self.max_vel_publisher_.publish(msg)

        

    def publish_cmd(self, linear, angular):
        twist = Twist()
        twist.linear.x = linear + self.linear_speed
        twist.angular.z = angular
        self.cmd_pub.publish(twist)
        self.update_cmd_ui(linear, angular, sent=True)

    def cmd_callback(self, msg):
        self.update_cmd_ui(msg.linear.x, msg.angular.z, sent=False)

    def update_cmd_ui(self, linear, angular, sent=False):
        self.cmd_label.setText(f"Linear: {linear:.2f}, Angular: {angular:.2f}")
        self.linear_bar.setValue(int(linear * 100))
        self.angular_bar.setValue(int(angular * 100))

        prefix = "🔼 Sent →" if sent else "📡 Received ←"
        self.cmd_log.append(f"{prefix} Linear: {linear:.2f}, Angular: {angular:.2f}")

    def closeEvent(self, event):
        self.destroy_node()
        rclpy.shutdown()

def main():
    app = QApplication(sys.argv)
    gui = ROS2ControlGUI()
    gui.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
