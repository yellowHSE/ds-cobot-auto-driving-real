#!/usr/bin/env python3
#
# Copyright 2025 HumanAI CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors:
#   - Rujin Kim, kimrujin32@gmail.com

import cv2
from cv_bridge import CvBridge
import numpy as np
from rcl_interfaces.msg import IntegerRange
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from std_msgs.msg import UInt8

import cv2
import numpy as np
import tkinter as tk
from PIL import ImageTk



class LaneGUI(Node):
    def __init__(self, ):
        super().__init__('lane_gui_node')

        self.counter = 0
        self.subscription = self.create_subscription(
                Image, '/camera/image_raw', self.image_callback, 1

                )
        
    
        self.pub_image_lane = self.create_publisher(
                Image, '/detect/image_output', 1
                )
                
      
        self.br = CvBridge()

        self.prev_x_left = 0
        self.prev_x_right = 320

    def image_callback(self, msg):
        frame = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if self.counter ==0:
            self.counter += 1

            height, width = frame.shape[:2]
            print(f"Width: {width}, Height: {height}")

        success, x_left, x_right,frame = self.lane_detect(frame)

        if success:
            x_center = (x_left + x_right) // 2
        else:
            x_left = x_right = x_center = None


     #   if x_left is not None:
     #       cv2.circle(frame, (x_left, 460), 6, (0, 255, 0), 5)
     #   if x_right is not None:
     #       cv2.circle(frame, (x_right, 460), 6, (0, 0, 255), 5)
     #   if x_center is not None:
     #       cv2.circle(frame, (x_center, 460), 6, (255, 0, 0), 5)



        self.viewer.update_image(frame, x_left, x_right, x_center)

        self.pub_image_lane.publish(self.br.cv2_to_imgmsg(frame, 'bgr8'))
        
    def draw_clean_roi_box(self,original_img):
        # 이미지 복사해서 원본 보존

        img = original_img.copy()

        h, w = img.shape[:2]
        x1 = 70
        x2 = w - 70
        y1 = 3
        y2 = h

        # ROI 박스 (초록색, 두께 2)
        cv2.rectangle(img, (x1, y1), (x2, y2), (255, 255, 255), 2)

        return img
        

    def lane_detect(self, img):

        roi = self.draw_clean_roi_box(img)

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        edge = cv2.Canny(blur, 60, 75)

        lines = cv2.HoughLinesP(edge, 1, np.pi/180, 50, minLineLength=50, maxLineGap=20)


        if lines is None:
            return False, 0, 0
        
        # 선 그리기
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(img, (x1, y1), (x2, y2), (0, 255, 0), 2)

        # 결과 보기
        cv2.imshow("Detected Lines", img)
        cv2.waitKey(0)



        left_lines = []
        right_lines = []
        for line in lines:
            x1, y1, x2, y2 = line[0]
            if x2 == x1:
                continue
            slope = (y2 - y1) / (x2 - x1)
            if abs(slope) < 0.2:
                continue
            if slope < 0 and x1 < 320:
                left_lines.append(line[0])
            elif slope > 0 and x1 > 320:
                right_lines.append(line[0])

   
        def average_line(lines):
            if not lines:
                return 0.0, 0.0
            x_sum = y_sum = m_sum = 0.0
            for x1, y1, x2, y2 in lines:
                m = (y2 - y1) / (x2 - x1) if x2 != x1 else 0.0
                m_sum += m
                x_sum += x1 + x2
                y_sum += y1 + y2
            m_avg = m_sum / len(lines)
            x_avg = x_sum / (2 * len(lines))
            y_avg = y_sum / (2 * len(lines))
            b = y_avg - m_avg * x_avg
            return m_avg, b

        m_left, b_left = average_line(left_lines)
        m_right, b_right = average_line(right_lines)

        def compute_x(m, b):
            if m == 0:
                return None
            return int((180 - b) / m)

        x_left = compute_x(m_left, b_left)
        x_right = compute_x(m_right, b_right)

        if x_left is None:
            x_left = self.prev_x_left
        if x_right is None:
            x_right = self.prev_x_right

        self.prev_x_left = x_left
        self.prev_x_right = x_right

        return True, x_left, x_right, img


def main():
    rclpy.init()
    node = LaneGUI()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
