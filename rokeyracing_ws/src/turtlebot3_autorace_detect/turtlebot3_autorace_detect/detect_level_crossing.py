#!/usr/bin/env python
#
# Copyright 2018 ROBOTIS CO., LTD.
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
#   - Leon Jung, Gilbert, Ashe Kim, ChanHyeong Lee
#   - [AuTURBO] Kihoon Kim (https://github.com/auturbo)

from enum import Enum
import math
import time

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
from std_msgs.msg import String


def fnCalcDistanceDot2Line(a, b, c, x0, y0):
    distance = abs(x0 * a + y0 * b + c) / math.sqrt(a * a + b * b)
    return distance


def fnCalcDistanceDot2Dot(x1, y1, x2, y2):
    distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    return distance


def fnArrangeIndexOfPoint(arr):
    new_arr = arr[:]
    arr_idx = list(range(len(arr)))
    for i in range(len(arr)):
        for j in range(i + 1, len(arr)):
            if new_arr[i] < new_arr[j]:
                new_arr[i], new_arr[j] = new_arr[j], new_arr[i]
                arr_idx[i], arr_idx[j] = arr_idx[j], arr_idx[i]
    return arr_idx


def fnCheckLinearity(point1, point2, point3):
    threshold_linearity = 50
    x1, y1 = point1
    x2, y2 = point3
    if x2 - x1 != 0:
        a = (y2 - y1) / (x2 - x1)
    else:
        a = 1000
    b = -1
    c = y1 - a * x1
    err = fnCalcDistanceDot2Line(a, b, c, point2[0], point2[1])
    return err < threshold_linearity


def fnCheckDistanceIsEqual(point1, point2, point3):
    threshold_distance_equality = 3
    distance1 = fnCalcDistanceDot2Dot(point1[0], point1[1], point2[0], point2[1])
    distance2 = fnCalcDistanceDot2Dot(point2[0], point2[1], point3[0], point3[1])
    std = np.std([distance1, distance2])
    return std < threshold_distance_equality


# ------------------------
# ROS2 Node: DetectLevelNode
# ------------------------
class DetectLevelNode(Node):

    def __init__(self):
        super().__init__('detect_level')
        self.get_logger().info('Starting detect_level node (ROS2)')

        hue_range = IntegerRange(from_value=0, to_value=179, step=1)
        sat_range = IntegerRange(from_value=0, to_value=255, step=1)
        light_range = IntegerRange(from_value=0, to_value=255, step=1)

        hue_l_descriptor = ParameterDescriptor(
            description='Lower hue threshold',
            integer_range=[hue_range]
        )
        hue_h_descriptor = ParameterDescriptor(
            description='Upper hue threshold',
            integer_range=[hue_range]
        )
        sat_l_descriptor = ParameterDescriptor(
            description='Lower saturation threshold',
            integer_range=[sat_range]
        )
        sat_h_descriptor = ParameterDescriptor(
            description='Upper saturation threshold',
            integer_range=[sat_range]
        )
        light_l_descriptor = ParameterDescriptor(
            description='Lower value (lightness) threshold',
            integer_range=[light_range]
        )
        light_h_descriptor = ParameterDescriptor(
            description='Upper value (lightness) threshold',
            integer_range=[light_range]
        )

        # delcare parameters
        self.declare_parameter('detect.level.red.hue_l', 0, descriptor=hue_l_descriptor)
        self.declare_parameter('detect.level.red.hue_h', 22, descriptor=hue_h_descriptor)
        self.declare_parameter('detect.level.red.saturation_l', 24, descriptor=sat_l_descriptor)
        self.declare_parameter('detect.level.red.saturation_h', 255, descriptor=sat_h_descriptor)
        self.declare_parameter('detect.level.red.lightness_l', 56, descriptor=light_l_descriptor)
        self.declare_parameter('detect.level.red.lightness_h', 162, descriptor=light_h_descriptor)

        self.declare_parameter('is_detection_calibration_mode', False)

        # get parameters
        self.hue_red_l = self.get_parameter('detect.level.red.hue_l').value
        self.hue_red_h = self.get_parameter('detect.level.red.hue_h').value
        self.saturation_red_l = self.get_parameter('detect.level.red.saturation_l').value
        self.saturation_red_h = self.get_parameter('detect.level.red.saturation_h').value
        self.lightness_red_l = self.get_parameter('detect.level.red.lightness_l').value
        self.lightness_red_h = self.get_parameter('detect.level.red.lightness_h').value
        self.is_calibration_mode = self.get_parameter('is_detection_calibration_mode').value

        self.add_on_set_parameters_callback(self.on_parameter_change)

        self.sub_image_type = 'raw'  # 'raw' or 'compressed'
        self.pub_image_type = 'compressed'  # 'raw' or 'compressed'

        self.StepOfLevelCrossing = Enum('StepOfLevelCrossing', 'pass_level exit')

        self.is_level_crossing_finished = False
        self.stop_bar_count = 0
        self.counter = 1
        self.cv_image = None

        self.cv_bridge = CvBridge()

        # create publishers
        if self.pub_image_type == 'compressed':
            self.pub_image_level = self.create_publisher(
                CompressedImage, '/detect/image_output/compressed', 10)
            # if self.is_calibration_mode:
            #     self.pub_image_color_filtered = self.create_publisher(
            #         CompressedImage, '/detect/image_output_sub1/compressed', 10)
            self.pub_image_color_filtered = self.create_publisher(
                    CompressedImage, '/detect/image_output_sub1/compressed', 10)
        else:  # raw
            self.pub_image_level = self.create_publisher(
                Image, '/detect/image_output', 10)
            # if self.is_calibration_mode:
            #     self.pub_image_color_filtered = self.create_publisher(
            #         Image, '/detect/image_output_sub1', 10)
            self.pub_image_color_filtered = self.create_publisher(
                    Image, '/detect/image_output_sub1', 10)
            
        self.pub_stop_bar_state = self.create_publisher(
                String, '/detect/bar_state', 10)

        # create subscribers
        if self.sub_image_type == 'compressed':
            self.create_subscription(
                CompressedImage, '/camera/image_raw/compressed', self.get_image, 10) # /detect/image_input/compressed
        else:  # raw
            self.create_subscription(
                Image, '/camera/image_raw', self.get_image, 10) #/detect/image_input

        self.create_subscription(
            UInt8, '/detect/level_crossing_order', self.level_crossing_order, 10)

        self.timer = self.create_timer(1.0/15.0, self.timer_callback)

        time.sleep(1.0)

    def on_parameter_change(self, params):
        for param in params:
            if param.name == 'detect.level.red.hue_l':
                self.hue_red_l = param.value
            elif param.name == 'detect.level.red.hue_h':
                self.hue_red_h = param.value
            elif param.name == 'detect.level.red.saturation_l':
                self.saturation_red_l = param.value
            elif param.name == 'detect.level.red.saturation_h':
                self.saturation_red_h = param.value
            elif param.name == 'detect.level.red.lightness_l':
                self.lightness_red_l = param.value
            elif param.name == 'detect.level.red.lightness_h':
                self.lightness_red_h = param.value
        self.get_logger().info('Dynamic parameters updated.')
        return SetParametersResult(successful=True)

    def timer_callback(self):
        if self.cv_image is not None:
            self.find_level()

    def get_image(self, image_msg):
        # if self.counter % 3 != 0:
        #     self.counter += 1
        #     return
        # else:
        #     self.counter = 1

        if self.sub_image_type == 'compressed':
            np_arr = np.frombuffer(image_msg.data, np.uint8)
            self.cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        else:
            try:
                self.cv_image = self.cv_bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
            except Exception as e:
                self.get_logger().error('CV Bridge error: %s' % str(e))

    def level_crossing_order(self, order_msg):
        pub_level_crossing_return = UInt8()
        if order_msg.data == self.StepOfLevelCrossing.pass_level.value:
            while rclpy.ok():
                is_level_detected, _, _ = self.find_level()
                rclpy.spin_once(self, timeout_sec=0.01)
                if is_level_detected:
                    self.get_logger().info('Level Detected')
                    max_vel_msg = Float64()
                    max_vel_msg.data = 0.03
                    self.pub_max_vel.publish(max_vel_msg)
                    break

            while rclpy.ok():
                _, is_level_close, _ = self.find_level()
                rclpy.spin_once(self, timeout_sec=0.01)
                if is_level_close:
                    self.get_logger().info('STOP')
                    max_vel_msg = Float64()
                    max_vel_msg.data = 0.0
                    self.pub_max_vel.publish(max_vel_msg)
                    break

            while rclpy.ok():
                _, _, is_level_opened = self.find_level()
                rclpy.spin_once(self, timeout_sec=0.01)
                if is_level_opened:
                    self.get_logger().info('GO')
                    max_vel_msg = Float64()
                    max_vel_msg.data = 0.05
                    self.pub_max_vel.publish(max_vel_msg)
                    break

            pub_level_crossing_return.data = self.StepOfLevelCrossing.exit.value

        self.get_logger().info(pub_level_crossing_return.data)
        time.sleep(3.0)

    def find_level(self):
        mask = self.mask_red_of_level()
        mask = cv2.GaussianBlur(mask, (5, 5), 0)
        return self.find_rect_of_level(mask)

    def mask_red_of_level(self):
        """
        전체가 빨간색인 차단바를 직접 마스크로 생성한다.
        - 빨강 HSV 두 구간(0~10, 170~179)을 OR로 합성
        - 모폴로지(열림→닫힘)로 노이즈 정리
        - 더 이상 반전(bitwise_not)하지 않음
        """
        if self.cv_image is None:
            return None

        image = self.cv_image.copy()
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # 빨강 Hue는 양끝(0 근처, 180 근처) 두 구간으로 분리하는 것이 일반적
        # Saturation/Value는 기존 파라미터를 그대로 사용
        # Hue 상한/하한은 안전하게 clamp
        h_l = int(self.hue_red_l)
        h_h = int(self.hue_red_h)
        s_l = int(self.saturation_red_l)
        s_h = int(self.saturation_red_h)
        v_l = int(self.lightness_red_l)
        v_h = int(self.lightness_red_h)

        # 구간1: 저쪽(0~10)
        lower_red1 = np.array([max(0, min(h_l, 10)), s_l, v_l], dtype=np.uint8)
        upper_red1 = np.array([min(10, max(h_h, 0)), s_h, v_h], dtype=np.uint8)

        # 구간2: 고쪽(170~179)
        lower_red2 = np.array([max(170, h_l), s_l, v_l], dtype=np.uint8)
        upper_red2 = np.array([179,        s_h, v_h], dtype=np.uint8)

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)

        # 노이즈 정리: opening(작은 점 제거) → closing(틈 메움)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        if self.pub_image_type == 'compressed':
            comp_img_msg = self.cv_bridge.cv2_to_compressed_imgmsg(mask, dst_format='jpg')
            self.pub_image_color_filtered.publish(comp_img_msg)
        else:
            img_msg = self.cv_bridge.cv2_to_imgmsg(mask, encoding='mono8')
            self.pub_image_color_filtered.publish(img_msg)

        # 전체가 빨간 차단바 → 빨간 부분이 흰색(255)인 마스크를 그대로 반환
        return mask

    def find_rect_of_level(self, mask):
        """
        빨간 마스크에서 가장 큰 컨투어를 차단바로 간주하고,
        회전사각형/선 피팅으로 양 끝점을 만들어 선분을 구성한다.
        기울기와 두께 기반 근접도를 이용해 slowdown/stop/go를 판정한다.
        """
        is_level_detected = False  # slowdown
        is_level_close = False     # stop
        is_level_opened = False    # go

        if mask is None or self.cv_image is None:
            return is_level_detected, is_level_close, is_level_opened

        frame = self.cv_image.copy()

        # 컨투어 추출
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            # 빨간 바가 안 잡히면 일단 OPEN(go)로 본다(정책에 따라 조정)
            is_level_opened = True
            self.stop_bar_state = 'go'
            if self.pub_image_type == 'compressed':
                comp_img_msg = self.cv_bridge.cv2_to_compressed_imgmsg(frame, dst_format='jpg')
                self.pub_image_level.publish(comp_img_msg)
            else:
                img_msg = self.cv_bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                self.pub_image_level.publish(img_msg)
            return is_level_detected, is_level_close, is_level_opened

        # 가장 큰 컨투어 선택(면적 기준)
        cnt = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(cnt)

        # 너무 작은 잡음 컨투어는 무시(필요시 값 조정)
        MIN_AREA = 1500
        if area < MIN_AREA:
            is_level_opened = True
            self.stop_bar_state = 'go'
            if self.pub_image_type == 'compressed':
                comp_img_msg = self.cv_bridge.cv2_to_compressed_imgmsg(frame, dst_format='jpg')
                self.pub_image_level.publish(comp_img_msg)
            else:
                img_msg = self.cv_bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                self.pub_image_level.publish(img_msg)
            return is_level_detected, is_level_close, is_level_opened

        # 회전 사각형으로 크기/방향 대략 추정
        rect = cv2.minAreaRect(cnt)  # ((cx, cy), (w, h), angle_deg)
        (cx, cy), (w, h), angle_deg = rect
        thickness = float(min(w, h))  # 막대 두께(짧은 변)
        length = float(max(w, h))     # 막대 길이(긴 변)

        self.get_logger().info(f"cx = {cx}, cy = {cy}")


        # 방향 벡터(주축) 추정: fitLine이 노이즈에 비교적 강함
        vx, vy, x0, y0 = cv2.fitLine(cnt, cv2.DIST_L2, 0, 0.01, 0.01)
        norm = math.sqrt(float(vx*vx + vy*vy)) + 1e-9
        vx = float(vx) / norm
        vy = float(vy) / norm

        half = length / 2.0
        p1 = (int(x0 - vx * half), int(y0 - vy * half))
        p3 = (int(x0 + vx * half), int(y0 + vy * half))
        p2 = (int(x0), int(y0))  # 중앙점

        # 시각화(선/박스)
        box = cv2.boxPoints(rect).astype(int)
        cv2.drawContours(frame, [box], 0, (0, 255, 255), 2)
        cv2.line(frame, p1, p3, (255, 0, 0), 5)
        cv2.circle(frame, p2, 5, (0, 255, 0), -1)

        # 기울기 기반 OPEN(go) 판정
        dx = p3[0] - p1[0]
        dy = p3[1] - p1[1]
        slope = float('inf') if dx == 0 else dy / dx

        # if dx == 0 or abs(slope) > 2.0:
        #     is_level_opened = True
        #     self.stop_bar_state = 'go'
        #     self.get_logger().info(self.stop_bar_state)
        # else:
        #     # 두께 기반 근접도: 가까울수록 thickness ↑ → 지표 ↓
        #     # 기존 로직과 단조 방향을 맞추기 위해 scale / thickness 사용
        #     SCALE_K = 50.0  # 필요시 조정; 카메라/FOV/해상도에 따라 달라짐
        #     distance_bar2car = SCALE_K / (thickness + 1e-6)
        #     self.stop_bar_count = 50
        #     self.get_logger().info(f"{distance_bar2car}")
        #     self.get_logger().info(f"{distance_bar2car}")

        #     if distance_bar2car > 1.0:
        #         is_level_detected = True   # slowdown
        #         self.stop_bar_state = 'slowdown'
        #         self.get_logger().info(f"{self.stop_bar_state} (d={distance_bar2car:.2f})")
        #     else:
        #         is_level_close = True      # stop
        #         self.stop_bar_state = 'stop'
        #         self.get_logger().info(f"{self.stop_bar_state} (d={distance_bar2car:.2f})")

        if not (dx == 0 or abs(slope) > 2.0) :
            # 두께 기반 근접도: 가까울수록 thickness ↑ → 지표 ↓
            # 기존 로직과 단조 방향을 맞추기 위해 scale / thickness 사용
            SCALE_K = 50.0  # 필요시 조정; 카메라/FOV/해상도에 따라 달라짐
            distance_bar2car = SCALE_K / (thickness + 1e-6)
            self.stop_bar_count = 50
            self.get_logger().info(f"{distance_bar2car}")
            self.get_logger().info(f"{distance_bar2car}")

            if cy >= 150:
                is_level_detected = True      # stop
                self.stop_bar_state = 'stop'
                self.get_logger().info(f"{self.stop_bar_state} (d={distance_bar2car:.2f})")
            else:
                is_level_detected = True   # slowdown
                self.stop_bar_state = 'slowdown'
                self.get_logger().info(f"{self.stop_bar_state} (d={distance_bar2car:.2f})")
        else:
            is_level_opened = True
            self.stop_bar_state = 'go'
            self.get_logger().info(self.stop_bar_state)
        
        msg = String()
        msg.data = self.stop_bar_state
        self.pub_stop_bar_state.publish(msg)

        # 결과 영상 퍼블리시
        if self.pub_image_type == 'compressed':
            comp_img_msg = self.cv_bridge.cv2_to_compressed_imgmsg(frame, dst_format='jpg')
            self.pub_image_level.publish(comp_img_msg)
        else:
            img_msg = self.cv_bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.pub_image_level.publish(img_msg)

        return is_level_detected, is_level_close, is_level_opened


def main(args=None):
    rclpy.init(args=args)
    node = DetectLevelNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
