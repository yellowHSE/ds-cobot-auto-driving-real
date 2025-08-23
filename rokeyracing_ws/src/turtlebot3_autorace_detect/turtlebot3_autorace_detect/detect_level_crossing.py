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

# -----------------------------------------------------------------------------
# 본 노드는 "수평 레벨 크로싱(차단바)"를 카메라 이미지에서 감지하여
# 상태를 판단하는 ROS2 노드 구현이다.
# - 입력: /camera/image_raw 또는 /camera/image_raw/compressed
# - 출력:
#    • /detect/image_output[(/compressed)]: 결과 시각화 이미지
#    • /detect/image_output_sub1[(/compressed)]: 색 필터(마스크) 시각화
#    • /detect/bar_state: 'slowdown' | 'stop' | 'go' 상태 문자열
# - 동작 개요:
#    1) HSV 공간에서 "빨간색" 마스크 생성(차단바 가정)
#    2) 가장 큰 컨투어에서 회전사각형/직선 피팅으로 막대의 길이·두께·기울기 계산
#    3) 기울기, 두께, 위치 등을 바탕으로 상태 판정
# - 파라미터:
#    • detect.level.red.*: 빨강 HSV 임계값
#    • is_detection_calibration_mode: 보정 모드 플래그(동적 파라미터 반영)
# -----------------------------------------------------------------------------

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
    # 점(x0, y0)에서 직선 ax + by + c = 0 까지의 유클리드 거리
    distance = abs(x0 * a + y0 * b + c) / math.sqrt(a * a + b * b)
    return distance


def fnCalcDistanceDot2Dot(x1, y1, x2, y2):
    # 두 점 사이의 유클리드 거리
    distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    return distance


def fnArrangeIndexOfPoint(arr):
    # 값이 큰 순으로 정렬했을 때의 원래 인덱스 배열 반환
    new_arr = arr[:]
    arr_idx = list(range(len(arr)))
    for i in range(len(arr)):
        for j in range(i + 1, len(arr)):
            if new_arr[i] < new_arr[j]:
                new_arr[i], new_arr[j] = new_arr[j], new_arr[i]
                arr_idx[i], arr_idx[j] = arr_idx[j], arr_idx[i]
    return arr_idx


def fnCheckLinearity(point1, point2, point3):
    # 세 점(point1, point2, point3)이 선형(일직선상)인지 간단히 확인
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
    # 세 점 간의 두 구간 거리 유사성(연속적인 균일 간격 여부) 확인
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

        # 파라미터 범위(Descriptor) 설정: hue/saturation/value 각각의 합리적 범위를 명시
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
        # 빨강 HSV 범위 기본값과 보정 모드 파라미터 선언
        self.declare_parameter('detect.level.red.hue_l', 0, descriptor=hue_l_descriptor)
        self.declare_parameter('detect.level.red.hue_h', 22, descriptor=hue_h_descriptor)
        self.declare_parameter('detect.level.red.saturation_l', 24, descriptor=sat_l_descriptor)
        self.declare_parameter('detect.level.red.saturation_h', 255, descriptor=sat_h_descriptor)
        self.declare_parameter('detect.level.red.lightness_l', 56, descriptor=light_l_descriptor)
        self.declare_parameter('detect.level.red.lightness_h', 162, descriptor=light_h_descriptor)

        self.declare_parameter('is_detection_calibration_mode', False)

        # get parameters
        # 위에서 선언한 파라미터를 멤버 변수로 로딩
        self.hue_red_l = self.get_parameter('detect.level.red.hue_l').value
        self.hue_red_h = self.get_parameter('detect.level.red.hue_h').value
        self.saturation_red_l = self.get_parameter('detect.level.red.saturation_l').value
        self.saturation_red_h = self.get_parameter('detect.level.red.saturation_h').value
        self.lightness_red_l = self.get_parameter('detect.level.red.lightness_l').value
        self.lightness_red_h = self.get_parameter('detect.level.red.lightness_h').value
        self.is_calibration_mode = self.get_parameter('is_detection_calibration_mode').value

        # 런타임 동적 파라미터 변경 시 적용할 콜백 등록
        self.add_on_set_parameters_callback(self.on_parameter_change)

        # 입력/출력 이미지 형식 선택('raw' 또는 'compressed')
        self.sub_image_type = 'raw'  # 'raw' or 'compressed'
        self.pub_image_type = 'compressed'  # 'raw' or 'compressed'

        # 레벨 크로싱 단계 정의(명령용 enum)
        self.StepOfLevelCrossing = Enum('StepOfLevelCrossing', 'pass_level exit')

        # 상태 변수 초기화
        self.is_level_crossing_finished = False
        self.stop_bar_count = 0  # 특정 상태 유지/전이 시 카운터로 사용
        self.counter = 1
        self.cv_image = None

        self.cv_bridge = CvBridge()

        # create publishers
        # 결과 이미지와 색 필터(마스크) 이미지를 퍼블리시할 퍼블리셔 구성
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
        # 카메라 입력 구독자 설정 (압축/비압축 선택 가능)
        if self.sub_image_type == 'compressed':
            self.create_subscription(
                CompressedImage, '/camera/image_raw/compressed', self.get_image, 10) # /detect/image_input/compressed
        else:  # raw
            self.create_subscription(
                Image, '/camera/image_raw', self.get_image, 10) #/detect/image_input

        # 외부로부터 레벨 크로싱 진행 명령을 수신(옵션)
        self.create_subscription(
            UInt8, '/detect/level_crossing_order', self.level_crossing_order, 10)

        # 주기적 처리 타이머(15Hz): 최근 프레임에 대해 find_level 수행
        self.timer = self.create_timer(1.0/15.0, self.timer_callback)

        # 초기 안정화를 위한 잠깐의 대기(센서 준비 등)
        time.sleep(1.0)

    def on_parameter_change(self, params):
        # 동적 파라미터 변경 시 HSV 임계값을 갱신
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
        # 타이머 콜백: 유효한 이미지가 있으면 레벨 감지 수행
        if self.cv_image is not None:
            self.find_level()

    def get_image(self, image_msg):
        # 카메라 이미지 콜백: 압축/비압축에 따라 디코딩 방식 분기
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
        # 외부 명령(pass_level)을 받았을 때 단계적으로 레벨 상태를 확인하고
        # 가상의 속도 명령 퍼블리시(pub_max_vel)를 수행하는 절차(참고용 로직)
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

        # self.get_logger().info(pub_level_crossing_return.data)
        time.sleep(3.0)

    def find_level(self):
        # 레벨 감지 파이프라인: 빨간색 마스크 → 블러 → 사각형/선 피팅 및 상태 판정
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

        # 파라미터에서 읽은 임계값 적용(필요 시 clamp)
        h_l = int(self.hue_red_l)
        h_h = int(self.hue_red_h)
        s_l = int(self.saturation_red_l)
        s_h = int(self.saturation_red_h)
        v_l = int(self.lightness_red_l)
        v_h = int(self.lightness_red_h)

        # 빨강 Hue는 0~10(저쪽), 170~179(고쪽) 두 구간으로 분리하여 inRange 후 OR
        lower_red1 = np.array([max(0, min(h_l, 10)), s_l, v_l], dtype=np.uint8)
        upper_red1 = np.array([min(10, max(h_h, 0)), s_h, v_h], dtype=np.uint8)

        lower_red2 = np.array([max(170, h_l), s_l, v_l], dtype=np.uint8)
        upper_red2 = np.array([179,        s_h, v_h], dtype=np.uint8)

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)

        # 모폴로지로 작은 노이즈 제거 및 간극 메움
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        if self.pub_image_type == 'compressed':
            comp_img_msg = self.cv_bridge.cv2_to_compressed_imgmsg(mask, dst_format='jpg')
            self.pub_image_color_filtered.publish(comp_img_msg)
        else:
            img_msg = self.cv_bridge.cv2_to_imgmsg(mask, encoding='mono8')
            self.pub_image_color_filtered.publish(img_msg)

        # 전체가 빨간 차단바 → 빨간 부분(흰색=255) 마스크 반환
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

        # 컨투어 추출(외곽만)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            # 빨간 바가 안 잡히면 일단 OPEN(go)로 판단(정책에 따라 조정 가능)
            is_level_opened = True
            self.stop_bar_state = 'go'
            if self.pub_image_type == 'compressed':
                comp_img_msg = self.cv_bridge.cv2_to_compressed_imgmsg(frame, dst_format='jpg')
                self.pub_image_level.publish(comp_img_msg)
            else:
                img_msg = self.cv_bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                self.pub_image_level.publish(img_msg)
            return is_level_detected, is_level_close, is_level_opened

        # 가장 큰 컨투어 선택(면적 최대)
        cnt = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(cnt)

        # 작은 잡음은 무시(하드 임계)
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

        # 회전 사각형으로 대략의 크기/각도 산출
        rect = cv2.minAreaRect(cnt)  # ((cx, cy), (w, h), angle_deg)
        (cx, cy), (w, h), angle_deg = rect
        thickness = float(min(w, h))  # 막대의 두께(짧은 변)
        length = float(max(w, h))     # 막대의 길이(긴 변)

        # self.get_logger().info(f'길이{length}')
        
        if length < 80:
            return

        # 직선 피팅으로 주축 방향(노이즈에 강함)
        vx, vy, x0, y0 = cv2.fitLine(cnt, cv2.DIST_L2, 0, 0.01, 0.01)
        norm = math.sqrt(float(vx*vx + vy*vy)) + 1e-9
        vx = float(vx) / norm
        vy = float(vy) / norm

        half = length / 2.0
        p1 = (int(x0 - vx * half), int(y0 - vy * half))
        p3 = (int(x0 + vx * half), int(y0 + vy * half))
        p2 = (int(x0), int(y0))  # 중심점

        # 시각화(회전사각형과 막대 중심선)
        box = cv2.boxPoints(rect).astype(int)
        cv2.drawContours(frame, [box], 0, (0, 255, 255), 2)
        cv2.line(frame, p1, p3, (255, 0, 0), 5)
        cv2.circle(frame, p2, 5, (0, 255, 0), -1)

        # 기울기 계산(수평에 가까우면 |slope|가 작음)
        dx = p3[0] - p1[0]
        dy = p3[1] - p1[1]
        slope = float('inf') if dx == 0 else dy / dx

        # 아래 로직:
        # - 기울기가 일정 범위 내(수평에 가까움)일 때만 차단바로 가정하여 거리/위치 기반 판정
        # - cy(중심 y)와 cx(중심 x) 조건으로 slowdown/stop 상태 구분(경험적 설정)
        if not (dx == 0 or abs(slope) > 0.5) :
            # 두께가 두꺼울수록 가까이 있다고 볼 수 있음(참고용 지표)
            SCALE_K = 50.0
            distance_bar2car = SCALE_K / (thickness + 1e-6)
            # self.get_logger().info(f"{distance_bar2car}")
            # self.get_logger().info(f"{distance_bar2car}")

            if cy >= 100 and cx <= 200:
                is_level_detected = True      # stop
                self.stop_bar_state = 'stop'
                self.get_logger().info(f"{self.stop_bar_state}")
                self.stop_bar_count = 1 
            elif cy < 100:
                is_level_detected = True   # slowdown
                self.stop_bar_state = 'slowdown'
                self.get_logger().info(f"{self.stop_bar_state}")
                # self.stop_bar_count = 1 

        else:
            # 충분히 수평이 아니거나 dx==0인 경우: stop_bar_count를 이용해 go/stop 상태를 유지/전이
            self.get_logger().info(f"카운트 :{self.stop_bar_count}")
            if self.stop_bar_count == 0:
                self.stop_bar_state = 'stop'

            else:
                is_level_opened = True
                self.stop_bar_state = 'go'
                self.get_logger().info(self.stop_bar_state)
        
        # 상태 문자열 퍼블리시
        msg = String()
        msg.data = self.stop_bar_state
        self.pub_stop_bar_state.publish(msg)

        # 결과 시각화 이미지 퍼블리시
        if self.pub_image_type == 'compressed':
            comp_img_msg = self.cv_bridge.cv2_to_compressed_imgmsg(frame, dst_format='jpg')
            self.pub_image_level.publish(comp_img_msg)
        else:
            img_msg = self.cv_bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.pub_image_level.publish(img_msg)

        return is_level_detected, is_level_close, is_level_opened


def main(args=None):
    # 노드 초기화/스핀/정리
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