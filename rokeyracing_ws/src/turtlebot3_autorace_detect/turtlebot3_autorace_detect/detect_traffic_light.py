#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# -------------------------------------------------------------------------------------------------
# Traffic Light Detection (ROS + Offline Video Test)
#  - Post-validation OFF: Hough 원 검출되면 즉시 통과
#  - Offline video mode via code settings (no CLI arg)
#  - ROI rectangle overlay
#  - Practical HSV defaults + Bright/Sat gate + mask guard
#  - Debug logs for mask counts and Hough circles
# -------------------------------------------------------------------------------------------------

import os
import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import IntegerRange, ParameterDescriptor, SetParametersResult
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError

# ========== [Settings] ==========
USE_OFFLINE_VIDEO   = False                          # True: use local video, False: subscribe ROS topic
DEFAULT_TEST_VIDEO  = '/home/pjs/Downloads/test.webm'
SHOW_WINDOW         = False                         # headless 환경이면 자동으로 비활성
WRITE_PATH          = '/tmp/out.mp4'                # None이면 저장 안 함
FORCE_FPS           = 0.0                           # 0이면 원본 FPS 시도, 실패 시 30
ENABLE_VALIDATION   = False                         # 후검증 사용 안 함 (허프 잡히면 즉시 통과)
MIN_STREAK_INIT     = 1                             # 퍼블리시 안정화 프레임 (테스트용 1)
FRAME_SKIP_3X       = False                         # True면 3프레임 중 1프레임만 처리

# ROI 비율 (영상에 맞게 조정 가능)
ROI_X0_FRAC, ROI_X1_FRAC = 0.50, 1.00              # 오른쪽 절반
ROI_Y0_FRAC, ROI_Y1_FRAC = 0.20, 0.85              # 세로 넓게

# HoughCircles 파라미터
HOUGH_DP, HOUGH_MIN_DIST = 1.2, 20
HOUGH_PARAM1, HOUGH_PARAM2 = 120, 5                # Canny 상한 / 누적 임계 (민감)
R_MIN, R_MAX = 10, 25                                # 원 반경 범위

# Bright/Sat 공통 게이트 하한
S_MIN, V_MIN = 60, 120
# =================================


class DetectTrafficLight(Node):
    def __init__(self):
        super().__init__('detect_traffic_light')

        # 퍼블리셔
        self.pub_traffic_light_state = self.create_publisher(String, '/detect/traffic_light_state', 10)

        # 안정화
        self.MIN_STREAK = MIN_STREAK_INIT
        self._stable_state = 'none'
        self._candidate_state = 'none'
        self._candidate_streak = 0

        # 파라미터 메타데이터
        desc_h = ParameterDescriptor(integer_range=[IntegerRange(from_value=0, to_value=179, step=1)],
                                     description='Hue 0~179')
        desc_sv = ParameterDescriptor(integer_range=[IntegerRange(from_value=0, to_value=255, step=1)],
                                      description='S/V 0~255')

        # ===== HSV 파라미터 선언 (실전 기본값으로 좁힘) =====
        # Red: wrap-around (170~179 ∪ 0~10)
        self.declare_parameter('red.hue_l',         170, desc_h)
        self.declare_parameter('red.hue_h',         10,  desc_h)
        self.declare_parameter('red.saturation_l',  80,  desc_sv)
        self.declare_parameter('red.saturation_h',  255, desc_sv)
        self.declare_parameter('red.lightness_l',   120, desc_sv)
        self.declare_parameter('red.lightness_h',   255, desc_sv)

        self.declare_parameter('yellow.hue_l',      15,  desc_h)
        self.declare_parameter('yellow.hue_h',      35,  desc_h)
        self.declare_parameter('yellow.saturation_l',70,  desc_sv)
        self.declare_parameter('yellow.saturation_h',255, desc_sv)
        self.declare_parameter('yellow.lightness_l', 140, desc_sv)
        self.declare_parameter('yellow.lightness_h', 255, desc_sv)

        self.declare_parameter('green.hue_l',       40,  desc_h)
        self.declare_parameter('green.hue_h',       90,  desc_h)
        self.declare_parameter('green.saturation_l',60,  desc_sv)
        self.declare_parameter('green.saturation_h',255, desc_sv)
        self.declare_parameter('green.lightness_l', 120, desc_sv)
        self.declare_parameter('green.lightness_h', 255, desc_sv)
        # ================================================

        # 캘리브레이션 모드
        self.declare_parameter('is_calibration_mode', False)
        self.is_calibration_mode = self.get_parameter('is_calibration_mode').get_parameter_value().bool_value
        if self.is_calibration_mode:
            self.add_on_set_parameters_callback(self._on_param_update)

        # 파라미터 로드 헬퍼
        def gp_int(name): return self.get_parameter(name).get_parameter_value().integer_value
        self.hue_red_l, self.hue_red_h = gp_int('red.hue_l'), gp_int('red.hue_h')
        self.sat_red_l, self.sat_red_h = gp_int('red.saturation_l'), gp_int('red.saturation_h')
        self.val_red_l, self.val_red_h = gp_int('red.lightness_l'), gp_int('red.lightness_h')
        self.hue_yel_l, self.hue_yel_h = gp_int('yellow.hue_l'), gp_int('yellow.hue_h')
        self.sat_yel_l, self.sat_yel_h = gp_int('yellow.saturation_l'), gp_int('yellow.saturation_h')
        self.val_yel_l, self.val_yel_h = gp_int('yellow.lightness_l'), gp_int('yellow.lightness_h')
        self.hue_grn_l, self.hue_grn_h = gp_int('green.hue_l'), gp_int('green.hue_h')
        self.sat_grn_l, self.sat_grn_h = gp_int('green.saturation_l'), gp_int('green.saturation_h')
        self.val_grn_l, self.val_grn_h = gp_int('green.lightness_l'), gp_int('green.lightness_h')

        # 토픽 타입
        self.sub_image_type = 'raw'         # 입력 Image (원하면 'compressed'로 변경 가능)
        self.pub_image_type = 'compressed'  # 디버그 출력

        # 프레임 스킵 카운터
        self.frame_mod_counter = 1

        # 구독자(리매핑 권장: /detect/image_input -> /camera/image_compensated)
        if self.sub_image_type == 'compressed':
            self.sub_image_original = self.create_subscription(
                CompressedImage, '/detect/image_input/compressed', self._on_image_msg, 1)
        else:
            self.sub_image_original = self.create_subscription(
                Image, '/detect/image_input', self._on_image_msg, 1)

        # 디버그 이미지 퍼블리셔
        if self.pub_image_type == 'compressed':
            self.pub_image_debug = self.create_publisher(CompressedImage, '/detect/image_output/compressed', 1)
        else:
            self.pub_image_debug = self.create_publisher(Image, '/detect/image_output', 1)

        # 캘리브레이션 모드: 마스크 퍼블리셔
        if self.is_calibration_mode:
            if self.pub_image_type == 'compressed':
                self.pub_mask_red    = self.create_publisher(CompressedImage, '/detect/image_output_sub1/compressed', 1)
                self.pub_mask_yellow = self.create_publisher(CompressedImage, '/detect/image_output_sub2/compressed', 1)
                self.pub_mask_green  = self.create_publisher(CompressedImage, '/detect/image_output_sub3/compressed', 1)
            else:
                self.pub_mask_red    = self.create_publisher(Image, '/detect/image_output_sub1', 1)
                self.pub_mask_yellow = self.create_publisher(Image, '/detect/image_output_sub2', 1)
                self.pub_mask_green  = self.create_publisher(Image, '/detect/image_output_sub3', 1)

        # OpenCV
        self.cvbridge = CvBridge()
        self.latest_bgr = None
        self.has_new_image = False

        # 타이머(ROS 토픽 모드에서만 사용)
        time.sleep(1)
        self.timer = self.create_timer(0.1, self._on_timer_tick)

    # ===== 오프라인 테스트용 프레임 주입 =====
    def feed_frame_for_test(self, bgr_frame) -> bool:
        if FRAME_SKIP_3X:
            if self.frame_mod_counter % 3 != 0:
                self.frame_mod_counter += 1
                return False
            else:
                self.frame_mod_counter = 1
        self.latest_bgr = bgr_frame.copy()
        self.has_new_image = True
        return True

    # ===== 상태 안정화 & 퍼블리시 =====
    def _update_state_and_maybe_publish(self, instant_state: str):
        if instant_state == self._candidate_state:
            self._candidate_streak += 1
        else:
            self._candidate_state = instant_state
            self._candidate_streak = 1

        if self._candidate_state != self._stable_state and self._candidate_streak >= self.MIN_STREAK:
            self._stable_state = self._candidate_state
            msg = String(); msg.data = self._stable_state
            self.pub_traffic_light_state.publish(msg)
            self.get_logger().info(f'[STATE] -> {self._stable_state} by {self._candidate_streak} frames')

    # ===== 파라미터 런타임 반영 =====
    def _on_param_update(self, params):
        for p in params:
            n, v = p.name, p.value
            if   n == 'red.hue_l': self.hue_red_l = v
            elif n == 'red.hue_h': self.hue_red_h = v
            elif n == 'red.saturation_l': self.sat_red_l = v
            elif n == 'red.saturation_h': self.sat_red_h = v
            elif n == 'red.lightness_l': self.val_red_l = v
            elif n == 'red.lightness_h': self.val_red_h = v
            elif n == 'yellow.hue_l': self.hue_yel_l = v
            elif n == 'yellow.hue_h': self.hue_yel_h = v
            elif n == 'yellow.saturation_l': self.sat_yel_l = v
            elif n == 'yellow.saturation_h': self.sat_yel_h = v
            elif n == 'yellow.lightness_l': self.val_yel_l = v
            elif n == 'yellow.lightness_h': self.val_yel_h = v
            elif n == 'green.hue_l': self.hue_grn_l = v
            elif n == 'green.hue_h': self.hue_grn_h = v
            elif n == 'green.saturation_l': self.sat_grn_l = v
            elif n == 'green.saturation_h': self.sat_grn_h = v
            elif n == 'green.lightness_l': self.val_grn_l = v
            elif n == 'green.lightness_h': self.val_grn_h = v
            else:
                continue
            self.get_logger().info(f'{n} set to: {v}')
        return SetParametersResult(successful=True)

    # ===== ROS 이미지 콜백 =====
    def _on_image_msg(self, image_msg):
        # if FRAME_SKIP_3X:
        #     if self.frame_mod_counter % 3 != 0:
        #         self.frame_mod_counter += 1
        #         return
        #     else:
        #         self.frame_mod_counter = 1

        if self.sub_image_type == 'compressed':
            np_arr = np.frombuffer(image_msg.data, np.uint8)
            self.latest_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        else:
            try:
                self.latest_bgr = self.cvbridge.imgmsg_to_cv2(image_msg, 'bgr8')
            except CvBridgeError as e:
                self.get_logger().error(f'CvBridge Error: {e}')
                return
        self.has_new_image = True

    # ===== 주기 처리 =====
    def _on_timer_tick(self):
        if self.has_new_image:
            self._detect_and_publish()

    # ===== 캘리브레이션 마스크 퍼블리시 =====
    def _publish_mask_if_needed(self, mask, pub_image):
        if not self.is_calibration_mode:
            return
        if self.pub_image_type == 'compressed':
            pub_image.publish(self.cvbridge.cv2_to_compressed_imgmsg(mask, 'png'))
        else:
            pub_image.publish(self.cvbridge.cv2_to_imgmsg(mask, 'mono8'))

    # ===== HSV 마스크(레드 랩어라운드) =====
    def _hsv_mask_with_wrap(self, hsv, h_l, h_h, s_l, s_h, v_l, v_h):
        if h_l <= h_h:
            return cv2.inRange(hsv, np.array([h_l, s_l, v_l]), np.array([h_h, s_h, v_h]))
        m1 = cv2.inRange(hsv, np.array([0,   s_l, v_l]), np.array([h_h, s_h, v_h]))
        m2 = cv2.inRange(hsv, np.array([h_l, s_l, v_l]), np.array([179, s_h, v_h]))
        return cv2.bitwise_or(m1, m2)

    def _mask_red(self, hsv):
        mask = self._hsv_mask_with_wrap(hsv, self.hue_red_l, self.hue_red_h, self.sat_red_l, self.sat_red_h, self.val_red_l, self.val_red_h)
        if self.is_calibration_mode: self._publish_mask_if_needed(mask, self.pub_mask_red)
        return mask

    def _mask_yellow(self, hsv):
        mask = self._hsv_mask_with_wrap(hsv, self.hue_yel_l, self.hue_yel_h, self.sat_yel_l, self.sat_yel_h, self.val_yel_l, self.val_yel_h)
        if self.is_calibration_mode: self._publish_mask_if_needed(mask, self.pub_mask_yellow)
        return mask

    def _mask_green(self, hsv):
        mask = self._hsv_mask_with_wrap(hsv, self.hue_grn_l, self.hue_grn_h, self.sat_grn_l, self.sat_grn_h, self.val_grn_l, self.val_grn_h)
        if self.is_calibration_mode: self._publish_mask_if_needed(mask, self.pub_mask_green)
        return mask

    # ===== 메인 파이프라인 =====
    def _detect_and_publish(self):
        bgr = np.copy(self.latest_bgr)
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

        # 색상별 마스크
        mask_red    = self._mask_red(hsv)
        mask_yellow = self._mask_yellow(hsv)
        mask_green  = self._mask_green(hsv)

        # 공통 Bright/Sat 게이트 적용
        bright = cv2.inRange(hsv, np.array([0, S_MIN, V_MIN]), np.array([179, 255, 255]))
        mask_red    = cv2.bitwise_and(mask_red,    bright)
        mask_yellow = cv2.bitwise_and(mask_yellow, bright)
        mask_green  = cv2.bitwise_and(mask_green,  bright)

        # (번짐 대응) 블러/모폴로지
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
        mask_red    = cv2.GaussianBlur(mask_red,    (7,7), 0)
        mask_yellow = cv2.GaussianBlur(mask_yellow, (7,7), 0)
        mask_green  = cv2.GaussianBlur(mask_green,  (7,7), 0)
        mask_red    = cv2.morphologyEx(mask_red,    cv2.MORPH_CLOSE, kernel, iterations=1)
        mask_yellow = cv2.morphologyEx(mask_yellow, cv2.MORPH_CLOSE, kernel, iterations=1)
        mask_green  = cv2.morphologyEx(mask_green,  cv2.MORPH_CLOSE, kernel, iterations=1)

        # 마스크 카운트 로그
        cnt_r = cv2.countNonZero(mask_red)
        cnt_y = cv2.countNonZero(mask_yellow)
        cnt_g = cv2.countNonZero(mask_green)
        self.get_logger().info(f'[MASK] R={cnt_r} Y={cnt_y} G={cnt_g}')

        # 마스크 가드(전부 하양/전부 검정 방지)
        total_px = hsv.shape[0] * hsv.shape[1]
        def mask_ok(m):
            nz = cv2.countNonZero(m)
            if nz == 0 or nz == total_px:
                self.get_logger().info(f'[MASK-GUARD] skip (nz={nz}, total={total_px})')
                return False
            return True

        red_ok    = mask_ok(mask_red)
        yellow_ok = mask_ok(mask_yellow)
        green_ok  = mask_ok(mask_green)

        # 탐지
        is_red    = self._find_circle_and_validate(mask_red,    'red')    if red_ok    else False
        is_yellow = self._find_circle_and_validate(mask_yellow, 'yellow') if yellow_ok else False
        is_green  = self._find_circle_and_validate(mask_green,  'green')  if green_ok  else False

        # 라벨 텍스트
        if is_red:
            cv2.putText(self.latest_bgr, 'RED', (self.last_circle_x, self.last_circle_y),
                        cv2.FONT_HERSHEY_DUPLEX, 0.5, (0, 0, 255), 1)
        if is_yellow:
            cv2.putText(self.latest_bgr, 'YELLOW', (self.last_circle_x, self.last_circle_y),
                        cv2.FONT_HERSHEY_DUPLEX, 0.5, (0, 255, 255), 1)
        if is_green:
            cv2.putText(self.latest_bgr, 'GREEN', (self.last_circle_x, self.last_circle_y),
                        cv2.FONT_HERSHEY_DUPLEX, 0.5, (0, 255, 0), 1)

        # 상태 퍼블리시
        instant_state = 'red' if is_red else 'yellow' if is_yellow else 'green' if is_green else 'none'
        self._update_state_and_maybe_publish(instant_state)

        # rqt 스트림 확인용 고정 텍스트
        cv2.putText(self.latest_bgr, f'DBG state={instant_state}', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)

        # 디버그 프레임 퍼블리시
        if self.pub_image_type == 'compressed':
            self.pub_image_debug.publish(self.cvbridge.cv2_to_compressed_imgmsg(self.latest_bgr, 'jpg'))
        else:
            self.pub_image_debug.publish(self.cvbridge.cv2_to_imgmsg(self.latest_bgr, 'bgr8'))

        self.has_new_image = False

    # ===== 허프 + (후검증 OFF → 즉시 통과) =====
    def _find_circle_and_validate(self, mask_pos, color: str) -> bool:
        h, w = mask_pos.shape[:2]
        roi_x0, roi_x1 = int(ROI_X0_FRAC * w), int(ROI_X1_FRAC * w)
        roi_y0, roi_y1 = int(ROI_Y0_FRAC * h), int(ROI_Y1_FRAC * h)

        # ROI 시각화
        cv2.rectangle(self.latest_bgr, (roi_x0, roi_y0), (roi_x1, roi_y1), (0, 255, 0), 2)

        roi = mask_pos[roi_y0:roi_y1, roi_x0:roi_x1]
        roi_for_hough = cv2.medianBlur(roi, 5)

        circles = cv2.HoughCircles(
            roi_for_hough, cv2.HOUGH_GRADIENT,
            dp=HOUGH_DP, minDist=HOUGH_MIN_DIST,
            param1=HOUGH_PARAM1, param2=HOUGH_PARAM2,
            minRadius=R_MIN, maxRadius=R_MAX
        )
        if circles is None:
            self.get_logger().info(f'[HOUGH] none in ROI {roi.shape[1]}x{roi.shape[0]}')
            return False

        circles = np.uint16(np.around(circles))
        self.get_logger().info(f'[HOUGH] {len(circles[0])} circles, radii={[int(r) for _,_,r in circles[0][:5]]}')

        for x_roi, y_roi, r in circles[0]:
            cx = int(x_roi + roi_x0)
            cy = int(y_roi + roi_y0)
            if not (R_MIN <= r <= R_MAX):
                continue

            # 후검증 OFF: 허프가 잡으면 즉시 통과
            if not ENABLE_VALIDATION:
                self.last_circle_x, self.last_circle_y = cx, cy
                cv2.circle(self.latest_bgr, (cx, cy), r, (255, 0, 0), 2)
                self.get_logger().info(f'{color} detected (NO post-validation, r={r})')
                return True

            # (필요 시 재활성화)
            # if self._validate_circle_scored(self.latest_bgr, mask_pos, cx, cy, r, color):
            #     self.last_circle_x, self.last_circle_y = cx, cy
            #     cv2.circle(self.latest_bgr, (cx, cy), r, (255, 0, 0), 2)
            #     self.get_logger().info(f'{color} detected (r={r})')
            #     return True

        return False

    # (보류) 후검증 더미
    def _validate_circle_scored(self, *args, **kwargs):
        return True


# =========================
# 오프라인 비디오 루프
# =========================
def offline_video_loop(node: DetectTrafficLight, video_path: str, show: bool, write_path: str, fps_opt: float):
    if not os.path.exists(video_path):
        node.get_logger().error(f'Video not found: {video_path}')
        return

    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        node.get_logger().error(f'Cannot open video: {video_path}')
        return

    writer = None
    if write_path:
        width  = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fourcc = cv2.VideoWriter_fourcc(*('mp4v' if write_path.lower().endswith('.mp4') else 'XVID'))
        fps_in = cap.get(cv2.CAP_PROP_FPS) or 30.0
        fps = fps_opt if fps_opt > 0 else (fps_in if fps_in > 1e-3 else 30.0)
        writer = cv2.VideoWriter(write_path, fourcc, fps, (width, height))
        node.get_logger().info(f'Writing annotated video to: {write_path} @ {fps:.2f} FPS')

    node.get_logger().info('=== Offline video test mode START ===')

    can_show = bool(show)
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        if node.feed_frame_for_test(frame):
            node._detect_and_publish()

        if writer is not None:
            writer.write(node.latest_bgr if node.latest_bgr is not None else frame)

        if can_show:
            try:
                disp = node.latest_bgr if node.latest_bgr is not None else frame
                if not hasattr(offline_video_loop, "_win_created"):
                    cv2.namedWindow('traffic_light_debug', cv2.WINDOW_NORMAL)
                    offline_video_loop._win_created = True
                cv2.imshow('traffic_light_debug', disp)
                key = cv2.waitKey(1) & 0xFF
                if key in (27, ord('q')):
                    break
            except cv2.error as e:
                node.get_logger().warn(f'GUI backend unavailable, disabling window (reason: {e})')
                can_show = False

        rclpy.spin_once(node, timeout_sec=0.0)

    cap.release()
    if writer is not None:
        writer.release()
    if can_show:
        cv2.destroyAllWindows()
    node.get_logger().info('=== Offline video test mode END ===')


def main(args=None):
    rclpy.init(args=args)
    node = DetectTrafficLight()

    if USE_OFFLINE_VIDEO:
        offline_video_loop(node, DEFAULT_TEST_VIDEO, SHOW_WINDOW, WRITE_PATH, FORCE_FPS)
        node.destroy_node()
        rclpy.shutdown()
    else:
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
