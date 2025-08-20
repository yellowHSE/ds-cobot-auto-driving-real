#!/usr/bin/env python3
# ... (license/header 동일)

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


class DetectLane(Node):

    def __init__(self):
        super().__init__('detect_lane')

        parameter_descriptor_hue = ParameterDescriptor(
            description='hue parameter range',
            integer_range=[IntegerRange(from_value=0, to_value=179, step=1)]
        )
        parameter_descriptor_saturation_lightness = ParameterDescriptor(
            description='saturation and lightness range',
            integer_range=[IntegerRange(from_value=0, to_value=255, step=1)]
        )

        self.declare_parameters(
            namespace='',
            parameters=[
                ('detect.lane.white.hue_l', 0,   parameter_descriptor_hue),
                ('detect.lane.white.hue_h', 179, parameter_descriptor_hue),
                ('detect.lane.white.saturation_l', 0,  parameter_descriptor_saturation_lightness),
                ('detect.lane.white.saturation_h', 70, parameter_descriptor_saturation_lightness),
                ('detect.lane.white.lightness_l', 105, parameter_descriptor_saturation_lightness),
                ('detect.lane.white.lightness_h', 255, parameter_descriptor_saturation_lightness),
                ('detect.lane.yellow.hue_l', 10,  parameter_descriptor_hue),
                ('detect.lane.yellow.hue_h', 127, parameter_descriptor_hue),
                ('detect.lane.yellow.saturation_l', 70,  parameter_descriptor_saturation_lightness),
                ('detect.lane.yellow.saturation_h', 255, parameter_descriptor_saturation_lightness),
                ('detect.lane.yellow.lightness_l', 95,   parameter_descriptor_saturation_lightness),
                ('detect.lane.yellow.lightness_h', 255,  parameter_descriptor_saturation_lightness),
                ('is_detection_calibration_mode', False)
            ]
        )

        # --- params load
        self.hue_white_l = self.get_parameter('detect.lane.white.hue_l').get_parameter_value().integer_value
        self.hue_white_h = self.get_parameter('detect.lane.white.hue_h').get_parameter_value().integer_value
        self.saturation_white_l = self.get_parameter('detect.lane.white.saturation_l').get_parameter_value().integer_value
        self.saturation_white_h = self.get_parameter('detect.lane.white.saturation_h').get_parameter_value().integer_value
        self.lightness_white_l = self.get_parameter('detect.lane.white.lightness_l').get_parameter_value().integer_value
        self.lightness_white_h = self.get_parameter('detect.lane.white.lightness_h').get_parameter_value().integer_value

        self.hue_yellow_l = self.get_parameter('detect.lane.yellow.hue_l').get_parameter_value().integer_value
        self.hue_yellow_h = self.get_parameter('detect.lane.yellow.hue_h').get_parameter_value().integer_value
        self.saturation_yellow_l = self.get_parameter('detect.lane.yellow.saturation_l').get_parameter_value().integer_value
        self.saturation_yellow_h = self.get_parameter('detect.lane.yellow.saturation_h').get_parameter_value().integer_value
        self.lightness_yellow_l = self.get_parameter('detect.lane.yellow.lightness_l').get_parameter_value().integer_value
        self.lightness_yellow_h = self.get_parameter('detect.lane.yellow.lightness_h').get_parameter_value().integer_value

        self.is_calibration_mode = self.get_parameter('is_detection_calibration_mode').get_parameter_value().bool_value
        if self.is_calibration_mode:
            self.add_on_set_parameters_callback(self.cbGetDetectLaneParam)

        self.sub_image_type = 'raw'
        self.pub_image_type = 'compressed'

        if self.sub_image_type == 'compressed':
            self.sub_image_original = self.create_subscription(
                CompressedImage, '/detect/image_input/compressed', self.cbFindLane, 10)
        else:
            self.sub_image_original = self.create_subscription(
                Image, '/detect/image_input', self.cbFindLane, 10)

        if self.pub_image_type == 'compressed':
            self.pub_image_lane = self.create_publisher(
                CompressedImage, '/detect/image_output/compressed', 1)
        else:
            self.pub_image_lane = self.create_publisher(
                Image, '/detect/image_output', 1)

        if self.is_calibration_mode:
            if self.pub_image_type == 'compressed':
                self.pub_image_white_lane = self.create_publisher(
                    CompressedImage, '/detect/image_output_sub1/compressed', 1)
                self.pub_image_yellow_lane = self.create_publisher(
                    CompressedImage, '/detect/image_output_sub2/compressed', 1)
            else:
                self.pub_image_white_lane = self.create_publisher(
                    Image, '/detect/image_output_sub1', 1)
                self.pub_image_yellow_lane = self.create_publisher(
                    Image, '/detect/image_output_sub2', 1)

        self.pub_lane = self.create_publisher(Float64, '/detect/lane', 1)
        self.pub_yellow_line_reliability = self.create_publisher(UInt8, '/detect/yellow_line_reliability', 1)
        self.pub_white_line_reliability  = self.create_publisher(UInt8, '/detect/white_line_reliability', 1)
        self.pub_lane_state = self.create_publisher(UInt8, '/detect/lane_state', 1)

        self.cvBridge = CvBridge()
        self.counter = 1
        self.window_width = 1000.
        self.window_height = 600.

        self.reliability_white_line = 100
        self.reliability_yellow_line = 100

        self.mov_avg_left = np.empty((0, 3))
        self.mov_avg_right = np.empty((0, 3))

        # 좌/우 선택 결과
        self.left_fraction_sel = 0
        self.right_fraction_sel = 0

        # --- EMA & 동적 차선폭
        self.alpha_fit = 0.2
        self.alpha_center = 0.3
        self.left_fit_ema = None
        self.right_fit_ema = None
        self.centerx_ema = None
        self.half_lane_px_ema = None
        self.delta_center_cap = 20.0  # 프레임당 중앙선 이동 제한(px)

        # --- fork handling params/state
        self.fork_ratio_thr = 1.25      # far/near 폭 비율 임계
        self.fork_hold_frames = 20      # 분기 감지 후 유지 프레임
        self.fork_frames_left = 0
        self.follow_preference = 'right' # 'left' | 'right' | 'center'
        self.fork_bias_alpha_left  = 0.40
        self.fork_bias_alpha_right = 0.60
        self.fork_bias_alpha_center = 0.50

    def cbGetDetectLaneParam(self, parameters):
        for param in parameters:
            if param.name == 'detect.lane.white.hue_l':             self.hue_white_l = param.value
            elif param.name == 'detect.lane.white.hue_h':           self.hue_white_h = param.value
            elif param.name == 'detect.lane.white.saturation_l':    self.saturation_white_l = param.value
            elif param.name == 'detect.lane.white.saturation_h':    self.saturation_white_h = param.value
            elif param.name == 'detect.lane.white.lightness_l':     self.lightness_white_l = param.value
            elif param.name == 'detect.lane.white.lightness_h':     self.lightness_white_h = param.value
            elif param.name == 'detect.lane.yellow.hue_l':          self.hue_yellow_l = param.value
            elif param.name == 'detect.lane.yellow.hue_h':          self.hue_yellow_h = param.value
            elif param.name == 'detect.lane.yellow.saturation_l':   self.saturation_yellow_l = param.value
            elif param.name == 'detect.lane.yellow.saturation_h':   self.saturation_yellow_h = param.value
            elif param.name == 'detect.lane.yellow.lightness_l':    self.lightness_yellow_l = param.value
            elif param.name == 'detect.lane.yellow.lightness_h':    self.lightness_yellow_h = param.value
        return SetParametersResult(successful=True)

    # ----------------- 유틸 -----------------
    def _bridge_dashed(self, mask):
        """점선을 세로 방향으로 잇고 노이즈 제거."""
        k_close = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 11))
        k_open  = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        m = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k_close, iterations=1)
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN,  k_open,  iterations=1)
        return m

    def split_lr(self, mask):
        """마스크를 좌/우 반으로 분할."""
        h, w = mask.shape[:2]
        mid = w // 2
        left = mask[:, :mid]
        right = mask[:, mid:]
        return left, right, np.count_nonzero(left), np.count_nonzero(right), mid, w

    def restore_half_to_full(self, half_mask, side, w_full):
        """반측(좌/우) 마스크를 풀 이미지 폭으로 복원(다른 쪽은 0)."""
        h_half, w_half = half_mask.shape[:2]
        full = np.zeros((h_half, w_full), dtype=np.uint8)
        if side == 'left':
            full[:, :w_half] = half_mask
        else:
            full[:, w_full - w_half:] = half_mask
        return full

    def _ema_arr(self, prev, new, a):
        if prev is None:
            return new.copy()
        return (1 - a) * prev + a * new

    # ----------------------------------------------------------

    def cbFindLane(self, image_msg):
        # frame skipping
        # if self.counter % 3 != 0:
        #     self.counter += 1
        #     return
        # else:
        #     self.counter = 1

        if self.sub_image_type == 'compressed':
            np_arr = np.frombuffer(image_msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        else:
            cv_image = self.cvBridge.imgmsg_to_cv2(image_msg, 'bgr8')

        # 전체 마스크
        white_fraction_total, mask_white_full = self.maskWhiteLane(cv_image)
        yellow_fraction_total, mask_yellow_full = self.maskYellowLane(cv_image)

        # 좌/우 분할
        wl, wr, wl_cnt, wr_cnt, mid, w = self.split_lr(mask_white_full)
        yl, yr, yl_cnt, yr_cnt, _, _    = self.split_lr(mask_yellow_full)

        # 좌/우 각각에서 픽셀 수가 더 큰 색 선택
        # if yl_cnt >= wl_cnt:
        #     left_half_sel = yl
        #     left_fraction = yl_cnt
        # else:
        #     left_half_sel = wl
        #     left_fraction = wl_cnt

        # if wr_cnt >= yr_cnt:
        #     right_half_sel = wr
        #     right_fraction = wr_cnt
        # else:
        #     right_half_sel = yr
        #     right_fraction = yr_cnt
        
        left_half_sel = yl
        left_fraction = yl_cnt
        right_half_sel = wr
        right_fraction = wr_cnt

        # 반측 마스크를 원본 폭으로 복원
        mask_left_sel_full  = self.restore_half_to_full(left_half_sel,  'left',  w)
        mask_right_sel_full = self.restore_half_to_full(right_half_sel, 'right', w)

        self.left_fraction_sel  = left_fraction
        self.right_fraction_sel = right_fraction

        # ---------------- 피팅 ----------------
        try:
            if left_fraction  > 3000:
                self.left_fitx,  self.left_fit  = self.fit_from_lines(getattr(self, 'left_fit', np.array([0,0,0])),  mask_left_sel_full)
                self.mov_avg_left  = np.append(self.mov_avg_left,  np.array([self.left_fit]),  axis=0)
                self.left_fit_ema  = self._ema_arr(self.left_fit_ema, self.left_fit, self.alpha_fit)

            if right_fraction > 3000:
                self.right_fitx, self.right_fit = self.fit_from_lines(getattr(self, 'right_fit', np.array([0,0,0])), mask_right_sel_full)
                self.mov_avg_right = np.append(self.mov_avg_right, np.array([self.right_fit]), axis=0)
                self.right_fit_ema = self._ema_arr(self.right_fit_ema, self.right_fit, self.alpha_fit)
        except Exception:
            if left_fraction  > 3000:
                self.left_fitx,  self.left_fit  = self.sliding_windown(mask_left_sel_full,  'left')
                self.mov_avg_left  = np.array([self.left_fit])
                self.left_fit_ema  = self._ema_arr(self.left_fit_ema, self.left_fit, self.alpha_fit)
            if right_fraction > 3000:
                self.right_fitx, self.right_fit = self.sliding_windown(mask_right_sel_full, 'right')
                self.mov_avg_right = np.array([self.right_fit])
                self.right_fit_ema = self._ema_arr(self.right_fit_ema, self.right_fit, self.alpha_fit)

        MOV_AVG_LENGTH = 5
        if self.mov_avg_left.shape[0]  > 0:
            self.left_fit  = np.array([
                np.mean(self.mov_avg_left[::-1][:, 0][0:MOV_AVG_LENGTH]),
                np.mean(self.mov_avg_left[::-1][:, 1][0:MOV_AVG_LENGTH]),
                np.mean(self.mov_avg_left[::-1][:, 2][0:MOV_AVG_LENGTH])])
        if self.mov_avg_right.shape[0] > 0:
            self.right_fit = np.array([
                np.mean(self.mov_avg_right[::-1][:, 0][0:MOV_AVG_LENGTH]),
                np.mean(self.mov_avg_right[::-1][:, 1][0:MOV_AVG_LENGTH]),
                np.mean(self.mov_avg_right[::-1][:, 2][0:MOV_AVG_LENGTH])])

        if self.mov_avg_left.shape[0]  > 1000: self.mov_avg_left  = self.mov_avg_left[0:MOV_AVG_LENGTH]
        if self.mov_avg_right.shape[0] > 1000: self.mov_avg_right = self.mov_avg_right[0:MOV_AVG_LENGTH]

        # 렌더/퍼블리시
        self.make_lane(cv_image, left_fraction=self.left_fraction_sel, right_fraction=self.right_fraction_sel)

    # ---------------- 마스크 ----------------
    def maskWhiteLane(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_white = np.array([self.hue_white_l, self.saturation_white_l, self.lightness_white_l])
        upper_white = np.array([self.hue_white_h, self.saturation_white_h, self.lightness_white_h])
        mask = cv2.inRange(hsv, lower_white, upper_white)
        mask = self._bridge_dashed(mask)  # 점선 연결
        fraction_num = np.count_nonzero(mask)

        if not self.is_calibration_mode:
            if fraction_num > 35000 and self.lightness_white_l < 250:
                self.lightness_white_l += 5
            elif fraction_num < 5000 and self.lightness_white_l > 50:
                self.lightness_white_l -= 5

        # 신뢰도: morphology 이후 계산
        how_much_short = 600 - sum(np.count_nonzero(mask[i, :]) > 0 for i in range(0, 600))
        if how_much_short > 100:
            if self.reliability_white_line >= 5: self.reliability_white_line -= 5
        else:
            if self.reliability_white_line <= 99: self.reliability_white_line += 5

        msg = UInt8(); msg.data = self.reliability_white_line
        self.pub_white_line_reliability.publish(msg)

        if self.is_calibration_mode:
            if self.pub_image_type == 'compressed':
                self.pub_image_white_lane.publish(self.cvBridge.cv2_to_compressed_imgmsg(mask, 'jpg'))
            else:
                self.pub_image_white_lane.publish(self.cvBridge.cv2_to_imgmsg(mask, 'bgr8'))

        return fraction_num, mask

    def maskYellowLane(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([self.hue_yellow_l, self.saturation_yellow_l, self.lightness_yellow_l])
        upper_yellow = np.array([self.hue_yellow_h, self.saturation_yellow_h, self.lightness_yellow_h])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        mask = self._bridge_dashed(mask)  # 점선 연결
        fraction_num = np.count_nonzero(mask)

        # if self.is_calibration_mode:
        #     if fraction_num > 35000 and self.lightness_yellow_l < 250:
        #         self.lightness_yellow_l += 20
        #     elif fraction_num < 5000 and self.lightness_yellow_l > 90:
        #         self.lightness_yellow_l -= 20

        how_much_short = 600 - sum(np.count_nonzero(mask[i, :]) > 0 for i in range(0, 600))
        if how_much_short > 100:
            if self.reliability_yellow_line >= 5: self.reliability_yellow_line -= 5
        else:
            if self.reliability_yellow_line <= 99: self.reliability_yellow_line += 5

        msg = UInt8(); msg.data = self.reliability_yellow_line
        self.pub_yellow_line_reliability.publish(msg)

        if self.is_calibration_mode:
            if self.pub_image_type == 'compressed':
                self.pub_image_yellow_lane.publish(self.cvBridge.cv2_to_compressed_imgmsg(mask, 'jpg'))
            else:
                self.pub_image_yellow_lane.publish(self.cvBridge.cv2_to_imgmsg(mask, 'bgr8'))

        return fraction_num, mask

    # ---------------- 피팅 ----------------
    def fit_from_lines(self, lane_fit_prev, image):
        """로버스트 2패스 + 선형 폴백 + 곡률 클램프."""
        nonzero = image.nonzero()
        y = np.array(nonzero[0]); x = np.array(nonzero[1])

        ploty = np.linspace(0, image.shape[0] - 1, image.shape[0])

        if x.size < 80:
            p = np.polyfit(y, x, 1)
            fitx = p[0] * ploty + p[1]
            return fitx, np.array([0.0, p[0], p[1]])

        p1 = np.polyfit(y, x, 1)
        x_pred = p1[0]*y + p1[1]
        resid = np.abs(x - x_pred)
        thr = np.percentile(resid, 70)
        inliers = resid < thr
        if inliers.sum() < 60:
            inliers = resid < (np.median(resid) + 1.5*np.std(resid))

        p2 = np.polyfit(y[inliers], x[inliers], 2)
        fitx = p2[0]*ploty**2 + p2[1]*ploty + p2[2]

        # 곡률 클램프
        a_max = 1.2e-3
        if abs(p2[0]) > a_max and lane_fit_prev is not None:
            p2 = 0.7*lane_fit_prev + 0.3*p2
            fitx = p2[0]*ploty**2 + p2[1]*ploty + p2[2]
        return fitx, p2

    def sliding_windown(self, img_w, left_or_right):
        histogram = np.sum(img_w[int(img_w.shape[0] / 2):, :], axis=0)
        out_img = np.dstack((img_w, img_w, img_w)) * 255
        midpoint = np.int_(histogram.shape[0] / 2)

        lane_base = np.argmax(histogram[:midpoint]) if left_or_right == 'left' else np.argmax(histogram[midpoint:]) + midpoint
        nwindows = 20
        window_height = np.int_(img_w.shape[0] / nwindows)
        nonzero = img_w.nonzero()
        nonzeroy = np.array(nonzero[0]); nonzerox = np.array(nonzero[1])
        x_current = lane_base

        margin_base = 50
        minpix_base = 50
        lane_inds = []
        max_shift = 25  # 윈도우 간 x 이동 제한

        for window in range(nwindows):
            margin = margin_base + int(10 * (window / nwindows))
            minpix = max(20, minpix_base - int(2 * window))

            win_y_low = img_w.shape[0] - (window + 1) * window_height
            win_y_high = img_w.shape[0] - window * window_height
            win_x_low = x_current - margin
            win_x_high = x_current + margin
            cv2.rectangle(out_img, (win_x_low, win_y_low), (win_x_high, win_y_high), (0, 255, 0), 2)

            good_lane_inds = (
                (nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                (nonzerox >= win_x_low) & (nonzerox < win_x_high)
            ).nonzero()[0]
            lane_inds.append(good_lane_inds)

            if len(good_lane_inds) > minpix:
                x_new = int(np.mean(nonzerox[good_lane_inds]))
                if abs(x_new - x_current) > max_shift:
                    x_current += np.sign(x_new - x_current) * max_shift
                else:
                    x_current = x_new

        lane_inds = np.concatenate(lane_inds) if len(lane_inds) else np.array([], dtype=int)
        x = nonzerox[lane_inds]; y = nonzeroy[lane_inds]
        ploty = np.linspace(0, img_w.shape[0] - 1, img_w.shape[0])

        if x.size < 60:
            p = np.polyfit(y, x, 1)
            fitx = p[0]*ploty + p[1]
            return fitx, np.array([0.0, p[0], p[1]])

        p2 = np.polyfit(y, x, 2)
        fitx = p2[0]*ploty**2 + p2[1]*ploty + p2[2]
        return fitx, p2

    # ---------------- 합성/퍼블리시 ----------------
    def make_lane(self, cv_image, left_fraction, right_fraction):
        warp_zero = np.zeros((cv_image.shape[0], cv_image.shape[1], 1), dtype=np.uint8)
        color_warp = np.dstack((warp_zero, warp_zero, warp_zero))
        color_warp_lines = np.dstack((warp_zero, warp_zero, warp_zero))
        ploty = np.linspace(0, cv_image.shape[0] - 1, cv_image.shape[0])

        lane_state = UInt8()

        # 라인 렌더용 pts
        pts_left = None
        pts_right = None

        is_left_ok  = (left_fraction  > 3000 and hasattr(self, 'left_fitx'))
        is_right_ok = (right_fraction > 3000 and hasattr(self, 'right_fitx'))

        if is_left_ok:
            pts_left = np.array([np.flipud(np.transpose(np.vstack([self.left_fitx, ploty])) )])
            cv2.polylines(color_warp_lines, np.int_([pts_left]), False, (0, 0, 255), 25)

        if is_right_ok:
            pts_right = np.array([np.transpose(np.vstack([self.right_fitx, ploty]))])
            cv2.polylines(color_warp_lines, np.int_([pts_right]), False, (255, 255, 0), 25)

        self.is_center_x_exist = True
        centerx = None

        # --- 분기 감지 ---
        fork_detected = False
        if is_left_ok and is_right_ok:
            lane_width_curve = (self.right_fitx - self.left_fitx)
            h = lane_width_curve.shape[0]
            near_band = lane_width_curve[int(h*0.65):]   # 근거리
            far_band  = lane_width_curve[:int(h*0.25)]   # 원거리
            near_w = float(np.median(near_band)) if near_band.size else 0.0
            far_w  = float(np.median(far_band))  if far_band.size else 0.0
            if near_w > 1.0:
                ratio = far_w / near_w
                fork_detected = (ratio > self.fork_ratio_thr)
            # 동적 반차선폭 EMA 갱신
            half = np.clip(np.median(lane_width_curve) / 2.0, 180, 420)
            self.half_lane_px_ema = half if self.half_lane_px_ema is None else 0.9*self.half_lane_px_ema + 0.1*half

        if fork_detected:
            self.fork_frames_left = self.fork_hold_frames
        elif self.fork_frames_left > 0:
            self.fork_frames_left -= 1
        in_fork = (self.fork_frames_left > 0)

        # --- 중앙선 결정 ---
        if is_left_ok and is_right_ok:
            if in_fork:
                if self.follow_preference == 'left':
                    alpha = self.fork_bias_alpha_left
                elif self.follow_preference == 'right':
                    alpha = self.fork_bias_alpha_right
                else:
                    alpha = self.fork_bias_alpha_center
                centerx = self.left_fitx + alpha * (self.right_fitx - self.left_fitx)
            else:
                centerx = np.mean([self.left_fitx, self.right_fitx], axis=0)

            lane_state.data = 2
            pts_center = np.array([np.transpose(np.vstack([centerx, ploty]))])
            cv2.polylines(color_warp_lines, np.int_([pts_center]), False, (0, 255, 255), 12)

            if pts_left is not None and pts_right is not None:
                pts = np.hstack((pts_left, pts_right))
                cv2.fillPoly(color_warp, np.int_([pts]), (0, 255, 0))

        elif is_right_ok:
            offset = self.half_lane_px_ema if self.half_lane_px_ema is not None else 280.0
            centerx = self.right_fitx - offset
            lane_state.data = 3
            pts_center = np.array([np.transpose(np.vstack([centerx, ploty]))])
            cv2.polylines(color_warp_lines, np.int_([pts_center]), False, (0, 255, 255), 12)

        elif is_left_ok:
            offset = self.half_lane_px_ema if self.half_lane_px_ema is not None else 280.0
            centerx = self.left_fitx + offset
            lane_state.data = 1
            pts_center = np.array([np.transpose(np.vstack([centerx, ploty]))])
            cv2.polylines(color_warp_lines, np.int_([pts_center]), False, (0, 255, 255), 12)

        else:
            self.is_center_x_exist = False
            lane_state.data = 0

        self.pub_lane_state.publish(lane_state)
        self.get_logger().info(f'Lane state: {lane_state.data}')

        final = cv2.addWeighted(cv_image, 1, color_warp, 0.2, 0)
        final = cv2.addWeighted(final, 1, color_warp_lines, 1, 0)

        # 퍼블리시 (EMA + Δcenter 캡 포함)
        if self.pub_image_type == 'compressed':
            if self.is_center_x_exist and centerx is not None:
                self.centerx_ema = self._ema_arr(self.centerx_ema, centerx, self.alpha_center)
                publish_x = self.centerx_ema if self.centerx_ema is not None else centerx
                idx = 350
                val = float(publish_x.item(idx))
                if hasattr(self, '_last_pub_center'):
                    diff = np.clip(val - self._last_pub_center, -self.delta_center_cap, self.delta_center_cap)
                    val = self._last_pub_center + diff
                self._last_pub_center = val

                msg_desired_center = Float64()
                msg_desired_center.data = val
                self.pub_lane.publish(msg_desired_center)

            self.pub_image_lane.publish(self.cvBridge.cv2_to_compressed_imgmsg(final, 'jpg'))
        else:
            if self.is_center_x_exist and centerx is not None:
                self.centerx_ema = self._ema_arr(self.centerx_ema, centerx, self.alpha_center)
                publish_x = self.centerx_ema if self.centerx_ema is not None else centerx
                idx = 350
                val = float(publish_x.item(idx))
                if hasattr(self, '_last_pub_center'):
                    diff = np.clip(val - self._last_pub_center, -self.delta_center_cap, self.delta_center_cap)
                    val = self._last_pub_center + diff
                self._last_pub_center = val

                msg_desired_center = Float64()
                msg_desired_center.data = val
                self.pub_lane.publish(msg_desired_center)

            self.pub_image_lane.publish(self.cvBridge.cv2_to_imgmsg(final, 'bgr8'))


def main(args=None):
    rclpy.init(args=args)
    node = DetectLane()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
