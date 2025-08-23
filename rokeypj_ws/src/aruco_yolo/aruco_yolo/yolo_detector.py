#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Camera Frame Capture and JPEG Compression

This script captures frames from a webcam using OpenCV,
retrieves the raw image's width and height, compresses
the image to JPEG format, and also determines the size
of the decoded (compressed) image.

Author: Rujin Kim
Date: 2025-05-17
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
import cv2
import numpy as np
import json
import time
from ultralytics import YOLO
import os
from ament_index_python.packages import get_package_share_directory

# Author: Karl.Kwon
# Email: mrthinks@gmail.com

# 카메라 보정 파라미터 (카메라 내부 행렬 및 왜곡 계수)
mtx = np.array(
    [[1381.01926,    0.     ,  695.134361],
        [0.     , 1382.09839,  446.413163],
        [0.     ,    0.     ,    1.     ]]
        )
dist = np.array([[-0.01792346, 0.68685818, 0.0023631, -0.00455134, -2.06831632]])

# YOLO 모델 경로 불러오기
package_share_directory = get_package_share_directory('aruco_yolo')
model_path = os.path.join(package_share_directory, 'models', 'yolov8s_trained.pt')
# model_path = "/home/rokey-jw/ds-cobot-auto-driving-real/rokeypj_ws/src/aruco_yolo/models/new_new_new_best.pt"
print(f"Model path: {model_path}")

# 모델 파일 존재 여부 확인
if not os.path.exists(model_path):
    raise FileNotFoundError(f"Model file not found at {model_path}")

# 픽셀 → 실제 거리(mm) 변환 비율 (실험적으로 추정)
# pix_2_mm = 0.00012 # distance(m) - 175mm
pix_2_mm = 0.000153 # distance(m) - mm


class YoloDetect(Node):
    """
    ROS2 Node: YOLO를 이용해 객체 탐지 후
    - 결과 이미지를 퍼블리시 (/yolo/compressed)
    - 탐지 정보(JSON 형식)를 퍼블리시 (/yolo/detected_info)
    """

    def __init__(self):
        super().__init__('yolo_detect')
        print('model loading...')
        self.model = YOLO(model_path)  # YOLO 모델 로드
        print('model load done.')

        # 카메라 이미지 구독 (압축 이미지)
        self.subscription_rgb = self.create_subscription(
            CompressedImage,
            'image_raw/compressed',
            self.listener_callback_rgb, 10)
        self.subscription_rgb  # prevent unused variable warning

        # 탐지된 이미지 퍼블리셔
        self.img_publisher_ = self.create_publisher(CompressedImage, 'yolo/compressed', 1)
        # 탐지 정보 퍼블리셔
        self.info_publisher_ = self.create_publisher(String, 'yolo/detected_info', 5)

        self.last_pub_time = time.time()  # 마지막 퍼블리시 시간 기록


    def publish_img(self, frame):
        """
        YOLO 탐지 결과 이미지를 JPEG 압축 후 퍼블리시
        - frame: OpenCV 이미지
        """
        time_cur = time.time()

        # 0.2초 미만 간격이면 퍼블리시하지 않음 (과도한 송출 방지)
        if time_cur - 0.2 < self.last_pub_time:
            return

        self.last_pub_time = time_cur

        # JPEG 압축 품질 설정 (30으로 낮춰서 용량 절감)
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 30]  
        _, compressed_image = cv2.imencode('.jpg', frame, encode_param)

        # 압축 이미지를 ROS 메시지로 변환
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()  # 타임스탬프
        msg.header.frame_id = "camera"  # 프레임 ID
        msg.format = "jpeg"  # 포맷
        msg.data = compressed_image.tobytes()

        # 퍼블리시
        self.img_publisher_.publish(msg)
        # self.get_logger().info('Publishing compressed image...')


    def convert_box_center_2_mm(self, results, img_size, pix_2_mm):
        """
        탐지 결과 박스 중심 좌표를 mm 단위로 변환
        - results: YOLO 탐지 결과
        - img_size: (가로, 세로) 이미지 크기
        - pix_2_mm: 픽셀 → mm 변환 계수
        """
        cls = results[0].boxes.cls.tolist()
        centers = []
        centers_img = []
        
        for i, r in enumerate(results[0].boxes.xywh):
            d = r.tolist()
            # 중심 좌표를 이미지 중심 기준 mm 단위로 변환
            centers.append( (int(cls[i]), (d[0] - img_size[0]/2)*pix_2_mm, (d[1] - img_size[1]/2)*pix_2_mm) )
            centers_img.append( (d[0], d[1]) )
        
        return centers, centers_img


    def inference(self, img_color, pix_2_mm):
        """
        YOLO 모델을 이용해 객체 탐지 수행
        - 탐지 결과 이미지를 반환 (좌표 표시 포함)
        - 탐지된 객체의 중심 좌표 정보를 반환
        """
        results = self.model(img_color, conf=0.4)  # 신뢰도(conf) 0.4 이상만 탐지
        plots = results[0].plot()  # 탐지된 결과 시각화
        shp = img_color.shape

        # 이미지 중앙선(가로/세로) 표시
        plots = cv2.line(plots, (0, int(shp[0]/2)), (shp[1], int(shp[0]/2)), (0, 255, 255), 3)
        plots = cv2.line(plots, (int(shp[1]/2), 0), (int(shp[1]/2), shp[0]-1), (0, 255, 255), 3)

        # 중심 좌표 mm 변환
        info, info_img = self.convert_box_center_2_mm(results, (shp[1], shp[0]), pix_2_mm)

        # 탐지된 객체 좌표 이미지에 출력
        font = cv2.FONT_ITALIC
        font_scale = 1
        font_thickness = 2
        for i, (x, y) in enumerate(info_img):
            x = int(x)
            y = int(y)
            text = "%d, %d" % (info[i][1]*1000, info[i][2]*1000)  # mm 단위 텍스트 표시

            text_size, _ = cv2.getTextSize(text, font, font_scale, font_thickness)
            text_w, text_h = text_size
            org = (x, y)

            # 좌표 표시 배경 사각형 및 텍스트 출력
            cv2.rectangle(plots, (x - 5, y - 5), (x + text_w + 5, y + text_h + 5), (0, 0, 0), -1)
            cv2.putText(plots, text, (x, y + text_h + font_scale - 1), font, font_scale, (0, 255, 0), font_thickness)

        return plots, info


    def publish_info(self, str_):
        """
        탐지된 객체 정보를 String 메시지로 퍼블리시
        """
        msg = String()
        msg.data = str_
        self.info_publisher_.publish(msg)


    def listener_callback_rgb(self, msg):
        """
        카메라 압축 이미지를 수신했을 때 실행되는 콜백
        - 영상 보정(undistort) 후 YOLO 탐지 수행
        """
        # ROS CompressedImage → OpenCV 이미지 변환
        np_arr = np.frombuffer(msg.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        h,  w = image_np.shape[:2]

        ####################################################################3    
        # 카메라 보정 (왜곡 보정)
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))
        image_undistort = cv2.undistort(image_np, mtx, dist, None, newcameramtx)

        # YOLO 추론 실행
        plots, info = self.inference(image_undistort, pix_2_mm)

        # JSON 포맷으로 변환 후 퍼블리시
        info = json.dumps(info)
        # print(info)

        # 탐지 결과 이미지 크기 조정 (640x360)
        plots = cv2.resize(plots, (640, 360))

        # 정보 및 이미지 퍼블리시
        self.publish_info(info)
        self.publish_img(plots)


# ros2 topic echo /yolo/detected_info

def main(args=None):
    """
    메인 함수: ROS2 노드 실행
    """
    rclpy.init(args=args)
    image_subscriber = YoloDetect()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()