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
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
from sensor_msgs.msg import Image, CameraInfo
import numpy as np

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        # OpenCV와 ROS 간 변환을 위한 CvBridge 초기화, <=Compressed Image : No need to O numpy => ros2 image
        self.bridge = CvBridge()
        
        # Declare parameter with default integer value
        self.video_port = self.declare_parameter('video_port', 0).get_parameter_value().integer_value
        self.image_width = self.declare_parameter('image_width', 320).get_parameter_value().integer_value
        self.image_height = self.declare_parameter('image_height', 240).get_parameter_value().integer_value
        self.video_fps = self.declare_parameter('video_fps', 25).get_parameter_value().integer_value


        self.get_logger().info(f'Get...... video_port parameter : {self.video_port}')

        # image_raw 이미지 퍼블리셔 생성
        self.image_raw_publisher_ = self.create_publisher(Image, 'image_raw', 10)
        self.image_publisher_ = self.create_publisher(Image, 'image', 10)

        # jpeg compressed corped 이미지 퍼블리셔 생성
        self.publisher_ = self.create_publisher(CompressedImage, 'image_croped_raw/compressed', 10)
        # jped full 이미지 퍼블리셔 생성
        self.full_publisher_ = self.create_publisher(CompressedImage, 'image_raw/compressed', 10)
        


        # 주기적인 이미지 전송을 위한 타이머 설정 (주기: 1초)
        self.timer = self.create_timer(0.2, self.publish_image)

        # OpenCV 비디오 캡처 객체 생성 (카메라 0번 장치 사용)
        self.cap = cv2.VideoCapture(self.video_port)

        self.cap.set(cv2.CAP_PROP_FPS, self.video_fps)    
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.image_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.image_height)

        # 카메라에서 한 프레임 읽기
        ret, frame = self.cap.read()

        # Get raw image size (before compression)
        height, width = frame.shape[:2]
        self.get_logger().info(f'Raw frame size: width={width}, height={height}')

        if ret:
            self.get_logger().info(f"Original Frame Information")
            self.get_logger().info(f"Frame type:, {type(frame)}")
            self.get_logger().info(f"Shape (Height, Width, Channels):, {frame.shape}")
            self.get_logger().info(f"Height:, {frame.shape[0]}")
            self.get_logger().info(f"Width:, {frame.shape[1]}")
            self.get_logger().info(f"Channels:, {frame.shape[2]}")  # Usually 3 for BGR
            self.get_logger().info(f"Data type:, {frame.dtype}")
            self.get_logger().info(f"--------------------------------")       

            self.info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', 10)
            #self.bridge = CvBridge()
            #self.timer = self.create_timer(0.1, self.publish)

            # Set dummy camera parameters
            self.camera_info = CameraInfo()

            if self.image_width == 640:
                self.camera_info.width = 640
                self.camera_info.height = 480
                self.camera_info.k = [525.0, 0.0, 320.0,
                                    0.0, 525.0, 240.0,
                                    0.0, 0.0, 1.0]  # dummy intrinsic matrix
                self.camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
                self.camera_info.r = [1.0, 0.0, 0.0,
                                    0.0, 1.0, 0.0,
                                    0.0, 0.0, 1.0]
                self.camera_info.p = [525.0, 0.0, 320.0, 0.0,
                                    0.0, 525.0, 240.0, 0.0,
                                    0.0, 0.0, 525.0, 0.0]
                
            elif self.image_width == 320:
                self.camera_info.width = 320
                self.camera_info.height = 240
                self.camera_info.k = [262.5, 0.0, 160.0,
                                        0.0, 262.5, 120.0,
                                        0.0, 0.0, 1.0]  # dummy intrinsic matrix
                self.camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
                self.camera_info.r = [1.0, 0.0, 0.0,
                                    0.0, 1.0, 0.0,
                                    0.0, 0.0, 1.0]
                self.camera_info.p = [262.5, 0.0, 160.0, 0.0,
                                    0.0, 262.5, 120.0, 0.0,
                                    0.0, 0.0, 1.0, 0.0]


            self.camera_info.distortion_model = 'plumb_bob'

    #YUV 색공간을 사용하여 **명도(Y)**만 히스토그램 평활화(equalizeHist)를 적용
    def auto_exposure(self,frame):
        yuv = cv2.cvtColor(frame, cv2.COLOR_BGR2YUV)
        yuv[:, :, 0] = cv2.equalizeHist(yuv[:, :, 0])
        return cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR)
    #LAB 색공간에서 색상 보정을 수행합니다. 특히 **a 채널(녹-적)과 b 채널(청-황)**의 평균값을 기준으로 자동 보정을 수행
    def white_balance(self,img):
        lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB).astype(np.float32)

        avg_a = np.average(lab[:, :, 1])
        avg_b = np.average(lab[:, :, 2])

        lab[:, :, 1] -= ((avg_a - 128) * (lab[:, :, 0] / 255.0) * 1.1)
        lab[:, :, 2] -= ((avg_b - 128) * (lab[:, :, 0] / 255.0) * 1.1)

        lab = np.clip(lab, 0, 255).astype(np.uint8)
        return cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)
    

    def publish_image(self):
        # 카메라에서 한 프레임 읽기
        ret, frame = self.cap.read()

        # 자동 노출 보정 + 화이트 밸런싱
        #frame = self.auto_exposure(frame) #Not good
        frame = self.white_balance(frame)

        frame = cv2.convertScaleAbs(frame, alpha=1.0, beta=-20) #<= Brightness -20
        #frame = cv2.convertScaleAbs(frame, alpha=1.5, beta=50) # <= Brightness + 50
        # 대비를 1.5배, 밝기를 50 증가
        # alpha: contrast, beta: brightness shift (negative = darker)

        # HSV 변환
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)


        
        if ret:
            ########## 1. Raw Image, Convert OpenCV image (numpy) to ROS Image message
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            now = self.get_clock().now().to_msg()

            msg.header.stamp = now

            msg.header.frame_id = 'camera_frame'

            self.image_raw_publisher_.publish(msg)
            #self.get_logger().info('Published image to /image_raw')

            ############## camera/image_raw topic publish
            self.image_publisher_.publish(msg)
        
            #### camera_info topic publish
            self.camera_info.header.stamp = now
            self.camera_info.header.frame_id = 'camera_frame'
            self.info_pub.publish(self.camera_info)

            
            ######### 2. Compressed Image, with some Rectanges
            #eight, width, _ = frame.shape
            #rect_w, rect_h = 300, 300
            #x1 = (width - rect_w) // 2
            #y1 = (height - rect_h) // 2
            #x2 = x1 + rect_w
            #y2 = y1 + rect_h

            #top_left = (x1, y1)
            #bottom_right = (x2, y2)
            #color = (200, 100,100)  # Green
            #thickness = 3        # Outline only
            
            #cv2.rectangle(frame, top_left, bottom_right, color, thickness)


            # Draw ROI rectangle: from (50, 0) to (1250, 720)
            #top_left = (50,720-50)
            #bottom_right = (50,1280-50)
            #color = (0, 255, 0)  # Green
            #thickness = 2        # Outline only

            #cv2.rectangle(frame, top_left, bottom_right, color, thickness)

            # OpenCV 이미지 (BGR)을 JPEG로 압축
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 30]  # 90은 압축 품질
            _, compressed_image = cv2.imencode('.jpg', frame, encode_param)

            # 압축된 이미지를 CompressedImage 메시지로 변환
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()  # 타임스탬프 추가
            msg.header.frame_id = "camera"  # 프레임 ID 설정
            msg.format = "jpeg"  # 압축 형식 설정
            msg.data = compressed_image.tobytes()  # 압축된 이미지 데이터

            # CompressedImage 퍼블리시
            self.full_publisher_.publish(msg)

            """
            #########  3. Croped Compressed Image for ROI
            # Crop 50px from each side
            frame = frame[50:720-50, 50:1280-50]  # frame[50:670, 50:1230]


             # OpenCV 이미지 (BGR)을 JPEG로 압축
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 30]  # 90은 압축 품질
            _, compressed_image = cv2.imencode('.jpg', frame, encode_param)

            # 압축된 이미지를 CompressedImage 메시지로 변환
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()  # 타임스탬프 추가
            msg.header.frame_id = "camera"  # 프레임 ID 설정
            msg.format = "jpeg"  # 압축 형식 설정
            msg.data = compressed_image.tobytes()  # 압축된 이미지 데이터

            # CompressedImage 퍼블리시
            # self.publisher_.publish(msg)
            """
        else:
            self.get_logger().error('Video Pen Error!...')

def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()
    
    # ROS 2 노드 실행
    rclpy.spin(image_publisher)

    # 종료 시 리소스 해제
    image_publisher.cap.release()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

