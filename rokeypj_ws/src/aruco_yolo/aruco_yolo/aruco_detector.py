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
import cv2
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory
import yaml
import argparse
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32, Int32
from aruco_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge
from std_msgs.msg import Float32


def detect_markers(image, camera_matrix, dist_coeffs, marker_size):
    """
    이미지에서 ArUco 마커를 탐지하고, 위치 및 자세를 계산하는 함수
    - image: 입력 이미지
    - camera_matrix: 카메라 내부 파라미터 행렬
    - dist_coeffs: 왜곡 계수
    - marker_size: 마커 한 변의 실제 길이 (미터 단위)
    """
    # OpenCV에서 제공하는 6x6 ArUco 마커 사전 사용 (최대 1000개 ID 지원)
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_1000)
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

    # 마커 탐지
    corners, ids, _ = detector.detectMarkers(image)
    detect_data = []

    if ids is not None:
        # 탐지된 마커 그리기
        cv2.aruco.drawDetectedMarkers(image, corners, ids)

        # 마커의 Pose (위치와 자세) 추정
        rvecs, tvecs, _ = my_estimatePoseSingleMarkers(corners, marker_size, camera_matrix, dist_coeffs)
        
        if rvecs is not None and tvecs is not None:
            for rvec, tvec, marker_id in zip(rvecs, tvecs, ids):
                # 회전 벡터를 회전 행렬로 변환
                rot_mat, _ = cv2.Rodrigues(rvec)
                # 오일러 각(yaw, pitch, roll) 계산
                yaw, pitch, roll = rotationMatrixToEulerAngles(rot_mat)
                # 카메라 좌표계에서 마커 위치 추정
                marker_pos = np.dot(-rot_mat.T, tvec).flatten()
                # 거리 계산 (tvec의 노름)
                distance = np.linalg.norm(tvec)
                # 마커 데이터 저장 [ID, 위치, 회전, 거리]
                detect_data.append([marker_id, marker_pos, (yaw, pitch, roll), distance])
    return image, detect_data


def my_estimatePoseSingleMarkers(corners, marker_size, mtx, distortion):
    """
    ArUco 마커의 3D Pose를 추정하는 함수 (solvePnP 사용)
    """
    marker_points = np.array([[-marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, -marker_size / 2, 0],
                              [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)
    rvecs = []
    tvecs = []
    for c in corners:
        _, R, t = cv2.solvePnP(marker_points, c, mtx, distortion, False, cv2.SOLVEPNP_IPPE_SQUARE)
        rvecs.append(R)
        tvecs.append(t)
    return rvecs, tvecs, []


def rotationMatrixToEulerAngles(R):
    """
    회전 행렬을 오일러 각 (roll, pitch, yaw)으로 변환
    """
    sy = np.sqrt(R[0,0] * R[0,0] + R[1,0] * R[1,0])
    singular = sy < 1e-6
    if not singular:
        x = np.arctan2(R[2,1], R[2,2])
        y = np.arctan2(-R[2,0], sy)
        z = np.arctan2(R[1,0], R[0,0])
    else:
        x = np.arctan2(-R[1,2], R[1,1])
        y = np.arctan2(-R[2,0], sy)
        z = 0
    return np.degrees(x), np.degrees(y), np.degrees(z)


def load_camera_parameters(yaml_file):
    """
    카메라 파라미터(YAML 파일)를 불러와서 내부행렬과 왜곡 계수 반환
    - calibration_params.yaml에서 camera_matrix와 distortion_coefficients를 읽음
    """
    package_share_directory = get_package_share_directory('aruco_yolo')
    calibration_file = os.path.join(package_share_directory, 'config', yaml_file)

    with open(calibration_file, 'r') as f:
        data = yaml.safe_load(f)
        camera_matrix = np.array(data["camera_matrix"]["data"], dtype=np.float32).reshape(3, 3)
        dist_coeffs = np.array(data["distortion_coefficients"]["data"], dtype=np.float32)
        
    return camera_matrix, dist_coeffs
    

class ArucoMarkerDetector(Node):
    """
    ROS2 Node: 카메라 영상에서 ArUco 마커를 탐지하고,
    가장 가까운 마커의 Pose와 거리 정보를 퍼블리시하는 노드
    """
    def __init__(self):
        super().__init__('aruco_marker_detector')

        self.get_logger().info(f'ArucoMarkerDetector(Node)')

        # 카메라 이미지 (압축) 구독
        self.subscription = self.create_subscription(
            CompressedImage,
            '/camera/image_raw/compressed',
            self.listener_callback,
            10)

        # 탐지된 마커 정보 퍼블리시
        self.marker_publisher = self.create_publisher(MarkerArray, 'detected_markers', 10)
        
        # 가장 가까운 마커까지의 거리 퍼블리시
        self.distance_publisher_ = self.create_publisher(Float32, '/aruco/distance', 10)

        # OpenCV와 ROS 이미지 변환을 위한 CvBridge
        self.bridge = CvBridge()
        self.marker_size = 0.04  # 마커 크기 (단위: m)
        
        # 카메라 캘리브레이션 파일 로드
        self.camera_matrix, self.dist_coeffs = load_camera_parameters('calibration_params.yaml')
        

    def listener_callback(self, msg):
        """
        카메라 이미지 수신 시 실행되는 콜백 함수
        - 마커 탐지 및 거리 계산 수행
        """
        # CompressedImage → OpenCV 이미지 변환
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
        # 카메라 보정 적용 후 마커 탐지
        frame, detect_data = detect_markers(frame, self.camera_matrix, self.dist_coeffs, self.marker_size)
        
        # detect_data = [marker_id, marker_pos, (yaw, pitch, roll), distance]

        if len(detect_data) == 0:
            self.get_logger().debug("No markers detected")
        else:
            # 가장 가까운 마커 선택
            closest_marker = min(detect_data, key=lambda x: x[3])

            # 거리 정보 문자열 작성 (디버깅용 화면 출력)
            distance = f"x:{closest_marker[1][0]:.2f},z:{closest_marker[1][2]:.2f},distance:{closest_marker[3]:.2f}m"

            # 거리 정보 퍼블리시
            msg_distance = Float32()
            msg_distance.data = closest_marker[3]
            self.distance_publisher_.publish(msg_distance)

            # 영상에 거리 정보 텍스트 표시
            cv2.putText(frame, distance, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, 
                        (0, 255, 0), 1, cv2.LINE_AA)

            # 마커 배열 메시지 생성
            marker_array_msg = MarkerArray()

            # 가장 가까운 마커만 퍼블리시
            for marker in detect_data:
                if int(closest_marker[0]) != int(marker[0]):
                    continue

                marker_msg = Marker()
                marker_msg.id = int(marker[0])
                marker_msg.pose.pose.position.x = marker[1][0]
                marker_msg.pose.pose.position.y = marker[1][1]
                marker_msg.pose.pose.position.z = marker[1][2]
                marker_msg.pose.pose.orientation.x = marker[2][2]
                marker_msg.pose.pose.orientation.y = marker[2][1]
                marker_msg.pose.pose.orientation.z = marker[2][0]
                marker_array_msg.markers.append(marker_msg)

            # 마커 퍼블리시
            self.marker_publisher.publish(marker_array_msg)
          
#######################################################################
# remove cv2...
#######################################################################      
        # 탐지 결과 영상 표시
        cv2.imshow('Detected Markers', frame)
        cv2.resizeWindow("Detected Markers", 320, 240)  # 창 크기 조정 (가로x세로)
        cv2.waitKey(1)


def main(args=None):
    """
    메인 함수: ROS2 노드 실행
    """
    rclpy.init(args=args)
    aruco_marker_detector = ArucoMarkerDetector()
    
    rclpy.spin(aruco_marker_detector)
    aruco_marker_detector.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    # 실행 시 마커 크기 인자 설정 가능 (--marker_size 0.05 등)
    parser = argparse.ArgumentParser(description='Detect ArUco markers.')
    parser.add_argument('--marker_size', type=float, default=0.04, help='Size of the ArUco markers in meters.')
    args = parser.parse_args()
    ArucoMarkerDetector.marker_size = args.marker_size
    main()