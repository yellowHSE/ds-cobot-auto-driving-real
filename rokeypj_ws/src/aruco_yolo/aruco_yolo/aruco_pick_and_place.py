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
from aruco_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import getkey
from std_msgs.msg import Header, Float32, Bool
from sensor_msgs.msg import JointState

from geometry_msgs.msg import Twist, Pose, PoseArray
from turtlebot_cosmo_interface.srv import MoveitControl
from aruco_yolo.moveit_client import TurtlebotArmClient
import time
import ast


# ANSI 색상 코드 정의 (터미널 출력 색상 지정용)
RED = "\033[91m"
GREEN = "\033[92m"
YELLOW = "\033[93m"
BLUE = "\033[94m"
MAGENTA = "\033[95m"
CYAN = "\033[96m"
RESET = "\033[0m"  # 색상 초기화


class ArucoMarkerListener(Node):
    def __init__(self):
        super().__init__('aruco_marker_listener')

        # pick_n_place.launch.py에서 설정하는 markerid 파라미터 불러오기 (기본값 1)
        self.markerid = self.declare_parameter('markerid', 1).get_parameter_value().integer_value
        self.target_marker_id = self.markerid 

        # ArUco 마커 검출 결과 구독
        self.subscription_markers = self.create_subscription(
            MarkerArray,
            '/camera/detected_markers',
            self.aruco_listener_callback,
            10)
        
        # ArUco 마커까지의 거리 정보 구독
        self.subscription_distance = self.create_subscription(
            Float32,
            '/aruco/distance',
            self.aruco_distance_callback,
            10)
        
        # 로봇 속도 명령 퍼블리셔 (/cmd_vel)
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            1
        )

        # 이동 제어용 Twist 메시지
        self.twist = Twist()
        self.finish_move = False  # 이동 완료 플래그

        # pick & place 완료 여부 퍼블리셔
        self.finish_publisher = self.create_publisher(
            Bool,
            '/pick_and_place_finish',
            1
        )

        # 로봇 관절 상태 구독 (/joint_states)
        self.subscription_joint = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10
        )

        self.get_joint = False  # 관절 상태 읽기 여부
        self.marker = []

        # 로봇 팔 관절 제어 퍼블리셔
        self.joint_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.trajectory_msg = JointTrajectory()
        self.trajectory_msg.header = Header()
        self.trajectory_msg.header.frame_id = ''
        self.trajectory_msg.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
    
        # 관절 trajectory 초기화
        self.point = JointTrajectoryPoint()
        self.point.velocities = [0.0] * 4
        self.point.accelerations = [0.0] * 4
        self.point.time_from_start.sec = 0
        self.point.time_from_start.nanosec = 500

        # 상태 변수
        self.aruco_marker_found = False  # 마커 발견 여부
        self.task_completed = False      # 작업 완료 여부
        self.armrun = False              # 팔 동작 여부
        self.aruco_location_x = 0
        self.aruco_location_y = 0
        self.aruco_location_z = 0       

        self.marker_id = None
        self.state = 'START'  # FSM 상태

        self.count = 0
        self.aruco_pose = None  # 마커 pose 저장 변수
        self.distance = 0.8     # 초기 거리값


    def joint_states_callback(self, msg):
        """ 로봇 관절 상태 확인 콜백 """
        if self.get_joint == False:
            return
        for i, name in enumerate(msg.name):
            position = msg.position[i] if i < len(msg.position) else None
            if 'joint1' in name:
                print(f'joint1 : {position}')
        for i, name in enumerate(msg.name):
            position = msg.position[i] if i < len(msg.position) else None
            if 'joint2' in name:
                print(f'joint2 : {position}')
        for i, name in enumerate(msg.name):
            position = msg.position[i] if i < len(msg.position) else None
            if 'joint3' in name:
                print(f'joint3 : {position}')
        for i, name in enumerate(msg.name):
            position = msg.position[i] if i < len(msg.position) else None
            if 'joint4' in name:
                print(f'joint4 : {position}')

    def aruco_distance_callback(self, msg):
        """ 거리 토픽 업데이트 콜백 """
        self.distance = msg.data
        

    def aruco_listener_callback(self, msg):
        """ ArUco 마커 감지 시 동작 콜백 """
        self.aruco_marker_found = True
        for marker in msg.markers:
            if marker.id == self.target_marker_id:
                # 마커 정보 출력
                self.get_logger().info(f'Marker Info: {marker}')
                self.get_logger().debug(f'Marker ID: {marker.id}, PositionZ: {marker.pose.pose.position.z}')
                self.get_logger().info(f'Position: x:[{marker.pose.pose.position.x}, y:[{marker.pose.pose.position.y},z:[{marker.pose.pose.position.z}]] ')
                self.get_logger().info(f'Orientation: x:[{marker.pose.pose.orientation.x}, y:[{marker.pose.pose.orientation.y},z:[{marker.pose.pose.orientation.z}]] ')

                # 마커 좌표 저장
                self.aruco_position_x = marker.pose.pose.position.x
                self.aruco_position_y = marker.pose.pose.position.y               
                self.aruco_position_z = marker.pose.pose.position.z

                # 마커와의 거리별 속도 제어
                if self.distance > 0.6:
                    self.get_logger().info(f'publish_cmd_vel(0.05)')
                    self.publish_cmd_vel(0.05)
                elif self.distance > 0.57:
                    self.get_logger().info(f'publish_cmd_vel(0.03)')                    
                    self.publish_cmd_vel(0.03)
                elif self.distance > 0.52 :
                    self.get_logger().info(f'publish_cmd_vel(0.01)')                    
                    self.publish_cmd_vel(0.01)
                else:
                    # 충분히 가까우면 정지 후 팔 동작 실행
                    self.publish_cmd_vel(0.0)
                    self.get_logger().info(f'finish_move = True')                    
                    self.aruco_arm_controll()
                break

    def publish_cmd_vel(self, linear_x):
        """ 속도 명령 퍼블리시 함수 """
        self.twist.linear.x = linear_x
        self.twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(self.twist)  


##################################################################################            
##################################################################################
    def append_pose_init(self, x,y,z):
        """ Pose 초기화 후 PoseArray 반환 """
        pose_array = PoseArray()
        pose = Pose()

        pose.position.x = x
        pose.position.y =  y
        pose.position.z =  z
        pose_array.poses.append(pose)
        
        self.get_logger().info(f"{CYAN}Pose initialized - x: {x}, y: {y}, z: {z}{RESET}")

        return pose_array

    def aruco_arm_controll(self):
        """ ArUco 기반 pick & place 동작 시퀀스 """
        print("Impossible Mission Start")        
        arm_client = TurtlebotArmClient()

        print(f"Mission Aruco marker Locaion coordinates: {self.aruco_location_x}, {self.aruco_location_y}, {self.aruco_location_z}")

        self.aruco_marker_found = True

        if self.aruco_marker_found:
            self.armrun = True

            # 좌표 수동 설정 (실제 환경에 맞게 고정값 사용)
            self.aruco_location_x = -0.92
            self.aruco_location_y = -0.45
            self.aruco_location_z = 0.33

            print(f"Remove Rock Initial Position")
            time.sleep(2)            
            self.point.positions = [0.17, -0.058, -0.258, 1.94]
            print("point",self.point.positions)
            self.joint_pub.publish(self.trajectory_msg)

            # 일련의 팔 동작 시퀀스 (서비스 요청 방식)
            print("Move Aruco Cube(Rock) Mission")
            response = arm_client.send_request(1, "01_home") # 초기 위치 이동
            arm_client.get_logger().info(f'Response: {response.response}')

            print("Gripper Open")
            response = arm_client.send_request(2, "open")
            arm_client.get_logger().info(f'Response: {response.response}')

            print("Cube Box Front Start...")       
            time.sleep(1)
            response = arm_client.send_request(1, "02_box_front")
            arm_client.get_logger().info(f'Response: {response.response}')

            print("Move to Box Start...")       
            time.sleep(0.1)                 
            response = arm_client.send_request(1, "03_move_to_box")
            arm_client.get_logger().info(f'Response: {response.response}')

            time.sleep(0.1)
            print("Gripper Close")
            response = arm_client.send_request(2, "pick")
            arm_client.get_logger().info(f'Response: {response.response}')

            print("Move up Start...")       
            time.sleep(0.1)                 
            response = arm_client.send_request(1, "04_move_up")
            arm_client.get_logger().info(f'Response: {response.response}')

            print("Conveyor up Start...")       
            time.sleep(0.1)  
            response = arm_client.send_request(1, "05_conveyor_up")
            arm_client.get_logger().info(f'Response: {response.response}')

            print("Conveyor down Start...")       
            time.sleep(0.1)  
            response = arm_client.send_request(1, "06_conveyor_down")
            arm_client.get_logger().info(f'Response: {response.response}')

            time.sleep(0.1)
            print("Gripper Open")
            response = arm_client.send_request(2, "open")
            arm_client.get_logger().info(f'Response: {response.response}')

            print("Conveyor up Start...")       
            time.sleep(0.1)  
            response = arm_client.send_request(1, "07_conveyor_up")
            arm_client.get_logger().info(f'Response: {response.response}')

            response = arm_client.send_request(1, "camera_start")
            arm_client.get_logger().info(f'Response: {response.response}')
            time.sleep(1) 

            self.finish_move = True
            print("Pick and place Mission Clear")
            self.armrun = False

    def aruco_move_pick(self):
        """ 팔 초기 위치 세팅 및 pick 수행 """
        joint_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        trajectory_msg = JointTrajectory()
        trajectory_msg.header = Header()
        trajectory_msg.header.frame_id = ''
        trajectory_msg.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']

        point = JointTrajectoryPoint()
        point.velocities = [0.0] * 4
        point.accelerations = [0.0] * 4
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 500

        # 초기 관절 위치 (실제 측정값 기반)
        point.positions = [0.0, 0.5317, -0.6918, 0.7701] 

        trajectory_msg.points = [point]
        joint_pub.publish(trajectory_msg)
        print("Pick and Place Init done")

        while True:
            rclpy.spin_once(self)
            if(self.finish_move == True):
                self.finish_move = False
                break

        self.destroy_node()
        # rclpy.shutdown()
    

def main(args=None):
    """ 노드 실행부 """
    rclpy.init(args=args)
    node = ArucoMarkerListener()

    joint_pub = node.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
    trajectory_msg = JointTrajectory()
    trajectory_msg.header = Header()
    trajectory_msg.header.frame_id = ''
    trajectory_msg.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']

    point = JointTrajectoryPoint()
    point.velocities = [0.0] * 4
    point.accelerations = [0.0] * 4
    point.time_from_start.sec = 0
    point.time_from_start.nanosec = 500
    print("Pick and Place Init done")

    while True:
        rclpy.spin_once(node)
        if(node.finish_move == True):
            node.finish_move = False
            break

    print("Pick and Place done")
    node.finish_publisher.publish(Bool(data=node.finish_move))
    # node.destroy_node()
    # rclpy.shutdown()

if __name__ == '__main__':
    main()