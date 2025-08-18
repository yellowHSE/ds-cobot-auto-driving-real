#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from aruco_msgs.msg import MarkerArray, Marker
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Pose, PoseArray
from turtlebot_cosmo_interface.srv import MoveitControl
from srv_call_test import TurtlebotArmClient
import time
import ast


# ANSI 색상 코드 정의
RED = "\033[91m"
GREEN = "\033[92m"
YELLOW = "\033[93m"
BLUE = "\033[94m"
MAGENTA = "\033[95m"
CYAN = "\033[96m"
RESET = "\033[0m"  # 색상 초기화


class IntegratedProcess(Node):
    def __init__(self):
        super().__init__('integrated_process')

        # Aruco Marker Listener 설정
        self.aruco_sub = self.create_subscription(
            MarkerArray,
            'detected_markers',
            self.aruco_listener_callback,
            10)
        
        # Yolo Detection Listener 설정
        # self.yolo_sub = self.create_subscription(
        #     String,
        #     '/yolo/detected_info',
        #     self.yolo_listener_callback,
        #     10)
        
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 2)
        
        # 상태 변수
        self.aruco_marker_found = False
        self.task_completed = False
        self.yolofind = True
        self.armrun = False
        self.yolo_x = 0
        self.yolo_y = 0
        self.marker_id = None
        self.state = 'START'  

        self.count = 0
        self.aruco_pose = None  # Aruco marker의 pose 정보를 저장할 변수

        self.create_timer(1.0, self.run_tasks)
        
        self.twist = Twist()


    def aruco_listener_callback(self, msg):
        # if self.state not in ('ARUCO', 'BACKWARD', 'CHECK'):
        if self.state not in ('ARUCO'):
            return
        
        target_marker_id = 0  # 초기에는 0번 마커를 찾음
        
        
        for marker in msg.markers:
            if marker.id == target_marker_id:
                self.marker_id = marker.id
                self.aruco_pose = marker.pose.pose  # Aruco의 위치 저장
                self.get_logger().info(f'Marker ID: {marker.id}, PositionZ: {self.aruco_pose.position.x}')
                self.aruco_marker_found = True
                if self.state ==  'ARUCO':
                    self.execute_forward_task(self.aruco_pose.position.x)  # 전진 작업 실행
                    # if self.state not in ('YOLO'):
                    #     return

                    # self.get_logger().info('check self.armrun')

                    # if not self.armrun:  # 로봇 암이 동작 중이 아니면
                    #     data = msg.data
                    #     try:
                    #         data_list = ast.literal_eval(data)
                    #         if len(data_list) > 0:
                    #             self.yolo_x = data_list[0][1]
                    #             self.yolo_y = data_list[0][2]
                                
                    #             print(f"Detected coordinates: {self.yolo_x}, {self.yolo_y}")
                    #             print("done")

                    #             if self.state == 'YOLO':
                    #                 if not self.yolofind:
                    #                     self.yolofind = True
                    #                     self.yolo_arm_controll()
                    #     except Exception as e:
                    #         self.get_logger().error(f"Error processing the data: {e}")

    def execute_aruco_task(self):
        self.state =  'ARUCO'
        

    def execute_forward_task(self, current_x_position):
        # 전진 작업: 30cm까지 전진 후 멈추고, 작업을 진행
        if self.aruco_marker_found and self.aruco_pose:
            self.get_logger().info("Executing forward task...")
            # 목표 z축 위치를 30cm로 설정
            if current_x_position > 0.04:
                self.publish_cmd_vel(0.05)
            # elif current_y_position > 0.25:
            #     self.publish_cmd_vel(0.025)
            else:
                self.publish_cmd_vel(0.0)
                self.get_logger().info("Target reached")
                # self.camera_arm_controll()
                self.yolo_arm_controll()
                # self.state = 'YOLO'
                
    def camera_arm_controll(self):
        arm_client = TurtlebotArmClient()
        response = arm_client.send_request(1, "conveyor_up")
        arm_client.get_logger().info(f'Response: {response.response}')
        time.sleep(3)        

    def home2_arm_controll(self):
        arm_client = TurtlebotArmClient()
        response = arm_client.send_request(1, "home2")
        arm_client.get_logger().info(f'Response: {response.response}')
        time.sleep(3)      

    def box_home_arm_controll(self):
        arm_client = TurtlebotArmClient()
        response = arm_client.send_request(1, "conveyor_up")
        arm_client.get_logger().info(f'Response: {response.response}')
        time.sleep(3)            
    
    def append_pose_init(self, x,y,z):
        pose_array = PoseArray()
        pose = Pose()

        pose.position.x = x
        pose.position.y =  y
        pose.position.z =  z

        pose_array.poses.append(pose)
        
        self.get_logger().info(f"{CYAN}Pose initialized - x: {x}, y: {y}, z: {z}{RESET}")

        return pose_array

    def yolo_arm_controll(self):
        arm_client = TurtlebotArmClient()

        # print ("yolo arm control start!")
        response = arm_client.send_request(2, "open")
        arm_client.get_logger().info(f'Response: {response.response}')
        time.sleep(1)

        response = arm_client.send_request(1, "03_move_to_box")
        arm_client.get_logger().info(f'Response: {response.response}')
        time.sleep(1)

        response = arm_client.send_request(2, "close")
        arm_client.get_logger().info(f'Response: {response.response}')
        time.sleep(1)

        response = arm_client.send_request(1, "camera_start")
        arm_client.get_logger().info(f'Response: {response.response}')
        time.sleep(1)



        # pose_array = self.append_pose_init(self.aruco_pose.position.x, self.aruco_pose.position.y, self.aruco_pose.position.z)
        # response = arm_client.send_request(0, "", pose_array)
        # arm_client.get_logger().info(f'Response: {response.response}')
        # arm_client.get_logger().info('')


        # pose_array = self.append_pose_init(self.aruco_pose.position.x, self.aruco_pose.position.y, self.aruco_pose.position.z)

        # response = arm_client.send_request(0, "", pose_array)
        # arm_client.get_logger().info(f'Response: {response.response}')     

     

        # response = arm_client.send_request(1, "home2")
        # arm_client.get_logger().info(f'Response: {response.response}')
        # time.sleep(1)

        # print ("conveyor task start")

        # response = arm_client.send_request(1, "conveyor_up")
        # arm_client.get_logger().info(f'Response: {response.response}')

        # response = arm_client.send_request(1, "test_conveyor")
        # arm_client.get_logger().info(f'Response: {response.response}')

        # response = arm_client.send_request(2, "open")
        # arm_client.get_logger().info(f'Response: {response.response}')

        # print("throw ")
        
        # response = arm_client.send_request(1, "conveyor_up")
        # arm_client.get_logger().info(f'Response: {response.response}')

        # response = arm_client.send_request(1, "camera_home")
        # arm_client.get_logger().info(f'Response: {response.response}')    

        time.sleep(3)

        print("jobs_done")

        self.armrun = False
        self.yolofind = False  # 작업 완료 후 초기화
        
        self.count += 1
        
    def final_task(self):
        arm_client = TurtlebotArmClient()
        response = arm_client.send_request(1, "box_back_put")
        arm_client.get_logger().info(f'Response: {response.response}') 
        time.sleep(1)
        response = arm_client.send_request(2, "open")
        arm_client.get_logger().info(f'Response: {response.response}')       
        self.state = "FINISH"

    def finish_task(self):
        # 모든 작업이 완료되었을 때
        if self.state == 'FINISH':
            self.get_logger().info("All tasks are complete!")
            self.destroy_node()
            rclpy.shutdown()

    def publish_cmd_vel(self, linear_x, angular_z=0.0):
        self.twist.linear.x = linear_x
        self.twist.angular.z = angular_z
        self.cmd_vel_publisher.publish(self.twist)

    def run_tasks(self):
        # 상태에 따라 각 작업을 순차적으로 실행
        if self.state == 'START':
            self.execute_aruco_task()
        elif self.state == 'FINISH':
            self.finish_task()

def main(args=None):
    rclpy.init(args=args)
    node = IntegratedProcess()
    
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
