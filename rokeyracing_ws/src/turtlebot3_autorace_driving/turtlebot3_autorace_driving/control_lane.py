#!/usr/bin/env python
# -*- coding: utf-8 -*-

################################################################################
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
################################################################################

# Author: Leon Jung, Gilbert, Ashe Kim
 


from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import Float64



class ControlLane():
    def __init__(self):
        self.sub_lane = rospy.Subscriber('/control/lane', Float64, self.cbFollowLane, queue_size = 1)
        self.sub_max_vel = rospy.Subscriber('/control/max_vel', Float64, self.cbGetMaxVel, queue_size = 1)
        self.pub_cmd_vel = rospy.Publisher('/control/cmd_vel', Twist, queue_size = 1)

        self.lastError = 0
        self.MAX_VEL = 0.1

        rospy.on_shutdown(self.fnShutDown)

    def cbGetMaxVel(self, max_vel_msg):
        self.MAX_VEL = max_vel_msg.data

    def cbFollowLane(self, desired_center):
        center = desired_center.data

        error = center - 500

        Kp = 0.0025
        Kd = 0.007

        angular_z = Kp * error + Kd * (error - self.lastError)
        self.lastError = error
        
        twist = Twist()
        # twist.linear.x = 0.05  
        # error	목표 위치/거리와 현재 위치 간의 오차
        # abs(error)	오차의 절댓값. 양수로 정규화
        # abs(error) / 500	오차를 0~1 사이로 정규화 (500은 기준 거리)
        # 1 - ...	오차가 작을수록 값이 커짐 (즉, 속도가 커짐)
        # ** 2.2	감속 곡선의 곡률. 곡률이 클수록 빠르게 감속
        # self.MAX_VEL * ...	최대 속도 (MAX_VEL)를 곱해서 비례 속도 결정  
        # min(..., 0.05)	속도가 너무 커지는 것을 막기 위한 최댓값 제한

        twist.linear.x = min(self.MAX_VEL * ((1 - abs(error) / 500) ** 2.2), 0.05)
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        """
        회전 속도 제한: angular_z의 범위를 -2.0 ~ +2.0으로 제한하여 급격한 회전을 방지
        부호 반전: -angular_z 형태로 오차 방향에 반대되는 방향으로 회전 (PID 컨트롤에서 사용)
        """
        #twist.angular.z = -max(angular_z, -2.0) if angular_z < 0 else -min(angular_z, 2.0)
        
        if angular_z < 0:
            twist.angular.z = -max(angular_z, -2.0)
        else:
            twist.angular.z = -min(angular_z, 2.0)



        self.pub_cmd_vel.publish(twist)

    def fnShutDown(self):
        rospy.loginfo("Shutting down. cmd_vel will be 0")

        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.pub_cmd_vel.publish(twist) 

    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('control_lane')
    node = ControlLane()
    node.main()
