#!/usr/bin/env python3
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
# Author: Leon Jung, Gilbert, Ashe Kim, Hyungyu Kim, ChanHyeong Lee

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import Float64
from std_msgs.msg import String
from aruco_yolo.aruco_pick_and_place import ArucoMarkerListener


class ControlLane(Node):

    def __init__(self):
        super().__init__('control_lane')

        self.sub_traffic_light_state = self.create_subscription(
            String,
            '/detect/traffic_light_state',
            self.callback_traffic_light_state,
            10
        )
        self.traffic_light_state = 'green'
        # self.started = False

        self.sub_lane = self.create_subscription(
            Float64,
            '/control/lane',
            self.callback_follow_lane,
            1
        )
        self.sub_max_vel = self.create_subscription(
            Float64,
            '/control/max_vel',
            self.callback_get_max_vel,
            1
        )
        self.sub_avoid_cmd = self.create_subscription(
            Twist,
            '/avoid_control',
            self.callback_avoid_cmd,
            1
        )
        self.sub_avoid_active = self.create_subscription(
            Bool,
            '/avoid_active',
            self.callback_avoid_active,
            1
        )

        self.sub_stop_bar = self.create_subscription(
            String,
            '/detect/bar_state',
            self.callback_stop_bar,
            1
        )

        self.pub_cmd_vel = self.create_publisher(
            Twist,
            '/control/cmd_vel',
            1
        )

        self.pub_cmd_vel = self.create_publisher(
            Twist,
            '/control/cmd_vel',
            1
        )

        # PD control related variables
        self.last_error = 0
        self.MAX_VEL = 0.1

        # Avoidance mode related variables
        self.avoid_active = False
        self.avoid_twist = Twist()
        self.stop_bar = 'go'

        self.aruco = ArucoMarkerListener()

    def callback_get_max_vel(self, max_vel_msg):
        self.MAX_VEL = max_vel_msg.data

    def callback_traffic_light_state(self, msg: String):
        self.traffic_light_state = msg.data
        self.get_logger().info(f'Traffic light state: {self.traffic_light_state}')

        if self.traffic_light_state == 'red':
            # 빨간불이면 바로 정지 명령 발행
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.pub_cmd_vel.publish(twist)
        elif self.traffic_light_state == 'green':
            # 초록불이면 별도 동작 없이 주행 명령 허용
            self.get_logger().info('Green light detected — starting lane following.')
            pass
        elif self.traffic_light_state == 'yellow':
            # 노란불 처리 필요하면 여기 작성
            pass

    def callback_follow_lane(self, desired_center):
        """
        Receive lane center data to generate lane following control commands.

        If avoidance mode is enabled, lane following control is ignored.
        
        """
        if self.avoid_active:
            return

        if self.stop_bar == 'slowdown':
            slowdown = Twist()
            slowdown.linear.x = min(self.MAX_VEL * 0.3, 0.05)
            self.pub_cmd_vel.publish(slowdown)
            self.get_logger().info(f'slow_down: {slowdown.linear.x}')
        elif self.stop_bar == 'stop' or self.traffic_light_state == 'red':
            stop = Twist()
            stop.linear.x = 0.0
            self.pub_cmd_vel.publish(stop)
            self.get_logger().info(f'self.stop_bar: {self.stop_bar}')
            self.get_logger().info(f'self.traffic_light_state: {self.traffic_light_state}')
            self.get_logger().info(f'stop: {stop.linear.x}')
        elif len(self.aruco.marker) > 0:
            self.aruco.aruco_move_pick(self.aruco)
        else:
            center = desired_center.data
            error = center - 460

            Kp = 0.0025
            Kd = 0.007

            angular_z = Kp * error + Kd * (error - self.last_error)
            self.last_error = error

            twist = Twist()
            # Linear velocity: adjust speed based on error (maximum 0.05 limit)
            # twist.linear.x = min(self.MAX_VEL * (max(1 - abs(error) / 500, 0) ** 2.2), 0.05)--------------------------------------------
            twist.linear.x = min(self.MAX_VEL * (max(1 - abs(error) / 460, 0) ** 1.7), 0.05)
            twist.angular.z = -max(angular_z, -2.0) if angular_z < 0 else -min(angular_z, 2.0)
            self.pub_cmd_vel.publish(twist)
            self.get_logger().info(f'basic: {twist.linear.x}')

    def callback_avoid_cmd(self, twist_msg):
        self.avoid_twist = twist_msg

        if self.avoid_active:
            self.pub_cmd_vel.publish(self.avoid_twist)

    def callback_avoid_active(self, bool_msg):
        self.avoid_active = bool_msg.data
        if self.avoid_active:
            self.get_logger().info('Avoidance mode activated.')
        else:
            self.get_logger().info('Avoidance mode deactivated. Returning to lane following.')

    def callback_stop_bar(self, msg: String):
        self.stop_bar = msg.data




    def shut_down(self):
        self.get_logger().info('Shutting down. cmd_vel will be 0')
        twist = Twist()
        self.pub_cmd_vel.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = ControlLane()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shut_down()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()