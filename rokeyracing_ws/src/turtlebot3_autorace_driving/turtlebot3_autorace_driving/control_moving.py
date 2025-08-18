#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
from enum import Enum
from std_msgs.msg import UInt8, Float64
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from turtlebot3_autorace_msgs.msg import MovingParam
from tf_transformations import euler_from_quaternion


class ControlMoving(Node):
    def __init__(self):
        super().__init__('control_moving')

        # Enums
        self.TypeOfMoving = Enum('TypeOfMoving', 'idle left right forward backward')
        self.TypeOfState = Enum('TypeOfState', 'idle start stop finish')

        # Variables
        self.moving_type = UInt8()
        self.moving_angluar = 0.0
        self.moving_linear = 0.0
        self.moving_msg = MovingParam()

        self.current_pos_x = 0.0
        self.current_pos_y = 0.0
        self.current_theta = 0.0
        self.last_current_theta = 0.0
        self.lastError = 0.0

        self.is_step_start = False
        self.is_step_left = False
        self.is_step_right = False
        self.is_step_forward = False
        self.is_step_backward = False

        # Subscribers
        self.create_subscription(MovingParam, '/control/moving/state', self.get_param, 10)
        self.create_subscription(Odometry, '/odom', self.cbOdom, 10)

        # Publishers
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_max_vel = self.create_publisher(Float64, '/control/max_vel', 10)
        self.pub_moving_complete = self.create_publisher(UInt8, '/control/moving/complete', 10)

        # Set initial velocity
        msg_pub_max_vel = Float64()
        msg_pub_max_vel.data = 0.05
        self.pub_max_vel.publish(msg_pub_max_vel)

        # Timer for loop
        self.timer = self.create_timer(0.01, self.loop_callback)

    def loop_callback(self):
        if self.is_step_left:
            self.turn_left(self.moving_msg)
        elif self.is_step_right:
            self.turn_right(self.moving_msg)
        elif self.is_step_forward:
            self.go_forward(self.moving_msg)
        elif self.is_step_backward:
            self.go_backward(self.moving_msg)

    def get_param(self, msg):
        self.moving_type = msg.moving_type
        self.moving_angluar = msg.moving_value_angular
        self.moving_linear = msg.moving_value_linear

        self.get_logger().info(f"Get param: type {self.moving_type}, angular {self.moving_angluar}, linear {self.moving_linear}")

        if self.is_step_start:
            self.get_logger().info("Already running a step")
            return

        self.moving_msg = msg
        if msg.moving_type == self.TypeOfMoving.left.value:
            self.is_step_left = True
        elif msg.moving_type == self.TypeOfMoving.right.value:
            self.is_step_right = True
        elif msg.moving_type == self.TypeOfMoving.forward.value:
            self.is_step_forward = True
        elif msg.moving_type == self.TypeOfMoving.backward.value:
            self.is_step_backward = True

    def step_completed(self):
        self.is_step_start = False
        self.is_step_left = False
        self.is_step_right = False
        self.is_step_forward = False
        self.is_step_backward = False

        self.lastError = 0.0

        self.fnStop()
        msg = UInt8()
        msg.data = self.TypeOfState.finish.value
        self.pub_moving_complete.publish(msg)

    def turn_left(self, msg):
        if not self.is_step_start:
            self.lastError = 0.0
            self.desired_theta = self.current_theta + math.radians(msg.moving_value_angular)
            self.is_step_start = True
        error = self.fnTurn()
        if abs(error) < 0.05:
            self.step_completed()

    def turn_right(self, msg):
        if not self.is_step_start:
            self.lastError = 0.0
            self.desired_theta = self.current_theta - math.radians(msg.moving_value_angular)
            self.is_step_start = True
        error = self.fnTurn()
        if abs(error) < 0.05:
            self.step_completed()

    def go_forward(self, msg):
        if not self.is_step_start:
            self.lastError = 0.0
            self.start_pos_x = self.current_pos_x
            self.start_pos_y = self.current_pos_y
            self.is_step_start = True
        error = self.fnStraight(msg.moving_value_linear)
        if abs(error) < 0.005:
            self.step_completed()

    def go_backward(self, msg):
        if not self.is_step_start:
            self.lastError = 0.0
            self.start_pos_x = self.current_pos_x
            self.start_pos_y = self.current_pos_y
            self.is_step_start = True
        error = self.fnBackStraight(msg.moving_value_linear)
        if abs(error) < 0.005:
            self.step_completed()

    def fnTurn(self):
        error = self.current_theta - self.desired_theta
        Kp, Kd = 0.45, 0.03
        angular_z = Kp * error + Kd * (error - self.lastError)
        self.lastError = error

        twist = Twist()
        twist.angular.z = -angular_z * 2
        self.pub_cmd_vel.publish(twist)
        return error

    def fnStraight(self, desired_dist):
        error = math.sqrt((self.current_pos_x - self.start_pos_x) ** 2 + (self.current_pos_y - self.start_pos_y) ** 2) - desired_dist
        self.lastError = error

        twist = Twist()
        twist.linear.x = 0.1 if error < 0 else -0.1
        self.pub_cmd_vel.publish(twist)
        return error

    def fnBackStraight(self, desired_dist):
        error = math.sqrt((self.current_pos_x - self.start_pos_x) ** 2 + (self.current_pos_y - self.start_pos_y) ** 2) - desired_dist
        self.lastError = error

        twist = Twist()
        twist.linear.x = -0.1 if error < 0 else 0.1
        self.pub_cmd_vel.publish(twist)
        return error

    def fnStop(self):
        twist = Twist()
        self.pub_cmd_vel.publish(twist)
        """normalize_angle()은 항상 -π ~ π 범위로 만들어줍니다.
        이후 desired_theta - current_theta를 구하면 정확한 회전 오차가 나옵니다.
        fnTurn() 함수에서 각도 오차를 바로 계산해도 됩니다.
        """
    def normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def cbOdom(self, msg):
        q = msg.pose.pose.orientation
        quaternion = (q.x, q.y, q.z, q.w)

        raw_theta = euler_from_quaternion(quaternion)[2]
        self.current_theta = normalize_angle(raw_theta)

        self.current_pos_x = msg.pose.pose.position.x
        self.current_pos_y = msg.pose.pose.position.y


    def cbOdom_old(self, msg):
        q = msg.pose.pose.orientation
        quaternion = (q.x, q.y, q.z, q.w)
        theta = euler_from_quaternion(quaternion)[2]

        if (theta - self.last_current_theta) < -math.pi:
            theta = 2.0 * math.pi + theta
            self.last_current_theta = math.pi
        elif (theta - self.last_current_theta) > math.pi:
            theta = -2.0 * math.pi + theta
            self.last_current_theta = -math.pi
        else:
            self.last_current_theta = theta

        self.current_theta = theta
        self.current_pos_x = msg.pose.pose.position.x
        self.current_pos_y = msg.pose.pose.position.y


def main(args=None):
    rclpy.init(args=args)
    node = ControlMoving()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.fnStop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
