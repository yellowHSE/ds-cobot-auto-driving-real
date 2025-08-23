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
# -----------------------------------------------------------------------------
# 차선 추종(ControlLane) 제어를 수행하는 ROS2 노드
# - 외부 감지 결과(신호등, 차단바, 장애물 회피, 아루코 마커)를 구독하여 주행 속도/조향을 결정
# - 핵심 입력 토픽:
#     /control/lane (Float64): 차선 중앙 추종을 위한 목표 x 좌표(픽셀)
#     /control/max_vel (Float64): 최대 선형속도 상한
#     /avoid_control (geometry_msgs/Twist): 회피 모드에서 사용할 속도 명령
#     /avoid_active (Bool): 회피 모드 활성화 플래그
#     /detect/traffic_light_state (String): 신호등 상태(red/green/yellow)
#     /detect/bar_state (String): 차단바 상태(go/slowdown/stop)
#     /camera/detected_markers (aruco_msgs/MarkerArray): 아루코 마커 감지 결과
#     /aruco/distance (Float32): 아루코 마커까지의 거리(m)
#     /pick_and_place_finish (Bool): 픽앤플레이스 완료/상태 알림(여기서는 아루코 관련 플래그 재사용)
# - 출력 토픽:
#     /cmd_vel (geometry_msgs/Twist): 최종 속도/조향 명령
# - 제어 로직 우선순위(상황 별 정지/저속/차선추종):
#     1) 회피 모드가 활성화되면 회피 명령을 우선 사용
#     2) 차단바 상태가 stop/slowdown이면 그에 맞게 속도를 제한
#     3) 신호등이 red이면 정지
#     4) 아루코가 근거리면 정지 또는 저속
#     5) 그 외에는 차선 중앙 오차 기반 PD 제어 수행
# -----------------------------------------------------------------------------

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64, Float32, String
from aruco_msgs.msg import MarkerArray


class ControlLane(Node):

    def __init__(self):
        super().__init__('control_lane')

        # 신호등 상태 구독: red/green/yellow 문자열로 가정
        self.sub_traffic_light_state = self.create_subscription(
            String,
            '/detect/traffic_light_state',
            self.callback_traffic_light_state,
            10
        )
        self.traffic_light_state = 'green'
        # self.started = False

        # 차선 중앙 좌표(픽셀) 구독: /control/lane
        self.sub_lane = self.create_subscription(
            Float64,
            '/control/lane',
            self.callback_follow_lane,
            1
        )
        # 최대 속도 상한 구독: /control/max_vel
        self.sub_max_vel = self.create_subscription(
            Float64,
            '/control/max_vel',
            self.callback_get_max_vel,
            1
        )
        # 장애물 회피 명령(속도) 구독: /avoid_control
        self.sub_avoid_cmd = self.create_subscription(
            Twist,
            '/avoid_control',
            self.callback_avoid_cmd,
            1
        )
        # 회피 모드 활성화 플래그 구독: /avoid_active
        self.sub_avoid_active = self.create_subscription(
            Bool,
            '/avoid_active',
            self.callback_avoid_active,
            1
        )

        # 차단바 상태 구독: /detect/bar_state ('go'|'slowdown'|'stop')
        self.sub_stop_bar = self.create_subscription(
            String,
            '/detect/bar_state',
            self.callback_stop_bar,
            1
        )

        # 아루코 마커 배열 구독: 특정 id 감지 여부 확인용
        self.sub_markers = self.create_subscription(
            MarkerArray,
            '/camera/detected_markers', 
            self.cb_markers, 
            10
        )

        # 아루코 마커까지의 거리(m) 구독
        self.sub_aruco_dist = self.create_subscription(
            Float32, 
            '/aruco/distance', 
            self.cb_aruco_distance, 
            10
        )

        # 픽앤플레이스 완료 플래그 구독(여기서는 아루코 관련 상태로 재사용)
        self.aruco_seen_publisher = self.create_subscription(
            Bool,
            '/pick_and_place_finish',
            self.cb_aruco_finish,
            1
        )

        # 최종 주행 명령 퍼블리셔
        self.pub_cmd_vel = self.create_publisher(
            Twist,
            '/cmd_vel',
            1
        )

        # PD control 관련 변수(차선 중앙 오차 제어용)
        self.last_error = 0
        self.MAX_VEL = 0.1  # 기본 최대 속도 상한

        # 회피 모드 상태 변수
        self.avoid_active = False
        self.avoid_twist = Twist()
        self.stop_bar = 'go'  # 차단바 기본 상태

        # 아루코 관련 상태 변수
        self.aruco_seen = False
        self.aruco_distance = None
        self.aruco_target_id = self.declare_parameter('aruco_target_id', 1).value
        self.ARUCO_STOP_DIST = self.declare_parameter('aruco_stop_distance', 0.48).value

    def callback_get_max_vel(self, max_vel_msg):
        # 외부에서 전달되는 최대 속도 상한 반영
        self.MAX_VEL = max_vel_msg.data

    def callback_traffic_light_state(self, msg: String):
        # 신호등 상태 갱신 및 즉시 제동 처리
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
            # 노란불 처리 필요하면 여기 작성(감속 등 정책 적용 가능)
            pass
    
    # ---------------- ArUco 콜백 ----------------
    def cb_markers(self, msg: MarkerArray):
        # target id 보이면 플래그 ON, 없으면 OFF
        for m in msg.markers:
            if int(m.id) == int(self.aruco_target_id):
                self.aruco_seen = True
                return
        # 프레임에 target이 없으면 false로 내려도 되고, 유지하려면 주석
        self.aruco_seen = False

    def cb_aruco_distance(self, msg: Float32):
        # 아루코 마커까지의 거리(m) 업데이트
        self.aruco_distance = float(msg.data)

    def cb_aruco_finish(self, msg: Bool):
        # 픽앤플레이스 완료 여부를 아루코 플래그로 재사용
        self.aruco_seen = msg.data
        self.get_logger().info(f'{self.aruco_seen}')

    def callback_follow_lane(self, desired_center):
        """
        Receive lane center data to generate lane following control commands.

        If avoidance mode is enabled, lane following control is ignored.
        
        """
        # 회피 모드가 활성화된 경우, 차선추종 제어를 수행하지 않음
        if self.avoid_active:
            return
        
        # 차단바 상태가 slowdown이면 저속 주행 명령 발행
        if self.stop_bar == 'slowdown':
            slowdown = Twist()
            slowdown.linear.x = min(self.MAX_VEL * 0.3, 0.05)
            self.pub_cmd_vel.publish(slowdown)
            self.get_logger().info(f'slow_down: {slowdown.linear.x}')
            return

        # 차단바 stop 또는 신호등 red이면 정지
        if self.stop_bar == 'stop' or self.traffic_light_state in ['red']:
            stop = Twist()
            stop.linear.x = 0.0
            self.pub_cmd_vel.publish(stop)
            self.get_logger().info(f'self.stop_bar: {self.stop_bar}')
            self.get_logger().info(f'self.traffic_light_state: {self.traffic_light_state}')
            self.get_logger().info(f'stop: {stop.linear.x}')
            return
        
        # 아루코가 가까이 있으면 정지 혹은 저속 주행 (팔 제어는 다른 노드가 함)
        #   - aruco_seen: 대상 마커 시야 내 존재 여부
        #   - aruco_distance: 대상 마커까지의 거리
        #   - ARUCO_STOP_DIST: 정지 임계거리(파라미터)
        if self.aruco_seen and self.aruco_distance is not None:
            if self.aruco_distance <= self.ARUCO_STOP_DIST:
                self.pub_cmd_vel.publish(Twist())  # 정지
                return
            elif self.aruco_distance <= (self.ARUCO_STOP_DIST + 0.2):
                t = Twist(); 
                t.linear.x = min(self.MAX_VEL * 0.2, 0.03)  # 근접구간에서는 저속 주행
                self.pub_cmd_vel.publish(t)
                return
        
        # 기본 차선 추종 제어(PD): 중앙 오차 기반 yaw 제어 및 속도 조절
        self.get_logger().info('basic')
        center = desired_center.data          # 목표 중앙 x 좌표(픽셀)
        error = center - 460                  # 460 픽셀을 화면 기준 중앙으로 가정

        Kp = 0.0025                           # 비례 게인
        Kd = 0.007                            # 미분 게인

        angular_z = Kp * error + Kd * (error - self.last_error)
        self.last_error = error

        twist = Twist()
        # 선형 속도: 오차가 클수록 감속. 상한은 0.05 m/s
        # twist.linear.x = min(self.MAX_VEL * (max(1 - abs(error) / 500, 0) ** 2.2), 0.05)--------------------------------------------
        twist.linear.x = min(self.MAX_VEL * (max(1 - abs(error) / 460, 0) ** 1.7), 0.05)
        # 각속도: 계산된 angular_z를 -2.0~2.0 범위에서 클램프하고 부호 반전(좌표계 보정)
        twist.angular.z = -max(angular_z, -2.0) if angular_z < 0 else -min(angular_z, 2.0)
        self.pub_cmd_vel.publish(twist)
        self.get_logger().info(f'basic: {twist.linear.x}')

    def callback_avoid_cmd(self, twist_msg):
        # 회피 모드 속도 명령 업데이트 및 활성 시 즉시 퍼블리시
        self.avoid_twist = twist_msg

        if self.avoid_active:
            self.pub_cmd_vel.publish(self.avoid_twist)

    def callback_avoid_active(self, bool_msg):
        # 회피 모드 활성/비활성 상태 갱신
        self.avoid_active = bool_msg.data
        if self.avoid_active:
            self.get_logger().info('Avoidance mode activated.')
        else:
            self.get_logger().info('Avoidance mode deactivated. Returning to lane following.')

    def callback_stop_bar(self, msg: String):
        # 차단바 상태(go/slowdown/stop) 업데이트
        self.stop_bar = msg.data

    def shut_down(self):
        # 종료 시 안전을 위해 정지 명령 발행
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
