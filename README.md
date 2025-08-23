# 🚀 두산로보틱스 부트캠프 ROKEY 4기 – B-3 비긴어게인조

## 팀 소개
- **팀명:** B-3 비긴어게인조  
- **팀원:** 홍송은(조장), 박준수, 이송은, 인제민  

---

## 프로젝트 개요
- **주제:** 협동-3: 디지털 트윈 기반 서비스 로봇 운영 시스템 구성  
- **부주제:** 터틀봇3을 활용한 자율주행  

본 깃허브 저장소는 **실제 로봇 구현 코드**를 다룹니다.  
시뮬레이션 관련 코드는 **별도의 GitHub 리포지토리**에 존재합니다.  

👉 시뮬레이션 전용 코드 리포지토리: [Simulation Repository](https://github.com/yellowHSE/ds-cobot-auto-driving-simulation)  

---

## 수행 절차 및 방법
### 1. 시뮬레이션 환경 (별도 GitHub)
- 기본 코드 분석 및 환경 구성 (08.07 - 08.09)  
- 기능 구현 (08.10 - 08.13)  
  - 차선 감지, 신호등 감지 및 제어, 속도 제한 구간 인식, 사람 감지 및 회피  

### 2. 실제 로봇 구현 (본 리포지토리)
- **기존 기능 최적화 (08.13 - 08.17)**  
  - 시뮬레이션 → 실제 맵 환경에 맞게 기능 최적화  
  - 실제 환경에서 로직 변경 및 적용  

- **Pick and Place 구현 (08.17 - 08.21)**  
  - ArUco Marker 감지 및 로봇 팔 제어  
  - 전체 코드 통합  

---

## 실행 방법
아래 명령어를 순서대로 실행합니다. (사진2 참고)

```bash
# 로봇 하드웨어 구동
ros2 launch turtlebot3_manipulation_bringup hardware.launch.py
ros2 launch turtlebot3_manipulation_moveit_config moveit_core.launch.py

# ArUco 및 YOLO 관련
ros2 launch aruco_yolo compressed_only.launch.py
ros2 launch aruco_yolo aruco_yolo.launch.py

# MoveIt 그룹 실행
ros2 launch turtlebot_moveit move_group.launch.py

# 카메라 보정
ros2 launch turtlebot3_autorace_camera intrinsic_camera_calibration.launch.py
ros2 launch turtlebot3_autorace_camera extrinsic_camera_calibration.launch.py

# 인식 노드 실행
ros2 launch turtlebot3_autorace_detect detect_lane.launch.py
ros2 launch turtlebot3_autorace_detect detect_level_crossing.launch.py
ros2 launch turtlebot3_autorace_detect detect_traffic_light.launch.py

# 주행 제어
ros2 launch turtlebot3_autorace_mission control_lane.launch.py

# Pick & Place (Aruco 기반)
python3 aruco_pick_and_place.py
```

---

## 장비 및 환경
### 하드웨어
- **TurtleBot3 Waffle + Robot Arm**: 주행 및 Pick & Place 수행  
- **카메라**: 차선, 신호등, 차단바 인식  
- **아두이노 제어장치**: 신호등/차단바 제어  
- **ArUco 마커 & 차선 맵**: 위치 인식 및 주행 환경 구성  

### 소프트웨어
- **ROS2 Humble**: 노드 기반 로봇 제어  
- **OpenCV & YOLO**: 영상 인식 및 주행 제어  

---

## 기능
- 차선 감지 및 자율주행  
- 신호등 감지 및 주행 제어  
- 차단바 감지 및 정지/재출발 로직  
- 아루코 마커 기반 Pick & Place (로봇 팔 제어 포함)  

---

## 참고 자료
- [TurtleBot3 Quick Start Guide](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/)  