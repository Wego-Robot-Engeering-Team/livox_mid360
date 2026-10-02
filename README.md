# Livox MID-360

Ubuntu 24.04 / ROS 2 Jazzy가 설치된 환경 기준입니다. [ROS 2 설치 안내](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)

## 1. 설치

```bash
# ROS 2 환경과 의존성 관리·빌드 도구
source /opt/ros/jazzy/setup.bash
sudo apt install git python3-vcstool python3-colcon-common-extensions python3-rosdep

# Jazzy 브랜치와 고정 버전의 공식 SDK·드라이버 받기
mkdir -p ~/sensors_ws/src
git clone -b jazzy https://github.com/Wego-Robot-Engeering-Team/livox_mid360.git ~/sensors_ws/src/livox_mid360
cd ~/sensors_ws/src/livox_mid360
vcs import . < dependencies.repos

# ROS 의존성 설치 — rosdep 최초 사용 시 먼저 sudo rosdep init
rosdep update
rosdep install --from-paths livox_mid360 third_party --ignore-src --rosdistro jazzy -r -y

# 워크스페이스 빌드
cd ~/sensors_ws
colcon build
```

## 2. 실행

기본 설정은 PC `192.168.1.5/24`, 센서 `192.168.1.136`입니다. PC 이더넷 주소를 맞추고 전원·이더넷을 연결합니다.
주소 변경은 [SDK 설정](livox_mid360/config/MID360_config.json)에서 하고 다시 빌드합니다.
두 모드는 하나씩 실행하며, 보정 초기화 중에는 센서를 정지시켜 둡니다.

```bash
# 새 터미널에서 ROS 2와 빌드한 패키지 로드
source /opt/ros/jazzy/setup.bash
source ~/sensors_ws/install/setup.bash

# 원본 점군·IMU 수신과 RViz
ros2 launch livox_mid360 bringup.launch.py

# 위 런치 종료 후: IMU 수평 보정·스캔 회전 보정과 RViz
ros2 launch livox_mid360 leveled.launch.py
```

RViz 없이 실행하려면 `rviz:=false`, 점별 시각이 없는 데이터는 `deskew:=false`를 추가합니다.
보정 파라미터는 [leveling.yaml](livox_mid360/config/leveling.yaml)에서 설정합니다.
