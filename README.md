# Livox MID-360

Ubuntu 22.04 / ROS 2 Humble가 설치된 환경 기준입니다.
`~/wego_ws`는 예시 워크스페이스 경로입니다.

## 1. 설치

```bash
# ROS 2 환경과 컴파일·의존성 관리 도구
source /opt/ros/humble/setup.bash
sudo apt install build-essential git python3-vcstool python3-colcon-common-extensions python3-rosdep

# Humble 브랜치와 고정 버전의 공식 SDK·드라이버 받기
mkdir -p ~/wego_ws/src
git clone -b humble https://github.com/Wego-Robot-Engeering-Team/livox_mid360.git ~/wego_ws/src/livox_mid360
cd ~/wego_ws/src/livox_mid360
vcs import . < dependencies.repos

# ROS 의존성 설치 — rosdep은 최초 사용 시 자동 초기화
[ -f /etc/ros/rosdep/sources.list.d/20-default.list ] || sudo rosdep init
rosdep update
rosdep install --from-paths livox_mid360 third_party --ignore-src --rosdistro humble -r -y

# 워크스페이스 빌드
cd ~/wego_ws
colcon build
```

## 2. 실행

기본 설정은 PC `192.168.1.5/24`, 센서 `192.168.1.136`입니다. PC 이더넷 주소를 맞추고 전원·이더넷을 연결합니다.
주소가 다르면 [SDK 설정](livox_mid360/config/MID360_config.json)의 `host_ip`(PC)와 `lidar_configs`의 `ip`(센서)를 수정하고 다시 빌드합니다.
아래 두 모드 중 하나만 실행합니다. 보정 초기화 중에는 센서를 정지시켜 둡니다.

```bash
# 원본 점군·IMU 수신과 RViz — 새 터미널에서 실행
source /opt/ros/humble/setup.bash
source ~/wego_ws/install/setup.bash
ros2 launch livox_mid360 bringup.launch.py
```

```bash
# IMU 수평·스캔 회전 보정과 RViz — 기존 런치를 Ctrl+C로 종료한 뒤 실행
source /opt/ros/humble/setup.bash
source ~/wego_ws/install/setup.bash
ros2 launch livox_mid360 leveled.launch.py
```

RViz 없이 실행하려면 `rviz:=false`, 점별 시각이 없는 데이터는 `deskew:=false`를 추가합니다.
가속도가 이미 m/s²인 데이터는 `acceleration_scale:=1.0`을 추가합니다.
보정 파라미터는 [leveling.yaml](livox_mid360/config/leveling.yaml)에서 설정합니다.
