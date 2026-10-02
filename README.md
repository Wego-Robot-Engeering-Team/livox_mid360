# Livox MID-360

MID-360 포인트클라우드·IMU 수신, RViz 시각화, IMU 기반 수평 보정.
**ROS 2 Humble / Ubuntu 22.04**용입니다. Jazzy는 `jazzy`, 프로젝트 안내는 `main` 브랜치에서 관리합니다.

## 구조

```text
livox_mid360/
├── dependencies.repos
├── livox_mid360/        # 우리 노드·테스트
│   ├── launch/          # bringup.launch.py, leveled.launch.py
│   ├── config/          # 센서·보정 설정
│   └── rviz/            # raw.rviz, leveled.rviz
└── third_party/         # SDK·드라이버 빌드 패키지
    ├── Livox-SDK2/
    └── livox_ros_driver2/
```

공식 소스는 `dependencies.repos`에 커밋을 고정하고 수정 없이 빌드합니다.

## 빌드

ROS 2 설치 후 최초 한 번 의존성을 준비합니다.

```bash
source /opt/ros/humble/setup.bash
sudo apt install python3-vcstool python3-colcon-common-extensions python3-rosdep
mkdir -p ~/sensors_ws/src
git clone -b humble https://github.com/Wego-Robot-Engeering-Team/livox_mid360.git \
  ~/sensors_ws/src/livox_mid360
cd ~/sensors_ws/src/livox_mid360
vcs import . < dependencies.repos
# rosdep 최초 사용 시 sudo rosdep init
rosdep update
rosdep install --from-paths livox_mid360 third_party --ignore-src --rosdistro humble -r -y

cd ~/sensors_ws
colcon build
source install/setup.bash
```

## 실행

호스트 NIC와 센서를 같은 서브넷으로 연결하고, 아래 IP를 실제 설정값으로 바꿉니다.

```bash
source ~/sensors_ws/install/setup.bash

# 원본 점군·IMU + RViz
ros2 launch livox_mid360 bringup.launch.py \
  host_ip:=192.168.1.5 lidar_ip:=192.168.1.12

# 원본 모드를 종료한 뒤 수평 보정 + RViz
ros2 launch livox_mid360 leveled.launch.py \
  host_ip:=192.168.1.5 lidar_ip:=192.168.1.12
```

- 초기 50개 IMU 샘플 동안 센서를 정지시킵니다. 보정은 중력 기준 roll/pitch이며 yaw·deskew는 제공하지 않습니다.
- 실행 중인 드라이버에 보정만 추가할 때는 `leveled.launch.py driver:=false`를 사용합니다.
- 공식 IMU 가속도는 g 단위입니다. 이미 m/s²인 입력에는 `acceleration_scale:=1.0`을 지정합니다.

토픽: `/livox/lidar`(원본 점군), `/livox/imu`, `/livox/points_leveled`(보정 점군).
전체 인자: `ros2 launch livox_mid360 bringup.launch.py --show-args`.

## 검증

```bash
cd ~/sensors_ws
colcon test --packages-select livox_mid360 --return-code-on-test-failure
colcon test-result --verbose
```

## 부하 측정

`12345`를 측정할 launch 또는 노드 PID로 바꿉니다. CPU·RSS를 CSV와 JSON으로 저장합니다.

```bash
ros2 run livox_mid360 measure_process --pid 12345 \
  --warmup 5 --duration 60 --output ~/sensors_ws/reports/raw.csv
```

동일 rosbag으로 원본·보정 모드를 비교하고, 양쪽에 `rviz:=false imu_visualization:=false`를 지정합니다.
재생 시 `driver:=false use_sim_time:=true`로 실행하고 `ros2 bag play <bag> --clock`을 사용합니다.
실제 센서 검증과 부하 비교는 아직 수행하지 않았습니다.
