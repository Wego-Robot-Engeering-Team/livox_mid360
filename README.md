# Livox MID-360

ROS 2 Humble / Ubuntu 22.04. 점군·IMU 수신, RViz 표시, 중력 기준 roll/pitch 보정.

## 빌드

```bash
source /opt/ros/humble/setup.bash
sudo apt install python3-vcstool python3-colcon-common-extensions python3-rosdep
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

호스트 NIC에 센서와 같은 서브넷의 IP를 설정하고, 아래 IP를 실제 값으로 바꿉니다.

```bash
source ~/sensors_ws/install/setup.bash
ros2 launch livox_mid360 bringup.launch.py host_ip:=192.168.1.5 lidar_ip:=192.168.1.12
# 위 실행을 종료한 뒤 보정 모드 실행
ros2 launch livox_mid360 leveled.launch.py host_ip:=192.168.1.5 lidar_ip:=192.168.1.12
```

초기화 동안 센서를 정지시킵니다. 기존 드라이버를 사용할 때는 `driver:=false`를 추가합니다.

## 검증·부하 측정

```bash
cd ~/sensors_ws
colcon test --packages-select livox_mid360 --return-code-on-test-failure
colcon test-result --verbose
ros2 run livox_mid360 measure_process --pid 12345 --output ~/sensors_ws/reports/cpu.csv
```

`12345`를 실제 launch/노드 PID로 바꿉니다. CPU·RSS를 CSV/JSON으로 기록합니다.
동일한 입력·시각화 조건에서 비교합니다. 실제 센서 검증·부하 비교는 미완료입니다.
