# Livox MID-360

ROS 2 Jazzy / Ubuntu 24.04 — 점군·IMU 수신, RViz, 수평·스캔 회전 보정.

```bash
source /opt/ros/jazzy/setup.bash
sudo apt install python3-vcstool python3-colcon-common-extensions python3-rosdep
cd ~/sensors_ws/src/livox_mid360
vcs import . < dependencies.repos
# rosdep 최초 사용 시 sudo rosdep init
rosdep update
rosdep install --from-paths livox_mid360 third_party --ignore-src --rosdistro jazzy -r -y
cd ~/sensors_ws
colcon build
source install/setup.bash

# NIC를 센서와 같은 서브넷에 설정하고 실제 IP로 교체. 모드는 하나씩 실행하며 초기화 중 정지.
ros2 launch livox_mid360 bringup.launch.py host_ip:=192.168.1.5 lidar_ip:=192.168.1.12
ros2 launch livox_mid360 leveled.launch.py host_ip:=192.168.1.5 lidar_ip:=192.168.1.12
# 점별 timestamp가 없는 데이터는 leveled.launch.py에 deskew:=false 추가.
```
