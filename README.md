# Livox MID-360

ROS 2 Humble / Ubuntu 22.04 — 점군·IMU 수신, RViz, 수평·스캔 회전 보정.

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

# IP 설정: livox_mid360/config/MID360_config.json. PC NIC도 같은 주소로 설정.
# 모드는 하나씩 실행하며 초기화 중 정지.
ros2 launch livox_mid360 bringup.launch.py
ros2 launch livox_mid360 leveled.launch.py
# 점별 timestamp가 없는 데이터는 leveled.launch.py에 deskew:=false 추가.
```
