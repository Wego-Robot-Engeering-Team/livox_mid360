# Livox MID-360

Livox MID-360의 포인트클라우드와 내장 IMU를 수신하고, RViz에서 확인하며,
IMU로 추정한 기울기를 이용해 포인트클라우드를 실시간으로 수평 보정하는 ROS 2 프로젝트입니다.

이 브랜치는 **ROS 2 Humble / Ubuntu 22.04**용 개발 코드입니다.
`main`은 프로젝트 안내만 관리하며 Jazzy 코드는 `jazzy` 브랜치에 있습니다.

| 브랜치 | ROS 2 | 운영체제 | 내용 |
| --- | --- | --- | --- |
| [humble](https://github.com/Wego-Robot-Engeering-Team/livox_mid360/tree/humble) | Humble | Ubuntu 22.04 | 소스, 실행·검증 안내 |
| [jazzy](https://github.com/Wego-Robot-Engeering-Team/livox_mid360/tree/jazzy) | Jazzy | Ubuntu 24.04 | 소스, 실행·검증 안내 |

## 구성

- **공식 SDK2**: MID-360 장치 제어 및 센서 데이터 수신.
- **공식 ROS 드라이버**: 포인트클라우드와 IMU ROS 토픽 발행.
- **`livox_mid360`**: 우리가 관리하는 ROS 패키지 하나. IMU 시각화·수평 보정 노드와
  원본·보정 모드 launch, 센서 설정, RViz 구성을 함께 관리합니다.

공식 코드는 `dependencies.repos`로 관리하며 커밋을 고정합니다.
원본 포인트클라우드와 보정 결과는 별도 토픽으로 제공하고, 보정 노드는 별도 프로세스로 실행합니다.

## 개발 위치와 실행 모드

저장소는 ROS 워크스페이스의 `src/livox_mid360`에 배치합니다.
현재 개발 경로는 `~/sensors_ws/src/livox_mid360`입니다.

| 모드 | 내용 |
| --- | --- |
| 원본 확인 | 포인트클라우드·IMU 수신, RViz에서 점군·가속도·각속도 표시 |
| 수평 보정 | IMU 초기화·기울기 추정, 점군 시각에 맞는 자세 보간, 별도 보정 점군 발행 |
| 부하 비교 | 같은 rosbag을 사용해 보정 프로세스 실행 여부에 따른 CPU·RSS 비교 |

수평 보정은 중력 기준 roll/pitch 보정입니다. 실제 경사면은 경사로 유지됩니다.
위치 추정, 절대 방위 추정, 스캔 내부의 이동 왜곡 보정은 이 단계의 범위에 포함하지 않습니다.
센서를 연결한 상태의 동작 검증과 실제 부하 측정은 별도로 수행합니다.

## 시작하기

설치된 ROS 배포판에 맞는 브랜치를 선택하고 해당 브랜치 README의 빌드·실행 절차를 따릅니다.

```bash
mkdir -p ~/sensors_ws/src
git clone -b humble https://github.com/Wego-Robot-Engeering-Team/livox_mid360.git \
  ~/sensors_ws/src/livox_mid360
```

Jazzy에서는 `-b jazzy`를 사용합니다.

## 공식 자료

- [Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2)
- [livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2)
- [MID-360 공식 사양](https://www.livoxtech.com/mid-360/specs)
- [MID-360 통신 문서](https://livox-wiki-en.readthedocs.io/en/latest/tutorials/new_product/mid360/mid360.html)

## 디렉터리 구조

```text
sensors_ws/
├── src/
│   ├── livox_mid360/                 # 센서별 Git 저장소, 루트에는 package.xml 없음
│   │   ├── README.md
│   │   ├── dependencies.repos
│   │   ├── livox_mid360/             # 우리가 만든 기능 패키지 하나
│   │   │   ├── package.xml
│   │   │   ├── CMakeLists.txt
│   │   │   ├── src/                 # 노드·보정 알고리즘·부하 측정 도구
│   │   │   ├── include/livox_mid360/
│   │   │   ├── launch/
│   │   │   ├── config/
│   │   │   └── test/                # colcon test로 실행하는 단위·ROS 연동 검증
│   │   └── third_party/             # 공식 소스와 SDK·드라이버 빌드 설정
│   │       ├── package.xml          # ROS 패키지 이름: livox_ros_driver2
│   │       ├── CMakeLists.txt       # 공식 원본을 그대로 컴파일
│   │       ├── livox_ros_driver2/   # 공식 드라이버 checkout
│   │       └── Livox-SDK2/          # 공식 SDK checkout
│   └── <다른 센서 저장소>/
├── build/
├── install/
└── log/
```

우리 노드와 설정·launch는 `livox_mid360` ROS 패키지 하나에서 관리합니다.
센서 관련 소스와 공식 의존성은 모두 이 Git 저장소 디렉터리 안에 둡니다.
공식 코드의 버전은 `.repos`의 커밋으로 고정하고 저장소 내부 `third_party`로 가져옵니다.
일반 `colcon` 탐색은 `livox_mid360/`과 `third_party/`를 각각 ROS 패키지로 발견합니다.
`third_party` 자체가 패키지이므로 그 안의 공식 checkout을 중복 패키지로 탐색하지 않습니다.

`third_party/CMakeLists.txt`와 `package.xml`은 우리가 관리하는 빌드 연결 설정입니다.
공식 C++·메시지 소스를 그대로 참조하고 SDK core를 함께 컴파일해 드라이버에 연결합니다.
SDK와 드라이버 라이브러리는 같은 `install/livox_ros_driver2` prefix에 설치되므로
별도 SDK 설치, `/usr/local` 라이브러리 탐색, 추가 CMake 인자가 필요하지 않습니다.
공식 원본의 빌드 스크립트나 manifest를 수정하지 않습니다.

## 빌드

ROS 2 Humble이 설치된 Ubuntu 22.04에서 실행합니다.
처음 한 번 공식 의존성을 다운로드하고 시스템 의존성을 설치합니다.

```bash
source /opt/ros/humble/setup.bash
cd ~/sensors_ws/src/livox_mid360
sudo apt install python3-vcstool python3-colcon-common-extensions python3-rosdep
vcs import . < dependencies.repos
# rosdep을 처음 사용하는 시스템에서는 sudo rosdep init을 한 번 수행합니다.
rosdep update
rosdep install --from-paths livox_mid360 third_party \
  --ignore-src --rosdistro humble -r -y
```

이후 워크스페이스 루트에서 일반 빌드를 사용합니다.

```bash
cd ~/sensors_ws
colcon list       # livox_mid360, livox_ros_driver2 확인
colcon build
source install/setup.bash
```

기본 빌드 타입은 Release입니다. SDK 샘플 실행 파일은 빌드하지 않습니다.
SDK 빌드·설치에 sudo가 필요하지 않습니다. 추가 빌드 옵션은 표준 colcon 옵션을 사용합니다.
Humble/Jazzy 전환 시에는 각 Ubuntu/ROS 환경에 별도 워크스페이스를 사용하거나
해당 워크스페이스의 기존 `build`, `install`, `log`를 정리합니다.

## 센서 실행

호스트 NIC에 실제 `host_ip`를 먼저 설정하고 센서와 같은 서브넷으로 연결합니다.
기본 JSON의 IP는 예시이며 장치의 실제 IP로 교체해야 합니다.

```bash
source ~/sensors_ws/install/setup.bash

# 원본 포인트클라우드 + IMU 화살표/수치 확인
ros2 launch livox_mid360 raw.launch.py \
  host_ip:=192.168.1.5 lidar_ip:=192.168.1.12

# 위 실행을 종료한 뒤 보정 모드 실행
ros2 launch livox_mid360 leveled.launch.py \
  host_ip:=192.168.1.5 lidar_ip:=192.168.1.12

# 공통 launch에서도 보정 노드를 선택할 수 있습니다.
ros2 launch livox_mid360 mid360.launch.py leveling:=true rviz:=false \
  host_ip:=192.168.1.5 lidar_ip:=192.168.1.12
```

두 센서 실행 모드를 동시에 실행하면 드라이버가 중복으로 센서에 접속합니다.
한 모드씩 실행하거나, 실행 중인 원본 드라이버에 보정 노드만 추가할 때는
`leveled.launch.py driver:=false`를 사용합니다.

초기화 동안 센서를 정지시켜 둡니다. 기본값은 연속 50개 IMU 샘플로 중력 방향과 자이로 바이어스를
추정합니다. 이후 자이로로 방향을 전파하고 중력 크기에 가까운 가속도 측정으로 보정합니다.
IMU 초기화 전, 데이터 단절, 시각 불일치가 발생하면 보정 점군을 보류하거나 드롭하고 로그를 남깁니다.

| 토픽 | 메시지 | 내용 |
| --- | --- | --- |
| `/livox/lidar` | `sensor_msgs/PointCloud2` | 원본 점군 |
| `/livox/imu` | `sensor_msgs/Imu` | 공식 드라이버의 원본 가속도·각속도 |
| `/livox/points_leveled` | `sensor_msgs/PointCloud2` | 수평 보정 점군, `livox_level` 프레임 |
| `/livox/imu_orientation` | `sensor_msgs/Imu` | 추정 기울기 quaternion, SI 단위 가속도 |
| `/livox/imu_markers` | `visualization_msgs/MarkerArray` | 가속도·각속도 화살표와 수치 |
| `/tf` | `tf2_msgs/TFMessage` | `livox_level`에서 센서 `livox_frame`으로의 회전 |

고정한 공식 드라이버는 가속도를 **g** 단위로 전달하고 orientation을 채우지 않습니다.
우리 노드의 기본 `acceleration_scale=9.80665`는 이를 m/s²로 바꿉니다.
이미 SI 단위인 rosbag을 사용할 때는 `acceleration_scale:=1.0`으로 실행합니다.
추정 자세는 중력 기준 기울기이며 절대 yaw를 제공하지 않습니다. covariance는 추정하지 않아 0(unknown)으로 둡니다.

RViz의 녹색 화살표는 중력을 포함한 가속도 측정(정지 시 위쪽), 분홍 화살표는 각속도입니다.
원본 뷰는 `livox_frame`, 보정 뷰는 `livox_level`을 Fixed Frame으로 사용합니다.
보정 뷰에서 원본 점군을 추가하면 TF에 의해 원본도 수평 좌표계로 변환되어 보입니다.
좌표 자체의 보정 전후를 비교하려면 각 뷰의 Fixed Frame을 구분해서 확인합니다.

점군 한 프레임의 `header.stamp`에 해당하는 IMU 자세를 보간해 전체 점군에 같은 회전을 적용합니다.
intensity, tag, line, timestamp 등 XYZ 외 필드와 원본 stamp는 유지합니다.
빠른 움직임에서의 점별 deskew와 이동 보정은 후속 기능입니다.
가속 운동·진동은 기울기 추정에 오차를 만들 수 있으므로 실제 장착 환경에서 검증해야 합니다.
공식 드라이버의 `extrinsic_parameter`는 모두 0으로 유지해 점군과 IMU의 축을 일치시킵니다.

주요 launch 인자는 `driver`, `leveling`, `rviz`, `imu_visualization`, `host_ip`, `lidar_ip`,
`publish_freq`, `config_file`, `leveling_config`, `acceleration_scale`, `use_sim_time`입니다.
`config_file`로 전체 SDK JSON을 지정하면 IP 인자 대신 해당 파일을 사용합니다.
`points_topic`, `imu_topic`, `leveled_topic`도 변경할 수 있으며 이 경우 `rviz_config`를 맞춰서 지정합니다.
전체 인자는 `ros2 launch livox_mid360 mid360.launch.py --show-args`로 확인합니다.

## 검증

```bash
source /opt/ros/humble/setup.bash
cd ~/sensors_ws
colcon test --packages-select livox_mid360 --return-code-on-test-failure
colcon test-result --verbose
```

핵심 알고리즘 단위 테스트 12개와 실제 ROS 프로세스를 대상으로 하는 합성 데이터 검증을 실행합니다.
합성 검증은 30도 기울어진 센서의 수평 바닥 복원, 시각 보간, SI 변환, 오래된 IMU 거부,
센서 시각 초기화 후 복구를 확인합니다. 실제 센서·네트워크 검증은 별도로 필요합니다.

## 나중에 수행할 부하 비교

보정 노드는 드라이버와 별도 프로세스입니다. `ros2 run livox_mid360 measure_process`로
노드 하나 또는 launch 프로세스와 그 자식들의 CPU·RSS를 CSV와 JSON으로 기록합니다.
100% CPU는 코어 하나를 뜻하며, 여러 프로세스의 RSS 합에는 공유 페이지가 중복 포함될 수 있습니다.
측정 간격 사이에 종료된 짧은 프로세스는 누락될 수 있습니다.

실센서의 원본 모드를 측정하는 예시는 다음과 같습니다. 보정 모드는 `raw`를 `leveled`로 바꿔 동일하게 실행합니다.

```bash
ros2 launch livox_mid360 raw.launch.py rviz:=false imu_visualization:=false \
  host_ip:=192.168.1.5 lidar_ip:=192.168.1.12 > /tmp/mid360-raw.log 2>&1 &
launch_pid=$!
ros2 run livox_mid360 measure_process \
  --pid "$launch_pid" --warmup 5 --duration 60 \
  --output ~/sensors_ws/reports/raw.csv
kill -INT "$launch_pid"
wait "$launch_pid"
```

정확한 비교에는 같은 원본 토픽을 기록한 rosbag을 같은 속도와 구간으로 재생하는 방법을 권장합니다.

```bash
# 원본 드라이버 실행 중 기록. 초기 정지 구간도 포함합니다.
ros2 bag record /livox/lidar /livox/imu -o ~/sensors_ws/bags/mid360

# 드라이버를 종료한 뒤, 보정 모드의 입력으로 재생
ros2 launch livox_mid360 leveled.launch.py driver:=false rviz:=false \
  imu_visualization:=false use_sim_time:=true
# 별도 터미널
ros2 bag play ~/sensors_ws/bags/mid360 --clock
```

재생 프로세스와 입력 토픽 소비 조건을 동일하게 유지하고, 보정 프로세스의 CPU/RSS 및 전체 프로세스 합을
함께 비교합니다. RViz·시각화 노드는 양쪽 모두 끄거나 양쪽 모두 같은 조건으로 둡니다.
CPU/RSS만으로 판단하지 않고 입력·출력 Hz, 점 수, 드롭 로그도 함께 확인합니다.
실제 부하 비교 결과는 아직 측정하지 않았습니다.
