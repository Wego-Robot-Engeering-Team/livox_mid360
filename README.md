# Livox MID-360

Livox MID-360의 포인트클라우드와 내장 IMU를 수신하고, RViz에서 확인하며,
IMU로 추정한 기울기를 이용해 포인트클라우드를 실시간으로 수평 보정하는 ROS 2 프로젝트입니다.

`main`은 프로젝트 안내 README만 관리합니다. 소스·설정·빌드·검증 절차는
배포판별 `humble`, `jazzy` 브랜치에서 관리합니다.

| 브랜치 | ROS 2 | 운영체제 | 내용 |
| --- | --- | --- | --- |
| [humble](https://github.com/Wego-Robot-Engeering-Team/livox_mid360/tree/humble) | Humble | Ubuntu 22.04 | 소스, 빌드 스크립트, 실행·검증 안내 |
| [jazzy](https://github.com/Wego-Robot-Engeering-Team/livox_mid360/tree/jazzy) | Jazzy | Ubuntu 24.04 | 소스, 빌드 스크립트, 실행·검증 안내 |

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
git clone -b jazzy https://github.com/Wego-Robot-Engeering-Team/livox_mid360.git \
  ~/sensors_ws/src/livox_mid360
```

Humble에서는 `-b humble`을 사용합니다.

## 공식 자료

- [Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2)
- [livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2)
- [MID-360 공식 사양](https://www.livoxtech.com/mid-360/specs)
- [MID-360 통신 문서](https://livox-wiki-en.readthedocs.io/en/latest/tutorials/new_product/mid360/mid360.html)

## 저장소 경계

센서 하나당 저장소 하나를 사용합니다. MID-360의 우리 코드·설정·공식 의존성은
모두 `sensors_ws/src/livox_mid360` 안에 둡니다.

```text
sensors_ws/src/
├── livox_mid360/
│   ├── package.xml, CMakeLists.txt   # 우리 ROS 패키지 하나
│   ├── src/, include/, test/
│   ├── launch/, config/, scripts/
│   ├── dependencies.repos
│   └── third_party/
│       ├── livox_ros_driver2/
│       └── Livox-SDK2/
└── <다른 센서 저장소>/
```

우리 노드는 `cloud_leveling_node`와 `imu_visualizer_node`입니다.
패키지는 하나로 관리하고 프로세스는 따로 실행하므로 보정 노드의 추가 부하를 측정할 수 있습니다.
외부 코드의 주소·커밋은 `.repos`에 기록하고 저장소 내부 `third_party`로 가져옵니다.
빌드 스크립트가 SDK를 먼저 빌드하고 우리 패키지와 내부 드라이버 경로를 명시적으로 지정합니다.
