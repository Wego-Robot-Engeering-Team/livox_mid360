# Livox MID-360

Livox MID-360의 포인트클라우드와 내장 IMU를 수신하고 RViz에서 확인하는 ROS 2 프로젝트입니다.
IMU 기반 roll/pitch 수평 보정과 보정 노드의 CPU·메모리 부하 측정 도구를 제공합니다.

`main`은 README만 관리합니다. 소스와 빌드·실행·검증 안내는 아래 개발 브랜치에 있습니다.

| 브랜치 | ROS 2 | 운영체제 |
| --- | --- | --- |
| [humble](https://github.com/Wego-Robot-Engeering-Team/livox_mid360/tree/humble) | Humble | Ubuntu 22.04 |
| [jazzy](https://github.com/Wego-Robot-Engeering-Team/livox_mid360/tree/jazzy) | Jazzy | Ubuntu 24.04 |

개발 브랜치의 저장소 구조는 다음과 같습니다.

```text
sensors_ws/src/livox_mid360/
├── README.md
├── dependencies.repos              # 공식 SDK2·드라이버 버전
├── livox_mid360/                   # IMU 시각화·수평 보정 ROS 패키지
│   └── src/, include/, launch/, config/, rviz/, test/
└── third_party/                    # 공식 SDK2·드라이버와 빌드 설정
```

설치한 ROS 배포판에 맞는 브랜치를 워크스페이스의 `src` 아래에 받습니다.

```bash
mkdir -p ~/sensors_ws/src
git clone -b jazzy https://github.com/Wego-Robot-Engeering-Team/livox_mid360.git \
  ~/sensors_ws/src/livox_mid360
```

Humble에서는 `-b humble`을 사용합니다.
의존성 준비, `colcon build`, 센서 실행과 테스트는 선택한 개발 브랜치의 README를 따릅니다.
