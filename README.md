# Livox MID-360

Livox MID-360의 점군과 내장 IMU를 수신하고, RViz 시각화와 IMU 기반 수평·스캔 회전 보정을 제공하는 ROS 2 저장소입니다.

## 1. Livox MID-360

MID-360은 이동 로봇의 주변 인식에 사용하는 소형 3D 라이다입니다. 수평 360° 시야와 내장 IMU를 제공합니다. [공식 제품 소개](https://www.livoxtech.com/mid-360)

아래는 [Livox 공식 제원](https://www.livoxtech.com/mid-360/specs) 기준입니다.

| 항목 | 제원 |
| --- | --- |
| 시야각 | 수평 360°, 수직 −7°~52° |
| 탐지 거리 | 40 m @ 반사율 10%, 70 m @ 80% — 주변 조도 100 klx |
| 거리 정밀도(1σ) | ≤ 2 cm @ 10 m — 25°C, 반사율 80% |
| 점군 출력 | 초당 200,000점(첫 번째 반사), 프레임률 대표값 10 Hz |
| 내장 IMU | ICM40609 |
| 통신 | 100BASE-TX Ethernet, PTPv2·GPS 시각 동기화 지원 |
| 전원 | 9~27 V DC, 평균 6.5 W — 저온 예열 시 최대 14 W |
| 크기·무게 | 65×65×60 mm, 265 g |
| 사용 환경 | IP67, −20~55°C |

## 2. 저장소 기능

- 공식 Livox SDK2·ROS 드라이버로 점군(`PointCloud2`)과 IMU(`Imu`) 수신.
- RViz에서 원본 점군, IMU 화살표·수치, 보정 점군 확인.
- IMU로 중력 방향을 추정해 roll/pitch 수평 정렬(`gravity alignment` / `leveling`).
- 점별 측정 시각과 자이로 회전 이력으로 스캔 중 회전 왜곡 보정(`rotational deskew`).
- 보정 실행 전후 부하 비교에 사용할 CPU·RSS 메모리 측정 도구 제공.

수평 정렬은 중력을 기준으로 하므로 경사진 지면의 경사는 유지됩니다. 현재 운동 보정 범위는 회전이며, 병진 이동 보정·절대 방위 추정·SLAM은 포함하지 않습니다.

주요 토픽은 `/livox/lidar`, `/livox/imu`, `/livox/points_leveled`입니다.

## 3. 브랜치와 의존성

`main`은 프로젝트 안내만 관리합니다. 설치·실행 명령과 소스는 배포판에 맞는 브랜치를 사용합니다.

| 브랜치 | ROS 2 | 운영체제 | 안내 |
| --- | --- | --- | --- |
| `humble` | Humble | Ubuntu 22.04 | [설치·실행](https://github.com/Wego-Robot-Engeering-Team/livox_mid360/blob/humble/README.md) |
| `jazzy` | Jazzy | Ubuntu 24.04 | [설치·실행](https://github.com/Wego-Robot-Engeering-Team/livox_mid360/blob/jazzy/README.md) |

공식 [Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2)와 [livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2)는 수정 없이 저장소 안의 `third_party/`에서 관리합니다. 각 개발 브랜치의 `dependencies.repos`에 지정된 커밋을 `vcstool`로 가져옵니다.
