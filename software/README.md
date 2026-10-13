# software — 상위 컴퓨터 소프트웨어

Jetson, RDK 같은 상위 컴퓨터에서 도는 인식 · 판단 · 주행 소프트웨어입니다.

## 대상 컴퓨터

| 이름 | 보드 | OS / 환경 |
| --- | --- | --- |
| dongki | Jetson Orin Nano 8 GB | JetPack 6.2 (Ubuntu 22.04), CUDA 12.6 |
| sunrise | RDK 보드 | aarch64 Ubuntu 22.04, ROS 2 Humble |

둘 다 Ubuntu 22.04이므로 ROS 2 **Humble**을 기준으로 합니다.

## 구성

```
software/
└── ros2_ws/
    └── src/        ROS 2 패키지 (nubi_*)
```

빌드는 워크스페이스 루트에서 합니다.

```bash
cd software/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## 패키지 계획

아직 만든 패키지는 없습니다. 만들 때 이 표를 갱신합니다.

| 패키지 | 역할 |
| --- | --- |
| `nubi_description` | URDF/xacro, 메시. 센서 장착 위치 |
| `nubi_base` | `cmd_vel` → 바퀴 제어기 시리얼 명령, 텔레메트리 → `odom` · 배터리 상태 |
| `nubi_bringup` | 실기 실행용 launch, 파라미터 |
| `nubi_perception` | 깊이 카메라 처리, 장애물 판단 |
| `nubi_navigation` | SLAM, Nav2 설정 |

## 규칙

- 패키지 이름은 `nubi_` + snake_case.
- 단위는 SI (m, rad, s), 좌표 프레임은 REP-105 (`map` → `odom` → `base_link`).
- 기기별 설정(포트 이름, IP)은 launch 인자나 파라미터 파일로 빼고 코드에 박지 않습니다.
- `build/`, `install/`, `log/`는 git에 올리지 않습니다.
- ROS 밖에서 먼저 시험하는 코드는 [experiments/](../experiments/)에서 시작하고, 자리를 잡으면
  패키지로 옮깁니다.
