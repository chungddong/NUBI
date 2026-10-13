# sim — 시뮬레이션

## 지금 있는 시뮬레이터 — 차체 웹 시뮬레이터

차체 설계 단계에서 만든 three.js 웹 시뮬레이터가
[hardware/vehicle/models/vehicle/sim/](../hardware/vehicle/models/vehicle/sim/README.md)에 있습니다.
차체 버전의 실제 형상(메시)을 불러와 주행 · 센서 시야 · 전원 · 사각지대 · 전복 · 비상정지 방식 ·
자율주행(점유 격자 + A*)을 추정합니다. 결과는 [D-19](../hardware/vehicle/models/vehicle/DECISIONS.md)에
있습니다.

차체 버전 폴더와 묶여 있어 그 자리에 둡니다. 실행은 `hardware/vehicle/`에서 합니다.

```bash
.venv/Scripts/python.exe models/vehicle/sim/web_export.py v28_wiring
```

```bash
.venv/Scripts/python.exe models/vehicle/sim/web/serve.py 8748
```

브라우저에서 http://localhost:8748 을 엽니다. Claude Code에서는 저장소 루트 `.claude/launch.json`의
`vehicle-sim`으로 띄울 수 있습니다.

## 이 폴더 — ROS 2용 시뮬레이션 (예정)

실제 ROS 2 노드(`software/ros2_ws`)를 그대로 돌려 보는 시뮬레이션을 둡니다. 시뮬레이터(Gazebo,
Isaac Sim 등)는 아직 정하지 않았고, 정하면 아래처럼 나눌 계획입니다.

```
sim/
├── worlds/       맵 · 환경 파일
├── models/       시뮬레이션 전용 모델 (로봇 본체 URDF는 software/ros2_ws/src/nubi_description)
└── scenarios/    시험 시나리오 (출발점, 장애물 배치, 성공 조건)
```

URDF는 차체 설계의 STEP(`hardware/vehicle/models/vehicle/<버전>/output/<버전>.step`)과 좌표계(지면 z = 0,
+x 전진, +y 왼쪽)가 같으므로 축은 그대로 두고 단위만 mm → m로 바꾸면 됩니다.
