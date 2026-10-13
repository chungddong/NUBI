# 누비 (NUBI) — Modular Autonomous Rover

> 주변을 인식하고 스스로 길을 찾아가는, 모듈형 자율주행 로봇 누비.

누비는 구동·전원·컴퓨트·센서를 모듈 단위로 나눠, 필요에 따라 바꿔 달 수 있게 만드는 자율주행
로봇 플랫폼입니다.

이 저장소는 누비의 기구 설계, 펌웨어, 상위 소프트웨어, 문서, 데이터를 한곳에서 관리하는 통합
저장소입니다. 지금까지 여러 폴더와 보드에 흩어져 있던 작업을 이곳으로 모으고, 새 작업도 여기서
시작합니다.

## 시스템 구성

```
상위 컴퓨터   Jetson Orin Nano / RDK 보드 — 인식 · 판단 · 경로 계획
   │  USB 시리얼 115200, 텍스트 명령 (s / t / x / m / w / h …)
   ▼
바퀴 제어기   LOLIN D32 (ESP32) — 안전장치 · 좌우 믹싱 · Wi-Fi 조종 페이지
   │  UART 19200 8N1, 명령 9 B / 텔레메트리 22 B, CRC16-XMODEM
   ▼
모터 보드     호버보드 GD32 메인보드 (RoboDurden Hoverboard-Firmware-Hack-Gen2.x)
   │  3상 + 홀센서
   ▼
구동         6.5" BLDC 허브모터
```

계층별 책임과 인터페이스는
[docs/architecture/system-overview.md](docs/architecture/system-overview.md)에 정리했습니다.

## 현재 상태 (2026-10-13)

| 영역 | 상태 |
| --- | --- |
| 기구 | 호버보드 허브모터 4륜 시제품 프레임 조립됨. CAD는 이 저장소에서 새로 시작 |
| 바퀴 제어 | ESP32 펌웨어 동작. 보드 모드로 4륜 회전 확인, 탱크 모드는 구현만 하고 지상 주행 전 |
| 상위 제어 | 깊이 카메라 장애물 가드(RDK), 초파리 전뇌 시뮬레이션 폐루프 주행(Jetson, 바퀴 띄움) 실험 |
| 자율주행 | SLAM · 내비게이션은 이 저장소에서 새로 시작 |

## 저장소 구조

```
NUBI/
├── docs/                  문서
│   ├── architecture/      시스템 구성, 계층 간 인터페이스
│   ├── decisions/         설계 결정 기록 (ADR)
│   ├── worklog/           월별 작업 기록
│   ├── guides/            셋업 · 빌드 · 운용 방법
│   ├── troubleshooting/   문제 해결과 실패 기록
│   ├── references/        데이터시트, 외부 자료 정리
│   └── assets/            문서용 이미지 · 다이어그램
├── hardware/              기구 · 전장 설계
│   ├── mechanical/        CAD 원본, 빌더 스크립트, 벤더 STEP, 내보낸 파일
│   ├── electrical/        배선도, 핀맵, 전원 계통
│   └── bom/               부품 목록
├── firmware/              MCU 펌웨어
│   ├── wheel-controller/      LOLIN D32 (ESP32) 바퀴 제어기
│   └── hoverboard-mainboard/  호버보드 GD32 메인보드 빌드 설정
├── software/              상위 컴퓨터 소프트웨어
│   └── ros2_ws/           ROS 2 Humble 워크스페이스
├── sim/                   시뮬레이션 (월드, 시나리오)
├── tools/                 PC 쪽 개발 · 분석 도구
├── data/                  데이터 — 원본은 git에 올리지 않음
│   ├── raw/               센서 원본 (rosbag, 이미지)
│   ├── processed/         가공 결과
│   ├── logs/              주행 · 텔레메트리 로그
│   ├── maps/              SLAM 맵
│   └── calibration/       센서 캘리브레이션 값
├── experiments/           날짜별 실험과 프로토타입
├── AGENTS.md              AI 에이전트(Claude Code, Codex) 작업 규칙
└── CLAUDE.md              AGENTS.md를 불러옴
```

각 최상위 폴더에는 그 폴더에 무엇을 어떻게 두는지 적은 README가 있습니다.

**한 가지 원칙:** 한 부품에만 해당하는 문서(펌웨어의 README, 디버깅 기록 등)는 그 부품 폴더에
두고, 여러 계층에 걸친 문서는 `docs/`에 둡니다.

## 기존 작업 통합 계획

하나씩 옮기면서 상태를 갱신합니다.

| 기존 위치 | 내용 | 옮길 곳 | 상태 |
| --- | --- | --- | --- |
| `F:\Develop\lolind32-wheel\` ([GitHub](https://github.com/chungddong/lolind32-wheel)) | ESP32 바퀴 제어 펌웨어, PC 도구, README · WORKLOG · ISSUES · DEBUGGING | [firmware/wheel-controller/](firmware/wheel-controller/) | **완료** (2026-10-13, `git subtree`로 커밋 이력 포함) |
| 이전 세션 스크래치패드 `mainboard-out/` | GD32 커스텀 빌드 (bin 2개, `config.master.h`, `config.slave.h`, 패치) | `firmware/hoverboard-mainboard/` | 폴더가 비어 있음. 채팅 첨부본을 찾거나 재빌드 ([ISSUES H1](firmware/wheel-controller/ISSUES.md)) |
| RDK 보드 "sunrise" `~/Develop/autocar` | 깊이 카메라 장애물 가드 (`guard.py`, `obstacle.py`, `hoverlink.py`) | `experiments/2026-09-17_depth-guard/` | 보드에서 가져와야 함 |
| Jetson "dongki" `~/Study/flysim` | 초파리 전뇌 폐루프 주행, `hover_link.py`, 웹 UI | `experiments/2026-09-18_flysim-drive/` | 보드에서 가져와야 함 |

`lolind32-wheel`은 커밋 이력째 가져왔기 때문에 원래 커밋 해시(`6e0a3a6`, `cd922a5`, `55f450a` 등)가
이 저장소에서도 그대로 통하고, `git blame`도 원래 커밋까지 따라갑니다. 다만
`git log -- firmware/wheel-controller`는 통합 커밋에서 멈추므로, 통합 전 이력은 `git log 3cca93e^2`로
봅니다.

## 작업 규칙

**이름**
- 폴더와 파일은 소문자 kebab-case를 씁니다 (`wheel-controller`, `depth-guard`).
  ROS 2 패키지는 언어 관례에 맞춰 `nubi_` 접두어 + snake_case입니다 (`nubi_bringup`).
- 날짜가 붙는 것은 `YYYY-MM-DD_` 접두어를 씁니다 (실험 폴더, 데이터 세트).

**좌표계와 단위**
- ROS REP-103 규약: x 전진, y 왼쪽, z 위. 기구 CAD도 같은 방향을 쓰고, 지면을 z = 0으로 둡니다.
- 기구 도면은 mm, 소프트웨어는 SI 단위(m, rad, s)입니다.

**문서**
- README는 "지금 상태", 작업 기록은 "시간순으로 무엇을 왜 했는지", 결정 기록은 "왜 이 방식을
  골랐는지"를 적습니다. 자세한 규칙은 [docs/README.md](docs/README.md)에 있습니다.
- 작업한 날은 [docs/worklog/](docs/worklog/)에 기록을 남기고, 수치와 판단 근거를 같이 적습니다.

**데이터**
- rosbag, 이미지, 대용량 로그는 커밋하지 않습니다. 맵과 캘리브레이션 값처럼 작고 재현에 필요한
  것만 커밋합니다. 자세한 기준은 [data/README.md](data/README.md)에 있습니다.

**커밋**
- 메시지 앞에 영역을 붙입니다: `docs:`, `hardware:`, `firmware:`, `software:`, `sim:`, `tools:`,
  `data:`, `exp:`.

## 라이선스

미정.
