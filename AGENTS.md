# 누비 (NUBI) — 에이전트 작업 규칙

모듈형 자율주행 로봇 누비의 통합 저장소입니다. 구조와 규칙의 기준은 [README.md](README.md)이고,
시스템 구성은 [docs/architecture/system-overview.md](docs/architecture/system-overview.md)에 있습니다.

## 어디에 두나

| 무엇 | 위치 |
| --- | --- |
| 여러 계층에 걸친 문서 | `docs/` (구성은 `docs/README.md`) |
| 한 부품에만 해당하는 문서 | 그 부품 폴더 (예: `firmware/wheel-controller/README.md`) |
| 차체 설계 (build123d 모델, 버전, BOM, 부품 조사, FEA, 웹 시뮬레이터) | `hardware/vehicle/` |
| 차체 모델 밖의 전장 문서 (핀맵, 회로도) | `hardware/electrical/` |
| MCU 펌웨어 | `firmware/<보드>/` |
| 상위 컴퓨터 코드 (ROS 2) | `software/ros2_ws/src/nubi_*` |
| ROS 2용 시뮬레이션 | `sim/` |
| 아직 자리 잡지 않은 시험 코드 | `experiments/YYYY-MM-DD_<주제>/` |
| 센서 원본, 로그 | `data/` (원본은 커밋하지 않음) |

## 하위 작업 공간

두 폴더는 원래 독립 저장소였고 자기 규칙이 있습니다. 그 안에서 일할 때는 그 폴더의 문서를 먼저 읽습니다.

- `hardware/vehicle/` — **[hardware/vehicle/CLAUDE.md](hardware/vehicle/CLAUDE.md)를 따릅니다.** 명령은 이
  폴더에서 실행합니다(스크립트가 자기 파일 위치 기준으로 경로를 잡음). 안쪽 폴더 구조를 바꾸지 않고,
  기록을 마친 버전 폴더는 고치지 않습니다.
- `firmware/wheel-controller/` — README · ISSUES와
  [링크 디버깅 기록](docs/worklog/topics/wheel-link-debugging.md)을 먼저 읽습니다. 명령은 이 폴더에서
  `pio.cmd`로 실행합니다.

## 규칙

- 문서는 한국어로 씁니다. 코드 식별자, 명령, 커밋 접두어는 영어입니다.
- 작업을 마치면 `docs/worklog/YYYY-MM.md`에 그날 항목을 추가하고 그 달 목차 표와 README 연표에 한 줄씩
  더합니다. 양식은 [docs/worklog/README.md](docs/worklog/README.md)를 따릅니다(제목 `날짜 (요일) · 영역: 제목`,
  인용 줄, 정해진 본문 칸). 일지와 시간순 기록은 여기 한 곳에만 쓰고, 여러 날에 걸친 주제 정리는
  `docs/worklog/topics/`에 씁니다. 옮겨 온 항목의 본문은 고치지 않습니다.
- 결정 기록: 차체 설계는 `hardware/vehicle/models/vehicle/DECISIONS.md`에 D-NN, 여러 계층에 걸친
  것은 `docs/decisions/`에 ADR.
- 바퀴 제어기의 열린 문제는 `firmware/wheel-controller/ISSUES.md`에서 관리합니다. 고치면 상태 칸과
  그날 일지를 같이 갱신합니다.
- 부품은 국내 구매처만 씁니다 (D-08).
- 좌표계는 REP-103 (x 전진, y 왼쪽, z 위, 지면 z = 0). 기구는 mm, 소프트웨어는 SI 단위.
- 바퀴 제어기 명령이나 프레임 형식을 바꾸면 `system-overview.md`도 같이 고칩니다. 차체 설계에서
  ESP32가 맡는 일을 정하면 `system-overview.md`의 "차체 설계가 바퀴 제어기에 요구하는 것" 표를
  갱신합니다.
- 이 저장소 밖의 파일(`F:\Develop\`의 다른 폴더, Jetson · RDK 보드)을 옮기거나 지울 때는 먼저
  사용자에게 확인합니다.
- 호버보드 메인보드 펌웨어(GPL-3.0 업스트림을 고친 것)와 그 빌드 · 굽기 기록은 `firmware/hoverboard-mainboard/`에
  로컬로만 둡니다(git 제외). 공개 문서에는 바퀴 제어기와 맞닿는 인터페이스만 적습니다.
- 이 저장소는 아직 비공개입니다. 원격을 붙이거나 푸시하기 전에
  [docs/guides/before-publishing.md](docs/guides/before-publishing.md)를 끝까지 따릅니다(라이선스, 이력 정리).
- `F:\Develop\freecadAI`의 av-platform CAD는 쓰지 않기로 했습니다. 참고하거나 가져오지 않습니다.
