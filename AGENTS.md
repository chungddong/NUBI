# 누비 (NUBI) — 에이전트 작업 규칙

모듈형 자율주행 로봇 누비의 통합 저장소입니다. 구조와 규칙의 기준은 [README.md](README.md)이고,
시스템 구성은 [docs/architecture/system-overview.md](docs/architecture/system-overview.md)에 있습니다.

## 어디에 두나

| 무엇 | 위치 |
| --- | --- |
| 여러 계층에 걸친 문서 | `docs/` (구성은 `docs/README.md`) |
| 한 부품에만 해당하는 문서 | 그 부품 폴더 (예: `firmware/wheel-controller/README.md`) |
| CAD, 빌더 스크립트, 벤더 STEP | `hardware/mechanical/` |
| 배선, 핀맵, BOM | `hardware/electrical/`, `hardware/bom/` |
| MCU 펌웨어 | `firmware/<보드>/` |
| 상위 컴퓨터 코드 (ROS 2) | `software/ros2_ws/src/nubi_*` |
| 아직 자리 잡지 않은 시험 코드 | `experiments/YYYY-MM-DD_<주제>/` |
| 센서 원본, 로그 | `data/` (원본은 커밋하지 않음) |

## 규칙

- 문서는 한국어로 씁니다. 코드 식별자, 명령, 커밋 접두어는 영어입니다.
- 작업을 마치면 `docs/worklog/YYYY-MM.md`에 그날 섹션을 추가합니다: 무엇을 왜 했고, 무엇을
  확인했고(수치와 조건), 무엇이 남았는지.
- 설계 결정은 `docs/decisions/`에 ADR로 남깁니다.
- 좌표계는 REP-103 (x 전진, y 왼쪽, z 위). 기구는 mm, 소프트웨어는 SI 단위.
- 바퀴 제어기 명령이나 프레임 형식을 바꾸면 `system-overview.md`도 같이 고칩니다.
- 이 저장소 밖의 파일(`F:\Develop\`의 다른 폴더, Jetson · RDK 보드)을 옮기거나 지울 때는 먼저
  사용자에게 확인합니다.
- `F:\Develop\freecadAI`의 av-platform CAD는 쓰지 않기로 했습니다. 참고하거나 가져오지 않습니다.
