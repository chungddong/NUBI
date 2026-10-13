# HoverboardVehicle — 호버보드 허브모터 4 륜 자율주행 차량

> 누비의 차체 설계 작업 공간입니다. 2026-10-13에 개인 저장소였던 `HoverboardVehicle`을
> 커밋 이력째 이 폴더로 옮겼습니다. 스크립트가 자기 파일 위치를 기준으로 경로를 잡기 때문에
> 안쪽 구조는 그대로 두었고, 아래 명령은 모두 **이 폴더(`hardware/vehicle/`)에서** 실행합니다.
> 이후 작업 일지는 저장소 공통 [docs/worklog/](../../docs/worklog/)에 씁니다. 시스템 전체에서의 위치는
> [system-overview.md](../../docs/architecture/system-overview.md)에 있습니다.

호버보드 6.5" 허브모터 4 개로 굴러가는 스키드 스티어 모빌리티 베이스 (실외 · 자율주행) 를
Python 코드 ([build123d](https://build123d.readthedocs.io)) 로 설계한 기록 전부.
처음엔 `D:\build123d` 작업 저장소 안에 있었고, 2026-10-13 차량 부분만 커밋 기록째로 떼어 이 저장소로 옮겼다.

- 개발 환경: [`SETUP.md`](SETUP.md) (Python 3.12 + build123d 0.11.1 + ocp_vscode 4.0.0), 패키지 [`requirements.txt`](requirements.txt)
- 작업 규칙 (Claude Code 용): [`CLAUDE.md`](CLAUDE.md)
- 뷰어: `.venv\Scripts\python.exe -m ocp_vscode` → http://127.0.0.1:3939

## 어디에 무엇이 있나

| 위치 | 내용 |
|---|---|
| [`models/vehicle/`](models/vehicle/README.md) | **차량 재설계 v00 ~ v29** — 버전 = 폴더 (동결 코드 + 이미지 + README), `record.py` 한 명령으로 짓기 · 검증 · STEP · 이미지 · 기록 |
| [`models/vehicle/DECISIONS.md`](models/vehicle/DECISIONS.md) | 결정 기록 D-01 ~ D-21 — 무엇을 왜 골랐나 · 버린 대안 · 확인할 것 |
| [`models/vehicle/research/`](models/vehicle/research/) | 부품 조사 원자료 (국내 구매처 · 가격 · 링크, 비슷한 차량 사례, 보드 · 배터리 · 센서 · 배선) |
| [`models/vehicle/sim/`](models/vehicle/sim/README.md) | 웹 시뮬레이터 (three.js) — 주행 · 센서 · 전원 · 사각지대 · 전복 · 비상정지 · 자율주행, 복도 환경 |
| [`models/hoverboard_mount/`](models/hoverboard_mount/) | 재설계 전 단계 — 휠 마운트 · 하부 클램프 (Design D), **구조해석 FEA** ([`FEA.md`](models/hoverboard_mount/FEA.md) · [`CLAUDE_HANDOFF.md`](models/hoverboard_mount/CLAUDE_HANDOFF.md)), 제작용 STEP · STL · DXF |
| `models/*.py` | 재설계 전 초기 스크립트 — `swerve_base` (4 륜 베이스) · `mobility_base` · `wheel_mount` · `wheeled_base` · `motor_stand` · `wheel_jig` · `tslot` (프로파일) · `drawings` · `export_cnc`, 렌더 도구 `iso_render` · `render` |
| [`docs/worklog/`](../../docs/worklog/README.md) (저장소 공통) | 설계 일지 · 설계 이야기 · 정리본 (2026-07 ~). 2026-10-13 HTML 일지와 `build-log.md`를 Markdown으로 옮겨 합쳤다 |
| [`docs/payload-interface.md`](docs/payload-interface.md) | 위에 올리는 모듈 규격 |
| [`docs/interactive/`](docs/interactive/README.md) | 설명 페이지 (바퀴가 바깥으로 밀리는 이유 — 지렛대) |
| [`docs/bom.md`](docs/bom.md) · `docs/floor_plate_cut*.md` | 재설계 전 BOM · 바닥판 재단 (지금 BOM 은 각 버전 폴더의 `bom.md`) |
| [`tools/fea/`](tools/fea/) · [`tools/frame/`](tools/frame/) · `tools/bom/` · `tools/figures/` | FEA 파이프라인 (Gmsh → CalculiX), 프레임 해석, BOM 집계, 전복 모멘트 그림 |
| `refmodel/` | 호버보드 6.5" 휠 모터 STEP (차를 지을 때 필요) |

## 지금 상태 (2026-10)

- **v28_wiring** 까지 검증 · 기록 완료 — 내부 배선 90 가닥, BOM 단가 확인된 합 약 264 만 원 (미확인 38 줄).
- **v29_floor_wiring** (배선을 바닥판 위 직각 경로로) 진행 중 — 경로는 다 찾았고, 판 닿음 2 · 선끼리 겹침 69 를 고쳐야 한다 (D-21).
- 결정 대기 (D-19): 옆 낮은 장애물 센서 · 비상정지 '정지 유지 + 상선 단락' · 12 V 변환기 키우기.

## 저장소에 안 넣은 것 (`.gitignore`)

| 빠진 것 | 다시 만드는 법 |
|---|---|
| `.venv/` | `SETUP.md` 대로 새로 만든다 |
| `exports/` | 해당 `models/*.py` 실행 |
| 차량 버전 `output/scene.npz` · `output/<버전>.step` | `.venv/Scripts/python.exe models/vehicle/record.py <버전> --rebuild` (v28_wiring STEP 만 공유용으로 담았다) |
| 차량 시뮬 웹 데이터 · 복도 맵 | `models/vehicle/sim/web_export.py <버전>` · `import_hallway.py` (원본: `D:\Develop\ClaudeGame\hallway-3d`) |
| FEA 중간 파일 · 외부 툴 (`tools/fea/calculix` · `gmsh_pyenv` · `python`) | 설치 안내는 `models/hoverboard_mount/FEA.md` |
| 다른 참조 STEP (`refmodel/` 의 모터 말고) | 출처에서 받는다 (`wheeled_base.py` 는 Cramped Robot Wheel STEP 이 있어야 돈다) |
| 로그, 개인 설정 | — |
