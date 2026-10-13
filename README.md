# build123d 작업 저장소

Python 코드 ([build123d](https://build123d.readthedocs.io)) 로 3D 모델을 만드는 작업 모음. 주 프로젝트는
**호버보드 허브모터 4 륜 자율주행 차량 (모빌리티 베이스)** 이고, 그 밖의 소형 기구 · 로봇팔 프로젝트가 함께 들어 있다.

- 개발 환경 만들기: [`SETUP.md`](SETUP.md) (Python 3.12 + build123d 0.11.1 + ocp_vscode 4.0.0), 패키지는 [`requirements.txt`](requirements.txt)
- 작업 규칙 (Claude Code 용): [`CLAUDE.md`](CLAUDE.md)
- 뷰어: `.venv\Scripts\python.exe -m ocp_vscode` → http://127.0.0.1:3939

## 차량 (모빌리티 베이스) — 어디에 무엇이 있나

| 위치 | 내용 |
|---|---|
| [`models/vehicle/`](models/vehicle/README.md) | **차량 재설계 v00 ~ v29** — 버전 = 폴더 (동결 코드 + 이미지 + README), `record.py` 한 명령으로 짓기 · 검증 · STEP · 이미지 · 기록 |
| [`models/vehicle/DECISIONS.md`](models/vehicle/DECISIONS.md) | 결정 기록 D-01 ~ D-21 — 무엇을 왜 골랐나 · 버린 대안 · 확인할 것 |
| [`models/vehicle/research/`](models/vehicle/research/) | 부품 조사 원자료 (국내 구매처 · 가격 · 링크, 비슷한 차량 사례, 보드 · 배터리 · 센서 · 배선) |
| [`models/vehicle/sim/`](models/vehicle/sim/README.md) | 웹 시뮬레이터 (three.js) — 주행 · 센서 · 전원 · 사각지대 · 전복 · 비상정지 · 자율주행, 복도 환경 |
| [`models/hoverboard_mount/`](models/hoverboard_mount/) | 재설계 전 단계 — 휠 마운트 · 하부 클램프 (Design D), **구조해석 FEA** ([`FEA.md`](models/hoverboard_mount/FEA.md) · [`CLAUDE_HANDOFF.md`](models/hoverboard_mount/CLAUDE_HANDOFF.md)), 제작용 STEP · STL · DXF |
| [`docs/build-log.md`](docs/build-log.md) · [`docs/journal/`](docs/journal/) | 설계 기록 · 작업일지 (2026-07 ~ 09) |
| [`docs/payload-interface.md`](docs/payload-interface.md) | 위에 올리는 모듈 규격 |
| [`docs/bom.md`](docs/bom.md) · `docs/floor_plate_cut*.md` | 재설계 전 BOM · 바닥판 재단 (지금 BOM 은 각 버전 폴더의 `bom.md`) |
| [`tools/fea/`](tools/fea/) · [`tools/frame/`](tools/frame/) | FEA 파이프라인 (Gmsh → CalculiX), 프레임 해석 |
| `models/swerve_base.py` · `mobility_base.py` · `wheel_mount.py` · `wheeled_base.py` · `tslot*.py` | 재설계 전 초기 스크립트 |

지금 상태 (2026-10): **v28_wiring** 까지 검증 · 기록 완료 (내부 배선 90 가닥, BOM 단가 확인된 합 약 264 만 원).
**v29_floor_wiring** (배선을 바닥판 위 직각 경로로) 는 진행 중 — 경로는 다 찾았고 겹침 검증이 남았다.

## 그 밖의 프로젝트 (`models/`)

`card_press` · `card_stamper` (명함) · `esp32_s31_case` · `galaxy_watch4_classic` · `lora_messenger` (LoRa 무전기) ·
`manuel_1_0` · `manuel_dm` · `manuel_mini` · `manuel_mini2` · `mycobot_280_pi` · `robot_arm_28byj48(_v2)` (로봇팔 · 웹 시뮬) ·
`mini_pager(_1602)` (삐삐) · `mo_robot` (M-O) · `nfc_turntable(_mini)` · `phone_laptop` · `phone_tablet` ·
`pillbox*` (필박스) · `pocketlab` · `pouch_dispenser` · `power_button`, 그리고 `models/*.py` 단일 파일 모델들.
각 폴더의 `README.md` 에 설명이 있다.

## 저장소에 안 넣은 것 (`.gitignore`)

다시 만들 수 있거나, 크거나, 남이 만든 것은 뺐다.

| 빠진 것 | 다시 만드는 법 |
|---|---|
| `.venv/` | `SETUP.md` 대로 새로 만든다 |
| `exports/` (생성 형상 2.7 GB) | 해당 `models/*.py` 실행 |
| 차량 버전 `output/scene.npz` · `output/<버전>.step` | `.venv/Scripts/python.exe models/vehicle/record.py <버전> --rebuild` (v28_wiring STEP 만 공유용으로 담았다) |
| 웹 시뮬레이터 메시 (`models/*/web/data*/`) | 각 프로젝트의 `web_export.py` |
| 차량 시뮬 복도 맵 (`models/vehicle/sim/web/hallway/`) | `models/vehicle/sim/import_hallway.py` (원본: 다른 프로젝트 hallway-3d) |
| `refmodel/` (남의 참조 STEP 1.4 GB) | 출처에서 받는다. **차량에 필요한 호버보드 휠 모터 STEP 하나만 담았다** |
| FEA 중간 파일 (`*.inp` · `*.frd` · `*.vtu` …), 외부 툴 바이너리 (`tools/fea/calculix` 등) | `tools/fea/run_clamp_fea.ps1` · 설치 안내는 `models/hoverboard_mount/FEA.md` |
| `cadview/` | 별도 git 저장소 |
| 로그 (`*.log`), 개인 설정 (`.claude/settings.local.json`) | — |
