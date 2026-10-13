# HoverboardVehicle — 호버보드 허브모터 4 륜 자율주행 차량

Python 코드로 파라메트릭 3D 모델을 만든다. CAD 라이브러리는
[build123d](https://build123d.readthedocs.io), 시각화는 ocp_vscode 뷰어. 저장소 지도는 [`README.md`](README.md).

## 환경

- Python 가상환경: `.venv` (Windows). **`.venv\Scripts\python.exe` 를 그대로 쓴다.** 만드는 법은 `SETUP.md`.
- 의존성: `requirements.txt`. 재설치는 `.venv\Scripts\python.exe -m pip install -r requirements.txt`
- 답변 · 기록은 한국어로, 쉬운 말로.

## 차량 재설계 (models/vehicle) — 주 작업

- **버전 = 폴더** `models/vehicle/vNN_<이름>`. 규칙 전문은 [`models/vehicle/README.md`](models/vehicle/README.md).
  - 새 버전: `.venv/Scripts/python.exe models/vehicle/record.py new vNN_이름 "제목"` → 새 폴더 `assembly.py` 수정 →
    `record.py vNN_이름` 한 번 (짓기 · 검증 · STEP · 뷰어 · 이미지 · README 자동 구역 · 버전 표)
  - 기록을 마친 버전은 고치지 않는다. 부품 label 은 버전 사이에 같은 부품이면 같은 이름.
  - 결정마다 `models/vehicle/DECISIONS.md` 에 D-NN (결정 · 이유 · 버린 대안 · 확인할 것), 조사 원자료는 `models/vehicle/research/날짜_주제.md`.
- 부품은 **국내 구매처만** (해외직구 · 구매대행 안 됨), 가격은 조사로 확인한 것만 BOM 에 적는다. IP 방수 등급은 기준에서 뺐다.
- 검증에는 대조군을 둔다 (간섭 0 이 '안 닿음' 인지 '잘 맞음' 인지 가르게). 겹침은 조각 (Solid) 하나씩 잰다 — 겹친 원기둥 · 구 묶음을 통째로 불리언하면 OCC 가 빈 결과를 낼 때가 있다.
- 시뮬레이터: `models/vehicle/sim/` (웹 8748, `.claude/launch.json` 의 `vehicle-sim`). 새 버전을 기록하면 `web_export.py <버전>` 후 새로고침.
- 뷰어는 PowerShell `Start-Process` 로 떼어 띄우고 (`-m ocp_vscode --port 3939`), 브라우저 탭을 연 **뒤에** 보낸다.
  배선이 많은 버전 (v29) 은 `inside.py --noimg --show [--full]` 로 가볍게 보낸다 (안 그러면 브라우저가 멈춘다).

## build123d 작성 규칙

- 빌더 API 를 기본으로 쓴다: `with BuildPart() as p:` / `BuildSketch` / `BuildLine`. 한 파일 안에서는 한 방식으로 통일.
- 치수는 파일 상단에 상수/파라미터로 모아 두고 형상 코드에서 참조한다. 단위는 밀리미터(mm).
- 코드를 고친 뒤에는 스크립트를 실제로 실행해 예외 없이 끝나는지 확인하고, 렌더 이미지를 직접 열어 본다.

## 호버보드 클램프 구조해석 (FEA)

Design-D 하부 클램프에는 실제 오픈소스 유한요소해석 파이프라인이 있다. 관련 작업 전에는 반드시
[`models/hoverboard_mount/FEA.md`](models/hoverboard_mount/FEA.md) 와
[`models/hoverboard_mount/CLAUDE_HANDOFF.md`](models/hoverboard_mount/CLAUDE_HANDOFF.md) 를 먼저 읽는다.

- 진입점: `models/hoverboard_mount/run_clamp_fea.py`
- 현재형 실행 (편심 · 방향을 반드시 준다 — 빼면 편심 0 케이스가 돌아간다):
  `powershell -ExecutionPolicy Bypass -File tools/fea/run_clamp_fea.ps1 -Model design_d -Material al6061 -Load 800 -MeshSize 3 -Eccentricity 55 -LoadDirection upward`
- 흐름: STEP → Gmsh C3D10 메쉬 → CalculiX 정적해석 → VTU/PNG 후처리. 외부 툴 (`tools/fea/calculix` 등) 은 git 밖.
- **기준 결과 (2026-08-21, 현재 형상)**: 800 N · 편심 55 · 상향 · 3 mm 메쉬에서 **구조응력 45.3 MPa / 변위 0.0582 mm** (AL6061-T6 안전율 6.1).
  같은 조건 PETG 근사는 45.6 MPa / 1.907 mm — 출력품은 800 N 에서 재료 한계다.
- **최대 von Mises (150.1 MPa) 는 인용하지 않는다.** 강체 고정한 볼트 구멍 벽의 특이점이다. 구조응력은 `tools/fea/structural_stress.py`.
- 결과만 보고 차량의 허용하중이나 안전 인증을 단정하지 않는다 (볼트 프리로드 · 접촉 · T 너트 변형 · 피로 없음).
  설계 변경 후에는 최소 두 메쉬 크기로 수렴성을 비교하고, 반력 합이 입력 하중과 일치하는지 확인한다.
