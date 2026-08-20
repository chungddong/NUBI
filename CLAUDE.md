# build123d 3D 모델링 프로젝트

Python 코드로 파라메트릭 3D 모델을 만드는 프로젝트다. CAD 라이브러리는
[build123d](https://build123d.readthedocs.io), 시각화는 ocp_vscode 뷰어를 쓴다.

## 환경

- Python 가상환경: `.venv` (Windows). **`.venv\Scripts\python.exe` 를 그대로 쓴다.**
  2026-08-21 확인: Python 3.12.10 / build123d 0.11.1 정상 동작.
  (예전에 원본 Python 제거로 깨진 적이 있어 우회 안내가 남아 있었다. 지금은
  멀쩡하니 우회부터 타지 마라. 실제로 깨졌을 때만 `CLAUDE_HANDOFF.md` §2 참조.)
- 의존성: `requirements.txt` (build123d, ocp_vscode). 재설치는
  `.venv\Scripts\python.exe -m pip install -r requirements.txt`
- 모델 스크립트는 `models/` 에, 내보낸 STEP/STL 등은 `exports/` 에 둔다.

## 실행 워크플로

1. 뷰어를 먼저 띄운다 (별도 터미널 유지):
   `.venv\Scripts\python.exe -m ocp_vscode`  → http://127.0.0.1:3939
   (또는 VS Code에서 OCP CAD Viewer 확장 실행)
2. 모델 실행: `.venv\Scripts\python.exe models\<파일>.py`
   → `show(...)` 가 뷰어로 형상을 보내고, `export_*` 가 파일로 저장한다.

## build123d 작성 규칙

- 빌더 API를 기본으로 쓴다: `with BuildPart() as p:` / `BuildSketch` / `BuildLine`.
  (필요하면 알제브라 API도 가능하지만 한 파일 안에서는 한 방식으로 통일한다.)
- 치수는 파일 상단에 상수/파라미터로 모아 두고 형상 코드에서 참조한다.
- 단위는 밀리미터(mm) 기준.
- 형상을 만든 뒤 반드시 `show(...)` 로 뷰어에 보내 확인하고, 최종본은
  `export_step` / `export_stl` 로 `exports/` 에 저장한다.
- 새 모델은 `models/example_box.py` 구조를 참고해 만든다.

## 검증

코드를 고친 뒤에는 스크립트를 실제로 실행해 예외 없이 끝나고 `exports/` 에
파일이 생성되는지 확인한다. 뷰어가 안 떠 있어도 export는 되어야 한다.

## 호버보드 클램프 구조해석(FEA)

Design-D 하부 클램프에는 OCP 뷰어의 단순 하중 화살표와 별개로 실제 오픈소스
유한요소해석 파이프라인이 있다. 관련 작업 전에는 반드시
[`models/hoverboard_mount/FEA.md`](models/hoverboard_mount/FEA.md)를 먼저 읽는다.
현재 채택 형상, 차량 프레임, 출력 파일, 뷰어 및 실물 파손 비교까지 포함한 전체
인수인계는
[`models/hoverboard_mount/CLAUDE_HANDOFF.md`](models/hoverboard_mount/CLAUDE_HANDOFF.md)에
정리되어 있다. Claude가 이 작업을 이어갈 때는 두 문서를 모두 읽는다.

- 진입점: `models/hoverboard_mount/run_clamp_fea.py`
- 현재형 실행 (편심·방향을 반드시 준다 — 빼면 편심 0 케이스가 돌아간다):
  `powershell -ExecutionPolicy Bypass -File tools/fea/run_clamp_fea.ps1 -Model design_d -Material al6061 -Load 800 -MeshSize 3 -Eccentricity 55 -LoadDirection upward`
- 구형 긴 ㄷ자 실행: 위 명령에서 `-Model legacy_u` 사용
- 흐름: STEP → Gmsh C3D10 메쉬 → CalculiX 정적해석 → VTU/PNG 후처리
- **기준 결과(2026-08-21, 현재 형상)**: 800 N · 편심 55 · 상향 · 3 mm 메쉬에서
  **구조응력 45.3 MPa / 변위 0.0582 mm** (AL6061-T6 안전율 6.1).
  같은 조건 PETG 근사는 45.6 MPa / 1.907 mm — 출력품은 800 N 에서 재료 한계다.
- **최대 von Mises(150.1 MPa)는 인용하지 않는다.** 강체 고정한 볼트 구멍 벽의
  특이점이라 메쉬를 조일수록 오른다. 구조응력은 `tools/fea/structural_stress.py`
- 옛 기록(편심 0·하향, 플랜지 있던 63.9mm 형상): 4.036 MPa / 0.001400 mm,
  구형 긴 ㄷ자 15.816 MPa / 0.009374 mm. 조건이 다르니 섞어 쓰지 않는다
- 실제 파손 재현: `legacy_u_nogusset`, PETG, 800 N upward, 편심 55 mm. 열린 면은
  차량 내부. 3 mm 결과 44.373 MPa / 2.979 mm이며 아래 볼트열부터 바깥으로 휜다.
- 17.677 MPa / 0.007293 mm 는 **옆 플랜지가 있던 형상**의 값이다 (구멍 6개 고정).
  플랜지 제거로 변위가 8.0배 커졌다. 되살리려면 `parameters.LOW_SIDE = True`.
- 결과 폴더: `models/hoverboard_mount/output/fea/clamp_<하중>N_h<메쉬>mm/`

현재 모델은 하부 클램프 단품에서 레일 체결 구멍 벽을 강체 고정한 비교 모델이다.
볼트 프리로드, 접촉, T너트/프로파일 변형, 피로가 없으므로 결과만 보고 차량의
허용하중이나 안전 인증을 단정하지 않는다. 설계 변경 후에는 최소 두 메쉬 크기로
수렴성을 비교하고, 반력 합이 입력 하중과 일치하는지 확인한다.
