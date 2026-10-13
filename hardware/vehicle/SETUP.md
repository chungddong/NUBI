# HoverboardVehicle 개발 환경 세팅 (build123d + Claude Code)

다른 컴퓨터에서 이 개발 환경을 그대로 만드는 방법. (`.venv` 폴더는 절대 복사하지 말 것 — 경로가 박혀 있어 깨진다.)

## 조합

- Python 3.12
- build123d 0.11.1  (코드로 3D 모델링)
- ocp_vscode 4.0.0  (독립 웹 뷰어, 포트 3939)
- numpy · scipy · matplotlib · pillow · vtk · ezdxf (기록 도구 · 배선 라우터 · 도면) — `requirements.txt`
- VS Code + Claude Code

## 설치 순서 (Windows / PowerShell)

```powershell
# 1. Python 3.12 설치 — python.org 에서. 이미 있으면 생략.
#    설치 시 "Add Python to PATH" 체크.

# 2. 저장소를 받아 그 안으로 이동
git clone <저장소 주소> HoverboardVehicle; cd HoverboardVehicle

# 3. 가상환경 생성
py -3.12 -m venv .venv

# 4. 패키지 설치 (버전 고정)
.venv\Scripts\python.exe -m pip install -r requirements.txt

# 5. 설치 확인
.venv\Scripts\python.exe -c "from build123d import *; from ocp_vscode import show; print('OK')"
```

`OK` 가 뜨면 설치 성공. 저장소 지도는 `README.md`.

## 사용법

```powershell
# 터미널 1 — 뷰어 켜두기 (계속 유지)
.venv\Scripts\python.exe -m ocp_vscode
#   → 브라우저에서 http://127.0.0.1:3939 열기

# 터미널 2 — 차량 버전 기록 (짓기 · 검증 · STEP · 뷰어 · 이미지). 메시 캐시가 없으면 다시 짓는다.
.venv\Scripts\python.exe models\vehicle\record.py v28_wiring --rebuild

# 웹 시뮬레이터 — 버전 메시를 웹으로 옮기고 서버를 띄운다 → http://localhost:8748
.venv\Scripts\python.exe models\vehicle\sim\web_export.py v28_wiring
.venv\Scripts\python.exe models\vehicle\sim\web\serve.py 8748
```

구조해석 (FEA) 은 외부 툴 (CalculiX · Gmsh) 이 따로 필요하다 — `models/hoverboard_mount/FEA.md`.

## Claude Code 연동

VS Code + Claude Code 를 설치하고 이 폴더를 열면 `CLAUDE.md` 가 자동으로
로드된다. 차량 작업 규칙 (버전 폴더 · record.py · 결정 기록 · 국내 구매처) 이 거기에 있다.
