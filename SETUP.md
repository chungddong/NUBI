# build123d + Claude Code 3D 모델링 세팅 방법

다른 컴퓨터에서 이 개발 환경을 그대로 만드는 방법. 이 파일과 `CLAUDE.md`만
있으면 된다. (`.venv` 폴더는 절대 복사하지 말 것 — 경로가 박혀 있어 깨진다.)

## 조합

- Python 3.12
- build123d 0.11.1  (코드로 3D 모델링)
- ocp_vscode 4.0.0  (독립 웹 뷰어, 포트 3939)
- VS Code + Claude Code

## 설치 순서 (Windows / PowerShell)

```powershell
# 1. Python 3.12 설치 — python.org 에서. 이미 있으면 생략.
#    설치 시 "Add Python to PATH" 체크.

# 2. 프로젝트 폴더를 만들고 그 안으로 이동
mkdir build123d; cd build123d

# 3. 가상환경 생성
py -m venv .venv

# 4. 패키지 설치 (버전 고정)
.venv\Scripts\python.exe -m pip install build123d==0.11.1 ocp_vscode==4.0.0

# 5. 설치 확인
.venv\Scripts\python.exe -c "from build123d import *; from ocp_vscode import show; print('OK')"
```

`OK` 가 뜨면 설치 성공.

## 폴더 구조 (직접 만들거나 옮겨오기)

```
build123d/
├─ .venv/          가상환경 (위 3~4단계로 새로 생성, 복사 금지)
├─ models/         모델 스크립트(.py) 를 여기에
├─ exports/        STEP/STL 출력물 (자동 생성)
├─ CLAUDE.md       Claude Code용 프로젝트 규칙  ← 꼭 같이 옮길 것
└─ SETUP.md        이 파일
```

## 사용법

```powershell
# 터미널 1 — 뷰어 켜두기 (계속 유지)
.venv\Scripts\python.exe -m ocp_vscode
#   → 브라우저에서 http://127.0.0.1:3939 열기

# 터미널 2 — 모델 실행
.venv\Scripts\python.exe models\<파일>.py
#   → 뷰어에 형상이 뜨고 exports\ 에 파일 저장
```

## Claude Code 연동

VS Code + Claude Code 를 설치하고 이 폴더를 열면 `CLAUDE.md` 가 자동으로
로드된다. 그 뒤 "지름 30, 높이 50 컵 만들어줘" 처럼 말하면 규칙대로
`models/` 에 스크립트를 짜고 실행·검증까지 해 준다.
