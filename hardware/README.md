# hardware — 기구 · 전장 설계

```
hardware/
├── vehicle/        차체 설계 작업 공간 (구 HoverboardVehicle 저장소)
└── electrical/     차체 모델 밖의 전장 문서 (보드별 핀맵, 회로도)
```

## vehicle/ — 차체 설계

호버보드 6.5" 허브모터 4개로 움직이는 스키드 스티어 차체를 Python(build123d) 코드로 설계한 작업 공간입니다.
2026-07-27 첫 설계부터의 기록이 모두 들어 있습니다. 시작점은 [vehicle/README.md](vehicle/README.md)입니다.

| 위치 | 내용 |
| --- | --- |
| [vehicle/models/vehicle/](vehicle/models/vehicle/README.md) | 차체 재설계 v00 ~ v29. 버전마다 폴더 하나 (코드 · 이미지 · BOM · README) |
| [vehicle/models/vehicle/DECISIONS.md](vehicle/models/vehicle/DECISIONS.md) | 차체 설계 결정 D-01 ~ D-21 (센서, 모터 드라이버, 배터리, 비상정지, 배선 …) |
| [vehicle/models/vehicle/research/](vehicle/models/vehicle/research/) | 부품 조사 원자료 (국내 구매처 · 가격) |
| [vehicle/models/vehicle/sim/](vehicle/models/vehicle/sim/README.md) | 웹 시뮬레이터 (three.js) — 주행, 센서 시야, 전원, 사각지대, 전복, 비상정지, 자율주행, 복도 |
| [vehicle/models/hoverboard_mount/](vehicle/models/hoverboard_mount/) | 재설계 전 단계: 휠 마운트, 하부 클램프 Design D, 구조해석(FEA), 제작용 STEP · STL · DXF |
| [vehicle/docs/](vehicle/docs/) | 재설계 전 BOM · 바닥판 재단, 페이로드 규격, 일지 그림, 인터랙티브 설명 페이지. 설계 일지는 [docs/worklog/](../docs/worklog/README.md)로 옮김 |

**규칙은 [vehicle/CLAUDE.md](vehicle/CLAUDE.md)를 따릅니다.** 요점만 옮기면:

- 명령은 `hardware/vehicle/`에서 실행합니다. 스크립트가 자기 파일 위치를 기준으로 경로를 잡으므로
  안쪽 폴더 구조를 바꾸지 않습니다.
- 버전 = 폴더. 기록을 마친 버전은 고치지 않고 다음 버전에서 고칩니다.
- 차체 설계 결정은 `DECISIONS.md`에 D-NN으로 남깁니다.
- 부품은 국내 구매처만 씁니다 (D-08).
- 좌표: 지면 z = 0, +x 전진, +y 왼쪽, +z 위, 단위 mm, 차량 중심 x = y = 0.

개발 환경(Python 3.12 + build123d)은 [vehicle/SETUP.md](vehicle/SETUP.md)대로 `hardware/vehicle/.venv`에 만듭니다.

## electrical/

차체 모델에 이미 들어간 배선(v28 · v29)과 BOM(버전 폴더의 `bom.md`)은 vehicle/에 있습니다. 여기에는 그 밖의
전장 문서를 둡니다.

- **핀맵** — 보드별로 `<보드>-pinout.md`. 펌웨어 코드의 핀 상수와 어긋나면 안 되므로, 핀을 바꾸면 펌웨어
  쪽과 같은 커밋에서 고칩니다.
- **회로도** — 비상정지 루프, 전원 분기 등 3D 모델로는 보기 어려운 것.
