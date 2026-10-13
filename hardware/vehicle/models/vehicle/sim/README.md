# 차량 웹 시뮬레이터

버전 폴더의 메시 캐시 (`vNN_*/output/scene.npz`) 를 웹으로 옮겨 three.js 로 운전 · 센서 · 전원을 본다.
왜 만들었는지는 [`../DECISIONS.md`](../DECISIONS.md) D-18, 시뮬레이터로 본 결과는 D-19.

```bash
.venv/Scripts/python.exe models/vehicle/sim/web_export.py v27_sensor_module
```

```bash
.venv/Scripts/python.exe models/vehicle/sim/web/serve.py 8748
```

브라우저 → http://localhost:8748 (launch.json `vehicle-sim`). 버전을 바꾸면 web_export 를 다시 돌리고 새로고침.

### 환경 — 시험 코스 / 복도

패널 '환경 · 출발점' 에서 고른다 (`?env=hallway`). **복도**는 영상으로 복원한 실제 건물 3 층 복도 · 라운지
(`D:\Develop\ClaudeGame\hallway-3d`, 벽 26 · 오브젝트 153, 사진 텍스처). 원본은 읽기만 하고 아래로 복사해 쓴다:

```bash
.venv/Scripts/python.exe models/vehicle/sim/import_hallway.py
```

→ `web/hallway/` (형상 코드 world/ · scene.json · 텍스처, gitignore). 원본 `buildArchitecture` · `buildObjects` 를 그대로 부르고
x 축 +90° 로 돌려 z 를 위로 (우리 (x, y, z) = 원본 (x, −z, y)). 충돌은 원본 충돌선 (벽 · 난간) + 고정 가구 상자 + 사람 2.
복도를 고치면 import_hallway 를 다시 돌린다.
버전 인자를 빼면 메시 캐시가 있는 마지막 버전.

## 파일

| 파일 | 하는 일 |
|---|---|
| `web_export.py` | scene.npz → `web/data/vehicle.bin` (부품별 삼각형, m) + `vehicle.json` (부품 · 질량 어림 · 바퀴 · 카메라 · 전조등 · 라이다 · 표시등 · 전원 값) |
| `web/app.js` | 주행 (스키드 스티어, 가속 한계 1 m/s²), 지형 따라 자세 (네 바퀴 높이), 충돌 · 턱 (바퀴 반지름 40 %), 패널 |
| `web/course.js` | 시험 코스 — 상자 · 기둥 · 0.8 m 통로 · 턱 20 / 40 mm · 경사 10° / 15° · 울타리 (0.5 m) |
| `web/sensors.js` | 뎁스 카메라 시야 (90° × 65°, 이상 0.3–3 m) · 깊이 점 (44 × 30 광선) · 앞 카메라 화면 · 라이다 (500 광선) · 전조등 · 표시등 |
| `web/power.js` | 전원 추정 — DECISIONS D-04 식 (바퀴 힘 · 미끄럼 회전 · 상전류 · 구리 손실 · 전자 30 W, 정지 유지 전류) |
| `web/coverage.js` | ① 사각지대 지도 — 차 둘레 6 × 6 m (5 cm 칸), 장애물 높이별 카메라 · 라이다가 보는 곳, 방향별 처음 보이는 거리 |
| `web/stability.js` | ② 전복 · 무게중심 — 지지 사각형 · 전복각 (옆 · 앞 · 뒤) · 미끄럼각, 추가 모듈 무게 · 높이, 회전 원심 |
| `web/nav.js` | ④ 자율주행 — 점유 격자 (라이다 층 · 깊이 층) → A* → 순수 추종 + 0.8 s 미리 보기 (간이 DWA) |
| `web/hallway_env.js` | 복도 환경 — course.js 와 같은 모양 (solids · height · hits · spots · update · movers · bounds) |
| `import_hallway.py` | 복도 프로젝트에서 형상 코드 · 데이터 · 텍스처 복사 |
| `web/serve.py` | 캐시 끈 정적 서버 (모듈 .js 를 고쳐도 옛 것을 안 쓰게) |

## 조작

- `W` `S` 앞뒤 · `A` `D` 회전 (방향키도) · `Space` 비상정지 · `R` 처음 자리
- ③ 비상정지 방식 (A 즉시 차단 · B 정지 후 차단 · C 정지 유지 · D 차단 + 상선 단락) 을 고르고 '10° 경사에서 네 방식 비교'.
- ④ '목표 찍기' 후 바닥을 누르면 자율주행. 사람 2 명이 오간다 (막히면 1.2 s 뒤 돌아선다).
- 검증용: 콘솔에서 `__sim.drive(v, w, 초)` · `__sim.run(초)` (자율주행 그대로) · `__sim.goal(x, y)` · `__sim.place('ramp15')` ·
  `__sim.estop(true)` · `__sim.estopTest()` · `__sim.cover.compute(0.3)` · `__sim.stab.set({m, h, x})` — 브라우저 패널이 숨어 화면 갱신이 멈춰도 돈다.
  **한 번에 10 s 안쪽으로** — 도구 시간 제한 (45 s) 에 걸린 스크립트는 페이지 안에서 계속 돌아 탭이 굳는다 (그때는 탭을 닫고 새로).
- 화면을 한 번도 안 그리면 matrixWorld 가 갱신되지 않아 광선 투사가 장애물을 원점에서 본다 → course.js 가 직접 갱신한다.
- 광선 투사는 **움직이지 않는 것을 월드 좌표로 한 메시에 합쳐 BVH (three-mesh-bvh 0.7.6) 하나**로 — 복도 (메시 443) 에서 한 번 재기 2.9 s → 21~35 ms.
- 점유 지도: 3 s 넘게 다시 안 보인 칸은 0.5 s 마다 줄인다 (지나간 사람 흔적), 깊이 점은 한 번 보이면 장애물.
  지금 길보다 1.5 배 (또는 4 m) 넘게 먼 길이 나오면 4 s 까지 기다린 뒤 돌아간다 (사람에 막혀 되돌아갔다 왔다 하지 않게).

## 어림 · 가정

- 질량은 부품 부피 × 밀도 + 알려진 무게 (허브모터 3 kg 은 [가정]) — v27 에서 약 33.6 kg.
- 턱은 바퀴 반지름의 40 % (33 mm) 까지만 넘는다고 본다. 경사 · 턱 위 자세는 네 바퀴 닿는 높이로만 (서스펜션 없음).
- 전원: 상 저항 0.15 Ω · 효율 0.75 · 미끄럼 마찰 0.6 은 [가정]. 실측으로 고친다.
