# 2026-10-13 · SfM 지도로 카메라 위치 다시 찾기 (정확도 재기)

> 상태: **미완** — 코드 (`relocalize.py`) 만 있고 결과는 없다. 2026-10-13 20:43 쯤 첫 실행 중 PC 가 멈췄다 (아래 '주의').

## 1. 목적

차가 카메라 사진만으로 복도 지도 위 자기 위치를 얼마나 정확히 찾는지 잰다. 시뮬레이터 '지도 + 오도메트리 + 시각 위치
인식' 모드는 지금 "걸음마다 80 % 확률로 ± 0.15 m" 를 [가정] 하고 있다 — 이 값을 실측으로 바꾸는 것이 목표.

## 2. 구성

- 자료 (읽기만): `F:\Develop\hallway-3d\tools\work\` — 복도 영상 263 프레임의 SIFT 특징점 DB (`sfm/database.db`), COLMAP
  복원 (`sfm/model/1`, 3D 점 51,220), 지도 좌표 변환 (`align.json`). 출처는 `F:\hallway-3d-data` 압축, 설명은 그 안
  `README_DATA.md`. 사진에 사람이 찍혀 있어 저장소에 넣지 않는다.
- 방법 (`relocalize.py` 머리말): 질의 프레임과 앞뒤 ± K 프레임의 관측을 지도에서 빼고, 질의 프레임의 SIFT 를 남은 3D 점
  기술자와 맞춰 (가장 가까운 이웃 + 비율 시험) PnP RANSAC (`pycolmap.estimate_and_refine_absolute_pose`) 으로 자세를
  구한 뒤, 복원에 있는 그 프레임 자세와 비교한다. 오차는 지도 좌표 m.
  - 변형 `full` (지도 전체), `low` (높이 0.8 m 밑 점만 — 낮은 로봇 카메라를 거칠게 흉내), `full_sfm`
  - K = 0, 5, 15, 40 프레임 (5 fps — ± 0 · 1 · 3 · 8 초)
- 파이썬: `F:\Develop\mnu-campus-tour\.venv` (pycolmap 4.2.1 · torch). 이 실험용 환경을 따로 만들지는 않았다.

## 주의 — 그래픽카드로 돌리지 말 것

첫 실행은 `--device auto` 로 그래픽카드 (RTX 3070, CUDA) 를 써서 특징점 거리 행렬 (블록당 약 1 ~ 2 GB) 을 계산했고,
같은 때 앱 브라우저의 시뮬레이터 (WebGL) 도 돌고 있었다. 20:43 ~ 20:49 사이 화면이 꺼지고 시스템이 멈췄다
(Windows 기록: Kernel-Power 41, 블루스크린 기록 없음). 원인이 직접 기록되지는 않았지만 그래픽카드 과부하가 가장 유력하다.
다시 돌릴 때는 **CPU 로, 시뮬레이터를 닫고, 작게 먼저**:

```bash
F:\Develop\mnu-campus-tour\.venv\Scripts\python.exe -I relocalize.py --device cpu --threads 4 --step 6 --ks 0,15 --tag _cpu_small
```

잘 끝나면 `--step` 을 줄이고 `--ks 0,5,15,40` 으로 늘린다. 결과는 `results.md` · `per_query.csv` · `error_cdf.png` (이 폴더).
코드는 하위 에이전트가 썼고 아직 사람이나 이 세션이 검토하지 않았다 — 결과를 쓰기 전에 오차 계산 (지도 좌표 변환 ·
기준 자세) 을 한 번 확인한다.

## 3. 결과

없음 (미완).

## 4. 결론

없음. 결과가 나오면 시뮬레이터 `web/agent.js` 의 `VIS_P` · `VIS_M` · `VIS_DEG` 를 이 값으로 바꾸고, 실차 위치 추정에
붙일지 정한다. 진짜 시험은 로봇 높이 (Xtion 렌즈) 에서 찍은 사진으로 해야 한다 — 이 영상은 1.37 m 높이에서 찍었다.
