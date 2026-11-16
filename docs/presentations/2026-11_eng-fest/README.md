# 2026 교내 공학 페스티벌 전시 덱

LoadGoesOn 팀 초안 PDF(`LoadGoesOn 공학페스티벌 전시.pdf`, 10장)의 디자인과 흐름을 따라, 누비 저장소의 기록으로 내용을 채운
전시용 PowerPoint입니다. 결과물은 [LoadGoesOn_공학페스티벌_전시.pptx](LoadGoesOn_공학페스티벌_전시.pptx) (16:9, 11장, 발표자 노트 포함).

## 다시 만들기

```powershell
cd docs\presentations\2026-11_eng-fest
npm install                 # pptxgenjs (처음 한 번)
py prep_images.py           # 저장소 그림을 잘라 images/ 에 만든다
node build.js               # pptx 생성
py render.py                # (선택) PowerPoint로 슬라이드를 render/ 에 PNG로 — 화면 점검용
```

글꼴은 맑은 고딕(Windows 기본)입니다.

표지 렌더를 다시 만들 때 (Blender 5.2, 저장소 루트에서, 약 40초):

```powershell
& "C:\Program Files\Blender Foundation\Blender 5.2\blender.exe" -b --factory-startup -P docs/presentations/2026-11_eng-fest/render_cover.py -- docs/presentations/2026-11_eng-fest/source/cover_render.png 384
```

## 아직 채울 것

| 장 | 자리 | 무엇 |
| --- | --- | --- |
| 2 | `[___]만 원` | 3D 라이다 한 대 가격 |
| 12 | `[ QR ]` | 주행 영상 QR |

사진 자리는 점선 상자입니다. 사진을 받으면 `build.js`의 해당 `photo(...)`를 `picture(...)`로 바꿉니다.

`source/`에는 저장소 밖에서 가져온 자료가 있습니다: 교내 복도 웹 시뮬레이터 캡처(`?env=hallway`, 뒤따라가기 시점, 2026-11-05),
RC카 SLAM 지도(Jetson `~/Downloads/slam_map/session_20260911_100506/really_final.png`).

팀 사진(5 · 9 · 10장)의 원본은 `~/Downloads/자율주행 플랫폼 관련/`에 있고, `prep_images.py`가 칸 비율로 잘라 `images/photo_*.jpg`로 만듭니다.
쓰지 않은 사진: 실내 테스트, 외부 테스트, 최소 구동 테스트.

## 수치 출처

| 수치 | 출처 |
| --- | --- |
| 위치 재인식 오차 1.1 cm · 성공률 97.7 % | [SfM 재위치 실험](../../../experiments/2026-10-13_sfm-relocalization/README.md) — OpenCV 특징점, 전체 지도, K = 15 (질의 앞뒤 약 ± 3 m 지도 사진 제외), Jetson 측정. 실제 주행 중 오차가 아니다 |
| 라이다 배달로봇 약 2,000만 원 | 이데일리 2021 뉴빌리티 기사 — 라이다 센서 로봇 한 대 평균 2,000만 원 (배민 딜리 가격은 비공개) |
| 부품 예산 약 264만 원 | 저장소 README "현재 상태" — 국내 구매처 단가 확인분 합 |
| 830 × 590 × 534 mm · 약 34 kg | 차체 v28 (무게는 시뮬레이터 추정) |
| 정책 신경망 7,170 파라미터 · 9/14 실주행 | Jetson `~/Downloads/LoadGoesOn_연구기록_실물RL배포_20260914.txt` |
| 웹캠 1,940장 · 주행 경로 5회 | `data/processed/2026-10-15_hallway_robot-map/robot_map_paths.png` |
