# 호버보드 2 세대 분리형 보드 조사 (2026-10-05)

사용자 보드: 호버보드 2 대에서 꺼낸 **직사각형 보드 4 장** (바퀴마다 1 장). mm 치수를 적어 둔 출처는 없었다 —
아래는 RoboDurden 저장소 사진을 LQFP48 (7 mm) · JST-XH (2.5 mm) · 2.54 헤더로 비례해 잰 값 (±10~15 %).
결정은 [`../DECISIONS.md`](../DECISIONS.md) D-03.

## 변형

| 레이아웃 | 기판 표기 | MCU | PCB | MOSFET · 방열 | 흔한 정도 |
|---|---|---|---|---|---|
| **Gen2.1.1** | **TT-SD2.2** (녹색) | GD32F130C8 | **약 50 × 113**, 모서리 M3 4 (간격 약 42 × 105) | TO-220 6 이 **PCB 밑** 긴 변에 눕고 그 밑 알루미늄 바 약 80 | **가장 흔함** (RoboDurden #126) — v21 모델 기준 |
| Gen2.1.3 | Ver 1.5 | GD32F130K8 | 약 103 × 47 | 윗면 MOSFET + 핀 달린 바 | |
| Gen2.4.2 | Type_A | MM32SPIN05PF | 약 103 × 46 | 2.1.3 과 비슷 | 저가형 |
| Gen2.1.4 | iB-2018-REV1.0 (청색) | GD32F130C8 | 약 133 × 57 | PCB 밑 | |
| Gen2.1.8 | JC ROBOT SXT_V33 | GD32F130C8 | 약 127 × 57 | PCB 밑 | |

공통: MOSFET 6 과 상선 3 가닥이 한쪽 긴 변, 신호 커넥터 (홀 6p · 마스터-슬레이브 4p · VBAT 2p) 는 반대 긴 변.
마스터에만 전원 버튼 · 충전 3p · 부저.

## 펌웨어 · 제어

- [RoboDurden Hoverboard-Firmware-Hack-Gen2.x](https://github.com/RoboDurden/Hoverboard-Firmware-Hack-Gen2.x) — GD32F130 · GD32F103/STM32F103 · GD32E230 · MM32SPIN05/06/25 · LKS32 · GD32E235. **AT32F415 없음.** MM32 는 [AILIFE 판](https://gitlab.com/ailife8881/Hoverboard-Firmware-Hack-Gen2.x-MM32).
- 4 장 제어 방식

  | 방식 | 구성 | 비고 |
  |---|---|---|
  | **A (채택)** | 원래 짝 2 개 (앞축 · 뒤축), ESP32 UART 2 개 → 각 마스터 PB6/PB7, 19200 baud | 다이오드 불필요, 켤 보드 2 장, 피드백에 좌우 속도 · 전류 · 전압 |
  | A' | 마스터 2 장을 UART 버스 하나에 (ID 0/1) — 공식 `Test_4Wheeler.ino` 기본 | 다이오드 필요 |
  | B | 4 장 모두 SINGLE + UARTBUS (ID 0~3) | 바퀴별 독립, 보드 TX 마다 쇼트키 + 풀업, 19200 에서 한 바퀴 약 50 ms |

- 굽기: ST-Link V2 (MM32 는 DAPLink). 전체 지우기로 잠금 해제 → 원래 펌웨어 영구 삭제. 미리 만든 바이너리는 낡음 → [온라인 컴파일러](https://pionierland.de/hoverhack/).
- **핀 설정이 틀리면 MOSFET 소손** — 첫 시험은 1~2 A 정전류 전원으로 REMOTE_DUMMY.
- 전원: `DISABLE_BUTTON` + ESP32 GPIO → 버튼 저전압 쪽 핀 (High = 켜짐). 버튼 다른 핀은 고저항 거쳐 약 42 V.
- **마스터-슬레이브 헤더 + 핀은 12~15 V** — ESP32 에 물리지 않는다. PB6/PB7 은 5 V 허용.
- `TIMEOUT_MS` 2000, `DC_CUR_LIMIT` 기본 보드당 15 A, 8 분 무동작 꺼짐.

## 방열 · 고정

- 원래 호버보드 안에서는 모서리 나사 4 개로 셸에 고정, MOSFET 은 알루미늄 바에 나사 (흰 절연 와셔가 사진에 보임).
- PC 바닥판은 단열재 — PCB 밑 MOSFET 형식은 **M3 스탠드오프 10 mm 이상**으로 띄우거나 바를 금속에 서멀패드로 (절연 확인 후).

## 사용자에게 확인받을 것

- MCU 각인 · 기판 실크 이름 · 게이트 드라이버 칩 수 · 두 대가 같은 종류인지 · 마스터/슬레이브 구분 · 블루투스 모듈 유무
- PCB 가로 · 세로 · 구멍 간격, 바 크기, 윗면 · 밑면 최고 높이, 커넥터 방향, 케이블 길이 (모눈종이 위 자와 함께 찍은 사진이면 충분)
