# 비슷한 차량 사례 조사 (2026-10-05)

호버보드 · 인휠 허브모터 이동 로봇을 다른 사람들은 어떤 부품 조합으로 만들었나.
서브에이전트 4 갈래로 약 50 사례를 봤다. 값은 출처에 적힌 것만 옮겼고, 추론은 **(추론)** 으로 표시한다.
이 조사로 내린 결정은 [`../DECISIONS.md`](../DECISIONS.md) 에 있다.

## 한눈에

- **우리와 거의 같은 구성이 해외에 2 대** — 허브모터 4 + ZLAC8015D 2 + Jetson Orin Nano
  ([Cemani](https://github.com/nicedreamzapp/CemaniHomesteadRobot), [VECTOR](https://github.com/metrox-eth/vector-dimos)).
  둘 다 **ZLTECH 전용 엔코더 모터**를 썼다. 일반 호버보드 모터가 ZLAC8015D · 엠디로봇 MD400T 에서
  도는지 확인해 준 출처는 없었다.
- **호버보드 모터로 검증된 방식은 원래 메인보드 + 오픈소스 펌웨어 (EFeru FOC)** — 해외 15 대.
- **주행 중 배터리 전류를 재서 공개한 사례는 0 건.** 평지 6WD 약 40 W (보드 3 장 대기만 30 W),
  탑승형 4WD 최대 출력에서 피크 약 50 A 정도가 전부다.
- 국내 학생 사례는 24 V 배터리 + 엠디로봇 드라이버 · 전용 모터가 대부분. 36 V 호버보드 모터 4 개로
  완성한 국내 사례는 못 찾았다.

## 1. 호버보드 원래 메인보드 + 펌웨어 (해외 15 대)

| 사례 | 바퀴 | 드라이버 · 제어 | 배터리 | 비상정지 | 배울 점 |
|---|---|---|---|---|---|
| [Robaka 2](https://github.com/alex-makarov/hoverboard-driver) | 2WD | EFeru 포크, Jetson UART 직결 | 원래 팩 | 미기재 | 오도메트리 보내도록 펌웨어 수정, ROS diff_drive |
| [HoverMower](https://hovermower.github.io/) | 2WD | EFeru, UART | 원래 10S2P | 킬스위치는 모터만, 컴퓨터 유지 | Meanwell DDR-30-5 로 5 V, BNO085 + 오도메트리 EKF |
| [Allegrobotics Skidrover](http://www.allegrobotics.com/skidrover.html) | **4WD 스키드** | 보드 2 장, **ESP32 하드웨어 UART 2 개** | 미기재 | 범퍼 | 우리 구조와 같다. 115200 → 19200 baud. 일부 보드 (AT32) 는 플래시 안 됨 |
| [geofrancis 로버](https://discuss.ardupilot.org/t/using-the-boards-from-the-hoverboard-in-autopilot/87229) | 4WD → 6WD | 보드 2~3 장, RP2040 이 보드마다 UART | 13S 20 Ah | 보드마다 전원버튼 릴레이, 하트비트 끊기면 차단 | **평지 약 40 W, 보드 대기 보드당 약 10 W.** VLT 모드는 저속 스키드 회전이 덜컥 → **SPD + 정지 유지**. **보드 12 V · 5 V 출력으로 폰 충전하다 보드 2 장 소손** |
| [golf 푸시카트](https://www.instructables.com/Cheap-Remote-Controlled-Electric-Pushcart-200-Usin/) | 2WD | EFeru SPEED 모드 | 10S4P 482 Wh | 리모컨 | 속도 0 명령으로 경사에서 정지 유지 |
| [larsmm 바비카](https://github.com/larsmm/hoverboard-firmware-hack-FOC-bbcar) | 4WD (보드 2) | EFeru 개조 | 12S 10 Ah | 저전압 경고 후 꺼짐 | **최대 출력 · 모래에서 피크 약 50 A**. 한 보드만 켜지는 일 → 끄는 절차 |
| [figch 바비카](https://figch.de/en/bobbycar_controllerteensy/) | 4WD (보드 2) | Teensy 가 두 보드 USART | 12S 10 Ah | 입력 이상 시 정지 | **소프트웨어 제한 2 × 15 A = 30 A** |
| [Aaron Christophel 탱크](https://hackaday.com/2019/08/24/hacked-hoverboards-become-potent-rc-tank/) | 4WD (보드 2) | Niklas 펌웨어 | 원래 팩 | 미기재 | 무거우면 제자리 회전이 힘듦 |
| 그 밖 | | Mowbot, Kambodia91, gaucho1978, PranayG, Hoverbot, TranspOtter, McKey | | | |

이 방식의 주의점 ([EFeru README](https://github.com/EFeru/hoverboard-firmware-hack-FOC), 위키)
- 지원 보드는 **STM32F103RCT6 / GD32F103RCT6 일체형 메인보드뿐**. 2 세대 분리형 · AT32 는 다른 펌웨어.
- ST-Link 로 굽는다 (쓰기 잠금 해제, 굽는 동안 전원버튼 누름, 36 V 배터리 연결). 시리얼은 3.3 V.
- 4WD 는 하위 MCU 하나가 두 보드에 같은 명령. 보드 2 장 전원 켜고 끄기 동기 문제가 보고됨.
- 명령이 끊기면 약 0.8 s 뒤 출력 0 (`SERIAL_TIMEOUT`). 8 분 무동작이면 자동 꺼짐.
- 전류 제한 기본 `I_MOT_MAX 15 A`, `I_DC_MAX 17 A` (모터당).
- 약계자 상태 제동 → 배터리 과전압 → BMS 차단 가능. 온도 차단은 기본 꺼짐.

## 2. 허브모터 + 상용 드라이버 (해외 17 대)

| 드라이버 | 사례 | 장점 | 단점 |
|---|---|---|---|
| ZLAC8015D (2 채널, RS485 Modbus / CANopen) | 4 (+국내 1) | 2 대로 4 륜. RS485 한 줄에 ID 로 2 대. ROS2 드라이버 많음 | **호버보드 모터 호환 미확인**, 문서 빈약, 이슈 무응답 |
| ODrive v3.6 / S1 | 6 | 호버보드 공식 가이드 | v3.6 단종, S1 은 4 대 필요, 해외 구매만. 홀 필터 커패시터 · **제동 저항 필수** |
| VESC / Flipsky | 4 | 저렴 | 저속 제어 약함, PPM 이면 바퀴 속도 불일치 |
| moteus-c1 | 1 | 저렴 | 피크 20 A |

- [ROMR](https://pmc.ncbi.nlm.nih.gov/articles/PMC10197097/) — 호버보드 모터 + ODrive, 36 V 4.4 Ah, **50 W 제동 저항**,
  Jetson 은 **별도 5 V 보조배터리** (모터 계통과 분리).
- Jetson ↔ ODrive 를 USB 로 묶었다가 접지 루프로 드라이버 손상 → 차동 (RS485 · CAN) 이나 절연 권장.

## 3. 국내 사례 (12 건)

| 사례 | 구성 | 배울 점 |
|---|---|---|
| [LiDAR 로버 (졸업논문)](https://dldptmd-mech.tistory.com/33) | 3030 프레임. 호버보드 6.5" + BLC-160 → 폐기 → MDH250 + MD400T | 저가 드라이버로는 저속 토크 약함. **MD400T 2 대를 CAN 마스터/슬레이브로 묶으면 슬레이브가 오픈루프라 미끄러짐** → 드라이버마다 RS485 |
| [Olaf 제설 로봇](https://stupidly-honest.tistory.com/72) | MDH100 ×4 + MD200T ×2, 24 V | 댓글: 제자리 회전 시 미끄럼 마찰로 모터 허용 부하 초과 |
| [RIS 플랫폼](https://youngseong.tistory.com/153) | MD200T, Xavier NX | 엠디로봇 ROS 패키지 체크섬 `uint8_t` 오버플로 → ARM 에서 명령 무시 |
| [Cartrasche (연세대)](https://github.com/Cartrasche-CreativeProductDesign) | MD200T, Jetson Nano | ESP32 무선 E-stop 이 MQTT "kill" 만 보냄 — 하트비트 없음 |
| [NEXTRON 농업로봇](https://github.com/jwejweiya/agri_robot) | ZLAC8015D + 24 V 허브 4, Orin Nano | 국내 ZLAC 사례 1 건 |
| [ZETIN (서울시립대)](https://github.com/lightminn/power-train-sw) | ODrive, Orin Nano | 0.75 s 토픽 타임아웃 + 래칭 E-stop |
| 하이제니스 (네이버, 본문 미확인) | 호버보드 6.5" + ODrive | **6.5" 모터 고정축 42 · 44 mm 두 종류**, 2020/4040 브래킷 국내 판매 |

- 국내 IMU: **iAHRS 2 건** (Olaf, RIS), BNO055, MPU6050.
- 하드웨어 E-stop · 퓨즈를 적은 학생 사례는 0 건.

## 4. 상용 플랫폼의 전원 · 안전 구조

| 플랫폼 | 배터리 | 사용자 전원 | 비상정지 |
|---|---|---|---|
| [Husky A300](https://docs.clearpathrobotics.com/docs_robots/outdoor_robots/husky/a300/user_manual_husky/) | 25.6 V LiFePO4, 연속 60 A, **주 차단기 100 A** | 12 V 5 A 레일 2, 레일별 자동복귀 퓨즈 | 버튼과 직렬 **릴레이-컨택터 2 개가 드라이버 전원 차단**, 차단 중 전기 제동, 수동 리셋, 무선 E-stop 은 통신 끊기면 작동 |
| [Husarion Panther](https://husarion.com/manuals/panther/) | **36 V 20 Ah (720 Wh)** — 우리와 같은 급, 총 출력 1.0 kW | 5 V 15 A, 12 V 25 A, **19 V 10 A (컴퓨터)** | 버튼 · ROS · **CPU 워치독** → 드라이버 정지 + 모터로 능동 제동 + 래치 |
| [Nova Carter](https://robotics.segway.com/nova-carter/) | 1033 Wh, 42 V 충전 (10S) | — | 모터 전원만 차단, 컴퓨터 유지. **허브모터라 경사에서 굴러갈 수 있다고 문서가 경고** |
| [AgileX Scout 2.0](https://agilexrobotics.gitbook.io/scout2.0/2-the-basics.md) | 24 V 30 Ah | 24 V 5 A | 로봇 전원 즉시 차단, **CAN 명령 500 ms 없으면 속도 0** |
| [Jackal](https://docs.clearpathrobotics.com/docs_robots/outdoor_robots/jackal/user_manual_jackal/) | 270 Wh | 12 V 7 A, 5 V 5 A | Motion Stop. **자력계는 섀시 위치 때문에 비권장** |
| [ROSbot XL](https://husarion.com/manuals/rosbot-xl/) | 3S 86 Wh | **12 / 19 V DIP 선택** | — |

공통 패턴
- 배터리를 모터 정격 합에 맞추지 않고, **드라이버 전류 제한 합을 배터리 연속 정격 아래**로 묶는다.
- **모터 전원만 끊고 컴퓨터는 유지**. 허브모터는 전원이 끊기면 자유 회전 → 제동 단계를 같이 둔다.
- 해제는 수동 (돌려서 복귀 + 리셋). 소프트웨어 감시 0.2~0.8 s.
- 퓨즈: 배터리 가까이 주 차단기 + 분기마다 개별 퓨즈.
- Jetson Orin Nano 개발킷 DC 잭 9~20 V (NVIDIA 포럼 답변 — 공식 사양서 원문 미확인).

## 5. 회전 · 경사 전류 근거

- Husky A200 매뉴얼: 모터 연속 10.7 A 가 제자리 회전에서 수 배로 튄다 (배수 미기재).
- [Shamah, CMU 1999](https://publications.ri.cmu.edu/experimental-comparison-of-skid-steering-vs-explicit-steering-for-a-wheeled-mobile-robot): 제자리 회전에서 스키드 방식은 조향식의 약 2 배 동력.
- 역학 근사 (40 kg, 6.5", 측면 마찰 0.6 가정): 바퀴 하나 토크 — 평지 0.16 · 10° 경사 1.56 · 제자리 회전 약 4.9 N·m.
  **토크는 상전류를, 출력 (토크 × 속도) 은 배터리 전류를 정한다** — 느린 제자리 회전은 드라이버 상전류를,
  속도가 붙은 경사 · 가속은 BMS 를 압박한다.

## 확인 못 한 것

- 호버보드 모터 + ZLAC8015D / MD400T 동작 여부 (판매처 문의 필요 — 이번엔 원래 보드로 가서 보류).
- 4WD 호버보드 로봇의 제자리 회전 · 경사 실측 전류.
- Reddit · 네이버 블로그 · 일부 hackaday 페이지는 열리지 않았다.
