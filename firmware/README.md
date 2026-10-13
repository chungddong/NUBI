# firmware — MCU 펌웨어

MCU 하나에 폴더 하나를 둡니다. 각 폴더는 그 자체로 빌드할 수 있는 독립 프로젝트입니다.

| 폴더 | 보드 | 역할 | 빌드 도구 | 상태 |
| --- | --- | --- | --- | --- |
| [wheel-controller/](wheel-controller/) | LOLIN D32 (ESP32-D0WD-V3) | 안전장치, 바퀴별 믹싱, Wi-Fi 조종 페이지, 상위 컴퓨터 시리얼 콘솔 | PlatformIO | 바퀴 모드 동작 중(2026-10-13 플래시, 바퀴 띄운 시험 통과). 그 뒤 프레임 코드를 직접 작성한 빌드는 실기 확인 전(H6) · 시험 GUI `wheelbench` · [열린 이슈](wheel-controller/ISSUES.md) |

모터 보드(호버보드 메인보드 ×4)의 펌웨어는 이 저장소에서 관리하지 않습니다. GPL-3.0 업스트림(RoboDurden
Hoverboard-Firmware-Hack-Gen2.x-GD32)을 고친 것이라 이 PC에서만 관리하고(`hoverboard-mainboard/`, git에서 제외),
나중에 다른 모터 드라이버로 바뀔 수 있습니다. 바퀴 제어기가 기대하는 보드 쪽 동작(프레임 칸의 의미 등)은
[system-overview.md](../docs/architecture/system-overview.md#인터페이스-2--바퀴-제어기--모터-보드)에 있습니다.

차체 설계가 펌웨어에 요구하는 것(보드 전원 핀 제어, 컨택터 감지, 전조등 PWM, 보드당 전류 제한 7 A 등)은
[system-overview.md](../docs/architecture/system-overview.md#차체-설계가-바퀴-제어기에-요구하는-것)에 모아 두었습니다.
메인보드 쪽 근거는 차체 설계 결정 [D-03](../hardware/vehicle/models/vehicle/DECISIONS.md)입니다.

## 폴더 안 구성

PlatformIO 프로젝트를 기준으로 합니다.

```
<보드-이름>/
├── README.md        현재 상태, 배선, 빌드 · 플래시 방법, 명령 목록
├── ISSUES.md        (있으면) 이 펌웨어의 열린 문제와 우선순위
├── requirements.txt PlatformIO 등 Python 쪽 도구 버전
├── platformio.ini
├── src/
├── include/
└── tools/           이 펌웨어 전용 PC 도구 (GUI, 진단, 회귀 테스트)
```

작업 일지와 디버깅 기록은 펌웨어 폴더에 따로 두지 않고 [docs/worklog/](../docs/worklog/README.md)에 씁니다.
바퀴 제어기의 통합 전 일지(`WORKLOG.md`)와 링크 디버깅 기록(`DEBUGGING.md`)도 그리로 옮겼습니다.

## 규칙

- 빌드 결과(`.pio/`)와 로컬 툴체인 venv는 git에 올리지 않습니다. venv는 폴더마다
  `requirements.txt`로 새로 만듭니다.
- 시리얼 포트(`COM20` 등)는 PC마다 다르므로 README에 찾는 방법을 같이 적습니다.
- 상위 컴퓨터와 주고받는 명령이나 프레임 형식을 바꾸면
  [docs/architecture/system-overview.md](../docs/architecture/system-overview.md)도 같이 고칩니다.
- 업스트림 코드에서 복사한 파일은 출처와 커밋/버전을 README에 적습니다.
