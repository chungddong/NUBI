# firmware — MCU 펌웨어

MCU 하나에 폴더 하나를 둡니다. 각 폴더는 그 자체로 빌드할 수 있는 독립 프로젝트입니다.

| 폴더 | 보드 | 역할 | 빌드 도구 | 상태 |
| --- | --- | --- | --- | --- |
| [wheel-controller/](wheel-controller/) | LOLIN D32 (ESP32-D0WD-V3) | 안전장치, 좌우 믹싱, Wi-Fi 조종 페이지, 상위 컴퓨터 시리얼 콘솔 | PlatformIO | `lolind32-wheel`에서 이전 예정 |
| [hoverboard-mainboard/](hoverboard-mainboard/) | 호버보드 GD32F130 메인보드 | BLDC 구동. RoboDurden Gen2.x-GD32 기반, `SLAVE_SAME_SIDE` 패치 · 부저 끔 | PlatformIO (`platform-gd32`) | 빌드 산출물 복구 필요 |

## 폴더 안 구성

PlatformIO 프로젝트를 기준으로 합니다.

```
<보드-이름>/
├── README.md        현재 상태, 배선, 빌드 · 플래시 방법, 명령 목록
├── WORKLOG.md       (있으면) 이 펌웨어만의 작업 기록
├── DEBUGGING.md     (있으면) 이 펌웨어만의 디버깅 기록
├── platformio.ini
├── src/
├── include/
└── tools/           이 펌웨어 전용 PC 도구 (GUI, 진단, 회귀 테스트)
```

## 규칙

- 빌드 결과(`.pio/`)와 로컬 툴체인 venv는 git에 올리지 않습니다.
- 시리얼 포트(`COM20` 등)는 PC마다 다르므로 README에 찾는 방법을 같이 적습니다.
- 상위 컴퓨터와 주고받는 명령이나 프레임 형식을 바꾸면
  [docs/architecture/system-overview.md](../docs/architecture/system-overview.md)도 같이 고칩니다.
- 업스트림 코드에서 복사한 파일은 출처와 커밋/버전을 README에 적습니다.
- `hoverboard-mainboard/`는 업스트림 저장소 전체를 넣지 않고, 업스트림 커밋 해시 · 바꾼 설정
  (`config.master.h`, `config.slave.h`) · 패치 파일 · 빌드된 `.bin`만 둡니다. `.bin`은 이 폴더에서만
  git에 올라가도록 예외 처리해 두었습니다.
