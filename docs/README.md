# docs — 문서

여러 계층에 걸친 문서를 둡니다. 펌웨어 하나, 기구 부품 하나에만 해당하는 문서는 그 폴더에 둡니다.

| 폴더 | 무엇을 | 예 |
| --- | --- | --- |
| [architecture/](architecture/) | 시스템 구성, 계층 간 인터페이스, 좌표계 · 토픽 규약 | [system-overview.md](architecture/system-overview.md) |
| [decisions/](decisions/) | 설계 결정 기록 (ADR). 왜 이 방식을 골랐고 무엇을 버렸는지 | `0001-drive-configuration.md` |
| [worklog/](worklog/) | 월별 작업 기록. 무엇을 왜 했고, 무엇을 확인했고, 무엇이 남았는지 | `2026-10.md` |
| [guides/](guides/) | 셋업 · 빌드 · 플래시 · 운용 방법 | `jetson-setup.md`, `first-drive.md` |
| [troubleshooting/](troubleshooting/) | 문제 해결과 실패 기록. 증상 → 원인 → 해결, 잘못 짚은 경로까지 | `ch340-noise-byte.md` |
| [references/](references/) | 데이터시트 링크, 외부 자료 요약 | `hub-motor.md` |
| [assets/](assets/) | 문서에 넣는 이미지 · 다이어그램 | `system-layers.png` |

## 문서마다 다른 역할

- **README** — 지금 상태. 처음 보는 사람이 읽고 바로 쓸 수 있게.
- **worklog** — 시간순 기록. 나중에 포트폴리오나 보고서를 쓸 때 근거가 되도록 수치와 판단 근거를
  같이 남깁니다.
- **decisions** — 한 번 정한 것을 다시 논의하지 않도록, 결정 시점의 선택지와 이유를 남깁니다.
- **troubleshooting** — 같은 실수를 반복하지 않도록, 실패한 시도도 남깁니다.

## 작성 규칙

- 한국어로 씁니다. 코드, 명령, 파일 이름은 원문 그대로 백틱으로 감쌉니다.
- 파일 이름은 소문자 kebab-case입니다.
- 이미지는 `assets/`에 두고 상대 경로로 링크합니다.
- 날짜는 `YYYY-MM-DD` 형식으로 씁니다.
