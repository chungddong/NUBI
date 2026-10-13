# docs — 문서

여러 계층에 걸친 문서를 둡니다. 펌웨어 하나, 기구 부품 하나에만 해당하는 설명서(README 등)는 그 폴더에
둡니다. 단, **일지와 기록은 영역과 상관없이 모두 [worklog/](worklog/README.md)에 둡니다.**

| 폴더 | 무엇을 | 예 |
| --- | --- | --- |
| [worklog/](worklog/README.md) | 모든 작업 일지(월별)와 주제별 기록. 2026-07-27 첫 설계부터, 한 가지 양식의 Markdown | `2026-10.md`, `topics/wheel-link-debugging.md` |
| [architecture/](architecture/) | 시스템 구성, 계층 간 인터페이스, 좌표계 · 토픽 규약 | [system-overview.md](architecture/system-overview.md) |
| [decisions/](decisions/README.md) | 여러 계층에 걸친 설계 결정(ADR). 차체 결정은 DECISIONS.md(D-NN) | `0001-wheel-board-pairing.md` |
| [guides/](guides/) | 셋업 · 빌드 · 플래시 · 운용 방법, 증상별 해결 방법 | `jetson-setup.md`, `first-drive.md` |
| [references/](references/) | 데이터시트 링크, 외부 자료 요약 | `hub-motor.md` |
| [assets/](assets/) | 문서에 넣는 이미지 · 다이어그램 | `system-layers.png` |

## 문서마다 다른 역할

- **README** — 지금 상태. 처음 보는 사람이 읽고 바로 쓸 수 있게.
- **worklog** — 시간순 기록. 나중에 포트폴리오나 보고서를 쓸 때 근거가 되도록 수치와 판단 근거를
  같이 남깁니다. 실패한 시도와 잘못 짚은 경로도 남깁니다(긴 것은 `topics/`).
- **decisions** — 한 번 정한 것을 다시 논의하지 않도록, 결정 시점의 선택지와 이유를 남깁니다.
- **guides** — 따라 하면 되는 절차. 왜 그렇게 됐는지는 worklog에 두고 링크만 겁니다.

## 작성 규칙

- 한국어로 씁니다. 코드, 명령, 파일 이름은 원문 그대로 백틱으로 감쌉니다.
- 파일 이름은 소문자 kebab-case입니다.
- 이미지는 `assets/`에 두고 상대 경로로 링크합니다(차체 그림은 `hardware/vehicle/docs/images/`).
- 날짜는 `YYYY-MM-DD` 형식으로 씁니다.
- HTML 대신 Markdown으로 씁니다. 따로 열어 보는 인터랙티브 페이지만 예외입니다.
