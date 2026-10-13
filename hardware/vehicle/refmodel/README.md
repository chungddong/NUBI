# refmodel — 외부 참조 STEP

남이 만든 참조 모델(STEP)을 두는 곳입니다. 출처가 따로 있고 재배포 조건을 확인하지 못해 git에는 넣지 않습니다
(`.gitignore`). 필요한 파일은 직접 받아 이 폴더에 같은 이름으로 둡니다.

| 파일 | 무엇 | 쓰는 곳 |
| --- | --- | --- |
| `Motor, BLDC, Hoverboard Wheel, 6.5in.stp` | 호버보드 6.5인치 휠 모터. 외경 Ø158.74, 폭 111(축 포함), 축 Ø16 D컷 | 차체 버전 v01 ~ (`models/vehicle/vNN_*/motor_reference.py`), `models/hoverboard_mount/motor_reference.py`, `models/swerve_base.py`, `models/wheel_mount.py` |
| `Cramped Robot Wheel` STEP | 바퀴 참조 모델 | `models/wheeled_base.py` (재설계 전 스크립트) |

휠 모터 STEP이 없으면 차체 버전을 다시 지을 수 없습니다(`record.py <버전> --rebuild`). 이미 기록된 이미지 · BOM ·
검증 결과(`scene.json`)는 저장소에 있으므로 보는 데는 필요 없습니다.

치수만 맞춘 단순 모델은 `models/hoverboard_mount/motor_reference.py`의 `motor_reference()`로 만들 수 있습니다
(간섭 확인용).
