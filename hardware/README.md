# hardware — 기구 · 전장 설계

```
hardware/
├── mechanical/
│   ├── cad/        CAD 원본 (.FCStd, .f3d 등)
│   ├── scripts/    파라메트릭 빌더 스크립트 (FreeCAD, build123d)
│   ├── vendor/     벤더가 배포한 부품 STEP (모터, 센서, 보드)
│   └── exports/    내보낸 파일 (STEP, STL, 3MF, 렌더 이미지)
├── electrical/     배선도, 핀맵, 전원 계통도
└── bom/            부품 목록 (CSV)
```

## 규칙

- **좌표계** — x 전진, y 왼쪽, z 위, 지면 z = 0 (ROS REP-103과 같은 방향). 단위는 mm.
  나중에 URDF로 옮길 때 축을 다시 맞추지 않아도 되도록 처음부터 통일합니다.
- **원본과 산출물 분리** — 스크립트로 다시 만들 수 있는 파일은 `exports/`에 두고, 손으로 만든
  원본만 `cad/`에 둡니다. 빌더 스크립트는 상단에 치수 파라미터를 모아 둡니다.
- **벤더 파일** — `vendor/`에 넣을 때는 받은 곳과 날짜를 `vendor/README.md`에 적습니다.
- **BOM** — `bom/`에 CSV로 둡니다. 열: 부품명, 사양, 수량, 단가, 구매처, 링크, 비고.
- **핀맵** — 보드별로 `electrical/<보드>-pinout.md`. 펌웨어 코드의 핀 상수와 어긋나면 안 되므로,
  핀을 바꾸면 펌웨어 쪽과 같은 커밋에서 고칩니다.

FreeCAD 백업 파일(`*.FCBak`, `*.FCStd1`)은 git에 올리지 않습니다.
