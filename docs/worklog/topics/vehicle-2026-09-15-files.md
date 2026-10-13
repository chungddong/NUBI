# 차체 설계 파일 지도 (2026-09-15, `D:\build123d` 기준)

> 영역: 차체 · 작성: 2026-09-15 · 원본: 차체 설계 일지 `synth_files.html` (2026-10-13 Markdown으로 옮김)

2026-09-15 저녁, 저장소 `D:\build123d` 를 직접 훑어(Glob·ls·git ls-files) 지금 있는 파일만 적었다. 모빌리티 베이스(3030 프로파일 스키드 4륜, 호버보드 6.5인치 허브모터, 외장셸, 적재함, Design-D 휠 클램프, FEA, 액슬 타이, BOM)에 관계된 파일이 대상이고, 같은 저장소에 있는 다른 프로젝트(필박스·NFC 턴테이블·볼 로봇 등)는 맨 끝에 한 줄로만 남겼다. "만든 날"은 이 일지의 청크 번호와 KST 날짜로 적었다. 일지에 안 나오는 파일은 파일 시각(mtime, KST)을 적고 *일지 밖*이라 표시했다 — 그 대부분은 08-15\~08-20 사이 GPT/Codex 로 작업한 구간이며(그 구간의 정리: [08-15 (토) 15:59 \~ 08-19 (수) 23:31 KST — 세션 밖(GPT/Codex): 프레임 폭 450 통일·FEA 파이프라인·구형 브라켓 파손 재현](../2026-08.md#2026-08-15-토--차체-세션-밖gptcodex-프레임-폭-450-통일fea-파이프라인구형-브라켓-파손-재현)), 08-21 새벽(청크 14a)에 "인수인계 검증"으로 이 세션에 들어왔다. 확인하지 못한 것은 "확실하지 않다"고 적었다.

## 청크 번호 → 날짜 (KST)

| 청크 | KST | 한 줄 |
| --- | --- | --- |
| 01a / 01b | 07-27 18:11\~21:24 | tslot 단면, mobility_base v1\~v7, 뷰어 |
| 02a / 02b | 07-27 21:25 \~ 07-28 02:03 | 모터 탐색, 호버보드 결정, build-log 첫 작성, 스커트·펜더·데크 레일 |
| 03 / 04 | 07-28 17:25 \~ 07-29 12:02 | swerve_base 시작, 셸·Xtion·서스펜션, 삼면도 |
| 05 / 06 | 07-29 16:26 \~ 07-31 11:07 | A급 부품, 적재함 v1, 핸들, 89 kg, payload-interface, 모듈 5종 |
| 07 | 07-31 11:20 \~ 08-01 01:11 | 아웃리거·확장레일, 실물 6.5인치 STEP 실측, MODULE_ONLY |
| 08a / 08b | 08-01 01:15\~18:13 | 스워브 포기·DRIVE=skid, 판재 브라켓, wheel_jig, motor_stand |
| 09 | 08-01 20:41 \~ 08-05 16:44 | 축 나사 없음, 축캡, 조인트 브라켓 자동 배치, export_cnc, 3030 휠포스트 |
| 10a / 10b | 08-07 01:07\~14:34 | wheel_mount ㄷ자, CNC/PRINT 분리, drawings.py 재작성, PLA 강도 |
| 11 | 08-10 00:00 \~ 08-11 16:44 | build-log 21\~28막, 적재함 v2\~v4(유로 컨테이너) |
| 12 | 08-12 20:32 \~ 08-13 01:09 | 실물 파손, report_loads/stress_view, 거싯, mount_low/mount_wrap, 볼트 물림 |
| 13 | 08-14 11:18 \~ 08-15 13:42 | hoverboard_mount 패키지 A/B → C → D → 통짜 D, drawing.py 작성 |
| (일지 밖) | 08-15 \~ 08-20 | GPT/Codex 작업 — FEA 파이프라인, 통합 뷰어, 인수인계 문서 |
| 14a / 14b | 08-21 00:17\~03:31 | 인수인계 검증, 플랜지 제거, FEA 재실행, 젖힘 시각화, 액슬 타이, 프레임 보요소 |
| 15 | 08-21 07:57 \~ 08-27 02:45 | 봉 400 고정, git 초기 커밋, 재료 검토, T텅, 3030 타이 |
| 16 | 09-10 10:43\~11:03, 09-15 19:06 | 전체 차량 뷰어, BOM, Trilium 일지화 착수 |

## 한눈에 보는 의존 관계

```
models/tslot.py ─┬─ models/tslot_frame.py
                 ├─ models/mobility_base.py        (07-27 스키드 v1~v7, 동결)
                 ├─ models/swerve_base.py ◀── models/wheel_mount.py (import wheel_mount as WM)
                 │        ├─ models/drawings.py, models/export_cnc.py
                 │        ├─ models/wheel_jig.py, models/motor_stand.py
                 │        ├─ models/hoverboard_mount/vehicle_*_integration.py
                 │        └─ tools/bom/bom.py
                 └─ (tslot.cut_list 과금 규칙을 bom.py 가 그대로 쓴다)

models/hoverboard_mount/parameters.py ── low_clamp.py ─┬─ export.py / export_low_clamp_step.py / export_clamp_stl.py
                                                        ├─ drawing.py, run_clamp_fea.py ◀ tools/fea/run_clamp_fea.ps1
                                                        ├─ vehicle_integration.py / vehicle_shell_integration.py
                                                        ├─ assembly_animation.py, render_assembly_gif.py, load_*.py
                                                        └─ tools/figures/tipping_moment.py, tools/frame/frame_fea.py

refmodel/Motor, BLDC, Hoverboard Wheel, 6.5in.stp ── swerve_base.py, wheel_mount.py, hoverboard_mount/motor_reference.py
공용 렌더: models/iso_render.py (뷰어 없이 PNG)  ·  models/render.py (이 일지에는 안 나온다)
```

한 파일이 다른 파일을 그리는 형태라 **원본은 한 곳**이다 — 휠 마운트는 `wheel_mount.py`, Design-D 치수는 `hoverboard_mount/parameters.py`, 프로파일 단면·과금은 `tslot.py`. 08-07(10b)에 "두 벌로 그리면 반드시 어긋난다"며 swerve_base.py 가 wheel_mount 를 import 하도록 바꾼 것이 이 구조의 시작이다.

---

## 1. 모델 스크립트 — models/\*.py

| 파일 | 역할 | 만든 날 (청크) | 비고 |
| --- | --- | --- | --- |
| `tslot.py` (125줄) | T슬롯 프로파일 라이브러리. PROFILES 20/30/40 단면(단면적 187.59 / 384.10 / 821.34 mm²), `bar()`, `cut_list()`. 100 mm 단위 올림 과금(CUT_UNIT=100, UNIT_PRICE=950원)까지 여기서 센다 | 07-27 18:27 (01a). 과금 계산 08-01 (08b), 단가 900→950 08-05 10:27 (09) | 다른 모델 4개 + bom.py 가 쓴다. 내부 리브를 단순화해 단면적이 실물보다 10\~15% 크다(머리말에 적혀 있다) |
| `tslot_frame.py` (64줄) | 2020 프로파일 400×300×400 직육면체 예제 프레임(빔 12본, 맞댐 조인트) | 07-27 18:11 (01a) | 이날 값 그대로. 산출물 `exports/tslot_frame.step/.stl` |
| `mobility_base.py` (880줄) | 첫날(07-27) 스키드 4륜 베이스. 3030 2층 프레임, 라운드 측면 커버(앞/뒤 분할), 비상정지·포트 도어·카메라·LED 얼굴, 흡기 슬롯·80 mm 팬, 배터리 브라켓, 탈착식 스커트/펜더(`LOWER_COVER`), 2020 데크 레일 3본 | 07-27 18:27 (01a) → 01b·02a·02b 에 걸쳐 v7 까지. 마지막 수정 07-28 17:45 (03, `reset_camera`) | PVC Ø210 통바퀴 + Ø20 데드 액슬, 0.90 축소(630×450, 축간 400) 상태로 **동결**. 인휠 전환은 이 파일에 반영되지 않았고 swerve_base.py 로 넘어갔다. 09-15 에 실행하면 전장 700 × 전폭 644 × 전고 376 |
| `swerve_base.py` (2,961줄) | **차량 본체.** 이름은 스워브지만 기본값은 `DRIVE=skid`(4륜 고정, 축간 420 / 윤거 520). 3030 프레임 760×450, 조인트 브라켓 자동 탐색(28곳), 외장셸 810×610 R60, 얼굴·후면 포트 베이·소형 카메라 6, 핸들, 범퍼, 안테나, 페이로드 5종(`PAYLOAD=cargo\|flat\|sensor\|arm\|inst\|none`), 적재함 v4(유로 컨테이너 400×300×220), 아웃리거·확장레일 옵션, 스워브 모듈(`DRIVE=swerve`) | 07-28 17:25 (03) 스워브로 시작 → 08-01 01:44 (08a) DRIVE=skid 기본. 페이로드 07-30\~31 (05·06), 적재함 v4 08-11 (11), `MOUNT=low` 08-13 (12). 마지막 수정 08-17 02:16 (일지 밖) | assert 51개. 환경변수 스위치가 20종 이상(NOSHOW, HUB_MOCK, ONLY=, MODULE_ONLY, CAM_RAIL, OUTRIGGER, SIDE_RAIL, TOP_RAIL_Y, SHELL_Y …). 전체 실행 약 2분, 간섭 검증은 4\~10분 |
| `wheel_mount.py` (677줄) | ㄷ자 휠마운트 작업대 — 3030 레일 한 토막 + 실물 허브모터 STEP + 실물 OEM 클램프 STEP 만 올린다. `mount()`(ㄷ자, 거싯 6장), `mount_low()`(하단 클램프 84×40×24), `mount_wrap()`(3면), `report_loads()`·`stress_view()`(하중 경로 사용률), PRINT=1 / CLAMP_BOLT / NO_GUSSET / U_OPEN_INBOARD 스위치 | 08-07 01:14 (10a) 생성, 같은 날 (10b) PRINT 분리·M6 결정. 08-12 (12) 거싯·report_loads·mount_low·mount_wrap. 마지막 08-18 02:23 (일지 밖, 파손 재현 옵션) | 실물 PETG 출력품이 부러진 그 형상이다(팔 53.5 MPa = PETG 119%). 지금 채택 설계가 아니라 FEA 의 `legacy_u` 로만 남았다. **OEM 클램프 STEP 을 저장소 밖** `%USERPROFILE%/Downloads/hoverboard-motor-shaft-retention-bracket-1.snapshot.6/Hub Motor Shaft Bracket.STEP`(66,959 B)에서 읽는다 — 그 파일이 없으면 import 가 죽는다 |
| `wheel_jig.py` (268줄) | 단륜 시험 지그. 3030 390 mm × 7본 ㄴ자, M8 앵커 4개, 장착 레일 y=+210 을 로봇과 같게 복제. `JIG=skid\|swerve`, `UNIV=1` | 08-01 12:46 (08a) 생성 → 같은 날 (08b) 네 번 깎아 ㄴ자 7본. 08-02 03:16 (09) 축캡 반영이 마지막 | 철물 합계식이 스틸 450원 기준으로 남아 있다(대화에서는 다이캐스팅 4,320원으로 바꿨다). 산출물 `exports/wheel_jig_skid.step` / `_swerve.step` |
| `motor_stand.py` (186줄) | 허브모터 프린트 스탠드 — 축 중심에서 가른 2조각 클램프, 케이블타이 슬롯. `refmodel/motorHolder.step` 의 방식만 가져왔다 | 08-01 17:10 (08b) | 머리 주석의 8.3 MPa / 5.4배 / 앵커 145 N 은 실행 출력(8.5 / 5.3 / 230)과 다르다. 산출물 `exports/motor_stand*.step` |
| `drawings.py` (162줄) | HLR 은선제거 6면 도면 시트 → `docs/images/41~47_skid_*` / `41~47_wheelmount_*`. `SUBJECT=vehicle\|wheelmount`, `HUB_MOCK=1`, `ONE=정면` | 08-07 12:09 (10b) — 사라진 스크래치 sheet.py 를 대신해 새로 짬. 08-12 23:20 (12) SUBJECT 추가 | 도면만은 실물 바퀴가 기본(6면 111초). 주석의 "140초"는 예상값이 남은 것 |
| `export_cnc.py` (106줄) | 판재 휠브라켓(wheel_bracket)을 두께 방향으로 갈라 `exports/cnc/` 에 부품별 STEP 7종 + 평판 DXF 6종. 분해 전후 부피 대조 | 08-04 23:27 (09) | 판재 브라켓 자체가 물러나 지금은 견적 비교용 |
| `iso_render.py` (249줄) | 뷰어 없이 PNG 로 굽는 아이소 렌더(삼각형 z-버퍼, 램버트 음영). `render(assembly, "out.png", skip=…)`, `collect()`, `animate()` | 이 일지 이전에 있었다(07-31 (07)에서 이미 사용). 08-15 08:56 (13) 접지면 원판을 `min(0, 모델 밑면)` 으로 수정 | tslot.py 와 함께 프로젝트 간에 공유되는 둘 뿐인 파일. 08-01 (08a)부터 이 세션의 렌더는 전부 이것 |
| `render.py` (169줄) | 또 다른 오프라인 ISO 렌더러(numpy/PIL, 직교투영 + 실루엣 선) | 파일 시각 07-29 02:12. 이 일지 청크에는 등장하지 않는다(디렉토리 목록에만 보인다) — 어느 세션 것인지 확실하지 않다 |  |
| `example_box.py` (33줄) | 새 모델 템플릿(CLAUDE.md 가 참조) | 07-19 환경 세팅 때 것. 08-10 03:17 수정(일지 밖) |  |

![41_skid_drawing_sheet](../../../hardware/vehicle/docs/images/41_skid_drawing_sheet.png)

위 도면이 `drawings.py` 의 산출물(08-12 23:27 판, 전장 873 × 전폭 712 × 전고 518, ㄷ자 휠마운트 시절)이다. 실행:

```
.venv\Scripts\python.exe models\swerve_base.py                      # 기본 베이스 (DRIVE=skid, PAYLOAD=none)
set PAYLOAD=cargo&& .venv\Scripts\python.exe models\swerve_base.py  # 적재함 포함
set ONLY=프레임,바퀴,휠마운트,휠클램프&& .venv\Scripts\python.exe models\swerve_base.py
.venv\Scripts\python.exe models\wheel_mount.py                      # ㄷ자 마운트 (CNC 판)
set PRINT=1&& .venv\Scripts\python.exe models\wheel_mount.py        # 프린트 판 → exports/print/
.venv\Scripts\python.exe models\drawings.py                          # 도면 41~47 (SUBJECT=vehicle|wheelmount)
.venv\Scripts\python.exe models\export_cnc.py                        # 판재 브라켓 → exports/cnc/
.venv\Scripts\python.exe models\wheel_jig.py                         # JIG=skid|swerve, UNIV=1
.venv\Scripts\python.exe models\motor_stand.py
```

---

## 2. 모델 스크립트 — models/hoverboard_mount/\*.py

08-14 11:18 (13)에 사용자가 정리해 온 프롬프트로 25분 만에 첫 11개 파일이 생겼고, 그 뒤 Design C → D → 통짜 D 로 바뀌면서 파일이 늘었다. 08-15\~08-20 은 GPT/Codex 가 작업한 구간이라 이 일지에 없고, 그때 생긴 파일은 *일지 밖*으로 표시했다. 패키지 관례: z=0 이 축 중심, 데이텀은 타이어 중심면.

| 파일 | 역할 | 만든 날 (청크) | 비고 |
| --- | --- | --- | --- |
| `parameters.py` (307줄) | **치수 원본.** 축 실측(Ø16, D컷 13, 평면 42.8, 축 끝 −78.6), OEM 클램프 42.72×39.22×12.57, EXTRUSION 프리셋(3030/3060/4040/4080), LOW\_\* (Design-D: 92.7×46.6, 블록 24, 다리 24, LOW_SIDE, LOW_BOLT_LEN=15, LOW_CBORE=(16, 32.3)), TIE\_\* (TIE_KIND="profile", TIE_ROD_LEN=400, TIE_PROF_LEN=360), TNUT_PULLOUT 1500(M6 보수 하한), DESIGN_W=800, [실측]/[TODO] 태그 | 08-14 11:20 (13). LOW_SIDE/볼트 역산 08-21 00:30 (14a), TIE\_\* 08-21 01:55 (14b), 400 고정·TONGUE·profile 08-21\~27 (15) | 마지막 08-27 02:01. **미커밋 수정(M)** — 타이 360 vs 390 미결정분과 묶여 있다 |
| `axle.py` (76줄) | `create_axle_profile()`/`axle_solid()` + `plane_at_y()` 헬퍼(Plane.XZ 법선이 −Y 인 함정을 봉인) | 08-14 11:20 (13) |  |
| `motor_reference.py` (45줄) | 모터 참조 형상(간섭 확인용) + `actual_motor_step_reference()` 로 실물 STEP 배치(타이어 중심면 X=(5.85+51.35)/2) | 08-14 (13). 08-15 22:08 수정(일지 밖) | `refmodel/Motor, BLDC, Hoverboard Wheel, 6.5in.stp` 를 읽는다 |
| `split_clamp.py` (68줄) | Design A — 2피스 스플릿 클램프(upper 47.6 + lower 62.0 cm³) | 08-14 11:26 (13) | 2줄 슬롯 프로파일 전용. 3030 에서는 export.py 가 건너뛴다 |
| `torque_plate.py` (71줄) | Design B — 강판 토크 플레이트 50.2 cm³ + 분할 칼라 13.9 cm³ | 08-14 11:37 (13) | 같은 이유로 3030 비대상 |
| `adapter_plate.py` (63줄) | A/B 공용 어댑터 판(2.5D, 108.4 cm³), 카운터보어·T너트 볼트 간격 100 | 08-14 11:36 (13) |  |
| `extrusion_reference.py` (82줄) | 프로파일 참조 단면 — 4면 T슬롯(입구 8.2, 홈 16.4) + 45° 대각 웹 + 보어 Ø6.8. 3030 1.10 / 3060 2.30 / 4040 1.98 / 4080 4.09 kg/m(실물보다 +10\~24%) | 08-14 (13) 생성. 08-15 12:51 (13) "3030 이 아닌 느낌"으로 슬롯 없던 면을 고침 | 이 참조가 틀려 있는 동안 밑면 볼트가 슬롯 없는 면으로 들어가도 검증이 통과했다 |
| `assembly.py` (123줄) | A/B 조립 + 검증(솔리드 1개, 전 조합 간섭, 축 포켓, 케이블, 공구 접근, 최소 두께 3 mm) | 08-14 11:35 (13) |  |
| `export.py` (94줄) | A/B/C/D STEP·STL 내보내기 + 검증 실행. 3030 프리셋이면 A/B 는 건너뛴다 | 08-14 (13). 08-15 02:28 마지막 | `output/step`, `output/stl` 의 대부분을 만든다 |
| `prototype.py` (43줄) | A 의 프린트 fitment 판(포켓 여유 0.05 → 0.35, 플라스틱 탭 하공) | 08-14 11:39 (13) | 산출물 `output/stl/prototype_axle_clamp_*.stl` |
| `saddle_bracket.py` (119줄) | Design C — 3060 세로 통짜 새들 브라켓 76×85×80, 217 cm³, 586 g, 핀치갭 0.4 | 08-14 23:31 (13) | 3060 전용이라 접었다. 09-15 에 돌려도 같은 값 |
| `low_clamp.py` (461줄) | **Design-D 하부 클램프** + OEM 상부 참조 + 3030 레일 + 볼트 참조 형상. `low_clamp()`(92.72×46.60×38.90, 116.3 cm³, 314 g), `report()`/`load_paths()`(경로별 사용률), `tie_force()`, `tie_rods()`/`tie_profile()`/`tie_brackets()`, 어서트(볼트 물림, 타이 탭 벽 2 mm, 물림 8\~14) | 08-15 02:05 (13) 생성, 08:56 통짜. 08-21 (14a 플랜지 제거·볼트 역산, 14b 타이 탭). 08-27 02:26 (15) 프로파일 타이·브래킷 | **미커밋 수정(M)**. 손계산(다리 2.5 / 블록 4.8 MPa)은 FEA 45 MPa 와 10배 차이 — 경로별 사용률용이고 국부 응력은 FEA |
| `drawing.py` (325줄) | Design-D 3면도 PNG/PDF + 낱장 DXF(정면·평면·우측·종합) + 3030 단면 DXF. 치수는 parameters.py 에서, 전고는 bbox 로 | 08-15 13:10 (13) 작성 직후 인증 오류로 미실행. 08-21 03:26 (14b) 플랜지 치수 제거·타이 탭 추가 후 실행. 08-21 07:59 (15) 재실행 | 산출물 `output/drawing/`, `output/dxf/` |
| `run_clamp_fea.py` (752줄) | FEA 파이프라인 본체 — STEP → Gmsh C3D10 → CalculiX 정적 → ccx2paraview VTU → VTK PNG + summary.txt. 모델 `design_d` / `legacy_u` / `legacy_u_nogusset`, 재료 al6061/petg, 편심·방향, 타이를 SPRING1 접지 스프링으로 | 08-15\~20 (일지 밖, GPT/Codex; 결과 폴더 08-18). 08-21 00:38 (14a) 측면 노드 고정을 `P.LOW_SIDE` 로 조건화, 01:43\~ (14b) 타이 스프링·반력 출력. 08-27 02:31 (15) 프로파일 타이 노드 | **미커밋 수정(M)**. 14b 에서 본 타이 노드 1,080개 선택 버그가 고쳐졌는지는 확인하지 못했다 |
| `vehicle_integration.py` (150줄) | 발주 확인용 기준 장면 — 3030 프레임 760×450 + 실제 휠모터 STEP 4 + Design-D 하부 4 + 기존 상부 4 + M6×15 프로파일 볼트 8 + 액슬 타이. 그룹 5\~6개 | 일지 밖 생성(08-15\~20). 08-21 (14a) 볼트 길이·개수를 파라미터 유도로, 08-27 01:16 (15) 레이블 M8x403 버그 → TIE_ROD_LEN 참조 | **미커밋(M)** |
| `vehicle_shell_integration.py` (144줄) | 외장셸(SHELL_Y=255, TOP_RAIL_Y=210, MOUNT=low)까지 포함한 차량. `VEHICLE_PAYLOAD=cargo` 면 적재함도 | 일지 밖 생성. 09-10 10:50 (16) 볼트를 `P.LOW_BOLT_LEN` 으로, `axle_ties()` 추가, 라벨 정리 | **미커밋(M)**. 09-10 결과: 전장 873 × 전폭 612 × 전고 606, 뷰어 38그룹 |
| `vehicle_cargo_integration.py` (11줄) | `VEHICLE_PAYLOAD=cargo` 를 걸고 `vehicle_shell_integration.main()` 을 부르는 래퍼 — "전체 버전" | 08-17 02:16 (일지 밖) | docs/bom.md 가 뷰어 명령으로 이 파일을 적는다 |
| `vehicle_flat_abs_integration.py` (186줄) | 라운드 셸을 평판 ABS 패널로 바꾼 비교 장면 | 08-17 03:00 (일지 밖) | 이 세션에서 다룬 기록 없음 |
| `assembly_animation.py` (89줄) | 조립 순서 애니메이션을 뷰어로(하부 → 축 → 상부 → 3030 → 볼트) | 08-15 23:48 (일지 밖) | 머리 주석의 "측면 M6x20 4개"는 플랜지 있던 시절 문구 |
| `render_assembly_gif.py` (113줄) | 같은 순서를 iso_render 로 오프라인 GIF → `output/animation/hoverboard_clamp_assembly.gif` | 08-16 00:31 (일지 밖) |  |
| `load_visualization.py` (56줄) | 보수적 정역학 하중 경로를 화살표 형상으로 | 08-15 23:10 (일지 밖) | FEA 아님 |
| `load_analysis_viewer.py` (226줄) | 하중 경로 설명용 뷰어 장면(조립·하중·반력·위험 구역을 그룹으로) | 08-18 00:29 (일지 밖) | FEA.md 첫 줄이 "이것은 FEA 결과가 아니다"라고 못박는다 |
| `bent_lower_clamp.py` (196줄) | Design-D 를 SS400 6 mm 절곡 강판(새들 U + 레일 햇 + 2.5 mm V 라이너)으로 대체하는 안 | 08-16 17:03 (일지 밖) | 채택되지 않았고 이 세션에서 언급 없음 |
| `export_low_clamp_step.py` (45줄) | `output/step/low_clamp_design_d.step`(솔버가 읽는 STEP) 재생성 + 되읽기 부피 검증 | 08-16 21:31 (일지 밖) | CLAUDE_HANDOFF §2 재생성 명령 |
| `export_clamp_stl.py` (68줄) | 프린트용 STL 2종(하부 클램프, OEM 상부 참조)을 최저면 z=0 으로 + 바이너리 STL 구조 검증 | 08-19 23:31 (일지 밖) | 산출물 `output/stl/design_d_lower_clamp_print.stl` (92.72 × 46.60 × 38.90, 4,332 삼각형) |
| `export_legacy_ungusseted.py` (36줄) | wheel_mount 를 `NO_GUSSET=1 U_OPEN_INBOARD=1` 로 불러 부러진 실물 그대로의 STEP 을 내보낸다(FEA 재현용) | 08-18 02:23 (일지 밖) | 산출물 `output/step/legacy_u_bracket_no_gussets_open_inboard.step` |

```
.venv\Scripts\python.exe models\hoverboard_mount\export.py                # A/B/C/D STEP·STL + 검증
.venv\Scripts\python.exe models\hoverboard_mount\export_low_clamp_step.py  # 솔버용 STEP
.venv\Scripts\python.exe models\hoverboard_mount\export_clamp_stl.py       # 프린트 STL 2종
.venv\Scripts\python.exe models\hoverboard_mount\drawing.py                # 도면 PNG/PDF + DXF
.venv\Scripts\python.exe models\hoverboard_mount\vehicle_integration.py    # 발주 기준 장면
.venv\Scripts\python.exe models\hoverboard_mount\vehicle_cargo_integration.py  # 셸+적재함 전체 버전
set EXTRUSION=3060&& .venv\Scripts\python.exe models\hoverboard_mount\export.py   # 프리셋 바꿔 보기
set LOW_SIDE=1&& ...                                                       # 옆 플랜지 되살리기 (기본 꺼짐)
```

![41_wheelmount_drawing_sheet](../../../hardware/vehicle/docs/images/41_wheelmount_drawing_sheet.png)

위 그림은 `models/drawings.py` 의 `SUBJECT=wheelmount` 산출물(08-12 23:29)로, 그때의 `mount_low()` 76×40×20 시점이다. 채택된 Design-D(92.7×46.6×38.9)의 도면은 `models/hoverboard_mount/output/drawing/low_clamp_drawing.png` 쪽이다.

---

## 3. 도구 — tools/\*\*

| 파일 | 역할 | 만든 날 (청크) | 비고 |
| --- | --- | --- | --- |
| `tools/fea/run_clamp_fea.ps1` (49줄) | FEA 실행기. `.venv` 를 확인하고 `models/hoverboard_mount/run_clamp_fea.py` 를 부른다. 인자 -Model / -Material / -Load / -MeshSize / -Eccentricity / -LoadDirection | 08-18 02:19 (일지 밖). 08-21 (14a·14b·15)에서 실행 | 기본 Eccentricity 0 / downward 라 **편심·방향을 빼면 편심 0 케이스가 돈다**(CLAUDE.md) |
| `tools/fea/structural_stress.py` (56줄) | VTU 를 읽어 볼트 구멍 벽 3 mm 밖의 최댓값(구조응력)을 뽑는다. 기본 3케이스 | 08-21 00:42 (14a) | 머리말(8\~9행)과 FEA.md 112행의 실행 예가 `tools<폼피드>ea\structural_stress.py` 로 깨져 있다 — 파일 안에 실제로 0x0C 바이트가 들어 있다(`cat -A` 로 보면 `tools^Lea`). `tools\fea` 의 `\f` 가 파이썬 문자열 이스케이프로 먹힌 것으로, 같은 밤 tipping_moment.py 에서 겪은 현상과 같다. 그 줄을 그대로 붙여 넣으면 실행되지 않는다 |
| `tools/fea/calculix/`, `calculix_2.23_4win.zip`, `gmsh_pyenv/`, `python/` | 외부 바이너리 — CalculiX 2.23(ccx_static.exe), Gmsh 4.15.2, meshio·ccx2paraview 3.2.0·numpy | 08-12 21:19 (12)에 "gmsh 는 pip 로 설치 가능"까지만 조사, 실제 설치는 일지 밖(08-15\~20) | 약 460 MB, `.gitignore` 로 제외 |
| `tools/figures/tipping_moment.py` (144줄) | 젖힘 모멘트 설명 그림 → `docs/images/63_why_tipping_moment.png`, `64_tipping_animation.gif`(48프레임 14 fps). 실제 좌표 1:1 Y-Z 단면, 회전 중심은 레일 바깥 밑모서리 | 08-21 01:19\~01:37 (14a) | 경로를 백슬래시로 적었다가 `\50`·`\f` 이스케이프로 파일명·주석이 깨진 뒤 pathlib 로 바꿨다 |
| `tools/frame/frame_fea.py` (188줄) | 차량 프레임 12자유도 3차원 보요소 솔버 — 3030 부재(E=69,000), 클램프 강체 링크, 타이 강철 트러스, 지지는 바퀴 접지점 4곳. 하중 3케이스(중앙 / 한쪽 구석 / 뒤축 위) | 08-21 03:14 (14b) | 구석 하중에서 바퀴 865 N → 설계하중 800 N 의 근거. 09-15 에 돌려도 처짐 0.078 / 0.379 / 0.161 mm |
| `tools/frame/frame_view.py` (119줄) | 위 결과 시각화 → `docs/images/65_frame_load_cases.png`, `66_frame_corner_load.gif` | 08-21 03:18 (14b) | 히어독 패치가 세 번 어긋나 Write 로 통째 다시 썼다 |
| `tools/bom/bom.py` (531줄) | 전체 차량 BOM/예상 견적. swerve_base 를 `NOSHOW=1 NOEXPORT=1` 로 임포트해 GROUPS 의 수량·부피·질량을 읽고, 절단은 tslot 과금 규칙, 단가는 맨 위 PRICE 표(확인 태그 4개만 실주문값). 조달 보유/출력/구매, PRINT_JOBS(출력 시간, 베드 220×220×250 초과 검사), 판재 대안 표 → `docs/bom.md`, `docs/bom.csv` | 09-10 10:57\~11:03 (16) | **미추적(??)**. 환경변수를 vehicle_shell_integration 과 같게 맞춰(MOUNT=low, PAYLOAD=cargo, TOP_RAIL_Y=210, SHELL_Y=255) 뷰어와 견적이 갈리지 않게 했다 |
| `tools/trilium/push_journal.py` (191줄) | `docs/journal/manifest.json` 을 읽어 Trilium ETAPI(기본 http://127.0.0.1:37840)에 일지 트리를 올린다. 토큰은 `TRILIUM_ETAPI_TOKEN` 또는 `~/.trilium-etapi-token` | 09-15 19:14 (16 의 후속) | **미추적(??)**. `docs/journal/` 폴더는 09-15 저녁 현재 저장소에 아직 없다 — 이 일지 자체가 그 입력이다 |
| `tools/fbxread.py` | 바이너리 FBX 파서 | 09-14 (일지 밖) | 다른 프로젝트(갤럭시 워치 refmodel)용. 이 지도 밖 |

```
powershell -ExecutionPolicy Bypass -File tools\fea\run_clamp_fea.ps1 -Model design_d -Material al6061 -Load 800 -MeshSize 3 -Eccentricity 55 -LoadDirection upward
powershell -ExecutionPolicy Bypass -File tools\fea\run_clamp_fea.ps1 -Model legacy_u_nogusset -Material petg -Load 800 -MeshSize 3 -Eccentricity 55 -LoadDirection upward
.venv\Scripts\python.exe tools\fea\structural_stress.py [폴더명 ...]
.venv\Scripts\python.exe tools\figures\tipping_moment.py
.venv\Scripts\python.exe tools\frame\frame_fea.py
.venv\Scripts\python.exe tools\frame\frame_view.py
.venv\Scripts\python.exe tools\bom\bom.py
.venv\Scripts\python.exe tools\trilium\push_journal.py --dry-run
```

![65_frame_load_cases](../../../hardware/vehicle/docs/images/65_frame_load_cases.png)

---

## 4. 문서 — docs/\*\*, \*.md

| 파일 | 역할 | 만든 날 (청크) | 비고 |
| --- | --- | --- | --- |
| `docs/build-log.md` (1,630줄, 80,084 B) | 설계기록. 0장 + 1\~28막 + 29\~32절(29 틀렸던 것들 / 30 방법론 / 31 최종 사양 / 32 예상 제작비) + 영상 챕터(25개) + 도면 색인 | 07-28 01:25 (02b) 368줄로 시작 → 07-29 (04) 676 → 07-29 (05) 762 → 07-30 (06) 1,034\~1,077 → 07-31 (07) 1,103 → 08-07 (10b) 스키드 치환 → 08-10 (11) 21\~28막 추가로 1,630 | 마지막 수정 08-10 00:14. **28막 + 29\~32절이 끝**이라 08-10 이후(적재함 v2\~v4, 실물 파손, hoverboard_mount, FEA, 타이, BOM)는 여기 없다. 과거는 고치지 않고 뒤에 붙이는 규칙 |
| `docs/payload-interface.md` (307줄) | 페이로드 슬라이드 인터페이스 규격 v1 — 데크 레일 3본(y=−200/0/+200, x −405\~+380), 채널·턱, 그로밋, 전원, 한계 60 kg / CG 700, 모듈 5종, 3.1 아웃리거 | 07-30 02:29 (05) 159줄 → 07-30\~31 (06) 258→279 → 07-31 (07) 307(아웃리거) → 08-07 (10b) 스키드 치환 | 마지막 08-07 12:27. 적재함 항목은 아직 높이 360 / 전고 840 시절 값 |
| `docs/bom.md` (163줄), `docs/bom.csv` (57줄) | bom.py 산출물. 요약(보유 471,840 / 출력 563,256 / 구매 941,382 / 신품 합계 1,976,478 / 추가 지출 1,504,638원), 카테고리 7, 절단 목록, 출력 14,600 cm³ · 406시간, 베드 초과 14건, 판재 대안 | 09-10 11:02 (16) | **미추적(??)**. 스프링너트 933 N / 드롭인 T너트 2271 N [추정] 이 parameters.py 의 1500 N 과 다르다 |
| `docs/interactive/lever.html` (303줄, 15,795 B) | 슬라이더로 팔 길이를 바꾸면 모멘트·볼트 인발·기울기가 따라오는 설명 페이지. 브라우저로 그냥 열면 된다 | 08-21 01:19 아티팩트 → 01:37 로컬 저장 (14a) | 숫자가 손으로 박혀 있어 형상·너트 한계(1500 N)가 바뀌면 직접 고쳐야 한다 |
| `docs/interactive/README.md` (37줄) | lever.html 의 출처 표와 이미지 63/64 재생성 방법 | 08-21 01:37 (14a) |  |
| `docs/images/` (112장) | 렌더·도면 보관. 이 주제는 `01_`\~`66_` 번호 파일 80장(41\~47 은 skid / swerve / wheelmount 세 벌, 58 은 deployed / stowed 두 장, 59 는 61 로 덮여 없다) | 아래 표 | 나머지 32장(`ball_robot_*`, `gring_*`, `ib_*`, `sphex_*`)은 다른 프로젝트 |
| `docs/ball_robot_todo.md` | 다른 프로젝트(볼 로봇) | 07-31 (일지 밖) | 이 지도 밖 |
| `CLAUDE.md` (70줄) | 프로젝트 규칙 — 환경(.venv 정상), 실행 워크플로, build123d 작성 규칙, FEA 절(기준 결과 45.3 MPa / 0.0582 mm, 최대 von Mises 인용 금지, 실물 파손 재현 조건) | 07-19 세팅 때 것. 08-21 00:17\~00:42 (14a) 환경 절·FEA 절 갱신 | 기준값 45.3 / 0.0582 는 지금 어느 결과 폴더의 summary.txt 와도 맞지 않는다(폴더가 다른 케이스로 덮였다) |
| `models/hoverboard_mount/README.md` (283줄) | 패키지 설명 — 실측 항목 12개 표, 네 설계안(D/C/A/B) 비교, 800 N 사용률, 조립 순서, 파일 목록 | 08-14 11:45 (13) 생성, 08-14 23:31 "세 설계안", 08-15 (13) Design D. 마지막 08-15 20:28 | Design D 절이 **플랜지 있는 63.9 mm 형상(볼트 6개, "끄지 마라")** 기준이라 08-21 이후 상태(플랜지 제거)와 어긋난다. 파일 목록도 첫 11개만 |
| `models/hoverboard_mount/FEA.md` (281줄) | FEA 인수인계 — 파이프라인, 실행법, 경계조건·재료, 기준 결과(08-21 갱신), 편심 0 옛 기록, legacy_u 비교, 실물 휨 재현, 코드 구현 주의 5가지, 허용중량 환산식 | 일지 밖(GPT, 08-15\~20; 08-21 00:17 에 226줄). 08-21 (14a) §4 갱신·조건 대조표 | 112행 실행 예에 폼피드 바이트(`tools^Lea`)가 들어 있다 |
| `models/hoverboard_mount/CLAUDE_HANDOFF.md` (363줄) | 전체 인수인계 — 채택 설계, 실물 시험(08-21) 반영, 중요 파일, 재생성 명령, 프레임 상태(390 mm 8본), 뷰어 그룹, 조립 순서, §5.0 액슬 타이, §5.1 설명 자료, FEA 파이프라인·결과·한계·안전 원칙 | 일지 밖(GPT; 08-21 00:17 에 280줄). 08-21 (14a·14b) 갱신, 최종 갱신 표기 2026-08-21 | §5.0 이 아직 "Ø8 × 397.4 mm, 물림 12×2" — 08-21 07:57 (15) 에 400 / 탭 15 로 바꾼 것이 반영되지 않았다. TIE_KIND=profile 도 없다 |
| `SETUP.md` (61줄) | 다른 컴퓨터에 환경을 만드는 법(Python 3.12, build123d 0.11.1, ocp_vscode 4.0.0) | 07-19 (일지 이전) |  |
| `requirements.txt` | build123d==0.11.1, ocp_vscode==4.0.0, cq-editor==0.7.0 | 07-19 (일지 이전) |  |
| `.gitignore` (38줄), `.gitattributes` (12줄) | "소스와 문서만 담는다". exports/·refmodel/·tools 바이너리·FEA 중간파일 제외, hoverboard_mount 발주 산출물은 예외. LF 통일, 형상·이미지는 -text | 08-21 08:08\~08:12 (15) 초기 커밋 14fb653 과 함께 | CRLF 로 low_clamp.py 가 두 번 깨진 것이 계기 |
| `ocp-viewer-*.log` (6개) | 뷰어 백그라운드 실행 로그 | 여러 날 | 본문 없는 0\~310 B. `ocp-viewer-stdout.log` 는 추적 중이라 수정(M)으로 잡힌다 |

**docs/images — 이 주제 80장의 출처**

| 번호 | 내용 | 만든 날 (청크) | 만든 코드 |
| --- | --- | --- | --- |
| 01\~19 | tslot 단면·프레임, mobility_base v1\~v7(2층, 커버, 얼굴, 벤트, 배터리) | 07-27 (01a·01b), 07-28 01:37 (02b) 저장 | 당시 스크래치 matplotlib HLR 렌더 — 코드 없음 |
| 20\~21 | SPG 카탈로그 PDF 토크 표(참고 이미지) | 07-27 23:46 (02b) | PyMuPDF 렌더 |
| 22\~25 | 스커트 / 펜더 | 07-28 01:45 (02b) | 스크래치 |
| 26\~33 | 스워브 v1, 셸, 얼굴, 모듈·서스펜션 | 07-28 (03) | 스크래치 |
| 34\~40 | --\\\_\_/-- 스커트, Xtion, 셸 플랜지·크래들, 후면 포트 | 07-29 00:03\~02:01 (04) | 스크래치 |
| 41\~47_swerve | 스워브 기본 베이스 6면 + 시트 | 07-31 09:38 (06) 재생성판 | 스크래치 sheet.py — **분실**(28막) |
| 48\~53 | 적재함 v1 슬라이드·내부·뚜껑, 핸들 | 07-29\~30 (05) | 스크래치 |
| 54\~57 | 평판 / 센서 타워 / 팔 마운트 / 계측 모듈 | 07-31 09:53 (06) | 스크래치 modsheet.py — 분실 |
| 58 ×2, 60, 61, 62 | 아웃리거, 실물 허브모터, 핸들·스커트, 모듈 작업대 | 07-31 \~ 08-01 (07) | iso_render 계열 스크래치 |
| 41\~47_skid | 스키드 기본 베이스 6면 + 시트(873×712×518) | 08-12 23:27 (12) 판 | `models/drawings.py` |
| 41\~47_wheelmount | 하단 클램프안(76×40×20 시점) 6면 | 08-12 23:29 (12) | `models/drawings.py SUBJECT=wheelmount` |
| 63, 64 | 젖힘 모멘트 3단 그림, GIF | 08-21 01:37 (14a) | `tools/figures/tipping_moment.py` |
| 65, 66 | 프레임 하중 3케이스, 구석 하중 GIF | 08-21 03:18 (14b) | `tools/frame/frame_view.py` |

![63_why_tipping_moment](../../../hardware/vehicle/docs/images/63_why_tipping_moment.png)

---

## 5. 산출물 — exports/, models/hoverboard_mount/output/

`exports/` 는 통째로 `.gitignore` 대상이라 **하나도 추적되지 않는다.** 반면 `models/hoverboard_mount/output/` 은 발주 확정본이라 58개(STEP 15 · STL 11 · DXF 5 · PNG 12 · summary.txt 11 · md 2 · pdf 1 · gif 1)가 추적된다. FEA 중간파일(.inp/.frd/.vtu/.dat/.msh, 케이스당 수십 MB)은 제외다.

**exports/ (이 주제분)**

| 파일 | 무엇 | 만든 날 (청크) | 비고 |
| --- | --- | --- | --- |
| `tslot_frame.step` (2.35 MB) / `.stl` (0.92 MB) | 2020 400×300×400 예제 프레임 | 07-27 18:36 (01a) | 이날 판 그대로 |
| `mobility_base.step` (7.45 MB) / `.stl` (15.5 MB) | 첫날(07-27) 스키드 베이스(PVC 바퀴 판) | 원래 07-28 17:48 (03) 판이었으나 현재 파일 시각은 09-15 19:51 — 일지 검수 중 재실행된 것으로 보인다 | 07-27 밤에 내보낸 판들은 남아 있지 않다 |
| `swerve_base.step` (26 MB) | 차량 본체(스키드 전환 직후) | 08-01 16:49 (08a·08b) | 그 뒤 모델이 바뀌었으나 다시 내보내지 않았다 |
| `swerve_module.step` (21.6 MB) | `MODULE_ONLY=1` 모듈 작업대(프레임 + 바퀴 + 스워브 모듈) | 08-01 00:07 (07) |  |
| `skid_base.step` (15 MB) | 스키드 차량 | 08-01 (08a) 첫 판, 08-13 00:09 (12) 다시 씀 |  |
| `wheel_jig_skid.step` (1.6 MB) / `wheel_jig_swerve.step` (6.1 MB) | 단륜 지그(ㄴ자 7본) | 08-01 16:45 / 16:49 (08b) | 08-02 (09) 축캡 반영 뒤 재내보내기는 없다 |
| `motor_stand.step` / `motor_stand_body.step` / `motor_stand_cap.step` | 프린트 스탠드(1조각 판 / 2조각 본체·캡) | 08-01 17:23 (08b) / 08-10 21:43 (11) |  |
| `exports/cnc/` 클램프판_안쪽·바깥_8t, 타이판_8t, 플랜지_8t, 브리지블록_20t, 축캡_24t, 위치스페이서_5t (STEP 7 + DXF 6) | 판재 휠브라켓 분해 발주 파일 | 08-04 23:28 (09), `export_cnc.py` | 설계 자체가 물러났다 |
| `exports/cnc/휠마운트_cnc_M6.step/.stl`, `휠마운트_끼움쿠폰_cnc_M6.stl`, `휠마운트_정면_XZ/측면_YZ/평면_XY_cnc_M6.dxf` | ㄷ자 휠마운트 CNC 판(거싯 포함 70×40×176.3) | 08-07 (10a·10b) 첫 판, 08-12 22:39 (12) 거싯 판으로 재생성 | FEA `legacy_u` 가 이 STEP 을 읽는다 |
| `exports/print/휠마운트_print_M6.step/.stl`, `휠마운트_끼움쿠폰_print_M6.stl` | ㄷ자 휠마운트 프린트 판(눕혀서 z=0, 슬롯 30.8, Ø7.2) | 08-07 04:03 (10b) | 실제로 이 파일을 뽑아 달았고 08-12 에 부러졌다(거싯 없는 판) |

**models/hoverboard_mount/output/**

| 경로 | 무엇 | 만든 날 (청크) | 만든 코드 |
| --- | --- | --- | --- |
| `step/low_clamp_design_d.step` (89 KB) | **솔버가 읽는 Design-D STEP**(92.72×46.60×38.90, 116.3 cm³) | 08-16 (일지 밖) 첫 판, 08-21 00:36 (14a) 플랜지 제거판, 08-27 02:31 (15) 타이 탭판 — **미커밋(M)** | `export_low_clamp_step.py` |
| `step/low_clamp.step`, `saddle_bracket.step`, `adapter_plate.step`, `axle_clamp_upper/lower.step`, `axle_retainer.step`, `torque_plate.step`, `hoverboard_motor_reference.step`, `3030_extrusion_reference.step` | D / C / A·B 부품과 참조 형상 | 08-14\~15 (13) 첫 판, 08-21 07:59 (15) 재생성 | `export.py` |
| `step/assembly_A.step`, `assembly_B.step`, `4080_extrusion_reference.step` | A/B 조립체(4080 기준) | 08-15 02:27 (13) | `export.py` (EXTRUSION=4080 시절) |
| `step/legacy_u_bracket_no_gussets.step`, `_open_inboard.step` | 부러진 실물 그대로의 구형 ㄷ자(거싯 없음, 열린 면 안쪽) | 08-18 02:13 / 02:23 (일지 밖) | `export_legacy_ungusseted.py` |
| `stl/design_d_lower_clamp_print.stl`, `oem_upper_clamp_reference_print.stl` | 프린트용 STL(최저면 z=0) | 08-19 (일지 밖) 첫 판, 08-21 00:36 (14a) 38.9 mm 판 | `export_clamp_stl.py` |
| `stl/prototype_axle_clamp_upper/lower.stl` | Design A 프린트 fitment 판 | 08-14 12:23 (13) | `prototype.py` |
| `stl/` 나머지 7개 | D / C / A·B 부품 STL | 08-21 07:59 (15) | `export.py` |
| `drawing/low_clamp_drawing.png/.pdf` | Design-D 3면도(타이 탭 포함) | 08-21 03:26 (14b) 첫 실행, 08-21 07:59 (15) 재생성 | `drawing.py` |
| `dxf/low_clamp_front/top/right/views.dxf`, `3030_profile.dxf` | 낱장 DXF(정면 31 / 평면 32 / 우측 43 / 종합 106 엔티티) + 3030 단면(45선) | 08-21 (14b → 15) | `drawing.py` |
| `animation/hoverboard_clamp_assembly.gif` (134 KB) | 조립 순서 GIF | 08-16 00:31 (일지 밖) | `render_assembly_gif.py` |
| `fea/comparison_800N.md` | Design-D(플랜지 있던 137.0 cm³) 대 구형 ㄷ자 202.8 cm³ 비교 보고(편심 0, AL) | 08-18 (일지 밖) | 수기 |
| `fea/legacy_failure_reproduction.md` | 구형 PETG 실물 휨 재현 보고(3 mm 44.373 MPa / 2.979 mm, 바깥 밀림 2.819 mm) | 08-18 (일지 밖) | 수기 |
| `fea/clamp_800N_h3mm/`, `clamp_800N_h4mm/` | 편심 0 · 하향 · 플랜지 있던 형상 · AL (4.036 / 3.613 MPa) | 08-18 00:46\~00:48 (일지 밖) | `run_clamp_fea.py` |
| `fea/legacy_u_800N_h3mm/`, `_h4mm/` | 구형 ㄷ자(거싯 있음) 편심 0 · AL (15.816 MPa / 0.009374 mm) | 08-18 01:46\~01:48 (일지 밖) |  |
| `fea/legacy_u_nogusset_800N_h{3,4}mm_petg_e55mm{,_up}/` (4개) | 실물 파손 재현. 인용하는 것은 `_h3mm_petg_e55mm_up`(44.373 MPa / 2.979 mm) | 08-18 02:16\~02:26 (일지 밖) |  |
| `fea/clamp_800N_h4mm_e55mm_up/` | M8×2 타이 · AL · 4 mm (90.3 MPa / 0.0372 mm / Fy 553 N) | 08-21 02:32 (14b) | summary.txt 가 그 실행 그대로 |
| `fea/clamp_800N_h3mm_e55mm_up/` | 폴더명은 기준 케이스지만 **플랜지 복원 + 타이 실행(18.0 MPa / 0.0076 mm, 구멍 6개, 타이 노드 1,080**)으로 덮여 있다 | 08-21 02:51 (14b) | CLAUDE.md 기준값 45.3 / 0.0582 는 이 폴더에 없다 — 결과 폴더 이름이 LOW_SIDE/TIE 를 담지 않는다 |
| `fea/clamp_800N_h3mm_petg_e55mm_up/` | PETG · 3030 프로파일 타이(26.5 MPa / 0.2627 mm / Fy 1,268 N) | 08-27 02:32 (15) — **미커밋(M)** | 08-21 의 PETG 무보강(45.6 / 1.9067)·M8 타이(27.3 / 0.2659) 결과는 덮여서 문서에만 남았다 |

결과 폴더의 구성은 어느 케이스든 같다: `clamp_mesh.msh`, `*.inp`, `*.frd`, `*.dat`(반력), `*.vtu`, `stress_deformed_800N.png`, `solver_stdout.log`, `summary.txt`. 추적되는 것은 PNG 와 summary.txt 뿐이다.

---

## 6. 참조 — refmodel/

| 파일 | 무엇 | 들어온 날 (청크) | 쓰는 코드 |
| --- | --- | --- | --- |
| `Motor, BLDC, Hoverboard Wheel, 6.5in.stp` (1.06 MB) | 실물 호버보드 6.5인치 휠모터. 실측 Ø158.74 × 55, 축 Ø16 D컷 13, 평면 −46.5\~−3.7(42.8 mm), 1 mm 모따기로 끝나고 나사부 없음, 솔리드 1개 958 cm³ | 07-31 19:58 (07) 사용자가 넣음. 0.1 mm 스캔은 08-01 (08a·09) | `swerve_base.py`, `wheel_mount.py`, `hoverboard_mount/motor_reference.py` |
| `RPI5+Pimoroni-NVME+Waveshare-POE v1.step` (25.2 MB) | 라즈베리파이 5 실물(솔리드 516개, 58.1 × 90.0 × 36.5) | 07-27 20:58 (02a) | `mobility_base.py`(REAL_PARTS), `swerve_base.py`(PI_DETAIL, 기본 꺼짐 — 간섭 검사가 10분을 넘겨서) |
| `ASUS Xtion Pro Live.step` (638 KB) | 전면 깊이 카메라(180.0 × 41.64 × 36.19, 솔리드 2) | 07-28 23:54 (04) | `swerve_base.py` — bbox 를 런타임에 읽는다(반올림 상수로 0.3 mm 물렸던 뒤) |
| `Servo MG996R.STEP` (730 KB) | 조향 서보 후보(53.6 × 47.6 × 20.0) | 07-29 01:19 (04) | 스워브 보류로 코드에서 쓰지 않는다 |
| `motorHolder.step` (252 KB) | 세로축 모터 홀더 참고(97×86×30) | 08-01 17:09 (08b) | `motor_stand.py` 가 방식만 참고 |
| `X6-7/`, `X6-8/`, `X6-7.zip`, `X6-8.zip` | RMD-X6 액추에이터 STEP·PDF | 07-29 21:26 | 이 일지 청크 어디에도 나오지 않는다. 용도 확실하지 않다 |
| `Cramped Robot Wheel.step`(+render.png), `_downloaded/`(25D 기어모터 13종, LD19 라이다, Pico), `_grabcad/`(N20, 스위블 캐스터, 진공 임펠러·브러시)와 그 zip | 07-30 01:53\~14:05 에 들어온 참조들 | 07-30 | 이 세션 청크에는 디렉토리 목록으로만 보인다. 다른 모델(robot_vacuum, wheeled_base 등) 용으로 보이나 확실하지 않다 |
| `28BYJ-48_Stepper.STEP`, `gwatch/`, `model-s26-ultra.step`, `Samsung Galaxy S26 Ultra Case.3mf`, `samsung-galaxy-s26-ultra-phone-holder.stl` | 다른 프로젝트 참조 | 08-31 \~ 09-14 | 이 지도 밖 |

![60_hub_motor_real](../../../hardware/vehicle/docs/images/60_hub_motor_real.png)

저장소 밖에 있지만 코드가 의존하는 참조가 하나 더 있다. **OEM 클램프 STEP** `%USERPROFILE%/Downloads/hoverboard-motor-shaft-retention-bracket-1.snapshot.6/Hub Motor Shaft Bracket.STEP`(66,959 B, 2022-06-08 GrabCAD 스냅샷, 08-07 01:23 압축 해제)을 `wheel_mount.py` 가 직접 읽는다. 외형 42.72 × 39.22 × 12.57 은 실측이고 구멍 Ø8.128 은 0.3200" 라운드 인치값이라 실측이 아니다(10b). `hoverboard_mount/` 쪽은 이 STEP 을 읽지 않고 parameters.py 의 OEM_W/H/T 상수로 Box 를 그린다. 같은 Downloads 폴더의 zip 안에 원본이 있으니 지워져도 복구는 된다.

---

## 7. 이 지도 밖 — 같은 저장소의 다른 프로젝트

이 세션 청크에서는 디렉토리 목록으로만 보이고 작업 기록이 없는 것들이다. 시각은 파일 mtime 이다.

- `models/`: ab_task1_bracket, ab_task2_enclosure(08-06, Fusion A/B 킷), ball_robot·\_control·\_sweep(07-31), box_robot(08-12), corner_shophouse, coupler_28byj48_hex3, mic_stand(07-19\~26, 일지 이전 네 모델), gear_ring(07-31), iris_ball(07-31), robot_vacuum(07-31), sphere_hexapod(07-30), upper_dome, vibe_deck·\_case·\_layout(07-31), wheeled_base(07-30)
- `models/` 하위 패키지(전부 미추적): card_press, card_stamper, galaxy_watch4_classic, nfc_turntable, nfc_turntable_mini, phone_tablet, pillbox, pillbox_cassette, pillbox_roll, pillbox_v54, pouch_dispenser, power_button (08-17 \~ 09-14)
- `exports/`: 위 프로젝트들의 STEP/STL/PNG 와 하위 폴더 — 그중 `robot_vacuum.step` 131 MB 가 git 을 617 MB 로 만들었던 파일
- `docs/images/` 32장(ball_robot\_\*, gring\_\*, ib\_\*, sphex\_\*), `docs/ball_robot_todo.md`, `tools/fbxread.py`, `.claude/launch.json`(pouch_dispenser 웹 서버 설정)

---

## 8. 지금 상태에서 알아둘 것

- **미커밋이 아홉 파일**이다(09-15 git status): `parameters.py`, `low_clamp.py`, `run_clamp_fea.py`, `vehicle_integration.py`, `vehicle_shell_integration.py`, `output/step/low_clamp_design_d.step`, `output/fea/clamp_800N_h3mm_petg_e55mm_up/`(summary·PNG)이 수정(M), `tools/bom/`·`docs/bom.md/.csv`·`tools/trilium/` 가 미추적(??). 09-10 (16)에 "타이 길이 360 vs 390 미결정분과 섞이면 안 된다"며 일부러 커밋하지 않았다. 원격 저장소는 없다.
- 같은 값이 **세 곳에 다르게** 적혀 있다. 타이 봉 길이: parameters.py 400 / CLAUDE_HANDOFF §5.0 397.4. 타이 종류: parameters.py TIE_KIND=profile(3030×360) / CLAUDE_HANDOFF M8×2 / bom.md 3030 L360 + PLA 브래킷 8. 스프링너트 인발: parameters.py 1500 / bom.md 933 [추정]. 어느 쪽이 최종인지는 저장소만으로는 확실하지 않다.
- FEA 기준값(45.3 MPa / 0.0582 mm)은 **문서에만** 있다. 결과 폴더는 조건이 바뀌면 같은 이름으로 덮인다.
- build-log.md 는 08-10 이후를 담지 않는다. 그 뒤의 기록은 hoverboard_mount 의 md 세 개와 이 일지뿐이다.
- 스크래치 폴더에서 돌린 검사·렌더 스크립트(verify\*.py, sheet.py, modsheet.py, stroke.py, mass.py, cargo_bom.py, an.py, br.py, engage.py, jigcheck.py, hw2\~hw7, geom.py, cap.py 등 수십 개)는 **하나도 남아 있지 않다.** 그 손해가 28막 "도면 스크립트가 사라져 있었다"이고, 그 뒤로는 `models/drawings.py`, `tools/frame/`, `tools/figures/`, `tools/bom/` 처럼 저장소 안에 두었다.

## 교훈

- 산출물이 아니라 산출하는 코드를 남긴다. 이미지 41\~47 만 남고 sheet.py 가 사라지자 모델이 바뀐 뒤에도 아무도 도면을 못 고쳤다(10b).
- 원본은 한 곳에 둔다. 휠 마운트는 wheel_mount.py, 치수는 parameters.py — 두 벌로 그리면 반드시 어긋난다. 라벨·문서에 박아 둔 상수(M6x20, 397.4, M8x403)는 실물이 바뀌어도 그대로 남았다(14a·15·16).
- 결과 폴더 이름에 조건(LOW_SIDE, TIE)이 없으면 덮어쓰기로 기준값이 사라진다. 인용할 값은 조건과 함께 문서에 적는다(14a·14b).
- 저장소 밖 경로(Downloads 의 클램프 STEP)에 의존하는 코드는 그 사실을 파일 지도에 적어 둔다.
- git 은 소스·문서·발주 확정본만. exports/ 는 재생성 가능하고 통째로 넣으면 617 MB 였다(15).
