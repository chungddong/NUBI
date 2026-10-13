# 휠 마운트 작업대 — 3030 레일 한 토막 + 실물 허브모터만 띄운다.
#
# 바퀴를 프레임에 어떻게 매달지 다시 잡기 위한 최소 무대다. swerve_base 를 통째로
# 부르면 2분이 걸려서, 여기서는 STEP 만 직접 읽는다.
#
# 좌표 — **로봇에서의 실제 상대 위치 그대로**다. 이걸 바꾸면 여기서 만든 게
# 로봇에 안 맞는다.
#   지면 z=0 / 타이어 중심면 y=0 / 축 z=79.35
#   하부 사이드레일: 중심 y=-50, z=185.7  (= 로봇의 y=210, z=185.7 를 옮긴 것)
#   → 레일 밑면 z=170.7 에서 축까지 **91.35mm**. 이 틈을 무엇으로 채울지가 과제다.

import math
import os
from pathlib import Path

from build123d import *
from ocp_vscode import Camera, set_port, show

from tslot import PROFILES, bar

SERIES = 30
S = PROFILES[SERIES]["size"]

HUB_STEP = Path(__file__).parent.parent / "refmodel" / "Motor, BLDC, Hoverboard Wheel, 6.5in.stp"
WHEEL_D = 158.7               # 타이어 외경 (STEP 실측)
AXLE_Z = WHEEL_D / 2
TIRE_X = (9.2, 48.0)          # 타이어 구간 (STEP X) — 이 중심이 접지 중심면
TIRE_C = sum(TIRE_X) / 2

# 축 실측 (0.1mm 스캔). **나사부가 없다** — Ø16 D컷이 1mm 모따기로 그냥 끝난다.
SHAFT_D, FLAT = 16.0, 13.0    # Ø16, D컷 평면은 축중심 +5
FLAT_X = (-46.5, -3.7)        # D컷 **평면** 구간 (STEP X) — 여기만 회전을 막을 수 있다
SHAFT_END_X = -50.0           # 축 끝 (모따기 시작)

RAIL_Y, RAIL_Z = -50.0, 185.7  # 하부 사이드레일 중심 (로봇과 동일한 상대 위치)
RAIL_L = 240                   # 작업대용 토막 길이

GAP = (RAIL_Z - S / 2) - AXLE_Z         # 레일 밑면 ~ 축 = 채워야 할 높이


def wheel():
    """실물 허브모터. 축을 -Y 로, 타이어 중심면을 y=0 에 맞춘다."""
    hub = import_step(str(HUB_STEP))
    b = hub.bounding_box()
    assert abs(b.size.Y - WHEEL_D) < 1.5, f"STEP 외경이 실측과 다르다 {b.size}"
    return Rot(0, 0, 90) * Pos(-TIRE_C, 0, 0) * hub


# ---- 기존 클램프 (실측) ----
# refmodel 이 아니라 Downloads 에 받아둔 STEP 을 그대로 쓴다.
#   외형 42.72 x 39.22 x 12.57 / 코너 R2.54
#   채널 깊이 **5.07** — 축의 D컷 평면(축중심 +5)을 그대로 덮는다. 즉 맞대는 면이
#   축 중심 높이에 오고, 채널이 평면을 물어 **회전을 막는다.**
#   볼트 4곳 Ø8.13 장공(16.25L), 중심 (±15.1, ±11.87) → M8
CLAMP_STEP = Path("C:/Users/<user>/Downloads/hoverboard-motor-shaft-retention-bracket-1"
                  ".snapshot.6/Hub Motor Shaft Bracket.STEP")
CL_W, CL_T, CL_H = 42.72, 12.57, 39.22
CL_BOLT = (15.1, 11.87)       # 볼트 중심 (축 수직 / 축 방향)
CL_CH_D = 5.07                # 채널 깊이 = 평면 높이
CLAMP_Y = (FLAT_X[0] + FLAT_X[1]) / 2 - TIRE_C     # D컷 평면 구간 한가운데
CL_SLOT_W, CL_SLOT_L = 8.128, 16.25   # 볼트 구멍은 **장공**이다 (조정 여유 8.12mm)
# 클램프 고정 볼트 크기. STEP 의 Ø8.128 = 정확히 0.3200" — 코너 R2.54(0.1")·필렛
# R0.254(0.01") 와 함께 **모델러가 친 라운드 인치값**이지 실측이 아니다 (외형
# 42.72/12.57/39.22 는 인치로 안 떨어진다 = 이쪽이 실측). 실물은 8.0 일 수도 8.3 일
# 수도 있으니 캘리퍼로 재고 정해라.
#   M8 → 장공 여유 0.13mm. 실물이 8.0 이면 안 들어간다. 게다가 블록 y쪽 살이 4.73mm
#   M6 → 여유 2.13mm. 장공은 원래 조정용이라 헐거워도 되고 와셔(OD 12)가 덮는다.
#        하중은 볼트당 480 N (구동반력 29 N·m 를 x=±15.1 커플로) — M6 8.8 이 23배 여유
CLAMP_BOLT = float(os.environ.get("CLAMP_BOLT", 6))
_TAP = {6.0: 5.0, 8.0: 6.8}                        # 금속 탭 하공
_TAP_PLASTIC = {6.0: 5.3, 8.0: 7.0}                # 플라스틱은 하공을 키운다
assert CLAMP_BOLT in _TAP, f"CLAMP_BOLT 는 6 또는 8 (받은 값 {CLAMP_BOLT})"
assert CLAMP_BOLT + 0.5 <= CL_SLOT_W, (
    f"M{CLAMP_BOLT:.0f} 이 클램프 장공(Ø{CL_SLOT_W})에 여유 "
    f"{CL_SLOT_W-CLAMP_BOLT:.2f}mm 뿐 — 최소 0.5 (KS 근접끼워맞춤)가 필요하다.\n"
    f"    Ø{CL_SLOT_W} 는 실측이 아니라 모델러가 친 0.32\" 값이다. "
    f"실물을 캘리퍼로 재서 CL_SLOT_W 를 그 값으로 고치면 M8 도 통과한다")
MNT_BOLT, RAIL_BOLT = _TAP[CLAMP_BOLT], 6.6        # 클램프 탭 하공 / M6 통과 (스프링너트)

# ---- 마운트 치수는 **세 개의 수**로만 정한다 ----
# 처음엔 블록 깊이를 클램프 STEP 값(39.22)으로, ㄷ자 팔은 따로(34) 잡아서 양쪽 끝에
# 턱이 남았다. 전폭 65 / 벽 9 / 블록 22 도 제각각이었다.
# → 전폭·깊이·벽두께 셋만 두고 나머지는 전부 여기서 유도한다. 턱이 생길 수가 없다.
MNT_W, MNT_D, WALL = 70.0, 40.0, 10.0   # 전폭(x) / 깊이(y) / 벽·기둥 두께
BLK_H = 20.0                            # 하단 블록 높이 (축 밑 살 11.85)
# 레일 고정 M6 의 x. **기둥 안쪽**이어야 한다 — 기둥 바로 위에 두면 구멍이 기둥을
# 파고들고(3.3mm), 아래팔 볼트는 렌치가 기둥에 막혀 못 조인다.
BOLT_X = 15.0
HEX_R = 6.0                             # 육각렌지볼트 머리 + 공구 반경 여유
# ---- 거싯 (실물이 부러지고 나서 넣었다) ----
# ㄷ자 팔이 웹에서 뻗은 **무보강 외팔보 판**이었다. 수직하중이 접지점에 걸리는데
# 그립은 55mm 안쪽이라, 그 오프셋 모멘트를 10mm 판 두 장이 그대로 받고 있었다.
# 800N(한쪽 코너 탑승)에서 53.5 MPa = PETG 의 119% → 실물이 그 자리에서 부러졌다.
# 삼각 거싯을 뿌리에 세우면 판이 **T 단면**이 되어 Zx 가 6배 넘게 오른다.
#   x 위치는 레일 볼트(±15, Ø6.6 → 11.7~18.3)를 피해야 한다.
NO_GUSSET = bool(os.environ.get("NO_GUSSET"))
U_OPEN_INBOARD = bool(os.environ.get("U_OPEN_INBOARD"))
GUSSET = ((0.0, 0.0, 0.0) if NO_GUSSET else (8.0, 25.0, 25.0))
                                               # 두께(x) / 높이(z) / 길이(y)
GUSSET_X = (0.0, 26.0)                  # 0 과 ±26 → 3장/팔. 볼트 구멍을 안 문다
assert all(abs(abs(gx) - BOLT_X) > (GUSSET[0] + RAIL_BOLT) / 2 or gx == 0
           for gx in GUSSET_X), "거싯이 레일 볼트 구멍을 문다"
# ㄷ자 슬롯 여유. **아래쪽에만** 준다.
#   처음엔 슬롯 높이가 정확히 30.000 이었다 — 프로파일 압출 공차 ±0.2~0.3 이라
#   레일이 30.2 로 나오면 안 끼워진다. 프린트면 줄로 밀면 되지만 CNC 알루미늄은
#   받아놓고 못 쓴다.
#   볼트를 조이면 스프링너트가 팔을 레일 면으로 끌어당기므로 이 틈은 저절로 닫힌다.
#   수직 하중은 미끄러졌을 때 **위팔 밑면**이 레일 윗면에 걸려 받으니, 위는 딱 맞추고
#   여유는 아래로 몰아야 한다.
#   PRINT=1 이면 FDM 오차를 얹는다. FDM 은 CNC 와 오차의 **성격**이 다르다:
#     · 내부 치수가 0.1~0.3 작게 나온다 (압출 겹침) → 슬롯을 더 벌린다
#     · 눕혀 뽑으면 볼트 구멍이 **수평**이라 작게 + 위가 처진다 → 구멍을 키운다
#     · Ø6.8 은 **금속** M8 하공이다. 플라스틱은 하공을 키워야 산이 안 뭉개진다
#   드릴로 후가공해도 되지만(32구멍) 모델에서 맞추는 쪽이 확실하다.
PRINT = bool(os.environ.get("PRINT"))
RAIL_FIT = 0.8 if PRINT else 0.4
if PRINT:
    MNT_BOLT = _TAP_PLASTIC[CLAMP_BOLT]   # 플라스틱 하공 (금속보다 크게 — 산이 안 뭉개진다)
    RAIL_BOLT = 7.2                       # 수평 구멍 수축·처짐 보정 (M6 이 지나가야 한다)

# ---- 볼트 길이 ----
# 실물 조립에서 나온 문제다. 프레임 코너 브라켓과 같은 M6x15 를 쓰면 **물림이
# 0.6mm** 다 — 조여지는 느낌은 나는데 스프링너트 나사산을 겨우 걸친 상태다.
#   팔 10 + 와셔 1.6 + 레일여유 0.4 + 슬롯립 2.4 = 14.4  ← 여기까지는 나사산 밖
# 브라켓은 4t 라 M6x15 로 물림 8.6mm 가 나온다. 같은 볼트를 쓸 수가 없다.
WASHER_T, HEAD_H = 1.6, 3.3       # M6 와셔 두께 / 둥근머리 높이
SLOT_LIP = PROFILES[SERIES]["wall"]   # 슬롯 립 = 스프링너트 윗면까지 깊이
MIN_ENGAGE = 6.0                  # M6 최소 물림 (1 x D)


def rail_bolt_len(arm_t=None, fit=None):
    """레일 고정 볼트의 **머리 밑** 최소 길이."""
    arm_t = WALL if arm_t is None else arm_t
    fit = RAIL_FIT if fit is None else fit
    return arm_t + WASHER_T + fit + SLOT_LIP + MIN_ENGAGE


RAIL_BOLT_L = 25.0                # 실제로 살 것 — 5 단위로 올림
assert RAIL_BOLT_L >= rail_bolt_len(), (
    f"레일 볼트 M6x{RAIL_BOLT_L:.0f} 은 짧다 — "
    f"머리 밑 {rail_bolt_len():.1f}mm 이상이 필요하다 "
    f"(팔 {WALL} + 와셔 {WASHER_T} + 여유 {RAIL_FIT} + 슬롯립 {SLOT_LIP} + 물림 {MIN_ENGAGE})")

MNT_Y1 = RAIL_Y - S / 2 - WALL          # 안쪽 끝 = ㄷ자 웹 바깥면
MNT_Y0 = MNT_Y1 + MNT_D                 # 바깥 끝 — 레일 바깥면을 지난다
CLAMP_Y = (MNT_Y0 + MNT_Y1) / 2         # 클램프 중심 = 마운트 한가운데
ARM_X = (MNT_W - WALL) / 2              # 기둥 두 팔의 x 중심
assert MNT_Y1 + WALL <= RAIL_Y - S / 2 + 0.01, "ㄷ자 웹이 레일 안쪽면을 파고든다"
assert MNT_Y0 >= RAIL_Y + 0.01, "ㄷ자 팔이 슬롯 중심을 못 지나 볼트가 안 물린다"
assert ARM_X - WALL / 2 > CL_W / 2 + 2, "기둥이 클램프와 부딪힌다"
assert BOLT_X + HEX_R < ARM_X - WALL / 2, \
    f"레일 볼트(x=±{BOLT_X})가 기둥(안쪽면 {ARM_X-WALL/2})에 막힌다 — 렌치가 안 들어간다"
# 평면 구간 안에 있어야 하는 건 블록이 아니라 **클램프**다. 블록의 반원 시트는
# 축 아래 둥근 면을 받으므로 평면과 상관없고, 회전 방지는 클램프 채널이 한다.
assert FLAT_X[0] - TIRE_C <= CLAMP_Y - CL_H / 2 \
    and CLAMP_Y + CL_H / 2 <= FLAT_X[1] - TIRE_C, \
    f"클램프({CLAMP_Y-CL_H/2:.1f}~{CLAMP_Y+CL_H/2:.1f})가 D컷 평면 구간을 벗어난다"


def clamp():
    """기존 클램프. STEP 로컬(채널이 Z 방향)을 로봇 좌표(축이 Y)로 돌린다."""
    c = import_step(str(CLAMP_STEP))
    # STEP: 채널이 Z 를 따라, 두께가 Y(0~12.57), 맞댐면이 y=12.57
    # 로봇: 축이 Y, 맞댐면이 아래(z=AXLE_Z), 클램프는 그 위
    return Pos(0, CLAMP_Y, AXLE_Z) * Rot(-90, 0, 0) * Pos(0, -CL_T, 0) * c


# ---- 대안 구조: 하단 클램프 (프레임이 바로 위에 온다) ----
# 지금 mount() 는 레일에서 축까지 91.35mm 를 건너느라 키가 크고, 그 캔틸레버 끝의
# 10mm 판(ㄷ자 팔)에서 실물이 부러졌다.
# 이 구조는 **기존 클램프의 짝**만 만들고 프레임을 축 바로 위로 내린다.
#   · 팔 굽힘·기둥 굽힘·웹 굽힘이 전부 사라진다 (그 형상이 없다)
#   · 부피 1/3 → CNC 값이 크게 준다
#   · 55mm 오프셋 모멘트는 **그대로 남는다** (허브모터 지오메트리라 못 없앤다)
#     밑면 고정은 지레가 블록 반깊이뿐이라 ㄷ자보다 3배 불리하다
#     → **레일 관통볼트**로 받는다. 스프링너트(인발 1.5kN) 대신 볼트 강도(11kN).
#       레일이 바로 위에 있는 이 구조라야 관통볼트를 쓸 수 있다.
#   · 대가: 프레임이 77mm 내려가고 바퀴가 레일보다 35mm 솟아 셸에 휠아치가 필요하다
# 전폭 76 / 다리 14 / 카운터보어 Ø11 로 시작했는데, 경로를 전부 훑어보니
# **카운터보어 바닥 면압이 지배**였다 (PETG 39%). 다리를 넓혀 보어를 키우면 내려간다.
# 블록 높이 20 → 24. 지배 모드가 **블록 Y굽힘**(다리 사이 48mm 스팬)이었고, 시트
# 아래 살이 11.8mm 뿐이라 Zx=936 이었다. 살은 (LOW_H - 축반경) 이고 Z 는 그 제곱이라
# 4mm 만 더해도 23% → 13% 로 떨어진다. 대가는 지상고 4mm 뿐이다.
LOW_W, LOW_D, LOW_H = 84.0, 40.0, 24.0     # 블록 (전폭 / 깊이 / 높이)
LOW_POST_W = 18.0                          # 다리 폭 — 클램프를 타고 넘는다
# 다리 x 는 **블록 바깥면에 맞춘다**. 30 이라고 눈대중으로 박았더니 다리가 23~37 이
# 되어 블록(±38) 바깥에 1mm 턱이 남았다. 유도값으로 두면 생길 수가 없다.
LOW_POST_X = LOW_W / 2 - LOW_POST_W / 2
LOW_RAIL_CLR = 2.0                         # 클램프 윗면 ~ 레일 밑면
LOW_CBORE = (16.0, 6.0)                    # 볼트머리 카운터보어 (지름, 깊이)
LOW_CY = (FLAT_X[0] + FLAT_X[1]) / 2 - TIRE_C      # D컷 평면 한가운데
LOW_RAIL_BOT = AXLE_Z + CL_T + LOW_RAIL_CLR
LOW_RAIL_Z = LOW_RAIL_BOT + S / 2
assert LOW_POST_X - LOW_POST_W / 2 > CL_W / 2 + 1, "다리가 기존 클램프와 부딪힌다"
assert LOW_POST_X + LOW_POST_W / 2 <= LOW_W / 2, "다리가 블록 밖으로 나간다"
assert LOW_CBORE[0] + 2 <= LOW_POST_W, "카운터보어가 다리보다 커서 벽이 안 남는다"
assert FLAT_X[0] - TIRE_C <= LOW_CY - CL_H / 2 and LOW_CY + CL_H / 2 <= FLAT_X[1] - TIRE_C,     "클램프가 D컷 평면 구간을 벗어난다"


# ---- 3면 감싸기 (하단 클램프 + 레일 3면 물기) ----
# 하단 클램프는 밑면 볼트 2개뿐이라 기울임 모멘트를 **인발**로 받는다 (800N 에서
# 스프링너트 인발의 72%). 옆면을 하나 더 물면 그 모멘트가 **전단**으로 넘어간다 —
# 스프링너트는 인발 1.5kN 인데 전단은 M6 볼트 강도 8kN 이라 5배 세다.
#   바깥쪽 옆면은 못 쓴다: 레일 바깥면(y=-38.7)에서 모터 몸통(y~-27)까지 12mm 뿐이라
#   팔 두께 + 볼트머리 + 렌치가 안 들어간다. → **안쪽 옆 + 윗면**을 쓴다.
# 볼트 6개: 하단 2 (수직) + 안쪽 옆 2 (수평, 전단) + 윗면 2 (수직)
WRAP_T = 10.0                              # 웹·윗팔 두께
WRAP_BOLT_X = 15.0                         # 옆·윗 볼트 x (다리 24~42 를 피한다)
WRAP_Y_IN = LOW_CY - S / 2 - WRAP_T        # 웹 바깥면 (= 안쪽 끝)
WRAP_Y_OUT = LOW_CY + LOW_D / 2            # 바깥 끝 — 하단 클램프와 같게 둔다
WRAP_D = WRAP_Y_OUT - WRAP_Y_IN            # 블록 깊이 (웹까지 이어지도록 늘린다)
WRAP_CY = (WRAP_Y_IN + WRAP_Y_OUT) / 2
assert WRAP_BOLT_X + RAIL_BOLT / 2 < LOW_POST_X - LOW_POST_W / 2,     "옆·윗 볼트가 다리와 겹친다"


def mount_wrap():
    """하단 클램프 + 레일 3면 물기 — 안쪽 옆면 볼트가 기울임을 전단으로 받는다."""
    r = SHAFT_D / 2 + 0.15
    rb, rt = LOW_RAIL_BOT, LOW_RAIL_BOT + S          # 레일 밑면 / 윗면
    ri = LOW_CY - S / 2                              # 레일 안쪽면
    with BuildPart() as p:
        with Locations((0, WRAP_CY, AXLE_Z - LOW_H / 2)):        # 블록 (웹까지 확장)
            Box(LOW_W, WRAP_D, LOW_H)
        for sx in (-1, 1):                                        # 다리 2
            with Locations((sx * LOW_POST_X, WRAP_CY, (AXLE_Z + rb) / 2)):
                Box(LOW_POST_W, WRAP_D, rb - AXLE_Z)
        with Locations((0, ri - WRAP_T / 2, (rb + rt + WRAP_T) / 2)):   # 안쪽 웹
            Box(LOW_W, WRAP_T, rt + WRAP_T - rb)
        with Locations((0, WRAP_CY, rt + WRAP_T / 2)):            # 윗팔
            Box(LOW_W, WRAP_D, WRAP_T)
        # 시트는 **블록 중심(WRAP_CY)** 기준으로 판다. LOW_CY 기준으로 뒀더니
        # 블록을 웹까지 늘린 만큼 안쪽 끝이 0.4mm 안 파여서 축을 물었다 (50mm3).
        with Locations((0, WRAP_CY, AXLE_Z)):                     # 축 반원 시트
            Cylinder(r, WRAP_D + 6, rotation=(90, 0, 0), mode=Mode.SUBTRACT)
        with Locations((0, WRAP_CY, AXLE_Z + r / 2)):             # 시트 위를 연다
            Box(2 * r, WRAP_D + 6, r, mode=Mode.SUBTRACT)
        for sx in (-1, 1):                                        # 기존 클램프 탭 4
            for sy in (-1, 1):
                with Locations((sx * CL_BOLT[0], LOW_CY + sy * CL_BOLT[1],
                                AXLE_Z - LOW_H / 2)):
                    Cylinder(MNT_BOLT / 2, LOW_H + 2, mode=Mode.SUBTRACT)
        for sx in (-1, 1):                                        # ① 하단 볼트 2
            with Locations((sx * LOW_POST_X, LOW_CY, (AXLE_Z - LOW_H + rb) / 2)):
                Cylinder(RAIL_BOLT / 2, rb - AXLE_Z + LOW_H + 2, mode=Mode.SUBTRACT)
            with Locations((sx * LOW_POST_X, LOW_CY,
                            AXLE_Z - LOW_H + LOW_CBORE[1] / 2 - 0.01)):
                Cylinder(LOW_CBORE[0] / 2, LOW_CBORE[1], mode=Mode.SUBTRACT)
        for sx in (-1, 1):                                        # ② 안쪽 옆 볼트 2
            with Locations((sx * WRAP_BOLT_X, ri - WRAP_T / 2, (rb + rt) / 2)):
                Cylinder(RAIL_BOLT / 2, WRAP_T + 4, rotation=(90, 0, 0),
                         mode=Mode.SUBTRACT)
        for sx in (-1, 1):                                        # ③ 윗면 볼트 2
            with Locations((sx * WRAP_BOLT_X, LOW_CY, rt + WRAP_T / 2)):
                Cylinder(RAIL_BOLT / 2, WRAP_T + 4, mode=Mode.SUBTRACT)
    p.part.label = "3면마운트"
    return p.part


def mount_low():
    """하단 클램프 — 기존 클램프의 짝. 위로 프레임 레일이 바로 얹힌다."""
    r = SHAFT_D / 2 + 0.15
    yc = LOW_CY
    with BuildPart() as p:
        with Locations((0, yc, AXLE_Z - LOW_H / 2)):                # 블록
            Box(LOW_W, LOW_D, LOW_H)
        for sx in (-1, 1):                                          # 다리 2개
            with Locations((sx * LOW_POST_X, yc,
                            (AXLE_Z + LOW_RAIL_BOT) / 2)):
                Box(LOW_POST_W, LOW_D, LOW_RAIL_BOT - AXLE_Z)
        with Locations((0, yc, AXLE_Z)):                            # 축 반원 시트
            Cylinder(r, LOW_D + 4, rotation=(90, 0, 0), mode=Mode.SUBTRACT)
        with Locations((0, yc, AXLE_Z + r / 2)):                    # 시트 위를 연다
            Box(2 * r, LOW_D + 4, r, mode=Mode.SUBTRACT)
        for sx in (-1, 1):                                          # 기존 클램프 탭 4
            for sy in (-1, 1):
                with Locations((sx * CL_BOLT[0], yc + sy * CL_BOLT[1],
                                AXLE_Z - LOW_H / 2)):
                    Cylinder(MNT_BOLT / 2, LOW_H + 2, mode=Mode.SUBTRACT)
        for sx in (-1, 1):                                          # 레일 관통볼트 2
            with Locations((sx * LOW_POST_X, yc,
                            (AXLE_Z - LOW_H + LOW_RAIL_BOT) / 2)):
                Cylinder(RAIL_BOLT / 2, LOW_RAIL_BOT - AXLE_Z + LOW_H + 2,
                         mode=Mode.SUBTRACT)
            # 머리를 묻는다 — 지상고 59mm 에서 볼트머리가 튀어나오면 걸린다
            with Locations((sx * LOW_POST_X, yc,
                            AXLE_Z - LOW_H + LOW_CBORE[1] / 2 - 0.01)):
                Cylinder(LOW_CBORE[0] / 2, LOW_CBORE[1], mode=Mode.SUBTRACT)
    p.part.label = "하단클램프"
    return p.part


def mount():
    """레일에 꽂아 아래로 뻗는 마운트 — 클램프의 짝.

    · 위: 레일 **밑면 슬롯**에 스프링너트 + 육각렌지볼트 2개. 레일을 타고 미끄러지므로
          휠베이스를 실물에서 바꿔 시험할 수 있고, 조인 자리에서 고정된다.
    · 아래: 축 반원(R{:.2f})을 받치고, 클램프 볼트 4곳을 M8 로 탭한다.
            **맞댐면이 축 중심 높이**여야 클램프 채널이 평면을 문다.
    """.format(SHAFT_D / 2)
    r = SHAFT_D / 2 + 0.15
    yc = CLAMP_Y
    rail_bot, rail_top = RAIL_Z - S / 2, RAIL_Z + S / 2
    # 위팔은 레일 윗면에 딱, 아래팔만 RAIL_FIT 만큼 내린다 (슬롯 30 → 30.4)
    c_bot, c_top = rail_bot - RAIL_FIT - WALL, rail_top   # ㄷ자 아래팔 밑 / 위팔 밑
    with BuildPart() as p:
        with Locations((0, yc, AXLE_Z - BLK_H / 2)):               # 하단 블록
            Box(MNT_W, MNT_D, BLK_H)
        # 기둥은 **클램프를 타고 넘어야** 한다. 가운데로 곧게 올리면 클램프가 앉을
        # 자리(축 중심 위 12.57mm)를 그대로 먹는다 (처음에 7220 mm3 물렸다).
        for sx in (-1, 1):
            with Locations((sx * ARM_X, yc, (AXLE_Z + c_bot) / 2)):
                Box(WALL, MNT_D, c_bot - AXLE_Z)
        # ㄷ자 — 레일을 안쪽에서 감싸 **밑면과 윗면 두 곳**에서 문다.
        # 밑면만 물면 횡력 모멘트 18.6 N·m 를 볼트가 프라잉(비틀어 뜯기)으로 받는다.
        # 위아래로 물면 레일 높이 30mm 를 팔로 쓰는 순수 커플이 되고, 마운트가
        # 레일에서 벗겨질 수가 없다.
        web_y = MNT_Y0 - WALL / 2 if U_OPEN_INBOARD else MNT_Y1 + WALL / 2
        with Locations((0, web_y, (c_bot + c_top + WALL) / 2)):               # 웹
            Box(MNT_W, WALL, c_top + WALL - c_bot)
        for z in (c_bot + WALL / 2, c_top + WALL / 2):             # 아래 팔 / 위 팔
            with Locations((0, yc, z)):
                Box(MNT_W, MNT_D, WALL)
        with Locations((0, yc, AXLE_Z)):                           # 축 반원 시트
            Cylinder(r, MNT_D + 4, rotation=(90, 0, 0), mode=Mode.SUBTRACT)
        with Locations((0, yc, AXLE_Z + r / 2)):                   # 시트 위를 연다
            Box(2 * r, MNT_D + 4, r, mode=Mode.SUBTRACT)
        for sx in (-1, 1):                                         # 클램프 M8 탭 4
            for sy in (-1, 1):
                with Locations((sx * CL_BOLT[0], yc + sy * CL_BOLT[1],
                                AXLE_Z - BLK_H / 2)):
                    Cylinder(MNT_BOLT / 2, BLK_H + 2, mode=Mode.SUBTRACT)
        # 거싯 — 팔 뿌리(웹 안쪽면)에 삼각 보강. 아래팔은 아래로, 위팔은 위로.
        gt, gh, gl = GUSSET
        if gt > 0 and gh > 0 and gl > 0:
            yw = MNT_Y1 + WALL                                 # 웹 안쪽면
            for gx in sorted({s * x for x in GUSSET_X for s in (-1, 1)}):
                for zc, sz in ((c_bot, -1), (c_top + WALL, +1)):
                    with BuildSketch(Plane.YZ.offset(gx)) as sk:
                        Polygon((yw, zc), (yw + gl, zc), (yw, zc + sz * gh),
                                align=None)
                    extrude(to_extrude=sk.sketch, amount=gt / 2, both=True)
        for sx in (-1, 1):                         # 레일 M6 통과 — 밑면 2 + 윗면 2
            for z in (c_bot + WALL / 2, c_top + WALL / 2):
                with Locations((sx * BOLT_X, RAIL_Y, z)):
                    Cylinder(RAIL_BOLT / 2, WALL + 4, mode=Mode.SUBTRACT)
    return p.part


def _ivol(a, b):
    """빈 교집합은 None 으로 온다 — 그대로 .volume 하면 죽는다."""
    try:
        x = a.moved(Location()) & b.moved(Location())
        return x.volume if x is not None else 0.0
    except Exception:
        return 0.0


# ---- 내보내기 ----
# STEP : 3D 프린트 슬라이서·CNC 둘 다 이걸 그대로 받는다 (STL 보다 정확하다)
# STL  : 슬라이서 호환용 예비
# DXF  : 이 부품은 **평판이 아니라 3D 형상**이라 DXF 로는 못 만든다. 단면 3장을
#        도면·검토용으로만 낸다 (치수 재기, 발주처와 형상 확인).
EXPORT = Path(__file__).parent.parent / "exports" / ("print" if PRINT else "cnc")
EXPORT.mkdir(parents=True, exist_ok=True)


def bed(shape):
    """프린트 자세로 눕혀 바닥(z=0)·중앙에 앉힌다.

    모델은 **로봇 좌표**라 Z 가 151mm 세로다. 그대로 슬라이서에 넣으면 세워서
    놓이는데, 그 자세는 서포트 40 cm2 에 응력이 층간 인장이 된다 (눕히면 0/층방향).
    사람이 손으로 돌리게 두면 언젠가 틀린 자세로 뽑는다 → 파일에서 눕혀 내보낸다.
    웹면 법선이 -Y 이므로 X 축 +90° 가 그걸 -Z(바닥)로 보낸다.
    """
    s = Rot(90, 0, 0) * shape.moved(Location())
    b = s.bounding_box()
    return Pos(-b.center().X, -b.center().Y, -b.min.Z) * s


# ---- 하중 경로 검사 ----
# **실물이 부러지고 나서 넣은 것이다.** 그 전에는 "접지점 횡력" 한 케이스만 손으로
# 계산하고 "여유 7배"라고 결론냈는데, 두 가지를 놓쳤다:
#   (1) 하중 케이스 — 89kg 로봇만 봤다. **사람이 올라타는 걸 안 넣었다.**
#       시험 플랫폼은 반드시 누가 올라타거나 기댄다. 예외가 아니라 기본 하중이다.
#   (2) 파괴 모드 — 수직하중이 접지점(y=0)에 걸리는데 그립은 y=-55 다. 이 55mm
#       오프셋 모멘트가 ㄷ자 팔로 가는데, 팔을 **보 단면**(Zx 18454)으로 쟀다.
#       실제로는 **웹에서 뻗은 외팔보 판**(Zx 947)이다 — 19배 차이.
#       실물은 이 자리(상단 고정부)에서 금이 갔고 측면 충격에 부러졌다.
# 그래서 한 군데를 고르지 않고 **경로의 모든 단면을 훑는다.** 손계산이라 ±2배는
# 각오해야 하지만, 지배 단면이 어디인지는 이걸로 안다.
DESIGN_W = 800.0          # 설계 수직하중/바퀴 — 한쪽 코너에 사람이 올라선 경우
WASHER_OD = 18.0          # 레일 볼트 와셔 바깥지름 — 머리 면압을 이걸로 낮춘다
TNUT_PULLOUT = 1500.0     # 스프링너트 306 슬롯 인발 (보수적 하한, N)
SAFETY = 2.0
MAT_STRENGTH = {"PETG": 45.0, "ASA": 44.0, "PLA": 55.0, "AL6061": 270.0}


def arm_section_Z():
    """팔 뿌리(웹 접합부)의 단면계수. 거싯이 있으면 T 단면으로 계산한다.

    뿌리에는 볼트 구멍이 없다 (볼트는 15mm 바깥이다) → 폭은 MNT_W 전체를 쓴다.
    """
    b, tf = MNT_W, WALL                                 # 플랜지 = 팔 판
    gt, gh, _ = GUSSET
    n = len([s * x for x in GUSSET_X for s in (-1, 1) if not (x == 0 and s < 0)])
    bw = gt * n                                          # 거싯 총 폭
    if bw <= 0 or gh <= 0:
        return b * tf ** 2 / 6
    a1, a2 = b * tf, bw * gh
    y1, y2 = tf / 2, tf + gh / 2                         # 판 바깥면 기준
    yb = (a1 * y1 + a2 * y2) / (a1 + a2)
    i = (b * tf ** 3 / 12 + a1 * (yb - y1) ** 2
         + bw * gh ** 3 / 12 + a2 * (y2 - yb) ** 2)
    return i / max(yb, tf + gh - yb)


def load_paths(w=DESIGN_W):
    """수직하중 w(N/바퀴) 가 지나는 모든 단면의 응력. (이름, 응력MPa, 설명) 목록."""
    off = abs(CLAMP_Y)                       # 접지점 → 그립 중심 오프셋 = 55
    m = w * off                              # 오프셋 모멘트
    cpl = 20.0                               # 레일 상하면 지압 커플 레버 (삼각분포 근사)
    lev = 23.0                               # 팔 뿌리(웹) → 접촉 합력 (2/3 지점)
    fc = m / cpl                             # 팔에 걸리는 커플 힘
    z_arm = arm_section_Z()
    z_web = MNT_W * WALL ** 2 / 6
    out = [
        ("기둥 굽힘(X)", m / 5333.0, "두 기둥 10x40, 오프셋 모멘트"),
        ("기둥 압축", w / (2 * WALL * MNT_D), "축방향"),
        ("축 시트 면압", w / (SHAFT_D * MNT_D), "반원 받침 투영면적"),
        ("**ㄷ자 팔 (판 외팔보)**", fc * lev / z_arm, f"Zx={z_arm:.0f} — 여기서 부러졌다"),
        ("ㄷ자 웹 굽힘", fc * WALL / z_web, f"Zx={z_web:.0f}"),
        # 볼트는 **인장**을 받는다. 커플 힘 fc 는 팔이 레일 면을 누르는 접촉압이지
        # 볼트를 통하지 않는다 (처음에 fc/(d*t) 로 놨던 건 틀린 모델이었다).
        # 실제 위험은 **볼트 머리가 플라스틱을 파고드는 것** — 그러면 예압이 풀리고
        # 마운트가 레일 위에서 조금씩 돌아간다. "점점 밀린다"의 원인일 수 있다.
        ("볼트머리 지압", (m / S / 2) / (math.pi / 4 * (WASHER_OD ** 2 - RAIL_BOLT ** 2)),
         f"와셔 OD{WASHER_OD:.0f} 아래 면압"),
    ]
    # 팔당 볼트가 **2개**다 (x=±BOLT_X). 커플 힘을 한 개에 다 걸면 2배 틀린다.
    return out, m / S / 2      # 볼트 1개당 인장


def load_paths_low(w=DESIGN_W):
    """하단 클램프 구조의 하중 경로. mount() 쪽 load_paths() 와 같은 기준으로 낸다.

    ㄷ자와 비교하려면 **같은 방식으로 훑어야** 한다. 앞서 볼트·블록·시트 세 개만
    손으로 집어서 "20% 미만"이라고 했는데, 그게 정확히 처음에 팔을 놓쳤던 방식이다.
    """
    off = abs(LOW_CY)
    m = w * off                                  # 오프셋 모멘트
    span = 2 * LOW_POST_X - LOW_POST_W           # 다리 사이 순 스팬
    seat_h = LOW_H - (SHAFT_D / 2 + 0.15)        # 축 시트 아래 남은 살
    z_blk_x = LOW_W * LOW_H ** 2 / 6             # 블록, X축 굽힘 (오프셋 모멘트)
    z_blk_y = LOW_D * seat_h ** 2 / 6            # 블록, Y축 굽힘 (다리 사이 스팬)
    z_post = 2 * (LOW_POST_W * LOW_D ** 3 / 12) / (LOW_D / 2)
    fb = m / (LOW_D / 2) / 2                     # 관통볼트 1개당 인장
    a_cb = math.pi / 4 * (LOW_CBORE[0] ** 2 - RAIL_BOLT ** 2)
    return [
        ("블록 굽힘(Y, 다리스팬)", w * span / 4 / z_blk_y,
         f"순스팬 {span:.0f}, 시트 아래 살 {seat_h:.1f} → Zx={z_blk_y:.0f}"),
        ("**볼트머리 지압(카운터보어)**", fb / a_cb,
         f"Ø{LOW_CBORE[0]:.0f} 카운터보어 바닥"),
        ("블록 굽힘(X, 오프셋)", m / z_blk_x, f"Zx={z_blk_x:.0f}"),
        ("다리 굽힘", m / z_post, f"14x40 x2 → Zx={z_post:.0f}"),
        ("축 시트 면압", w / (SHAFT_D * LOW_D), "반원 받침 투영면적"),
        ("블록 압축", w / (LOW_W * LOW_D), "축방향"),
    ], fb


def report_loads_low(mat="PETG", w=DESIGN_W):
    rows, fb = load_paths_low(w)
    lim = MAT_STRENGTH[mat]
    print("")
    print(f"[하단 클램프] 수직 {w:.0f} N/바퀴, {mat} (인장 {lim:.0f} MPa)")
    worst = max(rows, key=lambda r: r[1])
    for n, s, note in sorted(rows, key=lambda r: -r[1]):
        mark = " ← 지배" if n == worst[0] else ""
        print(f"    {n:28s} {s:6.1f} MPa  ({s/lim*100:3.0f}%)  {note}{mark}")
    print(f"    {'관통볼트 인장':28s} {fb:6.0f} N  ({fb/11000*100:3.0f}%)"
          f"  M6 8.8 약 11 kN  ← 스프링너트(1.5kN)가 아니다")
    return worst, fb


def report_loads(mat="PETG", w=DESIGN_W):
    rows, fbolt = load_paths(w)
    lim = MAT_STRENGTH[mat]
    print("")
    print(f"하중 경로 — 수직 {w:.0f} N/바퀴, {mat} (인장 {lim:.0f} MPa, 안전율 {SAFETY})")
    worst = max(rows, key=lambda r: r[1])
    for n, s, note in sorted(rows, key=lambda r: -r[1]):
        mark = " ← 지배" if n == worst[0] else ""
        print(f"    {n:24s} {s:6.1f} MPa  ({s/lim*100:3.0f}%)  {note}{mark}")
    print(f"    {'레일 볼트 인장':24s} {fbolt:6.0f} N  ({fbolt/TNUT_PULLOUT*100:3.0f}%)"
          f"  스프링너트 인발 하한 {TNUT_PULLOUT:.0f} N")
    if fbolt > TNUT_PULLOUT * 0.5:
        print(f"    !! 볼트 인장이 너트 인발의 {fbolt/TNUT_PULLOUT*100:.0f}% — "
              f"슬롯에서 뽑히거나 레일 립이 벌어질 수 있다")
    return worst, fbolt


def stress_view(mat="PETG", w=DESIGN_W):
    """마운트를 하중 경로 구간으로 쪼개 **사용률로 색칠**한다.

    FEA 가 아니다. load_paths() 가 구간별로 계산한 응력을 그 구간 형상에 입힌
    것뿐이라, 한 구간 안에서의 분포는 안 나온다. 그래도 **어디가 지배 단면인지**는
    한눈에 보인다 — 내가 표에서 놓쳤던 게 정확히 그거였다.
    """
    rows, _ = load_paths(w)
    s = {n: v for n, v, _ in rows}
    lim = MAT_STRENGTH[mat]
    rail_bot, rail_top = RAIL_Z - S / 2, RAIL_Z + S / 2
    c_bot, c_top = rail_bot - RAIL_FIT - WALL, rail_top
    big = 400

    def clip(z0, z1, y0=-big, y1=big):
        return Pos(0, (y0 + y1) / 2, (z0 + z1) / 2) * Box(big, y1 - y0, z1 - z0)

    yw = MNT_Y1 + WALL                      # 웹 안쪽면
    regions = [
        ("하단블록", clip(0, AXLE_Z + 1), s["축 시트 면압"]),
        ("기둥", clip(AXLE_Z + 1, c_bot), s["기둥 굽힘(X)"]),
        ("아래팔", clip(c_bot, c_bot + WALL, yw, big), s["**ㄷ자 팔 (판 외팔보)**"]),
        ("위팔", clip(c_top, c_top + WALL, yw, big), s["**ㄷ자 팔 (판 외팔보)**"]),
        ("웹", clip(c_bot, c_top + WALL, -big, yw), s["ㄷ자 웹 굽힘"]),
    ]
    m = mount()
    out = []
    for name, tool, sig in regions:
        try:
            part = m & tool
        except Exception:
            continue
        if part is None or part.volume < 10:
            continue
        u = sig / lim
        # 초록(안전) → 노랑 → 주황 → 빨강(초과)
        col = ("#2e7d32" if u < 0.3 else "#f9a825" if u < 0.6
               else "#ef6c00" if u < 1.0 else "#c62828")
        part.label = f"{name} {sig:.0f}MPa {u*100:.0f}%"
        part.color = Color(col)
        out.append(part)
    return out


def coupon(width=40.0):
    """끼움 시험용 쿠폰 — ㄷ자 부분만 잘라낸 토막.

    본품은 4시간짜리다. 슬롯이 안 맞으면 4시간을 버린다. 이 쿠폰은 20~40분이면
    나오고 **본품과 완전히 같은 자세·같은 슬라이서 설정**으로 뽑으므로, 여기서
    맞으면 본품도 맞는다. 레일 볼트 구멍(x=±15)도 들어 있어 스프링너트 위치까지
    같이 본다.
    """
    z0 = RAIL_Z - S / 2 - RAIL_FIT - WALL              # ㄷ자 아래팔 밑
    box = Pos(0, CLAMP_Y, (z0 + RAIL_Z + S / 2 + WALL) / 2) \
        * Box(width, MNT_D + 2, (RAIL_Z + S / 2 + WALL) - z0)
    return mount() & box


def _dxf_section(shape, plane, path):
    """평면으로 자른 단면을 XY 로 눕혀 DXF 로 낸다."""
    sec = shape & plane
    fs = [f for f in sec.faces() if f.geom_type == GeomType.PLANE]
    if not fs:
        return None
    ex = ExportDXF(unit=Unit.MM)
    inv = Plane(fs[0]).location.inverse()
    for f in fs:
        ex.add_shape(f.moved(inv))
    ex.write(str(path))
    return sum(f.area for f in fs)

# ---- 여기부터는 이 파일을 **직접 실행**했을 때만 돈다 ----
# swerve_base 가 mount()/clamp() 를 가져다 쓰므로, import 만으로 간섭검사·
# 내보내기·show 가 돌면 안 된다 (느리고, 남의 exports 를 덮어쓴다).
if __name__ == "__main__":
    w = Pos(0, 0, AXLE_Z) * wheel()
    w.label, w.color = "바퀴", Color("#2b2b2b")
    r = bar(SERIES, RAIL_L, (-RAIL_L / 2, RAIL_Y, RAIL_Z), "X")
    r.label, r.color = "하부사이드레일", Color("#c8ccd0")
    m = mount()
    m.label, m.color = "휠마운트", Color("#6b7480")
    cl = clamp()
    cl.label, cl.color = "기존클램프", Color("#c62828")

    wb = w.bounding_box()
    print(f"휠 마운트 작업대 — 3030 레일 한 토막 + 실물 허브모터")
    print(f"  바퀴 Ø{WHEEL_D} / 축 중심 z={AXLE_Z:.2f} / 몸통 y {wb.min.Y:.1f}~{wb.max.Y:.1f}")
    print(f"  축: Ø{SHAFT_D} D컷{FLAT}  평면 구간 y {FLAT_X[0]-TIRE_C:.1f}~{FLAT_X[1]-TIRE_C:.1f}"
          f" ({FLAT_X[1]-FLAT_X[0]:.1f}mm)  축 끝 y={SHAFT_END_X-TIRE_C:.1f}")
    print(f"     **나사부 없음** — 너트를 못 쓴다. 물려면 클램프뿐이다")
    print(f"  레일 {SERIES}{SERIES} 중심 (y={RAIL_Y}, z={RAIL_Z}) / 밑면 z={RAIL_Z-S/2:.1f}"
          f" / 바깥면 y={RAIL_Y+S/2:.1f}")
    print(f"  → 채워야 할 높이 {GAP:.2f}mm (레일 밑면 → 축 중심)")
    mb = m.bounding_box()
    print(f"\n기존 클램프 (실측): {CL_W} x {CL_H} x {CL_T} / 채널 깊이 {CL_CH_D}"
          f" = 평면 높이 → **채널이 D컷을 물어 회전을 막는다**")
    print(f"  볼트 4곳 Ø8.13 장공, 중심 (±{CL_BOLT[0]}, ±{CL_BOLT[1]}) → M8")
    print(f"\n휠마운트 (새로 만든 것): {mb.size.X:.1f} x {mb.size.Y:.1f} x {mb.size.Z:.1f}"
          f" / 부피 {m.volume/1000:.0f} cm3")
    print(f"  치수는 세 수로만 정해진다 — 전폭 {MNT_W:.0f} / 깊이 {MNT_D:.0f} /"
          f" 벽 {WALL:.0f}. 블록·기둥·ㄷ자가 전부 같은 폭·깊이라 턱이 없다")
    print(f"  위 — **ㄷ자로 레일을 감싼다** (벽 {WALL:.0f}t). 밑면 M6 x2 + 윗면 M6 x2"
          f" (x=±{BOLT_X:.0f}), 스프링너트 + 육각렌지볼트")
    print(f"       레일 높이 {S}mm 를 팔로 쓰는 커플 → 횡력 18.6 N·m 에 볼트당"
          f" {18600/S/2:.0f} N. 밑면만 물면 프라잉이라 훨씬 불리하다")
    print(f"       레일을 타고 미끄러진다 → 휠베이스 조정 후 그 자리에서 고정")
    print(f"       볼트는 기둥 안쪽(x=±{BOLT_X:.0f}) — 기둥 안쪽면 {ARM_X-WALL/2:.0f} 까지"
          f" {ARM_X-WALL/2-BOLT_X-HEX_R:.0f}mm 여유. 기둥 위에 두면 구멍이 물리고 렌치도 막힌다")
    print(f"       조립 순서: 마운트를 레일에 먼저 물린 뒤 바퀴·클램프를 넣는다")
    print(f"  아래 — 블록 {MNT_W:.0f}x{MNT_D:.0f}x{BLK_H:.0f}, 축 반원 R{SHAFT_D/2+0.15:.2f}"
          f" 받침 (축 밑 살 {BLK_H-SHAFT_D/2-0.15:.2f})")
    print(f"       클램프 고정 M{CLAMP_BOLT:.0f} 탭 4곳 (하공 Ø{MNT_BOLT})"
          f" — 장공 Ø{CL_SLOT_W} 에 여유 {CL_SLOT_W-CLAMP_BOLT:.2f}mm, 와셔 필수")
    print(f"       볼트당 하중 {29000/(4*CL_BOLT[0]):.0f} N (구동반력 29 N·m / x=±{CL_BOLT[0]} 커플)"
          f" → M{CLAMP_BOLT:.0f} 8.8 대비 여유 충분")
    print(f"       맞댐면 z={AXLE_Z:.2f} (축 중심) — 여기가 어긋나면 클램프가 평면을 못 문다")

    for _n, _a, _b in (("마운트x클램프", m, cl), ("마운트x바퀴", m, w), ("클램프x바퀴", cl, w),
                       ("마운트x레일", m, r), ("클램프x레일", cl, r)):
        print(f"  간섭 {_n}: {_ivol(_a, _b):.0f} mm3")

    if not os.environ.get("NO_EXPORT"):
        _m = mount()                      # show 에 넘긴 것과 다른 사본 (재부모화 회피)
        # 파일명에 판을 박는다. 예전엔 둘 다 "휠마운트.step" 로 나가서 PRINT 실행이
        # CNC 파일을 조용히 덮어썼다 — 어느 판인지 파일만 봐선 알 수 없었다.
        _tag = f"_print_M{CLAMP_BOLT:.0f}" if PRINT else f"_cnc_M{CLAMP_BOLT:.0f}"
        _cp = coupon()
        _cut = _m                         # DXF 단면은 **로봇 좌표** 기준이라 눕히기 전 것을 쓴다
        if PRINT:                         # 슬라이서에 그대로 넣도록 눕혀서 낸다
            _m, _cp = bed(_m), bed(_cp)
        export_stl(_cp, str(EXPORT / f"휠마운트_끼움쿠폰{_tag}.stl"))
        export_step(_m, str(EXPORT / f"휠마운트{_tag}.step"))
        try:
            export_stl(_m, str(EXPORT / f"휠마운트{_tag}.stl"))
            _stl = " + STL"
        except Exception as e:
            _stl = f" (STL 실패: {e})"
        print(f"\n내보내기 → {EXPORT}   [{'PRINT (FDM 보정)' if PRINT else 'CNC (기본)'}]")
        print(f"  휠마운트_끼움쿠폰{_tag}.stl   ㄷ자만 {_cp.bounding_box().size.X:.0f}mm 폭"
              f" / {_cp.volume/1000:.0f} cm3 — **이걸 먼저 뽑아 레일에 끼워봐라**")
        print(f"     슬롯 {S+RAIL_FIT:.1f} / 레일볼트 Ø{RAIL_BOLT} / 본품과 같은 자세·설정으로")
        print(f"  휠마운트.step{_stl}   {m.volume/1000:.0f} cm3"
              f" / PETG 40% 인필 약 {m.volume*1.27e-3*0.5:.0f} g"
              f" / 알루미늄 {m.volume*2.7e-6:.2f} kg")
        for _n, _pl in () if PRINT else (("정면_XZ", Plane(origin=(0, CLAMP_Y, 0), z_dir=(0, 1, 0))),
                        ("측면_YZ", Plane(origin=(0, 0, 0), z_dir=(1, 0, 0))),
                        ("평면_XY", Plane(origin=(0, 0, AXLE_Z - BLK_H / 2),
                                          z_dir=(0, 0, 1)))):
            a = _dxf_section(_cut, _pl, EXPORT / f"휠마운트_{_n}{_tag}.dxf")
            print(f"  휠마운트_{_n}{_tag}.dxf   단면적 {a/100:.0f} cm2" if a else
                  f"  휠마운트_{_n}{_tag}.dxf   (단면 없음)")
        print(f"  ※ DXF 는 **도면·검토용**이다. 이 부품은 평판이 아니라 레이저로 못 만든다")
        # 자세는 **눕히는 쪽**이다. 처음엔 세우라고 적었는데 반대였다.
        #   웹면(y=MNT_Y1)이 부품 실루엣 전체를 덮는 단일 평면 64.2 cm2 라, 여기 눕히면
        #   서포트 0 / 높이 40mm. 세우면 서포트 39.6 cm2 / 높이 151mm 다.
        #   더 중요한 건 응력 방향 — 굽힘은 sigma_zz 인데 눕히면 그게 층 안쪽이 된다.
        #   층간 인장 25 MPa 가 아니라 소재 인장 45~50 MPa 를 쓴다 (여유 8배 → 13배).
        #   M8 인서트도 축이 층 평면 안이라 뽑힘이 층을 벌리지 않는다.
        print(f"  프린트: **웹면(y={MNT_Y1:.0f})을 바닥에 눕힌다** — 접촉 64 cm2, 높이"
              f" {MNT_D:.0f}mm, 서포트 0")
        print(f"     세우면 서포트 40 cm2 에 응력이 층간 인장이 된다. 눕히면 층과 나란하다")
        print(f"     벽 4겹+, 인필 40%+. M8 은 탭 대신 황동 인서트")
        # 여기 "최대 응력 3.5 MPa / 여유 13배" 라고 적어뒀었다. **틀린 값이었다.**
        # 접지점 횡력만 봤고, 지배 하중인 **수직하중의 55mm 오프셋**을 안 봤다.
        # 실물은 그 상태로 사람이 올라탔다가 ㄷ자 팔에서 부러졌다.
        # 이제 report_loads() 가 경로 전체를 훑는다 — 숫자는 거기서 본다.
        report_loads("PETG")

    if not os.environ.get("NOSHOW"):
        try:
            set_port(3939)
            show(r, m, cl, w, progress=None,
                 names=["하부사이드레일", "휠마운트", "기존클램프", "바퀴"],
                 colors=["#c8ccd0", "#6b7480", "#c62828", "#2b2b2b"],
                 reset_camera=Camera.RESET)
        except Exception as e:
            print(f"뷰어에 표시하지 못함 (뷰어 미실행?): {e}")
