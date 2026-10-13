# Design D — **기존 호버보드 클램프 + 새 하부 클램프**, 레일 밑면에 매단다.
#
# 구조:
#   기존 클램프(상부)  D컷을 채널로 문다 → 반력토크. 이미 손에 4개 있다.
#   하부 클램프(신규)  축을 반원으로 받치고 상부와 M6 x4 로 물린다.
#                      다리 2개가 상부 클램프를 타고 넘어 레일 밑면까지 올라간다.
#   레일 3030          다리 윗면이 레일 밑면에 지압으로 닿고 **밑면 볼트 2개**로 잡는다.
#
# 옆 플랜지를 왜 뺐나 (2026-08-21 실물 시험).
#   설계상으로는 옆면 슬롯을 물면 기울임 모멘트가 전단으로 가서 유리하다
#   (밑면만: 볼트당 1433 N 인발 = M6 스프링너트 보수 하한 1500 N 의 96%.
#    옆면까지: 716 N 전단 = M6 전단 8000 N 의 9%).
#   그런데 출력해서 달아 보니 **지배 하중은 누르는 방향**이었고, 그건 볼트가 아니라
#   다리 윗면 ↔ 레일 밑면 지압이 받는다. 힘은 축 → 반원 시트 → 기존 클램프로 흐르고
#   그 경로는 살이 두꺼워 멀쩡했다. 정작 얇은 플랜지는 툭 치니까 부러졌다.
#   불리해지는 경우는 **프레임을 위로 들어 올릴 때**뿐이다. 그때는 밑면 볼트 2개가
#   바퀴 무게(약 3 kg → 30 N)를 인장으로 받는데, 이건 인발 한계의 1% 다.
#   숫자는 report() 가 매번 찍는다. 되살리려면 parameters.LOW_SIDE = True.
#
# 조립 순서 (위쪽 볼트 접근이 레일에 막히므로 순서가 강제된다):
#   ① 축을 하부 클램프 시트에 앉힌다
#   ② 상부(기존) 클램프를 덮고 M6 x4 를 **위에서** 조인다 — 이때 레일은 아직 없다
#   ③ 레일 슬롯에 스프링너트를 넣고 레일을 옆에서 밀어 넣는다
#   ④ 밑면 2볼트를 카운터보어로 올려 조인다 (M6x15, 카운터보어 Ø16 x 32.3)

from build123d import *

import parameters as P

AZ = 0.0                                  # 축 중심 = z 데이텀 (패키지 공통)
GROUND = -P.AXLE_Z                        # 지면 z
FLAT_H = P.AXLE_ACROSS_FLAT - P.AXLE_DIAMETER / 2          # 평면 높이 5.0
CLAMP_SEAT_Z = FLAT_H - P.OEM_CH_D        # 기존 클램프가 실제 앉는 z (= -0.07)
CLAMP_CY = P.AXLE_FLAT_Y0 - P.AXLE_FLAT_LENGTH / 2         # 평면 한가운데

# 하부 블록 윗면은 축 중심보다 핀치갭만큼 낮다. 안 낮추면 상부 클램프가 축(평면
# z=+5.0)이 아니라 하부 블록 윗면에 먼저 앉아, 볼트만 조여지고 축은 미끄러진다.
LOW_TOP = AZ - P.SADDLE_PINCH
LOW_BOT = LOW_TOP - P.LOW_H
RAIL_BOT = CLAMP_SEAT_Z + P.OEM_T + P.LOW_RAIL_CLR         # 레일 밑면
RAIL_CZ = RAIL_BOT + P.EXTRUSION_H / 2                     # 레일 중심
FLANGE_Y = P.EXTRUSION_W / 2 + P.LOW_RAIL_FIT              # 플랜지 안쪽면
FLANGE_TOP = RAIL_CZ + P.TNUT_BOLT + 4                     # 옆 볼트 위로 남기는 살
BOLT_Z = RAIL_CZ                                           # 옆면 슬롯 (3030 은 중앙 1줄)
# H 크로스 선반 — 클램프 안쪽면에서 안쪽(-y)으로 뻗어 크로스 밑면을 문다.
SHELF_FACE = CLAMP_CY - P.LOW_D / 2                        # 클램프 안쪽면 y (선반 뿌리)
TONGUE = P.LOW_SHELF and P.SHELF_KIND == "tongue"
SHELF_TOP = RAIL_BOT                                       # 윗면은 다리 윗면과 같은 높이
SHELF_BOT = LOW_BOT if TONGUE else RAIL_BOT - P.SHELF_T    # 혀는 전체 높이
SHELF_W = P.TONGUE_SHELF_W if TONGUE else (P.SHELF_W or P.LOW_W)
SHELF_BOLT_YS = tuple(SHELF_FACE - b for b in P.SHELF_BOLT_IN)
SHELF_CBORE = P.LOW_CBORE if TONGUE else P.SHELF_CBORE     # 혀는 레일볼트와 같은 스택
# 케이블 통로 — 포켓 바닥에서 직선 S 만큼 가다가 반경 R 로 꺾여 옆면 x=±W/2 으로 나간다.
# 원호 중심 (±R, y_bend-R). 옆면과 만나는 y: y_exit = y_bend - sqrt(R² - (R - W/2)²)
CABLE_Y0 = SHELF_FACE - P.SHELF_NOTCH[1]                   # 포켓 바닥 (통로 시작)
CABLE_Y_BEND = CABLE_Y0 - P.SHELF_CABLE_STRAIGHT           # 꺾임 시작
CABLE_Y_EXIT = (CABLE_Y_BEND - (P.SHELF_CABLE_R ** 2 - (P.SHELF_CABLE_R - SHELF_W / 2) ** 2) ** 0.5
                if TONGUE and P.SHELF_CABLE else None)

assert P.LOW_POST_X - P.LOW_POST_W / 2 > P.OEM_W / 2, "다리가 기존 클램프와 부딪힌다"
assert RAIL_BOT > CLAMP_SEAT_Z + P.OEM_T, "레일이 기존 클램프를 파고든다"
assert P.LOW_CBORE[0] + 2 <= P.LOW_POST_W, "카운터보어가 다리보다 커서 벽이 안 남는다"
assert P.LOW_H > P.AXLE_DIAMETER / 2 + 6, "축 시트 아래 살이 6mm 도 안 남는다"
# 일체형 T 텅 — 카운터보어 양옆 자투리에 한 조각씩 (다리당 2, 총 4조각)
_cb0 = P.LOW_POST_X - P.LOW_CBORE_D / 2
_cb1 = P.LOW_POST_X + P.LOW_CBORE_D / 2
_lg0 = P.LOW_POST_X - P.LOW_POST_W / 2
_lg1 = P.LOW_POST_X + P.LOW_POST_W / 2
TONGUE_XS = ((_lg0 + _cb0) / 2, (_cb1 + _lg1) / 2)     # 안쪽·바깥쪽 조각 중심
TONGUE_NECK = P.EXTRUSION_SLOT - P.TONGUE_FIT          # 목 폭 (슬롯 입구)
TONGUE_HEAD = P.EXTRUSION_CAVITY - P.TONGUE_FIT        # 머리 폭 (T 홈)
TONGUE_NECK_H = P.EXTRUSION_WALL + 0.3                 # 립을 통과하는 높이
TONGUE_HEAD_H = ((P.EXTRUSION_W / 2 - P.EXTRUSION_WALL)
                 - P.EXTRUSION_CORE / 2) - 0.8         # 캐비티 안 높이

if P.TONGUE:
    for _x in TONGUE_XS:
        assert _x - P.TONGUE_LEN / 2 > min(_lg0, _cb1) - 1e-6 or _x < P.LOW_POST_X, ""
    _in0, _in1 = TONGUE_XS[0] - P.TONGUE_LEN / 2, TONGUE_XS[0] + P.TONGUE_LEN / 2
    _ou0, _ou1 = TONGUE_XS[1] - P.TONGUE_LEN / 2, TONGUE_XS[1] + P.TONGUE_LEN / 2
    assert _in0 > _lg0 and _in1 < _cb0, "안쪽 텅이 다리/카운터보어와 겹친다"
    assert _ou0 > _cb1 and _ou1 < _lg1, "바깥 텅이 카운터보어/다리와 겹친다"

TIE_GAP = 2 * (210.0 - P.LOW_D / 2)              # 좌우 클램프 안쪽면 사이
TIE_GRIP = (P.TIE_ROD_LEN - TIE_GAP) / 2         # 한쪽 물림 (봉 길이에서 나온다)

if P.TIE:
    assert 8.0 <= TIE_GRIP <= P.TIE_TAP_DEPTH - 1.0, (
        f"봉 {P.TIE_ROD_LEN:.0f}mm → 한쪽 물림 {TIE_GRIP:.1f}mm. "
        f"8~{P.TIE_TAP_DEPTH-1:.0f}mm 사이여야 한다")
    _r = (P.TIE_ROD - 1.2) / 2                       # 탭 하공 반경
    assert P.TIE_Z - _r > LOW_BOT + 2.0, (
        f"타이 탭이 블록 밑면까지 {P.TIE_Z - _r - LOW_BOT:.1f}mm 뿐이다")
    assert P.TIE_Z + _r < AZ - (P.AXLE_DIAMETER / 2 + P.SADDLE_SEAT_FIT) - 2.0 or         P.TIE_X - _r > P.AXLE_DIAMETER / 2, (
        "타이 탭이 축 시트를 뚫는다")
    _to_oem = P.TIE_X - _r - (P.OEM_BOLT[0] + (P.OEM_BOLT_D - 1.0) / 2)
    _to_rail = (P.LOW_POST_X - P.TNUT_BOLT * 0.55) - (P.TIE_X + _r)
    assert min(_to_oem, _to_rail) > 2.0, (
        f"타이 탭 벽이 얇다 — 상부탭까지 {_to_oem:.1f}, 레일볼트공까지 {_to_rail:.1f}mm")


def rail_bolt_len():
    """밑면 볼트 필요 길이 — 카운터보어 바닥에서 스프링너트까지 + 최소 물림."""
    grip = RAIL_BOT - (LOW_BOT + P.LOW_CBORE[1])           # 머리 밑 ~ 레일 밑면
    return grip + P.EXTRUSION_WALL + P.LOW_MIN_ENGAGE


def side_bolt_len():
    return P.LOW_FLANGE_T + P.EXTRUSION_WALL + P.LOW_MIN_ENGAGE


# 이걸 지켜주는 게 없어서 실물에서 걸렸다 — 카운터보어 27.3 은 M6x20 전용이었는데
# 손에 있는 볼트가 15 라 밑에서 끼워도 스프링너트에 안 닿았다. LOW_H 나
# LOW_RAIL_CLR 을 건드리면 조용히 또 안 닿으므로 여기서 막는다.
assert rail_bolt_len() <= P.LOW_BOLT_LEN + 1e-6, (
    f"밑면 볼트가 {rail_bolt_len():.2f}mm 필요한데 가진 건 M{P.TNUT_BOLT:.0f}x"
    f"{P.LOW_BOLT_LEN:.0f} 이다 — 스프링너트에 안 닿는다")
assert not P.LOW_SIDE or side_bolt_len() <= P.LOW_SIDE_BOLT_LEN + 1e-6, (
    f"옆면 볼트가 {side_bolt_len():.2f}mm 필요하다")


def cable_channel(side=None):
    """혀 안의 케이블 통로 솔리드 (빼기용). 축 끝 보어에서 나온 케이블이 지나갈 길 —
    곧게 S 만큼 가다가 반경 R 로 90° 꺾어 옆면(±x)으로 나간다.

    side: +1/-1 (None 이면 parameters.SHELF_CABLE_SIDE). 차량에 달면 좌우 클램프가 z 축
    180° 로 돌아가므로 네 바퀴 케이블을 전부 차량 안쪽으로 내려면 **두 손잡이가 필요**하다
    — cable_side_for() 가 바퀴 위치에서 골라 준다.
    """
    sx = 1 if (P.SHELF_CABLE_SIDE if side is None else side) >= 0 else -1
    R = P.SHELF_CABLE_R
    with BuildPart() as c:
        with BuildLine() as cpath:
            l1 = Line((0, CABLE_Y0 + 1.0, AZ), (0, CABLE_Y_BEND, AZ))
            a1 = TangentArc(l1 @ 1, (sx * R, CABLE_Y_BEND - R, AZ), tangent=l1 % 1)
            Line(a1 @ 1, (sx * (SHELF_W / 2 + 2.0), CABLE_Y_BEND - R, AZ))
        with BuildSketch(Plane(origin=(0, CABLE_Y0 + 1.0, AZ), z_dir=(0, -1, 0))):
            Circle(P.SHELF_CABLE_D / 2)
        sweep(path=cpath.line)
    return c.part


def cable_side_for(x, sy):
    """차량 좌표 (축 x 부호, 레일 y 부호) → 케이블이 차량 중심(x=0) 쪽으로 나가는 로컬 옆면.

    로컬 +x 는 +y 쪽 클램프에서 차량 +x, -y 쪽(180° 회전)에서 차량 -x 다.
    안쪽 = -sign(x) 방향이므로  side = -sign(x) * sign(sy).
    """
    sgn = lambda v: 1 if v >= 0 else -1
    return -sgn(x) * sgn(sy)


def low_clamp(cable_side=None):
    """각기둥 하나에서 필요한 것만 파낸다. cable_side 는 케이블 통로 옆면(+1/-1, None=파라미터).

    처음엔 블록 + 다리 + 플랜지를 따로 쌓아 올렸다. 그랬더니 플랜지(바깥면 ±23.3)가
    몸체(±20)보다 3.3mm 튀어나오고, z=14.5 에서 18x40 기둥이 갑자기 18x8 지느러미로
    줄어드는 단차가 생겼다 — 가공 중에 떨리기 좋은 형상이다. 지금은 **바깥 윤곽이
    각기둥 하나**이고, 파내는 건 셋뿐이다:
        ① 다리 사이 통로 (기존 클램프가 위에서 내려가는 길)
        ② 레일 홈 (레일이 X 로 지나가는 길)
        ③ 축 반원 시트
    """
    r = P.AXLE_DIAMETER / 2 + P.SADDLE_SEAT_FIT
    with BuildPart() as p:
        with Locations((0, CLAMP_CY, (LOW_BOT + FLANGE_TOP) / 2)):       # 통짜 각기둥
            Box(P.LOW_W, P.LOW_D, FLANGE_TOP - LOW_BOT)
        # ① 다리 사이 통로 — 여기가 막히면 기존 클램프를 넣을 수가 없다.
        #    위로 끝까지 뚫려 있어야 클램프를 수직으로 떨어뜨릴 수 있다.
        with Locations((0, CLAMP_CY, (LOW_TOP + FLANGE_TOP) / 2 + 1)):
            Box(P.LOW_CLAMP_GAP, P.LOW_D + 4, FLANGE_TOP - LOW_TOP + 2,
                mode=Mode.SUBTRACT)
        with Locations((0, CLAMP_CY, (RAIL_BOT + FLANGE_TOP) / 2 + 1)):  # ② 레일 홈
            Box(P.LOW_W + 4,
                2 * FLANGE_Y if P.LOW_SIDE else P.LOW_D + 4,             # 플랜지 없으면 통째로
                FLANGE_TOP - RAIL_BOT + 2, mode=Mode.SUBTRACT)
        with Locations((0, CLAMP_CY, AZ)):                               # ③ 축 반원 시트
            Cylinder(r, P.LOW_D + 4, rotation=(90, 0, 0), mode=Mode.SUBTRACT)
        if TONGUE:                                                       # H 크로스 혀
            # 크로스 폭 그대로, 몸체 전체 높이. 윗면이 다리 윗면(레일 밑면)과 같은
            # 높이라 크로스 밑면에 지압으로 닿고, 크로스 밑면 슬롯(x=0) 에 볼트 2개.
            _h = SHELF_TOP - SHELF_BOT
            with Locations((0, SHELF_FACE - P.SHELF_REACH / 2, (SHELF_TOP + SHELF_BOT) / 2)):
                Box(SHELF_W, P.SHELF_REACH, _h)
            # 뿌리 세로 모서리 (x=±W/2, y=SHELF_FACE) — 몸체 가운데 토막 안쪽면과 만나는 곳
            _corners = [e for e in p.edges().filter_by(Axis.Z)
                        if abs(abs(e.center().X) - SHELF_W / 2) < 0.05
                        and abs(e.center().Y - SHELF_FACE) < 0.05
                        and LOW_BOT - 0.1 < e.center().Z < LOW_TOP + 0.1]
            if _corners:
                try:
                    fillet(_corners, radius=P.TONGUE_ROOT_R)
                except Exception:
                    pass
            # 축 끝(안쪽면보다 1.6mm 돌출)을 피하는 포켓. **둥근 보어**로 판다 — 네모 포켓은
            # 옆벽(x=±10)이 몸체 안쪽면·윗면과 만나는 세로 모서리에 FEA 최대 응력이 몰렸고,
            # 그 오목 모서리는 몸체 윗면(z=-0.4)에서 끝나 OCCT fillet 도 안 먹었다. 축이
            # 둥그니 여유도 둥글게 주면 모서리 자체가 없다.
            pk = P.AXLE_DIAMETER + 2 * P.SHELF_CLR
            with Locations((0, SHELF_FACE - P.SHELF_NOTCH[1] / 2, AZ)):
                Cylinder(pk / 2, P.SHELF_NOTCH[1], rotation=(90, 0, 0), mode=Mode.SUBTRACT)
            if P.SHELF_CABLE:                                            # 케이블 통로
                add(cable_channel(cable_side), mode=Mode.SUBTRACT)
            for by in SHELF_BOLT_YS:                                     # 볼트 2 + 카운터보어
                with Locations((0, by, (SHELF_TOP + SHELF_BOT) / 2)):
                    Cylinder(P.TNUT_BOLT * 0.55, _h + 2, mode=Mode.SUBTRACT)
                with Locations((0, by, SHELF_BOT + SHELF_CBORE[1] / 2 - 0.01)):
                    Cylinder(SHELF_CBORE[0] / 2, SHELF_CBORE[1], mode=Mode.SUBTRACT)
        elif P.LOW_SHELF:                                                # H 크로스 선반 (wide)
            # 안쪽면에서 뻗는 판. 윗면이 다리 윗면(레일 밑면)과 같은 높이라 크로스
            # 밑면에 지압으로 닿고, 크로스 밑면 슬롯(x=0) 에 볼트 2개로 물린다.
            with Locations((0, SHELF_FACE - P.SHELF_REACH / 2, (SHELF_TOP + SHELF_BOT) / 2)):
                Box(SHELF_W, P.SHELF_REACH, P.SHELF_T)
            # 축 끝이 안쪽면보다 튀어나온다 — 뿌리에 노치
            nx, ny = P.SHELF_NOTCH
            with Locations((0, SHELF_FACE - ny / 2 + 0.01, (SHELF_TOP + SHELF_BOT) / 2)):
                Box(nx, ny + 0.02, P.SHELF_T + 2, mode=Mode.SUBTRACT)
            # 노치 안쪽 모서리 (x=±nx/2, y=SHELF_FACE-ny) 의 세로 모서리만 둥글린다
            _ny = SHELF_FACE - ny
            _corners = [e for e in p.edges().filter_by(Axis.Z)
                        if abs(abs(e.center().X) - nx / 2) < 0.05 and abs(e.center().Y - _ny) < 0.05
                        and SHELF_BOT - 0.1 < e.center().Z < SHELF_TOP + 0.1]
            if _corners:
                fillet(_corners, radius=P.SHELF_NOTCH_R)
            for by in SHELF_BOLT_YS:                                     # 볼트 2 + 카운터보어
                with Locations((0, by, (SHELF_TOP + SHELF_BOT) / 2)):
                    Cylinder(P.TNUT_BOLT * 0.55, P.SHELF_T + 2, mode=Mode.SUBTRACT)
                with Locations((0, by, SHELF_BOT + SHELF_CBORE[1] / 2 - 0.01)):
                    Cylinder(SHELF_CBORE[0] / 2, SHELF_CBORE[1], mode=Mode.SUBTRACT)
        if P.TONGUE:                                                     # 일체형 T 텅
            # 목은 슬롯 입구를 지나고, 머리가 립 밑에 걸린다. 스프링너트가 하던
            # 일을 클램프 몸체가 직접 한다 — 물림 길이가 4배로 는다.
            for sx in (-1, 1):
                for tx in TONGUE_XS:
                    with Locations((sx * tx, CLAMP_CY,
                                    RAIL_BOT + TONGUE_NECK_H / 2)):
                        Box(P.TONGUE_LEN, TONGUE_NECK, TONGUE_NECK_H)
                    with Locations((sx * tx, CLAMP_CY,
                                    RAIL_BOT + TONGUE_NECK_H + TONGUE_HEAD_H / 2)):
                        Box(P.TONGUE_LEN, TONGUE_HEAD, TONGUE_HEAD_H)
        for sx in (-1, 1):                                               # 상부 클램프 탭 4
            for sy in (-1, 1):
                with Locations((sx * P.OEM_BOLT[0], CLAMP_CY + sy * P.OEM_BOLT[1],
                                (LOW_BOT + LOW_TOP) / 2)):
                    Cylinder((P.OEM_BOLT_D - 1.0) / 2, P.LOW_H + 2, mode=Mode.SUBTRACT)
        if P.TIE and P.TIE_KIND == "rod":                                 # 전산볼트 탭
            for sx in ((-1, 1) if P.TIE_N == 2 else (0,)):
                with Locations((sx * P.TIE_X, CLAMP_CY - P.LOW_D / 2 + P.TIE_ENGAGE / 2,
                                P.TIE_Z)):
                    Cylinder((P.TIE_ROD - 1.2) / 2, P.TIE_ENGAGE,
                             rotation=(90, 0, 0), mode=Mode.SUBTRACT)
        elif P.TIE:                                                      # 코너 브래킷 탭
            # 3030 타이를 옆에서 물리는 브래킷용. x 는 상부클램프 탭(17.6)과
            # 카운터보어(26.4) 사이 창의 한가운데뿐이라 자리가 좁다.
            for sx in (-1, 1):
                for bz in P.TIE_BRK_Z:
                    with Locations((sx * P.TIE_BRK_X,
                                    CLAMP_CY - P.LOW_D / 2 + P.TIE_BRK_TAP / 2, bz)):
                        Cylinder((P.OEM_BOLT_D - 1.0) / 2, P.TIE_BRK_TAP,
                                 rotation=(90, 0, 0), mode=Mode.SUBTRACT)
        for sx in (-1, 1):                                               # 레일 밑면 볼트 2
            with Locations((sx * P.LOW_POST_X, CLAMP_CY,
                            (LOW_BOT + RAIL_BOT) / 2)):
                Cylinder(P.TNUT_BOLT * 0.55, RAIL_BOT - LOW_BOT + 2, mode=Mode.SUBTRACT)
            with Locations((sx * P.LOW_POST_X, CLAMP_CY,
                            LOW_BOT + P.LOW_CBORE[1] / 2 - 0.01)):
                Cylinder(P.LOW_CBORE[0] / 2, P.LOW_CBORE[1], mode=Mode.SUBTRACT)
        if P.LOW_SIDE:                                                   # 레일 옆면 볼트 4
            for sx in (-1, 1):
                for sy in (-1, 1):
                    with Locations((sx * P.LOW_POST_X,
                                    CLAMP_CY + sy * (FLANGE_Y + P.LOW_FLANGE_T / 2),
                                    BOLT_Z)):
                        Cylinder(P.TNUT_BOLT * 0.55, P.LOW_FLANGE_T + 4,
                                 rotation=(90, 0, 0), mode=Mode.SUBTRACT)
    p.part.label = "low_clamp"
    return p.part


def tie_profile(rail_y=210.0):
    """3030 여분을 잘라 만든 타이. 좌우 클램프 사이에 끼우고 코너 브래킷으로 문다."""
    from extrusion_reference import extrusion_reference
    if not P.TIE or P.TIE_KIND != "profile":
        return []
    prof = Rot(0, 0, 90) * extrusion_reference(P.TIE_PROF_LEN)   # X → Y 로 눕힌다
    prof = Pos(0, 0, P.TIE_PROF_Z) * prof
    prof.label = f"3030_tie_{P.TIE_PROF_LEN:.0f}"
    return [prof]


def tie_brackets(rail_y=210.0):
    """3030 타이를 클램프 안쪽면에 붙이는 코너 브래킷 (한쪽 2개, 총 4개).

    L 자 판이다. 한쪽 다리는 **클램프 안쪽면**(y 법선)에 붙어 M6 탭 2개로 조이고,
    다른 다리는 **프로파일 옆면**(x 법선)에 붙어 슬롯 T너트로 조인다.
    프로파일 끝과 클램프 면 사이 간극은 브래킷이 건넌다.
    """
    if not P.TIE or P.TIE_KIND != "profile":
        return []
    face = rail_y - P.LOW_D / 2                    # 클램프 안쪽면 y
    pend = P.TIE_PROF_LEN / 2                      # 프로파일 끝 y
    px = P.EXTRUSION_W / 2                         # 프로파일 옆면 x
    t_ = 6.0                                       # 브래킷 두께
    leg = 30.0                                     # 프로파일 쪽 다리 길이
    zc = P.TIE_PROF_Z
    h = 26.0                                       # 브래킷 높이 (z)
    out = []
    for sy in (-1, 1):
        for sx in (-1, 1):
            with BuildPart() as b:
                with Locations((sx * (px + t_ / 2), sy * (face - t_ / 2), zc)):
                    Box(t_, t_, h)                                  # 코너
                with Locations((sx * (px + t_ / 2),
                                sy * (face - t_ / 2 - (leg - t_) / 2 - t_ / 2), zc)):
                    Box(t_, leg - t_, h)                            # 프로파일 쪽 다리
                with Locations((sx * (px + t_ / 2 + (P.TIE_BRK_X - px) / 1.0),
                                sy * (face - t_ / 2), zc)):
                    Box(2 * (P.TIE_BRK_X - px) + t_, t_, h)         # 클램프 쪽 다리
                for bz in P.TIE_BRK_Z:              # 클램프 탭 통과공
                    with Locations((sx * P.TIE_BRK_X, sy * (face - t_ / 2), bz)):
                        Cylinder(P.OEM_BOLT_D * 0.55, t_ + 2,
                                 rotation=(90, 0, 0), mode=Mode.SUBTRACT)
                with Locations((sx * (px + t_ / 2), sy * (face - leg / 2), zc)):
                    Cylinder(P.OEM_BOLT_D * 0.55, t_ + 2,
                             rotation=(0, 90, 0), mode=Mode.SUBTRACT)
            b.part.label = f"tie_bracket_{len(out)+1}"
            out.append(b.part)
    return out


def tie_rods(rail_y=210.0):
    """좌우 클램프를 잇는 인장 전산볼트 (차량 좌표). 클램프 탭에 직접 박힌다."""
    if not P.TIE or P.TIE_KIND != "rod":
        return []
    inner = rail_y - P.LOW_D / 2
    L = P.TIE_ROD_LEN
    out = []
    for sx in ((-1, 1) if P.TIE_N == 2 else (0,)):
        with BuildPart() as p:
            with Locations((sx * P.TIE_X, 0, P.TIE_Z)):
                Cylinder(P.TIE_ROD / 2, L, rotation=(90, 0, 0))
        p.part.label = f"M{P.TIE_ROD:.0f}_tie_rod_{len(out) + 1}"
        out.append(p.part)
    return out


def tie_force(w=None):
    """타이 인장력과 분담률. (부담률, 인장 N, 레일볼트 N)

    분담은 **회전강성 비**로 갈린다. k_joint 는 FEA(볼트구멍 강체 고정) 값이라
    실제보다 뻣뻣한 상한이다 — 진짜 T너트/슬롯은 더 물러서 타이가 더 가져간다.
    보수적으로 상한을 쓴다.
    """
    w = w or P.DESIGN_W
    m = w * abs(CLAMP_CY)
    if not P.TIE:
        return 0.0, 0.0, m / (P.EXTRUSION_W / 2) / 2
    # 전산볼트는 **강철**이다. 68900(알루미늄)으로 계산했다가 타이 강성을
    # 3배 낮게 봤다 — 분담률이 실제보다 작게 나왔다.
    E = 206000.0
    At = {6.0: 20.1, 8.0: 36.6, 10.0: 58.0, 12.0: 84.3}[P.TIE_ROD]   # 나사 유효단면
    d = RAIL_BOT - P.TIE_Z
    # 자유 길이는 간격 + 물림 한쪽분으로 본다 (봉 전체 길이가 아니다)
    L = TIE_GAP + TIE_GRIP
    k_tie = 2 * (E * At * P.TIE_N / L) * d * d
    k_joint = m / (0.0206 / 14.5)
    share = k_tie / (k_tie + k_joint)
    return share, m * share / d / P.TIE_N, m * (1 - share) / (P.EXTRUSION_W / 2) / 2


def oem_clamp_block():
    """기존 호버보드 클램프 (참조) — 축 위에 얹혀 D컷을 문다."""
    with BuildPart() as p:
        with Locations((0, CLAMP_CY, CLAMP_SEAT_Z + P.OEM_T / 2)):
            Box(P.OEM_W, P.OEM_H, P.OEM_T)
        with Locations((0, CLAMP_CY, CLAMP_SEAT_Z)):
            Box(P.AXLE_DIAMETER, P.OEM_H + 2, 2 * P.OEM_CH_D, mode=Mode.SUBTRACT)
        for sx in (-1, 1):
            for sy in (-1, 1):
                with Locations((sx * P.OEM_BOLT[0], CLAMP_CY + sy * P.OEM_BOLT[1],
                                CLAMP_SEAT_Z + P.OEM_T / 2)):
                    Cylinder((P.OEM_BOLT_D + 0.5) / 2, P.OEM_T + 2,
                             mode=Mode.SUBTRACT)
    p.part.label = "oem_clamp_reference"
    return p.part


def rail():
    from extrusion_reference import extrusion_reference
    return Pos(0, CLAMP_CY, RAIL_CZ) * extrusion_reference()


def button_head_screw_reference(length=20.0):
    """M6 둥근머리 렌치볼트 참조 형상 (ISO 7380 계열, 나사산 생략).

    로컬 +Z가 머리 밑면에서 나사 진행 방향이다. 길이는 머리 밑 기준이다.
    """
    shank_d = 6.0
    head_d, head_h = 10.5, 3.3
    socket_af, socket_depth = 4.0, 2.0
    with BuildPart() as p:
        with Locations((0, 0, length / 2)):
            Cylinder(shank_d / 2, length,
                     align=(Align.CENTER, Align.CENTER, Align.CENTER))
        with Locations((0, 0, -head_h / 2)):
            Cylinder(head_d / 2, head_h,
                     align=(Align.CENTER, Align.CENTER, Align.CENTER))
        # 머리 바깥쪽 윗모서리만 둥글게 만든다. 체결면은 평면으로 유지한다.
        try:
            top_edge = p.edges().filter_by(GeomType.CIRCLE).sort_by(Axis.Z)[0]
            fillet(top_edge, radius=1.2)
        except Exception:
            pass
        with BuildSketch(Plane.XY.offset(-head_h - 0.01)):
            RegularPolygon(socket_af / (3 ** 0.5), 6)
        extrude(amount=socket_depth, mode=Mode.SUBTRACT)
    p.part.label = f"M6x{length:g}_button_head_screw_reference"
    return p.part


def side_button_head_bolts_reference(length=None):
    """3030 양옆 슬롯을 체결하는 M6 둥근머리 렌치볼트 4개.

    LOW_SIDE 를 끄면 플랜지가 없으니 **빈 목록**이다. 안 그러면 허공에 볼트가 뜬다.
    """
    if not P.LOW_SIDE:
        return []
    length = P.LOW_BOLT_LEN if length is None else length
    out = []
    y_outer = FLANGE_Y + P.LOW_FLANGE_T
    for sx in (-1, 1):
        for sy in (-1, 1):
            screw = (Pos(sx * P.LOW_POST_X, CLAMP_CY + sy * y_outer, BOLT_Z)
                     * Rot(sy * 90, 0, 0)
                     * button_head_screw_reference(length))
            screw.label = f"M6x{length:g}_side_button_head_{len(out) + 1}"
            out.append(screw)
    return out


def bottom_button_head_bolts_reference(length=None):
    """바닥 카운터보어에서 3030 하부 슬롯으로 체결하는 M6 볼트 2개."""
    length = P.LOW_BOLT_LEN if length is None else length
    out = []
    head_seat_z = LOW_BOT + P.LOW_CBORE[1]
    for sx in (-1, 1):
        screw = (Pos(sx * P.LOW_POST_X, CLAMP_CY, head_seat_z)
                 * button_head_screw_reference(length))
        screw.label = f"M6x{length:g}_bottom_button_head_{len(out) + 1}"
        out.append(screw)
    return out


def shelf_button_head_bolts_reference(length=None):
    """선반 카운터보어에서 H 크로스 하부 슬롯으로 체결하는 M6 볼트 2개. 선반이 없으면 빈 목록."""
    if not P.LOW_SHELF:
        return []
    length = P.LOW_BOLT_LEN if length is None else length
    out = []
    for by in SHELF_BOLT_YS:
        screw = (Pos(0, by, SHELF_BOT + SHELF_CBORE[1])
                 * button_head_screw_reference(length))
        screw.label = f"M6x{length:g}_shelf_button_head_{len(out) + 1}"
        out.append(screw)
    return out


def h_cross(rail_y=210.0):
    """액슬 라인 위 H 크로스 (3030, 좌우 레일 안쪽면 사이). 선반이 없으면 빈 목록.

    차량 좌표(레일 y=±rail_y). 레일 중심 높이에 놓이며 축 끝(y=±185.1)과 기존
    클램프 윗면 위를 지난다 — 여유는 verify() 가 잰다.
    """
    if not P.LOW_SHELF:
        return []
    from extrusion_reference import extrusion_reference
    L = 2 * rail_y - P.EXTRUSION_W                   # 레일 안쪽면 사이
    prof = Rot(0, 0, 90) * extrusion_reference(L)    # X → Y 로 눕힌다
    prof = Pos(0, 0, RAIL_CZ) * prof
    prof.label = f"3030_h_cross_{L:.0f}"
    return [prof]


def oem_button_head_bolts_reference(length=20.0):
    """기존 상부 클램프를 하부 클램프 M6 탭에 조이는 볼트 4개."""
    out = []
    head_seat_z = CLAMP_SEAT_Z + P.OEM_T
    for sx in (-1, 1):
        for sy in (-1, 1):
            screw = (Pos(sx * P.OEM_BOLT[0], CLAMP_CY + sy * P.OEM_BOLT[1],
                         head_seat_z)
                     * Rot(180, 0, 0)
                     * button_head_screw_reference(length))
            screw.label = f"M6x{length:g}_oem_button_head_{len(out) + 1}"
            out.append(screw)
    return out


def load_paths(w=None):
    """800N 에서 각 경로의 사용률. (이름, 값, 한계, 단위, 설명)"""
    w = w or P.DESIGN_W
    m = w * abs(CLAMP_CY)                                  # 기울임 모멘트 N·mm
    seat_h = LOW_H_SEAT = P.LOW_H - (P.AXLE_DIAMETER / 2 + P.SADDLE_SEAT_FIT) - P.SADDLE_PINCH
    span = 2 * P.LOW_POST_X - P.LOW_POST_W                 # 다리 사이 순 스팬
    # 다리 윗면 ↔ 레일 밑면 지압. 슬롯 입구(8.2)를 뺀 양쪽 랜드만 닿는다.
    land = (P.EXTRUSION_W - P.EXTRUSION_SLOT) / 2
    bear_a = 2 * P.LOW_POST_W * 2 * land
    out = []
    # **누르는 방향이 지배 하중이고, 그건 볼트가 아니라 이 면이 받는다.**
    # 실물 시험에서 확인됐다 — 눌러도 멀쩡했고 부러진 건 얇은 옆 플랜지였다.
    out.append(("다리↔레일 지압 (누름)", w / bear_a, 60.0,
                "MPa", f"랜드 {land:.1f}mm x 2 x 다리 2 = {bear_a:.0f}mm2"))
    # 기울임을 어느 경로가 받느냐. 각 경로가 "혼자 다 받는다"고 보는 보수값이다.
    if P.TIE:
        share, H, bolt = tie_force(w)
        out.append((f"타이 M{P.TIE_ROD:.0f} 인장(x{P.TIE_N})", H, 0.6 * 640 * 36.6, "N",
                    f"좌우 클램프가 벌어지는 걸 잡는다 (모멘트의 {share*100:.0f}%)"))
        out.append(("레일 밑면 볼트 인발", bolt, P.TNUT_PULLOUT, "N",
                    f"타이가 {share*100:.0f}% 가져간 나머지"))
    else:
        out.append(("레일 밑면 볼트 인발", m / (P.EXTRUSION_W / 2) / 2, P.TNUT_PULLOUT,
                    "N", "피벗=레일 밑모서리, 지레 15mm. 이것만으로 받는 경우"))
    if not P.LOW_SIDE:
        # 프레임을 위로 들 때 — 이때만 볼트가 바퀴 무게를 인장으로 받는다
        out.append(("들어올릴 때 볼트 인장", P.WHEEL_MASS_N / 2, P.TNUT_PULLOUT,
                    "N", "밑면 볼트 2개가 바퀴+클램프 무게를 받는다"))
    if P.LOW_SIDE:
        # 옆 플랜지 2장의 수직 전단이 커플을 만든다 (지레 = 레일 폭)
        out.append(("레일 옆면 볼트 전단", m / P.EXTRUSION_W / 2, 8000.0,
                    "N", "플랜지 2장 커플, 지레 30mm. 이것만으로 받는 경우"))
    out.append(("다리 굽힘", m / (2 * P.LOW_POST_W * P.LOW_D ** 2 / 6), 240.0,
                "MPa", "다리 2개, X축 굽힘"))
    out.append(("블록 굽힘 (다리 스팬)", (w * span / 4) / (P.LOW_D * seat_h ** 2 / 6),
                240.0, "MPa", f"시트 아래 살 {seat_h:.1f}mm"))
    out.append(("축 시트 면압", w / (P.AXLE_DIAMETER * P.LOW_D), 120.0,
                "MPa", "반원 받침 투영면적"))
    out.append(("상부 클램프 볼트 전단", P.DRIVE_TORQUE * 1000 / (4 * 19.2), 8000.0,
                "N", "구동 반력 29 N·m / 볼트 4개 반경 19.2"))
    return out


def report(w=None):
    w = w or P.DESIGN_W
    if P.LOW_SHELF:
        if TONGUE and P.SHELF_CABLE:
            print(f"[케이블] Ø{P.SHELF_CABLE_D:.0f} 통로: 축 끝 → 직선 {P.SHELF_CABLE_STRAIGHT:.0f} → R{P.SHELF_CABLE_R:.0f} → "
                  f"{'+' if P.SHELF_CABLE_SIDE >= 0 else '-'}x 옆면 출구 y≈{CABLE_Y_EXIT:.1f}")
        print(f"[선반:{P.SHELF_KIND}] 안쪽면 y={SHELF_FACE:.1f} 에서 {P.SHELF_REACH:.0f}mm, {SHELF_W:.1f}x{SHELF_TOP - SHELF_BOT:.1f}t, "
              f"볼트 y={SHELF_BOLT_YS[0]:.1f}/{SHELF_BOLT_YS[1]:.1f} (레일 바깥모서리 지레 "
              f"{CLAMP_CY + P.EXTRUSION_W/2 - SHELF_BOLT_YS[0]:.0f}/{CLAMP_CY + P.EXTRUSION_W/2 - SHELF_BOLT_YS[1]:.0f}mm, "
              f"레일볼트 {P.EXTRUSION_W:.0f}/2={P.EXTRUSION_W/2:.0f}mm)")
    print(f"[Design D] {P.EXTRUSION} / 옆 플랜지 {'있음' if P.LOW_SIDE else '없음'}"
          f" / 하중 {w:.0f} N")
    for name, v, lim, unit, why in load_paths(w):
        pct = v / lim * 100
        mark = "  " if pct < 50 else ("주의" if pct < 80 else "위험")
        print(f"  {mark} {name:22s} {v:7.1f} {unit:4s} / {lim:6.0f} = {pct:5.1f}%   {why}")
    # 이 표는 **경로별 사용률을 빠르게 보는 용도**다. 단면을 통짜 보로 보기 때문에
    # 구멍 주변 국부 응력을 못 본다 — FEA 는 같은 조건에서 45.3 MPa 를 낸다(약 10배).
    # 국부 응력은 tools/fea/structural_stress.py 값을 쓴다.
    print("  국부 응력은 FEA: 800N 편심55 상향에서 구조응력 45.3 MPa "
          "(AL6061 안전율 6.1) — tools/fea/structural_stress.py")
    print(f"  볼트 길이: 밑면 M{P.TNUT_BOLT:.0f}x{rail_bolt_len():.0f} 이상"
          f" / 옆면 M{P.TNUT_BOLT:.0f}x{side_bolt_len():.0f} 이상")


def verify():
    """Design D 검증 — 간섭 / 연결 / 최소 두께. (문제 목록) 을 돌려준다."""
    from axle import axle_solid
    bad = []
    part = low_clamp()
    if len(part.solids()) != 1:
        bad.append(f"low_clamp 솔리드 {len(part.solids())}개 — 조각났다")
    named = [("low_clamp", part), ("oem_clamp", oem_clamp_block()),
             ("axle", axle_solid()), ("rail", rail())]
    for i in range(len(named)):
        for j in range(i + 1, len(named)):
            (na, a), (nb, b) = named[i], named[j]
            x = a & b
            v = x.volume if x is not None else 0.0
            if v > 1.0:
                bad.append(f"간섭 {na} x {nb}: {v:.0f} mm3")
    if P.LOW_SHELF:
        ax = axle_solid().bounding_box()
        # 노치 바닥(y) 이 축 끝보다 안쪽으로 SHELF_CLR 이상
        notch_floor = SHELF_FACE - P.SHELF_NOTCH[1]
        if not (ax.min.Y - notch_floor >= P.SHELF_CLR - 1e-6):
            bad.append(f"선반 노치/포켓이 축 끝을 못 피한다: 축 끝 y={ax.min.Y:.1f}, 바닥 {notch_floor:.1f}")
        if not TONGUE and P.SHELF_NOTCH[0] < P.AXLE_DIAMETER + 2 * P.SHELF_CLR:
            bad.append("선반 노치 폭이 축 지름 + 여유보다 좁다")
        x = part & axle_solid()
        if x is not None and x.volume > 1.0:
            bad.append(f"간섭 low_clamp x axle(선반): {x.volume:.0f} mm3")
        oem_in = CLAMP_CY - P.OEM_H / 2                                  # 기존 클램프 안쪽 끝
        if not (oem_in - SHELF_FACE >= P.SHELF_CLR - 1e-6):
            bad.append(f"선반 뿌리가 기존 클램프와 겹친다: 클램프 안쪽 {oem_in:.1f} vs 선반면 {SHELF_FACE:.1f}")
        # 볼트가 크로스 밑면 슬롯(x=0) 안에, 노치 밖에
        for by in SHELF_BOLT_YS:
            if by > notch_floor - SHELF_CBORE[0] / 2 - 2:
                bad.append(f"선반 볼트 y={by:.1f} 카운터보어가 노치를 파고든다")
        if TONGUE and P.SHELF_CABLE:
            # 통로 솔리드와 볼트 구멍·카운터보어 솔리드가 실제로 겹치는지 (1.5mm 살 포함)
            ch = cable_channel()
            for by in SHELF_BOLT_YS:
                with BuildPart() as bolt:
                    with Locations((0, by, (SHELF_TOP + SHELF_BOT) / 2)):
                        Cylinder(P.TNUT_BOLT * 0.55 + 1.5, SHELF_TOP - SHELF_BOT + 2)
                    with Locations((0, by, SHELF_BOT + SHELF_CBORE[1] / 2)):
                        Cylinder(SHELF_CBORE[0] / 2 + 1.5, SHELF_CBORE[1])
                x = ch & bolt.part
                if x is not None and x.volume > 0.5:
                    bad.append(f"케이블 통로와 볼트 y={by:.1f} 사이 살이 1.5mm 미만 ({x.volume:.0f} mm3 겹침)")
            # 통로 위·아래 살
            if SHELF_TOP - (AZ + P.SHELF_CABLE_D / 2) < 4.0:
                bad.append("케이블 통로 위 살이 4mm 미만")
            if (AZ - P.SHELF_CABLE_D / 2) - SHELF_BOT < 4.0:
                bad.append("케이블 통로 아래 살이 4mm 미만")
            if SHELF_BOLT_YS[-1] - SHELF_CBORE[0] / 2 < SHELF_FACE - P.SHELF_REACH + 2.0:
                bad.append("마지막 볼트 카운터보어가 혀 끝을 뚫는다")
        # H 크로스 vs 축·기존 클램프 (차량 좌표에서 클램프 로컬로 환산해 z 만 본다)
        cross_bot = RAIL_CZ - P.EXTRUSION_H / 2
        if cross_bot - (AZ + P.AXLE_DIAMETER / 2) < P.SHELF_CLR:
            bad.append("H 크로스 밑면이 축과 너무 가깝다")
        if cross_bot - (CLAMP_SEAT_Z + P.OEM_T) < 1.0:
            bad.append("H 크로스 밑면이 기존 클램프에 닿는다")
        for c in h_cross(210.0):
            c2 = Pos(0, CLAMP_CY - 210.0, 0) * c
            for nm, o in (("low_clamp", part), ("oem_clamp", oem_clamp_block()), ("axle", axle_solid())):
                x = c2 & o
                v = x.volume if x is not None else 0.0
                if v > 1.0:
                    bad.append(f"간섭 h_cross x {nm}: {v:.0f} mm3")
    bb = part.bounding_box()
    if min(bb.size.X, bb.size.Y, bb.size.Z) < 3.0:
        bad.append("최소 두께 3mm 미만")
    # 3030 + 플랜지 없음은 결함이 아니라 **선택**이다 (실물 시험 결과). 숫자는
    # report() 가 매번 찍으므로 여기서 실패로 세지 않는다.
    return bad


if __name__ == "__main__":
    import sys
    from pathlib import Path
    sys.path[:0] = [str(Path(__file__).parent), str(Path(__file__).parent.parent)]
    report()
