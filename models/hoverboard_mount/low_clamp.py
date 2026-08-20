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

assert P.LOW_POST_X - P.LOW_POST_W / 2 > P.OEM_W / 2, "다리가 기존 클램프와 부딪힌다"
assert RAIL_BOT > CLAMP_SEAT_Z + P.OEM_T, "레일이 기존 클램프를 파고든다"
assert P.LOW_CBORE[0] + 2 <= P.LOW_POST_W, "카운터보어가 다리보다 커서 벽이 안 남는다"
assert P.LOW_H > P.AXLE_DIAMETER / 2 + 6, "축 시트 아래 살이 6mm 도 안 남는다"
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


def low_clamp():
    """각기둥 하나에서 필요한 것만 파낸다.

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
        for sx in (-1, 1):                                               # 상부 클램프 탭 4
            for sy in (-1, 1):
                with Locations((sx * P.OEM_BOLT[0], CLAMP_CY + sy * P.OEM_BOLT[1],
                                (LOW_BOT + LOW_TOP) / 2)):
                    Cylinder((P.OEM_BOLT_D - 1.0) / 2, P.LOW_H + 2, mode=Mode.SUBTRACT)
        if P.TIE:                                                        # 액슬 타이 전산볼트 탭
            for sx in ((-1, 1) if P.TIE_N == 2 else (0,)):
                with Locations((sx * P.TIE_X, CLAMP_CY - P.LOW_D / 2 + P.TIE_ENGAGE / 2,
                                P.TIE_Z)):
                    Cylinder((P.TIE_ROD - 1.2) / 2, P.TIE_ENGAGE,
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


def tie_rods(rail_y=210.0):
    """좌우 클램프를 잇는 인장 전산볼트 (차량 좌표). 클램프 탭에 직접 박힌다."""
    if not P.TIE:
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
