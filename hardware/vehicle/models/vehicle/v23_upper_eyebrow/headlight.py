r"""전조등 — MR16 12 V 5 W LED 램프 ×2 + ASA 앞 테두리 (하우징) + 안쪽 고정 링 (v22).

램프: 건식 LED MR16 12 V 5 W 집중형 (주광 6500 K) — 0.41 A, 410 lm, 빔각 50°, 외곽 Ø50 × 49 (핀 포함), GU5.3 [판매처]
      https://www.11st.co.kr/products/3733952699 (1,720) · 소켓 GU5.3 사기 소켓 17 × 10, 리드 300 [판매처]
      https://www.11st.co.kr/products/7292435290 (1,000)
구조 [설계]: 앞판 (PC 3t) 에 Ø51 구멍. 바깥 ASA 테두리가 램프 앞 테두리를 립 (Ø46) 으로 잡고, 안쪽 ASA 고정 링의
      손가락 3 개가 램프 뒤 원뿔을 앞으로 민다. 테두리 · 고정 링은 M3 ×4 로 판을 사이에 두고 조인다
      (볼트는 안에서, 테두리에 열압입 인서트 — 바깥에 볼트 머리가 안 보인다).

로컬 좌표: X = 빛 방향 (+X = 앞), **X=0 = 앞판 바깥면**, 판은 X ∈ [-PANEL_T, 0]. Y 폭, Z 높이.
"""

from build123d import Box, BuildPart, Cylinder, Cone, Locations, Mode, Pos, Rot, Axis, fillet

PANEL_T = 3.0
LAMP_D, LAMP_L = 50.0, 49.0       # [판매처] 핀 포함
LAMP_RIM = 5.0                    # 앞 원통 테두리 길이 [추정]
LAMP_BACK_D = 30.0                # 뒤 원뿔 끝 지름 [추정]
PIN_L, PIN_PITCH = 7.0, 5.3       # GU5.3 [규격]
SOCKET = (14.0, 17.0, 10.0)       # 소켓 깊이 · 폭 · 두께 [판매처 17 × 10, 깊이 추정]

BEZEL = dict(w=80.0, h=64.0, depth=18.0, r=10.0, lip_d=46.0, recess=8.0)   # [설계] 램프 앞면이 테두리 앞에서 8 들어간다
HOLE_D = 51.0                     # 판 구멍 [설계]
SCREWS = [(sy * 32.0, sz * 24.0) for sy in (-1, 1) for sz in (-1, 1)]               # 판 M3 구멍 (y, z) [설계]
RING = dict(t=3.0, tube_l=16.0, tube_od=58.0, fingers=3)                            # 안쪽 고정 링 [설계]


def lamp_x0():
    """램프 앞면 x (테두리 앞면에서 recess 만큼 안쪽)."""
    return BEZEL["depth"] - BEZEL["recess"]


def lamp():
    """MR16 램프 + GU5.3 소켓 · 리드 토막. 앞면 = lamp_x0()."""
    ax = dict(rotation=(0, 90, 0))
    x0 = lamp_x0()
    body = Pos(x0 - LAMP_RIM / 2, 0, 0) * Cylinder(LAMP_D / 2, LAMP_RIM, **ax)          # 앞 테두리 + 렌즈
    cone_l = LAMP_L - PIN_L - LAMP_RIM
    body = body + Pos(x0 - LAMP_RIM - cone_l / 2, 0, 0) * Cone(LAMP_BACK_D / 2, LAMP_D / 2 - 1.0, cone_l, **ax)
    body = body - Pos(x0 - 0.5, 0, 0) * Cylinder(LAMP_D / 2 - 2.5, 1.02, **ax)           # 렌즈 오목
    xp = x0 - LAMP_L + PIN_L
    for dy in (-PIN_PITCH / 2, PIN_PITCH / 2):                                         # 핀 2
        body = body + Pos(xp - PIN_L / 2, dy, 0) * Cylinder(0.8, PIN_L + 0.02, **ax)
    sd, sw, st = SOCKET
    body = body + Pos(xp - PIN_L + 2.0 - sd / 2, 0, 0) * Box(sd, sw, st)              # 소켓 (핀 끝 2 mm 겹침)
    for dz in (-2.5, 2.5):                                                             # 리드 토막 2 (30)
        body = body + Pos(xp - PIN_L + 2.0 - sd - 15.0, 0, dz) * Cylinder(0.9, 30.0, **ax)
    return body


def bezel():
    """바깥 ASA 테두리 — 둥근 사각 앞면, 위쪽 차양 (아래로 2° 경사), 램프 구멍 + 앞 립. 뒷면 = 판 바깥면 (X=0)."""
    b = BEZEL
    with BuildPart() as p:
        with Locations((b["depth"] / 2, 0, 0)):
            Box(b["depth"], b["w"], b["h"])
        fillet(p.edges().filter_by(Axis.X), radius=b["r"])
        try:
            fillet(p.edges().group_by(Axis.X)[-1], radius=3.0)                           # 앞 모서리 둥글림
        except Exception:                                                               # noqa: BLE001
            pass
        x0 = lamp_x0()
        with Locations((x0 / 2, 0, 0)):                                                 # 램프 구멍 (판 ~ 램프 앞면)
            Cylinder(LAMP_D / 2 + 0.25, x0 + 0.02, rotation=(0, 90, 0), mode=Mode.SUBTRACT)
        with Locations((b["depth"] / 2, 0, 0)):                                         # 립 안쪽 (빛 나가는 곳) — 램프 앞 테두리가 닿는다
            Cylinder(b["lip_d"] / 2, b["depth"] + 0.02, rotation=(0, 90, 0), mode=Mode.SUBTRACT)
        fl = b["depth"] - x0 - 2.0                                                       # 립 앞쪽 나팔 (Ø46 → Ø58)
        with Locations((b["depth"] - fl / 2 + 0.01, 0, 0)):
            Cone(b["lip_d"] / 2, b["lip_d"] / 2 + 6.0, fl + 0.02, rotation=(0, 90, 0), mode=Mode.SUBTRACT)
        for y, z in SCREWS:                                                             # M3 열압입 인서트 구멍 Ø4.0 × 6
            with Locations((3.0, y, z)):
                Cylinder(2.0, 6.0, rotation=(0, 90, 0), mode=Mode.SUBTRACT)
    return p.part


def retainer():
    """안쪽 ASA 고정 링 — 판 안쪽면 플랜지 (M3 통과 ×4) + 뒤로 나온 관, 손가락 3 개가 램프 원뿔을 민다."""
    b, r = BEZEL, RING
    x_in = -PANEL_T
    with BuildPart() as p:
        with Locations((x_in - r["t"] / 2, 0, 0)):
            Box(r["t"], b["w"], b["h"])
        fillet(p.edges().filter_by(Axis.X), radius=b["r"])
        with Locations((x_in - r["t"] / 2, 0, 0)):
            Cylinder(HOLE_D / 2, r["t"] + 0.02, rotation=(0, 90, 0), mode=Mode.SUBTRACT)
        for y, z in SCREWS:
            with Locations((x_in - r["t"] / 2, y, z)):
                Cylinder(1.7, r["t"] + 0.02, rotation=(0, 90, 0), mode=Mode.SUBTRACT)
    ring = p.part
    # 관 — 안지름은 램프 원뿔이 그 깊이에서 갖는 지름에 맞춘다 (손가락 끝이 원뿔에 닿는다)
    x0 = lamp_x0()
    cone_l = LAMP_L - PIN_L - LAMP_RIM
    xt = x_in - r["t"] - r["tube_l"]                                                  # 관 끝 x
    xf = xt + 3.0                                                                      # 손가락 앞면 (원뿔이 가장 굵은 쪽)
    frac = (x0 - LAMP_RIM - xf) / cone_l                                               # 원뿔 앞 → 끝 비율
    d_at = (LAMP_D - 1.0) - frac * (LAMP_D - 1.0 - LAMP_BACK_D) + 0.2
    tube = Pos((x_in - r["t"] + xt) / 2, 0, 0) * Cylinder(r["tube_od"] / 2, r["tube_l"], rotation=(0, 90, 0))
    tube = tube - Pos((x_in - r["t"] + xt) / 2, 0, 0) * Cylinder(LAMP_D / 2 + 0.5, r["tube_l"] + 0.02, rotation=(0, 90, 0))
    for i in range(r["fingers"]):                                                      # 손가락 — 관 끝에서 안으로
        a = 360.0 * i / r["fingers"] + 90.0
        f_len = r["tube_od"] / 2 - d_at / 2
        tube = tube + Rot(a, 0, 0) * Pos(xt + 1.5, 0, d_at / 2 + f_len / 2) * Box(3.0, 8.0, f_len)
    return ring + tube
