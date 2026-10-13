r"""v29 배선 — 바닥판에 눕혀 직각 (ㄱ · ㄴ) 으로만 꺾어 정리한다 (DECISIONS D-21).

v28 (D-20) 과 같은 연결 (90 가닥) 을, 공중 · 배터리 위 · 대각선 대신 **바닥판 위**로 깐다.
  · 단자에서 짧게 빠져나와 (escape) 수직으로 바닥까지 내려오고, 바닥에서는 x · y 방향으로만 간다.
  · 경로는 router.py 가 격자 (2 mm) 위 A* 로 찾는다 — 꺾임 벌점 · 부품 · 다른 선 옆에 붙기 (다발) · 선 위로 넘어가기 (직각만).
  · 찾은 경로는 routes_data.py 에 굳혀 둔다 (짓기는 그것만 읽는다 — 결정적 · 빠름).
새 부품 자리도 바닥 배선에 맞춰 옮겼다 — XT90 은 바닥에, 분기 레일은 90° 돌려 (출력이 옆 보드 쪽), 음극 N 바는 컨택터 남쪽.
색 (그룹): 빨강 = 36 V + · 검정 = 0 V · 주황 = 12 V · 노랑 = 5 V · 파랑 = 신호 · 회색 = USB · LAN · 동축 · 초록 = 모터.
[가정] 은 실물을 보고 고칠 값 (단자 자리 · 부품 치수).
"""

import math

import numpy as np
from build123d import Align, Box, Compound, Cylinder, Location, Plane, Pos, Sphere

Z0 = 128.85                 # 바닥판 윗면
MIN = (Align.CENTER, Align.CENTER, Align.MIN)

OD = dict(AWG10=5.2, AWG14=3.4, AWG18=2.3, AWG22=1.6, LEAD=6.0, USB3=4.6, USB2=3.6, LAN=5.6, DC2=4.4,
          CAB5=4.2, CAB3=3.4, FAN2=3.0, ST5=6.0, HALL=4.5, PHASE=3.6, MOTOR=10.0, COAX=1.4, MS4=3.8)

GROUPS = {   # key: (그룹 이름, 색)
    "dist_rail": ("배전레일_차단기20A4_퓨즈3", "#eef0f2"),
    "dist_nbar": ("음극단자대_N바", "#c08a3e"),
    "connector": ("커넥터_XT90_XT60_WAGO_코일플러그", "#e0a800"),
    "usb_hub": ("USB허브_PL-UH307P", "#2f363c"),
    "aux_board": ("보조보드_MOSFET4_라이다USB", "#1b6b3a"),
    "wire_pos": ("전선_36V+_빨강", "#d32f2f"),
    "wire_neg": ("전선_0V_검정", "#1a1a1a"),
    "wire_12v": ("전선_12V_주황", "#f57c00"),
    "wire_5v": ("전선_5V_노랑", "#f9c80e"),
    "wire_sig": ("케이블_신호_파랑", "#1e88e5"),
    "wire_usb": ("케이블_USB_LAN_동축_회색", "#7b8794"),
    "wire_motor": ("케이블_모터_상선_홀_초록", "#2e7d32"),
}
NEW_KEYS = ("dist_rail", "dist_nbar", "connector", "usb_hub", "aux_board")


# ---------------------------------------------------------------- 기하 도우미
def _seg(a, b, r):
    a, b = np.asarray(a, float), np.asarray(b, float)
    v = b - a
    L = float(np.linalg.norm(v))
    return Location(Plane(origin=tuple((a + b) / 2), z_dir=tuple(v / L))) * Cylinder(r, L)


def tube(pts, d):
    """꺾은선 관 — 구간 원기둥 + 안쪽 꺾임마다 구."""
    r = d / 2
    pts = [np.asarray(p, float) for p in pts]
    solids = [_seg(a, b, r) for a, b in zip(pts, pts[1:]) if np.linalg.norm(b - a) > 1e-6]
    solids += [Pos(*p) * Sphere(r) for p in pts[1:-1]]
    return Compound(children=solids)


def plen(pts):
    return float(sum(np.linalg.norm(np.subtract(b, a)) for a, b in zip(pts, pts[1:])))


def _box(c, size):
    return Pos(*c) * Box(*size)


def zf(d):
    """바닥에 눕힌 지름 d 전선 중심 z."""
    return Z0 + d / 2 + 0.3


# ---------------------------------------------------------------- 새 부품 자리 (차량 좌표, mm)
XT90 = dict(x=(133.0, 173.0), w=20.8, h=11.0)                     # v29: 바닥에 눕힘. 배터리 쪽 = -x 끝 (133)
XT90_Z = Z0 + 5.5
XT90_PIN = 5.45
NBAR = dict(c=(152.0, -104.0), len=84.0, base=(20.0, 8.0), bar=(10.0, 10.0), pitch=7.0, n=12)  # v29: 컨택터 남쪽, x 방향
NB_Z = Z0 + 13.0
RAIL = dict(x=(209.0, 345.0), w=35.0, h=7.5)                      # v29: DIN 레일 x 방향 (y 0), 단위는 y 방향으로 섬
UNIT_Y = (-40.0, 40.0)                                            # 입력 -y 끝 · 출력 +y 끝
TERM_Z = 152.0
UNITS = [   # (이름, 종류, x 가운데, 정격, 입력 마디) — 오른쪽으로 가는 출력 (F2 · F4) 은 앞 끝, 뒤 (F6 · F7) 는 뒤 끝
    ("F7_충전", "fuse", 223.0, "15 A", "배터리"),
    ("F6_5V", "fuse", 241.0, "15 A", "상시"),
    ("F5_12V", "fuse", 259.0, "15 A", "상시"),
    ("F3_뒤왼", "mcb", 277.0, "20 A", "스위치"),
    ("F1_앞왼", "mcb", 295.0, "20 A", "스위치"),
    ("F4_뒤오른", "mcb", 313.0, "20 A", "스위치"),
    ("F2_앞오른", "mcb", 331.0, "20 A", "스위치"),
]
UX = {u[0][:2]: u[2] for u in UNITS}
JUMPERS = [("F3", "F1"), ("F1", "F4"), ("F4", "F2"), ("F5", "F6")]
JUMP_Z = 146.0
WAGO = dict(l=20.0, w=19.0, h=8.5)
W12P = (-40.0, 80.0)                                              # 12 V + (입구 +y 면) — v29: 나란히 놓고 입구를 북쪽으로 (RSD 단자 쪽)
W12N = (-16.0, 80.0)                                              # 12 V −
W5P = (-35.5, -162.0)                                             # 5 V + (입구 -y 면)
W5N = (-10.5, -162.0)                                             # 5 V −
HUB = dict(c=(-225.0, -5.0), size=(130.0, 46.0, 22.0))            # POWERLAN PL-UH307P [판매처] — v29: 뒤오른 XT60 자리를 비켜 y -5
HUB_PORTS = [-275.0 + 17.0 * k for k in range(7)]
SW4 = [(22.0, -158.0), (58.0, -158.0), (22.0, -179.0), (58.0, -179.0)]  # 표시등 MOSFET 1 채널 ×4 (34 × 17) [가정] — v29: ESP32 앞을 비우려고 남쪽으로
SW4_SIZE = (34.0, 17.0, 10.0)
COIL_PLUG = dict(x=(152.0, 164.0), y=(-51.15, -39.0), z=(146.0, 157.0))
ZB = Z0 + 15.1                                                    # 보드 전원선 · 상선 토막 높이
# XT60 (보드 전원) — 보드 리드 (토막) 끝에서 90° 꺾어 y 방향으로 매단다
XT60 = {   # 네 개 모두 토막 끝에서 90° 꺾어 차 가운데 쪽 (y) 으로 — pins = (+ x, − x) 와 하네스 끝 y (가운데 기준)
    "앞왼": dict(axis="y", c=(206.0, 64.0, ZB), pins=((209.6, -16.0), (202.4, -16.0)), end=(0, -1)),
    "뒤오른": dict(axis="y", c=(-206.0, -64.0, ZB), pins=((-209.6, 16.0), (-202.4, 16.0)), end=(0, 1)),
    "앞오른": dict(axis="y", c=(372.1, -64.0, ZB), pins=((375.7, 16.0), (368.5, 16.0)), end=(0, 1)),
    "뒤왼": dict(axis="y", c=(-372.1, 62.0, ZB), pins=((-375.7, -16.0), (-368.5, -16.0)), end=(0, -1)),
}


def xt60_pins(nm):
    """XT60 하네스 쪽 끝 (+, −) 점."""
    d = XT60[nm]
    cx, cy, cz = d["c"]
    out = []
    for a, b in d["pins"]:
        out.append((cx + a, b, cz) if d["axis"] == "x" else (a, cy + b, cz))
    return out


def wago_entries(c, face, n=5):
    cx, cy = c
    z = Z0 + WAGO["h"] / 2
    if face == "+x":
        return [(cx + WAGO["l"] / 2, cy + 3.6 * (k - 2), z) for k in range(n)]
    if face == "+y":
        return [(cx + 3.6 * (k - 2), cy + WAGO["l"] / 2, z) for k in range(n)]
    return [(cx + 3.6 * (k - 2), cy - WAGO["l"] / 2, z) for k in range(n)]


def nbar_hole(k):
    """N 바 구멍 k (0..11) 입구 — 남쪽 (-y) 면."""
    cx, cy = NBAR["c"]
    x = cx - NBAR["pitch"] * (NBAR["n"] - 1) / 2 + NBAR["pitch"] * k
    return (x, cy - NBAR["bar"][0] / 2, NB_Z)


def unit_in(name, dz=0.0):
    return (UX[name], UNIT_Y[0], TERM_Z + dz)


def unit_out(name):
    return (UX[name], UNIT_Y[1], TERM_Z)


def parts():
    """새 배전 · 연결 부품 — {그룹: [Part]}."""
    out = {k: [] for k in NEW_KEYS}

    def add(g, p, label):
        p.label = label
        out[g].append(p)

    (rx0, rx1) = RAIL["x"]
    add("dist_rail", _box(((rx0 + rx1) / 2, 0.0, Z0 + RAIL["h"] / 2), (rx1 - rx0, RAIL["w"], RAIL["h"])), "DIN레일_35_L136")
    zb = Z0 + RAIL["h"]
    for name, kind, x, rating, _ in UNITS:
        if kind == "mcb":
            body = _box((x, 0.0, zb + 71.0 / 2), (17.6, 80.0, 71.0)) + _box((x, 0.0, zb + 71.0 + 4.0), (9.0, 14.0, 8.0))
            add("dist_rail", body, f"분기차단기_{name}_DC{rating.replace(' ', '')}")
        else:
            body = _box((x, 0.0, zb + 58.0 / 2), (17.6, 81.0, 58.0)) + _box((x, 0.0, zb + 58.0 + 3.0), (12.0, 40.0, 6.0))
            add("dist_rail", body, f"퓨즈홀더_{name}_10x38_{rating.replace(' ', '')}")
    cx, cy = NBAR["c"]
    nb = _box((cx, cy, Z0 + NBAR["base"][1] / 2), (NBAR["len"], NBAR["base"][0], NBAR["base"][1]))
    nb = nb + _box((cx, cy, Z0 + 8.0 + 5.0), (NBAR["len"] - 4.0, NBAR["bar"][0], NBAR["bar"][1]))
    for k in range(NBAR["n"]):
        nb = nb + Pos(nbar_hole(k)[0], cy, Z0 + 18.0) * Cylinder(2.6, 2.5, align=MIN)
    add("dist_nbar", nb, "음극단자대_N바_12구멍")
    (x0, x1) = XT90["x"]
    add("connector", _box(((x0 + x1) / 2, 0.0, XT90_Z), (x1 - x0, XT90["w"], XT90["h"])), "커넥터_XT90_배터리")
    for nm, d in XT60.items():
        size = (32.0, 15.5, 8.2) if d["axis"] == "x" else (15.5, 32.0, 8.2)
        add("connector", _box(d["c"], size), f"커넥터_XT60_{nm}")
    cp = COIL_PLUG
    add("connector", _box(tuple((a + b) / 2 for a, b in (cp["x"], cp["y"], cp["z"])),
                          tuple(b - a for a, b in (cp["x"], cp["y"], cp["z"]))), "컨택터코일플러그_GPR-40ST")
    for (c, face), nm in zip(((W12P, "+y"), (W12N, "+y"), (W5P, "-y"), (W5N, "-y")), ("12V+", "12V-", "5V+", "5V-")):
        size = (WAGO["l"], WAGO["w"], WAGO["h"]) if face == "+x" else (WAGO["w"], WAGO["l"], WAGO["h"])
        add("connector", _box((c[0], c[1], Z0 + WAGO["h"] / 2), size), f"WAGO_221-415_{nm}")
    (hx, hy), (hl, hw, hh) = HUB["c"], HUB["size"]
    add("usb_hub", _box((hx, hy, Z0 + hh / 2), (hl, hw, hh)), "USB허브_POWERLAN_PL-UH307P_7포트")
    for i, (x, y) in enumerate(SW4):
        sx, sy, sz = SW4_SIZE
        p = _box((x, y, Z0 + 5.0 + 1.6 / 2), (sx, sy, 1.6)) + _box((x - 4.0, y, Z0 + 6.6 + 4.0), (10.0, 8.0, 8.0))
        p = p + _box((x + sx / 2 - 4.0, y, Z0 + 6.6 + 4.0), (7.0, 12.0, 8.0))
        for dx in (-sx / 2 + 3, sx / 2 - 3):
            p = p + Pos(x + dx, y, Z0) * Cylinder(2.0, 5.0, align=MIN)
        add("aux_board", p, f"표시등MOSFET_{['초록', '노랑', '빨강', '부저'][i]}_15A")
    zp = 320.85
    ad = _box((20.0, 40.0, zp + 3.0 + 0.8), (30.0, 18.0, 1.6)) + _box((7.0, 40.0, zp + 4.6 + 1.5), (8.0, 8.0, 3.0))
    for dx in (-12.0, 12.0):
        ad = ad + Pos(20.0 + dx, 40.0, zp) * Cylinder(1.5, 3.0, align=MIN)
    add("aux_board", ad, "라이다USB어댑터_CP2102")
    return out


# ---------------------------------------------------------------- 연결 목록
def E(p, *esc):
    """끝점 — 단자 p 와 빠져나오는 움직임 (dx, dy, dz) 들. 그 뒤에 router 가 수직으로 바닥까지 내린다."""
    return dict(p=tuple(float(v) for v in p), esc=[tuple(float(v) for v in e) for e in esc])


def top(p, side):
    """위에서 꽂는 XH 커넥터 — 6 위로, 그다음 side 쪽으로 10."""
    return E(p, (0, 0, 6), side)


class N:
    """연결 하나. pre / post = 바닥 경로 앞뒤에 붙는 고정 꺾은선 (모듈 · 리드). fixed = 경로 전부 고정 (라우터 안 씀)."""

    def __init__(self, grp, label, kind, A=None, B=None, ends=(), note="", pre=(), post=(), fixed=None, order=5):
        self.grp, self.label, self.kind, self.A, self.B = grp, label, kind, A, B
        self.ends, self.note, self.pre, self.post, self.fixed, self.order = tuple(ends), note, list(pre), list(post), fixed, order
        self.d = OD[kind]


def nets():
    L = []
    zx = XT90_Z
    e12p, e12n = wago_entries(W12P, "+y"), wago_entries(W12N, "+y")
    e5p, e5n = wago_entries(W5P, "-y"), wago_entries(W5N, "-y")
    ox, oy = (4, 0, 0), (0, 4, 0)
    ow = oy                                                              # 12 V WAGO 입구 → 북쪽
    # ======== 36 V ========
    for s, g, nm in ((1, "wire_pos", "+"), (-1, "wire_neg", "-")):
        y = s * 15.83
        L.append(N(g, f"배터리리드{nm}_XT90", "LEAD", ends=("배터리", "XT90"), order=0,
                   fixed=[(127.5, y, 173.85), (128.5, y, 173.85), (128.5, y, zx), (128.5, s * XT90_PIN, zx),
                          (133.5, s * XT90_PIN, zx)], note="배터리에 딸린 10 AWG 리드 (ZENION XT-90 옵션)"))
    xh = XT90["x"][1]
    L.append(N("wire_pos", "주전원+_XT90-메인차단기", "AWG10", E((xh - 0.5, XT90_PIN, zx), ox), E((162, 30.5, 152), (0, -4, 0)),
               ends=("XT90", "메인차단기"), order=1, note="XT90 암 → 메인 차단기 40 A 입력 (-y 끝 단자)"))
    L.append(N("wire_neg", "주전원-_XT90-N바", "AWG10", E((xh - 0.5, -XT90_PIN, zx), ox), E(nbar_hole(6), (0, -8, 0)),
               ends=("XT90", "N바"), order=1, note="배터리 − → 음극 단자대 (모든 0 V 의 한 점)"))
    L.append(N("wire_pos", "상시+_메인차단기-컨택터", "AWG10", E((162, 109.5, 152), oy), E((150, -70, 172), (-18, 0, 0)),
               ends=("메인차단기", "컨택터_LS"), order=1, note="차단기 출력 → 컨택터 + (배터리 쪽) 단자, M4 링단자"))
    L.append(N("wire_pos", "스위치+_컨택터-F3", "AWG10", E((166, -70, 172), (18, 0, 0)), E(unit_in("F3", 4.0), (0, -8, 0)),
               ends=("컨택터_LS", "F3_"), order=1, note="컨택터 출력 → 바퀴 차단기 F3 입력 (F1 · F4 · F2 는 점퍼)"))
    for a, b in JUMPERS:
        xa, xb = UX[a], UX[b]
        L.append(N("wire_pos", f"점퍼+_{a}-{b}", "AWG14", ends=(a + "_", b + "_"), order=0,
                   fixed=[(xa, -39.5, JUMP_Z), (xa, -43.0, JUMP_Z), (xb, -43.0, JUMP_Z), (xb, -39.5, JUMP_Z)],
                   note="입력끼리 점퍼 (14 AWG + 페룰) — 1P 핀 부스바는 국내에 없다"))
    L.append(N("wire_pos", "상시+_컨택터-F5", "AWG14", E((150, -70, 176), (0, 0, 8), (-24, 0, 0)), E(unit_in("F5", 4.0), (0, -8, 0)),
               ends=("컨택터_LS", "F5_"), order=2, note="컨택터 + 단자에 링단자 겹쳐 물림 (상시 = 키와 무관)"))
    L.append(N("wire_pos", "배터리+_XT90-F7", "AWG14", E((xh - 0.5, XT90_PIN + 2.5, zx + 2.0), ox, (0, 6, 0)),
               E(unit_in("F7", 4.0), (0, -8, 0)), ends=("XT90", "F7_"), order=2,
               note="XT90 땜납 컵에 같이 납땜 — 메인 차단기 앞 (꺼져 있어도 충전)"))
    # ======== 바퀴 보드 — 레일 출력 → XT60, XT60 → N 바 ========
    holes = {"앞왼": 9, "앞오른": 4, "뒤왼": 10, "뒤오른": 3}
    for nm, f in (("앞왼", "F1"), ("앞오른", "F2"), ("뒤왼", "F3"), ("뒤오른", "F4")):
        pp, pn = xt60_pins(nm)
        ex, ey = XT60[nm]["end"]
        L.append(N("wire_pos", f"보드+_{f}-{nm}", "AWG14", E(unit_out(f), oy), E(pp, (3 * ex, 3 * ey, 0)),
                   ends=(f + "_", "XT60"), order=2))
        L.append(N("wire_neg", f"보드-_{nm}-N바", "AWG14", E(pn, (3 * ex, 3 * ey, 0)), E(nbar_hole(holes[nm]), (0, -8, 0)),
                   ends=("XT60", "N바"), order=2))
    # 보드 리드 (보드에 딸림) — 토막 끝에서 XT60 까지 (앞오른 · 뒤왼만 꺾임)
    for nm, x_end, sy in (("앞오른", 369.0, -1), ("뒤왼", -369.0, 1), ("앞왼", 206.0, 1), ("뒤오른", -206.0, -1)):
        sx = 1 if x_end > 0 else -1
        if nm in ("앞왼", "뒤오른"):
            sx = -sx                                                     # 토막이 차 가운데 쪽 (-x · +x) 을 본다
        (xp, yp, z), (xn, yn, _) = [(a, b - 32 * XT60[nm]["end"][1], c) for a, b, c in xt60_pins(nm)]
        L.append(N("wire_pos", f"보드리드+_{nm}", "AWG14", ends=(f"{nm}_분리형", "XT60"), order=0,
                   fixed=[(x_end - sx * 0.5, sy * 101.0, ZB), (xp, sy * 101.0, ZB), (xp, yp, ZB)]))
        L.append(N("wire_neg", f"보드리드-_{nm}", "AWG14", ends=(f"{nm}_분리형", "XT60"), order=0,
                   fixed=[(x_end - sx * 0.5, sy * 89.0, ZB), (xn, sy * 89.0, ZB), (xn, yn, ZB)]))
    # ======== 12 V (RSD-60L-12) · 5 V (DKP-55) ========
    rsd = lambda y: E((-4.5, y, 138.85), (-4, 0, 0))                      # noqa: E731  RSD 단자 (-x 끝) [가정]
    L.append(N("wire_pos", "상시+_F5-RSD입력", "AWG14", E(unit_out("F5"), oy), rsd(135.0), ends=("F5_", "DCDC_36V-12V"), order=2))
    L.append(N("wire_12v", "12V+_RSD-WAGO", "AWG14", rsd(105.0), E(e12p[3], (0, 2, 0), (0, 0, 6), (0, 9, 0)), ends=("DCDC_36V-12V", "12V+"), order=3))
    L.append(N("wire_neg", "12V-_RSD-WAGO", "AWG14", rsd(115.0), E(e12n[3], (0, 2, 0), (0, 0, 6), (0, 9, 0)), ends=("DCDC_36V-12V", "12V-"), order=3))
    L.append(N("wire_neg", "RSD-Vin_12V-", "AWG14", rsd(125.0), E(e12n[4], (0, 4, 0)), ends=("DCDC_36V-12V", "12V-"), order=3,
               note="RSD 입력 − 와 출력 − 를 WAGO 에서 묶는다 (기준 한 점)"))
    L.append(N("wire_neg", "기준선_12V--N바", "AWG14", E(e12n[2], (0, 4, 0)), E(nbar_hole(5), (0, -8, 0)), ends=("12V-", "N바"), order=3,
               note="절연형 RSD 의 0 V 를 배터리 − 에 한 점으로 묶는다"))
    dkp = lambda x: E((x, -193.0, Z0 + 8.0))                             # noqa: E731  DKP 선 끝 (-y 면에서 15) — 바로 바닥으로
    L.append(N("wire_pos", "상시+_F6-DKP입력", "AWG14", E(unit_out("F6"), oy), dkp(-102.0), ends=("F6_", "DCDC_36V-5V"), order=2))
    L.append(N("wire_neg", "DKP-Vin_N바", "AWG14", dkp(-94.0), E(nbar_hole(7), (0, -8, 0)), ends=("DCDC_36V-5V", "N바"), order=3))
    L.append(N("wire_5v", "5V+_DKP-WAGO", "AWG14", dkp(-78.0), E(e5p[0], (0, -4, 0)), ends=("DCDC_36V-5V", "5V+"), order=3))
    L.append(N("wire_neg", "5V-_DKP-WAGO", "AWG14", dkp(-86.0), E(e5n[0], (0, -4, 0)), ends=("DCDC_36V-5V", "5V-"), order=3))
    jx = -136.3                                                          # 젯슨 I/O 면
    L.append(N("wire_12v", "젯슨DC_12V_5.5x2.5", "DC2", E(e12p[4], (0, 4, 0)), E((jx, 154.0, 142.3), (-25, 0, 0)),
               ends=("12V+", "12V-", "젯슨"), order=4, note="5.5 × 2.5 플러그 + 18 AWG 2 심 (+ 는 12V+, − 는 12V− WAGO)"))
    L.append(N("wire_12v", "키스위치_12V+", "AWG18", E(e12p[0], (0, 4, 0)), E((-316.7, -77.0, 201.0), ox), ends=("12V+", "키스위치"), order=4,
               note="12 V → 키 스위치 (포트 칸)"))
    L.append(N("wire_12v", "키-비상정지", "AWG18", E((-316.7, -73.0, 207.0), ox), E((-340.0, -121.2, 236.0), (0, 4, 0)),
               ends=("키스위치", "비상정지본체"), order=4, note="키 → 비상정지 1NC"))
    L.append(N("wire_12v", "비상정지-코일+", "AWG18", E((-340.0, -121.2, 248.0), (0, 4, 0)), E((156.0, -39.0, 151.5), oy),
               ends=("비상정지본체", "코일플러그"), order=4, note="비상정지 → 컨택터 코일 + (TVS 는 플러그 뒤에)"))
    L.append(N("wire_neg", "코일-_N바", "AWG18", E((160.0, -39.0, 151.5), oy), E(nbar_hole(2), (0, -8, 0)), ends=("코일플러그", "N바"), order=4))
    L.append(N("wire_12v", "12V+_WAGO-DFR0457", "AWG18", E(e12p[1], (0, 2, 0), (0, 0, 6), (0, 9, 0)), E((92.0, -126.0, 137.0), (0, -4, 0)), ends=("12V+", "MOSFET모듈_전조등"), order=4))
    L.append(N("wire_neg", "12V-_WAGO-DFR0457", "AWG18", E(e12n[0], (0, 4, 0)), E((97.0, -126.0, 137.0), (0, -4, 0)), ends=("12V-", "MOSFET모듈_전조등"), order=4))
    L.append(N("wire_12v", "12V+_WAGO-표시등공통", "AWG18", E(e12p[2], (0, 4, 0)), E((75.0, -158.0, 140.0), ox), ends=("12V+", "표시등MOSFET"), order=4,
               note="표시등 공통 (+12 V) — MOSFET 블록에서 표시등 케이블 공통선과 잇는다"))
    for s, nm, y, esc in ((1, "왼", -112.0, ((4, 0, 0), (0, 0, 14), (0, 24, 0))), (-1, "오른", -118.0, ((4, 0, 0), (0, -6, 0)))):
        L.append(N("wire_12v", f"전조등_{nm}_2심", "DC2", E((99.0, y, 142.5), *esc), E((302.5, s * 150.0, 225.0), (-6, 0, 0)),
                   ends=("MOSFET모듈_전조등", f"전조등_MR16_5W_{nm}"), order=6, note="램프 소켓 리드 300 + 연장 2 심 (로우사이드 출력)"))
    L.append(N("wire_pos", "충전+_F7-GX16", "AWG14", E(unit_out("F7"), oy), E((-337.4, -79.0, 181.0), ox), ends=("F7_", "충전단자"), order=4,
               note="충전 단자 (패널 쪽 암 핀이어야 한다 — D-10)"))
    L.append(N("wire_neg", "충전-_GX16-N바", "AWG14", E((-337.4, -71.0, 175.0), ox), E(nbar_hole(1), (0, -8, 0)), ends=("충전단자", "N바"), order=4))
    L.append(N("wire_pos", "전압계+_F2입력", "AWG22", E(unit_in("F2", 8.0), (0, -8, 0)), E((-308.9, 78.0, 207.0), ox),
               ends=("F2_", "전압계"), order=5, note="스위치 마디 (컨택터 뒤) — 주행 중에만 켜짐 (D-10)"))
    L.append(N("wire_neg", "전압계-_N바", "AWG22", E((-308.9, 72.0, 201.0), ox), E(nbar_hole(0), (0, -8, 0)), ends=("전압계", "N바"), order=5))
    # 5 V
    for k, (nm, y) in enumerate((("오른", -81.0), ("왼", 81.0))):
        s = -1 if y < 0 else 1
        L.append(N("wire_5v", f"팬_{nm}_5V", "FAN2", E((-376.5, y, 140.0), *(((0, -s * 4, 0),) if s < 0 else ())), E(e5n[2 + 2 * k], (0, -4, 0)),
                   ends=(f"배기팬_80mm_{nm}", "5V-"), order=6, note="팬 리드 300 + 2 핀 연장 — 2 심 (− 는 5V− WAGO, + 는 옆 5V+ WAGO)"))
    L.append(N("wire_5v", "USB허브_5V", "DC2", E(e5p[3], (0, -2, 0), (0, 0, 6), (0, -9, 0)), E((-290.0, -5.0, 140.0), (-6, 0, 0)),
               ends=("5V+", "5V-", "USB허브"), order=6, note="허브 5 V 2 A 잭 — 2 심 (+ 는 5V+, − 는 5V− WAGO)"))
    L.append(N("wire_5v", "5V+_WAGO-ESP32", "AWG22", E(e5p[1], (0, -2, 0), (0, 0, 6), (0, -9, 0)), E((-56.0, -104.0, 140.0), (8, 0, 0)), ends=("5V+", "ESP32"), order=5))
    L.append(N("wire_neg", "5V-_WAGO-ESP32", "AWG22", E(e5n[1], (0, -2, 0), (0, 0, 6), (0, -9, 0)), E((-56.0, -98.0, 141.0), (8, 0, 0)), ends=("5V-", "ESP32"), order=5))
    # ======== ESP32 신호 (받침보드 +x 끝 x -56 / -x 끝 x -124, 나사 단자 [가정]) ========
    L.append(N("wire_sig", "감지_컨택터출력-ESP32", "AWG22", E((166.0, -70.0, 176.0), (0, 0, 8), (0, 18, 0)), E((-56.0, -110.0, 140.0), (8, 0, 0)),
               ends=("컨택터_LS", "ESP32"), order=5, note="분압 (100 k / 8.2 k) 은 받침보드 위 — 끊기면 정지 · 래치"))
    L.append(N("wire_sig", "UART+전원_ESP32-앞왼마스터", "CAB5", E((-56.0, -90.0, 140.0), (8, 0, 0)), top((297.5, 74.0, 151.0), (0, -10, 0)),
               ends=("ESP32", "앞왼_분리형"), order=5, note="TX · RX · GND + 전원 핀 2 (DISABLE_BUTTON) — 5 심"))
    L.append(N("wire_sig", "UART+전원_ESP32-뒤왼마스터", "CAB5", E((-124.0, -72.0, 140.0), (-4, 0, 0)), top((-277.5, 74.0, 151.0), (0, -10, 0)),
               ends=("ESP32", "뒤왼_분리형"), order=5))
    L.append(N("wire_sig", "PWM_ESP32-DFR0457", "CAB3", E((-56.0, -82.0, 142.0), (8, 0, 0)), E((99.0, -102.0, 140.0), ox, (0, 0, 10), (0, 14, 0), (-12, 0, 0)),
               ends=("ESP32", "MOSFET모듈_전조등"), order=5, note="Gravity 3 핀 (GND · VCC · PWM)"))
    L.append(N("wire_sig", "표시등게이트_ESP32-MOSFET4", "CAB5", E((-56.0, -74.0, 140.0), (8, 0, 0)), E((5.0, -168.5, 140.0), (-4, 0, 0)),
               ends=("ESP32", "표시등MOSFET"), order=5, note="GPIO 4 + GND"))
    L.append(N("wire_sig", "마스터-슬레이브_앞", "MS4", top((322.5, 74.0, 151.0), (0, -10, 0)), top((252.5, -74.0, 151.0), (0, 10, 0)),
               ends=("앞왼_분리형", "앞오른_분리형"), order=5, note="호버보드 원래 4 핀 (12~15 V 신호 — ESP32 에 물리지 않는다)"))
    L.append(N("wire_sig", "마스터-슬레이브_뒤", "MS4", top((-252.5, 74.0, 151.0), (0, -10, 0)), top((-322.5, -74.0, 151.0), (0, 10, 0)),
               ends=("뒤왼_분리형", "뒤오른_분리형"), order=5))
    # ======== 모터 — 그로밋에서 올라와 상선 3 (보드 바깥 변) + 홀 (안쪽 변 XH) ========
    for (gx, gy, bx, rot, nm) in ((165.0, 154.3, 287.5, 0, "앞왼"), (165.0, -154.3, 287.5, 180, "앞오른"),
                                  (-165.0, 154.3, -287.5, 0, "뒤왼"), (-165.0, -154.3, -287.5, 180, "뒤오른")):
        sy = 1 if gy > 0 else -1
        sx = 1 if gx > 0 else -1
        L.append(N("wire_motor", f"모터케이블_{nm}", "MOTOR", ends=("케이블", "바닥판", "상선", "홀_"), order=0,
                   fixed=[(gx, gy, Z0 - 3.0), (gx, gy, Z0 + 5.0)], note="허브모터 케이블 (그로밋 아래는 v27 경로) — 위에서 상선 3 · 홀로 갈라진다"))
        xs = sorted((bx + d for d in (-10.0, 0.0, 10.0)), key=lambda x: abs(x - gx))
        for k, x in enumerate(xs):
            a = math.radians(90 * k + 45)
            st = (gx + 2.5 * math.cos(a), gy + 2.5 * math.sin(a), zf(OD["PHASE"]))
            L.append(N("wire_motor", f"상선{k+1}_{nm}", "PHASE", E(st), E((x, sy * 145.2, ZB), (0, sy * 4, 0)),
                       ends=(f"모터케이블_{nm}", f"{nm}_분리형"), order=1))
        hx = bx + (-38.0 if rot == 0 else 38.0)
        a = math.radians(-45 * sy)
        st = (gx + 2.5 * math.cos(a), gy + 2.5 * math.sin(a), zf(OD["HALL"]))
        L.append(N("wire_motor", f"홀_{nm}", "HALL", E(st), top((hx, sy * 74.0, 151.0), (0, -sy * 10, 0)),
                   ends=(f"모터케이블_{nm}", f"{nm}_분리형"), order=1))
    # ======== USB · LAN · 동축 ========
    hub = lambda k: E((HUB_PORTS[k], -5.0, 150.85), (0, 0, 30), (0, -32, 0))          # noqa: E731  허브 윗면 포트 (위로 꽂음)
    L.append(N("wire_usb", "USB3_앞카메라-젯슨", "USB3", E((357.3, -106.0, 172.0), (0, -5, 0)), E((jx, 117.4, 149.3), (-36, 0, 0)),
               ends=("뎁스카메라_앞", "젯슨"), order=7, note="USB-C ↔ A 3.x"))
    L.append(N("wire_usb", "USB3_뒤카메라-젯슨", "USB3", E((-357.3, 106.0, 250.0), (0, 5, 0)), E((jx, 117.4, 141.0), (-30, 0, 0)),
               ends=("뎁스카메라_뒤", "젯슨"), order=7))
    L.append(N("wire_usb", "USB3_허브-젯슨", "USB3", E((-160.0, -5.0, 140.0), (20, 0, 0)), E((jx, 100.4, 149.3), (-36, 0, 0)),
               ends=("USB허브", "젯슨"), order=7, note="허브 업스트림"))
    L.append(N("wire_usb", "USB3_포트A1-젯슨", "USB3", E((-320.5, 38.0, 192.0), ox), E((jx, 100.4, 141.0), (-30, 0, 0)),
               ends=("USB-A_1", "젯슨"), order=7, note="A 수-수 0.5 m"))
    L.append(N("wire_usb", "USB-C연장_포트-젯슨", "USB2", E((-300.5, 75.0, 178.0), ox), E((jx, 68.0, 138.6), (-22, 0, 0)),
               ends=("USB-C_MT723", "젯슨"), order=7, note="MT723 자체 케이블 30 cm — 젯슨 USB-C (복구 · 콘솔)"))
    L.append(N("wire_usb", "LAN_포트-젯슨", "LAN", E((-320.5, -38.0, 192.0), ox), E((jx, 83.0, 143.6), (-25, 0, 0)),
               ends=("RJ45", "젯슨"), order=7, note="CAT6 패치"))
    L.append(N("wire_usb", "USB3_포트A2-허브", "USB3", E((-320.5, 0.0, 192.0), ox), hub(0), ends=("USB-A_2", "USB허브"), order=7))
    L.append(N("wire_usb", "USB2_ESP32-허브", "USB2", E((-114.6, -92.0, 146.8), (-20, 0, 0)), hub(6), ends=("ESP32", "USB허브"), order=7,
               note="마이크로 5 핀 ↔ A 0.3 m"))
    L.append(N("wire_usb", "USB2_IMU-허브", "USB2", E((-12.5, -134.0, 138.35), (18, 0, 0)), hub(5), ends=("IMU_iAHRS", "USB허브"), order=7))
    for s, nm, y in ((1, "왼", 122.0), (-1, "오른", 128.0)):
        L.append(N("wire_usb", f"동축_안테나{nm}-젯슨", "COAX", E((345.5, s * 100.0, 260.0), (-4, 0, 0)), E((-58.0, y, 140.0), (6, 0, 0)),
                   ends=("안테나벌크헤드", "젯슨"), order=8, note="MHF4 ↔ RP-SMA — 젯슨 M.2 카드 (모듈 밑)"))
    # ======== 센서 모듈 케이블 — 모듈 위 고정 구간 + 덮개 그로밋 (-100, ±150) 밑에서부터 바닥 ========
    L.append(N("wire_usb", "USB2_GNSS-허브", "USB2", E((-94.0, 158.0, 290.0)), hub(3), ends=("GNSS", "USB허브"), order=8,
               pre=[(-69.5, 165.0, 325.85), (-94.0, 165.0, 325.85), (-94.0, 158.0, 325.85)],
               note="GPS630 자체 케이블 (모듈 밑판 가장자리 → 덮개 그로밋)"))
    L.append(N("wire_usb", "USB2_라이다어댑터-허브", "USB2", E((-104.0, 142.0, 290.0)), hub(2), ends=("라이다USB어댑터", "USB허브"), order=8,
               pre=[(5.0, 40.0, 326.0), (-66.0, 40.0, 326.0), (-66.0, 142.0, 326.0), (-104.0, 142.0, 326.0)]))
    L.append(N("wire_sig", "라이다_UART4", "CAB3", ends=("라이다", "라이다판", "라이다USB어댑터"), order=0,
               fixed=[(97.7, 19.5, 505.0), (97.7, 37.5, 505.0), (97.7, 37.5, 492.0), (100.0, 37.5, 340.0),
                      (100.0, 37.5, 324.0), (35.0, 37.5, 324.0)],
               note="STL-19P ZH1.5 4 핀 → CP2102 어댑터 (D500 키트) — 기둥 안쪽 면을 따라"))
    L.append(N("wire_sig", "표시등케이블_5심", "ST5", E((-96.0, -155.0, 290.0)), E((75.0, -179.0, 140.0), ox),
               ends=("상태표시등", "표시등MOSFET"), order=8,
               pre=[(-30.0, -165.0, 311.85), (-30.0, -165.0, 306.0), (-96.0, -165.0, 306.0), (-96.0, -155.0, 306.0)],
               note="공통 + 초록 · 노랑 · 빨강 · 부저 — MOSFET 4 출력"))
    return L


# ---------------------------------------------------------------- 짓기 (routes_data.ROUTES 를 읽는다)
class W:
    """지어진 전선 — 3D 꺾은선 하나 (pts). 검증 · BOM 이 쓴다."""

    def __init__(self, n, pts):
        self.grp, self.label, self.kind, self.ends, self.note = n.grp, n.label, n.kind, n.ends, n.note
        self.d = n.d
        self.pts = [tuple(p) for p in pts]

    def legs(self):
        return [self.pts]

    def length(self):
        return plen(self.pts)


def build(z0):
    assert abs(z0 - Z0) < 1e-6, f"바닥 높이가 다르다 {z0} != {Z0}"
    try:
        from routes_data import ROUTES
    except ImportError:                                           # 아직 경로가 없다 — router.py 를 먼저
        ROUTES = {}
    P = parts()
    groups = {k: [] for k in GROUPS}
    for k, v in P.items():
        groups[k] += v
    L = []
    missing = []
    for n in nets():
        pts = n.fixed if n.fixed else ROUTES.get(n.label)
        if not pts:
            missing.append(n.label)
            continue
        w = W(n, pts)
        L.append(w)
        t = tube(w.pts, w.d)
        t.label = w.label
        groups[w.grp].append(t)
    if missing:
        print(f"  ⚠ 경로 없음 {len(missing)}: {missing[:5]} … — router.py 를 돌린다")
    return groups, L


# ---------------------------------------------------------------- 겹침 검사 (조각별)
def solid_index(parts_):
    out = []
    for p in parts_:
        for s in p.solids():
            b = s.bounding_box()
            out.append((p.label, s, np.array(tuple(b.min)), np.array(tuple(b.max))))
    return out


def collide(shape, index, ends=(), tol=0.05):
    """shape 의 조각 ↔ index 조각 겹침 {이름: mm³}. 이름에 ends 문자열이 든 것은 봐준다.
    겹친 원기둥 · 구 묶음 (Compound) 을 통째로 불리언에 넣으면 OCC 가 빈 결과를 낼 때가 있다 → 조각 하나씩 (v28)."""
    out = {}
    for s in shape.solids():
        b = s.bounding_box()
        lo, hi = np.array(tuple(b.min)), np.array(tuple(b.max))
        for lab, t, tlo, thi in index:
            if (lo > thi).any() or (tlo > hi).any() or any(e in lab for e in ends):
                continue
            r = s & t
            v = r.volume if r is not None else 0.0
            if v > tol:
                out[lab] = round(out.get(lab, 0.0) + v, 1)
    return out


# ---------------------------------------------------------------- BOM (형상에서 길이 · 개수, 가격 2026-10-05 조사 — research/2026-10-05_wiring.md)
def bom_rows(L):
    m = {}
    for w in L:
        n = {"CAB5": 5, "CAB3": 3, "FAN2": 2, "DC2": 2}.get(w.kind, 1)
        g = {"CAB5": "AWG22", "CAB3": "AWG22", "FAN2": "AWG22", "DC2": "AWG18"}.get(w.kind, w.kind)
        m[g] = m.get(g, 0.0) + w.length() * n
    need = {g: math.ceil(v * 1.15 / 1000) for g, v in m.items()}
    named = {w.label: w.length() for w in L}
    WIRE_URL = "https://www.11st.co.kr/products/3306373594"
    rows = []
    for g, price, use in (("AWG10", 1860, "주전원 · 컨택터 · 레일 공급"), ("AWG14", 770, "바퀴 보드 · 변환기 · 충전 · 점퍼"),
                          ("AWG18", 420, "12 V · 코일 · 젯슨 DC · 전조등 (2 심은 2 가닥으로 셈)"),
                          ("AWG22", 320, "신호 · 전압계 · 감지 · 팬 연장 (다심은 심 수로 셈)")):
        if g in need:
            rows.append(("배선", f"실리콘선 {g[3:]} AWG (적 · 흑)", f"{m[g] / 1000:.2f} m + 여유 15 % — {use}", need[g], price,
                         WIRE_URL, "11번가 우리파파 1 m 단위 (배송비 3,000 별도). 적 · 흑 색상 옵션 확인"))
    front = named.get("USB3_앞카메라-젯슨", 0) / 1000
    rear = named.get("USB3_뒤카메라-젯슨", 0) / 1000
    cl, cr = named.get("동축_안테나왼-젯슨", 0) / 1000, named.get("동축_안테나오른-젯슨", 0) / 1000
    lan = named.get("LAN_포트-젯슨", 0) / 1000
    rows += [
        ("전원", "분기 차단기 (바퀴 보드 4)", "DC 차단기 1P 250 VDC **20 A** (BA49-63 DC) — 메인과 같은 계열", 4, 7000,
         "https://mega-tec.co.kr/product/detail.html?product_no=504",
         "국내 소용량 (10 A 이하) DC 차단기는 못 찾았다 — 20 A 는 14 AWG 전선 보호 (보드는 펌웨어 7 A 제한)"),
        ("전원", "DC 퓨즈 홀더 (변환기 2 · 충전 1)", "1P 1000 VDC 32 A, 10×38, DIN", 3, 3200,
         "https://mega-tec.co.kr/product/detail.html?product_no=507", "메가에너텍. 치수는 [가정] 18 × 81 × 58"),
        ("전원", "PV 퓨즈 10×38", "15 A (gPV, 1000 VDC) — 3 + 예비 3", 6, 1600,
         "https://mega-tec.co.kr/product/detail.html?product_no=508", "국내는 15 A 부터 — 14 AWG 보호. 변환기는 내부 보호"),
        ("전원", "DIN 레일", "35 mm 레일 160 mm (분기 레일 136 으로 자름)", 1, 450,
         "https://mega-tec.co.kr/product/detail.html?product_no=505", ""),
        ("전원", "DIN 엔드 스토퍼", "35 mm 레일용", 2, 300, "https://mega-tec.co.kr/product/detail.html?product_no=506", ""),
        ("전원", "음극 단자대 (N 바)", "황동 바 12 구멍 (구멍당 16 mm² 이상, 절연 받침)", 1, None, "",
         "모든 0 V 를 한 점에 — 국내 M6 버스바는 구매대행뿐이라 전기자재 N 단자대로 (구매처 확인)"),
        ("배선", "배터리 커넥터", "AMASS XT90H-F (암, 배터리 XT-90 수와 짝)", 1, 1045,
         "https://www.eleparts.co.kr/goods/view?no=12773228", "엘레파츠 VAT 포함. 충전선 14 AWG 를 같은 땜납 컵에"),
        ("배선", "레버 커넥터", "WAGO 221-415 (5 구) — 12 V ± · 5 V ±", 4, 847,
         "https://www.eleparts.co.kr/goods/view?no=7275762", "엘레파츠 VAT 포함. 바닥은 양면테이프"),
        ("배선", "링 단자 세트", "Coms IT-300 (링 · Y · 총알 300 개)", 1, 11900,
         "https://www.eleparts.co.kr/goods/view?no=4063230", "M4 (컨택터) 구성 확인"),
        ("배선", "페룰 세트", "Coms HA067", 1, 4680, "https://www.eleparts.co.kr/goods/view?no=18976115",
         "차단기 · 퓨즈 홀더 · WAGO · N 바에 넣는 선 끝"),
        ("배선", "젯슨 DC 플러그", "STech 납땜형 DC (수) 5.5 / 2.5", 1, 688,
         "https://www.eleparts.co.kr/goods/view?no=6783563", "18 AWG 2 심 연결"),
        ("배선", "케이블 타이", "100 × 2.5 (1,000 개)", 1, 3069, "https://www.navimro.com/g/1062007/",
         "바닥 다발을 타이 마운트에 묶는다 (꺾이는 곳마다)"),
        ("배선", "부착식 타이 마운트", "동아베스텍 DAMB-10 (구멍 4.3) — 바닥판 M5 격자 구멍에 볼트로도 가능", 40, None,
         "https://www.navimro.com/g/374730/", "1,000 개 19,990 (VAT 별도) 뿐 — 소량 구매처 확인"),
        ("제어", "USB 허브 (유전원 7 포트)", "POWERLAN PL-UH307P USB 3.0 — 130 × 46 × 22, 5 V 2 A", 1, 23100,
         "https://www.eleparts.co.kr/goods/view?no=12485398", "5 V 는 DKP-55 에서 — DC 잭 규격 실물 확인"),
        ("안전", "표시등 MOSFET 1 채널 (색 3 + 부저)", "15 A MOSFET 트리거 모듈 (로직 3.3~20 V, 부하 5~36 V, 로우사이드)", 4, 700,
         "https://mechasolution.com/goods/goods_view.php?goodsNo=605618", "메카솔루션 — '구매 불가' 표시, 재고 확인. 치수 [가정] 34 × 17"),
        ("포트", "USB 3.0 A 수-수 케이블", "NEXT-1672U3-AA 0.5 m (패널 USB-A 2 → 젯슨 · 허브)", 2, 1690,
         "https://www.eleparts.co.kr/goods/view?no=17672924", "경로가 길면 1 m"),
        ("포트", "LAN 패치", f"CAT6 UTP — 경로 {lan:.2f} m", 1, None, "https://www.eleparts.co.kr/goods/view?no=2922243",
         "Coms C3898 30 cm 1,450 — 바닥으로 돌면 길어진다, 길이 맞춰 구매"),
        ("제어", "ESP32 USB 케이블", "마이크로 5 핀 ↔ A (경로 길이에 맞춤)", 1, None,
         "https://www.eleparts.co.kr/goods/view?no=2933871", "NEXI NX15 0.3 m 603 (VAT 별도) — 바닥으로 돌면 0.5 m 이상"),
        ("통신", "안테나 동축 피그테일", f"MHF4 ↔ RP-SMA 벌크헤드 — 경로 왼 {cl:.2f} · 오른 {cr:.2f} m", 2, None, "",
         "키트의 300 mm 는 짧다 (젯슨 M.2 카드는 뒤, 안테나는 앞) — 길이 맞는 국내 구매처 확인"),
        ("센서", "카메라 USB 길이", f"경로 앞 {front:.2f} · 뒤 {rear:.2f} m — 카메라 줄 IK045 1 m ×2 로 {'모자람' if max(front, rear) > 0.95 else '충분'}",
         0, None, "", "모자라면 Coms IK045 1.5 m · 2 m 확인"),
    ]
    return rows
