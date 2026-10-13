r"""외관판 — 실외용 (v10, v11 에서 벽 높이 · 윗판 바꿈). 벽은 바퀴 안쪽, 프레임 바깥면, 펜더는 따로.

v11: **윗판은 아직 없다** — 상부 프레임을 올린 뒤 그 위에 상부판을 얹는다 (사용자 2026-10-04).
그래서 벽을 **상부 프레임 윗면** (기둥 위 3030 가정, z=296.85) 까지 올리고, 상부 레일 · 상부
크로스 바깥 슬롯 높이 (z=281.85) 에 M5 구멍을 미리 뚫는다. 비상정지 · 안테나는 상부판으로 간다.

판 (평판, 재단용 DXF 를 같이 낸다)
  · 옆면 2  ACM 3t   y=±(225~228), 하부 레일 밑면 (z=93.85) ~ 기둥 윗면 (z=266.85)
  · 앞 · 뒤  PC 3t (UV 코팅, **불투명** 흑색 · 회색 — v13)  x=±(380~383), 옆면 끝을 덮는다 — 범퍼 쪽이라 충격에 강한 PC
  · 윗판    ACM 3t   기둥 6 개 윗면에 얹고 M8 (기둥 중심 구멍에 탭)
3D 출력 (ASA — 실외 자외선)
  · (v12 에서 뺌) 펜더 4   바퀴 위 원호 덮개 (안쪽 R92, 타이어 R79.35 와 12.6 여유). 양 끝 탭을 옆판과 같이
             하부 레일 바깥 슬롯 (z=108.85) 에 M5 로 문다
  · 범퍼 앞 · 뒤 — 끝 크로스 바깥 슬롯 볼트를 같이 쓴다
  · 루버 덮개 2 — 옆면 흡기 루버 위, 아래로만 열려 빗물이 안 들어간다

고정은 전부 프로파일 바깥 슬롯 M5 T 너트 (구멍 Ø5.5). 판 이음새에는 EPDM 개스킷 테이프 (모델 없음).
"""

import math

from build123d import (Align, Box, BuildPart, BuildSketch, Circle, Cylinder, Locations, Mode,
                       Pos, Rectangle, Rot, SlotOverall)

MIN = (Align.CENTER, Align.CENTER, Align.MIN)

ACM_T, PC_T = 3.0, 3.0
HOLE = 5.5                          # M5 통과
LID_HOLE = 8.5                      # M8 — 기둥 중심 구멍 (Ø6.8) 에 탭
ESTOP_D = 22.5                      # 비상정지 22 파이 (Koino KSE 계열 B 22.2~22.5)
ANT_D = 6.5                         # SMA 벌크헤드
FAN = dict(d=76.0, pitch=71.5, hole=4.4)   # 80 mm 팬
FENDER_R, FENDER_T, FENDER_W = 92.0, 3.0, 70.0
FENDER_ANG = 15.0                   # 펜더가 축 높이에서 이 각도 위에서 끝난다
VENT = dict(l=80.0, w=6.0, n=3, pitch=12.0)
UPPER_S = 30.0                      # 상부 프레임 3030 가정 (v00_ref 의 옛 상부 프레임과 같은 높이)
UPPER_RAIL_X = (-290.0, -105.0, 105.0, 290.0)   # 옆판 위쪽 — 상부 사이드 레일 슬롯 볼트 x

# ---- v14 앞 · 뒤판 (좌표는 차량 y, z)
# 뎁스 카메라 — Intel RealSense D435 급 90 × 25 × 25 [공개 사양, 확인 필요]. 앞 · 뒤 하나씩, 판 안쪽.
CAM = dict(w=90.0, h=25.0, d=25.0, z=250.0, win=(92.0, 27.0))   # v15: z 240 → 250 (아래에 포트 칸)
VISOR = dict(w=100.0, out=30.0, t=3.0)              # 카메라 창 위 빗물막이 차양 (ASA)
# 고무 범퍼 — v15: 원기둥 4 개 → **이어진 고무 띠** 하나 (앞 · 뒤). 판 전체 폭, 높이 30, 두께 25,
# 바깥 모서리 R8. 끝 크로스 바깥 슬롯 높이에 M6 4 개 (y=±60 · ±170) 로 판을 지나 슬롯에 문다.
RUBBER = dict(h=30.0, out=25.0, r=8.0, ys=(-170.0, -60.0, 60.0, 170.0), hole=6.5)
# 뒤판 오목 포트 칸 — ASA 트레이 + 위로 여는 덮개. 뒤 드라이버 (z≤159.9) 위, 상부 프레임 (z≥266.9) 아래.
BAY = dict(y=0.0, z=196.0, w=200.0, h=68.0, depth=35.0, wall=2.5, flange=5.0, cover_t=3.0)  # v15: 카메라 아래 가운데
# 포트 (차량 y, z, 이름, ("o", Ø) 또는 ("r", 폭, 높이)) — 크기는 일반 패널형 [가정]
BAY_PORTS = [
    (-70.0, 213.0, "충전 GX16", ("o", 16.0)), (-35.0, 213.0, "키 스위치", ("o", 19.0)),
    (15.0, 213.0, "전압계", ("r", 46.0, 27.0)), (65.0, 213.0, "ESP32 USB-C", ("o", 16.0)),
    (-65.0, 181.0, "USB 1", ("o", 22.0)), (-30.0, 181.0, "USB 2", ("o", 22.0)),
    (10.0, 181.0, "RJ45", ("o", 28.0)),
]
ESTOP = dict(y=-150.0, z=240.0, head_d=40.0, head_l=32.5, body=37.0, body_l=48.5)   # Koino KSE 계열 [도면]
FANS = [(-150.0, 175.0), (150.0, 175.0)]          # v15: 80 mm 배기 팬 양쪽 (뒤판, 덮개 밖)


class Shell:
    def __init__(self, A):
        self.A = A
        self.L, self.W, self.S = A.BASE_L, A.BASE_W, A.S
        self.z0 = A.RAIL_Z - A.S / 2                         # 하부 레일 밑면
        self.z_post = A.FRAME_TOP + A.POST_H                 # 기둥 윗면 = 상부 프레임 밑면
        self.z1 = self.z_post + UPPER_S                      # 상부 프레임 윗면 (가정) = 벽 위끝
        self.upper_slot_z = self.z_post + UPPER_S / 2        # 상부 레일 · 상부 크로스 바깥 슬롯
        self.H = self.z1 - self.z0                           # 벽 높이
        self.rail_slot_z = A.RAIL_Z                          # 하부 레일 · 끝 크로스 바깥 슬롯
        self.post_slot_z = A.FRAME_TOP + A.POST_H / 2        # 기둥 바깥 슬롯 (가운데)
        self.y_out = self.W / 2 + ACM_T                      # 옆판 바깥면
        self.x_out = self.L / 2 + PC_T                       # 앞뒤판 바깥면
        ex = (FENDER_R + FENDER_T) * math.cos(math.radians(FENDER_ANG))
        self.fender_tab_x = sorted(sx * A.AXLE_X + d for sx in (-1, 1) for d in (-ex, ex))

    # ---- 판별 특징 (판 로컬 u, v — 벽은 v=0 이 판 아래끝)
    def features(self, kind):
        A, V0 = self.A, self.z0
        holes, slots, circles, rects = [], [], [], []
        if kind in ("side_l", "side_r"):
            U, Vh = self.L, self.H
            holes += [(x, self.rail_slot_z - V0, HOLE) for x in self.fender_tab_x]   # 펜더 탭과 같이
            holes += [(x, self.post_slot_z - V0, HOLE) for x in A.POST_X]
            holes += [(x, self.upper_slot_z - V0, HOLE) for x in UPPER_RAIL_X]        # 상부 레일 (나중)
            cx = (A.AXLE_CROSS_X + A.CROSS_X) / 2                                     # 앞 끝 칸
            for i in range(VENT["n"]):
                slots.append((cx, self.z_post - V0 - 22 - i * VENT["pitch"], VENT["l"], VENT["w"]))
        elif kind in ("front", "rear"):
            U, Vh = self.W + 2 * ACM_T, self.H
            holes += [(y, self.rail_slot_z - V0, RUBBER["hole"]) for y in RUBBER["ys"]]   # 고무 범퍼 M6
            holes += [(sy * A.RAIL_Y, self.post_slot_z - V0, HOLE) for sy in (-1, 1)]
            holes += [(y, self.upper_slot_z - V0, HOLE) for y in (-120.0, 0.0, 120.0)]   # 상부 끝 크로스 (나중)
            rects.append((0.0, CAM["z"] - V0, *CAM["win"]))                            # 카메라 창
            if kind == "rear":
                for fy, fz in FANS:                                                    # 배기 팬 양쪽
                    fu, fv = fy, fz - V0
                    circles.append((fu, fv, FAN["d"]))
                    holes += [(fu + sx * FAN["pitch"] / 2, fv + sy * FAN["pitch"] / 2, FAN["hole"])
                              for sx in (-1, 1) for sy in (-1, 1)]
                circles.append((ESTOP["y"], ESTOP["z"] - V0, ESTOP_D))                  # 비상정지
                rects.append((BAY["y"], BAY["z"] - V0, BAY["w"], BAY["h"]))             # 포트 칸 구멍
        elif kind == "lid":
            U, Vh = self.L + 2 * PC_T, self.W + 2 * ACM_T
            holes += [(x, sy * A.RAIL_Y, LID_HOLE) for x in A.POST_X for sy in (-1, 1)]
            circles.append((-300.0, -140.0, ESTOP_D))                                  # 비상정지
            holes += [(300.0, sy * 100.0, ANT_D) for sy in (-1, 1)]                     # GNSS · LTE
        else:
            raise ValueError(kind)
        if kind in ("side_r", "rear"):          # 이 두 판은 판 로컬 u 가 차량 -x / -y 로 간다 (바깥에서 본 모습)
            holes = [(-u, v, d) for u, v, d in holes]
            slots = [(-u, v, l, w) for u, v, l, w in slots]
            circles = [(-u, v, d) for u, v, d in circles]
            rects = [(-u, v, w, h) for u, v, w, h in rects]
        self._rects = rects
        return U, Vh, holes, slots, circles

    def sketch(self, kind):
        U, Vh, holes, slots, circles = self.features(kind)
        wall = kind != "lid"
        with BuildSketch() as sk:
            with Locations((0, Vh / 2 if wall else 0)):
                Rectangle(U, Vh)
            for u, v, d in holes + circles:
                with Locations((u, v)):
                    Circle(d / 2, mode=Mode.SUBTRACT)
            for u, v, l, w in slots:
                with Locations((u, v)):
                    SlotOverall(l, w, mode=Mode.SUBTRACT)
            for u, v, w, h in self._rects:
                with Locations((u, v)):
                    Rectangle(w, h, mode=Mode.SUBTRACT)
        return sk.sketch

    def panel(self, kind):
        """판 로컬 스케치를 3D 자리로 — 두께는 차량 바깥쪽으로."""
        from build123d import Plane, extrude
        t = PC_T if kind in ("front", "rear") else ACM_T
        pl = {
            # 판 로컬 v 가 위 (+z) 가 되게 고른 평면. 두께는 바깥면에서 안쪽으로 / 프레임면에서 바깥으로.
            "side_l": Plane(origin=(0, self.y_out, self.z0), x_dir=(1, 0, 0), z_dir=(0, -1, 0)),
            "side_r": Plane(origin=(0, -self.y_out, self.z0), x_dir=(-1, 0, 0), z_dir=(0, 1, 0)),
            "front": Plane(origin=(self.L / 2, 0, self.z0), x_dir=(0, 1, 0), z_dir=(1, 0, 0)),
            "rear": Plane(origin=(-self.L / 2, 0, self.z0), x_dir=(0, -1, 0), z_dir=(-1, 0, 0)),
            "lid": Plane(origin=(0, 0, self.z1), x_dir=(1, 0, 0), z_dir=(0, 0, 1)),
        }[kind]
        with BuildPart() as p:
            with BuildSketch(pl):
                from build123d import add
                add(self.sketch(kind))
            extrude(amount=t)
        part = p.part
        part.label = {"side_l": "옆판_왼쪽_ACM3t", "side_r": "옆판_오른쪽_ACM3t", "front": "앞판_PC3t",
                      "rear": "뒤판_PC3t", "lid": "윗판_ACM3t"}[kind]
        return part

    # ---- ASA 출력 부품
    def fender(self, sx, sy):
        A = self.A
        y0 = sy * self.y_out
        with BuildPart() as p:
            Cylinder(FENDER_R + FENDER_T, FENDER_W, rotation=(90, 0, 0))
            Cylinder(FENDER_R, FENDER_W + 2, rotation=(90, 0, 0), mode=Mode.SUBTRACT)
            cut = (FENDER_R + FENDER_T) * math.sin(math.radians(FENDER_ANG))
            with Locations((0, 0, cut - 200)):
                Box(400, FENDER_W + 4, 400, mode=Mode.SUBTRACT)
        arc = Pos(sx * A.AXLE_X, y0 + sy * FENDER_W / 2, A.AXLE_Z) * p.part
        tabs = []                                                  # 양 끝 탭 — 옆판 바깥면에 붙는다
        for tx in [x for x in self.fender_tab_x if (x > 0) == (sx > 0)]:
            tab = Pos(tx, y0 + sy * 1.5, self.rail_slot_z) * Box(25.0, 3.0, 32.0)
            hole = Pos(tx, y0 + sy * 1.5, self.rail_slot_z) * Cylinder(HOLE / 2, 4, rotation=(90, 0, 0))
            tabs.append(tab - hole)
        out = arc
        for tb in tabs:
            out = out + tb
        out.label = f"펜더_{'앞' if sx > 0 else '뒤'}{'왼' if sy > 0 else '오른'}_ASA"
        return out

    def bumper(self, sx):
        x0 = sx * self.x_out
        b = Pos(x0 + sx * 12.5, 0, self.z0) * Box(25.0, 400.0, 30.0, align=MIN)
        for y in (-120.0, 0.0, 120.0):
            b = b - Pos(x0 + sx * 12.5, y, self.rail_slot_z) * Cylinder(HOLE / 2, 30, rotation=(0, 90, 0))
        b.label = f"범퍼_{'앞' if sx > 0 else '뒤'}_ASA"
        return b

    def louver_hood(self, sy):
        A = self.A
        cx = (A.AXLE_CROSS_X + A.CROSS_X) / 2
        top = self.z_post - 22 + VENT["w"] / 2 + 6
        h, d, l = 48.0, 14.0, VENT["l"] + 16
        with BuildPart() as p:
            Box(l, d, h, align=(Align.CENTER, Align.MIN, Align.MAX))
            with Locations((0, 0, -2.5)):                         # 속을 비우고 아래만 연다
                Box(l - 5, d - 2.5, h, align=(Align.CENTER, Align.MIN, Align.MAX), mode=Mode.SUBTRACT)
        hood = Pos(cx, sy * self.y_out, top) * (Rot(0, 0, 180) * p.part if sy < 0 else p.part)
        hood.label = f"루버덮개_{'왼' if sy > 0 else '오른'}_ASA"
        return hood

    def rubber_bumpers(self):
        """이어진 고무 띠 (앞 · 뒤 하나씩) — 판 전체 폭. 볼트 구멍은 판 구멍과 같은 자리."""
        from build123d import Axis, fillet
        out = []
        r = RUBBER
        width = self.W + 2 * ACM_T
        for sx in (-1, 1):
            with BuildPart() as b:
                Box(r["out"], width, r["h"])
                edges = b.edges().filter_by(Axis.Y).filter_by_position(Axis.X, r["out"] / 2 - 0.01, r["out"] / 2 + 0.01) \
                    if sx > 0 else b.edges().filter_by(Axis.Y).filter_by_position(Axis.X, -r["out"] / 2 - 0.01, -r["out"] / 2 + 0.01)
                fillet(edges, radius=r["r"])
                with Locations(*[(0, y, 0) for y in r["ys"]]):
                    Cylinder(r["hole"] / 2, r["out"] + 2, rotation=(0, 90, 0), mode=Mode.SUBTRACT)
            strip = Pos(sx * (self.x_out + r["out"] / 2), 0, self.rail_slot_z) * b.part
            strip.label = f"고무범퍼띠_{'앞' if sx > 0 else '뒤'}"
            out.append(strip)
        return out

    def cameras(self):
        out = []
        for sx in (-1, 1):
            c = Pos(sx * (self.L / 2 - CAM["d"] / 2), 0, CAM["z"]) * Box(CAM["d"], CAM["w"], CAM["h"])
            c.label = f"뎁스카메라_{'앞' if sx > 0 else '뒤'}_D435급"
            out.append(c)
        return out

    def visor(self, sx):
        w, o, t = VISOR["w"], VISOR["out"], VISOR["t"]
        top_z = CAM["z"] + CAM["win"][1] / 2 + 3
        x = sx * (self.x_out + o / 2)
        v = Pos(x, 0, top_z) * Box(o, w, t, align=MIN)
        for sy in (-1, 1):
            v = v + Pos(x, sy * (w / 2 - t / 2), top_z - 30) * Box(o, t, 30, align=MIN)
        v.label = f"카메라차양_{'앞' if sx > 0 else '뒤'}_ASA"
        return v

    def bay_tray(self):
        """뒤판 오목 포트 칸 — 판 바깥면에서 안쪽으로 depth. 안벽에 패널형 커넥터 구멍."""
        y, z, w, h, d, t = BAY["y"], BAY["z"], BAY["w"], BAY["h"], BAY["depth"], BAY["wall"]
        xo = -self.x_out                                            # 뒤판 바깥면 x
        tray = Pos(xo + d / 2, y, z) * Box(d, w, h)
        tray = tray - Pos(xo + (d - t) / 2 - 0.01, y, z) * Box(d - t + 0.02, w - 2 * t, h - 2 * t)
        f = BAY["flange"]
        flange = Pos(xo - 1.25, y, z) * Box(2.5, w + 2 * f, h + 2 * f)
        flange = flange - Pos(xo - 1.25, y, z) * Box(2.6, w - 2 * t, h - 2 * t)
        tray = tray + flange
        xb = xo + d - t / 2                                         # 안벽 가운데 x
        for py, pz, _, shape in BAY_PORTS:
            if shape[0] == "o":
                tray = tray - Pos(xb, py, pz) * Cylinder(shape[1] / 2, t + 2, rotation=(0, 90, 0))
            else:
                tray = tray - Pos(xb, py, pz) * Box(t + 2, shape[1], shape[2])
        tray = tray - Pos(xo + 6, y, z - h / 2 + t / 2) * Box(4, 30, t + 2)   # 바닥 배수 홈
        tray.label = "포트칸_트레이_ASA"
        return tray

    def bay_cover(self):
        """윗변 경첩으로 위로 여는 덮개 (닫힌 모습). 둘레 EPDM 개스킷 · 자석/걸쇠는 모델 없음."""
        y, z, w, h, f, ct = BAY["y"], BAY["z"], BAY["w"], BAY["h"], BAY["flange"], BAY["cover_t"]
        x = -self.x_out - 2.5 - ct / 2
        c = Pos(x, y, z) * Box(ct, w + 2 * f, h + 2 * f)
        c = c + Pos(x, y, z + h / 2 + f) * Cylinder(3.0, w + 2 * f, rotation=(90, 0, 0))   # 경첩 너클
        c.label = "포트칸_덮개_ASA"
        return c

    def estop(self):
        e = ESTOP
        xo = -self.x_out
        head = Pos(xo - e["head_l"] / 2, e["y"], e["z"]) * Cylinder(e["head_d"] / 2, e["head_l"],
                                                                     rotation=(0, 90, 0))
        body = Pos(-self.L / 2 + e["body_l"] / 2, e["y"], e["z"]) * Box(e["body_l"], e["body"], e["body"])
        out = head + body
        out.label = "비상정지_KSEP_22파이"
        return out

    def fans(self):
        out = []
        for fy, fz in FANS:
            f = Pos(-self.L / 2 + 12.5, fy, fz) * Box(25.0, 80.0, 80.0)
            f = f - Pos(-self.L / 2 + 12.5, fy, fz) * Cylinder(36.0, 26.0, rotation=(0, 90, 0))
            f.label = f"배기팬_80mm_{'왼' if fy > 0 else '오른'}"
            out.append(f)
        return out

    def panels(self):
        return [self.panel(k) for k in ("side_l", "side_r", "front", "rear")]        # 윗판 없음 (v11)

    def printed(self):
        # v12: 펜더 4 개를 뺐다 (사용자). fender() 는 남겨 둔다 — 다시 달 때 쓴다.
        # 옆판의 펜더 탭 자리 구멍 (x=±118.2 · ±301.8, 레일 슬롯) 은 옆판 → 레일 고정으로 그대로 쓴다.
        # v14: ASA 긴 범퍼 → 고무 범퍼 (rubber_bumpers). 카메라 차양 · 포트 칸 트레이 · 덮개 추가.
        out = [self.louver_hood(sy) for sy in (-1, 1)]
        # v15: 카메라 차양 뺌 (visor() 는 남겨 둔다)
        out += [self.bay_tray(), self.bay_cover()]
        return out


# ---------------------------------------------------------------- 내보내기
def export(sh, out_dir, md, version):
    from build123d import ExportDXF, Unit
    from build123d.exporters import ColorIndex, LineType
    rows = [f"# 외관판 재단 규격 ({version})", "",
            "| 판 | 재료 | 크기 (u × v) | 구멍 | 파일 |", "|---|---|---|---|---|"]
    for kind, mat in (("side_l", "ACM 3t"), ("side_r", "ACM 3t"), ("front", "UV PC 3t 불투명"),
                      ("rear", "UV PC 3t 불투명")):
        U, Vh, holes, slots, circles = sh.features(kind)
        rects = sh._rects
        ex = ExportDXF(unit=Unit.MM, line_weight=0.25)
        ex.add_layer("CUT", color=ColorIndex.BLACK, line_type=LineType.CONTINUOUS)
        ex.add_shape(sh.sketch(kind).faces()[0], layer="CUT")
        fn = f"shell_{kind}.dxf"
        ex.write(str(out_dir / fn))
        rows.append(f"| {kind} | {mat} | {U:.0f} × {Vh:.1f} | Ø{HOLE} ×{sum(1 for h in holes if h[2] == HOLE)}"
                    + (f", 기타 {len(holes) + len(circles) - sum(1 for h in holes if h[2] == HOLE)}" if kind in ('rear', 'lid') else "")
                    + (f", 루버 {len(slots)}" if slots else "")
                    + (f", 사각 구멍 {len(rects)} (카메라 창 · 포트 칸)" if rects else "") + f" | `output/{fn}` |")
    rows += ["", "- 고정: 프로파일 바깥 슬롯 M5 T 너트. 위쪽 구멍 (z=281.85) 은 상부 프레임을 올린 뒤 쓴다.",
             "- 윗판 없음 — 상부 프레임 위 상부판으로 (비상정지 · 안테나 구멍도 그쪽).",
             "- 이음새: EPDM 개스킷 테이프. 쓰지 않는 T 슬롯은 커버로 막는다 (물길).",
             "- PC 는 UV 코팅 **불투명** (흑색 · 회색), CNC 재단. ACM 은 실외용 (PE 코어면 충분). 반투명 · 투명은 쓰지 않는다.",
             "- 범퍼 2 · 루버 덮개 2 는 ASA 출력 (PETG · PLA 는 햇볕에 변형). 펜더는 v12 에서 뺐다.", ""]
    md.write_text("\n".join(rows), encoding="utf-8", newline="\n")
