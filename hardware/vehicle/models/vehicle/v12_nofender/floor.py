r"""바닥판 — 하부 프레임 위에 얹는 판 (레이저 / CNC 재단).

hoverboard_mount/floor_plate.py (2026-09-17, 옛 프레임용) 를 지금 프레임에 맞춰 옮겼다.
  · 외곽 = 하부 프레임 760 x 450. 레일 · 크로스 윗면에 얹힌다 (밑면 z = 레일 윗면).
  · 기둥 노치는 **크로스 5 곳 전부** (x=0 · ±210 · ±365, y=±210) — 기둥 6 본이든 10 본이든
    같은 판을 쓴다 (v06). 기둥이 없는 노치는 그냥 비어 있다.
  · 고정 볼트 M6 — 크로스 5 본 (x=0 · ±210 · ±365) 윗면 슬롯 y=±110 + 사이드 레일 윗면
    슬롯 (기둥 사이 x=±105 · ±290). 스프링너트.
  · 케이블 구멍 4 (v09: 방수 글랜드 M20) — T 클램프 혀 옆면 케이블 출구에서 유도한다. 출구는 차량 안쪽 (x=±195)
    을 향하고, 케이블이 안쪽으로 30 간 뒤 올라오는 자리에 구멍을 둔다.
  · 장비 고정 M5 격자 (50 mm). 볼트 · 그로밋 · 노치 근처는 뺀다.

재료는 MATERIAL 하나로 바꾼다 — 두께 · 볼트 길이 · 무게가 따라온다.
"""

import math

from build123d import (Box, BuildPart, BuildSketch, Circle, Cylinder, Locations, Mode,
                       Pos, Rectangle, RectangleRounded, Rot, SlotOverall, add, extrude)

import low_clamp as LC
import parameters as P

# ---------------------------------------------------------------- 재료
# (이름, 두께 t, 밀도 g/cm³, 굽힘 탄성계수 GPa, 메모)
MATERIALS = {
    "acrylic": ("아크릴 (PMMA)", 5.0, 1.19, 3.2, "레이저 깔끔 · 투명 · 싸다 / 충격 · 볼트 조임에 갈라진다"),
    "pc": ("폴리카보네이트 (PC)", 5.0, 1.20, 2.4, "충격에 아크릴의 수십 배 / 레이저는 그을린다 → CNC 재단"),
    "al5052": ("알루미늄 5052", 3.0, 2.68, 70.0, "제일 단단 · 볼트 구멍 튼튼 / 무겁고 전도체 (절연)"),
    "plywood": ("자작합판", 9.0, 0.68, 9.0, "가볍고 단단 · 싸다 · 레이저 가능 / 습기 · 모서리 뜯김"),
}
MATERIAL = "pc"                        # v09 실외 주행 — PC 5t (진동 · 충격 · 추위에 안 깨지고 싸다)
MAT_NAME, PLATE_T, DENSITY, E_GPA, MAT_NOTE = MATERIALS[MATERIAL]

# ---------------------------------------------------------------- 치수 (A = assembly 모듈)
NOTCH_FIT = 1.0                        # 기둥 노치 한쪽 여유
NOTCH_R = 3.0                          # 노치 안쪽 모서리 R — 아크릴은 직각 모서리에서 금이 시작된다 (v07)
MOUNT_D = 6.5                          # M6 통과
MOUNT_Y = 110.0                        # 크로스 위 볼트 y (±)
RAIL_MOUNT_X = (105.0, 290.0)          # 사이드 레일 위 볼트 x (±) — 기둥 (0 · ±365) 사이
WASHER_T = 1.6                         # M6 평와셔 — 아크릴은 와셔 필수
BOLT_LENGTHS = (10, 12, 15, 16, 20)    # 흔한 M6 둥근머리 길이
GROMMET_D = 20.5                       # v09: 방수 케이블 글랜드 M20 (케이블 Ø6~12) 구멍. 실내판은 그로밋 Ø25
CABLE_BEND_IN = 30.0                   # 혀 출구에서 안쪽으로 이만큼 가서 올라온다
GRID_ON = False                        # v09: 실외판은 격자를 안 뚫는다 — 구멍이 곧 물길. 부품 고정 구멍만
GRID_PITCH, GRID_D = 50.0, 5.5         # 장비 고정 M5 격자 (GRID_ON 일 때)
GRID_KEEP_OUT = 22.0                   # 볼트 · 그로밋 중심에서 이 안의 격자 구멍은 뺀다
CORNER_R = 3.0                         # 외곽 코너


class Floor:
    """assembly 모듈 A 의 프레임 치수로 바닥판을 짓는다."""

    def __init__(self, A):
        self.A = A
        self.L, self.W, self.S = A.BASE_L, A.BASE_W, A.S
        self.z0 = A.FRAME_TOP                                   # 판 밑면 = 레일 윗면
        self.cross_xs = (-A.CROSS_X, -A.AXLE_CROSS_X, 0.0, A.AXLE_CROSS_X, A.CROSS_X)
        self.post_xs = tuple(A.POST_X)                         # 지금 서 있는 기둥
        self.notch_xs = self.cross_xs                           # 노치 = 기둥이 설 수 있는 자리 전부
        exit_x = A.AXLE_X - LC.SHELF_W / 2                      # 혀 옆면 (차량 안쪽)
        exit_y = A.WHEEL_Y + LC.CABLE_Y_EXIT
        self.cable_exit = [(sx * exit_x, sy * exit_y) for sx in (-1, 1) for sy in (-1, 1)]
        self.grommets = [(sx * (exit_x - CABLE_BEND_IN), sy * exit_y)
                         for sx in (-1, 1) for sy in (-1, 1)]
        self.cable_z = A.AXLE_Z                                 # 출구 높이 = 축 중심

    # ---- 구멍 배치
    def mount_holes(self):
        pts = [(x, sy * MOUNT_Y) for x in self.cross_xs for sy in (-1, 1)]
        pts += [(sx * x, sy * self.A.RAIL_Y) for x in RAIL_MOUNT_X for sx in (-1, 1)
                for sy in (-1, 1)]
        return pts

    def notches(self):
        """(중심 x, 중심 y, 폭 x, 폭 y). 판 밖으로 2 mm 더 열어 가장자리 슬리버를 없앤다."""
        w = self.S + 2 * NOTCH_FIT
        out = []
        for x in self.notch_xs:
            for sy in (-1, 1):
                end = abs(x) > 1
                out.append((x + (math.copysign(1.0, x) if end else 0.0), sy * (self.A.RAIL_Y + 1.0),
                            w + (2.0 if end else 0.0), w + 2.0))
        return out

    def grid_holes(self):
        if not GRID_ON:
            return []
        keep = self.mount_holes() + self.grommets
        n, m = int((self.L / 2 - 60) / GRID_PITCH), int((self.W / 2 - 60) / GRID_PITCH)
        pts = []
        for i in range(-n, n + 1):
            for j in range(-m, m + 1):
                pt = (i * GRID_PITCH, j * GRID_PITCH)
                if any(math.dist(pt, k) < GRID_KEEP_OUT for k in keep):
                    continue
                if any(abs(pt[0] - cx) < wx / 2 + 8 and abs(pt[1] - cy) < wy / 2 + 8
                       for cx, cy, wx, wy in self.notches()):
                    continue
                # 프레임 빔 바로 위 구멍은 볼트가 못 지나간다 (빔 윗면에 막힘) — 뺀다
                if any(abs(pt[0] - x) < self.S / 2 + GRID_D for x in self.cross_xs):
                    continue
                if abs(abs(pt[1]) - self.A.RAIL_Y) < self.S / 2 + GRID_D:
                    continue
                pts.append(pt)
        return pts

    def part_holes(self):
        import parts as PT
        return PT.plate_holes()

    def slots(self):
        import parts as PT
        return PT.strap_slots()

    def beam_bands(self):
        """판 밑 프레임 빔이 차지하는 띠 (x 띠 · y 띠) — 이 위 구멍은 너트가 못 들어간다."""
        xs = [(x - self.S / 2, x + self.S / 2) for x in self.cross_xs]
        ys = [(sy * self.A.RAIL_Y - self.S / 2, sy * self.A.RAIL_Y + self.S / 2) for sy in (-1, 1)]
        return xs, ys

    # ---- 형상
    def sketch(self):
        with BuildSketch() as sk:
            RectangleRounded(self.L, self.W, CORNER_R)
            for cx, cy, wx, wy in self.notches():
                with Locations((cx, cy)):
                    # 둥근 사각형으로 판다 → 판 안쪽 모서리가 R3 오목이 된다. 판 밖으로 3 mm
                    # 더 연 바깥 모서리는 둥글림이 판 밖에서 끝나서 가장자리에 슬리버가 안 남는다.
                    RectangleRounded(wx, wy, NOTCH_R, mode=Mode.SUBTRACT)
            with Locations(*self.mount_holes()):
                Circle(MOUNT_D / 2, mode=Mode.SUBTRACT)
            with Locations(*self.grommets):
                Circle(GROMMET_D / 2, mode=Mode.SUBTRACT)
            for x, y, d, _, _ in self.part_holes():               # 부품 고정 구멍 (v09)
                with Locations((x, y)):
                    Circle(d / 2, mode=Mode.SUBTRACT)
            for x, y, l, w in self.slots():                       # 배터리 밴드 슬롯
                with Locations((x, y)):
                    SlotOverall(l, w, mode=Mode.SUBTRACT)
            with Locations(*self.grid_holes()):
                Circle(GRID_D / 2, mode=Mode.SUBTRACT)
        return sk.sketch

    def plate(self):
        with BuildPart() as p:
            add(self.sketch())
            extrude(amount=PLATE_T)
        part = Pos(0, 0, self.z0) * p.part
        part.label = "바닥판"
        return part

    def bolt_len(self):
        """판 + 와셔 + 슬롯 립 + 최소 물림 이상, 립 아래 캐비티 (중심 보스까지) 이하인 길이."""
        grip = PLATE_T + WASHER_T
        need = grip + P.EXTRUSION_WALL + P.LOW_MIN_ENGAGE
        cavity = self.S / 2 - P.EXTRUSION_WALL - P.EXTRUSION_CORE / 2
        ok = [L for L in BOLT_LENGTHS if need - 1e-6 <= L <= grip + P.EXTRUSION_WALL + cavity + 1e-6]
        return (ok[0] if ok else None), need, grip + P.EXTRUSION_WALL + cavity

    def bolts(self):
        L, _, _ = self.bolt_len()
        L = L or 15
        top = self.z0 + PLATE_T + WASHER_T                       # 머리 밑면 = 와셔 윗면
        s0 = Rot(180, 0, 0) * LC.button_head_screw_reference(L)  # 머리 위, 나사가 아래로
        out = []
        for i, (x, y) in enumerate(self.mount_holes()):
            b = Pos(x, y, top) * s0
            b.label = f"판볼트{i+1}"
            out.append(b)
        return out

    def cable_paths(self):
        """혀 출구 → 안쪽으로 → 위로 → 그로밋 (Ø 케이블 통로와 같은 굵기). 간섭 검사용."""
        d = P.SHELF_CABLE_D
        out = []
        for i, ((ex, ey), (gx, gy)) in enumerate(zip(self.cable_exit, self.grommets)):
            with BuildPart() as h:                               # 수평 — 출구에서 안쪽으로
                with Locations(((ex + gx) / 2, gy, self.cable_z)):
                    Cylinder(d / 2, abs(ex - gx), rotation=(0, 90, 0))
            with BuildPart() as v:                               # 수직 — 판 윗면까지
                top = self.z0 + PLATE_T
                with Locations((gx, gy, (self.cable_z + top) / 2)):
                    Cylinder(d / 2, top - self.cable_z)
            h.part.label, v.part.label = f"케이블{i+1}_수평", f"케이블{i+1}_수직"
            out += [h.part, v.part]
        return out

    # ---- 수치
    def mass_kg(self, plate):
        return plate.volume / 1000 * DENSITY / 1000

    def spans(self):
        """크로스 사이 받침 없는 x 구간 (안쪽면 사이)."""
        xs = sorted(self.cross_xs)
        return [round((b - a) - self.S, 1) for a, b in zip(xs, xs[1:])]


# ---------------------------------------------------------------- 내보내기
def export_dxf(fl, path):
    from build123d import ExportDXF, Unit
    from build123d.exporters import ColorIndex, LineType
    ex = ExportDXF(unit=Unit.MM, line_weight=0.25)
    ex.add_layer("CUT", color=ColorIndex.BLACK, line_type=LineType.CONTINUOUS)
    ex.add_shape(fl.sketch().faces()[0], layer="CUT")
    ex.write(str(path))


def drawing(fl, png):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib import patches
    plt.rcParams["font.family"] = "Malgun Gothic"
    plt.rcParams["axes.unicode_minus"] = False
    A, S = fl.A, fl.S
    L2, W2 = fl.L / 2, fl.W / 2
    fig, ax = plt.subplots(figsize=(15, 9.5), dpi=130)
    ax.set_aspect("equal")
    for x in fl.cross_xs:                                       # 프레임 자리 (참고)
        ax.add_patch(patches.Rectangle((x - S / 2, -W2), S, fl.W, fc="#e8e8e8", ec="none", zorder=0))
    for sy in (-1, 1):
        ax.add_patch(patches.Rectangle((-L2, sy * A.RAIL_Y - S / 2), fl.L, S, fc="#e8e8e8",
                                       ec="none", zorder=0))
    face = fl.sketch().faces()[0]
    for w in [face.outer_wire()] + face.inner_wires():
        es = w.edges()
        if len(es) == 1 and es[0].geom_type.name == "CIRCLE":
            c = es[0].arc_center
            ax.add_patch(patches.Circle((c.X, c.Y), es[0].radius, fill=False, lw=0.7, color="k"))
            continue
        pts = [(v.X, v.Y) for e in es for v in (e.position_at(t) for t in (0, .25, .5, .75))]
        xs, ys = zip(*(pts + pts[:1]))
        ax.plot(xs, ys, "k-", lw=1.2)
    for x, y in fl.mount_holes():
        ax.plot(x, y, "+", color="#1565c0", ms=9, mew=1.2)
    for x, y in fl.grommets:
        ax.plot(x, y, "x", color="#c62828", ms=9, mew=1.2)
        ax.annotate(f"글랜드 M20 (Ø{GROMMET_D:g})\n({x:+.0f}, {y:+.0f})", (x, y),
                    xytext=(x + (30 if x > 0 else -30), y - (32 if y > 0 else -32)),
                    fontsize=8, ha="center", color="#c62828",
                    arrowprops=dict(arrowstyle="-", color="#c62828", lw=0.6))
    dim = dict(color="#1b5e20", lw=0.8)
    ax.annotate("", (L2, -W2 - 28), (-L2, -W2 - 28), arrowprops=dict(arrowstyle="<->", **dim))
    ax.text(0, -W2 - 40, f"{fl.L:.0f}", ha="center", va="top", fontsize=11, color=dim["color"])
    ax.annotate("", (L2 + 28, W2), (L2 + 28, -W2), arrowprops=dict(arrowstyle="<->", **dim))
    ax.text(L2 + 40, 0, f"{fl.W:.0f}", ha="left", va="center", fontsize=11, color=dim["color"],
            rotation=90)
    for x in fl.cross_xs:
        ax.text(x, W2 + 14, f"x={x:+.0f}", ha="center", fontsize=8, color="#555")
    for x, y, d, _, _ in fl.part_holes():
        ax.plot(x, y, ".", color="#6a1b9a", ms=4)
    ax.text(0, W2 + 34, f"바닥판 — {MAT_NAME} {PLATE_T:g}t, {fl.L:.0f}x{fl.W:.0f}, "
            f"프레임 M6 {len(fl.mount_holes())} (+) · 글랜드 M20 {len(fl.grommets)} (×) · "
            f"부품 고정 {len(fl.part_holes())} (·) · 밴드 슬롯 {len(fl.slots())} · 기둥 노치 {len(fl.notches())}",
            ha="center", fontsize=12)
    ax.set_xlim(-L2 - 70, L2 + 90)
    ax.set_ylim(-W2 - 60, W2 + 50)
    ax.set_xlabel("x (차량 앞 +)  [mm]")
    ax.set_ylabel("y  [mm]")
    ax.grid(True, lw=0.3, alpha=0.4)
    fig.tight_layout()
    fig.savefig(png, facecolor="white")
    plt.close(fig)


def cut_spec(fl, plate, md, version):
    L, need, maxlen = fl.bolt_len()
    nw = fl.S + 2 * NOTCH_FIT
    mh, g = fl.mount_holes(), fl.grid_holes()
    per = 2 * (fl.L + fl.W) + len(fl.notches()) * 2 * nw
    holes = sum(math.pi * d * n for d, n in ((MOUNT_D, len(mh)), (GROMMET_D, len(fl.grommets)),
                                             (GRID_D, len(g))))
    rows = [
        f"# 바닥판 재단 규격 ({version})", "",
        f"- 재질: **{MAT_NAME} {PLATE_T:g}t** — {MAT_NOTE}",
        f"- 외곽: **{fl.L:.0f} x {fl.W:.0f}**, 코너 R{CORNER_R:g} · 수량 1 · 무게 약 {fl.mass_kg(plate):.2f} kg",
        "- 파일: `output/floor_plate.dxf` (1:1 mm, 외곽 + 구멍 전부, 레이어 CUT)", "",
        "## 노치 (기둥 자리, 판 가장자리에서 파냄)", "",
        "| 위치 | 크기 | 설명 |", "|---|---|---|",
    ]
    for x in fl.notch_xs:
        for sy in (-1, 1):
            rows.append(f"| x={x:+.0f}, y={sy * fl.A.RAIL_Y:+.0f} | {nw:g} × {nw:g} | "
                        f"{'모서리' if abs(abs(x) - fl.A.CROSS_X) < 1 else '긴 변'} 기둥 3030 + 여유 {NOTCH_FIT:g}"
                        f"{'' if x in fl.post_xs else ' (지금은 기둥 없음 — 10 본 겸용)'} |")
    rows += [
        "", "## 구멍", "",
        "| 종류 | Ø | 수량 | 위치 (x, y) | 용도 |", "|---|---:|---:|---|---|",
        f"| 고정 볼트 | {MOUNT_D} | {len(mh)} | " + ", ".join(f"({x:+.0f},{y:+.0f})" for x, y in mh)
        + f" | M6x{L} 둥근머리 + 와셔 + 스프링너트 (크로스 5 본 · 사이드 레일 윗면 슬롯) |",
        f"| 케이블 글랜드 M20 | {GROMMET_D:g} | {len(fl.grommets)} | "
        + ", ".join(f"({x:+.0f},{y:+.0f})" for x, y in fl.grommets)
        + f" | 휠모터 케이블 — T 클램프 혀 출구에서 안쪽으로 {CABLE_BEND_IN:g} 간 자리 |",
        f"| 장비 격자 | {GRID_D} | {len(g)} | {GRID_PITCH:g} mm 격자 | M5 장비 고정 (실외판은 0) |",
    ] + [f"| 부품 고정 | {d:g} | 1 | ({x:+.1f},{y:+.1f}) | {bolt} — {name} |" for x, y, d, bolt, name in fl.part_holes()] + [
        f"| 배터리 밴드 슬롯 | {l:g}×{w:g} | 1 | ({x:+.0f},{y:+.0f}) | 폭 25 밴드 |" for x, y, l, w in fl.slots()] + [
        "", "## 절단 길이 (견적 근거)", "",
        f"- 외곽 + 노치 약 {per / 1000:.2f} m, 구멍 {len(mh) + len(fl.grommets) + len(g)} 개 약 "
        f"{holes / 1000:.2f} m → **합계 약 {(per + holes) / 1000:.1f} m**",
        "", "## 볼트", "",
        f"- 필요 길이 {need:.1f} (판 {PLATE_T:g} + 와셔 {WASHER_T:g} + 립 {P.EXTRUSION_WALL:g} + 물림 "
        f"{P.LOW_MIN_ENGAGE:g}) ~ 최대 {maxlen:.1f} (끝이 프로파일 중심 보스에 닿기 전) → **M6x{L}**",
        "", "## 주의", "",
        "- 아크릴 · PC 는 볼트를 세게 조이면 구멍에서 갈라진다 — **와셔 필수, 손으로 꽉 정도.**",
        "- 실외: 관통 볼트마다 **고무 (EPDM) 와셔** 로 물을 막는다. 글랜드는 O 링 + 로크너트. 기둥 노치 틈은 실리콘.",
        "- PC 는 레이저로 자르면 그을린다 — **CNC 라우터 재단**으로 주문.",
        "- 사이드 레일 볼트 구멍은 판 가장자리에서 11.8 mm 다 (구멍 중심 → 가장자리 15). 아크릴이면 특히 살살 조인다.",
        "- 레이저 커프 0.2 mm 는 업체가 보정한다. 구멍 지름은 도면값 그대로 주문.",
        f"- 받침 없는 구간 (크로스 안쪽면 사이): {', '.join(f'{s:g}' for s in fl.spans())} mm.",
        "",
    ]
    md.write_text("\n".join(rows), encoding="utf-8", newline="\n")
