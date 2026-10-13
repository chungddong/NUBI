r"""하부 프레임 위에 얹는 베이 바닥판 (아크릴, 레이저 재단용).

    LOW_SHELF=1 TIE=0 .venv/Scripts/python.exe models/hoverboard_mount/floor_plate.py

무엇을 만드나
  · 하부 프레임 760x450 외곽 그대로. 레일·크로스 윗면에 얹히고 기둥 6개 자리는 노치.
  · 케이블 그로밋 4 — **혀(tongue) 옆면 케이블 출구에서 유도**한다. 출구는 차량 안쪽을
    향하고(x=±195 → 중심 쪽), 케이블이 R 로 꺾여 올라오는 자리에 구멍을 둔다.
  · 고정 볼트 M6 — 크로스 5본 윗면 슬롯 (x=0, ±265, ±365) 에 스프링너트.
  · 장비 고정용 M5 격자 (50mm). 노치·그로밋·볼트 근처는 뺀다.

검증 (verify)
  · 프레임 빔 6+5+2 본, 기둥, H 크로스, 클램프와 간섭 0
  · 판을 0.5 내리면 레일·크로스와 겹친다 (= 실제로 얹혀 있다)
  · 케이블 경로(출구 → 그로밋) 가 프레임·클램프와 안 부딪힌다

내보내기
  output/dxf/floor_plate.dxf          레이저 재단 (외곽 + 구멍, 1:1 mm)
  output/step/floor_plate.step
  output/drawing/floor_plate.png      치수 도면 + 구멍표
  docs/floor_plate_cut.md             재단 규격서
"""

import math
import os
import sys
from pathlib import Path

HERE = Path(__file__).parent
sys.path[:0] = [str(HERE), str(HERE.parent)]

import matplotlib                                                    # noqa: E402
matplotlib.use("Agg")
import matplotlib.pyplot as plt                                      # noqa: E402
from matplotlib import font_manager, patches                         # noqa: E402

from build123d import *                                              # noqa: E402
from build123d.exporters import ColorIndex, LineType                  # noqa: E402
from build123d.exporters3d import export_step                        # noqa: E402

import parameters as P                                               # noqa: E402
import low_clamp as LC                                               # noqa: E402
import vehicle_integration as VI                                     # noqa: E402

for _f in ("Malgun Gothic", "맑은 고딕", "NanumGothic", "Gulim"):
    if any(f.name == _f for f in font_manager.fontManager.ttflist):
        plt.rcParams["font.family"] = _f
        break
plt.rcParams["axes.unicode_minus"] = False

OUT = HERE / "output"
DOCS = HERE.parent.parent / "docs"

# ---------------------------------------------------------------- 파라미터
PLATE_T = 5.0                    # 아크릴 두께. 3t 는 235 스팬에 배터리 12kg 이면 18mm 처진다 (5t: 2.7mm)
PLATE_L, PLATE_W = VI.BASE_L, VI.BASE_W          # 760 x 450 — 프레임 외곽 그대로
S = VI.S                                         # 30
POST_XS = (-VI.CROSS_X, 0.0, VI.CROSS_X)         # 기둥 x (레일 y=±210 위)
NOTCH_FIT = 1.0                                  # 기둥 노치 한쪽 여유
CROSS_XS = (-VI.CROSS_X, -(VI.AXLE_X + 55), 0.0, VI.AXLE_X + 55, VI.CROSS_X)   # 하부 크로스
HCROSS_XS = (-VI.AXLE_X, VI.AXLE_X) if P.LOW_SHELF else ()
MOUNT_D = 6.5                                    # M6 볼트 통과
MOUNT_Y = 110.0                                  # 크로스 위 볼트 y (±)
GROMMET_D = 25.0                                 # 고무 그로밋 외경 (케이블 Ø10 용)
CABLE_BEND_IN = 30.0                             # 출구에서 안쪽으로 이만큼 가서 올라온다
GRID_PITCH, GRID_D = 50.0, 5.5                   # 장비 고정 M5 격자
# FLOOR_GRID=0 → 격자 없이 꼭 필요한 구멍 18개만 (볼트 14 + 그로밋 4). 업체 견적용 최소판.
# 가공비가 절단 길이로 매겨지므로 격자 77개(1.3 m)가 빠지면 견적이 눈에 띄게 준다.
GRID_ON = os.environ.get("FLOOR_GRID", "1") != "0"
SUFFIX = "" if GRID_ON else "_min"
GRID_KEEP_OUT = 22.0                             # 볼트·그로밋 중심에서 이 안의 격자 구멍은 뺀다
CORNER_R = 3.0                                   # 레이저 외곽 코너

Z0 = VI.FRAME_TOP                                # 판 밑면 = 레일 윗면 (123.85)

# 케이블 그로밋 — 혀 출구에서 유도 (차량 좌표)
if P.LOW_SHELF and P.SHELF_KIND == "tongue" and P.SHELF_CABLE:
    _exit_x = VI.AXLE_X - LC.SHELF_W / 2                             # 혀 옆면 x (안쪽)
    _exit_y = VI.WHEEL_Y + LC.CABLE_Y_EXIT                           # 로컬 y → 차량 |y|
    GROMMETS = [(sx * (_exit_x - CABLE_BEND_IN), sy * _exit_y) for sx in (-1, 1) for sy in (-1, 1)]
    CABLE_EXIT_Z = VI.AXLE_Z
else:
    GROMMETS, CABLE_EXIT_Z = [], None


def mount_holes():
    pts = [(x, sy * MOUNT_Y) for x in CROSS_XS for sy in (-1, 1)]
    pts += [(x, sy * MOUNT_Y) for x in HCROSS_XS for sy in (-1, 1)]
    return pts


def notches():
    """(중심 x, 중심 y, 폭 x, 폭 y). 기둥 30x30 + 여유. 모서리 4 + 가운데 2."""
    w = S + 2 * NOTCH_FIT
    out = []
    for x in POST_XS:
        for sy in (-1, 1):
            cx = x
            cy = sy * VI.RAIL_Y
            # 판 밖으로 열리게 조금 키운다 (판 가장자리와 정확히 맞물리면 슬리버가 남는다)
            wx = w + (2.0 if abs(x) > 1 else 0.0)
            wy = w + 2.0
            cx += math.copysign(1.0, x) if abs(x) > 1 else 0.0
            cy += sy * 1.0
            out.append((cx, cy, wx, wy))
    return out


def grid_holes():
    if not GRID_ON:
        return []
    keep = mount_holes() + GROMMETS
    n = int((PLATE_L / 2 - 60) / GRID_PITCH)
    m = int((PLATE_W / 2 - 60) / GRID_PITCH)
    pts = []
    for i in range(-n, n + 1):
        for j in range(-m, m + 1):
            pt = (i * GRID_PITCH, j * GRID_PITCH)
            if any(math.dist(pt, k) < GRID_KEEP_OUT for k in keep):
                continue
            if any(abs(pt[0] - cx) < wx / 2 + 8 and abs(pt[1] - cy) < wy / 2 + 8 for cx, cy, wx, wy in notches()):
                continue
            pts.append(pt)
    return pts


def plate_sketch():
    with BuildSketch() as sk:
        RectangleRounded(PLATE_L, PLATE_W, CORNER_R)
        for cx, cy, wx, wy in notches():
            with Locations((cx, cy)):
                Rectangle(wx, wy, mode=Mode.SUBTRACT)
        with Locations(*mount_holes()):
            Circle(MOUNT_D / 2, mode=Mode.SUBTRACT)
        if GROMMETS:
            with Locations(*GROMMETS):
                Circle(GROMMET_D / 2, mode=Mode.SUBTRACT)
        g = grid_holes()
        if g:
            with Locations(*g):
                Circle(GRID_D / 2, mode=Mode.SUBTRACT)
    return sk.sketch


def plate():
    with BuildPart() as p:
        add(plate_sketch())
        extrude(amount=PLATE_T)
    part = Pos(0, 0, Z0) * p.part
    part.label = "베이바닥판_아크릴"
    return part


def cable_path_solids():
    """혀 출구 → 안쪽으로 → 위로 → 그로밋. 간섭 검사용 Ø16 튜브 2토막."""
    if not GROMMETS:
        return []
    d = P.SHELF_CABLE_D
    out = []
    for (gx, gy) in GROMMETS:
        sx = 1 if gx > 0 else -1
        x_exit = sx * (VI.AXLE_X - LC.SHELF_W / 2)
        run = abs(x_exit - gx)
        with BuildPart() as h:                                        # 수평 토막
            with Locations(((x_exit + gx) / 2, gy, CABLE_EXIT_Z)):
                Cylinder(d / 2, run, rotation=(0, 90, 0))
        with BuildPart() as v:                                        # 수직 토막 (그로밋까지)
            with Locations((gx, gy, (CABLE_EXIT_Z + Z0 + PLATE_T) / 2)):
                Cylinder(d / 2, Z0 + PLATE_T - CABLE_EXIT_Z)
        out += [h.part, v.part]
    return out


def verify(pl):
    bad = []
    beams = VI.vehicle_frame()
    hx = []
    for x in (-VI.AXLE_X, VI.AXLE_X):
        for c0 in LC.h_cross(VI.RAIL_Y):
            hx.append(Pos(x, 0, VI.AXLE_Z) * c0.moved(Location()))
    motors, lowers, uppers, bolts = VI.mounted_wheels()
    obstacles = [(b.label, b) for b in beams] + [("H크로스", c) for c in hx] \
        + [(lo.label, lo) for lo in lowers] + [(up.label, up) for up in uppers]
    for name, o in obstacles:
        x = pl & o
        if x is not None and x.volume > 1.0:
            bad.append(f"간섭 판 x {name}: {x.volume:.0f} mm3")
    # 얹혀 있나 — 0.5 내린 판이 레일·크로스와 겹쳐야 한다
    low = Pos(0, 0, -0.5) * pl.moved(Location())
    rest = sum((x.volume if (x := low & b) is not None else 0.0) for b in beams + hx)
    if rest < 100.0:
        bad.append(f"판이 프레임 위에 안 얹힌다 (겹침 {rest:.0f} mm3)")
    # 케이블 경로
    for i, tube in enumerate(cable_path_solids()):
        for name, o in obstacles:
            x = tube & o
            if x is not None and x.volume > 1.0:
                bad.append(f"케이블 경로 {i} x {name}: {x.volume:.0f} mm3")
        x = tube & pl
        if x is not None and x.volume > 1.0 and i % 2 == 0:
            bad.append(f"케이블 수평 토막 {i} 이 판을 뚫는다")
    return bad, rest


# ---------------------------------------------------------------- 도면
def drawing(png):
    fig, ax = plt.subplots(figsize=(15, 9.5), dpi=150)
    ax.set_aspect("equal")
    L2, W2 = PLATE_L / 2, PLATE_W / 2
    # 외곽 (노치 포함) — 스케치 면 외곽 와이어를 그대로 그린다
    sk = plate_sketch()
    face = sk.faces()[0]
    for w in [face.outer_wire()] + face.inner_wires():
        pts = []
        for e in w.edges():
            for t in (0.0, 0.25, 0.5, 0.75):
                v = e.position_at(t); pts.append((v.X, v.Y))
        v = w.edges()[-1].position_at(1.0); pts.append((v.X, v.Y))
        xs, ys = zip(*pts)
        # 원(구멍)은 위 샘플링이 거칠다 — 반지름으로 다시 그린다
        if len(w.edges()) == 1 and w.edges()[0].geom_type.name == "CIRCLE":
            c = w.edges()[0].arc_center; r = w.edges()[0].radius
            ax.add_patch(patches.Circle((c.X, c.Y), r, fill=False, lw=0.7, color="k"))
        else:
            ax.plot(list(xs) + [xs[0]], list(ys) + [ys[0]], "k-", lw=1.2)
    # 프레임 빔 자리 (참고, 회색)
    for x in CROSS_XS:
        ax.add_patch(patches.Rectangle((x - S / 2, -W2), S, PLATE_W, fill=True, fc="#e8e8e8", ec="none", zorder=0))
    for x in HCROSS_XS:
        ax.add_patch(patches.Rectangle((x - S / 2, -W2), S, PLATE_W, fill=True, fc="#dbe6f2", ec="none", zorder=0))
    for sy in (-1, 1):
        ax.add_patch(patches.Rectangle((-L2, sy * VI.RAIL_Y - S / 2), PLATE_L, S, fill=True, fc="#e8e8e8", ec="none", zorder=0))
    # 구멍 표시
    for (x, y) in mount_holes():
        ax.plot(x, y, "+", color="#1565c0", ms=9, mew=1.2)
    for (x, y) in GROMMETS:
        ax.plot(x, y, "x", color="#c62828", ms=9, mew=1.2)
        ax.annotate(f"그로밋 Ø{GROMMET_D:.0f}\n({x:+.0f}, {y:+.0f})", (x, y),
                    xytext=(x + (28 if x > 0 else -28), y + (30 if y > 0 else -30) * -1),
                    fontsize=8, ha="center", color="#c62828",
                    arrowprops=dict(arrowstyle="-", color="#c62828", lw=0.6))
    # 치수
    dim = dict(color="#1b5e20", lw=0.8)
    ax.annotate("", (L2, -W2 - 28), (-L2, -W2 - 28), arrowprops=dict(arrowstyle="<->", **dim))
    ax.text(0, -W2 - 40, f"{PLATE_L:.0f}", ha="center", va="top", fontsize=11, color=dim["color"])
    ax.annotate("", (L2 + 28, W2), (L2 + 28, -W2), arrowprops=dict(arrowstyle="<->", **dim))
    ax.text(L2 + 40, 0, f"{PLATE_W:.0f}", ha="left", va="center", fontsize=11, color=dim["color"], rotation=90)
    for x in CROSS_XS + tuple(HCROSS_XS):
        ax.text(x, W2 + 14, f"x={x:+.0f}", ha="center", fontsize=8, color="#555")
    nw = S + 2 * NOTCH_FIT
    ax.text(-L2 + 40, W2 - 12, f"기둥 노치 {nw:.0f}x{nw:.0f} (6곳)", fontsize=8, color="#555")
    ax.text(0, W2 + 34, f"베이 바닥판 — 아크릴 {PLATE_T:.0f}t, {PLATE_L:.0f}x{PLATE_W:.0f}, "
            f"M6 볼트 {len(mount_holes())} (+) · 그로밋 {len(GROMMETS)} (×) · M5 격자 {len(grid_holes())} (Ø{GRID_D})",
            ha="center", fontsize=12)
    ax.set_xlim(-L2 - 70, L2 + 90); ax.set_ylim(-W2 - 60, W2 + 50)
    ax.set_xlabel("x (차량 앞 +)  [mm]"); ax.set_ylabel("y  [mm]")
    ax.grid(True, lw=0.3, alpha=0.4)
    fig.tight_layout()
    fig.savefig(png, facecolor="white")
    plt.close(fig)


def cut_spec(md, rest_mm3):
    L = []
    A = L.append
    A("# 베이 바닥판 재단 규격 (아크릴)")
    A("")
    A(f"- 재질: 투명/불투명 아크릴 **{PLATE_T:.0f}t** (PC 5t 도 가능 — 볼트 구멍 균열에 더 강하다)")
    A(f"- 외곽: **{PLATE_L:.0f} x {PLATE_W:.0f}**, 코너 R{CORNER_R:.0f}")
    A(f"- 수량: 1")
    A(f"- 파일: `models/hoverboard_mount/output/dxf/floor_plate.dxf` (1:1 mm, 외곽 + 구멍 전부)")
    A("")
    A("## 노치 (기둥 자리, 판 가장자리에서 파냄)")
    A("")
    A("| 위치 | 크기 (x × y) | 설명 |")
    A("|---|---|---|")
    nw = S + 2 * NOTCH_FIT
    for x in POST_XS:
        for sy in (-1, 1):
            kind = "모서리" if abs(x) > 1 else "긴 변 가운데"
            A(f"| x={x:+.0f}, y={sy * VI.RAIL_Y:+.0f} | {nw:.0f} × {nw:.0f} | {kind} 기둥 3030 + 여유 {NOTCH_FIT:.0f}mm |")
    A("")
    A("## 구멍")
    A("")
    A("| 종류 | Ø | 수량 | 위치 (x, y) | 용도 |")
    A("|---|---:|---:|---|---|")
    mh = mount_holes()
    A(f"| 고정 볼트 | {MOUNT_D} | {len(mh)} | " + ", ".join(f"({x:+.0f},{y:+.0f})" for x, y in mh)
      + " | M6 둥근머리 + 스프링너트, 크로스 윗면 슬롯 |")
    if GROMMETS:
        A(f"| 케이블 그로밋 | {GROMMET_D:.0f} | {len(GROMMETS)} | " + ", ".join(f"({x:+.0f},{y:+.0f})" for x, y in GROMMETS)
          + f" | 허브모터 케이블 Ø10 — 혀 출구(x=±{VI.AXLE_X - LC.SHELF_W / 2:.0f}, y=±{abs(GROMMETS[0][1]):.0f}, z={CABLE_EXIT_Z:.0f}) 에서 안쪽으로 {CABLE_BEND_IN:.0f} 간 자리 |")
    g = grid_holes()
    A(f"| 장비 격자 | {GRID_D} | {len(g)} | {GRID_PITCH:.0f}mm 격자 (볼트·그로밋·노치 근처 제외) | M5 장비 고정 (없어도 됨) |")
    A("")
    per = 2 * (PLATE_L + PLATE_W) + 6 * 2 * (S + 2 * NOTCH_FIT)
    holes = sum(math.pi * d * n for d, n in ((MOUNT_D, len(mh)), (GROMMET_D, len(GROMMETS)), (GRID_D, len(g))))
    A("## 절단 길이 (견적 근거)")
    A("")
    A(f"- 외곽 + 노치 약 {per/1000:.2f} m, 구멍 {len(mh) + len(GROMMETS) + len(g)}개 약 {holes/1000:.2f} m → **합계 약 {(per + holes)/1000:.1f} m**")
    A("")
    A("## 얹히는 곳 / 검증")
    A("")
    A(f"- 판 밑면 z={Z0:.2f} (레일 윗면). 레일 2 + 크로스 5" + (" + H 크로스 2" if HCROSS_XS else "") +
      f" 윗면에 얹힌다 (0.5mm 내린 검사에서 겹침 {rest_mm3:.0f} mm³).")
    A("- 프레임 빔·기둥·H 크로스·클램프와 간섭 0. 케이블 경로(출구 → 그로밋) 간섭 0.")
    A("")
    A("## 주의")
    A("")
    A("- 아크릴은 볼트를 세게 조이면 구멍에서 갈라진다 — **와셔 필수, 손으로 꽉 정도.**")
    A("- 레이저 커프 0.2mm 는 업체가 보정한다. 구멍 지름은 도면값 그대로 주문.")
    A(f"- 3t 로 하면 크로스 사이 235 스팬에 배터리(12kg) 가 올라갈 때 약 18mm 처진다. {PLATE_T:.0f}t 는 2.7mm.")
    A("  배터리 브라켓은 크로스(x=-265) 위에 오게 두는 게 좋다 (지금 모델은 x=-140).")
    A("")
    md.write_text("\n".join(L), encoding="utf-8", newline="\n")


def main():
    pl = plate()
    bad, rest = verify(pl)
    (OUT / "dxf").mkdir(parents=True, exist_ok=True)
    (OUT / "step").mkdir(parents=True, exist_ok=True)
    (OUT / "drawing").mkdir(parents=True, exist_ok=True)

    ex = ExportDXF(unit=Unit.MM, line_weight=0.25)
    ex.add_layer("CUT", color=ColorIndex.BLACK, line_type=LineType.CONTINUOUS)
    ex.add_shape(plate_sketch().faces()[0], layer="CUT")
    ex.write(str(OUT / "dxf" / f"floor_plate{SUFFIX}.dxf"))
    export_step(pl, str(OUT / "step" / f"floor_plate{SUFFIX}.step"))
    drawing(OUT / "drawing" / f"floor_plate{SUFFIX}.png")
    cut_spec(DOCS / f"floor_plate_cut{SUFFIX}.md", rest)

    bb = pl.bounding_box()
    print(f"FLOOR_PLATE {bb.size.X:.1f}x{bb.size.Y:.1f}x{bb.size.Z:.1f} vol={pl.volume/1000:.0f}cm3 "
          f"mount={len(mount_holes())} grommet={len(GROMMETS)} grid={len(grid_holes())} rest={rest:.0f}mm3")
    for g in GROMMETS:
        print(f"  그로밋 ({g[0]:+.1f}, {g[1]:+.1f})")
    print("verify:", bad or "OK")
    if bad:
        sys.exit(1)
    if not os.environ.get("NOSHOW"):
        show_scene(pl)


def show_scene(pl):
    """프레임 + H 크로스 + 클램프 + 판 을 뷰어에 (판은 반투명)."""
    from build123d import Color, Compound
    from ocp_vscode import Camera, set_port, show
    motors, lowers, uppers, bolts = VI.mounted_wheels()
    beams = VI.vehicle_frame()
    hx = []
    for x in (-VI.AXLE_X, VI.AXLE_X):
        for c0 in LC.h_cross(VI.RAIL_Y):
            hx.append(Pos(x, 0, VI.AXLE_Z) * c0.moved(Location()))
    tubes = cable_path_solids()
    groups = [VI._group(beams, "프레임_3030", "#69727d"),
              VI._group(lowers, "하부클램프_4개", "#c9ced4"),
              VI._group(uppers, "상부클램프_4개", "#f5a623"),
              VI._group(motors, "휠모터_4개", "#34383d")]
    names = [g.label for g in groups]; colors = [g.color for g in groups]; alphas = [1, 1, 1, 1]
    if hx:
        groups.append(VI._group(hx, "H크로스_2개", "#4f7fb0")); names.append("H크로스_2개"); colors.append("#4f7fb0"); alphas.append(1)
    if tubes:
        groups.append(VI._group(tubes, "케이블_경로_Ø16", "#c62828")); names.append("케이블_경로_Ø16"); colors.append("#c62828"); alphas.append(1)
    pl.color = Color("#7fc8f8")
    groups.append(pl); names.append(f"베이바닥판_아크릴{PLATE_T:.0f}t"); colors.append("#7fc8f8"); alphas.append(0.45)
    set_port(3939)
    show(*groups, names=names, colors=colors, alphas=alphas, progress=None, reset_camera=Camera.RESET)
    print("FLOOR_PLATE_SCENE_SENT")


if __name__ == "__main__":
    main()
