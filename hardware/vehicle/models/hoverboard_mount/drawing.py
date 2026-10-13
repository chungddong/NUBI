# 치수 도면 + DXF — Design D 하부 클램프.
#
#   .venv\Scripts\python.exe models\hoverboard_mount\drawing.py
#
# 만드는 것:
#   output/drawing/low_clamp_drawing.png / .pdf   치수 기입 3면도 + 등각 + 표제란
#   output/dxf/low_clamp_front|top|right.dxf      낱장 (은선제거 투영)
#   output/dxf/low_clamp_views.dxf                3면 배치 1장
#   output/dxf/{시리즈}_profile.dxf                레일 단면
#
# **치수는 전부 parameters.py 에서 나온다.** 손으로 적은 숫자가 하나도 없어야
# 모델을 고쳤을 때 도면이 같이 따라온다 — 예전에 도면 스크립트를 잃어버려서
# 모델이 바뀐 뒤에도 옛 치수 도면이 남아 있던 적이 있다.
#
# 도면 데이텀은 **부품 밑면 z=0, 좌우 중심 x=0, 앞뒤 중심 y=0** 이다.
# 패키지 안에서는 축 중심이 z=0 이지만, 가공할 때 잡는 면은 밑면이다.

import sys
from pathlib import Path

sys.path[:0] = [str(Path(__file__).parent), str(Path(__file__).parent.parent)]

import matplotlib                                                    # noqa: E402

matplotlib.use("Agg")

import matplotlib.pyplot as plt                                      # noqa: E402
from matplotlib import font_manager                                  # noqa: E402

from build123d import *                                              # noqa: E402
from build123d.exporters import ColorIndex, LineType                  # noqa: E402

import parameters as P                                               # noqa: E402
import low_clamp as LC                                               # noqa: E402
from extrusion_reference import profile_sketch                       # noqa: E402

OUT = Path(__file__).parent / "output"
(OUT / "drawing").mkdir(parents=True, exist_ok=True)
(OUT / "dxf").mkdir(parents=True, exist_ok=True)

for _f in ("Malgun Gothic", "맑은 고딕", "NanumGothic", "Gulim"):
    if any(f.name == _f for f in font_manager.fontManager.ttflist):
        plt.rcParams["font.family"] = _f
        break
plt.rcParams["axes.unicode_minus"] = False

# ---------------------------------------------------------------- 도면 데이텀
DZ = -LC.LOW_BOT                 # 밑면이 0 이 되도록 올린다
DY = -LC.CLAMP_CY                # 앞뒤 중심이 0 이 되도록
PART = Pos(0, DY, DZ) * LC.low_clamp()

W = P.LOW_W                      # 전체 폭 (x)
D = P.LOW_D                      # 전체 깊이 (y)
# 전고는 **형상에서 잰다.** FLANGE_TOP 으로 계산했더니 플랜지를 끈 뒤에도
# 63.90 이 찍혔다 (실제 38.90). 파생값이 아니라 bbox 가 진실이다.
H = PART.bounding_box().size.Z
AXLE_Z = DZ                      # 축 중심 높이 (= 밑면에서)
MATE_Z = LC.LOW_TOP + DZ         # 상부 클램프 맞댐면
RAIL_Z0 = LC.RAIL_BOT + DZ       # 레일 홈 바닥 (= 레일 밑면)
SIDE_Z = LC.RAIL_CZ + DZ         # 옆면 볼트 중심
GAP = P.LOW_CLAMP_GAP            # 다리 사이 통로
SLOT = 2 * LC.FLANGE_Y           # 레일 홈 폭
SEAT_R = P.AXLE_DIAMETER / 2 + P.SADDLE_SEAT_FIT
TAP_D = P.OEM_BOLT_D - 1.0       # 탭 하공
BOLT_D = P.TNUT_BOLT * 1.1       # 레일 볼트 통과공


# ---------------------------------------------------------------- 투영
def edges_2d(shape, direction, up):
    """HLR 투영 → (보이는 폴리라인, 숨은 폴리라인)."""
    vis, hid = shape.project_to_viewport(
        viewport_origin=tuple(1e4 * c for c in direction), viewport_up=up)

    def poly(es):
        out = []
        for e in es:
            n = 1 if e.geom_type == GeomType.LINE else 48
            try:
                pts = [e @ (i / n) for i in range(n + 1)]
            except Exception:
                continue
            out.append(([p.X for p in pts], [p.Y for p in pts]))
        return out

    return poly(vis), poly(hid)


def edges_raw(shape, direction, up):
    """DXF 용 — 투영된 build123d Edge 를 그대로 돌려준다."""
    vis, hid = shape.project_to_viewport(
        viewport_origin=tuple(1e4 * c for c in direction), viewport_up=up)
    return list(vis), list(hid)


# 시선 방향과 화면 위쪽. 정면은 -Y 에서 보고(바퀴 쪽에서), 우측은 +X 에서.
VIEWS = {
    "정면": ((0, -1, 0), (0, 0, 1)),     # X(가로) - Z(세로)
    "평면": ((0, 0, 1), (1, 0, 0)),      # Y(가로) - X(세로)
    "우측": ((1, 0, 0), (0, 0, 1)),      # Y(가로) - Z(세로)
    "등각": ((1, -1, 0.7), (0, 0, 1)),
}


# ---------------------------------------------------------------- 치수 그리기
AR = dict(arrowstyle="<->", color="#0b57a4", lw=0.8,
          shrinkA=0, shrinkB=0, mutation_scale=8)
EXT = dict(color="#0b57a4", lw=0.5, ls=(0, (4, 3)))
TXT = dict(color="#0b57a4", fontsize=8.5, ha="center", va="center",
           bbox=dict(fc="white", ec="none", pad=0.8))


def dim_h(ax, x1, x2, y, text, ext_from=None):
    """수평 치수. y 는 치수선 높이, ext_from 은 치수보조선 시작 y."""
    if ext_from is not None:
        for x in (x1, x2):
            ax.plot([x, x], [ext_from, y], **EXT)
    ax.annotate("", (x1, y), (x2, y), arrowprops=AR)
    ax.text((x1 + x2) / 2, y, text, **TXT)


def dim_v(ax, y1, y2, x, text, ext_from=None):
    if ext_from is not None:
        for y in (y1, y2):
            ax.plot([ext_from, x], [y, y], **EXT)
    ax.annotate("", (x, y1), (x, y2), arrowprops=AR)
    ax.text(x, (y1 + y2) / 2, text, rotation=90, **TXT)


def leader(ax, x, y, tx, ty, text):
    """지시선 — 구멍·반경 표기."""
    ax.annotate("", (x, y), (tx, ty),
                arrowprops=dict(arrowstyle="->", color="#0b57a4", lw=0.8,
                                shrinkA=0, shrinkB=0, mutation_scale=8))
    ha = "left" if tx > x else "right"
    ax.text(tx + (1.5 if tx > x else -1.5), ty, text, color="#0b57a4",
            fontsize=8.5, ha=ha, va="center")


def draw_view(ax, polys, hidden=(), title=""):
    for xs, ys in hidden:
        ax.plot(xs, ys, color="#9aa0a6", lw=0.5, ls=(0, (5, 3)))
    for xs, ys in polys:
        ax.plot(xs, ys, color="#1b1b1b", lw=0.9, solid_joinstyle="round")
    ax.set_aspect("equal")
    ax.axis("off")
    if title:
        ax.set_title(title, fontsize=11, pad=8)


def centerline(ax, x=None, y=None, lim=None):
    st = dict(color="#c62828", lw=0.5, ls=(0, (8, 3, 1, 3)))
    if x is not None:
        ax.plot([x, x], lim, **st)
    if y is not None:
        ax.plot(lim, [y, y], **st)


# ---------------------------------------------------------------- DXF
def export_dxf():
    made = []

    def write(shapes, name):
        ex = ExportDXF(unit=Unit.MM, line_weight=0.25)
        ex.add_layer("VISIBLE", color=ColorIndex.BLACK,
                     line_type=LineType.CONTINUOUS)
        ex.add_shape(shapes, layer="VISIBLE")
        ex.write(str(OUT / "dxf" / name))
        made.append((name, len(shapes)))

    laid = []
    off = 0.0
    for key in ("정면", "평면", "우측"):
        d, up = VIEWS[key]
        vis, _hid = edges_raw(PART, d, up)
        tag = {"정면": "front", "평면": "top", "우측": "right"}[key]
        write(vis, f"low_clamp_{tag}.dxf")
        # 3면 배치 — 뷰마다 폭을 재서 20mm 씩 띄운다
        bb = Compound(children=list(vis)).bounding_box()
        laid += [Pos(off - bb.min.X, -bb.min.Y, 0) * e for e in vis]
        off += bb.size.X + 20
    write(laid, "low_clamp_views.dxf")

    with BuildSketch() as sk:                      # 레일 단면
        add(profile_sketch())
    write(sk.sketch.edges(), f"{P.EXTRUSION}_profile.dxf")
    return made


# ---------------------------------------------------------------- 시트
def sheet():
    proj = {}
    for key, (d, up) in VIEWS.items():
        proj[key] = edges_2d(PART, d, up)

    fig = plt.figure(figsize=(16.5, 11.7), dpi=150)          # A3 가로
    gs = fig.add_gridspec(2, 2, hspace=0.10, wspace=0.08,
                          left=0.04, right=0.98, top=0.90, bottom=0.168)

    # ---- 정면도 (X-Z) : 폭·높이·통로·시트
    ax = fig.add_subplot(gs[0, 0])
    draw_view(ax, proj["정면"][0], title="정면도  FRONT  (바퀴 쪽에서)")
    centerline(ax, x=0, lim=(-6, H + 10))
    centerline(ax, y=AXLE_Z, lim=(-W / 2 - 10, W / 2 + 10))
    dim_h(ax, -W / 2, W / 2, -14, f"{W:.2f}", ext_from=0)
    dim_h(ax, -GAP / 2, GAP / 2, H + 9, f"{GAP:.2f}", ext_from=H)
    dim_h(ax, -W / 2, -GAP / 2, H + 20, f"{P.LOW_POST_W:.2f}", ext_from=H)
    dim_v(ax, 0, H, W / 2 + 15, f"{H:.2f}", ext_from=W / 2)
    dim_v(ax, 0, MATE_Z, W / 2 + 30, f"{MATE_Z:.2f}", ext_from=W / 2)
    dim_v(ax, 0, AXLE_Z, -W / 2 - 15, f"{AXLE_Z:.2f}  축중심", ext_from=-W / 2)
    dim_v(ax, 0, RAIL_Z0, -W / 2 - 30, f"{RAIL_Z0:.2f}", ext_from=-W / 2)
    leader(ax, 0, AXLE_Z - SEAT_R, -18, AXLE_Z - SEAT_R - 12,
           f"R{SEAT_R:.2f}  축 시트")
    leader(ax, -P.LOW_POST_X, SIDE_Z, -W / 2 - 8, SIDE_Z + 14,
           f"4-Ø{BOLT_D:.1f}  옆면 볼트")
    ax.set_xlim(-W / 2 - 44, W / 2 + 44)
    ax.set_ylim(-24, H + 30)

    # ---- 평면도 (X-Y) : 볼트 배치
    ax = fig.add_subplot(gs[0, 1])
    vis, hid = proj["평면"]
    draw_view(ax, vis, hid, title="평면도  TOP  (탭·볼트 배치)")
    centerline(ax, x=0, lim=(-W / 2 - 10, W / 2 + 10))
    centerline(ax, y=0, lim=(-D / 2 - 10, D / 2 + 10))
    dim_h(ax, -D / 2, D / 2, -W / 2 - 12, f"{D:.2f}", ext_from=-W / 2)
    dim_h(ax, -P.OEM_BOLT[1], P.OEM_BOLT[1], -W / 2 - 24,
          f"{2*P.OEM_BOLT[1]:.2f}", ext_from=-W / 2)
    dim_h(ax, -SLOT / 2, SLOT / 2, W / 2 + 12, f"{SLOT:.2f}  레일 홈",
          ext_from=W / 2)
    dim_v(ax, -P.OEM_BOLT[0], P.OEM_BOLT[0], D / 2 + 12,
          f"{2*P.OEM_BOLT[0]:.2f}", ext_from=D / 2)
    dim_v(ax, -P.LOW_POST_X, P.LOW_POST_X, D / 2 + 26,
          f"{2*P.LOW_POST_X:.2f}", ext_from=D / 2)
    leader(ax, P.OEM_BOLT[1], P.OEM_BOLT[0], D / 2 + 4, W / 2 - 4,
           f"4-M{P.OEM_BOLT_D:.0f} 탭 (하공 Ø{TAP_D:.1f})")
    leader(ax, 0, P.LOW_POST_X, -D / 2 - 4, W / 2 - 4,
           f"2-Ø{BOLT_D:.1f} + Ø{P.LOW_CBORE[0]:.0f} 카운터보어 깊이 {P.LOW_CBORE[1]:.0f}")
    ax.set_xlim(-D / 2 - 40, D / 2 + 44)
    ax.set_ylim(-W / 2 - 34, W / 2 + 16)

    # ---- 우측면도 (Y-Z)
    ax = fig.add_subplot(gs[1, 0])
    vis, hid = proj["우측"]
    draw_view(ax, vis, hid, title="우측면도  RIGHT  (축 방향 단면 배치)")
    centerline(ax, x=0, lim=(-6, H + 10))
    centerline(ax, y=AXLE_Z, lim=(-D / 2 - 10, D / 2 + 10))
    dim_h(ax, -D / 2, D / 2, -14, f"{D:.2f}", ext_from=0)
    dim_v(ax, 0, MATE_Z, D / 2 + 14, f"{MATE_Z:.2f}", ext_from=D / 2)
    dim_v(ax, 0, H, D / 2 + 28, f"{H:.2f}", ext_from=D / 2)
    # 레일 홈·플랜지 치수는 **플랜지를 켰을 때만** 의미가 있다. 무조건 찍었더니
    # 플랜지를 끈 형상에 없는 30.60 / 8.00 이 남아 있었다.
    if P.LOW_SIDE:
        dim_h(ax, -SLOT / 2, SLOT / 2, H + 9, f"{SLOT:.2f}", ext_from=H)
        dim_h(ax, SLOT / 2, D / 2, H + 20, f"{P.LOW_FLANGE_T:.2f}", ext_from=H)
        dim_v(ax, 0, SIDE_Z, -D / 2 - 14, f"{SIDE_Z:.2f}", ext_from=-D / 2)
    leader(ax, 0, AXLE_Z, D / 2 + 4, AXLE_Z - 16,
           f"Ø{2*SEAT_R:.2f} 축 시트 (축 Ø{P.AXLE_DIAMETER:.0f} + {2*P.SADDLE_SEAT_FIT:.2f})")
    if P.TIE:                                  # 액슬 타이 탭
        tz = P.TIE_Z + DZ
        dim_v(ax, 0, tz, -D / 2 - 14, f"{tz:.2f}", ext_from=-D / 2)
        leader(ax, -D / 2, tz, -D / 2 - 18, tz - 14,
               f"2-M{P.TIE_ROD:.0f} 타이 탭 깊이 {P.TIE_ENGAGE:.0f}"
               f"  (x=±{P.TIE_X}, 하공 Ø{P.TIE_ROD-1.2:.1f})")
    ax.set_xlim(-D / 2 - 40, D / 2 + 44)
    ax.set_ylim(-24, H + 30)

    # ---- 등각 + 표제란
    ax = fig.add_subplot(gs[1, 1])
    draw_view(ax, proj["등각"][0], title="등각도  ISO")
    ax.set_xlim(*[c * 1.08 for c in ax.get_xlim()])

    part = LC.low_clamp()
    fig.suptitle("하부 클램프  LOW CLAMP  —  호버보드 허브모터 휠 마운트 (Design D)",
                 fontsize=16, y=0.965)
    left = (f"품명  하부 클램프 (low_clamp)\n"
            f"재질  6061-T6\n"
            f"수량  4 (차량 1대분)\n"
            f"소재  {W:.0f} x {D:.0f} x {H:.0f} 각재")
    n_thru = 6 if P.LOW_SIDE else 2
    taps = f"탭    M{P.OEM_BOLT_D:.0f} x 4 (상부 클램프)"
    if P.TIE:
        taps += f" / M{P.TIE_ROD:.0f} x {P.TIE_N} (액슬 타이)"
    mid = (f"체적  {part.volume/1000:.0f} cm³   질량  {part.volume*2.7e-3:.0f} g\n"
           + taps + "\n"
           + f"통과공 Ø{BOLT_D:.1f} x {n_thru} (레일 밑면"
           + (" 2 / 옆면 4)" if P.LOW_SIDE else " 2)") + "\n"
           + "모서리  디버링 (기능 필렛 없음)")
    right = (f"일반공차  ±0.1 (기계가공)\n"
             f"축 시트   Ø{2*SEAT_R:.2f} H8   맞댐면 평면도 0.05\n"
             f"모서리    날 제거\n"
             f"표면      Ra 3.2")
    bolts = (f"레일 {P.EXTRUSION} / 밑면 M{P.TNUT_BOLT:.0f}x{P.LOW_BOLT_LEN:.0f}"
             f" x2 + 스프링너트")
    if P.LOW_SIDE:
        bolts += f" / 옆면 M{P.TNUT_BOLT:.0f}x{P.LOW_SIDE_BOLT_LEN:.0f} x4"
    if P.TIE:
        Ltie = 2 * (210.0 - P.LOW_D / 2 + P.TIE_ENGAGE)
        bolts += f"\n액슬 타이  M{P.TIE_ROD:.0f} 전산볼트 Ø{P.TIE_ROD:.0f} x {Ltie:.1f} x{P.TIE_N} (축당)"
    seq = ("조립 ① 축 안착 → ② 상부 클램프 체결 → "
           + ("③ 레일 끝에서 밀어 끼움 → ④ 6볼트" if P.LOW_SIDE
              else "③ 레일 얹고 밑면 2볼트 → ④ 액슬 타이"))
    note = "짝  기존 호버보드 클램프 (상부) — 별도 가공 없음\n" + bolts + "\n" + seq
    for x, s in ((0.045, left), (0.300, mid), (0.600, right)):
        fig.text(x, 0.128, s, fontsize=9.5, va="top", linespacing=1.7)
    fig.text(0.045, 0.058, note, fontsize=9, va="top", color="#444",
             linespacing=1.6)
    fig.text(0.985, 0.030,
             "치수 mm / 데이텀: 밑면 z=0, 좌우·앞뒤 중심\n"
             "models/hoverboard_mount/low_clamp.py 에서 자동 생성",
             fontsize=8.5, va="top", ha="right", color="#777")
    fig.patches.append(plt.Rectangle((0.035, 0.015), 0.95, 0.128, fill=False,
                                     ec="#999", lw=0.8,
                                     transform=fig.transFigure))
    for p in (OUT / "drawing" / "low_clamp_drawing.png",
              OUT / "drawing" / "low_clamp_drawing.pdf"):
        fig.savefig(p, facecolor="white")
    plt.close(fig)
    return OUT / "drawing" / "low_clamp_drawing.png"


if __name__ == "__main__":
    png = sheet()
    print(f"도면 → {png}")
    print("DXF →")
    for name, n in export_dxf():
        print(f"  {name:32s} 선 {n}개")
