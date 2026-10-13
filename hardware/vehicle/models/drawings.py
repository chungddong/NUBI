# 도면 시트 — 은선제거(HLR) 6면 투영.
#
# docs/images/41~47 을 만든다. 예전에 만든 스크립트가 저장이 안 돼서 이미지만 남아
# 있었다 (그래서 모델이 스키드로 바뀐 뒤에도 도면은 스워브 시절 치수 그대로였다).
# 이제 스크립트가 있으니 모델을 고치면 도면도 같이 갱신된다.
#
# 쓰는 법:  .venv\Scripts\python.exe models\drawings.py
#   SUBJECT=vehicle | wheelmount   무엇을 그릴지 (기본 vehicle)
#   HUB_MOCK=1  바퀴를 목업으로 (빠른 확인용)
#   ONE=정면    한 뷰만 빠르게 확인

import os
import time

os.environ.setdefault("NOSHOW", "1")
os.environ.setdefault("DRIVE", "skid")
# 여기서는 **실물 바퀴가 기본**이다. 다른 렌더는 목업을 쓰지만(관례), 도면은 남한테
# 주는 문서라 정확도가 값어치다. 재보니 목업 4.6s → 실물 23s/면, 6면에 140초뿐이고
# 선은 1108 → 3278 개로 3배 촘촘해진다. 자주 굽는 것도 아니라 140초는 싸다.
# 빠르게 배치만 확인할 때는 HUB_MOCK=1.

import matplotlib                                                   # noqa: E402

matplotlib.use("Agg")

import matplotlib.pyplot as plt                                     # noqa: E402
from matplotlib import font_manager                                 # noqa: E402

from build123d import *                                             # noqa: E402

# 대상은 두 가지다. 차량 전체는 swerve_base 를 통째로 부르느라 2분이 걸리는데,
# 휠 마운트만 볼 때는 그럴 이유가 없다 → **필요한 쪽만 import 한다.**
SUBJECT = os.environ.get("SUBJECT", "vehicle")

if SUBJECT == "vehicle":
    import swerve_base as M                                         # noqa: E402
    from swerve_base import Path                                    # noqa: E402
    SHAPE = M.assembly
    _b = SHAPE.bounding_box()
    DIMS = (f"전장 {_b.size.X:.0f} x 전폭 {_b.size.Y:.0f} x 전고 {_b.size.Z:.0f} mm"
            f"  (축간 {2*M.AXLE_X:.0f} / 윤거 {2*M.AXLE_Y:.0f})")
    TITLE = f"스키드 스티어 모빌리티 베이스 (페이로드 없음)  —  {DIMS}"
    PREFIX, SRC = "skid", "models/swerve_base.py"
elif SUBJECT == "wheelmount":
    from pathlib import Path                                        # noqa: E402
    from tslot import bar                                           # noqa: E402
    import wheel_mount as WM                                        # noqa: E402
    _m = WM.mount_low();  _m.label, _m.color = "하단클램프", Color("#6b7480")
    _cl = Pos(0, WM.LOW_CY - WM.CLAMP_Y, 0) * WM.clamp()
    _cl.label, _cl.color = "기존클램프", Color("#c62828")
    _rl = bar(30, 300, (-150, WM.LOW_CY, WM.LOW_RAIL_Z), "X")
    _rl.label, _rl.color = "하부사이드레일", Color("#c8ccd0")
    _wh = Pos(0, 0, WM.AXLE_Z) * WM.wheel()
    _wh.label, _wh.color = "바퀴", Color("#2b2b2b")
    SHAPE = Compound(children=[_rl, _m, _cl, _wh])
    DIMS = (f"하단클램프 {WM.LOW_W:.0f}x{WM.LOW_D:.0f}x{WM.LOW_H:.0f} + 다리"
            f" (전체높이 {WM.LOW_RAIL_BOT-(WM.AXLE_Z-WM.LOW_H):.1f}, {_m.volume/1000:.0f} cm3)"
            f"  /  축 z={WM.AXLE_Z:.2f}  /  레일중심 z={WM.LOW_RAIL_Z:.2f}"
            f"  /  지상고 {WM.AXLE_Z-WM.LOW_H:.2f}")
    TITLE = f"휠 마운트 — 하단클램프 + 호버보드 기존클램프 + 3030 레일  —  {DIMS}"
    PREFIX, SRC = "wheelmount", "models/wheel_mount.py"
else:
    raise SystemExit(f"SUBJECT 는 vehicle | wheelmount (받은 값 {SUBJECT})")

OUT = Path(__file__).parent.parent / "docs" / "images"
OUT.mkdir(parents=True, exist_ok=True)

for _f in ("Malgun Gothic", "맑은 고딕", "NanumGothic", "Gulim"):
    if any(f.name == _f for f in font_manager.fontManager.ttflist):
        plt.rcParams["font.family"] = _f
        break
plt.rcParams["axes.unicode_minus"] = False

# (이름, 파일명, 시선 방향, 화면 위쪽)
# 좌표 규약: 지면 z=0 / 전방 +X / 좌 +Y / 상 +Z
_V = [("정면 FRONT", "front", (1, 0, 0), (0, 0, 1)),
      ("우측 RIGHT", "right", (0, -1, 0), (0, 0, 1)),
      ("배면 REAR", "rear", (-1, 0, 0), (0, 0, 1)),
      ("좌측 LEFT", "left", (0, 1, 0), (0, 0, 1)),
      ("평면 TOP", "top", (0, 0, 1), (1, 0, 0)),
      ("등각 ISO", "iso", (1, -1, 0.8), (0, 0, 1))]
VIEWS = [(n, f"{42+i}_{PREFIX}_view_{f}", d, u) for i, (n, f, d, u) in enumerate(_V)]
ELEV = {"정면 FRONT", "우측 RIGHT", "배면 REAR", "좌측 LEFT"}   # 축척을 공유하는 4면


def edges_2d(shape, direction, up):
    """HLR 투영 → 화면 좌표 폴리라인 목록. 보이는 선만 쓴다."""
    vis, _hid = shape.project_to_viewport(
        viewport_origin=tuple(1e4 * c for c in direction), viewport_up=up)
    out = []
    for e in vis:
        # 직선은 두 점이면 되고, 곡선만 잘게 나눈다 (파일 크기·그리기 속도)
        n = 1 if e.geom_type == GeomType.LINE else 48
        try:
            pts = [e @ (i / n) for i in range(n + 1)]
        except Exception:
            continue
        out.append(([p.X for p in pts], [p.Y for p in pts]))
    return out


def draw(ax, polys, title, half=None):
    for xs, ys in polys:
        ax.plot(xs, ys, color="#1b1b1b", lw=0.6, solid_joinstyle="round")
    ax.set_aspect("equal")
    ax.axis("off")
    ax.set_title(title, fontsize=13, pad=14)
    if half:                                   # 4개 입면 동일 축척
        # 중심은 **bbox 중심**이어야 한다. 점 평균으로 잡았더니 선이 촘촘한 쪽으로
        # 중심이 끌려가 좌우가 잘렸다 (곡선은 48분할, 직선은 2점이라 밀도가 다르다).
        xs = [x for p in polys for x in p[0]]
        ys = [y for p in polys for y in p[1]]
        cx, cy = (min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2
        ax.set_xlim(cx - half[0], cx + half[0])
        ax.set_ylim(cy - half[1], cy + half[1])


_one = os.environ.get("ONE")
todo = [v for v in VIEWS if not _one or _one in v[0]]
proj, t0 = {}, time.time()
for name, fn, d, up in todo:
    t = time.time()
    proj[name] = edges_2d(SHAPE, d, up)
    print(f"  {name:12s} 선 {len(proj[name]):5d}개  {time.time()-t:5.1f}s")
print(f"  HLR 투영 {len(todo)}면 / {time.time()-t0:.0f}s"
      f"  ({SUBJECT})")

# 4개 입면의 공통 축척 — 도면은 같은 배율이어야 치수를 눈으로 비교할 수 있다
_ev = [p for n, p in proj.items() if n in ELEV]
if _ev:
    _w = max(max(x for xs, _ in v for x in xs) - min(x for xs, _ in v for x in xs)
             for v in _ev)
    _h = max(max(y for _, ys in v for y in ys) - min(y for _, ys in v for y in ys)
             for v in _ev)
    HALF = (_w * 0.54, _h * 0.60)   # 여백 8% / 20%
else:
    HALF = None

for name, fn, d, up in todo:                       # 낱장
    fig, ax = plt.subplots(figsize=(9, 6.5), dpi=150)
    draw(ax, proj[name], f"{name}   —   {DIMS}", HALF if name in ELEV else None)
    fig.tight_layout()
    fig.savefig(OUT / f"{fn}.png", bbox_inches="tight", facecolor="white")
    plt.close(fig)

if not _one:                                       # 시트 (2행 x 3열)
    fig, axs = plt.subplots(2, 3, figsize=(19, 11), dpi=150)
    for ax, (name, fn, d, up) in zip(axs.flat, VIEWS):
        draw(ax, proj[name], name, HALF if name in ELEV else None)
    fig.suptitle(TITLE, fontsize=17, y=0.975)
    fig.text(0.5, 0.018,
             f"{SRC}   |   지면 z=0, 전방 +X, 좌 +Y, 상 +Z   |   "
             "은선 제거(HLR) 투영, 4개 입면 동일 축척", ha="center", fontsize=11,
             color="#555")
    fig.tight_layout(rect=(0, 0.035, 1, 0.955))
    fig.savefig(OUT / f"41_{PREFIX}_drawing_sheet.png", bbox_inches="tight",
                facecolor="white")
    plt.close(fig)

print(f"도면 → {OUT}")
print(f"  {f'41_{PREFIX}_drawing_sheet.png  +  ' if not _one else ''}낱장 {len(todo)}장")
print(f"  {TITLE}")
