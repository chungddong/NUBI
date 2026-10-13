r"""프레임 보요소 해석 결과 시각화 + 애니메이션.

  .venv/Scripts/python.exe tools/frame/frame_view.py

만드는 것:
  docs/images/65_frame_load_cases.png   하중 3가지 변형 형상 + 바퀴하중
  docs/images/66_frame_corner_load.gif  한쪽 구석 하중 애니메이션
"""

import sys
from pathlib import Path

import numpy as np
import matplotlib

matplotlib.use("Agg")

import matplotlib.animation as anim                               # noqa: E402
import matplotlib.pyplot as plt                                   # noqa: E402
from matplotlib import font_manager                               # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT / "tools" / "frame"),
                str(ROOT / "models" / "hoverboard_mount"), str(ROOT / "models")]

import frame_fea as F                                             # noqa: E402

for _f in ("Malgun Gothic", "NanumGothic", "Gulim"):
    if any(x.name == _f for x in font_manager.fontManager.ttflist):
        plt.rcParams["font.family"] = _f
        break
plt.rcParams["axes.unicode_minus"] = False

OUT = ROOT / "docs" / "images"
CASES = ["중앙", "한쪽구석", "뒤축위"]
SCALE = 200.0


def member_stress(store, U):
    """부재별 조합응력 [MPa]. 강체 링크는 0 으로 둔다."""
    out = []
    for dofs, T, kl, L, p, tag, a, b in store:
        if p is F.RIG:
            out.append(0.0)
            continue
        f = kl @ (T @ U[dofs])
        c = 15.0 if p is F.BAR else 4.0
        My = max(abs(f[4]), abs(f[10]))
        Mz = max(abs(f[5]), abs(f[11]))
        out.append(abs(f[6]) / p["A"] + (My + Mz) * c / p["Iy"])
    return out


def draw(ax, m, U, store, scale, title):
    ax.clear()
    P0 = np.array(m.nodes)
    P1 = P0 + U.reshape(-1, 6)[:, :3] * scale
    st = member_stress(store, U)
    smax = max(st) or 1.0
    for (dofs, T, kl, L, p, tag, a, b), s in zip(store, st):
        if tag == "바퀴":
            continue
        ax.plot(*np.array([P0[a], P0[b]]).T, color="#ccd3d8", lw=0.7, alpha=.5, zorder=1)
        if p is F.TIE:
            col, lw = "#b6483c", 2.8
        elif p is F.RIG:
            col, lw = "#9aa3ad", 1.2
        else:
            col, lw = plt.cm.turbo(min(s / smax, 1.0)), 2.1
        ax.plot(*np.array([P1[a], P1[b]]).T, color=col, lw=lw, zorder=3)
    for k, i in m.idx.items():
        if k[0] == "G":
            ax.scatter(*P1[i], c="#8a5a2b", s=24, zorder=4)
    ax.set_box_aspect((760, 640, 420))
    ax.set_axis_off()
    ax.view_init(elev=22, azim=-58)
    if title:
        ax.set_title(title, fontsize=12.5, pad=2)
    return smax


def main():
    fig = plt.figure(figsize=(18, 6.6), dpi=150)
    for i, case in enumerate(CASES):
        m, U, R, store, g = F.run(case, tie=True)
        dz = U.reshape(-1, 6)[:, 2]
        wheels = sorted(abs(R[6 * n + 2]) for n in g)
        ax = fig.add_subplot(1, 3, i + 1, projection="3d")
        smax = draw(ax, m, U, store, SCALE, "")
        head = case + "   최대처짐 " + f"{abs(dz).max():.3f} mm"
        sub = ("바퀴하중 " + f"{wheels[0]:.0f} ~ {wheels[-1]:.0f} N"
               + "   프레임 최대응력 " + f"{smax:.1f} MPa")
        ax.set_title(head + "\n" + sub, fontsize=12.5, pad=2)
    fig.suptitle("차량 프레임 보요소 해석 — 차량 40kg + 사람 70kg · 변형 200배 과장 "
                 "· 빨강 = 액슬 타이", fontsize=16, y=0.99)
    fig.tight_layout(rect=(0, 0, 1, 0.92))
    fig.savefig(OUT / "65_frame_load_cases.png", facecolor="white", bbox_inches="tight")
    plt.close(fig)

    m, U, R, store, g = F.run("한쪽구석", tie=True)
    dmax = abs(U.reshape(-1, 6)[:, 2]).max()
    figA = plt.figure(figsize=(8.4, 7.6), dpi=110)
    axA = figA.add_subplot(111, projection="3d")

    def frame(i):
        t = 0.5 - 0.5 * np.cos(2 * np.pi * i / 40)
        draw(axA, m, U, store, SCALE * t,
             "한쪽 구석에 70kg — 변형 200배 과장\n실제 처짐 "
             + f"{dmax * t:.3f} mm")
        return []

    anim.FuncAnimation(figA, frame, frames=40, interval=70).save(
        OUT / "66_frame_corner_load.gif", writer=anim.PillowWriter(fps=14))
    plt.close(figA)
    print("SAVED 65_frame_load_cases.png / 66_frame_corner_load.gif")


if __name__ == "__main__":
    main()
