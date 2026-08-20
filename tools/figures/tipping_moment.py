# 젖힘 모멘트 설명 그림 — docs/images/63_why_tipping_moment.png
#                          docs/images/64_tipping_animation.gif
#
#   .venv/Scripts/python.exe tools/figures/tipping_moment.py
#
# 실제 좌표(parameters.py / low_clamp.py)로 1:1 축척 Y-Z 단면을 그린다.
# 회전 중심은 **레일 바깥 밑모서리** — FEA 가 낸 변형 방향(축 시트가 바깥으로,
# 위는 거의 그대로)과 일치시킨 것이다. 축 중심으로 돌리면 방향이 반대로 나온다.
#
# 주석에 윈도 경로를 백슬래시로 적었다가  가 폼피드가 되어 파일이 쪼개졌다.
# 경로는 슬래시로 적는다.

import pathlib
import sys, numpy as np
sys.path[:0] = [r"D:\build123d\models\hoverboard_mount", r"D:\build123d\models"]
import pathlib
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.patches import Rectangle, FancyArrow, Arc
import parameters as P, low_clamp as LC
for f in ("Malgun Gothic", "NanumGothic", "Gulim"):
    if any(x.name == f for x in font_manager.fontManager.ttflist):
        plt.rcParams["font.family"] = f; break
plt.rcParams["axes.unicode_minus"] = False

GY, CY = -P.AXLE_Z, LC.CLAMP_CY
W, ARM = P.DESIGN_W, abs(LC.CLAMP_CY)
M, BOLT = W*ARM, W*ARM/(P.EXTRUSION_W/2)/2
# 피벗 = 레일 바깥쪽 밑모서리 (실제로 눌려서 닿는 자리)
PVT = (CY + P.EXTRUSION_W/2, LC.RAIL_BOT)

def draw_parts(ax, tilt=0.0, alpha=1.0, ec="#6b7480", lw=1.2):
    th = np.radians(tilt); c, s = np.cos(th), np.sin(th)
    def box(y0, z0, dy, dz, fc, ec2, z):
        p = np.array([[y0,z0],[y0+dy,z0],[y0+dy,z0+dz],[y0,z0+dz]], float) - PVT
        q = np.stack([p[:,0]*c - p[:,1]*s, p[:,0]*s + p[:,1]*c], 1) + PVT
        ax.add_patch(plt.Polygon(q, fc=fc, ec=ec2, alpha=alpha, zorder=z, lw=lw))
    box(-P.WHEEL_WIDTH/2, GY, P.WHEEL_WIDTH, P.WHEEL_DIAMETER, "#2b2f34", "#15181b", 2)
    box(P.AXLE_END_Y, -8, P.AXLE_START_Y-P.AXLE_END_Y, 16, "#8f96a0", "#5c6067", 5)
    box(CY-P.LOW_D/2, LC.LOW_BOT, P.LOW_D, LC.RAIL_BOT-LC.LOW_BOT, "#c3cad3", ec, 4)
    box(CY-P.OEM_H/2, LC.CLAMP_SEAT_Z, P.OEM_H, P.OEM_T, "#e0a24a", "#a06f22", 6)

def rail(ax, z=9):
    ax.add_patch(Rectangle((CY-P.EXTRUSION_W/2, LC.RAIL_BOT), P.EXTRUSION_W,
                           P.EXTRUSION_H, fc="#5d6570", ec="#222", zorder=z))
    ax.text(CY, LC.RAIL_BOT+P.EXTRUSION_H/2, "3030\n프레임 (고정)", color="white",
            ha="center", va="center", fontsize=10, zorder=z+1)

def panel2(ax, tilt, show_labels=True):
    draw_parts(ax, 0, alpha=0.15, ec="#aab", lw=0.9)
    draw_parts(ax, tilt)
    rail(ax)
    ax.plot([-125, 60], [GY, GY], color="#8a5a2b", lw=3, zorder=1)
    ax.plot(*PVT, marker="v", ms=13, color="#111", zorder=12)
    ax.text(PVT[0]+5, PVT[1]+2, "여기가 눌려서\n회전 중심이 된다", fontsize=10.5,
            color="#111", va="center")
    if show_labels:
        ax.add_patch(Arc((CY, 34), 100, 100, theta1=200, theta2=340,
                         color="#c62828", lw=3.2, zorder=10))
        ax.annotate("", (CY+52, 26), (CY+48, 38), arrowprops=dict(arrowstyle="-|>",
                    color="#c62828", lw=3.2, mutation_scale=21))
        ax.text(CY+20, 64, f"{M/1000:.0f} N·m", color="#c62828", ha="center",
                fontsize=17, weight="bold")
        x = CY - P.EXTRUSION_W/2
        ax.add_patch(FancyArrow(x, LC.RAIL_BOT+4, 0, 30, width=3.4, head_width=11,
                     head_length=11, fc="#c62828", ec="#c62828", zorder=11,
                     length_includes_head=True))
        ax.text(x-5, LC.RAIL_BOT+40, f"안쪽 볼트가\n뽑히려 한다\n{BOLT:.0f} N",
                color="#c62828", fontsize=11.5, weight="bold", ha="right", va="center")
    ax.set_xlim(-128, 72); ax.set_ylim(GY-46, 132); ax.set_aspect("equal"); ax.axis("off")

# ---------------- 정지 3단 ----------------
fig = plt.figure(figsize=(19, 8.6), dpi=150)
gs = fig.add_gridspec(1, 3, width_ratios=[1.15, 1.05, 1.0], wspace=0.04)
ax = fig.add_subplot(gs[0])
draw_parts(ax); rail(ax, 4)
ax.text(0, 40, "바퀴", color="white", ha="center", fontsize=13, zorder=3)
ax.plot([-125, 60], [GY, GY], color="#8a5a2b", lw=3, zorder=1)
for x, c, lab, ha in ((0,"#c62828","하중선\n타이어 접지","left"),
                      (CY,"#0b57a4","지지선\n프레임 볼트","right")):
    ax.plot([x,x],[GY-40,128], color=c, lw=1.7, ls=(0,(7,4)), zorder=6)
    ax.text(x+(4 if ha=="left" else -4), 126, lab, color=c, fontsize=11.5,
            va="top", ha=ha, weight="bold")
ax.add_patch(FancyArrow(0, GY-1, 0, 40, width=4, head_width=13, head_length=13,
             fc="#c62828", ec="#c62828", zorder=7, length_includes_head=True))
ax.text(7, GY+24, f"{W:.0f} N\n노면이 미는 힘", color="#c62828", fontsize=12, va="center")
ax.add_patch(FancyArrow(CY, 112, 0, -40, width=4, head_width=13, head_length=13,
             fc="#0b57a4", ec="#0b57a4", zorder=7, length_includes_head=True))
ax.text(CY-8, 95, f"{W:.0f} N\n차체가 누르는 힘", color="#0b57a4", fontsize=12,
        va="center", ha="right")
y0 = GY-26
ax.annotate("", (CY,y0), (0,y0), arrowprops=dict(arrowstyle="<->", color="#111",
            lw=2.4, mutation_scale=17))
for x in (0, CY): ax.plot([x,x],[GY-40,y0], color="#111", lw=0.8)
ax.text(CY/2, y0-13, f"팔 길이 {ARM:.1f} mm", ha="center", fontsize=15, weight="bold",
        bbox=dict(fc="#fff3cd", ec="#c9a227", pad=5))
ax.set_xlim(-128, 72); ax.set_ylim(GY-52, 140); ax.set_aspect("equal"); ax.axis("off")
ax.set_title("① 두 힘이 한 선에 없다", fontsize=15, pad=12, weight="bold")
ax.text(-126, 137, "← 차체 안쪽                 바깥 →", fontsize=11, color="#666")

ax = fig.add_subplot(gs[1]); panel2(ax, 6.0)
ax.set_title("② 레일 밑모서리를 축으로 젖혀진다", fontsize=15, pad=12, weight="bold")
ax.text(CY, GY-40, "위는 거의 그대로, 아래로 갈수록 바깥으로",
        ha="center", fontsize=12.5, weight="bold",
        bbox=dict(fc="#fdecea", ec="#c62828", pad=5))

ax = fig.add_subplot(gs[2]); ax.axis("off"); ax.set_xlim(0,10); ax.set_ylim(0,10)
ax.set_title("③ FEA 로 잰 실제 바깥 밀림", fontsize=15, pad=12, weight="bold")
for i,(lab,v,c) in enumerate([("부러진 구형 (PETG 출력)",2.16,"#c62828"),
                              ("지금 것 (PETG 출력)",0.67,"#e8a33d"),
                              ("지금 것 (알루미늄)",0.02,"#137333")]):
    y = 7.9 - i*1.95
    ax.text(0.25, y+0.78, lab, fontsize=13, weight="bold")
    ax.add_patch(Rectangle((0.25, y-0.12), 8.6*v/2.4, 0.6, fc=c, ec="none"))
    ax.text(0.25+8.6*v/2.4+0.18, y+0.18, f"{v:.2f} mm", fontsize=13, color=c,
            weight="bold", va="center")
ax.text(0.25, 2.7,
        "팔 길이가 같으니 젖히는 힘 43 N·m 도 같다.\n다른 건 '얼마나 안 휘고 버티느냐' 뿐.\n\n"
        "· 구형은 축~프레임 사이가 길어 많이 휘었다\n"
        "· 지금 것은 짧고 통짜라 3배 덜 휜다\n"
        "· 알루미늄은 PETG보다 33배 단단해 사실상 안 휜다",
        fontsize=12.3, va="top", linespacing=1.75,
        bbox=dict(fc="#f1f3f4", ec="#ccc", pad=9))
fig.suptitle("바퀴가 프레임보다 바깥에 있으면 왜 항상 젖히는 힘이 생기나  —  800 N, 1:1 축척",
             fontsize=17, y=0.985)
OUT = pathlib.Path(__file__).resolve().parents[2] / "docs" / "images"
fig.savefig(OUT / "63_why_tipping_moment.png", facecolor="white", bbox_inches="tight")
plt.close(fig)

# ---------------- 애니메이션 ----------------
import matplotlib.animation as anim
figA, axA = plt.subplots(figsize=(7.4, 8.2), dpi=110)
def frame(i):
    n = 48
    t = i / n
    tilt = 6.0 * (0.5 - 0.5*np.cos(2*np.pi*t))     # 0 → 6 → 0
    axA.clear(); panel2(axA, tilt, show_labels=tilt > 0.6)
    axA.set_title(f"레일 고정, 800 N 이 바퀴를 밀 때\n(변형 과장 — 실제 알루미늄은 0.02 mm)",
                  fontsize=13, pad=10)
    return []
a = anim.FuncAnimation(figA, frame, frames=48, interval=70, blit=False)
a.save(OUT / "64_tipping_animation.gif", writer=anim.PillowWriter(fps=14))
print("SAVED png + gif")
