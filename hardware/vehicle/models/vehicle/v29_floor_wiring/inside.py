r"""v28 속 보기 — 상부 프레임 · 덮개 · 외관판 · 모듈을 떼고 실내 부품과 배선만 본다.

    .venv/Scripts/python.exe models/vehicle/v28_wiring/inside.py          # 그림 (메시 캐시) → images/inside_*.png
    .venv/Scripts/python.exe models/vehicle/v28_wiring/inside.py --show   # + 뷰어 (3939) 에 속만 보낸다

record.py v28_wiring 를 먼저 돌려 output/scene.npz 가 있어야 한다.
"""

import sys
from pathlib import Path

HERE = Path(__file__).parent
sys.path.insert(0, str(HERE.parent))

import numpy as np                                     # noqa: E402
import build123d                                       # noqa: E402,F401 — record._purge 가 __main__ (이 파일) 을 지우기 전에 IPython 까지 불러 둔다

import record as R                                     # noqa: E402

VERSION = HERE.name
HIDE_KEYS = {"shell_acm", "shell_pc", "shell_lid", "lid_knob", "shell_rubber", "module_rail", "module_plate",
             "lidar", "gnss", "status_light", "frame_bracket"}
HIDE_LABEL = ("상부사이드", "상부크로스")              # frame 그룹 안의 상부 프레임
VIEWS = {
    "앞왼 위에서": ((0.9, 0.75, 1.25), (0, 0, 1)),
    "뒤오른 위에서": ((-0.9, -0.75, 1.25), (0, 0, 1)),
    "위": ((0, 0, 1), (0, 1, 0)),
    "옆 (-y 에서, 기둥 · 판 뺌)": ((0, -1, 0), (0, 0, 1)),
}
ZOOM = {   # 이름: (창 min, max)
    "앞 — 배전 (XT90 · 차단기 · 컨택터 · 분기 레일 · N 바)": ((115.0, -150.0, 125.0), (385.0, 150.0, 240.0)),
    "뒤 — 젯슨 · ESP32 · 허브 · 포트 칸": ((-390.0, -200.0, 125.0), (-40.0, 200.0, 270.0)),
}


def keep(leaf):
    return leaf["key"] not in HIDE_KEYS and not leaf["label"].startswith(HIDE_LABEL)


def images():
    meta, tris, vn, owner = R.load_scene(VERSION)
    k = np.array([keep(lf) for lf in meta["leaves"]])
    m = k[owner]
    t, v, o = tris[m], vn[m], owner[m]
    cols = R.leaf_colors(meta, o)
    out = HERE / "images"
    title = f"{meta['version']}  속 보기 — 상부 프레임 · 덮개 · 외관판 · 모듈 뺌"
    d, u = VIEWS["앞왼 위에서"]
    R.label(R.draw(t, v, cols, d, u, (1600, 1100)), title, px=32).save(out / "inside_iso.png")
    cells = [R.label(R.draw(t, v, cols, d, u, (900, 620), ground=(d[2] >= 0)), name) for name, (d, u) in VIEWS.items()]
    R.grid(cells, 2, header=title).save(out / "inside.png")
    side = np.array([keep(lf) and not lf["label"].startswith(("기둥", "하부사이드")) for lf in meta["leaves"]])[o]
    zc = []
    for name, win in ZOOM.items():
        for vname in ("앞왼 위에서", "위"):
            d, u = VIEWS[vname]
            zc.append(R.label(R.draw(t, v, cols, d, u, (900, 620), window=win), f"{name} · {vname}"))
    R.grid(zc, 2, header=f"{meta['version']}  배선 가까이").save(out / "inside_zoom.png")
    del side
    print("  속 보기:", ", ".join(p.name for p in sorted(out.glob("inside*.png"))))


def show():
    A = R.load_assembly(VERSION)
    groups = A.build()
    keepg = []
    full = "--full" in sys.argv                                 # 차 전체 (아무것도 안 뺀다)
    for g in groups:
        if g["key"] in HIDE_KEYS and not full:
            continue
        parts = [p for p in g["parts"] if full or not p.label.startswith(HIDE_LABEL)]
        if g["key"].startswith("wire_"):                       # 뷰어용으로 가볍게 — 꺾임 구 빼고 원기둥만 (바닥 배선은 토막이 수천 개)
            from build123d import Compound
            lean = []
            for p in parts:
                cyl = [s for s in p.solids() if len(s.faces()) > 1]
                c = Compound(children=cyl)
                c.label = p.label
                lean.append(c)
            parts = lean
        if parts:
            keepg.append(dict(g, parts=parts))
    comps = A.compounds(keepg)
    from ocp_vscode import Camera, set_port, show as oshow
    set_port(A.PORT)
    oshow(*comps, names=[c.label for c in comps], colors=[g["color"] for g in keepg], progress=None,
          reset_camera=Camera.RESET, render_edges=False, deviation=0.5, angular_tolerance=0.5)
    print(f"  뷰어 {A.PORT} 에 속만 보냄 — 그룹 {len(keepg)}")


if __name__ == "__main__":
    if "--noimg" not in sys.argv:
        images()
    if "--show" in sys.argv:
        show()
