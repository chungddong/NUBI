"""다시 돌려 보기 (replay.py) 결과를 시뮬레이터로 — 섞은 위치를 따라 시뮬레이터 차를 움직이고, 그 자리 웹캠 시점 렌더와
실제 웹캠 사진을 나란히 붙여 동영상으로 남긴다. 시뮬레이터 페이지를 ?env=hallway&cam=xtion&bridge=1 로 열어 둔다.

    python replay_sim.py <녹화 폴더> [--fps 4] [--url http://127.0.0.1:8748]   → <녹화 폴더>/replay_sim.mp4
"""

import argparse
import base64
import csv
import json
import math
import urllib.request
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from hallway_map import BOUNDS, HallwayMap, find_scene
from sim_bridge import cam_R

FONT = None
for f in (r"C:\Windows\Fonts\malgun.ttf", "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"):
    try:
        FONT = ImageFont.truetype(f, 18)
        SMALL = ImageFont.truetype(f, 14)
        break
    except OSError:
        pass


def post(url, obj, timeout=20):
    req = urllib.request.Request(url + "/api/bridge/job", data=json.dumps(obj).encode("utf-8"),
                                 headers={"Content-Type": "application/json"})
    return json.loads(urllib.request.urlopen(req, timeout=timeout).read())


def minimap(hm, path, raw, cams, i, W=1280, H=300):
    """복도 지도 위에 섞은 길 (파랑) · 오도메트리만 (회색) · 카메라 답 (주황), 지금 자리 (빨강)."""
    x0, y0, x1, y1 = -3.5, -13.0, 48.0, 6.0
    s = min(W / (x1 - x0), H / (y1 - y0))
    P = lambda x, y: ((x - x0) * s, (y1 - y) * s)
    im = Image.new("RGB", (W, H), (250, 250, 248))
    g = ImageDraw.Draw(im)
    for a, b, c, d in hm.segs:
        g.line([P(a, b), P(c, d)], fill=(90, 90, 90), width=2)
    if len(raw) > 1:
        g.line([P(*p) for p in raw[: i + 1]], fill=(170, 170, 170), width=2)
    for p in cams[: i + 1]:
        if p:
            q = P(*p)
            g.ellipse([q[0] - 3, q[1] - 3, q[0] + 3, q[1] + 3], fill=(235, 104, 52))
    if i > 0:
        g.line([P(*p) for p in path[: i + 1]], fill=(42, 120, 214), width=3)
    q = P(*path[i])
    g.ellipse([q[0] - 7, q[1] - 7, q[0] + 7, q[1] + 7], fill=(208, 59, 59))
    if FONT:
        g.text((10, H - 24), "파랑 = 카메라 + 오도메트리 · 회색 = 오도메트리만 · 주황 = 카메라 답", fill=(70, 70, 70), font=SMALL)
    return im


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run", type=Path)
    ap.add_argument("--fps", type=float, default=4.0)
    ap.add_argument("--url", default="http://127.0.0.1:8748")
    ap.add_argument("--csv", default="replay.csv")
    a = ap.parse_args()
    cfg = json.loads((Path(__file__).with_name("robot.json")).read_text(encoding="utf-8"))
    rows = list(csv.DictReader(open(a.run / a.csv, encoding="utf-8")))
    frames = {round(float(r["t"]), 1): r["i"] for r in csv.DictReader(open(a.run / "frames.csv", encoding="utf-8"))
              if (a.run / "webcam" / f"{int(r['i']):06d}.jpg").exists()}          # 사진을 일부만 가져왔어도
    ftimes = sorted(frames)
    hm = HallwayMap(find_scene())
    path = [(float(r["x"]), float(r["y"])) for r in rows]
    raw = [(float(r["raw_x"]), float(r["raw_y"])) if r["raw_x"] else path[k] for k, r in enumerate(rows)]
    cams = [(float(r["cam_x"]), float(r["cam_y"])) if r["cam_ok"] == "1" and r["cam_x"] else None for r in rows]
    hs = sorted(float(r["cam_h"]) for r in rows if r["cam_ok"] == "1" and r["cam_h"])
    cam_h = hs[len(hs) // 2] if hs else 0.35
    w, h = 640, 360
    f = (w / 2) / math.tan(math.radians(cfg.get("webcam_hfov_deg", 70.0)) / 2)
    off = cfg.get("webcam_x_m", 0.35)
    out = a.run / "replay_sim.mp4"
    vw = cv2.VideoWriter(str(out), cv2.VideoWriter_fourcc(*"mp4v"), a.fps, (1280, 360 + 300 + 40))
    for k, r in enumerate(rows):
        x, y, yaw = float(r["x"]), float(r["y"]), math.radians(float(r["yaw"]))
        view = dict(name="webcam", pos=[x + off * math.cos(yaw), y + off * math.sin(yaw), cam_h], R=cam_R(yaw, 0.0),
                    K=[f, f, w / 2, h / 2], size=[w, h])
        status = f"다시 돌려 보기 t {float(r['t']):.0f} s · {r['fused']}\n섞은 위치 ({x:.2f}, {y:.2f}) {float(r['yaw']):+.0f}°"
        res = post(a.url, dict(pose=dict(x=x, y=y, yaw=yaw), status=status, views=[view]))
        ren = np.zeros((h, w, 3), np.uint8)
        if res.get("ok"):
            v = res["views"]["webcam"]
            ren = cv2.imdecode(np.frombuffer(base64.b64decode(v["jpg"].split(",", 1)[1]), np.uint8), cv2.IMREAD_COLOR)
        t = float(r["t"])
        tf = min(ftimes, key=lambda q: abs(q - t))
        real = cv2.imread(str(a.run / "webcam" / f"{int(frames[tf]):06d}.jpg"))
        real = cv2.resize(real, (w, h)) if real is not None else np.zeros((h, w, 3), np.uint8)
        top = np.hstack([real, ren])
        mm = cv2.cvtColor(np.array(minimap(hm, path, raw, cams, k)), cv2.COLOR_RGB2BGR)
        bar = Image.new("RGB", (1280, 40), (24, 24, 24))
        if FONT:
            ok = "카메라로 찾음" if r["cam_ok"] == "1" else "카메라로 못 찾음 (오도메트리)"
            ImageDraw.Draw(bar).text((10, 9), f"t {t:6.1f} s   왼쪽: 실제 웹캠   오른쪽: 섞은 위치에서 3D 지도 렌더   ·   {ok} · {r['fused']}",
                                     fill=(235, 235, 235), font=FONT)
        frame = np.vstack([top, cv2.cvtColor(np.array(bar), cv2.COLOR_RGB2BGR), mm])
        vw.write(frame)
        if k % 50 == 0:
            print(f"  {k}/{len(rows)}", flush=True)
    vw.release()
    print(f"→ {out}", flush=True)


if __name__ == "__main__":
    main()
