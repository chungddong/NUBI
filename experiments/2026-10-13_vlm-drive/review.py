"""녹화 (record.py) 한 번을 훑어보기 쉬운 동영상으로 — Xtion 컬러 · Xtion 깊이 · 웹캠을 나란히, 아래에 시각 · 속도 · 거리.

    python3 review.py <녹화 폴더> [--speed 2] [--height 300]      → <녹화 폴더>/preview.mp4 (H.264, Jetson GStreamer x264)

사진은 5 Hz 로 저장돼 있으니 --speed 2 면 10 fps 로 재생 (실제의 2 배 빠르기).
"""

import argparse
import csv
import json
import sys
from pathlib import Path

import cv2
import numpy as np


def depth_color(png):
    d = cv2.imread(str(png), cv2.IMREAD_UNCHANGED)
    if d is None:
        return None
    v = np.clip(d.astype(np.float32) / 4000.0 * 255, 0, 255).astype(np.uint8)
    img = cv2.applyColorMap(255 - v, cv2.COLORMAP_JET)
    img[d == 0] = (40, 40, 40)
    return img


def fit(img, h):
    if img is None:
        return None
    return cv2.resize(img, (round(img.shape[1] * h / img.shape[0]) // 2 * 2, h), interpolation=cv2.INTER_AREA)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run", type=Path)
    ap.add_argument("--speed", type=float, default=2.0)
    ap.add_argument("--height", type=int, default=300)
    ap.add_argument("--out", type=Path)
    a = ap.parse_args()
    run, H = a.run, a.height
    meta = json.loads((run / "meta.json").read_text(encoding="utf-8"))
    rows = list(csv.DictReader(open(run / "frames.csv", encoding="utf-8")))
    sign = meta["robot"]["odo_sign"]
    m = meta["robot"]["m_per_step"]
    hz = meta.get("hz", 5.0)
    ws = [round(H * 4 / 3) // 2 * 2, round(H * 4 / 3) // 2 * 2]
    if "webcam" in meta:
        ws.append(round(H * meta["webcam"]["size"][0] / meta["webcam"]["size"][1]) // 2 * 2)
    W, BAR = sum(ws), 34
    out = a.out or run / "preview.mp4"
    pipe = (f"appsrc ! videoconvert ! video/x-raw,format=I420 ! x264enc speed-preset=veryfast bitrate=2500 key-int-max=30 "
            f"! h264parse ! mp4mux ! filesink location={out}")
    vw = cv2.VideoWriter(pipe, cv2.CAP_GSTREAMER, 0, hz * a.speed, (W, H + BAR), True)
    if not vw.isOpened():
        sys.exit("GStreamer x264 로 동영상을 못 연다")
    odo0, dist, prev = None, 0.0, None
    for k, r in enumerate(rows):
        i = int(r["i"])
        tiles = [fit(cv2.imread(str(run / "xtion" / f"{i:06d}.jpg")), H), fit(depth_color(run / "xtion_depth" / f"{i:06d}.png"), H)]
        if "webcam" in meta:
            tiles.append(fit(cv2.imread(str(run / "webcam" / f"{i:06d}.jpg")), H))
        frame = np.full((H + BAR, W, 3), 24, np.uint8)
        x = 0
        for t, w in zip(tiles, ws):
            if t is not None:
                frame[:H, x:x + min(w, t.shape[1])] = t[:, :w]
            x += w
        v = ""
        if r["odo_m"] != "":
            o = (int(r["odo_m"]), int(r["odo_s"]))
            if prev is not None:
                dr, dl = (o[0] - prev[0]) * sign[0] * m, (o[1] - prev[1]) * sign[1] * m
                if abs(dr) < 1 and abs(dl) < 1:
                    dist += (dr + dl) / 2
            prev = o
            v = f"v {(float(r['kmh_r']) - float(r['kmh_l'])) / 2:+.2f} km/h   dist {dist:6.2f} m"
        txt = f"t {float(r['t']):6.1f} s   frame {i}   {v}"
        cv2.putText(frame, txt, (10, H + 23), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (230, 230, 230), 1, cv2.LINE_AA)
        cv2.putText(frame, "xtion | depth 0-4 m | webcam", (W - 300, H + 23), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150, 150, 150), 1, cv2.LINE_AA)
        vw.write(frame)
        if k % 500 == 0:
            print(f"  {k}/{len(rows)}", flush=True)
    vw.release()
    print(f"→ {out} ({len(rows)} 프레임, {len(rows) / hz / a.speed:.0f} s 재생)", flush=True)


if __name__ == "__main__":
    main()
