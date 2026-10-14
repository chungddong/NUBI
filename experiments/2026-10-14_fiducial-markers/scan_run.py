"""녹화 한 번의 사진 (웹캠 · Xtion 5 Hz) 에서 마커를 모두 찾는다 → <녹화>/markers_<카메라>.csv, 찾은 장면 모음 그림.

    python3 scan_run.py <녹화 폴더> [--size-mm 160] [--camera webcam,xtion] [--video]   (--video: webcam.mkv 30 fps 전부)

csv: 프레임 (i 또는 동영상 n) · 시각 · ID · 카메라에서 거리 · 옆 · 높이 차 · 비스듬한 각 (마커 면 법선과 시선 사이) · 한 변 px.
웹캠 내부값은 가로 화각 70° (2026-10-14 SfM 추정), Xtion 은 meta.json 값.
"""

import argparse
import csv
import json
import math
import sys
import time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from markers import detect, tag_pose  # noqa: E402


def K_for(meta, cam, hfov_webcam):
    if cam == "xtion":
        fx, fy, cx, cy = meta["xtion"]["K_fx_fy_cx_cy"]
        return np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1.0]])
    w, h = meta["webcam"]["size"]
    f = (w / 2) / math.tan(math.radians(hfov_webcam) / 2)
    return np.array([[f, 0, w / 2], [0, f, h / 2], [0, 0, 1.0]])


def measure(img, K, size_m):
    out = []
    for tid, c in detect(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)):
        side = float(np.mean(np.linalg.norm(np.roll(c, -1, 0) - c, axis=1)))
        rvec, tvec = tag_pose(c, size_m, K)
        if tvec is None:
            continue
        t = tvec.ravel()
        n = cv2.Rodrigues(rvec)[0][:, 2]
        ray = t / np.linalg.norm(t)
        oblique = math.degrees(math.acos(min(1.0, abs(float(n @ ray)))))
        out.append(dict(id=tid, dist=float(np.linalg.norm(t)), x=float(t[0]), y=float(t[1]), z=float(t[2]),
                        oblique=oblique, side_px=side, corners=c))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run", type=Path)
    ap.add_argument("--size-mm", type=float, default=160)
    ap.add_argument("--camera", default="webcam,xtion")
    ap.add_argument("--hfov", type=float, default=70.0)
    ap.add_argument("--video", action="store_true", help="webcam.mkv (30 fps) 도 전부")
    ap.add_argument("--sheet", type=int, default=12, help="찾은 장면 모음 그림에 넣을 장 수")
    a = ap.parse_args()
    meta = json.loads((a.run / "meta.json").read_text(encoding="utf-8"))
    F = list(csv.DictReader(open(a.run / "frames.csv", encoding="utf-8")))
    size = a.size_mm / 1000
    for cam in [c for c in a.camera.split(",") if c in meta]:
        K = K_for(meta, cam, a.hfov)
        rows, hits, t0 = [], [], time.perf_counter()
        n_frames = 0
        for r in F:
            p = a.run / cam / f"{int(r['i']):06d}.jpg"
            img = cv2.imread(str(p))
            if img is None:
                continue
            n_frames += 1
            found = measure(img, K, size)
            for m in found:
                rows.append(dict(i=r["i"], t=r["t"], **{k: (round(v, 3) if isinstance(v, float) else v) for k, v in m.items() if k != "corners"}))
            if found:
                hits.append((r["i"], r["t"], img, found))
        dt = time.perf_counter() - t0
        out = a.run / f"markers_{cam}.csv"
        with open(out, "w", newline="", encoding="utf-8") as fh:
            w = csv.DictWriter(fh, fieldnames=["i", "t", "id", "dist", "x", "y", "z", "oblique", "side_px"])
            w.writeheader()
            w.writerows(rows)
        ids = sorted({m["id"] for m in rows})
        print(f"{a.run.name} {cam}: 사진 {n_frames} 장 중 {len(hits)} 장에서 마커 (ID {ids}), 찾은 것 {len(rows)} 개, "
              f"{dt / max(1, n_frames) * 1000:.0f} ms/장 → {out.name}", flush=True)
        for tid in ids:
            d = [m for m in rows if m["id"] == tid]
            print(f"   ID {tid}: {len(d)} 번 · 거리 {min(m['dist'] for m in d):.2f} ~ {max(m['dist'] for m in d):.2f} m · "
                  f"비스듬 {min(m['oblique'] for m in d):.0f} ~ {max(m['oblique'] for m in d):.0f}° · 한 변 "
                  f"{min(m['side_px'] for m in d):.0f} ~ {max(m['side_px'] for m in d):.0f} px · "
                  f"시각 {float(d[0]['t']):.1f} ~ {float(d[-1]['t']):.1f} s", flush=True)
        if hits:                                           # 찾은 장면 모음 (고르게 골라)
            pick = [hits[k] for k in np.linspace(0, len(hits) - 1, min(a.sheet, len(hits))).astype(int)]
            tiles = []
            for i, t, img, found in pick:
                im = img.copy()
                for m in found:
                    c = m["corners"].astype(int)
                    cv2.polylines(im, [c], True, (0, 255, 0), 3)
                    cv2.putText(im, f"{m['id']} {m['dist']:.1f}m {m['oblique']:.0f}d", tuple(c[0]), cv2.FONT_HERSHEY_SIMPLEX,
                                1.0 if cam == "webcam" else 0.6, (0, 0, 255), 2)
                cv2.putText(im, f"t {float(t):.1f}s", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
                tiles.append(cv2.resize(im, (480, round(480 * im.shape[0] / im.shape[1]))))
            cols = 4
            while len(tiles) % cols:
                tiles.append(np.zeros_like(tiles[0]))
            sheet = np.vstack([np.hstack(tiles[k:k + cols]) for k in range(0, len(tiles), cols)])
            cv2.imwrite(str(a.run / f"markers_{cam}_sheet.jpg"), sheet, [cv2.IMWRITE_JPEG_QUALITY, 85])
    if a.video and (a.run / "webcam.mkv").exists():
        V = [float(r["t"]) for r in csv.DictReader(open(a.run / "webcam_video.csv", encoding="utf-8"))]
        K = K_for(meta, "webcam", a.hfov)
        cap = cv2.VideoCapture(str(a.run / "webcam.mkv"))
        rows, n, t0 = [], 0, time.perf_counter()
        while True:
            ok, img = cap.read()
            if not ok:
                break
            for m in measure(img, K, size):
                rows.append(dict(n=n, t=V[n] if n < len(V) else "", **{k: (round(v, 3) if isinstance(v, float) else v)
                                                                       for k, v in m.items() if k != "corners"}))
            n += 1
        with open(a.run / "markers_webcam_video.csv", "w", newline="", encoding="utf-8") as fh:
            w = csv.DictWriter(fh, fieldnames=["n", "t", "id", "dist", "x", "y", "z", "oblique", "side_px"])
            w.writeheader()
            w.writerows(rows)
        fr = len({r["n"] for r in rows})
        print(f"{a.run.name} webcam.mkv: {n} 프레임 중 {fr} 프레임에서 마커 ({fr / max(1, n):.0%}), ID {sorted({r['id'] for r in rows})}, "
              f"{(time.perf_counter() - t0) / max(1, n) * 1000:.0f} ms/프레임", flush=True)


if __name__ == "__main__":
    main()
