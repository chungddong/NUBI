"""녹화 (record.py) 다시 돌려 보기 — 바퀴 보고 줄로 오도메트리를 돌리고, 사진으로 위치를 찾아 섞는다 (실차와 같은 길).

    python replay.py <녹화 폴더> [--from 163] [--to 400] [--every 1.0] [--camera webcam] [--m-per-step 0.0059]

결과: <녹화 폴더>/replay.csv (시각마다 섞은 자세 · 오도메트리만 · 카메라 답) 와 replay_summary.json
차 높이 · 카메라 위치 (robot.json 의 webcam_*) 는 [가정] 이면 그 값을 쓴다. 사진이 1 초에 한 장만 있어도 된다
(없는 사진은 건너뜀).
"""

import argparse
import csv
import json
import math
import sys
import time
from pathlib import Path

import cv2

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "2026-10-13_sfm-relocalization"))

from odometry import Odometry  # noqa: E402
from pose_fusion import PoseFusion  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run", type=Path)
    ap.add_argument("--from", dest="t_from", type=float, default=0.0)
    ap.add_argument("--to", dest="t_to", type=float, default=1e9)
    ap.add_argument("--every", type=float, default=1.0, help="몇 초마다 사진으로 위치를 찾을지")
    ap.add_argument("--camera", default="webcam")
    ap.add_argument("--m-per-step", type=float)
    ap.add_argument("--track", type=float, help="track_eff_m")
    ap.add_argument("--min-inliers", type=int, default=25)
    ap.add_argument("--global-every", type=float, default=5.0, help="길 잃었을 때 지도 전체 검색은 몇 초에 한 번")
    a = ap.parse_args()

    from localizer import Localizer, pinhole
    cfg = json.loads((HERE / "robot.json").read_text(encoding="utf-8"))
    if a.m_per_step:
        cfg["m_per_step"] = a.m_per_step
    if a.track:
        cfg["track_eff_m"] = a.track
    meta = json.loads((a.run / "meta.json").read_text(encoding="utf-8"))
    w, h = meta[a.camera]["size"]
    if a.camera == "webcam":
        hfov = cfg.get("webcam_hfov_deg", 70.0)
        cam = pinhole(w, h, (w / 2) / math.tan(math.radians(hfov) / 2))
        off = (cfg.get("webcam_x_m", 0.35), cfg.get("webcam_y_m", 0.0), cfg.get("webcam_yaw_deg", 0.0))
    else:
        fx, fy, cx, cy = meta["xtion"]["K_fx_fy_cx_cy"]
        cam = pinhole(w, h, fx, fy, cx, cy)
        off = (cfg.get("camera_x_m", 0.38), 0.0, 0.0)
    frames = {}
    for r in csv.DictReader(open(a.run / "frames.csv", encoding="utf-8")):
        p = a.run / a.camera / f"{int(r['i']):06d}.jpg"
        if p.exists():
            frames[float(r["t"])] = p
    ftimes = sorted(frames)
    wheels = [r for r in csv.DictReader(open(a.run / "wheels.csv", encoding="utf-8")) if a.t_from <= float(r["t"]) <= a.t_to]
    print(f"사진 {len(ftimes)} 장 · 보고 줄 {len(wheels)} 개 · m_per_step {cfg['m_per_step']} · track_eff_m {cfg['track_eff_m']} · "
          f"카메라 위치 {off}", flush=True)

    loc = Localizer()
    print(f"지도 불러옴 {loc.t_load:.1f} s", flush=True)
    odo = Odometry(cfg)                      # 섞는 쪽
    raw = Odometry(cfg)                      # 오도메트리만 (첫 고정에서 시작)
    fu = PoseFusion(odo, cam_off=off)
    rows, next_t, raw_started, last_global = [], None, False, -1e9
    stats = dict(tries=0, ok=0, accepted=0, rejected=0, ms=[])
    fi = 0
    for wr in wheels:
        t = float(wr["t"])
        o = (int(wr["odo_m"]), int(wr["odo_s"]))
        odo.update(o)
        raw.update(o)
        fu.record(t)
        if next_t is None:
            next_t = t
        if t < next_t:
            continue
        next_t += a.every
        while fi + 1 < len(ftimes) and abs(ftimes[fi + 1] - t) <= abs(ftimes[fi] - t):
            fi += 1
        if not ftimes or abs(ftimes[fi] - t) > 0.6:
            continue
        tf = ftimes[fi]
        img = cv2.imread(str(frames[tf]))
        prior = fu.cam_from_base(odo.x, odo.y, odo.yaw) if fu.n_fix and fu.rejects < fu.lost_after else None
        if prior is None and t - last_global < a.global_every:     # 길 잃은 동안 지도 전체 검색 (6 s 넘게) 은 가끔만
            continue
        if prior is None:
            last_global = t
        res = loc.locate(img, cam, prior=prior, min_inliers=a.min_inliers, h_range=(0.0, 0.8))
        stats["tries"] += 1
        stats["ms"].append(res["ms"])
        acc, why = False, "못 찾음"
        innov = (math.nan, math.nan, math.nan)
        if res["ok"]:
            stats["ok"] += 1
            acc, why, innov = fu.fix(tf, res["x"], res["y"], res["yaw"])
            stats["accepted" if acc else "rejected"] += 1
            if acc and not raw_started:                       # 오도메트리만 쓰는 쪽도 첫 고정에서 출발
                raw.x, raw.y, raw.yaw = odo.x, odo.y, odo.yaw
                raw_started = True
        rows.append(dict(t=round(t, 2), x=round(odo.x, 3), y=round(odo.y, 3), yaw=round(math.degrees(odo.yaw), 1),
                         raw_x=round(raw.x, 3) if raw_started else "", raw_y=round(raw.y, 3) if raw_started else "",
                         cam_ok=int(res["ok"]), inliers=res["inliers"], matches=res["matches"], prior=int(res["prior"]),
                         map_images=res["map_images"], cam_x=round(res["x"], 3) if res["ok"] else "",
                         cam_y=round(res["y"], 3) if res["ok"] else "", cam_h=round(res["h"], 3) if res["ok"] else "",
                         cam_yaw=round(res["yaw"], 1) if res["ok"] else "", fused=why,
                         innov_m=round(math.hypot(innov[0], innov[1]), 3) if res["ok"] else "", ms=round(res["ms"])))
        print(f"  t {t:6.1f}  {why:14s}  인라이어 {res['inliers']:3d}  {'근처 ' + str(res['map_images']) + '장' if res['prior'] else '전체'}  "
              f"→ ({odo.x:6.2f}, {odo.y:5.2f}, {math.degrees(odo.yaw):6.1f}°)  {res['ms']:.0f} ms", flush=True)
    with open(a.run / "replay.csv", "w", newline="", encoding="utf-8") as fh:
        wcsv = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        wcsv.writeheader()
        wcsv.writerows(rows)
    ms = sorted(stats.pop("ms"))
    stats.update(ms_median=ms[len(ms) // 2] if ms else None, m_per_step=cfg["m_per_step"], track_eff_m=cfg["track_eff_m"],
                 cam_off=off, camera=a.camera)
    (a.run / "replay_summary.json").write_text(json.dumps(stats, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(stats, ensure_ascii=False), flush=True)


if __name__ == "__main__":
    main()
