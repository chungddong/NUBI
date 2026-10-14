#!/usr/bin/env python
"""녹화 (vlm-drive/record.py) 의 사진마다 SfM 지도 위 위치를 찾는다 — 차 높이 카메라로 실제로 되는지 보는 시험.

    python3 locate_run.py <녹화 폴더> [--every 5] [--camera xtion,webcam] [--webcam-hfov auto]

빠르게 하려고 3D 점마다 대표 설명자 하나 (그 점 관측들 가운데 다른 관측과 거리 합이 가장 작은 것) 만 쓴다
(관측 389,085 → 점 51,220). 정답 자세가 없으니 결과는 (1) 성공률 · 인라이어, (2) 복도를 따라 이어지는 궤적인지,
(3) 카메라 높이 (지도 y, 바닥 0) 가 실제 장착 높이와 맞는지로 본다.

결과: <녹화 폴더>/locate_<카메라>.csv — i, t, ok, inliers, matches, x, y (장면 좌표: x 복도 안쪽, y 왼쪽), 높이, 방향°
웹캠은 내부값을 모르니 --webcam-hfov auto 면 몇 장으로 가로 화각 45 ~ 100° 를 훑어 인라이어가 가장 많은 값을 쓴다.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import cv2  # noqa: E402

from colmap_lite import Camera, extract_sift, read_db, read_model, solve_pnp  # noqa: E402
from relocalize import Matcher, SfmMap, find_work, load_align  # noqa: E402


class PointMap:
    """3D 점마다 대표 설명자 하나 + 지도 좌표."""

    def __init__(self, work: Path, all_obs=False):
        t0 = time.perf_counter()
        to_map, _ = load_align(work / "align.json")
        al = json.loads((work / "align.json").read_text(encoding="utf-8"))
        self.R_al = np.array(al["rotation"], float)
        self.to_map = to_map
        rec = read_model(work / "sfm/model/1")
        kps, descs = read_db(work / "sfm/database.db")
        sm = SfmMap(rec, kps, descs, to_map)
        D = sm.obs_desc.astype(np.float32)
        rep = np.empty((len(sm.pids), 128), np.float32)
        for i in range(len(sm.pids)):
            a, b = sm.off[i], sm.off[i + 1]
            g = D[a:b]
            if len(g) <= 2:
                rep[i] = g[0]
                continue
            d2 = (g * g).sum(1)[:, None] - 2 * g @ g.T + (g * g).sum(1)[None]
            rep[i] = g[np.argmin(np.sqrt(np.maximum(d2, 0)).sum(1))]
        self.desc, self.dn = rep, (rep * rep).sum(1)
        self.all = Matcher(sm.obs_desc, sm.obs_pt, 256) if all_obs else None
        self.n_obs = len(sm.obs_pt)
        self.xyz = sm.xyz
        self.map_cam_xyz = sm.Cs_map
        print(f"지도: 점 {len(rep):,} (관측 {len(D):,} 에서 대표 하나씩), {time.perf_counter() - t0:.1f} s", flush=True)

    def match(self, q: np.ndarray, ratio=0.8, chunk=512):
        if self.all is not None:                    # 관측 전부와 (느림): 가장 가까운 관측 · 가장 가까운 다른 3D 점
            pt, d1, ok, _ = self.all.match(q, np.ones(self.n_obs, bool), ratio)
            q_i = np.flatnonzero(ok)
            o = np.argsort(d1[q_i], kind="stable")
            _, first = np.unique(pt[q_i][o], return_index=True)
            q_i = q_i[o[first]]
            return q_i, pt[q_i]
        q = q.astype(np.float32)
        qn = (q * q).sum(1)
        idx, ok = [], []
        for a in range(0, len(q), chunk):
            d2 = q[a:a + chunk] @ self.desc.T
            d2 *= -2.0
            d2 += self.dn[None]
            d2 += qn[a:a + chunk, None]
            two = np.argpartition(d2, 1, axis=1)[:, :2]
            v = np.take_along_axis(d2, two, 1)
            o = np.argsort(v, 1)
            two, v = np.take_along_axis(two, o, 1), np.sqrt(np.maximum(np.take_along_axis(v, o, 1), 0))
            idx.append(two[:, 0])
            ok.append(v[:, 0] < ratio * v[:, 1])
        idx, ok = np.concatenate(idx), np.concatenate(ok)
        q_i = np.flatnonzero(ok)
        # 3D 점 하나에 질의 하나 (가장 가까운 것)
        _, first = np.unique(idx[q_i], return_index=True)
        q_i = q_i[first]
        return q_i, idx[q_i]

    def pose_to_scene(self, R, t):
        """cam_from_world (SfM) → 장면 좌표 (x 복도 안쪽, y 왼쪽), 높이 (바닥 0), 카메라가 보는 방향 (° , x 축에서 왼쪽 +)."""
        C = -R.T @ t
        m = self.to_map(C)[0]                       # 지도: x 복도, y 위, z 오른쪽
        f = self.R_al @ (R.T @ np.array([0, 0, 1.0]))
        return m[0], -m[2], m[1], math.degrees(math.atan2(-f[2], f[0]))


def locate(pm: PointMap, img, cam: Camera, min_inliers: int, max_error: float):
    xy, d = extract_sift(img, max_size=1600, max_features=5000)
    if len(xy) < 10:
        return dict(ok=False, inliers=0, matches=0, n_kp=len(xy))
    qi, pi = pm.match(d)
    res = solve_pnp(xy[qi], pm.xyz[pi], cam, max_error=max_error) if len(qi) >= 6 else None
    out = dict(ok=False, inliers=res["num_inliers"] if res else 0, matches=len(qi), n_kp=len(xy))
    if res and res["num_inliers"] >= min_inliers:
        x, y, h, yaw = pm.pose_to_scene(res["R"], res["t"])
        out.update(ok=True, x=x, y=y, h=h, yaw=yaw)
    return out


def camera_for(name, meta, hfov=None):
    if name == "xtion":
        w, h = meta["xtion"]["size"]
        fx, fy, cx, cy = meta["xtion"]["K_fx_fy_cx_cy"]
        return Camera(0, "PINHOLE", w, h, np.array([fx, fy, cx, cy]))
    w, h = meta["webcam"]["size"]
    f = (w / 2) / math.tan(math.radians(hfov) / 2)
    return Camera(0, "PINHOLE", w, h, np.array([f, f, w / 2, h / 2]))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run", type=Path)
    ap.add_argument("--every", type=int, default=5, help="몇 장마다 하나 (5 Hz 녹화면 5 = 1 초에 한 장)")
    ap.add_argument("--camera", default="xtion,webcam")
    ap.add_argument("--webcam-hfov", default="auto", help="auto 또는 가로 화각 (도)")
    ap.add_argument("--min-inliers", type=int, default=40)
    ap.add_argument("--max-error", type=float, default=8.0, help="재투영 허용 (px)")
    ap.add_argument("--work", type=Path)
    ap.add_argument("--all-obs", action="store_true", help="대표 설명자 대신 관측 전부와 맞춘다 (약 8 배 느림)")
    ap.add_argument("--tag", default="", help="결과 파일 이름 뒤에 붙임")
    a = ap.parse_args()
    meta = json.loads((a.run / "meta.json").read_text(encoding="utf-8"))
    rows = list(csv.DictReader(open(a.run / "frames.csv", encoding="utf-8")))
    pm = PointMap(find_work(a.work), a.all_obs)

    for name in [c.strip() for c in a.camera.split(",")]:
        if name not in meta:
            print(f"{name}: 이 녹화에 없음", flush=True)
            continue
        folder = a.run / name
        sel = [r for r in rows[::a.every] if (folder / f"{int(r['i']):06d}.jpg").exists()]
        hfov = None
        if name == "webcam":
            if a.webcam_hfov == "auto":
                probe = sel[len(sel) // 10::max(1, len(sel) // 8)][:8]
                best = None
                for hf in range(45, 101, 5):
                    cam = camera_for(name, meta, hf)
                    tot = sum(locate(pm, cv2.imread(str(folder / f"{int(r['i']):06d}.jpg")), cam, a.min_inliers,
                                     a.max_error)["inliers"] for r in probe)
                    print(f"  웹캠 가로 화각 {hf}°: 인라이어 합 {tot}", flush=True)
                    if best is None or tot > best[1]:
                        best = (hf, tot)
                hfov = best[0]
            else:
                hfov = float(a.webcam_hfov)
            print(f"웹캠 가로 화각 {hfov}° 로", flush=True)
        cam = camera_for(name, meta, hfov)
        out = a.run / f"locate_{name}{a.tag}.csv"
        t0, n_ok = time.perf_counter(), 0
        with open(out, "w", newline="", encoding="utf-8") as fh:
            w = csv.writer(fh)
            w.writerow(["i", "t", "ok", "inliers", "matches", "n_kp", "x", "y", "height", "yaw_deg"])
            for k, r in enumerate(sel):
                res = locate(pm, cv2.imread(str(folder / f"{int(r['i']):06d}.jpg")), cam, a.min_inliers, a.max_error)
                n_ok += res["ok"]
                pose = [f"{res[c]:.3f}" for c in ("x", "y", "h")] + [f"{res['yaw']:.1f}"] if res["ok"] else [""] * 4
                w.writerow([r["i"], r["t"], int(res["ok"]), res["inliers"], res["matches"], res["n_kp"], *pose])
                if k % 50 == 0:
                    fh.flush()
                    print(f"  {name} {k}/{len(sel)}  성공 {n_ok}  ({(time.perf_counter() - t0) / (k + 1):.2f} s/장)", flush=True)
        print(f"{name}: {n_ok}/{len(sel)} 성공 → {out}", flush=True)
        meta.setdefault("locate", {})[name] = dict(hfov=hfov, every=a.every, ok=n_ok, n=len(sel))
    (a.run / "locate_meta.json").write_text(json.dumps(meta.get("locate", {}), ensure_ascii=False, indent=2), encoding="utf-8")


if __name__ == "__main__":
    main()
