#!/usr/bin/env python
"""Leave-out relocalization against the hallway COLMAP SfM map.

For each query frame the map is built from every OTHER frame: observations from the query frame
and its +-K temporal neighbours are removed (3D points left without observations drop out).
The query's SIFT features are matched to the remaining map descriptors (exact nearest neighbours,
Lowe ratio against the nearest *other* 3D point, one match per 3D point), the absolute pose is
estimated with RANSAC PnP + LM refinement (OpenCV, known SIMPLE_RADIAL camera) and compared with
the frame's own pose in the reconstruction. Errors are reported in metres in the hallway map frame
(align.json): x along the corridor, y up (floor 0), z to the right when walking to the lounge.

Query features (--features)
  db      the frame's own keypoints/descriptors from database.db (what the map was built from)
  opencv  extracted again from the frame image with OpenCV SIFT, as a live camera would be

Variants
  full  the whole map
  low   rough proxy for a low robot camera: only 3D points with map height y < --low-max-y

numpy + OpenCV only (no pycolmap / torch), so the same script runs on the PC and on the Jetson.
Matching runs on the CPU (OpenBLAS); limit threads with OPENBLAS_NUM_THREADS if needed.
Data (opened read-only) is looked up in --work, $HALLWAY_SFM, <repo>/data/hallway-sfm,
F:\\Develop\\hallway-3d\\tools\\work: sfm/model/1, sfm/database.db, align.json, frames/ (opencv only).
  python3 relocalize.py --step 6 --ks 0,15 --tag _small
Writes results.md, per_query.csv and error_cdf.png next to this script (or --out-dir).
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import platform
import re
import sys
import time
from datetime import datetime
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))   # also under python -I
from colmap_lite import Model, extract_sift, read_db, read_model, solve_pnp

HERE = Path(__file__).resolve().parent
WORK_CANDIDATES = [os.environ.get("HALLWAY_SFM"), HERE.parents[1] / "data" / "hallway-sfm",
                   r"F:\Develop\hallway-3d\tools\work"]
CORRIDOR_LAST = 118          # frames 0..118: straight corridor, 119..262: lounge loop
FPS = 5.0

# variant -> (map filter, 3D point source)
#   all / low : every 3D point, or only points with map height y < --low-max-y
#   retri     : matched points re-triangulated from the left-in frames only (main result)
#   sfm       : the SfM point positions as they are (these were refined with the query frame too)
VARIANTS = {"full": ("all", "retri"), "low": ("low", "retri"), "full_sfm": ("all", "sfm"), "low_sfm": ("low", "sfm")}

# simulator assumption (hardware/vehicle/models/vehicle/sim/web/agent.js): VIS_P, VIS_M, VIS_DEG
SIM_P, SIM_SIGMA_M, SIM_SIGMA_DEG = 0.8, 0.15, 3.0


# ----------------------------------------------------------------------------- data
def load_align(path: Path):
    al = json.loads(Path(path).read_text(encoding="utf-8"))
    R = np.array(al["rotation"], float)
    s = float(al["scale"])
    o = np.array([al["x_offset_units"], al["floor_offset"], al["corridor_z_center_units"]], float)
    x0 = float(al["x_origin_m"])

    def to_map(X):
        Y = s * (np.atleast_2d(np.asarray(X, float)) @ R.T - o)
        Y[:, 0] -= x0
        return Y

    return to_map, s


def find_work(arg):
    for p in ([arg] if arg else WORK_CANDIDATES):
        if p and (Path(p) / "sfm" / "model" / "1" / "points3D.bin").exists():
            return Path(p)
    raise SystemExit("SfM data not found (sfm/model/1, sfm/database.db, align.json) — give --work or set HALLWAY_SFM")


def frame_no(name: str) -> int:
    return int(re.search(r"(\d+)", name).group(1))


class SfmMap:
    """All observations of all 3D points with their descriptor and frame number."""

    def __init__(self, rec: Model, kps, descs, to_map):
        self.rec = rec
        self.images = dict(rec.images)
        self.frame_of = {iid: frame_no(im.name) for iid, im in self.images.items()}
        for iid, im in self.images.items():          # DB and model must describe the same keypoints
            n = len(im.xy)
            if kps[im.name].shape[0] != n:
                raise SystemExit(f"{im.name}: {n} points2D in model vs {kps[im.name].shape[0]} keypoints in DB")
        im0 = next(iter(self.images.values()))
        if not np.allclose(im0.xy, kps[im0.name], atol=1e-3):
            raise SystemExit("DB keypoints do not match model points2D (different scale?)")

        self.pids = rec.point_ids                     # sorted
        self.xyz = rec.xyz
        self.xyz_map = to_map(self.xyz)
        # observations grouped by 3D point: point i owns obs off[i]:off[i+1]
        self.off = rec.track_off
        self.obs_pt = np.repeat(np.arange(len(self.pids)), np.diff(self.off))
        obs_img, obs_p2d = rec.track_img, rec.track_p2d
        self.obs_frame = np.array([self.frame_of[i] for i in obs_img], np.int64)
        self.obs_desc = np.empty((len(obs_img), 128), np.uint8)
        obs_xy = np.empty((len(obs_img), 2))
        for iid, im in self.images.items():
            m = obs_img == iid
            self.obs_desc[m] = descs[im.name][obs_p2d[m]]
            obs_xy[m] = kps[im.name][obs_p2d[m]]
        # for re-triangulation: undistorted normalized coordinates and the SfM pose of each observation
        self.camera = next(iter(rec.cameras.values()))
        self.focal = float(self.camera.params[0])
        self.obs_n = self.camera.cam_from_img(obs_xy)
        img_ids = sorted(self.images)
        k_of = {iid: k for k, iid in enumerate(img_ids)}
        self.obs_k = np.array([k_of[i] for i in obs_img], np.int64)
        self.Rs = np.stack([self.images[i].R for i in img_ids])
        self.ts = np.stack([self.images[i].t for i in img_ids])
        self.Cs = -np.einsum("kji,kj->ki", self.Rs, self.ts)            # camera centres
        self.Cs_map = to_map(self.Cs)
        self.img_frame = np.array([self.frame_of[i] for i in img_ids], np.int64)
        # per image: which 3D point (index into self.pids) each keypoint belongs to, -1 if none
        self.own_pt = {}
        for iid, im in self.images.items():
            own = np.full(len(im.p3d), -1, np.int64)
            has = im.p3d >= 0
            own[has] = np.searchsorted(self.pids, im.p3d[has])
            self.own_pt[iid] = own

    def retriangulate(self, pts, frame, K, min_angle_deg=1.5, max_err_px=4.0, iters=5):
        """Re-triangulate 3D points `pts` (indices into self.pids) from their observations in frames
        with |frame_no - frame| > K only, with the SfM camera poses held fixed: linear (DLT) start,
        a few Gauss-Newton steps on the reprojection error, then COLMAP-like checks (>= 2 views,
        triangulation angle >= min_angle_deg, positive depth, max reprojection error <= max_err_px).
        Returns (ok mask over pts, xyz (len(pts), 3) with NaN where not ok)."""
        pts = np.asarray(pts, np.int64)
        out = np.full((len(pts), 3), np.nan)
        if len(pts) == 0:
            return np.zeros(0, bool), out
        s, e = self.off[pts], self.off[pts + 1]
        lens = e - s
        idx = np.repeat(s - np.concatenate([[0], np.cumsum(lens)[:-1]]), lens) + np.arange(lens.sum())
        own = np.repeat(np.arange(len(pts)), lens)
        keep = np.abs(self.obs_frame[idx] - frame) > K
        idx, own = idx[keep], own[keep]
        cnt = np.bincount(own, minlength=len(pts))
        cand = np.flatnonzero(cnt >= 2)
        if len(cand) == 0:
            return np.zeros(len(pts), bool), out
        sel = cnt[own] >= 2
        idx, own = idx[sel], own[sel]
        compact = np.full(len(pts), -1, np.int64)
        compact[cand] = np.arange(len(cand))
        own = compact[own]
        starts = np.concatenate([[0], np.cumsum(cnt[cand])[:-1]])
        R, t, x = self.Rs[self.obs_k[idx]], self.ts[self.obs_k[idx]], self.obs_n[idx]

        # DLT: rows x*P3 - P1, y*P3 - P2, summed as A^T A per point; smallest eigenvector
        P = np.concatenate([R, t[:, :, None]], axis=2)
        r1 = x[:, 0, None] * P[:, 2] - P[:, 0]
        r2 = x[:, 1, None] * P[:, 2] - P[:, 1]
        A = r1[:, :, None] * r1[:, None, :] + r2[:, :, None] * r2[:, None, :]
        _, V = np.linalg.eigh(np.add.reduceat(A, starts, axis=0))
        Xh = V[:, :, 0]
        X = Xh[:, :3] / Xh[:, 3:4]

        def project(X):
            Xc = np.einsum("nij,nj->ni", R, X[own]) + t
            return Xc, Xc[:, :2] / Xc[:, 2:3]

        for _ in range(iters):                                # Gauss-Newton, poses fixed
            Xc, u = project(X)
            res = u - x
            z = Xc[:, 2]
            Jc = np.zeros((len(z), 2, 3))
            Jc[:, 0, 0] = Jc[:, 1, 1] = 1.0 / z
            Jc[:, 0, 2] = -Xc[:, 0] / z ** 2
            Jc[:, 1, 2] = -Xc[:, 1] / z ** 2
            J = Jc @ R
            H = np.add.reduceat(np.einsum("nki,nkj->nij", J, J), starts, axis=0)
            g = np.add.reduceat(np.einsum("nki,nk->ni", J, res), starts, axis=0)
            try:
                X = X - np.linalg.solve(H, g[:, :, None])[:, :, 0]
            except np.linalg.LinAlgError:                     # degenerate point somewhere
                X = X - (np.linalg.pinv(H) @ g[:, :, None])[:, :, 0]

        Xc, u = project(X)
        err_px = np.linalg.norm(u - x, axis=1) * self.focal
        ok = (np.minimum.reduceat(Xc[:, 2], starts) > 0) & (np.maximum.reduceat(err_px, starts) <= max_err_px)
        # triangulation angle: two-sweep estimate of the largest angle between viewing rays
        ray = self.Cs[self.obs_k[idx]] - X[own]
        ray /= np.linalg.norm(ray, axis=1, keepdims=True)
        ang = lambda a, b: np.arccos(np.clip((a * b).sum(1), -1.0, 1.0))
        v = ang(ray, ray[starts][own])
        far = v >= np.maximum.reduceat(v, starts)[own]
        pos = np.flatnonzero(far)
        _, first = np.unique(own[pos], return_index=True)
        w = ang(ray, ray[pos[first]][own])
        ok &= np.degrees(np.maximum.reduceat(w, starts)) >= min_angle_deg
        res_ok = np.zeros(len(pts), bool)
        res_ok[cand[ok]] = True
        out[cand[ok]] = X[ok]
        return res_ok, out


# ----------------------------------------------------------------------------- matching
class Matcher:
    """Exact L2 nearest neighbours of query descriptors among a subset of map observations, and the
    exact distance to the nearest observation of any *other* 3D point (for the ratio test)."""

    def __init__(self, obs_desc: np.ndarray, obs_pt: np.ndarray, chunk: int):
        self.chunk = chunk
        self.D = obs_desc.astype(np.float32)
        self.Dn = (self.D * self.D).sum(1)
        self.P = obs_pt                                   # non-decreasing: observations grouped by 3D point

    def match(self, q_desc: np.ndarray, valid: np.ndarray, ratio: float):
        """Returns per query descriptor: best 3D point index, its distance, ratio-test pass, and
        whether no other 3D point was left in the map (then the ratio test passes trivially)."""
        sel = np.flatnonzero(valid)
        M, Mn, P = self.D[sel], self.Dn[sel], self.P[sel]
        q = np.asarray(q_desc, np.float32)
        qn = (q * q).sum(1)
        pts, d1s, oks, nos = [], [], [], []
        for a in range(0, q.shape[0], self.chunk):
            d2 = q[a:a + self.chunk] @ M.T                # block of squared distances, |q|^2 - 2 q.m + |m|^2
            d2 *= -2.0
            d2 += Mn[None, :]
            d2 += qn[a:a + self.chunk, None]
            r = np.arange(len(d2))
            j1 = d2.argmin(1)
            v1 = d2[r, j1]
            p1 = P[j1]
            lo, hi = np.searchsorted(P, p1, "left"), np.searchsorted(P, p1, "right")
            n = hi - lo                                   # hide every observation of the best point, then the
            rows = np.repeat(r, n)                        # minimum left is the nearest *other* 3D point
            d2[rows, np.repeat(lo - np.cumsum(n) + n, n) + np.arange(n.sum())] = np.inf
            v2 = d2.min(1)
            del d2
            d1, dsec = np.sqrt(np.maximum(v1, 0)), np.sqrt(np.maximum(v2, 0))
            pts.append(p1); d1s.append(d1); oks.append(d1 < ratio * dsec); nos.append(~np.isfinite(v2))
        return np.concatenate(pts), np.concatenate(d1s), np.concatenate(oks), np.concatenate(nos)


def unique_per_point(qidx, pt, d1):
    """Keep one query keypoint per 3D point (the one with the smallest descriptor distance)."""
    order = np.argsort(d1, kind="stable")
    _, first = np.unique(pt[order], return_index=True)
    keep = order[first]
    return qidx[keep], pt[keep]


# ----------------------------------------------------------------------------- pose
def rot_angle_deg(Ra, Rb):
    c = (np.trace(Ra @ Rb.T) - 1.0) / 2.0
    return float(np.degrees(np.arccos(np.clip(c, -1.0, 1.0))))


def run(args):
    dev_name = f"CPU numpy ({platform.machine()}, {os.cpu_count()} cores)"
    t0 = time.perf_counter()
    to_map, scale = load_align(args.align)
    rec = read_model(args.model)
    kps, descs = read_db(args.database)
    smap = SfmMap(rec, kps, descs, to_map)
    camera = next(iter(rec.cameras.values()))
    matcher = Matcher(smap.obs_desc, smap.obs_pt, args.chunk)
    t_load = time.perf_counter() - t0
    low_pt = smap.xyz_map[:, 1] < args.low_max_y
    print(f"loaded in {t_load:.1f} s: {len(smap.images)} images, {len(smap.pids)} points, "
          f"{len(smap.obs_pt)} observations, low points {int(low_pt.sum())}, {dev_name}, "
          f"query features {args.features}", flush=True)

    def query_features(im):
        """Query keypoints (COLMAP pixel convention), descriptors, and per keypoint its SfM 3D point (or None)."""
        if args.features == "db":
            return kps[im.name], descs[im.name], smap.own_pt[im.id]
        import cv2
        img = cv2.imread(str(args.frames / im.name), cv2.IMREAD_GRAYSCALE)
        if img is None:
            raise SystemExit(f"frame not found: {args.frames / im.name}")
        xy, d = extract_sift(img, args.sift_max_size, args.sift_features)
        return xy, d, None

    by_frame = sorted(smap.images, key=lambda i: smap.frame_of[i])
    queries = by_frame[::args.step]
    if args.max_queries:
        queries = queries[:args.max_queries]
    ks = [int(k) for k in args.ks.split(",")]
    variants = [v.strip() for v in args.variants.split(",")]
    for v in variants:
        if v not in VARIANTS:
            raise SystemExit(f"unknown variant {v}; choose from {', '.join(VARIANTS)}")
    filters = [f for f in ("all", "low") if any(VARIANTS[v][0] == f for v in variants)]

    feats = {}                                          # query features, extracted once for all K
    rows = []
    for K in ks:
        t_block = time.perf_counter()
        for iid in queries:
            im = smap.images[iid]
            f = smap.frame_of[iid]
            if iid not in feats:
                t1 = time.perf_counter()
                feats[iid] = query_features(im) + (time.perf_counter() - t1,)
            q_kp, q_desc, own, t_feat = feats[iid]
            Cr = to_map(im.center)[0]
            R_ref = im.R
            left_in = np.abs(smap.img_frame - f) > K
            nearest_cam = float(np.linalg.norm(smap.Cs_map[left_in] - Cr, axis=1).min())
            for filt in filters:
                valid = np.abs(smap.obs_frame - f) > K           # leave out the query and its +-K neighbours
                if filt == "low":
                    valid &= low_pt[smap.obs_pt]
                n_map_pts = int((np.bincount(smap.obs_pt[valid], minlength=len(smap.pids)) > 0).sum())
                t1 = time.perf_counter()
                pt, d1, ok, nosec = matcher.match(q_desc, valid, args.ratio)
                qidx, mpt = unique_per_point(np.flatnonzero(ok), pt[ok], d1[ok])
                t_match = time.perf_counter() - t1

                for variant in (v for v in variants if VARIANTS[v][0] == filt):
                    t2 = time.perf_counter()
                    if VARIANTS[variant][1] == "retri":
                        tri_ok, X = smap.retriangulate(mpt, f, K)
                        q_use, X_use = qidx[tri_ok], X[tri_ok]
                    else:
                        q_use, X_use = qidx, smap.xyz[mpt]
                    t3 = time.perf_counter()
                    res = None
                    if len(q_use) >= 4:
                        res = solve_pnp(q_kp[q_use], X_use, camera, max_error=args.max_error, seed=args.seed)
                    t4 = time.perf_counter()
                    row = dict(variant=variant, K=K, frame=f, part="corridor" if f <= CORRIDOR_LAST else "lounge",
                               nearest_cam_m=nearest_cam,
                               n_kp=len(q_kp), n_map_obs=int(valid.sum()), n_map_points=n_map_pts,
                               n_own_3d=int((own >= 0).sum()) if own is not None else -1,
                               n_own_low=int(((own >= 0) & low_pt[np.maximum(own, 0)]).sum()) if own is not None else -1,
                               n_matches=len(qidx), n_nosecond=int(nosec.sum()),
                               sfm_agree=float(np.mean(own[qidx] == mpt)) if len(qidx) and own is not None else float("nan"),
                               n_pnp_input=len(q_use), n_inliers=0, success=False,
                               pos_err_m=float("nan"), hor_err_m=float("nan"), rot_err_deg=float("nan"),
                               ref_x=Cr[0], ref_y=Cr[1], ref_z=Cr[2], est_x=float("nan"), est_y=float("nan"),
                               est_z=float("nan"), t_feat_ms=t_feat * 1e3, t_match_ms=t_match * 1e3,
                               t_pnp_ms=(t4 - t3) * 1e3, t_retri_ms=(t3 - t2) * 1e3)
                    if res is not None:
                        R_est = res["R"]
                        Ce = to_map(-R_est.T @ res["t"])[0]
                        d = Ce - Cr
                        row.update(n_inliers=int(res["num_inliers"]),
                                   success=int(res["num_inliers"]) >= args.min_inliers,
                                   pos_err_m=float(np.linalg.norm(d)), hor_err_m=float(np.hypot(d[0], d[2])),
                                   rot_err_deg=rot_angle_deg(R_est, R_ref), est_x=Ce[0], est_y=Ce[1], est_z=Ce[2])
                    rows.append(row)
        for variant in variants:
            s = summarize([r for r in rows if r["variant"] == variant and r["K"] == K])
            print(f"K={K:2d} {variant:8s}: {s['n']} queries, success {s['succ_rate']*100:5.1f} %, "
                  f"hor median {s['hor_med']:.3f} m p90 {s['hor_p90']:.3f} m, rot median {s['rot_med']:.2f} deg, "
                  f"inliers median {s['inl_med']:.0f}", flush=True)
        print(f"K={K:2d} done in {time.perf_counter() - t_block:.1f} s", flush=True)

    order = np.argsort(smap.img_frame)
    step_m = float(np.median(np.linalg.norm(np.diff(smap.Cs_map[order], axis=0), axis=1)))
    meta = dict(step_m=step_m, device=dev_name, t_load=t_load, n_images=len(smap.images), n_points=len(smap.pids),
                n_obs=len(smap.obs_pt), n_low=int(low_pt.sum()), n_queries=len(queries), scale=scale,
                camera=str(camera), argv=" ".join(sys.argv[1:]))
    return rows, meta


# ----------------------------------------------------------------------------- summary
def pct(x, q):
    x = np.asarray(x, float)
    return float(np.percentile(x, q)) if len(x) else float("nan")


def summarize(rows):
    n = len(rows)
    ok = [r for r in rows if r["success"]]
    hor_all = np.array([r["hor_err_m"] if r["success"] else np.inf for r in rows])
    rot_all = np.array([r["rot_err_deg"] if r["success"] else np.inf for r in rows])
    t = [r["t_feat_ms"] + r["t_match_ms"] + r["t_pnp_ms"] for r in rows]
    return dict(
        n=n, n_succ=len(ok), succ_rate=len(ok) / n if n else float("nan"),
        pos_med=pct([r["pos_err_m"] for r in ok], 50), pos_p90=pct([r["pos_err_m"] for r in ok], 90),
        hor_med=pct([r["hor_err_m"] for r in ok], 50), hor_p90=pct([r["hor_err_m"] for r in ok], 90),
        rot_med=pct([r["rot_err_deg"] for r in ok], 50), rot_p90=pct([r["rot_err_deg"] for r in ok], 90),
        hor_max=max([r["hor_err_m"] for r in ok], default=float("nan")),
        gross=sum(1 for r in ok if r["hor_err_m"] > 1.0),
        inl_med=pct([r["n_inliers"] for r in rows], 50), match_med=pct([r["n_matches"] for r in rows], 50),
        map_pts_med=pct([r["n_map_points"] for r in rows], 50),
        near_med=pct([r["nearest_cam_m"] for r in rows], 50), near_min=pct([r["nearest_cam_m"] for r in rows], 0),
        own_low_med=pct([r["n_own_low"] for r in rows], 50),
        nosec_med=pct([r["n_nosecond"] for r in rows], 50),
        agree_med=pct([r["sfm_agree"] for r in rows if r["n_matches"]], 50),
        r015=float(np.mean(hor_all <= 0.15)), r030=float(np.mean(hor_all <= 0.30)),
        r050=float(np.mean(hor_all <= 0.50)), r100=float(np.mean(hor_all <= 1.00)),
        r015_3=float(np.mean((hor_all <= 0.15) & (rot_all <= 3.0))),
        t_med=pct(t, 50), t_p90=pct(t, 90),
        tm_med=pct([r["t_match_ms"] for r in rows], 50), tp_med=pct([r["t_pnp_ms"] for r in rows], 50),
        tf_med=pct([r["t_feat_ms"] for r in rows], 50),
    )


def sim_reference():
    """Simulator's visual fix: x, y each N(0, 0.15 m), yaw N(0, 3 deg), 80 % of steps."""
    s, sd = SIM_SIGMA_M, SIM_SIGMA_DEG
    r = lambda p: s * np.sqrt(-2.0 * np.log(1.0 - p))      # Rayleigh quantile (horizontal radius)
    from math import erf, sqrt
    within = lambda x: 1.0 - np.exp(-x * x / (2 * s * s))
    return dict(succ_rate=SIM_P, hor_med=r(0.5), hor_p90=r(0.9),
                yaw_med=sd * 0.6745, yaw_p90=sd * 1.6449,
                r015=SIM_P * within(0.15), r030=SIM_P * within(0.30), r050=SIM_P * within(0.50),
                r100=SIM_P * within(1.0), r015_3=SIM_P * within(0.15) * erf(3.0 / (sd * sqrt(2))))


VARIANT_KO = {"full": "전체 지도", "low": "낮은 점만", "full_sfm": "전체 지도 · SfM 점 그대로",
              "low_sfm": "낮은 점만 · SfM 점 그대로"}
PART_KO = {"corridor": f"복도 (0–{CORRIDOR_LAST})", "lounge": f"라운지 ({CORRIDOR_LAST + 1}–262)"}


def f2(x, d=2):
    return "–" if x != x else f"{x:.{d}f}"


def write_results(rows, meta, args, path_md: Path):
    variants = [v for v in VARIANTS if any(r["variant"] == v for r in rows)]
    ks = sorted({r["K"] for r in rows})
    combos = [(v, K) for v in variants for K in ks]
    L = []
    L.append("# 결과 — SfM 지도에 카메라 사진 위치 찾기 (leave-out)")
    L.append("")
    L.append(f"> 이 파일은 `relocalize.py`가 만든다 ({datetime.now():%Y-%m-%d %H:%M}). 해석과 주의점은 "
             "[README.md](README.md)에 있다.")
    L.append("")
    L.append("## 조건")
    L.append("")
    L.append(f"- 지도: 프레임 {meta['n_images']}장, 3D 점 {meta['n_points']:,}개, 관측 {meta['n_obs']:,}개. "
             f"낮은 점 (지도 높이 y < {args.low_max_y} m) {meta['n_low']:,}개.")
    L.append(f"- 질의: 프레임 {meta['n_queries']}장 (매 {args.step}번째). 질의 프레임과 앞뒤 ±K 프레임의 관측을 지도에서 뺀다. "
             + "빼는 범위: " + ", ".join(f"K = {k} → ±{k / FPS:g} 초 (약 ±{k * meta['step_m']:.1f} m)" for k in ks)
             + f" (프레임 사이 이동 중앙 {meta['step_m']:.2f} m).")
    feat = ("DB에 있는 그 프레임의 SIFT (지도를 만든 특징점 그대로)" if args.features == "db" else
            f"프레임 사진에서 OpenCV SIFT로 다시 뽑음 (긴 변 {args.sift_max_size} px, 최대 {args.sift_features}개, "
            "RootSIFT uint8 — 실제 카메라와 같은 길)")
    L.append(f"- 질의 특징점: {feat}.")
    L.append(f"- 매칭: 정확한 최근접 (L2, 지도 전체와 전수 비교), 비율 검사 {args.ratio} (가장 가까운 *다른* 3D 점과 비교), "
             "3D 점 하나에 질의 특징점 하나.")
    L.append("- 3D 점: '전체 지도' · '낮은 점만'은 매칭된 3D 점을 남은 프레임 (질의 ±K 밖)의 관측만으로 다시 삼각측량해서 쓴다 "
             "(SfM 카메라 자세는 고정, 2장 이상 · 삼각 각도 ≥ 1.5° · 재투영 ≤ 4 px). 'SfM 점 그대로'는 복원 모델의 점 위치를 "
             "그대로 쓴다. 그 점들은 질의 프레임까지 넣어 다듬은 것이라 오차가 거의 0으로 나온다 (비교용).")
    L.append(f"- 낮은 점만: 지도 높이 y < {args.low_max_y} m 인 3D 점의 관측만 지도에 남기고 그것과만 매칭한다 "
             "(낮은 카메라의 거친 대용, README 참고).")
    L.append(f"- 자세: OpenCV `solvePnPRansac` (AP3P) → 인라이어로 `solvePnPRefineLM` 두 번, 재투영 허용 "
             f"{args.max_error:g} px, 카메라는 알려진 값 ({meta['camera']}). "
             f"성공 = 자세가 나오고 인라이어 ≥ {args.min_inliers}.")
    L.append("- 기준: 같은 프레임의 SfM 복원 자세. 오차는 align.json으로 바꾼 지도 좌표 (m)에서 잰다. "
             "위치 = 카메라 중심 사이 거리, 수평 = x · z만, 회전 = 두 자세 사이 회전 각.")
    L.append(f"- 계산: {meta['device']}. 질의당 시간 = 특징점 뽑기 + 매칭 + PnP "
             f"(지도 불러오기 {meta['t_load']:.0f} s와 재삼각측량은 이 실험 방식 때문에 생긴 것이라 뺀다).")
    cmd = ("relocalize.py " + meta["argv"]).strip()
    L.append(f"- 실행: `{cmd}`, 전체 {meta['t_total']:.0f} s.")
    L.append("")

    L.append("## 1. 한눈에 (전체 질의)")
    L.append("")
    L.append("| 변형 | K | 질의 | 성공률 | 위치 오차 중앙 / 90 % (m) | 수평 오차 중앙 / 90 % (m) | 회전 오차 중앙 / 90 % (°) "
             "| 인라이어 중앙 | 매칭 중앙 | 1 m 넘는 오답 | 질의당 시간 중앙 (ms) |")
    L.append("|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|")
    for v, K in combos:
        s = summarize([r for r in rows if r["variant"] == v and r["K"] == K])
        L.append(f"| {VARIANT_KO.get(v, v)} | {K} | {s['n']} | {s['succ_rate']*100:.1f} % "
                 f"| {f2(s['pos_med'], 3)} / {f2(s['pos_p90'], 3)} | {f2(s['hor_med'], 3)} / {f2(s['hor_p90'], 3)} "
                 f"| {f2(s['rot_med'])} / {f2(s['rot_p90'])} | {s['inl_med']:.0f} | {s['match_med']:.0f} "
                 f"| {s['gross']} | {s['t_med']:.0f} |")
    sim = sim_reference()
    L.append(f"| *시뮬레이터 가정* | – | – | {sim['succ_rate']*100:.0f} % | – | {sim['hor_med']:.3f} / {sim['hor_p90']:.3f} "
             f"| {sim['yaw_med']:.1f} / {sim['yaw_p90']:.1f} (yaw만) | – | – | – | – |")
    L.append("")
    L.append("오차의 중앙 · 90 %는 성공한 질의만으로 낸다. 시뮬레이터 가정은 `sim/web/agent.js`의 "
             f"`VIS_P = {SIM_P}`, `VIS_M = {SIM_SIGMA_M}` (x · y 각각 표준편차), `VIS_DEG = {SIM_SIGMA_DEG:g}`에서 계산한 값이다.")
    L.append("")

    L.append("## 2. 정해진 오차 안에 든 비율 (모든 질의 기준, 실패는 밖으로 셈)")
    L.append("")
    L.append("| 변형 | K | 수평 ≤ 0.15 m | ≤ 0.30 m | ≤ 0.50 m | ≤ 1.0 m | ≤ 0.15 m 이고 회전 ≤ 3° |")
    L.append("|---|---:|---:|---:|---:|---:|---:|")
    for v, K in combos:
        s = summarize([r for r in rows if r["variant"] == v and r["K"] == K])
        L.append(f"| {VARIANT_KO.get(v, v)} | {K} | {s['r015']*100:.1f} % | {s['r030']*100:.1f} % | {s['r050']*100:.1f} % "
                 f"| {s['r100']*100:.1f} % | {s['r015_3']*100:.1f} % |")
    L.append(f"| *시뮬레이터 가정* | – | {sim['r015']*100:.1f} % | {sim['r030']*100:.1f} % | {sim['r050']*100:.1f} % "
             f"| {sim['r100']*100:.1f} % | {sim['r015_3']*100:.1f} % |")
    L.append("")

    L.append("## 3. 복도와 라운지 따로")
    L.append("")
    L.append("| 변형 | K | 구간 | 질의 | 성공률 | 수평 오차 중앙 / 90 % (m) | 회전 오차 중앙 (°) | 인라이어 중앙 | 1 m 넘는 오답 |")
    L.append("|---|---:|---|---:|---:|---:|---:|---:|---:|")
    for v, K in combos:
        for part in ("corridor", "lounge"):
            sub = [r for r in rows if r["variant"] == v and r["K"] == K and r["part"] == part]
            if not sub:
                continue
            s = summarize(sub)
            L.append(f"| {VARIANT_KO.get(v, v)} | {K} | {PART_KO[part]} | {s['n']} | {s['succ_rate']*100:.1f} % "
                     f"| {f2(s['hor_med'], 3)} / {f2(s['hor_p90'], 3)} | {f2(s['rot_med'])} | {s['inl_med']:.0f} | {s['gross']} |")
    L.append("")

    L.append("## 4. 지도와 매칭 규모")
    L.append("")
    L.append("| 변형 | K | 가장 가까운 남은 지도 카메라까지 중앙 (최소) (m) | 질의마다 남은 3D 점 (중앙) | 질의 사진의 SfM 대응 중 낮은 점 (중앙) | 매칭 (중앙) "
             "| 인라이어 (중앙) | SfM 대응과 같은 매칭 비율 (중앙) | 특징점 / 매칭 / PnP 시간 중앙 (ms) |")
    L.append("|---|---:|---:|---:|---:|---:|---:|---:|---:|")
    for v, K in combos:
        s = summarize([r for r in rows if r["variant"] == v and r["K"] == K])
        L.append(f"| {VARIANT_KO.get(v, v)} | {K} | {s['near_med']:.2f} ({s['near_min']:.2f}) | {s['map_pts_med']:,.0f} | {s['own_low_med']:.0f} | {s['match_med']:.0f} "
                 f"| {s['inl_med']:.0f} | {f2(s['agree_med'] * 100, 0)} % | {s['tf_med']:.0f} / {s['tm_med']:.0f} / {s['tp_med']:.0f} |")
    L.append("")
    L.append("'SfM 대응과 같은 매칭 비율'은 매칭된 3D 점이 복원 모델이 그 특징점에 붙인 3D 점과 같은 비율이다 "
             "(복원 모델이 3D 점을 붙이지 않은 특징점의 매칭은 틀린 것으로 센다).")
    L.append("")
    L.append(f"![수평 오차 누적 분포](error_cdf{args.tag}.png)")
    L.append("")
    L.append(f"질의마다의 수치는 `per_query{args.tag}.csv`에 있다.")
    L.append("")
    path_md.write_text("\n".join(L), encoding="utf-8")


def write_csv(rows, path: Path):
    keys = list(rows[0].keys())
    with open(path, "w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=keys)
        w.writeheader()
        for r in rows:
            w.writerow({k: (f"{v:.4f}" if isinstance(v, float) else v) for k, v in r.items()})


def plot_cdf(rows, path: Path):
    try:
        import matplotlib
    except ImportError:
        print("matplotlib not installed - no error_cdf plot", flush=True)
        return
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    names = {f.name for f in font_manager.fontManager.ttflist}
    # Jetson: the Noto CJK .ttc files are listed under their JP face name but carry Hangul too
    for fam in ("Malgun Gothic", "Noto Sans CJK KR", "NanumGothic", "Noto Sans CJK JP", "Noto Serif CJK JP"):
        if fam in names:
            plt.rcParams["font.family"] = fam
            break
    plt.rcParams["axes.unicode_minus"] = False
    ink, ink2, grid = "#0b0b0b", "#52514e", "#e4e3df"
    colors = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100"]          # categorical slots 1-4, fixed order
    variants = [v for v in ("full", "low") if any(r["variant"] == v for r in rows)]
    ks = sorted({r["K"] for r in rows})
    fig, axes = plt.subplots(1, len(variants), figsize=(5.2 * len(variants), 3.8), sharey=True, squeeze=False)
    x = np.linspace(0, 1.0, 401)
    s = SIM_SIGMA_M
    sim = SIM_P * (1 - np.exp(-x * x / (2 * s * s)))
    for ax, v in zip(axes[0], variants):
        for c, K in zip(colors, ks):
            e = np.array([r["hor_err_m"] if r["success"] else np.inf for r in rows if r["variant"] == v and r["K"] == K])
            y = (e[None, :] <= x[:, None]).mean(1)
            ax.plot(x, y * 100, color=c, lw=2, label=f"K = {K} (±{K / FPS:.0f} s 제외)")
        ax.plot(x, sim * 100, color=ink2, lw=1.5, ls="--", label="시뮬레이터 가정 (80 %, σ 0.15 m)")
        ax.set_title(VARIANT_KO[v] if v == "full" else "낮은 점만 (y < 0.8 m, 거친 대용)", color=ink, fontsize=11)
        ax.set_xlabel("수평 위치 오차 (m)", color=ink2)
        ax.set_xlim(0, 1.0); ax.set_ylim(0, 100)
        ax.grid(color=grid, lw=0.8); ax.set_axisbelow(True)
        for sp in ("top", "right"):
            ax.spines[sp].set_visible(False)
        for sp in ("left", "bottom"):
            ax.spines[sp].set_color(grid)
        ax.tick_params(colors=ink2, labelsize=9)
    axes[0][0].set_ylabel("오차 이하인 질의 비율 (%, 실패 포함)", color=ink2)
    axes[0][-1].legend(loc="lower right", fontsize=8, frameon=False, labelcolor=ink)
    fig.tight_layout()
    fig.savefig(path, dpi=110, facecolor="white")
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--work", type=Path, default=None, help="folder with sfm/, align.json, frames/")
    ap.add_argument("--features", choices=["db", "opencv"], default="db", help="query features (see above)")
    ap.add_argument("--sift-max-size", type=int, default=1600, help="opencv: longest image side for SIFT")
    ap.add_argument("--sift-features", type=int, default=5000, help="opencv: max SIFT features")
    ap.add_argument("--ks", default="0,5,15,40", help="leave-out half windows in frames (5 fps)")
    ap.add_argument("--variants", default="full,low,full_sfm", help=f"any of {', '.join(VARIANTS)}")
    ap.add_argument("--step", type=int, default=1, help="use every n-th frame as a query")
    ap.add_argument("--max-queries", type=int, default=0)
    ap.add_argument("--low-max-y", type=float, default=0.8, help="'low' variant: map height limit (m)")
    ap.add_argument("--ratio", type=float, default=0.8)
    ap.add_argument("--max-error", type=float, default=12.0, help="RANSAC reprojection threshold (px)")
    ap.add_argument("--min-inliers", type=int, default=30)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--chunk", type=int, default=256, help="query descriptors per distance block (256 x 250k x 4 B = 256 MB)")
    ap.add_argument("--out-dir", type=Path, default=HERE)
    ap.add_argument("--tag", default="", help="suffix for output file names, e.g. _cpu")
    args = ap.parse_args()
    work = find_work(args.work)
    args.model, args.database, args.align, args.frames = work / "sfm/model/1", work / "sfm/database.db", work / "align.json", work / "frames"
    print(f"data: {work}", flush=True)

    t0 = time.perf_counter()
    rows, meta = run(args)
    meta["t_total"] = time.perf_counter() - t0
    args.out_dir.mkdir(parents=True, exist_ok=True)
    write_csv(rows, args.out_dir / f"per_query{args.tag}.csv")
    plot_cdf(rows, args.out_dir / f"error_cdf{args.tag}.png")
    write_results(rows, meta, args, args.out_dir / f"results{args.tag}.md")
    print(f"done in {time.perf_counter() - t0:.0f} s -> {args.out_dir}", flush=True)


if __name__ == "__main__":
    main()
