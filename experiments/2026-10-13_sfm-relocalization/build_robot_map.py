"""차 높이 웹캠 녹화로 양방향 SfM 지도를 새로 만든다 (기존 지도는 사람 손 1.37 m · 한 방향뿐).

    python build_robot_map.py <작업 폴더> [--step features,pairs,match,map,align]
        <작업 폴더>/images/<녹화>_<프레임>.jpg   웹캠 사진 (녹화마다 2.5 Hz 등, 이름 앞부분이 녹화)
        → database.db · pairs.txt · model/ (만든 그대로) · aligned/ (장면 좌표) · align.json · build.log.json

쌍: 같은 녹화의 앞뒤 사진 (+1 ~ +6, +9, +12, +18) 과, SIFT 단어 묶음 (k-means 1024 단어 · tf-idf) 으로 고른 비슷한 사진
(다른 녹화 · 같은 녹화에서 되돌아온 곳, 사진마다 위 10 장). vocab tree 파일 없이 녹화끼리 잇는다.
지도: pycolmap global_mapping (CPU — 이 PC 의 pycolmap 은 CUDA 없음). 좌표: 다시 돌려 보기 (replay.csv) 에서 기존 지도로
위치를 찾은 사진의 카메라 중심과 Sim(3) 을 맞춰 (RANSAC + Umeyama) 장면 좌표 (x 복도 안쪽, y 왼쪽, z 위) 로 옮긴다.
파이썬: pycolmap 이 있는 환경 (F:\\Develop\\mnu-campus-tour\\.venv). 시뮬레이터 (WebGL) 와 무거운 그래픽 작업은 같이 돌리지 않는다.
"""

import argparse
import csv
import json
import math
import re
import sqlite3
import sys
import time
from pathlib import Path

import numpy as np
import pycolmap

RUNS = Path(r"F:\Develop\NUBI\data\logs\hallway-run")


def log(work, key, val):
    p = work / "build.log.json"
    d = json.loads(p.read_text(encoding="utf-8")) if p.exists() else {}
    d[key] = val
    p.write_text(json.dumps(d, ensure_ascii=False, indent=2), encoding="utf-8")
    print(key, json.dumps(val, ensure_ascii=False), flush=True)


def run_of(name):
    m = re.match(r"(\d+)_(\d+)\.jpg", name)
    return m.group(1), int(m.group(2))


def step_features(work, a):
    db = work / "database.db"
    if db.exists():
        db.unlink()
    t0 = time.time()
    reader = pycolmap.ImageReaderOptions()
    reader.camera_model = "SIMPLE_RADIAL"
    f = (1280 / 2) / math.tan(math.radians(70) / 2)              # 웹캠 가로 화각 약 70° (2026-10-14 추정)
    reader.camera_params = f"{f:.1f},640,360,0"
    ext = pycolmap.FeatureExtractionOptions()
    ext.use_gpu = False
    ext.max_image_size = 1600
    ext.sift.max_num_features = a.features
    pycolmap.extract_features(db, work / "images", camera_mode=pycolmap.CameraMode.SINGLE,
                              reader_options=reader, extraction_options=ext)
    log(work, "features", dict(seconds=round(time.time() - t0), max_num_features=a.features, focal_init=round(f, 1)))


def read_descs(db):
    con = sqlite3.connect(f"file:{db.as_posix()}?mode=ro", uri=True)
    names = dict(con.execute("SELECT image_id, name FROM images"))
    D = {}
    for iid, rows, cols, blob in con.execute("SELECT image_id, rows, cols, data FROM descriptors"):
        D[names[iid]] = np.frombuffer(blob, np.uint8).reshape(rows, cols) if blob else np.zeros((0, 128), np.uint8)   # 특징점 없는 사진
    con.close()
    return D


def step_pairs(work, a):
    t0 = time.time()
    D = read_descs(work / "database.db")
    names = sorted(D)
    rng = np.random.default_rng(0)
    sample = np.concatenate([d[rng.choice(len(d), min(len(d), 200), replace=False)] for d in D.values() if len(d)]).astype(np.float32)
    k = a.words
    C = sample[rng.choice(len(sample), k, replace=False)].copy()
    for _ in range(12):                                           # k-means (Lloyd), 표본 위에서
        lab = np.concatenate([np.argmin((C * C).sum(1)[None] - 2 * sample[i:i + 20000] @ C.T, 1)
                              for i in range(0, len(sample), 20000)])
        for j in range(k):
            m = lab == j
            if m.any():
                C[j] = sample[m].mean(0)
    Cn = (C * C).sum(1)
    H = np.zeros((len(names), k), np.float32)
    for r, n in enumerate(names):
        d = D[n].astype(np.float32)
        if len(d):
            H[r] = np.bincount(np.argmin(Cn[None] - 2 * d @ C.T, 1), minlength=k)
    idf = np.log(len(names) / np.maximum(1, (H > 0).sum(0)))
    V = H * idf
    V /= np.maximum(1e-9, np.linalg.norm(V, axis=1, keepdims=True))
    S = V @ V.T
    run_frame = [run_of(n) for n in names]
    pairs = set()
    by_run = {}
    for r, (run, fr) in enumerate(run_frame):
        by_run.setdefault(run, []).append(r)
    for run, idx in by_run.items():                               # 같은 녹화의 앞뒤
        idx = sorted(idx, key=lambda r: run_frame[r][1])
        for p, r in enumerate(idx):
            for d in (1, 2, 3, 4, 5, 6, 9, 12, 18):
                if p + d < len(idx):
                    pairs.add((min(r, idx[p + d]), max(r, idx[p + d])))
    n_seq = len(pairs)
    for r in range(len(names)):                                   # 비슷한 사진 (같은 녹화의 가까운 앞뒤는 빼고)
        run, fr = run_frame[r]
        order = np.argsort(-S[r])
        got = 0
        for q in order:
            if q == r:
                continue
            rq, fq = run_frame[q]
            if rq == run and abs(fq - fr) < 60:                   # 같은 녹화 12 s (2.5 Hz 사진 30 장) 안은 이미 앞뒤로
                continue
            pairs.add((min(r, q), max(r, q)))
            got += 1
            if got >= a.retrieval:
                break
    with open(work / "pairs.txt", "w", encoding="utf-8") as fh:
        for i, j in sorted(pairs):
            fh.write(f"{names[i]} {names[j]}\n")
    log(work, "pairs", dict(images=len(names), sequential=n_seq, retrieval=len(pairs) - n_seq, total=len(pairs),
                            words=k, seconds=round(time.time() - t0)))


def step_match(work, a):
    t0 = time.time()
    mo = pycolmap.FeatureMatchingOptions()
    mo.use_gpu = False
    po = pycolmap.ImportedPairingOptions()
    po.match_list_path = str(work / "pairs.txt")
    pycolmap.match_image_pairs(work / "database.db", matching_options=mo, pairing_options=po)
    con = sqlite3.connect(f"file:{(work / 'database.db').as_posix()}?mode=ro", uri=True)
    n_ok = con.execute("SELECT COUNT(*) FROM two_view_geometries WHERE rows >= 15").fetchone()[0]
    con.close()
    log(work, "match", dict(seconds=round(time.time() - t0), verified_pairs=n_ok))


def step_map(work, a):
    t0 = time.time()
    out = work / "model"
    out.mkdir(exist_ok=True)
    opts = pycolmap.GlobalPipelineOptions()
    opts.min_model_size = 20
    recs = pycolmap.global_mapping(work / "database.db", work / "images", out, opts)
    info = []
    for kk, rec in recs.items():
        runs = {}
        for im in rec.images.values():
            runs[run_of(im.name)[0]] = runs.get(run_of(im.name)[0], 0) + 1
        info.append(dict(model=kk, images=rec.num_reg_images(), points=len(rec.points3D), runs=runs,
                         reproj_px=round(rec.compute_mean_reprojection_error(), 2),
                         camera=str(next(iter(rec.cameras.values())))))
    log(work, "map", dict(seconds=round(time.time() - t0), models=info))


def step_imap(work, a):
    """순서대로 쌓기 (incremental) — 전역 방식 (GLOMAP) 은 긴 복도를 앞으로만 가는 영상에서 돌아오는 방향을 짓눌렀다 (2026-10-15)."""
    t0 = time.time()
    out = work / "model_inc"
    out.mkdir(exist_ok=True)
    o = pycolmap.IncrementalPipelineOptions()
    o.multiple_models = True
    o.min_model_size = 20
    o.ba_use_gpu = False
    o.ba_global_frames_ratio = 1.3                                # 전체 묶음 조정을 덜 자주 (CPU)
    o.ba_global_points_ratio = 1.3
    o.ba_global_max_num_iterations = 30
    o.max_runtime_seconds = int(a.max_minutes * 60)
    recs = pycolmap.incremental_mapping(work / "database.db", work / "images", out, o)
    info = []
    for kk, rec in recs.items():
        runs = {}
        for im in rec.images.values():
            runs[run_of(im.name)[0]] = runs.get(run_of(im.name)[0], 0) + 1
        info.append(dict(model=kk, images=rec.num_reg_images(), points=len(rec.points3D), runs=runs,
                         reproj_px=round(rec.compute_mean_reprojection_error(), 2),
                         camera=str(next(iter(rec.cameras.values())))))
    log(work, "imap", dict(seconds=round(time.time() - t0), models=info))


def anchors():
    """다시 돌려 보기에서 기존 지도로 찾은 사진 → 장면 좌표 카메라 중심 {이름: (x, y, h)}."""
    out = {}
    for rp in sorted(RUNS.glob("*/replay.csv")):
        run = rp.parent.name.split("_")[1]
        frames = [(float(r["t"]), int(r["i"])) for r in csv.DictReader(open(rp.parent / "frames.csv", encoding="utf-8"))]
        ft = np.array([t for t, _ in frames])
        for r in csv.DictReader(open(rp, encoding="utf-8")):
            if r["cam_ok"] != "1" or not r["cam_x"]:
                continue
            k = int(np.argmin(np.abs(ft - float(r["t"]))))
            i = frames[k][1]
            for ii in (i, i - 1, i + 1):                          # 2.5 Hz 로 골라 왔으니 짝수 프레임만 있다
                out[f"{run}_{ii:06d}.jpg"] = (float(r["cam_x"]), float(r["cam_y"]), float(r["cam_h"]))
    return out


def umeyama(src, dst):
    mu_s, mu_d = src.mean(0), dst.mean(0)
    A, B = src - mu_s, dst - mu_d
    U, S, Vt = np.linalg.svd(B.T @ A / len(src))
    E = np.eye(3)
    if np.linalg.det(U @ Vt) < 0:
        E[2, 2] = -1
    R = U @ E @ Vt
    s = np.trace(np.diag(S) @ E) / A.var(0).sum()
    return s, R, mu_d - s * R @ mu_s


def step_align(work, a):
    dirs = [d for d in (work / a.model_dir).iterdir() if d.is_dir() and (d / "images.bin").exists()]
    best = max(dirs, key=lambda d: pycolmap.Reconstruction(str(d)).num_reg_images())
    rec = pycolmap.Reconstruction(str(best))
    anc = anchors()
    pairs = [(np.array(im.projection_center()), np.array(anc[im.name])) for im in rec.images.values() if im.name in anc]
    src, dst = np.array([p[0] for p in pairs]), np.array([p[1] for p in pairs])
    rng = np.random.default_rng(0)
    bestin = None
    for _ in range(2000):
        idx = rng.choice(len(src), 3, replace=False)
        s, R, t = umeyama(src[idx], dst[idx])
        err = np.linalg.norm((s * src @ R.T + t) - dst, axis=1)
        inl = err < a.align_tol
        if bestin is None or inl.sum() > bestin.sum():
            bestin = inl
    s, R, t = umeyama(src[bestin], dst[bestin])
    err = np.linalg.norm((s * src @ R.T + t) - dst, axis=1)
    rec.transform(pycolmap.Sim3d(s, pycolmap.Rotation3d(R), t))
    out = work / ("aligned" if a.model_dir == "model" else f"aligned_{a.model_dir}")
    out.mkdir(exist_ok=True)
    rec.write(str(out))
    # localizer 는 align.json 으로 '지도 좌표 (x, 위, 오른쪽)' 를 만든 뒤 장면으로 바꾼다 — 장면 좌표 모델이면 축만 돌린다
    (work / "align.json").write_text(json.dumps(dict(
        rotation=[[1, 0, 0], [0, 0, 1], [0, -1, 0]], scale=1.0, x_offset_units=0.0, floor_offset=0.0,
        corridor_z_center_units=0.0, x_origin_m=0.0, note="aligned/ 는 장면 좌표 (x 복도 안쪽, y 왼쪽, z 위)"), indent=1),
        encoding="utf-8")
    hs = np.array([np.array(im.projection_center())[2] for im in rec.images.values()])
    log(work, f"align_{a.model_dir}", dict(model=best.name, anchors=len(src), inliers=int(bestin.sum()), scale=round(float(s), 4),
                            err_median_m=round(float(np.median(err[bestin])), 3), err_p90_m=round(float(np.percentile(err[bestin], 90)), 3),
                            cam_height_median_m=round(float(np.median(hs)), 3)))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("work", type=Path)
    ap.add_argument("--step", default="features,pairs,match,map,align")
    ap.add_argument("--features", type=int, default=4000)
    ap.add_argument("--words", type=int, default=1024)
    ap.add_argument("--retrieval", type=int, default=10)
    ap.add_argument("--align-tol", type=float, default=0.5)
    ap.add_argument("--model-dir", default="model", help="align 할 모델 폴더 (model = 전역, model_inc = 순서대로)")
    ap.add_argument("--max-minutes", type=float, default=120, help="imap 최대 시간")
    a = ap.parse_args()
    for st in a.step.split(","):
        print(f"== {st}", flush=True)
        {"features": step_features, "pairs": step_pairs, "match": step_match, "map": step_map, "imap": step_imap,
         "align": step_align}[st](a.work, a)


if __name__ == "__main__":
    sys.exit(main())
