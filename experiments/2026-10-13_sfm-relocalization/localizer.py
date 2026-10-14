"""카메라 사진 한 장 → 복도 지도 위 카메라 자세 (실차 · 다시 돌려 보기에서 같이 쓴다).

    loc = Localizer()                                   # 지도: --work · $HALLWAY_SFM · <repo>/data/hallway-sfm · F:\\...\\tools\\work
    r = loc.locate(bgr, cam, prior=(x, y, yaw_deg))     # prior 가 있으면 그 근처 지도 사진의 관측만 (빠르고 덜 헷갈린다)
    r["ok"], r["x"], r["y"], r["h"], r["yaw"], r["inliers"]

좌표는 장면 좌표 (시뮬레이터 · scene.json 과 같음): x 복도 안쪽, y 왼쪽, h 바닥에서 높이 (m), yaw 는 카메라가 보는 방향
(x 축에서 왼쪽 +, 도). 매칭은 관측 전부와 정확한 최근접 (relocalize.Matcher), 자세는 OpenCV PnP (colmap_lite.solve_pnp).
"""

from __future__ import annotations

import json
import math
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from colmap_lite import Camera, extract_sift, read_db, read_model, solve_pnp  # noqa: E402
from relocalize import Matcher, SfmMap, find_work, load_align  # noqa: E402

T_SCENE = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0.0]])    # 지도 (x 복도, y 위, z 오른쪽) → 장면 (x, y 왼쪽, z 위)


def pinhole(w, h, fx, fy=None, cx=None, cy=None):
    return Camera(0, "PINHOLE", w, h, np.array([fx, fy or fx, w / 2 if cx is None else cx, h / 2 if cy is None else cy]))


class Localizer:
    def __init__(self, work=None, radius_m=6.0, max_turn_deg=70.0, chunk=256):
        t0 = time.perf_counter()
        work = find_work(work)
        self.to_map, _ = load_align(work / "align.json")
        self.R_al = np.array(json.loads((work / "align.json").read_text(encoding="utf-8"))["rotation"], float)
        rec = read_model(work / "sfm/model/1")
        kps, descs = read_db(work / "sfm/database.db")
        sm = SfmMap(rec, kps, descs, self.to_map)
        self.xyz = sm.xyz
        self.obs_k = sm.obs_k                                     # 관측마다 지도 사진 번호
        self.matcher = Matcher(sm.obs_desc, sm.obs_pt, chunk)
        # 지도 사진마다 장면 좌표 위치 · 보는 방향
        self.img_xy = np.stack([sm.Cs_map[:, 0], -sm.Cs_map[:, 2]], 1)
        f = np.einsum("ij,kj->ki", self.R_al, np.einsum("kji,j->ki", sm.Rs, np.array([0, 0, 1.0])))
        self.img_yaw = np.degrees(np.arctan2(-f[:, 2], f[:, 0]))
        self.radius, self.max_turn = radius_m, max_turn_deg
        self.n_obs = len(sm.obs_pt)
        self.t_load = time.perf_counter() - t0

    def _valid(self, prior):
        if prior is None:
            return np.ones(self.n_obs, bool), len(self.img_xy)
        x, y, yaw = prior
        d = np.hypot(self.img_xy[:, 0] - x, self.img_xy[:, 1] - y)
        turn = np.abs((self.img_yaw - yaw + 180) % 360 - 180)
        imgs = (d < self.radius) & (turn < self.max_turn)
        return imgs[self.obs_k], int(imgs.sum())

    def locate(self, bgr, cam: Camera, prior=None, min_inliers=25, max_error=8.0, ratio=0.8, h_range=None):
        """h_range = (아래, 위) m 를 주면 카메라 높이가 그 밖인 답은 버린다 (차에 단 카메라가 바닥 밑 · 머리 위일 수는 없다)."""
        t0 = time.perf_counter()
        xy, d = extract_sift(bgr, max_size=1600, max_features=5000)
        valid, n_img = self._valid(prior)
        out = dict(ok=False, inliers=0, matches=0, n_kp=len(xy), map_images=n_img, prior=prior is not None)
        if len(xy) < 10 or n_img == 0:
            out["ms"] = (time.perf_counter() - t0) * 1e3
            return out
        pt, d1, ok, _ = self.matcher.match(d, valid, ratio)
        q = np.flatnonzero(ok)
        o = np.argsort(d1[q], kind="stable")
        _, first = np.unique(pt[q][o], return_index=True)          # 3D 점 하나에 질의 하나
        q = q[o[first]]
        out["matches"] = len(q)
        res = solve_pnp(xy[q], self.xyz[pt[q]], cam, max_error=max_error) if len(q) >= 6 else None
        if res:
            out["inliers"] = res["num_inliers"]
            C = -res["R"].T @ res["t"]
            m = self.to_map(C)[0]
            Rs = T_SCENE @ self.R_al @ res["R"].T                    # 카메라 축 (오른쪽 · 아래 · 앞) 을 장면 좌표로
            f = Rs[:, 2]
            out.update(x=float(m[0]), y=float(-m[2]), h=float(m[1]), yaw=math.degrees(math.atan2(f[1], f[0])),
                       pitch=math.degrees(math.asin(max(-1.0, min(1.0, f[2])))), R=Rs.tolist(),
                       ok=res["num_inliers"] >= min_inliers and (h_range is None or h_range[0] <= m[1] <= h_range[1]))
        out["ms"] = (time.perf_counter() - t0) * 1e3
        return out
