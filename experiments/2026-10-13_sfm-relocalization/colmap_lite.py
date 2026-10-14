"""COLMAP 지도를 pycolmap 없이 읽고 (numpy), 자세는 OpenCV 로 푼다 — Jetson (aarch64) 에는 pycolmap 휠이 없다.

    model = read_model("sfm/model/1")          cameras.bin · images.bin · points3D.bin
    kps, descs = read_db("sfm/database.db")    이미지 이름 → 특징점 (x, y) · SIFT 설명자 (uint8)
    xy, desc = extract_sift(bgr)               OpenCV SIFT 를 COLMAP 과 같은 꼴로 (좌표 관례 · RootSIFT uint8)
    res = solve_pnp(xy, X, camera)             RANSAC PnP + LM 다듬기 → R, t (cam_from_world), 인라이어

좌표 관례는 COLMAP 을 따른다: 왼쪽 위 픽셀의 중심이 (0.5, 0.5). DB 의 특징점과 카메라 값 (cx, cy) 이 이 관례라서
OpenCV 로 뽑은 특징점도 +0.5 해서 맞춘다. 자세는 cam_from_world (X_cam = R · X_world + t).
"""

from __future__ import annotations

import sqlite3
import struct
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

# COLMAP 카메라 모델 id → (이름, 값 개수). 이 지도는 SIMPLE_RADIAL 만 쓴다.
CAMERA_MODELS = {0: ("SIMPLE_PINHOLE", 3), 1: ("PINHOLE", 4), 2: ("SIMPLE_RADIAL", 4), 3: ("RADIAL", 5)}


@dataclass
class Camera:
    id: int
    model: str
    width: int
    height: int
    params: np.ndarray

    def K(self) -> np.ndarray:
        p = self.params
        fx, fy, cx, cy = (p[0], p[0], p[1], p[2]) if self.model != "PINHOLE" else (p[0], p[1], p[2], p[3])
        return np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1.0]])

    def dist(self) -> np.ndarray:
        """OpenCV 왜곡 계수 (k1, k2, p1, p2) — SIMPLE_RADIAL 의 k 는 OpenCV k1 과 같은 식이다."""
        p = self.params
        if self.model == "SIMPLE_RADIAL":
            return np.array([p[3], 0, 0, 0.0])
        if self.model == "RADIAL":
            return np.array([p[3], p[4], 0, 0.0])
        return np.zeros(4)

    def cam_from_img(self, xy: np.ndarray) -> np.ndarray:
        """픽셀 → 왜곡을 푼 정규 좌표 (z = 1 평면). 반지름 왜곡만 있으니 고정점 반복으로 푼다."""
        K, (k1, k2, _, _) = self.K(), self.dist()
        ud = (np.asarray(xy, float) - K[:2, 2]) / [K[0, 0], K[1, 1]]
        u = ud.copy()
        for _ in range(30):
            r2 = (u * u).sum(1, keepdims=True)
            u = ud / (1.0 + k1 * r2 + k2 * r2 * r2)
        return u

    def __str__(self):
        return f"{self.model} {self.width}x{self.height} " + " ".join(f"{v:.4g}" for v in self.params)


@dataclass
class Image:
    id: int
    name: str
    camera_id: int
    R: np.ndarray                 # cam_from_world 회전 (3, 3)
    t: np.ndarray                 # cam_from_world 이동 (3,)
    xy: np.ndarray                # 특징점 (n, 2)
    p3d: np.ndarray               # 특징점마다 3D 점 id, 없으면 -1 (n,)

    @property
    def center(self) -> np.ndarray:
        return -self.R.T @ self.t


@dataclass
class Model:
    cameras: dict = field(default_factory=dict)
    images: dict = field(default_factory=dict)
    point_ids: np.ndarray = None  # (P,) 오름차순
    xyz: np.ndarray = None        # (P, 3)
    track_off: np.ndarray = None  # (P + 1,) 점 i 의 관측 = track_*[off[i]:off[i+1]]
    track_img: np.ndarray = None
    track_p2d: np.ndarray = None


def quat_to_R(qw, qx, qy, qz) -> np.ndarray:
    n = np.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
    w, x, y, z = qw / n, qx / n, qy / n, qz / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


class _Reader:
    def __init__(self, path: Path):
        self.b = Path(path).read_bytes()
        self.i = 0

    def take(self, fmt: str):
        v = struct.unpack_from("<" + fmt, self.b, self.i)
        self.i += struct.calcsize("<" + fmt)
        return v

    def array(self, dtype, n: int):
        a = np.frombuffer(self.b, dtype, n, self.i)
        self.i += a.nbytes
        return a

    def cstr(self) -> str:
        j = self.b.index(b"\0", self.i)
        s = self.b[self.i:j].decode("utf-8")
        self.i = j + 1
        return s


P2D = np.dtype([("x", "<f8"), ("y", "<f8"), ("id", "<i8")])   # 3D 점 없음 = uint64 최댓값 → int64 -1
TRACK = np.dtype([("img", "<u4"), ("p2d", "<u4")])


def read_model(folder) -> Model:
    folder = Path(folder)
    m = Model()
    r = _Reader(folder / "cameras.bin")
    for _ in range(r.take("Q")[0]):
        cid, mid, w, h = r.take("iiQQ")
        if mid not in CAMERA_MODELS:
            raise SystemExit(f"카메라 모델 id {mid} 는 아직 못 읽는다")
        name, n = CAMERA_MODELS[mid]
        m.cameras[cid] = Camera(cid, name, w, h, np.array(r.take("d" * n)))
    r = _Reader(folder / "images.bin")
    for _ in range(r.take("Q")[0]):
        iid, qw, qx, qy, qz, tx, ty, tz, cid = r.take("I7dI")
        name = r.cstr()
        p = r.array(P2D, r.take("Q")[0])
        if not np.isfinite([qw, qx, qy, qz, tx, ty, tz]).all():
            continue                                        # 자세 없는 (등록 안 된) 사진
        m.images[iid] = Image(iid, name, cid, quat_to_R(qw, qx, qy, qz), np.array([tx, ty, tz]),
                              np.stack([p["x"], p["y"]], 1), p["id"].copy())
    r = _Reader(folder / "points3D.bin")
    n = r.take("Q")[0]
    ids, xyz, lens, tracks = np.empty(n, np.int64), np.empty((n, 3)), np.empty(n, np.int64), []
    for k in range(n):
        pid, x, y, z = r.take("Q3d")
        r.take("3Bd")                                       # 색 · 재투영 오차
        tl = r.take("Q")[0]
        ids[k], xyz[k], lens[k] = pid, (x, y, z), tl
        tracks.append(r.array(TRACK, tl))
    order = np.argsort(ids, kind="stable")
    tracks = [tracks[k] for k in order]
    m.point_ids, m.xyz, lens = ids[order], xyz[order], lens[order]
    m.track_off = np.concatenate([[0], np.cumsum(lens)])
    tr = np.concatenate(tracks) if tracks else np.empty(0, TRACK)
    m.track_img, m.track_p2d = tr["img"].astype(np.int64), tr["p2d"].astype(np.int64)
    return m


def read_db(path):
    """특징점 (x, y) 과 SIFT 설명자 (uint8, 128) 를 이미지 이름별로. 읽기 전용으로 연다."""
    con = sqlite3.connect(f"file:{Path(path).as_posix()}?mode=ro&immutable=1", uri=True)
    names = dict(con.execute("SELECT image_id, name FROM images"))
    kps, descs = {}, {}
    for iid, rows, cols, blob in con.execute("SELECT image_id, rows, cols, data FROM keypoints"):
        kps[names[iid]] = np.frombuffer(blob, np.float32).reshape(rows, cols)[:, :2].astype(np.float64)
    for iid, rows, cols, blob in con.execute("SELECT image_id, rows, cols, data FROM descriptors"):
        if cols != 128:
            raise SystemExit(f"설명자 길이 {cols} (128 이어야)")
        descs[names[iid]] = np.frombuffer(blob, np.uint8).reshape(rows, cols)
    con.close()
    return kps, descs


def rootsift_uint8(desc: np.ndarray) -> np.ndarray:
    """OpenCV SIFT 설명자 → COLMAP 이 DB 에 넣는 꼴 (L1 정규화 → 제곱근 → ×512, 255 에서 자름)."""
    d = np.asarray(desc, np.float32)
    d = d / np.maximum(d.sum(1, keepdims=True), 1e-12)
    return np.minimum(255.0, np.round(512.0 * np.sqrt(d))).astype(np.uint8)


def extract_sift(bgr_or_gray, max_size=1600, max_features=5000):
    """지도를 만들 때 (pycolmap 기본값, max_image_size 1600 · 특징 5000) 와 비슷하게 뽑는다."""
    import cv2
    g = bgr_or_gray if bgr_or_gray.ndim == 2 else cv2.cvtColor(bgr_or_gray, cv2.COLOR_BGR2GRAY)
    h, w = g.shape
    s = min(1.0, max_size / max(h, w))
    small = g if s == 1.0 else cv2.resize(g, (round(w * s), round(h * s)), interpolation=cv2.INTER_AREA)
    # contrastThreshold 0.02 = COLMAP 기본 peak_threshold (0.02 / 3 층) 을 OpenCV 식으로 옮긴 값 (OpenCV 기본 0.04)
    kp, desc = cv2.SIFT_create(nfeatures=max_features, contrastThreshold=0.02).detectAndCompute(small, None)
    if desc is None or not kp:
        return np.zeros((0, 2)), np.zeros((0, 128), np.uint8)
    sx, sy = w / small.shape[1], h / small.shape[0]
    xy = (np.array([k.pt for k in kp], float) + 0.5) * [sx, sy]     # OpenCV 관례 → COLMAP 관례, 원본 크기로
    return xy, rootsift_uint8(desc)


def solve_pnp(xy, X, camera: Camera, max_error=12.0, iters=10000, confidence=0.9999, seed=0, min_inliers=4):
    """RANSAC PnP (AP3P) → 인라이어로 LM 다듬기 → 인라이어 다시 세기 → 한 번 더 다듬기.
    돌려주는 것: dict(R, t, num_inliers, inliers) 또는 None. R, t 는 cam_from_world."""
    import cv2
    xy = np.ascontiguousarray(xy, np.float64)
    X = np.ascontiguousarray(X, np.float64)
    if len(xy) < 4:
        return None
    K, dist = camera.K(), camera.dist()
    cv2.setRNGSeed(int(seed))
    ok, rvec, tvec, inl = cv2.solvePnPRansac(X, xy, K, dist, iterationsCount=iters, reprojectionError=max_error,
                                             confidence=confidence, flags=cv2.SOLVEPNP_AP3P)
    if not ok or inl is None or len(inl) < min_inliers:
        return None

    def inliers(rvec, tvec):
        proj, _ = cv2.projectPoints(X, rvec, tvec, K, dist)
        R = cv2.Rodrigues(rvec)[0]
        z = X @ R[2] + tvec[2, 0]
        return (np.linalg.norm(proj[:, 0] - xy, axis=1) <= max_error) & (z > 0)

    m = np.zeros(len(xy), bool)
    m[inl[:, 0]] = True
    for _ in range(2):
        if m.sum() < min_inliers:
            return None
        rvec, tvec = cv2.solvePnPRefineLM(X[m], xy[m], K, dist, rvec, tvec)
        m = inliers(rvec, tvec)
    if m.sum() < min_inliers:
        return None
    return dict(R=cv2.Rodrigues(rvec)[0], t=tvec[:, 0].copy(), num_inliers=int(m.sum()), inliers=m)
