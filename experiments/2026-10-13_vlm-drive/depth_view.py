"""깊이 영상 → 바닥면 · 방향별 빈 거리 · 가까운 사각 · 화살표.

규칙은 시뮬레이터와 같다 (hardware/vehicle/models/vehicle/sim/web/sensors.js castDepth · agent.js candidates):
  · 바닥에서 5 cm ~ 차 높이 + 15 cm (0.68 m) 사이 점만 장애물 (더 높은 것은 밑으로 지나간다)
  · 세로줄 (각도 칸) 마다 가장 가까운 장애물까지 수평 거리 → 화살표 방향 띠 (차 반폭 + 여유 0.32 m) 에 비춰 빈 거리
Xtion 은 약 0.5 m 안쪽이 0 (값 없음) 이라, 바닥이 0.65 ~ 2 m 에 보여야 할 줄이 비어 있으면 '가까이 무언가 있다' (사각)
로 보고 그 방향을 막힘 (0.3 m) 으로 친다. 반짝이는 검은 바닥도 비어 보일 수 있다 — 그때는 보수적으로 선다.

좌표: 카메라 (x 오른쪽, y 아래, z 앞) → 바닥 기준 (X 앞, Y 왼쪽, Z 위, 카메라 바로 아래 바닥 = 원점).
카메라 높이 · 기울기는 시작할 때 바닥면을 맞춰 잰다 (fit_floor). 실물 장착 위치를 몰라도 된다.
"""

import base64
import math
from dataclasses import dataclass

import numpy as np

ARROWS_XTION = [24, 16, 8, 0, -8, -16, -24]       # 도, + = 왼쪽. 화각 58° 안쪽
HALF_W = 0.32
Z_LO, Z_HI = 0.05, 0.68
NEAR, FAR = 0.5, 4.0
BAND = (0.65, 2.0)                                # 바닥이 보여야 할 거리 (사각 판정)


@dataclass
class Ground:
    h: float                 # 카메라 높이 (m)
    up: np.ndarray           # 카메라 좌표의 위 방향
    fwd: np.ndarray          # 바닥에 눕힌 앞 방향
    left: np.ndarray
    pitch_deg: float         # + = 아래로 숙임
    roll_deg: float
    inlier_frac: float


def ground_from(h, pitch_deg=0.0, roll_deg=0.0):
    """재지 않고 정한 값으로 (가짜 카메라 · 설정 파일)."""
    p, r = math.radians(pitch_deg), math.radians(roll_deg)
    up = np.array([math.sin(r), -math.cos(r) * math.cos(p), -math.cos(r) * math.sin(p)])
    return _axes(h, up, 1.0)


def _axes(h, up, frac):
    up = up / np.linalg.norm(up)
    z = np.array([0.0, 0.0, 1.0])
    fwd = z - (z @ up) * up
    fwd /= np.linalg.norm(fwd)
    left = np.cross(up, fwd)
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, -(z @ up)))))
    roll = math.degrees(math.atan2(up[0], -up[1]))
    return Ground(float(h), up, fwd, left, pitch, roll, frac)


def back_project(depth, K, step=2, rows=None):
    fx, fy, cx, cy = K
    d = depth[::step, ::step]
    v, u = np.mgrid[0:depth.shape[0]:step, 0:depth.shape[1]:step]
    if rows is not None:
        keep = v >= rows
        d, u, v = d[keep], u[keep], v[keep]
    ok = d > 0
    d, u, v = d[ok], u[ok], v[ok]
    return np.stack([(u - cx) / fx * d, (v - cy) / fy * d, d], axis=-1)


def fit_floor(depth, K, iters=300, tol=0.02, seed=0):
    """아래쪽 45 % 줄의 점으로 RANSAC 평면 → 카메라 높이 · 기울기. 바닥이 안 보이면 None."""
    pts = back_project(depth, K, step=4, rows=int(depth.shape[0] * 0.55))
    if len(pts) < 200:
        return None
    rng = np.random.default_rng(seed)
    best, best_n = None, 0
    for _ in range(iters):
        a, b, c = pts[rng.choice(len(pts), 3, replace=False)]
        n = np.cross(b - a, c - a)
        if np.linalg.norm(n) < 1e-9:
            continue
        n /= np.linalg.norm(n)
        if abs(n[1]) < 0.8:                     # 바닥은 카메라 y (아래) 축과 거의 평행한 법선
            continue
        dist = np.abs((pts - a) @ n)
        cnt = int((dist < tol).sum())
        if cnt > best_n:
            best, best_n = (n, a), cnt
    if best is None or best_n < 0.3 * len(pts):
        return None
    n, a = best
    inl = pts[np.abs((pts - a) @ n) < tol]
    c = inl.mean(axis=0)
    n = np.linalg.svd(inl - c)[2][-1]
    d = -n @ c
    if d < 0:
        n, d = -n, -d                           # 원점 (카메라) 쪽이 + → n 이 위
    if not (0.05 < d < 1.5) or -n[1] < math.cos(math.radians(35)):
        return None
    return _axes(d, n, best_n / len(pts))


def to_level(pts, g):
    return pts @ g.fwd, pts @ g.left, g.h + pts @ g.up


class DepthAnalyzer:
    """화각 · 바닥이 정해지면 줄마다의 광선 (사각 판정 띠) 을 한 번만 계산해 둔다."""

    def __init__(self, shape, K, g, ncol=40, step=2):
        self.K, self.g, self.ncol, self.step = K, g, ncol, step
        fx, fy, cx, cy = K
        self.half = math.atan((shape[1] / 2) / fx)
        v, u = np.mgrid[0:shape[0]:step, 0:shape[1]:step]
        r = np.stack([(u - cx) / fx, (v - cy) / fy, np.ones_like(u, dtype=float)], axis=-1)
        down = -(r @ g.up)                          # 바닥 쪽 성분
        with np.errstate(divide="ignore", invalid="ignore"):
            t = np.where(down > 1e-6, g.h / down, np.inf)
        horiz = t * np.hypot(r @ g.fwd, r @ g.left)
        self.band = (horiz >= BAND[0]) & (horiz <= BAND[1])
        self.pix_bin = self._bin(np.arctan2(r @ g.left, r @ g.fwd))

    def _bin(self, ang):
        return np.clip(((ang + self.half) / (2 * self.half) * self.ncol).astype(int), 0, self.ncol - 1)

    def analyze(self, depth, blind_frac=0.6):
        s = self.step
        d = depth[::s, ::s]
        pts = back_project(depth, self.K, step=s)
        X, Y, Z = to_level(pts, self.g)
        obst = (Z > Z_LO) & (Z < Z_HI) & (X > 0.05)
        X, Y = X[obst], Y[obst]
        ang = np.arctan2(Y, X)
        rng = np.hypot(X, Y)
        free = np.full(self.ncol, FAR)
        b = self._bin(ang)
        np.minimum.at(free, b, rng)
        # 사각: 바닥이 보여야 할 띠에서 값 없는 (0) 픽셀 비율
        band_bins = self.pix_bin[self.band]
        empty = (d[self.band] <= 0)
        tot = np.bincount(band_bins, minlength=self.ncol)
        emp = np.bincount(band_bins, weights=empty, minlength=self.ncol)
        blind = (tot > 0) & (emp > blind_frac * np.maximum(tot, 1))
        edges = np.linspace(-self.half, self.half, self.ncol + 1)
        cols = [{"ang": float((edges[i] + edges[i + 1]) / 2), "free": float(min(free[i], FAR)), "blind": bool(blind[i])}
                for i in range(self.ncol)]
        band_w = math.atan(HALF_W / BAND[0])
        center_blind = any(c["blind"] for c in cols if abs(c["ang"]) < band_w)
        ahead = X[np.abs(Y) < HALF_W]
        gap = 0.0 if center_blind else (float(ahead.min()) if len(ahead) else FAR)
        return {"cols": cols, "gap_ahead": gap, "center_blind": center_blind,
                "valid_frac": float((d > 0).mean())}


def candidates(cols, arrows=ARROWS_XTION):
    out = []
    for k, deg in enumerate(arrows):
        th = math.radians(deg)
        free, blind = FAR, False
        for c in cols:
            if c["free"] < FAR:
                along = c["free"] * math.cos(c["ang"] - th)
                lat = c["free"] * math.sin(c["ang"] - th)
                if along > 0 and abs(lat) < HALF_W:
                    free = min(free, along)
            if c["blind"] and abs(c["ang"] - th) <= math.atan(HALF_W / BAND[0]):
                blind = True
        if blind:
            free = min(free, 0.3)
        out.append({"id": k + 1, "deg": deg, "free": round(free, 1), "blind": blind})
    return out


def draw_arrows(bgr, cands, K, g, show=3.0):
    import cv2
    img = bgr.copy()
    h, w = img.shape[:2]
    fx, fy, cx, cy = K
    s = w / 640                                         # 640 폭 기준 크기
    for c in cands:
        th = math.radians(c["deg"])
        r = min(max(c["free"], 0.4), show)
        p = r * math.cos(th) * g.fwd + r * math.sin(th) * g.left - g.h * g.up
        u, v = fx * p[0] / p[2] + cx, fy * p[1] / p[2] + cy
        u, v = int(round(min(max(u, 0), w - 1))), int(round(min(max(v, 0), h - 3)))
        col = (74, 191, 47) if c["free"] >= 1 else (0, 140, 240) if c["free"] >= 0.5 else (49, 49, 224)
        cv2.line(img, (w // 2, h - 2), (u, v), (0, 0, 0), int(5 * s), cv2.LINE_AA)
        cv2.line(img, (w // 2, h - 2), (u, v), col, int(3 * s), cv2.LINE_AA)
        cv2.circle(img, (u, v), int(12 * s), col, -1, cv2.LINE_AA)
        cv2.circle(img, (u, v), int(12 * s), (255, 255, 255), max(1, int(2 * s)), cv2.LINE_AA)
        txt = str(c["id"])
        (tw, th_), _ = cv2.getTextSize(txt, cv2.FONT_HERSHEY_SIMPLEX, 0.55 * s, 2)
        cv2.putText(img, txt, (u - tw // 2, v + th_ // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.55 * s, (255, 255, 255), 2, cv2.LINE_AA)
    return img


def jpeg_data_url(bgr, quality=85):
    import cv2
    ok, buf = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, quality])
    assert ok
    return "data:image/jpeg;base64," + base64.b64encode(buf.tobytes()).decode("ascii")
