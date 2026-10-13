r"""v29 바닥 배선 라우터 — 경로를 찾아 routes_data.py 에 굳힌다.

    .venv/Scripts/python.exe models/vehicle/v29_floor_wiring/router.py [--cache 폴더]

방법
  1. 바닥 장애물 지도 (2 mm 격자) — 부품을 바닥 위 얇은 판 (z = 바닥 + 0.6 … 16) 과 교차해 단면을 칠한다.
  2. 끝점마다: 단자 → 빠져나오기 (escape) → 둘레에서 수직으로 바닥까지 내려갈 기둥 자리 찾기 (OCC 로 부품 · 이미 깐 선과 겹침 확인).
  3. 바닥: A* (x · y 만, 꺾임 벌점, 다른 선 · 부품 옆에 붙으면 싸게 = 다발, 다른 선은 직각으로만 넘어간다 — 넘어가는 곳은 들어 올림).
  4. 굵은 · 중요한 선부터 차례로 깔고 (order), 깐 선은 다음 선의 장애물이 된다.
결과 경로는 짓기 때 그대로 쓴다 (짓기는 router 를 부르지 않는다).
"""

import heapq
import math
import pickle
import sys
import time
from pathlib import Path

HERE = Path(__file__).parent
sys.path.insert(0, str(HERE))

import numpy as np                                     # noqa: E402
from scipy import ndimage                              # noqa: E402

H = 2.0                                                # 격자 칸 (mm)
X0, X1, Y0, Y1 = -380.0, 380.0, -225.0, 225.0
NX, NY = int((X1 - X0) / H), int((Y1 - Y0) / H)
LEVELS = (0.6, 3.0, 6.0, 7.2, 9.0, 12.8, 16.0)              # 바닥 위 단면 높이 — 선 · 넘어가기 높이 (≤ 12) + 여유
CLEAR = 0.8                                            # 부품과 띄우는 틈
GAP = 0.6                                              # 선끼리 틈
TURN = 24.0                                            # 꺾임 벌점 (mm 상당)
CROSS = 18.0                                           # 다른 선 넘어가기 벌점 (칸마다)
STACK = 5.0                                            # 다른 선 위에 나란히 얹기 벌점 (칸마다) — 다발
HUG = 1.2                                              # 아무것에도 안 붙어 가면 mm 당 벌점
HUG_D = 2.0                                            # '붙어 간다' 거리
STRAPS = [(sx * 40.0, sy * 54.0, 25.0, 13.0) for sx in (-1, 1) for sy in (-1, 1)]   # 배터리 밴드 (x, y, 폭 x, 폭 y) — 판 슬롯 + 옆면
DX = (1, -1, 0, 0)
DY = (0, 0, 1, -1)


def cell(x, y):
    return int(math.floor((x - X0) / H)), int(math.floor((y - Y0) / H))


def center(i, j):
    return X0 + (i + 0.5) * H, Y0 + (j + 0.5) * H


# ---------------------------------------------------------------- 바닥 장애물 지도
def raster_tris(grid, tris):
    """xy 삼각형들을 칸 중심 기준으로 칠한다."""
    for t in tris:
        (ax, ay), (bx, by), (cx, cy) = t
        i0, j0 = cell(min(ax, bx, cx), min(ay, by, cy))
        i1, j1 = cell(max(ax, bx, cx), max(ay, by, cy))
        i0, j0, i1, j1 = max(i0, 0), max(j0, 0), min(i1, NX - 1), min(j1, NY - 1)
        if i1 < i0 or j1 < j0:
            continue
        ii, jj = np.meshgrid(np.arange(i0, i1 + 1), np.arange(j0, j1 + 1), indexing="ij")
        px, py = X0 + (ii + 0.5) * H, Y0 + (jj + 0.5) * H
        d1 = (px - bx) * (ay - by) - (ax - bx) * (py - by)
        d2 = (px - cx) * (by - cy) - (bx - cx) * (py - cy)
        d3 = (px - ax) * (cy - ay) - (cx - ax) * (py - ay)
        neg = (d1 < 0) | (d2 < 0) | (d3 < 0)
        pos = (d1 > 0) | (d2 > 0) | (d3 > 0)
        inside = ~(neg & pos)
        grid[i0:i1 + 1, j0:j1 + 1] |= inside


def floor_map(parts, z0):
    from build123d import Box, Pos
    grid = np.zeros((NX, NY), bool)
    slab_cache = {}
    for p in parts:
        bb = p.bounding_box()
        if bb.min.Z > z0 + LEVELS[-1] or bb.max.Z < z0:
            continue
        for lv in LEVELS:
            z = z0 + lv
            if not (bb.min.Z <= z <= bb.max.Z):
                continue
            if z not in slab_cache:
                slab_cache[z] = Pos(0, 0, z) * Box(X1 - X0 + 40, Y1 - Y0 + 40, 0.3)
            try:
                inter = p & slab_cache[z]
            except Exception:                                       # noqa: BLE001
                inter = None
            if inter is None or inter.volume <= 0:
                continue
            tris = []
            for f in inter.faces():
                if abs(f.normal_at().Z) < 0.9:
                    continue
                vs, ts = f.tessellate(0.3, 0.3)
                for a, b, c in ts:
                    tris.append(((vs[a].X, vs[a].Y), (vs[b].X, vs[b].Y), (vs[c].X, vs[c].Y)))
            raster_tris(grid, tris)
    for x, y, w, h in STRAPS:                                       # 배터리 밴드 (모델 없음)
        i0, j0 = cell(x - w / 2, y - h / 2)
        i1, j1 = cell(x + w / 2, y + h / 2)
        grid[max(i0, 0):i1 + 1, max(j0, 0):j1 + 1] = True
    return grid


# ---------------------------------------------------------------- 라우터
class Router:
    def __init__(self, obst, index, WR):
        self.WR = WR
        self.obst = obst
        self.dobs = ndimage.distance_transform_edt(~obst) * H - H / 2     # 칸 중심 → 가장 가까운 장애물 칸 가장자리 (mm, 어림)
        self.index = list(index)                                          # OCC 조각 (부품 + 깐 선)
        self.cl = np.full((NX, NY), -1, np.int32)                          # 깐 선 중심선 칸 → 선 번호
        self.cl_axis = np.zeros((NX, NY), np.int8)
        self.cl_r = np.zeros((NX, NY), np.float32)
        self.hard = np.zeros((NX, NY), bool)                               # 세움 기둥 · 넘어간 곳 · 들어 올린 곳
        self.stacked = np.zeros((NX, NY), bool)                            # 얹힌 선이 지나간 칸 (나란히 세 겹 금지)
        self.top = np.full((NX, NY), WR.Z0)                                # 칸마다 깐 것의 윗면 z (넘어갈 높이)
        self.res = np.zeros((NX, NY), bool)                                # 끝점 기둥 자리 (모든 선)
        self.own = {}                                                      # 선마다 제 기둥 자리
        self.n = 0
        self.log = []

    # ---- 깐 선 근처
    def near(self, r):
        if (self.cl >= 0).any():
            d, (ii, jj) = ndimage.distance_transform_edt(self.cl < 0, return_indices=True)
            dmm = d * H
            rr = self.cl_r[ii, jj]
            conflict = (self.cl[ii, jj] >= 0) & (dmm < r + rr + GAP + H * 0.5)
            axis = self.cl_axis[ii, jj]
            edge = dmm - rr
        else:
            conflict = np.zeros((NX, NY), bool)
            axis = np.zeros((NX, NY), np.int8)
            edge = np.full((NX, NY), 1e9)
        return conflict, axis, edge

    def free_mask(self, r, zone=None):
        free = (self.dobs >= r + CLEAR) & ~self.hard
        if zone is not None:
            free |= zone
        return free

    def zone_mask(self, zones):
        if not zones:
            return None
        z = np.zeros((NX, NY), bool)
        ii, jj = np.meshgrid(np.arange(NX), np.arange(NY), indexing="ij")
        px, py = X0 + (ii + 0.5) * H, Y0 + (jj + 0.5) * H
        for x, y, rad in zones:
            z |= (px - x) ** 2 + (py - y) ** 2 <= rad * rad
        return z

    # ---- 끝점: 빠져나와 바닥까지 내려갈 기둥 찾기
    def drop(self, end, r, ends, free, conflict, zf, zone=None):
        p = np.array(end["p"], float)
        pts = [tuple(p)]
        for e in end["esc"]:
            p = p + np.array(e)
            pts.append(tuple(p))
        E = p
        ci, cj = cell(E[0], E[1])
        order = []
        R = 50
        for k in range(0, R + 1):
            ring = set()
            for a in range(-k, k + 1):
                for b in (-k, k):
                    ring.add((a, b))
                    ring.add((b, a))
            order += sorted(ring, key=lambda ab: (abs(ab[0]) + abs(ab[1]), ab))
        tried = 0
        for a, b in order:
            i, j = ci + a, cj + b
            if not (0 <= i < NX and 0 <= j < NY):
                continue
            inz = zone is not None and zone[i, j]
            if not inz and (not free[i, j] or conflict[i, j]):
                continue
            x, y = center(i, j)
            leg = []
            if abs(x - E[0]) > 1e-6:
                leg.append((x, E[1], E[2]))
            if abs(y - E[1]) > 1e-6:
                leg.append((x, y, E[2]))
            bottom = (x, y, zf)
            path = pts + leg + ([bottom] if abs(E[2] - zf) > 1e-6 else [])
            tried += 1
            if tried > 400:
                break
            if inz:
                return path, (i, j)
            t = self.WR.tube(path, 2 * r)
            if not self.WR.collide(t, self.index, ends):
                return path, (i, j)
        return None, None

    # ---- A*
    def astar(self, s, g, free, conflict, axis, stacked, edge, r, zone=None, layer3=False):
        """x · y 만. conflict 칸 = 다른 선 곁 — 직각이면 넘어가기 (CROSS), 나란하면 그 위에 얹기 (STACK, 아래 선이 이미 얹힌 선이면 금지)."""
        hug = np.where(np.minimum(self.dobs - r, edge - r) > HUG_D, HUG * H, 0.0)
        gs = np.full((4, NX, NY), np.inf, np.float64)
        came = np.full((4, NX, NY), -1, np.int32)
        heap = []
        gi, gj = g
        for d in range(4):
            gs[d, s[0], s[1]] = 0.0
            heapq.heappush(heap, (H * (abs(s[0] - gi) + abs(s[1] - gj)), 0.0, s[0], s[1], d))
        inz = zone if zone is not None else np.zeros((NX, NY), bool)
        while heap:
            f, gc, i, j, d = heapq.heappop(heap)
            if gc > gs[d, i, j]:
                continue
            if i == gi and j == gj:
                path = [(i, j)]
                st = (d, i, j)
                while True:
                    c = came[st]
                    if c < 0:
                        break
                    pd, pi, pj = c // (NX * NY), (c // NY) % NX, c % NY
                    path.append((pi, pj))
                    st = (pd, pi, pj)
                return path[::-1]
            for nd in range(4):
                if nd == (d ^ 1):
                    continue
                ni, nj = i + DX[nd], j + DY[nd]
                if not (0 <= ni < NX and 0 <= nj < NY):
                    continue
                goal = ni == gi and nj == gj
                if not goal and not free[ni, nj]:
                    continue
                cost = H + hug[ni, nj] + (TURN if nd != d else 0.0)
                if conflict[ni, nj] and not goal and not inz[ni, nj]:
                    if axis[ni, nj] == nd // 2:
                        if stacked[ni, nj]:
                            if not layer3:
                                continue                                  # 세 겹은 안 쌓는다 (막히면 비싸게 허용)
                            cost += 4 * STACK
                        cost += STACK                                     # 나란히 — 그 선 위에 얹는다 (다발)
                    else:
                        cost += CROSS                                     # 직각으로 넘어간다
                ng = gc + cost
                if ng < gs[nd, ni, nj]:
                    gs[nd, ni, nj] = ng
                    came[nd, ni, nj] = d * NX * NY + i * NY + j
                    heapq.heappush(heap, (ng + H * (abs(ni - gi) + abs(nj - gj)), ng, ni, nj, nd))
        return None

    def to3d(self, cells, r, zf, conflict, zone, topmax):
        """칸 경로 → 3D. 다른 선 곁을 지나는 칸은 그 선 (곁의 가장 굵은 것) 위로 들어 올리고 45° 로 오르내린다."""
        nn = len(cells)
        zl = np.full(nn, zf)
        lift = np.zeros(nn, bool)
        for k, (i, j) in enumerate(cells):
            if conflict[i, j] and (zone is None or not zone[i, j]) and 0 < k < nn - 1:
                zl[k] = float(topmax[i, j]) + 0.5 + r
                lift[k] = True
        z = zl.copy()
        for k in np.nonzero(lift)[0]:
            for m in range(nn):
                z[m] = max(z[m], zl[k] - 1.0 * abs(m - k) * H)
        pts = [(*center(i, j), float(z[k])) for k, (i, j) in enumerate(cells)]
        out = [pts[0]]
        for k in range(1, nn - 1):
            a, b, c = np.array(out[-1]), np.array(pts[k]), np.array(pts[k + 1])
            u, v = b - a, c - b
            if np.linalg.norm(np.cross(u, v)) > 1e-6 * max(np.linalg.norm(u) * np.linalg.norm(v), 1e-9):
                out.append(pts[k])
        out.append(pts[-1])
        return out, lift

    def commit(self, cells, lift, r, path3d, label):
        n = self.n
        self.n += 1
        ax = 0
        for k, (i, j) in enumerate(cells):
            if k + 1 < len(cells):
                ax = 0 if cells[k + 1][0] != i else 1
            if lift[k]:
                self.stacked[i, j] = True
            if self.cl[i, j] < 0 or lift[k]:
                self.cl[i, j] = n
                self.cl_axis[i, j] = ax
                self.cl_r[i, j] = r
        rad = int(math.ceil(r / H))
        for a, b in zip(path3d, path3d[1:]):                               # 윗면 지도 — 바닥 근처 구간만
            if max(a[2], b[2]) > self.WR.Z0 + 20:
                continue
            L = max(int(np.linalg.norm(np.subtract(b, a)) / (H / 2)), 1)
            for t_ in np.linspace(0, 1, L + 1):
                x, y, z = np.add(a, np.multiply(t_, np.subtract(b, a)))
                i, j = cell(x, y)
                sl = (slice(max(i - rad, 0), i + rad + 1), slice(max(j - rad, 0), j + rad + 1))
                self.top[sl] = np.maximum(self.top[sl], z + r)
        t = self.WR.tube(path3d, 2 * r)
        t.label = label
        self.index += self.WR.solid_index([t])

    # ---- 1 단계: 모든 끝점의 내려갈 기둥을 먼저 잡는다 (나중 선이 단자 앞을 막지 않게)
    def reserve(self, n):
        r = n.d / 2
        zf = self.WR.zf(n.d)
        zone = self.zone_mask(self.zones(n))
        free = (self.dobs >= r + CLEAR) & ~self.res
        if zone is not None:
            free |= zone
        lab, nlab = ndimage.label(free)
        main = np.argmax(np.bincount(lab.ravel())[1:]) + 1 if nlab else 0
        dfree = (lab == main) if nlab else free
        none = np.zeros((NX, NY), bool)
        ends = n.ends + (n.label,)
        out = []
        for end in (n.A, n.B):
            path, c = self.drop(end, r, ends, dfree, none, zf, zone)
            if path is None:
                print(f"    ({'A' if end is n.A else 'B'} 끝 {end['p']} 막힘)")
                return None
            out.append((path, c))
            i, j = c
            rad = int(math.ceil((r + GAP * 0.5) / H))
            m = np.zeros((NX, NY), bool)
            m[max(i - rad, 0):i + rad + 1, max(j - rad, 0):j + rad + 1] = True
            if zone is not None:
                m &= ~zone
            self.res |= m
            self.own.setdefault(n.label, np.zeros((NX, NY), bool))
            self.own[n.label] |= m
            t = self.WR.tube(path, n.d)
            t.label = n.label
            self.index += self.WR.solid_index([t])
        return out

    def zones(self, n):
        if n.label.startswith(("상선", "홀_")):                         # 모터 케이블 위에서 갈라진다 — 그 둘레는 지나가도 된다
            return [(n.A["p"][0], n.A["p"][1], 9.0)]
        if n.label.startswith("팬_"):                                  # 팬 케이블 토막 밑 (지도에는 토막이 칠해진다)
            return [(n.A["p"][0], n.A["p"][1] + (4.0 if n.A["p"][1] > 0 else -4.0), 7.0)]
        return []

    # ---- 2 단계: 바닥 경로
    def route(self, n, drops):
        WR = self.WR
        r = n.d / 2
        zf = WR.zf(n.d)
        zone = self.zone_mask(self.zones(n))
        blocked = (self.res & ~self.own.get(n.label, np.zeros((NX, NY), bool))) | self.hard
        free = (self.dobs >= r + CLEAR) & ~blocked
        if zone is not None:
            free |= zone
        conflict, axis, edge = self.near(r)
        topmax = ndimage.maximum_filter(self.top, size=2 * int(math.ceil((r + 3.0) / H)) + 1)
        (pa, ca), (pb, cb) = drops
        cells = self.astar(ca, cb, free, conflict, axis, self.stacked, edge, r, zone)
        if cells is None:
            cells = self.astar(ca, cb, free, conflict, axis, self.stacked, edge, r, zone, layer3=True)
            if cells is not None:
                self.log.append(n.label)
        if cells is None:
            return None, "바닥 경로 없음"
        mid, lift = self.to3d(cells, r, zf, conflict, zone, topmax)
        path = n.pre + pa[:-1] + mid + pb[::-1][1:] + n.post
        path = [tuple(float(round(float(v), 2)) for v in p) for p in path]
        dedup = [path[0]]
        for p in path[1:]:
            if np.linalg.norm(np.subtract(p, dedup[-1])) > 1e-6:
                dedup.append(p)
        self.commit(cells, lift, r, dedup, n.label)
        return dedup, None


# ---------------------------------------------------------------- 실행
def load_env(cache):
    """환경 부품 (배선 빼고) — 캐시 폴더가 있으면 BREP 로 읽는다."""
    from build123d import export_brep, import_brep
    if cache and (cache / "index.pkl").exists():
        idx = pickle.load(open(cache / "index.pkl", "rb"))
        out = []
        for lab, fn in idx:
            s = import_brep(str(cache / fn))
            s.label = lab
            out.append(s)
        print(f"  환경 캐시 {len(out)} 부품")
        return out
    import assembly as A
    groups = A.build_env()
    parts = [p for g in groups if g["key"] != "cable" for p in g["parts"]]
    if cache:
        cache.mkdir(parents=True, exist_ok=True)
        idx = []
        for k, p in enumerate(parts):
            fn = f"p{k}.brep"
            export_brep(p, str(cache / fn))
            idx.append((p.label, fn))
        pickle.dump(idx, open(cache / "index.pkl", "wb"))
    return parts


def debug_plot(R, n, drops, cache):
    """개발용 — 실패하는 선의 끝점 둘레 상태 그림 (free · 예약 · 곁 · 단단)."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    r = n.d / 2
    blocked = (R.res & ~R.own.get(n.label, np.zeros((NX, NY), bool))) | R.hard
    free = (R.dobs >= r + CLEAR) & ~blocked
    conflict, axis, edge = R.near(r)
    img = np.zeros((NX, NY, 3))
    img[~free] = (0.55, 0.55, 0.55)
    img[R.obst] = (0.2, 0.2, 0.2)
    img[R.res & free] = (1.0, 0.6, 0.2)
    img[conflict & free] = (0.4, 0.7, 1.0)
    img[conflict & free & (axis == 0)] = (0.3, 0.5, 1.0)
    img[R.stacked] = (0.8, 0.3, 0.8)
    (pa, ca), (pb, cb) = drops
    fig, axs = plt.subplots(1, 2, figsize=(20, 9))
    for ax, c in zip(axs, (ca, cb)):
        ax.imshow(img.transpose(1, 0, 2), origin="lower", extent=(X0, X1, Y0, Y1))
        for cc, col in ((ca, "lime"), (cb, "red")):
            ax.plot(*center(*cc), "o", color=col, ms=8)
        x, y = center(*c)
        ax.set_xlim(x - 60, x + 60)
        ax.set_ylim(y - 40, y + 40)
        ax.set_title(f"{n.label} r {r}")
    fig.tight_layout()
    fig.savefig(cache / "debug.png", dpi=70)
    print("  debug 그림:", cache / "debug.png")


def main():
    t0 = time.time()
    cache = None
    if "--cache" in sys.argv:
        cache = Path(sys.argv[sys.argv.index("--cache") + 1])
    import wiring as WR
    env = load_env(cache)
    newp = [p for v in WR.parts().values() for p in v]
    t1 = time.time()
    mp = cache / "floor.npy" if cache else None
    if mp and mp.exists():
        obst = np.load(mp)
    else:
        obst = floor_map(env + newp, WR.Z0)
        if mp:
            np.save(mp, obst)
    print(f"  바닥 지도 {time.time() - t1:.1f}s — 막힌 칸 {obst.mean() * 100:.0f} %")
    R = Router(obst, WR.solid_index(env + newp), WR)
    routes = {}
    nets = WR.nets()
    for n in nets:                                                  # 고정 경로 (리드 · 점퍼 · 모터 케이블 뿌리) 를 먼저 장애물로
        if n.fixed:
            t = WR.tube(n.fixed, n.d)
            t.label = n.label
            R.index += WR.solid_index([t])
            for p in n.fixed:
                if abs(p[2] - WR.zf(n.d)) < 6 or n.label.startswith("모터케이블"):
                    i, j = cell(p[0], p[1])
                    rad = int(math.ceil((n.d / 2 + GAP) / H))
                    if not n.label.startswith("모터케이블"):
                        R.hard[max(i - rad, 0):i + rad + 1, max(j - rad, 0):j + rad + 1] = True
    todo = [n for n in nets if not n.fixed]
    dist = lambda n: abs(n.A["p"][0] - n.B["p"][0]) + abs(n.A["p"][1] - n.B["p"][1])     # noqa: E731
    todo.sort(key=lambda n: (n.order, -n.d, -dist(n)))              # 끝점 기둥: 굵은 · 중요한 것부터
    bad = []
    drops = {}
    t2 = time.time()
    for n in todo:                                                  # 1 단계 — 끝점 기둥
        d = R.reserve(n)
        if d is None:
            bad.append((n.label, "끝점 기둥 자리 없음"))
            print(f"  ✗ {n.label}: 끝점 기둥 자리 없음")
            continue
        drops[n.label] = d
    print(f"  끝점 기둥 {len(drops)} / {len(todo)} · {time.time() - t2:.0f}s")
    todo.sort(key=lambda n: (n.order > 1, dist(n)))                # 바닥 경로: 모터 · 주전원 다음엔 짧은 것부터 (긴 것은 돌아가도 된다)
    for n in todo:                                                  # 2 단계 — 바닥 경로
        if n.label not in drops:
            continue
        t2 = time.time()
        if "--debug" in sys.argv and n.label == sys.argv[sys.argv.index("--debug") + 1]:
            debug_plot(R, n, drops[n.label], cache)
        pts, err = R.route(n, drops[n.label])
        if err:
            bad.append((n.label, err))
            print(f"  ✗ {n.label}: {err}")
            continue
        routes[n.label] = pts
        print(f"  {n.label}: {WR.plen(pts) / 1000:.2f} m · 점 {len(pts)} · {time.time() - t2:.1f}s")
    with open(HERE / "routes_data.py", "w", encoding="utf-8") as f:
        f.write('"""router.py 가 쓴 바닥 배선 경로 (손으로 고치지 않는다). {라벨: [(x, y, z) …]} — mm, 차량 좌표."""\n\n')
        f.write("ROUTES = {\n")
        for k, v in routes.items():
            f.write(f"    {k!r}: {v!r},\n")
        f.write("}\n")
        f.write(f"FAILED = {bad!r}\n")
    print(f"  경로 {len(routes)} / {len(todo)} · 실패 {len(bad)} · 세 겹 {len(R.log)} {R.log} · {time.time() - t0:.0f}s")
    if cache and "--plot" in sys.argv:                              # 개발용 위 그림
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        col = {k: v[1] for k, v in WR.GROUPS.items()}
        col["wire_neg"] = "#555555"
        fig, ax = plt.subplots(figsize=(24, 14))
        ax.imshow(obst.T, origin="lower", extent=(X0, X1, Y0, Y1), cmap="Greys", alpha=0.35)
        grp = {n.label: n.grp for n in nets}
        for lab_, pts in list(routes.items()) + [(n.label, n.fixed) for n in nets if n.fixed]:
            p = np.array(pts)
            lw = 2.2 if p[:, 2].max() < WR.Z0 + 14 else 1.0
            ax.plot(p[:, 0], p[:, 1], "-", color=col[grp[lab_]], lw=lw)
        ax.set_xlim(X0, X1)
        ax.set_ylim(Y0, Y1)
        ax.set_aspect("equal")
        ax.grid(lw=0.2)
        fig.tight_layout()
        fig.savefig(cache / "routes_top.png", dpi=80)


if __name__ == "__main__":
    main()
