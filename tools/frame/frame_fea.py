r"""차량 프레임 보요소 해석 — 3030 프레임 + 클램프 + 액슬 타이.

클램프 단품은 3D 솔리드 FEA(CalculiX)로 이미 풀었다. 프레임은 가늘고 긴 부재라
보(beam) 요소가 맞는 도구다 — 솔리드로 메싱하면 수백만 요소가 되고, 얻는 건 없다.

  .venv/Scripts/python.exe tools/frame/frame_fea.py

모델:
  · 3030 부재      12자유도 3차원 보요소 (E=69000, A/I 는 참조단면 실측)
  · 클램프         강체 링크 (자체 변형은 CalculiX 결과를 따로 더한다)
  · 액슬 타이      강철 트러스 (M8 x2)
  · 지지           바퀴 접지점 4곳
"""

import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT / "models" / "hoverboard_mount"), str(ROOT / "models")]

import parameters as P                                            # noqa: E402
import low_clamp as LC                                            # noqa: E402

# ---------------------------------------------------------------- 제원
S = 30.0
BASE_L, BASE_W = 760.0, 450.0
RAIL_Y = (BASE_W - S) / 2                 # 210
CROSS_X = (BASE_L - S) / 2                # 365
AXLE_X = 210.0
AXLE_Z = P.AXLE_Z                         # 79.35
RAIL_Z = AXLE_Z + LC.RAIL_CZ              # 108.85
POST_H = 143.0
TOP_Z = RAIL_Z + S / 2 + POST_H + S / 2   # 281.85
WHEEL_Y = RAIL_Y - LC.CLAMP_CY            # 263.7
TIE_Y = RAIL_Y - P.LOW_D / 2              # 186.7
TIE_Z = AXLE_Z + P.TIE_Z                  # 61.35

E_AL, NU = 69000.0, 0.33
G_AL = E_AL / (2 * (1 + NU))
A30, I30 = 406.0, 3.10e4                  # 3030 단면 (참조모델 실측 / 30계열 공표)
# T슬롯은 슬롯이 단면을 열어 놔서 비틀림 강성이 낮다. 폐단면 근사(2I)를 쓰면
# 과대평가라 절반쯤으로 본다.
J30 = 1.5e4
RIG = dict(A=1e5, Iy=1e8, Iz=1e8, J=1e8, E=E_AL, G=G_AL)          # 강체 링크
BAR = dict(A=A30, Iy=I30, Iz=I30, J=J30, E=E_AL, G=G_AL)
TIE = dict(A=36.6 * P.TIE_N, Iy=1.0, Iz=1.0, J=1.0, E=206000.0, G=79000.0)


class Model:
    def __init__(self):
        self.nodes, self.idx, self.elems = [], {}, []

    def n(self, key, xyz):
        if key not in self.idx:
            self.idx[key] = len(self.nodes)
            self.nodes.append(np.array(xyz, float))
        return self.idx[key]

    def e(self, a, b, prop, tag=""):
        self.elems.append((a, b, prop, tag))


def beam_k(L, p):
    """국부좌표 12x12 강성 (x = 부재축)."""
    E, G, A, Iy, Iz, J = p["E"], p["G"], p["A"], p["Iy"], p["Iz"], p["J"]
    k = np.zeros((12, 12))
    k[0, 0] = k[6, 6] = E * A / L
    k[0, 6] = k[6, 0] = -E * A / L
    k[3, 3] = k[9, 9] = G * J / L
    k[3, 9] = k[9, 3] = -G * J / L
    for I, (i1, i2, j1, j2), sgn in ((Iz, (1, 7, 5, 11), 1.0), (Iy, (2, 8, 4, 10), -1.0)):
        a, b = 12 * E * I / L**3, 6 * E * I / L**2
        c, d = 4 * E * I / L, 2 * E * I / L
        k[i1, i1] = k[i2, i2] = a
        k[i1, i2] = k[i2, i1] = -a
        k[j1, j1] = k[j2, j2] = c
        k[j1, j2] = k[j2, j1] = d
        k[i1, j1] = k[j1, i1] = sgn * b
        k[i1, j2] = k[j2, i1] = sgn * b
        k[i2, j1] = k[j1, i2] = -sgn * b
        k[i2, j2] = k[j2, i2] = -sgn * b
    return k


def transform(v):
    """부재축 v 에 대한 3x3 방향코사인."""
    x = v / np.linalg.norm(v)
    up = np.array([0.0, 0.0, 1.0])
    if abs(np.dot(x, up)) > 0.99:
        up = np.array([1.0, 0.0, 0.0])
    z = np.cross(x, up); z /= np.linalg.norm(z)
    y = np.cross(z, x)
    return np.vstack([x, y, z])


def solve(m, loads, fixed):
    ndof = 6 * len(m.nodes)
    K = np.zeros((ndof, ndof))
    store = []
    for a, b, p, tag in m.elems:
        v = m.nodes[b] - m.nodes[a]
        L = float(np.linalg.norm(v))
        R = transform(v)
        T = np.zeros((12, 12))
        for i in range(4):
            T[3*i:3*i+3, 3*i:3*i+3] = R
        kl = beam_k(L, p)
        kg = T.T @ kl @ T
        dofs = [6*a + i for i in range(6)] + [6*b + i for i in range(6)]
        K[np.ix_(dofs, dofs)] += kg
        store.append((dofs, T, kl, L, p, tag, a, b))
    F = np.zeros(ndof)
    for nd, vec in loads:
        F[6*nd:6*nd+3] += vec
    free = np.array([d for d in range(ndof) if d not in fixed])
    U = np.zeros(ndof)
    U[free] = np.linalg.solve(K[np.ix_(free, free)], F[free])
    Rf = K @ U - F
    return U, Rf, store


def build(tie=True):
    m = Model()
    # ---- 하부 사이드레일 / 크로스 ----
    xs_low = sorted({-BASE_L/2 + S/2, -CROSS_X, -AXLE_X, -265.0, 0.0,
                     265.0, AXLE_X, CROSS_X, BASE_L/2 - S/2})
    for sy in (-1, 1):
        prev = None
        for x in xs_low:
            k = m.n(("L", x, sy), (x, sy*RAIL_Y, RAIL_Z))
            if prev is not None:
                m.e(prev, k, BAR, "하부사이드")
            prev = k
    for x in (-CROSS_X, -265.0, 0.0, 265.0, CROSS_X):
        m.e(m.idx[("L", x, -1)], m.idx[("L", x, 1)], BAR, "하부크로스")
    # ---- 기둥 / 상부 ----
    for sy in (-1, 1):
        prev = None
        for x in (-CROSS_X, 0.0, CROSS_X):
            k = m.n(("T", x, sy), (x, sy*RAIL_Y, TOP_Z))
            m.e(m.idx[("L", x, sy)], k, BAR, "기둥")
            if prev is not None:
                m.e(prev, k, BAR, "상부사이드")
            prev = k
    for x in (-CROSS_X, 0.0, CROSS_X):
        m.e(m.idx[("T", x, -1)], m.idx[("T", x, 1)], BAR, "상부크로스")
    # ---- 클램프 / 바퀴 / 타이 ----
    ground = []
    for sx in (-1, 1):
        for sy in (-1, 1):
            x = sx * AXLE_X
            r = m.idx[("L", x, sy)]
            c = m.n(("C", sx, sy), (x, sy*WHEEL_Y, AXLE_Z))
            g = m.n(("G", sx, sy), (x, sy*WHEEL_Y, 0.0))
            t = m.n(("Tie", sx, sy), (x, sy*TIE_Y, TIE_Z))
            m.e(r, c, RIG, "클램프")
            m.e(c, g, RIG, "바퀴")
            m.e(c, t, RIG, "클램프")
            ground.append(g)
        if tie:
            m.e(m.idx[("Tie", sx, -1)], m.idx[("Tie", sx, 1)], TIE, "액슬타이")
    return m, ground


def run(case, tie=True):
    m, ground = build(tie)
    VEH, RIDER = 40.0 * 9.80665, 70.0 * 9.80665
    loads = []
    tops = [k for k in m.idx if k[0] == "T"]
    for k in tops:                                   # 차량 자중은 상부에 분산
        loads.append((m.idx[k], np.array([0, 0, -VEH / len(tops)])))
    if case == "중앙":
        for sy in (-1, 1):
            loads.append((m.idx[("T", 0.0, sy)], np.array([0, 0, -RIDER / 2])))
    elif case == "한쪽구석":
        loads.append((m.idx[("T", CROSS_X, 1)], np.array([0, 0, -RIDER])))
    elif case == "뒤축위":
        for sy in (-1, 1):
            loads.append((m.idx[("T", CROSS_X, sy)], np.array([0, 0, -RIDER / 2])))
    fixed = set()
    for g in ground:                                  # 접지점 3방향 구속
        for i in range(3):
            fixed.add(6*g + i)
    fixed.add(6*ground[0] + 3); fixed.add(6*ground[0] + 4); fixed.add(6*ground[0] + 5)
    U, R, store = solve(m, loads, fixed)
    return m, U, R, store, ground
