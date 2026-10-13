r"""차량 프레임 보요소 해석 — 3030 프레임 + 클램프 + 액슬 타이 + H 크로스 혀.

클램프 단품은 3D 솔리드 FEA(CalculiX)로 이미 풀었다. 프레임은 가늘고 긴 부재라
보(beam) 요소가 맞는 도구다 — 솔리드로 메싱하면 수백만 요소가 되고, 얻는 건 없다.

  .venv/Scripts/python.exe tools/frame/frame_fea.py          # 검증 + 보강 비교 표/그림

모델:
  · 3030 부재      12자유도 3차원 보요소 (E=69000, A/I 는 참조단면 실측)
  · 클램프         (기본) 강체 링크  /  (k_clamp 지정) 레일 밑면 힌지에 x축 둘레 회전 스프링
  · 액슬 타이      트러스 (M8 x2 또는 3030) + 양 끝 클램프 몸체 국부 컴플라이언스 스프링
  · H 크로스 혀    x=±210 3030 크로스 + 혀 볼트 중간 z 스프링 (혀 ⊕ 브라켓 조인트 직렬)
  · 지지           바퀴 접지점 4곳

기존 호출 `run(case, tie=True)` 는 예전과 똑같이 동작한다 (frame_view.py 가 쓴다):
클램프 강체 링크, 접지 3방향 구속 + 첫 접지점 회전 구속.

단위: N, mm, rad. 회전강성은 N·mm/rad (표기할 때만 N·m/rad 로 바꾼다).
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
TIE = dict(A=36.6 * P.TIE_N, Iy=1.0, Iz=1.0, J=1.0, E=206000.0, G=79000.0)   # M8 전산볼트 x2
TIE_PROF = dict(A=A30, Iy=1.0, Iz=1.0, J=1.0, E=E_AL, G=G_AL)                # 3030 여분 (순수 인장)

# ---------------------------------------------------------------- 클램프 컴플라이언스 (단품 FEA 보정)
# 단품 FEA: PETG E=2100, 800 N 상향, 편심 55 → M = 44.0 N·m, 3 mm C3D10, 레일 볼트 구멍 벽 강체.
#   보강 없음  캠버 46.160 mrad, 시트 Uy +0.6699, 시트 Uz +0.0649   (clamp_800N_h3mm_petg_e55mm_up)
#   혀 (tongue) 캠버 7.439 mrad, 혀 볼트 반력 Fz 195.3+460.5 = 655.8 N (..._up_tongue)
#   3030 타이   캠버 7.576 mrad, 타이 탭 접지 스프링 2EA/L=155,633 N/mm  (..._up_tieprof)
#   M8 타이     캠버 7.573 mrad, 접지 스프링 80,780 N/mm                (..._up_tierod)
# 힌지 위치: 시트 Uy/θ = 0.6699/0.046160 = 14.51 mm → 축 중심 위 14.5 = **레일 밑면(다리 윗면)**.
#            시트 Uz 회귀 영점 y = -55.2 ≈ 레일 중심(-53.7). 그래서 힌지 = (레일 중심 y, 레일 밑면 z).
M_REF = 800.0 * 55.0                                  # 44,000 N·mm
K_CLAMP_PETG = M_REF / 0.046160                       # 953,206 N·mm/rad = 953 N·m/rad (몸체만)
K_CLAMP_AL = K_CLAMP_PETG * 68900.0 / 2100.0          # 선형탄성 → E 비례 (31,300 N·m/rad)
HINGE_Z = AXLE_Z + LC.RAIL_BOT                        # 93.85
HINGE_DZ = HINGE_Z - AXLE_Z                           # 14.5 (축 중심 위)

# 혀: 볼트 2개 y (클램프 로컬 -92, -122) 의 중간 → 차량 y = 263.7 - 107 = 156.7.
# (과제 문구의 ±165 는 레일 안쪽면 195 에서 15·30 을 뺀 값인데, 볼트는 레일 안쪽면이 아니라
#  클램프 안쪽면(로컬 -77, 레일 안쪽면보다 8.3 안쪽)에서 15·45 다. 파라미터에서 유도한다.)
TONGUE_Y = WHEEL_Y + (LC.SHELF_FACE - float(np.mean(P.SHELF_BOLT_IN)))   # 156.7
TONGUE_ARM = RAIL_Y - TONGUE_Y                                          # 53.3 (힌지 → 볼트 중간)
TONGUE_Z = HINGE_Z                                                      # 혀 윗면 = 레일 밑면
# 혀 강성: 단품 FEA 의 혀 구멍 벽은 강체 고정이라 그 노드의 Uz = 0 (평균 |U| 1.4e-5 mm) → 700/Uz 는
# 정의가 안 된다. 대신 **힌지 + 혀 z 스프링 (조인트 강체)** 모델이 단품 캠버 7.439 mrad 를
# 그대로 내도록 맞춘다:  K_CLAMP + k_t·a² = M/θ  →  k_t = (44,000/0.007439 - 953,206)/53.3².
# 같은 뜻으로 k_t = F/(θ·a) = 693 N / (0.007439·53.3 mm) — 단품 볼트 반력 656 N 과 6% 안에서 맞는다.
THETA_TONGUE_FEA = 0.007439
K_TONGUE = (M_REF / THETA_TONGUE_FEA - K_CLAMP_PETG) / TONGUE_ARM**2     # ≈ 1,747 N/mm
K_TONGUE_AL = K_TONGUE * 68900.0 / 2100.0
# 브라켓 조인트 (T슬롯 립+너트 직렬 모델, 볼트 1개당). 브라켓 1개 = 볼트 1개.
K_JOINT = {"springnut": 23430.0, "tnut": 98415.0}

# 타이: 타이 노드 z = 61.35 (로컬 -18) → 힌지에서 32.5 아래. 단품 타이 케이스는 접지 스프링이
# 거의 강체(155,633 N/mm)인데도 7.576 mrad 가 남는다 = 타이 탭과 시트 사이 **몸체 국부 변형**.
# 이걸 빼면 프레임 모델의 타이는 캠버를 0.3 mrad 로 눌러 단품과 25배 어긋난다. 그래서 타이 양
# 끝에 국부 스프링 k_loc 을 둔다:  K_CLAMP + (k_loc ⊕ 155,633)·b² = 44,000/0.007576.
TIE_ARM = HINGE_Z - TIE_Z                                               # 32.5
THETA_TIE_FEA = 0.0075757
_k_tie_series = (M_REF / THETA_TIE_FEA - K_CLAMP_PETG) / TIE_ARM**2     # ≈ 4,597 N/mm
_k_tie_ground = 2 * E_AL * A30 / P.TIE_PROF_LEN                         # 155,633 (단품 FEA 접지 스프링)
K_TIE_LOCAL = 1.0 / (1.0 / _k_tie_series - 1.0 / _k_tie_ground)         # ≈ 4,737 N/mm
K_TIE_LOCAL_AL = K_TIE_LOCAL * 68900.0 / 2100.0

KR_T, KR_R = 1e9, 1e12            # 조인트에서 '강체'로 묶는 자유도의 페널티 (N/mm, N·mm/rad)
RIG_SHORT = dict(A=1e5, Iy=1.5e6, Iz=1.5e6, J=1.5e6, E=E_AL, G=G_AL)   # 15 mm 짜리 강체 링크
# (RIG 를 15 mm 에 그대로 쓰면 12EI/L³ 이 2.4e10 까지 올라가 조건수를 망친다. 61 mm RIG 와
#  같은 3.6e8 N/mm 수준이 되도록 I 를 (15/61)³ 로 줄였다.)


def joint(k):
    """전역 자유도 스프링 (두 노드 사이, 보통 겹친 노드). k = (kx, ky, kz, kθx, kθy, kθz)."""
    return dict(kind="joint", k=np.asarray(k, float),
                A=1.0, Iy=1.0, Iz=1.0, J=1.0, E=1.0, G=1.0)   # A/Iy 는 frame_view 호환용


def series(*ks):
    ks = [k for k in ks if k is not None and np.isfinite(k)]
    return 1.0 / sum(1.0 / k for k in ks) if ks else np.inf


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


def joint_k(p):
    """전역좌표 12x12 스프링. 겹친 노드(또는 스프링 방향과 나란한 노드쌍)에만 쓴다."""
    D = np.diag(p["k"])
    return np.block([[D, -D], [-D, D]])


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
    """loads: [(node, vec)] — vec 는 힘 3개 또는 힘+모멘트 6개."""
    ndof = 6 * len(m.nodes)
    K = np.zeros((ndof, ndof))
    store = []
    for a, b, p, tag in m.elems:
        v = m.nodes[b] - m.nodes[a]
        L = float(np.linalg.norm(v))
        if p.get("kind") == "joint":
            T = np.eye(12)
            kl = joint_k(p)
            kg = kl
        else:
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
        vec = np.asarray(vec, float)
        F[6*nd:6*nd+len(vec)] += vec
    free = np.array([d for d in range(ndof) if d not in fixed])
    U = np.zeros(ndof)
    U[free] = np.linalg.solve(K[np.ix_(free, free)], F[free])
    Rf = K @ U - F
    return U, Rf, store


def build(tie=True, k_clamp=None, h_cross=False, k_tongue=None, k_joint=None,
          tie_local=None, tie_prop=None):
    """
    tie        액슬 타이 (좌우 클램프 사이 트러스)
    k_clamp    None → 클램프 강체 링크 (기존).  값(N·mm/rad) → 레일 밑면 힌지에 x축 회전 스프링.
               np.inf 도 받는다 (페널티 강체 — 기존 결과 재현 검증용).
    h_cross    x=±210 에 3030 크로스 추가. k_tongue 를 주면 혀 볼트 중간(y=±156.7)에서 크로스와
               클램프 몸체를 z 스프링으로 잇는다. 스프링 = k_tongue ⊕ k_joint (직렬).
    tie_local  None → 타이가 몸체에 강체로 붙음 (기존).  값(N/mm) → 양 끝 y 국부 스프링.
    tie_prop   타이 부재 단면 (기본 TIE = M8 x2, TIE_PROF = 3030).
    """
    tie_prop = tie_prop or TIE
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
    # ---- H 크로스 (액슬 라인) ----
    if h_cross:
        for sx in (-1, 1):
            x = sx * AXLE_X
            a = m.idx[("L", x, -1)]
            hm = m.n(("H", sx, -1), (x, -TONGUE_Y, RAIL_Z))
            hp = m.n(("H", sx, 1), (x, TONGUE_Y, RAIL_Z))
            b = m.idx[("L", x, 1)]
            m.e(a, hm, BAR, "H크로스"); m.e(hm, hp, BAR, "H크로스"); m.e(hp, b, BAR, "H크로스")
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
    # ---- 클램프 / 바퀴 / 타이 / 혀 ----
    ground = []
    for sx in (-1, 1):
        for sy in (-1, 1):
            x = sx * AXLE_X
            r = m.idx[("L", x, sy)]
            c = m.n(("C", sx, sy), (x, sy*WHEEL_Y, AXLE_Z))
            g = m.n(("G", sx, sy), (x, sy*WHEEL_Y, 0.0))
            t = m.n(("Tie", sx, sy), (x, sy*TIE_Y, TIE_Z))
            if k_clamp is None:
                body = c                                   # 기존: 레일→축 강체
                m.e(r, c, RIG, "클램프")
            else:
                # 레일 중심 → 레일 밑면 (강체) → 힌지 조인트 (θx 만 스프링) → 몸체 기준점
                k0 = m.n(("K0", sx, sy), (x, sy*RAIL_Y, HINGE_Z))
                k1 = m.n(("K1", sx, sy), (x, sy*RAIL_Y, HINGE_Z))
                m.e(r, k0, RIG_SHORT, "클램프")
                kth = KR_R if not np.isfinite(k_clamp) else float(k_clamp)
                m.e(k0, k1, joint((KR_T, KR_T, KR_T, kth, KR_R, KR_R)), "클램프힌지")
                m.e(k1, c, RIG, "클램프")
                body = k1
            m.e(c, g, RIG, "바퀴")
            m.e(body, t, RIG, "클램프")
            if h_cross and k_tongue is not None:
                tb = m.n(("Tb", sx, sy), (x, sy*TONGUE_Y, TONGUE_Z))
                m.e(body, tb, RIG, "클램프")                # 혀 뿌리~볼트 중간: 몸체와 강체
                kz = series(k_tongue, k_joint)
                m.e(tb, m.idx[("H", sx, sy)], joint((0, 0, kz, 0, 0, 0)), "혀스프링")
            ground.append(g)
        if tie:
            if tie_local is None:
                m.e(m.idx[("Tie", sx, -1)], m.idx[("Tie", sx, 1)], tie_prop, "액슬타이")
            else:
                ends = []
                for sy in (-1, 1):
                    t2 = m.n(("Tie2", sx, sy), (sx * AXLE_X, sy*TIE_Y, TIE_Z))
                    m.e(m.idx[("Tie", sx, sy)], t2,
                        joint((KR_T, tie_local, KR_T, KR_R, KR_R, KR_R)), "타이국부")
                    ends.append(t2)
                m.e(ends[0], ends[1], tie_prop, "액슬타이")
    return m, ground


CASES = ("중앙", "한쪽구석", "뒤축위")


def case_loads(m, case):
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
    else:
        raise ValueError(case)
    return loads


KY_ROLLER = 0.1      # 접지 y 를 놓아줄 때 강체모드만 죽이는 아주 약한 스프링 (N/mm)


def run(case, tie=True, ground="old", **model_kw):
    """
    ground = "old"   접지 3방향 구속 + 첫 접지점 회전 구속 (기존 결과 재현)
             "pin"   접지 3방향 구속만 (4점 핀 — 회전은 전부 자유)
             "roller" x, z 구속 + y 는 KY_ROLLER 스프링 (바퀴가 옆으로 벌어질 수 있다 — 캠버 비교용)
    model_kw 는 build() 로 간다 (k_clamp, h_cross, k_tongue, k_joint, tie_local, tie_prop).
    """
    m, gnd = build(tie, **model_kw)
    loads = case_loads(m, case)
    fixed = set()
    for g in gnd:
        for i in (0, 2) if ground == "roller" else (0, 1, 2):
            fixed.add(6*g + i)
    if ground == "old":
        fixed.add(6*gnd[0] + 3); fixed.add(6*gnd[0] + 4); fixed.add(6*gnd[0] + 5)
    if ground == "roller":
        for g in gnd:
            gk = m.n(("Gk", g), m.nodes[g])
            m.e(g, gk, joint((0, KY_ROLLER, 0, 0, 0, 0)), "접지y")
            for i in range(6):
                fixed.add(6*gk + i)
    U, R, store = solve(m, loads, fixed)
    return m, U, R, store, gnd


# ---------------------------------------------------------------- 결과 추출
def wheel_metrics(m, U, R, ground):
    """바퀴별 캠버(바깥으로 벌어짐 +), 축 노드 y 변위(바깥 +), 접지 반력."""
    out = {}
    for k, i in m.idx.items():
        if k[0] != "C":
            continue
        _, sx, sy = k
        g = m.idx[("G", sx, sy)]
        cam = sy * U[6*i + 3]
        uy = sy * U[6*i + 1]
        out[(sx, sy)] = dict(
            camber_mrad=cam * 1e3,                   # θx, 접지가 바깥으로 벌어지면 +
            uy_out=uy,                               # 축 노드 y, 바깥 +
            splay=uy + cam * AXLE_Z,                 # 접지점 옆이동 (바퀴 강체) = 롤러일 때 실제 벌어짐
            fz=abs(R[6*g + 2]), fy=R[6*g + 1])
    return out


def member_forces(store, U, tag):
    return [kl @ (T @ U[dofs]) for dofs, T, kl, L, p, t, a, b in store if t == tag]


def summarize(m, U, R, store, ground):
    dz = U.reshape(-1, 6)[:, 2]
    w = wheel_metrics(m, U, R, ground)
    worst = max(w.values(), key=lambda d: abs(d["camber_mrad"]))
    tongue = [abs(f[2]) for f in member_forces(store, U, "혀스프링")]
    tie = [abs(f[6]) for f in member_forces(store, U, "액슬타이")]
    return dict(max_dz=float(abs(dz).max()), wheels=w,
                camber=float(worst["camber_mrad"]), uy=float(worst["uy_out"]),
                splay=float(worst["splay"]),
                tongue_max=max(tongue) if tongue else 0.0,
                tie_max=max(tie) if tie else 0.0)


# ---------------------------------------------------------------- 검증
def verify_hinge(kth=K_CLAMP_PETG):
    """겹친 두 노드, 한쪽 고정, 다른 쪽에 Mx → θx = M/kθ 인지. 다른 5 자유도는 0 이어야 한다."""
    m = Model()
    a = m.n("a", (0, 0, 0)); b = m.n("b", (0, 0, 0))
    m.e(a, b, joint((KR_T, KR_T, KR_T, kth, KR_R, KR_R)), "힌지")
    M = 44000.0
    U, R, _ = solve(m, [(b, np.array([0, 0, 0, M, 0, 0]))], set(range(6)))
    th = U[6*b + 3]
    others = np.abs(np.delete(U[6*b:6*b+6], 3)).max()
    return dict(k_target=kth, k_measured=M / th, theta=th, others=others)


def verify_clamp_assembly(k_clamp=K_CLAMP_PETG, k_tongue=None, k_joint=None,
                          tie_local=None, tie=False, M=M_REF):
    """단품 FEA 와 같은 상황: 레일 노드 고정, 힌지+보강 스프링, 몸체에 Mx → 캠버.
    타이는 반대쪽이 거울이므로 트러스 중간점을 고정(2EA/L)한 반쪽 모델로 본다."""
    m = Model()
    r = m.n("r", (0, RAIL_Y, HINGE_Z)); k1 = m.n("k1", (0, RAIL_Y, HINGE_Z))
    m.e(r, k1, joint((KR_T, KR_T, KR_T, k_clamp, KR_R, KR_R)), "힌지")
    fixed = set(range(6))
    if k_tongue is not None:
        tb = m.n("tb", (0, TONGUE_Y, TONGUE_Z)); th = m.n("th", (0, TONGUE_Y, RAIL_Z))
        m.e(k1, tb, RIG, "몸체"); m.e(tb, th, joint((0, 0, series(k_tongue, k_joint), 0, 0, 0)), "혀")
        fixed |= set(range(6*th, 6*th + 6))
    if tie:
        t = m.n("t", (0, TIE_Y, TIE_Z)); t2 = m.n("t2", (0, TIE_Y, TIE_Z)); mid = m.n("mid", (0, 0, TIE_Z))
        m.e(k1, t, RIG, "몸체")
        m.e(t, t2, joint((KR_T, tie_local if tie_local else KR_T, KR_T, KR_R, KR_R, KR_R)), "타이국부")
        half = dict(TIE_PROF, A=A30 * (TIE_Y / (P.TIE_PROF_LEN / 2)))   # 반쪽 길이 → 2EA/L 과 같게
        m.e(t2, mid, half, "타이")
        fixed |= set(range(6*mid, 6*mid + 6))
    U, R, _ = solve(m, [(k1, np.array([0, 0, 0, M, 0, 0]))], fixed)
    return U[6*k1 + 3]


# ---------------------------------------------------------------- 보강 비교 연구
def configs(material="petg"):
    if material == "petg":
        kc, kt, kl = K_CLAMP_PETG, K_TONGUE, K_TIE_LOCAL
    else:
        kc, kt, kl = K_CLAMP_AL, K_TONGUE_AL, K_TIE_LOCAL_AL
    return {
        "보강 없음":          dict(tie=False, k_clamp=kc),
        "3030 타이":          dict(tie=True, k_clamp=kc, tie_local=kl, tie_prop=TIE_PROF),
        "H+혀 (스프링너트)":   dict(tie=False, k_clamp=kc, h_cross=True, k_tongue=kt,
                                  k_joint=K_JOINT["springnut"]),
        "H+혀 (T너트)":       dict(tie=False, k_clamp=kc, h_cross=True, k_tongue=kt,
                                  k_joint=K_JOINT["tnut"]),
    }


def study(ground="roller", material="petg"):
    rows = []
    for name, kw in configs(material).items():
        for case in CASES:
            m, U, R, store, gnd = run(case, ground=ground, **kw)
            s = summarize(m, U, R, store, gnd)
            rows.append(dict(config=name, case=case, **s))
    return rows


WHEEL_ORDER = [(1, 1), (1, -1), (-1, 1), (-1, -1)]


def _fmt_wheels(w, key="camber_mrad", fmt="+.2f"):
    return " / ".join(f"{w[k][key]:{fmt}}" for k in WHEEL_ORDER)


def write_report(path, rows_roller, rows_pin, checks, rows_extra):
    L = []
    L.append("# H 크로스 + 혀 — 차량 전체 보요소 평가\n")
    L.append("`tools/frame/frame_fea.py` (12-DOF 3D 빔, numpy). 실행: "
             "`.venv/Scripts/python.exe tools/frame/frame_fea.py`\n")
    L.append("하중: 차량 40 kg 상부 분산 + 사람 70 kg (중앙 / 한쪽구석 x=+365,y=+210 / 뒤축위 x=+365 양쪽). "
             "클램프 = 레일 밑면 힌지(x축 회전 스프링), 보강은 부재/스프링. 그림: `docs/images/67_frame_h_cross.png`\n")
    L.append("## 보정값 (단품 FEA PETG, 800 N 상향, 편심 55, 3 mm)\n")
    L.append("| 항목 | 값 | 근거 |")
    L.append("|---|---:|---|")
    L.append(f"| 힌지 위치 | y=±{RAIL_Y:.0f}, z={HINGE_Z:.2f} (레일 밑면) | 보강 없음 시트 Uy/θ = 0.6699/0.046160 = 14.51 mm, Uz 회귀 영점 y=-55.2 (`..._petg_e55mm_up`) |")
    L.append(f"| 클램프 몸체 kθ | {K_CLAMP_PETG/1e3:.0f} N·m/rad | 44.0 N·m / 46.160 mrad |")
    L.append(f"| 혀 볼트 중간 | y=±{TONGUE_Y:.1f} (힌지에서 {TONGUE_ARM:.1f} mm) | 볼트 로컬 y -92/-122 (클램프 안쪽면 -77 에서 15/45). 과제의 ±165 는 레일 안쪽면 기준이라 8.3 mm 어긋난다 |")
    L.append(f"| 혀 강성 k_t | {K_TONGUE:,.0f} N/mm | 구멍 벽이 강체 고정이라 700/Uz(구멍 노드) 는 0 으로 나눔 → 대신 힌지+z스프링(조인트 강체)이 단품 캠버 7.439 mrad (`..._up_tongue`) 를 재현하도록 (44,000/0.007439 − 953,206)/53.3² . 이때 혀 힘 = {K_TONGUE*TONGUE_ARM*THETA_TONGUE_FEA:.0f} N (단품 볼트 반력 655.8 N) |")
    L.append(f"| 조인트 | 스프링너트 23,430 / T너트 98,415 N/mm | 브라켓 1개 = 볼트 1개, 혀와 직렬 → {series(K_TONGUE, 23430):,.0f} / {series(K_TONGUE, 98415):,.0f} N/mm |")
    L.append(f"| 타이 국부 스프링 k_loc | {K_TIE_LOCAL:,.0f} N/mm (양 끝) | 단품 타이(접지 스프링 155,633 N/mm)가 7.576 mrad (`..._up_tieprof`) → (44,000/0.007576 − 953,206)/32.5² = {_k_tie_series:,.0f} 의 직렬 보정. 이걸 빼고 순수 트러스만 두면 캠버가 0.3 mrad 로 단품과 25배 어긋난다 |")
    L.append("")
    L.append("## 검증\n")
    for c in checks:
        L.append(f"- {c}")
    L.append("")
    L.append("## 결과 — 접지 롤러 (x,z 구속, y 자유: 바퀴가 옆으로 벌어질 수 있다) — **주 결과**\n")
    L.append("캠버 = 축 노드 θx, 접지가 바깥으로 벌어지는 방향 +. 네 바퀴 순서 (+x,+y) / (+x,−y) / (−x,+y) / (−x,−y). "
             "'최악' = |캠버| 최대 바퀴. 축 y = 축 노드 y 변위 (바깥 +). 접지 벌어짐 = 축 y + 캠버·79.35 (접지점 옆이동). "
             "최대처짐 = 전체 노드 |Uz| 최대. 힘 = 혀 스프링 z 힘 / 타이 축력 최대.\n")
    L.append("| 구성 | 하중 | 최악 캠버 mrad | 네 바퀴 캠버 mrad | 최악 축 y mm | 네 바퀴 축 y mm | 접지 벌어짐 mm | 최대처짐 mm | 혀/타이 힘 N | 바퀴하중 N |")
    L.append("|---|---|---:|---|---:|---|---:|---:|---:|---|")
    for r in rows_roller:
        w = r["wheels"]
        fz = sorted(d["fz"] for d in w.values())
        force = r["tongue_max"] or r["tie_max"]
        L.append(f"| {r['config']} | {r['case']} | {r['camber']:+.2f} | {_fmt_wheels(w)} | {r['uy']:+.3f} | "
                 f"{_fmt_wheels(w, 'uy_out', '+.3f')} | {r['splay']:+.2f} | "
                 f"{r['max_dz']:.3f} | {force:.0f} | {fz[0]:.0f}~{fz[-1]:.0f} |")
    L.append("")
    L.append("## 결과 — 접지 핀 (x,y,z 구속: 타이어 옆미끄럼 없음)\n")
    L.append("접지 y 를 잡으면 타이어 마찰이 바퀴를 못 벌어지게 잡는다 — 캠버 대신 접지 옆힘(마찰 요구)이 생긴다. "
             "이 경계조건에서 타이는 거의 아무것도 안 한다(접지가 이미 옆을 잡는다).\n")
    L.append("| 구성 | 하중 | 최악 캠버 mrad | 네 바퀴 캠버 mrad | 최대처짐 mm | 접지 옆힘 최대 N | 바퀴하중 N |")
    L.append("|---|---|---:|---|---:|---:|---|")
    for r in rows_pin:
        w = r["wheels"]
        fz = sorted(d["fz"] for d in w.values())
        fy = max(abs(d["fy"]) for d in w.values())
        L.append(f"| {r['config']} | {r['case']} | {r['camber']:+.2f} | {_fmt_wheels(w)} | {r['max_dz']:.3f} | {fy:.0f} | {fz[0]:.0f}~{fz[-1]:.0f} |")
    L.append("")
    L.append("## 감도 (접지 롤러)\n")
    L.append("| 구성 | 하중 | 최악 캠버 mrad | 접지 벌어짐 mm | 최대처짐 mm | 힘 N |")
    L.append("|---|---|---:|---:|---:|---:|")
    for r in rows_extra:
        L.append(f"| {r['config']} | {r['case']} | {r['camber']:+.2f} | {r['splay']:+.2f} | {r['max_dz']:.3f} | "
                 f"{(r['tongue_max'] or r['tie_max']):.0f} |")
    L.append("")
    Path(path).write_text("\n".join(L) + "\n", encoding="utf-8")


def append_interpretation(path, rows_roller, rows_pin, rows_extra):
    def get(rows, cfg, case, key="camber"):
        return next(r for r in rows if r["config"] == cfg and r["case"] == case)[key]
    L = ["## 해석\n"]
    n0, nt, ns, nn = "보강 없음", "3030 타이", "H+혀 (스프링너트)", "H+혀 (T너트)"
    L.append(f"1. **대칭 하중(중앙·뒤축위)에서는 타이와 H+혀가 비슷하다.** 중앙 {get(rows_roller, nt, '중앙'):.1f} vs "
             f"{get(rows_roller, ns, '중앙'):.1f} mrad, 뒤축위 {get(rows_roller, nt, '뒤축위'):.1f} vs "
             f"{get(rows_roller, ns, '뒤축위'):.1f} mrad (보강 없음 {get(rows_roller, n0, '중앙'):.1f} / "
             f"{get(rows_roller, n0, '뒤축위'):.1f}). 단품에서 둘 다 ≈7.5 mrad 였던 것과 같은 그림이다.")
    L.append(f"2. **한쪽 구석 하중에서는 H+혀가 타이보다 낫다.** {get(rows_roller, ns, '한쪽구석'):.1f} vs "
             f"{get(rows_roller, nt, '한쪽구석'):.1f} mrad (보강 없음 {get(rows_roller, n0, '한쪽구석'):.1f}). "
             f"타이는 반대쪽 클램프를 끌어당겨야 하는데 그쪽 바퀴는 거의 떠 있어(하중 수십 N) 자기 힌지(953)와 프레임 코너 "
             f"수직강성으로만 버틴다 — 그래서 반대쪽 바퀴가 안쪽으로 {abs(get(rows_roller, nt, '한쪽구석', 'wheels')[(1, -1)]['camber_mrad']):.1f} mrad "
             f"기운다. 혀는 같은 자리 레일에 바로 반력을 넣으니 반대쪽 상태와 무관하다.")
    kj_s, kj_t = series(K_TONGUE, K_JOINT["springnut"]), series(K_TONGUE, K_JOINT["tnut"])
    L.append(f"3. **PETG 에서는 브라켓 조인트 종류가 거의 안 보인다.** 스프링너트 vs T너트 차이 "
             f"{(get(rows_roller, ns, '한쪽구석') / get(rows_roller, nn, '한쪽구석') - 1) * 100:.1f}% (한쪽구석). "
             f"혀 경로 강성 {K_TONGUE:,.0f} N/mm 이 조인트 23,430 보다 13배 무르니 직렬합 {kj_s:,.0f} / {kj_t:,.0f} 이 "
             f"거의 혀 값이다. '혀 ⊕ 조인트 직렬' 이라는 요점은 맞지만 PETG 몸체가 병목이라 조인트는 7%/2% 만 먹는다. "
             f"AL6061 클램프면 혀 경로가 {K_TONGUE_AL:,.0f} N/mm 로 올라 조인트가 병목이 된다 — 감도 표에서 스프링너트 vs T너트 "
             f"{get(rows_extra, 'AL6061 클램프 · H+혀 (스프링너트)', '한쪽구석'):.2f} vs "
             f"{get(rows_extra, 'AL6061 클램프 · H+혀 (T너트)', '한쪽구석'):.2f} mrad.")
    L.append(f"4. **혀 힘.** 한쪽구석에서 혀 스프링 힘 {get(rows_roller, ns, '한쪽구석', 'tongue_max'):.0f} N (볼트 2개 합, 바퀴하중 "
             f"{max(d['fz'] for d in get(rows_roller, ns, '한쪽구석', 'wheels').values()):.0f} N). 볼트당 ≈"
             f"{get(rows_roller, ns, '한쪽구석', 'tongue_max') / 2:.0f} N = 경제형 스프링너트 보수 인발 한계 1500 N 의 "
             f"{get(rows_roller, ns, '한쪽구석', 'tongue_max') / 2 / 1500 * 100:.0f}%. 이 힘은 크로스 양 끝 브라켓 조인트에 그대로 걸린다.")
    L.append(f"5. **H 크로스 막대 자체의 몫은 작다.** 혀 없이 크로스만 걸면 (감도 'H 크로스만') 한쪽구석 "
             f"{get(rows_extra, 'H 크로스만 (혀 없음)', '한쪽구석'):.1f} mrad (보강 없음 {get(rows_roller, n0, '한쪽구석'):.1f}) — "
             f"레일 비틀림을 조금 잡을 뿐이고, 캠버 개선은 거의 전부 혀에서 온다. 접지 핀에서도 크로스만은 9.47 → 8.37, 혀를 더해야 2.25.")
    L.append(f"6. **접지를 핀으로 잡으면(미끄럼 없는 바닥)** 캠버는 마찰이 막고 대신 접지 옆힘 "
             f"{max(abs(d['fy']) for d in get(rows_pin, n0, '한쪽구석', 'wheels').values()):.0f} N (바퀴하중의 "
             f"{max(abs(d['fy']) for d in get(rows_pin, n0, '한쪽구석', 'wheels').values()) / max(d['fz'] for d in get(rows_pin, n0, '한쪽구석', 'wheels').values()):.2f}배)이 필요하다. "
             f"그 경우 타이는 효과가 없고({get(rows_pin, n0, '한쪽구석'):.2f} → {get(rows_pin, nt, '한쪽구석'):.2f} mrad), "
             f"H+혀는 {get(rows_pin, ns, '한쪽구석'):.2f} mrad / 처짐 {get(rows_pin, n0, '한쪽구석', 'max_dz'):.3f} → "
             f"{get(rows_pin, ns, '한쪽구석', 'max_dz'):.3f} mm 로 줄인다. 실물에서 '벌어짐'이 보였으니 롤러 쪽이 현실에 가깝다.")
    L.append("7. **한계.** 클램프는 힌지 하나(x축 회전)로 축약했다 — 힌지 위치·강성은 단품 FEA 한 케이스(800 N 상향, 편심 55)에서 "
             "읽은 값이고 선형이다. 혀·타이 스프링은 같은 단품 FEA 를 재현하도록 맞췄으므로 단품과 다른 하중 방향/크기에서는 "
             "보정이 흔들린다. 브라켓 조인트는 z 축강성만, 슬립·예압·마찰 없음. 3030 비틀림 J 는 절반 근사.")
    with open(path, "a", encoding="utf-8") as f:
        f.write("\n".join(L) + "\n")


def plot(rows, png):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    for f in ("Malgun Gothic", "NanumGothic", "Gulim"):
        if any(x.name == f for x in font_manager.fontManager.ttflist):
            plt.rcParams["font.family"] = f
            break
    plt.rcParams["axes.unicode_minus"] = False
    names = list(dict.fromkeys(r["config"] for r in rows))
    fig, axes = plt.subplots(1, 4, figsize=(20, 5.4), dpi=150)
    cols = ["#9aa3ad", "#b6483c", "#2f6f9f", "#1f9a6d"]
    panels = [("camber", "최악 바퀴 캠버 |θx| [mrad]", "{:.1f}"),
              ("uy", "최악 바퀴 축 노드 y 변위 (바깥 +) [mm]", "{:+.2f}"),
              ("splay", "접지점 벌어짐 = 축 y + θ·79.35 [mm]", "{:+.2f}"),
              ("max_dz", "프레임 최대 처짐 [mm]", "{:.2f}")]
    for ax, (key, label, fmt) in zip(axes, panels):
        x = np.arange(len(CASES))
        wdt = 0.8 / len(names)
        for i, nm in enumerate(names):
            vals = [next(r for r in rows if r["config"] == nm and r["case"] == c)[key] for c in CASES]
            if key == "camber":
                vals = [abs(v) for v in vals]
            bars = ax.bar(x + (i - (len(names) - 1) / 2) * wdt, vals, wdt, label=nm, color=cols[i % 4])
            for b_, v in zip(bars, vals):
                ax.text(b_.get_x() + b_.get_width() / 2, max(b_.get_height(), 0), fmt.format(v),
                        ha="center", va="bottom", fontsize=7)
        ax.set_xticks(x); ax.set_xticklabels(CASES)
        ax.set_title(label, fontsize=10.5)
        ax.axhline(0, color="#444", lw=.6)
        ax.grid(axis="y", alpha=.3)
    axes[0].legend(fontsize=9, loc="upper left")
    fig.suptitle("차량 프레임 보요소 — 클램프 = 레일 밑면 힌지(PETG 953 N·m/rad) + 보강 부재 · 접지 롤러(옆 자유) · "
                 "차량 40 kg + 사람 70 kg\n타이 = 3030 트러스 + 양끝 국부 4,737 N/mm · 혀 = 1,747 N/mm 와 조인트(스프링너트 23,430 / T너트 98,415) 직렬",
                 fontsize=12)
    fig.tight_layout(rect=(0, 0, 1, 0.9))
    fig.savefig(png, facecolor="white", bbox_inches="tight")
    plt.close(fig)


def main():
    checks = []
    # (a) 힌지 요소 단독
    v = verify_hinge()
    checks.append(f"힌지 요소: kθ 목표 {v['k_target']:.0f} N·mm/rad, 측정 {v['k_measured']:.0f} "
                  f"(θx={v['theta']*1e3:.3f} mrad @44 N·m, 다른 5 자유도 최대 {v['others']:.1e})")
    # (b) 조립 스프링이 단품 캠버를 재현하는지
    t0 = verify_clamp_assembly()
    t1 = verify_clamp_assembly(k_tongue=K_TONGUE)
    t2 = verify_clamp_assembly(k_tongue=K_TONGUE, k_joint=K_JOINT["springnut"])
    t3 = verify_clamp_assembly(k_tongue=K_TONGUE, k_joint=K_JOINT["tnut"])
    t4 = verify_clamp_assembly(tie=True, tie_local=K_TIE_LOCAL)
    t5 = verify_clamp_assembly(tie=True)
    checks.append(f"레일 고정 + 44 N·m: 몸체만 {t0*1e3:.3f} mrad (단품 46.160) · 혀(조인트 강체) {t1*1e3:.3f} "
                  f"(단품 7.439) · 혀+스프링너트 {t2*1e3:.3f} · 혀+T너트 {t3*1e3:.3f} · "
                  f"3030 타이+국부 {t4*1e3:.3f} (단품 7.576) · 타이 국부 없음 {t5*1e3:.3f}")
    # (c) 기존 결과 재현
    for label, kw in (("기존 경로 (k_clamp=None)", dict()), ("새 힌지 경로 kθ=∞", dict(k_clamp=np.inf))):
        m, U, R, store, gnd = run("한쪽구석", tie=True, ground="old", **kw)
        s = summarize(m, U, R, store, gnd)
        fz = sorted(d["fz"] for d in s["wheels"].values())
        checks.append(f"{label} · tie=True · 접지 old · 한쪽구석: 최대처짐 {s['max_dz']:.3f} mm, "
                      f"바퀴하중 {fz[0]:.0f}~{fz[-1]:.0f} N (기존 0.379 / 22~865)")
    m, U, R, store, gnd = run("한쪽구석", tie=True, ground="pin", k_clamp=np.inf)
    s = summarize(m, U, R, store, gnd)
    fz = sorted(d["fz"] for d in s["wheels"].values())
    checks.append(f"새 힌지 경로 kθ=∞ · 접지 pin(회전 자유): 최대처짐 {s['max_dz']:.3f} mm, 바퀴하중 {fz[0]:.0f}~{fz[-1]:.0f} N")
    for c in checks:
        print("[검증]", c)

    rows_roller = study("roller")
    rows_pin = study("pin")
    extra = []
    sens = [
        ("H 크로스만 (혀 없음)", dict(tie=False, k_clamp=K_CLAMP_PETG, h_cross=True)),
        ("H+혀, 조인트 강체", dict(tie=False, k_clamp=K_CLAMP_PETG, h_cross=True, k_tongue=K_TONGUE)),
        ("3030 타이, 국부 스프링 없음(순수 트러스)", dict(tie=True, k_clamp=K_CLAMP_PETG, tie_prop=TIE_PROF)),
        ("M8 전산볼트 타이 x2 + 국부", dict(tie=True, k_clamp=K_CLAMP_PETG, tie_local=K_TIE_LOCAL, tie_prop=TIE)),
        ("타이 + H+혀 (스프링너트) 같이", dict(tie=True, k_clamp=K_CLAMP_PETG, tie_local=K_TIE_LOCAL, tie_prop=TIE_PROF,
                                         h_cross=True, k_tongue=K_TONGUE, k_joint=K_JOINT["springnut"])),
        ("클램프 강체 (기존 모델) + 타이", dict(tie=True, tie_prop=TIE)),
    ]
    sens += [(f"AL6061 클램프 · {name}", kw) for name, kw in configs("al6061").items()]
    for name, kw in sens:
        for case in CASES:
            m, U, R, store, gnd = run(case, ground="roller", **kw)
            extra.append(dict(config=name, case=case, **summarize(m, U, R, store, gnd)))

    print(f"\n{'구성':16s} {'하중':8s} {'캠버':>8s} {'축y':>8s} {'벌어짐':>7s} {'처짐':>7s} {'힘':>6s}   네 바퀴 캠버")
    for r in rows_roller:
        print(f"{r['config']:16s} {r['case']:8s} {r['camber']:+8.3f} {r['uy']:+8.3f} {r['splay']:+7.3f} {r['max_dz']:7.3f} "
              f"{(r['tongue_max'] or r['tie_max']):6.0f}   {_fmt_wheels(r['wheels'])}")
    out_md = ROOT / "tools" / "frame" / "frame_h_cross_results.md"
    out_png = ROOT / "docs" / "images" / "67_frame_h_cross.png"
    write_report(out_md, rows_roller, rows_pin, checks, extra)
    append_interpretation(out_md, rows_roller, rows_pin, extra)
    plot(rows_roller, out_png)
    print(f"\nSAVED {out_md}\nSAVED {out_png}")
    return rows_roller, rows_pin, checks, extra


if __name__ == "__main__":
    main()
