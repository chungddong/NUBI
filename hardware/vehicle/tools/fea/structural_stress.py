r"""특이점을 뺀 **구조응력**을 뽑는다.

강체로 고정한 볼트 구멍 벽은 수학적 특이점이라 메쉬를 조일수록 응력이 계속
올라간다 (4mm 132.2 → 3mm 150.1 MPa, +13.5%). 그 값을 설계 응력으로 인용하면
안 된다. 구멍에서 조금만 벗어나면 값이 수렴한다 (45.1 → 45.3, +0.4%).

쓰는 법:
    .venv\Scripts\python.exe tools\fea\structural_stress.py            # 기본 3케이스
    .venv\Scripts\python.exe tools\fea\structural_stress.py <폴더명> ...
"""

import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools" / "fea" / "python"))
sys.path[:0] = [str(ROOT / "models" / "hoverboard_mount"), str(ROOT / "models")]

import meshio                                                    # noqa: E402

import low_clamp as LC                                           # noqa: E402
import parameters as P                                           # noqa: E402

FEA = ROOT / "models" / "hoverboard_mount" / "output" / "fea"
EXCLUDE = 3.0            # 구멍 벽에서 이만큼 밖을 '구조' 영역으로 본다
DEFAULT = ["clamp_800N_h4mm_e55mm_up", "clamp_800N_h3mm_e55mm_up",
           "clamp_800N_h3mm_petg_e55mm_up"]


def structural(folder, exclude=EXCLUDE):
    """(최대응력, 구조응력, 최대변위, 최대점 좌표) 를 낸다."""
    vtu = next((FEA / folder).glob("*.vtu"))
    m = meshio.read(str(vtu))
    s = np.asarray(m.point_data["S_Mises"]).ravel()
    u = np.linalg.norm(np.asarray(m.point_data["U"]), axis=1)
    pts = m.points
    r = P.TNUT_BOLT * 0.55                                   # 볼트 구멍 반경
    x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
    # 강체 고정한 구멍 벽 전부에서 exclude 만큼 밖 — 형상 스위치(LOW_SIDE/LOW_SHELF)를
    # 케이스와 **같은 환경변수**로 맞춰 돌려야 한다. 안 맞으면 특이점을 구조응력으로 센다.
    dist = np.full(len(pts), np.inf)
    for sx in (-1, 1):
        dist = np.minimum(dist, np.abs(np.hypot(x - sx * P.LOW_POST_X, y - LC.CLAMP_CY) - r))
        if P.LOW_SIDE:
            dist = np.minimum(dist, np.abs(np.hypot(x - sx * P.LOW_POST_X, z - LC.BOLT_Z) - r))
    if P.LOW_SHELF:
        for by in LC.SHELF_BOLT_YS:
            dist = np.minimum(dist, np.abs(np.hypot(x, y - by) - r))
    far = dist > exclude
    return s.max(), s[far].max(), u.max(), pts[int(np.argmax(s))]


if __name__ == "__main__":
    names = sys.argv[1:] or DEFAULT
    print(f"구멍 벽에서 {EXCLUDE:.0f}mm 밖을 구조응력으로 본다 "
          f"(구멍 반경 {P.TNUT_BOLT*0.55:.2f}mm)")
    for n in names:
        try:
            mx, st, disp, at = structural(n)
        except (StopIteration, FileNotFoundError):
            print(f"  {n:38s} 결과 없음")
            continue
        print(f"  {n:38s} 최대 {mx:6.1f} (특이점, x={at[0]:+.1f} y={at[1]:+.1f} "
              f"z={at[2]:+.1f})  구조 {st:5.1f} MPa  변위 {disp:.4f} mm")
