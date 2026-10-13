r"""FEA 결과에서 "바퀴가 바깥으로 밀리는 양" 을 뽑는다.

    .venv/Scripts/python.exe tools/fea/seat_splay.py <case_dir> [<case_dir> ...]

최대 변위(max |U|)는 어디가 얼마나 움직였는지 말해 주지 않는다. 사용자가 실물에서
본 것은 **바퀴 접지점이 바깥으로 벌어지는 것**이라 그걸 직접 잰다:

  Uy_seat   축 시트 노드의 평균 y 변위 (+y = 바퀴 쪽 = 바깥)
  θx        시트가 x 축 둘레로 기운 각 (Uz 를 y 로 회귀) — 바깥쪽이 내려가면 음수
  splay     접지점 바깥 밀림 = Uy_seat − θx · R_wheel   (양수 = 바깥)

시트 노드는 run_clamp_fea.classify_nodes 와 같은 규칙으로 고른다 (반원 시트 벽,
z ≤ 0.15, 클램프 깊이 안). 좌표는 클램프 로컬(축 중심 z=0, 레일 y=CLAMP_CY).
"""

import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT / "tools" / "fea" / "python"),
                str(ROOT / "tools" / "fea" / "gmsh_pyenv" / "Lib" / "site-packages"),
                str(ROOT / "models" / "hoverboard_mount"), str(ROOT / "models")]

import meshio                                  # noqa: E402
import low_clamp as LC                         # noqa: E402
import parameters as P                         # noqa: E402

R_WHEEL = P.WHEEL_DIAMETER / 2


def seat_metrics(vtu: Path, tol: float = 0.3):
    mesh = meshio.read(vtu)
    pts = np.asarray(mesh.points)
    U = np.asarray(mesh.point_data["U"])[:, :3]
    seat_r = P.AXLE_DIAMETER / 2 + P.SADDLE_SEAT_FIT
    radial = np.hypot(pts[:, 0], pts[:, 2])
    y0, y1 = LC.CLAMP_CY - P.LOW_D / 2 - tol, LC.CLAMP_CY + P.LOW_D / 2 + tol
    seat = (np.abs(radial - seat_r) <= tol) & (pts[:, 2] <= 0.15) & (pts[:, 1] >= y0) & (pts[:, 1] <= y1)
    if seat.sum() < 20:
        raise RuntimeError(f"시트 노드가 {seat.sum()}개뿐 — 좌표계가 다른 모델인가?")
    y = pts[seat, 1]
    uy = U[seat, 1].mean()
    uz = U[seat, 2]
    theta = np.polyfit(y, uz, 1)[0]                     # rad, dUz/dy
    splay = uy - theta * R_WHEEL
    umax = np.linalg.norm(U, axis=1).max()
    return dict(n=int(seat.sum()), uy=float(uy), theta_mrad=float(theta * 1e3),
                splay=float(splay), umax=float(umax))


def main():
    print(f"{'케이스':46s} {'시트 Uy':>9s} {'기움 mrad':>10s} {'접지 밀림':>10s} {'최대변위':>9s}")
    for arg in sys.argv[1:]:
        d = Path(arg)
        vtus = sorted(d.glob("*.vtu"))
        if not vtus:
            print(f"{d.name:46s} (VTU 없음)")
            continue
        m = seat_metrics(vtus[-1])
        print(f"{d.name:46s} {m['uy']:+9.4f} {m['theta_mrad']:+10.4f} {m['splay']:+10.4f} {m['umax']:9.4f}")


if __name__ == "__main__":
    main()
