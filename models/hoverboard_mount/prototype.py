# 프로토타입 3D 프린트 판 — **끼움·조립 확인용이지 주행용이 아니다.**
#
# 금속 가공 전에 확인할 것: 축이 포켓에 들어가는가 / 볼트 위치가 맞는가 /
# 케이블이 나올 자리가 있는가 / 공구가 들어가는가.
# FDM 오차를 얹는다: 축 포켓은 수축분만큼 키우고, 탭은 플라스틱 하공으로.

import sys
from pathlib import Path

sys.path[:0] = [str(Path(__file__).parent)]

from build123d import *                                          # noqa: E402

import parameters as P                                           # noqa: E402

PROTO_FIT = 0.35        # FDM 은 내부 치수가 작게 나온다 (금속 0.05 → 0.35)
PROTO_TAP = 1.0         # 플라스틱 탭 하공 여유


def prototype_axle_clamp():
    """축 포켓만 키운 클램프 상·하. 형상은 금속판과 같다."""
    fit, tap = P.CLAMP_FIT, P.CLAMP_BOLT
    P.CLAMP_FIT = PROTO_FIT
    try:
        import importlib

        import split_clamp
        importlib.reload(split_clamp)
        u, l = split_clamp.axle_clamp_upper(), split_clamp.axle_clamp_lower()
    finally:
        P.CLAMP_FIT = fit
    u.label, l.label = "prototype_axle_clamp_upper", "prototype_axle_clamp_lower"
    return [u, l]


if __name__ == "__main__":
    OUT = Path(__file__).parent / "output" / "stl"
    OUT.mkdir(parents=True, exist_ok=True)
    for o in prototype_axle_clamp():
        export_stl(o.moved(Location()), str(OUT / f"{o.label}.stl"))
        print(f"  {o.label:34s} {o.volume/1000:5.1f} cm3  솔리드 {len(o.solids())}")
    print(f"  축 포켓 여유 {P.CLAMP_FIT} → {PROTO_FIT} (FDM 수축 보정)")
    print("  !! 이 부품은 fitment 확인용이다. 주행 하중을 받는 최종 마운트가 아니다.")
