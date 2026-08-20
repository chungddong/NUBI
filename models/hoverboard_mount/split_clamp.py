# Design A — CNC 알루미늄 2피스 스플릿 클램프.
#
# 핵심은 **축 평면을 형상으로 문다**는 것이다. 원형 구멍에 마찰로만 잡으면
# 구동/제동 반력토크에 미끄러진다. 포켓이 D컷과 맞물려 토크를 프레임으로 보낸다.
# 상·하 사이에는 반드시 **핀치 갭**을 둔다 — 없으면 축을 물기 전에 두 블록이
# 먼저 맞닿아 조임이 안 걸린다.

from build123d import *

import parameters as P
from axle import create_axle_profile, plane_at_y


def _bolt_xy():
    return [(sx * P.CLAMP_BOLT_X, sy * P.CLAMP_BOLT_Y)
            for sx in (-1, 1) for sy in (-1, 1)]


def _pocket(depth_extra=4.0):
    """축 포켓 (클램프 좌표: 축 중심 원점, 축이 Y)."""
    with BuildPart() as p:
        with BuildSketch(plane_at_y(P.CLAMP_DEPTH / 2 + depth_extra / 2)):
            add(create_axle_profile(P.CLAMP_FIT))
        extrude(amount=P.CLAMP_DEPTH + depth_extra)
    return p.part


def _half(upper):
    """상 또는 하 블록. 분할면은 축 중심(z=0)이고 갭의 절반씩 물러난다."""
    h = P.CLAMP_UPPER_H if upper else P.CLAMP_LOWER_H
    g = P.CLAMP_PINCH_GAP / 2
    z0 = g if upper else -g - h
    with BuildPart() as p:
        with Locations((0, 0, z0 + h / 2)):
            Box(P.CLAMP_WIDTH, P.CLAMP_DEPTH, h)
        try:
            fillet(p.edges().filter_by(Axis.Z), radius=P.CLAMP_FILLET)
        except Exception:
            pass
        add(_pocket(), mode=Mode.SUBTRACT)                     # 축 포켓
        for x, y in _bolt_xy():                                # 클램프 볼트
            with Locations((x, y, z0 + h / 2)):
                if upper:                                      # 상: 통과 + 카운터보어
                    Cylinder(P.CLAMP_BOLT * 0.55, h + 2, mode=Mode.SUBTRACT)
                    with Locations((0, 0, h / 2 - 3)):
                        Cylinder(P.CLAMP_BOLT * 0.85, 8, mode=Mode.SUBTRACT)
                else:                                          # 하: 탭 하공
                    Cylinder((P.CLAMP_BOLT - 1.25) / 2, h + 2, mode=Mode.SUBTRACT)
        try:                                                   # 포켓 내부 코너
            fillet(p.edges().filter_by(Axis.Y).group_by(Axis.Z)[0 if upper else -1],
                   radius=P.CLAMP_POCKET_FILLET)
        except Exception:
            pass
    p.part.label = "axle_clamp_upper" if upper else "axle_clamp_lower"
    return p.part


def axle_clamp_upper():
    return _half(True)


def axle_clamp_lower():
    return _half(False)


def placed(part):
    """클램프 좌표 → 조립 좌표 (축 중심 y=CLAMP_CY, z=AXLE_Z)."""
    return Pos(0, P.CLAMP_CY, P.AXLE_Z) * part
