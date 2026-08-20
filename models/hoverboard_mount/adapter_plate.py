# 어댑터 플레이트 — 클램프와 프로파일 사이의 **교체 가능한 인터페이스**.
#
# 클램프를 프로파일에 직접 붙이지 않는 이유:
#   · 모터 하중을 넓은 면으로 분산한다
#   · 클램프만 따로 교체·개량할 수 있다
#   · 나중에 이 판을 서스펜션 암 피벗으로 갈아끼울 수 있다 (§10)
# 2.5D 형상이라 레이저·워터젯·라우터 어느 쪽으로도 나온다.
#
# 판 중심을 **축 높이(AXLE_Z)** 에 맞춘다. 프로파일도 같은 높이에 두면 축 하중이
# 볼트 패턴 중심으로 곧게 들어가 여분의 모멘트가 안 생긴다.

from build123d import *

import parameters as P
from axle import plane_at_y


def clamp_mount_xz():
    """하부 클램프를 판에 붙이는 볼트 (x, z) — **축 중심 기준** z."""
    # z 간격은 볼트 **머리 지름**보다 커야 한다 (M8 머리 Ø13.6). 12 로 뒀더니
    # 위아래 볼트 머리가 서로 겹쳤다.
    return [(sx * P.CLAMP_BOLT_X, z)
            for sx in (-1, 1) for z in (-P.CLAMP_LOWER_H + 6, -2)]


def tnut_xz():
    """프로파일 T너트 볼트 (x, z) — 축 중심 기준. 슬롯 2줄을 모두 쓴다."""
    return [(sx * P.BOLT_X_PITCH / 2, sz * P.EXTRUSION_SLOT_PITCH / 2)
            for sx in (-1, 1) for sz in (-1, 1)]


def adapter_plate():
    t = P.ADAPTER_PLATE_THICKNESS
    cz = P.AXLE_Z                      # 스케치 로컬 v = global z
    with BuildPart() as p:
        with BuildSketch(plane_at_y(P.PLATE_FACE_Y)):
            with Locations((0, cz)):
                RectangleRounded(P.ADAPTER_PLATE_W, P.ADAPTER_PLATE_H,
                                 P.ADAPTER_CORNER_R)
        extrude(amount=t)
        with BuildSketch(plane_at_y(P.PLATE_FACE_Y)) as sk:      # T너트 통과공 4
            with Locations(*[(x, cz + z) for x, z in tnut_xz()]):
                Circle(P.TNUT_BOLT * 0.55)
        extrude(to_extrude=sk.sketch, amount=t + 2, mode=Mode.SUBTRACT)
        # 축이 판을 파고든다 (축 끝 -74.1, 판 -76.2~-68.2 → 5.9mm). 케이블도 축
        # 중심으로 나오므로 **관통 구멍**을 뚫어 둘을 한꺼번에 해결한다.
        # **카운터보어**로 판다. 축은 바깥쪽 절반까지만 들어가고 안쪽 살이 남아
        # 축이 안쪽으로 빠지는 걸 막는다. 케이블만 관통 구멍으로 나간다.
        _pocket_d = t - 3.0
        with BuildSketch(plane_at_y(P.PLATE_FACE_Y)) as sk:
            with Locations((0, cz)):
                Circle(P.AXLE_DIAMETER / 2 + P.CABLE_CLEARANCE)
        extrude(to_extrude=sk.sketch, amount=_pocket_d, mode=Mode.SUBTRACT)
        with BuildSketch(plane_at_y(P.PLATE_FACE_Y)) as sk:      # 케이블 관통
            with Locations((0, cz)):
                Circle(P.CABLE_EXIT_DIAMETER / 2 + P.CABLE_CLEARANCE)
        extrude(to_extrude=sk.sketch, amount=t + 2, mode=Mode.SUBTRACT)
        with BuildSketch(plane_at_y(P.PLATE_FACE_Y)) as sk:      # 클램프 탭 4
            with Locations(*[(x, cz + z) for x, z in clamp_mount_xz()]):
                Circle((P.CLAMP_BOLT - 1.25) / 2)
        extrude(to_extrude=sk.sketch, amount=t + 2, mode=Mode.SUBTRACT)
    p.part.label = "adapter_plate"
    return p.part
