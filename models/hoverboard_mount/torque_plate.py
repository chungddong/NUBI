# Design B — 레이저 절단 강판 토크 플레이트.
#
# 스플릿 클램프는 3축 밀링이 필요하다. 이쪽은 **5~6mm 강판 2D 절단**만으로 나온다.
# 축 D컷과 꼭 맞는 컷아웃이 반력토크를 받고, 별도 리테이너가 축 빠짐을 막는다.
#   장점: 값이 싸다, 강판이라 강도 여유가 크다
#   단점: 축 방향 구속이 약해 리테이너가 필수, 면압이 판 두께에만 걸린다

from build123d import *

import parameters as P
from axle import create_axle_profile, plane_at_y
from adapter_plate import clamp_mount_xz, tnut_xz


def torque_plate():
    """축 D컷 컷아웃 + 어댑터 체결공. 판 하나로 끝난다."""
    t = P.TORQUE_PLATE_T
    # 어댑터 **바깥면에 바로 맞댄다**. 처음엔 클램프 자리에 세워서 어댑터와 32mm
    # 떠 있었다 — 볼트로 이을 수 없는 배치였다.
    # plane_at_y + extrude 는 **-Y 로** 뻗는다. 어댑터 바깥에 놓으려면 평면을
    # 판 두께만큼 더 바깥에 둬야 한다 (그냥 PLATE_FACE_Y 로 뒀더니 어댑터 속에 박혔다).
    y0 = P.PLATE_FACE_Y + P.TORQUE_PLATE_T
    cz = P.AXLE_Z
    with BuildPart() as p:
        with BuildSketch(plane_at_y(y0)):
            with Locations((0, cz)):
                RectangleRounded(P.TORQUE_PLATE_W, P.TORQUE_PLATE_H, 8)
        extrude(amount=t)
        with BuildSketch(plane_at_y(y0)) as sk:                  # 축 컷아웃 (D컷)
            with Locations((0, cz)):
                add(create_axle_profile(P.CLAMP_FIT))
        extrude(to_extrude=sk.sketch, amount=t + 2, mode=Mode.SUBTRACT)
        with BuildSketch(plane_at_y(y0)) as sk:                  # 어댑터 체결 4
            with Locations(*[(x, cz + z) for x, z in clamp_mount_xz()]):
                Circle(P.CLAMP_BOLT * 0.55)
        extrude(to_extrude=sk.sketch, amount=t + 2, mode=Mode.SUBTRACT)
    p.part.label = "torque_plate"
    return p.part


def axle_retainer():
    """축 리테이너 — **분할 칼라**. 토크판 바깥에서 축을 물어 축방향을 막는다.

    처음엔 평판으로 만들어 어댑터 안쪽 면에 붙이려 했는데, 거기는 프로파일 자리라
    4430mm3 물렸다. 애초에 **평판은 축방향 구속을 못 한다** — 조여서 물어야 한다.
    어댑터 쪽 카운터보어가 안쪽 이동을 막고, 이 칼라가 바깥쪽 이동을 막는다.
    원래 호버보드 축 캡과 병행해도 된다.
    """
    # 토크판 체결 볼트 **머리(4mm) 바깥**에 앉혀야 한다. 바로 붙여놨더니 머리와
    # 215mm3 물렸다. 평면 구간(-70.6~-27.8) 안이라 D컷은 그대로 문다.
    y0 = P.PLATE_FACE_Y + P.TORQUE_PLATE_T + 6 + P.RETAINER_T * 2
    cz = P.AXLE_Z
    w, h = P.AXLE_DIAMETER * 3.2, P.AXLE_DIAMETER * 2.0
    with BuildPart() as p:
        with BuildSketch(plane_at_y(y0)):
            with Locations((0, cz)):
                RectangleRounded(w, h, 5)
        extrude(amount=P.RETAINER_T * 2)
        with BuildSketch(plane_at_y(y0)) as sk:                  # 축 구멍 (D컷)
            with Locations((0, cz)):
                add(create_axle_profile(P.CLAMP_FIT))
        extrude(to_extrude=sk.sketch, amount=P.RETAINER_T * 2 + 2, mode=Mode.SUBTRACT)
        with Locations((0, y0 - P.RETAINER_T, cz - P.AXLE_DIAMETER / 2 - 3)):
            Box(1.2, P.RETAINER_T * 2 + 2, 12, mode=Mode.SUBTRACT)   # 핀치 슬릿
        with BuildSketch(plane_at_y(y0)) as sk:                  # 핀치 볼트 2
            with Locations(*[(sx * P.AXLE_DIAMETER * 1.25, cz - P.AXLE_DIAMETER / 2 - 3)
                             for sx in (-1, 1)]):
                Circle(2.6)
        extrude(to_extrude=sk.sketch, amount=P.RETAINER_T * 2 + 2, mode=Mode.SUBTRACT)
    p.part.label = "axle_retainer"
    return p.part
