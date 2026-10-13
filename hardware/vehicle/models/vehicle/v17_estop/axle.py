# 축 단면·형상 — 여기 하나만 고치면 클램프·토크플레이트 포켓이 전부 따라온다.

from build123d import *

import parameters as P


def plane_at_y(y):
    """global y 에 놓인 XZ 평면. **extrude 는 -Y 로 뻗는다.**

    Plane.XZ 의 법선이 -Y 라 `.offset(d)` 는 y = -d 에 놓인다. 이 부호를 이 패키지
    안에서만 세 번 틀렸다 (축 두 조각 / 모터 엉뚱한 자리 / 포켓이 클램프 밖).
    직접 offset 을 쓰지 말고 이 함수를 쓴다.
    """
    return Plane.XZ.offset(-y)


def create_axle_profile(clearance=0.0, flat=True):
    """축 단면 (원점 중심, XY 스케치). 평면은 **+Y** 쪽에 온다.

    clearance : 반경 방향으로 키울 양 (포켓용). 평면 쪽도 같이 밀린다.
    flat      : False 면 원형 구간 (평면 밖 영역)
    """
    r = P.AXLE_DIAMETER / 2 + clearance
    with BuildSketch() as sk:
        Circle(r)
        if flat and P.AXLE_FLAT_COUNT:
            # 평면 높이 = (평면~반대편 원호) - 반지름.  D컷 1면이면 +Y 만,
            # 2면이면 ±Y 를 같이 깎는다.
            h = P.AXLE_ACROSS_FLAT - P.AXLE_DIAMETER / 2 + clearance
            for sy in ((1,) if P.AXLE_FLAT_COUNT == 1 else (1, -1)):
                with Locations((0, sy * (h + r))):
                    Rectangle(4 * r, 2 * r, mode=Mode.SUBTRACT)
    return sk.sketch


def axle_solid(clearance=0.0):
    """축 실물 (하우징 면 y0 에서 -y 로 뻗는다). 평면 구간만 D컷."""
    y0 = P.AXLE_START_Y                              # 축이 하우징에서 나오는 지점
    f0 = P.AXLE_FLAT_Y0                              # 평면 시작
    f1 = f0 - P.AXLE_FLAT_LENGTH                     # 평면 끝
    y_end = P.AXLE_END_Y                             # 축 끝
    r = P.AXLE_DIAMETER / 2 + clearance
    with BuildPart() as p:
        # 원형 구간 전체를 먼저 만들고, 평면 구간만 깎는다
        with BuildSketch(plane_at_y(y0)):
            Circle(r)
        extrude(amount=y0 - y_end)
        with BuildSketch(plane_at_y(f0)):
            add(create_axle_profile(clearance))
        extrude(amount=P.AXLE_FLAT_LENGTH, mode=Mode.INTERSECT)
        # 평면 밖(하우징 쪽 / 끝쪽)은 원형으로 되살린다
        # Plane.XZ 의 법선은 **-Y** 다. offset(-y) 로 그 자리에 놓이고 extrude 는
        # -y 로 뻗는다. min() 을 쓰면 반대편으로 뻗어 축이 두 조각이 된다.
        for a, b in ((y0, f0), (f1, y_end)):
            if abs(a - b) < 0.01:
                continue
            with BuildSketch(plane_at_y(max(a, b))):
                Circle(r)
            extrude(amount=abs(a - b))
        if P.AXLE_END_CHAMFER > 0:
            try:
                chamfer(p.edges().group_by(Axis.Y)[0], length=P.AXLE_END_CHAMFER)
            except Exception:
                pass
        if P.CABLE_EXIT_DIAMETER > 0:                # 축 중심 배선 구멍
            with BuildSketch(plane_at_y(y0)):
                Circle(P.CABLE_EXIT_DIAMETER / 2)
            extrude(amount=y0 - y_end + 1, mode=Mode.SUBTRACT)
    p.part.label = "axle"
    return p.part


def flat_span():
    """평면 구간의 y 범위 (바깥, 안쪽)."""
    return P.AXLE_FLAT_Y0, P.AXLE_FLAT_Y0 - P.AXLE_FLAT_LENGTH
