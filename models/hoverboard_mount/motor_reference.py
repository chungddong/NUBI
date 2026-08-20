# 호버보드 모터 참조 모델 — **간섭 확인용**이지 정밀 모델이 아니다.
# 타이어 / 허브 캔 / 축 / D컷 / 배선 구멍만 있다.

from pathlib import Path

from build123d import *

import parameters as P
from axle import axle_solid, plane_at_y


REAL_MOTOR_STEP = (Path(__file__).parents[2]
                   / "refmodel" / "Motor, BLDC, Hoverboard Wheel, 6.5in.stp")
# STEP의 타이어 측면 X=5.85 / 51.35에서 얻은 실제 중심면.
REAL_MOTOR_WHEEL_CENTER_X = (5.85 + 51.35) / 2


def motor_reference():
    with BuildPart() as p:
        # Plane.XZ 법선이 -Y 라 offset 에 음수를 준다 (axle.py 와 같은 함정)
        with BuildSketch(plane_at_y(P.WHEEL_WIDTH / 2)):    # 타이어
            Circle(P.WHEEL_DIAMETER / 2)
            Circle(P.MOTOR_HOUSING_DIAMETER / 2, mode=Mode.SUBTRACT)
        extrude(amount=P.WHEEL_WIDTH)
        with BuildSketch(plane_at_y(P.MOTOR_HOUSING_WIDTH / 2)):  # 허브 캔
            Circle(P.MOTOR_HOUSING_DIAMETER / 2)
        extrude(amount=P.MOTOR_HOUSING_WIDTH)
    out = p.part
    out += axle_solid()
    out.label = "hoverboard_motor_reference"
    return Pos(0, 0, P.AXLE_Z) * out


def actual_motor_step_reference(axle_z=P.AXLE_Z):
    """실제 6.5인치 휠 모터 STEP을 Design D 좌표계에 정렬한다.

    원본은 축이 X축이고 긴 축이 -X 방향이다. 이를 클램프가 있는 -Y 방향으로
    돌리고, STEP에서 측정한 타이어 중심면을 y=0에 맞춘다. 축 중심 높이는 현재
    설계의 지면 기준 장면은 AXLE_Z를, 축 중심 기준인 Design D 장면은 0을 넘긴다.
    """
    motor = import_step(str(REAL_MOTOR_STEP))
    motor = (Pos(0, -REAL_MOTOR_WHEEL_CENTER_X, axle_z)
             * Rot(0, 0, 90) * motor)
    motor.label = "actual_hoverboard_wheel_motor_STEP"
    return motor
