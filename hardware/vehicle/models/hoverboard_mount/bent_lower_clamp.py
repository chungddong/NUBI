"""Low-cost folded-steel replacement for the machined Design-D lower clamp.

The bracket is intentionally split into parts whose bend axes stay parallel:

* saddle_u: 6 mm SS400 U bracket carrying the wheel load
* rail_hat: 6 mm SS400 hat bracket wrapping the 3030 rail
* v_liner: 2.5 mm folded replaceable axle cradle

All profiles are suitable for laser cutting and press-brake forming.  The model
keeps the current axle, OEM upper clamp, rail, and M6 attachment coordinates.
"""

from build123d import Align, Box, Compound, Cylinder, Pos, Rot

import parameters as P
from low_clamp import (
    BOLT_Z,
    CLAMP_CY,
    RAIL_BOT,
    RAIL_CZ,
    button_head_screw_reference,
    oem_clamp_block,
    rail,
    side_button_head_bolts_reference,
)


SHEET_T = 6.0
LINER_T = 2.5
SADDLE_OUT_W = 76.0
SADDLE_DEPTH = 38.0
SADDLE_BASE_Z = -13.0
SADDLE_TOP = RAIL_BOT - SHEET_T
SADDLE_SIDE_H = SADDLE_TOP - (SADDLE_BASE_Z - SHEET_T / 2)

RAIL_WRAP_W = 92.0
RAIL_WRAP_DEPTH = P.EXTRUSION_W + 2 * P.LOW_RAIL_FIT + 2 * SHEET_T
RAIL_FLANGE_H = 24.0


def _box(x, y, z):
    return Box(x, y, z, align=(Align.CENTER, Align.CENTER, Align.CENTER))


def saddle_u():
    """6T U bracket, represented at the finished folded dimensions."""
    base = Pos(0, CLAMP_CY, SADDLE_BASE_Z) * _box(
        SADDLE_OUT_W, SADDLE_DEPTH, SHEET_T
    )
    for sx in (-1, 1):
        for sy in (-1, 1):
            hole = Pos(
                sx * P.OEM_BOLT[0],
                CLAMP_CY + sy * P.OEM_BOLT[1],
                SADDLE_BASE_Z,
            ) * Cylinder(
                (P.OEM_BOLT_D + 0.6) / 2,
                SHEET_T + 2,
                align=(Align.CENTER, Align.CENTER, Align.CENTER),
            )
            base = base - hole
    side_x = (SADDLE_OUT_W - SHEET_T) / 2
    side_z = (SADDLE_BASE_Z - SHEET_T / 2 + SADDLE_TOP) / 2
    left = Pos(-side_x, CLAMP_CY, side_z) * _box(
        SHEET_T, SADDLE_DEPTH, SADDLE_SIDE_H
    )
    right = Pos(side_x, CLAMP_CY, side_z) * _box(
        SHEET_T, SADDLE_DEPTH, SADDLE_SIDE_H
    )
    result = Compound(children=[base, left, right])
    result.label = "6T_SS400_축지지_U절곡"
    return result


def rail_hat():
    """6T inverted-U bracket clamping the lower and side faces of 3030."""
    floor_z = RAIL_BOT - SHEET_T / 2
    floor = Pos(0, CLAMP_CY, floor_z) * _box(
        RAIL_WRAP_W, RAIL_WRAP_DEPTH, SHEET_T
    )
    for sx in (-1, 1):
        hole = Pos(sx * P.LOW_POST_X, CLAMP_CY, floor_z) * Cylinder(
            P.TNUT_BOLT * 0.55,
            SHEET_T + 2,
            align=(Align.CENTER, Align.CENTER, Align.CENTER),
        )
        floor = floor - hole
    flange_y = P.EXTRUSION_W / 2 + P.LOW_RAIL_FIT + SHEET_T / 2
    flange_z = RAIL_BOT + RAIL_FLANGE_H / 2
    front = Pos(0, CLAMP_CY - flange_y, flange_z) * _box(
        RAIL_WRAP_W, SHEET_T, RAIL_FLANGE_H
    )
    rear = Pos(0, CLAMP_CY + flange_y, flange_z) * _box(
        RAIL_WRAP_W, SHEET_T, RAIL_FLANGE_H
    )
    for sx in (-1, 1):
        for sy, flange in ((-1, front), (1, rear)):
            hole = Pos(
                sx * P.LOW_POST_X,
                CLAMP_CY + sy * flange_y,
                BOLT_Z,
            ) * Cylinder(
                P.TNUT_BOLT * 0.55,
                SHEET_T + 2,
                rotation=(90, 0, 0),
                align=(Align.CENTER, Align.CENTER, Align.CENTER),
            )
            if sy < 0:
                front = front - hole
            else:
                rear = rear - hole
    result = Compound(children=[floor, front, rear])
    result.label = "6T_SS400_3030_감싸기절곡"
    return result


def v_liner():
    """Replaceable 90-degree V cradle made from a 2.5T bent strip."""
    leg_len = 14.0
    cx = leg_len / (2 * 2 ** 0.5)
    cz = -10.0 + cx
    left = Pos(-cx, CLAMP_CY, cz) * Rot(0, -45, 0) * _box(
        leg_len, SADDLE_DEPTH, LINER_T
    )
    right = Pos(cx, CLAMP_CY, cz) * Rot(0, 45, 0) * _box(
        leg_len, SADDLE_DEPTH, LINER_T
    )
    result = Compound(children=[left, right])
    result.label = "2.5T_교체형_V축받침"
    return result


def folded_lower_clamp():
    result = Compound(children=[saddle_u(), rail_hat(), v_liner()])
    result.label = "절곡형_하부클램프_3부품"
    return result


def bottom_button_head_bolts_reference(length=20.0):
    """Two M6x20 screws entering the bottom slot of the rail."""
    bolts = []
    head_seat_z = RAIL_BOT - SHEET_T
    for sx in (-1, 1):
        bolt = Pos(sx * P.LOW_POST_X, CLAMP_CY, head_seat_z) * button_head_screw_reference(length)
        bolt.label = f"M6x{length:g}_바닥_{'L' if sx < 0 else 'R'}"
        bolts.append(bolt)
    result = Compound(children=bolts)
    result.label = "M6x20_바닥볼트_2개"
    return result


def folded_assembly_reference():
    result = Compound(children=[
        folded_lower_clamp(),
        oem_clamp_block(),
        rail(),
        side_button_head_bolts_reference(20.0),
        bottom_button_head_bolts_reference(20.0),
    ])
    result.label = "절곡형_하부클램프_조립참조"
    return result


if __name__ == "__main__":
    from ocp_vscode import Camera, show
    from motor_reference import actual_motor_step_reference

    show(
        saddle_u(),
        rail_hat(),
        v_liner(),
        oem_clamp_block(),
        rail(),
        actual_motor_step_reference(axle_z=0.0),
        side_button_head_bolts_reference(20.0),
        bottom_button_head_bolts_reference(20.0),
        names=[
            "절곡1_6T_축지지_U브래킷",
            "절곡2_6T_3030_감싸기브래킷",
            "절곡3_2.5T_V형_축받침",
            "기존_상부클램프",
            "3030_프로파일",
            "실제_휠모터_STEP",
            "M6x20_측면볼트_4개",
            "M6x20_바닥볼트_2개",
        ],
        colors=[
            "steelblue", "royalblue", "orange", "silver", "dimgray",
            "gray", "slategray", "slategray",
        ],
        reset_camera=Camera.CENTER,
    )
    print(
        "FOLDED_CLAMP_SENT "
        f"sheet={SHEET_T:.1f} liner={LINER_T:.1f} rail_z={RAIL_CZ:.1f}"
    )
