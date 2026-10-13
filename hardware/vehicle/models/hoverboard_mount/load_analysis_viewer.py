"""Interactive load-path view for the current Design-D lower clamp.

This is a conservative statics explainer, not a finite-element solution.  The
viewer tree keeps the assembly, load cases, reactions, and risk zones in
separate groups so each layer can be toggled independently.
"""

from pathlib import Path
import sys

HERE = Path(__file__).parent
sys.path[:0] = [str(HERE), str(HERE.parent)]

from build123d import (  # noqa: E402
    Align,
    Box,
    BuildPart,
    Color,
    Compound,
    Cone,
    Cylinder,
    Location,
    Locations,
    Mode,
    Pos,
    Rot,
)
from ocp_vscode import Camera, set_port, show  # noqa: E402

import low_clamp as LC  # noqa: E402
import parameters as P  # noqa: E402
from motor_reference import actual_motor_step_reference  # noqa: E402


PORT = 3939


def group(parts, label, color):
    parts = list(parts) if isinstance(parts, (list, tuple)) else [parts]
    c = Color(color)
    for part in parts:
        part.color = c
    result = Compound(children=parts)
    result.label = label
    result.color = c
    return result


def arrow(length=30.0, shaft_r=1.7, head_r=4.8, head_h=8.0):
    """Arrow whose local direction is +Z."""
    with BuildPart() as p:
        Cylinder(shaft_r, length - head_h)
        with Locations((0, 0, length - head_h)):
            Cone(head_r, 0, head_h)
    return p.part


def downward_load(load_n):
    scale = {800: 44.0, 1600: 54.0, 2400: 64.0}[int(load_n)]
    result = Pos(0, 0, 86) * Rot(180, 0, 0) * arrow(scale, 2.3, 6.5, 10)
    result.label = f"휠축_수직하중_{load_n:.0f}N_아래방향"
    return result


def bottom_reactions(load_n):
    """Conservative case: the bottom T-nuts alone resist overturning."""
    each = load_n * abs(LC.CLAMP_CY) / (P.EXTRUSION_W / 2) / 2
    length = 24 + 8 * (load_n / P.DESIGN_W - 1)
    out = []
    for side in (-1, 1):
        item = (
            Pos(side * P.LOW_POST_X, LC.CLAMP_CY, LC.LOW_BOT + 2)
            * arrow(length, 1.7, 4.5, 7)
        )
        item.label = f"바닥볼트_{'좌' if side < 0 else '우'}_인발_{each:.0f}N"
        out.append(item)
    return out, each


def side_reactions(load_n):
    """Conservative case: the four side bolts form a vertical shear couple."""
    each = load_n * abs(LC.CLAMP_CY) / P.EXTRUSION_W / 2
    length = 18 + 5 * (load_n / P.DESIGN_W - 1)
    y_outer = LC.FLANGE_Y + P.LOW_FLANGE_T + 4
    out = []
    for sx in (-1, 1):
        for sy in (-1, 1):
            # Opposite rail faces carry opposite vertical reactions, forming a
            # couple across the 30 mm profile width.
            tf = Rot(180, 0, 0) if sy < 0 else Location()
            z0 = LC.BOLT_Z + (length if sy < 0 else -length)
            item = Pos(sx * P.LOW_POST_X, LC.CLAMP_CY + sy * y_outer, z0) * tf * arrow(
                length, 1.4, 4.0, 6
            )
            item.label = f"측면볼트_{len(out)+1}_수직전단_{each:.0f}N"
            out.append(item)
    return out, each


def torque_couple():
    """Two tangential arrows indicating the 29 N·m motor reaction torque."""
    upper = Pos(-28, 2, 18) * Rot(0, 90, 0) * arrow(56, 1.8, 5.0, 8)
    lower = Pos(28, 2, -18) * Rot(0, -90, 0) * arrow(56, 1.8, 5.0, 8)
    upper.label = "모터토크_상부_반력방향"
    lower.label = "모터토크_하부_반력방향"
    return [upper, lower]


def risk_overlays():
    """Geometry-based inspection zones; colors are not computed FEA stress."""
    seat_r = P.AXLE_DIAMETER / 2 + P.SADDLE_SEAT_FIT
    with BuildPart() as ring:
        with Locations((0, LC.CLAMP_CY, 0)):
            Cylinder(seat_r + 2.2, P.LOW_D + 1, rotation=(90, 0, 0))
            Cylinder(seat_r - 0.8, P.LOW_D + 3, rotation=(90, 0, 0), mode=Mode.SUBTRACT)
        with Locations((0, LC.CLAMP_CY, seat_r + 6)):
            Box(2 * seat_r + 8, P.LOW_D + 8, 2 * seat_r + 12, mode=Mode.SUBTRACT)
    ring.part.label = "주의_축시트_면압과_시트아래_굽힘"

    roots = []
    for sx in (-1, 1):
        for sy in (-1, 1):
            item = Pos(
                sx * P.LOW_POST_X,
                LC.CLAMP_CY + sy * (LC.FLANGE_Y + P.LOW_FLANGE_T / 2),
                LC.RAIL_BOT - 2.0,
            ) * Box(
                P.LOW_POST_W + 1,
                P.LOW_FLANGE_T + 1,
                7.0,
                align=(Align.CENTER, Align.CENTER, Align.CENTER),
            )
            item.label = f"주의_측면플랜지_뿌리_{len(roots)+1}"
            roots.append(item)

    bores = []
    for sx in (-1, 1):
        with BuildPart() as marker:
            with Locations((sx * P.LOW_POST_X, LC.CLAMP_CY, LC.LOW_BOT + P.LOW_CBORE[1] - 2)):
                Cylinder(P.LOW_CBORE[0] / 2 + 2, 5)
                Cylinder(P.LOW_CBORE[0] / 2 - 1, 6, mode=Mode.SUBTRACT)
        marker.part.label = f"점검_깊은카운터보어_잔여두께_{'좌' if sx < 0 else '우'}"
        bores.append(marker.part)
    return ring.part, roots, bores


def load_case(load_n, color):
    applied = downward_load(load_n)
    bottom, bottom_each = bottom_reactions(load_n)
    side, side_each = side_reactions(load_n)
    children = [applied, *bottom, *side]
    result = group(
        children,
        (
            f"하중케이스_{load_n:.0f}N__바닥인발{bottom_each:.0f}N개당_"
            f"측면전단{side_each:.0f}N개당"
        ),
        color,
    )
    return result


def main():
    rail = LC.rail()
    lower = LC.low_clamp()
    upper = LC.oem_clamp_block()
    motor = actual_motor_step_reference(axle_z=0.0)
    rail_bolts = LC.side_button_head_bolts_reference(20.0) + LC.bottom_button_head_bolts_reference(20.0)
    clamp_bolts = LC.oem_button_head_bolts_reference(20.0)

    assembly = [
        group(rail, "01_3030프로파일_구속부", "#69747e"),
        group(lower, "02_해석대상_DesignD하부클램프_AL6061T6", "#c9ced4"),
        group(upper, "03_기존상부클램프", "#f5a623"),
        group(motor, "04_실제휠모터와축", "#30343a"),
        group(rail_bolts, "05_프로파일체결_M6x20_측면4_바닥2", "#9aa1a8"),
        group(clamp_bolts, "06_상하클램프체결_M6_4개", "#59616a"),
    ]

    seat, roots, bores = risk_overlays()
    overlays = [
        group(seat, "위험부A_축시트_면압과_블록굽힘_간이표시", "#ff8c00"),
        group(roots, "위험부B_측면플랜지뿌리_응력집중_간이표시", "#e53935"),
        group(bores, "점검부C_바닥카운터보어_잔여두께", "#ffd54f"),
        group(torque_couple(), "구동토크_29Nm_반력방향", "#8e44ad"),
    ]

    cases = [
        load_case(800.0, "#2f80ed"),
        load_case(1600.0, "#f2c94c"),
        load_case(2400.0, "#eb5757"),
    ]
    note = Box(0.8, 0.8, 0.8)
    notes = group(
        note,
        "주의_색상은FEA응력값아님_보수적정역학_하중경로표시",
        "#ffffff",
    )

    objects = assembly + overlays + cases + [notes]
    colors = [
        "#69747e", "#c9ced4", "#f5a623", "#30343a", "#9aa1a8", "#59616a",
        "#ff8c00", "#e53935", "#ffd54f", "#8e44ad",
        "#2f80ed", "#f2c94c", "#eb5757", "#ffffff",
    ]
    alphas = [1, 1, 1, 0.88, 1, 1, 0.72, 0.62, 0.72, 1, 1, 1, 1, 0.01]

    set_port(PORT)
    show(
        *objects,
        names=[item.label for item in objects],
        colors=colors,
        alphas=alphas,
        progress=None,
        reset_camera=Camera.RESET,
        collapse=1,
        axes=True,
        grid=False,
        transparent=False,
        black_edges=False,
    )
    print("LOAD_ANALYSIS_VIEW_SENT load_cases=800,1600,2400N mode=conservative_statics_not_FEA")


if __name__ == "__main__":
    main()
