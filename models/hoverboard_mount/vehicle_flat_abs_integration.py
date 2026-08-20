"""Show the compact cargo vehicle with straight, flat ABS body panels."""

import os
import sys
from pathlib import Path

HERE = Path(__file__).parent
sys.path[:0] = [str(HERE), str(HERE.parent)]

# Reuse the established compact vehicle, but replace its rounded exterior and
# cargo skin after construction.  This leaves the frame, sensors and Design-D
# wheel hardware unchanged for an honest visual comparison.
os.environ["VEHICLE_PAYLOAD"] = "cargo"

from build123d import (  # noqa: E402
    Align,
    Box,
    BuildPart,
    BuildSketch,
    Color,
    Compound,
    Cylinder,
    Locations,
    Mode,
    Plane,
    Pos,
    Rectangle,
    add,
    extrude,
)
from ocp_vscode import Camera, set_port, show  # noqa: E402

import vehicle_shell_integration as VSI  # noqa: E402

V = VSI.V
PORT = 3939
ABS_T = 3.0
PANEL_GAP = 1.5


def group(parts, label, color):
    c = Color(color)
    for part in parts:
        part.color = c
    result = Compound(children=parts)
    result.label, result.color = label, c
    return result


def _rectangular_ring(z0, z1, wall):
    """Thin four-sided shell made from flat sheet, with square corners."""
    with BuildPart() as p:
        with BuildSketch(Plane.XY.offset(z0)):
            Rectangle(2 * V.SHELL_X, 2 * V.SHELL_Y)
            Rectangle(
                2 * (V.SHELL_X - wall),
                2 * (V.SHELL_Y - wall),
                mode=Mode.SUBTRACT,
            )
        extrude(amount=z1 - z0)
    return p.part


def angular_lower_shell():
    """Straight ABS body panels following the 3030 frame envelope."""
    with BuildPart() as p:
        add(_rectangular_ring(V.SKIRT_LOW_Z, V.SHELL_TOP, ABS_T))

        # Preserve the real wheel clearance while making every manufactured
        # outer edge straight.  Only the necessary wheel openings remain curved.
        arch_r = V.WHEEL_D / 2 + V.WHEEL_ARCH_CLR
        arch_depth = V.WHEEL_W + 2 * (V.WHEEL_ARCH_CLR + ABS_T + 10)
        for x in (-V.AXLE_X, V.AXLE_X):
            for side in (-1, 1):
                with Locations((x, side * V.SHELL_WHEEL_Y, V.AXLE_Z)):
                    Cylinder(
                        arch_r,
                        arch_depth,
                        rotation=(90, 0, 0),
                        align=(Align.CENTER, Align.CENTER, Align.CENTER),
                        mode=Mode.SUBTRACT,
                    )

        # Keep the established face, service and ventilation interfaces.
        extrude(to_extrude=V.front_cutouts(), amount=40, mode=Mode.SUBTRACT)
        extrude(to_extrude=V.rear_cutouts(), amount=-40, mode=Mode.SUBTRACT)
        add(V.port_bay())
        add(V.mini_cuts(), mode=Mode.SUBTRACT)
        add(V.cooling_cuts(), mode=Mode.SUBTRACT)
        add(V.bumper_cuts(), mode=Mode.SUBTRACT)
        add(V.handle_cuts(), mode=Mode.SUBTRACT)

        # A narrow vertical reveal makes the two service halves readable as
        # separate cut-and-bent sheets in the viewer.
        with Locations((0, 0, (V.SKIRT_LOW_Z + V.SHELL_TOP) / 2)):
            Box(
                PANEL_GAP,
                2 * V.SHELL_Y + 10,
                V.SHELL_TOP - V.SKIRT_LOW_Z + 10,
                mode=Mode.SUBTRACT,
            )
    parts = p.solids()
    for index, part in enumerate(parts, start=1):
        part.label = f"ABS하부평판{index}"
    return parts


def angular_deck():
    """Square deck plate replacing the former R60 rounded deck edge."""
    with BuildPart() as p:
        with Locations((0, 0, V.SHELL_TOP + V.PANEL_T / 2)):
            Box(2 * V.SHELL_X, 2 * V.SHELL_Y, V.PANEL_T)
    p.part.label = "ABS사각데크"
    return p.part


def angular_cargo():
    """Four straight cargo walls and a removable flat top panel."""
    wall_top = V.CARGO_Z1 - ABS_T - PANEL_GAP
    walls = _rectangular_ring(V.CARGO_SKIRT_Z, wall_top, ABS_T)
    walls.label = "ABS적재함4면"

    with BuildPart() as lid:
        with Locations((0, 0, V.CARGO_Z1 - ABS_T / 2)):
            Box(2 * V.SHELL_X, 2 * V.SHELL_Y, ABS_T)
        # Small rear finger pull; this is a routed rectangular slot rather than
        # a molded recess, so it remains compatible with flat-sheet fabrication.
        with Locations((-V.SHELL_X + 45, 0, V.CARGO_Z1 - ABS_T / 2)):
            Box(55, 130, ABS_T + 3, mode=Mode.SUBTRACT)
    lid.part.label = "ABS평판덮개"
    return [walls, lid.part]


def main():
    remove = {
        "바퀴",
        "휠마운트",
        "휠클램프",
        "외장셸",
        "판재",
        "적재함",
        "적재함뚜껑",
        "적재함지붕",
    }
    retained = [g for g in V.GROUPS if g.label not in remove]

    motors, lowers, uppers, bolts = VSI.design_d_wheels()
    new_groups = [
        group([V.floor_panel, angular_deck()], "평판형_바닥과데크", "#aeb4ba"),
        group(angular_lower_shell(), "ABS_각진_하부외장", "#343b43"),
        group(angular_cargo(), "ABS_각진_적재함", "#343b43"),
        group(motors, "실제_휠모터_STEP_4개", "#2b2b2b"),
        group(lowers, "DesignD_원래_하부클램프_4개", "#c9ced4"),
        group(uppers, "기존_상부클램프_4개", "#f5a623"),
        group(bolts, "M6x20_프로파일체결볼트_24개", "#30343a"),
    ]
    groups = retained + new_groups
    colors = [V.CLR[g.label] for g in retained] + [
        "#aeb4ba",
        "#343b43",
        "#343b43",
        "#2b2b2b",
        "#c9ced4",
        "#f5a623",
        "#30343a",
    ]

    set_port(PORT)
    show(
        *groups,
        names=[g.label for g in groups],
        colors=colors,
        alphas=[1.0] * len(groups),
        progress=None,
        reset_camera=Camera.RESET,
    )
    print(
        "VEHICLE_FLAT_ABS_SENT "
        f"groups={len(groups)} panel={ABS_T:.1f}mm "
        f"body={2*V.SHELL_X:.0f}x{2*V.SHELL_Y:.0f} "
        f"cargo_top={V.CARGO_Z1:.0f}"
    )


if __name__ == "__main__":
    main()
