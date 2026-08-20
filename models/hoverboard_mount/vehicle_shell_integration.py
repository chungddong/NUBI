"""Show the complete skid vehicle shell with the Design-D wheel mounts."""

import os
import sys
from pathlib import Path

HERE = Path(__file__).parent
sys.path[:0] = [str(HERE), str(HERE.parent)]

# Build the established complete vehicle without sending/exporting its old scene.
os.environ["DRIVE"] = "skid"
os.environ["MOUNT"] = "low"
os.environ["PAYLOAD"] = os.environ.get("VEHICLE_PAYLOAD", "none")
# Match the upper frame to the 450 mm lower frame while keeping the original
# 760 mm length and all vertical dimensions.  The shell becomes 510 mm wide.
os.environ["TOP_RAIL_Y"] = "210"
os.environ["SHELL_Y"] = "255"
os.environ["SHELL_WHEEL_Y"] = "263.7"
os.environ["WHEEL_ARCH_CLR"] = "12"
os.environ["HUB_MOCK"] = "1"
os.environ["NOSHOW"] = "1"
os.environ["NOEXPORT"] = "1"

from build123d import Color, Compound, Location, Pos, Rot  # noqa: E402
from ocp_vscode import Camera, set_port, show               # noqa: E402

import swerve_base as V                                     # noqa: E402
import low_clamp as LC                                      # noqa: E402
from motor_reference import actual_motor_step_reference     # noqa: E402


PORT = 3939
WHEEL_Y = V.RAIL_Y - LC.CLAMP_CY


def group(parts, label, color):
    c = Color(color)
    for part in parts:
        part.color = c
    result = Compound(children=parts)
    result.label, result.color = label, c
    return result


def design_d_wheels():
    motor0 = actual_motor_step_reference(axle_z=0.0)
    lower0 = LC.low_clamp()
    upper0 = LC.oem_clamp_block()
    side0 = LC.side_button_head_bolts_reference(20.0)
    bottom0 = LC.bottom_button_head_bolts_reference(20.0)

    motors, lowers, uppers, bolts = [], [], [], []
    for index, (x, side) in enumerate(
        (point for point in ((-V.AXLE_X, -1), (-V.AXLE_X, 1),
                             (V.AXLE_X, -1), (V.AXLE_X, 1))),
        start=1,
    ):
        transform = Pos(x, side * WHEEL_Y, V.AXLE_Z) * (
            Rot(0, 0, 180) if side < 0 else Location()
        )
        motor = transform * motor0.moved(Location())
        lower = transform * lower0.moved(Location())
        upper = transform * upper0.moved(Location())
        motor.label = f"실제휠모터{index}"
        lower.label = f"DesignD하부클램프{index}"
        upper.label = f"기존상부클램프{index}"
        motors.append(motor)
        lowers.append(lower)
        uppers.append(upper)
        for bolt_index, source in enumerate(side0 + bottom0, start=1):
            bolt = transform * source.moved(Location())
            bolt.label = f"M6x20_{index}_{bolt_index}"
            bolts.append(bolt)
    return motors, lowers, uppers, bolts


def main():
    # Remove only the older wheel, wheel-mount and clamp groups.  Every exterior,
    # panel, bumper, light, camera, antenna and interior group remains unchanged.
    replaced = {"바퀴", "휠마운트", "휠클램프"}
    complete_vehicle = [g for g in V.GROUPS if g.label not in replaced]

    motors, lowers, uppers, bolts = design_d_wheels()
    additions = [
        group(motors, "실제_휠모터_STEP_4개", "#2b2b2b"),
        group(lowers, "DesignD_원래_하부클램프_4개", "#c9ced4"),
        group(uppers, "기존_상부클램프_4개", "#f5a623"),
        group(bolts, "M6x20_프로파일체결볼트_24개", "#30343a"),
    ]
    groups = complete_vehicle + additions

    # Keep the established material palette fully opaque.  The previous 0.82
    # frame alpha exposed overlapping internal T-slot faces and caused the
    # angle-dependent shredded/clipped appearance.
    colors = [V.CLR[g.label] for g in complete_vehicle] + [
        "#2b2b2b", "#c9ced4", "#f5a623", "#30343a"
    ]
    set_port(PORT)
    show(
        *groups,
        names=[g.label for g in groups],
        colors=colors,
        alphas=[1.0] * len(groups),
        progress=None,
        # RESET refits camera position and near/far planes to the complete
        # vehicle. CENTER kept the depth range of the previous small scene,
        # which clipped different halves of the vehicle as the camera rotated.
        reset_camera=Camera.RESET,
    )
    print(
        "VEHICLE_SHELL_DESIGN_D_SENT "
        f"groups={len(groups)} wheel_y=+/-{WHEEL_Y:.2f} "
        f"rail_z={V.RAIL_Z:.2f} shell=on alpha=1.0"
    )


if __name__ == "__main__":
    main()
