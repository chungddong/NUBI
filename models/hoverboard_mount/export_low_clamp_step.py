"""Export the current original machined lower clamp as a verified STEP file."""

from pathlib import Path

from build123d.exporters3d import export_step
from build123d.importers import import_step

from low_clamp import low_clamp


OUT = Path(__file__).parent / "output" / "step" / "low_clamp_design_d.step"


def main():
    OUT.parent.mkdir(parents=True, exist_ok=True)

    source = low_clamp()
    source.label = "Design_D_original_machined_lower_clamp_AL6061_T6"
    if not export_step(source, OUT):
        raise RuntimeError("STEP export failed")

    restored = import_step(OUT)
    source_bb = source.bounding_box()
    restored_bb = restored.bounding_box()
    volume_error = abs(restored.volume - source.volume)
    size_error = max(
        abs(restored_bb.size.X - source_bb.size.X),
        abs(restored_bb.size.Y - source_bb.size.Y),
        abs(restored_bb.size.Z - source_bb.size.Z),
    )
    if volume_error > 1e-3 or size_error > 1e-6:
        raise RuntimeError(
            f"STEP verification failed: volume_error={volume_error}, size_error={size_error}"
        )

    print(
        "LOW_CLAMP_STEP_OK "
        f"path={OUT} bytes={OUT.stat().st_size} "
        f"bbox={source_bb.size.X:.2f}x{source_bb.size.Y:.2f}x{source_bb.size.Z:.2f}mm "
        f"volume={source.volume:.3f}mm3"
    )


if __name__ == "__main__":
    main()
