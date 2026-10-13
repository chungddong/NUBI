"""Export the as-failed legacy long U-top bracket without later gussets."""

from __future__ import annotations

import os
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE.parent))
os.environ["NO_GUSSET"] = "1"
os.environ["U_OPEN_INBOARD"] = "1"

from build123d import export_step  # noqa: E402
import wheel_mount as WM  # noqa: E402


def main():
    part = WM.mount()
    part.label = "legacy_long_u_top_bracket_no_gussets_open_inboard"
    output = (
        HERE / "output" / "step"
        / "legacy_u_bracket_no_gussets_open_inboard.step"
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    export_step(part, str(output))
    box = part.bounding_box()
    print(
        f"EXPORTED {output} bbox={box.size.X:.2f}x{box.size.Y:.2f}x{box.size.Z:.2f}mm "
        f"volume={part.volume / 1000:.3f}cm3 solids={len(part.solids())}"
    )


if __name__ == "__main__":
    main()
