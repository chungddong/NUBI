"""Export the Design-D clamp parts as verified, slicer-ready binary STL files.

The parts keep the assembled orientation but are translated so their lowest
Z face is on the print bed (Z=0).  Units are millimetres.
"""

from pathlib import Path
import struct

from build123d import Location, export_stl

from low_clamp import low_clamp, oem_clamp_block
import parameters as P


def variant_tag() -> str:
    """형상 스위치를 파일명에 박는다 — 안 박으면 혀/날개/타이 판이 서로 덮어쓴다."""
    tag = ""
    if P.LOW_SIDE:
        tag += "_side"
    if P.LOW_SHELF:
        tag += "_tongue" if P.SHELF_KIND == "tongue" else "_shelf"
        if P.SHELF_KIND == "tongue" and P.SHELF_CABLE:
            tag += "_cable"
    if P.TONGUE:
        tag += "_ttongue"
    if P.TIE:
        tag += "_tieprof" if P.TIE_KIND == "profile" else "_tierod"
    return tag


OUT = Path(__file__).parent / "output" / "stl"
if P.LOW_SHELF and P.SHELF_KIND == "tongue" and P.SHELF_CABLE:
    # 케이블 통로는 좌우 대칭 두 벌이 필요하다 — 네 바퀴 케이블을 전부 차량 안쪽으로.
    #   R (+x): 앞축 왼쪽 + 뒷축 오른쪽 / L (-x): 앞축 오른쪽 + 뒷축 왼쪽  (low_clamp.cable_side_for)
    _tag = variant_tag()
    PARTS = (
        (f"design_d_lower_clamp_print{_tag}R", lambda: low_clamp(cable_side=+1)),
        (f"design_d_lower_clamp_print{_tag}L", lambda: low_clamp(cable_side=-1)),
        ("oem_upper_clamp_reference_print", oem_clamp_block),
    )
else:
    PARTS = (
        (f"design_d_lower_clamp_print{variant_tag()}", low_clamp),
        ("oem_upper_clamp_reference_print", oem_clamp_block),
    )


def put_on_bed(part):
    """Translate a part without rotating it so min Z is exactly zero."""
    z_min = part.bounding_box().min.Z
    return part.moved(Location((0.0, 0.0, -z_min)))


def verify_binary_stl(path: Path) -> int:
    """Return triangle count after checking the binary STL byte structure."""
    size = path.stat().st_size
    if size < 84:
        raise RuntimeError(f"STL is too small: {path} ({size} bytes)")
    with path.open("rb") as stream:
        stream.seek(80)
        triangles = struct.unpack("<I", stream.read(4))[0]
    expected_size = 84 + 50 * triangles
    if triangles == 0 or size != expected_size:
        raise RuntimeError(
            f"Invalid binary STL: {path} triangles={triangles} "
            f"bytes={size} expected={expected_size}"
        )
    return triangles


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    for filename, factory in PARTS:
        source = factory()
        if len(source.solids()) != 1 or not source.is_valid:
            raise RuntimeError(f"Source is not one valid solid: {filename}")

        printable = put_on_bed(source)
        path = OUT / f"{filename}.stl"
        if not export_stl(printable, path):
            raise RuntimeError(f"STL export failed: {path}")

        triangles = verify_binary_stl(path)
        bounds = printable.bounding_box()
        print(
            f"CLAMP_STL_OK path={path} triangles={triangles} "
            f"bbox={bounds.size.X:.2f}x{bounds.size.Y:.2f}x{bounds.size.Z:.2f}mm "
            f"z={bounds.min.Z:.3f}..{bounds.max.Z:.3f}mm "
            f"volume={printable.volume:.3f}mm3"
        )


if __name__ == "__main__":
    main()
