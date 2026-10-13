# 휠브라켓 가공 파일 내보내기 — CNC/레이저 발주용 STEP + DXF.
#
# STEP : 3D. CNC(3축) 는 이걸로 바로 간다. 탭·카운터보어 정보가 다 들어간다.
# DXF  : 2D 외곽. 레이저·워터젯은 이걸 원한다. 평판 부품만 뽑는다 (두께는 파일명에).
#
# 주의 — 지금 `wheel_bracket()` 은 클램프판 2 + 타이판 + 플랜지 + 브리지를 **한 솔리드로
# 융합**해 놓았다. 형상 검증에는 그게 편했지만 가공 발주는 부품별로 갈라야 한다.
# 여기서 두께 방향으로 잘라 개별 부품을 만든다. 자른 결과가 원본과 부피가 같은지
# 대조해서, 나중에 브라켓을 고쳤을 때 이 분해가 어긋나면 바로 잡히게 했다.

import os
from pathlib import Path

os.environ.setdefault("NOSHOW", "1")
os.environ.setdefault("DRIVE", "skid")

from build123d import *                                          # noqa: E402

import swerve_base as M                                          # noqa: E402

OUT = Path(__file__).parent.parent / "exports" / "cnc"
OUT.mkdir(parents=True, exist_ok=True)


def _slab(y0, y1):
    """y 구간을 잘라내는 도구 (판 두께 방향)."""
    return Pos(0, (y0 + y1) / 2, M.AXLE_Z) * Box(400, y1 - y0, 400)


def _zslab(z0, z1):
    return Pos(0, -60, (z0 + z1) / 2) * Box(400, 400, z1 - z0)


def bracket_parts():
    """융합된 브라켓을 발주 단위로 가른다."""
    br = M.wheel_bracket()
    top, t = M.MODULE_TOP, M.BR_T
    rail_out = M.RAIL_Y + M.S / 2 - M.AXLE_Y
    out = {}
    # 클램프 판 2장 — 두께 방향으로 잘라낸다
    for tag, yc in (("클램프판_안쪽", min(M.BR_YS)), ("클램프판_바깥", max(M.BR_YS))):
        out[tag] = br & _slab(yc - t / 2, yc + t / 2) & _zslab(0, top - t)
    # 브리지 블록 — 판 사이, D컷 평면 위
    _off = M.HUB_FLAT - M.HUB_SHAFT_D / 2 + M.BR_FIT / 2
    out["브리지블록"] = br & _slab(min(M.BR_YS) + t / 2, max(M.BR_YS) - t / 2) \
        & _zslab(M.AXLE_Z + _off, top - t)
    # 타이판 (레일 밑면) + 플랜지 (레일 바깥면) — 위쪽 t 구간 전체와 그 위
    out["타이판"] = br & _zslab(top - t, top)
    out["플랜지"] = br & _zslab(top, top + M.S + 10)
    return out


def parts():
    p = bracket_parts()
    p["축캡"] = M.axle_cap()[0]
    p["위치스페이서"] = M.axle_spacer()[0]
    return p


def flat_dxf(shape, path):
    """가장 넓은 **평면** 면을 XY 로 눕혀 2D 외곽으로 내보낸다.

    가장 넓은 면이 곡면이면(선반 부품) None 을 돌려준다 — DXF 로 낼 게 없다.
    """
    fs = [f for f in shape.faces() if f.geom_type == GeomType.PLANE]
    if not fs:
        return None
    f = max(fs, key=lambda x: x.area)
    pl = Plane(f)
    flat = f.moved(pl.location.inverse())
    ex = ExportDXF(unit=Unit.MM)
    ex.add_shape(flat)
    ex.write(str(path))
    return f.area


P = parts()
BR_VOL = M.wheel_bracket().volume
_split = sum(v.volume for k, v in P.items()
             if k not in ("축캡", "위치스페이서"))
print(f"휠브라켓 가공 파일  →  {OUT}")
print(f"  분해 검증: 원본 {BR_VOL/1000:.1f} cm3 vs 조각 합 {_split/1000:.1f} cm3"
      f"  (차이 {abs(BR_VOL-_split):.0f} mm3)")
assert abs(BR_VOL - _split) < 500, "분해가 원본을 다 못 담았다 — 자르는 구간을 확인해라"

# 부품별 두께 = 가장 얇은 bbox 축
for name, s in P.items():
    b = s.bounding_box()
    dims = sorted([b.size.X, b.size.Y, b.size.Z])
    th = dims[0]
    step = OUT / f"{name}_{th:.0f}t.step"
    export_step(s.moved(Location()), str(step))
    tag = f"  {name:14s} {b.size.X:6.1f} x {b.size.Y:6.1f} x {b.size.Z:6.1f}" \
          f"  두께 {th:4.1f}  {s.volume/1000:5.1f} cm3"
    area = flat_dxf(s, OUT / f"{name}_{th:.0f}t.dxf")
    if area is None:
        tag += "  → STEP (선반)"
    elif th <= 10:
        tag += f"  → STEP + DXF (외곽 {area/100:.0f} cm2, 레이저)"
    else:
        tag += f"  → STEP + DXF (외곽 {area/100:.0f} cm2, 워터젯/톱+밀링)"
    print(tag)

print(f"\n코너당 위 부품 1세트씩, 로봇 한 대 = x4")
print(f"  레이저/워터젯: DXF (두께별로 묶어서 발주)")
print(f"  CNC 3축: STEP — 브리지블록은 M6 탭 2, 축캡은 Ø6.6 관통 2")
