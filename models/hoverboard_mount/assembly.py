# 조립 + 검증. 두 설계안(A: 스플릿 클램프 / B: 토크 플레이트)을 같은 좌표에 얹는다.
#
# 좌표: 지면 z=0 / 타이어 중심면 y=0 / 축은 -y (차체 안쪽) / 축 중심 z=AXLE_Z

from build123d import *

import parameters as P
from adapter_plate import adapter_plate, clamp_mount_xz as _clamp_mount_xz, tnut_xz
from axle import axle_solid
from extrusion_reference import extrusion_reference
from motor_reference import motor_reference
from split_clamp import axle_clamp_lower, axle_clamp_upper
from torque_plate import axle_retainer, torque_plate

COL = {"hoverboard_motor_reference": "#2b2b2b", "axle_clamp_upper": "#6b7480",
       "axle_clamp_lower": "#8d9298", "adapter_plate": "#aeb4ba",
       "4080_extrusion_reference": "#c8ccd0", "torque_plate": "#c62828",
       "axle_retainer": "#ef6c00", "clamp_bolts_reference": "#f2c200",
       "t_nut_bolts_reference": "#f2c200"}


def _bolt(d, l, head_at, direction):
    """머리를 head_at 에 두고 direction(+1/-1 축방향)으로 l 만큼 뻗는 볼트.

    처음엔 Cylinder 를 중심 배치로 만들어 머리가 샤프트 한가운데 박히고, 길이가
    양쪽으로 뻗어 **클램프를 뚫었다**. 참조 모델도 자리가 맞아야 공구 접근을 본다.
    """
    ax, s = direction
    rot = {"Y": (90, 0, 0), "Z": (0, 0, 0), "X": (0, 90, 0)}[ax]
    off = {"Y": (0, 1, 0), "Z": (0, 0, 1), "X": (1, 0, 0)}[ax]
    mid = tuple(head_at[i] + off[i] * s * l / 2 for i in range(3))
    hd = tuple(head_at[i] - off[i] * s * 2 for i in range(3))
    with BuildPart() as p:
        with Locations(mid):
            Cylinder(d / 2, l, rotation=rot)
        with Locations(hd):
            Cylinder(d * 0.85, 4, rotation=rot)
    return p.part


def clamp_bolts_reference(design="A"):
    """Design A: 클램프 상→하 관통 4 (수직). Design B: 토크판→어댑터 4 (수평)."""
    out = []
    if design == "A":
        top = P.AXLE_Z + P.CLAMP_UPPER_H + P.CLAMP_PINCH_GAP / 2
        for i, (x, y) in enumerate([(sx * P.CLAMP_BOLT_X, sy * P.CLAMP_BOLT_Y)
                                    for sx in (-1, 1) for sy in (-1, 1)]):
            b = _bolt(P.CLAMP_BOLT, P.CLAMP_UPPER_H + P.CLAMP_LOWER_H - 4,
                      (x, P.CLAMP_CY + y, top), ("Z", -1))
            b.label = f"clamp_bolt{i+1}"
            out.append(b)
    else:
        y0 = P.PLATE_FACE_Y + P.TORQUE_PLATE_T
        for i, (x, z) in enumerate(_clamp_mount_xz()):
            b = _bolt(P.CLAMP_BOLT,
                      P.TORQUE_PLATE_T + P.ADAPTER_PLATE_THICKNESS - 2,
                      (x, y0, P.AXLE_Z + z), ("Y", -1))
            b.label = f"clamp_bolt{i+1}"
            out.append(b)
    return out


def t_nut_bolts_reference():
    """어댑터 → 4080 T너트 볼트 4 (수평, 슬롯 2줄)."""
    out = []
    for i, (x, z) in enumerate(tnut_xz()):
        b = _bolt(P.TNUT_BOLT, P.ADAPTER_PLATE_THICKNESS + 22,
                  (x, P.PLATE_FACE_Y, P.AXLE_Z + z), ("Y", -1))
        b.label = f"tnut_bolt{i+1}"
        out.append(b)
    return out


def build(design="A"):
    parts = [motor_reference(), adapter_plate(),
             Pos(0, P.EXTRUSION_FACE_Y - P.EXTRUSION_W / 2, P.AXLE_Z)
             * extrusion_reference()]
    if design == "A":
        parts += [Pos(0, P.CLAMP_CY, P.AXLE_Z) * axle_clamp_upper(),
                  Pos(0, P.CLAMP_CY, P.AXLE_Z) * axle_clamp_lower()]
    else:
        parts += [torque_plate(), axle_retainer()]
    parts += clamp_bolts_reference(design) + t_nut_bolts_reference()
    for q in parts:
        q.color = Color(COL.get(q.label, "#9aa0a6"))
    a = Compound(children=parts)
    a.label = f"hoverboard_wheel_mount_assembly_{design}"
    return a


def verify(design="A", quiet=False):
    """§12 검증. (문제목록, 부품수) 를 돌려준다."""
    parts = [(c.label, c) for c in build(design).children]
    bad = []
    for n, o in parts:                                   # 유효성 · 조각남
        if o.volume <= 0:
            bad.append(f"{n}: 부피 0")
        if len(o.solids()) > 1 and "reference" not in n:
            bad.append(f"{n}: 솔리드 {len(o.solids())}개로 갈라짐")
    skip = {("clamp_bolt", "axle_clamp"), ("tnut_bolt", "adapter_plate"),
            ("tnut_bolt", "4080"), ("clamp_bolt", "adapter_plate"),
            ("axle_retainer", "torque_plate"), ("hoverboard_motor", "axle_retainer")}
    for i, (na, a) in enumerate(parts):                  # 간섭
        for nb, b in parts[i + 1:]:
            if any(k in na and j in nb or k in nb and j in na for k, j in skip):
                continue
            ba, bb = a.bounding_box(), b.bounding_box()
            if (ba.min.X > bb.max.X or ba.max.X < bb.min.X or ba.min.Y > bb.max.Y
                    or ba.max.Y < bb.min.Y or ba.min.Z > bb.max.Z or ba.max.Z < bb.min.Z):
                continue
            try:
                v = (a & b)
                v = v.volume if v is not None else 0
            except Exception:
                v = 0
            if v > 20:
                bad.append(f"간섭 {na} x {nb}: {v:.0f} mm3")
    if not quiet:
        print(f"\n[Design {design}] 부품 {len(parts)}개")
        for m in bad:
            print(f"    !! {m}")
        print("    " + ("문제 없음" if not bad else f"문제 {len(bad)}건"))
    return bad, len(parts)
