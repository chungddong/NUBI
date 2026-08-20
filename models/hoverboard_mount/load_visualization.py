"""Design D의 보수적 정역학 하중 경로를 OCP 뷰어용 형상으로 만든다."""

from build123d import *

import low_clamp as LC
import parameters as P


def _arrow(length=30.0, shaft_r=1.8, head_r=5.0, head_h=8.0):
    """로컬 +Z 방향 화살표."""
    with BuildPart() as p:
        Cylinder(shaft_r, length - head_h)
        with Locations((0, 0, length - head_h)):
            Cone(head_r, 0, head_h)
    return p.part


def load_arrow(load_n=P.DESIGN_W):
    arrow = Pos(0, LC.CLAMP_CY, 72) * Rot(180, 0, 0) * _arrow(48, 2.5, 7, 10)
    arrow.label = f"LOAD_down_{load_n:.0f}N"
    return arrow


def bottom_pullout_path(load_n=P.DESIGN_W):
    force = load_n * abs(LC.CLAMP_CY) / (P.EXTRUSION_W / 2) / 2
    arrows = []
    for sx in (-1, 1):
        a = (Pos(sx * P.LOW_POST_X, LC.CLAMP_CY, LC.RAIL_BOT + 2)
             * _arrow(28, 1.8, 5, 8))
        a.label = f"bottom_pullout_{force:.0f}N_{'L' if sx < 0 else 'R'}"
        arrows.append(a)
    group = Compound(children=arrows)
    group.label = f"PATH_bottom_pullout_{force:.0f}N_each_95pct"
    return group


def side_shear_path(load_n=P.DESIGN_W):
    force = load_n * abs(LC.CLAMP_CY) / P.EXTRUSION_W / 2
    arrows = []
    y_outer = LC.FLANGE_Y + P.LOW_FLANGE_T + 5
    for sx in (-1, 1):
        for sy in (-1, 1):
            rot = Rot(180, 0, 0) if sy < 0 else Rot(0, 0, 0)
            a = (Pos(sx * P.LOW_POST_X, LC.CLAMP_CY + sy * y_outer,
                     LC.BOLT_Z - (18 if sy < 0 else 0))
                 * rot * _arrow(18, 1.5, 4.5, 7))
            a.label = f"side_shear_{force:.0f}N_{len(arrows) + 1}"
            arrows.append(a)
    group = Compound(children=arrows)
    group.label = f"PATH_side_shear_{force:.0f}N_each_9pct"
    return group


def utilization_markers(load_n=P.DESIGN_W):
    """(하중, 바닥 인발 경로, 측면 전단 경로) 토글 객체."""
    return load_arrow(load_n), bottom_pullout_path(load_n), side_shear_path(load_n)
