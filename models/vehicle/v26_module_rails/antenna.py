r"""눈썹 안테나 — 와이파이 듀얼밴드 RP-SMA 힌지 안테나 + 판넬 벌크헤드 피그테일 (v23).

젯슨 와이파이 카드 안테나 2 개를 앞판 밖으로 뺀다. 벌크헤드는 전조등 테두리 안쪽 위 모서리, 안테나는 힌지로 꺾어
전조등 위로 바깥쪽이 살짝 올라가게 눕힌다 (눈썹). 값 출처는 DECISIONS D-12 · research/2026-10-05_antenna.md.

로컬 좌표: X = 판 법선 (+X = 앞), **X=0 = 앞판 바깥면**, 판은 X ∈ [-PANEL_T, 0]. 벌크헤드 축 = X.
안테나는 힌지에서 +Y 쪽 (바깥) 으로 TILT° 올라가며 뻗는다 — 오른쪽 (−y) 것은 Y 를 뒤집어 쓴다.
"""

import math

from build123d import Box, BuildPart, BuildSketch, Cylinder, Plane, Pos, Rot, RegularPolygon, extrude

PANEL_T = 3.0
# 벌크헤드 (에즈윈 키트 동봉, 육각 너트 · 톱니 와셔) — 1/4"-36 UNS 나사 Ø6.35, 판 구멍 Ø6.5, 너트 맞변 8 [규격 일반값],
# 나사부 약 14 [사진 비례]. SMA / RP-SMA 표기 없음 — 키트 안 케이블 · 안테나끼리는 짝이 맞는다.
BH = dict(thread_d=6.35, hole_d=6.5, nut_af=8.0, nut_t=2.5, front_l=9.0, back_l=5.0, cable_d=3.0)
# 안테나 (에즈윈 AX200 키트 동봉, 전체 135) — 커넥터 끝에서 [사진 비례 ±0.5~1]:
#   로렛 커플링 너트 Ø9.9 × 7.5 → 힌지 베이스 Ø9.1 13 → 꺾임 마디 (약 21.5) → 목 Ø8.3 9 → 굵은 몸통 Ø9.8 41 → 테이퍼 35 (Ø9.7→6) → 끝 Ø5.8 28
ANT = dict(nut_d=9.9, nut_l=7.5, base_d=9.1, hinge_at=21.5, neck=(8.3, 9.0), main=(9.8, 41.0), taper=(9.7, 6.0, 35.0), tip=(5.8, 28.0))
TILT = 15.0                       # 수평에서 바깥쪽이 올라가는 각 [설계] — 전조등 테두리 위를 지나게


def _hex(af, x0, x1):
    with BuildPart() as p:
        with BuildSketch(Plane.YZ.offset(min(x0, x1))):
            RegularPolygon(af / 2 / 0.8660254, 6, major_radius=True)
        extrude(amount=abs(x1 - x0))
    return p.part


def _xcyl(d, x0, x1, y=0.0, z=0.0):
    return Pos((x0 + x1) / 2, y, z) * Cylinder(d / 2, abs(x1 - x0), rotation=(0, 90, 0))


def bulkhead():
    """벌크헤드 잭 + 앞뒤 너트 + 피그테일 토막 (안쪽 30). 판 구멍을 지나는 나사부는 Ø6.35 < 구멍 6.5."""
    b = BH
    s = _xcyl(b["thread_d"], -PANEL_T, 0) + _hex(b["nut_af"], 0, b["nut_t"]) + _xcyl(b["thread_d"], b["nut_t"], b["front_l"])
    s = s + _hex(b["nut_af"], -PANEL_T, -PANEL_T - b["nut_t"]) + _xcyl(b["thread_d"], -PANEL_T - b["nut_t"], -PANEL_T - b["back_l"])
    s = s + _xcyl(3.0, -PANEL_T - b["back_l"], -PANEL_T - b["back_l"] - 30.0)          # 피그테일 (수축튜브 포함 Ø3) 토막
    return s


def antenna(side=+1):
    """안테나 — 로렛 너트 (벌크헤드 앞 나사에 6 물림) + 힌지 베이스 + 꺾임 마디 + 몸통 (마디에서 Y 로, TILT 만큼 위로)."""
    from build123d import Cone
    a = ANT
    xc = BH["front_l"] - 6.0                                                       # 커넥터 끝 (너트 입구) x
    s = _xcyl(a["nut_d"], xc, xc + a["nut_l"]) + _xcyl(a["base_d"], xc + a["nut_l"], xc + a["hinge_at"])
    xh = xc + a["hinge_at"]                                                        # 꺾임 마디 중심
    s = s + Pos(xh, 0, 0) * Cylinder(a["base_d"] / 2, a["base_d"])                  # 마디 (축 = Z)
    # 몸통 — 마디와 같은 반지름으로 접하면 OCC 메시가 깨진다 → 마디 중심에서 3 비켜서 시작
    y = 3.0
    parts = []
    for d, L in (a["neck"], a["main"]):
        parts.append(Pos(0, side * (y + L / 2), 0) * Cylinder(d / 2, L, rotation=(-90, 0, 0)))
        y += L
    d0, d1, L = a["taper"]
    parts.append(Pos(0, side * (y + L / 2), 0) * Cone(d0 / 2, d1 / 2, L, rotation=(-90 * side, 0, 0)))
    y += L
    d, L = a["tip"]
    parts.append(Pos(0, side * (y + L / 2), 0) * Cylinder(d / 2, L, rotation=(-90, 0, 0)))
    body = parts[0]
    for p in parts[1:]:
        body = body + p
    s = s + Pos(xh, 0, 0) * Rot(side * TILT, 0, 0) * body     # 바깥 (±Y) 끝이 위로 — 거울 대신 직접 (거울은 솔리드가 뒤집혀 무효)
    return s


def reach(side=+1):
    """(y, z) 끝점 오프셋 — 바깥쪽 끝이 얼마나 나가고 올라가는지 (검사 · 배치용)."""
    L = 3.0 + sum(v[-1] for v in (ANT["neck"], ANT["main"], ANT["taper"], ANT["tip"]))
    return side * L * math.cos(math.radians(TILT)), L * math.sin(math.radians(TILT))
