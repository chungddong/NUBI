r"""80 mm 배기 팬 EVERCOOL EC8025H05B + 바깥 핑거 가드 EVERCOOL EFG-S80 (실물 치수 모델, v21).

팬: 5 VDC 0.28 A 1.4 W, 3000 rpm, 36.48 CFM, 2 볼 베어링, 케이블 300 (2510 2P) [제조사 사양표 · 도면]
    https://www.compuzone.co.kr/product/product_detail.htm?ProductNo=1047506 (8,740)
가드: 바깥 링 Ø76.5 · 가운데 링 Ø14.2 · 링 간격 4.5 · 와이어 Ø1.8 · 높이 5.5 · 고리 Ø4.6 [제조사 도면]
    https://www.compuzone.co.kr/product/product_detail.htm?ProductNo=351400 (380)
표기 [도면] 치수 있음  [도면 비례] 도면 축척으로 잼  [사진] 사진에서 셈

로컬 좌표: X = 팬 축, X=0 = 팬 프레임 앞면 (판 안쪽면에 붙는 면), 팬 몸통은 -X 쪽 (차량 안).
공기는 -X → +X (차량 안 → 밖, 배기). Y 폭, Z 높이.
"""

import math

from build123d import Box, BuildPart, Cylinder, Locations, Mode, Pos, Rot

SIZE, DEPTH = 80.0, 25.0          # [도면] ±0.5
HOLE_PITCH, HOLE_D = 71.5, 4.4    # [도면] 간격 ±0.3, Ø4.4 ±0.2
CORNER_R = 3.0                    # 프레임 모서리 R [도면 비례]
BORE_D = 76.5                     # 벤추리 안지름 [도면 비례]
HUB_D = 31.0                      # 허브 지름 [도면 비례]
BLADES = 7                        # 날개 수 [사진]
FLANGE_T = 4.0                    # 앞뒤 플랜지 두께 [도면 비례]
STRUTS = 4                        # 라벨면 (= 배기면) 지지대 [사진]
CABLE_L = 30.0                    # 케이블 300 중 토막만 (그림용)
CURRENT_A = 0.28                  # [제조사 사양표]

GUARD_WIRE, GUARD_H, GUARD_EAR_D = 1.8, 5.5, 4.6               # [도면]
GUARD_RINGS = tuple(76.5 - 9.0 * i for i in range(7)) + (14.2,)  # Ø76.5 에서 링 간격 4.5 (지름 9) … 가운데 Ø14.2 [도면]


def fan_80(cable_dir=-1):
    """팬 몸통 (프레임 + 허브 + 날개 + 배기면 지지대 + 케이블). 원점 = 앞면 가운데, 몸통은 -X.
    cable_dir: 케이블이 나가는 로컬 Y 방향 (±1) — 차량 가운데 쪽으로 (옆 기둥에 닿지 않게)."""
    ax = dict(rotation=(0, 90, 0))
    with BuildPart() as p:
        for x0 in (-FLANGE_T, -DEPTH):                            # 앞뒤 사각 플랜지
            with Locations((x0 + FLANGE_T / 2, 0, 0)):
                Box(FLANGE_T, SIZE, SIZE)
        with Locations((-DEPTH / 2, 0, 0)):                        # 가운데 원통 벽
            Cylinder(SIZE / 2 - 1.0, DEPTH, **ax)
        with Locations((-DEPTH / 2, 0, 0)):
            Cylinder(BORE_D / 2, DEPTH + 0.02, **ax, mode=Mode.SUBTRACT)
        for sy in (-1, 1):                                         # 모서리 구멍 4
            for sz in (-1, 1):
                with Locations((-DEPTH / 2, sy * HOLE_PITCH / 2, sz * HOLE_PITCH / 2)):
                    Cylinder(HOLE_D / 2, DEPTH + 0.02, **ax, mode=Mode.SUBTRACT)
    body = p.part
    hub = Pos(-DEPTH / 2 - 1.0, 0, 0) * Cylinder(HUB_D / 2, DEPTH - 4.0, rotation=(0, 90, 0))
    body = body + hub
    for i in range(BLADES):                                         # 날개 — 기울인 얇은 판
        a = 360.0 * i / BLADES
        r_mid = (HUB_D / 2 + BORE_D / 2) / 2
        blade = Box(14.0, BORE_D / 2 - HUB_D / 2 - 1.5, 1.6)
        blade = Rot(a, 0, 0) * Pos(-DEPTH / 2, r_mid, 0) * Rot(0, 0, 0) * Rot(0, 35, 0) * blade
        body = body + blade
    for i in range(STRUTS):                                         # 배기면 (앞면, x=0 쪽) 지지대
        a = 360.0 * i / STRUTS + 45.0
        s = Rot(a, 0, 0) * Pos(-1.5, (HUB_D / 2 + SIZE / 2) / 2, 0) * Box(3.0, SIZE / 2 - HUB_D / 2, 2.4)
        body = body + s
    body = body + Pos(-1.5, 0, 0) * Cylinder(HUB_D / 2, 3.0, rotation=(0, 90, 0))   # 라벨 원판 (배기면)
    # 케이블은 라벨면 (판 쪽) 지지대를 따라 프레임 옆으로 [사진] — 아래로 내리면 바닥판 (팬 밑 3 mm) 에 닿는다
    cable = Pos(-3.5, cable_dir * (SIZE / 2 + CABLE_L / 2 - 1.0), -SIZE / 2 + 8.0) * Cylinder(1.6, CABLE_L, rotation=(90, 0, 0))
    return body + cable


def guard_80():
    """바깥 핑거 가드 — 동심 와이어 링 + 십자 살 + 네 귀. 원점 = 판 바깥면, 가드는 +X (밖) 로 볼록."""
    ax = dict(rotation=(0, 90, 0))
    out = None
    for i, d in enumerate(GUARD_RINGS):
        x = GUARD_WIRE / 2 + (GUARD_H - GUARD_WIRE) * (i / (len(GUARD_RINGS) - 1))   # 전체 높이 5.5
        with BuildPart() as r:
            with Locations((x, 0, 0)):
                Cylinder(d / 2 + GUARD_WIRE / 2, GUARD_WIRE, **ax)
                Cylinder(d / 2 - GUARD_WIRE / 2, GUARD_WIRE + 0.02, **ax, mode=Mode.SUBTRACT)
        out = r.part if out is None else out + r.part
    for a in (0.0, 90.0):                                           # 십자 살
        out = out + Rot(a, 0, 0) * Pos(GUARD_WIRE / 2 + GUARD_H / 2, 0, 0) * Box(GUARD_WIRE, GUARD_RINGS[0], GUARD_WIRE)
    for sy in (-1, 1):                                              # 네 귀 (나사 자리)
        for sz in (-1, 1):
            y, z = sy * HOLE_PITCH / 2, sz * HOLE_PITCH / 2
            ear = Pos(GUARD_WIRE / 2, y, z) * Cylinder(4.0, GUARD_WIRE, rotation=(0, 90, 0))
            ear = ear - Pos(GUARD_WIRE / 2, y, z) * Cylinder(GUARD_EAR_D / 2, GUARD_WIRE + 0.02, rotation=(0, 90, 0))
            arm_len = math.hypot(y, z) - GUARD_RINGS[0] / 2 + 2.0
            arm = Pos(GUARD_WIRE / 2, 0, 0) * Rot(math.degrees(math.atan2(z, y)), 0, 0) \
                * Pos(0, GUARD_RINGS[0] / 2 + arm_len / 2 - 2.0, 0) * Box(GUARD_WIRE, arm_len, GUARD_WIRE)
            out = out + ear + arm
    return out
