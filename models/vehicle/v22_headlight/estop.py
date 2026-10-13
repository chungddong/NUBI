r"""비상정지 버튼 — Autonics SF2ER-E2RB (D40 짧은 헤드, 비조광, 1NC) (실물 치수 모델, v18).

v17 은 Schneider XB5AS8442 (IP69K) 였다. 사용자가 IP 등급은 기준에서 빼라고 해서 (2026-10-04)
국내 정품 · 재고 · 가격 (엘레파츠 23,100 VAT 포함) 이 나은 Autonics 로 바꿨다.

출처
  · Autonics 매뉴얼 TCD210164AC p.2–4 (치수 · 컷아웃) https://adv.quasarglobal.com/2BYBAdjuntosTienda/6a2e5936-79f5-47b1-96ce-c55da08bc3e7.pdf
  · 구매 https://www.eleparts.co.kr/goods/view?no=13038016
표기  [도면] 치수로 적힌 값  [도면 비례] 도면을 비례로 잰 값 (±0.5)  [가정] 출처 없음

로컬 좌표: X = 버튼 축, X=0 = 패널 앞면, 헤드는 +X (밖), 본체는 -X (안). Y 폭, Z 높이 (어댑터 레버 탭이 +Z).
"""

import math

from build123d import Box, BuildPart, Cylinder, Locations, Mode, Pos

PROJ = 33.7            # [도면] 패널 앞면 → 헤드 끝
DEPTH = 48.6           # [도면] 패널 앞면 → 본체 끝
HEAD_D = 40.0          # [도면] 톱니 링
KNURL_T = 5.0          # [도면 비례] 톱니 링 두께
HEAD_BODY_D, HEAD_BODY_L = 30.9, 14.9   # [도면 비례]
SHAFT_D = 27.9                          # [도면 비례]
BEZEL_D, BEZEL_L = 29.4, 2.8            # [도면 비례]
NUT_D, NUT_L = 30.9, 5.0                # [도면 비례] 패널 뒤 고정 너트 (톱니)
ADAPTER = (4.3, 30.4, 38.7)             # [도면 비례] 어댑터 틀 (두께 · 폭 [도면] · 높이), 레버 탭 포함 높이 48.1 [도면]
TAB = (4.3, 10.0, 48.1 - 38.7)          # [도면 비례] 레버 탭 (두께 · 폭 · 틀 위로 높이)
BLOCK = (26.9, 30.0, 39.2)              # 접점블록 깊이 · 폭 [가정] · 높이 [도면]
WEIGHT = 0.066                          # kg [매뉴얼, 접점블록 3 개 기준 약 66 g]
CUTOUT_D = 22.5                         # [도면]
NOTCH_W, NOTCH_TOP = 3.0, 13.0          # [도면] 홈 폭, 중심 → 홈 끝 (R1)
LEGEND_D, LEGEND_T = 60.0, 1.0          # 명판 SF2ER-B3 Ø60 · 두께 1 [매뉴얼]


def sf2er(legend=True, part="all"):
    """part = "head" (적색, 패널 앞) / "body" (회색, 패널 뒤) / "all"."""
    lt = LEGEND_T if legend else 0.0
    ax = dict(rotation=(0, 90, 0))
    with BuildPart() as p:
        x = lt
        for d, L in ((BEZEL_D, BEZEL_L), (SHAFT_D, PROJ - BEZEL_L - HEAD_BODY_L - KNURL_T),
                     (HEAD_BODY_D, HEAD_BODY_L), (HEAD_D, KNURL_T)):
            with Locations((x + L / 2, 0, 0)):
                Cylinder(d / 2, L, **ax)
            x += L
        n = 60                                                    # 톱니 링 (앞에서 본 그림의 이)
        for i in range(n):
            a = 2 * math.pi * i / n
            with Locations((x - KNURL_T / 2, (HEAD_D / 2) * math.cos(a), (HEAD_D / 2) * math.sin(a))):
                Box(KNURL_T + 0.02, 1.0, 1.0, rotation=(math.degrees(a), 0, 0), mode=Mode.SUBTRACT)
    head = p.part
    panel_t = 3.0
    xb = -panel_t
    body = Pos(xb - NUT_L / 2, 0, 0) * Cylinder(NUT_D / 2, NUT_L, rotation=(0, 90, 0))
    xb -= NUT_L
    body = body + Pos(xb - 1.0, 0, 0) * Cylinder(13.0, 2.0, rotation=(0, 90, 0))     # 나사부 [가정]
    xb -= 2.0
    at, aw, ah = ADAPTER
    body = body + Pos(xb - at / 2, 0, 0) * Box(at, aw, ah)
    tt, tw, th = TAB
    body = body + Pos(xb - tt / 2, 0, ah / 2 + th / 2) * Box(tt, tw, th)            # 레버 탭 (+Z)
    xb -= at
    bd, bw, bh = BLOCK
    legs_l = (-DEPTH + bd) - xb                                                     # 어댑터 ~ 접점블록 사이 다리
    body = body + Pos(xb + legs_l / 2, 0, 0) * Box(abs(legs_l), bw - 4.0, bh - 6.0)
    blk = Pos(-DEPTH + bd / 2, 0, 0) * Box(bd, bw, bh)
    for z in (bh / 2 - 6, -bh / 2 + 6):                                             # M3.5 단자 자리 [가정]
        blk = blk - Pos(-DEPTH + 1.5, 0, z) * Box(3.2, 8.0, 7.0)
    body = body + blk
    if part == "head":
        return head
    if part == "body":
        return body
    return head + body


def legend_only():
    return Pos(LEGEND_T / 2, 0, 0) * Cylinder(LEGEND_D / 2, LEGEND_T, rotation=(0, 90, 0))
