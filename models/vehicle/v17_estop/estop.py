r"""비상정지 버튼 — Schneider Electric Harmony XB5AS8442 (실물 치수 모델, v17).

구성: 헤드 ZB5AS844 (Ø40 적색 버섯, 돌려서 해제) + 본체 ZB5AZ102 (고정 칼라 ZB5AZ009 + 접점 ZBE102 1NC)
     + 노란 명판 ZBY9320 (Ø60 "EMERGENCY STOP") + 회전방지판 ZB5AZ902 (권장)
IP66 · IP67 · IP69 · IP69K · UL Type 4X/13, 직접개로 (IEC 60947-5-1 Annex K), IEC 60947-5-5 · ISO 13850.

출처 (2026-10-04 조사)
  · 2015 데이터시트 도면 · 컷아웃 https://datasheet.octopart.com/XB5AS8442-Schneider-Electric-datasheet-60721870.pdf
  · 현행 데이터시트 https://download.schneider-electric.com/files?p_enDocType=Product+Data+Sheet&p_File_Name=XB5AS8442_DATASHEET_WW_en-GB.pdf&p_Doc_Ref=XB5AS8442_DATASHEET_WW_en-GB
  · 본체 ZB5AZ102 CAD 외형 30 × 42 × 32, 무게 헤드 0.046 + 본체 0.021 kg

표기  [도면] 치수로 적힌 값  [도면 비례] 2015 도면을 비례로 잰 값 (±1)  [가정] 출처 없음
로컬 좌표: X = 버튼 축, X=0 = 패널 앞면, 헤드는 +X (밖), 본체는 -X (안). Y 폭, Z 높이 (위 +).
"""

from build123d import Axis, Box, BuildPart, Cylinder, Locations, Mode, Pos, fillet

PROJ = 39.5            # [도면] 패널 앞면 → 헤드 끝
DEPTH = 43.0           # [도면] 패널 앞면 → 본체 끝 (안쪽)
HEAD_D = 40.0          # [도면]
CAP_T = 8.5            # [도면 비례] 버섯 머리 두께
GRIP_D, GRIP_L = 29.0, 9.1      # [도면 비례] 톱니 그립 (돌려서 해제)
SHAFT_D = 26.7                  # [도면 비례]
BEZEL_D, BEZEL_L = 28.5, 6.1    # [도면 비례] 패널에 닿는 베젤
NUT_D, NUT_L = 28.5, 7.6        # [도면 비례] 패널 뒤 고정 링
COLLAR = (3.0, 30.0, 40.0)      # [도면 비례] 고정 칼라판 (두께 · 폭 · 높이), 위 돌기 포함 전체 높이 43 [도면]
BLOCK = (29.3, 29.8, 32.0)      # [도면 비례] 접점블록 ZBE102 (깊이 · 폭 · 높이) — 끝이 DEPTH 에 맞게
WEIGHT = 0.067                  # kg (헤드 0.046 + 본체 0.021)
CUTOUT_D = 22.5                 # [도면] 권장 (Ø22.3 +0.4)
NOTCH_W, NOTCH_TOP = 3.2, 12.95 # [도면] 회전방지 홈 폭, 중심 → 홈 끝 (24.1 - 22.3/2)
LEGEND_D, LEGEND_T = 60.0, 1.5  # ZBY9320 Ø60 [데이터시트], 두께 [가정]


def xb5as8442(legend=True, part="all"):
    """버튼 (헤드 + 본체). legend=True 면 노란 명판 (legend_only) 이 패널 앞면에 깔리므로 헤드를 그만큼 띄운다.
    명판은 색이 달라 따로 낸다."""
    lt = LEGEND_T if legend else 0.0
    ax = dict(rotation=(0, 90, 0))
    with BuildPart() as p:
        # --- 패널 앞 (헤드) — X 가 커질수록 바깥
        x = lt
        with Locations((x + BEZEL_L / 2, 0, 0)):
            Cylinder(BEZEL_D / 2, BEZEL_L, **ax)                    # 베젤
        x += BEZEL_L
        shaft_l = PROJ - BEZEL_L - GRIP_L - CAP_T
        with Locations((x + shaft_l / 2, 0, 0)):
            Cylinder(SHAFT_D / 2, shaft_l, **ax)                    # 축
        x += shaft_l
        with Locations((x + GRIP_L / 2, 0, 0)):
            Cylinder(GRIP_D / 2, GRIP_L, **ax)                      # 톱니 그립
        x += GRIP_L
        with Locations((x + CAP_T / 2, 0, 0)):
            Cylinder(HEAD_D / 2, CAP_T, **ax)                       # 버섯 머리
        try:
            fillet(p.edges().filter_by_position(Axis.X, x + CAP_T - 0.01, x + CAP_T + 0.01), radius=3.0)
        except Exception:                                          # noqa: BLE001
            pass
        for i in range(18):                                        # 그립 톱니 (축 방향 홈)
            import math
            a = 2 * math.pi * i / 18
            with Locations((x - GRIP_L / 2, (GRIP_D / 2) * math.cos(a), (GRIP_D / 2) * math.sin(a))):
                Box(GRIP_L + 0.02, 1.4, 1.4, mode=Mode.SUBTRACT)
    head = p.part
    # --- 패널 뒤 (본체) — 패널 두께는 호출하는 쪽이 정한다: 여기서는 3 mm 패널 기준으로 뒤에 둔다
    panel_t = 3.0
    xb = -panel_t
    nut = Pos(xb - NUT_L / 2, 0, 0) * Cylinder(NUT_D / 2, NUT_L, rotation=(0, 90, 0))
    xb -= NUT_L
    ct, cw, ch = COLLAR
    collar = Pos(xb - ct / 2, 0, 0) * Box(ct, cw, ch)
    lug = Pos(xb - ct / 2, 0, ch / 2) * Cylinder(3.0, ct, rotation=(0, 90, 0))   # 위 돌기 (전체 43)
    xb -= ct
    bd, bw, bh = BLOCK
    block = Pos(-DEPTH + bd / 2, 0, -1.0) * Box(bd, bw, bh)
    for z in (bh / 2 - 6, -bh / 2 + 4):                            # 단자 나사 자리 (위 · 아래) [가정]
        block = block - Pos(-DEPTH + 1.5, 0, z - 1.0) * Box(3.2, 7.0, 6.0)
    body = nut + collar + lug + block
    if part == "head":
        return head                     # 적색 헤드 (패널 앞)
    if part == "body":
        return body                     # 진회색 본체 (패널 뒤 — 고정 링 · 칼라 · 접점블록)
    return head + body


def legend_only():
    return Pos(LEGEND_T / 2, 0, 0) * Cylinder(LEGEND_D / 2, LEGEND_T, rotation=(0, 90, 0))
