r"""뎁스 카메라 — Orbbec Gemini 336 (실물 치수 모델, v19).

v16~v18 은 Gemini 336L (IP65, 기준선 95, 616,000 ×2) 이었다. 사용자가 IP 등급을 기준에서 빼서 (2026-10-04)
햇빛에 강한 IR-pass 필터는 그대로 두고 싼 Gemini 336 으로 바꿨다 (462,000 ×2, 308,000 싸다).
대가: 기준선 50 → 이상 범위 0.3–3 m (336L 0.25–6 m). 작동 범위는 0.1–20 m+.

출처 (2026-10-04 조사)
  · Gemini 330 series Datasheet V1.6 — p10 사양, p13 도면 표, p38 좌표계, p53 전력, 부록 C p66 Gemini 336 도면 (V1.0, 20240107)
    https://new-orbbec3d-s3.s3.amazonaws.com/wp-content/uploads/2025/04/22062452/Gemini-330-series-Datasheet-V1.6.pdf
  · 제품 https://www.orbbec.com/products/stereo-vision-camera/gemini-336/
  · 국내 구매 아이씨뱅큐 https://www.icbanq.com/P016230044 — 462,000 (VAT 포함, 출고 1주 이내)
  · 공식 STEP (내려받지 않음) https://orbbec-debian-repos-aws.s3.amazonaws.com/product/Gemini336_CAD_Files_V2.0.zip

표기  [도면] 공식 도면 숫자  [도면 비례] 도면 그림에서 비례로 잰 값 (±0.7)  [가정] 출처 없음

도면은 3각법 (표제란 기호). 왼쪽 측면도 = 렌즈를 마주볼 때 왼쪽 끝 → USB-C 는 그쪽 끝 (-Y).
제품 사진은 반대로 보이기도 해서, 브래킷은 양 끝을 막지 않게 아래 · 뒤에서만 잡는다.

로컬 좌표: 렌즈면 = X=0 이 +X 를 본다, 몸통은 -X 쪽. Y = 폭 (렌즈면을 마주볼 때 오른쪽 +, RGB 쪽), Z = 높이 (가운데 0).
"""

from build123d import (Axis, Box, BuildPart, BuildSketch, Cylinder, Locations, Mode, Plane, Pos,
                       Rectangle, SlotOverall, extrude, fillet)

W, H, D = 89.46, 25.0, 30.70            # [도면] 폭 · 높이 · 전체 깊이 (±0.3)
D_FRONT = 21.0                           # [도면 비례] 앞 껍데기 (스타디움, 전체 폭) 깊이
BACK_W, BACK_H = 70.6, 24.0              # [도면 비례] 뒤 블록 (방열 핀 · USB · sync 포트)
FIN_N, FIN_W, FIN_D = 18, 1.6, 4.4       # [도면 비례] 뒷면 세로 방열 핀 홈
WEIGHT = 0.099                           # kg [데이터시트]
IR_Y, BASELINE = 25.0, 50.0              # [도면] IR ±25 (검은 IR-pass 유리 뒤라 안 보인다)
LDM_D, RGB_D, RGB_Y = 7.0, 9.0, 11.0     # [도면] 프로젝터 Ø7 (가운데), RGB Ø9 (y=+11)
LRM_Y, LRM_W, LRM_H = -8.5, 2.4, 4.3     # [도면] dToF 슬롯
WIN_L, WIN_H = 32.4, 10.4                # [도면] 투명 창 (스타디움) — LRM · LDM · RGB 가 보이는 곳
M3_PITCH, M3_DEPTH = 45.0, 3.0           # [도면] 뒷면 M3 ×2, 최대 삽입 3 (0.4 N·m)
TRIPOD_X, TRIPOD_DEPTH = -20.5, 5.0      # 바닥 1/4-20 — 앞에서 20.5 [도면 비례], 최대 삽입 5 [도면] (4 N·m)
USB_X = -D + 5.0                         # [도면 비례] 왼쪽 끝 (-Y) USB-C — 뒷면에서 5, 세로 방향
USB_LOCK_PITCH = 15.0                    # [도면] USB 잠금 M2 ×2 (세로), 최대 삽입 3
GAP = 0.30                               # 판 구멍 여유 (한쪽)
WINDOW = (W + 2 * GAP, H + 2 * GAP)      # 판 구멍 (스타디움 모양)


def gemini_336(with_plug=True):
    """Gemini 336 몸통 + (선택) 일자 USB-C 플러그 (M2 나사 잠금) · 케이블 토막. 밑면 Z=-H/2."""
    with BuildPart() as p:
        with BuildSketch(Plane.YZ.offset(-D_FRONT)):              # 앞 껍데기 — 스타디움 89.46 × 25
            SlotOverall(W, H)
        extrude(amount=D_FRONT)
        try:                                                      # 앞으로 갈수록 좁아지는 테두리 [도면 비례]
            fillet(p.edges().filter_by_position(Axis.X, -0.01, 0.01), radius=2.5)
        except Exception:                                         # noqa: BLE001
            pass
        with BuildSketch(Plane.YZ.offset(-D)):                    # 뒤 블록
            Rectangle(BACK_W, BACK_H)
        extrude(amount=D - D_FRONT + 0.01)
        try:
            fillet(p.edges().filter_by(Axis.X).filter_by_position(Axis.X, -D, -D_FRONT - 0.5), radius=3.0)
        except Exception:                                         # noqa: BLE001
            pass
        with BuildSketch(Plane.YZ):                               # 검은 앞 유리 (테두리 안쪽 오목)
            SlotOverall(W - 6.0, H - 6.0)
        extrude(amount=-0.5, mode=Mode.SUBTRACT)
        with BuildSketch(Plane.YZ.offset(-0.5)):                  # 투명 창
            SlotOverall(WIN_L, WIN_H)
        extrude(amount=-0.3, mode=Mode.SUBTRACT)
        for y, d, depth in ((0.0, LDM_D, 1.5), (RGB_Y, RGB_D, 2.0)):
            with Locations((-0.8 - depth / 2, y, 0)):
                Cylinder(d / 2, depth + 0.01, rotation=(0, 90, 0), mode=Mode.SUBTRACT)
        with Locations((-0.8 - 0.5, LRM_Y, 0)):
            Box(1.0, LRM_W, LRM_H, mode=Mode.SUBTRACT)
        y0 = -BACK_W / 2 + 4.0                                    # 뒷면 방열 핀 홈 (M3 자리는 비운다)
        for i in range(FIN_N):
            y = y0 + i * (BACK_W - 8.0) / (FIN_N - 1)
            if min(abs(y - s * M3_PITCH / 2) for s in (-1, 1)) < 3.0:
                continue
            with Locations((-D + FIN_D / 2, y, 0)):
                Box(FIN_D + 0.01, FIN_W, BACK_H - 5.0, mode=Mode.SUBTRACT)
        for y in (-M3_PITCH / 2, M3_PITCH / 2):                   # 뒷면 M3 (탭 Ø2.5)
            with Locations((-D + M3_DEPTH / 2, y, 0)):
                Cylinder(1.25, M3_DEPTH, rotation=(0, 90, 0), mode=Mode.SUBTRACT)
        with Locations((TRIPOD_X, 0, -H / 2 + TRIPOD_DEPTH / 2)):  # 바닥 1/4-20
            Cylinder(3.175, TRIPOD_DEPTH, mode=Mode.SUBTRACT)
        with Locations((USB_X, -BACK_W / 2 + 3.5, 0)):            # 왼쪽 끝 USB-C 리셉터클 (세로)
            Box(3.5, 7.0, 9.2, mode=Mode.SUBTRACT)
        for dz in (-USB_LOCK_PITCH / 2, USB_LOCK_PITCH / 2):      # 잠금 M2
            with Locations((USB_X, -BACK_W / 2 + 1.5, dz)):
                Cylinder(0.8, 3.0, rotation=(90, 0, 0), mode=Mode.SUBTRACT)
        with Locations((USB_X, BACK_W / 2 - 2.0, 0)):             # 오른쪽 끝 sync 8 핀 [도면 비례]
            Box(6.0, 4.0, 14.0, mode=Mode.SUBTRACT)
    body = p.part
    if with_plug:                                                 # 일자 USB-C 플러그 + M2 나사 잠금 하우징 [가정]
        y1 = -BACK_W / 2
        plug = Pos(USB_X, y1 - 14.0, 0) * Box(7.0, 28.0, 21.0)
        boot = Pos(USB_X, y1 - 28.0 - 6.0, 0) * Cylinder(3.5, 12.0, rotation=(90, 0, 0))
        cable = Pos(USB_X, y1 - 40.0 - 15.0, 0) * Cylinder(2.5, 30.0, rotation=(90, 0, 0))
        body = body + plug + boot + cable
    return body


# ---- 브래킷 (ASA) — 카메라 밑 선반 (1/4-20) + 뒷판 (뒷면 M3 ×2) + 판 안쪽면 플랜지 (M3 ×2, 인서트).
# 양 끝 (USB-C · sync) 을 막지 않는다. 판 구멍 z 는 뒤판 포트 칸 바깥 테두리 (z≤227) 위로 올린다.
BR_T = 4.0                    # 두께
BR_W = 60.0                   # 폭 (y)
FLANGE_Z0 = -25.0             # 플랜지 아래끝 (카메라 중심 기준) — 포트 칸 윗벽 z=222 위 3
HOLE_DZ = -19.0               # 판 M3 구멍 높이 (카메라 중심 기준) — 바깥 볼트 머리가 포트 칸 테두리 위
BRACKET_HOLES = [(-20.0, HOLE_DZ), (20.0, HOLE_DZ)]   # (y, z) 판 M3 구멍


def bracket(panel_t=3.0):
    """카메라 렌즈면이 판 바깥면 (X=0) 과 같은 면에 오게 판 안쪽면 (X=-panel_t) 에 붙는다."""
    t, zb = BR_T, -H / 2
    x0, x1 = -D - t, -panel_t                                     # 뒷판 뒤끝 ~ 판 안쪽면
    shelf = Pos((x0 + x1) / 2, 0, zb - t / 2) * Box(x1 - x0, BR_W, t)
    rear = Pos(-D - t / 2, 0, (zb - t + 8.0) / 2) * Box(t, BR_W, 8.0 - (zb - t))
    fl = Pos(x1 - t / 2, 0, (FLANGE_Z0 + zb) / 2) * Box(t, BR_W, zb - FLANGE_Z0)
    out = shelf + rear + fl
    out = out - Pos(TRIPOD_X, 0, zb - t / 2) * Cylinder(3.3, t + 1)                            # 1/4-20 통과 Ø6.6
    for s in (-1, 1):
        out = out - Pos(-D - t / 2, s * M3_PITCH / 2, 0) * Cylinder(1.7, t + 1, rotation=(0, 90, 0))  # M3 통과
    for y, z in BRACKET_HOLES:                                    # 판 쪽 M3 열압입 인서트 구멍 Ø4.0
        out = out - Pos(x1 - t / 2, y, z) * Cylinder(2.0, t + 1, rotation=(0, 90, 0))
    return out
