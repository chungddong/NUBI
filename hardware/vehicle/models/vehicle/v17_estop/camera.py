r"""뎁스 카메라 — Orbbec Gemini 336L (실물 치수 모델, v16).

출처 (2026-10-04 조사)
  · Orbbec Gemini 330 series Datasheet V1.7 + Gemini 336L 공식 도면 (Drawing Gemini 336L V1.0, 20240517)
    https://orbbec-debian-repos-aws.s3.amazonaws.com/product/Gemini%20330%20series%20Datasheet-V1.7.pdf
  · 제품 https://www.orbbec.com/products/stereo-vision-camera/gemini-336l/
  · 국내 구매 아이씨뱅큐 https://www.icbanq.com/P016230045 — 560,000 (VAT 별도) / 616,000 (VAT 포함)

표기  [도면] 공식 도면 숫자  [도면 비례] 도면 그림에서 비례로 잰 값  [가정] 출처 없음

로컬 좌표: 렌즈면 = X=0 이 +X 를 본다, 몸통은 -X 쪽. Y = 폭 (렌즈면을 마주볼 때 오른쪽 +), Z = 높이 (가운데 0).
"""

from build123d import (Align, Axis, Box, BuildPart, BuildSketch, Cylinder, Locations, Mode, Plane,
                       Pos, SlotOverall, extrude, fillet)

W, H, D = 124.02, 29.03, 27.70          # [도면] 폭 · 높이 · 전체 깊이 (±0.3)
D_FRONT = 18.25                          # [도면] 앞쪽 전체 외곽 구간 깊이
WEIGHT = 0.135                           # kg [데이터시트]
IR_Y, BASELINE = 47.50, 95.0             # [도면] IR 카메라 ±47.5 (기준선 95)
IR_D, LDM_D, RGB_D, RGB_IN = 12.0, 6.70, 15.60, 12.00   # [도면] 창 지름
RGB_Y, LDM_Y, LRM_Y = 23.75, 0.0, -8.50  # [데이터시트 optics_x]
M4_PITCH, M4_DEPTH = 95.0, 4.0           # [도면] 뒷면 M4 ×2, 최대 삽입 4
TRIPOD_DEPTH = 8.0                       # [도면] 바닥 1/4-20, 최대 삽입 8
USB_Y, USB_X = -37.0, -21.0              # [도면 비례] 바닥 USB-C 자리 (렌즈면 볼 때 왼쪽 끝 쪽)
USB_LOCK_PITCH = 15.0                    # [도면] USB 잠금 M2 ×2 간격
FIN_N, FIN_W, FIN_D = 16, 2.0, 2.0       # [도면 비례] 뒷면 방열 핀 (홈 개수 · 폭 · 깊이)
GAP = 0.30                               # 판 구멍 여유 (한쪽) — EPDM 개스킷 자리
WINDOW = (W + 2 * GAP, H + 2 * GAP)      # 판 구멍 (스타디움 모양)


def gemini_336l(with_plug=True):
    """Gemini 336L 몸통 + (선택) ㄱ자 USB-C 플러그 · 케이블 토막. 밑면 Z=-H/2."""
    with BuildPart() as p:
        with BuildSketch(Plane.YZ.offset(-D_FRONT)):              # 앞 구간 — 스타디움 외곽
            SlotOverall(W, H)
        extrude(amount=D_FRONT)
        with BuildSketch(Plane.YZ.offset(-D)):                    # 뒤 구간 — 조금 작은 스타디움 [도면 비례]
            SlotOverall(W - 3.0, H - 3.0)
        extrude(amount=D - D_FRONT)
        try:                                                      # 앞 테두리 둥글림 [도면 비례]
            fillet(p.edges().filter_by_position(Axis.X, -0.01, 0.01), radius=2.0)
        except Exception:                                         # noqa: BLE001
            pass
        with BuildSketch(Plane.YZ):                               # 앞 유리 오목 (테두리 안쪽)
            SlotOverall(W - 7.0, H - 7.0)
        extrude(amount=-0.6, mode=Mode.SUBTRACT)
        for y, d, depth in ((IR_Y, IR_D, 1.5), (-IR_Y, IR_D, 1.5), (LDM_Y, LDM_D, 1.5),
                            (RGB_Y, RGB_D, 0.8), (RGB_Y, RGB_IN, 2.0), (LRM_Y, 4.0, 1.0)):
            with Locations((-depth / 2, y, 0)):
                Cylinder(d / 2, depth + 0.01, rotation=(0, 90, 0), mode=Mode.SUBTRACT)
        for i in range(FIN_N):                                    # 뒷면 방열 핀 홈
            y = -IR_Y + 8 + i * (2 * IR_Y - 16) / (FIN_N - 1)
            with Locations((-D + FIN_D / 2, y, 0)):
                Box(FIN_D + 0.01, FIN_W, H - 8.0, mode=Mode.SUBTRACT)
        for y in (-M4_PITCH / 2, M4_PITCH / 2):                   # 뒷면 M4 (탭 Ø3.3)
            with Locations((-D + M4_DEPTH / 2, y, 0)):
                Cylinder(1.65, M4_DEPTH, rotation=(0, 90, 0), mode=Mode.SUBTRACT)
        with Locations((-D_FRONT * 0.75, 0, -H / 2 + TRIPOD_DEPTH / 2)):   # 바닥 1/4-20
            Cylinder(3.175, TRIPOD_DEPTH, mode=Mode.SUBTRACT)
        with Locations((USB_X, USB_Y, -H / 2 + 3.5)):             # 바닥 USB-C 리셉터클
            Box(3.5, 9.2, 7.0, mode=Mode.SUBTRACT)
        for dy in (-USB_LOCK_PITCH / 2, USB_LOCK_PITCH / 2):      # 잠금 M2
            with Locations((USB_X, USB_Y + dy, -H / 2 + 1.75)):
                Cylinder(0.8, 3.5, mode=Mode.SUBTRACT)
    body = p.part
    if with_plug:                                                 # ㄱ자 USB-C 플러그 (안쪽 -X 로 꺾임) [가정]
        plug = Pos(USB_X - 3.0, USB_Y, -H / 2 - 4.0) * Box(14.0, 12.0, 8.0)
        cable = Pos(USB_X - 10.0 - 15.0, USB_Y, -H / 2 - 4.0) * Cylinder(2.25, 30.0, rotation=(0, 90, 0))
        body = body + plug + cable
    return body


def bracket(panel_t=3.0):
    """ASA 브래킷 — 뒷판 (카메라 뒷면 M4 ×2) + 양옆 팔 + 판 안쪽면 플랜지 (M3 ×2, 인서트).
    카메라 렌즈면이 판 바깥면 (X=0) 과 같은 높이로 오게 판 안쪽면 (X=-panel_t) 에 붙는다."""
    t = 4.0
    arm_y0 = W / 2 + 3.0                                          # 팔 안쪽면 (카메라 끝에서 3)
    out = Pos(-D - t / 2, 0, 0) * Box(t, 2 * (arm_y0 + t), H)     # 뒷판
    for y in (-M4_PITCH / 2, M4_PITCH / 2):
        out = out - Pos(-D - t / 2, y, 0) * Cylinder(2.25, t + 1, rotation=(0, 90, 0))
    for sy in (-1, 1):
        arm_x0, arm_x1 = -D - t, -panel_t
        out = out + Pos((arm_x0 + arm_x1) / 2, sy * (arm_y0 + t / 2), 0) * Box(arm_x1 - arm_x0, t, H)
        fl = Pos(-panel_t - 1.5, sy * (arm_y0 + t + 6.0), 0) * Box(3.0, 12.0, H)
        fl = fl - Pos(-panel_t - 1.5, sy * (arm_y0 + t + 6.0), 0) * Cylinder(1.7, 4.0, rotation=(0, 90, 0))
        out = out + fl
    return out


BRACKET_HOLE_Y = W / 2 + 3.0 + 4.0 + 6.0   # 판의 M3 구멍 y (±)
