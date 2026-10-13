r"""바닥판 위 부품 — **데이터시트 · 판매처 공개 치수**로 짓는다 (v08).

치수 출처는 아래 각 항목 src. 2026-10-04 조사. 표기
  [도면]  제조사 도면 숫자          [도면 비례] 제조사 도면 래스터에서 비례로 잰 값 (±0.5)
  [판매처] 판매 페이지 사양표       [가정]  출처 없음 — 실물 받으면 고칠 값

배치 원칙 (v07 과 같다)
  · 배터리 가운데 (x=0 크로스 위) · 드라이버 앞뒤 끝 칸 · 전원은 배터리 앞 · 젯슨 · ESP32 는 뒤
  · 드라이버는 90° 돌려 **모터 단자 끝면이 좌 · 우 바퀴 쪽**을 본다 (그로밋 y=±154 와 가깝다)
  · 젯슨 I/O 변은 뒤 (-x) 를 본다 — 케이블 꽂을 자리
"""

from build123d import BuildSketch, Plane, Polygon, Pos, Rot, add, extrude

import components as C

# ---------------------------------------------------------------- 고정 구멍 (부품 로컬, 원점 = 밑면 가운데)
# 판에 뚫을 구멍 = 부품 고정 구멍. (x, y, 판 구멍 Ø, 볼트). v09 실외판에서 격자 대신 쓴다.
_OX, _OY = -50.0, -39.5                                              # 젯슨 도면 원점 → 가운데
JETSON_HOLES = [(_OX + x, _OY + y) for x, y in ((4, 17), (90, 17), (4, 75), (90, 75))]
ESP32_CARRIER_HOLES = [(sx * 31.0, sy * 21.0) for sx in (-1, 1) for sy in (-1, 1)]
HB_SINK = (175.0, 100.0, 3.0)               # v20: 호버보드 보드 밑 알루미늄 방열판 (로컬 x × y × t) [가정]
HB_SINK_HOLES = [(sx * 80.0, sy * 42.0) for sx in (-1, 1) for sy in (-1, 1)]
MOUNT_HOLES = {
    # v20: ZLAC8015D → 호버보드 원래 메인보드 (방열판 네 귀 M5 로 판에 고정, 보드는 방열판에 M4 2 개)
    "hoverboard_mainboard": [(x, y, 5.5, "M5") for x, y in HB_SINK_HOLES],
    "jetson_orin_nano": [(x, y, 3.4, "M3 스탠드오프") for x, y in JETSON_HOLES],
    "esp32_devkitc": [(x, y, 3.4, "M3 스탠드오프") for x, y in ESP32_CARRIER_HOLES],
    "dc_breaker_din": [(sx * 18.0, 0.0, 4.5, "M4") for sx in (-1, 1)],          # v20: DIN 레일 고정
    "contactor_gpr_m040": [(sx * 28.0, 0.0, 4.5, "M4") for sx in (-1, 1)],      # v20: [도면] 56 간격
    "dcdc_12v_rsd60": [(sx * 50.0, 0.0, 3.4, "M3 [가정]") for sx in (-1, 1)],
    "dcdc_5v_dkp55": [(sx * 29.5, 0.0, 3.5, "M3 [가정]") for sx in (-1, 1)],
    "iahrs_rb_sda": [(sx * 14.5, sy * 14.5, 2.4, "M2 스탠드오프") for sx in (-1, 1) for sy in (-1, 1)],
}
BAT_L, BAT_W, BAT_H = 245.0, 95.0, 75.0     # v20: 알엠파워 ZENION 36V 20Ah 10S4P 직사형 [판매처]
STRAP_W, STRAP_SLOT = 25.0, (28.0, 5.0)    # 배터리 고정 밴드 (폭 25) 슬롯 (x × y)
STRAP_X = (-40.0, 40.0)                    # 밴드 2 줄 x — ±60 은 젯슨 · ESP32 · IMU 고정 구멍에 붙었다 (v09 검사)


# ---------------------------------------------------------------- 부품 빌더 (원점 = 밑면 가운데)


def zlac8015d():
    """ZLTECH ZLAC8015D — 150 (모터단자 끝면 사이) × 97 × 31 [도면]. 0.4 kg [판매처].
    귀 구멍 Ø4.5, y 간격 65.5 [도면], x 간격 141.5 [도면 비례]. 끝면 착탈 단자 13.3 돌출 [도면].
    몸통 132 × 97 × 27 + 밑판 4 는 [가정] (귀가 몸통 밖으로 나오게 나눔)."""
    L, W, H, base = 150.0, 97.0, 31.0, 4.0
    body_l = 132.0
    parts = [C._box(body_l, W, H - base, 0, 0, base)]
    parts += C.finned_box(L, W, base, ears=None)                     # 밑판 (방열판)
    with C.BuildPart() as holes:                                     # 귀 구멍 4 — 밑판에서 뺀다
        with C.Locations(*[(sx * 70.75, sy * 32.75) for sx in (-1, 1) for sy in (-1, 1)]):
            C.Cylinder(2.25, base, align=C.MIN)
    parts[1] = parts[1] - holes.part
    for sx in (-1, 1):                                               # 끝면 단자: 전원 · 모터 5p + 엔코더 12p
        parts.append(C._box(13.3, 40, 14, sx * (body_l / 2 + 6.65), -18, base + 4))
        parts.append(C._box(13.3, 30, 10, sx * (body_l / 2 + 6.65), 24, base + 6))
    return parts


def jetson_orin_nano():
    """Jetson Orin Nano Developer Kit (P3768) — 키트 외곽 103 × 90.5 × 34.77 [도면],
    캐리어 100 × 79 × 1.57 [도면], 구멍 86 × 58 [도면 비례], 히트싱크 · 팬 57.6 × 38.6,
    PCB 위 26.8 [도면 비례], PCB 밑면 6.4 [도면 비례]. 0.175 kg [판매처].
    원점은 PCB 가운데로 옮겼다 — x 를 I/O 변 방향, -y 쪽이 I/O 변."""
    pcb_l, pcb_w, t, z_pcb = 100.0, 79.0, 1.57, 6.4
    ox, oy = -pcb_l / 2, -pcb_w / 2                                  # 도면 원점 (I/O 변 왼쪽 모서리)
    # v09: 실외에서는 플라스틱 받침을 빼고 M3 스탠드오프 4 개로 판에 바로 고정한다 (높이는 받침과 같게)
    holes = JETSON_HOLES
    parts = C.pcb(pcb_l, pcb_w, t, holes=holes, hole_d=2.9, standoff=z_pcb)
    top = z_pcb + t
    hs = (19.4, 77.0, 39.6, 78.2)                                    # 히트싱크 · 팬 x, y 구간
    parts.append(C._box(hs[1] - hs[0], hs[3] - hs[2], 26.8,
                        ox + (hs[0] + hs[1]) / 2, oy + (hs[2] + hs[3]) / 2, top))
    io = {"DC잭": (1.3, 10.7, 11.0), "DP": (13.7, 31.9, 14.0), "USB-A1": (35.5, 49.7, 16.7),
          "USB-A2": (52.5, 66.7, 16.7), "RJ45": (68.8, 85.2, 13.5), "USB-C": (87.5, 96.5, 3.5)}
    for x0, x1, h in io.values():                                    # I/O 변 커넥터 (높이 일부 [가정])
        parts.append(C._box(x1 - x0, 16.0, h, ox + (x0 + x1) / 2, oy + 8.0 - 1.8, top))
    return parts


def esp32_devkitc():
    """ESP32-DevKitC V4 — 48.26 × 27.94 [도면], 핀헤더 2 열 c-c 25.40 [도면],
    안테나 18 폭 6.04 돌출 [도면]. 헤더 높이 8.5 · 모듈 높이 3.1 은 [가정].
    고정 구멍이 없어서 v09 부터 **받침보드** (만능기판 70 × 50, M3 스탠드오프 4, [가정]) 에 꽂는다."""
    L, W, t, hz = 48.26, 27.94, 1.6, 8.5
    parts = C.pcb(70.0, 50.0, 1.6, holes=ESP32_CARRIER_HOLES, hole_d=3.2, standoff=5.0)
    base = 5.0 + 1.6                                                 # 받침보드 윗면
    parts += [C._box(L, 2.54, hz, 0, sy * 12.70, base) for sy in (-1, 1)]   # 핀헤더 2 열 (아래로)
    parts.append(C._box(L, W, t, 0, 0, base + hz))
    parts.append(C._box(18.0 + 6.04, 18.0, 3.1, L / 2 - (18.0 + 6.04) / 2 + 6.04, 0, base + hz + t))
    parts.append(C._box(5.0, 7.5, 2.5, -L / 2 + 2.0, 0, base + hz + t))       # 마이크로 USB
    return parts


def battery_volton_20ah():
    """볼트온 36V 20Ah 기본형 — 234 × 105 × 75, 3.2 kg, XT60 × 2 (리드 400) [판매처].
    리드는 40 만 그린다."""
    return C.battery_pack(234.0, 105.0, 75.0, r=8.0, lead_side=+1, lead_d=8.0, lead_len=40.0)


def dcdc_19v():
    """Fidida 30–60V → 19V — 74 × 74 × 32, 0.28 kg [판매처]. 귀는 전선 나오는 면에 수직인 두 면.
    귀 크기 · 구멍 (Ø5, 같은 형태 MGGi 하우징 기준) 은 [가정] — 실물 재야 함. 전선은 -x 면."""
    L, W, H = 74.0, 74.0, 32.0
    parts = [C._box(L, W, H)]
    for sy in (-1, 1):                                               # 귀 (±y 면)
        with C.BuildPart() as e:
            C.Box(L * 0.5, 10.0, 3.0, align=C.MIN)
            C.Cylinder(2.5, 3.0, align=C.MIN, mode=C.Mode.SUBTRACT)
        parts.append(Pos(0, sy * (W / 2 + 5.0), 0) * e.part)
    for dy in (-15, -5, 5, 15):                                      # 전선 4 가닥 (-x 면, 20 만)
        parts.append(Pos(-L / 2, dy, 10) * Rot(0, -90, 0) * C.Cylinder(2.0, 20.0, align=C.MIN))
    return parts


def dcdc_5v():
    """DROK 200656 11–90V → 5V 6A — 66 (귀 끝↔끝) × 58 × 22 [판매처]. 귀 · 구멍은 [가정]."""
    L, W, H = 54.0, 58.0, 22.0
    parts = [C._box(L, W, H)]
    for sx in (-1, 1):
        with C.BuildPart() as e:
            C.Box(6.0, 24.0, 3.0, align=C.MIN)
            C.Cylinder(1.75, 3.0, align=C.MIN, mode=C.Mode.SUBTRACT)
        parts.append(Pos(sx * (L / 2 + 3.0), 0, 0) * e.part)
    for dx in (-12, -4, 4, 12):                                      # 전선 4 가닥 (-y 면)
        parts.append(Pos(dx, -W / 2, 8) * Rot(90, 0, 0) * C.Cylinder(1.5, 15.0, align=C.MIN))
    return parts


def midi_fuse_holder():
    """Victron MIDI 퓨즈 홀더 CIP000050001 — 51.36 × 29.36 (탭 포함) × 25.12 [도면],
    스터드 2 c-c 30 [도면], M5 [판매처], 고정 구멍 (±20.485, ±7.555) 대각 [도면], 58V · 200A."""
    return C.fuse_holder(51.36, 29.36, 25.12, stud_d=5.0, stud_h=12.0, stud_pitch=30.0,
                         holes=[(20.485, 7.555), (-20.485, -7.555)], hole_d=5.5)


def ssr_40dd():
    """Fotek SSR-40DD — 62 (귀 포함) × 45 × 22.5 [도면], 고정 c-c 48 · Ø4.8 [도면],
    입력 4–32VDC · 출력 5–60VDC. 단자 자리는 [가정] (양 끝 2 개씩)."""
    return C.ssr(62.0, 45.0, 22.5, ear_holes=[(-24.0, 0.0), (24.0, 0.0)], hole_d=4.8,
                 terminals=[(sx * 14.0, sy * 11.0) for sx in (-1, 1) for sy in (-1, 1)])


def bno085():
    """Adafruit BNO085 #4754 — 25.4 × 22.86 × 4.6 [도면], Ø2.5 구멍 20.32 × 17.78 [도면].
    M2.5 스탠드오프 5 는 [가정]."""
    holes = [(x - 12.7, y - 11.43) for x, y in ((2.54, 2.54), (22.86, 2.54), (2.54, 20.32),
                                                 (22.86, 20.32))]
    return C.pcb(25.4, 22.86, 1.6, holes=holes, hole_d=2.5, standoff=5.0,
                 items=[(0.0, 0.8, 5.2, 5.2, 1.2)])


# ---------------------------------------------------------------- v20 부품 (전원 · 구동 계통 확정, DECISIONS.md D-03~D-07)
def hoverboard_mainboard():
    """호버보드 1 세대 일체형 메인보드 (STM32F103RCT6 / GD32F103RCT6, EFeru 지원) + 밑 알루미늄 방열판.

    치수는 EFeru README 핀아웃 사진을 MCU 리드 간격으로 비례해 잰 값 [사진 비례 ±5~8 %] — 실물 재서 고친다.
      · PCB 123 × 94 팔각형 (윗변 두 모서리 모따기 약 27), 밑 알루미늄 판 137 × 91 (두께 3 [추정]),
        판 아래쪽 좌우 탭 끝↔끝 157, 탭 구멍 간격 148 (M4 여유공) — PCB 는 판 위 4.5 (TO-220 두께)
      · MOSFET 12 (모터마다 6) 이 PCB 둘레에서 판에 나사로 눌린다. 위 전해콘덴서 4 (Ø11, 높이 25 [추정])
      · 아래 변 커넥터 줄 (좌 사이드보드 4p · 좌 홀 5p · 우 홀 5p · 우 사이드보드 4p, JST XH),
        윗변 배터리선 (XT60 꼬리), 좌우 변으로 모터 상선 3 가닥씩.
    보드 판은 호버보드에선 알루미늄 주조 프레임에 붙어 열을 뺀다 — PC 바닥판은 단열재라
    **알루미늄 방열판 3t (175 × 100) [가정]** 을 깔고 그 네 귀를 판에 M5 로 고정한다 (HB_SINK).
    로컬: x = 탭 방향 (157), +y = 윗변 (배터리선 · 콘덴서), -y = 커넥터 변. 원점 = 방열판 밑면 가운데."""
    sl, sw, st = HB_SINK
    parts = []
    with C.BuildPart() as s:                                         # 방열판 + 네 귀 M5
        C.Box(sl, sw, st, align=C.MIN)
        with C.Locations(*HB_SINK_HOLES):
            C.Cylinder(2.75, st, align=C.MIN, mode=C.Mode.SUBTRACT)
    parts.append(s.part)
    z = st
    pl_l, pl_w, pl_t, cy = 137.0, 91.0, 3.0, 0.0                     # 보드 알루미늄 판 (가운데 = 방열판 가운데)
    with C.BuildPart() as b:
        with C.Locations((0, cy, z)):
            C.Box(pl_l, pl_w, pl_t, align=C.MIN)
        for sx in (-1, 1):                                           # 아래 좌우 탭 (끝↔끝 157, 높이 18)
            with C.Locations((sx * (pl_l / 2 + 5.0), cy - pl_w / 2 + 9.0, z)):
                C.Box(10.0 + 0.02, 18.0, pl_t, align=C.MIN)
            with C.Locations((sx * 74.0, cy - pl_w / 2 + 13.5, z)):
                C.Cylinder(2.25, pl_t, align=C.MIN, mode=C.Mode.SUBTRACT)
    parts.append(b.part)
    zp = z + pl_t + 4.5                                              # PCB 밑면
    with C.BuildPart() as pcb:                                       # 팔각형 PCB 123 × 94 (윗변 모따기 27)
        with BuildSketch(Plane.XY.offset(zp)):
            Polygon((-61.5, -47.0), (61.5, -47.0), (61.5, 20.0), (34.5, 47.0), (-34.5, 47.0),
                      (-61.5, 20.0), align=None)
        extrude(amount=1.6)
    parts.append(pcb.part)
    for sx in (-1, 1):                                               # MOSFET (판 위 · PCB 밑, 둘레로 탭이 나옴)
        for y in (-35.0, -20.0, -5.0, 10.0):
            parts.append(C._box(10.0, 10.0, 4.5, sx * 62.0, y, z + pl_t))
        for k in (0, 1):
            parts.append(C._box(10.0, 10.0, 4.5, sx * (52.0 - 13.0 * k), 28.0 + 9.0 * k, z + pl_t))
    top = zp + 1.6
    for x in (-21.0, -7.0, 7.0, 21.0):                               # DC 링크 콘덴서 4
        parts.append(C._cyl(5.5, 25.0, x, 32.0, top))
    for x, w in ((-15.0, 12.5), (0.0, 15.0), (16.0, 15.0), (31.0, 12.5)):   # 아래 변 커넥터 줄
        parts.append(C._box(w, 6.0, 7.0, x, -47.0 + 2.0, top))
    parts.append(C._cyl(8.0, 10.0, -50.0, -40.0, top))               # 부저
    for dx in (-3.0, 3.0):                                           # 배터리선 (윗변 → +y, 30 만)
        parts.append(Pos(1.5 + dx, 47.0, top + 3.0) * Rot(-90, 0, 0) * C.Cylinder(2.0, 30.0, align=C.MIN))
    for sx in (-1, 1):                                               # 모터 상선 3 가닥 (좌우 변 → ±x, 25 만)
        for y in (-8.0, 0.0, 8.0):
            parts.append(Pos(sx * 61.5, y, top + 3.0) * Rot(0, sx * 90, 0) * C.Cylinder(2.2, 25.0, align=C.MIN))
    return parts


def battery_zenion_20ah():
    """알엠파워 ZENION 삼성 21700 36V 20Ah 10S4P 직사형 — 95 × 245 × 75, 3 kg, XT-90 [판매처].
    최대 방전 30 A · 최대 충전 10 A [판매처]. 리드는 40 만 그린다."""
    return C.battery_pack(BAT_L, BAT_W, BAT_H, r=8.0, lead_side=+1, lead_d=9.0, lead_len=40.0)


def dc_breaker_din():
    """DC 차단기 1P 250 VDC 40A (BA49-63 DC) — 18 × 80 × 71 [판매처], 35 mm DIN 레일 (7.5 높이, 50 길이) 위.
    로컬: 레일 = x 방향, 차단기 80 은 y, 레버는 위 (+z)."""
    rail = C._box(50.0, 35.0, 7.5)
    with C.BuildPart() as r:
        add(rail)
        with C.Locations(*[(sx * 18.0, 0) for sx in (-1, 1)]):
            C.Cylinder(2.25, 7.5, align=C.MIN, mode=C.Mode.SUBTRACT)
    return [r.part, C._box(18.0, 80.0, 71.0, 0, 0, 7.5), C._box(10.0, 14.0, 8.0, 0, 0, 7.5 + 71.0)]


def contactor_gpr_m040():
    """LS일렉트릭 GPR-M040 DC12V — 고전압 DC 릴레이 40 A, 1000 VDC, 코일 12 V 3.5 W [카탈로그].
    몸통 46 × 33.1 × 39.5, 귀 끝↔끝 67, 고정 Ø4.5 간격 56, 주단자 M4 간격 16 (전체 높이 47),
    코일 커넥터 (Tyco 1379658-1) 한쪽 옆 (35.31) [도면]. 145 g."""
    with C.BuildPart() as f:                                         # 귀 판
        C.Box(67.0, 16.0, 3.0, align=C.MIN)
        with C.Locations(*[(sx * 28.0, 0) for sx in (-1, 1)]):
            C.Cylinder(2.25, 3.0, align=C.MIN, mode=C.Mode.SUBTRACT)
    parts = [f.part, C._box(46.0, 33.1, 39.5)]
    for sx in (-1, 1):                                               # 주단자 (+ / -)
        parts.append(C._cyl(4.0, 7.5, sx * 8.0, 0, 39.5))
    parts.append(C._box(10.0, 2.3, 9.0, 0, 33.1 / 2 + 1.15, 18.0))  # 코일 커넥터
    return parts


def dcdc_12v_rsd60():
    """MEAN WELL RSD-60L-12 — 입력 18~72 V → 12 V 5 A, 절연, 철도용. 128 × 60 × 25 [제조사 사양 · 검색].
    고정 구멍 · 단자 자리는 [가정] (양 끝 M3, 단자는 -x 끝면)."""
    parts = [C._box(128.0, 60.0, 25.0)]
    for dy in (-15.0, -5.0, 5.0, 15.0):                              # 단자 (입력 2 · 출력 2) 선 15
        parts.append(Pos(-64.0, dy, 10.0) * Rot(0, -90, 0) * C.Cylinder(1.5, 15.0, align=C.MIN))
    with C.BuildPart() as h:
        add(parts[0])
        with C.Locations(*[(sx * 50.0, 0) for sx in (-1, 1)]):
            C.Cylinder(1.7, 25.0, align=C.MIN, mode=C.Mode.SUBTRACT)
    parts[0] = h.part
    return parts


def dcdc_5v_dkp55():
    """디오시스템 DKP-55 — 입력 8~60 V → 5 V 10 A, IP67. 64 × 57 × 22 [판매처]. 귀 · 구멍은 [가정]."""
    L, W, H = 54.0, 57.0, 22.0
    parts = [C._box(L, W, H)]
    for sx in (-1, 1):
        with C.BuildPart() as e:
            C.Box(5.0, 20.0, 3.0, align=C.MIN)
            with C.Locations((0, 0)):
                C.Cylinder(1.75, 3.0, align=C.MIN, mode=C.Mode.SUBTRACT)
        parts.append(Pos(sx * 29.5, 0, 0) * e.part)
    for dx in (-12, -4, 4, 12):                                      # 전선 4 가닥 (-y 면, 15 만)
        parts.append(Pos(dx, -W / 2, 8) * Rot(90, 0, 0) * C.Cylinder(1.5, 15.0, align=C.MIN))
    return parts


def iahrs_rb_sda():
    """ROBOR iAHRS RB-SDA-v1 — 35 × 35 × 10 (케이스 5 + 5), Ø2 관통 4 개 간격 29 × 29 [도면],
    한쪽 옆면에 마이크로 USB + RS-232 4 핀 [도면]. M2 스탠드오프 5 는 [가정]."""
    holes = [(sx * 14.5, sy * 14.5) for sx in (-1, 1) for sy in (-1, 1)]
    with C.BuildPart() as b:
        with C.Locations((0, 0, 5.0)):
            C.Box(35.0, 35.0, 10.0, align=C.MIN)
        with C.Locations(*holes):
            C.Cylinder(1.0, 15.0, align=C.MIN, mode=C.Mode.SUBTRACT)
    parts = [b.part] + [C._cyl(2.0, 5.0, x, y) for x, y in holes]
    parts.append(C._box(5.0, 8.0, 3.0, 17.5 + 2.5, -4.0, 5.0 + 3.0))   # USB 플러그 자리 (+x 옆)
    return parts


# ---------------------------------------------------------------- 배치 표
# key, label (버전 사이 같은 역할이면 같은 이름), 빌더, 자리 (x, y), z 회전, 무게 kg (None = 출처 없음), 출처
PARTS = [
    # v20: 전원 · 구동 계통 확정 (DECISIONS.md D-03~D-07). 같은 역할이면 label 앞부분을 이어 쓴다.
    ("battery", "배터리_36V_20Ah_ZENION", battery_zenion_20ah, (0.0, 0.0), 0, 3.0,
     "https://rmpower.co.kr/shop_view/?idx=514"),
    ("driver", "모터드라이버_앞_호버보드보드", hoverboard_mainboard, (287.5, 0.0), 90, 0.25 + 0.14,
     "EFeru README 핀아웃 사진 비례 (보드 0.25 kg 추정 + 방열판 3t 0.14 kg)"),
    ("driver", "모터드라이버_뒤_호버보드보드", hoverboard_mainboard, (-287.5, 0.0), -90, 0.25 + 0.14, "〃"),
    ("compute", "젯슨_오린나노_개발킷", jetson_orin_nano, (-95.0, 110.0), -90, 0.175,
     "NVIDIA SP-11324-001 v1.3"),
    ("compute", "ESP32_보드", esp32_devkitc, (-90.0, -92.0), 0, None,
     "https://dl.espressif.com/dl/schematics/esp32_devkitc_v4_dimensions.pdf"),
    ("power", "메인차단기_DC40A_DIN", dc_breaker_din, (162.0, 70.0), 0, 0.12,
     "https://mega-tec.co.kr/product/detail.html?product_no=504&cate_no=169&display_group=1"),
    ("power", "컨택터_LS_GPR-M040", contactor_gpr_m040, (158.0, -70.0), 0, 0.145,
     "https://e-punglim.co.kr/product/detail.html?product_no=16656"),
    ("power", "DCDC_36V-12V_젯슨_RSD-60L", dcdc_12v_rsd60, (75.0, 120.0), 0, None,
     "https://meanwell.co.kr/product/detail.html?product_no=799&cate_no=52&display_group=1"),
    ("power", "DCDC_36V-5V_DKP-55", dcdc_5v_dkp55, (-90.0, -150.0), 0, None,
     "https://smpskorea.com/product/detail.html?product_no=741&cate_no=126&display_group=1"),
    ("sensor", "IMU_iAHRS_RB-SDA", iahrs_rb_sda, (-35.0, -130.0), 0, None,
     "https://www.eleparts.co.kr/goods/view?no=9907195"),
]

GROUPS = {   # key: (그룹 이름, 색)
    "battery": ("배터리", "#2f4b7c"),
    "driver": ("모터드라이버", "#5f7f6a"),
    "compute": ("컴퓨터_젯슨_ESP32", "#2e7d32"),
    "power": ("전원_차단기_컨택터_DCDC", "#d9822b"),
    "sensor": ("센서_IMU", "#7b1fa2"),
}

CABLE_KEEP_R = 15.0          # 그로밋 중심에서 이 반경 안은 비운다 (케이블이 올라온다)
CABLE_KEEP_H = 40.0          # 판 위로 이 높이까지


def plate_holes():
    """판 좌표 고정 구멍 [(x, y, Ø, 볼트, 부품 이름)] — 부품 자리 · 회전을 따른다."""
    import math
    out = []
    for key, label, fn, (px, py), rz, _, _ in PARTS:
        c, s_ = math.cos(math.radians(rz)), math.sin(math.radians(rz))
        for x, y, d, bolt in MOUNT_HOLES.get(fn.__name__, []):
            out.append((px + x * c - y * s_, py + x * s_ + y * c, d, bolt, label))
    return out


def strap_slots():
    """배터리 밴드 슬롯 [(x, y, 길이 x, 폭 y)] — 배터리 양옆으로 2 줄."""
    bx, by = next(p[3] for p in PARTS if p[0] == "battery")
    half_w = BAT_W / 2
    return [(bx + x, by + sy * (half_w + 8.0), *STRAP_SLOT) for x in STRAP_X for sy in (-1, 1)]


def build(z0):
    """key → [부품 Compound]. 밑면 = z0 (판 윗면)."""
    out = {}
    for key, label, fn, (x, y), rz, _, _ in PARTS:
        c = C.label(fn(), label)
        c = Pos(x, y, z0) * Rot(0, 0, rz) * c
        c.label = label
        out.setdefault(key, []).append(c)
    return out
