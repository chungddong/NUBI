r"""바닥판 위 부품 — **데이터시트 · 판매처 공개 치수**로 짓는다 (v08).

치수 출처는 아래 각 항목 src. 2026-10-04 조사. 표기
  [도면]  제조사 도면 숫자          [도면 비례] 제조사 도면 래스터에서 비례로 잰 값 (±0.5)
  [판매처] 판매 페이지 사양표       [가정]  출처 없음 — 실물 받으면 고칠 값

배치 원칙 (v07 과 같다)
  · 배터리 가운데 (x=0 크로스 위) · 드라이버 앞뒤 끝 칸 · 전원은 배터리 앞 · 젯슨 · ESP32 는 뒤
  · 드라이버는 90° 돌려 **모터 단자 끝면이 좌 · 우 바퀴 쪽**을 본다 (그로밋 y=±154 와 가깝다)
  · 젯슨 I/O 변은 뒤 (-x) 를 본다 — 케이블 꽂을 자리
"""

from build123d import Pos, Rot

import components as C

# ---------------------------------------------------------------- 고정 구멍 (부품 로컬, 원점 = 밑면 가운데)
# 판에 뚫을 구멍 = 부품 고정 구멍. (x, y, 판 구멍 Ø, 볼트). v09 실외판에서 격자 대신 쓴다.
_OX, _OY = -50.0, -39.5                                              # 젯슨 도면 원점 → 가운데
JETSON_HOLES = [(_OX + x, _OY + y) for x, y in ((4, 17), (90, 17), (4, 75), (90, 75))]
ESP32_CARRIER_HOLES = [(sx * 31.0, sy * 21.0) for sx in (-1, 1) for sy in (-1, 1)]
MOUNT_HOLES = {
    "zlac8015d": [(sx * 70.75, sy * 32.75, 4.5, "M4") for sx in (-1, 1) for sy in (-1, 1)],
    "jetson_orin_nano": [(x, y, 3.4, "M3 스탠드오프") for x, y in JETSON_HOLES],
    "esp32_devkitc": [(x, y, 3.4, "M3 스탠드오프") for x, y in ESP32_CARRIER_HOLES],
    "midi_fuse_holder": [(20.485, 7.555, 5.5, "M5"), (-20.485, -7.555, 5.5, "M5")],
    "ssr_40dd": [(-24.0, 0.0, 4.5, "M4"), (24.0, 0.0, 4.5, "M4")],
    "dcdc_19v": [(0.0, sy * 42.0, 5.5, "M5 [가정]") for sy in (-1, 1)],
    "dcdc_5v": [(sx * 30.0, 0.0, 3.5, "M3 [가정]") for sx in (-1, 1)],
    "bno085": [(x - 12.7, y - 11.43, 2.7, "M2.5 스탠드오프")
               for x, y in ((2.54, 2.54), (22.86, 2.54), (2.54, 20.32), (22.86, 20.32))],
}
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


# ---------------------------------------------------------------- 배치 표
# key, label (버전 사이 같은 역할이면 같은 이름), 빌더, 자리 (x, y), z 회전, 무게 kg (None = 출처 없음), 출처
PARTS = [
    ("battery", "배터리_36V_20Ah_볼트온", battery_volton_20ah, (0.0, 0.0), 0, 3.2,
     "https://www.voltonbattery.com/pedelec_36v/?idx=88"),
    ("driver", "모터드라이버_앞_ZLAC8015D", zlac8015d, (287.5, 0.0), 90, 0.4,
     "https://www.robotmotor.com/zltech-24v-48v-30a-canbus-modbus-dual-channel-dc-driver-for-agv-product/"),
    ("driver", "모터드라이버_뒤_ZLAC8015D", zlac8015d, (-287.5, 0.0), 90, 0.4, "〃"),
    ("compute", "젯슨_오린나노_개발킷", jetson_orin_nano, (-95.0, 110.0), -90, 0.175,
     "NVIDIA SP-11324-001 v1.3"),
    ("compute", "ESP32_보드", esp32_devkitc, (-90.0, -92.0), 0, None,
     "https://dl.espressif.com/dl/schematics/esp32_devkitc_v4_dimensions.pdf"),
    ("power", "메인퓨즈_MIDI40A_58V", midi_fuse_holder, (150.0, -40.0), 0, 0.2,
     "https://www.victronenergy.com/upload/documents/Fuse-holder-for-MIDI-fuse.pdf"),
    ("power", "SSR_모터전원차단_40A", ssr_40dd, (150.0, 45.0), 0, None,
     "https://www.fotek.com.tw/en-gb/product-category/144"),
    ("power", "DCDC_36V-19V_젯슨", dcdc_19v, (100.0, 120.0), 0, 0.28,
     "https://www.amazon.com/dp/B0CDH538MG"),
    ("power", "DCDC_36V-5V", dcdc_5v, (-90.0, -150.0), 0, None,
     "https://www.droking.com/ (200656)"),
    ("sensor", "IMU_BNO085", bno085, (-40.0, -80.0), 0, 0.0025,
     "https://www.adafruit.com/product/4754"),
]

GROUPS = {   # key: (그룹 이름, 색)
    "battery": ("배터리", "#2f4b7c"),
    "driver": ("모터드라이버", "#5f7f6a"),
    "compute": ("컴퓨터_젯슨_ESP32", "#2e7d32"),
    "power": ("전원_퓨즈_SSR_DCDC", "#d9822b"),
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
    half_w = 105.0 / 2
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
