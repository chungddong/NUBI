# Ø81 바퀴 4륜 로버 베이스 — refmodel/Cramped Robot Wheel.step 을 그대로 쓴다.
#
# 이 파일의 목적은 "저 바퀴로 어디까지 되나"를 치수로 확정하는 것이다. 그래서 모든
# 값이 바퀴 실측치에서 역산된다. 축 높이는 바퀴 반지름(40.54)에 못 박혀 있고, 트랙은
# 좌우 모터가 차체 중앙에서 서로 부딪히지 않는 최소값에서 잡았다.
#
# 설계에서 실제로 발목을 잡은 것 두 가지 —
#   1) 좌우 모터가 같은 축선에 있어서 x 로 어긋나게 둘 수가 없다. 트랙 160 에서
#      쓸 수 있는 모터 길이는 58mm 가 한계다. 인코더 달린 25D(71mm)를 쓰려면
#      트랙을 184 로 벌려야 한다(전폭 222). MOTOR_L 을 고치면 아래에서 검증한다.
#   2) 바퀴 안쪽 디시(Ø54 x 26.7 깊이)를 안 쓰면 트랙이 30mm 더 벌어진다. 그래서
#      베어링 하우징을 바퀴 **안으로** 밀어 넣고 모터를 거기에 붙였다.

from pathlib import Path

from build123d import *

REF = Path(__file__).parent.parent / "refmodel"
EXPORT_DIR = Path(__file__).parent.parent / "exports"
EXPORT_NAME = "wheeled_base"

# ---- 바퀴 실측 (Cramped Robot Wheel.step 에서 직접 뽑은 값. 단위 mm) ----
W_OD      = 81.08     # 스파이크 끝 외경 (카커스는 Ø77.4)
W_WIDTH   = 38.35     # 타이어 전폭
W_FACE    = 19.17     # 타이어 안/바깥 면 (바퀴 중앙면 기준)
W_RIM_OD  = 60.0      # 림 외경 = 타이어 비드
W_HEX_AC  = 12.0      # 육각 소켓 대각. 맞변은 10.392 — RC 표준 12mm 헥스(맞변 12)와 다르다
W_HEX_IN  = 9.17      # 소켓 입구 (+x = 바깥쪽)
W_HEX_OUT = 13.17     # 소켓 바닥
W_BOSS_D  = 13.0      # 허브 보스 외경
W_SCREW   = 5.0       # 중심 볼트 통과공 (바깥면에 Ø11x1.5 카운터보어)
W_CAV_D   = 54.0      # 안쪽 디시 지름 = 림 배럴 내벽
W_CAV_IN  = -17.5     # 디시가 열린 면

# ---- 차량 제원 ----
TRACK      = 160      # 바퀴 중심 간
WHEELBASE  = 170
AXLE_Z     = W_OD / 2         # 40.54 — 무부하. 하중이 걸리면 스파이크가 눌려 ~39.5

# 프레임 — 사이드 레일은 모터 **위**를 지난다. 모터에 구멍을 뚫어 통과시키면 레일이
# 남아나질 않아서(Ø27 구멍에 위아래 1mm), 아예 모터 상단 위로 올렸다.
RAIL_Y, RAIL_T = 54, 6        # 레일 중심 y / 판 두께
RAIL_Z         = (56, 86)     # 레일 하단 ~ 상단
RAIL_X         = 120          # 레일 반길이
END_X, END_T   = 117, 6       # 앞뒤 엔드플레이트 중심 x / 두께
DECK           = (240, 190, 4)   # 상판 (길이, 폭, 두께) — 하단이 RAIL_Z[1]

# 구동 — 25D 급 기어모터를 바퀴 디시 안에 코를 박고 세운다.
MOTOR_D, MOTOR_L = 25.0, 58.0    # 몸체 지름 / 길이 (인코더 없는 사양)
MOTOR_SHAFT      = (4.0, 12.5)   # 출력축 Ø4 x 12.5 D컷
HOUS_D, HOUS_X   = 36.0, (-17.0, -7.0)   # 베어링 하우징 지름 / 코너 로컬 x 구간
BEAR             = (28.0, 12.0, 8.0)     # 6001ZZ (외경, 내경, 폭)
PLATE_T          = 6                     # 모터 캐리어 판 두께
PLATE_YZ         = (56, -13, 42)         # 판 폭(전후) / 하단 z / 상단 z — 전부 코너 로컬
LEG              = (18, 8, 14)           # 포크 다리 (전후 오프셋, 폭, 높이)
ADP_OD           = 13.0                  # 허브 어댑터 몸통 = 보스 외경과 동일

# ---- 탑재물 (데크 위. 전부 (길이x폭x높이, 중심 x, 중심 y) ) ----
BATT      = ((43, 137, 25), -96, 0)      # 3S 3300mAh — 뒤쪽에 가로로 눕힌다
DRIVER    = ((65, 45, 20), -20, 60)      # 모터 드라이버 2장 (좌우 대칭)
PI_AT     = (-15, 0)                     # RPi5 + NVMe + PoE HAT 중심
PI_STAND  = 4                            # 스탠드오프
PI_DETAIL = True                         # False 면 실측 외형 박스로 대체 (렌더가 빨라진다)
CAM_X     = 105                          # Xtion 센서바 중심 x
CAM_PAD   = (26, 186, 9)                 # 카메라 크래들 (길이, 폭, 높이)

CLR = {
    "타이어": "#232323", "림": "#2f6fd0", "레일": "#c8ccd0", "데크": "#aeb4ba",
    "캐리어": "#6b7480", "모터": "#8d9298", "어댑터": "#c9a227", "베어링": "#e3e7ea",
    "배터리": "#455a64", "컴퓨터": "#2e7d32", "드라이버": "#37474f",
    "카메라": "#14181d", "크래들": "#9aa0a6",
}

# ---- 유도 치수 ----
DECK_Z0 = RAIL_Z[1]                 # 데크 하단
DECK_Z1 = DECK_Z0 + DECK[2]         # 데크 상면 = 탑재물 바닥
RAIL_IN = RAIL_Y - RAIL_T / 2       # 레일 안쪽 면 y
PLATE_X = TRACK / 2 - RAIL_IN       # 캐리어 판 바깥 면까지의 코너 로컬 거리 (= 29)
CORNERS = [(sx * WHEELBASE / 2, sy) for sx in (1, -1) for sy in (1, -1)]

# ---- 참조 STEP ----
_wheel = import_step(str(REF / "Cramped Robot Wheel.step"))
W_TIRE, W_RIM = sorted(_wheel.solids(), key=lambda s: -s.volume)
_wb = _wheel.bounding_box()
assert abs(_wb.size.Y - W_OD) < 0.1 and abs(_wb.size.X - W_WIDTH) < 0.1, \
    f"바퀴 STEP 이 실측과 다르다 ({_wb.size})"

_xt = import_step(str(REF / "ASUS Xtion Pro Live.step"))
XT_BAR = max(_xt.solids(), key=lambda s: s.volume)      # 스탠드는 떼고 센서 바만
# STEP 좌표: X=바 길이, Y=높이, Z=깊이(렌즈가 +Z). 로봇은 X=전후 이므로 축을 한 바퀴 돌린다.
XT_ROT = Rot(0, 0, 90) * Rot(90, 0, 0)                  # X→Y, Y→Z, Z→X
_xb = (XT_ROT * XT_BAR).bounding_box()
assert abs(_xb.size.Y - 180) < 2 and abs(_xb.size.Z - 42) < 2, \
    f"Xtion 방향이 틀렸다 ({_xb.size})"
XT_SIZE = (_xb.size.X, _xb.size.Y, _xb.size.Z)


def seat(shape, cx, cy, z0, rot=None):
    """STEP 부품의 XY 중심을 (cx, cy) 에, 바닥을 z0 에 맞춰 앉힌다."""
    s = (rot * shape) if rot else shape
    b = s.bounding_box()
    return Pos(cx - b.center().X, cy - b.center().Y, z0 - b.min.Z) * s


def place(shape, cx, sy):
    """코너 로컬 좌표(+X=차 바깥, +Z=위)를 로봇 좌표로 옮긴다. sy=+1 좌 / -1 우."""
    return Pos(cx, sy * TRACK / 2, AXLE_Z) * Rot(0, 0, 90 * sy) * shape


# ---- 프레임 ----
def frame():
    """사이드 레일 2 + 앞뒤 엔드플레이트 2. 레일은 모터 위를 지나간다."""
    out = []
    for i, sy in enumerate((1, -1)):
        with BuildPart() as p:
            with Locations((0, sy * RAIL_Y, sum(RAIL_Z) / 2)):
                Box(2 * RAIL_X, RAIL_T, RAIL_Z[1] - RAIL_Z[0])
            # 배선을 아래로 떨구는 구멍. 레일 중립축이라 강성 손해가 거의 없다.
            with Locations(*[(x, sy * RAIL_Y, sum(RAIL_Z) / 2) for x in (-45, 0, 45)]):
                Cylinder(8, RAIL_T + 2, rotation=(90, 0, 0), mode=Mode.SUBTRACT)
        p.part.label = f"레일{i + 1}"
        out.append(p.part)
    for i, sx in enumerate((1, -1)):
        with BuildPart() as p:
            with Locations((sx * END_X, 0, sum(RAIL_Z) / 2)):
                Box(END_T, 2 * RAIL_IN, RAIL_Z[1] - RAIL_Z[0])
        p.part.label = f"엔드플레이트{i + 1}"
        out.append(p.part)
    return out


def deck():
    """상판 — 탑재물은 전부 여기 위에 올라간다. 바퀴 꼭대기를 넘겨 전폭을 다 쓴다."""
    lx, ly, t = DECK
    with BuildPart() as p:
        with Locations((0, 0, DECK_Z0 + t / 2)):
            Box(lx, ly, t)
        fillet(p.edges().filter_by(Axis.Z), radius=10)
        # 레일·엔드플레이트 체결 M4 + 배선 통과 슬롯
        holes = [(x, sy * RAIL_Y) for x in (-100, -50, 0, 50, 100) for sy in (1, -1)]
        with Locations(*[(x, y, DECK_Z0 + t / 2) for x, y in holes]):
            Cylinder(2.2, t + 2, mode=Mode.SUBTRACT)
        with Locations((-55, 0, DECK_Z0 + t / 2), (35, 0, DECK_Z0 + t / 2)):
            Box(14, 60, t + 2, mode=Mode.SUBTRACT)
    p.part.label = "데크"
    return p.part


# ---- 구동 (코너 로컬 좌표에서 만든 뒤 4곳에 배치) ----
def carrier():
    """모터 캐리어 판 + 포크 다리 + 베어링 하우징. 한 덩어리로 프린트한다.

    포크로 만든 이유: 하우징이 바퀴 디시 **안**에 있어서, 연결부가 회전하는 림
    내벽(Ø54)을 통과해야 한다. 축을 비켜 지나가려면 다리 두 개로 갈라야 한다.
    """
    py, pz0, pz1 = PLATE_YZ
    ly, lw, lh = LEG
    with BuildPart() as p:
        with Locations((-(PLATE_X + PLATE_T / 2), 0, (pz0 + pz1) / 2)):
            Box(PLATE_T, py, pz1 - pz0)
        fillet(p.edges().filter_by(Axis.X), radius=8)
        with Locations(*[(-(PLATE_X + PLATE_T / 2), sy * 22, z)
                         for sy in (1, -1) for z in (24, 36)]):   # 레일 체결 M4
            Cylinder(2.2, PLATE_T + 2, rotation=(0, 90, 0), mode=Mode.SUBTRACT)
        for sy in (1, -1):                                        # 포크 다리
            with Locations(((-PLATE_X + HOUS_X[0]) / 2, sy * ly, 0)):
                Box(PLATE_X + HOUS_X[0], lw, lh)
        with Locations(((HOUS_X[0] + HOUS_X[1]) / 2, 0, 0)):      # 베어링 하우징
            Cylinder(HOUS_D / 2, HOUS_X[1] - HOUS_X[0], rotation=(0, 90, 0))
        with Locations((HOUS_X[0] + 1 + (HOUS_X[1] - HOUS_X[0] - 1) / 2, 0, 0)):
            Cylinder(BEAR[0] / 2, HOUS_X[1] - HOUS_X[0] - 1,
                     rotation=(0, 90, 0), mode=Mode.SUBTRACT)     # 베어링 자리
        with Locations((HOUS_X[0] + 0.5, 0, 0)):                  # 모터 코 파일럿
            Cylinder(MOTOR_D / 2 + 0.25, 1, rotation=(0, 90, 0), mode=Mode.SUBTRACT)
        with Locations(*[(HOUS_X[0] + 4, 0, sz * 13) for sz in (1, -1)]):
            Cylinder(1.6, 9, rotation=(0, 90, 0), mode=Mode.SUBTRACT)   # 모터 M3
        with Locations((-PLATE_X - PLATE_T / 2, 0, 0)):
            Cylinder(MOTOR_D / 2 + 0.25, PLATE_T + 2,
                     rotation=(0, 90, 0), mode=Mode.SUBTRACT)     # 몸통 2차 지지
    p.part.label = "캐리어"
    return p.part


def motor():
    """25D 급 기어모터 목업 — 코가 하우징 안쪽 면(x=-17)에 붙는다."""
    with BuildPart() as p:
        with Locations((HOUS_X[0] - MOTOR_L / 2, 0, 0)):
            Cylinder(MOTOR_D / 2, MOTOR_L, rotation=(0, 90, 0))
        with Locations((HOUS_X[0] + MOTOR_SHAFT[1] / 2, 0, 0)):
            Cylinder(MOTOR_SHAFT[0] / 2, MOTOR_SHAFT[1], rotation=(0, 90, 0))
    p.part.label = "모터"
    return p.part


def bearing():
    with BuildPart() as p:
        with Locations((HOUS_X[0] + 2 + BEAR[2] / 2, 0, 0)):
            Cylinder(BEAR[0] / 2, BEAR[2], rotation=(0, 90, 0))
            Cylinder(BEAR[1] / 2, BEAR[2] + 2, rotation=(0, 90, 0), mode=Mode.SUBTRACT)
    p.part.label = "베어링"
    return p.part


def adapter():
    """허브 어댑터 — 이 플랫폼에서 유일하게 '없으면 못 만드는' 부품.

    바퀴 육각이 맞변 10.392(대각 12)라 시판 RC 12mm 헥스가 안 들어간다. 안쪽은
    모터축 Ø4 를 물고, 가운데는 Ø12 저널로 베어링에 앉고, 바깥 4mm 만 육각이다.
    바퀴는 바깥에서 M5 한 개로 이 어댑터에 눌러 붙인다.
    """
    x0 = HOUS_X[0] + 1                       # 모터 면에서 1mm 띄운다
    with BuildPart() as p:
        with Locations(((x0 + HOUS_X[1]) / 2, 0, 0)):
            Cylinder(BEAR[1] / 2, HOUS_X[1] - x0, rotation=(0, 90, 0))   # 베어링 저널
        with Locations(((HOUS_X[1] + W_HEX_IN) / 2, 0, 0)):
            Cylinder(ADP_OD / 2, W_HEX_IN - HOUS_X[1], rotation=(0, 90, 0))
        with BuildSketch(Plane.YZ.offset(W_HEX_IN)):                     # 육각 물림
            RegularPolygon(W_HEX_AC / 2 - 0.15, 6)
        extrude(amount=W_HEX_OUT - W_HEX_IN)
        with Locations((x0 + MOTOR_SHAFT[1] / 2, 0, 0)):                 # 모터축 보어
            Cylinder(MOTOR_SHAFT[0] / 2 + 0.1, MOTOR_SHAFT[1] + 0.5,
                     rotation=(0, 90, 0), mode=Mode.SUBTRACT)
        with Locations((W_HEX_OUT - 4.5, 0, 0)):                         # 바퀴 고정 M5
            Cylinder(4.2 / 2, 9, rotation=(0, 90, 0), mode=Mode.SUBTRACT)
        with Locations((x0 + 5, 0, 0)):                                  # 축 무두볼트 M3
            Cylinder(1.3, ADP_OD, rotation=(0, 0, 0), mode=Mode.SUBTRACT)
    p.part.label = "어댑터"
    return p.part


# ---- 탑재물 ----
def payload():
    out = []
    (bx, by, bz), cx, cy = BATT
    with BuildPart() as p:
        with Locations((cx, cy, DECK_Z1 + bz / 2)):
            Box(bx, by, bz)
    p.part.label = "배터리"
    out.append((p.part, "배터리"))

    (dx, dy, dz), dcx, dcy = DRIVER
    for i, sy in enumerate((1, -1)):
        with BuildPart() as p:
            with Locations((dcx, sy * dcy, DECK_Z1 + dz / 2)):
                Box(dx, dy, dz)
        p.part.label = f"드라이버{i + 1}"
        out.append((p.part, "드라이버"))

    if PI_DETAIL:
        pi = import_step(str(REF / "RPI5+Pimoroni-NVME+Waveshare-POE v1.step"))
        pi = seat(pi, PI_AT[0], PI_AT[1], DECK_Z1 + PI_STAND, rot=Rot(0, 0, 90))
        pi.label = "컴퓨터"
        out.append((pi, "컴퓨터"))
    else:
        with BuildPart() as p:                       # 실측 외형 90 x 58.1 x 36.5
            with Locations((PI_AT[0], PI_AT[1], DECK_Z1 + PI_STAND + 36.48 / 2)):
                Box(90, 58.14, 36.48)
        p.part.label = "컴퓨터"
        out.append((p.part, "컴퓨터"))
    return out


def camera():
    """Xtion 센서바 + 크래들. 렌즈가 +X(전방) 를 본다."""
    cl, cw, ch = CAM_PAD
    with BuildPart() as p:
        with Locations((CAM_X, 0, DECK_Z1 + ch / 2)):
            Box(cl, cw, ch)
        fillet(p.edges().filter_by(Axis.Z), radius=6)
        with Locations(*[(CAM_X, sy * 80, DECK_Z1 + ch / 2) for sy in (1, -1)]):
            Cylinder(2.2, ch + 2, mode=Mode.SUBTRACT)
    p.part.label = "크래들"
    cam = seat(XT_BAR, CAM_X, 0, DECK_Z1 + ch, rot=XT_ROT)
    cam.label = "카메라"
    return p.part, cam


# ---- 조립 ----
items = []          # (형상, 색키, 테셀레이션 tol)

for part in frame():
    items.append((part, "레일", 0.4))
items.append((deck(), "데크", 0.4))

_carrier, _motor, _bearing, _adapter = carrier(), motor(), bearing(), adapter()
for cx, sy in CORNERS:
    items.append((place(W_TIRE, cx, sy), "타이어", 0.8))
    items.append((place(W_RIM, cx, sy), "림", 0.6))
    items.append((place(_carrier, cx, sy), "캐리어", 0.4))
    items.append((place(_motor, cx, sy), "모터", 0.4))
    items.append((place(_bearing, cx, sy), "베어링", 0.3))
    items.append((place(_adapter, cx, sy), "어댑터", 0.25))

for part, key in payload():
    items.append((part, key, 1.0 if key == "컴퓨터" and PI_DETAIL else 0.4))
_cradle, _cam = camera()
items.append((_cradle, "크래들", 0.4))
items.append((_cam, "카메라", 0.6))

GROUPS = {}
for shape, key, _ in items:
    shape.color = Color(CLR[key])
    GROUPS.setdefault(key, []).append(shape)
assembly = Compound(children=[Compound(children=v, label=k) for k, v in GROUPS.items()])
assembly.label = EXPORT_NAME

# ---- 제원 / 간섭 검증 ----
bb = assembly.bounding_box()
mot_gap = TRACK - 2 * (-HOUS_X[0] + MOTOR_L)                 # 좌우 모터 사이
rail_gap = (TRACK / 2 - W_FACE) - (RAIL_Y + RAIL_T / 2)      # 바퀴 안쪽 면 ~ 레일 바깥 면
deck_gap = DECK_Z0 - W_OD                                    # 바퀴 꼭대기 ~ 데크 하단
mot_top = AXLE_Z + MOTOR_D / 2
leg_gap = W_CAV_D / 2 - ((LEG[0] + LEG[1] / 2) ** 2 + (LEG[2] / 2) ** 2) ** 0.5
low = min(AXLE_Z - MOTOR_D / 2, AXLE_Z + PLATE_YZ[1])        # 최저 구조물

print(f"전장 {bb.size.X:.0f} x 전폭 {bb.size.Y:.0f} x 전고 {bb.size.Z:.0f} mm")
print(f"  축간 {WHEELBASE} / 윤거 {TRACK} / 바퀴 Ø{W_OD:.1f} x {W_WIDTH:.1f}, 축 z={AXLE_Z:.2f}")
print(f"  데크 {DECK[0]}x{DECK[1]} 상면 z={DECK_Z1} / 최저 구조물 z={low:.1f} (= 지상고)")
print(f"  1회전 이동 {3.14159 * (W_OD - 2):.0f}mm (유효 Ø{W_OD - 2:.1f}, 스파이크 눌림 반영)")
print("검증:")
print(f"  좌우 모터 간격      {mot_gap:6.1f} mm  (모터 {MOTOR_L:.0f}mm, 면 x={HOUS_X[0]:.0f})")
print(f"  바퀴 ~ 사이드레일   {rail_gap:6.2f} mm")
print(f"  바퀴 꼭대기 ~ 데크  {deck_gap:6.2f} mm")
print(f"  모터 꼭대기 ~ 레일  {RAIL_Z[0] - mot_top:6.2f} mm")
print(f"  포크 다리 ~ 림 내벽 {leg_gap:6.2f} mm  (Ø{W_CAV_D:.0f} 디시를 통과한다)")
print(f"  육각 물림           {W_HEX_OUT - W_HEX_IN:6.1f} mm  (대각 Ø{W_HEX_AC}, 맞변 10.39)")
for name, v in [("좌우 모터", mot_gap), ("바퀴/레일", rail_gap), ("바퀴/데크", deck_gap),
                ("모터/레일", RAIL_Z[0] - mot_top), ("다리/림", leg_gap)]:
    assert v > 1.5, f"{name} 간섭 또는 여유 부족: {v:.2f}mm"

_max_motor = TRACK / 2 + HOUS_X[0] - 4
print(f"  → 이 트랙에서 쓸 수 있는 모터 최대 길이 {_max_motor:.0f}mm. "
      f"인코더형 25D(71mm)를 쓰려면 트랙 {2 * (71 + 4 - HOUS_X[0]):.0f} (전폭 "
      f"{2 * (71 + 4 - HOUS_X[0]) + W_WIDTH:.0f})")

# 질량·무게중심 추정 — [추정] 값이다. 실측이 생기면 여기만 고친다.
MASS = [("바퀴", 4 * 60, AXLE_Z), ("모터", 4 * 90, AXLE_Z), ("캐리어", 4 * 35, AXLE_Z),
        ("어댑터+베어링", 4 * 30, AXLE_Z), ("레일+엔드", 170, 71), ("데크", 250, 88),
        ("배터리", 280, DECK_Z1 + 13), ("컴퓨터", 120, DECK_Z1 + 22),
        ("드라이버", 80, DECK_Z1 + 10), ("카메라+크래들", 260, DECK_Z1 + 25),
        ("배선·볼트", 150, 80)]
m = sum(g for _, g, _ in MASS)
cg = sum(g * z for _, g, z in MASS) / m
import math
print(f"질량 {m / 1000:.2f} kg (휠당 {m / 4000:.2f} kg) / 무게중심 z≈{cg:.0f}mm "
      f"→ 정적 전복각 {math.degrees(math.atan2(TRACK / 2, cg)):.0f}°")
print(f"  림 배럴이 1mm 벽인 프린트 파트라 휠당 1.5kg 를 상시 한계로 본다 "
      f"→ 적재 여유 약 {(1.5 * 4 - m / 1000):.1f} kg")

VIEWER = False          # True 로 바꾸면 ocp_vscode(3939) 로 쏜다. 기본은 PNG 렌더만.
if VIEWER:
    try:
        from ocp_vscode import Camera, set_port, show
        set_port(3939)
        show(*assembly.children, names=[c.label for c in assembly.children],
             progress=None, reset_camera=Camera.RESET)
    except Exception as e:
        print(f"뷰어에 표시하지 못함 (뷰어 미실행?): {e}")

# ---- 내보내기 + 오프라인 렌더 ----
EXPORT_DIR.mkdir(exist_ok=True)
export_step(assembly, str(EXPORT_DIR / f"{EXPORT_NAME}.step"))
export_step(_adapter, str(EXPORT_DIR / f"{EXPORT_NAME}_adapter.step"))
export_stl(_adapter, str(EXPORT_DIR / f"{EXPORT_NAME}_adapter.stl"))
print(f"내보내기: {EXPORT_DIR / EXPORT_NAME}.step / _adapter.step / _adapter.stl")

from render import DIR_FRONT, DIR_ISO, DIR_TOP, render, tile

RITEMS = [(s, CLR[k], t) for s, k, t in items]
views = [("iso", DIR_ISO), ("top", DIR_TOP), ("side", (0.02, -1.0, 0.05))]
paths = []
for name, d in views:
    p = EXPORT_DIR / f"{EXPORT_NAME}_{name}.png"
    render(RITEMS, str(p), direction=d, size=760, ss=2)
    paths.append(str(p))
    print(f"  렌더: {p}")
tile(paths, str(EXPORT_DIR / f"{EXPORT_NAME}.png"), cols=3,
     labels=[f"ISO  {bb.size.X:.0f}x{bb.size.Y:.0f}x{bb.size.Z:.0f}",
             f"TOP  축간{WHEELBASE} 윤거{TRACK}", f"SIDE 축 z={AXLE_Z:.1f} 지상고 {low:.0f}"])
print(f"  렌더: {EXPORT_DIR / EXPORT_NAME}.png")
