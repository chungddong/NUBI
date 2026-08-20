# 실내외 테스트용 모빌리티 베이스 (4륜 스키드 스티어) — 2층 프레임 + 측면 커버
#
# 1층: 하부 프레임 = 바퀴/액슬 장착 + 전장 베이 바닥
# 2층: 상부 프레임 = 데크 받침. 바퀴 꼭대기보다 높게 올려 적재면이 바퀴에 안 걸린다.
# 상부 프레임은 하부와 같은 700x500 이라 측면 커버(740x550) 안쪽에 완전히 숨는다.
# 밖에서 보이는 건 데크 모서리와 커버, 하부 프레임뿐이다.
# 전장 베이는 바닥판부터 데크까지 라운드 코너 절곡 커버로 완전히 감싼다.
# 커버는 앞/뒤 두 짝으로 나뉜다 — 후면이 통판이어야 비상정지와 커넥터를 가운데 놓을 수 있다.
# 뒤판 위쪽에 22mm 비상정지, 그 아래에 커넥터 묶음 + 이를 덮는 서비스 도어가 들어간다.
#
# 주의 1: 리지드 4륜이라 울퉁불퉁한 실외에서는 대각 2륜만 닿는 순간이 생긴다.
#         나중에 앞축 크로스멤버를 로커(중앙 피벗)로 바꾸면 4륜 접지가 유지된다.
# 주의 2: 베이가 사방으로 막혀 있으니 모터 드라이버 발열을 확인해라. 필요하면
#         커버 앞뒤 면에 슬롯 벤트를 뚫거나 데크 밑에 팬을 달면 된다.
# 주의 3: 구동계(모터·감속·전달)는 아직 미정이다. 지금은 바퀴가 자체 베어링으로
#         데드 액슬 위에서 도는 상태까지만 모델링돼 있다.
# 주의 4: 전장 목업은 공간 확인용이고 STEP/STL 에는 안 들어간다.

from pathlib import Path

from build123d import *
from ocp_vscode import Camera, show

from tslot import ALU_DENSITY, PROFILES, bar, cut_list

# ---- 파라미터 (단위: mm, 지면 z=0) ----
SERIES = 30                   # 프로파일 시리즈
BASE_L, BASE_W = 630, 450     # 하부 프레임 외곽 (앞뒤 X, 좌우 Y) — 700x500 에서 0.90 축소
BAY_H = 140                   # 전장 베이 내부 높이 (바닥판 위 ~ 상부 프레임 아래)

DECK_LIP = 5                  # 데크가 측면커버보다 사방으로 더 나오는 양 (그림자 틈)

# 바퀴 — 8인치 PVC 통바퀴 (볼베어링 내장, 허용중량 70kg/개)
WHEEL_D, WHEEL_W = 210, 58    # 바퀴 지름 / 폭
WHEEL_BORE = 20               # 휠 내경 = 축 지름
BEARING_OD, BEARING_W = 47, 14            # 6204 (20x47x14) 2개가 양쪽에 들어간다고 가정
HUB_BOSS_D = 70               # 휠 중앙 보스 지름 (구동 플랜지가 물릴 자리)
AXLE_D = WHEEL_BORE           # 축 지름 20 — 바퀴 자체 베어링으로 도는 데드 액슬
AXLE_FIT, AXLE_NUT = 0.2, 12  # 마운트 판 구멍 여유 / 판 안쪽 너트 자리
WHEELBASE = 400               # 앞뒤 축간거리 — 짧을수록 스키드 제자리회전 토크가 정비례로 준다
AXLE_GAP = 20                 # 마운트 판 바깥면 ~ 바퀴 안쪽면 (커버가 지나갈 틈)
WHEEL_SET_KG = 1.6            # PVC 통바퀴 1개 추정 (모터는 별도 집계)

RAIL_Z = 140                  # 하부 사이드 레일 중심 높이 → 프레임 지상고 125
PLATE_L, PLATE_H, PLATE_T = 200, 80, 8    # 모터 마운트 판 (X, Z, 두께)
PLATE_Z0 = 75                 # 판 아랫면 높이
BOLT_D, BOLT_XS = 6.6, (25, 55, 85)       # M6 통과 구멍 / 볼트 X 위치(±)

PANEL_T = 3                   # 베이 바닥판 / 데크 두께 (알루미늄)
DECK_HOLE, DECK_PITCH = 6.6, 100          # 데크 장비 장착 구멍 (M6, 100 격자)
BAY_HOLE, BAY_PITCH = 5.5, 50             # 베이 바닥판 장착 구멍 (M5, 50 격자)

SKIRT_X, SKIRT_Y = 335, 245   # 커버 바깥면 반치수 (X, Y) — 바퀴 안쪽면까지 8mm 여유
SKIRT_R, SKIRT_T = 50, 2      # 커버 코너 R / 판 두께
SKIRT_FLANGE, SKIRT_SEAM = 16, 2          # 상하 절곡 플랜지 폭 / 앞뒤 분할 틈

# 후면 패널 — 위에 비상정지, 아래에 커넥터. 분할선이 x=0 이라 후면은 통판이다.
ESTOP_HOLE, ESTOP_KEY = 22.3, (3.5, 1.6)  # 22mm 표준 취부 구멍 / 회전방지 홈
ESTOP_Z = 272                             # 버튼 중심 높이 (상부 크로스멤버를 피한 값)
ESTOP_HEAD, ESTOP_HEAD_H = 40, 20         # 버섯 머리
ESTOP_COLLAR, ESTOP_COLLAR_H = 32, 4      # 패널에 닿는 칼라
ESTOP_BODY, ESTOP_BODY_L = 28, 45         # 패널 뒤 몸체 + 접점 블록

PORT_DOOR = (170, 80)                     # 포트 도어 크기 (Y, Z)
PORT_DOOR_Z, PORT_DOOR_T, PORT_DOOR_R = 200, 2, 12
PORT_SCREW, DOOR_INSET = 4.3, 13          # 도어 고정 M4 / 모서리에서 들어간 거리
PORTS = [
    # (이름, 모양, 치수, (y, z))  — 2mm 판이라 패널마운트 커넥터가 그대로 물린다
    ("충전 커넥터", "round", 16.0, (-45, 218)),
    ("메인 전원 스위치", "round", 19.0, (0, 218)),
    ("상태 표시등", "round", 8.0, (45, 218)),
    ("이더넷 RJ45", "rect", (19, 15), (-40, 182)),
    ("USB-A", "rect", (14, 12), (5, 182)),
    ("예비 그로밋", "round", 12.0, (48, 182)),
]

# 전면 패널 — 가운데 원형 카메라 하나, 그 아래 1자 LED 바.
# 눈 하나에 입 하나로 읽히게 세로 간격을 좁게(31mm) 잡았다.
CAM_Z = 262                               # 카메라 중심 높이 (지면에서 262mm)
CAM_HOLE, CAM_PCD, CAM_SCREW = 32, 60, 3.4    # 렌즈 통과공 / 나사원 지름 / M3
CAM_BEZEL, CAM_BEZEL_T = 76, 6            # 바깥 베젤 링 — 크게 잡아야 눈처럼 보인다
CAM_LENS, CAM_LENS_L = 30, 16             # 렌즈 배럴
CAM_BODY = (26, 40, 40)                   # 커버 안쪽 카메라 몸체 (X, Y, Z)

LED_Z = 196                               # LED 바 중심 높이
LED_SLOT = (290, 16)                      # 커버에 뚫는 슬롯 (Y, Z) — 양 끝 반원
LED_BACK = (314, 34, 3)                   # 디퓨저 안쪽 플랜지 (Y, Z, 두께)
LED_FIT = 0.6                             # 슬롯과 디퓨저 사이 틈

# 환기 — 앞면 하단 슬롯으로 들어와 베이를 지나 후면 팬으로 나간다.
# 측면(바퀴 옆)이 아니라 앞면에 둔 이유: 바퀴가 튀기는 물·흙을 정면으로 덜 맞는다.
VENT_SLOT, VENT_Z = (60, 6), 172          # 슬롯 하나 (Y, Z) / 중심 높이
VENT_YS = (-145, -72, 0, 72, 145)         # LED 바 아래 한 줄
FAN_SIZE, FAN_T = 80, 25                  # 80mm 팬 몸체
FAN_HOLE, FAN_PCD, FAN_SCREW = 76, 71.5, 4.5
FAN_Y, FAN_Z = 150, 200                   # 후면 팬 중심 — 포트 도어 양옆

# 배선 관통부 (고무 그로밋)
GROM_FLOOR, GROM_DECK = 20, 25            # 바닥판 / 데크 관통 구멍 지름
GROM_FLOOR_PTS = [(sx * 150, sy * 150) for sx in (-1, 1) for sy in (-1, 1)]  # 바퀴 4개분
# ↑ 축 크로스멤버(x=±AXLE_X) 위를 피해야 케이블이 지나갈 수 있다
GROM_DECK_PTS = [(sx * 250, 150) for sx in (-1, 1)]    # 데크 위 장비 배선 (라이다 자리를 피함)

# 배터리 브라켓 — 앵글 4개로 미끄러짐을, 스트랩 2개로 들림을 잡는다.
# 볼트 자리를 전부 바닥판 50mm 격자 위에 올려서 구멍을 새로 뚫지 않는다.
BATT = (260, 170, 90)                     # 배터리 (X, Y, Z) — 목업과 공유
BATT_GAP, ANGLE_L, ANGLE_T = 1, 25, 3     # 배터리와 브라켓 틈 / 앵글 다리 / 두께
STRAP_W, STRAP_T, STRAP_FOOT = 25, 3, 25  # 스트랩 폭 / 두께 / 발 길이
BATT_BOLT = 5.5                           # M5
BATT_SIDE_BOLT = (100, 100)               # 측면 앵글 볼트 (|x|, |y|)
BATT_END_BOLT = (150, 50)                 # 단부 앵글 볼트 (|x|, |y|)
BATT_STRAP_BOLT = (50, 150)               # 스트랩 발 볼트 (|x|, |y|)

# 하부 커버 — 두 가지 스타일 중 택일
#   "skirt"  얇은 탈착식 스커트. 바퀴 사이만 가리고 바퀴는 그대로 노출된다.
#   "fender" 바퀴 위·바깥까지 감싸는 광폭 셸. 바퀴는 아래 50mm 만 보인다.
LOWER_COVER = "fender"        # "skirt" | "fender" | None
SKIRT2_BOT, SKIRT2_T = 50, 3  # 하단 높이(=이때 지상고) / 두께
SKIRT2_TAB, ARCH_CLR = 15, 10 # 상단 고정 탭 길이 / 휠 아치 여유
FENDER_CLR, FENDER_OVER = 8, 7    # 바퀴 바깥·위 여유

# 데크 모듈 레일 — 2020 프로파일을 데크 위에 얹어 적재함·로봇팔 베이스를 밀어 끼운다.
DECK_RAIL = True
RAIL_SERIES, RAIL_L = 20, 660         # 2020, 길이 (데크 680 안쪽)
RAIL_YS = (-200, 0, 200)              # 데크 100mm 격자 위에 정확히 얹힌다
RAIL_STOP = (20, 20, 25)              # 앞쪽 엔드 스톱 (X, Y, Z) — 밀어 넣으면 같은 자리에 선다

MOCKUP = True                 # 전장 목업(배터리/드라이버) 표시 여부
REAL_PARTS = False             # refmodel/ 의 실물 STEP 을 불러올지 (로딩 20초쯤 걸린다)
REF_PARTS = [
    # (파일명, 베이 안 위치 (x, y), Z축 회전)
    ("RPI5+Pimoroni-NVME+Waveshare-POE v1.step", (250, 150), 0),
]

CLR = {
    "프레임": "#c8ccd0",       # 알루미늄 아노다이징
    "액슬마운트": "#4a5058",
    "축·베어링": "#b6bbc0",
    "바퀴": "#2b2b2b",
    "판재": "#9aa0a6",
    "측면커버": "#39414a",     # 흑회색 도장 절곡판
    "비상정지": "#c62828",     # 적색 버섯머리 + 황색 칼라 대신 단색
    "후면패널": "#2c333a",     # 포트 도어 / 버튼 몸체
    "카메라": "#14181d",       # 베젤 + 렌즈 — 어둡게 해서 눈처럼 보이게
    "LED바": "#f2f4f0",        # 백색 디퓨저
    "배터리브라켓": "#8d9298",
    "사이드스커트": "#2f363d",  # 커버보다 약간 어둡게
    "데크레일": "#c8ccd0",
    "그로밋": "#1b1b1b",       # 고무
    "목업": "#2f7d5c",
    "실물부품": "#1f6f8b",     # refmodel 에서 불러온 STEP
}

REF_DIR = Path(__file__).parent.parent / "refmodel"
EXPORT_DIR = Path(__file__).parent.parent / "exports"
EXPORT_NAME = "mobility_base"

# ---- 유도 치수 ----
S = PROFILES[SERIES]["size"]
RAIL_Y = (BASE_W - S) / 2                 # 하부 사이드 레일 중심 Y (±)
CROSS_X = (BASE_L - S) / 2                # 앞뒤 크로스멤버 중심 X (±)
POST_X = (-CROSS_X, 0.0, CROSS_X)         # 기둥/상부 크로스멤버 X 위치 (앞·중앙·뒤)
AXLE_X = WHEELBASE / 2                    # 축 위치 X (±)
AXLE_Z = WHEEL_D / 2                      # 축 높이 = 바퀴 반지름
PLATE_Y = BASE_W / 2                      # 판 안쪽면 = 사이드 레일 바깥면
WHEEL_Y = PLATE_Y + PLATE_T + AXLE_GAP + WHEEL_W / 2      # 바퀴 중심 Y (±)
CLEARANCE = RAIL_Z - S / 2                # 프레임 지상고

FRAME_TOP = RAIL_Z + S / 2                # 하부 프레임 윗면 = 베이 바닥판이 얹히는 면
POST_H = PANEL_T + BAY_H                  # 기둥 길이 (바닥판 두께 + 베이 높이)
TOP_RAIL_Z = FRAME_TOP + POST_H + S / 2   # 상부 프레임 중심 높이
DECK_Z = TOP_RAIL_Z + S / 2               # 데크 아랫면 = 상부 프레임 윗면
BAY_Z = FRAME_TOP + PANEL_T               # 베이 바닥 (장비가 앉는 면)
SKIRT_TOP = DECK_Z                        # 커버 윗단 = 데크 아랫면 (베이를 완전히 막는다)

# 상부 프레임은 하부와 같은 700x500 — 커버(740x550) 안쪽으로 들어가 밖에서 안 보인다
TOP_RAIL_Y, TOP_RAIL_L = RAIL_Y, BASE_L

# 데크는 커버보다 사방으로 DECK_LIP 만큼만 크다. 커버와 코너 중심이 같아져
# 그림자 틈이 직선부·코너 할 것 없이 정확히 DECK_LIP 으로 일정해진다.
DECK_L = 2 * SKIRT_X + 2 * DECK_LIP
DECK_W = 2 * SKIRT_Y + 2 * DECK_LIP
DECK_R = SKIRT_R + DECK_LIP

# 커버 안쪽면 / 플랜지 볼트 열 — 데크·바닥판에 같은 좌표로 구멍을 뚫어 맞춘다
SKIRT_IN_X, SKIRT_IN_Y = SKIRT_X - SKIRT_T, SKIRT_Y - SKIRT_T
BOLT_LINE_X = SKIRT_IN_X - SKIRT_FLANGE / 2
BOLT_LINE_Y = SKIRT_IN_Y - SKIRT_FLANGE / 2
SKIRT_BOLTS = ([(sx * BOLT_LINE_X, y) for y in (-150, 0, 150) for sx in (-1, 1)]
               + [(x, sy * BOLT_LINE_Y) for x in (-250, -125, 125, 250) for sy in (-1, 1)]
               # ↓ 바퀴 사이 구간. 사이드 스커트의 옆구리 조각이 여기에 매달린다
               + [(x, sy * BOLT_LINE_Y) for x in (-50, 50) for sy in (-1, 1)])
POST_PTS = [(x, sy * RAIL_Y) for x in POST_X for sy in (-1, 1)]   # 기둥 6본 중심
POST_CUT = S + 3                          # 기둥이 지나갈 구멍 (베이 바닥판)
FRAME_CLR = 1.5                           # 커버 플랜지와 프레임 사이 여유

assert DECK_Z > WHEEL_D, "데크가 바퀴 꼭대기보다 낮다 — BAY_H 를 키워라"
assert SKIRT_Y + 5 < WHEEL_Y - WHEEL_W / 2, "커버가 바퀴에 닿는다 — AXLE_GAP 을 키워라"
assert SKIRT_IN_Y - SKIRT_FLANGE > BASE_W / 2, "커버 플랜지가 상부 사이드 레일에 닿는다"
assert SKIRT_IN_X - SKIRT_FLANGE > BASE_L / 2, "커버 플랜지가 상부 크로스멤버에 닿는다"

# ---- 프레임 빔 ----
CROSS_L = BASE_W - 2 * S                  # 하부 크로스멤버 (사이드 레일 사이 맞댐)
CENTER_L = WHEELBASE - S                  # 중앙 세로 (축 크로스멤버 안쪽면끼리 맞댐)
TOP_CROSS_L = CROSS_L                     # 상부 크로스멤버 (상부 사이드 레일 사이 맞댐)

BEAMS = (
    # 1층 — 바퀴가 달리는 사다리 프레임
    [(f"하부사이드{i+1}", BASE_L, (-BASE_L / 2, sy * RAIL_Y, RAIL_Z), "X")
     for i, sy in enumerate((-1, 1))]
    + [(f"앞뒤크로스{i+1}", CROSS_L, (sx * CROSS_X, -CROSS_L / 2, RAIL_Z), "Y")
       for i, sx in enumerate((-1, 1))]
    + [(f"축크로스{i+1}", CROSS_L, (sx * AXLE_X, -CROSS_L / 2, RAIL_Z), "Y")
       for i, sx in enumerate((-1, 1))]
    + [("중앙세로1", CENTER_L, (-CENTER_L / 2, 0, RAIL_Z), "X")]
    # 기둥 6본 — 앞·중앙·뒤 x 좌우. 중앙 기둥이 700 스팬의 처짐을 잡는다
    + [(f"기둥{i+1}", POST_H, (x, sy * RAIL_Y, FRAME_TOP), "Z")
       for i, (x, sy) in enumerate([(x, sy) for x in POST_X for sy in (-1, 1)])]
    # 2층 — 크로스멤버가 기둥 위에 얹혀 바퀴 위까지 뻗고, 그 끝을 사이드 레일이 받는다
    + [(f"상부크로스{i+1}", TOP_CROSS_L, (x, -TOP_CROSS_L / 2, TOP_RAIL_Z), "Y")
       for i, x in enumerate(POST_X)]
    + [(f"상부사이드{i+1}", TOP_RAIL_L, (-TOP_RAIL_L / 2, sy * TOP_RAIL_Y, TOP_RAIL_Z), "X")
       for i, sy in enumerate((-1, 1))]
)

frame = []
for name, length, start, axis in BEAMS:
    b = bar(SERIES, length, start, axis)
    b.label = name
    frame.append(b)


# ---- 모터 마운트 ----
def motor_plate():
    """사이드 레일 바깥면(y=+PLATE_Y)에 붙는 액슬 마운트 판 한 장.

    레일 바깥 슬롯 한 줄에 M6 6개로 물린다. 세로 하중은 T너트가 슬롯 벽에
    직접 받치고, 구동 반력(Y축 모멘트)은 X 로 170mm 벌어진 볼트열이 받는다.

    ※ 구동 방식이 정해지면 여기에 모터 취부 구멍(플랜지 볼트원)이 추가된다.
    """
    # Plane.XZ 는 법선이 -Y 라, 원점을 y=PLATE_Y+PLATE_T 로 옮기고 안쪽으로 압출한다
    with BuildPart() as p:
        with BuildSketch(Plane.XZ.offset(-(PLATE_Y + PLATE_T))) as sk:
            with Locations((0, PLATE_Z0 + PLATE_H / 2)):
                Rectangle(PLATE_L, PLATE_H)
            fillet(sk.vertices(), radius=10)
            with Locations(*[(sx * x, RAIL_Z) for x in BOLT_XS for sx in (-1, 1)]):
                Circle(BOLT_D / 2, mode=Mode.SUBTRACT)          # 레일 체결 볼트
            with Locations((0, AXLE_Z)):
                Circle(AXLE_D / 2 + AXLE_FIT, mode=Mode.SUBTRACT)   # 데드 액슬 관통공
        extrude(amount=PLATE_T)
    return p.part


# ---- 바퀴 (PVC 통바퀴 + 베어링 + 데드 액슬) ----
def tire():
    """PVC 통바퀴 — Ø210 x 58, 중앙 Ø20 보어. 트레드 모서리만 살짝 굴린다."""
    with BuildPart() as p:
        Cylinder(WHEEL_D / 2, WHEEL_W, rotation=(90, 0, 0))
        fillet(p.edges().filter_by(GeomType.CIRCLE), radius=6)
        Cylinder(WHEEL_BORE / 2, WHEEL_W + 4, rotation=(90, 0, 0), mode=Mode.SUBTRACT)
        # 양쪽 베어링 자리 (실물은 보어 안에 압입돼 있다 — 자리만 표현)
        for s in (-1, 1):
            with Locations((0, s * (WHEEL_W - BEARING_W) / 2, 0)):
                Cylinder(BEARING_OD / 2, BEARING_W, rotation=(90, 0, 0), mode=Mode.SUBTRACT)
    return p.part


def bearings():
    """휠 보어 양쪽에 압입되는 볼베어링 2개 (20 x 47 x 14)."""
    parts = []
    for i, s in enumerate((-1, 1)):
        with BuildPart() as p:
            with Locations((0, s * (WHEEL_W - BEARING_W) / 2, 0)):
                Cylinder(BEARING_OD / 2, BEARING_W, rotation=(90, 0, 0))
                Cylinder(WHEEL_BORE / 2, BEARING_W + 2, rotation=(90, 0, 0), mode=Mode.SUBTRACT)
        p.part.label = f"베어링{i+1}"
        parts.append(p.part)
    return parts


def axle():
    """데드 액슬 Ø20 — 마운트 판을 관통해 너트로 조이고, 바퀴가 그 위에서 돈다.

    차량 하중이 바퀴 자체 베어링으로 흘러서 기어박스 출력축에 굽힘이 안 걸린다.
    """
    y_in = WHEEL_Y - PLATE_Y + AXLE_NUT       # 판 안쪽 너트 자리까지
    y_out = WHEEL_W / 2 + 5                   # 바깥은 와셔·너트 자리만
    with BuildPart() as p:
        with Locations((0, (y_out - y_in) / 2, 0)):
            Cylinder(AXLE_D / 2, y_in + y_out, rotation=(90, 0, 0))
    return p.part


# ---- 판재 ----
def panel(w, d, r, t, z, holes=(), cutouts=()):
    """z 높이에 얹는 라운드 코너 평판.

    holes = [(지름, [(x,y), ...]), ...] / cutouts = [((폭,깊이), [(x,y), ...]), ...]
    """
    with BuildPart() as p:
        with BuildSketch():
            RectangleRounded(w, d, r)
        extrude(amount=t)
        for dia, pts in holes:
            if pts:
                with Locations(*sorted(set(pts))):
                    Hole(dia / 2)
        for (cw, cd), pts in cutouts:
            with BuildSketch() as sk:
                with Locations(*pts):
                    Rectangle(cw, cd)
            extrude(to_extrude=sk.sketch, amount=t, mode=Mode.SUBTRACT)
    return Pos(0, 0, z) * p.part


def grid(x1, y1, pitch):
    """원점 대칭 격자 좌표 (±x1, ±y1 범위)."""
    n, m = int(x1 / pitch), int(y1 / pitch)
    return [(i * pitch, j * pitch) for i in range(-n, n + 1) for j in range(-m, m + 1)]


def bay_floor():
    """전장 베이 바닥판 — 하부 프레임 위에 얹히고 기둥 6본이 뚫고 지나간다.

    외형선을 커버 안쪽면과 맞춰서 커버 아래 플랜지가 그대로 얹힌다.
    50mm 격자 M5 구멍이라 배터리 밴드·드라이버·컨버터를 아무 데나 물릴 수 있다.
    """
    fix = ([(x, sy * RAIL_Y) for x in (-250, -125, 0, 125, 250) for sy in (-1, 1)]
           + [(sx * CROSS_X, y) for y in (-150, 150) for sx in (-1, 1)])
    return panel(2 * SKIRT_IN_X, 2 * SKIRT_IN_Y, SKIRT_R - SKIRT_T, PANEL_T, FRAME_TOP,
                 holes=[(BAY_HOLE, grid(300, 200, BAY_PITCH) + SKIRT_BOLTS),
                        (BOLT_D, fix),
                        (GROM_FLOOR, GROM_FLOOR_PTS)],     # 구동모터 배선 관통
                 cutouts=[((POST_CUT, POST_CUT), POST_PTS)])


def deck():
    """적재 데크 — 100mm 격자 M6 + 상부 프레임 체결 + 커버 윗 플랜지 체결 구멍."""
    fix = ([(x, sy * TOP_RAIL_Y) for x in (-300, -150, 0, 150, 300) for sy in (-1, 1)]
           + [(x, sy * 150) for x in POST_X for sy in (-1, 1)])
    return panel(DECK_L, DECK_W, DECK_R, PANEL_T, DECK_Z,
                 holes=[(DECK_HOLE, grid(300, 200, DECK_PITCH) + fix),
                        (BAY_HOLE, SKIRT_BOLTS),
                        (GROM_DECK, GROM_DECK_PTS)])       # 데크 위 장비 배선


# ---- 후면 패널 (비상정지 + 커넥터) ----
# 후면은 x=-SKIRT_X 의 평면이고, Plane.YZ 는 법선이 +X 라 국부 (u,v) 가 (Y, Z) 로 그대로 간다.
REAR = Plane.YZ.offset(-(SKIRT_X + 10))   # 후면보다 10mm 바깥에서 시작하는 절단용 평면


def door_screws():
    """포트 도어 네 귀퉁이 나사 위치 (y, z)."""
    w, h = PORT_DOOR
    return [(sy * (w / 2 - DOOR_INSET), PORT_DOOR_Z + sz * (h / 2 - DOOR_INSET))
            for sy in (-1, 1) for sz in (-1, 1)]


def rear_cutouts():
    """후면 커버에 뚫는 구멍 — 비상정지 취부공 + 커넥터 + 도어 나사 + 배기팬."""
    with BuildSketch(REAR) as sk:
        for sy in (-1, 1):                                 # 배기팬 2개
            with Locations((sy * FAN_Y, FAN_Z)):
                Circle(FAN_HOLE / 2)
                with GridLocations(FAN_PCD, FAN_PCD, 2, 2):
                    Circle(FAN_SCREW / 2)
        with Locations((0, ESTOP_Z)):
            Circle(ESTOP_HOLE / 2)
            with Locations((0, ESTOP_HOLE / 2)):
                Rectangle(*ESTOP_KEY)                  # 회전방지 홈 (22mm 표준)
        for _, kind, size, pos in PORTS:
            with Locations(pos):
                if kind == "round":
                    Circle(size / 2)
                else:
                    RectangleRounded(size[0], size[1], 2)
        with Locations(*door_screws()):
            Circle(PORT_SCREW / 2)
    return sk.sketch


def estop():
    """비상정지 버튼 (22mm 표준) — 머리와 몸체를 따로 돌려 색을 달리 준다."""
    x0 = -SKIRT_X                                      # 커버 바깥면
    with BuildPart() as head:
        with Locations((x0 - ESTOP_COLLAR_H / 2, 0, ESTOP_Z)):
            Cylinder(ESTOP_COLLAR / 2, ESTOP_COLLAR_H, rotation=(0, 90, 0))
        with Locations((x0 - ESTOP_COLLAR_H - ESTOP_HEAD_H / 2, 0, ESTOP_Z)):
            Cylinder(ESTOP_HEAD / 2, ESTOP_HEAD_H, rotation=(0, 90, 0))
        fillet(head.edges().filter_by(GeomType.CIRCLE).sort_by(Axis.X)[0], radius=6)
    with BuildPart() as body:                          # 패널 뒤 몸체 + 접점 블록
        with Locations((x0 + SKIRT_T + ESTOP_BODY_L / 2, 0, ESTOP_Z)):
            Cylinder(ESTOP_BODY / 2, ESTOP_BODY_L, rotation=(0, 90, 0))
        with Locations((x0 + SKIRT_T + ESTOP_BODY_L, 0, ESTOP_Z)):
            Box(24, 34, 34, align=(Align.MIN, Align.CENTER, Align.CENTER))
    return head.part, body.part


def fans():
    """후면 배기팬 2개 — 커버 안쪽면에 붙는다."""
    parts = []
    for i, sy in enumerate((-1, 1)):
        with BuildPart() as p:
            with Locations((-SKIRT_IN_X + FAN_T / 2, sy * FAN_Y, FAN_Z)):
                Box(FAN_T, FAN_SIZE, FAN_SIZE)
                Cylinder(FAN_HOLE / 2, FAN_T + 2, rotation=(0, 90, 0), mode=Mode.SUBTRACT)
        p.part.label = f"배기팬{i+1}"
        parts.append(p.part)
    return parts


def port_door():
    """커넥터 구멍을 덮는 서비스 도어 — 커버 바깥면에 M4 4개로 붙는다.

    비상정지는 항상 눌러야 하니 도어 밖에 둔다.
    """
    with BuildPart() as p:
        with BuildSketch(Plane.YZ.offset(-(SKIRT_X + PORT_DOOR_T))) as sk:
            with Locations((0, PORT_DOOR_Z)):
                RectangleRounded(*PORT_DOOR, PORT_DOOR_R)
            with Locations(*door_screws()):
                Circle(PORT_SCREW / 2, mode=Mode.SUBTRACT)
        extrude(amount=PORT_DOOR_T)
    return p.part


# ---- 전면 패널 (카메라 + LED 바) ----
def front_cutouts():
    """전면 커버에 뚫는 구멍 — 카메라 렌즈공 + 취부 나사 + LED 슬롯 + 흡기 슬롯."""
    lw, lh = LED_SLOT
    vw, vh = VENT_SLOT
    with BuildSketch(Plane.YZ.offset(SKIRT_X - 10)) as sk:
        with Locations((0, CAM_Z)):
            Circle(CAM_HOLE / 2)
            with PolarLocations(CAM_PCD / 2, 4, start_angle=45):
                Circle(CAM_SCREW / 2)
        with Locations((0, LED_Z)):
            RectangleRounded(lw, lh, lh / 2 - 0.1)      # 양 끝이 둥근 1자 슬롯
        with Locations(*[(y, VENT_Z) for y in VENT_YS]):
            RectangleRounded(vw, vh, vh / 2 - 0.1)      # LED 아래 흡기 그릴
    return sk.sketch


def camera():
    """전면 카메라 — 바깥 베젤 링 + 렌즈 배럴 + 커버 안쪽 몸체."""
    x0 = SKIRT_X                                        # 커버 바깥면
    with BuildPart() as p:
        with Locations((x0 + CAM_BEZEL_T / 2, 0, CAM_Z)):
            Cylinder(CAM_BEZEL / 2, CAM_BEZEL_T, rotation=(0, 90, 0))
            Cylinder(CAM_HOLE / 2, CAM_BEZEL_T + 2, rotation=(0, 90, 0), mode=Mode.SUBTRACT)
        fillet(p.edges().filter_by(GeomType.CIRCLE).sort_by(Axis.X)[-1], radius=2)
        with Locations((x0 + CAM_BEZEL_T - 2 - CAM_LENS_L / 2, 0, CAM_Z)):
            Cylinder(CAM_LENS / 2, CAM_LENS_L, rotation=(0, 90, 0))     # 렌즈 (베젤 안 2mm 들어감)
        with Locations((x0 - SKIRT_T - CAM_BODY[0] / 2, 0, CAM_Z)):
            Box(*CAM_BODY)                                              # 몸체 (커버 안쪽)
    return p.part


def led_bar():
    """LED 바 디퓨저 — 슬롯을 채우는 혀 + 안쪽에서 걸리는 플랜지 (T 단면).

    실제로는 알루미늄 채널에 스트립을 넣고 이 디퓨저를 덮는다.
    """
    lw, lh = LED_SLOT[0] - LED_FIT, LED_SLOT[1] - LED_FIT
    bw, bh, bt = LED_BACK
    with BuildPart() as p:
        with BuildSketch(Plane.YZ.offset(SKIRT_X)) as s:        # 슬롯을 채우는 혀
            with Locations((0, LED_Z)):
                RectangleRounded(lw, lh, lh / 2 - 0.1)
        extrude(amount=-(SKIRT_T + 1))                          # 법선 반대 = 안쪽
        with BuildSketch(Plane.YZ.offset(SKIRT_X - SKIRT_T)) as s2:   # 안쪽 플랜지
            with Locations((0, LED_Z)):
                RectangleRounded(bw, bh, 8)
        extrude(amount=-bt)
    return p.part


# ---- 측면 커버 ----
def skirt():
    """전장 베이를 감싸는 라운드 코너 절곡 커버 — 좌/우 두 짝으로 갈라진다.

    수직 밴드 + 상하 안쪽 플랜지(절곡)로 이루어진다. 베이 바닥판부터 데크 아랫면까지
    통으로 올라가 베이를 완전히 막는다. 아래 플랜지는 바닥판에, 위 플랜지는 데크에 물린다.
    플랜지 폭은 상부 프레임에 닿지 않는 선까지만 준다 (위 assert 로 확인).
    """
    h = SKIRT_TOP - BAY_Z
    with BuildPart() as p:
        with BuildSketch(Plane.XY.offset(BAY_Z)):          # 수직 밴드
            RectangleRounded(2 * SKIRT_X, 2 * SKIRT_Y, SKIRT_R)
            RectangleRounded(2 * SKIRT_IN_X, 2 * SKIRT_IN_Y, SKIRT_R - SKIRT_T,
                             mode=Mode.SUBTRACT)
        extrude(amount=h)
        for z0 in (BAY_Z, SKIRT_TOP - SKIRT_T):
            with BuildSketch(Plane.XY.offset(z0)):         # 안쪽으로 접히는 플랜지
                RectangleRounded(2 * SKIRT_IN_X, 2 * SKIRT_IN_Y, SKIRT_R - SKIRT_T)
                RectangleRounded(2 * (SKIRT_IN_X - SKIRT_FLANGE), 2 * (SKIRT_IN_Y - SKIRT_FLANGE),
                                 SKIRT_R - SKIRT_T - SKIRT_FLANGE, mode=Mode.SUBTRACT)
            extrude(amount=SKIRT_T)
            with BuildSketch(Plane.XY.offset(z0)) as bs:   # 플랜지 볼트 구멍
                with Locations(*SKIRT_BOLTS):
                    Circle(BAY_HOLE / 2)
            extrude(to_extrude=bs.sketch, amount=SKIRT_T, mode=Mode.SUBTRACT)
        # 직선부에서는 플랜지가 프레임에 닿지 않지만 코너 라운드 구간에서는 파고든다.
        # 프레임 외곽선(+여유)만큼 비워 코너에서만 플랜지를 잘라낸다.
        with Locations((0, 0, BAY_Z + h / 2)):
            Box(BASE_L + 2 * FRAME_CLR, BASE_W + 2 * FRAME_CLR, h + 10, mode=Mode.SUBTRACT)
        extrude(to_extrude=rear_cutouts(), amount=20, mode=Mode.SUBTRACT)    # 후면 패널 구멍
        extrude(to_extrude=front_cutouts(), amount=20, mode=Mode.SUBTRACT)   # 전면 패널 구멍
        with Locations((0, 0, BAY_Z + h / 2)):             # 앞/뒤 분할
            Box(SKIRT_SEAM, 2 * SKIRT_Y + 20, h + 20, mode=Mode.SUBTRACT)
    return p.solids()


# ---- 사이드 스커트 (탈착식) / 데크 모듈 레일 ----
def side_skirt():
    """바퀴 사이 하부를 가리는 탈착식 스커트 — 앞/뒤 두 짝.

    커버와 같은 외형선을 이어받아 단차 없이 내려온다. 바퀴 자리는 아치로 파낸다.
    상단 탭 6개가 커버 하단 플랜지 볼트를 공유하므로, 그 볼트만 풀면 스커트만 떨어진다.
    액슬 마운트 판(x 100~300)을 피해 앞뒤 면에만 탭을 둔다.
    """
    top = FRAME_TOP                        # 커버 하단과 만나는 높이
    h = top - SKIRT2_BOT
    inx, iny = SKIRT_X - SKIRT2_T, SKIRT_Y - SKIRT2_T
    with BuildPart() as p:
        with BuildSketch(Plane.XY.offset(SKIRT2_BOT)):          # 수직 밴드
            RectangleRounded(2 * SKIRT_X, 2 * SKIRT_Y, SKIRT_R)
            RectangleRounded(2 * inx, 2 * iny, SKIRT_R - SKIRT2_T, mode=Mode.SUBTRACT)
        extrude(amount=h)
        # 상단 고정 탭 — 앞뒤 면 6개 + 바퀴 사이 옆구리 4개.
        # 측면 x 100~300 은 액슬 마운트 판이 지나가므로 그 구간엔 탭을 못 둔다.
        tabs = ([(sx * BOLT_LINE_X, y) for y in (-150, 0, 150) for sx in (-1, 1)]
                + [(x, sy * BOLT_LINE_Y) for x in (-50, 50) for sy in (-1, 1)])
        with BuildSketch(Plane.XY.offset(top - SKIRT2_T)) as tb:
            with Locations(*[(sx * (inx - SKIRT2_TAB / 2), y)
                             for y in (-150, 0, 150) for sx in (-1, 1)]):
                Rectangle(SKIRT2_TAB, 34)
            with Locations(*[(x, sy * (iny - SKIRT2_TAB / 2)) for x in (-50, 50)
                             for sy in (-1, 1)]):
                Rectangle(34, SKIRT2_TAB)
        extrude(to_extrude=tb.sketch, amount=SKIRT2_T)
        with BuildSketch(Plane.XY.offset(top - SKIRT2_T)) as bh:
            with Locations(*tabs):
                Circle(BAY_HOLE / 2)
        extrude(to_extrude=bh.sketch, amount=SKIRT2_T, mode=Mode.SUBTRACT)
        for sx in (-1, 1):                                      # 휠 아치
            with Locations((sx * AXLE_X, 0, AXLE_Z)):
                Cylinder(WHEEL_D / 2 + ARCH_CLR, 2 * SKIRT_Y + 20,
                         rotation=(90, 0, 0), mode=Mode.SUBTRACT)
        with Locations((0, 0, SKIRT2_BOT + h / 2)):             # 앞/뒤 분할
            Box(SKIRT_SEAM, 2 * SKIRT_Y + 20, h + 20, mode=Mode.SUBTRACT)
    return p.solids()


def fender_shell():
    """바퀴 위·바깥까지 감싸는 광폭 하부 셸 — 앞/뒤 두 짝.

    아래가 트인 상자다. 바퀴가 그 안에 들어가고 지면 쪽 50mm 만 노출된다.
    옆벽은 바퀴 바깥면에서 FENDER_CLR, 지붕은 바퀴 꼭대기에서 FENDER_OVER 띄운다.
    지붕이 커버 바깥면(SKIRT_Y)까지 이어져 위에서 봐도 바퀴가 안 보인다.
    """
    fy = WHEEL_Y + WHEEL_W / 2 + FENDER_CLR + SKIRT2_T    # 셸 바깥면 반치수
    top = WHEEL_D + FENDER_OVER + SKIRT2_T                # 셸 윗면
    with BuildPart() as p:
        with BuildSketch(Plane.XY.offset(SKIRT2_BOT)):     # 옆·앞뒤 벽
            RectangleRounded(2 * SKIRT_X, 2 * fy, SKIRT_R)
            RectangleRounded(2 * (SKIRT_X - SKIRT2_T), 2 * (fy - SKIRT2_T),
                             SKIRT_R - SKIRT2_T, mode=Mode.SUBTRACT)
        extrude(amount=top - SKIRT2_BOT)
        with BuildSketch(Plane.XY.offset(top - SKIRT2_T)) as rf:   # 지붕 (커버 옆면까지)
            RectangleRounded(2 * (SKIRT_X - SKIRT2_T), 2 * (fy - SKIRT2_T),
                             SKIRT_R - SKIRT2_T)
            RectangleRounded(2 * SKIRT_X, 2 * SKIRT_Y, SKIRT_R, mode=Mode.SUBTRACT)
        extrude(to_extrude=rf.sketch, amount=SKIRT2_T)
        # 상단 고정 탭 — 커버 하단 플랜지 볼트 공유 (스커트와 동일)
        with BuildSketch(Plane.XY.offset(FRAME_TOP - SKIRT2_T)) as tb:
            with Locations(*[(sx * (SKIRT_X - SKIRT2_T - SKIRT2_TAB / 2), y)
                             for y in (-150, 0, 150) for sx in (-1, 1)]):
                Rectangle(SKIRT2_TAB, 34)
        extrude(to_extrude=tb.sketch, amount=SKIRT2_T)
        with BuildSketch(Plane.XY.offset(FRAME_TOP - SKIRT2_T)) as bh:
            with Locations(*[(sx * BOLT_LINE_X, y) for y in (-150, 0, 150) for sx in (-1, 1)]):
                Circle(BAY_HOLE / 2)
        extrude(to_extrude=bh.sketch, amount=SKIRT2_T, mode=Mode.SUBTRACT)
        # 측면커버가 이미 차지한 영역(프레임 윗면 위쪽)을 비운다.
        # 그래야 펜더가 '커버 바깥에 덧붙는 껍데기'가 되고 LED·팬과도 안 부딪힌다.
        with BuildSketch(Plane.XY.offset(FRAME_TOP)) as cut:
            RectangleRounded(2 * SKIRT_X, 2 * SKIRT_Y, SKIRT_R)
        extrude(to_extrude=cut.sketch, amount=top - FRAME_TOP + 10, mode=Mode.SUBTRACT)
        with Locations((0, 0, (SKIRT2_BOT + top) / 2)):    # 앞/뒤 분할
            Box(SKIRT_SEAM, 2 * fy + 20, top - SKIRT2_BOT + 20, mode=Mode.SUBTRACT)
    return p.solids()


def deck_rails():
    """데크 위 2020 레일 3본 + 엔드 스톱.

    적재함·로봇팔 베이스에 T너트를 달아 뒤에서 밀어 끼우고 원하는 위치에서 조인다.
    레일은 데크 100mm 격자 구멍으로 아래에서 볼트를 올려 T너트로 물린다.
    """
    z0 = DECK_Z + PANEL_T + PROFILES[RAIL_SERIES]["size"] / 2   # 레일 단면 중심
    parts = []
    for i, y in enumerate(RAIL_YS):
        b = bar(RAIL_SERIES, RAIL_L, (-RAIL_L / 2, y, z0), "X")
        b.label = f"데크레일{i+1}"
        parts.append(b)
        sw, sd, sh = RAIL_STOP
        rs = PROFILES[RAIL_SERIES]["size"]
        s = Pos(RAIL_L / 2 - sw / 2, y, z0 + rs / 2 + sh / 2) * Box(sw, sd, sh)
        s.label = f"엔드스톱{i+1}"
        parts.append(s)
    return parts


# ---- 배선 관통부 / 배터리 브라켓 ----
def grommets(dia, panel_t, z, pts):
    """배선 관통부에 끼우는 고무 그로밋.

    목(dia)이 판 구멍에 딱 끼고 위아래 플랜지가 판을 물어 판재와 간섭하지 않는다.
    """
    fl, bore = 4, dia - 6                      # 플랜지 돌출 / 케이블이 지나는 안지름
    parts = []
    for i, (x, y) in enumerate(pts):
        with BuildPart() as p:
            with Locations((x, y, z + panel_t / 2)):
                Cylinder(dia / 2, panel_t)                          # 구멍에 끼는 목
            for zc in (z - fl / 2, z + panel_t + fl / 2):
                with Locations((x, y, zc)):
                    Cylinder((dia + 8) / 2, fl)                     # 위아래 플랜지
            with Locations((x, y, z + panel_t / 2)):
                Cylinder(bore / 2, panel_t + 4 * fl, mode=Mode.SUBTRACT)
        p.part.label = f"그로밋{i+1}"
        parts.append(p.part)
    return parts


def angle_profile(inner, leg=ANGLE_L, t=ANGLE_T):
    """ㄱ 앵글 단면 꼭짓점. u=inner 면이 배터리에 닿고, 바깥으로 수평 다리가 눕는다."""
    return [(inner, 0), (inner + leg, 0), (inner + leg, t),
            (inner + t, t), (inner + t, leg), (inner, leg)]


def battery_bracket():
    """배터리 고정 — 측면/단부 앵글 4개(미끄러짐) + 상부 스트랩 2개(들림).

    볼트 자리는 전부 바닥판 50mm 격자 위라 판에 구멍을 새로 뚫을 필요가 없다.
    """
    bx, by, bz = BATT
    parts = []

    def drill(part, pts):
        with BuildPart() as p:
            add(part)
            with Locations(*[(x, y, BAY_Z + ANGLE_T) for x, y in pts]):
                Cylinder(BATT_BOLT / 2, 20, mode=Mode.SUBTRACT)
        return p.part

    # 측면 앵글 (좌우) — X 방향으로 뻗는다
    half = bx / 2 - 40
    with BuildPart() as side:
        with BuildSketch(Plane.YZ.offset(-half)):
            Polygon(*[(u, v + BAY_Z) for u, v in angle_profile(by / 2 + BATT_GAP)], align=None)
        extrude(amount=2 * half)
    sx_, sy_ = BATT_SIDE_BOLT
    for s in (1, -1):
        a = side.part if s > 0 else mirror(side.part, about=Plane.XZ)
        a = drill(a, [(x, s * sy_) for x in (-sx_, sx_)])
        a.label = f"측면앵글{'L' if s < 0 else 'R'}"
        parts.append(a)

    # 단부 앵글 (앞뒤) — Y 방향으로 뻗는다
    halfy = by / 2 + 5
    with BuildPart() as end:
        with BuildSketch(Plane.XZ.offset(-halfy)):
            Polygon(*[(u, v + BAY_Z) for u, v in angle_profile(bx / 2 + BATT_GAP)], align=None)
        extrude(amount=2 * halfy)
    ex_, ey_ = BATT_END_BOLT
    for s in (1, -1):
        a = end.part if s > 0 else mirror(end.part, about=Plane.YZ)
        a = drill(a, [(s * ex_, y) for y in (-ey_, ey_)])
        a.label = f"단부앵글{'앞' if s > 0 else '뒤'}"
        parts.append(a)

    # 상부 스트랩 (ㄷ자) — 배터리를 눌러 바닥판에 물린다
    px_, py_ = BATT_STRAP_BOLT
    leg_y = py_ - STRAP_FOOT / 2 - STRAP_T / 2          # 세로 다리 중심 Y
    for i, s in enumerate((-1, 1)):
        with BuildPart() as st:
            with Locations((s * px_, 0, BAY_Z + bz + STRAP_T / 2)):
                Box(STRAP_W, 2 * leg_y + STRAP_T, STRAP_T)          # 윗판
            for sy in (-1, 1):
                with Locations((s * px_, sy * leg_y, BAY_Z + bz / 2)):
                    Box(STRAP_W, STRAP_T, bz)                        # 세로 다리
                with Locations((s * px_, sy * py_, BAY_Z + STRAP_T / 2)):
                    Box(STRAP_W, STRAP_FOOT, STRAP_T)                # 발
                    Cylinder(BATT_BOLT / 2, 20, mode=Mode.SUBTRACT)  # 발 볼트
        st.part.label = f"스트랩{i+1}"
        parts.append(st.part)
    return parts


# ---- 전장 목업 (공간 확인용, 내보내기에는 포함하지 않는다) ----
MOCKUPS = [
    # (이름, 크기 XYZ, 중심 XY, 바닥 높이)
    # 그로밋 4개(±200, ±150)와 배터리 브라켓을 피해 배치했다.
    # 라즈베리파이는 REF_PARTS 의 실물 STEP 으로 대체했다.
    ("배터리 24V 20Ah", BATT, (0, 0), BAY_Z),        # 축간 정중앙 — 무게중심
    ("모터드라이버1", (100, 80, 35), (230, -60), BAY_Z),
    ("모터드라이버2", (100, 80, 35), (230, 60), BAY_Z),
    ("DC-DC 컨버터", (110, 60, 40), (-250, 150), BAY_Z),
]


def ref_parts():
    """refmodel/ 의 실물 STEP 을 불러와 XY 중심을 맞추고 베이 바닥에 앉힌다."""
    parts = []
    for name, (x, y), rot in REF_PARTS:
        path = REF_DIR / name
        if not path.exists():
            print(f"  (실물 STEP 없음: {path.name})")
            continue
        p = Rot(0, 0, rot) * import_step(str(path))
        bb = p.bounding_box()
        p = Pos(x - bb.center().X, y - bb.center().Y, BAY_Z - bb.min.Z) * p
        p.label = name.split(" v")[0]
        parts.append(p)
    return parts


def mockups():
    parts = []
    for name, (w, d, h), (x, y), z in MOCKUPS:
        b = Pos(x, y, z) * Box(w, d, h, align=(Align.CENTER, Align.CENTER, Align.MIN))
        b.label = name
        parts.append(b)
    return parts   # 데크 위 센서는 뺐다 — 레일에 뭘 얹을지 정해지면 그때 넣는다


# ---- 조립 ----
# +Y 쪽 부품을 한 번만 만들고 -Y 쪽은 미러해서 쓴다 (판·축이 좌우 비대칭이다)
PLATE_R, AXLE_R, TIRE = motor_plate(), axle(), tire()
BEAR = bearings()
PLATE_L_, AXLE_L_ = mirror(PLATE_R, about=Plane.XZ), mirror(AXLE_R, about=Plane.XZ)

plates, wheels_t, wheels_h = [], [], []
for i, (sx, sy) in enumerate([(-1, -1), (1, -1), (1, 1), (-1, 1)]):
    pl = Pos(sx * AXLE_X, 0, 0) * (PLATE_R if sy > 0 else PLATE_L_)
    pl.label = f"액슬마운트{i+1}"
    plates.append(pl)
    at = Pos(sx * AXLE_X, sy * WHEEL_Y, AXLE_Z)
    t = at * TIRE
    t.label = f"바퀴{i+1}"
    wheels_t.append(t)
    for j, b in enumerate(BEAR):
        bb = at * b
        bb.label = f"베어링{i+1}-{j+1}"
        wheels_h.append(bb)
    a = Pos(sx * AXLE_X, 0, AXLE_Z) * (Pos(0, WHEEL_Y, 0) * AXLE_R if sy > 0
                                       else Pos(0, -WHEEL_Y, 0) * AXLE_L_)
    a.label = f"액슬{i+1}"
    wheels_h.append(a)

floor_panel, deck_panel = bay_floor(), deck()
floor_panel.label, deck_panel.label = "베이바닥판", "데크"
covers = skirt()
for i, c in enumerate(covers):
    c.label = f"측면커버{'앞' if c.center().X > 0 else '뒤'}"
estop_head, estop_body = estop()
door = port_door()
estop_head.label, estop_body.label, door.label = "비상정지버튼", "비상정지몸체", "포트도어"
cam, led = camera(), led_bar()
cam.label, led.label = "전면카메라", "LED바"
exhaust = fans()
brackets = battery_bracket()
groms = (grommets(GROM_FLOOR, PANEL_T, FRAME_TOP, GROM_FLOOR_PTS)
         + grommets(GROM_DECK, PANEL_T, DECK_Z, GROM_DECK_PTS))


def group(parts, name):
    """같은 색으로 묶은 표시 단위."""
    color = Color(CLR[name])
    for p in parts:
        p.color = color
    g = Compound(children=parts)
    g.label, g.color = name, color
    return g


GROUPS = [
    group(frame, "프레임"),
    group(plates, "액슬마운트"),
    group(wheels_h, "축·베어링"),
    group(wheels_t, "바퀴"),
    group([floor_panel, deck_panel], "판재"),
    group(covers, "측면커버"),
    group([estop_head], "비상정지"),
    group([estop_body, door] + exhaust, "후면패널"),
    group([cam], "카메라"),
    group([led], "LED바"),
    group(brackets, "배터리브라켓"),
    group(groms, "그로밋"),
]
if LOWER_COVER:
    sk = side_skirt() if LOWER_COVER == "skirt" else fender_shell()
    for s in sk:
        s.label = f"{'스커트' if LOWER_COVER=='skirt' else '펜더'}{'앞' if s.center().X > 0 else '뒤'}"
    GROUPS.append(group(sk, "사이드스커트"))
if DECK_RAIL:
    GROUPS.append(group(deck_rails(), "데크레일"))
assembly = Compound(children=GROUPS)
assembly.label = EXPORT_NAME

# ---- 제원 ----
bb = assembly.bounding_box()
print(f"전장 {bb.size.X:.0f} x 전폭 {bb.size.Y:.0f} x 전고 {bb.size.Z:.0f} mm"
      f" (바퀴 Ø{WHEEL_D}, 지상고 {CLEARANCE:.0f})")
print(f"  축간거리 {WHEELBASE:.0f} / 윤거 {2*WHEEL_Y:.0f} (비율 {WHEELBASE/(2*WHEEL_Y):.2f})")
print(f"  전장 베이 {2*SKIRT_IN_X:.0f} x {2*SKIRT_IN_Y:.0f} x {BAY_H:.0f} mm"
      f" (바닥 z={BAY_Z:.0f}, 기둥 6본이 안쪽으로 {S:.0f}mm 들어옴)")
print(f"    바닥 유효 {2*(SKIRT_IN_X-SKIRT_FLANGE):.0f} x {2*(SKIRT_IN_Y-SKIRT_FLANGE):.0f}"
      f" — 커버 아래 플랜지 안쪽까지가 장비를 놓을 수 있는 범위다")
print(f"  환기: 흡기 슬롯 {VENT_SLOT[0]}x{VENT_SLOT[1]} x {len(VENT_YS)}개(앞면 LED 아래, "
      f"개구 {len(VENT_YS)*VENT_SLOT[0]*VENT_SLOT[1]/100:.0f} cm²) / 배기 {FAN_SIZE}mm 팬 2개 @ z={FAN_Z}")
print(f"  배선: 바닥판 Ø{GROM_FLOOR} 그로밋 {len(GROM_FLOOR_PTS)}개(구동모터), "
      f"데크 Ø{GROM_DECK} {len(GROM_DECK_PTS)}개")
print(f"  배터리 브라켓: 앵글 4 + 스트랩 2, 볼트 12개 전부 바닥판 {BAY_PITCH}mm 격자 위")
print(f"  데크 {DECK_L:.0f} x {DECK_W:.0f} (코너 R{DECK_R}) @ z={DECK_Z:.0f}"
      f" — 바퀴 꼭대기보다 {DECK_Z - WHEEL_D:.0f} 위, 적재면 {DECK_L*DECK_W/1e6:.2f} m²")
print(f"  측면커버 {2*SKIRT_X} x {2*SKIRT_Y} (코너 R{SKIRT_R}) 높이 {SKIRT_TOP-BAY_Z:.0f}"
      f" / 바퀴까지 {WHEEL_Y - WHEEL_W/2 - SKIRT_Y:.0f}mm, 데크 그림자 틈 {DECK_LIP}mm")
print(f"  상부 프레임 {BASE_L} x {BASE_W} — 커버 안쪽으로 "
      f"{SKIRT_IN_Y - BASE_W/2:.0f}mm 들어가 밖에서 안 보인다")
print(f"  후면 패널: 비상정지 Ø{ESTOP_HOLE} @ z={ESTOP_Z} (머리 Ø{ESTOP_HEAD}, "
      f"{ESTOP_COLLAR_H + ESTOP_HEAD_H}mm 돌출) / 포트 도어 {PORT_DOOR[0]}x{PORT_DOOR[1]} "
      f"@ z={PORT_DOOR_Z}")
for name, kind, size, (y, z) in PORTS:
    spec = f"Ø{size}" if kind == "round" else f"{size[0]}x{size[1]}"
    print(f"    {name:16s} {spec:>9s}  (y={y:+4.0f}, z={z})")
print(f"  전면 패널: 카메라 Ø{CAM_HOLE} @ z={CAM_Z} (베젤 Ø{CAM_BEZEL}) / "
      f"LED 바 {LED_SLOT[0]}x{LED_SLOT[1]} @ z={LED_Z}"
      f" — 눈·입 간격 {CAM_Z - CAM_BEZEL/2 - (LED_Z + LED_SLOT[1]/2):.0f}mm")

_, frame_kg = cut_list([(n, l) for n, l, _, _ in BEAMS], SERIES)
panel_kg = sum(p.volume for p in (floor_panel, deck_panel)) * ALU_DENSITY / 1000
mount_kg = sum(p.volume for p in plates) * ALU_DENSITY / 1000
cover_kg = sum(c.volume for c in covers) * ALU_DENSITY / 1000
wheel_kg = WHEEL_SET_KG * 4
print("중량 집계:")
for label, kg in [("프레임", frame_kg), (f"판재 {PANEL_T}t", panel_kg),
                  (f"측면커버 {SKIRT_T}t", cover_kg), ("액슬마운트", mount_kg),
                  ("바퀴 4개(추정, 모터 별도)", wheel_kg)]:
    print(f"  {label:22s} {kg:5.2f} kg")
total = frame_kg + panel_kg + cover_kg + mount_kg + wheel_kg
print(f"  {'합계 (배터리 제외)':20s} {total:5.2f} kg")

# ---- 뷰어 표시 (뷰어가 없으면 건너뜀) ----
view = list(GROUPS)
if MOCKUP:
    view.append(group(mockups(), "목업"))
if REAL_PARTS:
    rp = ref_parts()
    if rp:
        bb0 = Compound(children=rp).bounding_box()
        print(f"실물 STEP {len(rp)}개 로드 — 솔리드 {sum(len(p.solids()) for p in rp)}개, "
              f"점유 {bb0.size.X:.0f} x {bb0.size.Y:.0f} x {bb0.size.Z:.0f} mm "
              f"(z {bb0.min.Z:.0f}~{bb0.max.Z:.0f})")
        view.append(group(rp, "실물부품"))
try:
    show(*view, names=[g.label for g in view], colors=[CLR[g.label] for g in view],
         reset_camera=Camera.RESET)
except Exception as e:
    print(f"뷰어에 표시하지 못함 (뷰어 미실행?): {e}")

# ---- 내보내기 ----
EXPORT_DIR.mkdir(exist_ok=True)
export_step(assembly, str(EXPORT_DIR / f"{EXPORT_NAME}.step"))
export_stl(assembly, str(EXPORT_DIR / f"{EXPORT_NAME}.stl"))
print(f"내보내기 완료: {EXPORT_DIR / EXPORT_NAME}.step / .stl")
