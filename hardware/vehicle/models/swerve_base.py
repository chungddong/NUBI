# 모빌리티 테스트 베이스 — 호버보드 6.5인치 인휠 허브모터 4륜
#
# 구동 방식이 두 가지다. DRIVE 환경변수로 고른다.
#
#   DRIVE=skid   (기본) 4륜 고정 스키드 스티어. 좌우 속도차로 돈다.
#                 바퀴가 안 돌아가니 킹핀·베어링·조향모터·벨트·서스펜션이 전부 없다.
#                 남는 건 축을 잡는 브라켓 하나. 먼저 이걸로 굴려보고 스워브로 올린다.
#   DRIVE=swerve       4륜 독립조향 + 트레일링암 서스펜션. 아래 제약이 붙는다.
#                 바퀴가 킹핀 둘레로 반경 84mm 원을 쓸어서 차체가 바퀴 옆으로 못 내려온다.
#                 → 모듈이 아래, 차체가 위로 층이 갈린다 (FRC 스웨어브 방식).
#
# 스키드가 잃는 것: 게걸음(crab) 불가, 제자리회전 시 타이어가 끌린다(바닥 자국·소음).
#   회전 저항 M = μ·W·L/2, 구동 M = μ·W·T/2 → T/L 이 클수록 헛돎이 준다.
#   실차 스키드로더가 T/L 1.2~1.5 라서 휠베이스를 520 → 420 으로 줄였다 (T/L=1.24).
# 스키드가 얻는 것: 전고 -26mm, 제어축 4개(구동만), 부품 4세트 삭제, 비용 급감.
#
# 축 실측 (refmodel STEP): 축이 **한쪽으로만** 나온다. 더블 시어가 불가능하다.
#   -51~-48 Ø14 나사 / -48~-3 Ø16 축 / -3~+3 숄더 / 6~51 바퀴 / 54~60 커버
#   그중 **D컷 평면은 -46.5~-3.7 (42.8mm)** 뿐이다. 양 끝은 원형이라 판이 못 물린다.
#   → 외팔보다. 42.8mm 안에 판 두 장을 벌려 세워 굽힘을 couple 로 받는다.

import os
from pathlib import Path

import math

from build123d import *
from ocp_vscode import Camera, set_port, show

from tslot import ALU_DENSITY, PROFILES, bar, cut_list
# 휠 마운트는 wheel_mount.py 가 **원본**이다. 거기서 치수를 잡고 검증하고 STEP/STL 을
# 내보내므로, 여기서 다시 그리면 두 벌이 어긋난다. 함수와 상수를 그대로 가져다 쓴다.
# (wheel_mount 의 좌표계가 이미 "타이어 중심면 y=0 · 지면 z=0" 이라 그대로 얹힌다.)
import wheel_mount as WM

DRIVE = os.environ.get("DRIVE", "skid")      # skid | swerve
assert DRIVE in ("skid", "swerve"), f"DRIVE 는 skid 또는 swerve (받은 값: {DRIVE})"
SKID = DRIVE == "skid"

# MODULE_ONLY=1 → 구동부 설계 전용 모드. 프레임 + 바퀴 + 장착부만 남긴다
MODULE_ONLY = bool(os.environ.get("MODULE_ONLY"))

# ---- 파라미터 (단위: mm, 지면 z=0) ----
SERIES = 30
BASE_L, BASE_W = 760, 450     # 차체 프레임 외곽 — 스웨어브는 축간을 늘려도 토크 손해가 없다

# 호버보드 6.5인치 허브모터 — refmodel STEP 실측값
HUB_STEP = Path(__file__).parent.parent / "refmodel" / "Motor, BLDC, Hoverboard Wheel, 6.5in.stp"
# HUB_MOCK=1 → 렌더용 목업(원기둥+축). 실물 STEP 은 테셀레이션이 뷰당 10분씩 걸린다.
# 간섭검사·내보내기는 항상 실물로 돌려라 (목업은 트레드가 없어 여유를 과대평가한다).
# 목업일 때는 뷰어 전송을 막는다 — 안 그러면 뷰어의 실물 모델을 목업이 덮어쓴다.
HUB_REAL = not os.environ.get("HUB_MOCK")
if not HUB_REAL:
    os.environ.setdefault("NOSHOW", "1")
WHEEL_D, WHEEL_W = 158.7, 55  # 타이어 외경 / 몸통 폭 (STEP X 5~60)
HUB_TIRE_X = (9.2, 48.0)      # 타이어 구간 (STEP X) — 이 중심이 접지 중심면
HUB_SHAFT_D, HUB_FLAT = 16.0, 13.0    # 축 Ø16, D컷 13 (한쪽 3mm 평면)
# D컷 **평면** 구간 (STEP X) — 0.1mm 단위로 스캔한 실측. Ø16 축은 -48~-3 이지만
# 양 끝은 평면이 없는 완전 원형(나사 언더컷 / 숄더 필렛)이라 판이 물리면 안 된다.
# 처음 -48~-3 으로 잡았다가 안쪽 판이 원형 구간을 밟아 34mm3 간섭으로 잡혔다.
#
# 한때 이 언더컷을 줄로 밀어내려 했다 — 닫힌 D보어를 축방향으로 밀어 끼우려면
# 통과해야 했기 때문이다. 브라켓을 **아래로 열린 U + 캡** 클램프로 바꾸면서
# 축방향 삽입 자체가 없어져 필요가 사라졌다. 판을 평면 구간 안에만 두면 된다.
# (True 로 두면 wheel() 이 줄 작업 끝난 축을 그린다 — 지금은 쓰지 않는다.)
HUB_FILE_UNDERCUT = False
HUB_UNDERCUT_X = (-50.0, -46.5)          # 0.1mm 스캔 실측 — 평면 없는 Ø16 원형 3.5mm
HUB_SHAFT_X = (HUB_UNDERCUT_X[0] if HUB_FILE_UNDERCUT else HUB_UNDERCUT_X[1], -3.7)
# STEP 은 X=-51 에서 축을 **잘라놨다**. -51~-50 은 1mm 모따기고 **나사부가 없다.**
# 처음엔 이 모따기를 'Ø14 나사 3mm' 로 읽어서 언더컷을 1.5mm 로, 너트 자리를 -48 로
# 잡았다. 실물 호버보드 축은 여기서 더 나가고 그 끝에 나사(보통 M12, 15~25mm)가 있다.
# → **실물을 재기 전까지 너트 인터페이스는 확정할 수 없다.** 아래는 잠정값이다.
HUB_SHAFT_END_X = -50.0       # STEP 이 보여주는 축의 끝 (모따기 시작)
HUB_THREAD_MEASURED = False   # 실물 나사 규격·길이를 재서 넣으면 True 로
KINGPIN_X, KINGPIN_Y = 260, 260   # 조향축 위치 — 바퀴 중심선 바로 위(스크럽 0)
#   Y 는 베어링 하우징(Ø59)이 사이드 레일 바깥면(225)에 안 닿는 최소값에서 잡았다
SKID_WHEELBASE = 420          # 스키드 휠베이스 — T/L=1.24. 짧을수록 제자리회전이 쉽고
#   길수록 피치 전복각·직진 안정성이 좋다. 브라켓이 레일을 타므로 실물에서 조정 가능.
AXLE_X = SKID_WHEELBASE / 2 if SKID else KINGPIN_X
AXLE_Y = KINGPIN_Y
AXLE_Z = WHEEL_D / 2
YOKE_CLR = 3                  # 최대 압축 시 바퀴 꼭대기와 요크 수평팔 사이 여유
WHEEL_CLR = 12                # (스키드) 바퀴 꼭대기 ~ 차체 바닥

# 고정 브라켓 (DRIVE=skid) — 클램프 판 2장 + 상단 타이 + 레일 옆면 플랜지.
# 축이 외팔보라서 판 하나로는 구멍 안에서 축이 기운다. 두 장을 벌려 couple 로 받는다.
BR_T, BR_R, BR_W = 8, 30, 60  # 판 두께 / 축 보스 반경 / 판 x 폭
BR_FIT = 0.3                  # 축 구멍 끼움 여유 (지름 기준)
BR_SLEEVE = 26                # 위치 스페이서 외경
BR_OUT_SLEEVE = 5             # 허브 숄더 ~ 바깥 판 사이 위치 스페이서
# 하부 클램프 캡. 높이는 **과체결 견딤**으로 정해진다 — 필요 체결력은 438 N/볼트뿐인데
# M6 를 규정대로 조이면 8 kN 이 나온다. 축 밑 살 16mm 면 107 MPa (6061 여유 2.6배).
# 처음엔 30(살 21.8, 여유 4.8배)으로 뒀는데 과했다. 지상고도 6mm 벌었다.
CAP_H, CAP_BOLT_X = 24, 22    # 캡 높이 / 캡 볼트 x (±)
# ---- 프로파일 방식 휠 마운트 (2026-08 채택) ----
# 판재 6종을 깎던 브라켓은 사실 **레일에서 91.3mm 를 내려주는 일**만 했다.
# 축을 무는 건 호버보드에 달려 나온 클램프가 이미 한다. 남은 일은 레일에서 축까지
# 91.35mm 를 내려주는 것뿐이고, 그건 **ㄷ자 휠마운트** 한 덩이가 한다
# (wheel_mount.py 가 원본 — 치수·검증·STEP/STL 이 전부 거기 있다).
#   3030 휠포스트 x4 → 마운트 x4 로 대체. 포스트도 끝단 M8 탭도 없어졌다.
#   클램프는 **실물 STEP** 이다 (42.72x39.22x12.57, 채널 깊이 5.07 = D컷 평면 높이).
#     예전 CLAMP=(50,40,44) 가정값은 지워졌다 — 이제 가정이 아니라 실물이다.
# **핀치 갭** — 캡 윗면과 판 밑면 사이 틈. 이게 없으면 볼트를 조일 때 캡이 축을 물기
# 전에 판에 먼저 닿아서 클램프가 아예 안 된다 (처음에 딱 맞대 놨었다).
CAP_GAP, CAP_FIT = 1.0, 0.1   # 핀치 갭 / 캡 시트 끼움 (판 보어와 달리 거의 0)
# 캡 볼트는 **판 사이 브리지 블록**에 탭한다. 8mm 판 모서리에 세로로 M6 를 탭하면
# 구멍 옆 살이 (8-5)/2 = 1.5mm 뿐이라 터진다 (M6 는 둘레 Ø12 이상 필요).
# 브리지는 판 사이 빈 구간을 채우므로 y 로 22.5mm, 위로 BRIDGE_Z 만큼 살이 있다.
BRIDGE_Z = 20                 # 브리지 블록 높이 = M6 탭 깊이 (3.3xD)
CAP_BOLT = (6.6, 5.0)         # 캡 통과공 / 브리지 하공 (M6)
_TIRE_C = sum(HUB_TIRE_X) / 2         # 타이어 중심 (STEP X) — 이게 바퀴 배치 기준면
# 축방향 스택. 너트는 Ø14 나사에만 물리고 Ø16 단차에서 멈춘다. 그래서 숄더~나사 45mm 를
# [슬리브 + 판 + 슬리브 + 판] 으로 **정확히** 채워야 조임이 판까지 전달된다.
# 처음엔 양 끝을 합쳐 7.2mm 띄워 두어서, 축이 구멍을 지나기만 하고 축방향으로는
# 전혀 안 잡혀 있었다 (간섭 0건이라 검사에는 안 걸렸다).
BR_SHOULDER_Y = -3.0 - _TIRE_C                 # 허브 숄더 면
BR_THREAD_Y = HUB_SHAFT_END_X - _TIRE_C        # 축 끝 = 안쪽 판 바깥면 (너트가 여기 눌러야)
BR_FLAT = (HUB_SHAFT_X[0] - _TIRE_C, HUB_SHAFT_X[1] - _TIRE_C)   # D컷 평면 구간
BR_YS = (BR_SHOULDER_Y - BR_OUT_SLEEVE - BR_T / 2,   # 바깥 판 — 스페이서로 숄더에 붙는다
         BR_FLAT[0] + BR_T / 2)                      # 안쪽 판 — D컷 평면 안쪽 끝에 맞춘다
BR_SPAN = abs(BR_YS[0] - BR_YS[1])
BRIDGE_W = BR_SPAN - BR_T                 # 판 사이 빈 구간 = 브리지 폭
BRIDGE_Y = (BR_YS[0] + BR_YS[1]) / 2      # 브리지·캡볼트 y
# 토크는 바깥 판의 D컷이 받는다. 안쪽 판은 너트 면에 붙어야 해서 평면이 없는
# 원형 구간을 1.5mm 밟는다 → 그쪽 구멍만 원형으로 판다 (평면으로 파면 간섭).
assert BR_FLAT[0] < BR_YS[0] - BR_T / 2 and BR_YS[0] + BR_T / 2 < BR_FLAT[1], \
    "바깥 클램프 판이 D컷 평면 구간을 벗어난다 (구동 토크를 못 받는다)"

KP_D, KP_BEAR, KP_BW = 25, 47, 12     # 킹핀 축 / 베어링 6005 (25 x 47 x 12)
KP_COLLAR = (40, 10)                  # 축 빠짐 방지 칼라 (지름, 두께)
BELT_T, BELT_W = 3, 15                # 타이밍벨트 두께 / 폭
YOKE_T, YOKE_GAP = 8, 6       # 요크 판 두께 / 바퀴와의 틈

# 서스펜션 — 모듈 안 트레일링 암. 조향계(킹핀·베어링·벨트·모터)는 전혀 안 건드린다.
SUS = True
# 서스펜션 좌표는 **바퀴 크기에 따라 움직여야 한다.** 절대 z 로 박아두면 바퀴를
# 줄였을 때 쇼크가 모듈 밖으로 튀어나온다 (6.5인치로 바꿀 때 실제로 그랬다).
SUS_PIVOT_X, SUS_PIVOT_DZ = -95, 60   # 피벗 x / 차축에서 올린 높이
SUS_TRAVEL = 25               # 바퀴 상하 트래블 — 모듈 높이가 여기서 역산된다
SHOCK_TOP_X, SHOCK_TOP_DZ = -25, -33  # 쇼크 상단 (x / 모듈 상한에서 내린 높이)
SHOCK_BOT_X, SHOCK_BOT_DZ = -15, 7    # 쇼크 하단 (x / 차축에서 올린 높이)
SHOCK_D, SHOCK_Y = 22, -16    # 쇼크 몸통 지름 / 암 판에서 안쪽 오프셋
PIVOT_D, BUMP = 10, (20, 12)  # 피벗 볼트 / 범프스톱 (지름, 높이)
STEER_D, STEER_L = 45, 90     # 조향 모터 몸체 (Ø45 유성감속 급)
STEER_OFFSET = 70             # 킹핀에서 조향 모터까지 (벨트 센터거리)
STEER_PULLEY = (24, 48)       # 모터 24T → 킹핀 48T (2:1)

# 외장 셸 — 바퀴 위를 덮는 본체는 z>=MODULE_TOP 에서 전둘레로 두른다.
# 그 아래로는 앞뒤를 내리지 않고, 옆면만 바퀴 사이에서 --\__/-- 로 내려온다.
SHELL = not MODULE_ONLY
SHELL_Y = float(os.environ.get("SHELL_Y", "305"))
SHELL_R, SHELL_T = 60, 3                  # 옆면 반폭 / 코너 R / 두께
COMPACT_SHELL = SHELL_Y < 280
SWEEP_CLR = 10                            # 조향 스윕 여유
# 스커트 저부 반길이 / 경사가 끝나는 반길이 — 스키드는 휠베이스가 짧아 같이 좁아진다
SKIRT_LOW_X, SKIRT_TOP_X = (60, 118) if SKID else (70, 138)
SKIRT_DROP = 45                           # 스커트가 본체 밑단에서 내려오는 깊이
#   6.5인치로 바꾸며 차체가 내려앉아 지상고가 112 까지 떨어졌다. 깊이를 줄여 되돌린다
SKIRT_T = 10                              # 스커트 벽 두께 — 본체(3mm)보다 두툼하게
FLANGE_T, FLANGE_W = 6, 45                # 상단 고정 플랜지 두께 / 안쪽으로 뻗는 폭
FLANGE_BOLT_X = (-300, -100, 100, 300)    # 옆면 플랜지 볼트 x (상부사이드 레일 위)
FLANGE_BOLT_Y = (-150, 150)               # 앞뒤 플랜지 볼트 y (상부크로스 위)
DECK_LIP2 = 0                             # 데크를 셸과 같은 크기로 (실루엣이 이어진다)

# 앞면 얼굴 — 앞 패널이 z=MODULE_TOP~DECK_Z 로 짧아진 만큼 위로 몰아 배치한다.
# 카메라는 실물 STEP (ASUS Xtion Pro Live) 을 그대로 얹는다.
FACE = not MODULE_ONLY
XTION_STEP = Path(__file__).parent.parent / "refmodel" / "ASUS Xtion Pro Live.step"
# STEP 원본 좌표: 렌즈가 +Z, 스탠드 아래가 -Y, 바 길이가 X.
XT_KEEP_BASE = False          # 스탠드·베이스는 떼어내고 센서 바만 쓴다
CAM_WIN_CLR = 1.0             # 창 여유 — 바 실루엣에 딱 맞춘다
CAM_BRK_T, CAM_CHEEK_T = 8, 4  # 크래들 바닥판 두께 / 옆 치크 두께
_LED_W = 360 if COMPACT_SHELL else 430
LED_SLOT, LED_BACK, LED_FIT = (_LED_W, 22), (_LED_W + 24, 40, 3), 0.6
VENT_SLOT = (80, 8)
VENT_YS = (-170, -85, 0, 85, 170) if COMPACT_SHELL else (-200, -100, 0, 100, 200)

# 후면 — 비상정지는 늘 눌러야 하니 밖으로, 커넥터는 안으로 들어간 베이 안에 둔다.
ESTOP_HOLE, ESTOP_KEY = 22.3, (3.5, 1.6)  # 22mm 표준 취부 구멍 / 회전방지 홈
ESTOP_HEAD, ESTOP_HEAD_H = 40, 20         # 버섯 머리
ESTOP_COLLAR, ESTOP_COLLAR_H = 32, 4      # 패널에 닿는 칼라
ESTOP_BODY, ESTOP_BODY_L = 28, 45         # 패널 뒤 몸체 + 접점 블록
PORT_BAY, PORT_BAY_D, PORT_BAY_R = (220, 95), 20, 12   # 베이 (폭,높이) / 깊이 / 코너 R

# 소형 카메라 — 후방 2 / 전방 2 / 좌우 1 씩. 서라운드 뷰용.
MINI_HOLE, MINI_BODY, MINI_LENS = 14, (20, 26, 26), (14, 12)   # 렌즈공 / 몸체(깊이,폭,높이)
MINI_PCD, MINI_SCREW = 24, 2.4            # 취부 나사 간격 / M2.5

# 냉각 — 앞면 그릴은 모듈실용이다(바닥판 아래). 베이 공기는 옆에서 받아 뒤로 뺀다.
FAN_SIZE, FAN_T, FAN_HOLE = 60, 25, 56    # 60mm 배기팬
FAN_PCD, FAN_SCREW = 50, 4.3
FAN_YS = (-160, 160)                      # 포트 베이 옆, 뒤쪽 기둥은 피한다
LOUVER = (60, 8)                          # 측면 흡기 슬롯 (길이, 높이)
LOUVER_XS = (170, 240)
LOUVER_DZ = (-20, 0, 20)                  # 베이 중간 높이 기준 3단

# 베이 내부 탑재물 — 전부 바닥판 위에 앉는다.
INTERIOR = not MODULE_ONLY
RPI_STEP = Path(__file__).parent.parent / "refmodel" / "RPI5+Pimoroni-NVME+Waveshare-POE v1.step"
PI_AT = (120, -130)                       # 라즈베리파이5 + NVMe + PoE HAT
PI_ENV = (58.1, 90.0, 36.5)               # 실물 STEP 실측 외형
PI_DETAIL = False                         # True 면 실물 STEP (솔리드 516개 — 아래 주석 참고)
BATT = (200, 150, 90)                     # 배터리 — [추정] 실측되면 여기만 고치면 된다
BATT_AT = (-140, 0)
BATT_WALL, BATT_STRAP = 6, (20, 4)        # 브라켓 벽 두께 / 상단 스트랩 (폭, 두께)
HB_BOARD, HB_AT = (130, 70, 22), ((-140, -140), (-140, 140))   # 호버보드 메인보드 2장
STEER_BOARD, STEER_BOARD_AT = (90, 60, 20), (120, 130)         # 조향 서보 드라이버 + BEC

# 데크 모듈 레일 — 적재함·로봇팔을 슬롯 너트로 아무 위치에나 문다.
DECK_RAIL = not MODULE_ONLY
RAIL_SERIES = 20
# Keep the outer rails inside the compact 510 mm shell and its rounded skirt.
RAIL_YS = (-160, 0, 160) if COMPACT_SHELL else (-200, 0, 200)
RAIL_X1 = 380                 # 레일 앞 끝. 뒤 끝은 셸 뒷면(-SHELL_X)까지 나간다.
#   적재함을 **앞에서 뒤로** 밀어 넣으므로 레일은 함의 뒷면으로 들어온다.
#   그래서 노치는 뒷면에만 있고 앞면은 통으로 막힌다 = 구멍이 뒤에서만 보인다.

# 안테나 — 뒷면 상부 코너. 데크는 적재함이 다 쓰므로 셸에 붙인다.
ANT_D, ANT_L, ANT_TILT = 10, 150, 10      # 휩 지름 / 길이 / 뒤로 기운 각도(수직 기준)
ANT_BASE = (16, 14)                       # SMA 벌크헤드 (지름, 높이)
ANT_Y, ANT_DZ = (170 if COMPACT_SHELL else 230), -11  # 좌우 위치 / 셸 윗단에서 내린 높이

# 적재함 — 셸과 같은 라운드 단면(810x610 R60)을 그대로 위로 이어 올린다.
# 데크 레일에 **뒤에서 밀어 끼운다**. 그래서 스커트는 레일 구간만 덮고, 뒤쪽만 열려 있다.
#   (스커트를 데크까지 내리면 라운드 코너가 좁아져 밀어 넣을 수가 없다 — 계산해보고 알았다)
PAYLOAD = "none" if MODULE_ONLY else os.environ.get("PAYLOAD", "none")   # cargo | flat | sensor | arm | inst | none
#   환경변수로 덮어쓸 수 있다 — 모듈 5종을 한 번에 검증하려고
CARGO = PAYLOAD == "cargo"
FLAT = PAYLOAD == "flat"
TOWER = PAYLOAD == "sensor"
ARM = PAYLOAD == "arm"
INST = PAYLOAD == "inst"
# 계측 모듈 — 이 프로젝트의 숫자(89kg / CG 357 / 13 N/mm / 처짐 19mm)는 전부 계산값이다.
# 로드셀 4개로 페이로드 질량·무게중심을 실측하고, 서스펜션 스트로크 4채널을 받아
# 스프링 상수와 코너 하중을 역산한다. **바퀴 하중은 데크에서 직접 못 잰다** —
# 모듈과 데크 사이 로드셀은 페이로드만 본다.
INST_PLATE_T = 6              # 베이스판
INST_CELL = (40, 25)          # 버튼형 로드셀 (지름, 높이)
INST_CELL_AT = (50, 210, 190) # 로드셀 배치 (중심 x, x간격 반, y간격 반)
INST_TOP = (600, 500, 8)      # 시험 플랫폼 (여기에 밸러스트를 올린다)
INST_DAQ, INST_DAQ_X = (120, 200, 80), -315   # DAQ 박스 / 위치
INST_DISP = (110, 70, 14)     # 디스플레이
INST_CONN, INST_CONN_N = 16, 4        # 스트로크 센서 커넥터 (지름, 개수)
INST_TOW = (60, 46, 26)       # 견인 훅 브라켓 (x, y, z)
INST_BTN = (22, 4)            # 로드셀 상부 가압 버튼 (지름, 높이)
# 로봇 팔 마운트 — 팔 자체는 구매품이다. 여기서 만드는 건 하중을 레일로 분산하는
# 두꺼운 베이스판, 취부 보스, 컨트롤러 자리, 케이블 경로다.
# 아래 팔 제원은 [가정] xArm6 급. 기종이 정해지면 데이터시트로 바꿀 것.
ARM_AT = (0, 0)               # 팔 베이스 — 전복 대칭을 위해 축간 중심에 둔다
ARM_PLATE_T = 10              # 베이스판 (팔 모멘트를 3본 레일로 분산)
ARM_BOSS = (150, 14)          # 취부 보스 (지름, 높이)
ARM_PCD, ARM_BOLT, ARM_NB = 100.0, 6.6, 4     # 취부 볼트 PCD / 구멍 / 개수 [가정]
ARM_BASE_D, ARM_BASE_H = 126, 110             # 팔 베이스 몸통 [가정]
ARM_REACH, ARM_MASS, ARM_PAYLOAD = 700, 12.0, 3.0   # 도달반경 / 자중 / 가반하중 [가정]
ARM_CTRL, ARM_CTRL_AT = (200, 160, 90), (250, 0)    # 컨트롤러 박스
# 센서 타워 — 3D 라이다 + 팬틸트 카메라 + GNSS + IMU.
# 마스트에 얹은 라이다는 진동에 죽는다. 기둥 2본 + 가로타이 + 대각 브레이스로 삼각화한다.
TOWER_SER, TOWER_POST_Y = 20, 100     # 마스트 프로파일 / 기둥 2본 간격(반)
TOWER_H = 940                 # 헤드 플레이트 밑면 높이
TOWER_BASE_T = 6              # 베이스 판 두께
TOWER_HEAD = (240, 240, 6)    # 헤드 플레이트
TOWER_TIE_Z = (640, 810)      # 가로타이 높이
TOWER_BRACE = (180, 700, 8, 30)   # 대각 브레이스 (밑단 x, 윗단 z, 두께, 폭)
LIDAR, LIDAR_RISER = (65, 65), 30     # 3D 라이다 (지름, 높이) — Livox Mid-360 급
PANTILT, PANTILT_Z = (70, 90, 62), 745   # 팬틸트 카메라 (x,y,z) / 높이
GNSS, GNSS_AT = (60, 15), (-90, 0)        # GNSS 안테나 (지름, 높이) / 헤드판 위 위치
IMU = (36, 26, 12)            # IMU — 베이스 판 위, 회전중심 근처
# 평판 모듈 — 슬라이드 인터페이스는 적재함과 완전히 같다. 위만 다르다.
FLAT_PT, FLAT_LIP, FLAT_LIP_T = 4, 20, 6      # 상판 두께 / 가장자리 립 (높이, 두께)
FLAT_GRID, FLAT_TIE_XS = 100, (-260, 0, 260)  # M6 장착 격자 / 타이다운 x 위치
# 높이는 **바구니가 정한다**. 유로 컨테이너 600x400x220 한 개가 기준이고,
# 그 위에 바닥판·띄움·여유·어깨·뚜껑립이 얹힌 값이 CARGO_H 다.
#   220(바구니) + 3(바닥판) + 8(띄움) + 8(여유) + 30(어깨) + 10(뚜껑립) = 279
# 어깨를 55 에서 30 으로 줄였다 — 360 짜리 상자엔 55 가 어울렸지만 280 에서는
# 높이의 20%를 그냥 먹는다.
CARGO_H = 280                 # 360 → 280 (실측 근거는 위 계산)
CARGO_SHOULDER = 30           # 윗단 어깨 라운드 — 각진 상자를 부드럽게 만든다
CARGO_CAP_N = 6               # 어깨를 근사하는 단면 수
CARGO_T = 5                   # 프린트 셸 두께 — 리브를 없애고 벽으로 버틴다
CARGO_CH = 22                 # 슬라이드 채널 안폭 (높이는 레일 구간에서 나온다)
CARGO_TAB = (2.5, 4)          # 레일 측면 슬롯에 물리는 턱 (깊이, 두께)
CARGO_PT = 3                  # 바닥판 / 뚜껑 판재
CARGO_LID_GAP, CARGO_LID_LIP = 1.0, 10        # 뚜껑 틈 / 안쪽 위치 립
CARGO_HINGE = (10, 70)        # 내부 힌지 축 (지름, 길이)
CARGO_HINGE_Y = None          # 바구니칸 폭에서 유도한다 (CARGO_BAY_Y 뒤)
CARGO_LOCK = (46, 34, 28)     # 솔레노이드 잠금 (x, y, z)
# ---- 바구니(유로 컨테이너)를 꽂는 구조 ----
# 라이너 트레이를 직접 그려 넣었었다. 이제는 **기성 유로 컨테이너**를 꽂는다.
#   600x400 은 VDA 4500(KLT) 규격이고, 같은 발자국으로 EPP 보온 서모박스도 나온다.
#   → 로봇은 600x400 을 잡는 구조 하나만 만들면 범용 화물과 보온 배달이 다 된다.
#     단열·씰·세척이 전부 소모품 쪽으로 넘어간다.
# 크기는 **앞쪽 장비 베이와 맞바꾼 값**이다. 600x400 이면 적재 53 L 로 크지만
#   바구니칸이 데크 길이를 다 먹어 앞에 78mm 밖에 안 남는다 (전자부품을 못 넣는다).
#   400x300 으로 내리면 적재 26 L (Cartken 27 급) 에 앞쪽 272mm 가 열린다.
# 방향: 긴 변(400)을 x 로 놓는다. 그러면 KLT 손잡이 구멍이 있는 **짧은 변이 ±x**
#   를 향하므로 손 넣을 자리도 x 다 — 격벽 위치가 여기서 역산된다.
CARGO_BIN = (400, 300, 220)   # 유로 컨테이너 (KLT / EPP 서모박스 공용 발자국)
CARGO_BIN_HAND = 45           # 짧은 변에 손 넣는 자리 — KLT 손잡이 구멍에 손가락이 들어간다
CARGO_BULK_T = 5              # 격벽 두께 (앞 장비베이 / 뒤 바구니칸)
CARGO_EQ_GRID = 50            # 장비 베이 마운트 격자 (M4)
CARGO_BIN_CLR = 6             # 리브 안쪽 여유 — 드롭인이라 넉넉해야 한다
CARGO_FOOT = 8                # 바닥에서 띄우는 높이 (배수 + 손가락)
CARGO_RIB = (10, 40)          # 위치 리브 (두께, 높이) — 낮게 둬야 손이 들어간다
CARGO_GRIP = (240, 55, 28)    # 뚜껑 손잡이 — 손 넣어 들어올리는 홈 (폭 y, 길이 x, 깊이 z)
CARGO_GRID = 100              # 상단 센서 마운트 격자 (M5)
CARGO_TIE = 30                # 바닥 타이다운 앵커 (지름) — 화물을 바닥에 묶는다
CARGO_RET = (44, 30, 4)       # 노치 리테이너 (y, z, 두께) — 함이 앞으로 빠지는 걸 막는다
CARGO_STRUT = (12, 260)       # 가스 스트럿 (지름, 길이)

# 배선 관통 그로밋 — 바닥은 구동·조향, 데크는 적재물용
GROMMET_D = 25
GROMMET_FLOOR = [(sx * 300, sy * 170) for sx in (-1, 1) for sy in (-1, 1)]
GROMMET_DECK = ((-100, 150), (-100, -150))

# ---- 안전·제어 구조 ----
# 인휠모터 배선(3상 3 + 홀 5 = 8선)이 킹핀을 지나야 한다. 슬립링은 20~30A 를
# 못 버티니, 중공축 + 조향각 제한 + 서비스 루프로 간다. 그래서 아래 셋은 한 묶음이다.
KP_BORE = 13                  # 킹핀 중공 내경 (8선 단면 ~30mm² < 133mm²)
STOP_A = (15, 165)            # 스톱 핀 각도 — 베어링 판 위에 얹히는 범위 안에서
STOP_R, STOP_D, STOP_H = 45, 8, 7         # 핀 반경 / 지름 / 길이
STOP_TAB_W, STOP_TAB_A = 12, 270          # 칼라 걸림턱 각도폭 / 조향 0° 기준 각도
HOME_MAG = (6, 3)             # 원점 자석 (지름, 두께) — 칼라 외주에 박는다
HOME_PCB = (20, 12, 1.6)      # 홀 IC 보드
BREAKER = (30, 30, 60)        # 주회로 차단기 (포트 베이 스위치 구멍 뒤)
FUSEBOX = (110, 70, 45)       # 퓨즈 + 프리차지 보드
FUSEBOX_AT = (-280, 120)

# 범퍼 — 셸이 직접 부딪히면 깨진다. 밑단에 바를 대고 뒤에 리미트 스위치를 둔다.
BUMPER = ((360 if COMPACT_SHELL else 480), 38, 24)  # 폭 y, 높이 z, 두께 x
BUMPER_DZ = 3                 # 셸 밑단(MODULE_TOP)에서 아래로 내린 양
BUMPER_SW_Y = (-110, 110) if COMPACT_SHELL else (-150, 150)
BUMPER_PLUNGER, BUMPER_SW = 10, (20, 12, 10)

# 후미등 / 부저
TAIL, TAIL_YS = ((100, 14), (-95, 95)) if COMPACT_SHELL else ((150, 14), (-135, 135))
TAIL_DZ = -21                             # 데크 밑에서 내린 높이
BUZZER = (30, 22)                         # 부저 (지름, 길이) — 앞면 그릴 뒤에서 운다
BUZZER_Y = 210

# 리프팅 핸들 — 50kg 이라 데크에서 상부 사이드 레일로 하중이 바로 가야 한다
# 아웃리거 — 팔 작업·계측 때 서스펜션을 우회해 차체를 지면에 지지한다.
#   **모듈이 아니라 베이스에 붙인다.** 모듈에 달면 로봇 전체 무게가 슬라이드
#   인터페이스(레일 슬롯에 2.5mm 물린 턱)를 통과하게 된다.
#   자리 제약: z<MODULE_TOP 에서는 90° 조향 스윕이 x 151~369 를 다 쓴다. 남는 건
#   |x|<151 (앞뒤로 너무 짧다) 뿐이라, **스윕 위(상부 사이드 레일)에서 피벗**해
#   셸 밖으로 내려오는 접이식으로 간다.
OUTRIGGER = False             # 팔·계측 작업용. 평소엔 끈다
OUT_X = (-360, 360)           # 앞뒤 위치 — 지지 베이스를 앞뒤로도 벌린다
#   (상부 레일 위라 x 제약이 없다. 레일 끝 ±380 안에서 최대한 벌렸다)
OUT_PIVOT_DY = 15             # 셸 바깥면에서 피벗까지
OUT_ARM_L, OUT_ARM = 240, (26, 22)    # 암 길이 / 단면 (x, 두께)
OUT_ANG = 45                  # 전개 각도 (수직 기준)
OUT_JACK = (34, 110)          # 잭 튜브 (지름, 길이)
OUT_SCREW, OUT_FOOT = 18, (80, 14)    # 스크류 / 발판 (지름, 두께)
OUT_DEPLOY = False            # True 면 전개 상태로 그린다

# 측면 확장 레일 — 라이다·초음파·조명 등을 아무 위치에나 다는 확장 인터페이스.
#   셸 옆면에서 비는 z 대역은 290~325 뿐이다 (아래는 핸들, 위는 루버가 쓴다).
#   브라켓이 셸을 지나 **하부사이드 레일** 바깥 슬롯에 물린다 (핸들과 같은 하중 경로).
SIDE_RAIL = False             # 측면 확장 레일
SR_SERIES = 20
SR_Z, SR_Y = 310, 330         # 레일 중심 높이 / 셸 바깥면(305)에서 25 띄운다
SR_L = 760
SR_BRK_X = (-340, -180, 180, 340)     # 브라켓 x — 핸들 개구(|x| 10~140) 를 피한다
SR_BRK_T = 6
# 2D 라이다 예시 — 대각 2개면 각 270° 로 360° 를 덮는다 (AMR 표준 배치)
SR_LIDAR = True
SR_LIDAR_D, SR_LIDAR_H = 70, 60
SR_LIDAR_AT = ((340, 1), (-340, -1))  # (x, y부호) — 전방좌 / 후방우

# 리프팅 핸들 — 하부사이드 레일에 직결하고 셸에는 개구만 낸다.
# 상부 레일에 매달려던 앞 설계는 레일~셸 사이 22mm 가 비어 있어 사실상 3mm
# 프린트 셸에 붙는 꼴이었다. 50kg 을 들면 셸이 뜯어진다.
HANDLE_XS = (0,)              # 양쪽 1개씩 = 2개. 무게중심(x≈0) 에 둔다
HANDLE_SPAN = 170             # 팔 두 개 사이 간격 = 그립 길이
#   개수를 줄인 만큼 봉을 길게 잡았다. 한 손잡이에 두 손이 들어가야
#   4인 인양(1인 22kg)이 유지된다. 짧게 두면 2인 44kg 이 되어 못 든다.
#   단 220 까지 늘리면 팔(x=±110)이 요크 수평팔 회전 반경과 스친다
HANDLE_ARM, HANDLE_HOLE = 20, 24      # 팔 지름 / 셸 관통공 (그로밋 여유 2mm)
HANDLE_BAR, HANDLE_STANDOFF = 18, 42  # 그립 봉 지름 / 셸 바깥면에서 봉 중심까지

CAM_RAIL = bool(os.environ.get("CAM_RAIL"))   # 카메라크로스 (Xtion 받침)
ANIMATE = not os.environ.get("NOSHOW")   # 뷰어에 조향·구름 애니메이션 전송

BAY_H = 140
PANEL_T = 3
DECK_LIP = 5

CLR = {
    "프레임": "#c8ccd0", "모듈요크": "#4a5058", "바퀴": "#2b2b2b",
    "모듈": "#4a5058", "베어링": "#b6bbc0", "킹핀브라켓": "#6b7480",
    "조향모터": "#8d9298", "조향벨트": "#1b1b1b", "판재": "#9aa0a6",
    "조인트브라켓": "#8d9298",
    "휠클램프": "#c62828", "휠마운트": "#6b7480",
    "스윕영역": "#c62828", "외장셸": "#39414a",
    "카메라": "#14181d", "카메라브라켓": "#9aa0a6", "LED바": "#f2f4f0",
    "소형카메라": "#14181d", "비상정지": "#c62828", "배기팬": "#1b1b1b",
    "아웃리거": "#6b7480", "확장레일": "#c8ccd0", "2D라이다": "#2e7d32",
    "원점센서": "#2e7d32", "범퍼": "#1b1b1b", "범퍼스위치": "#8d9298",
    "후미등": "#c62828", "부저": "#37474f", "핸들": "#6b7480", "전원": "#455a64",
    "적재함": "#39414a", "적재함판재": "#aeb4ba", "적재함뚜껑": "#39414a",
    "평판": "#39414a", "평판타이다운": "#6b7480",
    "센서타워": "#c8ccd0", "센서장비": "#14181d",
    "팔마운트": "#aeb4ba", "로봇팔": "#e8a33d",
    "계측모듈": "#aeb4ba", "계측장비": "#2e7d32",
    "적재함지붕": "#39414a",
    "슬라이드채널": "#6b7480", "적재함철물": "#8d9298", "타이다운": "#6b7480",
    "리테이너": "#8d9298", "바구니크래들": "#8d9298", "유로컨테이너": "#2e7d32",
    "적재함지붕": "#39414a", "장비마운트": "#aeb4ba",
    "컴퓨터": "#2e7d32", "드라이버": "#37474f", "배터리": "#455a64",
    "배터리브라켓": "#9aa0a6", "데크레일": "#c8ccd0", "안테나": "#1b1b1b",
}
EXPORT_DIR = Path(__file__).parent.parent / "exports"
EXPORT_NAME = "skid_base" if SKID else "swerve_base"

# ---- Xtion 실측 ----
# 치수를 (180, 41, 36) 으로 반올림해 두었더니 창과 크래들이 0.3mm 씩 물렸다.
# STEP 을 한 번 읽고 실측 bbox 를 그대로 쓴다.
_XT_ALL = import_step(str(XTION_STEP))
_XT_BAR = max(_XT_ALL.solids(), key=lambda s: s.volume)          # 큰 쪽이 센서 바
XT_OBJ = _XT_ALL if XT_KEEP_BASE else _XT_BAR
_xb = _XT_BAR.bounding_box()
XT_BAR = (_xb.size.X, _xb.size.Y, _xb.size.Z)   # STEP (X,Y,Z) = 로봇 (길이, 높이, 깊이)
XT_BAR_C = (_xb.center().X, _xb.center().Y)
XT_FRONT = _xb.max.Z                            # 렌즈면 (STEP Z 최대)
assert abs(XT_BAR[0] - 180) < 2 and abs(XT_BAR[1] - 42) < 2, \
    f"Xtion STEP 이 예상과 다르다 (센서 바 {XT_BAR})"

# ---- 유도 치수 ----
# 스워브: 요크 수평팔이 바퀴 바로 위를 건너간다. 그래서 모듈 높이(=차체 바닥)를 임의로
#   정하면 안 되고, '최대 압축된 바퀴 + 여유' 위에 팔이 오도록 역산해야 한다.
#   처음엔 MODULE_TOP=WHEEL_D+20 으로 두었다가 압축 4°에서 바퀴가 팔을 파고들었다.
# 스키드: 바퀴 위를 건너가는 게 없으니 바퀴 꼭대기 + 여유가 곧 바닥이다. 26mm 낮아진다.
YOKE_ARM_Z = WHEEL_D + SUS_TRAVEL + YOKE_CLR + YOKE_T / 2   # 요크 수평팔 중심
# MOUNT=low 면 **하단 클램프 구조**다. 레일이 축 바로 위로 내려와 91mm 캔틸레버가
# 사라진다 (실물이 부러진 그 캔틸레버다). 대가는 차체 바닥이 바퀴 꼭대기보다
# 낮아진다는 것 — 셸에 휠아치가 필요하고, 지금은 그 작업이 안 돼 있다.
MOUNT_LOW = os.environ.get("MOUNT") == "low"
MODULE_TOP = ((WM.LOW_RAIL_BOT if MOUNT_LOW else WHEEL_D + WHEEL_CLR)
              if SKID else (YOKE_ARM_Z + YOKE_T / 2 + 2))
S = PROFILES[SERIES]["size"]
RAIL_Y = (BASE_W - S) / 2                 # 사이드 레일 중심 Y
CROSS_X = (BASE_L - S) / 2
POST_X = (-CROSS_X, 0.0, CROSS_X)
RAIL_Z = MODULE_TOP + S / 2               # 하부 프레임 중심 = 모듈 바로 위
FRAME_TOP = RAIL_Z + S / 2
BAY_Z = FRAME_TOP + PANEL_T
POST_H = PANEL_T + BAY_H
TOP_RAIL_Z = FRAME_TOP + POST_H + S / 2
DECK_Z = TOP_RAIL_Z + S / 2

SUS_PIVOT = (SUS_PIVOT_X, AXLE_Z + SUS_PIVOT_DZ)
SHOCK_TOP = (SHOCK_TOP_X, MODULE_TOP + SHOCK_TOP_DZ)
SHOCK_BOT = (SHOCK_BOT_X, AXLE_Z + SHOCK_BOT_DZ)
SHOCK_L = math.dist(SHOCK_TOP, SHOCK_BOT)             # 아이투아이
ARM_L = math.dist(SUS_PIVOT, (0, AXLE_Z))             # 피벗→차축 = 트레일링 암 길이
MOTION = math.dist(SUS_PIVOT, SHOCK_BOT) / ARM_L      # 모션비
SWEEP_R = (WHEEL_D ** 2 + WHEEL_W ** 2) ** 0.5 / 2    # 조향 시 바퀴 스윕 반경
# 바퀴가 차체 안쪽으로 침범하는 한계 반경. 스워브는 스윕 원, 스키드는 바퀴 반지름.
WHEEL_R_XY = WHEEL_D / 2 if SKID else SWEEP_R
TRACK = 2 * AXLE_Y
WHEELBASE = 2 * AXLE_X
SHELL_WHEEL_Y = float(os.environ.get("SHELL_WHEEL_Y", str(AXLE_Y)))
WHEEL_ARCH_CLR = float(os.environ.get("WHEEL_ARCH_CLR", "12"))
# 스키드 제자리회전 성립 조건 — 회전저항 μWL/2 vs 구동 μWT/2. T/L 이 여유다.
SKID_TL = TRACK / WHEELBASE
if SKID:
    assert SKID_TL > 1.15, \
        f"휠베이스가 길어 제자리회전에 헛돎이 심하다 (T/L={SKID_TL:.2f}, 1.2 이상 권장)"

assert MODULE_TOP <= RAIL_Z - S / 2 + 0.01, "차체가 바퀴 스윕 안으로 내려왔다"
if MOUNT_LOW:
    _over = WHEEL_D - (RAIL_Z + S / 2)
    print(f"  [MOUNT=low] 레일 중심 z={RAIL_Z:.2f} / 차체 바닥 z={MODULE_TOP:.2f}"
          f" / 지상고 {AXLE_Z-WM.LOW_H:.2f}")
    print(f"             바퀴가 레일 윗면보다 {_over:.1f}mm 솟는다 → 셸 휠아치 필요")
assert SKID or YOKE_ARM_Z - YOKE_T / 2 >= WHEEL_D + SUS_TRAVEL + YOKE_CLR - 0.01, \
    "요크 수평팔이 낮아 서스펜션이 트래블만큼 못 움직인다 (바퀴가 팔을 파고든다)"

CROSS_L = BASE_W - 2 * S
SKIRT_LOW_Z = MODULE_TOP - SKIRT_DROP     # 스커트 저부 높이(= 이 구간 지상고)
SHELL_X = BASE_L / 2 + 25                 # 셸 앞뒤 반길이 (스윕 원 밖 + 코너 여유)
TOP_RAIL_Y = float(os.environ.get("TOP_RAIL_Y", "265"))  # 상부 사이드 레일 중심
# 라운드 코너 셸이 사각 프레임을 품으려면 0.293R 만큼 커야 한다 (커버에서 겪은 문제)
_need = 0.293 * (SHELL_R - SHELL_T)
assert SHELL_X - SHELL_T >= BASE_L / 2 + _need, "셸이 짧아 프레임 앞뒤 모서리를 못 품는다"
assert SHELL_Y - SHELL_T >= TOP_RAIL_Y + S / 2 + _need, "셸이 좁아 프레임 옆 모서리를 못 품는다"
TOP_CROSS_L = 2 * TOP_RAIL_Y - S
assert SHELL_X > AXLE_X + WHEEL_R_XY + SWEEP_CLR + SHELL_T, "앞뒤 패널이 바퀴와 겹친다"
# 스커트 경사 끝은 바퀴 안쪽 경계보다 안에 있어야 바퀴가 안 스친다
assert SKIRT_TOP_X <= AXLE_X - WHEEL_R_XY - SWEEP_CLR, "스커트가 바퀴를 침범한다"
assert SKIRT_LOW_X < SKIRT_TOP_X, "스커트 경사가 없다 (--\\__/-- 가 안 나온다)"
KP_PTS = [(sx * AXLE_X, sy * AXLE_Y) for sx in (-1, 1) for sy in (-1, 1)]
# 클램프 y — wheel_mount 가 정한 값을 그대로 옮긴다 (타이어 중심면 기준 → 로봇 좌표).
# 예전엔 3030 휠포스트가 레일에서 클램프까지 내려왔지만, 지금은 ㄷ자 마운트 한 덩이가
# 레일을 감싸고 축까지 통째로 내려온다. 포스트도 WPOST_* 도 없어졌다.
CLAMP_Y = AXLE_Y + WM.CLAMP_Y
if SKID:
    # 두 파일의 좌표계가 어긋나면 여기서 즉시 잡힌다 — 이게 이식의 유일한 접합면이다.
    assert abs((AXLE_Y + WM.RAIL_Y) - RAIL_Y) < 0.01, \
        f"레일 y 가 어긋난다: wheel_mount {AXLE_Y+WM.RAIL_Y:.1f} vs swerve_base {RAIL_Y}"
    assert MOUNT_LOW or (abs(WM.RAIL_Z - RAIL_Z) < 0.01), \
        f"레일 높이가 어긋난다: WM {WM.RAIL_Z} vs {RAIL_Z}"
    assert abs(WM.AXLE_Z - AXLE_Z) < 0.01, f"축 높이가 어긋난다: {WM.AXLE_Z} vs {AXLE_Z}"
    assert abs(WM.S - S) < 0.01, f"프로파일 시리즈가 다르다 (WM {WM.S} vs {S})"

SHELL_TOP = DECK_Z + FLANGE_T             # 셸 윗면 = 플랜지가 프레임 위에 얹힌 높이
assert SHELL_R - SHELL_T - FLANGE_W > 0, "플랜지가 코너 R 을 먹어버린다"
_fin_x, _fin_y = SHELL_X - SHELL_T - FLANGE_W, SHELL_Y - SHELL_T - FLANGE_W
assert _fin_y < TOP_RAIL_Y < SHELL_Y - SHELL_T, "옆면 플랜지 볼트가 상부사이드 레일을 못 문다"
assert _fin_x < CROSS_X < SHELL_X - SHELL_T, "앞뒤 플랜지 볼트가 상부크로스를 못 문다"

# 앞면 얼굴 배치 — 앞 패널은 z=MODULE_TOP~DECK_Z 다. 여기서 상대 위치로 잡아야
# 모듈 높이가 바뀌어도 눈·입·흡기가 패널 밖으로 밀려나지 않는다.
CAM_Z = DECK_Z - 66                                   # 센서 바 중심 (눈)
LED_Z = MODULE_TOP + 50                               # LED 바 (입)
VENT_Z = CAM_Z - 45                                   # 흡기 그릴 — 밑단은 범퍼가 쓴다

# 조향 한계각은 스톱 핀 두 개 사이 빈 호에서 자동으로 나온다
STEER_LIMIT = (360 - (STOP_A[1] - STOP_A[0]) - STOP_TAB_W) / 2
assert SKID or STEER_LIMIT >= 95, f"스톱이 ±90° 조향을 방해한다 (여유 없이 ±{STEER_LIMIT:.1f}°)"
assert SKID or KP_BORE < KP_D - 8, "킹핀 중공이 너무 커서 벽이 4mm 미만이다"
BUMPER_Z = MODULE_TOP - BUMPER_DZ + BUMPER[1] / 2     # 범퍼 중심 = 셸 밑단에 걸친다
TAIL_Z = DECK_Z + TAIL_DZ
# 카메라 — 센서 바 밑면에서 거꾸로 크래들·마운트 빔 높이를 잡는다
CAM_X = SHELL_X - SHELL_T                             # 바 앞면을 셸 안쪽면에 붙인다
CAM_BAR_BOT = CAM_Z - XT_BAR[1] / 2
CAM_RAIL_Z = CAM_BAR_BOT - CAM_BRK_T - S / 2          # 카메라크로스 빔 중심
CAM_WIN = (XT_BAR[0] + 2 * CAM_WIN_CLR, XT_BAR[1] + 2 * CAM_WIN_CLR, 8)
assert FRAME_TOP < CAM_RAIL_Z - S / 2 and CAM_RAIL_Z + S / 2 < TOP_RAIL_Z - S / 2, \
    "카메라크로스가 기둥 구간을 벗어난다"
assert CAM_Z + CAM_WIN[1] / 2 < DECK_Z - 10 and CAM_Z - CAM_WIN[1] / 2 > LED_Z + 25, \
    "카메라 창이 데크나 LED 와 부딪힌다"
assert VENT_Z - VENT_SLOT[1] / 2 > LED_Z + LED_BACK[1] / 2 and \
    VENT_Z + VENT_SLOT[1] / 2 < CAM_Z - CAM_WIN[1] / 2, "흡기 그릴이 입·눈과 겹친다"
assert BUMPER_Z + BUMPER[1] / 2 < LED_Z - LED_SLOT[1] / 2, "범퍼가 LED 바를 가린다"
assert BUMPER[0] / 2 < SHELL_Y - SHELL_R, "범퍼가 셸 라운드 코너 밖으로 나간다"

# 후면 배치 — 비상정지 몸체가 상부크로스 밑을 지나야 하므로 거기서 내려 잡는다
ESTOP_Z = TOP_RAIL_Z - S / 2 - ESTOP_BODY / 2 - 7
PORT_BAY_Z = MODULE_TOP + 72
assert PORT_BAY_Z + PORT_BAY[1] / 2 + 18 < ESTOP_Z - ESTOP_BODY / 2, \
    "포트 베이가 비상정지와 겹친다"
assert PORT_BAY_Z - PORT_BAY[1] / 2 > MODULE_TOP + 20, "포트 베이가 뒷판 밑단을 벗어난다"
assert SHELL_X - PORT_BAY_D - SHELL_T > CROSS_X + S / 2, "포트 베이 뒷벽이 뒤쪽 프레임과 겹친다"
PORTS = [
    # (이름, 모양, 치수, (y, z)) — 3mm 벽이라 패널마운트 커넥터가 그대로 물린다
    ("충전 커넥터", "round", 16.0, (-70, PORT_BAY_Z + 22)),
    ("메인 전원 스위치", "round", 19.0, (-25, PORT_BAY_Z + 22)),
    ("상태 표시등", "round", 8.0, (20, PORT_BAY_Z + 22)),
    ("이더넷 RJ45", "rect", (19, 15), (-70, PORT_BAY_Z - 20)),
    ("USB-A", "rect", (14, 12), (-25, PORT_BAY_Z - 20)),
    ("HDMI", "rect", (16, 8), (15, PORT_BAY_Z - 20)),
    ("예비 그로밋", "round", 12.0, (60, PORT_BAY_Z - 20)),
]
assert min(abs(y) for y in TAIL_YS) - TAIL[0] / 2 > ESTOP_HEAD / 2 or \
    TAIL_Z - TAIL[1] / 2 > ESTOP_Z + ESTOP_HEAD / 2, "후미등이 비상정지와 겹친다"
HANDLE_Z = RAIL_Z + 4                     # 손잡이 높이 (하부사이드 레일 바로 위)
assert HANDLE_Z - HANDLE_HOLE / 2 > MODULE_TOP + 2, "손잡이 관통공이 셸 밑단을 벗어난다"
# 관통공은 x0 ± SPAN/2 두 곳이다. 손잡이가 분할선을 걸쳐도 구멍만 안 물리면 된다.
for _hx in [x + sx * HANDLE_SPAN / 2 for x in HANDLE_XS for sx in (-1, 1)]:
    assert abs(_hx) - HANDLE_HOLE / 2 > 4, "손잡이 관통공이 셸 분할선을 문다"
assert HANDLE_STANDOFF - HANDLE_BAR / 2 >= 30, "봉과 셸 사이가 좁아 손이 안 들어간다"
ANT_Z = DECK_Z + ANT_DZ                   # 안테나 축 높이 (셸 윗단 바로 밑)
assert ANT_Y + ANT_BASE[0] / 2 < SHELL_Y - SHELL_R, "안테나가 셸 라운드 코너에 걸친다"
assert ANT_Y - ANT_BASE[0] / 2 > max(abs(y) for y in TAIL_YS) + TAIL[0] / 2,     "안테나가 후미등과 겹친다"
MINI_Z = ESTOP_Z                          # 소형 카메라 높이 (앞쪽만 Xtion 과 맞춘다)
_MINI_REAR_Y = 160 if COMPACT_SHELL else 215
_MINI_FRONT_Y = 170 if COMPACT_SHELL else 225
MINI_CAMS = (
    ("후방1", "뒤", -_MINI_REAR_Y, MINI_Z), ("후방2", "뒤", _MINI_REAR_Y, MINI_Z),
    ("전방1", "앞", -_MINI_FRONT_Y, CAM_Z), ("전방2", "앞", _MINI_FRONT_Y, CAM_Z),
    ("좌측", "좌", 120, MINI_Z), ("우측", "우", 120, MINI_Z),
)
# 냉각 — 배기팬은 포트 베이 옆, 뒤쪽 기둥(y=195~)은 피한다. 둘 다 베이 높이 안이어야 한다.
FAN_Z = PORT_BAY_Z
LOUVER_Z = BAY_Z + BAY_H / 2
assert BAY_Z + 5 < FAN_Z - FAN_SIZE / 2 and FAN_Z + FAN_SIZE / 2 < BAY_Z + BAY_H - 5,     "배기팬이 베이 높이를 벗어난다 (베이 공기가 안 빠진다)"
assert min(abs(y) for y in FAN_YS) - FAN_SIZE / 2 > PORT_BAY[0] / 2, "배기팬이 포트 베이와 겹친다"
assert max(abs(y) for y in FAN_YS) + FAN_SIZE / 2 < RAIL_Y - S / 2, "배기팬이 뒤쪽 기둥과 겹친다"
assert BAY_Z < LOUVER_Z - LOUVER[1] / 2 + min(LOUVER_DZ) and     LOUVER_Z + LOUVER[1] / 2 + max(LOUVER_DZ) < BAY_Z + BAY_H, "측면 루버가 베이 높이를 벗어난다"

_flat_y, _flat_x = SHELL_Y - SHELL_R, SHELL_X - SHELL_R
for _n, _f, _u, _z in MINI_CAMS:          # 라운드 코너에 걸치면 구멍이 찌그러진다
    assert abs(_u) + MINI_BODY[1] / 2 < (_flat_y if _f in "앞뒤" else _flat_x), \
        f"소형카메라 {_n} 가 셸 라운드 코너에 걸친다"

RS = PROFILES[RAIL_SERIES]["size"]
CARGO_SKIRT_Z = SHELL_TOP + PANEL_T + 1          # 스커트 밑단 = 데크 윗면 바로 위 (1mm 그림자선)
CARGO_Z0 = SHELL_TOP + PANEL_T + RS              # 레일 윗면 = 적재함 바닥판
CARGO_Z1 = CARGO_Z0 + CARGO_H
RAIL_Z2 = SHELL_TOP + PANEL_T + RS / 2      # 데크 레일 중심 = 슬롯 높이
CARGO_IN = (SHELL_X - CARGO_T, SHELL_Y - CARGO_T, SHELL_R - CARGO_T)   # 셸 안쪽 윤곽
# 격벽 위치는 **바구니가 정한다**. 눈대중으로 x=230 을 박아뒀던 게 예전 방식인데,
# 바구니를 바꾸면 조용히 어긋난다 (13막 MODULE_TOP 과 같은 실수).
#   뒤 칸 = 크래들(바구니 + 여유 + 리브) + 손 넣는 자리 x2
CARGO_BAY_X = (CARGO_BIN[0] + 2 * (CARGO_BIN_CLR + CARGO_RIB[0])
               + 2 * CARGO_BIN_HAND)                       # 바구니칸 길이
CARGO_BULK_X = -CARGO_IN[0] + CARGO_BAY_X + CARGO_BULK_T / 2
CARGO_BIN_X = (-CARGO_IN[0] + CARGO_BULK_X - CARGO_BULK_T / 2) / 2   # 바구니 중심
CARGO_EQ_X = CARGO_IN[0] - CARGO_BULK_X - CARGO_BULK_T / 2           # 장비베이 깊이
# y 도 막는다. 안 막았더니 뚜껑을 열면 **열린 면적의 58%가 빈 바닥**이었다
# (522x600 개구에 바구니는 416x316). 비가 들이치고 물건이 바구니 옆으로 빠진다.
#   좌우로 남는 폭은 닫힌 수납이 된다 — 배선·배터리를 여기 두면 무게중심도 내려간다.
CARGO_BAY_Y = CARGO_BIN[1] + 2 * (CARGO_BIN_CLR + CARGO_RIB[0]) + 18   # 바구니칸 폭
CARGO_SIDE_Y = CARGO_IN[1] - CARGO_BAY_Y / 2 - CARGO_BULK_T            # 옆 수납 폭
assert CARGO_SIDE_Y > 40, f"옆 수납이 {CARGO_SIDE_Y:.0f}mm 뿐 — 벽 세울 값어치가 없다"
# 힌지·잠금은 **바구니칸 폭 안**이어야 한다. ±200 으로 박아뒀더니 새로 세운
# 옆벽(±175)을 뚫었다 (힌지 1241 / 스트라이커 600 mm3 x2).
CARGO_HINGE_Y = (-(CARGO_BAY_Y / 2 - 45), CARGO_BAY_Y / 2 - 45)
# 잠금은 바구니 **옆**에 못 둔다 — 칸이 350 인데 바구니가 316 이라 17mm 뿐이다.
# 옆 수납으로 뺐더니 몸통과 래치가 벽으로 갈려 **한 부품이 2조각**이 됐다.
# → **바구니 뒤쪽**에 둔다. 거기는 x 로 53mm 가 비고, 손잡이 홈(±125) 바깥 y 도 빈다.
CARGO_LOCK_X = -CARGO_IN[0] + CARGO_LOCK[0] / 2 + 5
CARGO_LOCK_Y = CARGO_GRIP[0] / 2 + CARGO_T + CARGO_LOCK[1] / 2 + 5
assert CARGO_LOCK_Y + CARGO_LOCK[1] / 2 < CARGO_BAY_Y / 2 - 5, "잠금이 바구니칸 옆벽을 뚫는다"
assert CARGO_EQ_X > 150, f"장비 베이가 {CARGO_EQ_X:.0f}mm 뿐이다 — 바구니를 줄여라"
CARGO_LID_Z = CARGO_Z1 - CARGO_SHOULDER - CARGO_LID_LIP    # 뚜껑 립 밑면
CARGO_HINGE_AT = (CARGO_BULK_X - 18, CARGO_LID_Z - 12)     # 힌지 축 (x, z) — 격벽 위
# 밀어 끼우는 동안 스커트 안쪽 폭이 레일 폭보다 항상 넓어야 한다
if CARGO:
    assert CARGO_IN[1] - (SHELL_R - (SHELL_R ** 2 - SHELL_R ** 2) ** 0.5) > max(RAIL_YS) + RS / 2, \
        "스커트가 레일보다 좁아 밀어 끼울 수 없다"
    assert RAIL_X1 + CARGO_T + 2 < SHELL_X - CARGO_T, \
        "레일 앞끝이 적재함 앞벽에 닿는다"

# ---- 프레임 ----
BEAMS = (
    [(f"하부사이드{i+1}", BASE_L, (-BASE_L / 2, sy * RAIL_Y, RAIL_Z), "X")
     for i, sy in enumerate((-1, 1))]
    + [(f"하부크로스{i+1}", CROSS_L, (x, -CROSS_L / 2, RAIL_Z), "Y")
       for i, x in enumerate(POST_X)]
    + [(f"기둥{i+1}", POST_H, (x, sy * RAIL_Y, FRAME_TOP), "Z")
       for i, (x, sy) in enumerate([(x, sy) for x in POST_X for sy in (-1, 1)])]
    + [(f"상부크로스{i+1}", TOP_CROSS_L, (x, -TOP_CROSS_L / 2, TOP_RAIL_Z), "Y")
       for i, x in enumerate(POST_X)]
    + [(f"상부사이드{i+1}", BASE_L, (-BASE_L / 2, sy * TOP_RAIL_Y, TOP_RAIL_Z), "X")
       for i, sy in enumerate((-1, 1))]
    # 카메라크로스는 Xtion 크래들 받침 하나뿐이다. 첫 제작에서는 빼고, 카메라를
    # 달 때 390 한 본 + 브라켓 2개만 추가하면 된다 (CAM_RAIL=1).
    + ([("카메라크로스", CROSS_L, (CROSS_X, -CROSS_L / 2, CAM_RAIL_Z), "Y")]
       if (FACE and CAM_RAIL) else [])
    # 스키드는 축이 x=±210 이라 끝단 크로스(±365)와 어긋난다. 브라켓 바로 위에 하나 더
    # 넣어 바퀴 하중이 사이드 레일 단일 스팬을 굽히지 않게 한다.
    # 축 바로 위(x=±AXLE_X)에 두었더니 휠브라켓과 같은 자리를 다퉈 코너 브라켓을
    # 걸 면이 네 방향 다 막혔다. 55mm 비켜도 하중은 레일이 받아 처짐 차이가 없다.
    + ([(f"축크로스{i+1}", CROSS_L, (sx * (AXLE_X + 55), -CROSS_L / 2, RAIL_Z), "Y")
        for i, sx in enumerate((-1, 1))] if SKID else [])
    # 휠포스트(3030 x4)는 없앴다. ㄷ자 마운트가 레일을 감싸 축까지 한 덩이로 내려오므로
    # 프로파일도, 그 끝단 M8 탭도, 포스트-레일 코너 브라켓도 필요가 없다.
)
frame = []
for n, l, st, ax in BEAMS:
    b = bar(SERIES, l, st, ax)
    b.label = n
    frame.append(b)


# ---- 조인트 철물 ----
# 지금까지 모델엔 **구멍만** 있고 브라켓 실물이 없었다. 그러면 두 가지를 못 본다:
#   (1) 브라켓이 베이 안쪽 공간을 얼마나 먹는지 (전장 배치와 부딪힌다)
#   (2) 수량이 모델에서 안 나온다 — 손으로 세면 틀린다
# 조인트는 BEAMS 에서 자동으로 찾는다: 빔 A 의 **끝면**이 다른 축의 빔 B 몸통에
# 닿으면 맞댐 조인트다. 손으로 목록을 유지하면 프레임을 고칠 때마다 어긋난다.
BRK = dict(leg=30, w=25, t=4, hole=6.6, inset=12)   # 3025 (30x30 면, 25 폭)
# BRK=inner → 이너 브라켓 306. 슬롯 **안**에 들어가 프로파일 밖으로 안 나온다.
# 값은 두 배(770 vs 360)지만 셸·판재·다른 브라켓과의 간섭이 원천적으로 없다.
BRK_TYPE = os.environ.get("BRK", "outer")
assert BRK_TYPE in ("outer", "inner"), f"BRK 는 outer 또는 inner (받은 값: {BRK_TYPE})"
BRK_IN = dict(leg=30, w=14, d=9)      # 슬롯 안 L (챔버 16.5 안에 들어가는 단면)
WALL = PROFILES[SERIES]["wall"]       # 슬롯 바깥벽 두께 — 이만큼 파고들어 간다


def _beam_box(l, st, ax):
    x, y, z = st
    h = S / 2
    return {"X": ((x, y - h, z - h), (x + l, y + h, z + h)),
            "Y": ((x - h, y, z - h), (x + h, y + l, z + h)),
            "Z": ((x - h, y - h, z), (x + h, y + h, z + l))}[ax]


def joints():
    """(빔A, 빔B, 모서리점, A축, 제3축, A방향, 제3축부호) 목록."""
    boxes = [(n, ax) + _beam_box(l, st, ax) for n, l, st, ax in BEAMS]
    out = []
    for i, (n, l, st, ax) in enumerate(BEAMS):
        lo, hi = _beam_box(l, st, ax)
        k = "XYZ".index(ax)
        for end, sgn in ((lo[k], 1), (hi[k], -1)):     # sgn = 빔이 뻗는 방향
            pt = [(lo[m] + hi[m]) / 2 for m in range(3)]
            pt[k] = end
            for j, (n2, ax2, lo2, hi2) in enumerate(boxes):
                if j == i or ax2 == ax:
                    continue
                if all(lo2[m] - 0.6 <= pt[m] <= hi2[m] + 0.6 for m in range(3)):
                    out.append((n, n2, tuple(pt), ax, sgn))
                    break
    return out


def corner_bracket():
    """3025 코너 브라켓 — 로컬: 모서리가 원점, 다리가 +X(빔A) / +Z(빔B), 폭이 +Y."""
    L, W, T, H, I = BRK["leg"], BRK["w"], BRK["t"], BRK["hole"], BRK["inset"]
    with BuildPart() as p:
        with Locations((L / 2, 0, T / 2)):
            Box(L, W, T)
        with Locations((T / 2, 0, L / 2)):
            Box(T, W, L)
        with Locations((L - I, 0, T / 2)):
            Cylinder(H / 2, T + 4, mode=Mode.SUBTRACT)
        with Locations((T / 2, 0, L - I)):
            Cylinder(H / 2, T + 4, rotation=(0, 90, 0), mode=Mode.SUBTRACT)
    return p.part


def inner_bracket():
    """이너 브라켓 306 — 로컬: 모서리가 원점, 다리가 +X / +Z, 폭이 +Y.

    바깥 브라켓과 달리 **슬롯 챔버 안에 잠긴다.** 그래서 프로파일 겉면 밖으로
    아무것도 안 나오고, 셸·판재·다른 브라켓과 부딪힐 여지가 없다.
    """
    L, W, D = BRK_IN["leg"], BRK_IN["w"], BRK_IN["d"]
    with BuildPart() as p:
        with Locations((L / 2, 0, D / 2)):
            Box(L, W, D)
        with Locations((D / 2, 0, L / 2)):
            Box(D, W, L)
    return p.part


U = {"X": (1, 0, 0), "Y": (0, 1, 0), "Z": (0, 0, 1)}


def _brk_box(pt, leg_a, leg_b):
    """그 자리에 브라켓을 놨을 때 차지하는 박스 (해석적으로 — 솔리드를 안 만든다)."""
    L, W = BRK["leg"], BRK["w"]
    wd = [1 - abs(leg_a[m]) - abs(leg_b[m]) for m in range(3)]      # 남는 축 = 폭
    lo, hi = [], []
    for m in range(3):
        c = pt[m] + leg_b[m] * S / 2
        vals = [c, c + leg_a[m] * L, c + leg_b[m] * L]
        lo.append(min(vals) - wd[m] * W / 2)
        hi.append(max(vals) + wd[m] * W / 2)
    return lo, hi


def _ovl(a, b):
    lo1, hi1 = a
    lo2, hi2 = b
    v = 1.0
    for m in range(3):
        d = min(hi1[m], hi2[m]) - max(lo1[m], lo2[m])
        if d <= 0.5:
            return 0.0
        v *= d
    return v


def brackets(obstacles=()):
    """조인트마다 브라켓을 놓되, **네 면 중 빈 자리**를 고른다.

    빔 A 축에 수직인 면은 4개다. 처음엔 '제3축 + 방향' 하나만 썼다가 절반이
    판재·셸 자리로 나갔고, 그 다음 '제3축 중 안쪽'으로 고쳐도 12건이 남았다.
    (상부크로스는 위가 데크, 아래가 기둥이라 제3축 두 방향이 **둘 다** 막혀 있다.)
    → 네 방향을 다 놓아 보고 장애물과 겹치는 부피가 가장 작은 자리를 고른다.
    """
    inner = BRK_TYPE == "inner"
    BR = inner_bracket() if inner else corner_bracket()
    out, worst = [], []
    for i, (na, nb, pt, ax, sgn) in enumerate(joints()):
        leg_a = tuple(sgn * v for v in U[ax])
        obs = [o for o in obstacles if o[0] not in (na, nb)]
        best = None
        for d in "XYZ":
            if d == ax:
                continue
            for ds in (-1, 1):
                leg_b = tuple(ds * v for v in U[d])
                box = _brk_box(pt, leg_a, leg_b)
                score = sum(_ovl(box, (o[1], o[2])) for o in obs)
                # 동점 규칙 — 셸·범퍼·카메라는 bbox 로 걸러낼 수 없어서(속 빈 상자)
                # 여기서 방향으로 피한다. 처음엔 '무조건 -' 로 뒀다가 뒤쪽 기둥이
                # 차 바깥(-X)을 향해 후방 셸·범퍼·카메라와 12건 물렸다.
                #   Z 면 → 아래 (판재가 프레임 위에 얹힌다)
                #   X/Y 면 → 원점 쪽 (셸은 바깥에 있다)
                m2 = "XYZ".index(d)
                tie = (0 if ds < 0 else 1) if d == "Z"                     else (0 if ds * pt[m2] <= 0 else 1)
                if best is None or (score, tie) < (best[0], best[1]):
                    best = (score, tie, d, ds, leg_b)
        score, _, d, ds, leg_b = best
        if score > 100:
            worst.append((f"브라켓{i+1}_{na}", score))
        if inner:
            off = S / 2 - WALL - BRK_IN["d"]
            corner = tuple(pt[m] + leg_b[m] * off - leg_a[m] * (WALL + BRK_IN["d"])
                           for m in range(3))
        else:
            corner = tuple(pt[m] + leg_b[m] * S / 2 for m in range(3))
        b = Plane(origin=corner, x_dir=leg_a, z_dir=leg_b).location * BR
        b.label = f"브라켓{i+1}_{na}"
        out.append(b)
    if worst:
        print(f"  ! 빈 면을 못 찾은 브라켓 {len(worst)}개: "
              + ", ".join(f"{n}({v:.0f})" for n, v in worst[:5]))
    return out


# ---- 스웨어브 모듈 ----
def wheel():
    """호버보드 6.5인치 허브모터 (인휠 구동, 별도 모터 없음).

    STEP 원본은 축이 X 이고 축이 -X 쪽으로 51mm 나와 있다.
    로봇 좌표(축 = Y, 요크가 -Y 쪽)로 돌리고, **타이어 중심면**을 y=0 에 맞춘다.
    (몸통 중심이 아니라 접지면 중심이어야 킹핀 스크럽이 0 이 된다)
    """
    if not HUB_REAL:
        with BuildPart() as p:
            # 몸통 — 실측 구간(STEP X 5~60) 을 그대로. 타이어 중심면이 y=0 이다.
            y0, y1 = 5 - _TIRE_C, 60 - _TIRE_C
            with Locations((0, (y0 + y1) / 2, 0)):
                Cylinder(WHEEL_D / 2, y1 - y0, rotation=(90, 0, 0))
            fillet(p.edges().filter_by(GeomType.CIRCLE), radius=6)
            # 축까지 넣어야 브라켓이 허공에 뜬 그림이 안 나온다 (목업만 보고 넘어가면
            # 브라켓이 축을 안 잡는 걸 못 본다). D컷도 실물과 같은 +Z 쪽으로 깎는다.
            s0, s1 = HUB_SHAFT_END_X - 1 - _TIRE_C, -3.0 - _TIRE_C
            with Locations((0, (s0 + s1) / 2, 0)):
                Cylinder(HUB_SHAFT_D / 2, s1 - s0, rotation=(90, 0, 0))
            _o = HUB_FLAT - HUB_SHAFT_D / 2
            with Locations((0, (s0 + s1) / 2, _o + HUB_SHAFT_D / 2)):
                Box(HUB_SHAFT_D + 2, s1 - s0, HUB_SHAFT_D, mode=Mode.SUBTRACT)
        return p.part
    hub = import_step(str(HUB_STEP))
    b = hub.bounding_box()
    assert abs(b.size.Y - WHEEL_D) < 1.5, f"허브모터 STEP 외경이 실측과 다르다 {b.size}"
    assert abs(b.min.X - (HUB_SHAFT_END_X - 1)) < 1 and abs(b.max.X - 60) < 1, \
        f"허브모터 축 방향 실측이 다르다 (X {b.min.X:.1f}~{b.max.X:.1f})"
    if HUB_FILE_UNDERCUT:
        # 나사쪽 언더컷을 D컷 평면까지 밀어낸 상태로 그린다 (실물은 평줄 작업).
        # 이걸 안 하면 판이 축에 안 들어가는데, 모델에서는 그게 안 보인다.
        x0, x1 = HUB_UNDERCUT_X
        off = HUB_FLAT - HUB_SHAFT_D / 2                 # 평면 z = +5
        with BuildPart() as p:
            add(hub)
            with Locations(((x0 + x1) / 2, 0, off + HUB_SHAFT_D / 2)):
                Box(x1 - x0, HUB_SHAFT_D + 4, HUB_SHAFT_D, mode=Mode.SUBTRACT)
        hub = p.part
    return Rot(0, 0, 90) * Pos(-_TIRE_C, 0, 0) * hub


# ---- 고정 브라켓 (DRIVE=skid) ----
def _shaft_cut(yc, t, flat=True):
    """축 구멍 — Ø16 원형, flat 이면 D컷 평면까지 (+Z 쪽. STEP 실측 Z -8~+5).

    원형만 파면 구동 토크 29 N·m 에 축이 구멍 안에서 돈다. 한쪽을 13mm 에서 잘라
    평면을 되살린다. 딱 Ø16 으로 파면 끼움 틈이 0 이라 불리언이 슬라이버를 남기고
    (실제로 축x브라켓 37mm3 간섭으로 잡혔다) 실물로도 안 들어간다 → BR_FIT 만큼 키운다.
    flat=False 는 평면이 없는 축 구간에 앉는 판용 — 평면으로 파면 거기서 간섭난다.
    """
    r = HUB_SHAFT_D / 2 + BR_FIT / 2
    with Locations((0, yc, AXLE_Z)):
        Cylinder(r, t + 4, rotation=(90, 0, 0), mode=Mode.SUBTRACT)
    if not flat:
        return
    off = HUB_FLAT - HUB_SHAFT_D / 2 + BR_FIT / 2        # 축 중심 → 평면 = +5 (+여유)
    with Locations((0, yc, AXLE_Z + (off + r) / 2)):
        Box(2 * r, t, r - off)


def wheel_bracket():
    """허브모터 고정 브라켓 (+Y 쪽 기준. 원점 = 타이어 중심면 · 지면).

    축이 한쪽으로만 나오는 외팔보라서 판 하나로는 구멍 안에서 축이 기운다.
    판 두 장을 BR_SPAN 만큼 벌려 세워 굽힘을 두 판의 couple 로 나눈다.

    **아래로 열린 U + 하부 캡**이다. 처음엔 닫힌 D보어 + 축 너트 예압으로 잡으려 했는데,
    이 허브모터 축은 **나사부가 아예 없다** (Ø16 D컷이 1mm 모따기로 그냥 끝난다 —
    0.1mm 스캔으로 확인). 너트를 걸 데가 없으니 클램프로 무는 수밖에 없다.
    가르는 면은 축 중심이다. 바퀴 무게(218 N)는 **위쪽 판**이 받고, 캡은 축이 빠지지
    않게 아래에서 받쳐 볼트로 당긴다 (볼트당 109 N). D컷 평면이 +Z 라 **회전 방지는
    판이 직접** 맡는다. 그리고 축방향으로 밀어 끼울 필요가 없어져서, 축을 줄로
    미는 작업도 사라졌다.

    프레임에는 두 곳에서 물린다. 한 줄만으로는 x 축 둘레 모멘트를 못 받는다.
      · 상단 타이판 → 하부 사이드레일 **밑면** 슬롯 (M6 T너트 2)
      · 옆면 플랜지 → 같은 레일 **바깥면** 슬롯 (M6 T너트 2)
    레일을 타고 미끄러지므로 휠베이스를 실물에서 바꿔 시험할 수 있다.

    실제로는 레이저 절단 알루미늄 3종(클램프판 x2, 타이판+플랜지)을 볼트로 조립한다.
    여기서는 한 솔리드로 융합해 둔다.
    """
    top = MODULE_TOP                     # 레일 밑면 = 차체 바닥
    y_in, y_out = min(BR_YS), max(BR_YS)
    rail_out = RAIL_Y + S / 2 - AXLE_Y   # 레일 바깥면 (타이어 중심면 기준, 음수)
    with BuildPart() as p:
        for yc in BR_YS:                             # 클램프 판 2장
            # 다리는 축 중심에서 레일 밑면까지. 축 아래는 캡이 맡는다.
            # (처음엔 top-BR_T 에서 끊고 타이판을 얹어 머리에 단차가 생겼었다.)
            with Locations((0, yc, (AXLE_Z + top) / 2)):
                Box(BR_W, BR_T, top - AXLE_Z)
        # 상단 타이판 — 레일 밑면에 붙고 바깥면 플랜지까지 넘어가 겹친다.
        # 딱 맞대면 선접촉이라 솔리드가 안 붙는다 (BR_T 만큼 물려야 한다).
        with Locations((0, (y_in + rail_out + BR_T) / 2, top - BR_T / 2)):
            Box(BR_W, rail_out + BR_T - y_in, BR_T)
        with Locations((0, rail_out + BR_T / 2, top + (S - BR_T) / 2)):  # 레일 바깥면 플랜지
            Box(BR_W, BR_T, S + BR_T)
        # 언더컷을 밀어낸 축이라 **두 판 모두 D컷**이다 → 토크를 둘이 나눠 받는다.
        # (밀어내지 않으면 안쪽 판이 원형 구간에 걸려 원형 보어여야 하고, 그러면
        #  바깥 판 혼자 29 N·m 을 받는다.)
        for yc in BR_YS:
            _shaft_cut(yc, BR_T, flat=HUB_FILE_UNDERCUT)
        # 판 사이 브리지 — D컷 평면 위에 얹혀 캡 볼트를 받는다. 두 역할이다:
        #   (1) M6 탭이 앉을 살 (판 모서리엔 1.5mm 뿐이라 못 앉는다)
        #   (2) 평면에 직접 눌러 회전 방지를 한 번 더 잡는다
        _off = HUB_FLAT - HUB_SHAFT_D / 2 + BR_FIT / 2
        with Locations((0, BRIDGE_Y, AXLE_Z + _off + BRIDGE_Z / 2)):
            Box(BR_W, BRIDGE_W, BRIDGE_Z)
        for x in (-CAP_BOLT_X, CAP_BOLT_X):          # 캡 볼트 하공 (M6 탭)
            with Locations((x, BRIDGE_Y, AXLE_Z + _off + BRIDGE_Z / 2)):
                Cylinder(CAP_BOLT[1] / 2, BRIDGE_Z + 2, mode=Mode.SUBTRACT)
        for x in (-20, 20):                          # 밑면 슬롯 M6
            with Locations((x, RAIL_Y - AXLE_Y, top - BR_T / 2)):
                Cylinder(3.3, BR_T + 2, mode=Mode.SUBTRACT)
        for x in (-20, 20):                          # 바깥면 슬롯 M6
            with Locations((x, rail_out + BR_T / 2, top + S / 2)):
                Cylinder(3.3, BR_T + 2, rotation=(90, 0, 0), mode=Mode.SUBTRACT)
    assert len(p.part.solids()) == 1, "브라켓이 끊겼다 (판이 타이판에 안 닿는다)"
    return p.part


def axle_cap():
    """축 하부 클램프 캡 — 아래에서 축을 받치고 위로 볼트 4개로 당겨 문다.

    판 두 장에 걸쳐 하나로 간다. 그러면 판 사이 빈 구간(BR_MID)까지 캡이 축을
    감싸서, 무는 길이가 판 두께 합(16)이 아니라 **{:.0f}mm** 가 된다.
    """
    y0, y1 = min(BR_YS) - BR_T / 2, max(BR_YS) + BR_T / 2
    top = AXLE_Z - CAP_GAP                              # 판 밑면보다 CAP_GAP 낮다
    with BuildPart() as p:
        with Locations((0, (y0 + y1) / 2, top - CAP_H / 2)):
            Box(BR_W, y1 - y0, CAP_H)
        # 시트는 축 반경에 거의 붙인다. 판 보어(BR_FIT)처럼 헐거우면 조여도 안 문다.
        with Locations((0, (y0 + y1) / 2, AXLE_Z)):
            Cylinder(HUB_SHAFT_D / 2 + CAP_FIT / 2, y1 - y0 + 4,
                     rotation=(90, 0, 0), mode=Mode.SUBTRACT)
        for x in (-CAP_BOLT_X, CAP_BOLT_X):
            with Locations((x, BRIDGE_Y, top - CAP_H / 2)):
                Cylinder(CAP_BOLT[0] / 2, CAP_H + 4, mode=Mode.SUBTRACT)
    p.part.label = "축캡"
    return [p.part]


def axle_spacer():
    """위치 스페이서 — 허브 숄더와 바깥 판 사이. 바퀴의 y 위치를 정한다.

    클램프가 마찰로 잡긴 하지만, 조립할 때 '숄더가 여기 닿으면 끝' 이라는 기준이
    없으면 코너마다 윤거가 달라진다.
    """
    with BuildPart() as p:
        with Locations((0, BR_SHOULDER_Y - BR_OUT_SLEEVE / 2, AXLE_Z)):
            Cylinder(BR_SLEEVE / 2, BR_OUT_SLEEVE, rotation=(90, 0, 0))
            Cylinder(HUB_SHAFT_D / 2 + BR_FIT / 2, BR_OUT_SLEEVE + 4,
                     rotation=(90, 0, 0), mode=Mode.SUBTRACT)
    p.part.label = "위치스페이서"
    return [p.part]



def fixed_wheels():
    """4륜 고정 — 바퀴 + 휠마운트 + 기존 클램프. 축은 항상 차체 안쪽을 향한다.

    레일에서 축까지 91.35mm 를 내려주는 일은 **ㄷ자 휠마운트 한 덩이**가 한다
    (wheel_mount.py 가 원본). 클램프는 호버보드에서 떼어낸 실물 STEP 그대로다 —
    목업이 아니다. 채널 깊이 5.07 이 D컷 평면 높이와 같아서 채널이 평면을 문다.

    옛 판재 브라켓 6종은 wheel_bracket()/axle_cap() 에 남겨 뒀다 — CNC 견적
    비교용이고 조립에는 안 쓴다.
    """
    W = wheel()
    if MOUNT_LOW:                       # 하단 클램프 — 레일이 축 바로 위
        MN, CL = WM.mount_low(), Pos(0, WM.LOW_CY - WM.CLAMP_Y, 0) * WM.clamp()
    else:                               # ㄷ자 마운트 — 레일에서 91mm 내려온다
        MN, CL = WM.mount(), WM.clamp()
    wheels, mounts, clamps = [], [], []
    for i, (x, y) in enumerate(KP_PTS):
        # +Y 쪽 기준으로 만들었으니 -Y 쪽은 z 축 180° 회전 (미러 대신 — 좌우 손대칭 유지)
        R = Rot(0, 0, 180) if y < 0 else Location()
        w = Pos(x, y, AXLE_Z) * R * W
        w.label = f"바퀴{i+1}"
        wheels.append(w)
        m = Pos(x, y, 0) * R * MN.moved(Location())
        m.label = f"휠마운트{i+1}"
        mounts.append(m)
        c = Pos(x, y, 0) * R * CL.moved(Location())
        c.label = f"휠클램프{i+1}"
        clamps.append(c)
    return wheels, mounts, clamps


def _y_plate():
    """요크·암 판의 Y 위치 (킹핀=바퀴 중심면 기준 안쪽)."""
    return -(WHEEL_W / 2 + YOKE_GAP + YOKE_T / 2)


def yoke_upper():
    """모듈 고정부 — 킹핀축에 물리고 피벗과 쇼크 상단을 잡는다 (킹핀이 원점).

    여기까지가 '차체에 매달리는 쪽'이다. 아래 스윙암만 위아래로 움직인다.
    """
    yp = _y_plate()
    px, pz = SUS_PIVOT
    arm_z = YOKE_ARM_Z
    with BuildPart() as p:
        with Locations((0, yp / 2, arm_z)):                  # 킹핀까지 건너가는 수평 팔
            Box(110, abs(yp) + 40, YOKE_T)
        z0, z1 = pz - 20, arm_z + YOKE_T / 2          # 모듈 상한(MODULE_TOP) 안에 있어야 한다
        with Locations(((px + 20) / 2, yp, (z0 + z1) / 2)):      # 수직판 (피벗까지)
            Box(abs(px) + 60, YOKE_T, z1 - z0)
        # 피벗 보스 — 요크판과 그 바깥 스윙암까지 덮되, 안쪽 끝이 바퀴 폭(±WHEEL_W/2)
        # 안으로 들어가면 안 된다. 처음엔 yp 중심 30 길이로 두었다가 5mm 파고들었다.
        boss_in = -(WHEEL_W / 2 + 2)
        boss_out = yp - YOKE_T - YOKE_T / 2 - 2
        with Locations((px, (boss_in + boss_out) / 2, pz)):
            Cylinder(PIVOT_D / 2 + 8, boss_in - boss_out, rotation=(90, 0, 0))
            Cylinder(PIVOT_D / 2, 40, rotation=(90, 0, 0), mode=Mode.SUBTRACT)
        with Locations((SHOCK_TOP[0], yp + SHOCK_Y, SHOCK_TOP[1])):   # 쇼크 상단 마운트
            Cylinder(12, 14, rotation=(90, 0, 0))
        with Locations((SHOCK_BOT[0] - 25, yp, SUS_PIVOT[1] - 30)):   # 범프스톱 자리
            Cylinder(BUMP[0] / 2, BUMP[1])
    return p.part


def swing_arm():
    """스윙암 — 피벗에서 바퀴축까지. 위아래로 SUS_TRAVEL 만큼 움직인다."""
    yp = _y_plate()
    px, pz = SUS_PIVOT
    with BuildPart() as p:
        with BuildSketch(Plane.XZ.offset(-(yp - YOKE_T / 2))) as sk:
            with Locations((px, pz), (0, AXLE_Z), (SHOCK_BOT[0], SHOCK_BOT[1])):
                Circle(16)
            make_hull()
        extrude(amount=YOKE_T)
        with Locations((px, yp, pz)):
            Cylinder(PIVOT_D / 2, 40, rotation=(90, 0, 0), mode=Mode.SUBTRACT)
        # 바퀴축 관통공 — 허브모터는 축이 **고정자**다. 원형 구멍이면 구동토크
        # 21~29 N·m 에 미끄러진다. STEP 실측대로 Ø16 D컷(13) 으로 판다.
        # Ø16 D컷 (브라켓과 같은 함수). 두께·중심을 판 실제 값으로 줘야 한다 —
        # 40 을 넘기면 평면 복원 박스가 판 밖 32mm 까지 뻗어 바퀴를 문다 (86/298 mm3).
        _shaft_cut(yp - YOKE_T, YOKE_T)
        with Locations((SHOCK_BOT[0], yp + SHOCK_Y, SHOCK_BOT[1])):   # 쇼크 하단 마운트
            Cylinder(10, 14, rotation=(90, 0, 0))
    return p.part


def shock():
    """코일오버 — 몸통 + 로드 + 스프링. 상·하 마운트를 잇는다."""
    yp = _y_plate() + SHOCK_Y
    (tx, tz), (bx, bz) = SHOCK_TOP, SHOCK_BOT
    L = ((tx - bx) ** 2 + (tz - bz) ** 2) ** 0.5
    ang = math.degrees(math.atan2(tx - bx, tz - bz))         # Z 에서 기운 각도
    with BuildPart() as p:
        with Locations(Location((bx + (tx - bx) / 2, yp, bz + (tz - bz) / 2),
                                (0, ang, 0))):
            Cylinder(SHOCK_D / 2, L * 0.6)                   # 스프링/몸통
            Cylinder(6, L)                                   # 로드
    return p.part


def kingpin_shaft():
    """조향축 + 종동 풀리 — 요크와 함께 돈다 (킹핀이 원점)."""
    z0 = YOKE_ARM_Z - YOKE_T / 2           # 요크 팔을 관통해 물린다 (배선이 여기로 들어온다)
    z1 = FRAME_TOP + 30 + BELT_W / 2      # 종동 풀리 윗면에서 끝난다 (모터판을 안 뚫는다)
    with BuildPart() as p:
        with Locations((0, 0, (z0 + z1) / 2)):
            Cylinder(KP_D / 2, z1 - z0)
        with Locations((0, 0, FRAME_TOP + 30)):
            Cylinder(STEER_PULLEY[1] * 5 / 3.1416 / 2, BELT_W)  # 종동 풀리 48T
        # 칼라는 베어링 바깥쪽에 붙어 축이 위아래로 빠지는 걸 막는다
        z_col = (MODULE_TOP + 4 - KP_BW / 2 - KP_COLLAR[1] / 2,
                 FRAME_TOP + 6 + 4 + KP_BW / 2 + KP_COLLAR[1] / 2)
        for z in z_col:
            with Locations((0, 0, z)):
                Cylinder(KP_COLLAR[0] / 2, KP_COLLAR[1])
        # 조향 하드스톱 걸림턱 — **상부** 칼라 밑단에서 뻗는다.
        # 하부 칼라 높이에는 요크 수평팔이 지나가서 핀을 세울 수가 없다.
        # 위로도 벨트(z=300.5~)를 피해야 하므로 6mm 만 쓴다.
        a = math.radians(STOP_TAB_A)
        w = 2 * STOP_R * math.sin(math.radians(STOP_TAB_W / 2))
        z_tab = z_col[1] - KP_COLLAR[1] / 2
        with Locations(Location(((STOP_R + 4) / 2 * math.cos(a),
                                 (STOP_R + 4) / 2 * math.sin(a), z_tab + 3),
                                (0, 0, STOP_TAB_A))):
            Box(STOP_R + 4, w, 6)
        with Locations((0, KP_COLLAR[0] / 2 - HOME_MAG[1] / 2, z_col[0])):
            Cylinder(HOME_MAG[0] / 2, HOME_MAG[1], rotation=(90, 0, 0))   # 원점 자석
        # 배선 통로 — 인휠모터 3상+홀 8선이 여기로 지나간다 (마지막에 뚫어야 한다)
        with Locations((0, 0, (z0 + z1) / 2)):
            Cylinder(KP_BORE / 2, z1 - z0 + 40, mode=Mode.SUBTRACT)
    return p.part


def kingpin_bearings():
    """킹핀 베어링 2개 (6005). 브라켓 보어에 압입되고 축을 잡는다.

    이게 없으면 축이 보어 안에서 11mm 놀아 바퀴가 앞뒤좌우로 흔들린다.
    두 개를 32mm 벌려 주행·조향 반력의 모멘트를 받는다.
    """
    with BuildPart() as p:
        for z in (MODULE_TOP + 4, FRAME_TOP + 6 + 4):
            with Locations((0, 0, z)):
                Cylinder(KP_BEAR / 2, KP_BW)
                Cylinder(KP_D / 2, KP_BW + 2, mode=Mode.SUBTRACT)
    return p.part


def steer_belt():
    """조향 타이밍벨트 — 두 풀리를 감는 폐루프 (킹핀이 원점)."""
    r2 = STEER_PULLEY[1] * 5 / 3.1416 / 2
    r1 = STEER_PULLEY[0] * 5 / 3.1416 / 2
    with BuildPart() as p:
        with BuildSketch(Plane.XY.offset(FRAME_TOP + 30 - BELT_W / 2)) as sk:
            with Locations((0, 0)):
                Circle(r2 + BELT_T)
            with Locations((-STEER_OFFSET, 0)):
                Circle(r1 + BELT_T)
            make_hull()
            with Locations((0, 0)):
                Circle(r2, mode=Mode.SUBTRACT)
            with Locations((-STEER_OFFSET, 0)):
                Circle(r1, mode=Mode.SUBTRACT)
        extrude(amount=BELT_W)
    return p.part


def kingpin_mount():
    """킹핀 + 조향모터를 함께 잡는 브라켓 (+Y 쪽, 킹핀이 원점).

    ㄷ자로 사이드 레일 바깥면에 물리고, 상·하 두 판이 베어링을 잡는다.
    거기서 라이저 2장으로 올려 세워 **조향모터까지 같은 브라켓 위에 얹는다.**
    모터를 상부 프레임에 따로 달면 조립 오차·열변형으로 벨트 센터거리가 흔들린다.
    한 판 위에 있어야 센터거리 70mm 가 고정된다.

    하중 경로: 프레임 → 이 브라켓 → 베어링 → 킹핀축 → 요크 → 바퀴
    """
    y_rail = -(KINGPIN_Y - RAIL_Y - S / 2)              # 레일 바깥면 (킹핀 기준)
    t = 8
    y_out = KP_BEAR / 2 + 12                            # 베어링 바깥까지
    r2 = STEER_PULLEY[1] * 5 / 3.1416 / 2               # 종동 풀리 반경
    # x0 은 모터 플랜지까지만(길면 셸 라운드 코너를 파고든다), x1 은 벨트·풀리를 피해서
    x0, x1 = -STEER_OFFSET - 35, r2 + BELT_T + 20
    z_bear = (MODULE_TOP + t / 2, FRAME_TOP + 6 + t / 2)
    z_mot = FRAME_TOP + 30 + BELT_W / 2                 # 모터판 밑면 = 벨트 윗면
    with BuildPart() as p:
        # 레일에 붙는 수직판. 밑단은 MODULE_TOP 아래로 내려오면 안 된다 —
        # 요크 수평팔이 바로 그 밑을 지나간다 (모듈 높이를 올렸을 때 여기서 물렸다).
        with Locations((0, y_rail + t / 2, MODULE_TOP + (S + 10) / 2)):
            Box(140, t, S + 10)
        for z in z_bear:                                # 상·하 베어링 판
            with Locations((0, (y_rail + t + y_out) / 2, z)):
                Box(140, y_out - y_rail - t, t)
            with Locations((0, 0, z)):
                Cylinder(KP_BEAR / 2, t + 4, mode=Mode.SUBTRACT)
        # 모터판 + 라이저 2장 (벨트를 피해 X 양 끝에 세운다)
        with Locations(((x0 + x1) / 2, (y_rail + t + y_out) / 2, z_mot + t / 2)):
            Box(x1 - x0, y_out - y_rail - t, t)
        for x in (x0 + 5, x1 - 5):      # 라이저는 벨트 바깥에 붙여 세운다
            with Locations((x, (y_rail + t + y_out) / 2, (z_bear[1] + t / 2 + z_mot) / 2)):
                Box(10, y_out - y_rail - t, z_mot - z_bear[1] - t / 2)
        with Locations((-STEER_OFFSET, 0, z_mot + t / 2)):      # 모터 축 관통공
            Cylinder(20, t + 4, mode=Mode.SUBTRACT)
        with Locations(*[(-STEER_OFFSET + dx, dy, z_mot + t / 2)
                         for dx, dy in ((28, 0), (-28, 0), (0, 28), (0, -28))]):
            Cylinder(5.5 / 2, t + 4, mode=Mode.SUBTRACT)        # 모터 취부 M5 x4
        with Locations(*[(sx * 50, y_rail + t / 2, RAIL_Z) for sx in (-1, 1)]):
            Cylinder(6.6 / 2, t + 4, rotation=(90, 0, 0), mode=Mode.SUBTRACT)   # 레일 볼트
        # 조향 하드스톱 핀 2개 — 상부 베어링 판에서 위로 세워 칼라 걸림턱을 막는다.
        # 전자 리밋이 놓치면 배선이 끊어진다. 최후 방어선이라 기계식이어야 한다.
        for ang in STOP_A:
            a = math.radians(ang)
            with Locations((STOP_R * math.cos(a), STOP_R * math.sin(a),
                            z_bear[1] + t / 2 + STOP_H / 2)):
                Cylinder(STOP_D / 2, STOP_H)
        # 원점 센서 브라켓 — 칼라 자석을 옆에서 읽는다 (축 중앙은 배선이 쓴다)
        z_home = MODULE_TOP + 4 - KP_BW / 2 - KP_COLLAR[1] / 2
        with Locations((0, KP_COLLAR[0] / 2 + 8, (z_home + z_bear[0] - t / 2) / 2)):
            Box(24, 5, z_bear[0] - t / 2 - z_home)
    return p.part


def home_sensor():
    """조향 원점 홀 IC 보드 — 이게 없으면 전원 넣었을 때 바퀴 방향을 모른다."""
    z_home = MODULE_TOP + 4 - KP_BW / 2 - KP_COLLAR[1] / 2
    with BuildPart() as p:
        with Locations((0, KP_COLLAR[0] / 2 + 5 - HOME_PCB[2] / 2, z_home)):
            Box(HOME_PCB[0], HOME_PCB[2], HOME_PCB[1])
    return p.part


def steer_motor():
    """조향 모터 — 킹핀 옆에 세우고 벨트로 2:1 감속 (킹핀이 원점)."""
    with BuildPart() as p:
        # 몸체(Ø45)가 풀리(Ø38)보다 굵어서 벨트를 치지 않게 벨트 위로 올린다
        # 브라켓 모터판(두께 8) 위에 앉는다
        with Locations((-STEER_OFFSET, 0, FRAME_TOP + 30 + BELT_W / 2 + 8 + STEER_L / 2)):
            Cylinder(STEER_D / 2, STEER_L)
        with Locations((-STEER_OFFSET, 0, FRAME_TOP + 30)):
            Cylinder(STEER_PULLEY[0] * 5 / 3.1416 / 2, BELT_W)  # 구동 풀리 24T
    return p.part


def modules():
    """4 코너 스웨어브 모듈.

    회전부(요크·바퀴·킹핀축·베어링)를 하나의 Compound 로 묶고, 그 Compound 의
    원점을 킹핀에 둔다. 그래야 애니메이션에서 rz 한 번으로 통째로 돌아
    바퀴와 요크가 따로 놀 수 없다.
    """
    YU, SA, SH = yoke_upper(), swing_arm(), shock()
    W, K, B = wheel(), kingpin_shaft(), kingpin_bearings()
    MT, SM, BL, HS = kingpin_mount(), steer_motor(), steer_belt(), home_sensor()
    mods, statics = [], {"킹핀브라켓": [], "조향모터": [], "조향벨트": [], "베어링": [],
                         "원점센서": []}
    for i, (x, y) in enumerate(KP_PTS):
        flip = y < 0
        R = Rot(0, 0, 180) if flip else Location()     # 미러 대신 회전 (좌우 손대칭 유지)
        yl = R * YU
        yl.label = f"요크상부{i+1}"
        kl = R * K            # 걸림턱·자석이 생겨서 이제 방향이 있다
        kl.label = f"킹핀축{i+1}"
        # 스윙암·쇼크·바퀴는 서스펜션 피벗이 원점인 서브 Compound 로 묶는다.
        # 그래야 애니메이션에서 ry 한 트랙이 곧 서스펜션 바운스가 된다.
        px, pz = SUS_PIVOT
        to_pivot = Pos(-px, 0, -pz)
        al = to_pivot * SA
        al.label = f"스윙암{i+1}"
        sl = to_pivot * SH
        sl.label = f"쇼크{i+1}"
        wl = to_pivot * Pos(0, 0, AXLE_Z) * W
        wl.label = f"바퀴{i+1}"
        arm = R * (Pos(px, 0, pz) * Compound(children=[al, sl, wl]))
        arm.label = f"서스암{i+1}"
        m = Pos(x, y, 0) * Compound(children=[yl, kl, arm])
        m.label = f"모듈{i+1}"
        mods.append(m)
        for part, key in ((R * MT, "킹핀브라켓"), (R * SM, "조향모터"),
                          (R * BL, "조향벨트"), (B.moved(Location()), "베어링"),
                          (R * HS, "원점센서")):
            o = Pos(x, y, 0) * part
            o.label = f"{key}{i+1}"
            statics[key].append(o)
    return mods, statics


def _face(f):
    """면 이름 → (바깥면 법선, 구멍 중심을 만드는 함수).

    앞/뒤는 X 법선, 좌/우는 Y 법선이라 회전이 다르다. 여기서 한 번만 갈라 두면
    소형 카메라 6개를 면 상관없이 같은 코드로 뚫고 앉힐 수 있다.
    """
    return {
        "앞": ((1, 0, 0), lambda u, z: (SHELL_X, u, z)),
        "뒤": ((-1, 0, 0), lambda u, z: (-SHELL_X, u, z)),
        "좌": ((0, 1, 0), lambda u, z: (u, SHELL_Y, z)),     # 전방 +X 기준 좌측 = +Y
        "우": ((0, -1, 0), lambda u, z: (u, -SHELL_Y, z)),
    }[f]


def bumper_cuts():
    """범퍼 플런저가 셸을 뚫고 들어가는 구멍 (앞뒤 각 2개)."""
    with BuildPart(mode=Mode.PRIVATE) as p:
        for sx in (-1, 1):
            for y in BUMPER_SW_Y:
                with Locations((sx * SHELL_X, y, BUMPER_Z)):
                    Cylinder(BUMPER_PLUNGER / 2, 6 * SHELL_T, rotation=(0, 90, 0))
    return p.part


def mini_cuts():
    """소형 카메라 6개가 셸에 뚫는 구멍 (렌즈공 + 취부 나사 2개) — 솔리드로 돌려준다."""
    with BuildPart(mode=Mode.PRIVATE) as p:
        for _, f, u, z in MINI_CAMS:
            n, at = _face(f)
            rot = (0, 90, 0) if n[0] else (90, 0, 0)
            with Locations(at(u, z)):
                Cylinder(MINI_HOLE / 2, 3 * SHELL_T + MINI_BODY[2], rotation=rot)
            for s in (-1, 1):                       # 취부 나사 — 면 안에서 좌우로
                du = (0, s * MINI_PCD / 2, 0) if n[0] else (s * MINI_PCD / 2, 0, 0)
                with Locations(tuple(a + b for a, b in zip(at(u, z), du))):
                    Cylinder(MINI_SCREW / 2, 4 * SHELL_T, rotation=rot)
    return p.part


def mini_cams():
    """소형 카메라 6개 — 렌즈 배럴은 셸 밖으로 조금 나오고 몸체는 안쪽에 붙는다."""
    ld, ll = MINI_LENS
    out = []
    for name, f, u, z in MINI_CAMS:
        n, at = _face(f)
        rot = (0, 90, 0) if n[0] else (90, 0, 0)
        c = at(u, z)
        def off(d):                                  # 면 바깥(+) / 안쪽(-) 으로 d
            return tuple(a + n[i] * d for i, a in enumerate(c))
        with BuildPart() as p:
            with Locations(off(ll / 2 - 3)):
                Cylinder(ld / 2, ll, rotation=rot)               # 렌즈 배럴
            # MINI_BODY[0] 이 '면 법선 방향 깊이'다. [2] 를 쓰면 3mm 더 들어가
            # 앞뒤 기둥을 물어버린다 (실제로 물렸다).
            with Locations(off(-(SHELL_T + MINI_BODY[0] / 2))):
                Box(*(MINI_BODY if n[0] else
                      (MINI_BODY[1], MINI_BODY[0], MINI_BODY[2])))
        p.part.label = name
        out.append(p.part)
    return out


def port_bay():
    """후면 포트 베이 — 뒤판을 안으로 밀어 넣은 상자. 커넥터가 밖으로 안 튀어나온다.

    베이 뒷벽은 x=-(SHELL_X-PORT_BAY_D) 에 서고, 거기에 패널마운트 커넥터가 물린다.
    뒷벽이 뒤쪽 기둥(x=-380 부터)보다 앞으로 오면 안 된다.
    """
    w, h = PORT_BAY
    x0, x1 = -SHELL_X, -SHELL_X + PORT_BAY_D          # 바깥면 ~ 뒷벽 바깥
    with BuildPart() as p:
        with BuildSketch(Plane.YZ.offset(x0)):
            with Locations((0, PORT_BAY_Z)):
                RectangleRounded(w, h, PORT_BAY_R)
        extrude(amount=PORT_BAY_D + SHELL_T)          # 뒷벽 두께까지
        with BuildSketch(Plane.YZ.offset(x0 - 1)):    # 안쪽 공동 (뒤로 열린다)
            with Locations((0, PORT_BAY_Z)):
                RectangleRounded(w - 2 * SHELL_T, h - 2 * SHELL_T, PORT_BAY_R - SHELL_T)
        extrude(amount=PORT_BAY_D + 1, mode=Mode.SUBTRACT)
        with BuildSketch(Plane.YZ.offset(x1 + SHELL_T / 2)) as sk:   # 커넥터 구멍
            for _, kind, size, pos in PORTS:
                with Locations(pos):
                    if kind == "round":
                        Circle(size / 2)
                    else:
                        RectangleRounded(size[0], size[1], 2)
        extrude(to_extrude=sk.sketch, amount=SHELL_T + 2, both=True, mode=Mode.SUBTRACT)
    return p.part


def rear_cutouts():
    """뒷면에 뚫는 구멍 — 비상정지 + 포트 베이 입구 + 후미등."""
    lw, lh = TAIL
    with BuildSketch(Plane.YZ.offset(-SHELL_X + 20)) as sk:
        with Locations(*[(y, TAIL_Z) for y in TAIL_YS]):
            RectangleRounded(lw, lh, lh / 2 - 0.1)
        with Locations((0, ESTOP_Z)):
            Circle(ESTOP_HOLE / 2)
            with Locations((0, ESTOP_HOLE / 2)):
                Rectangle(*ESTOP_KEY)                  # 회전방지 홈 (22mm 표준)
        with Locations((0, PORT_BAY_Z)):
            RectangleRounded(PORT_BAY[0] - 2 * SHELL_T, PORT_BAY[1] - 2 * SHELL_T,
                             PORT_BAY_R - SHELL_T)
    return sk.sketch


def estop():
    """비상정지 버튼 (22mm 표준) — 머리는 셸 밖, 몸체·접점 블록은 안쪽."""
    x0 = -SHELL_X
    with BuildPart() as p:
        with Locations((x0 - ESTOP_COLLAR_H / 2, 0, ESTOP_Z)):
            Cylinder(ESTOP_COLLAR / 2, ESTOP_COLLAR_H, rotation=(0, 90, 0))
        with Locations((x0 - ESTOP_COLLAR_H - ESTOP_HEAD_H / 2, 0, ESTOP_Z)):
            Cylinder(ESTOP_HEAD / 2, ESTOP_HEAD_H, rotation=(0, 90, 0))
        fillet(p.edges().filter_by(GeomType.CIRCLE).sort_by(Axis.X)[0], radius=6)
        with Locations((x0 + SHELL_T + ESTOP_BODY_L / 2, 0, ESTOP_Z)):
            Cylinder(ESTOP_BODY / 2, ESTOP_BODY_L, rotation=(0, 90, 0))
        with Locations((x0 + SHELL_T + ESTOP_BODY_L, 0, ESTOP_Z)):
            Box(24, 34, 34, align=(Align.MIN, Align.CENTER, Align.CENTER))
    return p.part


def front_cutouts():
    """앞면에 뚫는 구멍 — 카메라 + LED 슬롯 + 흡기 그릴."""
    lw, lh = LED_SLOT
    vw, vh = VENT_SLOT
    with BuildSketch(Plane.YZ.offset(SHELL_X - 20)) as sk:
        with Locations((0, CAM_Z)):                       # Xtion 센서 창
            RectangleRounded(*CAM_WIN)
        with Locations((0, LED_Z)):
            RectangleRounded(lw, lh, lh / 2 - 0.1)
        with Locations(*[(y, VENT_Z) for y in VENT_YS]):
            RectangleRounded(vw, vh, vh / 2 - 0.1)
    return sk.sketch


def camera():
    """ASUS Xtion Pro Live 실물 STEP.

    원본은 렌즈가 +Z, 스탠드 아래가 -Y 다. 로봇 좌표(전방 +X, 위 +Z)로 눕히려면
    STEP z→로봇 x, STEP y→로봇 z, STEP x→로봇 y 로 돌린다.
    """
    cam = import_step(str(XTION_STEP))
    b = cam.bounding_box()
    assert abs(b.size.X - XT_BAR[0]) < 1 and abs(b.size.Z - XT_BAR[2] - 4) < 2, \
        f"Xtion STEP 치수가 실측과 다르다 {b.size}"
    if not XT_KEEP_BASE:                      # 스탠드·베이스는 떼고 센서 바만 남긴다
        cam = max(cam.solids(), key=lambda x: x.volume)
    orient = Plane(origin=(0, 0, 0), x_dir=(0, 1, 0), z_dir=(1, 0, 0)).location
    return Pos(CAM_X, -XT_BAR_C[0], CAM_Z - XT_BAR_C[1]) * orient * cam


def cam_bracket():
    """카메라 크래들 — 베이스를 떼어낸 센서 바를 직접 감싼다.

    바닥판이 카메라크로스 빔 윗면에 M6 두 개로 물리고, 뒤판·옆 치크·상단 스트랩이
    바를 3면에서 잡는다. 앞은 열려 있어야 하므로 치크 안쪽면은 창 가장자리에 맞춘다.
    """
    x1 = CAM_X - 5                                        # 창보다 5 뒤 (밖에서 안 보이게)
    x0 = CROSS_X - S / 2 - 5
    z0 = CAM_BAR_BOT - CAM_BRK_T                          # 바닥판 밑면 = 빔 윗면
    z1 = CAM_Z + XT_BAR[1] / 2                            # 바 윗면
    cin = XT_BAR[0] / 2 + 1                               # 치크 안쪽면 (바에서 1 여유)
    with BuildPart() as p:
        with Locations(((x0 + x1) / 2, 0, z0 + CAM_BRK_T / 2)):          # 바닥판
            Box(x1 - x0, 2 * (cin + CAM_CHEEK_T), CAM_BRK_T)
        with Locations((x0 + CAM_CHEEK_T / 2, 0, (z0 + z1 + 6) / 2)):    # 뒤판
            Box(CAM_CHEEK_T, 2 * (cin + CAM_CHEEK_T), z1 + 6 - z0)
        for sy in (-1, 1):                                               # 옆 치크
            with Locations(((x0 + x1) / 2, sy * (cin + CAM_CHEEK_T / 2),
                            (z0 + z1 + 6) / 2)):
                Box(x1 - x0, CAM_CHEEK_T, z1 + 6 - z0)
        with Locations(((x0 + x1 - 12) / 2, 0, z1 + 3)):                 # 상단 스트랩
            Box(x1 - x0 - 12, 2 * (cin + CAM_CHEEK_T), 6)
        with Locations(*[(CROSS_X, y, z0 + CAM_BRK_T / 2) for y in (-60, 60)]):
            Cylinder(3.3, CAM_BRK_T + 2, mode=Mode.SUBTRACT)             # M6 → 빔 상단 슬롯
    return p.part


def led_bar():
    """LED 바 디퓨저 — 슬롯을 채우는 혀 + 안쪽 플랜지."""
    lw, lh = LED_SLOT[0] - LED_FIT, LED_SLOT[1] - LED_FIT
    bw, bh, bt = LED_BACK
    with BuildPart() as p:
        with BuildSketch(Plane.YZ.offset(SHELL_X)):
            with Locations((0, LED_Z)):
                RectangleRounded(lw, lh, lh / 2 - 0.1)
        extrude(amount=-(SHELL_T + 1))
        with BuildSketch(Plane.YZ.offset(SHELL_X - SHELL_T)):
            with Locations((0, LED_Z)):
                RectangleRounded(bw, bh, 8)
        extrude(amount=-bt)
    return p.part


def _low_cut():
    """모듈 높이 아래를 잘라내는 도구.

    z<MODULE_TOP 을 통째로 지우되 바퀴 사이 사다리꼴만 남긴다.
    → 앞뒤 패널은 MODULE_TOP 에서 끊기고, 옆면만 --\\__/-- 로 내려온다.
    사다리꼴 경사 끝(SKIRT_TOP_X)은 조향 스윕 원 안쪽 경계보다 안에 있어야 한다.
    """
    with BuildPart(mode=Mode.PRIVATE) as p:
        with Locations((0, 0, (SKIRT_LOW_Z - 20 + MODULE_TOP) / 2)):
            Box(2 * SHELL_X + 20, 2 * SHELL_Y + 20, MODULE_TOP - SKIRT_LOW_Z + 20)
        with BuildSketch(Plane.XZ) as sk:      # Plane.XZ 법선은 -Y 다 → both 로 양쪽 다 덮는다
            Polygon((-SKIRT_TOP_X, MODULE_TOP + 1), (-SKIRT_LOW_X, SKIRT_LOW_Z),
                    (SKIRT_LOW_X, SKIRT_LOW_Z), (SKIRT_TOP_X, MODULE_TOP + 1),
                    align=None)
        extrude(to_extrude=sk.sketch, amount=SHELL_Y + 20, both=True, mode=Mode.SUBTRACT)
    return p.part


def exterior():
    """외장 셸 — 앞/뒤 두 짝.

    본체는 z=MODULE_TOP 부터 데크 밑까지 전둘레. 바퀴 위는 여기서 덮인다.
    그 아래는 옆면 가운데만 사다리꼴로 내려와 --\\__/-- 실루엣이 된다.
    상단에는 안쪽으로 플랜지를 접어 프레임 윗면에 얹고 볼트로 문다 (데크가 덮어 가린다).
    """
    top = SHELL_TOP
    with BuildPart() as p:
        with BuildSketch(Plane.XY.offset(SKIRT_LOW_Z)):
            RectangleRounded(2 * SHELL_X, 2 * SHELL_Y, SHELL_R)
            RectangleRounded(2 * (SHELL_X - SHELL_T), 2 * (SHELL_Y - SHELL_T),
                             SHELL_R - SHELL_T, mode=Mode.SUBTRACT)
        extrude(amount=top - SKIRT_LOW_Z)
        with BuildSketch(Plane.XY.offset(DECK_Z)):        # 고정 플랜지 (프레임 윗면에 얹힌다)
            RectangleRounded(2 * (SHELL_X - SHELL_T), 2 * (SHELL_Y - SHELL_T),
                             SHELL_R - SHELL_T)
            RectangleRounded(2 * _fin_x, 2 * _fin_y, SHELL_R - SHELL_T - FLANGE_W,
                             mode=Mode.SUBTRACT)
        extrude(amount=FLANGE_T)
        for x in FLANGE_BOLT_X:                           # 상부사이드 레일 상단 슬롯
            with Locations(*[(x, sy * TOP_RAIL_Y, DECK_Z + FLANGE_T / 2) for sy in (-1, 1)]):
                Cylinder(3.3, FLANGE_T + 2, mode=Mode.SUBTRACT)
        for y in FLANGE_BOLT_Y:                           # 상부크로스 상단 슬롯
            with Locations(*[(sx * CROSS_X, y, DECK_Z + FLANGE_T / 2) for sx in (-1, 1)]):
                Cylinder(3.3, FLANGE_T + 2, mode=Mode.SUBTRACT)
        # 스커트 구간만 안쪽으로 두껍게. 아래 _low_cut 이 사다리꼴만 남겨준다.
        with BuildSketch(Plane.XY.offset(SKIRT_LOW_Z)):
            RectangleRounded(2 * SHELL_X, 2 * SHELL_Y, SHELL_R)
            RectangleRounded(2 * (SHELL_X - SKIRT_T), 2 * (SHELL_Y - SKIRT_T),
                             SHELL_R - SKIRT_T, mode=Mode.SUBTRACT)
        extrude(amount=MODULE_TOP - SKIRT_LOW_Z)
        add(_low_cut(), mode=Mode.SUBTRACT)
        # A compact shell intersects the wheel width, so open a true wheel arch
        # around each tire.  The cutter follows the actual wheel axis (Y) and
        # extends well past the side wall; radial clearance covers tire growth,
        # frame flex and the small Design-D axle offset from the legacy model.
        if COMPACT_SHELL:
            arch_r = WHEEL_D / 2 + WHEEL_ARCH_CLR
            arch_depth = WHEEL_W + 2 * (WHEEL_ARCH_CLR + SHELL_T + 10)
            for x in (-AXLE_X, AXLE_X):
                for sy in (-1, 1):
                    with Locations((x, sy * SHELL_WHEEL_Y, AXLE_Z)):
                        Cylinder(
                            arch_r,
                            arch_depth,
                            rotation=(90, 0, 0),
                            align=(Align.CENTER, Align.CENTER, Align.CENTER),
                            mode=Mode.SUBTRACT,
                        )
        if FACE:
            extrude(to_extrude=front_cutouts(), amount=40, mode=Mode.SUBTRACT)
            extrude(to_extrude=rear_cutouts(), amount=-40, mode=Mode.SUBTRACT)
            add(port_bay())                       # 안으로 들어간 커넥터 베이
            add(mini_cuts(), mode=Mode.SUBTRACT)  # 소형 카메라 6개
            add(cooling_cuts(), mode=Mode.SUBTRACT)   # 배기팬 + 측면 흡기 루버
            add(bumper_cuts(), mode=Mode.SUBTRACT)    # 범퍼 플런저
            add(handle_cuts(), mode=Mode.SUBTRACT)    # 리프팅 손잡이 개구
            if OUTRIGGER:
                add(outrigger_cuts(), mode=Mode.SUBTRACT)
            if SIDE_RAIL:
                add(side_rail_cuts(), mode=Mode.SUBTRACT)
        with Locations((0, 0, (SKIRT_LOW_Z + top) / 2)):      # 앞/뒤 분할
            Box(2, 2 * SHELL_Y + 20, top - SKIRT_LOW_Z + 20, mode=Mode.SUBTRACT)
    return p.solids()


# ---- 판재 ----
def panel(w, d, r, t, z, pitch, hole, grommets=()):
    with BuildPart() as p:
        with BuildSketch():
            RectangleRounded(w, d, r)
        extrude(amount=t)
        n, m = int((w / 2 - 50) / pitch), int((d / 2 - 50) / pitch)
        grid = [(i * pitch, j * pitch)
                for i in range(-n, n + 1) for j in range(-m, m + 1)]
        near = lambda pt: any(math.dist(pt, g) < GROMMET_D / 2 + 6 for g in grommets)
        with Locations(*[pt for pt in grid if not near(pt)]):   # 그로밋에 먹히는 구멍은 뺀다
            Hole(hole / 2)
        if grommets:
            with Locations(*grommets):
                Hole(GROMMET_D / 2)
    return Pos(0, 0, z) * p.part


# ---- 베이 내부 · 냉각 · 데크 ----
def fans():
    """후면 배기팬 2개 — 셸 안쪽면에 붙는다."""
    out = []
    for i, y in enumerate(FAN_YS):
        with BuildPart() as p:
            with Locations((-SHELL_X + SHELL_T + FAN_T / 2, y, FAN_Z)):
                Box(FAN_T, FAN_SIZE, FAN_SIZE)
                Cylinder(FAN_HOLE / 2, FAN_T + 2, rotation=(0, 90, 0), mode=Mode.SUBTRACT)
        p.part.label = f"배기팬{i+1}"
        out.append(p.part)
    return out


def cooling_cuts():
    """배기팬 취부공(뒷면) + 측면 흡기 루버 — 솔리드로 돌려준다.

    앞면 그릴은 z<바닥판 이라 모듈실만 식힌다. 베이 공기는 옆에서 받아 뒤로 뺀다.
    """
    lw, lh = LOUVER
    with BuildPart(mode=Mode.PRIVATE) as p:
        for y in FAN_YS:                                    # 배기팬
            with Locations((-SHELL_X, y, FAN_Z)):
                Cylinder(FAN_HOLE / 2, 4 * SHELL_T, rotation=(0, 90, 0))
            with Locations(*[(-SHELL_X, y + sy * FAN_PCD / 2, FAN_Z + sz * FAN_PCD / 2)
                             for sy in (-1, 1) for sz in (-1, 1)]):
                Cylinder(FAN_SCREW / 2, 4 * SHELL_T, rotation=(0, 90, 0))
        for sy in (-1, 1):                                  # 측면 흡기 루버
            with BuildSketch(Plane.XZ.offset(sy * SHELL_Y)) as sk:
                with Locations(*[(x, LOUVER_Z + dz)
                                 for x in LOUVER_XS for dz in LOUVER_DZ]):
                    RectangleRounded(lw, lh, lh / 2 - 0.1)
            extrude(to_extrude=sk.sketch, amount=4 * SHELL_T, both=True)
    return p.part


def pi5():
    """라즈베리파이5 + NVMe + PoE HAT.

    실물 STEP 은 솔리드가 516개다. 그대로 넣으면 간섭검사와 HLR 렌더가
    수십 배 느려진다(10분 넘게 안 끝났다). 외피 설계에 필요한 건 외형뿐이라
    기본은 실측 외형 박스를 쓰고, 배선·커넥터를 볼 때만 PI_DETAIL 을 켠다.
    """
    if PI_DETAIL:
        pi = import_step(str(RPI_STEP))
        b = pi.bounding_box()
        return Pos(PI_AT[0] - b.center().X, PI_AT[1] - b.center().Y, BAY_Z - b.min.Z) * pi
    with BuildPart() as p:
        with Locations((*PI_AT, BAY_Z + PI_ENV[2] / 2)):
            Box(*PI_ENV)
        fillet(p.edges().filter_by(Axis.Z), radius=3)
    p.part.label = "라즈베리파이5"
    return p.part


def boards():
    """드라이버 보드 목업 — 호버보드 메인보드 2장 + 조향 서보 드라이버."""
    out = []
    for i, (x, y) in enumerate(HB_AT):
        with BuildPart() as p:
            with Locations((x, y, BAY_Z + HB_BOARD[2] / 2)):
                Box(*HB_BOARD)
        p.part.label = f"인휠드라이버{i+1}"
        out.append(p.part)
    if not SKID:                      # 스키드는 조향축이 없다
        with BuildPart() as p:
            with Locations((*STEER_BOARD_AT, BAY_Z + STEER_BOARD[2] / 2)):
                Box(*STEER_BOARD)
        p.part.label = "조향드라이버"
        out.append(p.part)
    return out


def battery():
    with BuildPart() as p:
        with Locations((*BATT_AT, BAY_Z + BATT[2] / 2)):
            Box(*BATT)
        fillet(p.edges().filter_by(Axis.Z), radius=6)
    p.part.label = "배터리"
    return p.part


def battery_bracket():
    """배터리 브라켓 — 앞뒤 벽으로 물고 상단 스트랩으로 눌러 뜨는 걸 막는다."""
    bx, by = BATT_AT
    w, d, h = BATT
    t, (sw, st) = BATT_WALL, BATT_STRAP
    with BuildPart() as p:
        for sx in (-1, 1):                                  # 앞뒤 벽
            with Locations((bx + sx * (w / 2 + t / 2), by, BAY_Z + h * 0.6 / 2)):
                Box(t, d + 2 * t, h * 0.6)
        for sx in (-1, 1):                                  # 바닥판에 물리는 발
            with Locations((bx + sx * (w / 2 + t + 12), by, BAY_Z + t / 2)):
                Box(24 + t, d + 2 * t, t)
                with GridLocations(0, d, 1, 2):
                    Cylinder(2.75, t + 2, mode=Mode.SUBTRACT)   # M5 → 바닥판
        with Locations((bx, by, BAY_Z + h + st / 2)):       # 상단 스트랩
            Box(w + 2 * t, sw, st)
    p.part.label = "배터리브라켓"
    return p.part


def bumpers():
    """앞뒤 범퍼 — 셸이 직접 부딪히면 3mm 벽이 깨진다. 바가 먼저 맞고
    뒤의 플런저가 리미트 스위치를 누른다."""
    w, h, t = BUMPER
    out = []
    for i, sx in enumerate((1, -1)):
        with BuildPart() as p:
            with Locations((sx * (SHELL_X + t / 2), 0, BUMPER_Z)):
                Box(t, w, h)
            fillet(p.edges().filter_by(Axis.X), radius=h / 3)
            for y in BUMPER_SW_Y:      # 플런저 — 셸을 뚫고 스위치 앞 2mm 까지만 (평시 비접촉)
                with Locations((sx * (SHELL_X + 5), y, BUMPER_Z)):
                    Cylinder(BUMPER_PLUNGER / 2 - 0.6, 20, rotation=(0, 90, 0))
        p.part.label = f"범퍼{['앞', '뒤'][i]}"
        out.append(p.part)
    return out


def bumper_switches():
    """범퍼 뒤 리미트 스위치 4개 — 셸 안쪽면에 붙는다."""
    out = []
    for i, sx in enumerate((1, -1)):
        for j, y in enumerate(BUMPER_SW_Y):
            with BuildPart() as p:
                with Locations((sx * (SHELL_X - SHELL_T - 4 - BUMPER_SW[2] / 2), y, BUMPER_Z)):
                    Box(BUMPER_SW[2], BUMPER_SW[0], BUMPER_SW[1])
            p.part.label = f"범퍼스위치{['앞', '뒤'][i]}{j+1}"
            out.append(p.part)
    return out


def tail_lights():
    """후미등 2개 — 비상정지를 피해 좌우로 갈랐다. 뒤가 깜깜하면 실외에서 못 쓴다."""
    lw, lh = TAIL
    out = []
    for i, y in enumerate(TAIL_YS):
        with BuildPart() as p:
            with BuildSketch(Plane.YZ.offset(-SHELL_X)):
                with Locations((y, TAIL_Z)):
                    RectangleRounded(lw - LED_FIT, lh - LED_FIT, lh / 2 - 0.6)
            extrude(amount=SHELL_T + 1)
            with BuildSketch(Plane.YZ.offset(-(SHELL_X - SHELL_T - 1))):
                with Locations((y, TAIL_Z)):
                    RectangleRounded(lw + 20, lh + 16, 6)
            extrude(amount=3)
        p.part.label = f"후미등{i+1}"
        out.append(p.part)
    return out


def buzzer():
    """부저 — 앞면 흡기 그릴 뒤에서 운다. 별도 구멍을 안 뚫어도 소리가 나간다."""
    d, l = BUZZER
    with BuildPart() as p:
        with Locations((SHELL_X - SHELL_T - l / 2, BUZZER_Y, VENT_Z)):
            Cylinder(d / 2, l, rotation=(0, 90, 0))
    p.part.label = "부저"
    return p.part


def outrigger_cuts():
    """아웃리거 브라켓이 셸 옆면을 지나는 구멍 4개."""
    with BuildPart(mode=Mode.PRIVATE) as p:
        for x in OUT_X:
            for sy in (-1, 1):
                with Locations((x, sy * SHELL_Y, TOP_RAIL_Z)):
                    Box(6 * SHELL_T, OUT_ARM[0] + 8, 46, rotation=(0, 0, 0))
    return p.part


def outriggers():
    """접이식 아웃리거 4 — 상부 사이드 레일 바깥 슬롯에 피벗.

    팔이 뻗으면 서스펜션이 눌려 툴이 19mm 움직인다. 정밀 작업이나 계측 때는
    아웃리거로 차체를 지면에 직접 지지해 서스펜션을 우회한다.

    접었을 때는 셸 옆면을 따라 수직으로 붙어 전폭을 안 늘리고, 펴면 45° 로
    벌어져 지지 반폭이 {}mm 가 된다 (윤거 반폭 260 대비).
    """.format(int(SHELL_Y + OUT_PIVOT_DY + OUT_ARM_L * math.sin(math.radians(OUT_ANG))))
    py = SHELL_Y + OUT_PIVOT_DY
    pz = TOP_RAIL_Z
    ax, at = OUT_ARM
    jd, jl = OUT_JACK
    fd, ft = OUT_FOOT
    ang = OUT_ANG if OUT_DEPLOY else 0.0
    out = []
    for i, (x, sy) in enumerate([(x, sy) for x in OUT_X for sy in (-1, 1)]):
        with BuildPart() as p:
            # 셸을 지나 상부 레일 바깥 슬롯에 물리는 브라켓
            with Locations((x, sy * (TOP_RAIL_Y + S / 2 + py) / 2, pz)):
                Box(ax + 14, py - TOP_RAIL_Y - S / 2, 40)
            with Locations((x, sy * (TOP_RAIL_Y + S / 2 + 3), pz)):
                Box(ax + 30, 6, 44)          # 레일에 붙는 판 (M6 x2)
                with GridLocations(0, 60, 1, 2):
                    Cylinder(6.6 / 2, 8, rotation=(90, 0, 0), mode=Mode.SUBTRACT)
            # 피벗에서 도는 암 (여기까지만 회전한다)
            rot = Location((x, sy * py, pz), (sy * ang, 0, 0))
            with Locations(rot * Location((0, 0, -OUT_ARM_L / 2))):
                Box(ax, at, OUT_ARM_L)
            # 잭은 **수직**이다. 암과 같이 기울이면 발판이 지면에 안 닿고
            # 옆으로만 벌어진다 (실제로 전폭 1432 가 나왔다).
            ey = sy * (py + OUT_ARM_L * math.sin(math.radians(ang)))
            ez = pz - OUT_ARM_L * math.cos(math.radians(ang))
            with Locations((x, ey, ez - jl / 2 + 18)):
                Cylinder(jd / 2, jl)
            z_tube = ez - jl + 18
            drop = z_tube - ft if OUT_DEPLOY else 70      # 접으면 스크류를 넣어둔다
            with Locations((x, ey, z_tube - drop / 2)):
                Cylinder(OUT_SCREW / 2, drop)
            with Locations((x, ey, z_tube - drop - ft / 2)):
                Cylinder(fd / 2, ft)
        p.part.label = f"아웃리거{i+1}"
        out.append(p.part)
    return out


def side_rail_cuts():
    """확장 레일 브라켓이 셸 옆면을 지나는 구멍."""
    with BuildPart(mode=Mode.PRIVATE) as p:
        for x in SR_BRK_X:
            for sy in (-1, 1):
                with Locations((x, sy * SHELL_Y, (RAIL_Z + SR_Z) / 2)):
                    Box(50, 6 * SHELL_T, SR_Z - RAIL_Z + 46)
    return p.part


def side_rails():
    """측면 확장 레일 2본 + 브라켓 8 — 센서를 아무 위치에나 다는 확장 인터페이스.

    셸 옆면에서 비는 z 대역은 290~325 뿐이다 (아래는 핸들, 위는 측면 루버).
    브라켓이 셸을 지나 하부사이드 레일 바깥 슬롯에 물려 하중이 프레임으로 간다.
    2020 이라 표준 슬롯 너트로 어디든 물릴 수 있다.
    """
    SRS = PROFILES[SR_SERIES]["size"]
    y_rail = RAIL_Y + S / 2                       # 하부사이드 레일 바깥면
    out = []
    for i, sy in enumerate((-1, 1)):
        b = bar(SR_SERIES, SR_L, (-SR_L / 2, sy * SR_Y, SR_Z), "X")
        b.label = f"확장레일{i+1}"
        out.append(b)
    for i, (x, sy) in enumerate([(x, sy) for x in SR_BRK_X for sy in (-1, 1)]):
        with BuildPart() as p:
            with Locations((x, sy * (y_rail + 3), RAIL_Z)):            # 레일에 붙는 판
                Box(44, 6, 40)
                with GridLocations(0, 0, 1, 1):
                    Cylinder(6.6 / 2, 8, rotation=(90, 0, 0), mode=Mode.SUBTRACT)
            with Locations((x, sy * (y_rail + SR_Y - SRS / 2) / 2,
                            (RAIL_Z + SR_Z) / 2)):                     # 셸을 지나는 웨브
                Box(SR_BRK_T, SR_Y - SRS / 2 - y_rail, SR_Z - RAIL_Z + 40)
            with Locations((x, sy * (SR_Y - SRS / 2 - 3), SR_Z)):      # 레일에 붙는 판
                Box(44, 6, 36)
        p.part.label = f"확장브라켓{i+1}"
        out.append(p.part)
    return out


def side_lidars():
    """2D 라이다 2대 — 대각 배치. 각 270° 라 둘이면 360° 를 덮는다."""
    SRS = PROFILES[SR_SERIES]["size"]
    d, h = SR_LIDAR_D, SR_LIDAR_H
    z0 = SR_Z + SRS / 2
    out = []
    for i, (x, sy) in enumerate(SR_LIDAR_AT):
        with BuildPart() as p:
            with Locations((x, sy * SR_Y, z0 + 5)):                    # 마운트 판
                Box(d + 20, d + 20, 10)
            with Locations((x, sy * SR_Y, z0 + 10 + h / 2)):
                Cylinder(d / 2, h)
            fillet(p.edges().filter_by(GeomType.CIRCLE).sort_by(Axis.Z)[-1], radius=5)
        p.part.label = f"2D라이다{i+1}"
        out.append(p.part)
    return out


def handle_cuts():
    """셸 옆면 손잡이 관통공 8개 — 팔 하나당 Ø24. 봉이 여기로 나온다."""
    with BuildPart(mode=Mode.PRIVATE) as p:
        for x in HANDLE_XS:
            for sy in (-1, 1):
                for sx in (-1, 1):
                    with Locations((x + sx * HANDLE_SPAN / 2, sy * SHELL_Y, HANDLE_Z)):
                        Cylinder(HANDLE_HOLE / 2, 6 * SKIRT_T, rotation=(90, 0, 0))
    return p.part


def handles():
    """리프팅 핸들 4개 — **하부사이드 레일** 바깥 슬롯에 직결한다.

    하중 경로: 손 → 그립 봉 → 팔 2개 → 베이스판 → 3030 하부사이드 레일 → 프레임.
    셸은 개구만 낸다 (하중을 안 받는다). 바퀴 사이라 무게중심에도 가깝다.
    """
    y_rail = RAIL_Y + S / 2                       # 레일 바깥면
    y_bar = SHELL_Y + HANDLE_STANDOFF             # 봉 중심 (셸 밖으로 나온다)
    out = []
    for i, (x, sy) in enumerate([(x, sy) for x in HANDLE_XS for sy in (-1, 1)]):
        with BuildPart() as p:
            with Locations((x, sy * (y_rail + 3), HANDLE_Z)):     # 레일에 붙는 베이스판
                Box(HANDLE_SPAN + 40, 6, 44)
                with GridLocations(HANDLE_SPAN, 0, 2, 1):
                    Cylinder(6.6 / 2, 8, rotation=(90, 0, 0), mode=Mode.SUBTRACT)
            for sx in (-1, 1):                                    # 팔 2개 (셸을 관통)
                with Locations((x + sx * HANDLE_SPAN / 2,
                                sy * (y_rail + y_bar) / 2, HANDLE_Z)):
                    Cylinder(HANDLE_ARM / 2, y_bar - y_rail, rotation=(90, 0, 0))
            with Locations((x, sy * y_bar, HANDLE_Z)):            # 그립 봉
                Cylinder(HANDLE_BAR / 2, HANDLE_SPAN + HANDLE_BAR, rotation=(0, 90, 0))

        p.part.label = f"핸들{i+1}"
        out.append(p.part)
    return out


def power_box():
    """주회로 차단기 + 퓨즈/프리차지 보드.

    차단기는 포트 베이 뒷벽에 물려 손잡이가 베이 안으로 나온다.
    36V 에 드라이버 커패시터가 붙으면 인러시가 크다 → 프리차지가 같이 있어야 한다.
    """
    bw, bd, bl = BREAKER
    out = []
    with BuildPart() as p:
        with Locations((-(SHELL_X - PORT_BAY_D - SHELL_T) + bl / 2, -25, PORT_BAY_Z + 22)):
            Box(bl, bw, bd)
    p.part.label = "주회로차단기"
    out.append(p.part)
    with BuildPart() as p:
        with Locations((*FUSEBOX_AT, BAY_Z + FUSEBOX[2] / 2)):
            Box(*FUSEBOX)
        fillet(p.edges().filter_by(Axis.Z), radius=4)
    p.part.label = "퓨즈·프리차지"
    out.append(p.part)
    return out


def deck_rails():
    """데크 모듈 레일 — 2020 을 데크 위에 얹는다. 적재물은 슬롯 너트로 문다."""
    out = []
    for i, y in enumerate(RAIL_YS):
        b = bar(RAIL_SERIES, RAIL_X1 + SHELL_X, (-SHELL_X, y, SHELL_TOP + PANEL_T
                                                 + PROFILES[RAIL_SERIES]["size"] / 2), "X")
        b.label = f"데크레일{i+1}"
        out.append(b)
    return out


# ---- 적재함 ----
def cargo_shell():
    """적재함 셸 — 본체 셸과 같은 라운드 단면을 그대로 위로 이어 올린다.

    아래 20mm(레일 구간)는 스커트다. 여기만 데크와 적재함 사이 빈 공간을 덮는다.
    스커트의 **뒷면만** 잘라내서, 뒤에서 보면 밀어 넣는 구멍이 보이고 앞·옆에서는 안 보인다.
    """
    with BuildPart() as p:
        with BuildSketch(Plane.XY.offset(CARGO_SKIRT_Z)):
            RectangleRounded(2 * SHELL_X, 2 * SHELL_Y, SHELL_R)
            RectangleRounded(2 * CARGO_IN[0], 2 * CARGO_IN[1], CARGO_IN[2],
                             mode=Mode.SUBTRACT)
        extrude(amount=CARGO_Z1 - CARGO_SHOULDER - CARGO_SKIRT_Z)   # 위는 어깨 캡
        # 뒷면 스커트 레일 노치 — 앞에서 뒤로 밀어 넣으면 레일이 여기로 들어온다.
        # 최종 위치에서 레일 끝면이 노치를 채운다. 앞면·옆면은 통으로 막혀 있다.
        for y in RAIL_YS:
            with Locations((-SHELL_X, y, (CARGO_SKIRT_Z + CARGO_Z0) / 2)):
                Box(4 * CARGO_T, CARGO_CH, CARGO_Z0 - CARGO_SKIRT_Z + 2,
                    mode=Mode.SUBTRACT)
        # 뚜껑 손잡이용 손가락 노치 — 뒷벽 상단을 파서 손이 들어가게 한다
        with Locations((-SHELL_X, 0, CARGO_Z1 - CARGO_SHOULDER - CARGO_GRIP[2] / 2)):
            Box(4 * CARGO_T, CARGO_GRIP[0] + 2 * CARGO_PT + 6, CARGO_GRIP[2],
                mode=Mode.SUBTRACT)
        # 격벽 — 앞 장비 베이와 뒤 바구니칸을 나눈다. 셸 보강도 겸한다.
        bz0, bz1 = CARGO_Z0 + CARGO_PT, CARGO_Z1 - CARGO_SHOULDER - CARGO_LID_LIP
        with Locations((CARGO_BULK_X, 0, (bz0 + bz1) / 2)):
            Box(CARGO_BULK_T, 2 * CARGO_IN[1], bz1 - bz0)
        # 바구니칸 옆벽 2장 — 뚜껑을 열었을 때 바구니만 보이게 한다.
        _sx = (-CARGO_IN[0] + CARGO_BULK_X) / 2
        for sy in (-1, 1):
            with Locations((_sx, sy * (CARGO_BAY_Y + CARGO_BULK_T) / 2,
                            (bz0 + bz1) / 2)):
                Box(CARGO_IN[0] + CARGO_BULK_X, CARGO_BULK_T, bz1 - bz0)
    return p.part


def payload_skirt():
    """페이로드 공통 슬라이드 스커트 — **표준 인터페이스**의 기구 부분.

    레일 구간(z=461~480)을 사방으로 덮어 데크와 모듈 사이 빈 공간을 가린다.
    뒷면에만 레일 노치가 있어 앞에서 뒤로 밀어 끼우고, 구멍은 뒤에서만 보인다.
    적재함이든 평판이든 이 부분은 동일하다.
    """
    with BuildPart(mode=Mode.PRIVATE) as p:
        with BuildSketch(Plane.XY.offset(CARGO_SKIRT_Z)):
            RectangleRounded(2 * SHELL_X, 2 * SHELL_Y, SHELL_R)
            RectangleRounded(2 * CARGO_IN[0], 2 * CARGO_IN[1], CARGO_IN[2],
                             mode=Mode.SUBTRACT)
        extrude(amount=CARGO_Z0 - CARGO_SKIRT_Z)
        for y in RAIL_YS:                       # 뒷면 레일 노치
            with Locations((-SHELL_X, y, (CARGO_SKIRT_Z + CARGO_Z0) / 2)):
                Box(4 * CARGO_T, CARGO_CH, CARGO_Z0 - CARGO_SKIRT_Z + 2,
                    mode=Mode.SUBTRACT)
    return p.part


def flat_deck():
    """평판 페이로드 모듈 — 적재함 대신 얹는 가장 단순한 모듈.

    슬라이드 스커트는 적재함과 동일하고, 그 위가 그냥 판이다.
    M6 100격자로 아무거나 볼트로 물리고, 가장자리 립이 미끄러짐을 막는다.
    """
    with BuildPart() as p:
        add(payload_skirt())
        with BuildSketch(Plane.XY.offset(CARGO_Z0)):        # 상판
            RectangleRounded(2 * SHELL_X, 2 * SHELL_Y, SHELL_R)
        extrude(amount=FLAT_PT)
        with BuildSketch(Plane.XY.offset(CARGO_Z0 + FLAT_PT)):   # 가장자리 립
            RectangleRounded(2 * SHELL_X, 2 * SHELL_Y, SHELL_R)
            RectangleRounded(2 * (SHELL_X - FLAT_LIP_T), 2 * (SHELL_Y - FLAT_LIP_T),
                             SHELL_R - FLAT_LIP_T, mode=Mode.SUBTRACT)
        extrude(amount=FLAT_LIP)
        n = int((SHELL_X - 90) / FLAT_GRID)
        m = int((SHELL_Y - 90) / FLAT_GRID)
        with Locations(*[(i * FLAT_GRID, j * FLAT_GRID, CARGO_Z0 + FLAT_PT / 2)
                         for i in range(-n, n + 1) for j in range(-m, m + 1)]):
            Cylinder(6.6 / 2, FLAT_PT + 4, mode=Mode.SUBTRACT)
    p.part.label = "평판"
    return p.part


def flat_ties():
    """평판 타이다운 D링 6개 — 가장자리 립 안쪽."""
    z = CARGO_Z0 + FLAT_PT
    out = []
    for i, (x, sy) in enumerate([(x, sy) for x in FLAT_TIE_XS for sy in (-1, 1)]):
        with BuildPart() as p:
            with Locations((x, sy * (SHELL_Y - FLAT_LIP_T - 22), z)):
                Box(30, 10, 10, align=(Align.CENTER, Align.CENTER, Align.MIN))
            with Locations((x, sy * (SHELL_Y - FLAT_LIP_T - 22), z + 10 + 13)):
                Torus(CARGO_TIE / 2, 4, rotation=(0, 90, 0))
        p.part.label = f"평판타이다운{i+1}"
        out.append(p.part)
    return out


def sensor_tower():
    """센서 타워 구조 — 베이스판 + 마스트 2본 + 가로타이 2 + 대각 브레이스 4 + 헤드판.

    라이다를 외팔 마스트에 얹으면 주행 진동으로 포인트클라우드가 흐려진다.
    기둥 2본을 200mm 띄워 Y축 굽힘을 잡고, 전후 대각 브레이스로 X축을 삼각화한다.
    """
    TS = PROFILES[TOWER_SER]["size"]
    z0 = CARGO_Z0 + TOWER_BASE_T
    hw, hd, ht = TOWER_HEAD
    bx, bz, bt, bw = TOWER_BRACE
    out = []
    with BuildPart() as p:                                  # 베이스 판
        with BuildSketch(Plane.XY.offset(CARGO_Z0)):
            RectangleRounded(2 * SHELL_X, 2 * SHELL_Y, SHELL_R)
        extrude(amount=TOWER_BASE_T)
        n, m = int((SHELL_X - 90) / FLAT_GRID), int((SHELL_Y - 90) / FLAT_GRID)
        with Locations(*[(i * FLAT_GRID, j * FLAT_GRID, CARGO_Z0 + TOWER_BASE_T / 2)
                         for i in range(-n, n + 1) for j in range(-m, m + 1)]):
            Cylinder(6.6 / 2, TOWER_BASE_T + 4, mode=Mode.SUBTRACT)
        add(payload_skirt())
    p.part.label = "타워베이스"
    out.append(p.part)
    for i, sy in enumerate((-1, 1)):                         # 마스트 기둥 2본
        b = bar(TOWER_SER, TOWER_H - z0, (0, sy * TOWER_POST_Y, z0), "Z")
        b.label = f"마스트기둥{i+1}"
        out.append(b)
    for i, z in enumerate(TOWER_TIE_Z):                      # 가로타이
        L = 2 * TOWER_POST_Y - TS
        b = bar(TOWER_SER, L, (0, -L / 2, z), "Y")
        b.label = f"마스트타이{i+1}"
        out.append(b)
    for i, (sx, sy) in enumerate([(sx, sy) for sx in (-1, 1) for sy in (-1, 1)]):
        # 윗단을 기둥 중심(x=0)까지 보내면 전후 브레이스끼리 서로 파고든다.
        # 기둥 **바깥면**(x=±TS/2)에 붙여야 한다.
        xt = TS / 2
        ang = math.degrees(math.atan2(bx - xt, bz - z0))     # 대각 브레이스
        L = math.hypot(bx - xt, bz - z0)
        with BuildPart() as q:
            # 부호를 반대로 주면 마스트 밑에서 바깥으로 뻗어 올라가 브레이스가 아니게 된다.
            # 베이스판 바깥(낮은 쪽) → 마스트(높은 쪽) 방향이어야 한다.
            with Locations(Location((sx * (bx + xt) / 2, sy * TOWER_POST_Y, (z0 + bz) / 2),
                                    (0, -sx * ang, 0))):
                Box(bt, bw, L)
        q.part.label = f"마스트브레이스{i+1}"
        out.append(q.part)
    with BuildPart() as q:                                   # 헤드 플레이트
        with Locations((0, 0, TOWER_H + ht / 2)):
            Box(hw, hd, ht)
        fillet(q.edges().filter_by(Axis.Z), radius=20)
    q.part.label = "헤드플레이트"
    out.append(q.part)
    return out


def sensor_devices():
    """센서 목업 — 3D 라이다 / 팬틸트 카메라 / GNSS / IMU.

    라이다는 라이저로 띄워 스캔 평면이 GNSS·헤드판보다 위로 오게 한다.
    IMU 는 베이스판 위(진동 적고 회전중심 근처)에 둔다.
    """
    ld, lh = LIDAR
    ht = TOWER_HEAD[2]
    z_head = TOWER_H + ht
    out = []
    with BuildPart() as p:                                   # 라이다 + 라이저
        with Locations((0, 0, z_head + LIDAR_RISER / 2)):
            Cylinder(ld / 2 - 8, LIDAR_RISER)
        with Locations((0, 0, z_head + LIDAR_RISER + lh / 2)):
            Cylinder(ld / 2, lh)
        fillet(p.edges().filter_by(GeomType.CIRCLE).sort_by(Axis.Z)[-1], radius=4)
    p.part.label = "3D라이다"
    out.append(p.part)
    px, py, pz = PANTILT
    with BuildPart() as q:                                   # 팬틸트 카메라 + 마스트 브라켓
        with Locations((px / 2 + 22, 0, PANTILT_Z)):
            Box(px, py, pz)
        fillet(q.edges().filter_by(Axis.X), radius=8)
        with Locations((11, 0, PANTILT_Z)):      # 기둥 **사이**에 끼워 붙인다.
            # 기둥 폭까지 덮으면 프로파일을 관통한다 (실제로 3.8cm³ 물렸다)
            Box(22, 2 * TOWER_POST_Y - PROFILES[TOWER_SER]["size"] - 2, 40)
    q.part.label = "팬틸트카메라"
    out.append(q.part)
    gd, gh = GNSS
    with BuildPart() as r:                                   # GNSS 안테나
        with Locations((GNSS_AT[0], GNSS_AT[1], z_head + gh / 2)):
            Cylinder(gd / 2, gh)
    r.part.label = "GNSS"
    out.append(r.part)
    with BuildPart() as t:                                   # IMU
        with Locations((-200, 0, CARGO_Z0 + TOWER_BASE_T + IMU[2] / 2)):
            Box(*IMU)
    t.part.label = "IMU"
    out.append(t.part)
    return out


def arm_mount():
    """로봇 팔 마운트 — 두꺼운 베이스판 + 취부 보스 + 컨트롤러 박스 자리.

    팔이 뻗으면 베이스에 큰 굽힘 모멘트가 걸린다. 3mm 판으로는 휘어서 툴 위치가
    흐트러지므로 10mm 판을 쓰고, 보스 밑에 리브를 넣어 레일 3본으로 분산한다.
    팔은 축간 중심(x=0)에 둔다 — 어느 방향으로 뻗어도 전복 여유가 같다.
    """
    ax, ay = ARM_AT
    bd, bh = ARM_BOSS
    z0 = CARGO_Z0
    zt = z0 + ARM_PLATE_T
    out = []
    with BuildPart() as p:
        with BuildSketch(Plane.XY.offset(z0)):                # 베이스판 10mm
            RectangleRounded(2 * SHELL_X, 2 * SHELL_Y, SHELL_R)
        extrude(amount=ARM_PLATE_T)
        with Locations((ax, ay, zt + bh / 2)):                # 취부 보스
            Cylinder(bd / 2, bh)
        for i in range(ARM_NB):                               # 팔 취부 볼트
            a = 2 * math.pi * i / ARM_NB + math.pi / ARM_NB
            with Locations((ax + ARM_PCD / 2 * math.cos(a),
                            ay + ARM_PCD / 2 * math.sin(a), zt + bh / 2)):
                Cylinder(ARM_BOLT / 2, bh + ARM_PLATE_T + 4, mode=Mode.SUBTRACT)
        with Locations((ax, ay, zt + bh / 2)):                # 케이블 통과
            Cylinder(22, bh + ARM_PLATE_T + 4, mode=Mode.SUBTRACT)
        for y in RAIL_YS:      # 보스 밑 분산 리브 — 보스 윗면을 넘으면 팔 베이스를 문다
            with Locations((ax, y, zt + bh / 2)):
                Box(2 * abs(ax) + 260, 8, bh)
        n, m = int((SHELL_X - 120) / FLAT_GRID), int((SHELL_Y - 120) / FLAT_GRID)
        with Locations(*[(i * FLAT_GRID, j * FLAT_GRID, z0 + ARM_PLATE_T / 2)
                         for i in range(-n, n + 1) for j in range(-m, m + 1)
                         if abs(i * FLAT_GRID - ax) > bd / 2 + 20]):
            Cylinder(6.6 / 2, ARM_PLATE_T + 4, mode=Mode.SUBTRACT)
        add(payload_skirt())
    p.part.label = "팔베이스판"
    out.append(p.part)
    cw, cd, ch = ARM_CTRL
    with BuildPart() as q:                                    # 컨트롤러 박스
        with Locations((ARM_CTRL_AT[0], ARM_CTRL_AT[1], zt + ch / 2)):
            Box(cw, cd, ch)
        fillet(q.edges().filter_by(Axis.Z), radius=8)
    q.part.label = "팔컨트롤러"
    out.append(q.part)
    return out


def robot_arm():
    """팔 목업 — 접힌 자세. 실제 모델은 기종 확정 후 STEP 을 얹는다."""
    ax, ay = ARM_AT
    z0 = CARGO_Z0 + ARM_PLATE_T + ARM_BOSS[1]
    L1, L2, L3 = 0.42 * ARM_REACH, 0.38 * ARM_REACH, 0.14 * ARM_REACH
    with BuildPart() as p:
        with Locations((ax, ay, z0 + ARM_BASE_H / 2)):         # 베이스 몸통
            Cylinder(ARM_BASE_D / 2, ARM_BASE_H)
        z1 = z0 + ARM_BASE_H
        with Locations((ax, ay, z1)):                          # 숄더 관절
            Cylinder(52, 96, rotation=(90, 0, 0))
        with Locations(Location((ax, ay, z1 + L1 / 2), (0, 18, 0))):
            Cylinder(38, L1)                                   # 상완
        z2, x2 = z1 + L1 * math.cos(math.radians(18)), ax + L1 * math.sin(math.radians(18))
        with Locations((x2, ay, z2)):                          # 엘보
            Cylinder(44, 86, rotation=(90, 0, 0))
        with Locations(Location((x2 + L2 / 2 * 0.5, ay, z2 - L2 / 2 * 0.87),
                                (0, 150, 0))):
            Cylinder(32, L2)                                   # 전완 (접혀 내려온다)
        x3, z3 = x2 + L2 * 0.5, z2 - L2 * 0.87
        with Locations((x3, ay, z3)):                          # 손목 + 툴 플랜지
            Cylinder(34, 60, rotation=(0, 90, 0))
        with Locations((x3 + 40, ay, z3)):
            Cylinder(31, 14, rotation=(0, 90, 0))
    p.part.label = "로봇팔"
    return p.part


def inst_module():
    """계측 모듈 구조 — 베이스판 + 로드셀 4 + 시험 플랫폼 + DAQ 박스 + 견인 훅.

    시험 플랫폼은 **로드셀 4개에만** 닿는다. 다른 데 닿으면 하중이 새서 못 잰다.
    그래서 DAQ 박스는 플랫폼 밖(뒤쪽 베이스판)에 따로 앉힌다.
    """
    z0 = CARGO_Z0
    zt = z0 + INST_PLATE_T
    cd, chh = INST_CELL
    cx, dx, dy = INST_CELL_AT
    tw, td, tt = INST_TOP
    out = []
    with BuildPart() as p:                                  # 베이스판
        with BuildSketch(Plane.XY.offset(z0)):
            RectangleRounded(2 * SHELL_X, 2 * SHELL_Y, SHELL_R)
        extrude(amount=INST_PLATE_T)
        n, m = int((SHELL_X - 90) / FLAT_GRID), int((SHELL_Y - 90) / FLAT_GRID)
        with Locations(*[(i * FLAT_GRID, j * FLAT_GRID, z0 + INST_PLATE_T / 2)
                         for i in range(-n, n + 1) for j in range(-m, m + 1)]):
            Cylinder(6.6 / 2, INST_PLATE_T + 4, mode=Mode.SUBTRACT)
        add(payload_skirt())
    p.part.label = "계측베이스판"
    out.append(p.part)
    with BuildPart() as q:                                  # 시험 플랫폼
        with BuildSketch(Plane.XY.offset(zt + chh + INST_BTN[1])) as sk:
            with Locations((cx, 0)):
                RectangleRounded(tw, td, 24)
        extrude(amount=tt)
        gn, gm = int((tw / 2 - 60) / FLAT_GRID), int((td / 2 - 60) / FLAT_GRID)
        with Locations(*[(cx + i * FLAT_GRID, j * FLAT_GRID,
                          zt + chh + INST_BTN[1] + tt / 2)
                         for i in range(-gn, gn + 1) for j in range(-gm, gm + 1)]):
            Cylinder(6.6 / 2, tt + 4, mode=Mode.SUBTRACT)   # 밸러스트 고정 M6
    q.part.label = "시험플랫폼"
    out.append(q.part)
    dw, dd, dh = INST_DAQ
    with BuildPart() as r:                                  # DAQ 박스 + 커넥터 패널
        with Locations((INST_DAQ_X, 0, zt + dh / 2)):
            Box(dw, dd, dh)
        fillet(r.edges().filter_by(Axis.Z), radius=8)
        for i in range(INST_CONN_N):                        # 스트로크 센서 커넥터
            y = -dd / 2 + 30 + i * (dd - 60) / (INST_CONN_N - 1)
            with Locations((INST_DAQ_X - dw / 2, y, zt + 26)):
                Cylinder(INST_CONN / 2, 14, rotation=(0, 90, 0))
    r.part.label = "DAQ박스"
    out.append(r.part)
    tx, ty, tz = INST_TOW
    with BuildPart() as t:                                  # 견인 훅 (인라인 로드셀 자리)
        with Locations((-SHELL_X - 10, 0, zt + tz / 2)):   # DAQ 앞이 아니라 후단 밖으로
            Box(tx, ty, tz)
        with Locations((-SHELL_X - 24, 0, zt + tz / 2)):
            Torus(16, 5, rotation=(0, 90, 0))
    t.part.label = "견인훅"
    out.append(t.part)
    return out


def inst_devices():
    """계측 장비 목업 — 로드셀 4 / IMU / 디스플레이."""
    z0 = CARGO_Z0 + INST_PLATE_T
    cd, chh = INST_CELL
    cx, dx, dy = INST_CELL_AT
    dw, dd, dh = INST_DAQ
    out = []
    for i, (sx, sy) in enumerate([(sx, sy) for sx in (-1, 1) for sy in (-1, 1)]):
        with BuildPart() as p:                              # 버튼형 로드셀
            with Locations((cx + sx * dx, sy * dy, z0 + chh / 2)):
                Cylinder(cd / 2, chh)
            with Locations((cx + sx * dx, sy * dy, z0 + chh + INST_BTN[1] / 2)):
                Cylinder(INST_BTN[0] / 2, INST_BTN[1])      # 상부 가압 버튼 (위로만)
        p.part.label = f"로드셀{i+1}"
        out.append(p.part)
    with BuildPart() as q:                                  # 디스플레이 (뒤로 기울임)
        with Locations(Location((INST_DAQ_X, 0, z0 + dh + 32), (0, -25, 0))):
            Box(*INST_DISP)
    q.part.label = "디스플레이"
    out.append(q.part)
    with BuildPart() as r:                                  # IMU (베이스판, 진동 적은 곳)
        with Locations((250, 0, z0 + IMU[2] / 2)):
            Box(*IMU)
    r.part.label = "IMU"
    out.append(r.part)
    return out


def cargo_channels():
    """슬라이드 채널 3개 — 데크 레일을 감싸고 측면 슬롯에 턱이 물린다.

    뒤에서 밀어 넣으면 턱이 슬롯을 타고 들어가 위로 못 빠진다.
    앞쪽 끝은 막아서 스토퍼로 쓴다.
    """
    cw = CARGO_CH
    td, tt = CARGO_TAB
    z0 = CARGO_SKIRT_Z                       # 데크 윗면 위에서 시작 (더 내리면 데크를 문다)
    ch = CARGO_Z0 - z0
    L = 300                                  # 앞쪽 구간만 물면 된다
    cx = RAIL_X1 - L / 2 + CARGO_T      # 앞쪽 스토퍼 뒷면이 레일 앞끝에 딱 닿는다
    out = []
    for i, y in enumerate(RAIL_YS):
        with BuildPart() as p:
            # 윗면 웨브를 두면 레일 꼭대기와 부딪힌다. 벽 2장만 세우고 위는 바닥판이 막는다.
            for sy in (-1, 1):
                with Locations((cx, y + sy * (cw + CARGO_T) / 2, (z0 + CARGO_Z0) / 2)):
                    Box(L, CARGO_T, ch)
                with Locations((cx, y + sy * (cw / 2 - td / 2), RAIL_Z2)):
                    Box(L, td, tt)          # 레일 측면 슬롯에 물리는 턱
            with Locations((cx + L / 2 - CARGO_T / 2, y, (z0 + CARGO_Z0) / 2)):
                Box(CARGO_T, cw + 2 * CARGO_T, ch)                 # 앞쪽 스토퍼
        p.part.label = f"슬라이드채널{i+1}"
        out.append(p.part)
    return out


def cargo_floor():
    """적재함 바닥판 — 레일 위에 얹힌다. 배선 관통·타이다운 구멍이 여기 있다."""
    with BuildPart() as p:
        with BuildSketch(Plane.XY.offset(CARGO_Z0)):
            RectangleRounded(2 * CARGO_IN[0], 2 * CARGO_IN[1], CARGO_IN[2])
        extrude(amount=CARGO_PT)
        with Locations(*[(x, y, CARGO_Z0 + CARGO_PT / 2) for x, y in GROMMET_DECK]):
            Cylinder(GROMMET_D / 2, CARGO_PT + 2, mode=Mode.SUBTRACT)
    p.part.label = "함바닥판"
    return p.part


def _cap_sections(inset0):
    """어깨 라운드 단면들 — 1/4 원을 따라 안으로 들어가며 올라간다."""
    R = CARGO_SHOULDER
    out = []
    for i in range(CARGO_CAP_N + 1):
        ang = math.radians(90 * i / CARGO_CAP_N)
        ins = inset0 + R * (1 - math.cos(ang))
        z = CARGO_Z1 - R + R * math.sin(ang)
        out.append((z, ins))
    return out


def cargo_cap(front):
    """적재함 어깨 캡 — 옆벽에서 윗면으로 둥글게 넘어간다.

    각진 상자를 부드럽게 만드는 가장 큰 요소다. 바구니칸 쪽(front=False)은
    **뚜껑**이 되어 열리고, 장비 베이 쪽(front=True)은 고정 지붕이 된다.
    뚜껑이 실제로 열려야 하는 범위(바구니칸)만 덮으므로 그만큼 가볍다.
    """
    g = CARGO_LID_GAP
    # 절단면은 **곧은 수직면**이다. 처음엔 개구에도 ins 를 먹였는데, ins 는 어깨
    # 곡률이라 위로 갈수록 커진다 → 개구가 위에서 닫혀 지붕이 뚜껑 자리를 덮었다
    # (22,454mm3). 뚜껑·지붕은 g 만큼만 어긋나면 되고, 벽 두께는 로프트가 만든다.
    d = g / 2 if front else -g / 2
    x1 = CARGO_BULK_X + d                         # 개구 앞끝 (격벽)
    yh = CARGO_BAY_Y / 2 + d                      # 개구 반폭
    x0 = -SHELL_X - 10                            # 개구 뒤끝 (셸 밖까지)

    def sec(ins):
        RectangleRounded(2 * (SHELL_X - ins), 2 * (SHELL_Y - ins),
                         max(SHELL_R - ins, 6))
        with Locations(((x0 + x1) / 2, 0)):
            Rectangle(x1 - x0, 2 * yh,
                      mode=Mode.SUBTRACT if front else Mode.INTERSECT)

    def loft_of(off, dz, extra_bottom):
        secs = []
        if extra_bottom:
            with BuildSketch(Plane.XY.offset(CARGO_Z1 - CARGO_SHOULDER - dz
                                             - extra_bottom)) as sk:
                sec(off)
            secs.append(sk.sketch)
        for z, ins in _cap_sections(off):
            with BuildSketch(Plane.XY.offset(z - dz)) as sk:
                sec(ins)
            secs.append(sk.sketch)
        return secs

    with BuildPart() as p:
        loft(loft_of(0.0, 0, 0))
        loft(loft_of(CARGO_T, CARGO_T, CARGO_LID_LIP + 10), mode=Mode.SUBTRACT)
        # 아래로 물리는 위치·실링 립.
        # **캡보다 위로 4mm 더 올려 붙인다.** 립은 몸통에 들어가려고 g/2 만큼 더
        # 들여져 있어서(CARGO_T + g/2), 캡 안쪽면(CARGO_T)과 0.5mm 링 틈이 생긴다.
        # 그대로 두면 립이 캡에 안 붙어 **부품이 솔리드 2개로 갈라진다** — 원래
        # 그 상태였고, 적재함에 연결성 검사를 안 돌려서 못 잡고 있었다.
        z0 = CARGO_Z1 - CARGO_SHOULDER - CARGO_LID_LIP
        with BuildSketch(Plane.XY.offset(z0)) as sk:
            sec(CARGO_T + g / 2)
        extrude(to_extrude=sk.sketch, amount=CARGO_LID_LIP + 4)
        with BuildSketch(Plane.XY.offset(z0 - 1)) as sk:
            sec(CARGO_T + g / 2 + 6)
        extrude(to_extrude=sk.sketch, amount=CARGO_LID_LIP + 2, mode=Mode.SUBTRACT)
    return p.part


def cargo_lid():
    """화물칸 뚜껑 = 어깨 캡의 뒤쪽 + 손잡이 홈.

    위로 연다. 힌지는 격벽 위(내부)라 밖에서는 이음선만 보인다.
    """
    out = []
    with BuildPart() as p:
        add(cargo_cap(False))
        # 손잡이 — 뚜껑 후단 밑면 ㄷ자 홈. 뒤에서 손을 넣어 들어올린다.
        gw, gl, gd = CARGO_GRIP
        # 홈은 **뒤쪽 손 넣는 자리 안**에 들어가야 한다. 바구니 윗단이 뚜껑 립보다
        # 9mm 아래라, 뚜껑 밑면에서 그보다 깊게 튀어나온 게 바구니 위에 오면
        # 무조건 닿는다 (실제로 1352mm3 물렸다). 깊이를 줄이는 대신 자리를 옮긴다.
        _g0 = -CARGO_IN[0]                                  # 뒤 벽 안쪽
        _g1 = CARGO_BIN_X - CARGO_BIN[0] / 2 - 8            # 바구니 뒷면 (테이퍼 포함)
        gl = min(gl, (_g1 - _g0) - 10)
        gx = (_g0 + _g1) / 2
        zt = CARGO_Z1 - CARGO_SHOULDER
        with Locations((gx, 0, zt - gd / 2)):
            Box(gl, gw + 2 * CARGO_T, gd)
        with Locations((gx, 0, zt - gd / 2 + CARGO_PT / 2)):
            Box(gl - 2 * CARGO_PT, gw, gd, mode=Mode.SUBTRACT)
        # 상단 센서 마운트 격자
        n = int((SHELL_X + CARGO_BULK_X - 2 * CARGO_SHOULDER - 200) / CARGO_GRID)
        m = int((SHELL_Y - CARGO_SHOULDER - 120) / CARGO_GRID)
        x_mid = (CARGO_BULK_X - SHELL_X) / 2
        with Locations(*[(x_mid + i * CARGO_GRID, j * CARGO_GRID, CARGO_Z1 - CARGO_T / 2)
                         for i in range(-n, n + 1) for j in range(-m, m + 1)]):
            Cylinder(4.2 / 2, CARGO_T + 4, mode=Mode.SUBTRACT)
    p.part.label = "함뚜껑"
    out.append(p.part)
    return out


def cargo_hardware():
    """내부 힌지 2 + 솔레노이드 잠금 + 스트라이커 + 가스 스트럿 2.

    힌지는 격벽 위 **안쪽**에 둔다. 밖에서 보이면 실루엣이 깨지고 비를 맞는다.
    잠금은 뒤쪽 중앙 — 사람이 서는 쪽이고, 원격 해제도 여기가 편하다.
    """
    hd, hl = CARGO_HINGE
    lx, ly, lz = CARGO_LOCK
    sd, sl = CARGO_STRUT
    z_lid = CARGO_LID_Z
    out = []
    for i, y in enumerate(CARGO_HINGE_Y):                  # 힌지 — 격벽 위 (내부)
        with BuildPart() as p:
            with Locations((CARGO_HINGE_AT[0], y, CARGO_HINGE_AT[1])):
                Cylinder(hd / 2, hl, rotation=(90, 0, 0))
                Box(30, hl, 8)
        p.part.label = f"함힌지{i+1}"
        out.append(p.part)
    # 잠금은 뒤 중앙 1개였다. 바구니(600x400)가 들어오면서 그 자리가 없어졌고
    # (2039mm3 물렸다), 손잡이 홈까지 피하면 x 로 22mm 창밖에 안 남는다.
    # → **y 여유(±208~±300)로 옮겨 좌우 2개**로 바꾼다. 810x610 통짜 뚜껑에는
    #   중앙 1점 래치가 애초에 부족하다 — 반대쪽이 들린다.
    for i, sy in enumerate((-1, 1)):
        ly_ = sy * CARGO_LOCK_Y
        with BuildPart() as p:                              # 솔레노이드 — 바구니 뒤
            with Locations((CARGO_LOCK_X, ly_, z_lid - lz / 2 - 16)):
                Box(lx, ly, lz)
            with Locations((CARGO_LOCK_X, ly_, z_lid - 8)):
                Cylinder(5, 18)                             # 래치 볼트 (위로 뚜껑을 문다)
        p.part.label = f"함잠금{i+1}"
        out.append(p.part)
        with BuildPart() as p:                              # 뚜껑 밑면 스트라이커
            with Locations((CARGO_LOCK_X, ly_, z_lid - 5)):
                Box(30, 30, 10)
                Cylinder(6, 12, mode=Mode.SUBTRACT)
        p.part.label = f"함스트라이커{i+1}"
        out.append(p.part)
    for i, sy in enumerate((-1, 1)):                        # 가스 스트럿
        with BuildPart() as p:
            with Locations(Location((-40, sy * (CARGO_IN[1] - 45), z_lid - 110),
                                    (0, -30, 0))):
                Cylinder(sd / 2, sl)
        p.part.label = f"함스트럿{i+1}"
        out.append(p.part)
    return out


def cargo_cradle():
    """바구니 크래들 — 유로 600x400 을 위에서 떨어뜨려 앉히는 자리.

    라이너 트레이를 직접 그려 넣었던 걸 대체한다. 이제 안에 들어가는 건 기성
    컨테이너라 우리가 만들 건 **위치를 잡아주는 것**뿐이다.

    처음엔 바닥 레일 2줄에 리브를 얹었는데, 렌더해 보니 **두 조각으로 떨어져
    있었다** — 조립할 때 간격이 안 잡히고 부피도 1461cm3(PETG 0.9kg) 였다.
    지금은 **닫힌 사각 링** 하나다. 간격이 형상에 박혀 있고 통째로 떨어뜨리면 끝이다.

    · 링       바구니를 {foot}mm 띄운다. 물이 안 고이고 바닥 청소가 된다.
    · 리브     안쪽 {clr}mm 여유. 드롭인이라 꽉 끼면 안 들어간다.
               높이 {rh}mm 면 충분하다 — 상자 벽이 바구니보다 9mm 높을 뿐이라
               바구니가 **넘어질 수가 없다.** 리브는 미끄러짐만 막으면 된다.
    · 짧은 변  리브를 안 세운다. KLT 손잡이 구멍 자리이자 손 넣는 자리다.
    """.format(foot=CARGO_FOOT, clr=CARGO_BIN_CLR, rh=CARGO_RIB[1])
    bw, bd, _ = CARGO_BIN
    c, rt, rh = CARGO_BIN_CLR, CARGO_RIB[0], CARGO_RIB[1]
    z0 = CARGO_Z0 + CARGO_PT                       # 바닥판 윗면
    ow, od = bw + 2 * c, bd + 2 * c                # 리브 안쪽 치수
    rw = 32                                        # 링 폭 — 바구니 밑면이 얹히는 자리
    out = []
    with BuildPart() as p:
        with BuildSketch(Plane.XY.offset(z0)):     # 닫힌 사각 링
            with Locations((CARGO_BIN_X, 0)):
                RectangleRounded(ow + 2 * rt, od + 2 * rt, 12)
                RectangleRounded(ow + 2 * rt - 2 * rw, od + 2 * rt - 2 * rw, 6,
                                 mode=Mode.SUBTRACT)
        extrude(amount=CARGO_FOOT)
        for sy in (-1, 1):                         # 긴 변 리브 — y 를 잡는다
            for sx in (-1, 1):
                with Locations((CARGO_BIN_X + sx * (bw / 4), sy * (od + rt) / 2,
                                z0 + CARGO_FOOT + rh / 2)):
                    Box(bw / 2 - 20, rt, rh)
        for sx in (-1, 1):                         # 코너 리브 — x 를 잡는다
            for sy in (-1, 1):
                with Locations((CARGO_BIN_X + sx * (ow + rt) / 2, sy * (od - 70) / 2,
                                z0 + CARGO_FOOT + rh / 2)):
                    Box(rt, 70, rh)
        try:                    # 링 안쪽 R6 모서리에서 터진다 — 모따기는 선택사항
            fillet(p.edges().filter_by(Axis.Z).filter_by(
                lambda e: e.length > rh - 1), radius=3)
        except Exception:
            pass
    p.part.label = "바구니크래들"
    out.append(p.part)
    return out


def cargo_bin():
    """기성 유로 컨테이너 목업 (600x400x220).

    우리가 만드는 물건이 아니다. 들어가는지·손이 닿는지 보려고 띄운다.
    같은 발자국으로 EPP 보온 서모박스가 나오므로 배달용도 이 자리에 그대로 꽂힌다.
    """
    bw, bd, bh = CARGO_BIN
    z0 = CARGO_Z0 + CARGO_PT + CARGO_FOOT
    tw = 3.0                                        # 벽 두께
    with BuildPart() as p:
        for off, r0, r1 in ((0.0, 20, 24), (tw, 18, 22)):
            with BuildSketch(Plane.XY.offset(z0 + (tw if off else 0))):
                with Locations((CARGO_BIN_X, 0)):
                    RectangleRounded(bw - 2 * off, bd - 2 * off, r0)
            with BuildSketch(Plane.XY.offset(z0 + bh)):
                with Locations((CARGO_BIN_X, 0)):   # 유로 컨테이너는 위가 살짝 넓다
                    RectangleRounded(bw + 16 - 2 * off, bd + 16 - 2 * off, r1)
            loft(mode=Mode.SUBTRACT if off else Mode.ADD)
        for sx in (-1, 1):                          # 짧은 변 손잡이 구멍
            with Locations((CARGO_BIN_X + sx * (bw / 2 + 5), 0, z0 + bh - 45)):
                Box(20, 120, 40, mode=Mode.SUBTRACT)
    p.part.label = "유로컨테이너"
    return [p.part]


def cargo_equip():
    """장비 베이 — 격벽 앞면 마운트 판 + 바닥 판. 전자부품이 여기 들어간다.

    바구니를 400x300 으로 내리면서 앞에 {d:.0f}mm 가 열렸다. 닫힌 베이라
    비·먼지를 막고, 지붕은 고정(cargo_cap(True))이라 뚜껑을 열어도 안 젖는다.
    """.format(d=CARGO_EQ_X)
    g = CARGO_EQ_GRID
    x0 = CARGO_BULK_X + CARGO_BULK_T / 2
    z0 = CARGO_Z0 + CARGO_PT
    zt = CARGO_Z1 - CARGO_SHOULDER - CARGO_LID_LIP
    out = []
    with BuildPart() as p:                                  # 격벽 앞면 마운트 판
        with Locations((x0 + CARGO_PT / 2, 0, (z0 + zt) / 2)):
            Box(CARGO_PT, 2 * CARGO_IN[1] - 20, zt - z0 - 20)
        m = int((CARGO_IN[1] - 40) / g)
        n = int((zt - z0 - 60) / 2 / g)
        with Locations(*[(x0 + CARGO_PT / 2, j * g, (z0 + zt) / 2 + k * g)
                         for j in range(-m, m + 1) for k in range(-n, n + 1)]):
            Cylinder(3.4 / 2, CARGO_PT + 2, rotation=(0, 90, 0), mode=Mode.SUBTRACT)
    p.part.label = "장비마운트판"
    out.append(p.part)
    with BuildPart() as q:                                  # 장비 베이 바닥 판
        with BuildSketch(Plane.XY.offset(z0)):
            RectangleRounded(2 * (CARGO_IN[0] - 2), 2 * (CARGO_IN[1] - 2),
                             CARGO_IN[2] - 2)
            with Locations((x0 + 2 - 2 * CARGO_IN[0], 0)):
                Rectangle(4 * CARGO_IN[0], 4 * CARGO_IN[1], mode=Mode.SUBTRACT)
        extrude(amount=CARGO_PT)
    q.part.label = "장비베이바닥"
    out.append(q.part)
    return out


def cargo_retainers():
    """노치 리테이너 3개 — 뒷면 레일 노치를 덮으면서 레일 끝 보어에 M5 로 물린다.

    채널 스토퍼는 **뒤로 밀 때만** 멈춘다. 제동하면 관성으로 함이 앞으로 빠져나간다.
    손잡이 볼트 3개만 풀면 다시 슬라이드로 빠진다.
    """
    ry, rz, rt = CARGO_RET
    out = []
    for i, y in enumerate(RAIL_YS):
        with BuildPart() as p:
            with Locations((-(SHELL_X + rt / 2), y, (CARGO_SKIRT_Z + CARGO_Z0) / 2)):
                Box(rt, ry, rz)
                Cylinder(5.5 / 2, rt + 2, rotation=(0, 90, 0), mode=Mode.SUBTRACT)
            fillet(p.edges().filter_by(Axis.X), radius=4)
            with Locations((-(SHELL_X + rt + 5), y, (CARGO_SKIRT_Z + CARGO_Z0) / 2)):
                Cylinder(9, 10, rotation=(0, 90, 0))          # 손잡이 볼트 머리
        p.part.label = f"리테이너{i+1}"
        out.append(p.part)
    return out


def cargo_ties():
    """타이다운 앵커 4개 — 바구니를 눌러 묶거나, 긴 물건을 옆 여유에 고정한다.

    바구니보다 위로 올렸더니 **뚜껑을 1837mm3 씩 뚫었다** (상자를 280으로 낮추면서
    뚜껑 립이 내려온 걸 안 봤다). 이제 립 밑면 기준으로 역산한다 — 상자 높이를
    바꿔도 따라온다.
    """
    z = CARGO_LID_Z - 35
    out = []
    for i, (x, sy) in enumerate([(x, sy) for x in (-250, 60) for sy in (-1, 1)]):
        with BuildPart() as p:
            with Locations((x, sy * (CARGO_IN[1] - 5), z)):
                Box(30, 10, 12)
            with Locations((x, sy * (CARGO_IN[1] - 22), z)):
                Torus(CARGO_TIE / 2, 4, rotation=(0, 90, 0))
        p.part.label = f"타이다운{i+1}"
        out.append(p.part)
    return out


def antennas():
    """안테나 2본 — 뒷면 상부 코너에서 뒤로 기울여 세운다.

    데크에 세우면 적재함이 통째로 덮어버린다. 셸에 붙이면 적재함을 떼도 남는다.
    """
    bd, bh = ANT_BASE
    z = ANT_Z
    out = []
    for i, sy in enumerate((-1, 1)):
        with BuildPart() as p:
            with Locations((-(SHELL_X + bh / 2), sy * ANT_Y, z)):
                Cylinder(bd / 2, bh, rotation=(0, 90, 0))
            # 휩을 베이스 끝면에 딱 붙이면 두 덩어리가 한 점에서만 만나 솔리드가
            # 갈라진다. HLR 이 통째로 실패했다. 베이스 안쪽까지 겹쳐 넣는다.
            with Locations(Location((-(SHELL_X + bh / 2), sy * ANT_Y, z),
                                    (0, -ANT_TILT, 0))):
                Cylinder(ANT_D / 2, ANT_L + bh / 2,
                         align=(Align.CENTER, Align.CENTER, Align.MIN))
        p.part.label = f"안테나{i+1}"
        out.append(p.part)
    return out


# ---- 조립 ----
if SKID:
    _wheels, _mounts, _clamps = fixed_wheels()
else:
    mods, statics = modules()
floor_panel = panel(BASE_L - 2 * S, BASE_W - 2 * S, 20, PANEL_T, FRAME_TOP, 50, 5.5,
                    GROMMET_FLOOR)
deck_panel = panel(2 * (SHELL_X + DECK_LIP2), 2 * (SHELL_Y + DECK_LIP2),
                   SHELL_R + DECK_LIP2, PANEL_T, SHELL_TOP, 100, 6.6, GROMMET_DECK)
if SKID:
    # ㄷ자 위팔이 레일 **윗면 위로** 10mm 올라온다 (z 200.7~210.7). 베이 바닥판이
    # 바로 그 높이(FRAME_TOP)에 깔리므로 코너 네 곳에서 2100 mm3 씩 물렸다.
    # 판을 마운트 자리만큼 파낸다 — 레일 쪽으로 열린 노치라 마운트를 끼운 뒤 판을
    # 위에서 내려놓을 수 있다. 마운트 bbox 에서 자동으로 따므로 마운트 치수를
    # 바꿔도 노치가 따라온다.
    _NOTCH_FIT = 2.0
    for _m in _mounts:
        _b = _m.bounding_box()
        floor_panel -= Pos((_b.min.X + _b.max.X) / 2, (_b.min.Y + _b.max.Y) / 2, FRAME_TOP) \
            * Box(_b.size.X + 2 * _NOTCH_FIT, _b.size.Y + 2 * _NOTCH_FIT, 4 * PANEL_T)
floor_panel.label, deck_panel.label = "베이바닥판", "데크"


def group(parts, name):
    c = Color(CLR[name])
    for p in parts:
        p.color = c
    g = Compound(children=parts)
    g.label, g.color = name, c
    return g


# 브라켓 자리를 고를 때 피할 것 — 프레임 빔, 판재, 휠브라켓 (bbox 로 빠르게 본다).
# 셸은 속이 빈 상자라 bbox 가 전부를 덮어 후보 선택에 못 쓴다 → 최종 간섭검사로 잡는다.
_OBS = [(n,) + _beam_box(l, st, ax) for n, l, st, ax in BEAMS]
for _p in ([floor_panel, deck_panel] + (_mounts + _clamps if SKID else [])):
    _b = _p.bounding_box()
    _OBS.append((_p.label, (_b.min.X, _b.min.Y, _b.min.Z), (_b.max.X, _b.max.Y, _b.max.Z)))
GROUPS = [group(frame, "프레임"), group(brackets(_OBS), "조인트브라켓")]
if SKID:
    GROUPS += [group(_wheels, "바퀴"), group(_mounts, "휠마운트"),
               group(_clamps, "휠클램프")]
else:
    GROUPS += [
        group(mods, "모듈"),
        group(statics["베어링"], "베어링"),
        group(statics["킹핀브라켓"], "킹핀브라켓"),
        group(statics["조향모터"], "조향모터"),
        group(statics["조향벨트"], "조향벨트"),
        group(statics["원점센서"], "원점센서"),
    ]
GROUPS += [group([floor_panel, deck_panel], "판재")]
if SHELL:
    sh = exterior()
    for x in sh:
        x.label = f"외장{'앞' if x.center().X > 0 else '뒤'}"
    GROUPS.append(group(sh, "외장셸"))
    if FACE:
        cam, brk, led = camera(), cam_bracket(), led_bar()
        cam.label, brk.label, led.label = "Xtion", "카메라브라켓", "LED바"
        GROUPS.append(group([cam], "카메라"))
        GROUPS.append(group([brk], "카메라브라켓"))
        GROUPS.append(group([led], "LED바"))
        es = estop(); es.label = "비상정지"
        GROUPS.append(group([es], "비상정지"))
        GROUPS.append(group(mini_cams(), "소형카메라"))
        GROUPS.append(group(fans(), "배기팬"))
        GROUPS.append(group(bumpers(), "범퍼"))
        GROUPS.append(group(bumper_switches(), "범퍼스위치"))
        GROUPS.append(group(tail_lights(), "후미등"))
        GROUPS.append(group([buzzer()], "부저"))
if INTERIOR:
    GROUPS.append(group([pi5()], "컴퓨터"))
    GROUPS.append(group(boards(), "드라이버"))
    GROUPS.append(group([battery()], "배터리"))
    GROUPS.append(group([battery_bracket()], "배터리브라켓"))
    GROUPS.append(group(power_box(), "전원"))
if DECK_RAIL:
    GROUPS.append(group(deck_rails(), "데크레일"))
    GROUPS.append(group(antennas(), "안테나"))
    GROUPS.append(group(handles(), "핸들"))
if OUTRIGGER:
    GROUPS.append(group(outriggers(), "아웃리거"))
if SIDE_RAIL:
    GROUPS.append(group(side_rails(), "확장레일"))
    if SR_LIDAR:
        GROUPS.append(group(side_lidars(), "2D라이다"))
if INST:
    GROUPS.append(group(inst_module(), "계측모듈"))
    GROUPS.append(group(inst_devices(), "계측장비"))
    GROUPS.append(group(cargo_channels(), "슬라이드채널"))
    GROUPS.append(group(cargo_retainers(), "리테이너"))
if ARM:
    GROUPS.append(group(arm_mount(), "팔마운트"))
    GROUPS.append(group([robot_arm()], "로봇팔"))
    GROUPS.append(group(cargo_channels(), "슬라이드채널"))
    GROUPS.append(group(cargo_retainers(), "리테이너"))
if TOWER:
    GROUPS.append(group(sensor_tower(), "센서타워"))
    GROUPS.append(group(sensor_devices(), "센서장비"))
    GROUPS.append(group(cargo_channels(), "슬라이드채널"))
    GROUPS.append(group(cargo_retainers(), "리테이너"))
if FLAT:
    GROUPS.append(group([flat_deck()], "평판"))
    GROUPS.append(group(flat_ties(), "평판타이다운"))
    GROUPS.append(group(cargo_channels(), "슬라이드채널"))
    GROUPS.append(group(cargo_retainers(), "리테이너"))
if CARGO:
    _sh = cargo_shell(); _sh.label = "함셸"
    GROUPS.append(group([_sh], "적재함"))
    GROUPS.append(group([cargo_floor()], "적재함판재"))
    # 뚜껑은 힌지가 원점인 Compound 로 감싼다. 그래야 ry 한 트랙이 곧 여닫힘이 된다
    # (서스펜션 서스암과 같은 방식).
    _hx, _hz = CARGO_HINGE_AT
    _lid = Pos(_hx, 0, _hz) * Compound(
        children=[Pos(-_hx, 0, -_hz) * c for c in cargo_lid()])
    _lid.label = "뚜껑"
    GROUPS.append(group([_lid], "적재함뚜껑"))
    GROUPS.append(group(cargo_channels(), "슬라이드채널"))
    GROUPS.append(group(cargo_hardware(), "적재함철물"))
    _rf = cargo_cap(True); _rf.label = "장비베이지붕"
    GROUPS.append(group([_rf], "적재함지붕"))
    GROUPS.append(group(cargo_equip(), "장비마운트"))
    GROUPS.append(group(cargo_cradle(), "바구니크래들"))
    GROUPS.append(group(cargo_bin(), "유로컨테이너"))
    GROUPS.append(group(cargo_retainers(), "리테이너"))
    GROUPS.append(group(cargo_ties(), "타이다운"))
if MODULE_ONLY:                       # 구동부 작업대 — 프레임·바퀴 장착부만 남긴다
    _KEEP = {"프레임", "바퀴", "휠클램프", "휠마운트", "조인트브라켓", "모듈", "베어링", "킹핀브라켓",
             "조향모터", "조향벨트", "원점센서"}
    GROUPS = [g for g in GROUPS if g.label in _KEEP]
# ONLY="프레임,바퀴,휠브라켓,외장셸" 처럼 볼 그룹만 골라낸다. MODULE_ONLY 와 달리
# 셸·페이로드를 **짓기는 하고** 표시만 거른다 (셸 구멍 등 상호작용은 그대로 검증된다).
_ONLY = os.environ.get("ONLY")
if _ONLY:
    _want = {s.strip() for s in _ONLY.split(",") if s.strip()}
    _have = {g.label for g in GROUPS}
    assert not (_want - _have), \
        f"ONLY 에 없는 그룹: {sorted(_want - _have)}\n  있는 것: {sorted(_have)}"
    GROUPS = [g for g in GROUPS if g.label in _want]
assembly = Compound(children=GROUPS)
assembly.label = f"{EXPORT_NAME}_module" if MODULE_ONLY else EXPORT_NAME

# ---- 제원 ----
bb = assembly.bounding_box()
print(f"전장 {bb.size.X:.0f} x 전폭 {bb.size.Y:.0f} x 전고 {bb.size.Z:.0f} mm")
print(f"  구동 {DRIVE} / 축간 {WHEELBASE:.0f} / 윤거 {TRACK} / 바퀴 Ø{WHEEL_D}")
if SKID:
    print(f"  스키드 T/L {SKID_TL:.2f} (1.2 이상이면 제자리회전 여유) / "
          f"클램프 판 간격 {BR_SPAN:.1f} mm")
else:
    print(f"  조향 스윕 반경 {SWEEP_R:.0f} mm → 90° 조향 시 전폭 {2*(AXLE_Y+SWEEP_R):.0f} mm")
print(f"  구동부 점유 z 0~{MODULE_TOP} / 차체 z {MODULE_TOP}~{SHELL_TOP+PANEL_T:.0f}"
      f" / 스커트 지상고 {SKIRT_LOW_Z:.0f}")
print(f"  베이 {BASE_L-2*S} x {BASE_W-2*S} x {BAY_H} (바닥 z={BAY_Z:.0f})")
print(f"  데크 {2*(SHELL_X+DECK_LIP2):.0f} x {2*(SHELL_Y+DECK_LIP2):.0f} @ z={SHELL_TOP:.0f}")
print(f"  셸 고정: 상단 플랜지 폭 {FLANGE_W} x {FLANGE_T}t, "
      f"M6 {2*len(FLANGE_BOLT_X)+2*len(FLANGE_BOLT_Y)}개로 상부 프레임 슬롯에 물린다")
print(f"  카메라 Xtion 센서바 {XT_BAR[0]:.0f}x{XT_BAR[1]:.0f} @ z={CAM_Z}, "
      f"창 {CAM_WIN[0]:.0f}x{CAM_WIN[1]:.0f} (여유 {CAM_WIN_CLR}), "
      f"크래들 → 카메라크로스 z={CAM_RAIL_Z:.0f}")
print(f"  외장셸 {2*SHELL_X:.0f} x {2*SHELL_Y:.0f}, 본체 z={MODULE_TOP}~{DECK_Z:.0f}")
print(f"  옆면 스커트 --\\__/-- : 저부 {2*SKIRT_LOW_X}mm @ z={SKIRT_LOW_Z} → "
      f"경사 끝 {2*SKIRT_TOP_X}mm @ z={MODULE_TOP} (앞뒤는 스커트 없음)")
print(f"  뒷면 비상정지 Ø{ESTOP_HOLE:.0f} @ z={ESTOP_Z:.0f} / 포트베이 "
      f"{PORT_BAY[0]}x{PORT_BAY[1]} @ z={PORT_BAY_Z:.0f}, 깊이 {PORT_BAY_D} (커넥터 {len(PORTS)}종)")
print(f"  소형카메라 {len(MINI_CAMS)}개 = " +
      " / ".join(f"{n}" for n, _, _, _ in MINI_CAMS))
if SKID:
    _mv = _mounts[0].volume
    print(f"  휠 마운트: **ㄷ자 마운트 x4** ({WM.MNT_W:.0f}x{WM.MNT_D:.0f}x151.4,"
          f" {_mv/1000:.0f} cm3) + 호버보드 기존 클램프 재사용")
    print(f"     레일(z={RAIL_Z}) 을 ㄷ자로 감싸 M6 4개(밑2+위2)로 물고, 축(z={AXLE_Z:.2f})"
          f" 까지 {WM.GAP:.2f}mm 를 한 덩이로 내려온다")
    print(f"     클램프는 실물 STEP {WM.CL_W}x{WM.CL_H}x{WM.CL_T} — 채널 깊이"
          f" {WM.CL_CH_D} 가 D컷 평면 높이와 같아 **채널이 평면을 물어 회전을 막는다**")
    print(f"     고정: 레일 M6 x4 (스프링너트) + 클램프 M{WM.CLAMP_BOLT:.0f} 탭 x4"
          f"  → 코너당 8, 전체 32")
    print(f"     하중: 횡력 파단 1519 N vs 마찰 최대 218 N (여유 7.0배)"
          f" / 전후는 27배. 주행으론 안 깨지고 측면 충격만 위험하다")
    print(f"     제작: PETG/ASA 프린트(눕혀서, 서포트 0) 또는 6061 CNC(밀링 셋업 1회)."
          f" 파일 exports/print · exports/cnc")
else:
    print(f"  킹핀 중공 Ø{KP_BORE} (인휠 3상+홀 8선 통과) / 조향 하드스톱 ±{STEER_LIMIT:.0f}° "
          f"(±90° 작동 + 여유) / 원점 홀센서 4")
print(f"  범퍼 {BUMPER[0]}x{BUMPER[1]} 앞뒤 2 (리미트 스위치 4) / 후미등 {TAIL[0]}x{TAIL[1]} x2 "
      f"@ z={TAIL_Z:.0f} / 부저 Ø{BUZZER[0]}")
print(f"  핸들 {2*len(HANDLE_XS)} (봉 Ø{HANDLE_BAR} x {HANDLE_SPAN} 셸 관통 → "
      f"하부사이드 레일 직결, x={HANDLE_XS}, z={HANDLE_Z:.0f}) / "
      f"주회로 차단기 + 퓨즈·프리차지")
if INST:
    _cap = 4 * 200                                    # 로드셀 200kg급 x4 [가정]
    print(f"  계측 모듈 — 로드셀 4 @ ({INST_CELL_AT[0]}±{INST_CELL_AT[1]}, "
          f"±{INST_CELL_AT[2]}) / 시험 플랫폼 {INST_TOP[0]}x{INST_TOP[1]} M6 격자")
    print(f"    DAQ 박스 {INST_DAQ[0]}x{INST_DAQ[1]}x{INST_DAQ[2]} + 스트로크 센서 "
          f"커넥터 {INST_CONN_N}채널 + 디스플레이 / IMU / 견인 훅")
    print(f"    잴 수 있다: 페이로드 질량·평면 무게중심(로드셀 4점 배분), "
          f"서스펜션 스트로크→스프링상수·코너하중, 기울기, 견인력")
    print(f"    못 잰다: 바퀴별 하중 (데크 로드셀은 페이로드만 본다 — "
          f"외부 패드 4개나 서스펜션 계장이 필요)")
if ARM:
    _W = 89 + ARM_MASS + ARM_PAYLOAD
    _rest = (89 + ARM_MASS) * 9.81 * 0.26                     # 복원 모멘트 (축간 절반)
    _over = ARM_PAYLOAD * 9.81 * (ARM_REACH / 1000 - 0.26)    # 최대 도달 시 전도 모멘트
    # 서스펜션 처짐 → 툴 위치 흐트러짐
    _M = (ARM_PAYLOAD * (ARM_REACH / 1000) + ARM_MASS * 0.3) * 9.81
    _dF = _M / (WHEELBASE / 1000) / 2                         # 코너당 수직력 변화
    _dz = _dF / 13.0 / 0.85                                   # 쇼크 13 N/mm, 모션비 0.85
    _tilt = 2 * _dz / WHEELBASE
    print(f"  로봇 팔 마운트 — 베이스판 {ARM_PLATE_T}mm, 보스 Ø{ARM_BOSS[0]} "
          f"M{int(ARM_BOLT-0.6)}x{ARM_NB} PCD{ARM_PCD:.0f} [가정], 리브 3본")
    print(f"    팔 [가정] 도달 {ARM_REACH} / 자중 {ARM_MASS}kg / 가반 {ARM_PAYLOAD}kg, "
          f"컨트롤러 {ARM_CTRL[0]}x{ARM_CTRL[1]}x{ARM_CTRL[2]} @ x={ARM_CTRL_AT[0]}")
    print(f"    전복: 복원 {_rest:.0f} N·m vs 최대도달 전도 {_over:.0f} N·m "
          f"→ 여유 {_rest/_over:.1f}배")
    print(f"    서스펜션 처짐: 코너당 {_dz:.1f}mm → 기울기 {math.degrees(_tilt):.2f}° "
          f"→ 툴 이동 약 {_tilt*(ARM_REACH+300):.0f}mm  ← 정밀작업엔 락아웃 필요")
if TOWER:
    _lz = TOWER_H + TOWER_HEAD[2] + LIDAR_RISER + LIDAR[1] / 2
    print(f"  센서 타워 — 마스트 {TOWER_SER}x{TOWER_SER} 기둥 2본 (간격 {2*TOWER_POST_Y}) "
          f"+ 가로타이 2 + 대각 브레이스 4")
    print(f"    3D 라이다 Ø{LIDAR[0]} 스캔면 z={_lz:.0f} / 팬틸트 z={PANTILT_Z} / "
          f"GNSS Ø{GNSS[0]} / IMU (베이스판)")
    print(f"    헤드판 {TOWER_HEAD[0]}x{TOWER_HEAD[1]} @ z={TOWER_H}, 베이스판 M6 격자")
if FLAT:
    print(f"  평판 모듈 {2*SHELL_X:.0f}x{2*SHELL_Y:.0f} R{SHELL_R} @ z={CARGO_SKIRT_Z:.0f}~"
          f"{CARGO_Z0+FLAT_PT+FLAT_LIP:.0f} / M6 {FLAT_GRID}격자 / 타이다운 6")
    print(f"    슬라이드 인터페이스는 적재함과 동일 (스커트 + 채널 3 + 리테이너 3)")
if CARGO:
    _in = (2 * CARGO_IN[0] * 2 * CARGO_IN[1] * (CARGO_Z1 - CARGO_Z0 - 2 * CARGO_PT)) / 1e6
    print(f"  적재함 {2*SHELL_X:.0f}x{2*SHELL_Y:.0f} R{SHELL_R} (셸과 동일 단면) "
          f"z={CARGO_SKIRT_Z:.0f}~{CARGO_Z1:.0f}, 내부 약 {_in:.0f} L")
    print(f"    스커트 z={CARGO_SKIRT_Z:.0f}~{CARGO_Z0:.0f} — 레일 구간을 사방으로 덮는다. "
          f"앞에서 뒤로 밀어 넣고, 레일 노치 3개는 뒷면에만 있다")
    print(f"    슬라이드 채널 3 (레일 측면 슬롯에 턱 물림) / 상부 뚜껑 + 내부 힌지 2 "
          f"+ 솔레노이드 잠금 / 가스 스트럿 2")
    print(f"    벽 {CARGO_T}mm / 격벽 x={CARGO_BULK_X:.0f} → 바구니칸 {CARGO_BAY_X:.0f}"
          f" + 장비베이 {CARGO_EQ_X:.0f}  (격벽 위치는 바구니에서 역산된다)")
    print(f"    뚜껑은 **바구니칸만** 덮는다 ({CARGO_BAY_X:.0f}x{2*CARGO_IN[1]:.0f}) —"
          f" 장비베이 위는 고정 지붕이라 뚜껑을 열어도 안 젖는다")
    print(f"    바구니: 유로 컨테이너 {CARGO_BIN[0]}x{CARGO_BIN[1]}x{CARGO_BIN[2]} "
          f"= {CARGO_BIN[0]*CARGO_BIN[1]*CARGO_BIN[2]/1e6:.0f} L  (KLT / EPP 보온 공용 발자국)")
    print(f"    크래들: 리브 안쪽 여유 {CARGO_BIN_CLR}mm, 바닥에서 {CARGO_FOOT}mm 띄움 "
          f"(배수 + 손가락) / 리브 높이 {CARGO_RIB[1]}")
    print(f"    손 넣는 자리: 짧은 변 {CARGO_BIN_HAND}mm x2 (KLT 손잡이 구멍 쪽) / "
          f"긴 변 {CARGO_IN[1]-CARGO_BIN[1]/2-CARGO_BIN_CLR-CARGO_RIB[0]:.0f}mm x2")
    print(f"    바구니 중심 x={CARGO_BIN_X:.0f} (뒤로 치우침) / 장비베이 M4 {CARGO_EQ_GRID}격자")
    print(f"    타이다운 4 — 바구니 윗단보다 위 벽면 (낮으면 꺼낼 때 손이 걸린다)")
    print(f"    함 고정: 뒤로 = 채널 스토퍼, 앞으로 = 노치 리테이너 3 (레일 끝 M5)")
if OUTRIGGER:
    _hw = SHELL_Y + OUT_PIVOT_DY + OUT_ARM_L * math.sin(math.radians(OUT_ANG))
    print(f"  아웃리거 4 ({'전개' if OUT_DEPLOY else '접힘'}) — 상부 사이드 레일 피벗 "
          f"@ x=±{OUT_X[1]}, 암 {OUT_ARM_L} / 전개각 {OUT_ANG}°")
    print(f"    지지 베이스 {2*OUT_X[1]} x {2*_hw:.0f} (바퀴 {WHEELBASE} x {TRACK}) → "
          f"전복각 좌우 {math.degrees(math.atan(_hw/357)):.0f}° / "
          f"앞뒤 {math.degrees(math.atan(OUT_X[1]/357)):.0f}° (서스펜션 우회)")
if SIDE_RAIL:
    print(f"  측면 확장 레일 {SR_SERIES}x{SR_SERIES} 2본 x {SR_L} @ y=±{SR_Y}, z={SR_Z} "
          f"(브라켓 8 → 하부사이드 레일)")
    if SR_LIDAR:
        print(f"    2D 라이다 Ø{SR_LIDAR_D} 2대 대각 배치 (전방좌/후방우) — "
              f"각 270° 로 360° 커버")
print("  제어축 4개 = 구동 4(인휠)" if SKID else "  제어축 8개 = 구동 4(인휠) + 조향 4")
_J = joints()
# 철물은 **한 종류씩만** 쓴다 (2026-08 확정). 종류가 늘면 조립 때 헷갈리고 남는다.
#   다이캐스팅 브라켓 3025 / 둥근머리 렌지볼트 M6x12 / 스프링너트 306
# 스프링너트는 슬롯 **옆에서 아무 데나** 끼워진다 — 사각너트(50원)보다 비싸지만
# 조립 순서에 안 묶이고, 나중에 브라켓을 옮기거나 추가할 수 있다.
HW = dict(brk=("다이캐스팅 브라켓 3025", 360), bolt=("둥근머리 렌지볼트 M6x15", 70),
          nut=("스프링너트 306 (M6)", 180))
_nb = len(_J)                                   # 조인트 브라켓
_n_wheel = 4 * 4 if SKID else 0                 # 휠마운트 → 레일 M6 (코너당 밑2+위2)
_n_shell = 2 * len(FLANGE_BOLT_X) + 2 * len(FLANGE_BOLT_Y) if SHELL else 0
_n_cap = 0                                      # 축캡 없음 (프로파일 방식)
_bolts = _nb * 2 + _n_wheel + _n_shell + _n_cap
_nuts = _nb * 2 + _n_wheel + _n_shell           # 슬롯에 들어가는 것만
print(f"조인트 철물 — 빔 끝면이 다른 축 빔에 맞대지는 곳을 BEAMS 에서 자동으로 찾는다")
print(f"    {HW['brk'][0]:24s} {_nb:3d}개  {_nb*HW['brk'][1]:8,}원   조인트 {_nb}곳")
print(f"    {HW['bolt'][0]:24s} {_bolts:3d}개  {_bolts*HW['bolt'][1]:8,}원   "
      f"브라켓 {_nb*2} + 휠마운트 {_n_wheel} + 셸 {_n_shell} + 축캡 {_n_cap}")
print(f"    {HW['nut'][0]:24s} {_nuts:3d}개  {_nuts*HW['nut'][1]:8,}원   "
      f"슬롯에 들어가는 것만 (축캡은 브리지에 탭)")
_hw = _nb*HW['brk'][1] + _bolts*HW['bolt'][1] + _nuts*HW['nut'][1]
print(f"    {'철물 합계':24s}       {_hw:8,}원")
cut_list([(n, l) for n, l, _, _ in BEAMS], SERIES)

# NOSHOW=1 → 뷰어 전송 생략. 검증·렌더 스크립트가 이 파일을 import 하면 show() 도
# 같이 돌아서, HUB_MOCK 으로 돌린 목업이 뷰어의 실물 모델을 덮어쓴다 (실제로 그랬다).
if os.environ.get("NOSHOW"):
    print("NOSHOW=1 — 뷰어 전송 생략")
else:
    try:
        set_port(3939)      # 자동 탐색은 콘솔을 찾다가 파이프에서 죽는다
        show(*GROUPS, progress=None,
             names=[g.label for g in GROUPS], colors=[CLR[g.label] for g in GROUPS],
             reset_camera=Camera.RESET)   # 모델이 바뀌면 카메라를 새로 맞춘다
    except Exception as e:
        print(f"뷰어에 표시하지 못함 (뷰어 미실행?): {e}")

# ---- 애니메이션 (조향 + 구름) ----
if ANIMATE and SKID:
    try:
        from ocp_vscode import Animation
        a = Animation()
        T = [0, 2, 4, 6, 8]
        # 스키드는 조향이 없다. 좌우 속도차로 도는 걸 바퀴 회전 방향으로 보여준다.
        # 앞 4초 직진(4륜 같은 방향), 뒤 4초 제자리회전(좌우 반대).
        for i, (x, y) in enumerate(KP_PTS):
            sign = 1 if y > 0 else -1
            a.add_track(f"/Group/바퀴/바퀴{i+1}", "ry",
                        T, [0, 360, 720, 720 + 360 * sign, 720 + 720 * sign])
        if CARGO:
            a.add_track("/Group/적재함뚜껑/뚜껑", "ry",
                        [0, 8.5, 10, 12, 13.5], [0, 0, 72, 72, 0])
        a.animate(speed=1)
        print("애니메이션: 0~4초 직진, 4~8초 제자리회전(좌우 역회전)"
              + (", 8.5~13.5초 뚜껑 여닫힘" if CARGO else ""))
    except Exception as e:
        print(f"애니메이션 실패: {e}")
elif ANIMATE:
    try:
        from ocp_vscode import Animation
        # Animation 은 show() 다음에 만들어야 경로를 안다. 루트는 show 가 만든 "/Group".
        a = Animation()
        T = [0, 2, 4, 6, 8]                       # 초
        STEER = [0, 90, 0, 0, 0]                  # 앞 4초: 조향 왕복
        ROLL = [0, 0, 0, 360, 720]                # 뒤 4초: 바퀴 구름
        # 서스펜션 바운스 — 5초쯤 턱을 넘는다. 앞바퀴가 먼저, 뒷바퀴가 나중에.
        # 서스암 원점이 피벗이라 ry 가 곧 암 각도. 음수 = 압축(바퀴가 위로).
        BUMP_T = [4.0, 4.6, 5.0, 5.4, 5.8, 6.2, 6.6, 7.0, 8.0]
        HIT = [0, -10, 4, -3, 0, 0, 0, 0, 0]      # 턱을 밟고 두어 번 출렁
        MAX_ANG = math.degrees(math.asin(SUS_TRAVEL / ARM_L))
        assert max(abs(v) for v in HIT) <= MAX_ANG, "바운스 각이 트래블을 넘는다"
        for i in range(4):
            n, front = i + 1, KP_PTS[i][0] > 0
            lag = 0 if front else 1.2             # 축간거리만큼 뒷바퀴가 늦게 밟는다
            a.add_track(f"/Group/모듈/모듈{n}", "rz", T, STEER)      # 모듈 통째로 조향
            a.add_track(f"/Group/모듈/모듈{n}/서스암{n}", "ry",
                        [t + lag for t in BUMP_T], HIT)             # 서스펜션 바운스
            a.add_track(f"/Group/모듈/모듈{n}/서스암{n}/바퀴{n}", "ry", T, ROLL)
        if CARGO:                                 # 뚜껑 여닫힘 (힌지 = 격벽 위)
            a.add_track("/Group/적재함뚜껑/뚜껑", "ry",
                        [0, 8.5, 10, 12, 13.5], [0, 0, 72, 72, 0])
        a.animate(speed=1)
        print(f"애니메이션: 0~4초 조향, 4~8초 구름 + 턱 바운스"
              + (", 8.5~13.5초 뚜껑 여닫힘" if CARGO else "") + " "
              f"(암 {abs(min(HIT)):.0f}° = 바퀴 {ARM_L*math.sin(math.radians(abs(min(HIT)))):.0f}mm, "
              f"한계 {MAX_ANG:.0f}°)")
    except Exception as e:
        print(f"애니메이션 실패: {e}")

if not os.environ.get("NOEXPORT"):
    EXPORT_DIR.mkdir(exist_ok=True)
    export_step(assembly, str(EXPORT_DIR / f"{EXPORT_NAME}.step"))
    print(f"내보내기 완료: {EXPORT_DIR / EXPORT_NAME}.step")
