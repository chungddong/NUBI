# Design C — CNC 알루미늄 **통짜 새들 브라켓**.
#
# 이 프로젝트에서 실물이 부러진 뒤 나온 설계다. 무엇이 달라졌나:
#   · 레일을 **축 옆**에 세운다 → 91mm 캔틸레버가 통째로 사라진다 (거기서 부러졌다)
#   · 3060 의 60mm 면이 수직 → 슬롯 2줄이 **30mm 높이차**를 만든다
#     기울임 모멘트를 볼트 인발이 아니라 두 줄의 **커플**로 받는다
#   · 상부 절반은 **호버보드 기존 클램프**를 그대로 쓴다 (채널 5.07 = D컷 평면 높이)
#   · 통짜라 무보강 얇은 판이 존재하지 않는다
#
# 형상은 Y-Z 평면에서 본 ㄴ자다: 안쪽 끝이 백플레이트(레일 2줄을 덮는 높이),
# 바깥쪽은 축을 받치는 블록.

from build123d import *

import parameters as P
from axle import plane_at_y

# 이 패키지의 z 데이텀은 **축 중심 = 0** 이다 (axle.py / split_clamp.py 전부 그렇다).
# 새들만 지면 기준(z=AXLE_Z)으로 지었다가 축이 79mm 아래에 따로 떠 버렸다.
AZ = 0.0
GROUND = -P.AXLE_Z              # 지면 z (지상고 계산용)

# 레일 높이는 **아래 슬롯줄이 블록 윗면 위로 나오도록** 역산한다.
# 축보다 10mm 만 올렸더니 아래 줄(z=74.3)이 블록 살(51.3~79.35) 속에 묻혀
# 바깥에서 알루미늄 47mm 를 뚫어야 볼트가 들어갔다 — 조립이 불가능한 형상이었다.
RAIL_CZ = AZ + P.SADDLE_RAIL_CLR + P.EXTRUSION_SLOT_PITCH / 2
CLAMP_CY_ = P.AXLE_FLAT_Y0 - P.AXLE_FLAT_LENGTH / 2
# 레일 위치는 **기존 클램프 안쪽면에서 역산**한다. 평면 끝에서 6mm 로 잡았더니
# 백플레이트가 클램프를 2585mm3 파고들었다 — 클램프가 축 위로 올라와 있어서
# 백플레이트의 z 구간과 겹친다.
# 간극은 **필렛 반경 + 2** 여야 한다. 2.0 으로 줬더니 블록-백플레이트 오목
# 필렛 비드가 클램프를 9mm3 밀었다 (평면끼리는 안 닿는데 비드가 닿는다).
RAIL_FACE_Y = (CLAMP_CY_ - P.OEM_H / 2) - (P.SADDLE_FILLET + 2.0) - P.SADDLE_BACK_T
SLOT_ZS = (RAIL_CZ - P.EXTRUSION_SLOT_PITCH / 2,
           RAIL_CZ + P.EXTRUSION_SLOT_PITCH / 2)
# 블록 윗면(클램프 맞댐면)은 축 중심보다 **핀치갭만큼 낮다.** 0 으로 두면 기존
# 클램프가 축(평면 z=+5.0)이 아니라 새들 윗면에 먼저 닿아, 축과 0.07mm 뜬 채로
# 볼트만 조여진다 — 축이 그냥 미끄러진다. 낮춰야 볼트 예압이 축을 통해 걸린다.
BLOCK_TOP = AZ - P.SADDLE_PINCH
BLOCK_BOT = AZ - P.AXLE_DIAMETER / 2 - P.SADDLE_UNDER
BACK_TOP = SLOT_ZS[1] + P.TNUT_BOLT + 6
BACK_BOT = min(BLOCK_BOT, SLOT_ZS[0] - P.TNUT_BOLT - 6)
RAIL_BOT = RAIL_CZ - P.EXTRUSION_H / 2                     # 레일 밑면
SHELF_TOP = RAIL_BOT                                       # 선반 윗면 = 레일 밑면
CLAMP_CY = CLAMP_CY_                                       # 기존 클램프 중심 y

assert RAIL_FACE_Y + P.SADDLE_BACK_T <= CLAMP_CY_ - P.OEM_H / 2,     "백플레이트가 기존 클램프를 파고든다"
assert P.SADDLE_BOLT_X - P.TNUT_BOLT / 2 > P.OEM_W / 2 + 3, \
    "레일 볼트가 기존 클램프와 겹쳐 공구가 안 들어간다"
assert P.SADDLE_W / 2 >= P.SADDLE_BOLT_X + P.TNUT_BOLT, "레일 볼트가 브라켓 밖으로 나간다"
assert SLOT_ZS[0] - P.TNUT_BOLT > BLOCK_TOP, \
    "아래 슬롯줄이 블록 살에 묻힌다 — 볼트를 넣을 수 없다"
FLAT_H = P.AXLE_ACROSS_FLAT - P.AXLE_DIAMETER / 2       # 평면 높이 (축중심 위)
CLAMP_SEAT_Z = FLAT_H - P.OEM_CH_D      # 클램프가 실제로 앉는 z (= -0.07)
# 클램프는 채널 천장이 평면에 닿는 데서 멈춘다. 채널(5.07)이 평면(5.0)보다 0.07
# 깊으니 **핀치갭은 그 0.07보다 커야** 맞댐면이 아니라 축에 앉는다.
# 처음엔 부등호를 거꾸로 걸어 "채널 깊이를 넘으면 안 된다"고 썼는데, 넘어야 맞다.
assert P.SADDLE_PINCH > P.OEM_CH_D - FLAT_H + 0.1, \
    "핀치갭이 작아 클램프가 축이 아니라 새들 윗면에 먼저 앉는다 (축이 미끄러진다)"


def saddle_bracket():
    r = P.AXLE_DIAMETER / 2 + P.SADDLE_SEAT_FIT
    y0, y1 = RAIL_FACE_Y, P.SADDLE_Y_OUT               # 안쪽 / 바깥 끝
    with BuildPart() as p:
        with Locations((0, (y0 + y1) / 2, (BLOCK_BOT + BLOCK_TOP) / 2)):  # 축 받침 블록
            Box(P.SADDLE_W, y1 - y0, BLOCK_TOP - BLOCK_BOT)
        with Locations((0, y0 + P.SADDLE_BACK_T / 2,                      # 백플레이트
                        (BACK_BOT + BACK_TOP) / 2)):
            Box(P.SADDLE_W, P.SADDLE_BACK_T, BACK_TOP - BACK_BOT)
        try:
            fillet(p.edges().filter_by(Axis.X), radius=P.SADDLE_FILLET)
        except Exception:
            pass
        # 시트는 **축이 실제로 있는 구간까지만** 판다. y1-y0 전체로 팠더니 백플레이트
        # 한복판(기울임 모멘트를 받는 자리)에 D자 구멍이 뚫렸다.
        sy0 = max(P.AXLE_END_Y, y0 + P.SADDLE_BACK_T)     # 시트 안쪽 끝
        sy1 = y1 + 1                                      # 바깥은 열어둔다
        with Locations((0, (sy0 + sy1) / 2, AZ)):                   # 축 반원 시트
            Cylinder(r, sy1 - sy0, rotation=(90, 0, 0), mode=Mode.SUBTRACT)
        if P.SADDLE_SHELF > 0:                                            # 레일 밑 선반
            # 기울임의 지레는 선반 없이도 확보된다 (백플레이트가 레일 바깥면에
            # 눌리는 접촉점이 레일 밑모서리 z=59.3 에 잡힌다). 선반이 하는 일은
            # 다른 둘이다: ① 800N 수직을 볼트 전단이 아닌 **직접 지지**로 받는다
            # ② 조립할 때 브라켓 1.6kg 을 레일에 걸어두고 볼트를 시작할 수 있다.
            # **안쪽(-Y)** 으로 뻗어야 레일 밑에 들어간다. +Y 로 만들었더니 이미 있는
            # 블록 살 속에 묻혀 부피가 1mm3 도 안 늘었다.
            with Locations((0, y0 - P.SADDLE_SHELF / 2,
                            (BACK_BOT + SHELF_TOP) / 2)):
                Box(P.SADDLE_W, P.SADDLE_SHELF, SHELF_TOP - BACK_BOT)
        for sx in (-1, 1):                                                # 기존 클램프 탭 4
            for sy in (-1, 1):
                with Locations((sx * P.OEM_BOLT[0], CLAMP_CY + sy * P.OEM_BOLT[1],
                                (BLOCK_BOT + BLOCK_TOP) / 2)):
                    Cylinder((P.OEM_BOLT_D - 1.0) / 2, BLOCK_TOP - BLOCK_BOT + 2,
                             mode=Mode.SUBTRACT)
        for sx in (-1, 1):                                                # 레일 볼트 4 (2줄)
            for z in SLOT_ZS:
                with Locations((sx * P.SADDLE_BOLT_X, y0 + P.SADDLE_BACK_T / 2, z)):
                    Cylinder(P.TNUT_BOLT * 0.55, P.SADDLE_BACK_T + 4,
                             rotation=(90, 0, 0), mode=Mode.SUBTRACT)
    p.part.label = "saddle_bracket"
    return p.part


def oem_clamp_block():
    """기존 호버보드 클램프 (참조) — 축 위에 얹힌다."""
    with BuildPart() as p:
        with Locations((0, CLAMP_CY, CLAMP_SEAT_Z + P.OEM_T / 2)):
            Box(P.OEM_W, P.OEM_H, P.OEM_T)
        with Locations((0, CLAMP_CY, CLAMP_SEAT_Z)):      # D컷 채널
            Box(P.AXLE_DIAMETER, P.OEM_H + 2, 2 * P.OEM_CH_D, mode=Mode.SUBTRACT)
    p.part.label = "oem_clamp_reference"
    return p.part


def rail():
    from extrusion_reference import extrusion_reference
    return Pos(0, RAIL_FACE_Y - P.EXTRUSION_W / 2, RAIL_CZ) * extrusion_reference()
