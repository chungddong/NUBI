# 허브모터 단품 스탠드 — 바퀴 하나를 공중에 띄워 돌려보는 최소 지그.
#
# refmodel/motorHolder.step 참고. 그건 축이 **세로**인 모터용이라 원뿔 + 방사 리브 +
# 쪼개진 클램프 보스 구조였다. 우리 허브모터는 축이 **가로**라 축대칭이 안 되고
# 파일런(기둥) 형태가 된다. 가져온 건 형상이 아니라 방식이다 —
#   "플랜지 + 테이퍼 몸통 + 클램프 보스를 한 덩어리로 뽑는다."
#
# wheel_jig.py(3030 프로파일 7본, 34,600원)와 목적이 다르다.
#   이 스탠드 : 모터가 도는가 / 드라이버 설정 / 홀센서 결선 / 전류·속도  ← 프린트, 재료비만
#   프로파일 지그 : 위에 더해 **휠브라켓이 축을 제대로 잡는가**, 나중에 스워브 모듈까지
#                   (로봇의 3030 레일 인터페이스를 그대로 복제하기 때문)
#
# 클램프를 판 두 장이 아니라 **한 덩어리 보스**로 바꾼 이유:
#   판 2장(8t x2 = 16mm 물림)으로 하면 D컷 평면 지압이 22 MPa 라 PETG 항복 45 의
#   2배밖에 안 남는다. 보스 하나로 D컷 평면 전 구간(42.8mm)을 물면 8.3 MPa 로 떨어져
#   여유가 5.4배가 된다. 프린트에서는 길게 무는 게 두껍게 무는 것보다 효율이 좋다.
#
# **반드시 바닥/벤치에 볼트로 고정해서 쓴다.** 구동 반력 29 N·m 을 무게로 버티려면
# 자중 4 kg 기준 반경 740mm 짜리 받침이 필요하다. 앵커 4개면 개당 145 N 이면 끝난다.

import os
from pathlib import Path

from build123d import *
from ocp_vscode import Camera, set_port, show

os.environ.setdefault("NOSHOW", "1")
os.environ.setdefault("DRIVE", "skid")

import swerve_base as M                                          # noqa: E402

# ---- 파라미터 (단위: mm, 지면 z=0) ----
AXLE_Z = 120                  # 축 높이 → 바퀴 밑면 지상고 40.6
BOSS_D, BOSS_W = 46, 64       # 클램프 블록 (높이 z, 폭 x) — 캡 볼트 살까지 포함
BOLT_X = 23                   # 캡 볼트 x (±) — 보어 Ø16.3 과 블록 끝 사이
PINCH = 6.6                   # 캡 볼트 M6 통과공
CAP_PILOT = 5.0               # 본체 하공 (M6 열삽입 인서트 / 셀프탭)
FLANGE = (170, 190, 10)       # 베이스 플랜지 (x, y, 두께)
FLANGE_R = 22
FLANGE_Y0 = -140              # 플랜지 y 범위 — 바퀴 쪽으로도 조금 나가야 안 넘어간다
BOLT_D, BOLT_INSET = 8.5, 22  # 바닥 앵커 M8 / 모서리에서 들어간 거리
# 몸통은 통짜 파일런이 아니라 **핀 2장**이다. 구동 반력 29 N·m 은 축을 비트는데,
# 핀을 x 로 벌려 두면 그 비틀림이 핀 두 장의 상하 커플(±725 N)로 바뀐다 — 판의
# 강한 방향이다. 통짜로 뽑으면 1350 cm3 (프린트 12시간)라 쓸 수가 없다.
FIN_X, FIN_T, FIN_SPREAD = 20, 12, 38     # 핀 중심 x / 두께 / 밑동이 y 로 퍼지는 양
WEB_T = 8                     # 두 핀을 잇는 가운데 웹 (좌굴·비틀림 보강)
TIE_Y, TIE_Z, TIE_SLOT = -70, (50, 80), (5, 3)   # 케이블타이 슬롯 (y, z 두 곳, 크기)

# 클램프 구간은 **D컷 평면이 있는 데까지만** 문다. 평면 없는 원형 구간을 물면
# 축이 돌고, 실제로 브라켓에서 34 mm3 간섭으로 한 번 잡힌 자리다.
Y0, Y1 = M.BR_FLAT                        # -75.1 ~ -32.3 (타이어 중심면 기준)
BOSS_L = Y1 - Y0
EXPORT_DIR = Path(__file__).parent.parent / "exports"

assert BOSS_L > 40, f"클램프 물림이 짧다 ({BOSS_L:.1f}mm)"
assert AXLE_Z - M.WHEEL_D / 2 > FLANGE[2] + 20, "바퀴가 플랜지에 닿는다"


def _blank():
    """구멍 뚫기 전 통짜 — 플랜지 + 핀 2장 + 웹 + 클램프 보스."""
    y_mid = (Y0 + Y1) / 2
    with BuildPart() as p:
        # 베이스 플랜지
        with BuildSketch(Plane.XY.offset(0)):
            with Locations((0, FLANGE_Y0 + FLANGE[1] / 2)):
                RectangleRounded(FLANGE[0], FLANGE[1], FLANGE_R)
        extrude(amount=FLANGE[2])
        # 핀 2장 — Plane.YZ 의 로컬 좌표는 (y, z) 다. 밑동을 y 로 퍼뜨려 사다리꼴로.
        for sx in (-1, 1):
            with BuildSketch(Plane.YZ.offset(sx * FIN_X - FIN_T / 2)):
                # 앞(+y)으로는 벌리지 않는다. 바퀴가 축 둘레 원통이라 x=±20 지점에서도
                # z=43 부터 살이 있어서, 앞으로 기운 경사는 바로 파고든다 (6483 mm3).
                Polygon((Y0 - FIN_SPREAD, FLANGE[2]), (Y1, FLANGE[2]),
                        (Y1, AXLE_Z), (Y0, AXLE_Z), align=None)
            extrude(amount=FIN_T)
        # 가운데 웹 — 두 핀을 이어 비틀림을 받는다 (보스 밑까지만)
        z_web = AXLE_Z - BOSS_D / 2 - 2
        with BuildSketch(Plane.YZ.offset(-WEB_T / 2)):
            Polygon((Y0 - FIN_SPREAD, FLANGE[2]), (Y1, FLANGE[2]),
                    (Y1, z_web), (Y0, z_web), align=None)
        extrude(amount=WEB_T)
        # 클램프 보스 — 캡 볼트가 물릴 살이 있어야 해서 원기둥이 아니라 각블록이다
        with Locations((0, y_mid, AXLE_Z)):
            Box(BOSS_W, BOSS_L, BOSS_D)
        # 배선 타이 슬롯 — 인휠 8선(3상+홀)이 축 속을 지나 **축 끝(y=-79.6)** 으로 나온다.
        # 클램프는 y=-75.1 에서 끝나 출구를 막지 않지만, 잡아줄 데가 없으면 다발이
        # 늘어져 바퀴에 감긴다. 웹에 케이블타이 구멍 2개를 뚫어 둔다.
        for z in TIE_Z:
            with Locations((0, TIE_Y, z)):
                Box(WEB_T + 4, TIE_SLOT[0], TIE_SLOT[1], mode=Mode.SUBTRACT)
        # 바닥 앵커 M8 x4
        for sx in (-1, 1):
            for y in (FLANGE_Y0 + BOLT_INSET, FLANGE_Y0 + FLANGE[1] - BOLT_INSET):
                with Locations((sx * (FLANGE[0] / 2 - BOLT_INSET), y, FLANGE[2] / 2)):
                    Cylinder(BOLT_D / 2, FLANGE[2] + 4, mode=Mode.SUBTRACT)
    return p.part


def stand():
    """본체 + 캡 2조각. 축 중심에서 갈라 축을 **위에서 떨군다**.

    통짜 D컷 보어로는 조립이 불가능하다 — 보어 높이가 13.3 인데 나사 언더컷 구간이
    Ø16 완전 원형이라 축 방향으로 밀어 끼울 수가 없고, 반대쪽은 숄더(Ø39)가 막는다.
    호버보드 프레임이 전부 열린 슬롯인 이유가 이거다.

    가르는 면은 **축 중심(z=AXLE_Z)** 이다. 그보다 위에서 자르면 입구가 16 보다
    좁아져 축이 안 떨어지고, 아래에서 자르면 D컷 평면이 본체 쪽으로 넘어간다.
    평면은 +Z 쪽이라 **캡이 회전 방지를 맡고**, 토크는 캡 → 볼트 → 본체로 흐른다.
    """
    y_mid = (Y0 + Y1) / 2
    r = M.HUB_SHAFT_D / 2 + M.BR_FIT / 2
    off = M.HUB_FLAT - M.HUB_SHAFT_D / 2 + M.BR_FIT / 2
    with BuildPart() as p:
        add(_blank())
        with Locations((0, y_mid, AXLE_Z)):                       # 축 보어
            Cylinder(r, BOSS_L + 4, rotation=(90, 0, 0), mode=Mode.SUBTRACT)
        with Locations((0, y_mid, AXLE_Z + (off + r) / 2)):        # D컷 평면 복원
            Box(2 * r, BOSS_L, r - off)
    full = p.part
    big = 400
    lower = full & Pos(0, y_mid, AXLE_Z - big / 2) * Box(big, big, big)
    upper = full & Pos(0, y_mid, AXLE_Z + big / 2) * Box(big, big, big)
    # 캡 볼트 — 캡은 통과공, 본체는 하공(열삽입 인서트 M6 권장).
    # 토크 29 N·m 을 볼트 2개가 ±BOLT_X 에서 커플로 받는다.
    with BuildPart() as pc:
        add(upper)
        for sx in (-1, 1):
            with Locations((sx * BOLT_X, y_mid, AXLE_Z + BOSS_D / 4)):
                Cylinder(PINCH / 2, BOSS_D, mode=Mode.SUBTRACT)
    with BuildPart() as pb:
        add(lower)
        for sx in (-1, 1):
            with Locations((sx * BOLT_X, y_mid, AXLE_Z - BOSS_D / 4)):
                Cylinder(CAP_PILOT / 2, BOSS_D, mode=Mode.SUBTRACT)
    return pb.part, pc.part


body_part, cap_part = stand()
body_part.label, body_part.color = "스탠드본체", Color("#6b7480")
cap_part.label, cap_part.color = "클램프캡", Color("#8d9298")
wheel = Pos(0, 0, AXLE_Z) * M.wheel()
wheel.label, wheel.color = "바퀴", Color("#2b2b2b")
stand_part = body_part                       # 하위 호환 (렌더 스크립트용)

assembly = Compound(children=[body_part, cap_part, wheel])
assembly.label = "motor_stand"

# ---- 제원 ----
bb = assembly.bounding_box()
sb = (body_part + cap_part).bounding_box()
vol = body_part.volume + cap_part.volume
print(f"허브모터 단품 스탠드  {bb.size.X:.0f} x {bb.size.Y:.0f} x {bb.size.Z:.0f} mm")
print(f"  스탠드 {sb.size.X:.0f} x {sb.size.Y:.0f} x {sb.size.Z:.0f}"
      f" / 본체 {body_part.volume/1000:.0f} + 캡 {cap_part.volume/1000:.0f} = "
      f"{vol/1000:.0f} cm3 → PETG 40% 인필 약 {vol*1.27e-3*0.5:.0f} g")
print(f"  축 높이 {AXLE_Z} → 바퀴 지상고 {AXLE_Z - M.WHEEL_D/2:.1f} mm (공중 회전)")
print(f"  클램프 {BOSS_W}x{BOSS_D} x {BOSS_L:.1f}L, 축 Ø{M.HUB_SHAFT_D} D컷{M.HUB_FLAT}"
      f" (평면 구간 전체를 문다)")
print(f"  **축 중심에서 갈라 위에서 떨군다** — 통짜 D컷 보어(높이 13.3)는 나사 언더컷의"
      f" Ø16 원형 구간을 못 지나 조립 자체가 불가능하다. 캡 볼트 M6 x2 (x=±{BOLT_X})")
# D컷은 평면과 그 반대편 원호가 짝으로 토크를 받는다 → 유효 반경 ~ 축 반경의 8할
_F = 29000 / (0.8 * M.HUB_SHAFT_D / 2)
_A = BOSS_L * 2 * (M.HUB_SHAFT_D ** 2 / 4 - (M.HUB_FLAT - M.HUB_SHAFT_D / 2) ** 2) ** 0.5
print(f"  구동 토크 29 N·m → D컷 평면 지압 {_F/_A:.1f} MPa (PETG 항복 45 → 여유 "
      f"{45/(_F/_A):.1f}배)")
print(f"  ** 바닥 앵커 M8 x4 필수 — 반력 29 N·m 을 자중으로 버티려면 받침 반경 740mm 가"
      f" 필요하다. 앵커로 가면 개당 {29/(2*(FLANGE[0]/2-BOLT_INSET)/1000):.0f} N 이다")
print(f"  배선: 인휠 8선이 축 속 → 축 끝(y={-79.6:.1f})으로 나온다. 클램프는 y={Y0:.1f}"
      f" 에서 끝나 출구를 안 막는다. 웹에 케이블타이 슬롯 {TIE_SLOT[0]}x{TIE_SLOT[1]} x2")
print(f"  프린트: 본체는 플랜지를 바닥에, 캡은 평면을 바닥에 놓고 뽑는다."
      f" 벽 4겹 이상, 인필 40% 이상. 캡 볼트는 M6 x 35 + 본체에 열삽입 인서트 권장")

EXPORT_DIR.mkdir(exist_ok=True)
# assembly 에 넣은 조각은 재부모화돼서 export 가 죽는다 → 새로 만들어 내보낸다
for _n, _p in zip(("motor_stand_body", "motor_stand_cap"), stand()):
    export_step(_p, str(EXPORT_DIR / f"{_n}.step"))
print(f"내보내기 완료: {EXPORT_DIR}\motor_stand_body.step, _cap.step")

if not os.environ.get("STAND_NOSHOW"):
    try:
        set_port(3939)
        show(body_part, cap_part, wheel, progress=None,
             names=["스탠드본체", "클램프캡", "바퀴"],
             colors=["#6b7480", "#8d9298", "#2b2b2b"], reset_camera=Camera.RESET)
    except Exception as e:
        print(f"뷰어에 표시하지 못함 (뷰어 미실행?): {e}")
