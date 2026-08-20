"""기존 760x450 3030 차량 프레임에 Design D 휠 마운트 4세트를 적용한다."""

import sys
from pathlib import Path

HERE = Path(__file__).parent
sys.path[:0] = [str(HERE), str(HERE.parent)]

from build123d import Color, Compound, Location, Pos, Rot  # noqa: E402
from ocp_vscode import Camera, set_port, show               # noqa: E402

import low_clamp as LC                                      # noqa: E402
import parameters as P                                      # noqa: E402
from motor_reference import actual_motor_step_reference     # noqa: E402
from tslot import bar                                       # noqa: E402


PORT = 3939
BASE_L, BASE_W = 760.0, 450.0
S = 30.0
AXLE_X = 210.0
AXLE_Z = P.WHEEL_DIAMETER / 2
RAIL_Y = (BASE_W - S) / 2
RAIL_Z = AXLE_Z + LC.RAIL_CZ
FRAME_TOP = RAIL_Z + S / 2
POST_H = 143.0
# Final compact frame: match the upper-frame outside width to the 450 mm
# lower frame.  With 30 mm side rails the between-rail cross member is 390 mm.
TOP_RAIL_Y = RAIL_Y
TOP_RAIL_Z = FRAME_TOP + POST_H + S / 2
CROSS_X = (BASE_L - S) / 2

# local rail y=CLAMP_CY가 차량 rail y=RAIL_Y에 오도록 휠 중심을 정한다.
WHEEL_Y = RAIL_Y - LC.CLAMP_CY


def _group(parts, label, color):
    c = Color(color)
    for p in parts:
        p.color = c
    g = Compound(children=parts)
    g.label, g.color = label, c
    return g


def vehicle_frame():
    beams = []

    def add(name, length, start, axis):
        p = bar(30, length, start, axis)
        p.label = name
        beams.append(p)

    for i, sy in enumerate((-1, 1)):
        add(f"하부사이드{i+1}", BASE_L, (-BASE_L / 2, sy * RAIL_Y, RAIL_Z), "X")
    cross_l = BASE_W - 2 * S
    for i, x in enumerate((-CROSS_X, 0, CROSS_X)):
        add(f"하부크로스{i+1}", cross_l, (x, -cross_l / 2, RAIL_Z), "Y")
    # 기존 스키드 차량의 축 하중 보강 크로스.
    for i, x in enumerate((-(AXLE_X + 55), AXLE_X + 55)):
        add(f"축크로스{i+1}", cross_l, (x, -cross_l / 2, RAIL_Z), "Y")
    for i, (x, sy) in enumerate((x, sy) for x in (-CROSS_X, 0, CROSS_X)
                                 for sy in (-1, 1)):
        add(f"기둥{i+1}", POST_H, (x, sy * RAIL_Y, FRAME_TOP), "Z")
    top_cross_l = 2 * TOP_RAIL_Y - S
    for i, x in enumerate((-CROSS_X, 0, CROSS_X)):
        add(f"상부크로스{i+1}", top_cross_l,
            (x, -top_cross_l / 2, TOP_RAIL_Z), "Y")
    for i, sy in enumerate((-1, 1)):
        add(f"상부사이드{i+1}", BASE_L,
            (-BASE_L / 2, sy * TOP_RAIL_Y, TOP_RAIL_Z), "X")
    return beams


def mounted_wheels():
    motor0 = actual_motor_step_reference(axle_z=0)
    lower0, upper0 = LC.low_clamp(), LC.oem_clamp_block()
    # 길이·개수를 박지 않는다. 플랜지를 끄면 옆 볼트는 빈 목록이 돌아오고,
    # 길이는 parameters.LOW_BOLT_LEN(실측 15mm)에서 나온다.
    side0 = LC.side_button_head_bolts_reference()
    bottom0 = LC.bottom_button_head_bolts_reference()

    motors, lowers, uppers, bolts = [], [], [], []
    for i, (x, sy) in enumerate((x, sy) for x in (-AXLE_X, AXLE_X)
                                 for sy in (-1, 1)):
        y = sy * WHEEL_Y
        # 기본 형상은 +Y 차륜 기준(축이 -Y=차량 안쪽). -Y 측은 Z축 180° 회전.
        tf = Pos(x, y, AXLE_Z) * (Rot(0, 0, 180) if sy < 0 else Location())
        w = tf * motor0.moved(Location())
        lo = tf * lower0.moved(Location())
        up = tf * upper0.moved(Location())
        w.label, lo.label, up.label = f"휠모터{i+1}", f"하부클램프{i+1}", f"상부클램프{i+1}"
        motors.append(w); lowers.append(lo); uppers.append(up)
        for j, b0 in enumerate(side0 + bottom0):
            b = tf * b0.moved(Location())
            b.label = f"프로파일볼트_{i+1}_{j+1}"
            bolts.append(b)
    return motors, lowers, uppers, bolts


def tie_rods():
    """앞뒤 축마다 좌우 클램프를 잇는 인장 전산볼트.

    좌우 클램프는 젖힘 모멘트로 **아래쪽이 서로 반대로 벌어진다.** 그래서 이 부재는
    순수 인장만 받는다 — 나사봉이면 충분하고, 클램프 안쪽면 탭에 바로 박힌다.
    """
    rod0 = LC.tie_rods(RAIL_Y)
    out = []
    for i, x in enumerate((-AXLE_X, AXLE_X)):
        for j, r0 in enumerate(rod0):
            r = Pos(x, 0, AXLE_Z) * r0.moved(Location())
            r.label = f"타이봉_{i+1}_{j+1}"
            out.append(r)
    return out


def main():
    motors, lowers, uppers, bolts = mounted_wheels()
    groups = [
        _group(vehicle_frame(), "변경차량_3030프레임_상하폭450", "#69727d"),
        _group(lowers, "DesignD_하부클램프_4개", "#c9ced4"),
        _group(uppers, "기존_상부클램프_4개", "#f5a623"),
        _group(motors, "실제_휠모터_STEP_4개", "#34383d"),
        _group(bolts, f"M{P.TNUT_BOLT:.0f}x{P.LOW_BOLT_LEN:.0f}_프로파일체결볼트_{len(bolts)}개",
               "#30343a"),
    ]
    ties = tie_rods()
    if ties:
        groups.append(_group(ties, f"액슬타이봉_M{P.TIE_ROD:.0f}x{2*(RAIL_Y-P.LOW_D/2+P.TIE_ENGAGE):.0f}_{len(ties)}개", "#b6483c"))
    set_port(PORT)
    show(*groups, names=[g.label for g in groups],
         colors=[g.color for g in groups],
         alphas=[1]*len(groups), progress=None, reset_camera=Camera.RESET)
    print(
        "VEHICLE_INTEGRATION_SENT "
        f"wheels=4 wheel_y=+/-{WHEEL_Y:.1f} rail_z={RAIL_Z:.1f} "
        f"lower_cross={BASE_W - 2 * S:.0f} "
        f"upper_cross={2 * TOP_RAIL_Y - S:.0f}"
    )


if __name__ == "__main__":
    main()
