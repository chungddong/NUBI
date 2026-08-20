# 단륜 시험 지그 — 바퀴 한 개만 올려놓고 구동·조향·하중을 시험한다.
#
# 목적: 로봇 한 대를 다 만들기 전에 구동부만 검증한다.
#   · 허브모터가 도는가 (드라이버 설정, 홀센서 결선, 전류·속도)
#   · 브라켓이 축을 제대로 잡는가 (구동 토크 29 N·m 에 안 도는가, 흔들림 없는가)
#   · 나중에 스워브로 올릴 때 — 조향 토크 실측, 하드스톱, 원점센서, 킹핀 배선
#
# 핵심 원칙 1: **로봇의 장착 인터페이스를 그대로 복제한다.**
#   장착 레일을 로봇의 하부사이드 레일과 같은 3030, 같은 y=+210, 같은 높이에 둔다.
#   지그 전용 어댑터를 만들면 "지그에서는 됐는데 로봇에서는 안 되는" 차이의 출처를
#   알 수 없다.
#
# 핵심 원칙 2: **프레임 크기를 시험 대상 bbox 에서 역산한다.**
#   처음엔 로봇의 하부 프레임을 통째로 베껴 반대쪽 레일(y=-210)과 기둥까지 넣고
#   높이도 스워브 기준으로 고정해 400x800x514 가 나왔다. 그런데 실측해 보니
#   휠브라켓도 킹핀브라켓도 **+Y 레일 한 본에만** 물리고, y<176 으로는 아무것도
#   내려오지 않는다. 반대쪽 절반은 통째로 낭비였다.
#
# **바퀴는 항상 공중에 띄운 채로 시험한다** (지상고 LIFT). 그래서
#   · 3점 지지·조절 발·상단 추 데크가 전부 필요 없다 — 베이스는 바닥에 그냥 놓는다
#   · 베이스가 바퀴 **밑으로** 지나가도 된다 → 조향 스윕을 피해 앞으로 뻗을 이유가 없다
#     (스워브 스윕이 y=344 까지 가지만 z=50 위라서 z 0~30 인 베이스와 안 만난다)
#
# 대신 새 제약이 생긴다. 바퀴가 떠 있으면 구동 토크가 지면으로 안 빠지고
# **반력 29 N·m 이 그대로 지그를 축(Y) 둘레로 뒤집으려 한다.**
#   버티는 힘 = 총무게 x 베이스 반폭.  지그+바퀴 6 kg, 반폭 134mm → 7.9 N·m 뿐이다.
#   → 네 귀 M8 앵커로 바닥/벤치에 고정해서 쓴다. 무게로 버티려면 15 kg 을 더 얹어야 한다.
#
# JIG=skid|swerve 로 무엇을 얹을지 고른다 (swerve_base 의 DRIVE 와 같이 움직인다).

import os
from pathlib import Path

from build123d import *
from ocp_vscode import Camera, set_port, show

# swerve_base 를 import 하면 그 파일의 show()·export 도 같이 돈다 → 막는다.
JIG = os.environ.get("JIG", "skid")
assert JIG in ("skid", "swerve"), f"JIG 는 skid 또는 swerve (받은 값: {JIG})"
os.environ["DRIVE"] = JIG
os.environ["NOSHOW"] = "1"

import swerve_base as M                                          # noqa: E402
from tslot import ALU_DENSITY, PROFILES, bar, cut_list, snap_cut           # noqa: E402

SERIES = M.SERIES
S = PROFILES[SERIES]["size"]


# ---- 뼈대 치수 (시험 대상보다 먼저 정해진다) ----
# 절단 길이를 **본체 재단표의 390 하나로 통일**한다. 시험이 끝나면 그대로 본체
# (축크로스·하부크로스·카메라크로스 6본)로 옮겨 쓴다. 그래서 가공(홀·탭)은 금지 —
# 구멍이 뚫리는 순간 재활용이 막힌다. 조립은 브라켓 + T너트로만 한다.
REUSE_LEN = 390
BASE_TOP = S                              # 베이스 프로파일 윗면 (바닥에 그냥 놓는다)
POST_TOP = BASE_TOP + REUSE_LEN           # 기둥 윗면
RAIL_Z = POST_TOP - S / 2                 # 장착 레일 = 기둥 맨 위
# 바퀴 지상고는 여기서 **역산**된다. 레일을 기둥 꼭대기에 못박아 두면 스키드/스워브
# 사이에 레일을 옮길 필요조차 없다 — 바퀴만 다른 높이에 뜬다.
LIFT = RAIL_Z - M.RAIL_Z


def specimen():
    """바퀴 + 장착부. 로봇 좌표의 한 코너를 지그 원점(x=0)으로 옮기고 LIFT 만큼 띄운다."""
    groups = {}
    if JIG == "skid":
        W, BR = M.wheel(), M.wheel_bracket()
        SPS = M.axle_cap() + M.axle_spacer()
        w = Pos(0, M.AXLE_Y, M.AXLE_Z + LIFT) * W
        w.label = "바퀴"
        groups["바퀴"] = [w]
        b = Pos(0, M.AXLE_Y, LIFT) * BR
        b.label = "휠브라켓"
        parts = [b]
        for sp in SPS:
            s = Pos(0, M.AXLE_Y, LIFT) * sp
            s.label = sp.label
            parts.append(s)
        groups["휠브라켓"] = parts
    else:
        # modules() 는 KP_PTS 네 곳에 만든다. (+X,+Y) 코너를 x=0 으로 끌어온다.
        mods, statics = M.modules()
        dx = -M.AXLE_X
        m = Pos(dx, 0, LIFT) * mods[3].moved(Location())
        m.label = "모듈"
        groups["모듈"] = [m]
        for key, lst in statics.items():
            o = Pos(dx, 0, LIFT) * lst[3].moved(Location())
            o.label = key
            groups[key] = [o]
    return groups


SPEC = specimen()
_bbs = [p.bounding_box() for v in SPEC.values() for p in v]
SPEC_Y0 = min(b.min.Y for b in _bbs)      # 시험 대상 안쪽 끝
SPEC_Y1 = max(b.max.Y for b in _bbs)      # 바깥 끝
SPEC_Z1 = max(b.max.Z for b in _bbs)      # 꼭대기 (스워브는 조향모터)
SPEC_X = max(max(abs(b.min.X), abs(b.max.X)) for b in _bbs)

# 스워브 모듈 포락선 — 스키드로 지어도 프레임이 이걸 품어야 나중에 그대로 올린다.
# **레일 중심 기준 상대값**으로 둔다. 절대 z 로 뒀더니 LIFT 를 바꾸는 순간 어긋났다
# (그때 assert 가 잡아서 알았다).
SWERVE_ENV = dict(x=115.0, y1=301.0, dz=150.5)     # dz = 레일 중심 → 조향모터 꼭대기
if JIG == "swerve":
    assert (abs(SPEC_X - SWERVE_ENV["x"]) < 5 and abs(SPEC_Y1 - SWERVE_ENV["y1"]) < 5
            and abs((SPEC_Z1 - RAIL_Z) - SWERVE_ENV["dz"]) < 5), \
        f"SWERVE_ENV 가 실측과 어긋났다 " \
        f"(x{SPEC_X:.0f} y1 {SPEC_Y1:.0f} dz {SPEC_Z1-RAIL_Z:.0f})"
# 스워브까지 품는지 항상 본다 — 프레임 치수(REUSE_LEN)가 고정이라 모드와 무관하게
# 같은 프레임이 나온다. 즉 지금 지어 두면 나중에 손댈 게 없다.
NEED_X = max(SPEC_X, SWERVE_ENV["x"])

# ---- 파라미터 (단위: mm, 지면 z=0) ----
# 3030 은 **100mm 단위로 올림 과금**된다 (770 주문 = 800 값).
#
# 형상: 박스가 아니라 **ㄴ자 2장 + 가로 3본**이다. 왜 박스를 버렸나 —
#   지그는 바닥에 앵커로 잡고 쓴다. 전복 반력은 프레임이 아니라 **앵커가** 받는다.
#   그러면 박스의 위쪽 절반(상단 프레임 4본 + 뒤 기둥 2본)은 하는 일이 없다.
#   ㄴ자 코너 모멘트는 재보니 0.8 N·m 뿐이다 — 구동 반력 29 N·m 은 레일에서 기둥
#   2본의 ±69 N 축력으로 갈리고(모멘트가 아니다), 남는 건 바퀴가 기둥보다 50mm 앞에
#   있어서 생기는 1.65 N·m 을 코너 둘이 나눈 값이다. 스틸 브라켓 2개면 15~30 N·m 을
#   잡으니 20배 여유다. 처음에 넣었던 거싯판 200x200x6t 는 과설계라 뺐다.
WHEEL_Y = M.AXLE_Y                        # 260 — 타이어 중심면 (로봇과 동일)
RAIL_Y = M.RAIL_Y                         # 210 — 장착 레일 = 로봇 하부사이드 레일
SWING_R = M.SWEEP_R                       # 조향 스윕 (스키드로 지어도 이걸 품는다)
BASE_Z = S / 2                            # 베이스 중심 — 바닥에 그냥 놓는다
RAIL_X = (REUSE_LEN + S) / 2              # 기둥 x (±) — 가로재가 기둥 사이에 끼는 길이
BASE_Y1 = RAIL_Y + S / 2                  # 베이스 세로 앞끝 (기둥 앞면)
BASE_Y0 = BASE_Y1 - REUSE_LEN             # 뒤끝

assert RAIL_X - S / 2 > SWING_R + 5, "기둥이 바퀴 스윕 안으로 들어온다"
assert RAIL_X - S / 2 > NEED_X + 5, "기둥이 시험 대상(스워브 포함)과 겹친다"
# 공중 시험의 전제 — 바퀴/모듈 밑면이 베이스 위로 떠 있어야 베이스가 밑을 지날 수 있다
SPEC_Z0 = min(b.min.Z for b in _bbs)
assert SPEC_Z0 > BASE_TOP + 15, f"바퀴 지상고 {LIFT:.0f} 가 작아 베이스에 닿는다"

EXPORT_DIR = Path(__file__).parent.parent / "exports"
COLORS = {"지그프레임": "#c8ccd0", "앵커": "#37474f",
          "바퀴": "#2b2b2b", "휠브라켓": "#6b7480", "모듈": "#4a5058",
          "베어링": "#b6bbc0", "킹핀브라켓": "#6b7480", "조향모터": "#8d9298",
          "조향벨트": "#1b1b1b", "원점센서": "#2e7d32", "추": "#455a64"}

# ---- 프레임 ----
# 기둥 4본 = (x=±RAIL_X) x (y = RAIL_Y, BACK_Y). 앞 두 기둥 사이를 장착 레일이 건넌다.
# 뒤 레일은 로봇에 없는 지그 전용 보강 — 앞 레일과 짝을 이뤄야 하중이 트러스로 흐른다.
# ㄴ자 2장 (x=±RAIL_X 평면) + 가로 3본. 전부 REUSE_LEN 한 길이다.
BEAMS = (
    [(f"기둥{i+1}", REUSE_LEN, (sx * RAIL_X, RAIL_Y, BASE_TOP), "Z")
     for i, sx in enumerate((-1, 1))]
    + [(f"베이스세로{i+1}", REUSE_LEN, (sx * RAIL_X, BASE_Y0, BASE_Z), "Y")
       for i, sx in enumerate((-1, 1))]
    # 장착 레일 — 브라켓/킹핀브라켓이 여기 물린다. 기둥 **사이**에 끼운다.
    + [("장착레일", REUSE_LEN, (-RAIL_X + S / 2, RAIL_Y, RAIL_Z), "X")]
    + [(f"베이스가로{i+1}", REUSE_LEN, (-RAIL_X + S / 2, y, BASE_Z), "X")
       for i, y in enumerate((RAIL_Y, BASE_Y0 + S / 2))]
)



def frame():
    out = []
    for n, l, st, ax in BEAMS:
        b = bar(SERIES, l, st, ax)
        b.label = n
        out.append(b)
    return out


def anchors():
    """바닥 고정 앵커 목업 — 베이스 네 귀. 지그는 바닥/벤치에 볼트로 잡고 쓴다.

    바퀴가 떠 있으면 구동 반력 29 N·m 이 지그를 축 둘레로 뒤집으려 한다.
    자중(6.7 kg x 반폭 134mm = 8.8 N·m)으로는 3배 넘게 모자란다. 무게로 버티려면
    15 kg 을 더 얹어야 하니, 그냥 고정하는 게 맞다.
    """
    # 전복은 x 둘레가 아니라 **y(축) 둘레**로 일어난다 → 앵커의 y 위치는 자유롭다.
    # 베이스 세로대 윗면에서 가로대(뒤끝)와 거싯(앞쪽 200mm)을 피한 구간에 둔다.
    # 처음엔 기둥 자리에, 그 다음엔 거싯 자리에 놨다가 각각 2854 / 2112 mm3 물렸다.
    out = []
    ys = (BASE_Y0 + S + 35, RAIL_Y - S - 40)
    for i, (sx, y) in enumerate([(sx, y) for sx in (-1, 1) for y in ys]):
        with BuildPart() as p:
            with Locations((sx * RAIL_X, y, BASE_TOP + 4)):
                Box(44, 44, 8)
                Cylinder(4.5, 12, mode=Mode.SUBTRACT)          # M8 앵커
        p.part.label = f"앵커{i+1}"
        out.append(p.part)
    return out


# 조인트별 브라켓 수 — 모멘트를 받는 곳만 2개, 나머지는 1개.
# (부재이름, 조인트수, 브라켓/조인트, 비고)
JOINTS = [
    ("기둥 밑 ↔ 베이스세로", 2, 2, "ㄴ자 코너 — 모멘트 0.8 N·m"),
    ("장착레일 끝 ↔ 기둥", 2, 2, "바퀴 하중 + 반력이 지나간다"),
    ("베이스가로 끝 ↔ 베이스세로", 4, 1, "바닥에 눕는 조인트"),
]


def group(parts, name):
    c = Color(COLORS[name])
    for p in parts:
        p.color = c
    g = Compound(children=parts)
    g.label, g.color = name, c
    return g


GROUPS = [group(frame(), "지그프레임"), group(anchors(), "앵커")]
for k, v in SPEC.items():
    GROUPS.append(group(v, k))

assembly = Compound(children=GROUPS)
assembly.label = f"wheel_jig_{JIG}"

# ---- 제원 ----
bb = assembly.bounding_box()
_alu = sum(l for _, l, _, _ in BEAMS)
_kg = _alu * PROFILES[SERIES]["size"] ** 2 * 0.42 * ALU_DENSITY / 1000
print(f"단륜 시험 지그 ({JIG}, 스키드·스워브 공용)"
      f"  {bb.size.X:.0f} x {bb.size.Y:.0f} x {bb.size.Z:.0f} mm")
print(f"  ※ 프레임 치수가 REUSE_LEN={REUSE_LEN} 하나로 고정이라 두 모드가 **같은 프레임**이다."
      f" 레일도 안 옮긴다 — 바퀴만 다른 높이({LIFT:.0f}mm)에 뜬다")
print(f"  시험 대상 실측: y {SPEC_Y0:.0f}~{SPEC_Y1:.0f} / z {SPEC_Z0:.0f}~{SPEC_Z1:.0f}"
      f" / x ±{SPEC_X:.0f}  → 프레임은 여기서 역산한다")
print(f"  장착 레일 {SERIES}{SERIES} @ y={RAIL_Y} z={RAIL_Z:.1f}"
      f"  ← 로봇 하부사이드 레일과 동일 (브라켓을 그대로 옮겨 문다)")
print(f"  ㄴ자 2장 (x=±{RAIL_X:.0f}) / 기둥 z {BASE_TOP}~{POST_TOP:.0f}"
      f" / 베이스 y {BASE_Y0:.0f}~{BASE_Y1:.0f} / 앵커 M8 x4")
print(f"  바퀴 지상고 {LIFT:.0f} mm (공중 시험 — 레일을 기둥 꼭대기에 고정해 역산)"
      + (f" / 조향 스윕 y {WHEEL_Y-M.SWEEP_R:.0f}~{WHEEL_Y+M.SWEEP_R:.0f} — "
         f"베이스는 z<{BASE_TOP} 라 밑으로 지나간다" if JIG == "swerve" else ""))
print(f"  프로파일 {_alu/1000:.2f} m / 알루미늄 약 {_kg:.1f} kg")
# 전복 — 바퀴가 떠 있으면 구동 반력이 지그를 축(Y) 둘레로 뒤집는다
T_DRIVE, _a = 29.0, (RAIL_X + S / 2) / 1000
_have = _kg + 3.4                                  # 지그 + 바퀴·브라켓
print(f"  ** 구동 반력 {T_DRIVE:.0f} N·m vs 베이스 반폭 {_a*1000:.0f}mm →"
      f" 자중 {_have:.1f} kg 으로는 {_have*9.81*_a:.1f} N·m 까지만 버틴다")
print(f"     → 네 귀 M8 앵커로 바닥에 고정해서 쓴다 (안 잡으면 추 {max(0, T_DRIVE/(9.81*_a) - _have):.0f} kg 필요)")
cut_list([(n, l) for n, l, _, _ in BEAMS], SERIES)

# ---- 철물 ----
# 브라켓 1개당 볼트·너트 2세트. 여기에 휠브라켓 장착분(레일 밑면 2 + 바깥면 2)을 더한다.
_brk = sum(n * b for _, n, b, _ in JOINTS)
_spec_bolt = 4
print(f"철물 (주문 옵션에서 고르는 것):")
for tag, n, b, note in JOINTS:
    print(f"    브라켓 {n*b:2d}개   {tag:24s} {n}조인트 x {b}   ({note})")
print(f"    브라켓 합계 {_brk}개  → 볼트·너트 {_brk*2}세트")
print(f"    + 휠브라켓 장착 {_spec_bolt}세트 (자주 뺀다 → 스프링너트)")
print(f"    = M6 볼트 {_brk*2+_spec_bolt}개 / 너트 {_brk*2}(사각) + {_spec_bolt}(스프링)")
print(f"    + 3030 앤드캡 8 (노출 끝면) / M8 앵커볼트 4 (철물점, 옵션에 없음)")
_cost = _brk * 450 + (_brk * 2 + _spec_bolt) * 70 + _brk * 2 * 50 + _spec_bolt * 180 + 8 * 150
print(f"    철물 합계 약 {_cost:,}원 (스틸브라켓 450 / 볼트 70 / 사각너트 50 / "
      f"스프링너트 180 / 앤드캡 150 기준)")

EXPORT_DIR.mkdir(exist_ok=True)
export_step(assembly, str(EXPORT_DIR / f"wheel_jig_{JIG}.step"))
print(f"내보내기 완료: {EXPORT_DIR / f'wheel_jig_{JIG}.step'}")

if not os.environ.get("JIG_NOSHOW"):
    try:
        set_port(3939)
        show(*GROUPS, progress=None, names=[g.label for g in GROUPS],
             colors=[COLORS[g.label] for g in GROUPS], reset_camera=Camera.RESET)
    except Exception as e:
        print(f"뷰어에 표시하지 못함 (뷰어 미실행?): {e}")
