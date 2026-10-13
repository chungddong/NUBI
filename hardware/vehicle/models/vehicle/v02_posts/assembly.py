r"""v02_posts — 기둥 배치 (2026-10-04)

    .venv/Scripts/python.exe models/vehicle/v02_posts/assembly.py    # 검증 + 뷰어 + STEP
    .venv/Scripts/python.exe models/vehicle/record.py v02_posts      # 기록 이미지 + README

v01_base 에 **상부 프레임과 이을 기둥 4 본**을 세웠다. 나머지는 v01 그대로다.
  · 기둥 3030 x 143 — 축크로스 자리 (x=±210) 의 사이드 레일 윗면 (y=±210)
    축크로스가 레일 안쪽면에 붙는 바로 그 x 라서 기둥 · 레일 · 크로스 · 클램프 다리가
    한 단면에 모인다 → 위에서 오는 하중이 레일을 휘지 않고 바로 클램프로 내려간다
  · 높이 143 = 옛 상부 프레임 (v00_ref) 그대로 — 상부 레일 밑면이 z=266.8 에 온다
  · 상부 프레임 자체는 아직 없다 (다음 버전)

parameters / low_clamp / axle / extrusion_reference / motor_reference / tslot 은
v01_base 의 동결 사본을 그대로 복사했다 (내용 같음).

버전 계약 (record.py 가 읽는다)
  VERSION, TITLE, DATE, PREV, SUMMARY
  build()           → [dict(key, label, color, parts)]   key 는 버전 사이에 안 바뀌는 이름
  verify(groups)    → 실패 항목 목록 (빈 목록 = 통과). 결과를 print 한다
  metrics(groups)   → [(항목, 값)]  기록 표에 들어간다
  WHEEL_KEYS        → 지상고 계산에서 뺄 그룹 key
"""

import sys
from pathlib import Path

HERE = Path(__file__).parent
sys.path.insert(0, str(HERE))

from build123d import Color, Compound, Location, Pos, Rot   # noqa: E402

import low_clamp as LC                                      # noqa: E402
import parameters as P                                      # noqa: E402
import tslot                                                # noqa: E402
from motor_reference import actual_motor_step_reference     # noqa: E402

VERSION = "v02_posts"
TITLE = "기둥 배치"
DATE = "2026-10-04"
PREV = "v01_base"
SUMMARY = "축크로스 자리(x=±210) 레일 위에 상부 프레임용 기둥 4 본 (3030 x 143)"

PORT = 3939
OUT = HERE / "output"

# ---------------------------------------------------------------- 프레임 치수
BASE_L, BASE_W = 760.0, 450.0       # 하부 프레임 외곽 (x, y)
S = 30.0                            # 3030
AXLE_X = 210.0                      # 앞뒤 축 x — 휠베이스 420
AXLE_Z = P.WHEEL_DIAMETER / 2       # 지면 z=0
RAIL_Y = (BASE_W - S) / 2           # 사이드 레일 중심 y = 210
RAIL_Z = AXLE_Z + LC.RAIL_CZ        # 레일 중심 z
CROSS_X = (BASE_L - S) / 2          # 끝 크로스 x = 365
CROSS_L = BASE_W - 2 * S            # 레일 사이 크로스 길이 390
# 축크로스 — **T 클램프 혀 바로 위**. 크로스 밑면이 혀 윗면(= 레일 밑면 높이)에 얹히고
# 혀 볼트 2 개가 크로스 밑면 슬롯에 물린다. 혀 폭 30 = 크로스 폭.
AXLE_CROSS_X = AXLE_X
WHEEL_Y = RAIL_Y - LC.CLAMP_CY      # 클램프 로컬 레일(y=CLAMP_CY)이 차량 레일에 오는 휠 중심
# 기둥 — 축크로스와 같은 x, 사이드 레일 윗면에 선다. 높이는 옛 상부 프레임 (v00_ref) 그대로.
FRAME_TOP = RAIL_Z + S / 2          # 하부 레일 윗면
POST_X = AXLE_CROSS_X
POST_H = 143.0                      # 하부 레일 윗면 → 상부 레일 밑면

WHEEL_KEYS = ("motor",)


def _bar(label, length, start, axis):
    b = tslot.bar(30, length, start, axis)
    b.label = label
    return b


def lower_frame():
    beams = [_bar(f"하부사이드{i+1}", BASE_L, (-BASE_L / 2, sy * RAIL_Y, RAIL_Z), "X")
             for i, sy in enumerate((-1, 1))]
    beams += [_bar(f"하부크로스{i+1}", CROSS_L, (x, -CROSS_L / 2, RAIL_Z), "Y")
              for i, x in enumerate((-CROSS_X, 0, CROSS_X))]
    beams += [_bar(f"축크로스{i+1}", CROSS_L, (x, -CROSS_L / 2, RAIL_Z), "Y")
              for i, x in enumerate((-AXLE_CROSS_X, AXLE_CROSS_X))]
    return beams


def posts():
    """상부 프레임과 이을 기둥 — 축크로스 x, 사이드 레일 윗면에서 위로."""
    return [_bar(f"기둥{i+1}", POST_H, (x, sy * RAIL_Y, FRAME_TOP), "Z")
            for i, (x, sy) in enumerate((x, sy) for x in (-POST_X, POST_X) for sy in (-1, 1))]


def wheels():
    """휠모터·T 클램프·상부 클램프·볼트 (레일 밑면 2 + 혀 → 축크로스 2, 바퀴당)."""
    motor0 = actual_motor_step_reference(axle_z=0)
    upper0 = LC.oem_clamp_block()
    # 혀 케이블 통로는 전부 차량 안쪽(x=0 쪽)으로 — 좌우 두 벌
    lowers_by_side = {s: LC.low_clamp(cable_side=s) for s in (+1, -1)}
    bolt0 = (LC.bottom_button_head_bolts_reference()
             + LC.shelf_button_head_bolts_reference())

    motors, lowers, uppers, bolts = [], [], [], []
    for i, (x, sy) in enumerate((x, sy) for x in (-AXLE_X, AXLE_X) for sy in (-1, 1)):
        # 기본 형상은 +Y 차륜 기준(축이 -Y = 차량 안쪽). -Y 측은 z 축 180° 회전.
        tf = Pos(x, sy * WHEEL_Y, AXLE_Z) * (Rot(0, 0, 180) if sy < 0 else Location())
        w = tf * motor0.moved(Location())
        lo = tf * lowers_by_side[LC.cable_side_for(x, sy)].moved(Location())
        up = tf * upper0.moved(Location())
        w.label, lo.label, up.label = f"휠모터{i+1}", f"T클램프{i+1}", f"상부클램프{i+1}"
        motors.append(w); lowers.append(lo); uppers.append(up)
        for j, b0 in enumerate(bolt0):
            b = tf * b0.moved(Location())
            b.label = f"볼트_{i+1}_{j+1}"
            bolts.append(b)
    return motors, lowers, uppers, bolts


def build():
    frame = lower_frame() + posts()
    motors, lowers, uppers, bolts = wheels()
    return [
        dict(key="frame", label=f"프레임_3030_{len(frame)}본", color="#69727d", parts=frame),
        dict(key="clamp_low", label="T클램프_혀선반_4개", color="#c9ced4", parts=lowers),
        dict(key="clamp_oem", label="기존_상부클램프_4개", color="#f5a623", parts=uppers),
        dict(key="motor", label="휠모터_4개", color="#34383d", parts=motors),
        dict(key="bolt", label=f"체결볼트_M{P.TNUT_BOLT:.0f}x{P.LOW_BOLT_LEN:.0f}_{len(bolts)}개",
             color="#30343a", parts=bolts),
    ]


# ---------------------------------------------------------------- 검증
def _overlap(a_parts, b_parts):
    """두 무리 사이 (겹침 부피 합 mm³, 겹친 조각의 가장 얇은 치수 최댓값)."""
    total, thick = 0.0, 0.0
    for a in a_parts:
        abb = a.bounding_box()
        for b in b_parts:
            bbb = b.bounding_box()
            if (abb.min.X > bbb.max.X or bbb.min.X > abb.max.X or
                    abb.min.Y > bbb.max.Y or bbb.min.Y > abb.max.Y or
                    abb.min.Z > bbb.max.Z or bbb.min.Z > abb.max.Z):
                continue
            inter = a & b
            if inter and inter.volume > 0:
                total += inter.volume
                s = inter.bounding_box().size
                thick = max(thick, min(s.X, s.Y, s.Z))
    return total, thick


def _box_gap(box_part, shape, tol=0.5):
    """축 정렬 상자 부품 ~ shape 메시 꼭짓점 사이 최소 거리 (mm)."""
    import numpy as np
    bb = box_part.bounding_box()
    vs, _ = shape.tessellate(tol, 0.3)
    p = np.array([(v.X, v.Y, v.Z) for v in vs])
    lo, hi = np.array(tuple(bb.min)), np.array(tuple(bb.max))
    d = np.maximum(np.maximum(lo - p, 0), p - hi)
    return float(np.linalg.norm(d, axis=1).min())


def verify(groups):
    g = {d["key"]: d["parts"] for d in groups}
    frame, lowers, uppers, motors, bolts = (g["frame"], g["clamp_low"], g["clamp_oem"],
                                            g["motor"], g["bolt"])
    pairs = {
        "프레임-T클램프": (frame, lowers),
        "프레임-상부클램프": (frame, uppers),
        "프레임-휠모터": (frame, motors),
        "T클램프-휠모터": (lowers, motors),
        "T클램프-상부클램프": (lowers, uppers),
        "볼트-프레임": (bolts, frame),       # 슬롯 입구로 들어가야 한다 — 립을 뚫으면 안 된다
        "볼트-T클램프": (bolts, lowers),
        "볼트-휠모터": (bolts, motors),
    }
    # 볼트 머리는 카운터보어 바닥과 0.01mm 겹친다 — low_clamp 가 면 겹침을 피하려고
    # 카운터보어를 0.01 덜 파 둔 값이다. 부피(개당 0.52)가 아니라 **두께**로 거른다.
    TOL_T = 0.02
    bad = []
    for name, (a, b) in pairs.items():
        v, t = _overlap(a, b)
        print(f"  간섭 {name}: {v:.2f} mm3" + (f" (두께 {t:.3f})" if v > 0 else ""))
        if v > 1.0 and t > TOL_T:
            bad.append(name)
    # 대조군 — 1mm 올리면 겹쳐야 한다. 안 겹치면 클램프가 떠 있는 것이다.
    raised = [Pos(0, 0, 1.0) * lo for lo in lowers]
    for name, prefix in (("레일", "하부사이드"), ("축크로스", "축크로스")):
        v, _ = _overlap([b for b in frame if b.label.startswith(prefix)], raised)
        print(f"  대조군 클램프 +1mm 올림 → {name} 겹침 {v:.0f} mm3")
        if v < 100.0:
            bad.append(f"대조군: 클램프가 {name}에 안 닿는다")
    # 기둥 — 1mm 내리면 레일과 겹쳐야 한다 (= 레일 윗면에 서 있다). 공중에 떠 있으면 0 이다.
    post = [b for b in frame if b.label.startswith("기둥")]
    rails = [b for b in frame if b.label.startswith("하부사이드")]
    v, _ = _overlap(rails, [Pos(0, 0, -1.0) * p for p in post])
    print(f"  대조군 기둥 -1mm 내림 → 레일 겹침 {v:.0f} mm3")
    if v < 100.0 * len(post):
        bad.append("대조군: 기둥이 레일 윗면에 안 선다")
    # distance_to(모터 STEP) 는 B-스플라인 면 정밀 거리라 15 분을 넘겨도 안 끝났다 —
    # 기둥은 축 정렬 상자이므로 모터 메시 점 → 상자 거리로 잰다 (메시 오차 ±0.5).
    gap = min(_box_gap(p, m) for p in post for m in motors)
    print(f"  기둥 ~ 휠모터 최소 간격 {gap:.1f} mm (메시 점 기준)")
    if gap < 5.0:
        bad.append(f"기둥이 타이어에 {gap:.1f} mm 까지 붙는다")
    return bad


def metrics(groups):
    g = {d["key"]: d["parts"] for d in groups}
    lengths = [(b.label, max(tuple(b.bounding_box().size))) for b in g["frame"]]  # 가장 긴 변
    total = sum(l for _, l in lengths)
    billed = sum(tslot.snap_cut(l) for _, l in lengths)
    cuts = {}
    for _, l in lengths:
        cuts[round(l)] = cuts.get(round(l), 0) + 1
    return [
        ("프레임 외곽 (x × y)", f"{BASE_L:.0f} × {BASE_W:.0f} mm"),
        ("휠베이스 (앞뒤 축 간격)", f"{2 * AXLE_X:.0f} mm"),
        ("트레드 (좌우 바퀴 중심 간격)", f"{2 * WHEEL_Y:.1f} mm"),
        ("레일 윗면 높이", f"{RAIL_Z + S / 2:.1f} mm"),
        ("축크로스 x", f"±{AXLE_CROSS_X:.0f} mm"),
        ("기둥", f"{sum(b.label.startswith('기둥') for b in g['frame'])} 본, x=±{POST_X:.0f} · "
               f"y=±{RAIL_Y:.0f}, 높이 {POST_H:.0f} (상부 레일 밑면 z={FRAME_TOP + POST_H:.1f})"),
        ("3030 절단", " + ".join(f"{l} × {n}" for l, n in sorted(cuts.items(), reverse=True))),
        ("3030 합계", f"실길이 {total / 1000:.2f} m / 과금 {billed / 1000:.2f} m "
                    f"({billed / tslot.CUT_UNIT * tslot.UNIT_PRICE:,.0f}원)"),
        ("체결볼트 M6x15", f"{len(g['bolt'])} 개"),
    ]


# ---------------------------------------------------------------- 출력
def compounds(groups):
    out = []
    for d in groups:
        c = Color(d["color"])
        for p in d["parts"]:
            p.color = c
        comp = Compound(children=d["parts"])
        comp.label, comp.color = d["label"], c
        out.append(comp)
    return out


def main():
    from build123d import export_step
    from ocp_vscode import Camera, set_port, show

    groups = build()
    print(f"{VERSION} — {TITLE}")
    bad = verify(groups)
    print("verify:", bad or "OK")
    comps = compounds(groups)
    OUT.mkdir(exist_ok=True)
    step = OUT / f"{VERSION}.step"
    asm = Compound(children=comps)
    asm.label = VERSION
    export_step(asm, str(step))
    print(f"  STEP 저장: {step}")
    set_port(PORT)
    show(*comps, names=[c.label for c in comps], colors=[d["color"] for d in groups],
         alphas=[1] * len(comps), progress=None, reset_camera=Camera.RESET)
    print(f"{VERSION}_SENT")
    if bad:
        sys.exit(1)


if __name__ == "__main__":
    main()
