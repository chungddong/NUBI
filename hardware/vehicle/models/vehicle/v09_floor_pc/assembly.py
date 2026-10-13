r"""v09_floor_pc — 실외 바닥판: PC 5t · 부품 고정 구멍 (2026-10-04)

    .venv/Scripts/python.exe models/vehicle/record.py v09_floor_pc   # 짓기·검증·STEP·DXF·뷰어·기록 한 번에

실외 주행으로 바뀌어 바닥판을 다시 냈다 (floor.py · parts.py).
  · 재료 PC 5t (아크릴은 실외 진동 · 추위에 볼트 구멍 · 노치부터 금이 간다)
  · 케이블 그로밋 Ø25 → **방수 케이블 글랜드 M20 (Ø20.5)**
  · M5 격자 66 → 없앰 (구멍이 곧 물길). 대신 **부품 고정 구멍** 을 부품 자리에서 바로 뚫는다
    (드라이버 귀 M4 · 젯슨 M3 스탠드오프 · ESP32 받침보드 M3 · MIDI M5 · SSR M4 · DC-DC · IMU M2.5)
  · 배터리 고정 밴드 슬롯 4
  · 젯슨은 플라스틱 받침 대신 M3 스탠드오프, ESP32 는 받침보드에 꽂는다. IMU 는 가운데 크로스
    위를 피해 옮겼다 (빔 위 구멍은 너트가 못 들어간다)

v03 에서 바꾼 검증 · 짓기 속도 (형상과 무관)
  · _overlap: 부품마다 바운딩박스를 **한 번만** 잰다 (빠른 박스). v02 까지는 쌍마다 모터 STEP
    의 정밀 박스 (1.5 s) 를 다시 재서 검증만 3 분이었다. 불리언 자체는 0.2 s.
  · _box_gap: 모터 메시를 한 번만 뜨고 Location 만 곱한다 (같은 STEP 4 개).
  · tslot.beam: 같은 길이 토막은 한 번만 짓는다.

parameters / low_clamp / axle / extrusion_reference / motor_reference 는 v01 동결 사본 그대로,
tslot 은 beam 캐시만 더했다.

버전 계약 (record.py 가 읽는다)
  VERSION, TITLE, DATE, PREV, SUMMARY, PORT, WHEEL_KEYS
  build()           → [dict(key, label, color, parts)]   key 는 버전 사이에 안 바뀌는 이름
  verify(groups)    → 실패 항목 목록 (빈 목록 = 통과). 결과를 print 한다
  metrics(groups)   → [(항목, 값)]  기록 표에 들어간다
  compounds(groups) → 뷰어 · STEP 용 Compound 목록
"""

import sys
from pathlib import Path

HERE = Path(__file__).parent
sys.path.insert(0, str(HERE))

import numpy as np                                          # noqa: E402
from build123d import Align, Box, Color, Compound, Cylinder, Location, Pos, Rot   # noqa: E402

import low_clamp as LC                                      # noqa: E402
import parameters as P                                      # noqa: E402
import tslot                                                # noqa: E402
from motor_reference import actual_motor_step_reference     # noqa: E402
import floor as FL                                          # noqa: E402
import parts as PT                                          # noqa: E402

VERSION = "v09_floor_pc"
TITLE = "실외 바닥판 (PC · 부품 고정 구멍)"
DATE = "2026-10-04"
PREV = "v08_realparts"
SUMMARY = "실외 바닥판 — PC 5t, 글랜드 M20, 격자 없앰, 부품 고정 구멍 · 배터리 밴드 슬롯"

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
POST_H = 143.0                      # 하부 레일 윗면 → 상부 레일 밑면
# 기둥 x — 옛 설계 (v00_ref) 자리 = 끝 · 가운데 크로스. 이름은 v03 과 같은 기둥5~10.
POST_X = (-CROSS_X, 0.0, CROSS_X)
POST_FIRST = 5

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
    """상부 프레임과 이을 기둥 — 사이드 레일 윗면에서 위로, 크로스가 레일에 붙는 x 마다."""
    xs = [(x, sy) for x in POST_X for sy in (-1, 1)]
    return [_bar(f"기둥{i + POST_FIRST}", POST_H, (x, sy * RAIL_Y, FRAME_TOP), "Z")
            for i, (x, sy) in enumerate(xs)]


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
    ] + floor_groups()


def floor_groups():
    fl = FL.Floor(sys.modules[__name__])
    pb = fl.bolts()
    L, _, _ = fl.bolt_len()
    return [
        dict(key="floor", label=f"바닥판_{FL.MAT_NAME.split()[0]}{FL.PLATE_T:g}t", color="#7fc8f8",
             parts=[fl.plate()], alpha=0.5),
        dict(key="floor_bolt", label=f"판볼트_M6x{L}_{len(pb)}개", color="#30343a", parts=pb),
        dict(key="cable", label="케이블_경로_참고", color="#c62828", parts=fl.cable_paths()),
    ] + part_groups(fl)


def part_groups(fl):
    """parts.PARTS → 그룹별 부품 (실측 치수 모델). 밑면 = 판 윗면."""
    by = PT.build(fl.z0 + FL.PLATE_T)
    return [dict(key=f"part_{k}", label=PT.GROUPS[k][0], color=PT.GROUPS[k][1], parts=v)
            for k, v in by.items()]


# ---------------------------------------------------------------- 검증
_BB = {}


def _bb(part):
    """부품 바운딩박스 (min, max) — **한 번만** 잰다. 빠른 박스라 실제보다 조금 크다 (걸러내기용)."""
    k = id(part)
    if k not in _BB:
        b = part.bounding_box(optimal=False)
        _BB[k] = (part, np.array(tuple(b.min)), np.array(tuple(b.max)))
    return _BB[k][1], _BB[k][2]


def _overlap(a_parts, b_parts):
    """두 무리 사이 (겹침 부피 합 mm³, 겹친 조각의 가장 얇은 치수 최댓값)."""
    total, thick = 0.0, 0.0
    for a in a_parts:
        alo, ahi = _bb(a)
        for b in b_parts:
            blo, bhi = _bb(b)
            if (alo > bhi).any() or (blo > ahi).any():
                continue
            inter = a & b
            if inter and inter.volume > 0:
                total += inter.volume
                s = inter.bounding_box().size
                thick = max(thick, min(s.X, s.Y, s.Z))
    return total, thick


_MESH = []


def _trsf(loc):
    t = loc.wrapped.Transformation()
    m = np.array([[t.Value(i, j) for j in range(1, 5)] for i in range(1, 4)])
    return m[:, :3], m[:, 3]


def _mesh_pts(shape, tol=0.5):
    """메시 꼭짓점 (월드). 같은 형상을 옮긴 사본은 원본 메시에 상대 행렬만 곱한다.

    located() 는 깊은 복사라 (모터 STEP 이면 수 초) 쓰지 않는다 — 원본을
    제자리에서 쪼개고, 사본은 (사본 Location × 원본 Location⁻¹) 로 옮긴다."""
    for base, pts, (R0, T0) in _MESH:
        if shape.wrapped.IsPartner(base.wrapped):
            R1, T1 = _trsf(shape.location)
            R = R1 @ R0.T
            return pts @ R.T + (T1 - R @ T0)
    vs, _ = shape.tessellate(tol, 0.3)
    pts = np.array([(v.X, v.Y, v.Z) for v in vs])
    _MESH.append((shape, pts, _trsf(shape.location)))
    return pts


def _box_gap(box_part, shape):
    """축 정렬 상자 부품 ~ shape 메시 꼭짓점 사이 최소 거리 (mm, 메시 오차 ±0.5)."""
    bb = box_part.bounding_box()
    p = _mesh_pts(shape)
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
    # 기둥 — 1mm 내리면 레일과 겹쳐야 한다 (= 레일 윗면에 서 있다).
    post = [b for b in frame if b.label.startswith("기둥")]
    rails = [b for b in frame if b.label.startswith("하부사이드")]
    v, _ = _overlap(rails, [Pos(0, 0, -1.0) * p for p in post])
    print(f"  대조군 기둥 -1mm 내림 → 레일 겹침 {v:.0f} mm3")
    if v < 100.0 * len(post):
        bad.append("대조군: 기둥이 레일 윗면에 안 선다")
    gap = min(_box_gap(p, m) for p in post for m in motors)
    print(f"  기둥 ~ 휠모터 최소 간격 {gap:.1f} mm (메시 점 기준)")
    if gap < 5.0:
        bad.append(f"기둥이 타이어에 {gap:.1f} mm 까지 붙는다")
    return bad + verify_floor(g) + verify_parts(g) + verify_holes()


def verify_holes():
    """부품 고정 구멍 — (1) 판 밑에 빔이 없어야 너트가 들어간다 (2) 다른 구멍 · 노치 · 가장자리와 안 겹친다."""
    import math
    fl = FL.Floor(sys.modules[__name__])
    xs, ys = fl.beam_bands()
    NUT_R = {2.7: 4.0, 3.4: 5.0, 3.5: 5.0, 4.5: 6.5, 5.5: 7.5}       # 너트 · 와셔 반경 (구멍 Ø → mm)
    bad, over = [], 0
    for x, y, d, bolt, name in fl.part_holes():
        r = NUT_R.get(d, d)
        if any(a - r < x < b + r for a, b in xs) or any(a - r < y < b + r for a, b in ys):
            bad.append(f"빔 위 구멍: {name} ({x:+.1f},{y:+.1f})")
            over += 1
    others = ([(x, y, FL.MOUNT_D) for x, y in fl.mount_holes()]
              + [(x, y, FL.GROMMET_D) for x, y in fl.grommets])

    def slot_gap(x, y, d):
        """구멍 가장자리 ~ 밴드 슬롯 (x 방향 장공) 가장자리 최소 거리. 원으로 뭉개면 과하게 걸린다."""
        out = 1e9
        for sx, sy, l, w in fl.slots():
            h = (l - w) / 2
            px = min(max(x, sx - h), sx + h)
            out = min(out, math.dist((x, y), (px, sy)) - w / 2 - d / 2)
        return out
    ph = [(x, y, d) for x, y, d, _, _ in fl.part_holes()]
    close = 0
    for i, (x, y, d) in enumerate(ph):
        for (u, v, e) in ph[i + 1:] + others:
            if math.dist((x, y), (u, v)) < (d + e) / 2 + 2.0:
                close += 1
                bad.append(f"구멍끼리 2 mm 안: ({x:+.1f},{y:+.1f}) ~ ({u:+.1f},{v:+.1f})")
        if slot_gap(x, y, d) < 2.0:
            close += 1
            bad.append(f"밴드 슬롯에 2 mm 안: ({x:+.1f},{y:+.1f})")
        if abs(x) + d / 2 > fl.L / 2 - 3 or abs(y) + d / 2 > fl.W / 2 - 3:
            bad.append(f"가장자리 3 mm 안: ({x:+.1f},{y:+.1f})")
        for cx, cy, wx, wy in fl.notches():
            if abs(x - cx) < wx / 2 + d / 2 + 2 and abs(y - cy) < wy / 2 + d / 2 + 2:
                bad.append(f"노치에 붙음: ({x:+.1f},{y:+.1f})")
    print(f"  부품 고정 구멍 {len(ph)} — 빔 위 {over} · 서로/다른 구멍과 2 mm 안 {close}")
    # 대조군 — 일부러 가운데 크로스 위 (x=0) 에 둔 구멍은 걸려야 한다
    a, b = xs[2]
    print(f"  대조군 x=0 구멍 → 빔 위 판정 {a - 5 < 0 < b + 5}")
    return bad


def verify_floor(g):
    """바닥판 — 간섭 · 얹힘 · 볼트 · 케이블 길."""
    plate, pbolts, cables = g["floor"][0], g["floor_bolt"], g["cable"]
    frame, lowers, uppers, motors = g["frame"], g["clamp_low"], g["clamp_oem"], g["motor"]
    bad = []
    for name, (a, b) in {"바닥판-프레임 (기둥 포함)": ([plate], frame),
                         "바닥판-T클램프": ([plate], lowers), "바닥판-휠모터": ([plate], motors),
                         "판볼트-프레임": (pbolts, frame), "판볼트-바닥판": (pbolts, [plate]),
                         "케이블-프레임": (cables, frame), "케이블-T클램프": (cables, lowers),
                         "케이블-상부클램프": (cables, uppers), "케이블-휠모터": (cables, motors),
                         "케이블-바닥판": (cables, [plate])}.items():
        v, t = _overlap(a, b)
        print(f"  간섭 {name}: {v:.2f} mm3" + (f" (두께 {t:.3f})" if v > 0 else ""))
        if v > 1.0:
            bad.append(name)
    # 대조군 — 0.5 mm 내리면 레일 · 크로스와 겹친다 (= 얹혀 있다). 판볼트를 1 mm 내리면 판과 겹친다.
    v, _ = _overlap([Pos(0, 0, -0.5) * plate], frame)
    print(f"  대조군 바닥판 -0.5mm → 프레임 겹침 {v:.0f} mm3")
    if v < 100.0:
        bad.append("대조군: 바닥판이 프레임에 안 얹힌다")
    # 10 본 겸용 — 축 자리 (±210) 에 가상 기둥 4 본을 세워도 판과 안 겹쳐야 한다.
    extra = [_bar(f"가상기둥{i+1}", POST_H, (x, sy * RAIL_Y, FRAME_TOP), "Z")
             for i, (x, sy) in enumerate((x, sy) for x in (-AXLE_CROSS_X, AXLE_CROSS_X)
                                         for sy in (-1, 1))]
    v, _ = _overlap([plate], extra)
    print(f"  10 본 겸용: 축 자리 가상 기둥 4 본 ~ 바닥판 간섭 {v:.2f} mm3")
    if v > 1.0:
        bad.append("10 본 겸용 실패: 축 자리 기둥이 판과 겹친다")
    # 대조군 — 노치가 없다면 같은 가상 기둥이 판과 겹쳐야 한다 (검사가 실제로 보는지)
    with_notch_gone = [Pos(0, 0, 0) * b for b in extra]
    solid = Pos(0, 0, FRAME_TOP) * Box(BASE_L, BASE_W, FL.PLATE_T, align=(Align.CENTER, Align.CENTER, Align.MIN))
    v, _ = _overlap([solid], with_notch_gone)
    print(f"  대조군 노치 없는 판 ~ 가상 기둥 겹침 {v:.0f} mm3")
    if v < 100.0:
        bad.append("대조군: 가상 기둥 검사가 판을 못 본다")
    fl = FL.Floor(sys.modules[__name__])
    L, need, maxlen = fl.bolt_len()
    print(f"  판볼트 길이: 필요 {need:.1f} ~ 최대 {maxlen:.1f} → M6x{L}")
    if L is None:
        bad.append(f"판볼트 길이 맞는 것 없음 ({need:.1f}~{maxlen:.1f})")
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
        ("기둥", f"{sum(b.label.startswith('기둥') for b in g['frame'])} 본, "
               f"x={' · '.join(f'{x:+.0f}' for x in POST_X)} × y=±{RAIL_Y:.0f}, "
               f"높이 {POST_H:.0f} (윗면 z={FRAME_TOP + POST_H:.1f})"),
        ("3030 절단", " + ".join(f"{l} × {n}" for l, n in sorted(cuts.items(), reverse=True))),
        ("3030 합계", f"실길이 {total / 1000:.2f} m / 과금 {billed / 1000:.2f} m "
                    f"({billed / tslot.CUT_UNIT * tslot.UNIT_PRICE:,.0f}원)"),
        ("체결볼트 M6x15", f"{len(g['bolt'])} 개"),
    ] + metrics_floor() + metrics_parts(g)


def verify_parts(g):
    """부품 상자 — 서로 · 기둥 · 판볼트 머리와 안 겹치고, 판 위에 있고, 케이블 구멍을 안 막는다."""
    fl = FL.Floor(sys.modules[__name__])
    parts = [p for k, v in g.items() if k.startswith("part_") for p in v]
    bad = []
    v_self = 0.0
    for i, a in enumerate(parts):
        v, _ = _overlap([a], parts[i + 1:])
        v_self += v
        if v > 1.0:
            bad.append(f"부품끼리 겹침: {a.label}")
    print(f"  간섭 부품끼리: {v_self:.2f} mm3")
    for name, other in (("프레임 (기둥)", g["frame"]), ("판볼트 머리", g["floor_bolt"]),
                        ("바닥판", g["floor"])):
        v, _ = _overlap(parts, other)
        print(f"  간섭 부품-{name}: {v:.2f} mm3")
        if v > 1.0:
            bad.append(f"부품-{name}")
    # 판 밖으로 나가지 않는다 (노치 · 가장자리)
    for p in parts:
        lo, hi = _bb(p)
        if (abs(lo[:2]) > [fl.L / 2, fl.W / 2]).any() or (abs(hi[:2]) > [fl.L / 2, fl.W / 2]).any():
            bad.append(f"판 밖: {p.label}")
    # 케이블 구멍 위는 비운다
    keep = [Pos(gx, gy, fl.z0 + FL.PLATE_T) * Cylinder(PT.CABLE_KEEP_R, PT.CABLE_KEEP_H,
                                                      align=(Align.CENTER, Align.CENTER, Align.MIN))
            for gx, gy in fl.grommets]
    v, _ = _overlap(parts, keep)
    print(f"  간섭 부품-케이블 구멍 위 (반경 {PT.CABLE_KEEP_R:g}): {v:.2f} mm3")
    if v > 1.0:
        bad.append("부품이 케이블 구멍을 막는다")
    # 대조군 — 부품을 1 mm 내리면 판과 겹친다 (= 판 위에 놓여 있다)
    v, _ = _overlap([Pos(0, 0, -1.0) * p for p in parts], g["floor"])
    print(f"  대조군 부품 -1mm → 바닥판 겹침 {v:.0f} mm3")
    if v < 100.0 * len(parts):
        bad.append("대조군: 부품이 판 위에 안 놓였다")
    return bad


def metrics_parts(g):
    """무게중심은 무게가 알려진 부품만 (바운딩박스 중심 기준). 무게 미상은 따로 적는다."""
    mass = {p[1]: p[5] for p in PT.PARTS}
    parts = [p for k, v in g.items() if k.startswith("part_") for p in v]
    known = [(p, mass[p.label]) for p in parts if mass.get(p.label)]
    m = sum(w for _, w in known)
    c = sum(np.array(tuple(p.bounding_box().center())) * w for p, w in known) / m
    top = max(p.bounding_box().max.Z for p in parts)
    unknown = [p.label for p in parts if not mass.get(p.label)]
    return [
        ("부품", f"{len(parts)} 개 (실측 치수 모델, 출처 parts.py)"),
        ("부품 무게 (알려진 것)", f"{m:.2f} kg" + (f" — 무게 미상: {', '.join(unknown)}" if unknown else "")),
        ("부품 무게중심", f"x={c[0]:+.1f} · y={c[1]:+.1f} · z={c[2]:.1f} mm (휠베이스 가운데 x=0 기준)"),
        ("가장 높은 부품 윗면", f"z={top:.1f} — 기둥 윗면 z={FRAME_TOP + POST_H:.1f} 까지 "
                         f"{FRAME_TOP + POST_H - top:.1f} mm 남음"),
    ]


def metrics_floor():
    fl = FL.Floor(sys.modules[__name__])
    plate = fl.plate()
    L, need, maxlen = fl.bolt_len()
    return [
        ("바닥판 재료", f"{FL.MAT_NAME} {FL.PLATE_T:g}t — {FL.MAT_NOTE}"),
        ("바닥판 크기 · 무게", f"{fl.L:.0f} × {fl.W:.0f} × {FL.PLATE_T:g}, 약 {fl.mass_kg(plate):.2f} kg"),
        ("바닥판 구멍", f"M6 고정 {len(fl.mount_holes())} · 그로밋 Ø{FL.GROMMET_D:g} {len(fl.grommets)} · "
                      f"M5 격자 {len(fl.grid_holes())} · 기둥 노치 {len(fl.notches())} "
                      f"(x={' · '.join(f'{x:+.0f}' for x in fl.notch_xs)}, 기둥 6 · 10 본 겸용)"),
        ("판볼트", f"M6x{L} + 와셔 (필요 {need:.1f} ~ 최대 {maxlen:.1f})"),
        ("받침 없는 구간 (크로스 안쪽면 사이)", " · ".join(f"{s:g}" for s in fl.spans()) + " mm"),
    ]


def export(groups, out):
    """record.py 가 짓기 직후 부른다 — 재단용 DXF · 도면 · 재단 규격."""
    fl = FL.Floor(sys.modules[__name__])
    plate = {d["key"]: d["parts"] for d in groups}["floor"][0]
    FL.export_dxf(fl, out / "floor_plate.dxf")
    FL.drawing(fl, out / "floor_plate.png")
    FL.cut_spec(fl, plate, HERE / "floor_plate_cut.md", VERSION)


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
    """record.py 로 넘긴다 — 짓기 · 검증 · STEP · 뷰어 · 기록을 한 프로세스에서 한 번에."""
    sys.path.insert(0, str(HERE.parent))
    import record
    sys.argv = [record.__file__, VERSION, "--rebuild"]
    record.main()                          # 상주 워커가 있으면 그쪽으로 (import 20 s 생략)


if __name__ == "__main__":
    main()
