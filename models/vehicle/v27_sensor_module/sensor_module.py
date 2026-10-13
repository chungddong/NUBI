r"""센서 모듈 — 모듈 레일 (2020 ×3, 윗면 z 316.85) 위에 T 너트로 올리는 첫 모듈 (v27).

부품 (국내 구매, research/2026-10-05_sensor_module.md · DECISIONS D-17)
  · 2D 라이다 LDROBOT **STL-19P** (D500 키트) — 54 × 46.29 × 35, 회전부 Ø35.29 × 12.8, 스캔 평면 = 바닥 + 27.2,
    고정 Ø2.5 ×3 (양옆 귀 간격 46.8, 세 번째 귀는 그 중심선에서 31.92), 60 klux, 12 m, 45 g [데이터시트]
  · GNSS **아센코리아 GPS630** (USB, u-blox M8) — 55 × 46 × 14.5, 바닥 자석 + 구멍 4 (40 × 34.5), 98 g [판매처]
  · 상태 표시등 **큐라이트 ST45ML-BZ-3-12-RAG** — Ø45, 장착면 위 188.9 (= 68 + 40 × 3 단), 스터드 M4 ×3 PCD 30, 0.25 kg [제조사 3D]
구조 [설계]: 밑판 AL5052 4t (x 200 × y 460) 이 레일 3 본에 걸쳐 M5 T 너트 6 개. 라이다는 2020 기둥 2 본 (간격 100) +
  윗판 AL 3t 위 — 스캔 평면이 표시등 꼭대기보다 위 (표시등이 라이다 시야를 가리지 않게).

좌표: 차량 좌표 (mm). 모듈 원점 = 밑판 밑면 가운데 (MODULE["x"], 0, 레일 윗면).
"""

from build123d import Box, BuildPart, BuildSketch, Cylinder, Locations, Mode, Plane, Pos, RectangleRounded, Rot, extrude

import tslot

MODULE = dict(x=40.0, plate=(200.0, 460.0, 4.0))          # 밑판 가운데 x · 크기 (x, y, 두께). x −60~+140 — 덮개 그로밋 (x −100) 을 피함
MAST = dict(x=60.0, post_y=50.0, post_h=175.0, top=(80.0, 140.0, 3.0))   # 밑판 위 기둥 (밑판 원점 기준 x), 윗판 (x, y, t)
LIDAR_SCAN = 27.2                                          # STL-19P 바닥 → 스캔 평면 [데이터시트]
GNSS_AT = (-70.0, 165.0)                                    # 밑판 원점 기준 (x, y)
STATUS_AT = (-70.0, -165.0)
BOLTS = [(x, y) for x in (-70.0, 70.0) for y in (-210.0, 0.0, 210.0)]   # 밑판 → 레일 M5 (y = 레일 중심)


def lidar_stl19p():
    """STL-19P — 원점 = 바닥 · 회전 중심. 몸체 38.59 □ × 22.2, 회전부 Ø35.29 × 12.8 (전체 35)."""
    with BuildPart() as p:
        with Locations((0, 0, 11.1)):
            Box(38.59, 38.59, 22.2)
        ears = ((8.5, 23.4), (8.5, -23.4), (-23.42, 0.0))                  # 귀 3 (두께 3) + 구멍 Ø2.5
        for x, y in ears:
            with Locations((x, y, 1.5)):
                Cylinder(3.5, 3.0)
            with Locations((x, y / 2, 1.5) if y else (x / 2, 0, 1.5)):
                Box(7.0, abs(y), 3.0) if y else Box(abs(x), 7.0, 3.0)
        for x, y in ears:
            with Locations((x, y, 1.5)):
                Cylinder(1.25, 3.02, mode=Mode.SUBTRACT)
        with Locations((0, 0, 22.2 + 6.4)):
            Cylinder(35.29 / 2, 12.8)
        with Locations((19.3 + 1.5, -8.0, 8.0)):                          # ZH1.5 4P 커넥터 [도면 비례]
            Box(3.0, 7.0, 5.0)
    return p.part


def gps630():
    """GPS630 — 55 × 46 × 14.5 둥근 모서리, 케이블은 55 변 가운데 (+x 쪽). 원점 = 바닥 가운데."""
    with BuildPart() as p:
        with BuildSketch():
            RectangleRounded(55.0, 46.0, 8.0)
        extrude(amount=14.5)
        with Locations((27.5 + 6.0, 0, 5.0)):
            Cylinder(2.0, 12.0, rotation=(0, 90, 0))
    return p.part


def status_light():
    """큐라이트 ST45ML-BZ-3 — 베이스 · 부저 60, 단 40 × 3 (아래부터 초록 · 노랑 · 빨강), 캡 8. 원점 = 장착면 가운데.
    반환: [(이름, 형상)] — 단마다 따로 (시뮬레이터가 색을 켠다)."""
    out = [("베이스", Pos(0, 0, 30.0) * Cylinder(22.5, 60.0) + Pos(0, 0, -4.5) * Cylinder(9.0, 9.0))]   # 스터드 자리 묶음 [가정]
    for i, nm in enumerate(("초록", "노랑", "빨강")):
        out.append((nm, Pos(0, 0, 60.0 + 40.0 * i + 20.0) * Cylinder(22.5, 39.6)))
    out.append(("캡", Pos(0, 0, 60.0 + 120.0 + 4.45) * Cylinder(21.0, 8.9)))
    return out


def build(z_rail_top):
    """그룹별 부품 dict — key: [Part]. 차량 좌표."""
    mx = MODULE["x"]
    px, py, pt = MODULE["plate"]
    z0 = z_rail_top
    plate = Pos(mx, 0, z0 + pt / 2) * Box(px, py, pt)
    for x, y in BOLTS:                                                     # M5 통과 구멍 Ø5.5
        plate = plate - Pos(mx + x, y, z0 + pt / 2) * Cylinder(2.75, pt + 0.02)
    for x, y in (GNSS_AT, STATUS_AT):                                      # 케이블 구멍 Ø12 (표시등은 가운데 케이블)
        plate = plate - Pos(mx + x, y, z0 + pt / 2) * Cylinder(6.0, pt + 0.02)
    plate.label = "센서모듈_밑판_AL4t"
    zp = z0 + pt
    posts = []
    for i, sy in enumerate((-1, 1)):
        b = tslot.bar(20, MAST["post_h"], (mx + MAST["x"], sy * MAST["post_y"], zp), "Z")
        b.label = f"센서모듈_기둥_2020_{i + 1}"
        posts.append(b)
    tx, ty, tt = MAST["top"]
    ztop = zp + MAST["post_h"]
    top = Pos(mx + MAST["x"], 0, ztop + tt / 2) * Box(tx, ty, tt)
    top.label = "센서모듈_라이다판_AL3t"
    brackets = []
    for i, sy in enumerate((-1, 1)):                                        # 기둥 밑 2020 코너 브래킷 (기둥 앞뒤 2 개씩)
        for sx in (-1, 1):
            x0 = mx + MAST["x"] + sx * 10.0
            br = (Pos(x0 + sx * 1.5, sy * MAST["post_y"], zp + 9.0) * Box(3.0, 18.0, 18.0)
                  + Pos(x0 + sx * 9.0, sy * MAST["post_y"], zp + 1.5) * Box(18.0, 18.0, 3.0))
            br.label = f"센서모듈_브래킷_2020_{len(brackets) + 1}"
            brackets.append(br)
    lid = Pos(mx + MAST["x"], 0, ztop + tt) * lidar_stl19p()
    lid.label = "라이다_LDROBOT_STL-19P"
    gn = Pos(mx + GNSS_AT[0], GNSS_AT[1], zp) * gps630()
    gn.label = "GNSS_아센코리아_GPS630"
    status = []
    for nm, s in status_light():
        p = Pos(mx + STATUS_AT[0], STATUS_AT[1], zp) * s
        p.label = f"상태표시등_{nm}_ST45ML"
        status.append(p)
    scan_z = ztop + tt + LIDAR_SCAN
    return dict(module_plate=[plate, top] + posts + brackets, lidar=[lid], gnss=[gn], status_light=status), scan_z
