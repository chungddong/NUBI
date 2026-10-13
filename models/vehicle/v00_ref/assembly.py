r"""v00_ref — 재설계 직전 참조 (2026-09-17 마지막 설계)

    .venv/Scripts/python.exe models/vehicle/v00_ref/assembly.py     # 뷰어로 보기
    .venv/Scripts/python.exe models/vehicle/record.py v00_ref       # 기록 이미지

**설계 버전이 아니라 비교 기준이다.** v01_base 의 "무엇이 바뀌었나" 이미지를 만들려고
재설계 직전 상태를 같은 그룹 key 로 다시 세운다.
  · 09-17 floor_plate.py 장면 그대로 (LOW_SHELF=1 TIE=0) — 바닥판·케이블 경로만 뺐다
  · 프레임 3030 760x450: 하부사이드 2, 하부크로스 3 (0, ±365), 축크로스 2 (±265),
    기둥 6 (143), 상부사이드 2, 상부크로스 3
  · H 크로스 2 (±210) — 혀 선반이 이 밑면을 물었다
  · 바퀴·T 클램프·상부 클램프·볼트는 v01 과 같다

코드는 따로 동결하지 않는다 — 클램프·바퀴는 **v01_base 의 동결 사본을 그대로 불러 쓴다**
(09-17 설정이 v01 고정값과 같다: LOW_SHELF=True, SHELF_KIND=tongue, TIE=False).
"""

import importlib.util
import sys
from pathlib import Path

HERE = Path(__file__).parent
V01 = HERE.parent / "v01_base"
_spec = importlib.util.spec_from_file_location("v01_assembly", V01 / "assembly.py")
B = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(B)              # v01 폴더를 sys.path 에 넣고 동결 모듈을 불러온다

VERSION = "v00_ref"
TITLE = "재설계 직전 참조"
DATE = "2026-09-17"
PREV = None
SUMMARY = "09-17 마지막 설계 — H 크로스 + 혀 선반 + 상부 프레임 (비교 기준)"

OUT = HERE / "output"
WHEEL_KEYS = B.WHEEL_KEYS

OLD_AXLE_CROSS_X = B.AXLE_X + 55         # 옛 축크로스 자리 265
POST_H = 143.0                           # 기둥 (하부 레일 윗면 → 상부 레일 밑면)
FRAME_TOP = B.RAIL_Z + B.S / 2
TOP_RAIL_Z = FRAME_TOP + POST_H + B.S / 2


def frame():
    bar = B._bar
    beams = [b for b in B.lower_frame() if not b.label.startswith("축크로스")]
    beams += [bar(f"축크로스{i+1}", B.CROSS_L, (x, -B.CROSS_L / 2, B.RAIL_Z), "Y")
              for i, x in enumerate((-OLD_AXLE_CROSS_X, OLD_AXLE_CROSS_X))]
    beams += [bar(f"기둥{i+1}", POST_H, (x, sy * B.RAIL_Y, FRAME_TOP), "Z")
              for i, (x, sy) in enumerate((x, sy) for x in (-B.CROSS_X, 0, B.CROSS_X)
                                          for sy in (-1, 1))]
    beams += [bar(f"상부크로스{i+1}", B.CROSS_L, (x, -B.CROSS_L / 2, TOP_RAIL_Z), "Y")
              for i, x in enumerate((-B.CROSS_X, 0, B.CROSS_X))]
    beams += [bar(f"상부사이드{i+1}", B.BASE_L, (-B.BASE_L / 2, sy * B.RAIL_Y, TOP_RAIL_Z), "X")
              for i, sy in enumerate((-1, 1))]
    return beams


def build():
    beams = frame()
    hx = [B._bar(f"H크로스{i+1}", B.CROSS_L, (x, -B.CROSS_L / 2, B.RAIL_Z), "Y")
          for i, x in enumerate((-B.AXLE_X, B.AXLE_X))]
    groups = B.build()
    groups[0] = dict(key="frame", label=f"프레임_3030_{len(beams)}본", color="#69727d",
                     parts=beams)
    groups.insert(1, dict(key="h_cross", label="H크로스_3030_2본", color="#4f7fb0", parts=hx))
    return groups


verify = None                            # 비교 기준이라 검증하지 않는다 (09-17 에 floor_plate 가 했다)


def metrics(groups):
    return [("프레임 3030", f"{len(groups[0]['parts'])} 본 + H 크로스 2 본"),
            ("기둥 높이", f"{POST_H:.0f} mm"),
            ("옛 축크로스 x", f"±{OLD_AXLE_CROSS_X:.0f} mm")]


def main():
    from ocp_vscode import Camera, set_port, show
    groups = build()
    comps = B.compounds(groups)
    set_port(B.PORT)
    show(*comps, names=[c.label for c in comps], colors=[d["color"] for d in groups],
         alphas=[1] * len(comps), progress=None, reset_camera=Camera.RESET)
    print(f"{VERSION}_SENT")


if __name__ == "__main__":
    main()
