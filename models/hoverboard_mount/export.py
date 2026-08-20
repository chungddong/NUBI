# STEP / STL 내보내기 + 최종 검증.
#   .venv\Scripts\python.exe models\hoverboard_mount\export.py

import sys
from pathlib import Path

sys.path[:0] = [str(Path(__file__).parent), str(Path(__file__).parent.parent)]

from build123d import *                                          # noqa: E402

import parameters as P                                           # noqa: E402
from adapter_plate import adapter_plate                          # noqa: E402
from assembly import build, verify                               # noqa: E402
import low_clamp as LC                                           # noqa: E402
from extrusion_reference import extrusion_reference              # noqa: E402
from motor_reference import motor_reference                      # noqa: E402
from low_clamp import low_clamp                                  # noqa: E402
from saddle_bracket import saddle_bracket                        # noqa: E402
from split_clamp import axle_clamp_lower, axle_clamp_upper       # noqa: E402
from torque_plate import axle_retainer, torque_plate             # noqa: E402

OUT = Path(__file__).parent / "output"
(OUT / "step").mkdir(parents=True, exist_ok=True)
(OUT / "stl").mkdir(parents=True, exist_ok=True)

# 제작 대상만 STL 을 낸다. 참조 모델(모터·프로파일)은 STEP 만.
MAKE = [("low_clamp", low_clamp, True),                    # Design D — 하단 클램프
        ("saddle_bracket", saddle_bracket, True),          # Design C — 통짜 CNC
        ("axle_clamp_upper", axle_clamp_upper, True),
        ("axle_clamp_lower", axle_clamp_lower, True),
        ("adapter_plate", adapter_plate, True),
        ("torque_plate", torque_plate, True),
        ("axle_retainer", axle_retainer, True),
        ("hoverboard_motor_reference", motor_reference, False),
        (f"{P.EXTRUSION}_extrusion_reference", extrusion_reference, False)]

MIN_WALL = 3.0


def thin_check(part, name):
    """지나치게 얇은 곳 — bbox 최소축이 MIN_WALL 이하인 조각이 있는지 훑는다."""
    b = part.bounding_box()
    t = min(b.size.X, b.size.Y, b.size.Z)
    return None if t >= MIN_WALL else f"{name}: 최소 두께 {t:.1f}mm < {MIN_WALL}"


def main():
    print(f"파라미터: 축 Ø{P.AXLE_DIAMETER} D컷 {P.AXLE_ACROSS_FLAT}"
          f" (평면 {P.AXLE_FLAT_LENGTH}mm) / 나사부 {'있음' if P.AXLE_HAS_THREAD else '없음'}")
    print(f"          {P.EXTRUSION} {P.EXTRUSION_W:.0f}x{P.EXTRUSION_H:.0f},"
          f" 슬롯 {'2줄 간격 '+str(P.EXTRUSION_SLOT_PITCH) if P.EXTRUSION_SLOT_PITCH else '면당 1줄'}"
          f" / T너트 M{P.TNUT_BOLT:.0f} x4 (x ±{P.BOLT_X_PITCH/2:.0f})")
    warn = []
    for name, fn, stl in MAKE:
        o = fn()
        n_sol = len(o.solids())
        w = thin_check(o, name)
        if w:
            warn.append(w)
        if n_sol != 1 and "reference" not in name:
            warn.append(f"{name}: 솔리드 {n_sol}개")
        export_step(o.moved(Location()), str(OUT / "step" / f"{name}.step"))
        if stl:
            try:
                export_stl(o.moved(Location()), str(OUT / "stl" / f"{name}.stl"))
            except Exception as e:
                warn.append(f"{name}: STL 실패 {e}")
        print(f"  {name:28s} {o.volume/1000:6.1f} cm3  솔리드 {n_sol}"
              + ("  +STL" if stl else ""))
    # Design A/B 는 어댑터 판이 **슬롯 2줄**에 물리는 걸 전제로 한다. 1줄짜리
    # (3030/4040)에서는 두 줄이 같은 높이로 겹쳐 성립하지 않는다 — 볼트끼리
    # 1175mm3 간섭으로 잡힌다. 프로파일에 맞는 설계만 검증한다.
    if P.EXTRUSION_SLOT_PITCH > 0:
        for d in ("A", "B"):
            bad, n = verify(d)
            export_step(build(d), str(OUT / "step" / f"assembly_{d}.step"))
            warn += [f"[{d}] {m}" for m in bad]
    else:
        print(f"\n[Design A/B] {P.EXTRUSION} 은 면당 슬롯 1줄 — 어댑터 판 전제가 깨진다. 건너뜀")
    bad_d = LC.verify()
    warn += [f"[D] {m}" for m in bad_d]
    print(f"[Design D] 하단 클램프 + 기존 클램프 + {P.EXTRUSION}"
          f" / 옆 플랜지 {'있음' if P.LOW_SIDE else '없음'}"
          f"    {'문제 없음' if not bad_d else f'{len(bad_d)}건'}")
    LC.report()
    print(f"\n검증: {'문제 없음' if not warn else f'{len(warn)}건'}")
    for w in warn:
        print(f"    !! {w}")
    print(f"내보내기 → {OUT}")
    return warn


if __name__ == "__main__":
    main()
