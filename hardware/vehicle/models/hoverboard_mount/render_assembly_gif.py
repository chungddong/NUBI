r"""Design D 조립 순서를 오프라인 GIF로 렌더링한다.

실행:
    .venv\Scripts\python.exe models\hoverboard_mount\render_assembly_gif.py
"""

import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).parent
sys.path[:0] = [str(HERE), str(HERE.parent)]

from build123d import Color                            # noqa: E402
from iso_render import animate, collect                # noqa: E402
from low_clamp import (                                # noqa: E402
    bottom_button_head_bolts_reference,
    low_clamp,
    oem_clamp_block,
    rail,
    side_button_head_bolts_reference,
)
from motor_reference import actual_motor_step_reference  # noqa: E402


OUT = HERE / "output" / "animation" / "hoverboard_clamp_assembly.gif"
FPS = 4
DURATION = 12.0


def _ease_offset(t, t0, t1, start):
    if t <= t0:
        return float(start)
    if t >= t1:
        return 0.0
    u = (t - t0) / (t1 - t0)
    u = u * u * (3 - 2 * u)
    return float(start) * (1 - u)


def _translation(x=0.0, y=0.0, z=0.0):
    return np.eye(3), np.array((x, y, z), dtype=float)


def _caption(t):
    if t < 1.9:
        return "1  하부 클램프 배치"
    if t < 4.1:
        return "2  휠 모터 축을 하부 시트에 안착"
    if t < 5.6:
        return "3  상부 클램프 장착"
    if t < 7.9:
        return "4  3030 프로파일을 끝에서 밀어 넣기"
    if t < 9.6:
        return "5  측면 M6×20 볼트 체결"
    return "6  바닥 M6×20 볼트 체결"


def main():
    profile = rail()
    lower = low_clamp()
    motor = actual_motor_step_reference(axle_z=0)
    upper = oem_clamp_block()
    side = side_button_head_bolts_reference(20)
    bottom = bottom_button_head_bolts_reference(20)

    profile.color = Color("#69727d")
    lower.color = Color("#c9ced4")
    motor.color = Color("#34383d")
    upper.color = Color("#f5a623")
    for bolt in side + bottom:
        bolt.color = Color("#30343a")

    shapes = {
        "profile": profile,
        "lower": lower,
        "motor": motor,
        "upper": upper,
        **{f"side{i}": b for i, b in enumerate(side)},
        **{f"bottom{i}": b for i, b in enumerate(bottom)},
    }
    print("메시 생성")
    groups = {name: collect(shape, tol=1.2, ang=0.5)
              for name, shape in shapes.items()}

    times = np.linspace(0, DURATION, int(DURATION * FPS) + 1)
    poses, captions = [], []
    for t in times:
        pose = {
            "lower": _translation(z=_ease_offset(t, 0.3, 1.8, -70)),
            "motor": _translation(z=_ease_offset(t, 1.9, 4.0, 95)),
            "upper": _translation(z=_ease_offset(t, 4.1, 5.5, 200)),
            "profile": _translation(x=_ease_offset(t, 5.6, 7.8, 300)),
        }
        for i in range(4):
            sy = -1 if i % 2 == 0 else 1
            pose[f"side{i}"] = _translation(
                y=_ease_offset(t, 7.9, 9.5, sy * 32))
        for i in range(2):
            pose[f"bottom{i}"] = _translation(
                z=_ease_offset(t, 9.6, 11.2, -45))
        poses.append(pose)
        captions.append(_caption(t))

    animate(groups, poses, OUT, direction=(1.0, -1.2, 0.65),
            size=(720, 540), ss=1, ground=False, fps=FPS,
            captions=captions, margin=1.08)
    print(f"GIF_READY {OUT}")


if __name__ == "__main__":
    main()
