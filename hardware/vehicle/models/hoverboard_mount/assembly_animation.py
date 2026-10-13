"""Design D 조립 순서 애니메이션을 ocp_vscode 뷰어로 보낸다.

순서: 하부 클램프 → 실제 휠 모터 축 → 기존 상부 클램프 → 3030 밀어 넣기
      → 측면 M6x20 4개 → 바닥 M6x20 2개
"""

import sys
from pathlib import Path

sys.path[:0] = [str(Path(__file__).parent), str(Path(__file__).parent.parent)]

from ocp_vscode import Animation, Camera, set_port, show  # noqa: E402

from low_clamp import (                                    # noqa: E402
    bottom_button_head_bolts_reference,
    low_clamp,
    oem_clamp_block,
    rail,
    side_button_head_bolts_reference,
)
from motor_reference import actual_motor_step_reference     # noqa: E402


PORT = 3939


def main():
    profile = rail()
    lower = low_clamp()
    motor = actual_motor_step_reference(axle_z=0)
    upper = oem_clamp_block()
    side = side_button_head_bolts_reference(20)
    bottom = bottom_button_head_bolts_reference(20)

    names = [
        "01_3030_profile_fixed",
        "02_lower_clamp",
        "03_actual_wheel_motor",
        "04_oem_upper_clamp",
        "05_side_bolt_L_front",
        "05_side_bolt_L_rear",
        "05_side_bolt_R_front",
        "05_side_bolt_R_rear",
        "06_bottom_bolt_L",
        "06_bottom_bolt_R",
    ]
    parts = [profile, lower, motor, upper] + side + bottom
    colors = [
        "#69727d", "#c9ced4", "#34383d", "#f5a623",
        "#30343a", "#30343a", "#30343a", "#30343a",
        "#30343a", "#30343a",
    ]

    set_port(PORT)
    show(*parts, names=names, colors=colors, progress=None,
         reset_camera=Camera.CENTER)

    anim = Animation()
    # 완성 위치를 기준으로 한 폭발 오프셋이 0으로 모이도록 만든다.
    anim.add_track("/Group/02_lower_clamp", "tz",
                   [0, 0.3, 1.8, 12.0], [-70, -70, 0, 0])
    # 축은 옆에서 관통시키지 않는다. 하부 반원 시트 위로 수직 하강시켜 얹는다.
    anim.add_track("/Group/03_actual_wheel_motor", "tz",
                   [0, 1.9, 4.0, 12.0], [95, 95, 0, 0])
    # 휠이 내려오는 동안 상부 클램프는 휠의 초기 상단(z≈174)보다 위에서 대기한다.
    anim.add_track("/Group/04_oem_upper_clamp", "tz",
                   [0, 4.1, 5.5, 12.0], [200, 200, 0, 0])

    # 상·하 클램프가 축을 문 뒤, 3030을 레일 끝에서 X방향으로 밀어 넣는다.
    anim.add_track("/Group/01_3030_profile_fixed", "tx",
                   [0, 5.6, 7.8, 12.0], [480, 480, 0, 0])

    # 측면 볼트는 각 플랜지의 바깥쪽에서 동시에 들어온다.
    for i, name in enumerate(names[4:8]):
        sy = -1 if i % 2 == 0 else 1
        anim.add_track(f"/Group/{name}", "ty",
                       [0, 7.9, 9.5, 12.0], [sy * 32, sy * 32, 0, 0])

    # 바닥 볼트는 카운터보어 아래에서 위로 들어온다.
    for name in names[8:10]:
        anim.add_track(f"/Group/{name}", "tz",
                       [0, 9.6, 11.2, 12.0], [-45, -45, 0, 0])

    anim.animate(speed=1)
    print("ASSEMBLY_ANIMATION_SENT 0-12.0s")


if __name__ == "__main__":
    main()
