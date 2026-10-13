"""하부 클램프만 뷰어에 올린다 — 현행(날개 없음) + 비교용 옛 형상(날개 있음).

    .venv/Scripts/python.exe models/hoverboard_mount/clamp_scene.py

왼쪽: 현행 Design D 를 레일·기존 상부 클램프·축·밑면 볼트·타이 브래킷과 조립한 모습.
오른쪽(x+150): 08-21 에 뺀 옛 형상 — 옆 날개 4개 달린 것. 날개가 어디에 어떻게
붙어 있었는지 보라고 반투명으로 둔다. LOW_SIDE 는 parameters 모듈 상수라 같은
프로세스에서 두 벌을 만들 수 없어 자식 프로세스로 STEP 을 뽑아 들여온다.
"""

import os
import subprocess
import sys
import tempfile
from pathlib import Path

HERE = Path(__file__).parent
sys.path[:0] = [str(HERE), str(HERE.parent)]

from build123d import Color, Compound, Location, Pos             # noqa: E402
from build123d.importers import import_step                      # noqa: E402
from ocp_vscode import Camera, set_port, show                    # noqa: E402

import low_clamp as LC                                           # noqa: E402
from axle import axle_solid                                      # noqa: E402
import parameters as P                                           # noqa: E402

PORT = 3939
OFFSET_X = 150.0          # 비교용 옛 형상을 옆에 놓는 간격


def _colored(part, label, color):
    part = part.moved(Location())
    part.label, part.color = label, Color(color)
    return part


def flanged_old_clamp():
    """LOW_SIDE=1 로 자식 프로세스에서 옛 형상 STEP 을 뽑아 들여온다."""
    out = Path(tempfile.gettempdir()) / "low_clamp_flanged_compare.step"
    code = (
        "import sys; sys.path[:0]=[%r,%r]\n"
        "from build123d.exporters3d import export_step\n"
        "import low_clamp as LC\n"
        "export_step(LC.low_clamp(), %r)\n" % (str(HERE), str(HERE.parent), str(out))
    )
    env = dict(os.environ, LOW_SIDE="1", LOW_SHELF="0", TIE="0")   # 옛 형상은 날개만
    subprocess.run([sys.executable, "-c", code], env=env, check=True,
                   capture_output=True, text=True)
    return import_step(str(out))


def main():
    clamp = _colored(LC.low_clamp(), "하부클램프_현행_DesignD", "#c9ced4")
    oem = _colored(LC.oem_clamp_block(), "기존_상부클램프", "#f5a623")
    axle = _colored(axle_solid(), "축_Ø16_D컷", "#8d9298")
    rail = _colored(LC.rail(), "레일_3030", "#69727d")
    bolts = [_colored(b, f"밑면볼트_M6x{P.LOW_BOLT_LEN:.0f}_{i+1}", "#30343a")
             for i, b in enumerate(LC.bottom_button_head_bolts_reference())]
    brks = [_colored(b, f"타이브래킷_{i+1}", "#b6483c")
            for i, b in enumerate(LC.tie_brackets(210.0))]
    # 브래킷은 차량 좌표(레일 y=210)로 나오므로 클램프 로컬(레일 y=CLAMP_CY)로 옮긴다
    brks = [Pos(0, LC.CLAMP_CY - 210.0, 0) * b for b in brks]
    # tie_brackets() 는 좌우 클램프 몫 4개를 다 낸다 — 이 클램프(y≈CLAMP_CY) 쪽 2개만
    brks = [b for b in brks if abs(b.bounding_box().center().Y - LC.CLAMP_CY) < 60.0]
    for i, b in enumerate(brks):
        b.label = f"타이브래킷_{i+1}"

    shelf_bolts = [_colored(b, f"선반볼트_M6x{P.LOW_BOLT_LEN:.0f}_{i+1}", "#30343a")
                   for i, b in enumerate(LC.shelf_button_head_bolts_reference())]
    hx = [Pos(0, LC.CLAMP_CY - 210.0, 0) * _colored(c, "H크로스_3030", "#4f7fb0")
          for c in LC.h_cross(210.0)]
    for c in hx:
        c.label = "H크로스_3030"

    old = flanged_old_clamp()
    old = Pos(OFFSET_X, 0, 0) * old
    old.label = "옛형상_옆날개4개_08-21에_뺌"

    groups = [clamp, oem, axle, rail, Compound(children=bolts)]
    names = [clamp.label, oem.label, axle.label, rail.label, "밑면볼트_2개"]
    colors = ["#c9ced4", "#f5a623", "#8d9298", "#69727d", "#30343a"]
    alphas = [1, 1, 1, 0.6, 1]
    if brks:
        groups.append(Compound(children=brks)); names.append(f"타이브래킷_{len(brks)}개")
        colors.append("#b6483c"); alphas.append(1)
    if hx:
        groups += hx + [Compound(children=shelf_bolts)]
        names += ["H크로스_3030", f"선반볼트_{len(shelf_bolts)}개"]
        colors += ["#4f7fb0", "#30343a"]; alphas += [0.6, 1]
    groups.append(old); names.append(old.label); colors.append("#d97757"); alphas.append(0.45)
    set_port(PORT)
    show(*groups, names=names, colors=colors, alphas=alphas,
         progress=None, reset_camera=Camera.RESET)
    bb = clamp.bounding_box()
    print(f"CLAMP_SCENE_SENT clamp={bb.size.X:.2f}x{bb.size.Y:.2f}x{bb.size.Z:.2f} "
          f"vol={clamp.volume/1000:.1f}cm3 old_vol={old.volume/1000:.1f}cm3 "
          f"LOW_SIDE={P.LOW_SIDE} LOW_SHELF={P.LOW_SHELF} TIE={P.TIE and P.TIE_KIND}")


if __name__ == "__main__":
    main()
