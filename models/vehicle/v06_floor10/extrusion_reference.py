# T슬롯 참조 모델 (EXTRUSION 프리셋).
#
# 처음엔 "속 빈 사각관 + 가운데 리브 + 긴 면에만 슬롯" 으로 대충 그렸다. 외형
# 30x30 은 맞았지만 **윗면·밑면에 슬롯이 없었다.** Design D 의 밑면 볼트 2개가
# 슬롯 없는 면으로 들어가는데도 검사가 통과했다 — 참조 모델이 틀리면 검증이
# 틀린 걸 통과시킨다.
#
# 정확도: 외형·슬롯 입구 폭/위치·T홈 폭·립 두께는 규격값 그대로다 (볼트와 너트가
# 닿는 면은 전부 맞다). T홈을 직사각으로 단순화해서 **단면 중량이 실물보다
# 3030 +10%, 40 계열 +24%** 나온다. 간섭·조립 확인용이지 중량 산출용이 아니다.
#
# 지금은 실제 단면 구성을 따라간다:
#   외곽 사각 - 각 면의 T슬롯(입구 + 안쪽 홈) + 대각 웹(X) - 중심 보어
# T홈을 파면 중앙 보스가 사방에서 끊겨 떨어져 나간다. 실물에서 그걸 붙들고 있는
# 게 45° 대각 웹이라, 홈을 판 뒤 웹을 다시 더한다.

from build123d import *

import parameters as P


def cell_zs():
    """셀 중심 z 목록. 1줄짜리는 [0], 2줄짜리는 [-pitch/2, +pitch/2]."""
    pitch = P.EXTRUSION_SLOT_PITCH
    return [0.0] if pitch == 0 else [-pitch / 2, pitch / 2]


def slot_zs():
    """긴 면 슬롯의 z (프로파일 중심 기준) — 셀 중심과 같다."""
    return tuple(cell_zs())


def profile_sketch():
    w, h, t = P.EXTRUSION_W, P.EXTRUSION_H, P.EXTRUSION_WALL
    slot, cav, core, bore = (P.EXTRUSION_SLOT, P.EXTRUSION_CAVITY,
                             P.EXTRUSION_CORE, P.EXTRUSION_BORE)
    depth = (w / 2 - t) - core / 2                    # 입구 안쪽 홈 깊이
    cells = cell_zs()
    arm = w * 1.4                                     # 대각 웹 길이 (모서리까지)
    web = t                                           # 대각 웹 두께
    with BuildSketch() as sk:
        Rectangle(w, h)
        # 셀 하나당 4면을 같은 규칙으로 처리한다.
        #   바깥 면  → 입구(슬롯) + 안쪽 T홈
        #   셀 경계  → T홈만. 입구가 없으니 양쪽 셀이 t 씩 남겨 4.8mm 판이 된다.
        # 경계를 통살로 두었더니 3060 이 2.81 kg/m 이 나왔다 (실물 약 2.1).
        for cz in cells:
            for is_z, fdir in ((False, -1), (False, 1), (True, -1), (True, 1)):
                face = (cz + fdir * w / 2) if is_z else (fdir * w / 2)
                lim = (h / 2) if is_z else (w / 2)
                outer = abs(abs(face) - lim) < 1e-6
                if outer:                             # 입구
                    c = face - fdir * t / 2
                    with Locations((0, c) if is_z else (c, cz)):
                        Rectangle(*((slot, t + 0.02) if is_z else (t + 0.02, slot)),
                                  mode=Mode.SUBTRACT)
                c = face - fdir * (t + depth / 2)     # 안쪽 T홈
                with Locations((0, c) if is_z else (c, cz)):
                    Rectangle(*((cav, depth) if is_z else (depth, cav)),
                              mode=Mode.SUBTRACT)
        for cz in cells:                              # 대각 웹 — 보스를 붙든다
            for ang in (45, -45):
                with Locations(Location((0, cz), ang)):
                    Rectangle(web, arm)
        # 대각 웹이 모서리 밖으로 삐져나와 외형이 30.00 → 31.40 이 됐다. 잘라낸다.
        Rectangle(w, h, mode=Mode.INTERSECT)
        for cz in cells:
            with Locations((0, cz)):
                Circle(bore / 2, mode=Mode.SUBTRACT)
    return sk.sketch


def extrusion_reference(length=None):
    L = length or P.EXTRUSION_LEN
    with BuildPart() as p:
        # 프로파일 길이의 중심을 조립 데이텀 x=0에 맞춘다.
        # Plane.YZ의 법선은 +X이므로 -L/2에서 시작해 +X로 L만큼 압출한다.
        with BuildSketch(Plane.YZ.offset(-L / 2)):
            add(profile_sketch())
        extrude(amount=L)
    p.part.label = f"{P.EXTRUSION}_extrusion_reference"
    return p.part
