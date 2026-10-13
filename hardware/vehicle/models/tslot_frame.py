# T슬롯 알루미늄 프로파일 프레임 (20/30/40 시리즈) — 가장 단순한 직육면체 예제
#
# 단면과 빔 생성은 tslot.py 라이브러리가 맡고, 여기서는 배치만 한다.
# 조인트는 맞댐(butt) 방식이라 프레임 외곽 치수 = FRAME_L x FRAME_W x FRAME_H 그대로다.

from pathlib import Path

from build123d import *
from ocp_vscode import show

from tslot import PROFILES, bar, cut_list

# ---- 파라미터 (단위: mm) ----
SERIES = 20           # 20 / 30 / 40 — 프로파일 시리즈

FRAME_L = 400         # 프레임 외곽 X
FRAME_W = 300         # 프레임 외곽 Y
FRAME_H = 400         # 프레임 외곽 Z

COLOR = "#c8ccd0"     # 알루미늄 아노다이징 은색

EXPORT_DIR = Path(__file__).parent.parent / "exports"
EXPORT_NAME = "tslot_frame"

S = PROFILES[SERIES]["size"]

# ---- 배치 (프레임 중심이 원점, 바닥이 z=0) ----
cx, cy = (FRAME_L - S) / 2, (FRAME_W - S) / 2   # 기둥 중심 오프셋
LEVELS = [S / 2, FRAME_H - S / 2]               # 가로대 2단 (바닥/천장)

BEAMS = (
    # (이름, 길이, 시작점, 방향)
    [(f"기둥{i+1}", FRAME_H, (sx * cx, sy * cy, 0), "Z")
     for i, (sx, sy) in enumerate([(-1, -1), (1, -1), (1, 1), (-1, 1)])]
    + [(f"X가로대{i+1}", FRAME_L - 2 * S, (-(FRAME_L / 2 - S), sy * cy, z), "X")
       for i, (sy, z) in enumerate([(s, z) for z in LEVELS for s in (-1, 1)])]
    + [(f"Y가로대{i+1}", FRAME_W - 2 * S, (sx * cx, -(FRAME_W / 2 - S), z), "Y")
       for i, (sx, z) in enumerate([(s, z) for z in LEVELS for s in (-1, 1)])]
)

parts = []
for name, length, start, axis in BEAMS:
    b = bar(SERIES, length, start, axis)
    b.label, b.color = name, Color(COLOR)
    parts.append(b)

assembly = Compound(children=parts)
assembly.label, assembly.color = EXPORT_NAME, Color(COLOR)

bb = assembly.bounding_box()
print(f"외곽 {bb.size.X:.0f} x {bb.size.Y:.0f} x {bb.size.Z:.0f} mm")
cut_list([(n, l) for n, l, _, _ in BEAMS], SERIES)

# ---- 뷰어 표시 (뷰어가 없으면 건너뜀) ----
try:
    show(assembly)
except Exception as e:
    print(f"뷰어에 표시하지 못함 (뷰어 미실행?): {e}")

# ---- 내보내기 ----
EXPORT_DIR.mkdir(exist_ok=True)
export_step(assembly, str(EXPORT_DIR / f"{EXPORT_NAME}.step"))
export_stl(assembly, str(EXPORT_DIR / f"{EXPORT_NAME}.stl"))
print(f"내보내기 완료: {EXPORT_DIR / EXPORT_NAME}.step / .stl")
