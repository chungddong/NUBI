# 예제 모델: 모서리를 둥글린 단순 상자
# 새 모델을 만들 때 이 파일 구조를 참고한다.

from pathlib import Path

from build123d import *
from ocp_vscode import show

# ---- 파라미터 (단위: mm) ----
LENGTH = 60
WIDTH = 40
HEIGHT = 20
FILLET_RADIUS = 3

EXPORT_DIR = Path(__file__).parent.parent / "exports"
EXPORT_NAME = "example_box"

# ---- 형상 ----
with BuildPart() as box:
    Box(LENGTH, WIDTH, HEIGHT)
    fillet(box.edges().filter_by(Axis.Z), radius=FILLET_RADIUS)

# ---- 뷰어 표시 (뷰어가 없으면 건너뜀) ----
try:
    show(box)
except Exception as e:
    print(f"뷰어에 표시하지 못함 (뷰어 미실행?): {e}")

# ---- 내보내기 ----
EXPORT_DIR.mkdir(exist_ok=True)
export_step(box.part, str(EXPORT_DIR / f"{EXPORT_NAME}.step"))
export_stl(box.part, str(EXPORT_DIR / f"{EXPORT_NAME}.stl"))
print(f"내보내기 완료: {EXPORT_DIR / EXPORT_NAME}.step / .stl")
