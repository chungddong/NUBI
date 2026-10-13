r"""복도 3D 맵 (F:\Develop\hallway-3d) 을 시뮬레이터 환경으로 가져온다 — 원본은 읽기만 한다.

    .venv\Scripts\python.exe models\vehicle\sim\import_hallway.py [원본 폴더]

복사하는 것 → models/vehicle/sim/web/hallway/  (git 에 안 넣는다 — 원본 프로젝트에 있으니)
    world/**        형상 코드 (buildArchitecture · buildObjects · 재질 · 팩토리) — 그대로 쓴다
    data/scene.json 벽 26 · 바닥 · 천장 · 오브젝트 153 (m, x = 복도 방향, y = 위, z = 라운지 쪽 오른쪽)
    textures/**     사진 텍스처 — 원본에 있을 때만 (2026-09 부터 원본은 텍스처를 코드로 그려 이 폴더가 없다)
    SOURCE.json     원본 경로 · git 커밋 · 복사 시각
"""

import json
import shutil
import subprocess
import sys
import time
from pathlib import Path

SRC = Path(next((a for a in sys.argv[1:] if not a.startswith("-")), r"F:\Develop\hallway-3d"))
DST = Path(__file__).resolve().parent / "web" / "hallway"


def main():
    if not (SRC / "public" / "data" / "scene.json").exists():
        sys.exit(f"복도 프로젝트를 못 찾았다: {SRC}")
    if DST.exists():
        shutil.rmtree(DST)
    shutil.copytree(SRC / "src" / "world", DST / "world")
    (DST / "data").mkdir(parents=True)
    shutil.copy2(SRC / "public" / "data" / "scene.json", DST / "data" / "scene.json")
    if (SRC / "public" / "textures").exists():
        shutil.copytree(SRC / "public" / "textures", DST / "textures")
    try:
        commit = subprocess.run(["git", "-C", str(SRC), "rev-parse", "--short", "HEAD"], capture_output=True, text=True).stdout.strip()
    except OSError:
        commit = ""
    (DST / "SOURCE.json").write_text(json.dumps({"source": str(SRC), "commit": commit,
                                                 "copied": time.strftime("%Y-%m-%d %H:%M")}, ensure_ascii=False), encoding="utf-8")
    n = sum(1 for _ in DST.rglob("*") if _.is_file())
    size = sum(p.stat().st_size for p in DST.rglob("*") if p.is_file()) / 1e6
    print(f"복도 가져옴: {SRC} ({commit}) → {DST}  파일 {n} · {size:.1f} MB")


if __name__ == "__main__":
    main()
