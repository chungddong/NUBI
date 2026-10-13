r"""Design-D 하부 클램프를 Ender-3 V3 SE 용으로 슬라이스한다 — Creality Print 7.2 CLI.

    .venv\Scripts\python.exe models\hoverboard_mount\slicing\slice.py

입력  models/hoverboard_mount/output/stl/design_d_lower_clamp_print.stl
      (export_clamp_stl.py 가 만든다. 밑면이 z=0 — 그대로 눕혀 뽑는다)
출력  exports/hoverboard_mount/print/gcode/design_d_lower_clamp.gcode

왜 따로 프로파일을 두나 (2026-09-16):
  같은 STL 을 뱀부랩에서 뽑은 건 멀쩡한데 V3 SE 에서 뽑은 건 옆 날개가 툭 치면
  부러졌다. 날개는 층을 가로질러 굽힘을 받는 자세라 **층간 접착**이 곧 강도다.
  V3 SE 순정 CR-PLA 프로파일(205°C, 팬 100%, 벽 2~3줄)은 거기에 약하다.
  → 노즐 +10°C, 팬 40~70%, 벽 5줄, 인필 60% cubic 으로 얹는다.
  (gyroid 는 같은 60% 에서 2.6배 느렸다 — 16h54 vs 6h29. 순정 속도 60/500 도 250 mm/s
   기계의 반도 안 쓰는 값이라 벽 120/150, 인필 200, 가속 2000 으로 올렸다.)
  (지금 STL 은 날개가 없는 현행 Design D 다 — 날개는 08-21 실물 시험 뒤 뺐다.)

CLI 메모: inherits 는 값을 안 물려준다 — 중요한 키는 전부 JSON 에 명시했고,
gcode 꼬리의 설정 덤프로 실제 적용됐는지 여기서 검증한다 (memory: creality-print-cli).
"""

import json
import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

HERE = Path(__file__).parent
ROOT = HERE.parent.parent.parent
# 인자로 STL 을 주면 그걸 썬다 (예: design_d_lower_clamp_print_tongue.stl — H 크로스 혀 판).
# 안 주면 기본 Design D (날개·혀 없음).
STL = (Path(sys.argv[1]) if len(sys.argv) > 1
       else HERE.parent / "output" / "stl" / "design_d_lower_clamp_print.stl")
# 두 번째 인자 = 인필 밀도 override (예: 30%). 벽·온도·팬은 그대로 — 그게 강도의 본체다.
INFILL = sys.argv[2] if len(sys.argv) > 2 else None
GCODE_DIR = ROOT / "exports" / "hoverboard_mount" / "print" / "gcode"
GCODE = GCODE_DIR / (STL.stem.replace("_print", "") + (f"_{INFILL.rstrip('%')}pct" if INFILL else "") + ".gcode")

EXE = Path(r"C:\Program Files\Creality\Creality Print 7.2\CrealityPrint.exe")
SYSP = Path(r"C:\Program Files\Creality\Creality Print 7.2\resources\profiles\Creality")
MACHINE = SYSP / "machine" / "Creality Ender-3 V3 SE 0.4 nozzle.json"
PROCESS = HERE / "process_clamp.json"
FILAMENT = HERE / "filament_clamp.json"

# gcode 설정 덤프에서 확인할 키 → (프리셋 파일, 키)
CHECK = {
    "nozzle_temperature": (FILAMENT, "nozzle_temperature"),
    "nozzle_temperature_initial_layer": (FILAMENT, "nozzle_temperature_initial_layer"),
    "fan_min_speed": (FILAMENT, "fan_min_speed"),
    "fan_max_speed": (FILAMENT, "fan_max_speed"),
    "wall_loops": (PROCESS, "wall_loops"),
    "top_shell_layers": (PROCESS, "top_shell_layers"),
    "bottom_shell_layers": (PROCESS, "bottom_shell_layers"),
    "sparse_infill_density": (PROCESS, "sparse_infill_density"),
    "sparse_infill_pattern": (PROCESS, "sparse_infill_pattern"),
    "layer_height": (PROCESS, "layer_height"),
    "brim_width": (PROCESS, "brim_width"),
    "outer_wall_speed": (PROCESS, "outer_wall_speed"),
    "sparse_infill_speed": (PROCESS, "sparse_infill_speed"),
    "default_acceleration": (PROCESS, "default_acceleration"),
    "enable_support": (PROCESS, "enable_support"),
}


def want(path, key):
    v = json.loads(path.read_text(encoding="utf-8"))[key]
    return v[0] if isinstance(v, list) else v


def main():
    if not STL.exists():
        sys.exit(f"STL 없음: {STL} — 먼저 export_clamp_stl.py 를 돌려라")
    GCODE_DIR.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory() as td:
        proc = PROCESS
        if INFILL:
            d = json.loads(PROCESS.read_text(encoding="utf-8"))
            d["sparse_infill_density"] = INFILL
            d["name"] = d["name"] + f" {INFILL}"
            proc = Path(td) / "process_override.json"
            proc.write_text(json.dumps(d, ensure_ascii=False), encoding="utf-8")
        cmd = [str(EXE), "--debug", "0",
               "--load-settings", f"{MACHINE};{proc}",
               "--load-filaments", str(FILAMENT),
               "--slice", "0", "--export-slicedata", td, "--outputdir", td, str(STL)]
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=900)
        out = Path(td) / "plate_1.gcode"
        if not out.exists():
            sys.exit(f"슬라이스 실패 (exit {r.returncode})\n{(r.stdout or '')[-800:]}")
        shutil.copy(out, GCODE)

    text = GCODE.read_text(encoding="utf-8", errors="ignore")
    tail = text[-40000:]
    t = re.search(r"estimated printing time.*= (.+)", tail)
    g = re.search(r"filament used \[cm3\] = ([\d.]+)", tail)      # [g] 줄은 이 CLI 가 안 낸다
    grams = f"{float(g.group(1)) * 1.24:.0f} g (PLA 1.24)" if g else "?"
    print(f"GCODE {GCODE.relative_to(ROOT)}")
    print(f"  예상 시간 {t.group(1).strip() if t else '?'} / 필라멘트 {g.group(1) if g else '?'} cm³ ≈ {grams}")

    bad = []
    for key, (path, k) in CHECK.items():
        m = re.search(rf"^; {re.escape(key)} = (.+)$", tail, re.M)
        got = m.group(1).strip() if m else None
        exp = INFILL if (INFILL and k == "sparse_infill_density") else str(want(path, k))
        ok = got is not None and got.split(",")[0].strip() == exp
        print(f"  {'OK ' if ok else 'NG '} {key:34s} = {got}   (원함 {exp})")
        if not ok:
            bad.append(key)
    # 베드 범위 — 퍼지 라인 이후 압출 이동만 본다 (memory: creality-print-cli)
    body = text.split(";LAYER_CHANGE", 1)[-1]
    xs = [float(v) for v in re.findall(r"G1 [^\n]*X([\d.]+)[^\n]*E[\d.]", body)]
    ys = [float(v) for v in re.findall(r"G1 [^\n]*Y([\d.]+)[^\n]*E[\d.]", body)]
    if xs and ys:
        print(f"  압출 범위 X {min(xs):.1f}~{max(xs):.1f} / Y {min(ys):.1f}~{max(ys):.1f} (베드 220x220)")
    if bad:
        sys.exit(f"설정이 적용되지 않은 키: {bad}")
    print("SLICE_OK")


if __name__ == "__main__":
    main()
