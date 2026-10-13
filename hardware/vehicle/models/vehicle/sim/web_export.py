r"""차량 웹 시뮬레이터 데이터 — 버전의 메시 캐시 (output/scene.npz) 를 웹용으로 옮긴다.

    .venv\Scripts\python.exe models\vehicle\sim\web_export.py [버전]      (기본: 메시 캐시가 있는 마지막 버전)

결과: models/vehicle/sim/web/data/  (다시 만들 수 있으니 git 에 안 넣는다)
    vehicle.json  부품 (이름 · 그룹 · 색 · 질량 · 역할) · 바퀴 · 센서 · 전원 값 · 버전 정보
    vehicle.bin   부품마다 float32 xyz 삼각형 수프 (m 단위, 차량 좌표: 지면 z=0, 앞 +x, 왼 +y)

record.py 의 메시 캐시를 그대로 쓰므로 버전을 다시 지을 필요가 없다. 버전 상수 (바퀴 · 카메라 · 전조등 ·
라이다 등) 는 그 버전의 assembly · shell 을 import 해 읽는다.
"""

import importlib.util
import json
import re
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
VEH = HERE.parent
OUT = HERE / "web" / "data"
sys.path.insert(0, str(VEH))

import record as R                                                     # noqa: E402

# 밀도 g/cm³ (= kg/L) — 부피로 질량을 어림하는 그룹. 무게가 알려진 부품은 아래 KNOWN.
DENSITY = {"frame": 2.70, "module_rail": 2.70, "frame_bracket": 2.70, "floor": 1.20, "floor_bolt": 7.85,
           "bolt": 7.85, "clamp_low": 1.07, "clamp_oem": 2.70, "shell_acm": 1.50, "shell_pc": 1.20,
           "shell_lid": 1.50, "shell_asa": 1.07, "shell_rubber": 1.20, "lid_knob": 1.2}
KNOWN = {"motor": 3.0}            # 6.5" 호버보드 허브모터 한 개 약 3 kg [가정]
ROLE = {"motor": "wheel", "camera": "camera", "headlight": "headlight", "antenna": "antenna",
        "estop": "estop", "status_light": "status", "lidar": "lidar"}


def latest_version():
    vs = sorted(p.name for p in VEH.iterdir() if re.match(r"v\d\d_", p.name) and (p / "output" / "scene.npz").exists())
    return vs[-1]


def load_version_modules(ver):
    """그 버전 폴더의 assembly 를 불러온다 (상수 · shell · parts 를 읽으려고)."""
    d = VEH / ver
    sys.path.insert(0, str(d))
    spec = importlib.util.spec_from_file_location(f"asm_{ver}", d / "assembly.py")
    A = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = A
    spec.loader.exec_module(A)
    return A


def main():
    ver = next((a for a in sys.argv[1:] if not a.startswith("-")), None) or latest_version()
    meta, tris, vn, owner = R.load_scene(ver)
    A = load_version_modules(ver)
    SH, PT = A.SH, A.PT
    sh = SH.Shell(A)
    known_parts = {p[1]: p[5] for p in PT.PARTS}

    OUT.mkdir(parents=True, exist_ok=True)
    order = np.argsort(owner, kind="stable")
    tris_m = (tris[order] / 1000.0).astype(np.float32)               # mm → m
    own = owner[order]
    starts = np.searchsorted(own, np.arange(len(meta["leaves"])))
    ends = np.searchsorted(own, np.arange(len(meta["leaves"])), side="right")

    parts, mass_sum, com_sum = [], 0.0, np.zeros(3)
    for i, l in enumerate(meta["leaves"]):
        key = l["key"]
        vol_l = abs(l.get("volume") or 0.0) / 1e6                       # mm³ → L
        if key in KNOWN:
            m = KNOWN[key]
        elif key.startswith("part_") and known_parts.get(l["label"]):
            m = known_parts[l["label"]]
        else:
            m = vol_l * DENSITY.get(key, 1.2)
        com = np.array(l.get("com") or [(a + b) / 2 for a, b in zip(l["min"], l["max"])]) / 1000.0
        mass_sum += m
        com_sum += m * com
        parts.append(dict(name=l["label"], group=l["group"], key=key, color=l["color"],
                          start=int(starts[i]), count=int(ends[i] - starts[i]), mass=round(m, 4),
                          role=ROLE.get(key, "body"), min=[v / 1000 for v in l["min"]], max=[v / 1000 for v in l["max"]]))
    tris_m.tofile(OUT / "vehicle.bin")

    # 바퀴 — 축 중심 (차량 좌표, m). 순서는 휠모터 1~4 (assembly.wheels 와 같다)
    wheels = []
    for i, (x, sy) in enumerate((x, sy) for x in (-A.AXLE_X, A.AXLE_X) for sy in (-1, 1)):
        wheels.append(dict(name=f"휠모터{i + 1}", x=x / 1000, y=sy * A.WHEEL_Y / 1000, z=A.AXLE_Z / 1000, side=sy))
    r_wheel = A.AXLE_Z / 1000

    cams = []
    for sx in (1, -1):
        z = SH.CAM.get("z_front", SH.CAM["z"]) if sx > 0 else SH.CAM["z"]
        cams.append(dict(name="앞 뎁스 카메라" if sx > 0 else "뒤 뎁스 카메라", x=sx * sh.x_out / 1000, y=0.0, z=z / 1000,
                         yaw=0.0 if sx > 0 else 180.0, hfov=90.0, vfov=65.0, near=0.1, ideal=[0.3, 3.0], far=6.0,
                         model="Orbbec Gemini 336"))
    lights = []
    if hasattr(SH, "HEADLIGHTS"):
        x0 = (sh.x_out + SH.HL.lamp_x0()) / 1000
        lights = [dict(x=x0, y=hy / 1000, z=hz / 1000, beam=50.0, lm=410) for hy, hz in SH.HEADLIGHTS]
    lidar = None
    lid_parts = [p for p in parts if p["role"] == "lidar"]
    if lid_parts:
        lo = np.min([p["min"] for p in lid_parts], axis=0)
        hi = np.max([p["max"] for p in lid_parts], axis=0)
        L = getattr(A, "LIDAR", {}) or {}
        lidar = dict(x=float((lo[0] + hi[0]) / 2), y=float((lo[1] + hi[1]) / 2),
                     z=float(L.get("scan_z", hi[2] * 1000 - 10)) / 1000 if "scan_z" in L else float(hi[2] - 0.01),
                     range=float(L.get("range_m", 12.0)), rays=int(L.get("rays", 360)), model=L.get("model", "2D 라이다"))
    status = None
    st = [p for p in parts if p["role"] == "status"]
    if st:
        lo = np.min([p["min"] for p in st], axis=0)
        hi = np.max([p["max"] for p in st], axis=0)
        status = dict(x=float((lo[0] + hi[0]) / 2), y=float((lo[1] + hi[1]) / 2), z0=float(lo[2]), z1=float(hi[2]))

    lo = np.min([l["min"] for l in meta["leaves"]], axis=0) / 1000
    hi = np.max([l["max"] for l in meta["leaves"]], axis=0) / 1000
    data = dict(
        version=ver, title=meta["title"], date=meta["date"], summary=meta["summary"],
        parts=parts, wheels=wheels, wheel_radius=r_wheel, wheel_keys=meta["wheel_keys"],
        track=2 * A.WHEEL_Y / 1000, wheelbase=2 * A.AXLE_X / 1000,
        bbox=dict(min=lo.tolist(), max=hi.tolist()),
        mass=round(mass_sum, 2), com=(com_sum / mass_sum).tolist(),
        cameras=cams, headlights=lights, lidar=lidar, status=status,
        # 전원 · 구동 (DECISIONS D-04 · D-05)
        power=dict(v_nom=36.0, v_full=42.0, wh=712.0, bms_max_a=30.0, elec_w_typ=30.0, elec_w_max=64.0,
                   eta=0.75, kt=0.52, i_phase_max=15.0, mu_lat=0.6, f_roll=0.02, v_max=1.5,
                   note="DECISIONS.md D-04 전력 예산과 같은 식 — 추정값"),
    )
    (OUT / "vehicle.json").write_text(json.dumps(data, ensure_ascii=False), encoding="utf-8")
    print(f"{ver}: 부품 {len(parts)} · 삼각형 {len(tris_m):,} · {(OUT / 'vehicle.bin').stat().st_size / 1e6:.1f} MB · "
          f"질량 어림 {mass_sum:.1f} kg · 무게중심 {np.round(com_sum / mass_sum * 1000, 0)} mm"
          + (f" · 라이다 {lidar['model']}" if lidar else ""))


if __name__ == "__main__":
    main()
