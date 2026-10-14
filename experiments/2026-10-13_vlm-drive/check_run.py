"""녹화 (record.py · pad.py A 버튼) 점검 — 빠진 사진 · 끊긴 보고 줄 · 전압 · 이동 거리 · 정지 · 패드 입력.

    python3 check_run.py <녹화 폴더> [...]        폴더마다 한 단락, 끝에 표 한 줄씩 (--json 이면 run_check.json 도)
"""

import argparse
import csv
import json
import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from odometry import Odometry  # noqa: E402


def gaps(ts):
    ts = [t for t in ts if t is not None]
    d = [b - a for a, b in zip(ts, ts[1:])]
    return (max(d) if d else 0.0), sum(1 for x in d if x > 0.5)


def check(run: Path):
    meta = json.loads((run / "meta.json").read_text(encoding="utf-8"))
    cfg = meta.get("robot") or json.loads((HERE / "robot.json").read_text(encoding="utf-8"))
    cfg = dict(cfg, **{k: v for k, v in json.loads((HERE / "robot.json").read_text(encoding="utf-8")).items()
                       if k in ("m_per_step", "track_eff_m", "odo_scale", "odo_sign")})   # 지금 보정값으로
    F = list(csv.DictReader(open(run / "frames.csv", encoding="utf-8")))
    W = list(csv.DictReader(open(run / "wheels.csv", encoding="utf-8")))
    s = dict(run=run.name, seconds=float(F[-1]["t"]) if F else 0.0, frames=len(F))
    for cam in ("xtion", "webcam"):
        col = f"{cam}_t"
        ts = [float(r[col]) if r.get(col) else None for r in F]
        have = sum(t is not None for t in ts)
        files = len(list((run / cam).glob("*.jpg"))) if (run / cam).exists() else 0
        g, n = gaps(ts)
        s[cam] = dict(frames=have, files=files, max_gap_s=round(g, 2), gaps_over_0_5s=n)
    if (run / "webcam_video.csv").exists():
        V = list(csv.DictReader(open(run / "webcam_video.csv", encoding="utf-8")))
        tv = [float(r["t"]) for r in V]
        g, n = gaps(tv)
        s["webcam_video"] = dict(frames=len(V), fps=round(len(V) / max(1e-6, tv[-1] - tv[0]), 1) if len(tv) > 1 else 0,
                                 max_gap_s=round(g, 2), mkv_mb=round((run / "webcam.mkv").stat().st_size / 1e6, 1)
                                 if (run / "webcam.mkv").exists() else None)
    tw = [float(r["t"]) for r in W]
    g, n = gaps(tw)
    volts = [float(r["volt"]) for r in W if float(r["volt"]) > 1]
    rx = [int(r["rx"]) for r in W]
    rxb = [int(r["rx_b"]) for r in W]
    odo = Odometry(cfg)
    stops, still0, moving_s = [], None, 0.0
    speeds = []
    for i, r in enumerate(W):
        odo.update((int(r["odo_m"]), int(r["odo_s"])))
        v = (float(r["kmh_r"]) - float(r["kmh_l"]) + float(r["kmh_b_r"]) - float(r["kmh_b_l"])) / 4
        if abs(v) < 8:
            speeds.append(v)
        t = tw[i]
        if abs(v) < 0.1:
            still0 = t if still0 is None else still0
        else:
            if still0 is not None and t - still0 >= 2.0:
                stops.append((round(still0, 1), round(t - still0, 1)))
            still0 = None
            if i:
                moving_s += t - tw[i - 1]
    s["wheels"] = dict(reports=len(W), rate_hz=round(len(W) / max(1e-6, tw[-1] - tw[0]), 1) if len(tw) > 1 else 0,
                       max_gap_s=round(g, 2), gaps_over_0_5s=n,
                       volt=[round(min(volts), 1), round(max(volts), 1)] if volts else None,
                       board_link=dict(front_rx=rx[-1] - rx[0] if rx else 0, rear_rx=rxb[-1] - rxb[0] if rxb else 0),
                       odo_jumps=odo.jumps, distance_m=round(odo.dist, 1), moving_s=round(moving_s, 1),
                       max_kmh=round(max((abs(x) for x in speeds), default=0), 2),
                       turn_deg_net=round(math.degrees(odo.yaw), 0), stops=stops)
    if (run / "commands.csv").exists():
        C = list(csv.DictReader(open(run / "commands.csv", encoding="utf-8")))
        s["pad"] = dict(rows=len(C), armed=round(sum(r["armed"] == "1" for r in C) / max(1, len(C)), 2),
                        l1=round(sum(r["l1"] == "1" for r in C) / max(1, len(C)), 2),
                        levels=sorted({r["level"] for r in C}), turns=sum(1 for r in C if r.get("turn") not in ("", "0", None)))
    return s


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("runs", nargs="+", type=Path)
    ap.add_argument("--json", action="store_true")
    a = ap.parse_args()
    rows = []
    for run in a.runs:
        try:
            s = check(run)
        except Exception as e:
            print(f"{run.name}: 점검 실패 — {type(e).__name__}: {e}")
            continue
        rows.append(s)
        print(json.dumps(s, ensure_ascii=False))
        if a.json:
            (run / "run_check.json").write_text(json.dumps(s, ensure_ascii=False, indent=2), encoding="utf-8")


if __name__ == "__main__":
    main()
