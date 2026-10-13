"""하드웨어 없이 하는 자체 시험 — 가짜 ESP32 · 가짜 Xtion · 복도 지도로 조각마다, 그리고 끝까지.

    python selftest.py            (이 폴더의 .venv 로)
"""

import math
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from depth_view import DepthAnalyzer, candidates, draw_arrows, fit_floor  # noqa: E402
from fake import FakeCamera, FakeWheelSerial  # noqa: E402
from hallway_map import HallwayMap, find_scene  # noqa: E402
from motion import Motion  # noqa: E402
from odometry import Odometry  # noqa: E402
from wheel_link import WheelLink, parse_report  # noqa: E402

import json  # noqa: E402

CFG = json.loads((HERE / "robot.json").read_text(encoding="utf-8"))
fails = []


def check(name, ok, detail=""):
    print(f"  {'PASS' if ok else 'FAIL'}  {name}  {detail}")
    if not ok:
        fails.append(name)


def main():
    hm = HallwayMap(find_scene())
    print("1. 보고 줄 해석 (main.cpp printf 모양)")
    line = ("cmd s 180> 180 t   0>   0 | meas   1.20   1.19 km/h |  1.40  1.35 A | 36.72 V | odo 145 -142 | rx 881 | "
            "m2   1.10   1.12 km/h  1.20  1.30 A 36.70 V rx2 870 | out 300 300")
    r = parse_report(line)
    check("필드", r is not None and r.odo == (145, -142) and r.volt == 36.72 and r.rx == 881 and r.out == (300, 300))

    print("2. 바닥면 맞추기 (가짜 카메라 높이 0.30 m, 3° 숙임)")
    ser = FakeWheelSerial(hm.segs, start=(1.4, 0.0, 0.0), deadband=200)
    cam = FakeCamera(ser)
    f = cam.read()
    g = fit_floor(f.depth, f.K)
    check("높이 · 숙임", g is not None and abs(g.h - 0.30) < 0.01 and abs(g.pitch_deg - 3.0) < 0.5,
          f"→ {g.h:.3f} m, {g.pitch_deg:+.2f}°" if g else "→ 못 찾음")
    an = DepthAnalyzer(f.depth.shape, f.K, g)

    print("3. 복도 입구에서 화살표 (정면이 가장 멀고 양옆은 벽)")
    v = an.analyze(f.depth)
    cs = candidates(v["cols"])
    mid = cs[len(cs) // 2]
    check("정면 트임", mid["free"] >= 3.0 and not mid["blind"], f"정면 {mid['free']} m")
    check("양옆이 더 짧음", cs[0]["free"] < mid["free"] and cs[-1]["free"] < mid["free"], " ".join(f"{c['deg']:+d}°/{c['free']}" for c in cs))
    check("사진에 화살표", draw_arrows(f.bgr, cs, f.K, g).shape == f.bgr.shape)

    print("4. 0.5 m 안쪽 사각 (앞 0.35 m 에 원기둥 — Xtion 은 깊이 0)")
    ser.obstacles = [(1.4 + 0.38 + 0.35 + 0.15, 0.0, 0.15, 1.7)]
    v2 = an.analyze(cam.read().depth)
    cs2 = candidates(v2["cols"])
    mid2 = cs2[len(cs2) // 2]
    check("정면 사각 → 막힘", mid2["blind"] and mid2["free"] <= 0.3 and v2["gap_ahead"] == 0.0,
          f"정면 {mid2['free']} m, 사각 {mid2['blind']}, gap {v2['gap_ahead']}")
    ser.obstacles = [(1.4 + 0.38 + 1.2, 0.0, 0.2, 1.7)]
    v3 = an.analyze(cam.read().depth)
    mid3 = candidates(v3["cols"])[3]
    check("1.2 m 앞 원기둥은 거리로", (not mid3["blind"]) and 0.8 < mid3["free"] < 1.2, f"정면 {mid3['free']} m")
    ser.obstacles = []

    print("5. 돌기 · 가기 (오도메트리 닫힌 루프 — 가짜 장치는 축척 2 % · 윤거 4 % 다르게)")
    link = WheelLink(ser)
    link.wait_ready()
    odo = Odometry(CFG, (ser.x, ser.y, ser.yaw))
    link.arm(CFG["max_cmd"])
    mo = Motion(link, odo, CFG, sense=lambda: an.analyze(cam.read().depth)["gap_ahead"])
    mo._settle(0.3)
    y0 = ser.yaw
    turned, why = mo.turn_by(math.radians(90))
    real = math.degrees(math.atan2(math.sin(ser.yaw - y0), math.cos(ser.yaw - y0)))
    check("왼쪽 90° 돌기", why == "도착" and 75 < real < 100, f"오도메트리 {math.degrees(turned):+.1f}°, 진짜 {real:+.1f}°")
    mo.turn_by(math.radians(-90))
    x0, y0 = ser.x, ser.y
    moved, why = mo.forward(1.0)
    real = math.hypot(ser.x - x0, ser.y - y0)
    check("1 m 가기", why == "도착" and 0.9 < real < 1.1, f"오도메트리 {moved:.3f} m, 진짜 {real:.3f} m")
    print(f"     오도메트리 ({odo.x:.2f}, {odo.y:.2f}, {math.degrees(odo.yaw):+.1f}°) / 진짜 ({ser.x:.2f}, {ser.y:.2f}, {math.degrees(ser.yaw):+.1f}°)")

    print("6. 앞이 막히면 선다 (1.0 m 앞 원기둥, 2 m 가라고 함)")
    ser.obstacles = [(ser.x + 0.38 + 1.0, ser.y, 0.2, 1.7)]
    x0 = ser.x
    moved, why = mo.forward(2.0)
    check("깊이로 정지", why.startswith("깊이") and (ser.x - x0) < 1.0, f"{why}, {ser.x - x0:.2f} m 감")
    ser.obstacles = []

    print("7. 워치독 임대 — 제어 루프가 멈추면 1 s 뒤 ESP32 가 선다")
    n = len(ser.stops)
    link.lease(0.3)
    link.drive(180, 0)
    time.sleep(1.8)
    check("host watchdog", "host watchdog" in ser.stops[n:] and ser.s_t == 0, str(ser.stops[n:]))
    link.close()

    print("8. 지도 그림 · 글")
    goal = hm.find("310앞")
    url, im = hm.draw(goal, {"x": 3.0, "y": 0.0, "yaw": 0.0, "sigma": 0.2, "src": "시험"})
    txt = hm.text(goal, {"x": 3.0, "y": 0.0, "yaw": 0.0, "sigma": 0.2, "src": "시험"})
    check("목표 · 그림", goal["label"] == "310" and url.startswith("data:image/png") and im.size[0] == 1280 and "문 앞 점 (10.84, 0.60)" in txt)

    print("9. 끝까지 — drive.py --fake --model mock --auto (4 걸음)")
    p = subprocess.run([sys.executable, str(HERE / "drive.py"), "--fake", "--model", "mock", "--auto", "--max-steps", "4",
                        "--log-dir", str(HERE / "_selftest_logs")], capture_output=True, text=True, encoding="utf-8", timeout=180)
    tail = "\n".join(p.stdout.strip().splitlines()[-4:])
    check("주행 끝까지", p.returncode == 0 and "가짜 장치의 진짜 위치" in p.stdout, "\n" + tail + ("\n" + p.stderr[-800:] if p.returncode else ""))

    print("\n" + ("모두 통과" if not fails else f"실패 {len(fails)}: {', '.join(fails)}"))
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
