"""실차 보정 — 연결 확인 · 카메라 바닥면 · 직진 / 제자리 회전으로 오도메트리 값 재기.

    python3 calibrate.py report                  보고 줄 3 s (움직이지 않음)
    python3 calibrate.py camera                  Xtion 바닥면 · 화살표 → data/logs/vlm-drive/calib_camera.jpg
    python3 calibrate.py straight --runs 3       곧게 2 s → 앞바퀴 두 표시의 이동 거리 (mm) → m_per_step · odo_sign
    python3 calibrate.py spin --runs 2           왼쪽 · 오른쪽 번갈아 오도메트리로 한 바퀴 → 왼쪽 두 바퀴 표시가 처음 선에서 벗어난 거리 (mm)
                                                 → 실제로 돈 각도 → track_eff_m
    (--fake 를 붙이면 가짜 장치로 연습, --write 를 주면 여러 번 잰 평균을 robot.json 에 적는다)

재는 법 (기준점 · 표시 붙이는 자리) 은 README '바닥 보정'. 요점:
- 기준점은 바퀴 축 중심 (허브 바깥 면 한가운데) 바로 아래 바닥. 직각자를 허브 중심에 대고 바닥에 표시한다.
- 직진: 앞 왼쪽 · 앞 오른쪽 바퀴 표시를 출발 전 · 멈춘 뒤 찍어 둘의 이동 거리를 따로 잰다 (오도메트리는 앞축으로 센다).
- 회전: 출발 전 앞 왼쪽 · 뒤 왼쪽 바퀴 표시를 지나는 긴 직선을 바닥에 붙여 두고, 한 바퀴 돈 뒤 같은 두 바퀴 표시가
  그 선에서 수직으로 얼마나 벗어났는지 잰다 (선 바깥 = +, 차 밑 쪽 = −). 각도 = 360° ± asin((앞 − 뒤) / 휠베이스).
"""

import argparse
import json
import math
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]

from drive import stop_on_signals  # noqa: E402
from wheel_link import WheelLink, open_serial  # noqa: E402

WHEELBASE_MM = 420.0      # 앞뒤 바퀴 축 사이 (차체 설계 v29, AXLE_X ±210)
TREAD_MM = 527.4          # 좌우 바퀴 중심 사이 (v29 트레드)


def connect(a):
    if a.fake:
        from fake import FakeWheelSerial
        from hallway_map import HallwayMap, find_scene
        ser = FakeWheelSerial(HallwayMap(find_scene()).segs, deadband=200)
    else:
        ser = open_serial(a.port)
    link = WheelLink(ser)
    r = link.wait_ready(need_board=a.what != "report")
    print(f"바퀴 제어기 {link.port}: 전압 {r.volt:.1f} V · 앞축 rx {r.rx} · 뒤축 rx2 {r.rx_b} · odo {r.odo}")
    return link, ser


def run_for(link, cfg, speed, steer, seconds):
    odo0 = link.report.odo
    link.arm(cfg["max_cmd"])                  # 운전 (drive.py) 과 같은 상한 — t 는 상한에 대한 비율이다
    t0 = time.monotonic()
    try:
        while time.monotonic() - t0 < seconds:
            link.lease()
            link.drive(speed, steer)
            time.sleep(0.1)
    finally:
        link.stop()
    time.sleep(0.8)
    link.lease()
    odo1 = link.report.odo
    return odo1[0] - odo0[0], odo1[1] - odo0[1]


def spin_turn(link, cfg, left=True, turns=1.0, timeout=40.0):
    """지금 robot.json 값 (track_eff_m) 의 오도메트리로 turns 바퀴를 돈다. 실제로는 그 값이 틀린 만큼 더 / 덜 돈다.
    돌려주는 것: 앞축 (마스터, 슬레이브) 스텝 변화."""
    sr, sl, m = cfg["odo_sign"][0], cfg["odo_sign"][1], cfg["m_per_step"]
    need = cfg["track_eff_m"] * 2 * math.pi * turns              # 바깥 − 안쪽 바퀴 거리 (m)
    odo0 = link.report.odo
    link.arm(cfg["max_cmd"])
    t0 = time.monotonic()
    try:
        while True:
            link.lease()
            o = link.report.odo
            dr, dl = (o[0] - odo0[0]) * sr * m, (o[1] - odo0[1]) * sl * m
            done = (dr - dl) if left else (dl - dr)
            if done >= need:
                break
            if time.monotonic() - t0 > timeout:
                print(f"  시간 초과 ({timeout:.0f} s) — 오도메트리로 {math.degrees(done / cfg['track_eff_m']):.0f}° 에서 멈춤")
                break
            rest = math.degrees((need - done) / cfg["track_eff_m"])
            mag = cfg["turn_cmd"] if rest > 30 else cfg["turn_cmd_slow"]
            link.drive(0, -mag if left else mag)                   # t 는 + 가 오른쪽 회전
            time.sleep(0.05)
    finally:
        link.stop()
    time.sleep(0.8)
    link.lease()
    o = link.report.odo
    return o[0] - odo0[0], o[1] - odo0[1]


def ask(prompt):
    try:
        return input(prompt).strip()
    except EOFError:
        return ""


def ask_float(prompt):
    while True:
        v = ask(prompt)
        if not v:
            return None
        try:
            return float(v)
        except ValueError:
            print("  숫자로 (모르면 Enter)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("what", choices=["report", "camera", "straight", "spin"])
    ap.add_argument("--cmd", type=int, default=None, help="straight: 속도 명령 (기본 robot.json fwd_cmd)")
    ap.add_argument("--seconds", type=float, default=None, help="straight: 가는 시간 (기본 2 s)")
    ap.add_argument("--runs", type=int, default=1, help="몇 번 재서 평균낼지")
    ap.add_argument("--right", action="store_true", help="spin: 오른쪽부터 돈다 (기본 왼쪽부터, 번갈아)")
    ap.add_argument("--port")
    ap.add_argument("--fake", action="store_true")
    ap.add_argument("--write", action="store_true", help="잰 값을 robot.json 에 적는다")
    a = ap.parse_args()
    stop_on_signals()
    cfg_path = HERE / "robot.json"
    cfg = json.loads(cfg_path.read_text(encoding="utf-8"))

    if a.what == "camera":
        import cv2
        from depth_view import DepthAnalyzer, candidates, draw_arrows, fit_floor
        if a.fake:
            from fake import FakeCamera, FakeWheelSerial
            from hallway_map import HallwayMap, find_scene
            cam = FakeCamera(FakeWheelSerial(HallwayMap(find_scene()).segs))
        else:
            from xtion import XtionCamera
            cam = XtionCamera()
        try:
            f = cam.read()
            if f is None:
                print("카메라 프레임이 안 온다")
                return
            print(f"깊이 {f.depth.shape[1]}×{f.depth.shape[0]}, 값 있는 픽셀 {(f.depth > 0).mean():.0%}, "
                  f"화각 {math.degrees(2 * math.atan(f.depth.shape[1] / 2 / f.K[0])):.1f}° × {math.degrees(2 * math.atan(f.depth.shape[0] / 2 / f.K[1])):.1f}°")
            g = fit_floor(f.depth, f.K)
            if g is None:
                print("바닥면을 못 찾았다 — 앞 1 ~ 2 m 바닥이 보이게 (카메라가 너무 위를 보거나 바닥이 반짝이면 실패)")
                return
            print(f"바닥면: 카메라 높이 {g.h:.3f} m, 숙임 {g.pitch_deg:+.1f}°, 옆 기울기 {g.roll_deg:+.1f}° (바닥 점 {g.inlier_frac:.0%})")
            v = DepthAnalyzer(f.depth.shape, f.K, g).analyze(f.depth, cfg["blind_frac"])
            cs = candidates(v["cols"])
            print("화살표: " + "  ".join(f"{c['id']}:{c['deg']:+d}°/{c['free']:.1f}m" + ("(사각)" if c["blind"] else "") for c in cs))
            print(f"범퍼 앞 빈 거리 {v['gap_ahead']:.2f} m, 정면 사각 {v['center_blind']}")
            out = REPO / "data/logs/vlm-drive"
            out.mkdir(parents=True, exist_ok=True)
            cv2.imwrite(str(out / "calib_camera.jpg"), draw_arrows(f.bgr, cs, f.K, g))
            print(f"사진: {out / 'calib_camera.jpg'}")
        finally:
            cam.close()
        return

    link, ser = connect(a)
    try:
        if a.what == "report":
            t0 = time.monotonic()
            while time.monotonic() - t0 < 3:
                r = link.report
                print(f"  s {r.speed_now:4d} t {r.steer_now:4d} | {r.kmh[0]:6.2f} {r.kmh[1]:6.2f} km/h | {r.volt:5.2f} V | odo {r.odo} | rx {r.rx} rx2 {r.rx_b}")
                time.sleep(0.5)
            for t, line in list(link.events)[-8:]:
                print("  알림:", line)
            r = link.report
            print("→ " + ("보드 텔레메트리가 들어온다 (rx 증가)" if r and r.rx > 0 else
                          "ESP32 는 응답하지만 메인보드 텔레메트리가 없다 — 보드 전원 (배터리 · 전원 버튼) · 링크 A 배선"))
            return

        if a.what == "straight":
            cmd, sec = a.cmd or cfg["fwd_cmd"], a.seconds or 2.0
            got = []                                                 # (오른쪽 스텝, 왼쪽 스텝, 오른쪽 mm, 왼쪽 mm)
            for k in range(a.runs):
                print(f"\n[{k + 1}/{a.runs}] 앞 왼쪽 · 앞 오른쪽 바퀴 축 아래에 출발 표시를 했나?")
                if ask(f"앞으로 s {cmd} 를 {sec} s — 앞 2.5 m 가 비었나? (y = 출발) > ").lower() != "y":
                    break
                x0, y0 = getattr(ser, "x", 0.0), getattr(ser, "y", 0.0)
                dm, ds = run_for(link, cfg, cmd, 0, sec)
                print(f"  홀 스텝: 마스터 (오른쪽) {dm:+d}, 슬레이브 (왼쪽) {ds:+d}")
                if dm == 0 or ds == 0:
                    print("  한쪽이 0 — 바퀴가 안 돌았거나 (듀티 부족 · 보드) 텔레메트리 문제. --cmd 를 올리거나 배선 확인")
                    break
                sign = [1 if dm > 0 else -1, 1 if ds > 0 else -1]
                if sign != cfg["odo_sign"]:
                    print(f"  ! odo_sign 이 robot.json ({cfg['odo_sign']}) 과 다르다 → {sign} (바퀴가 뒤로 갔거나 보드 방향이 바뀜)")
                cfg["odo_sign"] = sign
                L = ask_float("  앞 왼쪽 바퀴 표시 사이 거리 (mm) > ")
                R = ask_float("  앞 오른쪽 바퀴 표시 사이 거리 (mm) > ")
                if a.fake and L is None and R is None:
                    L = R = 1000 * math.hypot(ser.x - x0, ser.y - y0)
                    print(f"  (가짜 장치의 진짜 거리 {L:.0f} mm 를 쓴다)")
                if L is None or R is None:
                    print("  거리를 안 넣어 이번 것은 뺀다")
                    continue
                got.append((abs(dm), abs(ds), R, L))
                print(f"  → 스텝당 오른쪽 {R / abs(dm):.3f} mm · 왼쪽 {L / abs(ds):.3f} mm, "
                      f"가는 동안 방향이 {math.degrees((R - L) / TREAD_MM):+.1f}° 틀어짐 (+ = 왼쪽)")
            if got:
                g = list(zip(*got))
                m = (sum(g[2]) + sum(g[3])) / 1000 / (sum(g[0]) + sum(g[1]))
                each = [(r + l) / 1000 / (sr + sl) for sr, sl, r, l in got]
                print(f"\n→ m_per_step = {m:.6f}  ({len(got)} 번, 한 번씩 {', '.join(f'{v:.6f}' for v in each)}; "
                      f"지금 robot.json {cfg['m_per_step']})")
                print(f"  바퀴별: 오른쪽 {sum(g[2]) / sum(g[0]) / 1000:.6f}, 왼쪽 {sum(g[3]) / sum(g[1]) / 1000:.6f} "
                      f"(이론값: 바퀴 지름 158.7 mm · 한 바퀴 90 스텝이면 {math.pi * 0.1587 / 90:.6f})")
                cfg.update(m_per_step=round(m, 6))
            else:
                a.write = False
        else:
            got = []                                                 # (바깥 − 안쪽 바퀴 거리 m, 실제 각도 rad, 왼쪽?)
            for k in range(a.runs):
                left = (k % 2 == 0) != a.right                       # 왼쪽 · 오른쪽 번갈아 — 평균이 한쪽으로 치우치지 않게
                side = "왼쪽" if left else "오른쪽"
                print(f"\n[{k + 1}/{a.runs}] 앞 왼쪽 · 뒤 왼쪽 바퀴 축 아래를 지나는 긴 직선을 바닥에 붙였나?")
                if ask(f"{side} 제자리 한 바퀴 (오도메트리 기준) — 둘레 1 m 가 비었나? (y = 출발) > ").lower() != "y":
                    break
                y0 = getattr(ser, "yaw_total", 0.0)
                dm, ds = spin_turn(link, cfg, left)
                dr, dl = dm * cfg["odo_sign"][0] * cfg["m_per_step"], ds * cfg["odo_sign"][1] * cfg["m_per_step"]
                diff = (dr - dl) if left else (dl - dr)
                print(f"  바퀴 거리: 오른쪽 {dr:+.3f} m, 왼쪽 {dl:+.3f} m → 오도메트리 각도 "
                      f"{math.degrees(diff / cfg['track_eff_m']):.1f}° (지금 track_eff_m {cfg['track_eff_m']})")
                fo = ask_float("  앞 왼쪽 바퀴 표시가 선에서 벗어난 거리 (mm, 선 바깥 + · 차 밑 쪽 −, 모르면 Enter) > ")
                ro = ask_float("  뒤 왼쪽 바퀴 표시가 선에서 벗어난 거리 (mm, 같은 부호) > ") if fo is not None else None
                if fo is not None and ro is not None:
                    eps = math.degrees(math.asin(max(-1.0, min(1.0, (fo - ro) / WHEELBASE_MM))))
                    deg = 360.0 + (eps if left else -eps)
                    print(f"  → 한 바퀴에서 {deg - 360:+.1f}° → 실제 {deg:.1f}°")
                else:
                    deg = ask_float("  실제로 돈 각도 (도, 한 바퀴면 360 근처, 모르면 Enter) > ")
                    if a.fake and deg is None and hasattr(ser, "yaw_total"):
                        deg = abs(math.degrees(ser.yaw_total - y0))
                        print(f"  (가짜 장치의 진짜 각도 {deg:.1f}° 를 쓴다)")
                if not deg:
                    print("  각도를 안 넣어 이번 것은 뺀다")
                    continue
                got.append((diff, math.radians(deg), left))
                print(f"  → 이번 track_eff_m = {diff / math.radians(deg):.3f}")
            if got:
                tr = sum(g[0] for g in got) / sum(g[1] for g in got)
                per = {n: [g for g in got if g[2] == lf] for n, lf in (("왼쪽", True), ("오른쪽", False))}
                each = ", ".join(f"{n} {sum(g[0] for g in v) / sum(g[1] for g in v):.3f} ({len(v)} 번)" for n, v in per.items() if v)
                print(f"\n→ track_eff_m = {tr:.3f}  ({each}; 실제 트레드 {TREAD_MM / 1000:.3f} m, "
                      f"지금 robot.json {cfg['track_eff_m']})")
                cfg.update(track_eff_m=round(tr, 3))
            else:
                a.write = False
        if a.write:
            cfg_path.write_text(json.dumps(cfg, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
            print(f"robot.json 에 적었다")
    finally:
        link.close()


if __name__ == "__main__":
    sys.exit(main())
