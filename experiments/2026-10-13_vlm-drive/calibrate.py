"""실차 보정 — 연결 확인 · 카메라 바닥면 · 직진 / 제자리 회전으로 오도메트리 값 재기.

    python3 calibrate.py report                       보고 줄 3 s (움직이지 않음)
    python3 calibrate.py camera                       Xtion 바닥면 · 화살표 → data/logs/vlm-drive/calib_camera.jpg
    python3 calibrate.py straight --cmd 180 --seconds 2   곧게 가고 홀 스텝을 센다 → 줄자로 잰 거리를 넣으면 m_per_step · odo_sign
    python3 calibrate.py spin --cmd 500 --seconds 3       왼쪽 제자리 회전 → 잰 각도를 넣으면 track_eff_m
    (--fake 를 붙이면 가짜 장치로 연습)

움직이기 전에 묻는다. 처음에는 바퀴를 띄우고 방향만 보고, 그다음 바닥에서 앞 2 m 를 비우고 잰다.
결과는 화면에 robot.json 에 적을 값으로 보여 준다 (--write 를 주면 robot.json 을 고친다).
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


def run_for(link, speed, steer, seconds):
    odo0 = link.report.odo
    link.arm(400)
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


def ask(prompt):
    try:
        return input(prompt).strip()
    except EOFError:
        return ""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("what", choices=["report", "camera", "straight", "spin"])
    ap.add_argument("--cmd", type=int, default=None)
    ap.add_argument("--seconds", type=float, default=None)
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
            if ask(f"앞으로 s {cmd} 를 {sec} s — 앞 2 m 가 비었나? (y = 출발) > ").lower() != "y":
                return
            dm, ds = run_for(link, cmd, 0, sec)
            print(f"홀 스텝: 마스터 (오른쪽) {dm:+d}, 슬레이브 (왼쪽) {ds:+d}")
            if dm == 0 or ds == 0:
                print("한쪽이 0 — 바퀴가 안 돌았거나 (듀티 부족 · 보드) 텔레메트리 문제. fwd_cmd 를 올리거나 배선 확인")
                return
            sign = [1 if dm > 0 else -1, 1 if ds > 0 else -1]
            meas = ask("줄자로 잰 이동 거리 (m, 모르면 Enter) > ")
            if a.fake and not meas:
                meas = f"{math.hypot(ser.x - 1.4, ser.y):.3f}"
                print(f"  (가짜 장치의 진짜 거리 {meas} m 를 쓴다)")
            print(f"→ odo_sign = {sign}")
            if meas:
                m = float(meas) / ((abs(dm) + abs(ds)) / 2)
                print(f"→ m_per_step = {m:.6f}  (지금 robot.json: {cfg['m_per_step']})")
                cfg.update(odo_sign=sign, m_per_step=round(m, 6))
        else:
            cmd, sec = a.cmd or cfg["turn_cmd"], a.seconds or 3.0
            if ask(f"왼쪽 제자리 회전 t -{cmd} 를 {sec} s — 둘레가 비었나? (y = 출발) > ").lower() != "y":
                return
            y0 = getattr(ser, "yaw", 0.0)
            dm, ds = run_for(link, 0, -cmd, sec)
            dr, dl = dm * cfg["odo_sign"][0] * cfg["m_per_step"], ds * cfg["odo_sign"][1] * cfg["m_per_step"]
            print(f"바퀴 거리: 오른쪽 {dr:+.3f} m, 왼쪽 {dl:+.3f} m (왼쪽 회전이면 오른쪽 +, 왼쪽 −)")
            meas = ask("실제로 돈 각도 (도, 왼쪽 +, 모르면 Enter) > ")
            if a.fake and not meas:
                meas = f"{math.degrees(ser.yaw - y0):.1f}"
                print(f"  (가짜 장치의 진짜 각도 {meas}° 를 쓴다)")
            if meas and float(meas) != 0:
                tr = (dr - dl) / math.radians(float(meas))
                print(f"→ track_eff_m = {tr:.3f}  (실제 윤거 0.527 m, 지금 robot.json: {cfg['track_eff_m']})")
                cfg.update(track_eff_m=round(tr, 3))
        if a.write:
            cfg_path.write_text(json.dumps(cfg, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
            print(f"robot.json 에 적었다")
    finally:
        link.close()


if __name__ == "__main__":
    sys.exit(main())
