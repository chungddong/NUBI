"""누비 실차 운전 — Xtion 사진 + 깊이 화살표 + 지도 · 오도메트리 위치를 GPT · Claude 에 보내 한 걸음씩 몬다.

    python3 drive.py --goal 310앞 --model gpt-6-luna            Jetson + ESP32 + Xtion, 걸음마다 Enter 로 확인
    python3 drive.py --goal 310앞 --model mock --fake --auto      PC 에서 가짜 장치로 (API 없이)

모델에 보내는 것과 받는 것은 시뮬레이터 ⑤ 와 같다 (hardware/vehicle/models/vehicle/sim/agent.py · agent_claude.py 를
그대로 부른다 — 키는 그 폴더의 agent.local.json 또는 OPENAI_API_KEY · ANTHROPIC_API_KEY).
기록: data/logs/vlm-drive/<시각>/ — 걸음마다 보낸 사진 · 지도, steps.jsonl (git 에 안 올라간다 — 사진에 사람이 찍힌다).
"""

import argparse
import datetime as dt
import json
import math
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
sys.path.insert(0, str(REPO / "hardware/vehicle/models/vehicle/sim"))

from depth_view import DepthAnalyzer, candidates, draw_arrows, fit_floor, ground_from, jpeg_data_url  # noqa: E402
from hallway_map import HallwayMap, find_scene  # noqa: E402
from motion import Motion  # noqa: E402
from odometry import Odometry  # noqa: E402
from wheel_link import WheelLink, open_serial  # noqa: E402

D2R = math.pi / 180


def stop_on_signals():
    """강제 종료 (SIGTERM) · SSH 끊김 (SIGHUP) 도 Ctrl+C 처럼 — finally 에서 바퀴를 세우고 카메라를 닫는다.
    카메라를 닫지 않고 죽으면 Xtion 이 걸린 상태가 되어 USB 를 다시 꽂아야 한다 (2026-10-14)."""
    import signal

    def _raise(*_):
        raise KeyboardInterrupt
    for s in ("SIGTERM", "SIGHUP"):
        if hasattr(signal, s) and signal.getsignal(getattr(signal, s)) is not signal.SIG_IGN:   # nohup 으로 띄우면 SIGHUP 은 그대로 무시
            signal.signal(getattr(signal, s), _raise)


def load_driver(model):
    if model.startswith("claude-"):
        import agent_claude as d
    else:
        import agent as d
    return d


def setup_ground(cam, cfg):
    f = cam.read()
    if f is None:
        raise SystemExit("카메라 프레임이 안 온다")
    g = fit_floor(f.depth, f.K)
    if g is None:
        if cfg.get("camera_height_m") is None:
            raise SystemExit("바닥면을 못 찾았다 — 차 앞 1 ~ 2 m 바닥이 비어 보이게 두고 다시, 또는 robot.json 에 camera_height_m")
        g = ground_from(cfg["camera_height_m"], cfg.get("camera_pitch_deg") or 0.0)
        print(f"  바닥면을 못 찾아 설정값을 쓴다: 높이 {g.h:.3f} m, 숙임 {g.pitch_deg:.1f}°")
    else:
        print(f"  바닥면: 카메라 높이 {g.h:.3f} m, 숙임 {g.pitch_deg:+.1f}°, 옆 기울기 {g.roll_deg:+.1f}° (바닥 점 {g.inlier_frac:.0%})")
    return f, g


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--goal", default="310앞", help="목표 문: 310앞 · 309 · 305뒤 · 학과 사무실 …")
    ap.add_argument("--model", default="mock", help="gpt-6-luna · gpt-6.1-sol · gpt-6-astra · claude-haiku-5-5 · … · mock")
    ap.add_argument("--effort", default="medium", choices=["low", "medium", "high"])
    ap.add_argument("--pose", default="odom", choices=["vis", "odom", "map", "none"],
                    help="vis = 지도 + 오도메트리 + 웹캠 위치 인식, odom = 지도 + 오도메트리, map = 지도만, none = 사진만")
    ap.add_argument("--start", default="1.4,0,0", help="출발 위치 x,y,방향° (지도 좌표, 기본 = 복도 입구에서 안쪽을 봄)")
    ap.add_argument("--max-steps", type=int, default=40)
    ap.add_argument("--auto", action="store_true", help="걸음마다 묻지 않는다 (처음에는 쓰지 말 것)")
    ap.add_argument("--confirm", default="enter", choices=["enter", "pad"],
                    help="걸음마다 승인: enter = 터미널, pad = 게임패드 (START 실행 · 십자 아래 건너뜀 · SELECT 그만, "
                         "움직이는 동안 L1 을 누르고 있어야 함 — 이더넷 없이 따라가며)")
    ap.add_argument("--no-directory", action="store_true", help="층 안내 (방 순서) 를 주지 않는다")
    ap.add_argument("--fake", action="store_true", help="가짜 ESP32 · 가짜 카메라 (PC 시험)")
    ap.add_argument("--sim", action="store_true", help="가짜 ESP32 + 시뮬레이터 3D 지도 카메라 (페이지 ?env=hallway&cam=xtion&bridge=1)")
    ap.add_argument("--sim-url", default="http://127.0.0.1:8748")
    ap.add_argument("--port", help="ESP32 포트 (기본: CH340 자동)")
    ap.add_argument("--scene", help="복도 scene.json 경로")
    ap.add_argument("--config", default=str(HERE / "robot.json"))
    ap.add_argument("--log-dir", default=str(REPO / "data/logs/vlm-drive"))
    a = ap.parse_args()
    stop_on_signals()

    cfg = json.loads(Path(a.config).read_text(encoding="utf-8"))
    hm = HallwayMap(find_scene(a.scene))
    goal = hm.find(a.goal)
    sx, sy, syaw = (float(v) for v in a.start.split(","))
    start = (sx, sy, syaw * D2R)
    driver = load_driver(a.model)

    virtual = a.fake or a.sim
    bridge = None
    if a.sim:
        from fake import FakeWheelSerial
        from sim_bridge import SimBridge, SimCamera
        ser = FakeWheelSerial(hm.segs, start=start, deadband=200)
        bridge = SimBridge(ser, a.sim_url)
        cam = SimCamera(bridge, ser, cfg)
    elif a.fake:
        from fake import FakeCamera, FakeWheelSerial
        ser = FakeWheelSerial(hm.segs, start=start, deadband=200)
        cam = FakeCamera(ser)
    else:
        from xtion import XtionCamera
        ser = open_serial(a.port)
        cam = XtionCamera()
    link = WheelLink(ser)
    run_id = dt.datetime.now().strftime("%Y-%m-%d_%H%M%S")
    out = Path(a.log_dir) / run_id
    out.mkdir(parents=True, exist_ok=True)
    print(f"목표 {goal['name']} · 모델 {a.model} ({a.effort}) · 위치 {a.pose} · 기록 {out}")

    result = None
    vis = fusion = None
    try:
        r = link.wait_ready()
        print(f"  바퀴 제어기 ({link.port}): 전압 {r.volt:.1f} V, 앞축 rx {r.rx}")
        if 0 < r.volt < 33.0:
            print("  !! 배터리 전압이 낮다 (10S 기준 33 V 밑)")
        f, g = setup_ground(cam, cfg)
        an = DepthAnalyzer(f.depth.shape, f.K, g)
        hf = math.degrees(2 * math.atan(f.depth.shape[1] / 2 / f.K[0]))
        vf = math.degrees(2 * math.atan(f.depth.shape[0] / 2 / f.K[1]))
        cam_text = (f"ASUS Xtion Pro Live, 렌즈 높이 {g.h * 100:.0f} cm (숙임 {g.pitch_deg:+.0f}°), 수평 시야 {hf:.0f}°, 세로 시야 {vf:.0f}°, "
                    "깊이 0.5 ~ 4 m (0.5 m 안쪽은 깊이가 없어 '사각' 으로 친다)")
        odo = Odometry(cfg, start)
        if a.pose == "vis":
            from pose_fusion import PoseFusion
            cam_off = (cfg.get("webcam_x_m", 0.35), cfg.get("webcam_y_m", 0.0), cfg.get("webcam_yaw_deg", 0.0))
            fusion = PoseFusion(odo, cam_off=cam_off)
            if virtual:
                from sim_bridge import SimVision
                vis = SimVision(ser, cam_off)
            else:
                from vision_fix import RealVision
                vis = RealVision(cfg)
        link.arm(cfg["max_cmd"])

        def sense():
            f = cam.read(timeout=0.5)
            return 0.0 if f is None else an.analyze(f.depth, cfg["blind_frac"])["gap_ahead"]   # 깊이가 안 오면 막힘

        motion = Motion(link, odo, cfg, sense)
        padc = None
        if a.confirm == "pad" and not a.auto:
            from pad_confirm import PadConfirm
            padc = PadConfirm()
            motion.guard = padc.guard
        motion._settle(0.4)                                   # 오도메트리 기준점

        history, memory = [], ""
        for step in range(1, a.max_steps + 1):
            f = cam.read()
            if f is None:
                print("카메라 프레임이 안 온다 — 멈춤")
                break
            view = an.analyze(f.depth, cfg["blind_frac"])
            cands = candidates(view["cols"])
            img = draw_arrows(f.bgr, cands, f.K, g)
            if vis is not None:                               # 멈춰 있을 때 웹캠으로 위치를 찾아 오도메트리에 섞는다
                fusion.record(time.monotonic())
                prior = fusion.cam_from_base(odo.x, odo.y, odo.yaw) if fusion.n_fix and fusion.rejects < fusion.lost_after else None
                fx = vis.locate(prior)
                if fx is None:
                    print("  위치 인식: 못 찾음 — 오도메트리로")
                else:
                    ok, why, inn = fusion.fix(fx["t"], fx["x"], fx["y"], fx["yaw"])
                    print(f"  위치 인식: {why} (인라이어 {fx['inliers']}, 차이 {math.hypot(inn[0], inn[1]):.2f} m {inn[2]:+.0f}°, {fx['ms']:.0f} ms)")
            pose = odo.pose() if a.pose in ("odom", "vis") else None
            map_url = map_text = map_im = None
            if a.pose != "none":
                if pose:
                    for c in cands:
                        c["map_deg"] = round(math.degrees(math.atan2(math.sin(pose["yaw"] + c["deg"] * D2R), math.cos(pose["yaw"] + c["deg"] * D2R))))
                map_url, map_im = hm.draw(goal, pose)
                map_text = hm.text(goal, pose)
            import cv2
            cv2.imwrite(str(out / f"{step:02d}_cam.jpg"), img)
            if map_im:
                map_im.save(out / f"{step:02d}_map.png")
            req = {"run": run_id, "step": step, "max_steps": a.max_steps, "model": a.model, "effort": a.effort,
                   "goal": goal["name"], "camera": cam_text, "directory": "" if a.no_directory else hm.directory(), "candidates": cands,
                   "history": history[-8:], "memory": memory, "image": jpeg_data_url(img), "map_image": map_url,
                   "map_text": map_text, "truth": {"x": ser.x, "y": ser.y, "yaw": ser.yaw} if virtual else None}
            print(f"\n#{step}  화살표 " + "  ".join(f"{c['id']}:{c['deg']:+d}°/{c['free']:.1f}m" + ("(사각)" if c["blind"] else "") for c in cands)
                  + (f"  | 추정 ({pose['x']:.2f}, {pose['y']:.2f}) {math.degrees(pose['yaw']):+.0f}° ±{pose['sigma']:.2f}" if pose else ""))
            t0 = time.monotonic()
            res = driver.step(req)
            if not res.get("ok"):
                print("  오류:", res.get("error"))
                break
            d = res["decision"]
            print(f"  ({time.monotonic() - t0:.1f} s, ${res['usage']['usd']:.4f}) 보임: {d['observation']}\n  까닭: {d['reason']}\n  메모: {d['memory']}")
            what = (f"가기 {d['arrow']}번 {d['distance_m']:.1f} m" if d["action"] == "move" else "멈춤" if d["action"] == "stop"
                    else f"{'왼쪽' if d['action'] == 'turn_left' else '오른쪽'} {d['turn_deg']:.0f}°")
            print(f"  → {what}")
            if bridge is not None:
                est = f"추정 ({pose['x']:.2f}, {pose['y']:.2f}) {math.degrees(pose['yaw']):+.0f}°" if pose else "추정 없음"
                bridge.note = (f"#{step} {a.model}: {what}\n{est} · 진짜 ({ser.x:.2f}, {ser.y:.2f}) {math.degrees(ser.yaw):+.0f}°"
                               f" · 목표 {goal['name']}")
            if not a.auto and d["action"] != "stop":
                if padc is not None:
                    print("  게임패드: L1 을 누른 채 START = 실행 · 십자 아래 = 건너뜀 · SELECT = 그만", flush=True)
                    ans = padc.ask()
                else:
                    ans = input("  Enter = 실행 · s = 건너뜀 · q = 그만 > ").strip().lower()
                if ans == "q":
                    break
                if ans == "s":
                    history.append(f"걸음 {step}: {what} → 사람이 건너뜀")
                    continue
            outcome = execute(d, cands, motion, cfg)
            print(f"  결과: {outcome}")
            history.append(f"걸음 {step}: {what} → {outcome}")
            memory = d["memory"]
            rec = {k: v for k, v in req.items() if k not in ("image", "map_image")}
            rec.update(decision=d, usage=res.get("usage"), latency_ms=res.get("latency_ms"), outcome=outcome, pose_after=odo.pose(),
                       view={k: view[k] for k in ("gap_ahead", "center_blind", "valid_frac")},
                       truth_after={"x": ser.x, "y": ser.y, "yaw": ser.yaw} if virtual else None)
            with open(out / "steps.jsonl", "a", encoding="utf-8") as fp:
                fp.write(json.dumps(rec, ensure_ascii=False) + "\n")
            if d["action"] == "stop":
                result = "모델이 멈춤"
                break
    except KeyboardInterrupt:
        print("\n멈춤 (Ctrl+C)")
    finally:
        link.close()
        cam.close()
        if vis is not None and hasattr(vis, "close"):
            vis.close()
        if "padc" in locals() and padc is not None:
            padc.close()

    p = odo.pose() if "odo" in locals() else None
    if p:
        e = math.hypot(p["x"] - goal["front"][0], p["y"] - goal["front"][1])
        print(f"\n끝 ({result or '중단'}) — 오도메트리 위치 ({p['x']:.2f}, {p['y']:.2f}), 목표 문 앞까지 (추정) {e:.2f} m")
    if virtual:
        e = math.hypot(ser.x - goal["front"][0], ser.y - goal["front"][1])
        print(f"  가짜 장치의 진짜 위치 ({ser.x:.2f}, {ser.y:.2f}) — 목표 문 앞까지 {e:.2f} m → {'성공' if e <= 1.0 else '실패'}")
        print(f"  펌웨어 STOP 기록: {ser.stops[-5:]}")


def execute(d, cands, motion, cfg):
    try:
        if d["action"] == "move":
            c = next((k for k in cands if k["id"] == d["arrow"]), cands[len(cands) // 2])
            turned, why1 = motion.turn_by(c["deg"] * D2R)
            dist = max(0.0, min(d["distance_m"], cfg["max_move_m"], c["free"] - cfg["margin_m"]))
            if dist < 0.05:
                return f"화살표 {c['id']} ({c['deg']:+d}°) 로 {math.degrees(turned):+.0f}° 돌았지만 빈 거리가 모자라 안 감"
            moved, why2 = motion.forward(dist)
            return (f"화살표 {c['id']} ({c['deg']:+d}°) 로 돌아 {math.degrees(turned):+.0f}°, {moved:.2f} m 감"
                    + (f" (요청 {d['distance_m']:.1f} m 를 줄임)" if dist < d["distance_m"] - 0.05 else "")
                    + ("" if why2 == "도착" else f" — {why2}") + ("" if why1 == "도착" else f" (돌기: {why1})"))
        if d["action"] in ("turn_left", "turn_right"):
            deg = min(max(abs(d["turn_deg"]), 5), 180) * (1 if d["action"] == "turn_left" else -1)
            turned, why = motion.turn_by(deg * D2R)
            return f"제자리 {'왼쪽' if turned > 0 else '오른쪽'} {abs(math.degrees(turned)):.0f}°" + ("" if why == "도착" else f" — {why}")
        return "멈춤"
    except RuntimeError as e:
        motion.link.stop()
        return f"중단 — {e}"


if __name__ == "__main__":
    main()
