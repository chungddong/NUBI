"""게임패드로 걸음마다 승인 (drive.py --confirm pad) — 이더넷 없이 차를 따라가며 쓴다. 패드 읽기 · 버튼 배치는 pad.py · pad.json.

자율 (걸음마다 묻는 동안):
    L1 + START   실행 (움직이는 동안 L1 을 누르고 있는다 — 떼면 Motion 이 바로 `x` 로 세운다, motion.guard)
    십자 아래     건너뜀
    SELECT       손 운전으로 (아래)
    B            그만
손 운전 (pad.py drive 와 같은 조작):
    START 잡기 · L1 누른 채 스틱 · B 비상정지 (놓기) · 십자 좌우 90° · R1 한 단계 빠르게
    SELECT       자율로 (손 운전을 놓고 바로 다음 걸음)

모드 바꾸기는 SELECT — 이 패드의 MODE · 가운데 버튼은 조이스틱 장치 (event1) 가 아니라 '시스템 제어' 장치로 나와 읽지
않는다 (2026-10-16). pad.py drive 와 같이 띄우지 않는다 (둘 다 ESP32 시리얼 — drive_ctl.sh start 가 pad.py 를 끈다).
"""

import time

from pad import TICK, Teleop, load_binds, open_pad


class PadConfirm:
    def __init__(self):
        cfg = load_binds()
        self.b = cfg["binds"]
        self.mode_b = self.b["select"]
        self.quit_b = self.b.get("estop", {"key": 305})
        self.pad = open_pad(None, cfg.get("name"))
        if self.pad is None:
            raise SystemExit("게임패드가 안 보인다 — 수신기 · 패드 전원 확인 (pad.py show)")
        print(f"  게임패드: {self.pad.name} ({self.pad.path})", flush=True)

    def ask(self):
        """START · 십자 아래 · SELECT · B 중 하나가 새로 눌릴 때까지 → '' (실행) · 's' · 'm' (손 운전) · 'q'."""
        keys = (("start", self.b["start"], ""), ("slower", self.b["slower"], "s"), ("mode", self.mode_b, "m"),
                ("quit", self.quit_b, "q"))
        prev = {k: self.pad.pressed(b) for k, b, _ in keys}
        while True:
            try:
                self.pad.poll(0.1)
            except OSError:
                print("  게임패드 끊김 — 그만", flush=True)
                return "q"
            for k, b, ans in keys:
                now = self.pad.pressed(b)
                if now and not prev[k]:
                    return ans
                prev[k] = now

    def manual(self, link, cfg, odo=None, fusion=None):
        """손 운전 — SELECT 를 누르면 놓고 돌아간다 ('auto'), 패드가 끊기면 'q'. 그동안 오도메트리는 계속 센다."""
        tel = Teleop(link, self.b, cfg, log=lambda m: print("  손 운전:", m, flush=True))
        print("  → 손 운전 (SELECT) — START 잡기 · L1 누른 채 스틱 · B 비상정지 · SELECT = 자율로", flush=True)
        prev, last = self.pad.pressed(self.mode_b), None
        try:
            while True:
                try:
                    self.pad.poll(TICK)
                except OSError:
                    print("  게임패드 끊김", flush=True)
                    return "q"
                tel.update(self.pad, time.monotonic())
                r = link.report
                if odo is not None and r is not None and r is not last:
                    odo.update(r.odo)
                    last = r
                    if fusion is not None:
                        fusion.record(time.monotonic())
                now = self.pad.pressed(self.mode_b)
                if now and not prev:
                    print("  → 자율로 (SELECT)", flush=True)
                    return "auto"
                prev = now
        finally:
            if tel.armed:
                tel.release("자율로 넘김")
            else:
                link.stop()

    def guard(self):
        try:
            self.pad.poll(0)
        except OSError:
            return "게임패드 끊김"
        return None if self.pad.pressed(self.b["deadman"]) else "L1 (데드맨) 을 뗌"

    def close(self):
        self.pad.close()
        time.sleep(0.01)
