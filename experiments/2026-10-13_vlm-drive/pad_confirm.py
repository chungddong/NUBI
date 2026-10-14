"""게임패드로 걸음마다 승인 (drive.py --confirm pad) — 이더넷 없이 차를 따라가며 쓴다. 패드 읽기 · 버튼 배치는 pad.py · pad.json.

    START        실행 (그동안 L1 을 누르고 있어야 한다)
    십자 아래     건너뜀
    SELECT       그만
    L1 (데드맨)   움직이는 동안 누르고 있는다 — 떼면 Motion 이 바로 `x` 로 세운다 (motion.guard)

pad.py drive (손 운전) 와 같이 띄우지 않는다 — 둘 다 ESP32 시리얼을 쓴다.
"""

import time

from pad import load_binds, open_pad


class PadConfirm:
    def __init__(self):
        cfg = load_binds()
        self.b = cfg["binds"]
        self.pad = open_pad(None, cfg.get("name"))
        if self.pad is None:
            raise SystemExit("게임패드가 안 보인다 — 수신기 · 패드 전원 확인 (pad.py show)")
        print(f"  게임패드: {self.pad.name} ({self.pad.path})", flush=True)

    def ask(self):
        """START · 십자 아래 · SELECT 중 하나가 새로 눌릴 때까지 기다린다 → '' (실행) · 's' · 'q'."""
        keys = (("start", ""), ("slower", "s"), ("select", "q"))
        prev = {k: self.pad.pressed(self.b[k]) for k, _ in keys}
        while True:
            try:
                self.pad.poll(0.1)
            except OSError:
                print("  게임패드 끊김 — 그만", flush=True)
                return "q"
            for k, ans in keys:
                now = self.pad.pressed(self.b[k])
                if now and not prev[k]:
                    return ans
                prev[k] = now

    def guard(self):
        try:
            self.pad.poll(0)
        except OSError:
            return "게임패드 끊김"
        return None if self.pad.pressed(self.b["deadman"]) else "L1 (데드맨) 을 뗌"

    def close(self):
        self.pad.close()
        time.sleep(0.01)
