"""기본 동작 — 제자리 돌기 · 곧게 가기. 오도메트리로 닫힌 루프, 깊이로 정지.

ESP32 명령은 듀티 (속도가 아님) 라 일정 값을 주고 오도메트리로 목표에 닿으면 `x` (램프 없이 즉시 정지) 한다.
`t` 는 + 가 오른쪽 회전이므로 왼쪽 (+ 각도) 으로 돌 때는 음수를 보낸다. 가는 동안 방향이 틀어지면 작은 조향으로 바로잡는다.
멈추는 조건: 목표 도달 · 깊이로 범퍼 앞이 막힘 (또는 가까운 사각) · 보고 줄 끊김 · 시간 초과 · 움직임 없음.
"""

import math
import time

wrap = lambda a: math.atan2(math.sin(a), math.cos(a))


class Motion:
    def __init__(self, link, odo, cfg, sense=None, log=print):
        self.link, self.odo, self.cfg = link, odo, cfg
        self.sense = sense                    # () -> 범퍼 앞 빈 거리 (m), 새 깊이 사진으로
        self.log = log

    def _tick(self):
        """보고 줄 하나를 기다려 오도메트리를 갱신. 끊겼으면 예외."""
        r0 = self.link.report
        t0 = time.monotonic()
        while time.monotonic() - t0 < 0.6:
            self.link.lease()
            r = self.link.report
            if r is not None and r is not r0:
                self.odo.update(r.odo)
                return r
            time.sleep(0.02)
        raise RuntimeError("바퀴 제어기 보고 줄이 0.6 s 넘게 안 온다")

    def turn_by(self, rad):
        c = self.cfg
        target = self.odo.yaw + rad
        t_end = time.monotonic() + 2.0 + abs(rad) / math.radians(c["turn_min_dps"])
        why = "도착"
        try:
            while True:
                self._tick()
                err = wrap(target - self.odo.yaw)
                if abs(err) < math.radians(c["turn_tol_deg"]):
                    break
                if time.monotonic() > t_end:
                    why = "시간 초과"
                    break
                mag = c["turn_cmd"] if abs(err) > math.radians(20) else c["turn_cmd_slow"]
                self.link.drive(0, -mag if err > 0 else mag)      # + 조향 = 오른쪽
        finally:
            self.link.stop()
        self._settle()
        return wrap(self.odo.yaw - (target - rad)), why

    def forward(self, dist):
        c = self.cfg
        x0, y0, yaw0 = self.odo.x, self.odo.y, self.odo.yaw
        t_end = time.monotonic() + 3.0 + dist / c["min_speed_mps"]
        why, last_d, still = "도착", 0.0, 0
        try:
            while True:
                self._tick()
                done = math.hypot(self.odo.x - x0, self.odo.y - y0)
                if dist - done < 0.03:
                    break
                if self.sense is not None and self.sense() < c["stop_gap_m"]:
                    why = "깊이로 앞이 막혀 섬"
                    break
                if time.monotonic() > t_end:
                    why = "시간 초과"
                    break
                still = still + 1 if done - last_d < 0.002 else 0
                last_d = done
                if still > 10:                                     # 2 s 동안 안 움직임 (턱 · 걸림 · 듀티 부족)
                    why = "움직이지 않음"
                    break
                steer = max(-c["steer_fix_max"], min(c["steer_fix_max"], c["steer_fix_k"] * wrap(self.odo.yaw - yaw0)))
                speed = c["fwd_cmd"] if dist - done > 0.3 else c["fwd_cmd_slow"]
                self.link.drive(speed, steer)                     # 왼쪽으로 틀어졌으면 (+) 오른쪽 조향 (+)
        finally:
            self.link.stop()
        self._settle()
        return math.hypot(self.odo.x - x0, self.odo.y - y0), why

    def _settle(self, seconds=0.5):
        t0 = time.monotonic()
        while time.monotonic() - t0 < seconds:
            try:
                self._tick()
            except RuntimeError:
                break
