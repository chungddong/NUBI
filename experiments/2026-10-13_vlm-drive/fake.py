"""하드웨어 없이 시험하는 가짜 장치 — PC 에서 drive.py --fake · selftest.py 가 쓴다.

FakeWheelSerial: ESP32 펌웨어 (firmware/wheel-controller/src/main.cpp) 를 흉내 낸 시리얼. 명령 해석 (|값| < 50 → 0,
클램프), 램프 (속도 100 ms 당 20 · 조향 100), 좌우 믹싱 (+ 조향 = 오른쪽 → 왼쪽 바퀴가 더), 데드밴드, 호스트 워치독 1 s,
200 ms 마다 보고 줄. 바퀴는 듀티 20 % 밑에서는 안 돈다. 스키드 스티어 몸체 회전은 유효 윤거 TRACK_TRUE 로,
홀 스텝은 STEP_TRUE 로 센다 — robot.json 의 기본값과 일부러 조금 다르게 해서 오도메트리 오차가 생기게 했다.
FakeCamera: 복도 벽 선분 (높이 2.68 m) · 바닥 · 천장 · 원기둥 장애물에 광선을 쏘아 Xtion 같은 깊이 (0.5 ~ 4 m 밖은 0) 를 만든다.
"""

import math
import threading
import time

import numpy as np

from depth_view import ground_from
from xtion import Frame, intrinsics

STEP_TRUE = 0.00576 * 1.02          # 실제 스텝당 거리 (설정값보다 2 % 큼)
TRACK_TRUE = 0.82                   # 실제 유효 윤거 (설정값 0.79 와 다름)
VMAX = 1.6                          # 듀티 1000 일 때 바퀴 속도 m/s [가정]
STALL = 200                         # 이 듀티 밑에서는 안 돈다 (약 20 %)
CEIL = 2.68
RADIUS = 0.36                       # 충돌 판정용 차 반지름


def _seg_dist(px, py, ax, ay, bx, by):
    vx, vy = bx - ax, by - ay
    t = max(0.0, min(1.0, ((px - ax) * vx + (py - ay) * vy) / max(vx * vx + vy * vy, 1e-12)))
    return math.hypot(px - ax - t * vx, py - ay - t * vy)


class FakeWheelSerial:
    def __init__(self, segs, start=(1.4, 0.0, 0.0), deadband=0, obstacles=()):
        self.port = "FAKE"
        self.segs, self.obstacles = segs, list(obstacles)
        self.x, self.y, self.yaw = start
        self.timeout = 0.1
        self._in, self._out = b"", bytearray()
        self._lock = threading.Lock()
        self.s_t = self.t_t = self.s_n = self.t_n = 0
        self.maxc, self.deadband = 400, deadband
        self.watchdog, self.host_at, self.host_lost = False, time.monotonic(), False
        self.odo_m = self.odo_s = 0.0              # 마스터 (오른쪽) · 슬레이브 (왼쪽) 스텝
        self.rx, self.out = 0, (0, 0)
        self.stops = []                            # 펌웨어가 STOP 을 찍은 까닭
        self._alive = True
        self._print("Gen2.x hoverboard console (FAKE)")
        threading.Thread(target=self._loop, daemon=True).start()

    # ---- pyserial 흉내
    def write(self, data):
        with self._lock:
            self._in += data
            while b"\n" in self._in:
                line, self._in = self._in.split(b"\n", 1)
                self.host_at = time.monotonic()
                self._handle(line.decode().strip())
        return len(data)

    def read(self, n=1):
        t0 = time.monotonic()
        while time.monotonic() - t0 < self.timeout:
            with self._lock:
                if self._out:
                    b = bytes(self._out[:n])
                    del self._out[:n]
                    return b
            time.sleep(0.005)
        return b""

    def close(self):
        self._alive = False

    # ---- 펌웨어 흉내
    def _print(self, s):                           # 만들 때만 (스레드 전) — 그 뒤는 잠금 안에서 _out 에 바로 쓴다
        self._out += (s + "\n").encode()

    def _stop(self, why):
        self.s_t = self.t_t = self.s_n = self.t_n = 0
        self.stops.append(why)
        self._out += f"STOP ({why})\n".encode()

    def _handle(self, s):
        if not s:
            return self._stop("blank line")
        c, arg = s[0], int(s[1:] or 0) if s[1:].strip().lstrip("-").isdigit() else 0
        still = lambda v: 0 if -50 < v < 50 else v
        if c == "s":
            self.s_t = still(max(-self.maxc, min(self.maxc, arg)))
        elif c == "t":
            self.t_t = still(max(-1000, min(1000, arg)))
        elif c == "x":
            self._stop("user")
        elif c == "m":
            self.maxc = max(0, min(1000, arg))
        elif c == "w":
            self.watchdog, self.host_lost = arg != 0, False

    def _wheel(self, out):
        if out == 0:
            return 0
        if self.deadband:
            out = int(math.copysign(self.deadband + abs(out) * (1000 - self.deadband) / 1000, out))
        return 0.0 if abs(out) < STALL else math.copysign((abs(out) - STALL * 0.75) / (1000 - STALL * 0.75) * VMAX, out)

    def blocked(self, x, y):
        if any(_seg_dist(x, y, *s) < RADIUS for s in self.segs):
            return True
        return any(math.hypot(x - ox, y - oy) < RADIUS + r for ox, oy, r, _ in self.obstacles)

    def _loop(self):
        dt, next_report = 0.1, time.monotonic()
        while self._alive:
            time.sleep(dt)
            now = time.monotonic()
            with self._lock:
                if self.watchdog and now - self.host_at > 1.0:
                    if not self.host_lost:
                        self.host_lost = True
                        self._stop("host watchdog")
                    self.s_t = self.t_t = 0
                ramp = lambda cur, tgt, k: tgt if abs(tgt - cur) <= k else cur + math.copysign(k, tgt - cur)
                self.s_n, self.t_n = int(ramp(self.s_n, self.s_t, 20)), int(ramp(self.t_n, self.t_t, 100))
                turn = self.t_n * self.maxc // 1000
                outl = max(-self.maxc, min(self.maxc, self.s_n + turn))
                outr = max(-self.maxc, min(self.maxc, self.s_n - turn))
                self.out = (outl, outr)
                vl, vr = self._wheel(outl), self._wheel(outr)
                v, w = (vl + vr) / 2, (vr - vl) / TRACK_TRUE
                nyaw = self.yaw + w * dt
                nx = self.x + v * math.cos(self.yaw + w * dt / 2) * dt
                ny = self.y + v * math.sin(self.yaw + w * dt / 2) * dt
                if not self.blocked(nx, ny):
                    self.x, self.y = nx, ny
                self.yaw = math.atan2(math.sin(nyaw), math.cos(nyaw))
                # 실물 (2026-10-14 바퀴 점검): 앞으로 갈 때 마스터 (오른쪽) 스텝은 −, 슬레이브 (왼쪽) 는 +, 속도는 왼쪽 바퀴가 −
                self.odo_m -= vr * dt / STEP_TRUE        # 바퀴는 막혀도 헛돈다
                self.odo_s += vl * dt / STEP_TRUE
                self.rx += 1
                if now >= next_report:
                    next_report = now + 0.2
                    kr, kl = vr * 3.6, -vl * 3.6
                    self._out += (f"cmd s{self.s_n:4d}>{self.s_t:4d} t{self.t_n:4d}>{self.t_t:4d} | meas {kr:6.2f} {kl:6.2f} km/h | "
                                  f" 0.50  0.50 A | 38.20 V | odo {int(self.odo_m)} {int(self.odo_s)} | rx {self.rx} | "
                                  f"m2 {kr:6.2f} {kl:6.2f} km/h  0.50  0.50 A 38.20 V rx2 {self.rx} | out {outl} {outr}\n").encode()


class FakeCamera:
    """Xtion 비슷한 깊이 · 색 — 화각 58° × 45°, 320 × 240, 렌즈 높이 cam_h, cam_pitch 만큼 숙임, 차 중심 앞 cam_x."""

    def __init__(self, robot, w=320, h=240, hfov=58.0, vfov=45.0, cam_x=0.38, cam_h=0.30, cam_pitch=3.0):
        self.robot, self.w, self.h = robot, w, h
        self.K = intrinsics(w, h, math.radians(hfov), math.radians(vfov))
        self.cam_x, self.cam_h = cam_x, cam_h
        g = ground_from(cam_h, cam_pitch)
        fx, fy, cx, cy = self.K
        v, u = np.mgrid[0:h, 0:w]
        r = np.stack([(u - cx) / fx, (v - cy) / fy, np.ones((h, w))], axis=-1)
        self.dl = np.stack([r @ g.fwd, r @ g.left, r @ g.up], axis=-1)       # 바닥 기준 광선 (앞, 왼쪽, 위), 깊이 1 당
        seg = np.array(robot.segs, dtype=float)
        self.A, self.B = seg[:, :2], seg[:, 2:]

    def read(self, timeout=None):
        rb = self.robot
        c, s = math.cos(rb.yaw), math.sin(rb.yaw)
        ox, oy, oz = rb.x + self.cam_x * c, rb.y + self.cam_x * s, self.cam_h
        D = self.dl
        dx, dy, dz = D[..., 0] * c - D[..., 1] * s, D[..., 0] * s + D[..., 1] * c, D[..., 2]
        t = np.full(dz.shape, np.inf)
        with np.errstate(divide="ignore", invalid="ignore"):
            t = np.where(dz < -1e-6, oz / -dz, t)                               # 바닥
            t = np.minimum(t, np.where(dz > 1e-6, (CEIL - oz) / dz, np.inf))   # 천장
            kind = np.where(dz < 0, 0, 1)
            e = self.B - self.A                                                  # 벽 선분 (M, 2)
            ddx, ddy = dx[..., None], dy[..., None]
            den = ddx * e[:, 1] - ddy * e[:, 0]
            wx, wy = self.A[:, 0] - ox, self.A[:, 1] - oy
            tt = (wx * e[:, 1] - wy * e[:, 0]) / den
            uu = (wx * ddy - wy * ddx) / den
            zz = oz + tt * dz[..., None]
            ok = (np.abs(den) > 1e-9) & (tt > 1e-3) & (uu >= 0) & (uu <= 1) & (zz >= 0) & (zz <= CEIL)
            tw = np.where(ok, tt, np.inf).min(axis=-1)
            kind = np.where(tw < t, 2, kind)
            t = np.minimum(t, tw)
            for (px, py, pr, ph) in rb.obstacles:                               # 원기둥 (사람 · 기둥)
                fx_, fy_ = ox - px, oy - py
                a = dx * dx + dy * dy
                b = 2 * (fx_ * dx + fy_ * dy)
                cc = fx_ * fx_ + fy_ * fy_ - pr * pr
                disc = b * b - 4 * a * cc
                tc = np.where(disc >= 0, (-b - np.sqrt(np.maximum(disc, 0))) / (2 * a), np.inf)
                zc = oz + tc * dz
                tc = np.where((tc > 1e-3) & (zc >= 0) & (zc <= ph), tc, np.inf)
                kind = np.where(tc < t, 3, kind)
                t = np.minimum(t, tc)
        depth = np.where((t >= 0.5) & (t <= 4.0), t, 0.0).astype(np.float32)   # D = t (광선이 깊이 1 당으로 정규화)
        shade = np.clip(1.0 - np.minimum(t, 8) / 10, 0.2, 1.0)
        base = np.array([[150, 150, 150], [235, 235, 235], [170, 200, 220], [60, 90, 220]], dtype=float)
        bgr = (base[kind] * shade[..., None]).astype(np.uint8)
        return Frame(bgr, depth, self.K, time.monotonic())

    def close(self):
        pass
