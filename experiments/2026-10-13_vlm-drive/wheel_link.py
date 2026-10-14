"""ESP32 바퀴 제어기 (firmware/wheel-controller) 시리얼 링크 — Jetson 쪽.

명령 (시리얼 115200, 한 줄 텍스트 — docs/architecture/system-overview.md "인터페이스 1"):
    s <n>  속도 (±클램프)          t <n>  조향, 클램프 대비 ‰ (+ = 오른쪽, 정지 중이면 제자리 회전)
    x      즉시 정지 (램프 없이)    m <n>  클램프 상한 0..1000
    w 1    호스트 워치독 켜기 (1 s 동안 줄이 안 오면 정지)   h  하트비트
펌웨어는 200 ms 마다 보고 줄을 찍는다 (main.cpp loop()). odo 는 앞축 (링크 A) 마스터 · 슬레이브 바퀴의 홀 스텝이고,
마스터는 오른쪽 바퀴다 (ISSUES H3, ESP32 NVS 에 `i 1`).

안전: 하트비트는 '임대 (lease)' 가 살아 있을 때만 보낸다. 제어 루프가 lease() 를 계속 불러야 바퀴가 돈다 —
제어 루프가 멈추면 (예외 · 멈춤 · GPT 응답 기다림) 하트비트가 끊기고 1 s 뒤 ESP32 가 스스로 선다.
"""

import os
import re
import threading
import time
from collections import deque
from dataclasses import dataclass

BAUD = 115200
HEARTBEAT_S = 0.25
CH340 = (0x1A86, 0x7523)

# firmware/wheel-controller/tools/wheelbench.py 의 REPORT_RE 와 같은 줄
REPORT_RE = re.compile(
    r"cmd s\s*(-?\d+)>\s*(-?\d+) t\s*(-?\d+)>\s*(-?\d+) \| "
    r"meas\s*(-?[\d.]+)\s+(-?[\d.]+) km/h \|\s*(-?[\d.]+)\s+(-?[\d.]+) A \|\s*(-?[\d.]+) V \| "
    r"odo\s*(-?\d+)\s+(-?\d+) \| rx (\d+) \| "
    r"m2\s*(-?[\d.]+)\s+(-?[\d.]+) km/h\s+(-?[\d.]+)\s+(-?[\d.]+) A\s+(-?[\d.]+) V rx2 (\d+) \| "
    r"out (-?\d+) (-?\d+)"
)


@dataclass
class Report:
    t: float                     # time.monotonic() 받은 시각
    speed_now: int
    speed_target: int
    steer_now: int
    steer_target: int
    kmh: tuple                   # 앞축 (마스터 = 오른쪽, 슬레이브 = 왼쪽)
    amps: tuple
    volt: float
    odo: tuple                   # 앞축 홀 스텝 (마스터, 슬레이브)
    rx: int                      # 앞축 보드에서 받은 프레임 수 — 늘어야 링크가 살아 있다
    kmh_b: tuple
    rx_b: int
    out: tuple                   # 실제로 내보낸 바퀴 듀티 (왼쪽, 오른쪽)


def parse_report(line, t=None):
    m = REPORT_RE.search(line)
    if not m:
        return None
    g = m.groups()
    i, f = int, float
    return Report(t if t is not None else time.monotonic(), i(g[0]), i(g[1]), i(g[2]), i(g[3]),
                  (f(g[4]), f(g[5])), (f(g[6]), f(g[7])), f(g[8]), (i(g[9]), i(g[10])), i(g[11]),
                  (f(g[12]), f(g[13])), i(g[17]), (i(g[18]), i(g[19])))


def find_port():
    from serial.tools import list_ports
    ports = list(list_ports.comports())
    for p in ports:
        if (p.vid, p.pid) == CH340:
            return p.device
    raise SystemExit("ESP32 (CH340) 포트를 못 찾았다 — 케이블 · 드라이버 (ch341) · brltty 확인. 있는 포트: "
                     + ", ".join(p.device for p in ports))


def open_serial(port=None):
    import serial
    s = serial.Serial()
    s.port, s.baudrate, s.timeout = port or find_port(), BAUD, 0.1
    s.dtr = False                # 열 때 ESP32 를 리셋하지 않게 (리눅스에서는 그래도 리셋될 수 있다 — 아래 wait_ready)
    s.rts = False
    if os.name == "posix":
        s.exclusive = True       # bench.py · pad.py · drive.py 가 한 포트에 같이 쓰지 않게 (flock). 윈도우 COM 은 원래 혼자 연다
    s.open()
    return s


class WheelLink:
    def __init__(self, ser):
        self.ser = ser
        self.port = getattr(ser, "port", "?")
        self.report = None
        self.events = deque(maxlen=100)          # STOP · host back 같은 펌웨어 알림 줄
        self._lock = threading.Lock()
        self._lease_until = 0.0
        self._alive = True
        self.error = None
        threading.Thread(target=self._reader, daemon=True).start()
        threading.Thread(target=self._heartbeat, daemon=True).start()

    # ---- 받기
    def _reader(self):
        buf = b""
        while self._alive:
            try:
                buf += self.ser.read(256)
            except Exception as e:               # 케이블이 빠짐
                self.error = str(e)
                self._alive = False
                return
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                line = raw.decode("utf-8", "replace").strip()
                r = parse_report(line)
                if r:
                    self.report = r
                elif line:
                    line = re.sub(r'pass "[^"]*"', 'pass "***"', line)   # 부팅 때 찍는 Wi-Fi 비밀번호는 기록하지 않는다
                    self.events.append((time.monotonic(), line))

    def _heartbeat(self):
        while self._alive:
            if time.monotonic() < self._lease_until:
                self.send("h")
            time.sleep(HEARTBEAT_S)

    # ---- 보내기
    def send(self, line):
        assert line and "\n" not in line          # 빈 줄은 펌웨어에서 STOP — 일부러 보내지 않는다
        with self._lock:
            self.ser.write((line + "\n").encode("ascii"))

    def lease(self, seconds=0.6):
        """이 시간 동안 하트비트를 보낸다. 제어 루프가 걸음마다 부른다."""
        self._lease_until = time.monotonic() + seconds

    def wait_ready(self, timeout=6.0, need_board=True):
        """앞축 보드 텔레메트리 (rx 가 늘어남) 를 기다린다. 리셋 직후 2 s 는 펌웨어가 듣기만 한다.
        need_board=False 면 ESP32 보고 줄만 와도 돌려준다 (보드 전원이 꺼져 있어도 ESP32 점검)."""
        t0 = time.monotonic()
        first = None
        while time.monotonic() - t0 < timeout:
            r = self.report
            if r and first is None:
                first = r
                if not need_board:
                    return r
            if r and first and r.rx > first.rx:
                return r
            time.sleep(0.05)
        raise RuntimeError("바퀴 제어기 보고 줄 또는 보드 텔레메트리가 안 온다 — 메인보드 전원 · 링크 A 확인"
                           + (f" (마지막 알림: {self.events[-1][1]})" if self.events else ""))

    def arm(self, max_cmd):
        self.lease(1.0)
        self.send("x")
        self.send(f"m {int(max_cmd)}")
        self.send("w 1")

    def drive(self, speed, steer=0):
        self.send(f"s {int(speed)}")
        self.send(f"t {int(steer)}")

    def stop(self):
        try:
            self.send("x")
            self.send("x")                        # 포트 잡음이 앞에 붙어도 한 번은 먹게 (2026-09-17 RDK 와 같은 우회)
        except Exception:
            pass

    def fresh(self, max_age=0.6):
        r = self.report
        return r is not None and time.monotonic() - r.t <= max_age

    def close(self):
        self.stop()
        try:
            self.send("w 0")                      # 워치독을 끄고 나간다 — 켜 둔 채 끝나면 폰 조종이 막힌다 (ISSUES N2)
        except Exception:
            pass
        self._alive = False
        time.sleep(0.15)
        try:
            self.ser.close()
        except Exception:
            pass
