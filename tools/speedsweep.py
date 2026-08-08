"""Step the duty command up and record what the wheel actually does.

Answers "how fast does this motor go, and how linear is it" with measurements
instead of a datasheet guess. Each step is held long enough to reach steady
state, then speed and current are averaged over the tail of the hold.

    .\\.venv\\Scripts\\python.exe tools\\speedsweep.py            up to 60 % duty
    .\\.venv\\Scripts\\python.exe tools\\speedsweep.py --max 1000  full scale

THE WHEEL MUST BE OFF THE GROUND AND THE BOARD CLAMPED DOWN.
Ctrl+C stops the sweep and commands zero on the way out.
"""

import argparse
import re
import sys
import time

import serial
from serial.tools import list_ports

RX = re.compile(r"\| rx (\d+)")
TELE = re.compile(
    r"meas\s*(-?[\d.]+)\s+(-?[\d.]+) km/h \|\s*(-?[\d.]+)\s+(-?[\d.]+) A \|\s*(-?[\d.]+) V"
)

WHEEL_INCHES = 6.5
CIRCUMFERENCE_M = 3.14159265 * WHEEL_INCHES * 0.0254

HOLD_S = 4.0        # ramp is 20/100 ms, so this covers the ramp plus settling
AVERAGE_TAIL_S = 2.0


def find_port(preferred=None):
    if preferred:
        return preferred
    for p in list_ports.comports():
        blob = f"{p.description} {p.manufacturer or ''}".lower()
        if any(k in blob for k in ("ch340", "cp210", "silicon labs", "usb-serial")):
            return p.device
    raise SystemExit("no USB-serial adapter found; pass --port COMxx")


def open_port(port):
    try:
        return serial.Serial(port, 115200, timeout=0.1)
    except serial.SerialException as exc:
        if "PermissionError" in str(exc) or "Access is denied" in str(exc):
            raise SystemExit(
                f"{port} is already open in another program.\n"
                f"Only one process can hold a serial port -- close hoverctl "
                f"(or any serial monitor) and try again.")
        raise SystemExit(f"could not open {port}: {exc}")


class Board:
    def __init__(self, port):
        self.s = open_port(port)
        self.s.dtr = False
        self.s.rts = False
        self.buf = b""
        self.rx = None
        self.samples = []

    def pump(self, window):
        advanced = False
        t0 = time.time()
        while time.time() - t0 < window:
            self.buf += self.s.read(256)
            while b"\n" in self.buf:
                line, self.buf = self.buf.split(b"\n", 1)
                text = line.decode("utf-8", "replace")
                m = RX.search(text)
                if m:
                    v = int(m.group(1))
                    if self.rx is not None and v > self.rx:
                        advanced = True
                    self.rx = v
                g = TELE.search(text)
                if g:
                    self.samples.append((time.time(), *[float(x) for x in g.groups()]))
        return advanced

    def send(self, text):
        self.s.write((text + "\n").encode())

    def tail(self, secs):
        cut = time.time() - secs
        return [s for s in self.samples if s[0] >= cut]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port")
    ap.add_argument("--max", type=int, default=600, help="highest duty command")
    ap.add_argument("--step", type=int, default=100)
    args = ap.parse_args()

    b = Board(find_port(args.port))
    print("press the hoverboard power button ...")
    t0 = time.time()
    while not b.pump(1.0):
        if time.time() - t0 > 150:
            print("timed out")
            return 1
    print("link up\n")

    b.send(f"m {args.max}")
    b.pump(0.3)

    print(f"  {'duty':>5}  {'%':>4}  {'km/h':>7}  {'rpm':>6}  {'amps':>6}  {'volts':>6}")
    print("  " + "-" * 44)
    rows = []
    try:
        for duty in range(args.step, args.max + 1, args.step):
            b.send(f"s {duty}")
            b.pump(HOLD_S)
            tail = b.tail(AVERAGE_TAIL_S)
            if not tail:
                print(f"  {duty:>5}  no telemetry")
                continue
            kmh = sum(abs(t[1]) for t in tail) / len(tail)
            amp = sum(abs(t[3]) for t in tail) / len(tail)
            volt = sum(t[5] for t in tail) / len(tail)
            rpm = kmh / 3.6 / CIRCUMFERENCE_M * 60
            pct = 100.0 * duty / 1000
            print(f"  {duty:>5}  {pct:>4.0f}  {kmh:>7.2f}  {rpm:>6.0f}  {amp:>6.2f}  {volt:>6.2f}")
            rows.append((duty, kmh, rpm, amp, volt))
    except KeyboardInterrupt:
        print("\n  interrupted")
    finally:
        b.send("s 0")
        b.send("x")
        b.pump(1.5)
        b.s.close()

    if len(rows) >= 2:
        # Fit on the top half only: friction dominates the low end and would
        # drag the slope down, understating the extrapolated top speed.
        top = rows[len(rows) // 2:]
        d0, k0 = top[0][0], top[0][1]
        d1, k1 = top[-1][0], top[-1][1]
        if d1 != d0:
            slope = (k1 - k0) / (d1 - d0)
            full = k0 + slope * (1000 - d0)
            volt = rows[-1][4]
            kv = (full / 3.6 / CIRCUMFERENCE_M * 60) / volt if volt else 0
            print(f"\n  measured peak      : {max(r[1] for r in rows):.2f} km/h "
                  f"at duty {max(r[0] for r in rows)}")
            print(f"  extrapolated 100 %  : {full:.1f} km/h  (linear fit on the top half)")
            print(f"  implied motor Kv    : {kv:.1f} rpm/V at {volt:.1f} V")
    return 0


if __name__ == "__main__":
    sys.exit(main())
