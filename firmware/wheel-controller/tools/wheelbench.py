"""Bench panel for the wheel-mode firmware (src/main.cpp since 2026-10-13).

Shows all four wheels, both axle links and the vehicle settings (side swap,
deadband, clamp), and drives with hold-to-run buttons or short timed tests.
Like hoverctl.py it arms the firmware's host watchdog on connect and
heartbeats every 250 ms, so closing the window, pulling the cable or a crash
stops the wheels within a second.

    .\\wheelbench.cmd                 pick the port automatically
    .\\wheelbench.cmd --port COM10
    .\\wheelbench.cmd --selftest 5    connect, read telemetry and settings, exit (no motion)

Keys: space / Esc = STOP. Arrow keys drive while held.
"""

import argparse
import queue
import re
import threading
import time
import tkinter as tk
from tkinter import ttk

import serial

from hoverctl import find_port

BAUD = 115200
HEARTBEAT_MS = 250
POLL_MS = 40
TEST_SECONDS = 3

# The firmware prints this every 200 ms whether or not the boards talk; only
# the rx / rx2 counters prove a link is alive. "meas a b" is link A (front):
# a = the master's own wheel, b = its slave's. "m2" is the same for link B.
REPORT_RE = re.compile(
    r"cmd s\s*(-?\d+)>\s*(-?\d+) t\s*(-?\d+)>\s*(-?\d+) \| "
    r"meas\s*(-?[\d.]+)\s+(-?[\d.]+) km/h \|\s*(-?[\d.]+)\s+(-?[\d.]+) A \|\s*(-?[\d.]+) V \| "
    r"odo\s*(-?\d+)\s+(-?\d+) \| rx (\d+) \| "
    r"m2\s*(-?[\d.]+)\s+(-?[\d.]+) km/h\s+(-?[\d.]+)\s+(-?[\d.]+) A\s+(-?[\d.]+) V rx2 (\d+) \| "
    r"out (-?\d+) (-?\d+)"
)
SETTINGS_RE = re.compile(r"masters on the (left|RIGHT) wheel \| deadband (\d+)")
MAX_RE = re.compile(r"max clamp \(now (\d+)\)|^max -> (\d+)")

STOP_RED = "#c0392b"
LINK_OK, LINK_WARN, LINK_BAD, LINK_NONE = "#1e8449", "#b9770e", "#c0392b", "#7f8c8d"


class Link:
    """Serial reader thread. Opens the port without toggling DTR/RTS, so the
    ESP32 keeps running instead of resetting (ISSUES U1)."""

    def __init__(self, port):
        self.rx = queue.Queue()
        self.alive = True
        self.error = None
        self.ser = serial.Serial()
        self.ser.port, self.ser.baudrate, self.ser.timeout = port, BAUD, 0.2
        self.ser.dtr = False
        self.ser.rts = False
        try:
            self.ser.open()
        except serial.SerialException as exc:
            if "PermissionError" in str(exc) or "Access is denied" in str(exc):
                raise SystemExit(f"{port} is already open in another program "
                                 f"(hoverctl, a serial monitor, speedsweep) -- close it first.")
            raise SystemExit(f"could not open {port}: {exc}")
        self.port = port
        threading.Thread(target=self._reader, daemon=True).start()

    def _reader(self):
        buf = b""
        while self.alive:
            try:
                buf += self.ser.read(256)
            except Exception as exc:  # port yanked
                self.error = str(exc)
                self.alive = False
                return
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                self.rx.put(line.decode("utf-8", "replace").strip())

    def send(self, text):
        if not self.alive:
            return
        try:
            self.ser.write((text + "\n").encode())
        except Exception as exc:
            self.error = str(exc)
            self.alive = False

    def close(self):
        if self.alive:
            self.send("x")
            self.send("w 0")
            time.sleep(0.1)
        self.alive = False
        try:
            self.ser.close()
        except Exception:
            pass


class Wheel:
    """One tile: name, speed, current."""

    def __init__(self, parent, name):
        self.frame = tk.Frame(parent, bd=1, relief="solid", padx=10, pady=6, bg="#f4f6f7")
        tk.Label(self.frame, text=name, font=("Malgun Gothic", 11, "bold"), bg="#f4f6f7").pack()
        self.speed = tk.Label(self.frame, text="--", font=("Consolas", 18, "bold"), width=7, bg="#f4f6f7")
        self.speed.pack()
        self.amp = tk.Label(self.frame, text="-- A", font=("Consolas", 10), bg="#f4f6f7")
        self.amp.pack()

    def show(self, kmh, amp, live):
        if not live:
            self.speed.config(text="--", fg="#7f8c8d")
            self.amp.config(text="-- A")
            return
        arrow = "▲" if kmh > 0.1 else "▼" if kmh < -0.1 else "■"
        self.speed.config(text=f"{arrow}{abs(kmh):5.2f}", fg="#1a5276" if abs(kmh) > 0.1 else "#2c3e50")
        self.amp.config(text=f"{amp:5.2f} A")


class App:
    def __init__(self, root, link, selftest=0):
        self.root, self.link, self.selftest = root, link, selftest
        self.swap = tk.BooleanVar(value=False)
        self.maxcmd = tk.IntVar(value=400)
        self.dead = tk.IntVar(value=100)
        self.sp_pct = tk.IntVar(value=40)
        self.st_pct = tk.IntVar(value=75)
        self.sdir = self.tdir = 0
        self.sent = {"s": None, "t": None}
        self.test_job = None
        self.held_keys = set()
        self.report = None
        self.rx_hist = []          # (time, rx, rx2) for the link meters
        self.settings_seen = False
        self._build()
        root.protocol("WM_DELETE_WINDOW", self.quit)
        for key in ("<space>", "<Escape>"):
            root.bind(key, lambda e: self.stop("키"))
        for key, (s, t) in {"Up": (1, 0), "Down": (-1, 0), "Left": (0, -1), "Right": (0, 1)}.items():
            root.bind(f"<KeyPress-{key}>", lambda e, k=key, s=s, t=t: self.key_down(k, s, t))
            root.bind(f"<KeyRelease-{key}>", lambda e, k=key: self.key_up(k))
        link.send("x")
        link.send("w 1")   # arm the firmware watchdog; heartbeat() feeds it
        link.send("?")     # settings come back as text
        self.root.after(HEARTBEAT_MS, self.heartbeat)
        self.root.after(POLL_MS, self.poll)
        if selftest:
            self.root.after(int(selftest * 1000), self.finish_selftest)

    # --- layout ----------------------------------------------------------------
    def _build(self):
        r = self.root
        r.title(f"wheelbench — {self.link.port}")
        top = tk.Frame(r, padx=10, pady=6)
        top.pack(fill="x")
        self.lbl_a = tk.Label(top, text="앞 링크 --", fg="white", bg=LINK_NONE, padx=8, font=("Malgun Gothic", 10, "bold"))
        self.lbl_a.pack(side="left")
        self.lbl_b = tk.Label(top, text="뒤 링크 --", fg="white", bg=LINK_NONE, padx=8, font=("Malgun Gothic", 10, "bold"))
        self.lbl_b.pack(side="left", padx=6)
        self.lbl_volt = tk.Label(top, text="-- V", font=("Consolas", 11))
        self.lbl_volt.pack(side="left", padx=10)
        self.lbl_cmd = tk.Label(top, text="", font=("Consolas", 10), fg="#555")
        self.lbl_cmd.pack(side="right")

        body = tk.Frame(r, padx=10)
        body.pack(fill="both", expand=True)

        # wheels, laid out as seen from above with the front at the top
        car = tk.LabelFrame(body, text=" 바퀴 (위에서 본 모습, 앞이 위) ", padx=8, pady=8)
        car.grid(row=0, column=0, sticky="n")
        self.w_fl, self.w_fr = Wheel(car, "앞좌"), Wheel(car, "앞우")
        self.w_rl, self.w_rr = Wheel(car, "뒤좌"), Wheel(car, "뒤우")
        self.w_fl.frame.grid(row=0, column=0, padx=6, pady=6)
        self.w_fr.frame.grid(row=0, column=2, padx=6, pady=6)
        tk.Label(car, text="▲ 앞", font=("Malgun Gothic", 10)).grid(row=0, column=1)
        self.w_rl.frame.grid(row=1, column=0, padx=6, pady=6)
        self.w_rr.frame.grid(row=1, column=2, padx=6, pady=6)
        self.lbl_out = tk.Label(car, text="출력 좌 -- / 우 --", font=("Consolas", 10))
        self.lbl_out.grid(row=2, column=0, columnspan=3, pady=(6, 0))

        # driving
        drive = tk.LabelFrame(body, text=" 구동 (누르는 동안만) ", padx=8, pady=8)
        drive.grid(row=0, column=1, sticky="n", padx=10)
        pad = tk.Frame(drive)
        pad.pack()
        for text, s, t, row, col in (("▲\n전진", 1, 0, 0, 1), ("◀\n좌회전", 0, -1, 1, 0),
                                     ("▶\n우회전", 0, 1, 1, 2), ("▼\n후진", -1, 0, 2, 1)):
            b = tk.Button(pad, text=text, width=8, height=3, font=("Malgun Gothic", 11))
            b.grid(row=row, column=col, padx=3, pady=3)
            b.bind("<ButtonPress-1>", lambda e, s=s, t=t: self.hold(s, t))
            b.bind("<ButtonRelease-1>", lambda e: self.release())
        tk.Button(pad, text="STOP", width=8, height=3, bg=STOP_RED, fg="white",
                  activebackground="#e74c3c", font=("Malgun Gothic", 11, "bold"),
                  command=lambda: self.stop("버튼")).grid(row=1, column=1, padx=3, pady=3)

        sliders = tk.Frame(drive)
        sliders.pack(fill="x", pady=(8, 0))
        tk.Label(sliders, text="속도 % (max 기준)").grid(row=0, column=0, sticky="w")
        tk.Scale(sliders, from_=10, to=100, resolution=5, orient="horizontal", length=180,
                 variable=self.sp_pct, command=lambda _: self.apply()).grid(row=0, column=1)
        tk.Label(sliders, text="조향 % (max 기준)").grid(row=1, column=0, sticky="w")
        tk.Scale(sliders, from_=10, to=100, resolution=5, orient="horizontal", length=180,
                 variable=self.st_pct, command=lambda _: self.apply()).grid(row=1, column=1)

        tests = tk.LabelFrame(drive, text=f" {TEST_SECONDS}초 시험 (끝나면 자동 정지) ", padx=6, pady=6)
        tests.pack(fill="x", pady=(8, 0))
        for i, (text, s, t) in enumerate((("전진", 1, 0), ("후진", -1, 0),
                                          ("제자리 좌", 0, -1), ("제자리 우", 0, 1))):
            tk.Button(tests, text=text, width=9, command=lambda s=s, t=t: self.run_test(s, t)).grid(
                row=i // 2, column=i % 2, padx=3, pady=3)

        # settings
        cfg = tk.LabelFrame(body, text=" 설정 (ESP32 플래시에 저장) ", padx=8, pady=8)
        cfg.grid(row=0, column=2, sticky="n")
        tk.Checkbutton(cfg, text="좌우 바꿈 (마스터가 오른쪽 바퀴)", variable=self.swap,
                       command=self.on_swap).grid(row=0, column=0, columnspan=3, sticky="w")
        tk.Label(cfg, text="max 클램프").grid(row=1, column=0, sticky="w", pady=(8, 0))
        tk.Spinbox(cfg, from_=0, to=1000, increment=50, width=6, textvariable=self.maxcmd).grid(
            row=1, column=1, pady=(8, 0))
        tk.Button(cfg, text="적용", command=self.on_max).grid(row=1, column=2, padx=4, pady=(8, 0))
        tk.Label(cfg, text="데드밴드").grid(row=2, column=0, sticky="w")
        tk.Spinbox(cfg, from_=0, to=1000, increment=10, width=6, textvariable=self.dead).grid(row=2, column=1)
        tk.Button(cfg, text="적용", command=self.on_dead).grid(row=2, column=2, padx=4)
        tk.Label(cfg, text="max는 재부팅하면 400으로 돌아갑니다.\n좌우 바꿈 · 데드밴드는 저장됩니다.",
                 fg="#555", justify="left").grid(row=3, column=0, columnspan=3, sticky="w", pady=(8, 0))

        logf = tk.LabelFrame(r, text=" ESP32 메시지 ", padx=6, pady=4)
        logf.pack(fill="both", padx=10, pady=(6, 10))
        self.log = tk.Text(logf, height=7, font=("Consolas", 9), state="disabled")
        self.log.pack(fill="both")

    # --- driving -----------------------------------------------------------------
    def send_st(self, s, t):
        for key, val in (("s", s), ("t", t)):
            if self.sent[key] != val:
                self.link.send(f"{key} {val}")
                self.sent[key] = val

    def apply(self):
        s = self.sdir * round(self.maxcmd.get() * self.sp_pct.get() / 100)
        t = self.tdir * round(1000 * self.st_pct.get() / 100)
        self.send_st(s, t)

    def hold(self, s, t):
        self.cancel_test()
        self.sdir, self.tdir = s, t
        self.apply()

    def release(self):
        self.sdir = self.tdir = 0
        self.apply()

    def key_down(self, key, s, t):
        if key in self.held_keys:      # Windows auto-repeat
            return
        if isinstance(self.root.focus_get(), (tk.Entry, tk.Spinbox)):
            return                     # arrows edit the field, not the wheels (ISSUES U2)
        self.cancel_test()
        self.held_keys.add(key)
        if s:
            self.sdir = s
        if t:
            self.tdir = t
        self.apply()

    def key_up(self, key):
        self.held_keys.discard(key)
        if key in ("Up", "Down") and not self.held_keys & {"Up", "Down"}:
            self.sdir = 0
        if key in ("Left", "Right") and not self.held_keys & {"Left", "Right"}:
            self.tdir = 0
        self.apply()

    def run_test(self, s, t):
        self.cancel_test()
        self.sdir, self.tdir = s, t
        self.apply()
        self.test_job = self.root.after(TEST_SECONDS * 1000, lambda: self.stop("시험 끝"))

    def cancel_test(self):
        if self.test_job:
            self.root.after_cancel(self.test_job)
            self.test_job = None

    def stop(self, why):
        self.cancel_test()
        self.held_keys.clear()
        self.sdir = self.tdir = 0
        self.link.send("x")             # zeroes target and ramp at once
        self.sent = {"s": 0, "t": 0}
        self.add_log(f"[GUI] STOP ({why})")

    # --- settings ----------------------------------------------------------------
    def on_swap(self):
        self.stop("좌우 바꿈")
        self.link.send(f"i {1 if self.swap.get() else 0}")

    def on_max(self):
        try:
            v = max(0, min(1000, int(self.maxcmd.get())))
        except (tk.TclError, ValueError):
            return
        self.link.send(f"m {v}")
        self.apply()

    def on_dead(self):
        try:
            v = max(0, min(1000, int(self.dead.get())))
        except (tk.TclError, ValueError):
            return
        self.link.send(f"b {v}")

    # --- telemetry ---------------------------------------------------------------
    def heartbeat(self):
        if self.link.alive:
            self.link.send("h")
            self.root.after(HEARTBEAT_MS, self.heartbeat)

    def add_log(self, line):
        self.log.config(state="normal")
        self.log.insert("end", time.strftime("%H:%M:%S ") + line + "\n")
        self.log.delete("1.0", "end-200l")
        self.log.see("end")
        self.log.config(state="disabled")

    def rate(self, idx):
        if len(self.rx_hist) < 2:
            return None
        t0, *c0 = self.rx_hist[0]
        t1, *c1 = self.rx_hist[-1]
        return (c1[idx] - c0[idx]) / (t1 - t0) if t1 > t0 else None

    def poll(self):
        while True:
            try:
                line = self.link.rx.get_nowait()
            except queue.Empty:
                break
            m = REPORT_RE.search(line)
            if m:
                self.report = m
                now = time.monotonic()
                self.rx_hist.append((now, int(m.group(12)), int(m.group(18))))
                self.rx_hist = [h for h in self.rx_hist if now - h[0] <= 1.2]
                continue
            sm = SETTINGS_RE.search(line)
            if sm:
                self.swap.set(sm.group(1) == "RIGHT")
                self.dead.set(int(sm.group(2)))
                self.settings_seen = True
            mm = MAX_RE.search(line)
            if mm:
                self.maxcmd.set(int(mm.group(1) or mm.group(2)))
            if line and not line.startswith(("s <n>", "m <n>", "w <0", "i <0")):
                self.add_log(line)
        self.refresh()
        if not self.link.alive:
            self.lbl_a.config(text=f"연결 끊김: {self.link.error}", bg=LINK_BAD)
            return
        self.root.after(POLL_MS, self.poll)

    def refresh(self):
        m = self.report
        if not m:
            return
        ra, rb = self.rate(0), self.rate(1)
        rx2 = int(m.group(18))
        for lbl, name, r, seen in ((self.lbl_a, "앞 링크", ra, True), (self.lbl_b, "뒤 링크", rb, rx2 > 0)):
            if not seen:
                lbl.config(text=f"{name} 없음", bg=LINK_NONE)
            elif r is None:
                lbl.config(text=f"{name} --", bg=LINK_NONE)
            else:
                bg = LINK_OK if r >= 7 else LINK_WARN if r >= 1 else LINK_BAD
                lbl.config(text=f"{name} {r:4.1f}/s", bg=bg)
        live_a = ra is not None and ra >= 1
        live_b = rb is not None and rb >= 1
        f_m, f_s, a_m, a_s = (float(m.group(i)) for i in (5, 6, 7, 8))
        r_m, r_s, b_m, b_s = (float(m.group(i)) for i in (13, 14, 15, 16))
        # telemetry L is the master's own wheel; the swap puts the masters on the right
        sw = self.swap.get()
        self.w_fl.show(*((f_s, a_s) if sw else (f_m, a_m)), live_a)
        self.w_fr.show(*((f_m, a_m) if sw else (f_s, a_s)), live_a)
        self.w_rl.show(*((r_s, b_s) if sw else (r_m, b_m)), live_b)
        self.w_rr.show(*((r_m, b_m) if sw else (r_s, b_s)), live_b)
        volt = float(m.group(9)) or float(m.group(17))
        self.lbl_volt.config(text=f"{volt:5.2f} V" if volt else "-- V")
        self.lbl_out.config(text=f"출력 좌 {int(m.group(19)):4d} / 우 {int(m.group(20)):4d}")
        self.lbl_cmd.config(text=f"s {m.group(1)}>{m.group(2)}  t {m.group(3)}>{m.group(4)}")

    # --- exit ----------------------------------------------------------------------
    def finish_selftest(self):
        m = self.report
        ra, rb = self.rate(0), self.rate(1)
        print(f"port {self.link.port}: report lines {'yes' if m else 'NO'}, "
              f"link A {ra if ra is None else round(ra, 1)}/s, link B {rb if rb is None else round(rb, 1)}/s, "
              f"settings {'read' if self.settings_seen else 'NOT read'}: swap={self.swap.get()} "
              f"dead={self.dead.get()} max={self.maxcmd.get()}"
              + (f", battery {m.group(9)} V" if m else ""))
        self.quit()

    def quit(self):
        self.link.close()
        self.root.destroy()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port")
    ap.add_argument("--selftest", type=float, default=0,
                    help="connect, read for N seconds, print what was seen, exit (sends no motion)")
    args = ap.parse_args()
    link = Link(find_port(args.port))
    root = tk.Tk()
    App(root, link, args.selftest)
    root.mainloop()


if __name__ == "__main__":
    main()
