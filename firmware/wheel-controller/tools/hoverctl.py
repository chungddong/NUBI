"""Mouse-driven control panel for the Gen2.x hoverboard link.

Talks to the ESP32 console firmware in src/main.cpp over USB serial. The
firmware owns the safety envelope (clamp, ramp, telemetry dead-man); this adds
the host-side half: it arms the firmware watchdog on connect and heartbeats
every 250 ms, so killing this window, unplugging the cable, or a Python crash
all stop the wheel within a second.

    .\\hoverctl.cmd                 pick the port automatically
    .\\hoverctl.cmd --port COM20
    .\\hoverctl.cmd --selftest 5    connect, verify telemetry, exit

Keys: space / Esc = STOP, up/down = speed, left/right = steer, 0 = centre.
"""

import argparse
import queue
import re
import sys
import threading
import time
import tkinter as tk
from tkinter import ttk

import serial
from serial.tools import list_ports

BAUD = 115200
HEARTBEAT_MS = 250
POLL_MS = 40
SEND_MIN_INTERVAL = 0.05  # s, rate limit for slider drags

# cmd s 30>  50 t 0> 0 | meas 1.20 1.19 km/h | 1.40 1.35 A | 36.72 V | odo 145 142 | rx 881
#
# The firmware emits this on a timer whether or not the hoverboard is talking,
# so its arrival proves nothing. Only the trailing rx counter -- incremented per
# frame actually decoded off UART2 -- distinguishes a live board from a dead one.
TELEMETRY_RE = re.compile(
    r"cmd s\s*(-?\d+)>\s*(-?\d+) t\s*(-?\d+)>\s*(-?\d+) \| "
    r"meas\s*(-?[\d.]+)\s+(-?[\d.]+) km/h \| "
    r"\s*(-?[\d.]+)\s+(-?[\d.]+) A \| "
    r"\s*(-?[\d.]+) V \| odo\s*(-?\d+)\s+(-?\d+) \| rx (\d+)"
)

STOP_RED = "#c0392b"
STOP_RED_ACTIVE = "#e74c3c"


def find_port(preferred=None):
    ports = list(list_ports.comports())
    if preferred:
        for p in ports:
            if p.device.upper() == preferred.upper():
                return p.device
        raise SystemExit(f"{preferred} not found. Available: "
                         + (", ".join(p.device for p in ports) or "none"))
    for p in ports:
        blob = f"{p.description} {p.manufacturer or ''}".lower()
        if any(k in blob for k in ("ch340", "cp210", "silicon labs", "usb-serial")):
            return p.device
    raise SystemExit("no USB-serial adapter found; pass --port COMxx")


class Link:
    """Serial reader thread plus a rate-limited writer."""

    def __init__(self, port):
        self.rx = queue.Queue()
        self.alive = True
        self.error = None
        self._last_sent = {}
        try:
            self.ser = serial.Serial(port, BAUD, timeout=0.2)
        except serial.SerialException as exc:
            if "PermissionError" in str(exc) or "Access is denied" in str(exc):
                raise SystemExit(
                    f"{port} is already open in another program.\n"
                    f"Only one process can hold a serial port -- close the other "
                    f"hoverctl window, speedsweep, or serial monitor first.")
            raise SystemExit(f"could not open {port}: {exc}")
        self.ser.dtr = False
        self.ser.rts = False
        self.port = port
        self._thread = threading.Thread(target=self._reader, daemon=True)
        self._thread.start()

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

    def send_throttled(self, key, text):
        """Collapse slider drags into at most one write per SEND_MIN_INTERVAL."""
        now = time.monotonic()
        last_t, last_text = self._last_sent.get(key, (0.0, None))
        if text == last_text or now - last_t < SEND_MIN_INTERVAL:
            self._last_sent[key] = (last_t, text)  # remember for the flush
            return False
        self._last_sent[key] = (now, text)
        self.send(text)
        return True

    def flush_throttled(self):
        """Emit any value that the rate limiter swallowed."""
        now = time.monotonic()
        for key, (last_t, text) in list(self._last_sent.items()):
            if text is not None and now - last_t >= SEND_MIN_INTERVAL:
                self._last_sent[key] = (now, text)
                self.send(text)

    def drop_pending(self):
        """Forget swallowed values so a later flush cannot resurrect them.

        Without this, stopping while a slider value was still rate-limited let
        the next heartbeat flush re-send the old speed a quarter second after
        the wheel had already stopped.
        """
        self._last_sent.clear()

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


class App:
    def __init__(self, root, link, selftest=0):
        self.root = root
        self.link = link
        self.selftest = selftest
        self.frames = 0
        self.last_rx = None
        self.last_frame_at = 0.0

        # Link quality, sampled once a second off the rx counter. The board
        # sends ~10 frames/s; a second that lands between 1 and 7 means frames
        # went missing, which is what a marginal connector looks like. A second
        # with none at all is the board being off, counted separately.
        self.fps = 0
        self.dropouts = 0
        self.prev_sec_rx = None
        self.stopped = True
        self._zeroing = False
        self.peak = 0.0
        self.speed = tk.IntVar(value=0)
        self.steer = tk.IntVar(value=0)
        self.maxcmd = tk.IntVar(value=400)
        self.hold = tk.BooleanVar(value=True)

        root.title(f"hoverctl - {link.port}")
        root.configure(padx=14, pady=14)
        root.minsize(620, 430)

        self._build()
        self.link.send("w 1")   # arm the firmware watchdog
        self.link.send("x")
        self.link.send(f"m {self.maxcmd.get()}")

        root.bind("<space>", lambda e: self.stop())
        root.bind("<Escape>", lambda e: self.stop())
        root.bind("<Up>", lambda e: self.nudge_speed(10))
        root.bind("<Down>", lambda e: self.nudge_speed(-10))
        root.bind("<Left>", lambda e: self.nudge_steer(-10))
        root.bind("<Right>", lambda e: self.nudge_steer(10))
        root.bind("<Key-0>", lambda e: self.stop())
        root.protocol("WM_DELETE_WINDOW", self.quit)

        self.root.after(POLL_MS, self.poll)
        self.root.after(HEARTBEAT_MS, self.heartbeat)
        self.root.after(1000, self.link_tick)
        if selftest:
            self.root.after(int(selftest * 1000), self.finish_selftest)

    # --- layout ------------------------------------------------------------
    def _build(self):
        big = ("Consolas", 26, "bold")
        mid = ("Consolas", 12)

        top = ttk.Frame(self.root)
        top.pack(fill="x", pady=(0, 10))
        self.status = ttk.Label(top, text="connecting...", font=mid)
        self.status.pack(side="left")

        body = ttk.Frame(self.root)
        body.pack(fill="both", expand=True)

        # speed column
        left = ttk.LabelFrame(body, text=" speed ", padding=10)
        left.pack(side="left", fill="y", padx=(0, 12))
        self.speed_lbl = ttk.Label(left, text="0", font=big, width=6, anchor="center")
        self.speed_lbl.pack()
        self.speed_scale = tk.Scale(
            left, from_=400, to=-400, resolution=5, showvalue=False,
            variable=self.speed, orient="vertical", length=210, width=34,
            command=self.on_speed,
        )
        self.speed_scale.pack(pady=6)
        self.speed_scale.bind("<ButtonPress-1>", self.arm)
        self.speed_scale.bind("<ButtonRelease-1>", self.on_release)
        ttk.Checkbutton(left, text="snap to 0 on release",
                        variable=self.hold).pack()

        # right side: steer + telemetry
        right = ttk.Frame(body)
        right.pack(side="left", fill="both", expand=True)

        steer_box = ttk.LabelFrame(right, text=" steer ", padding=10)
        steer_box.pack(fill="x")
        self.steer_lbl = ttk.Label(steer_box, text="0", font=("Consolas", 16, "bold"))
        self.steer_lbl.pack()
        self.steer_scale = tk.Scale(
            steer_box, from_=-400, to=400, resolution=5, showvalue=False,
            variable=self.steer, orient="horizontal", length=300, width=26,
            command=self.on_steer,
        )
        self.steer_scale.pack(fill="x")
        self.steer_scale.bind("<ButtonPress-1>", self.arm)
        self.steer_scale.bind("<ButtonRelease-1>", self.on_release)

        tele = ttk.LabelFrame(right, text=" telemetry ", padding=10)
        tele.pack(fill="both", expand=True, pady=(10, 0))
        self.tele = {}
        rows = [("speed L/R", "km/h"), ("peak speed", "km/h"), ("current L/R", "A"),
                ("battery", "V"), ("odometry L/R", "steps")]
        for i, (name, unit) in enumerate(rows):
            ttk.Label(tele, text=name, font=mid).grid(row=i, column=0, sticky="w", pady=2)
            v = ttk.Label(tele, text="--", font=("Consolas", 14, "bold"))
            v.grid(row=i, column=1, sticky="e", padx=12)
            ttk.Label(tele, text=unit, font=mid).grid(row=i, column=2, sticky="w")
            self.tele[name] = v
        tele.columnconfigure(1, weight=1)

        clamp = ttk.Frame(right)
        clamp.pack(fill="x", pady=(10, 0))
        ttk.Label(clamp, text="max", font=mid).pack(side="left")
        spin = tk.Spinbox(clamp, from_=50, to=1000, increment=100, width=6,
                          textvariable=self.maxcmd, command=self.on_max, font=mid)
        spin.pack(side="left", padx=8)
        spin.bind("<Return>", lambda e: self.on_max())     # typed values count too
        spin.bind("<FocusOut>", lambda e: self.on_max())
        ttk.Label(clamp, text="of 1000  (= % duty x10)", font=mid).pack(side="left")

        self.stop_btn = tk.Button(
            self.root, text="STOP   (space / esc)", command=self.stop,
            bg=STOP_RED, fg="white", activebackground=STOP_RED_ACTIVE,
            activeforeground="white", font=("Segoe UI", 16, "bold"),
            relief="raised", bd=3, height=2, cursor="hand2",
        )
        self.stop_btn.pack(fill="x", pady=(12, 0))

    # --- control -----------------------------------------------------------
    def _blocked(self, var, label):
        """While stopped, force any incoming callback value back to zero.

        Tk fires scale callbacks from its own class bindings after ours, so a
        stopped slider can still be handed the value it held mid-drag.
        """
        if not self.stopped:
            return False
        if var.get() != 0 and not self._zeroing:
            self._zeroing = True
            var.set(0)
            self._zeroing = False
        label.config(text="0")
        return True

    def on_speed(self, _=None):
        if self._blocked(self.speed, self.speed_lbl):
            return
        v = self.speed.get()
        self.speed_lbl.config(text=str(v))
        self.link.send_throttled("s", f"s {v}")

    def on_steer(self, _=None):
        if self._blocked(self.steer, self.steer_lbl):
            return
        v = self.steer.get()
        self.steer_lbl.config(text=str(v))
        self.link.send_throttled("t", f"t {v}")

    def on_release(self, _=None):
        if self.hold.get():
            self.stop()
        else:
            self.link.flush_throttled()

    def on_max(self):
        try:
            v = max(50, min(1000, self.maxcmd.get()))
        except tk.TclError:      # mid-typing garbage in the spinbox
            return
        for scale, var in ((self.speed_scale, self.speed), (self.steer_scale, self.steer)):
            scale.config(from_=v if scale is self.speed_scale else -v,
                         to=-v if scale is self.speed_scale else v)
            var.set(max(-v, min(v, var.get())))
        self.link.send(f"m {v}")

    def nudge_speed(self, d):
        self.stopped = False
        self.speed.set(max(-self.maxcmd.get(), min(self.maxcmd.get(), self.speed.get() + d)))
        self.on_speed()

    def nudge_steer(self, d):
        self.stopped = False
        self.steer.set(max(-self.maxcmd.get(), min(self.maxcmd.get(), self.steer.get() + d)))
        self.on_steer()

    def stop(self):
        # Latch first: every callback below checks self.stopped, so nothing
        # queued or fired afterwards can put a non-zero command back on the wire.
        self.stopped = True
        self.link.drop_pending()
        self.speed.set(0)
        self.steer.set(0)
        self.speed_lbl.config(text="0")
        self.steer_lbl.config(text="0")
        self.link.send("s 0")
        self.link.send("t 0")
        self.link.send("x")

    def arm(self, _=None):
        """Only a fresh press releases the stop latch."""
        self.stopped = False

    # --- pumps -------------------------------------------------------------
    def link_tick(self):
        if self.last_rx is not None:
            if self.prev_sec_rx is not None:
                got = self.last_rx - self.prev_sec_rx
                self.fps = got
                if 0 < got < 8:
                    self.dropouts += 1
            self.prev_sec_rx = self.last_rx
        self.root.after(1000, self.link_tick)

    def heartbeat(self):
        self.link.send("h")
        if not self.stopped:
            self.link.flush_throttled()
        self.root.after(HEARTBEAT_MS, self.heartbeat)

    def poll(self):
        while True:
            try:
                line = self.link.rx.get_nowait()
            except queue.Empty:
                break
            m = TELEMETRY_RE.search(line)
            if not m:
                continue
            g = m.groups()
            rx = int(g[11])
            if self.last_rx is not None and rx > self.last_rx:
                self.frames += rx - self.last_rx
                self.last_frame_at = time.monotonic()
            self.last_rx = rx
            sL, sR = float(g[4]), float(g[5])
            self.peak = max(self.peak, abs(sL), abs(sR))
            self.tele["speed L/R"].config(text=f"{sL:6.2f} {sR:6.2f}")
            self.tele["peak speed"].config(text=f"{self.peak:6.2f}")
            self.tele["current L/R"].config(text=f"{float(g[6]):5.2f} {float(g[7]):5.2f}")
            self.tele["battery"].config(text=f"{float(g[8]):5.2f}")
            self.tele["odometry L/R"].config(text=f"{g[9]} {g[10]}")

        if not self.link.alive:
            self.status.config(text=f"DISCONNECTED - {self.link.error or 'port closed'}")
        elif self.last_frame_at and time.monotonic() - self.last_frame_at < 1.0:
            drops = "link clean" if not self.dropouts else f"DROPOUTS {self.dropouts}"
            self.status.config(text=f"{self.link.port}   linked  {self.fps}/s"
                                    f"   {self.frames} frames   {drops}"
                                    f"   watchdog armed")
        else:
            self.status.config(text=f"{self.link.port}  open, no telemetry"
                                    f"  -  is the hoverboard powered on?")
        self.root.after(POLL_MS, self.poll)

    def finish_selftest(self):
        ok = self.frames > 0
        print(f"selftest: {self.frames} hoverboard frames decoded, "
              f"link={'OK' if ok else 'NO DATA (board powered off?)'}")
        self.quit()
        sys.exit(0 if ok else 1)

    def quit(self):
        self.link.close()
        self.root.destroy()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port")
    ap.add_argument("--selftest", type=float, default=0,
                    help="run headlessly for N seconds, report telemetry, exit")
    args = ap.parse_args()

    link = Link(find_port(args.port))
    root = tk.Tk()
    try:
        ttk.Style().theme_use("vista")
    except tk.TclError:
        pass
    App(root, link, args.selftest)
    try:
        root.mainloop()
    finally:
        link.close()


if __name__ == "__main__":
    main()
