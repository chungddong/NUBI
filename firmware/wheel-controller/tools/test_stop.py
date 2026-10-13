"""Regression test: a stop must not be undone by a queued slider value.

Drives the real Link throttle logic through the sequence that made the wheel
spin back up after STOP. Uses an injected clock rather than sleeps -- Windows
resolves time.monotonic() to ~15.6 ms ticks, so a real 60 ms sleep sometimes
measures as 47 ms and the test would flake on the 50 ms threshold.

    .\\.venv\\Scripts\\python.exe tools\\test_stop.py
"""

import importlib.util
import os
import sys
import types

HERE = os.path.dirname(os.path.abspath(__file__))

# Stub pyserial so hoverctl imports with no hardware present.
fake = types.ModuleType("serial")
fake.Serial = object
fake.SerialException = Exception
tools = types.ModuleType("serial.tools")
ports = types.ModuleType("serial.tools.list_ports")
ports.comports = lambda: []
fake.tools = tools
tools.list_ports = ports
sys.modules.update({"serial": fake, "serial.tools": tools,
                    "serial.tools.list_ports": ports})

spec = importlib.util.spec_from_file_location("hoverctl", os.path.join(HERE, "hoverctl.py"))
hc = importlib.util.module_from_spec(spec)
spec.loader.exec_module(hc)


class Clock:
    """Replaces the time module inside hoverctl so elapsed time is exact."""

    def __init__(self):
        self.t = 1000.0

    def monotonic(self):
        return self.t

    def sleep(self, dt):
        self.t += dt


clock = Clock()
hc.time = clock


class FakeLink(hc.Link):
    def __init__(self):
        self.sent = []
        self.alive = True
        self._last_sent = {}

    def send(self, text):
        self.sent.append(text)


def scenario(with_fix):
    clock.t = 1000.0
    link = FakeLink()
    link.send_throttled("s", "s 100")     # goes out immediately
    clock.sleep(0.01)                     # still inside the rate limit window
    link.send_throttled("s", "s 300")     # swallowed, held pending

    if with_fix:
        link.drop_pending()
    link.send("s 0")                      # what stop() puts on the wire
    link.send("x")

    clock.sleep(0.25)                     # the heartbeat, one interval later
    link.flush_throttled()
    return link.sent


def after_stop(seq):
    last_stop = len(seq) - 1 - seq[::-1].index("x")
    return [c for c in seq[last_stop:] if c.startswith("s ") and c != "s 0"]


def main():
    unfixed = scenario(with_fix=False)
    fixed = scenario(with_fix=True)

    print("without drop_pending:", unfixed)
    print("with    drop_pending:", fixed)
    print()
    print("non-zero speed after the stop, unfixed:", after_stop(unfixed))
    print("non-zero speed after the stop, fixed  :", after_stop(fixed))

    assert after_stop(unfixed), \
        "the old bug no longer reproduces -- this test has stopped testing anything"
    assert not after_stop(fixed), \
        "FIX BROKEN: a non-zero speed still follows the stop"
    print("\nPASS: the fix holds, and the test still reproduces the original bug")


if __name__ == "__main__":
    main()
