"""Reproduce the stop-then-restart bug against the real Link throttle logic."""
import importlib.util, sys, time, types

# stub pyserial so hoverctl imports without hardware
fake = types.ModuleType("serial"); fake.Serial = object
tools = types.ModuleType("serial.tools"); lp = types.ModuleType("serial.tools.list_ports")
lp.comports = lambda: []
fake.tools = tools; tools.list_ports = lp
sys.modules.update({"serial": fake, "serial.tools": tools, "serial.tools.list_ports": lp})

spec = importlib.util.spec_from_file_location(
    "hoverctl", r"D:\Develop\lolind32-wheel\tools\hoverctl.py")
hc = importlib.util.module_from_spec(spec); spec.loader.exec_module(hc)

class FakeLink(hc.Link):
    def __init__(self):
        self.sent = []; self.alive = True; self._last_sent = {}
    def send(self, text): self.sent.append(text)

def scenario(with_fix):
    L = FakeLink()
    L.send_throttled("s", "s 100")          # goes out
    L.send_throttled("s", "s 300")          # too soon -> swallowed, held pending
    if with_fix:
        L.drop_pending()
    L.send("s 0"); L.send("x")              # what stop() puts on the wire
    time.sleep(hc.SEND_MIN_INTERVAL + 0.01)
    L.flush_throttled()                     # the heartbeat, 250 ms later
    return L.sent

before = scenario(with_fix=False)
after  = scenario(with_fix=True)
print("without drop_pending:", before)
print("with    drop_pending:", after)

def resurrects(seq):
    stop = len(seq) - 1 - seq[::-1].index("x")
    return [c for c in seq[stop:] if c.startswith("s ") and c != "s 0"]

print()
print("commands after the stop, unfixed:", resurrects(before), "<- the bug")
print("commands after the stop, fixed  :", resurrects(after))
assert resurrects(before), "expected the old bug to reproduce"
assert not resurrects(after), "fix failed: a non-zero speed still follows the stop"
print("\nPASS: nothing non-zero follows the stop")
