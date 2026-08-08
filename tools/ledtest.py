"""Does the mainboard receive our commands at all?

The firmware copies wStateMaster straight onto its own LEDs
(remoteUart.c: `wState = pData->wStateMaster`), and nothing else it reports
depends on our transmissions. So the LEDs are the only observable proof that
the ESP32 -> hoverboard direction works. Watch the board, not this window.

    .\\.venv\\Scripts\\python.exe tools\\ledtest.py
"""

import sys
import time

import serial
from serial.tools import list_ports

STATES = [(1, "GREEN"), (2, "ORANGE"), (4, "RED"), (8, "UP arrow"), (16, "DOWN arrow")]
HOLD_S = 3.0


def find_port(preferred=None):
    ports = list(list_ports.comports())
    if preferred:
        return preferred
    for p in ports:
        blob = f"{p.description} {p.manufacturer or ''}".lower()
        if any(k in blob for k in ("ch340", "cp210", "silicon labs", "usb-serial")):
            return p.device
    raise SystemExit("no USB-serial adapter found; pass the port as argv[1]")


def main():
    port = find_port(sys.argv[1] if len(sys.argv) > 1 else None)
    s = serial.Serial(port, 115200, timeout=0.2)
    s.dtr = False
    s.rts = False

    print(f"{port}: waiting for the link to settle...")
    time.sleep(3.0)
    s.reset_input_buffer()
    s.write(b"x\n")  # speed stays 0 throughout; this test never moves anything

    print("\nWATCH THE HOVERBOARD LEDS -- not this window.\n")
    for _ in range(2):
        for bits, name in STATES:
            print(f"  sending wState={bits:<3} -> {name}")
            t0 = time.time()
            while time.time() - t0 < HOLD_S:
                s.write(f"l {bits}\n".encode())
                time.sleep(0.25)

    s.write(b"l 32\n")
    s.write(b"x\n")
    time.sleep(0.2)
    s.close()

    print("\nDid the board LEDs change colour in step with the lines above?")
    print("  yes -> commands ARE arriving; the problem is downstream (motor side)")
    print("  no  -> the ESP32 TX (GPIO17) -> hoverboard RX direction is dead")


if __name__ == "__main__":
    main()
