# lolind32-wheel

LOLIN D32 (ESP32-D0WD-V3) firmware. Built and flashed entirely from the CLI — no Arduino IDE.

## Toolchain

PlatformIO Core lives in a project-local venv (`.venv/`), so nothing is installed system-wide.
`pio.cmd` is a wrapper that calls it for you. It is a batch file rather than a `.ps1`
on purpose — PowerShell's default ExecutionPolicy (`Restricted`) refuses to run local
scripts, and batch files are not subject to it.

| what | command |
| --- | --- |
| build | `.\pio.cmd run` |
| build + flash | `.\pio.cmd run -t upload` |
| flash + open monitor | `.\pio.cmd run -t upload -t monitor` |
| serial monitor only | `.\pio.cmd device monitor` |
| clean | `.\pio.cmd run -t clean` |
| list ports | `.\pio.cmd device list` |

Exit the monitor with `Ctrl+C`.

## Board

- MCU: ESP32-D0WD-V3 rev 3.1, 240 MHz, dual core
- Port: `COM20` (CH340 USB-UART) — set in `platformio.ini`
- Auto-reset works over DTR/RTS, so no BOOT-button dance is needed
- Onboard LED: GPIO5, active-LOW

If the port number changes (different USB socket), update `upload_port` / `monitor_port`
in [platformio.ini](platformio.ini), or find it with `.\pio.ps1 device list`.

## Hoverboard link

Mainboard runs [RoboDurden's Hoverboard-Firmware-Hack-Gen2.x](https://github.com/RoboDurden/Hoverboard-Firmware-Hack-Gen2.x),
binary `2.1.8 master uart.bin` — the plain UART build, **not** UARTBUS.
Source lives in the separate [-GD32 repo](https://github.com/RoboDurden/Hoverboard-Firmware-Hack-Gen2.x-GD32);
`include/hoverserial.h` and `include/util.h` are copied verbatim from its
`PlatformIO Examples/TestSpeed/`.

| | |
| --- | --- |
| Link | 19200 8N1, ESP32 UART2, RX=GPIO16, TX=GPIO17 |
| Checksum | CRC16-XMODEM (poly `0x1021`, init `0x0000`), little-endian on the wire |
| Rate | board transmits ~10 Hz unprompted; we command at 10 Hz |

Both directions are verified against the live board. Receive: 86 consecutive
frames, no drops. Transmit: `STATE_Shutoff` asserted after an 8-second steady
baseline made the board go quiet 2.4 s and 2.0 s later on two separate trials —
far too prompt to be its inactivity timeout.

Mainboard config that shapes what commands mean:

| `config.h` | value | consequence |
| --- | --- | --- |
| `DRIVING_MODE` | `0` = pwm | the command *is* a duty cycle; `100` of 1000 is 10 % and barely creeps |
| `SPEED_COEFFICIENT` | `-1` | direction is inverted — decide which sign is "forwards" empirically |
| `STEER_COEFFICIENT` | `1` | |
| `BEEP_BACKWARDS` | on, `beepsBackwards = SET` | speed below −50 beeps, which confirms the command reached the motor logic without needing a motor |

### Liveness is not the report line

The firmware prints its telemetry line on a timer regardless of whether the
hoverboard is talking, reusing the last values it saw. Treating that line as
proof of life produced two confident and completely fictional test results
before it was caught. The line carries a trailing `rx N` counter that only
advances when a frame is actually decoded off UART2 — **watch that counter, not
the line.** `tools/hoverctl.py` and every diagnostic now do.

**Receive** — `SerialHover2Server`, 22 bytes, start `0xABCD`:

| off | field | units |
| --- | --- | --- |
| 0 | `cStart` = 0xABCD | |
| 2 | `iSpeedL` | 100× km/h |
| 4 | `iSpeedR` | 100× km/h |
| 6 | `iVolt` | 100× V |
| 8 | `iAmpL` | 100× A |
| 10 | `iAmpR` | 100× A |
| 12 | `iOdomL` | hall steps (int32) |
| 16 | `iOdomR` | hall steps (int32) |
| 20 | `checksum` | CRC16 of bytes 0–19 |

**Transmit** — `SerialServer2Hover`, 9 bytes. Note the start byte is `'/'`
(0x2F), *not* 0xABCD, and `HoverSend()` takes **steer before speed**:

| off | field | |
| --- | --- | --- |
| 0 | `cStart` = `'/'` | |
| 1 | `iSpeed` | −1000..1000 |
| 3 | `iSteer` | −1000..1000 |
| 5 | `wStateMaster` | LED/state bits |
| 6 | `wStateSlave` | LED/state bits |
| 7 | `checksum` | CRC16 of bytes 0–6 |

State bits: `1`=ledGreen `2`=ledOrange `4`=ledRed `8`=ledUp `16`=ledDown
`32`=Battery3Led `64`=Disable `128`=ShutOff.

### Driving it — GUI

```
.\hoverctl.cmd
```

[tools/hoverctl.py](tools/hoverctl.py) is a tkinter panel: vertical speed
slider, horizontal steer slider, live telemetry, max-clamp spinbox, and a
full-width STOP button. It finds the CH340 port on its own; `--port COM20`
overrides. `--selftest 5` connects, counts telemetry frames and exits — no
window, useful for checking the link from a script.

The status bar doubles as a link-quality meter: `linked 10/s … link clean`.
The board sends ~10 frames/s, so a second that lands between 1 and 7 means
frames went missing and the counter switches to `DROPOUTS n` — that is what a
marginal connector looks like while you drive. A second with none at all is the
board being off, reported separately.

Only one process can hold the serial port, so the GUI and the command-line
tools cannot run at the same time.

Keys: `space`/`Esc` stop, arrows nudge speed and steer, `0` centres.
"snap to 0 on release" (on by default) makes the speed slider spring back when
you let go of the mouse, so the wheel only turns while you hold it.

### Driving it — serial console

[src/main.cpp](src/main.cpp) also takes typed commands. Open the monitor and type:

| cmd | effect |
| --- | --- |
| `s <n>` | target speed (a bare number works too) |
| `t <n>` | target steer |
| `x` or blank line | stop |
| `m <n>` | raise/lower the clamp, 0..1000 |
| `l <n>` | LED/state bits |
| `w <0\|1>` | arm/disarm the host watchdog |
| `h` | heartbeat (feeds the watchdog, nothing else) |
| `?` | help |

### Diagnostics

The protocol gives no acknowledgement: the mainboard echoes nothing that
depends on our commands, and with the status LEDs unpopulated there is nothing
to watch either. These commands exist because of that.

| cmd | what it answers |
| --- | --- |
| `k` | Does the board hear us at all? Asserts `STATE_Shutoff`; if it obeys, its telemetry stops. Recover with the power button. |
| `d` | Where is the link broken? Frame counts on each pin (catches swapped wires), a pad loopback with a known-unconnected control pin (catches a dead output driver), a hex dump of the frame actually leaving the pin, and a pull-up probe (catches a wire reaching nothing). |
| `y` | Is the frame layout right? Sends each known command layout with `STATE_Shutoff` and reports which one silences the board. |

Reference for reading `d` output — the pull-up probe compares against GPIO25,
which is deliberately left unconnected:

```
unconnected  GPIO25  after pulldown: 0/0   -> this is what a dangling pin looks like
our TX       GPIO17  after pulldown: 1/1   -> external pull-up, so the wire reaches PB7
```

Steer follows the upstream convention `steer = right − left`, `speed = mean`,
so a single wheel only needs `s`. This assumes stock `SPEED_COEFFICIENT 16384`
and `STEER_COEFFICIENT 8192` in the mainboard's `config.h`.

### Safety

Three guards, all in [src/main.cpp](src/main.cpp):

- **Clamp** — `MAX_CMD_BOOT` is 400 of 1000 at boot. In pwm mode that is 40 %
  duty; raise it toward 1000 with `m` or the GUI spinbox to reach top speed.
- **Ramp** — `RAMP_PER_TICK` limits change to 20 per 100 ms, so nothing steps
  straight to full command. Stops are exempt and take effect immediately.
- **Dead-man** — telemetry gaps over `RX_TIMEOUT_MS` zero the command, so a
  broken link cannot leave a wheel spinning.
- **Host watchdog** — opt-in via `w 1`, which the GUI sends on connect and then
  feeds every 250 ms. Silence over `HOST_TIMEOUT_MS` stops the wheel, so a
  crashed GUI or an unplugged cable does not leave it running. It is off by
  default because a human watching a serial monitor types nothing for minutes.

Both timeouts compare against `loop()`'s single `iNow` timestamp. Do not
substitute a fresh `millis()` when stamping `iLastRxAt` / `iLastHostAt`: it can
land a millisecond ahead of `iNow`, and the unsigned subtraction then wraps to
~4.3 billion and trips the timeout instantly.

The `l` command masks off bits `64` (Disable) and `128` (ShutOff) — those cut
the board rather than steering it.

### A stop must not be undone by anything queued behind it

The GUI rate-limits slider drags to one write per 50 ms and lets the heartbeat
flush whatever the limiter swallowed. That combination made stopping unsafe: if
a value was still pending when you hit STOP, the wheel halted and then spun
back up to the old speed a quarter second later, when the flush fired.

Three guards now, all in [tools/hoverctl.py](tools/hoverctl.py):

- `stop()` calls `drop_pending()`, discarding swallowed values outright.
- A `stopped` latch, released only by a fresh mouse press or key, forces any
  later callback value back to zero — Tk runs its own scale bindings after ours,
  so a stopped slider can still be handed its mid-drag value.
- The heartbeat skips the flush entirely while stopped.

[tools/test_stop.py](tools/test_stop.py) drives the real throttle logic through
that exact sequence and asserts no non-zero command follows the stop. It fails
against the old code, so it will catch a regression:

```
.\.venv\Scripts\python.exe tools\test_stop.py
```

### Split boards

Gen2.x hoverboards carry two mainboards, one per wheel, joined by the sensor
cable. `2.1.8 master Uart.bin` covers the master only; the second wheel needs
`2.1.8 slave.bin` flashed onto the other board. Until then `iSpeedR` stays 0 no
matter what is commanded.

The board powers itself off after idling. Silence on GPIO16 usually means the
mainboard is off, not a wiring fault — the first five seconds after reset are
listen-only and print a raw byte count, which tells the two apart.

## Recovering from a bad flash

`esptool` is in the same venv:

```
.\.venv\Scripts\esptool.exe --port COM20 erase-flash
```

Then flash again normally.
