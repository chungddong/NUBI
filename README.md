# lolind32-wheel

LOLIN D32 (ESP32-D0WD-V3) firmware driving a 6.5" hoverboard hub motor over
UART. Built and flashed entirely from the CLI — no Arduino IDE.

Status: link verified in both directions, motor runs under command, GUI control
panel working, on-board Wi-Fi hotspot with a captive-portal control page.
Tank drive (one master per wheel, mixing on the ESP32) is implemented but not
yet driven on the ground — see [Drive modes](#drive-modes). [WORKLOG.md](WORKLOG.md) is the
dated record of each working session and its open issues. [DEBUGGING.md](DEBUGGING.md) records how it got there, including
the wrong turns — worth reading before adding a new diagnostic, since most of
the lost time came from measurement mistakes rather than hardware.

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
in [platformio.ini](platformio.ini), or find it with `.\pio.cmd device list`.

## Wiring

Six-pin header on the mainboard, next to the ST-Link flash header:

| header | signal | MCU pin | LOLIN D32 |
| --- | --- | --- | --- |
| 1 | 3.3 V | 48 | *leave disconnected* |
| 2 | PB4 | 40 | — |
| 3 | PB5 | 41 | — |
| 4 | USART TX (board → us) | 42 (PB6) | GPIO16 (RX2) |
| 5 | USART RX (us → board) | 43 (PB7) | GPIO17 (TX2) |
| 6 | GND | — | GND |

Identify the ends before counting: pin 6 reads 0 Ω to the battery negative,
pin 1 reads 3.3 V to it with the board on. **Do not connect header 1 to the
D32's 3V3 while the hoverboard runs on its battery** — that ties two supplies
together.

`REMOTE_USART 0` in the mainboard's `config.h` is what puts the external
controller on PB6/PB7. The 4-pin master–slave header is `MASTERSLAVE_USART 1`
(PA2/PA3) and is not for us.

A component marked `015` sits in series between MCU pin 43 and header pin 5, so
that pin fails a continuity beep test. It passes signal fine — see
[DEBUGGING.md](DEBUGGING.md) for why "no beep" says less than it appears to.

With two masters, each gets its own UART — plain-UART masters transmit
unprompted, so they cannot share a wire:

| | master 1 (link A) | master 2 (link B) |
| --- | --- | --- |
| ESP32 RX ← header 4 | GPIO16 | GPIO26 |
| ESP32 TX → header 5 | GPIO17 | GPIO27 |
| GND ↔ header 6 | ✓ | ✓ |

What the two links *mean* is the drive mode (see [Drive modes](#drive-modes)):
in tank mode link A is the **left** wheel's master and link B the **right**
wheel's; in board mode each link carries a master+slave pair. Either way any
slave's remote header stays empty, both GNDs go to the D32, and GPIO25 stays
reserved as the diagnostics' unconnected control pin.

## Tools

Everything runs from the project venv. Only one may hold the serial port at a
time.

| tool | purpose |
| --- | --- |
| [tools/hoverctl.py](tools/hoverctl.py) | mouse control panel — sliders, telemetry, STOP, link-quality meter |
| [tools/speedsweep.py](tools/speedsweep.py) | step duty 100→1000, record speed and current, fit the motor's Kv |
| [tools/speedcalc.py](tools/speedcalc.py) | Kv → speeds and currents, flat and on grades, for any wheel count |
| [tools/test_stop.py](tools/test_stop.py) | regression test for the stop-resurrection bug |
| [tools/ledtest.py](tools/ledtest.py) | cycle the LED state bits — only useful if the LED module is populated |

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

### Driving it — phone (Wi-Fi hotspot)

The firmware raises its own access point at boot — no router involved:

| | |
| --- | --- |
| SSID / password | `hoverwheel` / `********` — constants at the top of [src/main.cpp](src/main.cpp); change them before taking the wheel anywhere public |
| Page | `http://192.168.4.1/` — a captive portal, so Android/iOS pop it automatically right after joining |

The page is a landscape button pad, game-controller style: steer buttons (◀ ▶)
on the left half, throttle buttons (▲ 전진 / ▼ 후진) on the right, STOP and
telemetry in the middle. Hold to drive, release to zero that axis; buttons
are multi-touch, so throttle and steer press together. Settings: speed
strength (25–100 % of the clamp), steer strength (25–100 %; of the clamp in
tank mode, of full lock in board mode), speed/steer invert toggles (persisted
in the browser; direction with `SPEED_COEFFICIENT -1` is an empirical
matter), the max-clamp field, and the [drive-mode](#drive-modes) settings
(mode, link B invert, deadband — these live in the firmware's flash, not the
browser). The link pill is the same meter as the GUI status bar (`link clean`
/ `DROPOUTS` / `board silent`), fed by the `rx` counter, not the report line;
while the dead-man has cut the wheel it shows the reason instead.

In board mode **steer only works while moving** (see [Drive modes](#drive-modes)
for why); the page says so and the steer buttons are meant to be held together
with a throttle button. In tank mode the steer buttons alone pivot the frame.

Two things carry over from the desktop tools by design:

- **The stop guards.** The page rate-limits slider drags exactly like
  hoverctl.py did when it grew the stop-resurrection bug, so it implements the
  same three fixes: STOP drops the pending send, a `stopped` latch zeroes any
  late slider callback until a fresh press, and the heartbeat never flushes
  commands.
- **A watchdog, but not opt-in.** The page heartbeats every 250 ms; if the
  firmware hears nothing from it for 1 s after it has issued a command —
  closed tab, crashed browser, phone out of range — the wheel stops
  (`STOP (web client lost)` on serial). Unlike the serial `w` watchdog there
  is no off switch, because the web client is always a program, never a human
  idling at a monitor.

Wi-Fi control is independent of the USB serial port: the serial console and
the web page work at the same time (last writer wins), and only the COM-port
tools exclude each other. `hoverctl.py --port` and the monitor still cannot
run together.

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
| `v <0\|1>` | drive mode: `0` board mixes, `1` tank (persisted; stops the wheel first) |
| `i <0\|1>` | invert link B (tank mode; persisted; stops the wheel first) |
| `b <n>` | deadband duty, 0 = off (persisted) |
| `?` | help, and the current drive-mode settings |

### Drive modes

Two ways to turn a two-wheel frame, chosen with `v` (or the page's 구동
selector) and remembered in flash across reboots. Boot default is board
mode — the stock master+slave wiring drives as before; tank mode refuses to
move until both links deliver telemetry.

**Board mode (`v 0`)** — one master+slave pair per link, and the mainboard
mixes speed and steer itself. Upstream `main.c` does that *multiplicatively*
in pwm mode: steer is scaled down by an expo as speed rises, mapped to an
angle, and a lookup table turns that into the inner/outer duty **ratio**
(`pwmSlave = xScale × pwmMaster`). Two consequences, both measured against
the upstream source rather than guessed:

- steer with zero speed moves nothing — there is no pivot turn;
- at full lock the inner wheel gets at most ~12 % reverse duty (−109 at
  speed 200, −120 at 300, ~0 by 600, and +30 % *forward* at 1000). This motor
  needs ~20 % to move at all, so the inner wheel just sits and drags.

That is a hoverboard-rider tuning, not a bug here, and nothing sent in the
9-byte frame changes it.

**Tank mode (`v 1`)** — both boards run the *master* binary, one per wheel,
link A left and link B right, and the ESP32 does the mixing:

```
turn  = steer × max / 1000
left  = clamp(speed + turn, ±max)
right = clamp(speed − turn, ±max)      → steer field on the wire is 0
```

Steer alone pivots on the spot at ±max; full steer at full speed turns about
the stationary inner wheel; neither wheel ever exceeds the clamp. Positive
steer is a right turn (left wheel gets the extra). Left and right hub motors
face opposite ways, so link B is negated by default (`i 1`) — exactly what
the hoverboard master does to its slave. This is empirical: if "forward"
spins the frame on the spot, flip it with `i 0`; if left and right are
swapped, swap the link A/B wires or use the page's 조향 반전. Tank mode needs
**both** links alive — one wheel driving alone is a spin, so master 2 going
silent stops everything, from boot, with no hot-join grace.

**Deadband (`b`, both modes)** — below ~200 of 1000 (20 % duty) the wheel does
not move under the frame's weight, so with a 20-per-tick ramp a press used to
be a second of nothing. Every non-zero per-wheel command is now remapped onto
`[deadband, max]` before it goes on the wire; the ramp still runs in command
space, so only the first tick out of zero jumps. `b 0` disables it, and
`max ≤ deadband` disables it implicitly. Tune it to the lowest value that
reliably starts the loaded wheel. The serial report line ends in `out A B`,
the values actually sent per link.

### Diagnostics

The protocol gives no acknowledgement: the mainboard echoes nothing that
depends on our commands, and with the status LEDs unpopulated there is nothing
to watch either. These commands exist because of that.

| cmd | what it answers |
| --- | --- |
| `k` | Does the board hear us at all? Asserts `STATE_Shutoff`; if it obeys, its telemetry stops. Recover with the power button. |
| `d` | Where is the link broken? Frame counts on each pin (catches swapped wires), a pad loopback with a known-unconnected control pin (catches a dead output driver), a hex dump of the frame actually leaving the pin, and a pull-up probe (catches a wire reaching nothing). |
| `y` | Is the frame layout right? Sends each known command layout with `STATE_Shutoff` and reports which one silences the board. |
| `r` | How big is the series resistance on the TX line? Times the RC recovery against a known-unconnected control pin. Only ratios are meaningful, but a 40 kΩ pull-up and a 1 MΩ series part differ by 25×. |

Reference for reading `d` output — the pull-up probe compares against GPIO25,
which is deliberately left unconnected:

```
unconnected  GPIO25  after pulldown: 0/0   -> this is what a dangling pin looks like
our TX       GPIO17  after pulldown: 1/1   -> something external holds it high
```

Note the wording: the probe proves *something* holds the line high. It cannot
tell a pull-up on an input from a pin being driven high, so it narrows the
possibilities rather than identifying the far end.

Reading the `r` and pull-up probes requires the hoverboard to be **powered** —
PB7's pull-up exists only while the GD32 does. Each diagnostic checks this and
refuses to run otherwise. [DEBUGGING.md](DEBUGGING.md) lists what each one
cannot see.

### Safety

Four guards, all in [src/main.cpp](src/main.cpp):

- **Clamp** — `MAX_CMD_BOOT` is 400 of 1000 at boot. In pwm mode that is 40 %
  duty; raise it toward 1000 with `m` or the GUI spinbox to reach top speed.
  The clamp bounds what reaches the wire in both drive modes, deadband remap
  included. Steer is deliberately outside it (fixed ±1000): in board mode the
  mainboard maps steer to a wheel-speed *ratio* that can only slow the inner
  wheel, so full lock is speed-safe, and capping it made turns weak on the
  ground; in tank mode steer is scaled by the clamp and each wheel is clamped
  after mixing.
- **Ramp** — `RAMP_PER_TICK` limits speed change to 20 per 100 ms, so nothing
  steps straight to full command; steer ramps at 100 per 100 ms (full lock in
  ~1 s) because steering has to respond. Stops are exempt and take effect
  immediately. The deadband remap sits after the ramp, so the rate of change
  is unchanged; only the first tick out of zero jumps to a duty that moves.
- **Dead-man** — telemetry gaps over `RX_TIMEOUT_MS` zero the command, so a
  broken link cannot leave a wheel spinning. In tank mode both links must be
  alive; in board mode link B joins the dead-man after its first frame.
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
cable. Both are flashed now — `2.1.8 master Uart.bin` on the left-wheel board,
`2.1.8 slave.bin` on the right-wheel board — so `iSpeedR` is live and steer
actually steers. The ESP32 wires to the **master's** 6-pin header only; the
slave gets everything over the master–slave USART cable, and its own remote
header stays empty.

That is board mode. With a second master board the same two boards can
instead each be a master driving one wheel — tank mode, where the mixing moves
to the ESP32 and pivot turns become possible; see [Drive modes](#drive-modes).
The firmware talks to both masters over two independent UARTs (wiring table
above) and reads telemetry from each. In board mode link B is hot-join: it
takes part in the dead-man only after its master has delivered a frame, so a
not-yet-wired second pair cannot veto the first — but once seen, its silence
stops everything. In tank mode it is required from boot. Per-link wheels show
in the web page as ①/② and the serial report line carries a second block after
`rx N` (appended there so hoverctl.py's regex still matches).

For a 4-wheel frame in tank mode, each side becomes a master+slave pair with
the pair's steer at 0 — the slave then mirrors its master exactly. The master
negates its slave (`SendSlave(-pwmSlave)`, for the mirrored hoverboard
mounting), so a slave motor on the *same* side as its master must be mounted
the other way round to spin with it.

The board powers itself off after a few minutes of idling, and it did so in the
middle of three separate measurements during bring-up. Silence on GPIO16 almost
always means the mainboard is off, not a wiring fault. The firmware stays
listen-only for `QUIET_MS` (2 s) after reset to let the link settle before
commanding anything.

## Going to four motors

The eventual frame carries four of these motors. Top speed barely changes —
it is set by voltage and Kv, and sharing the road load across four motors cuts
each one's current and IR droop, so loaded speed moves slightly *up*. What
changes is thermal headroom and hill performance: on a 20 % grade, per-motor
current drops from ~21 A to ~8 A, and heating goes with its square.

```
.\.venv\Scripts\python.exe tools\speedcalc.py --motors 4 --mass 120
```

Two things will need real work:

- **Protocol changes.** Four motors means `REMOTE_UARTBUS` — one bus, several
  boards addressed by `iSlave`. Different command and feedback structs (both
  already present in `include/hoverserial.h` behind `#ifdef REMOTE_UARTBUS`),
  different mainboard binaries, and the boards stop providing pull-ups, so the
  ESP32 must. Upstream has an `Arduino Examples/Test_4Wheeler` sketch.
- **Wheels fighting each other.** On a rigid frame the ground forces all four to
  the same speed, but in pwm mode each motor only receives a voltage. A 3 % Kv
  spread is ~1.1 V of mismatch, which at ~0.5 Ω circulates **2.2 A** between
  motors — larger than the ~0.7 A each needs to cruise. `DRIVING_MODE 1`
  (closed-loop speed) is effectively required.

## Recovering from a bad flash

`esptool` is in the same venv:

```
.\.venv\Scripts\esptool.exe --port COM20 erase-flash
```

Then flash again normally.
