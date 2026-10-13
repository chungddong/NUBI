# Field notes: bringing up the hoverboard link

What actually happened while getting a LOLIN D32 to drive a Gen2.x hoverboard
mainboard, including the wrong turns. Kept because most of the wrong turns were
caused by *measurement*, not by the hardware, and that failure mode is easy to
repeat.

## The short version

There was probably never a hardware fault.

`DRIVING_MODE 0` means the speed command **is** a PWM duty cycle. The first
test used `s 30` — 30 of 1000, i.e. **3 % duty**, which cannot overcome cogging
and bearing drag in a 6.5" hub motor. The wheel sitting still was correct
behaviour, not a symptom. At 10 % it managed 0.30 km/h; at 100 % it spins
freely.

Everything after that was chasing a phantom. The chase then got much longer
than it needed to be because the test scripts were watching the wrong device.

## The wrong turns

### 1. A test whose failure looked exactly like its success

The starting plan — inherited from an earlier session — was to send a bare
`0xAA` byte and watch for a buzzer or LED reaction, then sweep baud rates if
nothing happened.

The mainboard's receiver is a frame parser: start byte, fixed-length struct,
CRC. A stray `0xAA` is discarded. **Silence was the expected outcome even with
perfect wiring and the correct baud rate**, so the test could not distinguish a
working link from a broken one, and the baud sweep built on it was hunting a
signal that would never appear.

Replaced with a listener: the board transmits telemetry unprompted, so sweeping
baud rates while *receiving* finds the answer in one pass. 19200, first try.

### 2. Assuming the wrong protocol family

Initial framing assumed EFeru's `hoverboard-firmware-hack-FOC`: 115200 baud,
`0xABCD` start, 8-byte command, XOR checksum. This board runs RoboDurden's
Gen2.x instead: 19200, 22-byte feedback, 9-byte command starting with `'/'`,
CRC16-XMODEM.

Caught early, because the captured checksum was clearly not an XOR — changing
one byte of the payload changed the whole checksum. Brute-forcing all 65536
polynomials returned exactly one match, which later turned out to be identical
to the upstream `CalcCRC()`.

Lesson: the capture disagreed with the assumption within minutes. Checking an
assumption against bytes is cheap; carrying it forward is not.

### 3. Recommending 3 % throttle as the first test

See the short version. The advice "start at `s 30`" came from the EFeru
convention where ±300 is a normal command, and was never re-checked against
`DRIVING_MODE 0`. This single mistake generated the entire investigation below.

### 4. The instrument was measuring the wrong device

**The expensive one.** Test scripts decided whether the hoverboard was alive by
regex-matching the firmware's telemetry line:

```python
TELE = re.compile(r"\| meas .* V \| odo")
```

That line is printed on a 200 ms timer **whether or not the hoverboard is
transmitting**, reusing the last values received. The scripts were watching the
ESP32 talk to itself.

The evidence was there to catch it earlier. One test reported "15 frames in 3 s,
20 frames in 4 s" — exactly 5 lines per second, exactly the print cadence, and
nothing to do with the board's ~10 Hz.

Consequences:

- "Commands are not arriving" — unfounded, and stated with confidence.
- "The board shutting down was not our doing" — unfounded, and it was used to
  **overturn a valid result** (the format probe, which measured real decoded
  frames and had been right).
- Two reversals of the verdict in front of the user.

Fixed by making the firmware count real decoded frames and print `| rx N`.
A host now confirms life by watching that number advance. The same bug was
present in the GUI's "linked" indicator and was fixed there too.

### 5. A probe that could not tell a pull-up from a driven output

`ProbePin()` pulls a pin low, releases it, and checks whether it returns high.
It reported "external PULL-UP present → the wire reaches PB7", which was
presented as established.

It is not. A pin **driven** high by an output behaves identically. The probe
narrows the possibilities; it does not identify the far end.

It also silently depended on the hoverboard being powered — PB7's internal
pull-up only exists while the GD32 has power — so running it against a sleeping
board produced a confident "no pull-up out there at all".

### 6. Measuring while the board was asleep, three times

The mainboard powers itself off after a few minutes of inactivity. It did so in
the middle of a diagnosis, an RC probe, and a format probe, each time producing
a result that looked meaningful.

Every board-dependent diagnostic now calls `CountFrames()` first and refuses to
run if nothing is arriving. Preconditions belong in the tool, not in the
operator's head.

### 7. A conclusion that outran its measurement

An earlier session identified a component marked `015` in series with PB7,
measured it as "over 50 Ω" with a continuity beeper, and concluded it was a
harmless series damping resistor.

The measurement only establishes *"more than the beeper threshold"*, roughly
50 Ω. It cannot distinguish 100 Ω from 1 MΩ, and the difference matters
enormously: against PB7's ~40 kΩ pull-up, 100 Ω pulls the pin to 0.01 V while
1 MΩ leaves it at 3.17 V and the line is dead. The `015` marking reads as
1 MΩ under the 3-digit EIA convention, which made this look likely.

It turned out to be innocent — signal passes, so the value is small. But both
the original "it's fine" and the later alarm were inferences past the data. The
right statement was "we know only that it is above ~50 Ω, and that range spans
working and broken."

### 8. A stop that undid itself

Found by the user while driving, not by any test here: pressing STOP halted the
wheel, which then spun back up to its previous speed a moment later.

The GUI rate-limits slider writes to one per 50 ms and lets the heartbeat flush
whatever the limiter swallowed. A value still pending at the moment of the stop
was flushed 250 ms *after* it. The stop was correct; something queued behind it
overwrote it.

Fixed with three independent guards and a regression test
([tools/test_stop.py](tools/test_stop.py)) that reproduces the exact sequence
and fails against the old code.

Lesson: a safety action must invalidate everything already in flight, not merely
send the right thing itself.

### 9. A rule of thumb where the physics was cheap

"Loaded speed is 70–85 % of no-load" was offered from memory. Working it out
properly — cruise draws about 2 A, which costs ~1 V across the winding — gives
**97 %** on flat ground. The rule of thumb was off by a factor that would have
mattered for design decisions.

### 10. A safety regression test that only sometimes tested anything

Found while writing this document. `test_stop.py` originally slept 60 ms and
checked that the 50 ms rate limiter had expired. On Windows `time.monotonic()`
advances in ~15.6 ms ticks, so a real 60 ms sleep can measure as 46.9 ms — under
the threshold. The flush would not fire, the bug would not reproduce, and the
test failed with "expected the old bug to reproduce".

It had passed when first written. A flaky test guarding a safety fix is worse
than no test, because a red run gets read as noise.

Rewritten to inject a fake clock into the module under test, so no wall time is
involved at all. Now deterministic, instant, and it still asserts *both*
directions: that the fix works, and that the scenario still reproduces the
original bug — otherwise the test could silently decay into asserting nothing.

### 11. Operational friction worth remembering

- Background Python buffers stdout; long-running scripts need `python -u` or
  their progress arrives only after they exit.
- A serial port admits exactly one process. The GUI and every command-line tool
  are mutually exclusive; this produced two cryptic `PermissionError` tracebacks
  before the tools grew a readable message.

## Rules extracted

1. **Validate the instrument before trusting the measurement.** Ask what
   physically generates the signal being keyed on. A number that matches your
   own polling rate is a warning sign.
2. **A diagnostic must separate success from failure.** If "nothing happens" is
   the expected result in both cases, it is not a test.
3. **Include a control.** The unconnected GPIO25 is the only reason the pin
   probe output could be interpreted at all.
4. **State what a measurement establishes, not what it suggests.** ">50 Ω" is
   not "a damping resistor".
5. **Put preconditions in the tool.** Anything requiring the board to be powered
   should check, not assume.
6. **Repeatability separates cause from coincidence.** One board shutdown after
   a command is suggestive; the same thing twice after an 8-second steady
   baseline is proof.
7. **Know what each probe cannot see.** Every diagnostic below has a blind spot;
   they are listed so the next person does not rediscover them.
8. **Assert both directions in a regression test.** It must fail against the old
   code, or it can quietly stop testing the thing it was written for.
9. **Keep wall-clock time out of tests.** Inject the clock. `time.monotonic()`
   resolves to ~15.6 ms on Windows, which is enough to flake any threshold
   measured in tens of milliseconds.

## What each diagnostic actually proves

| tool | proves | blind to |
| --- | --- | --- |
| `d` frame counts per pin | which pin carries the board's TX; catches swapped wires | needs the board powered |
| `d` pad loopback (+ GPIO25 control) | the ESP32's output driver and GPIO matrix work | anything past the pad; would not catch a signal inversion, since both ends share it |
| `d` tx frame hex dump | the exact bytes leaving the pin, verifiable offline | nothing about whether they arrive |
| `d` pull-up probe (+ control) | something external holds the line high | cannot distinguish a pull-up from a driven output; needs the board powered |
| `r` RC probe | rough size of the external resistance, by time constant ratio | needs the board powered; capacitance is unknown, so only ratios are meaningful |
| `k` ShutOff | the board accepts our commands — the only acknowledgement this protocol offers | kills the board; recover with the power button |
| `y` format probe | which command layout the board accepts | only tests layouts that are coded in |
| `rx N` counter | real frames decoded off UART2 | — this is the reference the others should have used from the start |

## Timeline of the verdict on "do commands arrive?"

Recorded because it reversed twice, and the reversals were the instrument's
fault rather than the hardware's.

| # | measurement | instrument | verdict | valid? |
| --- | --- | --- | --- | --- |
| 1 | `acktest.py` | ESP32's own printout | not arriving | no |
| 2 | `y` format probe | firmware `CountFrames()` | arriving | **yes** |
| 3 | `repeat.py` | ESP32's own printout | not arriving | no |
| 4 | `repeat3.py`, 2 trials | `rx` counter | arriving, 2.4 s and 2.0 s | **yes** |
| 5 | negative-speed test | measured `iSpeedL` responded | arriving, and reaching the motor | **yes** |

Both valid measurements agreed from the start. Both invalid ones agreed with
each other too — sharing a bug does not make a result corroborated.
