// Interactive speed/steer console for a Gen2.x hoverboard mainboard.
//
// Mainboard runs RoboDurden's Hoverboard-Firmware-Hack-Gen2.x, binary
// "2.1.8 master uart.bin" (plain UART build, not UARTBUS). Protocol details
// and the safety notes are in README.md.
//
// Type commands into the USB serial monitor at 115200:
//
//   s <n>   target speed, -MAX..MAX   (also: a bare number)
//   t <n>   target steer, -MAX..MAX
//   x       stop now (also: a blank line)
//   m <n>   raise/lower the MAX clamp, 0..1000
//   l <n>   LED/state bits: 1=green 2=orange 4=red 8=up 16=down 32=batt3
//   w <0|1> host watchdog: once on, silence from the host stops the wheel
//   h       heartbeat, feeds the watchdog and nothing else
//   ?       help
//
// Commands ramp instead of stepping, and the wheel is cut if telemetry stops.
//
// The watchdog is opt-in because a human at a serial monitor sends nothing
// while just watching. Automated hosts (tools/hoverctl.py) turn it on so that
// a crashed GUI or a yanked USB cable cannot leave the wheel spinning.

#define _DEBUG

#include <Arduino.h>

#include "util.h"
#include "hoverserial.h"

static const int PIN_RX = 16;  // ESP32 RX <- hoverboard TX
static const int PIN_TX = 17;  // ESP32 TX -> hoverboard RX

static const uint32_t BAUDRATE = 19200;
static const uint32_t SEND_MILLIS = 100;    // command rate
static const uint32_t REPORT_MILLIS = 200;  // telemetry print rate

// --- safety envelope -------------------------------------------------------
// Start conservative. Raise MAX with the 'm' command once the wheel behaves.
static const int MAX_CMD_BOOT = 400;      // |speed| and |steer| clamp at boot
static const int RAMP_PER_TICK = 20;      // per 100 ms -> ~2 s across the clamp
static const uint32_t RX_TIMEOUT_MS = 500;  // telemetry gap that cuts the command
static const uint32_t QUIET_MS = 2000;    // listen-only settle time after reset
static const uint32_t HOST_TIMEOUT_MS = 1000;  // host silence that cuts it, if armed

HardwareSerial oSerialHover(2);
SerialHover2Server oFb;

static int iMaxCmd = MAX_CMD_BOOT;
static int iSpeedTarget = 0, iSteerTarget = 0;
static int iSpeedNow = 0, iSteerNow = 0;
static uint8_t wState = 32;  // Battery3Led; never set 64 (Disable) or 128 (ShutOff)

static unsigned long iNextSend = 0, iNextReport = 0, iLastRxAt = 0;
static bool bCut = false;

static bool bWatchdog = false;      // armed by the host with 'w 1'
static bool bHostLost = false;
static unsigned long iLastHostAt = 0;

// One-shot diagnostic. With the status LEDs unpopulated there is no other way
// to observe whether the mainboard accepts our frames: nothing it reports back
// depends on them. STATE_Shutoff does -- main.c does `if (wState &
// STATE_Shutoff) ShutOff()`, which kills PWM, drops SELF_HOLD and spins in
// while(1). Either way its telemetry stops, and that we can see. Recover with
// the board's power button.
static uint8_t iShutoffFrames = 0;

// Printed with every report. The report line itself is emitted on a timer
// whether or not the hoverboard is talking, so it is NOT evidence of life --
// a host must watch this counter advance instead. Getting that wrong once
// already produced a confident, entirely fictional test result.
static uint32_t iRxFrames = 0;

static void Diagnose();     // defined below, next to their helpers
static void TryFormats();
static void MeasureRC();

static void Help() {
  Serial.println(F("  s <n>  speed   t <n>  steer   x  stop"));
  Serial.printf("  m <n>  max clamp (now %d)   l <n>  led bits (now %u)\n", iMaxCmd, wState);
  Serial.printf("  w <0|1> host watchdog (now %s)   h  heartbeat\n", bWatchdog ? "on" : "off");
}

static void StopNow(const char *why) {
  iSpeedTarget = iSteerTarget = 0;
  iSpeedNow = iSteerNow = 0;
  Serial.printf("STOP (%s)\n", why);
}

static void HandleLine(char *s) {
  while (*s == ' ') s++;
  if (!*s) {
    StopNow("blank line");
    return;
  }

  const char cmd = *s;
  const int arg = atoi(s + 1);

  switch (cmd) {
    case 's':
      iSpeedTarget = CLAMP(arg, -iMaxCmd, iMaxCmd);
      Serial.printf("speed -> %d\n", iSpeedTarget);
      break;
    case 't':
      iSteerTarget = CLAMP(arg, -iMaxCmd, iMaxCmd);
      Serial.printf("steer -> %d\n", iSteerTarget);
      break;
    case 'x':
      StopNow("user");
      break;
    case 'm':
      iMaxCmd = CLAMP(arg, 0, 1000);
      iSpeedTarget = CLAMP(iSpeedTarget, -iMaxCmd, iMaxCmd);
      iSteerTarget = CLAMP(iSteerTarget, -iMaxCmd, iMaxCmd);
      Serial.printf("max -> %d\n", iMaxCmd);
      break;
    case 'l':
      wState = (uint8_t)(arg & 0x3F);  // mask off Disable(64) and ShutOff(128)
      Serial.printf("led -> %u\n", wState);
      break;
    case 'w':
      bWatchdog = arg != 0;
      bHostLost = false;
      Serial.printf("watchdog -> %s\n", bWatchdog ? "on" : "off");
      break;
    case 'h':
      break;  // heartbeat: PollConsole already refreshed the timer
    case 'k':
      iSpeedTarget = iSteerTarget = iSpeedNow = iSteerNow = 0;  // skip ShutOff's ramp-down
      iShutoffFrames = 10;
      Serial.println(F("ShutOff asserted -- board should go quiet if it hears us"));
      break;
    case 'd':
      Diagnose();
      break;
    case 'y':
      TryFormats();
      break;
    case 'r':
      MeasureRC();
      break;
    case '?':
      Help();
      break;
    default:
      if (cmd == '-' || cmd == '+' || (cmd >= '0' && cmd <= '9')) {
        iSpeedTarget = CLAMP(atoi(s), -iMaxCmd, iMaxCmd);
        Serial.printf("speed -> %d\n", iSpeedTarget);
      } else {
        Serial.println(F("? unknown -- '?' for help"));
      }
  }
}

// Takes iNow rather than calling millis() itself: the watchdog compares against
// loop()'s timestamp, and a fresher millis() here would make iLastHostAt exceed
// it, underflowing the unsigned subtraction into a bogus timeout.
static void PollConsole(unsigned long iNow) {
  static char aLine[24];
  static uint8_t iLen = 0;

  while (Serial.available()) {
    const char c = Serial.read();
    iLastHostAt = iNow;
    if (c == '\r') continue;
    if (c == '\n') {
      aLine[iLen] = 0;
      HandleLine(aLine);
      iLen = 0;
    } else if (iLen < sizeof(aLine) - 1) {
      aLine[iLen++] = c;
    }
  }
}

// --- link diagnosis --------------------------------------------------------
// Counts 0xABCD start frames arriving on whatever pins UART2 is bound to.
static int CountFrames(uint32_t iMs) {
  const unsigned long t0 = millis();
  int n = 0;
  uint8_t cPrev = 0;
  while (millis() - t0 < iMs) {
    while (oSerialHover.available()) {
      const uint8_t c = oSerialHover.read();
      if (cPrev == 0xCD && c == 0xAB) n++;
      cPrev = c;
    }
  }
  return n;
}

// Drives a pin and reads it straight back through its own pad. Proves the UART
// output driver and GPIO matrix work without needing a jumper wire.
static int PadLoopback(int iPin) {
  static const uint8_t aProbe[] = {0x55, 0xAA, 0x5A, 0xA5};
  oSerialHover.end();
  oSerialHover.begin(BAUDRATE, SERIAL_8N1, iPin, iPin);
  delay(50);
  while (oSerialHover.available()) oSerialHover.read();

  oSerialHover.write(aProbe, sizeof(aProbe));
  oSerialHover.flush();
  delay(50);

  int n = 0;
  while (oSerialHover.available() && n < (int)sizeof(aProbe)) {
    if (oSerialHover.read() != aProbe[n]) break;
    n++;
  }
  return n;
}

// Reads back an actual command frame through the TX pad, so the bytes we put
// on the wire can be checked byte-for-byte instead of merely assumed correct.
static void DumpTxFrame() {
  oSerialHover.end();
  oSerialHover.begin(BAUDRATE, SERIAL_8N1, PIN_TX, PIN_TX);
  delay(50);
  while (oSerialHover.available()) oSerialHover.read();

  HoverSend(oSerialHover, 0, 0, (uint8_t)32, (uint8_t)32);
  oSerialHover.flush();
  delay(80);

  Serial.printf("  tx frame on the wire (%u bytes expected):",
                (unsigned)sizeof(SerialServer2Hover));
  int n = 0;
  while (oSerialHover.available()) {
    Serial.printf(" %02X", oSerialHover.read());
    n++;
  }
  Serial.printf("   [%d read]\n", n);
}

// Is anything actually attached to a pin? The GD32 configures its UART RX with
// an internal pull-up (setup.c: GPIO_PUPD_PULLUP), so a wire that reaches PB7
// snaps back HIGH on its own after we pull it down and let go. A wire that
// reaches nothing stays LOW -- only leakage can charge it.
static void ProbePin(int iPin, const char *sName) {
  pinMode(iPin, OUTPUT);
  digitalWrite(iPin, LOW);
  delayMicroseconds(500);
  pinMode(iPin, INPUT);  // release, no internal pull either way
  delayMicroseconds(50);
  const int iFast = digitalRead(iPin);
  delay(2);
  const int iSlow = digitalRead(iPin);

  pinMode(iPin, OUTPUT);
  digitalWrite(iPin, HIGH);
  delayMicroseconds(500);
  pinMode(iPin, INPUT);
  delay(2);
  const int iAfterHigh = digitalRead(iPin);

  Serial.printf("  %-14s GPIO%-2d  after pulldown: %d/%d   after pullup: %d   -> %s\n",
                sName, iPin, iFast, iSlow, iAfterHigh,
                iSlow ? "external PULL-UP present (wire reaches a pulled-up pin)"
                      : "no external pull-up (wire reaches nothing?)");
}

// The flashed binary is prebuilt and may predate or postdate the source we
// read. Two command layouts exist in that source -- one live, one commented
// out. Send each with STATE_Shutoff set and see which one silences the board;
// that is the only acknowledgement this protocol offers.
static uint16_t Crc16Xmodem(const uint8_t *p, int n) {
  uint16_t crc = 0;
  while (n-- > 0) {
    crc ^= (uint16_t)*p++ << 8;
    for (int i = 0; i < 8; i++) {
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
  }
  return crc;
}

static void SendRaw(uint8_t *aBuf, int iLen) {
  const uint16_t crc = Crc16Xmodem(aBuf, iLen - 2);
  aBuf[iLen - 2] = crc & 0xFF;
  aBuf[iLen - 1] = crc >> 8;
  oSerialHover.write(aBuf, iLen);
}

static void TryFormats() {
  Serial.println(F("\n--- command format probe ---"));
  oSerialHover.end();
  oSerialHover.begin(BAUDRATE, SERIAL_8N1, PIN_RX, PIN_TX);

  if (CountFrames(1500) == 0) {
    Serial.println(F("  board is silent to begin with -- power it on first"));
    iLastRxAt = iLastHostAt = millis();
    return;
  }

  const uint8_t wKill = 32 | 128;  // Battery3Led + STATE_Shutoff

  for (int iFmt = 0; iFmt < 2; iFmt++) {
    uint8_t aBuf[10];
    int iLen;
    if (iFmt == 0) {
      iLen = 9;                      // uint8_t cStart = '/'
      aBuf[0] = '/';
      aBuf[1] = aBuf[2] = 0;         // iSpeed
      aBuf[3] = aBuf[4] = 0;         // iSteer
      aBuf[5] = aBuf[6] = wKill;
      Serial.println(F("  format A: 9 bytes, cStart='/'"));
    } else {
      iLen = 10;                     // uint16_t cStart = 0xABCD
      aBuf[0] = 0xCD;
      aBuf[1] = 0xAB;
      aBuf[2] = aBuf[3] = 0;         // iSpeed
      aBuf[4] = aBuf[5] = 0;         // iSteer
      aBuf[6] = aBuf[7] = wKill;
      Serial.println(F("  format B: 10 bytes, cStart=0xABCD"));
    }

    for (int i = 0; i < 12; i++) {
      SendRaw(aBuf, iLen);
      oSerialHover.flush();
      delay(80);
    }

    const int n = CountFrames(1500);
    Serial.printf("    -> board sent %d frames after ShutOff%s\n", n,
                  n == 0 ? "   <<< THIS FORMAT IS ACCEPTED" : "   (ignored)");
    if (n == 0) break;
  }

  iLastRxAt = iLastHostAt = millis();
  Serial.println(F("--- end ---"));
}

// How big is the resistance feeding this pin? Pull it to ground, let go, and
// time the RC recovery. Absolute values need a known capacitance, but the
// question here only needs a ratio: a direct 40k pull-up and the same pull-up
// behind a 1M series part differ by ~25x, which is unmissable.
static uint32_t RiseTimeUs(int iPin, bool bInternalPullup) {
  uint32_t iBest = 0xFFFFFFFF;
  for (int iTry = 0; iTry < 9; iTry++) {  // interrupts only ever add time
    pinMode(iPin, OUTPUT);
    digitalWrite(iPin, LOW);
    delayMicroseconds(3000);
    pinMode(iPin, bInternalPullup ? INPUT_PULLUP : INPUT);

    const uint32_t t0 = micros();
    uint32_t iDt = 0;
    while (!digitalRead(iPin)) {
      iDt = micros() - t0;
      if (iDt > 200000) break;
    }
    if (iDt < iBest) iBest = iDt;
  }
  pinMode(iPin, INPUT);
  return iBest;
}

static void MeasureRC() {
  static const int PIN_CAL = 25;  // unconnected
  Serial.println(F("\n--- RC probe on the TX line ---"));

  // PB7's pull-up exists only while the GD32 is powered, so this measurement is
  // meaningless with the board off -- and this board keeps timing out and
  // shutting itself down mid-session.
  oSerialHover.end();
  oSerialHover.begin(BAUDRATE, SERIAL_8N1, PIN_RX, PIN_TX);
  if (CountFrames(1500) == 0) {
    Serial.println(F("  board is not transmitting -- power it on and retry"));
    iLastRxAt = iLastHostAt = millis();
    return;
  }
  oSerialHover.end();

  const uint32_t iCal = RiseTimeUs(PIN_CAL, true);   // ESP32 internal pull-up, ~45k
  const uint32_t iExt = RiseTimeUs(PIN_TX, false);   // whatever the wire finds
  const uint32_t iBoth = RiseTimeUs(PIN_TX, true);   // internal 45k in parallel

  Serial.printf("  GPIO%d internal pull-up (~45k reference): %lu us\n",
                PIN_CAL, (unsigned long)iCal);
  Serial.printf("  GPIO%d external path only:                %lu us\n",
                PIN_TX, (unsigned long)iExt);
  Serial.printf("  GPIO%d with internal pull-up too:         %lu us\n",
                PIN_TX, (unsigned long)iBoth);

  if (iExt >= 200000) {
    Serial.println(F("  never rose -> no pull-up out there at all"));
  } else if (iCal) {
    // Wire capacitance inflates the TX pin's time, so this ratio is an upper
    // bound on the resistance ratio -- good enough to separate 40k from 1M.
    Serial.printf("  ratio external/reference = %.1fx\n", (float)iExt / (float)iCal);
    Serial.println(F("  ~1-3x  -> series part is small, line is fine"));
    Serial.println(F("  >10x   -> big series resistance, we cannot pull PB7 low"));
  }

  oSerialHover.begin(BAUDRATE, SERIAL_8N1, PIN_RX, PIN_TX);
  iLastRxAt = iLastHostAt = millis();
  Serial.println(F("--- end ---"));
}

static void Diagnose() {
  static const int PIN_SPARE = 25;  // unconnected: control for the loopback method
  Serial.println(F("\n--- link diagnosis (takes ~5 s) ---"));

  // 1. Listen on the pin we normally transmit on. Hearing the board there
  //    means the two wires are the wrong way round.
  oSerialHover.end();
  oSerialHover.begin(BAUDRATE, SERIAL_8N1, PIN_TX, PIN_RX);
  const int nSwap = CountFrames(1500);

  // 2. Baseline on the pin we normally receive on.
  oSerialHover.end();
  oSerialHover.begin(BAUDRATE, SERIAL_8N1, PIN_RX, PIN_TX);
  const int nNorm = CountFrames(1500);

  Serial.printf("  frames on GPIO%d (our RX): %d\n", PIN_RX, nNorm);
  Serial.printf("  frames on GPIO%d (our TX): %d%s\n", PIN_TX, nSwap,
                nSwap ? "   <== WIRES SWAPPED" : "");

  // 3. Loopback. Validate the method on a spare pin first, so a failure on
  //    GPIO17 can be blamed on GPIO17 rather than on the technique.
  const int nCtl = PadLoopback(PIN_SPARE);
  Serial.printf("  loopback control GPIO%d: %d/4\n", PIN_SPARE, nCtl);

  if (nCtl != 4) {
    Serial.println(F("  control failed -- loopback inconclusive, ignore it"));
  } else if (nSwap) {
    Serial.println(F("  skipping GPIO17 loopback: something else drives that line"));
  } else {
    const int nTx = PadLoopback(PIN_TX);
    Serial.printf("  loopback GPIO%d: %d/4  %s\n", PIN_TX, nTx,
                  nTx == 4 ? "-> pin drives fine, so the WIRE or the far end is at fault"
                           : "-> PIN IS NOT DRIVING");
    if (nTx == 4) DumpTxFrame();
  }

  // Electrical continuity check. GPIO25 is the negative control: nothing is
  // attached to it, so whatever it reports is what "dangling" looks like.
  oSerialHover.end();
  Serial.println(F("  --- pin probe ---"));
  if (nNorm == 0) {
    Serial.println(F("  board was silent, so PB7 is unpowered -- probe skipped"));
  } else {
    ProbePin(PIN_SPARE, "unconnected");
    ProbePin(PIN_TX, "our TX");
  }

  oSerialHover.end();
  oSerialHover.begin(BAUDRATE, SERIAL_8N1, PIN_RX, PIN_TX);
  iLastRxAt = iLastHostAt = millis();  // the blocking above is not a real timeout
  Serial.println(F("--- end ---"));
}

static int Ramp(int iNow, int iTarget) {
  const int iDelta = iTarget - iNow;
  if (iDelta > RAMP_PER_TICK) return iNow + RAMP_PER_TICK;
  if (iDelta < -RAMP_PER_TICK) return iNow - RAMP_PER_TICK;
  return iTarget;
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.println(F("Gen2.x hoverboard console"));
  Serial.printf("uart2 %lu 8N1, rx=GPIO%d tx=GPIO%d\n",
                (unsigned long)BAUDRATE, PIN_RX, PIN_TX);
  Help();
  Serial.println(F("wheel off the ground before you touch 's'."));

  HoverSetupEsp32(oSerialHover, (int)BAUDRATE, PIN_RX, PIN_TX);
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  const unsigned long iNow = millis();
  digitalWrite(LED_BUILTIN, (iNow % 1000) < 200 ? LOW : HIGH);  // active-LOW on the D32

  PollConsole(iNow);

  while (Receive(oSerialHover, oFb)) {
    iRxFrames++;
    iLastRxAt = iNow;
    if (bCut) {
      bCut = false;
      Serial.println(F("telemetry back"));
    }
  }

  // Dead-man: a dead link must not leave the wheel spinning.
  if (iNow > QUIET_MS && iNow - iLastRxAt > RX_TIMEOUT_MS) {
    if (!bCut) {
      bCut = true;
      StopNow(iLastRxAt ? "telemetry lost" : "no telemetry -- board powered on?");
    }
    iSpeedTarget = iSteerTarget = 0;
  }

  // Same idea, other end of the wire: a dead host must not either.
  if (bWatchdog && iNow - iLastHostAt > HOST_TIMEOUT_MS) {
    if (!bHostLost) {
      bHostLost = true;
      StopNow("host watchdog");
    }
    iSpeedTarget = iSteerTarget = 0;
  } else if (bHostLost) {
    bHostLost = false;
    Serial.println(F("host back"));
  }

  if (iNow < QUIET_MS) return;  // let the link settle before commanding anything

  if (iNow > iNextSend) {
    iNextSend = iNow + SEND_MILLIS;
    iSpeedNow = Ramp(iSpeedNow, iSpeedTarget);
    iSteerNow = Ramp(iSteerNow, iSteerTarget);

    uint8_t wSend = wState;
    if (iShutoffFrames) {
      iShutoffFrames--;
      wSend |= 128;  // STATE_Shutoff, only ever from the 'k' diagnostic
    }
    HoverSend(oSerialHover, iSteerNow, iSpeedNow, wSend, wSend);
  }

  if (iNow > iNextReport) {
    iNextReport = iNow + REPORT_MILLIS;
    Serial.printf("cmd s%4d>%4d t%4d>%4d | meas %6.2f %6.2f km/h | %5.2f %5.2f A "
                  "| %5.2f V | odo %ld %ld | rx %lu\n",
                  iSpeedNow, iSpeedTarget, iSteerNow, iSteerTarget,
                  oFb.iSpeedL / 100.0, oFb.iSpeedR / 100.0,
                  oFb.iAmpL / 100.0, oFb.iAmpR / 100.0, oFb.iVolt / 100.0,
                  (long)oFb.iOdomL, (long)oFb.iOdomR, (unsigned long)iRxFrames);
  }
}
