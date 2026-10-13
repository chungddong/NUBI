// UART frames between this wheel controller and a Gen2.x hoverboard mainboard
// running a plain-UART build (one link per master board).
//
// Written for NUBI from the frame layout in README.md ("Hoverboard link"):
//
//   command   ESP32 -> board,  9 bytes
//     0  uint8   '/'            start byte
//     1  int16   iSpeed         passthrough master: the master's wheel duty
//     3  int16   iSteer         passthrough master: the slave's wheel duty
//     5  uint8   wStateMaster   LED/state bits
//     6  uint8   wStateSlave    LED/state bits
//     7  uint16  checksum
//
//   feedback  board -> ESP32, 22 bytes, sent ~10 Hz unprompted
//     0  uint16  0xABCD         start word (0xCD first on the wire)
//     2  int16   iSpeedL        100x km/h
//     4  int16   iSpeedR        100x km/h
//     6  uint16  iVolt          100x V
//     8  int16   iAmpL          100x A
//    10  int16   iAmpR          100x A
//    12  int32   iOdomL         hall steps
//    16  int32   iOdomR         hall steps
//    20  uint16  checksum
//
// Multi-byte fields are little-endian (the ESP32's own byte order), and the
// checksum is CRC16-XMODEM (poly 0x1021, init 0) over every byte before it.
#pragma once

#include <Arduino.h>
#include <stddef.h>

#define CLAMP(x, lo, hi) ((x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)))

static const uint8_t  HOVER_CMD_START = '/';
static const uint16_t HOVER_FB_START = 0xABCD;

struct __attribute__((packed)) HoverCommand {
  uint8_t  cStart;
  int16_t  iSpeed;
  int16_t  iSteer;
  uint8_t  wStateMaster;
  uint8_t  wStateSlave;
  uint16_t checksum;
};

struct __attribute__((packed)) HoverFeedback {
  uint16_t cStart;
  int16_t  iSpeedL;
  int16_t  iSpeedR;
  uint16_t iVolt;
  int16_t  iAmpL;
  int16_t  iAmpR;
  int32_t  iOdomL;
  int32_t  iOdomR;
  uint16_t checksum;
};

static_assert(sizeof(HoverCommand) == 9, "command frame must be 9 bytes");
static_assert(offsetof(HoverCommand, iSteer) == 3 && offsetof(HoverCommand, checksum) == 7,
              "command field offsets");
static_assert(sizeof(HoverFeedback) == 22, "feedback frame must be 22 bytes");
static_assert(offsetof(HoverFeedback, iVolt) == 6 && offsetof(HoverFeedback, iOdomL) == 12 &&
              offsetof(HoverFeedback, checksum) == 20, "feedback field offsets");

// --- CRC16-XMODEM -------------------------------------------------------------
// Written recursively so it can also run at compile time (C++11 constexpr).
constexpr uint16_t HoverCrcShift(uint16_t crc, int bits) {
  return bits == 0 ? crc
                   : HoverCrcShift((crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021)
                                                  : (uint16_t)(crc << 1),
                                   bits - 1);
}
constexpr uint16_t HoverCrcByte(uint16_t crc, uint8_t b) {
  return HoverCrcShift((uint16_t)(crc ^ ((uint16_t)b << 8)), 8);
}
constexpr uint16_t HoverCrcText(const char *s, size_t n, uint16_t crc = 0) {
  return n == 0 ? crc : HoverCrcText(s + 1, n - 1, HoverCrcByte(crc, (uint8_t)*s));
}
// Standard check value for CRC16-XMODEM.
static_assert(HoverCrcText("123456789", 9) == 0x31C3, "CRC16-XMODEM check value");

inline uint16_t HoverCrc16(const uint8_t *p, size_t n) {
  uint16_t crc = 0;
  while (n--) crc = HoverCrcByte(crc, *p++);
  return crc;
}

// --- Link ---------------------------------------------------------------------
template <typename S>
void HoverBegin(S &serial, uint32_t baud, int rxPin, int txPin) {
  serial.begin(baud, SERIAL_8N1, rxPin, txPin);
}

// Sends one command frame. The steer field comes first, as it always has in
// this firmware's call sites: with a passthrough master it is the slave's wheel.
template <typename S>
void HoverSend(S &serial, int16_t iSteer, int16_t iSpeed,
               uint8_t wStateMaster = 32, uint8_t wStateSlave = 0) {
  HoverCommand cmd;
  cmd.cStart = HOVER_CMD_START;
  cmd.iSpeed = iSpeed;
  cmd.iSteer = iSteer;
  cmd.wStateMaster = wStateMaster;
  cmd.wStateSlave = wStateSlave;
  cmd.checksum = HoverCrc16(reinterpret_cast<const uint8_t *>(&cmd), sizeof(cmd) - 2);
  serial.write(reinterpret_cast<const uint8_t *>(&cmd), sizeof(cmd));
}

// Reads buffered bytes until one feedback frame with a valid checksum has been
// taken (returns true and fills `out`) or fewer bytes than a whole frame are
// left (returns false; they stay buffered for the next call). Bytes that do
// not start a frame, and frames with a bad checksum, are dropped.
template <typename S>
bool HoverReceive(S &serial, HoverFeedback &out) {
  const uint8_t lo = HOVER_FB_START & 0xFF, hi = HOVER_FB_START >> 8;
  while (serial.available() >= (int)sizeof(HoverFeedback)) {
    if (serial.read() != lo) continue;
    if (serial.peek() != hi) continue;  // keep it: it may be the next frame's 0xCD

    HoverFeedback fb;
    uint8_t *p = reinterpret_cast<uint8_t *>(&fb);
    p[0] = lo;
    for (size_t i = 1; i < sizeof(fb); i++) p[i] = (uint8_t)serial.read();

    if (HoverCrc16(p, sizeof(fb) - 2) == fb.checksum) {
      out = fb;
      return true;
    }
  }
  return false;
}
