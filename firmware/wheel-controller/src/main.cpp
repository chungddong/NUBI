// Interactive speed/steer console for a Gen2.x hoverboard mainboard.
//
// Mainboard runs RoboDurden's Hoverboard-Firmware-Hack-Gen2.x, binary
// "2.1.8 master uart.bin" (plain UART build, not UARTBUS). Protocol details
// and the safety notes are in README.md.
//
// Type commands into the USB serial monitor at 115200:
//
//   s <n>   target speed, -MAX..MAX   (also: a bare number)
//   t <n>   target steer, -1000..1000 (not under the MAX clamp -- see below)
//   x       stop now (also: a blank line)
//   m <n>   raise/lower the MAX clamp, 0..1000
//   l <n>   LED/state bits: 1=green 2=orange 4=red 8=up 16=down 32=batt3
//   w <0|1> host watchdog: once on, silence from the host stops the wheel
//   h       heartbeat, feeds the watchdog and nothing else
//   v <0|1> drive mode: 0 = mainboard mixes (master+slave pair on link A),
//           1 = tank (link A = left wheel's master, link B = right wheel's)
//   i <0|1> invert link B (tank mode; the right motor is mounted mirrored)
//   b <n>   deadband: duty below which the wheel does not move, 0 = off
//   ?       help
//
// v, i and b persist in flash (NVS) across reboots.
//
// Commands ramp instead of stepping, and the wheel is cut if telemetry stops.
//
// The watchdog is opt-in because a human at a serial monitor sends nothing
// while just watching. Automated hosts (tools/hoverctl.py) turn it on so that
// a crashed GUI or a yanked USB cable cannot leave the wheel spinning.
//
// The board also raises its own Wi-Fi access point (AP_SSID below) with a
// captive-portal control page: join the network with a phone and the page
// opens by itself, or browse to http://192.168.4.1/. The page mirrors the
// serial commands and has its own always-armed watchdog -- see the
// "WiFi hotspot" section below. Serial control keeps working alongside it.

#define _DEBUG

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Preferences.h>

#include "util.h"
#include "hoverserial.h"

// Two masters, one per link. Plain-UART masters all talk unprompted at
// ~10 Hz, so they cannot share a wire -- each gets its own UART. What the two
// links mean depends on the drive mode (see "drive modes" below).
// GPIO25 stays reserved: it is the unconnected control pin the diagnostics
// compare against.
static const int PIN_RX = 16;   // link A: ESP32 RX <- master 1 TX (header 4)
static const int PIN_TX = 17;   // link A: ESP32 TX -> master 1 RX (header 5)
static const int PIN_RX2 = 26;  // link B: ESP32 RX <- master 2 TX (header 4)
static const int PIN_TX2 = 27;  // link B: ESP32 TX -> master 2 RX (header 5)

static const uint32_t BAUDRATE = 19200;
static const uint32_t SEND_MILLIS = 100;    // command rate
static const uint32_t REPORT_MILLIS = 200;  // telemetry print rate

// --- safety envelope -------------------------------------------------------
// Start conservative. Raise MAX with the 'm' command once the wheel behaves.
static const int MAX_CMD_BOOT = 400;      // |speed| clamp at boot
static const int RAMP_PER_TICK = 20;      // per 100 ms -> ~2 s across the clamp
// Steer is deliberately NOT under the speed clamp. In pwm mode the mainboard
// maps steer to a wheel-speed ratio -- it can only slow the inner wheel, never
// raise speed -- so full lock (+/-1000) is speed-safe, and capping it at the
// clamp just makes turns weak. It also ramps faster: steering must respond.
static const int MAX_STEER = 1000;
static const int STEER_RAMP_PER_TICK = 100;  // per 100 ms -> full lock in ~1 s
// Measured on the assembled frame 2026-08-12: positive iSteer turned it LEFT.
// Flipped here so that positive steer = right turn on every interface (web,
// serial 't', GUI) without each one needing its own invert.
static const int STEER_SIGN = -1;

// --- drive modes -------------------------------------------------------------
// MODE_BOARD: each link carries a master+slave pair and the mainboard mixes
//   speed and steer itself. Upstream main.c does that multiplicatively (steer
//   sets the inner/outer duty RATIO through a lookup table, scaled down by an
//   expo as speed rises), so steer with zero speed moves nothing, and at full
//   lock the inner wheel never gets more than ~12 % reverse duty -- below the
//   ~20 % this motor needs to move at all. No pivot turns, weak steering.
// MODE_TANK: link A is the LEFT wheel's master, link B the RIGHT wheel's,
//   and the mixing happens here: left = speed + turn, right = speed - turn,
//   with the frame's steer field at 0. Pivot turns work, and both wheels get a
//   duty they can act on. Both links must be alive to drive.
// Persisted in NVS with the 'v' command; the web page has the same switch.
// Boot default is board mode: that is what a stock master+slave pair does
// with no rewiring, and tank mode refuses to drive until both links talk.
enum { MODE_BOARD = 0, MODE_TANK = 1 };
static uint8_t iMode = MODE_BOARD;
// Left and right hub motors face opposite ways, so the same duty spins them
// in opposite directions -- the hoverboard master negates its slave for the
// same reason. Master 2 has nobody to negate it, so we do. Empirical: if
// "forward" spins the frame on the spot, flip this ('i' command).
static bool bInvertB = true;
// Below this duty the motor cannot overcome cogging and load (measured
// ~200 of 1000 on the assembled frame). Non-zero commands are remapped onto
// [iDeadband, iMaxCmd] so the wheel turns at the first ramp tick instead of
// after a second of nothing. 0 disables the remap. 'b' command.
static int iDeadband = 200;

static const uint32_t RX_TIMEOUT_MS = 500;  // telemetry gap that cuts the command
static const uint32_t QUIET_MS = 2000;    // listen-only settle time after reset
static const uint32_t HOST_TIMEOUT_MS = 1000;  // host silence that cuts it, if armed

HardwareSerial oSerialHover(2);   // link A (master 1)
HardwareSerial oSerialHover2(1);  // link B (master 2)
SerialHover2Server oFb;
SerialHover2Server oFb2;

static int iMaxCmd = MAX_CMD_BOOT;
static int iSpeedTarget = 0, iSteerTarget = 0;
static int iSpeedNow = 0, iSteerNow = 0;
static uint8_t wState = 32;  // Battery3Led; never set 64 (Disable) or 128 (ShutOff)

static int iOutA = 0, iOutB = 0;  // what last went on the wire, per link

static unsigned long iNextSend = 0, iNextReport = 0, iLastRxAt = 0, iLastRxAt2 = 0;
static bool bCut = false;
static const char *sCutWhy = "";  // shown on the web page while bCut

static bool bWatchdog = false;      // armed by the host with 'w 1'
static bool bHostLost = false;
static unsigned long iLastHostAt = 0;

// --- WiFi hotspot ------------------------------------------------------------
// SSID and password live in include/wifi_config.h, which is git-ignored so the
// password never reaches the repository: copy include/wifi_config.example.h.
// Anyone holding the password can drive the wheel.
#if __has_include("wifi_config.h")
#include "wifi_config.h"
#else
#error "include/wifi_config.h is missing: copy include/wifi_config.example.h and set WIFI_AP_PASS"
#endif
static_assert(sizeof(WIFI_AP_PASS) - 1 >= 8, "WPA2 needs a password of 8+ characters");
static const char *AP_SSID = WIFI_AP_SSID;
static const char *AP_PASS = WIFI_AP_PASS;
static const uint32_t WEB_TIMEOUT_MS = 1000;  // web-client silence that cuts it

static WebServer oWeb(80);
static DNSServer oDns;

// Web handlers run from oWeb.handleClient() inside loop(), and stamp their
// liveness with loop()'s timestamp -- NOT a fresh millis(), for the same
// unsigned-wrap reason documented at PollConsole().
static unsigned long g_iNow = 0;
static unsigned long iLastWebAt = 0;
static bool bWebActive = false;  // armed by the first web command

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
static uint32_t iRxFrames = 0, iRxFrames2 = 0;

static void Diagnose();     // defined below, next to their helpers
static void TryFormats();
static void MeasureRC();

// --- settings (NVS) ---------------------------------------------------------
static Preferences oPrefs;

static void PrintSettings() {
  Serial.printf("mode %s | link B %s | deadband %d\n",
                iMode == MODE_TANK ? "tank (A=left, B=right, mixed here)"
                                   : "board (mainboard mixes)",
                bInvertB ? "inverted" : "normal", iDeadband);
}

static void LoadSettings() {
  // Read/write open on purpose: read-only fails (and logs) until the
  // namespace exists, and it does not exist on the first boot.
  if (!oPrefs.begin("wheel", false)) return;
  iMode = oPrefs.getUChar("mode", iMode) ? MODE_TANK : MODE_BOARD;
  bInvertB = oPrefs.getBool("invb", bInvertB);
  const int iDead = oPrefs.getInt("dead", iDeadband);  // CLAMP is a macro: no calls inside it
  iDeadband = CLAMP(iDead, 0, 1000);
  oPrefs.end();
}

static void SaveSettings() {
  if (!oPrefs.begin("wheel", false)) return;
  oPrefs.putUChar("mode", iMode);
  oPrefs.putBool("invb", bInvertB);
  oPrefs.putInt("dead", iDeadband);
  oPrefs.end();
}

static void Help() {
  Serial.println(F("  s <n>  speed   t <n>  steer   x  stop"));
  Serial.printf("  m <n>  max clamp (now %d)   l <n>  led bits (now %u)\n", iMaxCmd, wState);
  Serial.printf("  w <0|1> host watchdog (now %s)   h  heartbeat\n", bWatchdog ? "on" : "off");
  Serial.println(F("  v <0|1> drive mode 0=board 1=tank   i <0|1> invert link B   b <n> deadband"));
  PrintSettings();
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
      iSteerTarget = CLAMP(arg, -MAX_STEER, MAX_STEER);
      Serial.printf("steer -> %d\n", iSteerTarget);
      break;
    case 'x':
      StopNow("user");
      break;
    case 'm':
      iMaxCmd = CLAMP(arg, 0, 1000);
      iSpeedTarget = CLAMP(iSpeedTarget, -iMaxCmd, iMaxCmd);
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
    case 'v':
      StopNow("mode change");  // the links change meaning under a moving wheel
      iMode = arg ? MODE_TANK : MODE_BOARD;
      SaveSettings();
      PrintSettings();
      break;
    case 'i':
      StopNow("link B inverted");
      bInvertB = arg != 0;
      SaveSettings();
      PrintSettings();
      break;
    case 'b':
      iDeadband = CLAMP(arg, 0, 1000);
      SaveSettings();
      PrintSettings();
      break;
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

// --- WiFi hotspot + captive-portal control page ------------------------------
// The page is a landscape button pad: steer buttons on the left half,
// throttle buttons on the right, STOP in the middle. Hold to drive, release
// to zero that axis -- commands are edge-triggered (press/release), so there
// is no rate-limiter queue for a stop to lose against; a 300 ms repeater
// re-sends the *current* intent only, which after a stop is zero.
//
// In board mode the steer mixing happens on the master mainboard and, in pwm
// mode (DRIVING_MODE 0), it is multiplicative: steer sets the left/right
// speed RATIO, so steer with zero speed moves nothing and the page says so.
// In tank mode the steer buttons alone pivot the frame. The drive-mode
// settings (mode, link B invert, deadband) live in the firmware's NVS, not in
// the browser, so the page reads them back from /api/status.
static const char PAGE_HTML[] PROGMEM = R"HTML(<!doctype html>
<html lang="ko"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<title>hoverwheel</title>
<style>
*{box-sizing:border-box;margin:0;padding:0;-webkit-tap-highlight-color:transparent}
html,body{height:100%}
body{background:#14161a;color:#e8e8e8;font:14px/1.4 system-ui,sans-serif;user-select:none;-webkit-user-select:none;overflow:hidden}
#wrap{display:flex;gap:10px;height:100%;padding:10px}
.cluster{flex:1;display:flex;gap:10px}
#steer{flex-direction:row}
#throttle{flex-direction:column}
.btn{flex:1;border:0;border-radius:16px;background:#23262d;color:#e8e8e8;font-size:36px;touch-action:none;display:flex;flex-direction:column;align-items:center;justify-content:center;gap:6px}
.btn small{font-size:13px;color:#9aa0a6}
.btn.on{background:#2f6fb4}
.btn.on small{color:#dbeaff}
#mid{width:240px;flex:none;display:flex;flex-direction:column;gap:8px;overflow-y:auto}
header{display:flex;justify-content:space-between;align-items:center}
h1{font-size:15px;font-weight:600}
#link{font-size:11px;padding:2px 8px;border-radius:10px;background:#2a2d33;color:#9aa0a6}
#link.ok{background:#123a24;color:#7ee2a8}
#link.warn{background:#3d2f10;color:#f0c36d}
#link.bad{background:#3d1512;color:#f1948a}
#hint{display:none}
@media (orientation:portrait){#hint{display:block;text-align:center;background:#3d2f10;color:#f0c36d;border-radius:8px;padding:5px;font-size:12px}}
.tele{display:grid;grid-template-columns:repeat(4,1fr);gap:4px}
.tele div{background:#1d2026;border-radius:6px;padding:4px 2px;text-align:center}
.tele b{display:block;font-size:13px;font-variant-numeric:tabular-nums}
.tele span{font-size:9px;color:#9aa0a6}
#cmds{display:flex;justify-content:space-around;font-size:15px;font-variant-numeric:tabular-nums;color:#9aa0a6}
#cmds b{color:#e8e8e8}
#stop{background:#c0392b;font-size:20px;font-weight:700;border:0;border-radius:12px;color:#fff;padding:16px}
.row{display:flex;gap:10px;align-items:center;font-size:12px;color:#9aa0a6;flex-wrap:wrap}
select,.row input[type=number]{background:#1d2026;border:1px solid #333;border-radius:6px;color:#e8e8e8;padding:4px;font-size:13px}
.row input[type=number]{width:64px}
.note{font-size:11px;color:#565c66}
</style></head><body>
<div id="wrap">
<div class="cluster" id="steer">
<button class="btn" id="bl">&#9664;<small>좌</small></button>
<button class="btn" id="br">&#9654;<small>우</small></button>
</div>
<div id="mid">
<header><h1>hoverwheel</h1><span id="link">connecting</span></header>
<div id="hint">가로 화면 권장</div>
<div class="tele">
<div><b id="s1l">--</b><span>①좌 km/h</span></div>
<div><b id="s1r">--</b><span>①우 km/h</span></div>
<div><b id="s2l">--</b><span>②좌 km/h</span></div>
<div><b id="s2r">--</b><span>②우 km/h</span></div>
<div><b id="volt">--</b><span>V</span></div>
<div><b id="a1">--</b><span>①A</span></div>
<div><b id="a2">--</b><span>②A</span></div>
<div><b id="rx">--</b><span>rx ①/②</span></div>
</div>
<div id="cmds"><span>S <b id="sval">0</b></span><span>T <b id="tval">0</b></span></div>
<button id="stop">STOP</button>
<div class="row">
<label>속도 <select id="sp"><option value="0.25">25%</option><option value="0.5">50%</option><option value="0.75">75%</option><option value="1">100%</option></select></label>
<label>조향 <select id="sg"><option value="0.25">25%</option><option value="0.5">50%</option><option value="0.75">75%</option><option value="1">100%</option></select></label>
</div>
<div class="row">
<label><input id="ivs" type="checkbox"> 속도 반전</label>
<label><input id="ivt" type="checkbox"> 조향 반전</label>
</div>
<div class="row"><span>max</span><input id="mx" type="number" min="0" max="1000" step="50" value="400"><button id="mxset">set</button></div>
<div class="row">
<label>구동 <select id="md"><option value="1">탱크 (A=좌 B=우)</option><option value="0">보드 믹싱</option></select></label>
<label><input id="ivb" type="checkbox"> B 반전</label>
</div>
<div class="row"><span>데드밴드</span><input id="db" type="number" min="0" max="1000" step="10" value="200"><button id="dbset">set</button></div>
<div class="note" id="note"></div>
</div>
<div class="cluster" id="throttle">
<button class="btn" id="bf">&#9650;<small>전진</small></button>
<button class="btn" id="bb">&#9660;<small>후진</small></button>
</div>
</div>
<script>
var $=function(i){return document.getElementById(i)};
var S=0,T=0,curMax=400;
function q(u){fetch(u).catch(function(){})}
function push(){q('/api/cmd?s='+S+'&t='+T)}
function lab(){$('sval').textContent=S;$('tval').textContent=T}
function opt(k,d){var v=localStorage.getItem(k);return v===null?d:v}
$('sp').value=opt('sp','0.5');$('sg').value=opt('sg','0.75');
$('ivs').checked=opt('ivs','0')==='1';$('ivt').checked=opt('ivt','0')==='1';
$('sp').onchange=function(){localStorage.setItem('sp',this.value)};
$('sg').onchange=function(){localStorage.setItem('sg',this.value)};
$('ivs').onchange=function(){localStorage.setItem('ivs',this.checked?'1':'0')};
$('ivt').onchange=function(){localStorage.setItem('ivt',this.checked?'1':'0')};
function spVal(){return Math.round(curMax*parseFloat($('sp').value)/10)*10*($('ivs').checked?-1:1)}
function stVal(){return Math.round(1000*parseFloat($('sg').value)/10)*10*($('ivt').checked?-1:1)}
function bind(id,set,clear){
 var b=$(id);
 b.addEventListener('pointerdown',function(e){try{b.setPointerCapture(e.pointerId)}catch(_){}b.classList.add('on');set();lab();push()});
 function up(){b.classList.remove('on');clear();lab();push()}
 b.addEventListener('pointerup',up);b.addEventListener('pointercancel',up);
}
bind('bf',function(){S=spVal()},function(){S=0});
bind('bb',function(){S=-spVal()},function(){S=0});
bind('bl',function(){T=-stVal()},function(){T=0});
bind('br',function(){T=stVal()},function(){T=0});
function clr(){['bf','bb','bl','br'].forEach(function(i){$(i).classList.remove('on')})}
function stop(){S=0;T=0;clr();lab();q('/api/stop')}
$('stop').addEventListener('click',stop);
var KEYS={ArrowUp:['bf'],ArrowDown:['bb'],ArrowLeft:['bl'],ArrowRight:['br']};
var kact={};
window.addEventListener('keydown',function(e){
 if(e.key===' '||e.key==='Escape'){stop();return}
 var k=KEYS[e.key];if(!k||e.repeat||kact[e.key])return;kact[e.key]=1;
 $(k[0]).dispatchEvent(new PointerEvent('pointerdown'));
});
window.addEventListener('keyup',function(e){
 var k=KEYS[e.key];if(!k)return;kact[e.key]=0;
 $(k[0]).dispatchEvent(new PointerEvent('pointerup'));
});
$('mxset').addEventListener('click',function(){q('/api/max?v='+(+$('mx').value||0))});
$('md').onchange=function(){q('/api/cfg?mode='+this.value)};
$('ivb').onchange=function(){q('/api/cfg?invb='+(this.checked?1:0))};
$('dbset').addEventListener('click',function(){q('/api/cfg?dead='+(+$('db').value||0))});
var NOTE={1:'탱크 모드: 좌/우만 눌러도 제자리 회전합니다. 조향 %는 max 기준이며, 어느 바퀴도 max를 넘지 않습니다. 전진을 눌렀는데 제자리에서 돌면 "B 반전"을 바꾸세요.',
 0:'보드 믹싱: 조향은 주행 중에만 듣습니다 — 전진/후진을 누른 채 좌/우를 함께 누르세요. 조향 %는 풀 조향각 기준이며 max와 무관합니다'};
setInterval(function(){q('/api/ping')},250);
setInterval(function(){if(S||T)push()},300);
var pRx=-1,pRx2=0,pT=0;
function poll(){fetch('/api/status').then(function(r){return r.json()}).then(function(j){
 if(j.max!==curMax){curMax=j.max;if(document.activeElement!==$('mx'))$('mx').value=j.max}
 if(document.activeElement!==$('md'))$('md').value=j.mode;
 $('ivb').checked=!!j.invb;
 if(document.activeElement!==$('db'))$('db').value=j.dead;
 $('note').textContent=NOTE[j.mode]||'';
 $('s1l').textContent=(j.spdL/100).toFixed(2);
 $('s1r').textContent=(j.spdR/100).toFixed(2);
 $('s2l').textContent=(j.spdL2/100).toFixed(2);
 $('s2r').textContent=(j.spdR2/100).toFixed(2);
 $('volt').textContent=(j.volt/100).toFixed(1);
 $('a1').textContent=((Math.abs(j.ampL)+Math.abs(j.ampR))/100).toFixed(1);
 $('a2').textContent=((Math.abs(j.ampL2)+Math.abs(j.ampR2))/100).toFixed(1);
 var t=Date.now(),el=$('link');
 if(pRx>=0&&t>pT){
  var dt=t-pT,f1=(j.rx-pRx)*1000/dt,f2=(j.rx2-pRx2)*1000/dt;
  $('rx').textContent=f1.toFixed(0)+'/'+f2.toFixed(0);
  if(f1>=8&&(j.rx2===0||f2>=8)){el.textContent='link clean';el.className='ok'}
  else if(f1>=1||f2>=1){el.textContent='DROPOUTS';el.className='warn'}
  else{el.textContent='board silent';el.className='bad'}}
 pRx=j.rx;pRx2=j.rx2;pT=t;
 if(j.cut){el.textContent='CUT: '+j.why;el.className='bad'}
}).catch(function(){var el=$('link');el.textContent='no connection';el.className='bad'})}
setInterval(poll,400);poll();
</script></body></html>
)HTML";

static void WebRoot() {
  iLastWebAt = g_iNow;
  oWeb.send_P(200, "text/html", PAGE_HTML);
}

// Every URL we do not serve redirects to the control page. Phones probe a
// known URL right after joining (generate_204, hotspot-detect.html, ...);
// getting a redirect instead of the expected answer is what makes them pop
// the captive-portal sheet. The DNS server resolves every name to us, so the
// probe is guaranteed to land here.
static void WebPortal() {
  oWeb.sendHeader("Location", String("http://") + WiFi.softAPIP().toString() + "/", true);
  oWeb.send(302, "text/plain", "");
}

// Commands (and only commands) arm the web watchdog: watching telemetry from
// the page must not make the firmware treat that phone as the operator.
static void WebCmd() {
  iLastWebAt = g_iNow;
  bWebActive = true;
  if (oWeb.hasArg("s")) iSpeedTarget = CLAMP((int)oWeb.arg("s").toInt(), -iMaxCmd, iMaxCmd);
  if (oWeb.hasArg("t")) iSteerTarget = CLAMP((int)oWeb.arg("t").toInt(), -MAX_STEER, MAX_STEER);
  oWeb.send(200, "text/plain", "ok");
}

static void WebStop() {
  iLastWebAt = g_iNow;
  bWebActive = true;
  StopNow("web");
  oWeb.send(200, "text/plain", "ok");
}

static void WebPing() {
  iLastWebAt = g_iNow;
  oWeb.send(200, "text/plain", "ok");
}

static void WebMax() {
  iLastWebAt = g_iNow;
  if (oWeb.hasArg("v")) {
    iMaxCmd = CLAMP((int)oWeb.arg("v").toInt(), 0, 1000);
    iSpeedTarget = CLAMP(iSpeedTarget, -iMaxCmd, iMaxCmd);
    Serial.printf("max -> %d (web)\n", iMaxCmd);
  }
  oWeb.send(200, "text/plain", "ok");
}

// Drive-mode settings. Like /api/max these do not arm the web watchdog: they
// change how a command is interpreted, they do not issue one. Mode and invert
// stop the wheel first -- the links change meaning under it.
static void WebCfg() {
  iLastWebAt = g_iNow;
  bool bChanged = false;
  if (oWeb.hasArg("mode")) {
    StopNow("mode change (web)");
    iMode = oWeb.arg("mode").toInt() ? MODE_TANK : MODE_BOARD;
    bChanged = true;
  }
  if (oWeb.hasArg("invb")) {
    StopNow("link B inverted (web)");
    bInvertB = oWeb.arg("invb").toInt() != 0;
    bChanged = true;
  }
  if (oWeb.hasArg("dead")) {
    iDeadband = CLAMP((int)oWeb.arg("dead").toInt(), 0, 1000);
    bChanged = true;
  }
  if (bChanged) {
    SaveSettings();
    PrintSettings();
  }
  oWeb.send(200, "text/plain", "ok");
}

static void WebStatus() {
  iLastWebAt = g_iNow;
  char aBuf[448];
  snprintf(aBuf, sizeof(aBuf),
           "{\"s\":%d,\"t\":%d,\"max\":%d,"
           "\"spdL\":%d,\"spdR\":%d,\"volt\":%d,\"ampL\":%d,\"ampR\":%d,\"rx\":%lu,"
           "\"spdL2\":%d,\"spdR2\":%d,\"volt2\":%d,\"ampL2\":%d,\"ampR2\":%d,\"rx2\":%lu,"
           "\"outA\":%d,\"outB\":%d,\"mode\":%u,\"invb\":%d,\"dead\":%d,"
           "\"cut\":%d,\"why\":\"%s\"}",
           iSpeedTarget, iSteerTarget, iMaxCmd,
           (int)oFb.iSpeedL, (int)oFb.iSpeedR, (int)oFb.iVolt,
           (int)oFb.iAmpL, (int)oFb.iAmpR, (unsigned long)iRxFrames,
           (int)oFb2.iSpeedL, (int)oFb2.iSpeedR, (int)oFb2.iVolt,
           (int)oFb2.iAmpL, (int)oFb2.iAmpR, (unsigned long)iRxFrames2,
           iOutA, iOutB, (unsigned)iMode, bInvertB ? 1 : 0, iDeadband,
           bCut ? 1 : 0, bCut ? sCutWhy : "");
  oWeb.send(200, "application/json", aBuf);
}

static void WebSetup() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  delay(100);  // softAPIP() is not valid immediately after softAP()

  oDns.start(53, "*", WiFi.softAPIP());

  oWeb.on("/", WebRoot);
  oWeb.on("/api/cmd", WebCmd);
  oWeb.on("/api/stop", WebStop);
  oWeb.on("/api/ping", WebPing);
  oWeb.on("/api/max", WebMax);
  oWeb.on("/api/cfg", WebCfg);
  oWeb.on("/api/status", WebStatus);
  oWeb.onNotFound(WebPortal);
  oWeb.begin();

  Serial.printf("wifi ap \"%s\" pass \"%s\" -> http://%s/\n",
                AP_SSID, AP_PASS, WiFi.softAPIP().toString().c_str());
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
    iLastRxAt = iLastHostAt = iLastWebAt = millis();
  if (iLastRxAt2) iLastRxAt2 = millis();  // bump only if armed -- never arm link B here
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

  iLastRxAt = iLastHostAt = iLastWebAt = millis();
  if (iLastRxAt2) iLastRxAt2 = millis();  // bump only if armed -- never arm link B here
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
    iLastRxAt = iLastHostAt = iLastWebAt = millis();
  if (iLastRxAt2) iLastRxAt2 = millis();  // bump only if armed -- never arm link B here
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
  iLastRxAt = iLastHostAt = iLastWebAt = millis();
  if (iLastRxAt2) iLastRxAt2 = millis();  // bump only if armed -- never arm link B here
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
  iLastRxAt = iLastHostAt = iLastWebAt = millis();
  if (iLastRxAt2) iLastRxAt2 = millis();  // bump only if armed -- never arm link B here  // the blocking above is not a real timeout
  Serial.println(F("--- end ---"));
}

static int Ramp(int iNow, int iTarget, int iStep) {
  const int iDelta = iTarget - iNow;
  if (iDelta > iStep) return iNow + iStep;
  if (iDelta < -iStep) return iNow - iStep;
  return iTarget;
}

// Remaps a non-zero duty onto [iDeadband, iMaxCmd], keeping its sign. The
// ramp still runs in command space (so the change rate is unchanged); only
// the first tick out of zero lands on a duty the wheel can act on. The
// result never exceeds iMaxCmd, so the clamp still bounds the wire.
static int Deadband(int iCmd) {
  if (iCmd == 0 || iDeadband <= 0 || iMaxCmd <= iDeadband) return iCmd;
  int iOut = iDeadband + (int)((long)abs(iCmd) * (iMaxCmd - iDeadband) / iMaxCmd);
  if (iOut > iMaxCmd) iOut = iMaxCmd;  // iSpeedNow may still be above a just-lowered clamp
  return iCmd < 0 ? -iOut : iOut;
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.println(F("Gen2.x hoverboard console"));
  Serial.printf("link A: uart2 %lu 8N1, rx=GPIO%d tx=GPIO%d\n",
                (unsigned long)BAUDRATE, PIN_RX, PIN_TX);
  Serial.printf("link B: uart1 %lu 8N1, rx=GPIO%d tx=GPIO%d\n",
                (unsigned long)BAUDRATE, PIN_RX2, PIN_TX2);
  LoadSettings();
  Help();
  Serial.println(F("wheel off the ground before you touch 's'."));

  HoverSetupEsp32(oSerialHover, (int)BAUDRATE, PIN_RX, PIN_TX);
  HoverSetupEsp32(oSerialHover2, (int)BAUDRATE, PIN_RX2, PIN_TX2);
  pinMode(LED_BUILTIN, OUTPUT);

  WebSetup();
}

void loop() {
  const unsigned long iNow = millis();
  g_iNow = iNow;  // web handlers below stamp iLastWebAt with this
  digitalWrite(LED_BUILTIN, (iNow % 1000) < 200 ? LOW : HIGH);  // active-LOW on the D32

  PollConsole(iNow);
  oDns.processNextRequest();
  oWeb.handleClient();

  while (Receive(oSerialHover, oFb)) {
    iRxFrames++;
    iLastRxAt = iNow;
  }
  while (Receive(oSerialHover2, oFb2)) {
    iRxFrames2++;
    iLastRxAt2 = iNow;
  }

  // Dead-man: a dead link must not leave a wheel spinning.
  // Board mode: link B only participates once it has ever delivered a frame
  // -- master 2 may simply not be wired yet, and must not veto master 1. Once
  // it has joined, its silence stops everything: two axles must not disagree
  // about driving.
  // Tank mode: link B IS the right wheel, so it must be alive from the start
  // -- one wheel driving alone is a spin, not a degraded mode.
  const bool bAliveA = iLastRxAt && iNow - iLastRxAt <= RX_TIMEOUT_MS;
  const bool bAliveB = iLastRxAt2 && iNow - iLastRxAt2 <= RX_TIMEOUT_MS;
  const bool bVetoB = iMode == MODE_TANK ? !bAliveB : (iLastRxAt2 && !bAliveB);
  if (iNow > QUIET_MS) {
    if (!bAliveA || bVetoB) {
      if (!bCut) {
        bCut = true;
        sCutWhy = !iLastRxAt ? "no telemetry -- board powered on?"
                : !bAliveA   ? "telemetry lost (master 1)"
                : !iLastRxAt2 ? "no telemetry from master 2 -- tank mode needs both links"
                              : "telemetry lost (master 2)";
        StopNow(sCutWhy);
      }
      iSpeedTarget = iSteerTarget = 0;
    } else if (bCut) {
      bCut = false;
      Serial.println(F("telemetry back"));
    }
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

  // Same again for the WiFi page. It heartbeats every 250 ms while open, so a
  // closed tab, a crashed browser or a phone that walked out of range stops
  // the wheel. Armed by the first web command and disarmed when it fires, so
  // it cannot interfere with a serial-only session -- and unlike the serial
  // watchdog it is not opt-in, because the web client is always a program.
  if (bWebActive && iNow - iLastWebAt > WEB_TIMEOUT_MS) {
    bWebActive = false;
    StopNow("web client lost");
  }

  if (iNow < QUIET_MS) return;  // let the link settle before commanding anything

  if (iNow > iNextSend) {
    iNextSend = iNow + SEND_MILLIS;
    iSpeedNow = Ramp(iSpeedNow, iSpeedTarget, RAMP_PER_TICK);
    iSteerNow = Ramp(iSteerNow, iSteerTarget, STEER_RAMP_PER_TICK);

    uint8_t wSend = wState;
    if (iShutoffFrames) {
      iShutoffFrames--;
      wSend |= 128;  // STATE_Shutoff, only ever from the 'k' diagnostic
    }

    if (iMode == MODE_TANK) {
      // Steer is relative to the clamp: full lock at zero speed pivots at
      // +/-iMaxCmd, and neither wheel ever exceeds iMaxCmd. Positive steer
      // is a right turn, so the left wheel gets the extra.
      const int iTurn = (int)((long)iSteerNow * iMaxCmd / 1000);
      iOutA = Deadband(CLAMP(iSpeedNow + iTurn, -iMaxCmd, iMaxCmd));
      iOutB = Deadband(CLAMP(iSpeedNow - iTurn, -iMaxCmd, iMaxCmd));
      if (bInvertB) iOutB = -iOutB;
      HoverSend(oSerialHover, 0, iOutA, wSend, wSend);
      HoverSend(oSerialHover2, 0, iOutB, wSend, wSend);
    } else {
      iOutA = iOutB = Deadband(iSpeedNow);
      HoverSend(oSerialHover, STEER_SIGN * iSteerNow, iOutA, wSend, wSend);
      HoverSend(oSerialHover2, STEER_SIGN * iSteerNow, iOutB, wSend, wSend);
    }
  }

  if (iNow > iNextReport) {
    iNextReport = iNow + REPORT_MILLIS;
    // Fields for master 2 and the per-link wire values are appended AFTER
    // "rx N" -- hoverctl.py's regex has no end anchor, so the line stays
    // parseable by the existing GUI.
    Serial.printf("cmd s%4d>%4d t%4d>%4d | meas %6.2f %6.2f km/h | %5.2f %5.2f A "
                  "| %5.2f V | odo %ld %ld | rx %lu | m2 %6.2f %6.2f km/h "
                  "%5.2f %5.2f A %5.2f V rx2 %lu | out %d %d\n",
                  iSpeedNow, iSpeedTarget, iSteerNow, iSteerTarget,
                  oFb.iSpeedL / 100.0, oFb.iSpeedR / 100.0,
                  oFb.iAmpL / 100.0, oFb.iAmpR / 100.0, oFb.iVolt / 100.0,
                  (long)oFb.iOdomL, (long)oFb.iOdomR, (unsigned long)iRxFrames,
                  oFb2.iSpeedL / 100.0, oFb2.iSpeedR / 100.0,
                  oFb2.iAmpL / 100.0, oFb2.iAmpR / 100.0, oFb2.iVolt / 100.0,
                  (unsigned long)iRxFrames2, iOutA, iOutB);
  }
}
