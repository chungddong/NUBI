"""무선 게임패드로 실차 운전 — Jetson 에서 띄운다 (패드 수신기는 Jetson USB 에).

    python3 pad.py show                이벤트를 그대로 찍는다 (버튼 번호 · 축 값 확인)
    python3 pad.py learn               안내대로 눌러 pad.json (버튼 · 축 배치) 을 만든다
    python3 pad.py drive [--fake]      운전. --fake 는 가짜 ESP32 (바퀴 없이 패드만 시험)
    bash pad_ctl.sh start | stop | status     뒤로 띄우기 (기록 /tmp/pad.log)

조작 (어느 버튼인지는 pad.json):
    START           잡기 — 워치독을 켜고 이 프로그램이 바퀴를 맡는다. 켜자마자 움직이지 않게 처음은 놓은 상태
    SELECT          놓기 — 세우고 워치독을 끈다 (폰 웹 조종으로 돌아갈 수 있게, ISSUES N2)
    B               비상정지 — `x` 하고 놓는다 (잡지 않았어도 `x`). 다시 몰려면 START
    L1 누르는 동안   운전 허용 (데드맨). 떼면 바로 `x`
    왼쪽 스틱 위아래 속도 · 오른쪽 스틱 좌우 조향 (서 있으면 제자리 회전). 두 스틱을 다 놓으면 바로 `x`
    십자 위 / 아래   속도 단계 (LEVELS, robot.json 의 max_cmd 가 상한) · R1 누르는 동안 한 단계 위
    십자 왼 / 오른   (L1 누른 채, 스틱은 놓고) 오도메트리로 제자리 90° 돌기. L1 을 떼거나 스틱을 건드리면 멈춘다
    A               주행 기록 시작 / 끝 — record.py 의 녹화기를 이 프로세스에서 (data/logs/hallway-run/<시각>/):
                    사진 5 Hz · 바퀴 보고 줄 · 웹캠 동영상 (webcam.mkv) · 패드 입력 (commands.csv), 끝나면 preview.mp4 를 뒤에서

안전: 잡은 동안 제어 루프가 돌 때만 하트비트 임대 — 프로그램이 멈추면 1 s 뒤 ESP32 가 선다 · 수신기가 빠지면 세우고
놓는다 · 보고 줄이 0.6 s 끊기면 선다 · 잡을 때나 수신기가 다시 붙은 뒤 L1 이 눌려 있으면 한 번 뗀 뒤에야 움직인다.
폰 웹 조종과 같이 쓰지 않는다 (웹 버튼을 누르는 동안은 시리얼 `x` 가 덮인다, ISSUES R7).

장치는 리눅스 evdev (/dev/input/eventN) 를 표준 라이브러리로 직접 읽는다 (python-evdev 없이). 권한: 바탕화면에 로그인한
사용자는 udev uaccess 로 읽을 수 있다. 로그인 없이 쓰려면 input 그룹에 넣는다.
"""

import argparse
import csv
import json
import math
import os
import struct
import subprocess
import sys
import threading
import time
from pathlib import Path

from motion import wrap
from odometry import Odometry

HERE = Path(__file__).resolve().parent
PAD_JSON = HERE / "pad.json"
CFG = json.loads((HERE / "robot.json").read_text(encoding="utf-8"))

EV_KEY, EV_ABS = 1, 3
EVENT = struct.Struct("llHHi")           # struct input_event — 64 비트 리눅스: timeval 16 B + type · code · value
BTN_JOY = range(0x120, 0x140)            # BTN_JOYSTICK · BTN_GAMEPAD — 키보드 · 마우스 · 웹캠 버튼과 가른다
LEVELS = (100, 150, 200, 250, 300)
STILL = 50                               # 펌웨어가 |s| · |t| < 50 을 0 으로 친다 (main.cpp Still) — 스틱을 조금만 밀어도 움직이게 여기서 시작
DEADZONE = 0.15
TURN_GAIN = 0.8                          # 조향을 다 꺾으면 바퀴 듀티로 속도 단계의 이만큼 (제자리 회전 세기)
TICK = 0.02

KEY_NAMES = dict(enumerate(("TRIGGER", "THUMB", "THUMB2", "TOP", "TOP2", "PINKIE", "BASE", "BASE2", "BASE3", "BASE4",
                            "BASE5", "BASE6", "", "", "", "DEAD", "A", "B", "C", "X", "Y", "Z", "TL", "TR", "TL2", "TR2",
                            "SELECT", "START", "MODE", "THUMBL", "THUMBR"), 0x120))
ABS_NAMES = {0: "X", 1: "Y", 2: "Z", 3: "RX", 4: "RY", 5: "RZ", 6: "THROTTLE", 7: "RUDDER", 9: "GAS", 10: "BRAKE",
             16: "HAT0X", 17: "HAT0Y"}


def _ior(nr, size):
    return (2 << 30) | (size << 16) | (ord("E") << 8) | nr        # linux/input.h _IOR('E', nr, size)


EVIOCGNAME = _ior(0x06, 256)
EVIOCGKEY = _ior(0x18, 96)               # 지금 눌린 버튼 (KEY_MAX 0x2ff → 96 B)
EVIOCGBIT_ABS = _ior(0x20 + EV_ABS, 8)


def EVIOCGABS(code):
    return _ior(0x40 + code, 24)         # struct input_absinfo: value min max fuzz flat resolution


def list_pads():
    """/proc/bus/input/devices 에서 게임패드 (조이스틱 버튼 + X · Y 축) → [(이름, /dev/input/eventN)]."""
    word = struct.calcsize("l") * 8
    out = []
    for block in Path("/proc/bus/input/devices").read_text().split("\n\n"):
        name, ev, keys, absb = "", None, 0, 0
        for line in block.splitlines():
            if line.startswith("N: Name="):
                name = line[8:].strip('"')
            elif line.startswith("H: Handlers="):
                ev = next((h for h in line[12:].split() if h.startswith("event")), None)
            elif line.startswith("B: KEY="):
                for w in line[7:].split():
                    keys = (keys << word) | int(w, 16)
            elif line.startswith("B: ABS="):
                for w in line[7:].split():
                    absb = (absb << word) | int(w, 16)
        if ev and absb & 0b11 and any(keys >> b & 1 for b in BTN_JOY):
            out.append((name, "/dev/input/" + ev))
    return out


class Pad:
    """evdev 게임패드 하나. poll() 로 상태를 갱신하고 pressed() · axis() 로 묶음 (pad.json) 을 읽는다."""

    def __init__(self, path):
        import fcntl
        self.path = path
        self.fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
        self.name = fcntl.ioctl(self.fd, EVIOCGNAME, bytes(256)).split(b"\0")[0].decode("utf-8", "replace")
        kb = fcntl.ioctl(self.fd, EVIOCGKEY, bytes(96))
        self.keys = {c: 1 for c in range(len(kb) * 8) if kb[c // 8] >> (c % 8) & 1}
        absbits = int.from_bytes(fcntl.ioctl(self.fd, EVIOCGBIT_ABS, bytes(8)), "little")
        self.info, self.abs = {}, {}
        for c in range(64):
            if absbits >> c & 1:
                v, lo, hi, *_ = struct.unpack("6i", fcntl.ioctl(self.fd, EVIOCGABS(c), bytes(24)))
                self.info[c], self.abs[c] = (lo, hi), v
        self.last = time.monotonic()      # 마지막 이벤트 시각

    def poll(self, timeout):
        """이벤트를 읽어 상태를 갱신하고 [(type, code, value)] 를 돌려준다. 수신기가 빠지면 OSError (ENODEV)."""
        import select
        if not select.select([self.fd], [], [], timeout)[0]:
            return []
        try:
            data = os.read(self.fd, EVENT.size * 64)
        except BlockingIOError:
            return []
        evs = []
        for i in range(0, len(data) - EVENT.size + 1, EVENT.size):
            _, _, typ, code, val = EVENT.unpack_from(data, i)
            if typ == EV_KEY:
                self.keys[code] = 1 if val else 0          # 2 (자동 반복) 도 눌림
            elif typ == EV_ABS:
                self.abs[code] = val
            else:
                continue
            evs.append((typ, code, val))
        if evs:
            self.last = time.monotonic()
        return evs

    def close(self):
        try:
            os.close(self.fd)
        except OSError:
            pass

    def norm(self, code):
        """축 값 → −1 .. 1 (가운데 0). 십자 (HAT) 는 −1 · 0 · 1 그대로."""
        lo, hi = self.info.get(code, (-1, 1))
        half = (hi - lo) / 2 or 1
        return max(-1.0, min(1.0, (self.abs.get(code, (lo + hi) / 2) - (lo + hi) / 2) / half))

    def pressed(self, b):
        """버튼 묶음: {"key": 코드} 또는 축으로 나오는 버튼 (십자 · 트리거) {"abs": 코드, "dir": ±1}."""
        if "key" in b:
            return self.keys.get(b["key"], 0) == 1
        return self.norm(b["abs"]) * b["dir"] > 0.5

    def axis(self, b):
        """축 묶음 {"abs": 코드, "sign": ±1} → −1 .. 1, 가운데 DEADZONE 은 0 이고 그 밖을 0 .. 1 로 다시 편다."""
        v = self.norm(b["abs"]) * b["sign"]
        return 0.0 if abs(v) < DEADZONE else math.copysign((abs(v) - DEADZONE) / (1 - DEADZONE), v)


def describe(b):
    if "key" in b:
        return f"버튼 {KEY_NAMES.get(b['key']) or '?'} ({b['key']})"
    pm = "+" if b.get("dir", b.get("sign")) > 0 else "−"
    return f"축 {ABS_NAMES.get(b['abs'], '?')} ({b['abs']}) {pm}"


def open_pad(dev=None, name=None, quiet=False):
    """패드를 연다. dev 를 주면 그 장치, 아니면 이름이 name 인 것을 먼저, 없으면 처음 찾은 게임패드. 못 열면 None."""
    pads = [("", dev)] if dev else sorted(list_pads(), key=lambda p: p[0] != name)
    for n, path in pads:
        try:
            return Pad(path)
        except PermissionError:
            if not quiet:
                print(f"{path} ({n}) 를 읽을 권한이 없다 — 바탕화면에 로그인하거나 `sudo usermod -aG input $USER` 뒤 다시 로그인")
        except OSError as e:
            if not quiet:
                print(f"{path} 열기 실패: {e}")
    if not pads and not quiet:
        print("게임패드가 안 보인다 — 수신기 · 패드 전원 · `cat /proc/bus/input/devices` 확인")
    return None


# ---------------------------------------------------------------- 운전

class Teleop:
    """패드 상태 → 바퀴 명령. 틱마다 update(pad, now) — 패드가 없으면 pad=None.
    pad.json 에 없는 묶음 (예전 파일의 estop · boost · left · right · record) 은 쓰지 않는다."""

    def __init__(self, link, binds, cfg, level=150, log=print, on_record=None):
        self.link, self.b, self.cfg, self.log = link, binds, cfg, log
        self.max_cmd = int(cfg["max_cmd"])
        self.levels = [v for v in LEVELS if v <= self.max_cmd] or [self.max_cmd]
        self.li = min(range(len(self.levels)), key=lambda i: abs(self.levels[i] - level))
        self.armed = False
        self.hold = True                  # L1 을 한 번 떼야 움직인다 (잡을 때 · 패드가 다시 붙을 때)
        self.moving = False               # 0 아닌 명령을 보낸 뒤 아직 x 를 안 보냄
        self.sent, self.sent_at = None, 0.0
        self.prev = {}
        self.dead = self.boost = False
        self.thr = self.st = 0.0          # 스틱 (데드존 뒤) — 녹화의 commands.csv 에
        self.turn = None                  # 90° 돌기 중이면 {"name", "yaw0", "target", "t_end"}
        self.odo = Odometry(cfg)          # 돌기에 쓰는 방향 (앞축 홀 스텝 — 돌 때 미끄러지면 덜 돈다)
        self._rep = None
        self.on_record = on_record        # A 버튼

    @property
    def base_level(self):
        return self.levels[self.li]

    @property
    def level(self):
        return self.levels[min(self.li + self.boost, len(self.levels) - 1)]     # R1 누르는 동안 한 단계 위

    def _pressed(self, pad, name):
        b = self.b.get(name)
        return b is not None and pad.pressed(b)

    def _edge(self, pad, name):
        now = self._pressed(pad, name)
        was = self.prev.get(name, now)    # 처음 본 틱은 가장자리가 아니다 (켤 때부터 눌려 있던 버튼)
        self.prev[name] = now
        return now and not was

    def arm(self):
        self.link.arm(self.max_cmd)       # x · m · w 1
        self.armed, self.hold = True, True
        self.log(f"잡음 — L1 을 누른 채 스틱으로 (상한 {self.max_cmd}, 단계 {self.base_level})")

    def release(self, why):
        self.link.stop()
        if self.armed:
            self.link.send("w 0")
        self.armed, self.moving, self.sent, self.turn = False, False, None, None
        self.log(f"놓음 ({why}) — 워치독 끔, 폰 웹 조종으로 돌아갈 수 있다")

    def _halt(self):
        if self.moving:
            self.link.stop()
        self.moving, self.sent = False, None

    def _send(self, s, t, now):
        if (s, t) != self.sent or now - self.sent_at > 0.25:
            self.link.drive(s, t)
            self.sent, self.sent_at = (s, t), now
        self.moving = True
        return s, t

    def update(self, pad, now):
        """→ 보낸 (s, t), 서 있으면 (0, 0)."""
        r = self.link.report
        if r is not None and r is not self._rep:
            self.odo.update(r.odo)
            self._rep = r
        if pad is None:
            if self.armed:
                self.release("패드 수신기 빠짐")
            self.prev.clear()
            self.hold, self.dead, self.boost = True, False, False
            self.thr = self.st = 0.0
            return 0, 0
        e = {n: self._edge(pad, n) for n in ("estop", "select", "start", "faster", "slower", "left", "right", "record")}
        if e["estop"]:
            if self.armed:
                self.release("B 비상정지 — 다시 몰려면 START")
            else:
                self.link.stop()          # 잡지 않았어도 x (폰 웹 버튼을 누르고 있으면 덮인다, ISSUES R7)
                self.log("B 비상정지 (x)")
        if e["select"] and self.armed:
            self.release("SELECT")
        if e["start"] and not self.armed:
            self.arm()
        if e["faster"] and self.li < len(self.levels) - 1:
            self.li += 1
            self.log(f"단계 {self.base_level}")
        if e["slower"] and self.li > 0:
            self.li -= 1
            self.log(f"단계 {self.base_level}")
        if e["record"] and self.on_record:
            self.on_record()
        self.dead = self._pressed(pad, "deadman")
        self.boost = self._pressed(pad, "boost")
        self.thr, self.st = pad.axis(self.b["throttle"]), pad.axis(self.b["steer"])
        if not self.dead:
            self.hold = False
        if not self.armed:
            return 0, 0
        self.link.lease(0.3)              # 이 루프가 도는 동안만 하트비트
        if not self.dead or self.hold or not self.link.fresh(0.6):
            if self.turn:
                self._end_turn("L1 뗌" if not self.dead else "보고 줄 끊김")
            self._halt()
            return 0, 0
        thr, st = self.thr, self.st
        if (e["left"] or e["right"]) and self.turn is None and thr == 0 and st == 0:
            self._start_turn(now, 1 if e["left"] else -1)
        if self.turn is not None:
            if thr == 0 and st == 0:
                return self._turn_step(now)
            self._end_turn("스틱으로 넘겨받음")
        if thr == 0 and st == 0:
            self._halt()                  # 손을 놓으면 램프 없이 바로 선다 (ISSUES R5 와 같은 뜻)
            return 0, 0
        lv = self.level
        a = 0.0 if thr == 0 else STILL + abs(thr) * (lv - STILL)      # 앞뒤 세기 (바퀴 듀티)
        # 조향 b (바퀴 듀티): 서 있으면 단계의 TURN_GAIN 배까지 제자리 회전. 가면서는 바깥 바퀴를 a 로 두고 안쪽 바퀴만 줄인다
        # — 스틱 반이면 안쪽 바퀴 0, 끝까지면 거꾸로 a (제자리 회전). 2026-10-14 밖에서 잰 것: 예전처럼 속도에 조향을 더하기만
        # 하면 듀티 (280, 120) 에 바퀴가 3.85 / 2.84 km/h — 안쪽 바퀴가 직진 속도의 3 배로 끌려가 거의 돌지 않았다.
        b = abs(st) * max(a, lv * TURN_GAIN)
        a = max(0.0, a - b)                                          # 바깥 바퀴 = (a − b) + b = 처음 a
        s = 0 if a == 0 else round(math.copysign(max(STILL, a), thr))   # 펌웨어가 50 밑을 0 으로 치므로 가는 중이면 50 이상
        t = 0 if st == 0 else round(math.copysign(min(1000, max(STILL, b * 1000 / self.max_cmd)), st))  # t 는 클램프 대비 ‰
        return self._send(s, t, now)

    # ---- 십자 왼 / 오른: 제자리 90°. motion.turn_by 와 같은 규칙을 한 틱씩 — 패드를 계속 읽어 L1 · 스틱으로 끊을 수 있게
    def _start_turn(self, now, sign):
        self.turn = {"name": "왼쪽" if sign > 0 else "오른쪽", "yaw0": self.odo.yaw,
                     "target": wrap(self.odo.yaw + sign * math.pi / 2),
                     "t_end": now + 2.0 + 90 / self.cfg["turn_min_dps"]}
        self.log(f"{self.turn['name']} 90° 돌기 — L1 을 떼거나 스틱을 건드리면 멈춘다")

    def _turn_step(self, now):
        c = self.cfg
        err = wrap(self.turn["target"] - self.odo.yaw)
        if abs(err) < math.radians(c["turn_tol_deg"]):
            return self._end_turn("도착")
        if now > self.turn["t_end"]:
            return self._end_turn("시간 초과")
        mag = c["turn_cmd"] if abs(err) > math.radians(20) else c["turn_cmd_slow"]
        return self._send(0, -mag if err > 0 else mag, now)    # + 조향 = 오른쪽, 왼쪽 (+ 각도) 으로 돌 때는 −

    def _end_turn(self, why):
        tn, self.turn = self.turn, None
        self._halt()
        self.log(f"{tn['name']} 90° 돌기 끝 ({why}) — 오도메트리 {math.degrees(wrap(self.odo.yaw - tn['yaw0'])):+.0f}°")
        return 0, 0


class PadRecorder:
    """A 버튼 녹화 — record.py 의 Recorder 를 이 프로세스에서 패드와 같은 바퀴 링크로 돌린다. 시리얼은 한 프로그램만
    열고, 두 프로그램이 한 포트를 나눠 읽으면 보고 줄이 깨진다. 카메라 (Xtion · 웹캠) 는 처음 누를 때 뒤에서 연다."""

    def __init__(self, link, log):
        self.link, self.log = link, log
        self.rc, self.opening = None, False
        self._rec, self._cmd_f, self._cmd_w = None, None, None
        self._last_row, self._last_at, self._rows = None, 0.0, 0

    @property
    def recording(self):
        return bool(self.rc is not None and self.rc.rec)

    def toggle(self):
        if self.opening:
            self.log("카메라 여는 중 — 열리면 바로 녹화가 시작된다")
        elif self.rc is None:
            self.opening = True
            self.log("녹화 준비 — 카메라 여는 중 (몇 초)")
            threading.Thread(target=self._open, daemon=True).start()
        elif self.rc.rec:
            self.rc.stop("A 버튼")
        else:
            err = self.rc.start()
            if err:
                self.log(f"녹화 못 함: {err}")

    def _open(self):
        try:
            from record import REPO, Recorder
            a = argparse.Namespace(hz=5.0, minutes=20.0, webcam="auto", webcam_size="1280x720", no_xtion=False,
                                   no_wheels=True, no_video=False, serial=None, out=REPO / "data/logs/hallway-run")
            a.out.mkdir(parents=True, exist_ok=True)
            rc = Recorder(a, link=self.link)
            if rc.xt is None and rc.cam is None:
                rc.close()
                self.log("카메라가 하나도 안 열려 녹화하지 않는다")
                return
            self.rc = rc
            rc.start()
        except Exception as e:
            self.log(f"녹화 준비 실패: {e}")
        finally:
            self.opening = False

    def tick(self, tel, s, t):
        """메인 루프에서 틱마다. 녹화 중이면 패드 입력과 보낸 명령을 commands.csv 에 (바뀔 때, 아니면 1 s 마다).
        녹화가 끝나면 (A · 20 분) review.py 로 preview.mp4 (세 카메라 나란히, 2 배속) 를 뒤에서 만든다."""
        r = self.rc.rec if self.rc is not None else None
        if r is not self._rec:
            ended = self._rec
            self._close_cmd()
            self._rec = r
            if ended is not None:
                self._preview(ended["out"])
            if r is not None:
                self._cmd_f = open(r["out"] / "commands.csv", "w", newline="", encoding="utf-8")
                self._cmd_w = csv.writer(self._cmd_f)
                self._cmd_w.writerow(["t", "armed", "l1", "level", "throttle", "steer", "s", "t_cmd", "turn"])
                self._last_row, self._rows = None, 0
        if r is None:
            return
        now = time.monotonic()
        row = [int(tel.armed), int(tel.dead), tel.level, round(tel.thr, 3), round(tel.st, 3), s, t, int(bool(tel.turn))]
        if row != self._last_row or now - self._last_at > 1.0:
            self._cmd_w.writerow([f"{now - r['t0']:.3f}", *row])
            self._last_row, self._last_at, self._rows = row, now, self._rows + 1
            if self._rows % 50 == 0:
                self._cmd_f.flush()

    def _close_cmd(self):
        if self._cmd_f is not None:
            self._cmd_f.close()
            self._cmd_f = self._cmd_w = None

    def _preview(self, out):
        if os.name != "posix":
            return
        try:
            logf = open(out / "preview.log", "w", encoding="utf-8")
            subprocess.Popen(["nice", "-n", "10", sys.executable, str(HERE / "review.py"), str(out)], cwd=str(HERE),
                             stdin=subprocess.DEVNULL, stdout=logf, stderr=subprocess.STDOUT, start_new_session=True)
            self.log(f"미리보기 동영상을 뒤에서 만든다 → {out / 'preview.mp4'}")
        except Exception as e:
            self.log(f"미리보기 못 만듦: {e}")

    def close(self):
        if self.rc is not None:
            ended = self._rec if self.rc.rec is not None else None
            self.rc.close()               # 녹화 중이면 파일 · 동영상을 닫는다
            self._close_cmd()
            if ended is not None:
                self._preview(ended["out"])


def load_binds():
    if not PAD_JSON.exists():
        raise SystemExit(f"{PAD_JSON.name} 가 없다 — 먼저 `python3 pad.py learn`")
    return json.loads(PAD_JSON.read_text(encoding="utf-8"))


def cmd_drive(a):
    from drive import stop_on_signals
    from wheel_link import WheelLink, open_serial
    cfg = load_binds()
    stop_on_signals()
    if a.fake:
        from fake import FakeWheelSerial
        from hallway_map import HallwayMap, find_scene
        ser = FakeWheelSerial(HallwayMap(find_scene()).segs, deadband=100)
    else:
        ser = open_serial(a.serial)
    link = WheelLink(ser)
    pad, next_scan, warned, code = None, 0.0, False, 0
    log = lambda m: print(time.strftime("%H:%M:%S"), m, flush=True)    # noqa: E731
    rec = PadRecorder(link, log)
    try:
        r = link.wait_ready(need_board=False)
        print(f"바퀴 제어기 {link.port}: {r.volt:.1f} V, rx {r.rx}. START = 잡기 · L1 누른 채 스틱 · SELECT = 놓기 · B = 비상정지",
              flush=True)
        tel = Teleop(link, cfg["binds"], CFG, a.level, log=log, on_record=rec.toggle)
        ev_seen = time.monotonic()
        shown, shown_at = None, 0.0
        while True:
            now = time.monotonic()
            if pad is None and now >= next_scan:
                next_scan = now + 1.0
                pad = open_pad(a.dev, cfg.get("name"), quiet=warned)
                warned = True
                if pad:
                    print(time.strftime("%H:%M:%S"), f"패드: {pad.name} ({pad.path})", flush=True)
            if pad is not None:
                try:
                    pad.poll(TICK)
                except OSError as e:
                    print(time.strftime("%H:%M:%S"), f"패드 끊김: {e}", flush=True)
                    pad.close()
                    pad = None
            else:
                time.sleep(TICK)
            s, t = tel.update(pad, time.monotonic())
            rec.tick(tel, s, t)
            for et, line in list(link.events):            # 펌웨어 알림 (STOP · host back …), 명령 메아리는 빼고
                if et > ev_seen and not line.startswith(("speed ->", "steer ->", "max ->")):
                    print(time.strftime("%H:%M:%S"), "ESP32:", line, flush=True)
            if link.events:
                ev_seen = max(ev_seen, link.events[-1][0])
            state = (tel.armed, tel.dead, tel.hold, tel.level, bool(tel.turn), rec.recording, s, t)
            if state != shown and time.monotonic() - shown_at > 0.25:
                rr = link.report
                print(time.strftime("%H:%M:%S"), f"{'잡음' if tel.armed else '놓음'} · L1 {'●' if tel.dead else '○'}"
                      f"{' (한 번 떼기)' if tel.armed and tel.hold and tel.dead else ''} · 단계 {tel.level}"
                      f"{' (R1)' if tel.boost else ''} · s {s:+4d} t {t:+5d}{' · 90° 돌기' if tel.turn else ''}"
                      f"{' · ● 녹화' if rec.recording else ''}" + (f" · {rr.volt:.1f} V" if rr else ""), flush=True)
                shown, shown_at = state, time.monotonic()
            if link.error:
                raise RuntimeError(f"시리얼 끊김: {link.error}")
    except KeyboardInterrupt:
        print("끝", flush=True)
    except Exception:
        import traceback
        traceback.print_exc()
        code = 1
    finally:
        link.close()
        if pad:
            pad.close()
        if rec.rc is not None:
            rec.close()
            sys.stdout.flush()
            sys.stderr.flush()
            os._exit(code)                # OpenNI 정리 중 멈추는 일을 피한다 (record.py 와 같은 이유)
    if code:
        raise SystemExit(code)


# ---------------------------------------------------------------- 확인 · 배치 익히기

def cmd_show(a):
    pad = open_pad(a.dev)
    if not pad:
        raise SystemExit(1)
    print(f"{pad.name} ({pad.path})")
    print("축: " + ", ".join(f"{ABS_NAMES.get(c, '?')}({c}) {lo}..{hi} = {pad.abs[c]}" for c, (lo, hi) in pad.info.items()))
    print("지금 눌린 버튼: " + (", ".join(f"{KEY_NAMES.get(c) or '?'}({c})" for c, v in pad.keys.items() if v) or "없음"), flush=True)
    t_end = time.monotonic() + a.secs if a.secs else math.inf
    try:
        while time.monotonic() < t_end:
            for typ, code, val in pad.poll(0.2):
                if typ == EV_KEY:
                    print(f"버튼 {KEY_NAMES.get(code) or '?'}({code}) {'누름' if val else '뗌'}", flush=True)
                else:
                    print(f"축 {ABS_NAMES.get(code, '?')}({code}) = {val} ({pad.norm(code):+.2f})", flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        pad.close()


STEPS = (
    ("deadman", "button", "L1 — 누르고 있는 동안만 움직인다 (데드맨)"),
    ("start", "button", "START — 잡기 (패드로 조종 시작)"),
    ("select", "button", "SELECT — 놓기 (폰 웹 조종으로 돌려주기)"),
    ("faster", "button", "십자 위 — 속도 단계 올리기"),
    ("slower", "button", "십자 아래 — 속도 단계 내리기"),
    ("boost", "button", "R1 — 누르는 동안 한 단계 빠르게"),
    ("estop", "button", "B — 비상정지 (세우고 놓기)"),
    ("left", "button", "십자 왼쪽 — 제자리 왼쪽 90°"),
    ("right", "button", "십자 오른쪽 — 제자리 오른쪽 90°"),
    ("record", "button", "A — 주행 기록 시작 / 끝"),
    ("throttle", "axis", "왼쪽 스틱을 위로 끝까지 밀었다 놓기 — 속도"),
    ("steer", "axis", "오른쪽 스틱을 오른쪽으로 끝까지 밀었다 놓기 — 조향"),
)


def _wait_input(pad, kind, base):
    t0 = time.monotonic()
    reminded = False
    while True:
        evs = pad.poll(0.1)
        if not reminded and time.monotonic() - t0 > 12:
            print("  (입력이 안 온다 — 패드가 잠들었으면 아무 버튼으로 깨우고, 이 버튼이 안 잡히면 다른 버튼을 눌러도 된다)", flush=True)
            reminded = True
        keys = [c for typ, c, v in evs if typ == EV_KEY and v == 1]
        if kind == "button" and keys:
            b = {"key": keys[0]}
            break
        # 버튼 단계에서는 십자 (HAT, −1 .. 1) 만 축으로 받는다 — 스틱을 건드려 버튼 자리에 묶이지 않게 (2026-10-14)
        moved = [(c, pad.norm(c) - base.get(c, 0.0)) for typ, c, _ in evs
                 if typ == EV_ABS and (kind == "axis" or pad.info.get(c) == (-1, 1))]
        moved = [m for m in moved if abs(m[1]) > 0.6]
        if moved:
            c, d = max(moved, key=lambda m: abs(m[1]))
            b = {"abs": c, ("dir" if kind == "button" else "sign"): 1 if d > 0 else -1}
            break
    quiet = time.monotonic()                           # 다 놓고 0.4 s 조용할 때까지
    while time.monotonic() - quiet < 0.4:
        pad.poll(0.05)
        if any(pad.keys.values()) or any(abs(pad.norm(c) - base.get(c, 0.0)) > 0.3 for c in pad.info):
            quiet = time.monotonic()
    return b


def _clash(b, kind, binds):
    for k, v in binds.items():
        if "key" in b and v.get("key") == b["key"]:
            return k
        if "abs" in b and v.get("abs") == b["abs"] and (kind == "axis" or "sign" in v or v.get("dir") == b.get("dir")):
            return k
    return None


def cmd_learn(a):
    pad = open_pad(a.dev)
    if not pad:
        raise SystemExit(1)
    print(f"{pad.name} ({pad.path})")
    print("패드에서 손을 떼고 잠깐 기다리세요. MODE 불이 켜져 있어야 (아날로그) 스틱이 축으로 나옵니다.", flush=True)
    t_end = time.monotonic() + 1.5
    while time.monotonic() < t_end:
        pad.poll(0.05)
    base = {c: pad.norm(c) for c in pad.info}
    binds = {}
    try:
        for key, kind, prompt in STEPS:
            while True:
                print(f"\n▶ {prompt}", flush=True)
                b = _wait_input(pad, kind, base)
                other = _clash(b, kind, binds)
                if other is None:
                    break
                print(f"  {describe(b)} 는 이미 '{other}' 에 썼다.", end=" ")
                if kind == "axis":
                    print("스틱이 십자와 같은 축으로 나오면 MODE 를 눌러 아날로그로 바꾸고 learn 을 처음부터.", flush=True)
                else:
                    print("다른 버튼을 누르세요.", flush=True)
            binds[key] = b
            print(f"  → {describe(b)}", flush=True)
    except KeyboardInterrupt:
        print("\n그만 — 저장하지 않음")
        return
    finally:
        pad.close()
    PAD_JSON.write_text(json.dumps({"name": pad.name, "learned": time.strftime("%Y-%m-%d"), "binds": binds},
                                   ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"\n저장: {PAD_JSON}. 다음은 바퀴 없이 `python3 pad.py drive --fake`.")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mode", nargs="?", default="drive", choices=("drive", "show", "learn"))
    ap.add_argument("--dev", help="/dev/input/eventN (기본: 찾은 게임패드)")
    ap.add_argument("--serial")
    ap.add_argument("--fake", action="store_true", help="가짜 ESP32")
    ap.add_argument("--level", type=int, default=150, help="처음 속도 단계")
    ap.add_argument("--secs", type=float, default=0, help="show 를 이 시간 뒤 끝냄 (0 = Ctrl+C 까지)")
    a = ap.parse_args()
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    {"drive": cmd_drive, "show": cmd_show, "learn": cmd_learn}[a.mode](a)


if __name__ == "__main__":
    main()
