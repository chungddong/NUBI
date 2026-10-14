"""복도 주행 녹화 — Xtion 컬러 · 깊이, 웹캠, ESP32 보고 줄 (앞 · 뒤축 오도메트리) 을 같은 시계로 남긴다.
바퀴에는 아무 명령도 보내지 않는다 (보고 줄만 읽음). 차는 다른 수단 (ESP32 웹 페이지 등) 으로 몰거나 민다.
게임패드로 몰 때는 이것을 따로 띄우지 않고 pad.py 의 A 버튼으로 녹화한다 — 같은 프로세스에서 패드의 바퀴 링크를 같이
읽는다 (시리얼은 한 프로그램만 연다).

    python3 record.py                 웹 화면 http://192.168.127.2:8093 — 카메라 실시간 보기, 녹화 시작 / 끝 버튼
    python3 record.py --now           화면 없이 바로 녹화 (--minutes 뒤 스스로 끝)
    bash record_ctl.sh start | stop | status        (Jetson 에서 뒤로 띄우기, 기록 /tmp/record.log)

웹 화면은 유선 직결 주소에만 연다 (학교 Wi-Fi 에서는 안 보인다). 이더넷을 뽑아 화면이 끊겨도 녹화는 계속되고,
녹화를 시작하고 --minutes (기본 20 분) 이 지나면 스스로 끝낸다.

결과: <repo>/data/logs/hallway-run/<시각>/ (git 에 올리지 않음 — 사람이 찍힌다)
    xtion/000123.jpg · xtion_depth/000123.png (uint16 mm, 컬러와 같은 시점) · webcam/000123.jpg
    frames.csv   프레임마다 시각 · 저장한 사진의 찍힌 시각 · 그때의 오도메트리
    wheels.csv   ESP32 보고 줄 전부 (200 ms) — 멈춘 구간 (문 앞) 은 나중에 이것으로 찾는다
    webcam.mkv   웹캠 동영상 (받는 대로 ~30 fps, H.264 x264 — 사람이 볼 것). 프레임 시각은 webcam_video.csv.
                 mkv 는 전원이 그냥 꺼져도 그때까지 열린다 (mp4 는 끝마무리를 못 하면 통째로 못 연다). --no-video 로 끔
    meta.json    카메라 크기 · Xtion 내부값 · robot.json · 옵션 · clock_synced (Jetson 은 시계 배터리가 없어 네트워크 없이
                 켜면 시계가 마지막 저장 시각부터 간다 — "no" 면 폴더 이름 · start 시각이 실제와 다르다)
"""

import argparse
import csv
import json
import os
import shutil
import subprocess
import sys
import threading
import time
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]

from drive import stop_on_signals  # noqa: E402


def find_webcam():
    """UVC 카메라 중 HP60C (ASJ ZNX_NVT) 가 아닌 첫 캡처 장치."""
    base = Path("/sys/class/video4linux")
    for d in sorted(base.glob("video*"), key=lambda p: int(p.name[5:])):
        name = (d / "name").read_text().strip() if (d / "name").exists() else ""
        if "ZNX" in name:
            continue
        idx = (d / "index").read_text().strip() if (d / "index").exists() else "0"
        if idx != "0":                                   # 같은 카메라의 메타데이터 노드는 건너뛴다
            continue
        return f"/dev/{d.name}", name
    return None, None


class Webcam:
    """MJPG 로 열고 뒤에서 계속 읽어 마지막 프레임만 들고 있는다 (버퍼에 쌓인 옛 프레임을 쓰지 않게)."""

    def __init__(self, dev, name="", width=1280, height=720, fps=30):
        self.dev, self.name = dev, name
        self.cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        ok, f = self.cap.read()
        if not ok:
            self.cap.release()
            raise RuntimeError(f"웹캠 {dev} 에서 프레임이 안 온다")
        self.size = (f.shape[1], f.shape[0])
        self.latest, self.t, self.n = f, time.monotonic(), 1
        self._alive = True
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        fails = 0
        while self._alive:
            ok, f = self.cap.read()
            if ok:
                self.latest, self.t, self.n, fails = f, time.monotonic(), self.n + 1, 0
            else:
                fails += 1
                time.sleep(0.02)
                if fails > 100:                          # 2 s 넘게 안 옴 — 빠졌다고 본다
                    self._alive = False

    @property
    def alive(self):
        return self._alive

    def close(self):
        self._alive = False
        time.sleep(0.05)
        self.cap.release()


def clock_synced():
    """시계가 NTP 로 맞춰졌나 — "yes" · "no" · "" (모름)."""
    try:
        return subprocess.run(["timedatectl", "show", "-p", "NTPSynchronized", "--value"], capture_output=True, text=True,
                              timeout=2).stdout.strip()
    except Exception:
        return ""


class Recorder:
    def __init__(self, a, link=None):
        """link: 다른 프로그램이 쥔 WheelLink 를 같이 읽을 때 (pad.py 의 A 버튼). 이때는 포트를 열지도 닫지도 않는다."""
        self.a = a
        self.own_link = link is None
        self.lock = threading.Lock()
        self.xt = self.cam = self.link = None
        self.rec = None                                  # 녹화 중이면 dict (폴더 · 파일 · 프레임 수 · 시작 시각)
        self.last_saved = None
        self.fps = {"xtion": 0.0, "webcam": 0.0}
        self.xtion_error = self.webcam_error = self.wheels_error = ""
        self.meta = dict(hz=a.hz, argv=sys.argv[1:], robot=json.loads((HERE / "robot.json").read_text(encoding="utf-8")))
        if not a.no_xtion:
            try:
                from xtion import XtionCamera
                self.xt = XtionCamera()
                f = self.xt.read(5.0)
                if f is None:
                    raise RuntimeError("첫 프레임이 5 s 안에 안 온다")
                self.meta["xtion"] = dict(size=[f.bgr.shape[1], f.bgr.shape[0]], K_fx_fy_cx_cy=list(f.K),
                                          mirror=list(self.xt.mirror), hfov=self.xt.hfov, vfov=self.xt.vfov,
                                          depth="uint16 mm, 컬러와 같은 시점")
                print(f"Xtion {f.bgr.shape[1]}x{f.bgr.shape[0]}, fx {f.K[0]:.1f}", flush=True)
            except Exception as e:
                self.xt, self.xtion_error = None, str(e)
                print(f"Xtion 못 엶: {e}", flush=True)
        if a.webcam != "none":
            self.open_webcam()
        if link is not None:
            self.link = link
            self.meta["wheels"] = dict(port=link.port, shared="pad.py")
        elif not a.no_wheels:
            try:
                from wheel_link import WheelLink, open_serial
                self.link = WheelLink(open_serial(a.serial))
                self.link.wait_ready(need_board=False)
                self.meta["wheels"] = dict(port=self.link.port)
                print(f"바퀴 제어기 {self.link.port} (읽기만)", flush=True)
            except Exception as e:                           # 포트를 다른 프로그램 (bench.py) 이 쥐고 있거나 없음
                self.link, self.wheels_error = None, str(e)
                print(f"바퀴 제어기 못 엶 — 오도메트리 없이: {e}", flush=True)
        self._alive = True
        threading.Thread(target=self._loop, daemon=True).start()

    # ---- 장치
    def open_webcam(self):
        if self.cam is not None and self.cam.alive:
            return "이미 열려 있다"
        dev, name = (find_webcam() if self.a.webcam == "auto" else (self.a.webcam, ""))
        if dev is None:
            self.cam, self.webcam_error = None, "웹캠이 안 보인다 (HP60C 말고 다른 UVC 카메라)"
            print(self.webcam_error, flush=True)
            return self.webcam_error
        try:
            w, h = (int(v) for v in self.a.webcam_size.split("x"))
            self.cam = Webcam(dev, name, w, h)
            self.webcam_error = ""
            self.meta["webcam"] = dict(device=dev, name=name, size=list(self.cam.size), fourcc="MJPG")
            print(f"웹캠 {dev} '{name}' {self.cam.size[0]}x{self.cam.size[1]}", flush=True)
            return ""
        except Exception as e:
            self.cam, self.webcam_error = None, str(e)
            print(f"웹캠 못 엶: {e}", flush=True)
            return str(e)

    # ---- 녹화
    def start(self):
        with self.lock:
            if self.rec:
                return "이미 녹화 중"
            out = self.a.out / datetime.now().strftime("%Y-%m-%d_%H%M%S")
            (out / "xtion").mkdir(parents=True)
            (out / "xtion_depth").mkdir()
            (out / "webcam").mkdir()
            t0 = time.monotonic()
            meta = dict(self.meta, start=datetime.now().isoformat(timespec="seconds"), t0_monotonic=t0,
                        clock_synced=clock_synced())
            (out / "meta.json").write_text(json.dumps(meta, ensure_ascii=False, indent=2), encoding="utf-8")
            fcsv = open(out / "frames.csv", "w", newline="", encoding="utf-8")
            wcsv = open(out / "wheels.csv", "w", newline="", encoding="utf-8")
            fw, ww = csv.writer(fcsv), csv.writer(wcsv)
            fw.writerow(["i", "t", "xtion_t", "webcam_t", "odo_m", "odo_s", "kmh_r", "kmh_l"])
            ww.writerow(["t", "odo_m", "odo_s", "kmh_r", "kmh_l", "kmh_b_r", "kmh_b_l", "volt", "rx", "rx_b", "out_l", "out_r"])
            self.rec = r = dict(out=out, t0=t0, i=0, fcsv=fcsv, wcsv=wcsv, fw=fw, ww=ww, next_t=t0, xt_last=None,
                                cam_last=None, last_r=None, n_xt=0, n_cam=0, video=None)
        if self.cam is not None and self.cam.alive and not getattr(self.a, "no_video", False):
            r["video"] = threading.Thread(target=self._video, args=(r,), daemon=True)
            r["video"].start()
        print(f"녹화 시작 → {out}", flush=True)
        return ""

    def _video(self, r):
        """녹화하는 동안 웹캠 프레임을 받는 대로 webcam.mkv 에 (사진 5 Hz 와 따로, 사람이 볼 것). 시각은 webcam_video.csv.
        Orin Nano 에는 하드웨어 인코더가 없어 x264 (CPU, ultrafast) 로 — 720p 30 fps 에 코어 하나 남짓."""
        cam = self.cam
        fps = round(self.fps.get("webcam") or 0)
        fps = fps if fps >= 5 else 30                        # 막 연 참이면 아직 못 쟀다 — 웹캠 설정값
        w, h = cam.size
        pipe = (f"appsrc ! videoconvert ! video/x-raw,format=I420 ! x264enc speed-preset=ultrafast tune=zerolatency "
                f"bitrate=4000 key-int-max={fps * 2} ! h264parse ! matroskamux ! filesink location={r['out'] / 'webcam.mkv'}")
        vw = cv2.VideoWriter(pipe, cv2.CAP_GSTREAMER, 0, fps, (w, h), True)
        if not vw.isOpened():
            print("웹캠 동영상을 못 연다 (GStreamer x264) — 사진만 남긴다", flush=True)
            return
        f = open(r["out"] / "webcam_video.csv", "w", newline="", encoding="utf-8")
        cw = csv.writer(f)
        cw.writerow(["n", "t"])
        last, n = cam.n, 0
        try:
            while self.rec is r and cam.alive:
                if cam.n == last:
                    time.sleep(0.004)
                    continue
                frame, t, last = cam.latest, cam.t, cam.n
                if (frame.shape[1], frame.shape[0]) != (w, h):
                    frame = cv2.resize(frame, (w, h))
                vw.write(frame)
                cw.writerow([n, f"{t - r['t0']:.3f}"])
                n += 1
                if n % 60 == 0:
                    f.flush()
        finally:
            vw.release()                                     # EOS — 동영상 끝마무리
            f.close()
            r["video_frames"] = n

    def stop(self, why="버튼"):
        with self.lock:
            r, self.rec = self.rec, None
        if not r:
            return "녹화 중이 아니다"
        r["fcsv"].close()
        r["wcsv"].close()
        if r["video"] is not None:
            r["video"].join(timeout=5.0)                     # 동영상을 닫고 나서 끝 (바로 뒤 os._exit 여도 열리게)
        self.last_saved = dict(out=str(r["out"]), frames=r["i"], seconds=round(time.monotonic() - r["t0"], 1), why=why)
        print(f"녹화 끝 ({why}): {r['out']}  프레임 {r['i']}"
              + (f" · 동영상 {r.get('video_frames', 0)} 프레임" if r["video"] is not None else ""), flush=True)
        return ""

    def _loop(self):
        xt_n0, cam_n0, t_fps = 0, 0, time.monotonic()
        while self._alive:
            now = time.monotonic()
            if now - t_fps >= 2.0:                           # 화면에 보일 카메라 fps
                xn = self.xt._n if self.xt else 0
                cn = self.cam.n if self.cam else 0
                self.fps = {"xtion": (xn - xt_n0) / (now - t_fps), "webcam": (cn - cam_n0) / (now - t_fps)}
                xt_n0, cam_n0, t_fps = xn, cn, now
            with self.lock:
                r = self.rec
                expired = r is not None and now - r["t0"] > self.a.minutes * 60
                if r is not None and not expired:
                    self._record_step(r, now)
            if expired:
                self.stop(f"{self.a.minutes:g} 분 지남")
            time.sleep(0.01)

    def _record_step(self, r, now):
        t0 = r["t0"]
        rep = self.link.report if self.link else None
        if rep is not None and rep is not r["last_r"]:
            r["ww"].writerow([f"{rep.t - t0:.3f}", rep.odo[0], rep.odo[1], rep.kmh[0], rep.kmh[1], rep.kmh_b[0], rep.kmh_b[1],
                              rep.volt, rep.rx, rep.rx_b, rep.out[0], rep.out[1]])
            r["last_r"] = rep
        if now < r["next_t"]:
            return
        r["next_t"] += 1.0 / self.a.hz
        if r["next_t"] < now:                                # 밀렸으면 따라잡지 말고 다음 칸부터
            r["next_t"] = now + 1.0 / self.a.hz
        i, out = r["i"], r["out"]
        xf = None
        if self.xt is not None and self.xt._n != r["xt_last"]:   # 지난번 저장 뒤 새 프레임만
            with self.xt._lock:
                xf, r["xt_last"] = self.xt._latest, self.xt._n
            cv2.imwrite(str(out / "xtion" / f"{i:06d}.jpg"), xf.bgr, [cv2.IMWRITE_JPEG_QUALITY, 92])
            cv2.imwrite(str(out / "xtion_depth" / f"{i:06d}.png"), (xf.depth * 1000).astype(np.uint16))
            r["n_xt"] += 1
        cam_t = None
        if self.cam is not None and self.cam.n != r["cam_last"]:
            wf, cam_t, r["cam_last"] = self.cam.latest, self.cam.t, self.cam.n
            cv2.imwrite(str(out / "webcam" / f"{i:06d}.jpg"), wf, [cv2.IMWRITE_JPEG_QUALITY, 92])
            r["n_cam"] += 1
        r["fw"].writerow([i, f"{now - t0:.3f}", f"{xf.t - t0:.3f}" if xf is not None else "",
                          f"{cam_t - t0:.3f}" if cam_t is not None else "",
                          *(rep.odo if rep else ("", "")), *(rep.kmh if rep else ("", ""))])
        r["i"] += 1
        if i % 25 == 0:
            r["fcsv"].flush()
            r["wcsv"].flush()

    # ---- 화면
    def preview(self, which, width=640):
        """화면용 JPEG (작게). 없으면 None."""
        img = None
        if which == "xtion" and self.xt is not None and self.xt._latest is not None:
            img = self.xt._latest.bgr
        elif which == "depth" and self.xt is not None and self.xt._latest is not None:
            d = self.xt._latest.depth
            v = np.clip(d / 4.0 * 255, 0, 255).astype(np.uint8)
            img = cv2.applyColorMap(255 - v, cv2.COLORMAP_JET)
            img[d <= 0] = (40, 40, 40)                       # 값 없음 = 진회색
        elif which == "webcam" and self.cam is not None and self.cam.alive:
            img = self.cam.latest
        if img is None:
            return None
        if img.shape[1] > width:
            img = cv2.resize(img, (width, round(img.shape[0] * width / img.shape[1])), interpolation=cv2.INTER_AREA)
        ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 75])
        return buf.tobytes() if ok else None

    def state(self):
        rep = self.link.report if self.link else None
        age = (time.monotonic() - rep.t) if rep else None
        s = dict(xtion=bool(self.xt), xtion_error=self.xtion_error, webcam=bool(self.cam and self.cam.alive),
                 webcam_name=self.cam.name if self.cam else "", webcam_size=list(self.cam.size) if self.cam else None,
                 webcam_error=self.webcam_error, fps={k: round(v, 1) for k, v in self.fps.items()},
                 wheels=bool(self.link), wheels_error=self.wheels_error, last=self.last_saved,
                 disk_free_gb=round(shutil.disk_usage(self.a.out if self.a.out.exists() else REPO).free / 1e9, 1))
        if rep:
            # 앞으로 = + (2026-10-14 바퀴 방향: 앞으로 갈 때 보드가 왼쪽 바퀴 속도를 − 로 보고)
            s.update(odo=list(rep.odo), volt=rep.volt, rx=rep.rx, rx_b=rep.rx_b, report_age=round(age, 1),
                     speed_kmh=round((rep.kmh[0] - rep.kmh[1] + rep.kmh_b[0] - rep.kmh_b[1]) / 4, 2))
        r = self.rec
        if r:
            s["rec"] = dict(out=str(r["out"]), seconds=round(time.monotonic() - r["t0"], 1), frames=r["i"],
                            xtion=r["n_xt"], webcam=r["n_cam"], minutes=self.a.minutes)
        return s

    def close(self):
        self.stop("프로그램 끝")
        self._alive = False
        for c in (self.xt, self.cam):
            if c is not None:
                c.close()
        if self.link is not None and self.own_link:          # 읽기만 했으니 x · w 0 도 보내지 않는다 (다른 쪽이 몰고 있을 수 있다)
            self.link._alive = False
            try:
                self.link.ser.close()
            except Exception:
                pass


PAGE = """<!doctype html><html lang="ko"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>누비 녹화</title><style>
:root{--bg:#f6f6f4;--card:#fff;--ink:#1b1b1a;--mut:#6b6a66;--line:#dcdbd5;--rec:#d03b3b;--ok:#0c8a0c}
@media (prefers-color-scheme:dark){:root{--bg:#161615;--card:#1f1f1e;--ink:#ecebe6;--mut:#a2a19b;--line:#383835}}
body{margin:0;background:var(--bg);color:var(--ink);font:15px/1.5 system-ui,"Malgun Gothic",sans-serif}
main{max-width:1500px;margin:0 auto;padding:14px 16px}
.bar{display:flex;flex-wrap:wrap;gap:10px;align-items:center;margin-bottom:12px}
button{font:inherit;padding:8px 16px;border-radius:8px;border:1px solid var(--line);background:var(--card);color:var(--ink);cursor:pointer}
#rec{font-weight:600;min-width:150px}#rec.on{background:var(--rec);color:#fff;border-color:var(--rec)}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(320px,1fr));gap:12px}
.cam{background:var(--card);border:1px solid var(--line);border-radius:10px;padding:8px}
.cam h2{font-size:14px;font-weight:600;margin:0 0 6px;display:flex;flex-wrap:wrap;justify-content:space-between;align-items:baseline;gap:2px 8px;white-space:nowrap}
.cam h2 span{font-size:12px;font-weight:400;color:var(--mut);white-space:normal}
.cam img{width:100%;display:block;border-radius:6px;background:#222;min-height:120px}
.stats{display:grid;grid-template-columns:repeat(auto-fit,minmax(150px,1fr));gap:8px;margin-top:12px}
.st{background:var(--card);border:1px solid var(--line);border-radius:8px;padding:8px 10px}
.st b{display:block;font-size:12px;font-weight:400;color:var(--mut)}.st div{font-size:18px;font-weight:600}
.dot{display:inline-block;width:10px;height:10px;border-radius:50%;background:var(--line);margin-right:6px}
.dot.on{background:var(--rec);animation:p 1s infinite}@keyframes p{50%{opacity:.3}}
#msg{color:var(--mut);font-size:13px}.err{color:var(--rec)}
</style></head><body><main>
<div class="bar"><button id="rec" onclick="toggle()">녹화 시작</button><span><span class="dot" id="dot"></span><span id="recinfo">대기</span></span>
<button onclick="post('/webcam/rescan')">웹캠 다시 찾기</button><span id="msg"></span></div>
<div class="grid">
<div class="cam"><h2>Xtion 컬러 <span id="fx"></span></h2><img src="/stream/xtion" alt="Xtion 컬러"></div>
<div class="cam"><h2>Xtion 깊이 <span>0 ~ 4 m · 빨강 가까움 · 회색 값 없음</span></h2><img src="/stream/depth" alt="Xtion 깊이"></div>
<div class="cam"><h2>웹캠 <span id="fw"></span></h2><img id="wimg" src="/stream/webcam" alt="웹캠"></div>
</div>
<div class="stats">
<div class="st"><b>속도 (네 바퀴 평균, 앞으로 +)</b><div id="spd">–</div></div>
<div class="st"><b>앞축 홀 스텝 (오른쪽, 왼쪽)</b><div id="odo">–</div></div>
<div class="st"><b>전압 · 보드 링크</b><div id="volt">–</div></div>
<div class="st"><b>남은 디스크</b><div id="disk">–</div></div>
</div>
<p id="last" style="color:var(--mut);font-size:13px"></p>
</main><script>
let recOn=false, webcamOn=null;
async function post(u){const r=await fetch(u,{method:'POST'});const j=await r.json();document.getElementById('msg').textContent=j.msg||'';
 if(u==='/webcam/rescan'){document.getElementById('wimg').src='/stream/webcam?'+Date.now()}poll()}
let armStop=0;function toggle(){const b=document.getElementById('rec');if(!recOn){post('/record/start');return}if(Date.now()-armStop<3000){armStop=0;post('/record/stop');return}armStop=Date.now();b.textContent='한 번 더 누르면 끝';setTimeout(()=>{if(recOn)b.textContent='녹화 끝'},3000)}
function fmt(s){const m=Math.floor(s/60);return m+':'+String(Math.floor(s%60)).padStart(2,'0')}
async function poll(){try{const s=await (await fetch('/state')).json();
 recOn=!!s.rec;const b=document.getElementById('rec');if(!(recOn&&Date.now()-armStop<3000))b.textContent=recOn?'녹화 끝':'녹화 시작';b.classList.toggle('on',recOn);
 document.getElementById('dot').classList.toggle('on',recOn);
 document.getElementById('recinfo').textContent=recOn?`녹화 중 ${fmt(s.rec.seconds)} · 프레임 ${s.rec.frames} (Xtion ${s.rec.xtion}, 웹캠 ${s.rec.webcam}) · ${s.rec.minutes}분 뒤 자동 끝`:'대기 (저장 안 함)';
 document.getElementById('fx').textContent=s.xtion?`${s.fps.xtion} fps`:(s.xtion_error||'없음');
 document.getElementById('fw').textContent=s.webcam?`${s.webcam_name} ${s.webcam_size[0]}×${s.webcam_size[1]} · ${s.fps.webcam} fps`:(s.webcam_error||'없음');
 if(webcamOn===false&&s.webcam)document.getElementById('wimg').src='/stream/webcam?'+Date.now();webcamOn=s.webcam;
 document.getElementById('spd').textContent=s.speed_kmh!==undefined?`${s.speed_kmh.toFixed(2)} km/h`:'–';
 document.getElementById('odo').textContent=s.odo?`${s.odo[0]}, ${s.odo[1]}`:(s.wheels_error?'못 엶':'–');
 const link=s.rx!==undefined?(s.volt>1?'켜짐':'보드 꺼짐'):'–';
 document.getElementById('volt').innerHTML=s.volt!==undefined?`${s.volt.toFixed(1)} V · <span class="${s.volt>1?'':'err'}">${link}</span>`:'–';
 document.getElementById('disk').textContent=`${s.disk_free_gb} GB`;
 document.getElementById('last').textContent=s.last?`마지막 녹화: ${s.last.out} · ${fmt(s.last.seconds)} · 프레임 ${s.last.frames} (${s.last.why})`:'';
}catch(e){document.getElementById('recinfo').textContent='Jetson 과 연결이 끊김 (녹화는 계속된다)'}}
setInterval(poll,700);poll();
</script></body></html>"""


def serve(rc, host, port):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _json(self, obj, code=200):
            b = json.dumps(obj, ensure_ascii=False).encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(b)))
            self.end_headers()
            self.wfile.write(b)

        def do_GET(self):
            path = self.path.split("?")[0]
            if path == "/":
                b = PAGE.encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(b)))
                self.end_headers()
                self.wfile.write(b)
            elif path == "/state":
                self._json(rc.state())
            elif path.startswith("/stream/"):
                which = path.rsplit("/", 1)[1]
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                try:
                    while True:
                        jpg = rc.preview(which)
                        if jpg is None:
                            if which == "webcam" and not (rc.cam and rc.cam.alive):
                                return                       # 웹캠이 없으면 끊고, 다시 찾으면 페이지가 새로 연다
                            time.sleep(0.2)
                            continue
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                         + str(len(jpg)).encode() + b"\r\n\r\n" + jpg + b"\r\n")
                        time.sleep(0.1)                      # 화면은 10 fps 면 충분 (녹화는 따로)
                except (BrokenPipeError, ConnectionResetError):
                    return
            else:
                self._json({"msg": "없음"}, 404)

        def do_POST(self):
            if self.path == "/record/start":
                self._json({"msg": rc.start() or "녹화 시작"})
            elif self.path == "/record/stop":
                self._json({"msg": rc.stop() or "녹화 끝"})
            elif self.path == "/webcam/rescan":
                self._json({"msg": rc.open_webcam() or "웹캠 열림"})
            else:
                self._json({"msg": "없음"}, 404)

    srv = ThreadingHTTPServer((host, port), H)
    srv.daemon_threads = True
    print(f"화면: http://{host}:{port}", flush=True)
    srv.serve_forever()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--hz", type=float, default=5.0, help="사진 저장 빈도 (지도 영상도 5 fps)")
    ap.add_argument("--minutes", type=float, default=20.0, help="녹화를 시작하고 이 시간이 지나면 스스로 끝낸다")
    ap.add_argument("--now", action="store_true", help="화면 없이 바로 녹화하고 --minutes 뒤 프로그램도 끝낸다")
    ap.add_argument("--host", default="192.168.127.2", help="웹 화면 주소 (유선 직결)")
    ap.add_argument("--port", type=int, default=8093)
    ap.add_argument("--webcam", default="auto", help="auto | /dev/videoN | none")
    ap.add_argument("--webcam-size", default="1280x720")
    ap.add_argument("--no-xtion", action="store_true")
    ap.add_argument("--no-wheels", action="store_true")
    ap.add_argument("--no-video", action="store_true", help="웹캠 동영상 (webcam.mkv) 을 남기지 않는다 — 사진만")
    ap.add_argument("--serial", help="ESP32 시리얼 (기본: 찾아서)")
    ap.add_argument("--out", type=Path, default=REPO / "data/logs/hallway-run")
    a = ap.parse_args()
    stop_on_signals()
    a.out.mkdir(parents=True, exist_ok=True)
    rc = Recorder(a)
    try:
        if a.now:
            rc.start()
            while rc.rec is not None:
                time.sleep(0.5)
        else:
            serve(rc, a.host, a.port)
    except KeyboardInterrupt:
        print("멈춤 신호로 끝냄", flush=True)
    finally:
        rc.close()
        os._exit(0)                                          # OpenNI 정리 중 멈추는 일을 피한다 (xtion.py 와 같은 이유)


if __name__ == "__main__":
    main()
