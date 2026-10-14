"""웹캠 노출 브라케팅 → Mertens 노출 합성으로 HDR 처럼 보이게 — 실시간 웹 화면 (Jetson).

    python3 hdr_view.py                 → http://192.168.127.2:8094 (유선 직결 주소에만 연다)
    python3 hdr_view.py --device /dev/video0 --size 960x540

프레임마다 노출을 짧게 → 중간 → 길게 돌려 가며 찍고 (UVC 수동 노출, 30 fps 유지), 가장 최근 세 장을 밝기 순으로 정렬해
(짧게 · 중간 · 길게) cv2.createMergeMertens 로 합친다. 세 장이 0.1 s 에 걸쳐 찍히므로 빨리 움직이면 겹쳐 보인다.
'자동' 이면 중간 노출을 화면 밝기에 맞추고 짧게 · 길게는 그 앞뒤로 EV 간격만큼 둔다. 중간은 33 ms 까지, 길게는 '긴 노출 최대'
(기본 66 ms) 까지 — 33 ms 를 넘는 긴 프레임은 그만큼 늦게 나오지만 세 장 중 한 장이라 카메라는 20 fps 넘게 남는다.
끝날 때 카메라를 처음 설정 (자동 노출 · 게인) 으로 되돌린다.
"""

import argparse
import fcntl
import json
import os
import struct
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np

VIDIOC_S_CTRL, VIDIOC_G_CTRL = 0xC008561C, 0xC008561B
EXPOSURE_AUTO, EXPOSURE_ABS, GAIN = 0x009A0901, 0x009A0902, 0x00980913
MANUAL, AUTO = 1, 3                       # V4L2_CID_EXPOSURE_AUTO: 1 = 손으로, 3 = 조리개 우선 (자동)
EXP_MIN, EXP_MAX_FAST = 1, 330            # 0.1 ms 단위. 33 ms 를 넘으면 30 fps 를 못 지킨다


def setc(fd, cid, v):
    fcntl.ioctl(fd, VIDIOC_S_CTRL, bytearray(struct.pack("Ii", cid, int(v))))


def getc(fd, cid):
    b = bytearray(struct.pack("Ii", cid, 0))
    fcntl.ioctl(fd, VIDIOC_G_CTRL, b)
    return struct.unpack("Ii", b)[1]


class HDR:
    def __init__(self, dev, size, proc):
        self.cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        w, h = size
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        ok, f = self.cap.read()
        if not ok:
            raise SystemExit(f"{dev} 에서 프레임이 안 온다")
        self.size = (f.shape[1], f.shape[0])
        self.fd = os.open(dev, os.O_RDWR)
        self.orig = {c: getc(self.fd, c) for c in (EXPOSURE_AUTO, EXPOSURE_ABS, GAIN)}
        setc(self.fd, EXPOSURE_AUTO, MANUAL)
        self.cfg = dict(auto=True, mid=80, ev=2.0, gain=0, proc=proc, target=110, long_max=660)
        setc(self.fd, GAIN, self.cfg["gain"])
        self.ring = deque(maxlen=3)               # (프레임, 밝기, 찍은 시각)
        self.n = 0
        self.hdr = None
        self.triple = None
        self.stat = dict(cam_fps=0.0, hdr_fps=0.0, fuse_ms=0.0)
        self.merge = cv2.createMergeMertens()
        self.lock = threading.Lock()
        self._alive = True
        threading.Thread(target=self._grab, daemon=True).start()
        threading.Thread(target=self._fuse, daemon=True).start()

    def exps(self):
        c = self.cfg
        k = 2 ** c["ev"]
        mid = max(EXP_MIN, min(EXP_MAX_FAST, round(c["mid"])))
        return [max(EXP_MIN, round(mid / k)), mid, max(mid, min(c["long_max"], round(mid * k)))]

    def _grab(self):
        k, t0, n0 = 0, time.monotonic(), 0
        while self._alive:
            e = self.exps()
            setc(self.fd, EXPOSURE_ABS, e[(k + 1) % 3])    # 카메라에는 짧게 · 중간 · 길게가 차례로 들어간다
            ok, f = self.cap.read()
            if not ok:
                time.sleep(0.01)
                continue
            small = cv2.resize(f, (160, 90), interpolation=cv2.INTER_AREA)
            with self.lock:
                self.ring.append((f, float(small.mean()), time.monotonic()))
                self.n += 1
            k += 1
            now = time.monotonic()
            if now - t0 >= 1.0:
                self.stat["cam_fps"] = (self.n - n0) / (now - t0)
                t0, n0 = now, self.n
                self._auto()

    def _auto(self):
        """중간 프레임 밝기를 목표에 맞춘다 (1 초마다, 로그 비례)."""
        c, t = self.cfg, self.triple
        if not c["auto"] or t is None:
            return
        m = max(1.0, t[1][1])
        c["mid"] = float(np.clip(c["mid"] * (c["target"] / m) ** 0.7, EXP_MIN, EXP_MAX_FAST))

    def _fuse(self):
        last, t0, h0, nh = -1, time.monotonic(), 0, 0
        while self._alive:
            with self.lock:
                if self.n == last or len(self.ring) < 3:
                    ring = None
                else:
                    ring, last = list(self.ring), self.n
            if ring is None:
                time.sleep(0.003)
                continue
            ring.sort(key=lambda r: r[1])                    # 어두운 것 = 짧은 노출
            w = self.cfg["proc"]
            h = round(w * self.size[1] / self.size[0]) // 2 * 2
            imgs = [cv2.resize(r[0], (w, h), interpolation=cv2.INTER_AREA) for r in ring]
            t1 = time.perf_counter()
            out = self.merge.process(imgs)
            out = np.clip(out * 255, 0, 255).astype(np.uint8)
            self.stat["fuse_ms"] = (time.perf_counter() - t1) * 1000
            self.hdr, self.triple = out, [(im, r[1]) for im, r in zip(imgs, ring)]
            nh += 1
            now = time.monotonic()
            if now - t0 >= 1.0:
                self.stat["hdr_fps"] = (nh - h0) / (now - t0)
                t0, h0 = now, nh

    def jpeg(self, which):
        if which == "hdr":
            img = self.hdr
        else:
            t = self.triple
            img = None if t is None else t[{"short": 0, "mid": 1, "long": 2}[which]][0]
        if img is None:
            return None
        ok, b = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 85])
        return b.tobytes() if ok else None

    def state(self):
        e = self.exps()
        t = self.triple
        return dict(cfg=self.cfg, exp_ms=[v / 10 for v in e], size=list(self.size),
                    bright=[round(x[1]) for x in t] if t else None,
                    **{k: round(v, 1) for k, v in self.stat.items()})

    def set(self, d):
        c = self.cfg
        if "auto" in d:
            c["auto"] = bool(d["auto"])
        if "mid_ms" in d:
            c["mid"] = float(np.clip(float(d["mid_ms"]) * 10, EXP_MIN, EXP_MAX_FAST))
        if "ev" in d:
            c["ev"] = float(np.clip(float(d["ev"]), 0.5, 4.0))
        if "gain" in d:
            c["gain"] = int(np.clip(int(d["gain"]), 0, 63))
            setc(self.fd, GAIN, c["gain"])
        if "proc" in d and int(d["proc"]) in (480, 640, 960, 1280):
            c["proc"] = int(d["proc"])
        if "long_max_ms" in d:
            c["long_max"] = int(np.clip(float(d["long_max_ms"]) * 10, EXP_MAX_FAST, 2000))
        if "target" in d:
            c["target"] = float(np.clip(float(d["target"]), 40, 200))

    def close(self):
        self._alive = False
        time.sleep(0.1)
        for cid in (GAIN, EXPOSURE_ABS, EXPOSURE_AUTO):    # 처음 설정으로 (자동 노출을 마지막에)
            try:
                setc(self.fd, cid, self.orig[cid])
            except OSError:
                pass
        os.close(self.fd)
        self.cap.release()


PAGE = """<!doctype html><html lang="ko"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>웹캠 HDR</title><style>
:root{--bg:#f6f6f4;--card:#fff;--ink:#1b1b1a;--mut:#6b6a66;--line:#dcdbd5;--acc:#2a78d6}
@media (prefers-color-scheme:dark){:root{--bg:#161615;--card:#1f1f1e;--ink:#ecebe6;--mut:#a2a19b;--line:#383835}}
body{margin:0;background:var(--bg);color:var(--ink);font:15px/1.5 system-ui,"Malgun Gothic",sans-serif}
main{max-width:1500px;margin:0 auto;padding:14px 16px}
.big{display:grid;grid-template-columns:repeat(auto-fit,minmax(340px,1fr));gap:12px}
.small{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:10px;margin-top:12px}
.cam{background:var(--card);border:1px solid var(--line);border-radius:10px;padding:8px}
.cam h2{font-size:14px;font-weight:600;margin:0 0 6px;display:flex;flex-wrap:wrap;justify-content:space-between;gap:2px 8px}
.cam h2 span{font-size:12px;font-weight:400;color:var(--mut)}
.cam img{width:100%;display:block;border-radius:6px;background:#222;min-height:80px}
.ctl{display:grid;grid-template-columns:repeat(auto-fit,minmax(230px,1fr));gap:10px 18px;margin-top:14px;background:var(--card);
 border:1px solid var(--line);border-radius:10px;padding:12px 14px}
.ctl label{display:flex;flex-direction:column;font-size:13px;color:var(--mut)}.ctl b{color:var(--ink);font-weight:600}
input[type=range]{width:100%}select{font:inherit}
#st{color:var(--mut);font-size:13px;margin-top:8px}
</style></head><body><main>
<div class="big">
<div class="cam"><h2>HDR 합성 <span id="hs"></span></h2><img src="/stream/hdr" alt="HDR 합성"></div>
<div class="cam"><h2>보통 (중간 노출 한 장) <span id="ms"></span></h2><img src="/stream/mid" alt="중간 노출"></div>
</div>
<div class="small">
<div class="cam"><h2>짧게 <span id="e0"></span></h2><img src="/stream/short" alt="짧은 노출"></div>
<div class="cam"><h2>중간 <span id="e1"></span></h2><img src="/stream/mid" alt="중간 노출"></div>
<div class="cam"><h2>길게 <span id="e2"></span></h2><img src="/stream/long" alt="긴 노출"></div>
</div>
<div class="ctl">
<label><span><input type="checkbox" id="auto" checked onchange="send({auto:this.checked})"> <b>자동</b> — 중간 노출을 화면 밝기에 맞춤</span>
 <span>목표 밝기 <b id="tv"></b></span><input type="range" id="target" min="60" max="180" step="5" oninput="tv.textContent=this.value" onchange="send({target:+this.value})"></label>
<label><span>중간 노출 <b id="mv"></b> ms</span><input type="range" id="mid" min="0.5" max="33" step="0.1" oninput="mv.textContent=this.value" onchange="auto.checked=false;send({auto:false,mid_ms:+this.value})"></label>
<label><span>노출 간격 ±<b id="evv"></b> EV (짧게 = 중간 ÷ 2<sup>EV</sup>, 길게 = × 2<sup>EV</sup>)</span><input type="range" id="ev" min="0.5" max="4" step="0.5" oninput="evv.textContent=this.value" onchange="send({ev:+this.value})"></label>
<label><span>게인 <b id="gv"></b> (높이면 어두운 곳이 밝아지지만 잡음)</span><input type="range" id="gain" min="0" max="63" step="1" oninput="gv.textContent=this.value" onchange="send({gain:+this.value})"></label>
<label><span>긴 노출 최대 <b id="lv"></b> ms (33 넘으면 카메라 fps 가 조금 준다)</span><input type="range" id="lmax" min="33" max="200" step="1" oninput="lv.textContent=this.value" onchange="send({long_max_ms:+this.value})"></label>
<label><span>합성 해상도 (클수록 느림)</span><select id="proc" onchange="send({proc:+this.value})"><option>480</option><option>640</option><option>960</option><option>1280</option></select></label>
</div>
<div id="st"></div>
</main><script>
let first=true;
async function send(d){await fetch('/set',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(d)});poll()}
async function poll(){try{const s=await (await fetch('/state')).json();const c=s.cfg;
 for(let i=0;i<3;i++)document.getElementById('e'+i).textContent=`${s.exp_ms[i].toFixed(1)} ms`+(s.bright?` · 밝기 ${s.bright[i]}`:'');
 document.getElementById('hs').textContent=`${s.hdr_fps} fps · 합성 ${s.fuse_ms} ms`;
 document.getElementById('ms').textContent=`${s.exp_ms[1].toFixed(1)} ms`;
 if(first||c.auto){mid.value=(c.mid/10).toFixed(1);mv.textContent=mid.value}
 if(first){auto.checked=c.auto;target.value=c.target;tv.textContent=c.target;ev.value=c.ev;evv.textContent=c.ev;gain.value=c.gain;gv.textContent=c.gain;proc.value=c.proc;lmax.value=c.long_max/10;lv.textContent=lmax.value;first=false}
 document.getElementById('st').textContent=`카메라 ${s.size[0]}×${s.size[1]} · ${s.cam_fps} fps (노출 세 가지를 돌려 가며 — HDR 한 장 = 최근 세 프레임, 빨리 움직이면 겹쳐 보인다)`;
}catch(e){document.getElementById('st').textContent='Jetson 과 연결이 끊김'}}
setInterval(poll,800);poll();
</script></body></html>"""


def serve(hdr, host, port):
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
                self._json(hdr.state())
            elif path.startswith("/stream/"):
                which = path.rsplit("/", 1)[1]
                if which not in ("hdr", "short", "mid", "long"):
                    return self._json({"msg": "없음"}, 404)
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                try:
                    while True:
                        jpg = hdr.jpeg(which)
                        if jpg is not None:
                            self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                             + str(len(jpg)).encode() + b"\r\n\r\n" + jpg + b"\r\n")
                        time.sleep(0.066)                    # 화면 15 fps
                except (BrokenPipeError, ConnectionResetError):
                    return
            else:
                self._json({"msg": "없음"}, 404)

        def do_POST(self):
            if self.path == "/set":
                n = int(self.headers.get("Content-Length", 0))
                hdr.set(json.loads(self.rfile.read(n) or b"{}"))
                self._json({"msg": "ok"})
            else:
                self._json({"msg": "없음"}, 404)

    srv = ThreadingHTTPServer((host, port), H)
    srv.daemon_threads = True
    print(f"화면: http://{host}:{port}", flush=True)
    srv.serve_forever()


def main():
    import signal
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--device", default="/dev/video0")
    ap.add_argument("--size", default="1280x720", help="카메라에서 받는 크기")
    ap.add_argument("--proc", type=int, default=640, help="합성 해상도 (가로)")
    ap.add_argument("--host", default="192.168.127.2")
    ap.add_argument("--port", type=int, default=8094)
    a = ap.parse_args()
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt))
    hdr = HDR(a.device, tuple(int(v) for v in a.size.split("x")), a.proc)
    print(f"웹캠 {a.device} {hdr.size[0]}x{hdr.size[1]}, 처음 설정 {hdr.orig}", flush=True)
    try:
        serve(hdr, a.host, a.port)
    except KeyboardInterrupt:
        pass
    finally:
        hdr.close()
        print("카메라 설정을 되돌리고 끝냄", flush=True)


if __name__ == "__main__":
    main()
