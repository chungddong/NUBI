"""바퀴 점검 웹 패널 — Jetson 에서 띄우고 PC 브라우저로 본다 (유선 직결 주소에만 연다 — 학교 Wi-Fi 에서는 안 보인다).

    python3 bench.py                         → http://192.168.127.2:8090
    python3 bench.py --host 127.0.0.1        Jetson 화면에서만

네 바퀴 속도 · 앞축 오도메트리 · 전압 · 전류 · 링크를 0.2 s 마다 보이고, 짧은 시험 (앞 · 뒤 · 왼쪽 / 오른쪽 제자리,
최대 3 s) 을 버튼으로 한다. 안전: '바퀴를 모두 띄웠다' 를 체크해야 시험 버튼이 켜진다 · 시험 중에만 하트비트 임대 →
서버가 멈추면 1 s 뒤 ESP32 가 선다 · 페이지가 1 s 넘게 상태를 안 물으면 (탭 닫힘 · 네트워크 끊김) 선다 · STOP 은 즉시 `x`.
시험마다 바퀴별 평균 속도 부호와 앞축 홀 스텝 변화를 남긴다 — 앞으로 시험의 부호가 robot.json 의 odo_sign 후보다.
"""

import argparse
import json
import sys
import threading
import time
from pathlib import Path
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from drive import stop_on_signals
from wheel_link import WheelLink, open_serial

MAX_CMD = 300                     # ESP32 클램프 (부팅 400 보다 낮게)
MAX_SECONDS = 3.0
WHEELS = ("앞 왼쪽", "앞 오른쪽", "뒤 왼쪽", "뒤 오른쪽")
# 보드가 보고하는 속도 부호를 '앞으로 = +' 로 — 2026-10-14 바퀴를 띄우고 잰 값 (앞으로 갈 때 왼쪽 바퀴가 −).
# 앞축 홀 스텝은 robot.json 의 odo_sign (마스터 = 오른쪽, 슬레이브 = 왼쪽) 으로 같은 뜻이 되게 한다.
KMH_FWD = {"앞 왼쪽": -1, "앞 오른쪽": 1, "뒤 왼쪽": -1, "뒤 오른쪽": 1}
ODO_SIGN = json.loads((Path(__file__).with_name("robot.json")).read_text(encoding="utf-8"))["odo_sign"]


class Bench:
    def __init__(self, link):
        self.link = link
        self.lock = threading.Lock()
        self.test = None
        self.results = []
        self.page_seen = time.monotonic()
        self.armed = False
        threading.Thread(target=self._loop, daemon=True).start()

    @staticmethod
    def wheels(r):
        """보고 줄 → 바퀴별 km/h (앞축 meas = (마스터 = 오른쪽, 슬레이브 = 왼쪽), 뒤축 m2 같음)."""
        raw = {"앞 왼쪽": r.kmh[1], "앞 오른쪽": r.kmh[0], "뒤 왼쪽": r.kmh_b[1], "뒤 오른쪽": r.kmh_b[0]}
        return {w: KMH_FWD[w] * v for w, v in raw.items()}, raw

    def start(self, name, seconds, cmd):
        r = self.link.report
        if r is None or not self.link.fresh(1.0):
            return "보고 줄이 안 온다"
        if self.test:
            return "시험 중"
        seconds = max(0.3, min(float(seconds), MAX_SECONDS))
        cmd = max(60, min(int(cmd), MAX_CMD))
        turn = round(cmd * 1000 / MAX_CMD)                    # t 는 클램프 대비 ‰ — 바퀴 듀티가 cmd 가 되게
        speed, steer = {"앞으로": (cmd, 0), "뒤로": (-cmd, 0), "왼쪽 제자리": (0, -turn), "오른쪽 제자리": (0, turn)}[name]
        if not self.armed:
            self.link.arm(MAX_CMD)
            self.armed = True
        with self.lock:
            self.test = {"name": name, "cmd": cmd, "seconds": seconds, "speed": speed, "steer": steer,
                         "t0": time.monotonic(), "until": time.monotonic() + seconds, "odo0": r.odo,
                         "sum": {w: 0.0 for w in WHEELS}, "n": 0, "why": None}
        return None

    def stop(self, why="STOP 버튼"):
        self.link.stop()
        with self.lock:
            if self.test and not self.test["why"]:
                self.test["why"] = why
                self.test["until"] = 0

    def _loop(self):
        while True:
            time.sleep(0.1)
            now = time.monotonic()
            with self.lock:
                t = self.test
                if not t:
                    continue
                if t["until"] and now < t["until"]:
                    if now - self.page_seen > 1.0:
                        t["why"], t["until"] = "페이지가 1 s 넘게 안 보임", 0
                    elif not self.link.fresh(0.6):
                        t["why"], t["until"] = "보고 줄 끊김", 0
                    else:
                        self.link.lease(0.5)
                        self.link.drive(t["speed"], t["steer"])
                        r = self.link.report
                        for w, v in self.wheels(r)[0].items():
                            t["sum"][w] += v
                        t["n"] += 1
                        continue
                if "stopped_at" not in t:
                    self.link.stop()
                    t["stopped_at"] = now
                    continue
                if now - t["stopped_at"] < 0.8:                  # 멈춘 뒤 오도메트리가 자리 잡게
                    continue
                r = self.link.report
                dm, ds = r.odo[0] - t["odo0"][0], r.odo[1] - t["odo0"][1]
                mean = {w: round(s / max(t["n"], 1), 2) for w, s in t["sum"].items()}
                self.results.insert(0, {"name": t["name"], "cmd": t["cmd"], "seconds": t["seconds"], "why": t["why"] or "끝",
                                        "mean_kmh": mean, "odo": {"앞 오른쪽 (마스터)": dm, "앞 왼쪽 (슬레이브)": ds},
                                        "odo_fwd": {"앞 오른쪽": ODO_SIGN[0] * dm, "앞 왼쪽": ODO_SIGN[1] * ds},
                                        "t": time.strftime("%H:%M:%S")})
                self.results = self.results[:12]
                self.test = None

    def state(self):
        self.page_seen = time.monotonic()
        r = self.link.report
        out = {"ok": r is not None, "fresh": self.link.fresh(1.0), "armed": self.armed, "results": self.results,
               "events": [e for _, e in list(self.link.events)[-6:]], "max_cmd": MAX_CMD, "max_seconds": MAX_SECONDS}
        if r:
            fwd, raw = self.wheels(r)
            out.update(wheels=fwd, wheels_raw=raw, volt=r.volt, amps=r.amps, odo=r.odo, rx=r.rx, rx_b=r.rx_b,
                       out=r.out, cmd={"s": [r.speed_now, r.speed_target], "t": [r.steer_now, r.steer_target]})
        t = self.test
        if t:
            out["test"] = {"name": t["name"], "left": max(0.0, t["until"] - time.monotonic()) if t["until"] else 0}
        return out


PAGE = """<!doctype html><html lang="ko"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>누비 바퀴 점검</title>
<style>
:root{--bg:#f4f5f7;--card:#fff;--ink:#1f2328;--sub:#5d6670;--line:#dfe3e7;--acc:#2f7de1;--ok:#2b8a3e;--bad:#c62828}
@media (prefers-color-scheme:dark){:root{--bg:#16181b;--card:#202327;--ink:#e9edf1;--sub:#9aa4ae;--line:#33383e}}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--ink);font:14px "Malgun Gothic","Noto Sans CJK KR",system-ui,sans-serif}
main{max-width:980px;margin:0 auto;padding:14px 16px}h1{font-size:18px;margin:0 0 4px}.sub{color:var(--sub);font-size:12.5px}
.grid{display:grid;grid-template-columns:repeat(2,1fr);gap:10px;margin:12px 0}.card{background:var(--card);border:1px solid var(--line);border-radius:10px;padding:10px 12px}
.w h3{margin:0 0 6px;font-size:14px}.bar{position:relative;height:16px;background:var(--line);border-radius:8px;overflow:hidden}
.bar i{position:absolute;top:0;bottom:0;left:50%;background:var(--acc)}.bar b{position:absolute;left:50%;top:0;bottom:0;width:1px;background:var(--sub)}
.num{font-variant-numeric:tabular-nums;font-size:20px;font-weight:700;margin-top:4px}
.row{display:flex;flex-wrap:wrap;gap:8px;align-items:center;margin:8px 0}button{font:inherit;padding:9px 14px;border-radius:8px;border:1px solid var(--line);background:var(--card);color:var(--ink);cursor:pointer}
button:disabled{opacity:.4;cursor:not-allowed}button.go{border-color:var(--acc)}#stop{background:var(--bad);color:#fff;border:0;font-weight:800;font-size:18px;padding:14px 28px}
table{width:100%;border-collapse:collapse;font-size:13px}td,th{border-bottom:1px solid var(--line);padding:4px 6px;text-align:left;font-variant-numeric:tabular-nums}
.ok{color:var(--ok)}.bad{color:var(--bad)}label{display:flex;gap:6px;align-items:center}
@media (max-width:640px){.grid{grid-template-columns:1fr}}
</style></head><body><main>
<h1>누비 바퀴 점검</h1><div class="sub">ESP32 바퀴 제어기 · 0.2 s 마다 갱신 · <b>스페이스 = STOP</b> · 페이지가 1 s 끊기면 바퀴가 선다 · 속도 · 스텝은 <b>앞으로 = +</b> (보드 원시 값은 괄호)</div>
<div class="row"><span id="link" class="sub"></span></div>
<div class="grid" id="wheels"></div>
<div class="card"><div class="row">
<label><input type="checkbox" id="lifted"> <b>바퀴 네 개를 모두 띄웠다</b></label>
<label>세기 <input type="range" id="cmd" min="100" max="300" step="10" value="180"> <span id="cmdv">180</span></label>
<label>시간 <input type="range" id="sec" min="0.5" max="3" step="0.5" value="2"> <span id="secv">2</span> s</label></div>
<div class="row"><button class="go" data-t="앞으로">앞으로</button><button class="go" data-t="뒤로">뒤로</button>
<button class="go" data-t="왼쪽 제자리">왼쪽 제자리</button><button class="go" data-t="오른쪽 제자리">오른쪽 제자리</button>
<button id="stop">STOP</button> <span id="tst" class="sub"></span></div>
<div class="sub">볼 것: <b>앞으로</b> → 네 바퀴 모두 앞으로 · <b>왼쪽 제자리</b> → 오른쪽 두 바퀴 앞, 왼쪽 두 바퀴 뒤. 실제 도는 방향을 눈으로 보고 아래 기록과 맞춰 본다.</div></div>
<div class="card" style="margin-top:10px"><h3 style="margin:0 0 6px;font-size:14px">시험 기록 (바퀴별 평균 km/h · 앞축 홀 스텝 변화)</h3><table id="res"></table></div>
<div class="card" style="margin-top:10px"><div id="misc" class="sub"></div><div id="ev" class="sub" style="margin-top:6px"></div></div>
</main><script>
const $=id=>document.getElementById(id), W=["앞 왼쪽","앞 오른쪽","뒤 왼쪽","뒤 오른쪽"];
$("wheels").innerHTML=W.map(w=>`<div class="card w"><h3>${w}</h3><div class="bar"><b></b><i id="b-${w}"></i></div><div class="num" id="v-${w}">-</div></div>`).join("");
const post=(u,b)=>fetch(u,{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify(b||{})}).then(r=>r.json());
$("cmd").oninput=e=>$("cmdv").textContent=e.target.value;$("sec").oninput=e=>$("secv").textContent=e.target.value;
document.querySelectorAll("button.go").forEach(b=>b.onclick=async()=>{const r=await post("/api/test",{name:b.dataset.t,cmd:+$("cmd").value,seconds:+$("sec").value});if(r.error)alert(r.error)});
const stop=()=>post("/api/stop");$("stop").onclick=stop;addEventListener("keydown",e=>{if(e.code==="Space"){e.preventDefault();stop()}});
const fmt=v=>(v>=0?"+":"")+v.toFixed(2);
async function tick(){try{const s=await (await fetch("/api/state")).json();
 const running=!!s.test;document.querySelectorAll("button.go").forEach(b=>b.disabled=!$("lifted").checked||running||!s.fresh);
 $("link").innerHTML=s.ok?`<span class="${s.fresh?"ok":"bad"}">●</span> 보고 줄 ${s.fresh?"들어옴":"끊김"} · 앞축 rx ${s.rx} · 뒤축 rx2 ${s.rx_b} · ${s.volt.toFixed(1)} V · 워치독 ${s.armed?"켬":"아직"}`:"보고 줄 없음";
 if(s.wheels)for(const w of W){const v=s.wheels[w];$("v-"+w).textContent=fmt(v)+" km/h ("+fmt(s.wheels_raw[w])+")";const p=Math.max(-1,Math.min(1,v/3));const el=$("b-"+w);
  el.style.left=(p<0?50+p*50:50)+"%";el.style.width=Math.abs(p*50)+"%";el.style.background=v<0?"#d9480f":"#2f7de1"}
 $("tst").textContent=running?`${s.test.name} — ${s.test.left.toFixed(1)} s 남음`:"";
 $("res").innerHTML="<tr><th>시각</th><th>시험</th>"+W.map(w=>`<th>${w}</th>`).join("")+"<th>앞 오른쪽 스텝</th><th>앞 왼쪽 스텝</th><th>끝난 까닭</th></tr>"+
  s.results.map(r=>`<tr><td>${r.t}</td><td>${r.name} ${r.cmd} · ${r.seconds}s</td>`+W.map(w=>`<td>${fmt(r.mean_kmh[w])}</td>`).join("")+
  `<td>${r.odo_fwd["앞 오른쪽"]} (${r.odo["앞 오른쪽 (마스터)"]})</td><td>${r.odo_fwd["앞 왼쪽"]} (${r.odo["앞 왼쪽 (슬레이브)"]})</td><td>${r.why}</td></tr>`).join("");
 $("misc").textContent=s.ok?`명령 s ${s.cmd.s[0]}→${s.cmd.s[1]} · t ${s.cmd.t[0]}→${s.cmd.t[1]} · 바퀴 듀티 (왼, 오른) ${s.out.join(", ")} · 전류 앞축 ${s.amps.map(a=>a.toFixed(2)).join(" / ")} A · 앞축 odo ${s.odo.join(", ")} · 상한 ${s.max_cmd}`:"";
 $("ev").textContent="ESP32 알림: "+s.events.join(" | ");
}catch(e){$("link").innerHTML='<span class="bad">● 서버와 끊김</span>'}}
setInterval(tick,200);tick();
</script></body></html>"""


def make_handler(bench):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _send(self, code, body, ctype="application/json; charset=utf-8"):
            b = body if isinstance(body, bytes) else body.encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(b)))
            self.end_headers()
            self.wfile.write(b)

        def do_GET(self):
            if self.path == "/":
                self._send(200, PAGE, "text/html; charset=utf-8")
            elif self.path == "/api/state":
                self._send(200, json.dumps(bench.state(), ensure_ascii=False))
            else:
                self._send(404, "{}")

        def do_POST(self):
            n = int(self.headers.get("Content-Length", 0))
            req = json.loads(self.rfile.read(n) or b"{}")
            if self.path == "/api/stop":
                bench.stop()
                self._send(200, "{}")
            elif self.path == "/api/test":
                err = bench.start(req.get("name"), req.get("seconds", 2), req.get("cmd", 180))
                self._send(200, json.dumps({"error": err} if err else {}, ensure_ascii=False))
            else:
                self._send(404, "{}")
    return H


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="192.168.127.2", help="유선 직결 주소에만 연다 (학교 Wi-Fi 에서는 안 보이게)")
    ap.add_argument("--port", type=int, default=8090)
    ap.add_argument("--serial")
    ap.add_argument("--fake", action="store_true", help="가짜 ESP32 (PC 시험)")
    a = ap.parse_args()
    stop_on_signals()
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")      # Windows 콘솔 (cp949) 에서도
    if a.fake:
        from fake import FakeWheelSerial
        from hallway_map import HallwayMap, find_scene
        ser = FakeWheelSerial(HallwayMap(find_scene()).segs, deadband=100)
    else:
        ser = open_serial(a.serial)
    link = WheelLink(ser)
    try:
        r = link.wait_ready(need_board=False)
        print(f"바퀴 제어기 {link.port}: {r.volt:.1f} V, rx {r.rx} — http://{a.host}:{a.port}", flush=True)
        bench = Bench(link)
        ThreadingHTTPServer((a.host, a.port), make_handler(bench)).serve_forever()
    except KeyboardInterrupt:
        print("끝", flush=True)
    finally:
        link.close()


if __name__ == "__main__":
    main()
