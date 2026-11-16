"""실차 상태 화면 — 휴대폰 · 태블릿 브라우저로 본다 (보기만, 조종은 게임패드).

    python3 status_web.py [--port 8095]          bash drive_ctl.sh start 가 같이 띄운다

drive.py 가 /tmp/nubi_status/ 에 남기는 단계 · 결정 · 사진 · 지도와 /tmp/drive.log 끝, 그리고 이 서버가 5 s 마다 재는 인터넷
(api.openai.com 이름 찾기 · 443 연결) · Wi-Fi (이름 · 세기 · 주소) 를 한 화면에. 왼쪽은 카메라 (Xtion · 웹캠 지금 화면, 모델이
받은 사진, 위치 인식 사진, 지도 — 누르면 크게), 오른쪽은 상태 · 결정 · 알림 · 기록. 좁은 화면 (휴대폰 세로) 에서는 위아래로 쌓인다.
Jetson 의 모든 주소에 열리므로 (휴대폰 핫스팟에서 보려고) 주소에 비밀 토큰 ?k=… 이 있어야 보인다 — 토큰은
~/.nubi_status_token (처음 띄울 때 만든다). 사진에 사람이 찍힌다.
"""

import argparse
import json
import secrets
import socket
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

DIR = Path("/tmp/nubi_status")
LOG = Path("/tmp/drive.log")
TOKEN_FILE = Path.home() / ".nubi_status_token"
NET = dict(internet=None, ms=None, error="", wifi="", ips=[], t=0)


def token():
    if not TOKEN_FILE.exists():
        TOKEN_FILE.write_text(secrets.token_urlsafe(8))
        TOKEN_FILE.chmod(0o600)
    return TOKEN_FILE.read_text().strip()


def net_loop():
    while True:
        t0 = time.monotonic()
        try:
            addr = socket.getaddrinfo("api.openai.com", 443, proto=socket.IPPROTO_TCP)[0][4]
            with socket.create_connection(addr[:2], timeout=4):
                pass
            NET.update(internet=True, ms=round((time.monotonic() - t0) * 1000), error="")
        except Exception as e:
            NET.update(internet=False, ms=None, error=f"{type(e).__name__}: {e}"[:120])
        try:
            out = subprocess.run(["nmcli", "-t", "-f", "IN-USE,SSID,SIGNAL", "dev", "wifi"], capture_output=True, text=True,
                                 timeout=4).stdout
            cur = [l.split(":") for l in out.splitlines() if l.startswith("*")]
            NET["wifi"] = f"{cur[0][1]} ({cur[0][2]} %)" if cur else "Wi-Fi 안 붙음"
            ips = subprocess.run(["ip", "-4", "-o", "addr"], capture_output=True, text=True, timeout=4).stdout
            NET["ips"] = [f"{p[1]} {p[3]}" for p in (l.split() for l in ips.splitlines()) if p[1] not in ("lo", "docker0")]
        except Exception:
            pass
        NET["t"] = time.time()
        time.sleep(5)


def tail(path, lines=80, nbytes=32768):
    """긴 기록도 끝 32 KB 만 읽는다 (0.7 s 마다 불리므로)."""
    try:
        with open(path, "rb") as f:
            f.seek(0, 2)
            size = f.tell()
            f.seek(max(0, size - nbytes))
            text = f.read().decode("utf-8", errors="replace").splitlines()
        if size > nbytes:
            text = text[1:]                                  # 잘린 첫 줄
        return "\n".join(text[-lines:])
    except OSError:
        return ""


PAGE = """<!doctype html><html lang="ko"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>누비 상태</title><style>
:root{--bg:#f6f6f4;--card:#fff;--ink:#1b1b1a;--mut:#6b6a66;--line:#dcdbd5;--ok:#0c8a0c;--bad:#d03b3b;--warn:#b97a00;--hl:#2457c5}
@media (prefers-color-scheme:dark){:root{--bg:#161615;--card:#1f1f1e;--ink:#ecebe6;--mut:#a2a19b;--line:#383835;--ok:#4cc24c;--bad:#ef6b6b;--warn:#e0a63a;--hl:#7fa6ff}}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--ink);font:14px/1.45 system-ui,"Malgun Gothic",sans-serif}
main{display:grid;grid-template-columns:minmax(0,1.6fr) minmax(320px,1fr);gap:10px;padding:10px;height:100vh;height:100dvh}
.left{overflow:auto;min-height:0}
.side{display:flex;flex-direction:column;gap:8px;min-height:0}
#phase{font-size:20px;font-weight:700;margin:0 0 8px;line-height:1.3}
.cams{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:8px}
.cam{background:var(--card);border:1px solid var(--line);border-radius:8px;padding:4px;cursor:zoom-in}
.cam.wide,.cam.big{grid-column:1/-1}.cam.big{cursor:zoom-out}
.ph{position:relative;aspect-ratio:4/3;background:#111;border-radius:5px;overflow:hidden}
.cam.wide .ph{aspect-ratio:16/9}
.ph span{position:absolute;inset:0;display:grid;place-items:center;color:#888;font-size:12px}
.ph img{position:absolute;inset:0;width:100%;height:100%;object-fit:contain}.ph img:not([src]){display:none}
.cap{display:flex;justify-content:space-between;gap:6px;font-size:12px;color:var(--mut);padding:2px 2px 0}
.t{font-size:12px;color:var(--mut);font-weight:400}
.stats{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:6px}
.st{background:var(--card);border:1px solid var(--line);border-radius:8px;padding:4px 8px;min-width:0}
.st b{display:block;font-size:11px;font-weight:400;color:var(--mut)}.st div{font-size:14px;font-weight:600;overflow-wrap:anywhere}
.ok{color:var(--ok)}.bad{color:var(--bad)}.warn{color:var(--warn)}.hl{color:var(--hl)}
.card{background:var(--card);border:1px solid var(--line);border-radius:8px;padding:6px 10px;min-height:0}
.card>b{font-size:12px;color:var(--mut);font-weight:400}
#evc{flex:0 1 auto;max-height:26%;overflow:auto}
#ev div{font-size:13px;border-bottom:1px solid var(--line);padding:2px 0}
#logc{flex:1 1 0;display:flex;flex-direction:column;min-height:140px}
pre{white-space:pre-wrap;overflow-wrap:anywhere;font-size:12px;margin:0;flex:1;overflow:auto}
#ips{font-size:11px;color:var(--mut)}
@media (max-width:820px),(max-height:500px){
 main{display:flex;flex-direction:column;height:auto;padding:8px}
 .left,.side{display:contents}
 #phase{order:0;margin:0}.stats{order:1}#decc{order:2}.cams{order:3}#evc{order:4;max-height:40vh}#logc{order:5}#ips{order:6}
 #logc pre{max-height:60vh}
}
</style></head><body><main>
<div class="left">
<div id="phase">…</div>
<div class="cams">
<div class="cam"><div class="ph"><span>Xtion 없음</span><img id="i_live_xtion" alt=""></div><div class="cap"><span>Xtion 지금</span><span id="a_live_xtion"></span></div></div>
<div class="cam"><div class="ph"><span>웹캠 없음</span><img id="i_live_webcam" alt=""></div><div class="cap"><span>웹캠 지금</span><span id="a_live_webcam"></span></div></div>
<div class="cam"><div class="ph"><span>아직 없음</span><img id="i_xtion" alt=""></div><div class="cap"><span>모델이 받은 사진 (화살표)</span><span id="a_xtion"></span></div></div>
<div class="cam"><div class="ph"><span>아직 없음</span><img id="i_webcam" alt=""></div><div class="cap"><span>위치 인식에 쓴 웹캠 사진</span><span id="a_webcam"></span></div></div>
<div class="cam wide"><div class="ph"><span>아직 없음</span><img id="i_map" alt=""></div><div class="cap"><span>지도 · 추정 위치</span><span id="a_map"></span></div></div>
</div>
</div>
<div class="side">
<div class="stats">
<div class="st"><b>인터넷 (OpenAI)</b><div id="net">…</div></div>
<div class="st"><b>Wi-Fi</b><div id="wifi">…</div></div>
<div class="st"><b>전압 · 보드</b><div id="volt">…</div></div>
<div class="st"><b>걸음 · 목표 · 모델</b><div id="step">…</div></div>
<div class="st"><b>위치 추정</b><div id="pose">…</div></div>
<div class="st"><b>위치 인식 (웹캠)</b><div id="loc">…</div></div>
</div>
<div class="card" id="decc"><b>결정</b><div id="dec">–</div></div>
<div class="card" id="evc"><b>알림 (새것이 위)</b><div id="ev"></div></div>
<div class="card" id="logc"><b>기록 (/tmp/drive.log 끝)</b><pre id="log"></pre></div>
<div id="ips"></div>
</div>
</main><script>
const K = new URLSearchParams(location.search).get('k') || '';
const seen = {};
let lastLog = null;
const $ = id => document.getElementById(id);
function esc(s){return String(s).replace(/[&<>]/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;'}[c]))}
function ago(a){return a < 1.5 ? '방금' : a < 90 ? Math.round(a) + ' s 전' : Math.round(a / 60) + ' 분 전'}
document.querySelectorAll('.cam').forEach(c => c.onclick = () => c.classList.toggle('big'));
function img(name, t, now, stale){
 const el = $('i_' + name), a = $('a_' + name);
 if (a) { a.textContent = ago(now - t); a.className = now - t > stale ? 'warn' : ''; }
 if (!el || seen[name] === t || el.dataset.busy) return;          // 앞 사진을 다 받기 전에는 새로 안 부른다 (느린 핫스팟)
 seen[name] = t; el.dataset.busy = 1;
 el.onload = el.onerror = () => { delete el.dataset.busy; };
 el.src = '/img/' + name + '.jpg?k=' + K + '&t=' + t;
}
async function tick(){
 try{
  const s = await (await fetch('/state?k=' + K)).json(), d = s.drive || {}, n = s.net || {};
  const age = d.t ? (s.now - d.t) : 1e9;                 // Jetson 시계로 (PC · 휴대폰 시계와 어긋나도)
  $('phase').innerHTML = (s.alive ? '' : '<span class="bad">운전 프로그램 꺼짐</span> · ') + esc(d.phase || '–')
     + (d.t ? ' <span class="t">(' + ago(age) + ')</span>' : '');
  $('net').innerHTML = n.internet === true ? '<span class="ok">됨</span> ' + n.ms + ' ms' : n.internet === false ? '<span class="bad">안 됨</span> <small>' + esc(n.error) + '</small>' : '확인 중';
  $('wifi').textContent = n.wifi || '–';
  $('volt').innerHTML = d.volt !== undefined ? d.volt.toFixed(1) + ' V · ' + (d.board_ok ? '<span class="ok">링크</span>' : '<span class="bad">보드 없음</span>') : '–';
  $('step').textContent = (d.step != null ? '#' + d.step : '–') + ' · ' + (d.goal || '–') + ' · ' + (d.model || '–');
  $('pose').textContent = d.pose ? `(${d.pose.x.toFixed(2)}, ${d.pose.y.toFixed(2)}) ${d.pose.yaw_deg.toFixed(0)}°` : '–';
  $('loc').innerHTML = d.loc ? (d.loc.ok ? '<span class="ok">' + esc(d.loc.text) + '</span>' : '<span class="warn">' + esc(d.loc.text) + '</span>') : '–';
  $('dec').innerHTML = d.decision ? '<b>' + esc(d.decision.what || '') + '</b> — ' + esc(d.decision.reason || '') + '<br><span class="t">보임: ' + esc(d.decision.observation || '') + (d.latency_s ? ' · ' + d.latency_s + ' s' : '') + '</span>' + (d.outcome ? '<br>결과: ' + esc(d.outcome) : '') + (d.error ? '<br><span class="bad">' + esc(d.error) + '</span>' : '') : (d.error ? '<span class="bad">' + esc(d.error) + '</span>' : '–');
  $('ev').innerHTML = (d.events || []).slice().reverse().map(e => '<div><span class="t">' + esc(e[0]) + '</span> ' + esc(e[1]) + '</div>').join('');
  if (s.log !== lastLog) {                                // 바뀔 때만 (읽던 곳 · 고른 글자 유지), 맨 아래를 보고 있었으면 따라 내려감
   lastLog = s.log;
   const el = $('log'), bottom = el.scrollHeight - el.scrollTop - el.clientHeight < 40;
   el.innerHTML = (s.log || '').split('\\n').map(l => {
    const c = /오류|Error|Traceback|!!|실패|끊김|안 된다|안 보인다/.test(l) ? 'bad' : /→|결과:/.test(l) ? 'hl' : '';
    return c ? '<span class="' + c + '">' + esc(l) + '</span>' : esc(l);
   }).join('\\n');
   if (bottom) el.scrollTop = el.scrollHeight;
  }
  $('ips').textContent = '주소: ' + (n.ips || []).join(' · ');
  for (const [name, t] of Object.entries(s.live || {})) img(name, t, s.now, 5);     // 지금 화면 — 5 s 넘게 안 바뀌면 노란 글씨
  for (const [name, t] of Object.entries(d.images || {})) img(name, t, s.now, 1e9);
 }catch(e){ $('phase').innerHTML = '<span class="bad">Jetson 과 연결이 끊김</span>'; }
}
setInterval(tick, 700); tick();
</script></body></html>"""


def serve(port):
    tok = token()

    class H(BaseHTTPRequestHandler):
        def log_message(self, fmt, *a):                     # 누가 들어왔는지만 (토큰은 빼고) — 휴대폰이 닿는지 볼 때
            if "/img/" not in self.path and "/state" not in self.path:
                print(time.strftime("%H:%M:%S"), self.client_address[0], self.command, self.path.split("?")[0], flush=True)

        def _ok(self):
            q = parse_qs(urlparse(self.path).query)
            if q.get("k", [""])[0] != tok:
                self.send_response(403)
                self.end_headers()
                self.wfile.write("토큰이 없거나 틀림 (?k=…)".encode("utf-8"))
                return False
            return True

        def _send(self, body, ctype):
            self.send_response(200)
            self.send_header("Content-Type", ctype)
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            path = urlparse(self.path).path
            if not self._ok():
                return
            if path == "/":
                self._send(PAGE.encode("utf-8"), "text/html; charset=utf-8")
            elif path == "/state":
                try:
                    drive = json.loads((DIR / "state.json").read_text(encoding="utf-8"))
                except Exception:
                    drive = {}
                live = {}                                    # Status.live 가 0.5 s 마다 바꿔 쓰는 지금 화면 — 파일 시각으로
                for p in DIR.glob("live_*.jpg"):
                    if ".tmp" not in p.name:
                        try:
                            live[p.stem] = p.stat().st_mtime
                        except OSError:
                            pass
                alive = subprocess.run(["pgrep", "-f", "python3 -u drive[.]py"], capture_output=True).returncode == 0
                self._send(json.dumps(dict(drive=drive, live=live, net=NET, log=tail(LOG), now=time.time(), alive=alive),
                                      ensure_ascii=False).encode("utf-8"), "application/json; charset=utf-8")
            elif path.startswith("/img/") and path.endswith(".jpg"):
                p = DIR / Path(path).name
                if p.exists():
                    self._send(p.read_bytes(), "image/jpeg")
                else:
                    self.send_response(404)
                    self.end_headers()
            else:
                self.send_response(404)
                self.end_headers()

    threading.Thread(target=net_loop, daemon=True).start()
    srv = ThreadingHTTPServer(("0.0.0.0", port), H)
    srv.daemon_threads = True
    print(f"상태 화면: http://<Jetson 주소>:{port}/?k={tok}", flush=True)
    srv.serve_forever()


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=8095)
    serve(ap.parse_args().port)
