"""시뮬레이터 정적 서버 — 캐시를 끈다 (모듈 .js 를 고쳐도 브라우저가 옛 것을 쓰지 않게).

    .venv\\Scripts\\python.exe models\\vehicle\\sim\\web\\serve.py [포트]   → http://localhost:8748
    포트: 인자 > 환경변수 PORT (미리보기 패널이 빈 포트를 정해 준다) > 8748

POST /api/agent/step 은 운전 한 걸음 — GPT · 모의는 ../agent.py, claude-* 는 ../agent_claude.py.
POST /api/render/save 는 sensors.renderView 로 그린 컬러 · 깊이를 ../agent_logs/renders/ 에 남긴다 (실차 사진 비교용).
API 키는 서버에만 있고 브라우저로 보내지 않는다.

연결 (?bridge=1 로 연 페이지 ↔ 실차 운전 프로그램 drive.py --sim):
    POST /api/bridge/job      파이썬 → 일 하나 {pose, status, views:[renderView 인자 + name], wait}. views 가 있으면 브라우저가
                              그린 결과를 기다려 돌려준다 (wait=false 면 바로 돌아옴 — 자세 · 상태 표시만).
    GET  /api/bridge/next     브라우저 → 다음 일 (없으면 1 s 기다렸다 빈 답)
    POST /api/bridge/result   브라우저 → 그린 결과 {seq, views:{name:{jpg, depth}}}
"""

import base64
import functools
import http.server
import json
import os
import re
import sys
import threading
import time
from collections import deque
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))

BRIDGE = dict(cv=threading.Condition(), jobs=deque(), results={}, seq=0, last_seen=0.0)


class NoCache(http.server.SimpleHTTPRequestHandler):
    def end_headers(self):
        self.send_header("Cache-Control", "no-store")
        super().end_headers()

    def log_message(self, fmt, *args):
        if "/api/bridge/" not in (args[0] if args else ""):   # 연결은 초당 여러 번 — 기록에서 뺀다
            super().log_message(fmt, *args)

    def _json(self, obj):
        body = json.dumps(obj, ensure_ascii=False).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _body(self):
        return json.loads(self.rfile.read(int(self.headers.get("Content-Length", 0))).decode("utf-8"))

    def do_GET(self):
        if self.path.startswith("/api/bridge/next"):
            b = BRIDGE
            with b["cv"]:
                b["last_seen"] = time.monotonic()
                if not b["jobs"]:
                    b["cv"].wait(1.0)
                job = b["jobs"].popleft() if b["jobs"] else {}
            self._json(job)
            return
        if self.path.startswith("/api/bridge/alive"):
            self._json({"page": time.monotonic() - BRIDGE["last_seen"] < 3.0})
            return
        super().do_GET()

    def do_POST(self):
        if self.path == "/api/bridge/job":
            job, b = self._body(), BRIDGE
            with b["cv"]:
                if time.monotonic() - b["last_seen"] > 3.0:
                    self._json({"ok": False, "error": "시뮬레이터 페이지가 안 열려 있다 (?bridge=1)"})
                    return
                b["seq"] += 1
                job["seq"] = seq = b["seq"]
                if not job.get("views"):                        # 자세만 바꾸는 일은 밀린 것 중 마지막 하나만 남긴다
                    b["jobs"] = deque(j for j in b["jobs"] if j.get("views"))
                b["jobs"].append(job)
                b["cv"].notify_all()
                if not job.get("views") or job.get("wait") is False:
                    self._json({"ok": True, "seq": seq})
                    return
                t_end = time.monotonic() + float(job.get("timeout", 15))
                while seq not in b["results"] and time.monotonic() < t_end:
                    b["cv"].wait(t_end - time.monotonic())
                res = b["results"].pop(seq, None)
            self._json({"ok": res is not None, "seq": seq, "views": (res or {}).get("views"),
                        "error": None if res else "시뮬레이터가 제때 그리지 못했다"})
            return
        if self.path == "/api/bridge/result":
            res, b = self._body(), BRIDGE
            with b["cv"]:
                b["results"][res["seq"]] = res
                b["cv"].notify_all()
            self._json({"ok": True})
            return
        if self.path == "/api/render/save":                 # 3D 지도 렌더 (실차 사진과 비교) → ../agent_logs/renders/ (git 밖)
            req = self._body()
            out = HERE.parent / "agent_logs" / "renders"
            out.mkdir(parents=True, exist_ok=True)
            name = re.sub(r"[^\w.-]", "_", str(req["name"]))
            (out / f"{name}_rgb.jpg").write_bytes(base64.b64decode(req["jpg"].split(",", 1)[1]))
            (out / f"{name}_depth.u16").write_bytes(base64.b64decode(req["depth"]))
            (out / f"{name}.json").write_text(json.dumps({"w": req["w"], "h": req["h"], "depth": "uint16 mm, 광축 거리"}))
            self._json({"ok": True, "dir": str(out)})
            return
        if self.path != "/api/agent/step":
            self.send_error(404)
            return
        try:
            req = self._body()
            if str(req.get("model", "")).startswith("claude-"):
                import agent_claude as driver               # Claude (Anthropic SDK) — 처음 부를 때만 불러온다
            else:
                import agent as driver                      # GPT (표준 라이브러리) · 모의
            res = driver.step(req)
        except Exception as e:                              # 브라우저 패널에 그대로 보여 준다
            res = {"ok": False, "error": f"{type(e).__name__}: {e}"}
        self._json(res)


if __name__ == "__main__":
    port = int(sys.argv[1]) if len(sys.argv) > 1 else int(os.environ.get("PORT", 8748))
    handler = functools.partial(NoCache, directory=str(HERE))
    http.server.ThreadingHTTPServer(("127.0.0.1", port), handler).serve_forever()
