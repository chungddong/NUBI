"""시뮬레이터 정적 서버 — 캐시를 끈다 (모듈 .js 를 고쳐도 브라우저가 옛 것을 쓰지 않게).

    .venv\\Scripts\\python.exe models\\vehicle\\sim\\web\\serve.py [포트]   → http://localhost:8748
    포트: 인자 > 환경변수 PORT (미리보기 패널이 빈 포트를 정해 준다) > 8748

POST /api/agent/step 은 운전 한 걸음 — GPT · 모의는 ../agent.py, claude-* 는 ../agent_claude.py.
API 키는 서버에만 있고 브라우저로 보내지 않는다.
"""

import functools
import http.server
import json
import os
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))


class NoCache(http.server.SimpleHTTPRequestHandler):
    def end_headers(self):
        self.send_header("Cache-Control", "no-store")
        super().end_headers()

    def do_POST(self):
        if self.path != "/api/agent/step":
            self.send_error(404)
            return
        try:
            req = json.loads(self.rfile.read(int(self.headers.get("Content-Length", 0))).decode("utf-8"))
            if str(req.get("model", "")).startswith("claude-"):
                import agent_claude as driver               # Claude (Anthropic SDK) — 처음 부를 때만 불러온다
            else:
                import agent as driver                      # GPT (표준 라이브러리) · 모의
            res = driver.step(req)
        except Exception as e:                              # 브라우저 패널에 그대로 보여 준다
            res = {"ok": False, "error": f"{type(e).__name__}: {e}"}
        body = json.dumps(res, ensure_ascii=False).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)


if __name__ == "__main__":
    port = int(sys.argv[1]) if len(sys.argv) > 1 else int(os.environ.get("PORT", 8748))
    handler = functools.partial(NoCache, directory=str(HERE))
    http.server.ThreadingHTTPServer(("127.0.0.1", port), handler).serve_forever()
