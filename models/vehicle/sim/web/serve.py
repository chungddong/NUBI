"""시뮬레이터 정적 서버 — 캐시를 끈다 (모듈 .js 를 고쳐도 브라우저가 옛 것을 쓰지 않게).

    .venv\\Scripts\\python.exe models\\vehicle\\sim\\web\\serve.py [포트]   → http://localhost:8748
    포트: 인자 > 환경변수 PORT (미리보기 패널이 빈 포트를 정해 준다) > 8748
"""

import functools
import http.server
import os
import sys
from pathlib import Path


class NoCache(http.server.SimpleHTTPRequestHandler):
    def end_headers(self):
        self.send_header("Cache-Control", "no-store")
        super().end_headers()


if __name__ == "__main__":
    port = int(sys.argv[1]) if len(sys.argv) > 1 else int(os.environ.get("PORT", 8748))
    handler = functools.partial(NoCache, directory=str(Path(__file__).resolve().parent))
    http.server.ThreadingHTTPServer(("127.0.0.1", port), handler).serve_forever()
