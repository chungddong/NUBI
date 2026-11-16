"""실차 운전 상태를 파일로 남긴다 — status_web.py 가 휴대폰 · 태블릿 화면으로 보인다 (drive.py 가 쓴다).

/tmp/nubi_status/state.json (단계 · 걸음 · 결정 · 위치 · 전압 …) 과 xtion.jpg · webcam.jpg · map.jpg (걸음마다), live_xtion.jpg ·
live_webcam.jpg (0.5 s 마다, Status.live). 쓰기는 임시 파일을 만든 뒤
바꿔치기 (os.replace) 해서 화면이 반쯤 쓴 파일을 읽지 않게 한다. 실패해도 운전에는 영향이 없게 모든 예외를 삼킨다.
"""

import json
import os
import tempfile
import time
from pathlib import Path

DIR = Path("/tmp/nubi_status") if os.name == "posix" else Path(tempfile.gettempdir()) / "nubi_status"


class Status:
    def __init__(self, path=DIR):
        self.dir = Path(path)
        self.d = dict(phase="시작", events=[], started=time.time())
        try:
            self.dir.mkdir(parents=True, exist_ok=True)
        except OSError:
            pass

    def _write(self):
        try:
            self.d["t"] = time.time()
            tmp = self.dir / "state.json.tmp"
            tmp.write_text(json.dumps(self.d, ensure_ascii=False, default=str), encoding="utf-8")
            os.replace(tmp, self.dir / "state.json")
        except Exception:
            pass

    def set(self, **kw):
        self.d.update(kw)
        self._write()

    def event(self, text):
        self.d["events"] = (self.d.get("events", []) + [[time.strftime("%H:%M:%S"), str(text)]])[-30:]
        self._write()

    def phase(self, text, **kw):
        self.d.update(kw, phase=text)
        self.event(text)

    def image(self, name, img):
        """img: OpenCV BGR 배열 또는 PIL 그림."""
        try:
            tmp = self.dir / f"{name}.tmp.jpg"
            if hasattr(img, "save"):
                img.convert("RGB").save(tmp, quality=80)
            else:
                import cv2
                h, w = img.shape[:2]
                if w > 960:
                    img = cv2.resize(img, (960, round(h * 960 / w)))
                cv2.imwrite(str(tmp), img, [cv2.IMWRITE_JPEG_QUALITY, 80])
            os.replace(tmp, self.dir / f"{name}.jpg")
            self.d.setdefault("images", {})[name] = time.time()
            self._write()
        except Exception:
            pass

    def live(self, sources, every=0.5, width=640):
        """sources: {이름: 최신 BGR 배열을 돌려주는 함수 (없으면 None)}. 따로 도는 스레드가 every 초마다 live_<이름>.jpg 로
        남긴다 — 걸음 사이 · 손 운전 · 준비 중에도 화면이 산다. state.json 은 안 건드리고 status_web.py 가 파일 시각을 본다."""
        import threading

        def loop():
            import cv2
            seen = {}
            while True:
                for name, get in sources.items():
                    try:
                        img = get()
                        if img is None or img is seen.get(name):
                            continue
                        seen[name] = img
                        h, w = img.shape[:2]
                        if w > width:
                            img = cv2.resize(img, (width, round(h * width / w)), interpolation=cv2.INTER_AREA)
                        tmp = self.dir / f"live_{name}.tmp.jpg"
                        cv2.imwrite(str(tmp), img, [cv2.IMWRITE_JPEG_QUALITY, 70])
                        os.replace(tmp, self.dir / f"live_{name}.jpg")
                    except Exception:
                        pass
                time.sleep(every)

        threading.Thread(target=loop, daemon=True).start()
