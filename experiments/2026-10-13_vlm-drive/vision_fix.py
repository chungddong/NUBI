"""실차 위치 인식 — 웹캠 사진 한 장을 SfM 복도 지도에 맞춰 카메라 자세를 낸다 (drive.py --pose vis).

웹캠 (record.Webcam) 과 지도 (../2026-10-13_sfm-relocalization/localizer.py) 를 묶는다. 시뮬레이터에서는 sim_bridge.SimVision.
"""

import math
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "2026-10-13_sfm-relocalization"))


class RealVision:
    def __init__(self, cfg, device="auto", min_inliers=25):
        from localizer import Localizer, pinhole
        from record import Webcam, find_webcam
        dev, name = find_webcam() if device == "auto" else (device, "")
        if dev is None:
            raise RuntimeError("웹캠이 안 보인다")
        self.cam = Webcam(dev, name)
        w, h = self.cam.size
        hfov = cfg.get("webcam_hfov_deg", 70.0)
        self.K = pinhole(w, h, (w / 2) / math.tan(math.radians(hfov) / 2))
        self.loc = Localizer()
        self.min_inliers = min_inliers
        print(f"  위치 인식: 웹캠 {dev} {w}x{h} (화각 {hfov}°), 지도 {self.loc.t_load:.1f} s", flush=True)

    def locate(self, prior=None):
        n0 = self.cam.n
        t_end = time.monotonic() + 1.0
        while self.cam.n == n0 and time.monotonic() < t_end:   # 멈춘 뒤 새로 찍힌 사진
            time.sleep(0.01)
        t, img = self.cam.t, self.cam.latest
        r = self.loc.locate(img, self.K, prior=prior, min_inliers=self.min_inliers, h_range=(0.0, 0.8))
        if not r["ok"]:
            return None
        return dict(t=t, x=r["x"], y=r["y"], yaw=r["yaw"], inliers=r["inliers"], ms=r["ms"])

    def close(self):
        self.cam.close()
