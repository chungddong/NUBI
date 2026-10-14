"""drive.py --sim — 실차 코드를 그대로 두고 차와 카메라만 시뮬레이터로 바꾼다.

    바퀴: 가상 ESP32 (fake.FakeWheelSerial — 펌웨어 명령 · 램프 · 클램프 · 혼합 · 홀 스텝을 흉내, 벽에 막힘)
    카메라: 그 가상 차의 지금 자세에서 시뮬레이터 3D 지도가 그린 Xtion 컬러 · 깊이 (SimCamera)
    위치 인식: 웹캠 + SfM 지도는 렌더 사진으로는 안 되므로 실측 성능으로 흉내 (SimVision)
    화면: 시뮬레이터 페이지 (?env=hallway&cam=xtion&bridge=1) 의 차가 가상 차를 따라 움직이고, 바퀴 명령 · 추정 위치가 보인다.

serve.py 의 /api/bridge/* 로 페이지와 주고받는다 (같은 PC, http://127.0.0.1:8748).
"""

import base64
import json
import math
import random
import threading
import time
import urllib.request

import cv2
import numpy as np

from xtion import Frame, intrinsics


def cam_R(yaw, up_deg):
    """카메라 축 (열 = 오른쪽 · 아래 · 앞) 을 장면 좌표로 — 방향 yaw (rad, 왼쪽 +), 위로 든 각 up_deg."""
    p = math.radians(up_deg)
    z = np.array([math.cos(p) * math.cos(yaw), math.cos(p) * math.sin(yaw), math.sin(p)])
    x = np.array([math.sin(yaw), -math.cos(yaw), 0.0])
    return np.stack([x, np.cross(z, x), z], 1).tolist()


class SimBridge:
    def __init__(self, robot, url="http://127.0.0.1:8748"):
        self.robot, self.url = robot, url.rstrip("/")
        self.note = ""
        try:
            alive = json.loads(urllib.request.urlopen(self.url + "/api/bridge/alive", timeout=3).read())["page"]
        except Exception as e:
            raise RuntimeError(f"시뮬레이터 서버 ({self.url}) 에 못 붙는다: {e}")
        if not alive:
            raise RuntimeError(f"시뮬레이터 페이지를 {self.url}/?env=hallway&cam=xtion&bridge=1 로 열어 둔다")
        self._alive = True
        threading.Thread(target=self._follow, daemon=True).start()

    def _post(self, obj, timeout=20):
        req = urllib.request.Request(self.url + "/api/bridge/job", data=json.dumps(obj).encode("utf-8"),
                                     headers={"Content-Type": "application/json"})
        return json.loads(urllib.request.urlopen(req, timeout=timeout).read())

    def pose(self):
        r = self.robot
        return dict(x=r.x, y=r.y, yaw=r.yaw)

    def status(self):
        r = self.robot
        return (f"가상 ESP32  s {r.s_t:+d}  t {r.t_t:+d}  →  바퀴 출력 왼쪽 {r.out[0]:+d} · 오른쪽 {r.out[1]:+d}"
                f"  (클램프 {r.maxc})\n{self.note}")

    def render(self, views, timeout=15):
        res = self._post(dict(pose=self.pose(), status=self.status(), views=views, timeout=timeout), timeout + 5)
        if not res.get("ok"):
            print("  시뮬레이터:", res.get("error"))
            return None
        return res["views"]

    def _follow(self):
        """그리는 일이 없을 때도 페이지의 차가 가상 차를 따라가게 (초당 5 번)."""
        while self._alive:
            try:
                self._post(dict(pose=self.pose(), status=self.status()), 3)
            except Exception:
                pass
            time.sleep(0.2)

    def close(self):
        self._alive = False


class SimCamera:
    """Xtion 대신 — 가상 차의 지금 자세에서 그린 640 × 480 컬러 · 깊이 (깊이는 Xtion 처럼 0.5 ~ 4 m 밖 = 0)."""

    def __init__(self, bridge, robot, cfg, w=640, h=480, hfov=58.6, vfov=45.6, near=0.5, far=4.0):
        self.bridge, self.robot, self.w, self.h = bridge, robot, w, h
        self.K = intrinsics(w, h, math.radians(hfov), math.radians(vfov))
        self.cam_x = cfg.get("camera_x_m", 0.38)
        self.cam_h = cfg.get("camera_height_m") or 0.30          # 2026-10-14 첫 주행: Xtion 바닥 맞추기 0.30 m
        self.up = -(cfg.get("camera_pitch_deg") or -2.5)         # 같은 날 바닥 맞추기: 2.5° 위를 봄
        self.near, self.far = near, far

    def read(self, timeout=None):
        r = self.robot
        c, s = math.cos(r.yaw), math.sin(r.yaw)
        view = dict(name="xtion", pos=[r.x + self.cam_x * c, r.y + self.cam_x * s, self.cam_h], R=cam_R(r.yaw, self.up),
                    K=list(self.K), size=[self.w, self.h])
        v = self.bridge.render([view])
        if not v:
            return None
        v = v["xtion"]
        bgr = cv2.imdecode(np.frombuffer(base64.b64decode(v["jpg"].split(",", 1)[1]), np.uint8), cv2.IMREAD_COLOR)
        d = np.frombuffer(base64.b64decode(v["depth"]), np.uint16).reshape(self.h, self.w).astype(np.float32) / 1000
        d[(d < self.near) | (d > self.far)] = 0.0
        return Frame(bgr, d, self.K, time.monotonic())

    def close(self):
        self.bridge.close()


class SimVision:
    """웹캠 위치 인식 흉내 — 2026-10-14 첫 주행 다시 돌려 보기 (replay.py) 실측 값:
    지도 영상과 같은 방향 (복도 안쪽 · 로비 고리) 을 볼 때 p_ok 로 찾고 위치 · 방향에 σ 만큼 오차, 반대 방향이면 못 찾는다."""

    def __init__(self, robot, cam_off, p_ok=0.9, sigma_m=0.05, sigma_deg=1.5, seed=0):
        self.robot, self.off = robot, cam_off
        self.p_ok, self.sm, self.sd = p_ok, sigma_m, sigma_deg
        self.rng = random.Random(seed)

    def locate(self, prior=None):
        r = self.robot
        facing_map = math.cos(r.yaw) > math.cos(math.radians(70)) or (r.x > 35 and r.y < 0)   # 로비 고리는 지도에 양쪽 다 있음
        if not (-1.5 < r.x < 43.0) or not facing_map or self.rng.random() > self.p_ok:
            return None
        fx, fy, fyaw = self.off
        cx = r.x + fx * math.cos(r.yaw) - fy * math.sin(r.yaw)
        cy = r.y + fx * math.sin(r.yaw) + fy * math.cos(r.yaw)
        return dict(t=time.monotonic(), x=cx + self.rng.gauss(0, self.sm), y=cy + self.rng.gauss(0, self.sm),
                    yaw=math.degrees(r.yaw) + fyaw + self.rng.gauss(0, self.sd), inliers=60, ms=0)
