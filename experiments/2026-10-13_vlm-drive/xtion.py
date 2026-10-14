"""ASUS Xtion Pro Live — OpenNI2 (libOpenNI2.so) + 파이썬 바인딩 `openni` (없으면 `primesense`).

순서는 2026-09-18 초파리 주행에서 이 Jetson 으로 Xtion 을 읽은 코드 (~/Study/flysim/fly/retina.py) 와 같다:
모드 → 깊이를 컬러 시점에 맞춤 (IMAGE_REGISTRATION_DEPTH_TO_COLOR, 시작 **전**) → 시작 → 백그라운드 스레드로 계속 읽기.
장치 설정 (미러링 · 깊이-컬러 동기화) 은 바꾸지 않는다 — 2026-10-14 시작 뒤에 그런 설정을 넣은 첫 판이 프레임을 못 받고
멈췄고, 강제 종료 뒤 Xtion 이 열리지 않아 USB 를 다시 꽂아야 했다. 미러링이 켜져 있으면 소프트웨어로 뒤집는다.
깊이는 mm → m. 약 0.5 m 안쪽과 약 4 m 바깥은 0 (값 없음). 내부 파라미터는 스트림이 알려 주는 화각 (약 58° × 45°).
"""

import math
import os
import threading
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np

# libOpenNI2.so (+ OpenNI2/Drivers/libPS1080.so) 를 찾는 곳. `openni` 바인딩의 기본 경로에는 Jetson 의 시스템 자리
# (/usr/lib/aarch64-linux-gnu) 가 없다. ~/Study/flysim/data/openni2 는 초파리 주행에서 Xtion 을 읽은 배포본이다.
REDIST = [os.environ.get("OPENNI2_REDIST"), str(Path.home() / "Study/flysim/data/openni2"), "/usr/lib/aarch64-linux-gnu"]


def _openni2():
    try:
        from openni import openni2
    except ImportError:
        from primesense import openni2           # 같은 API (openni 는 primesense 바인딩을 이은 것)
    return openni2


@dataclass
class Frame:
    bgr: np.ndarray        # (h, w, 3) uint8
    depth: np.ndarray      # (h, w) float32 m, 0 = 값 없음
    K: tuple               # (fx, fy, cx, cy) — 깊이와 컬러가 같은 시점
    t: float


def intrinsics(w, h, hfov, vfov):
    return ((w / 2) / math.tan(hfov / 2), (h / 2) / math.tan(vfov / 2), w / 2, h / 2)


class XtionCamera:
    def __init__(self, width=640, height=480, fps=30, redist=None, open_timeout=8.0):
        openni2 = self.ni = _openni2()
        dirs = [d for d in ([redist] if redist else REDIST) if d and (Path(d) / "libOpenNI2.so").exists()]
        if not dirs:
            raise SystemExit("libOpenNI2.so 를 못 찾았다 — sudo apt install libopenni2-0, 또는 OPENNI2_REDIST=폴더")
        openni2.initialize(dirs[0])
        print(f"  OpenNI2: {dirs[0]}")
        self.dev = _with_timeout(openni2.Device.open_any, open_timeout,
                                 "Xtion 을 열지 못했다 — 다른 프로그램이 쓰고 있거나 걸린 상태. USB 를 뽑았다 다시 꽂고 재시도")
        self.depth = self.dev.create_depth_stream()
        self.color = self.dev.create_color_stream()
        self.depth.configure_mode(width, height, fps, openni2.PIXEL_FORMAT_DEPTH_1_MM)
        self.color.configure_mode(width, height, fps, openni2.PIXEL_FORMAT_RGB888)
        try:
            self.dev.set_image_registration_mode(openni2.IMAGE_REGISTRATION_DEPTH_TO_COLOR)
        except Exception as e:
            print(f"  (깊이 → 컬러 정합 안 됨: {e} — 화살표 위치가 몇 cm 어긋날 수 있다)")
        self.depth.start()
        self.color.start()
        self.mirror = (self.depth.get_mirroring_enabled(), self.color.get_mirroring_enabled())
        self.hfov = self.depth.get_horizontal_fov()
        self.vfov = self.depth.get_vertical_fov()
        self._latest, self._n = None, 0
        self._lock = threading.Lock()
        self._alive = True
        threading.Thread(target=self._loop, daemon=True).start()
        if self.read(timeout=5.0) is None:
            self.close()
            raise SystemExit("Xtion 프레임이 5 s 안에 안 온다 — USB 를 다시 꽂고 재시도")
        print(f"  Xtion: 화각 {math.degrees(self.hfov):.1f}° × {math.degrees(self.vfov):.1f}°, 미러링 (깊이, 컬러) = {self.mirror}")

    def _loop(self):
        ni = self.ni
        while self._alive:
            try:
                if not ni.wait_for_any_stream([self.depth], timeout=0.5):
                    continue
                df = self.depth.read_frame()
                d = np.frombuffer(df.get_buffer_as_uint16(), dtype=np.uint16).reshape(df.height, df.width).astype(np.float32) / 1000.0
                if not ni.wait_for_any_stream([self.color], timeout=0.5):
                    continue
                cf = self.color.read_frame()
                rgb = np.frombuffer(cf.get_buffer_as_uint8(), dtype=np.uint8).reshape(cf.height, cf.width, 3)
                bgr = rgb[..., ::-1]
                if self.mirror[0]:
                    d = d[:, ::-1]
                if self.mirror[1]:
                    bgr = bgr[:, ::-1]
                bgr = np.ascontiguousarray(bgr)
                d = np.ascontiguousarray(d)
                if bgr.shape[:2] != d.shape:
                    import cv2
                    bgr = cv2.resize(bgr, (d.shape[1], d.shape[0]))
                h, w = d.shape
                with self._lock:
                    self._latest = Frame(bgr, d, intrinsics(w, h, self.hfov, self.vfov), time.monotonic())
                    self._n += 1
            except Exception:
                time.sleep(0.01)

    def read(self, timeout=2.0):
        """마지막으로 부른 뒤 새로 들어온 프레임 (없으면 timeout 까지 기다림, 그래도 없으면 None)."""
        n0 = self._n if self._latest is not None else -1
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            with self._lock:
                if self._latest is not None and self._n != n0:
                    return self._latest
            time.sleep(0.005)
        return None

    def close(self):
        self._alive = False
        time.sleep(0.6)
        for s in (self.depth, self.color):
            try:
                s.stop()
            except Exception:
                pass
        try:
            self.ni.unload()
        except Exception:
            pass


def _with_timeout(fn, seconds, msg):
    out = {}

    def run():
        try:
            out["v"] = fn()
        except Exception as e:
            out["e"] = e
    th = threading.Thread(target=run, daemon=True)
    th.start()
    th.join(seconds)
    if th.is_alive():
        # 멈춘 libusb 호출을 남긴 채 파이썬이 정리하면 libusb assert 로 코어 덤프가 난다 — 정리 없이 바로 끝낸다
        print(msg, flush=True)
        os._exit(3)
    if "e" in out:
        raise out["e"]
    return out["v"]
