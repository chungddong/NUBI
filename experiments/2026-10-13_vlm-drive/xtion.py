"""ASUS Xtion Pro Live — OpenNI2 (libOpenNI2.so) + 파이썬 바인딩 `openni` (pip install openni).

깊이를 컬러 시점에 맞추고 (IMAGE_REGISTRATION_DEPTH_TO_COLOR), 미러링을 끈다 (OpenNI 기본이 거울상인 장치가 있다).
깊이는 mm (PIXEL_FORMAT_DEPTH_1_MM) → m. 약 0.5 m 안쪽과 약 4 m 바깥은 0 (값 없음) 이다.
내부 파라미터는 스트림이 알려 주는 화각으로 만든다 (Xtion 은 약 58° × 45°).
"""

import math
import time
from dataclasses import dataclass

import numpy as np


@dataclass
class Frame:
    bgr: np.ndarray        # (h, w, 3) uint8
    depth: np.ndarray      # (h, w) float32 m, 0 = 값 없음
    K: tuple               # (fx, fy, cx, cy) — 깊이와 컬러가 같은 시점
    t: float


def intrinsics(w, h, hfov, vfov):
    return ((w / 2) / math.tan(hfov / 2), (h / 2) / math.tan(vfov / 2), w / 2, h / 2)


class XtionCamera:
    def __init__(self, width=640, height=480, fps=30, redist=None):
        from openni import openni2
        self.ni = openni2
        openni2.initialize(redist) if redist else openni2.initialize()
        self.dev = openni2.Device.open_any()
        self.depth = self.dev.create_depth_stream()
        self.color = self.dev.create_color_stream()
        self.depth.configure_mode(width, height, fps, openni2.PIXEL_FORMAT_DEPTH_1_MM)
        self.color.configure_mode(width, height, fps, openni2.PIXEL_FORMAT_RGB888)
        for s in (self.depth, self.color):
            try:
                s.set_mirroring_enabled(False)
            except Exception as e:
                print(f"  (미러링 설정 안 됨: {e})")
        self.depth.start()
        self.color.start()
        try:
            self.dev.set_image_registration_mode(openni2.IMAGE_REGISTRATION_DEPTH_TO_COLOR)
        except Exception as e:
            print(f"  (깊이 → 컬러 정합 안 됨: {e} — 화살표 위치가 몇 cm 어긋날 수 있다)")
        try:
            self.dev.set_depth_color_sync_enabled(True)
        except Exception:
            pass
        self.hfov = self.depth.get_horizontal_fov()
        self.vfov = self.depth.get_vertical_fov()

    def read(self):
        d = self.depth.read_frame()
        depth = np.frombuffer(d.get_buffer_as_uint16(), dtype=np.uint16).reshape(d.height, d.width).astype(np.float32) / 1000.0
        c = self.color.read_frame()
        rgb = np.frombuffer(c.get_buffer_as_uint8(), dtype=np.uint8).reshape(c.height, c.width, 3)
        bgr = np.ascontiguousarray(rgb[..., ::-1])
        if bgr.shape[:2] != depth.shape:
            import cv2
            bgr = cv2.resize(bgr, (depth.shape[1], depth.shape[0]))
        h, w = depth.shape
        return Frame(bgr, depth, intrinsics(w, h, self.hfov, self.vfov), time.monotonic())

    def close(self):
        for s in (self.depth, self.color):
            try:
                s.stop()
            except Exception:
                pass
        try:
            self.ni.unload()
        except Exception:
            pass
