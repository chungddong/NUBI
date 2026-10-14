"""Nuwa-HP60C 깊이 카메라 (Angstrong, USB 3482:6723 'ASJ ZNX_NVT') — 제조사 SDK 없이 표준 UVC (uvcvideo) 로 읽는다.

형식은 jcfurey/hp60c_driver 의 docs/FRAME_FORMAT.md (Apache-2.0, 2026-09-26, 같은 Jetson Orin Nano 에서 관찰) 를 읽고
이 저장소에서 새로 짰다 (그쪽 코드는 가져오지 않음). 640x642 MJPG 모드의 한 버퍼는 세 부분이다:
    0      2560 B   머리 — 0..1023 보정 (f32: 2-5 깊이 fx fy cx cy, 11-14 컬러 fx fy cx cy, 20-28 R, 29-31 t mm), 뒤는 프레임 메타
    2560   ~17 KB   컬러 JPEG 640x480 (끝은 엔트로피 구간 뒤 첫 마커 FF D9 — 버퍼의 마지막 FF D9 를 쓰면 안 된다)
    그 뒤  614400 B 깊이: 480 x 640 uint16 LE, 90° 돌아 있음 (np.rot90(d, 3) 이 컬러와 같은 방향), 단위 1/16 mm, 0 = 없음
           + 512 B  꼬리 (해독 안 됨)
깊이 → 컬러: P_colour = R · P_depth + t (mm). 깊이를 컬러 시점으로 옮겨 (가장 가까운 픽셀, 앞에 있는 것 우선) Xtion 과 같은
Frame (컬러, 컬러 시점 깊이 m, 컬러 내부 파라미터) 으로 돌려준다. 실제 프레임은 약 20.7 fps (문서).
"""

import re
import threading
import time

import cv2
import numpy as np

from xtion import Frame

PREFIX = 2560
DEPTH_W, DEPTH_H = 480, 640                   # 버퍼 안 (돌아 있는) 모양
EOI = re.compile(rb"\xff[^\x00\xd0-\xd7]")     # 엔트로피 구간에서 첫 진짜 마커


def parse(buf):
    """버퍼 (bytes) → (bgr 640x480, depth_mm (480, 640) uint16 컬러 방향 그대로가 아닌 깊이 카메라 시점, calib dict)."""
    w = np.frombuffer(buf[:1024], dtype="<f4")
    cal = {"Kd": tuple(float(v) for v in w[2:6]), "Kc": tuple(float(v) for v in w[11:15]),
           "R": w[20:29].astype(np.float64).reshape(3, 3), "t": w[29:32].astype(np.float64)}
    if buf[PREFIX:PREFIX + 2] != b"\xff\xd8":
        raise ValueError("JPEG 시작 (FF D8) 이 2560 에 없다 — 640x642 MJPG 모드가 맞나")
    sos = buf.find(b"\xff\xda", PREFIX)
    seg = int.from_bytes(buf[sos + 2:sos + 4], "big")
    m = EOI.search(buf, sos + 2 + seg)
    if not m or buf[m.start() + 1] != 0xD9:
        raise ValueError("JPEG 끝 (FF D9) 을 못 찾았다")
    end = m.start() + 2
    bgr = cv2.imdecode(np.frombuffer(buf, dtype=np.uint8, count=end - PREFIX, offset=PREFIX), cv2.IMREAD_COLOR)
    raw = np.frombuffer(buf, dtype="<u2", count=DEPTH_W * DEPTH_H, offset=end).reshape(DEPTH_H, DEPTH_W)
    depth = np.rot90(raw, 3)                                            # (480, 640) 가로
    mm = ((depth.astype(np.uint32) + 8) >> 4).astype(np.uint16)        # 1/16 mm → mm (반올림)
    return bgr, mm, cal


def register(mm, cal, shape):
    """깊이 카메라 시점 깊이 (mm) → 컬러 시점 깊이 (m). 앞에 있는 점이 이긴다."""
    fx, fy, cx, cy = cal["Kd"]
    v, u = np.nonzero(mm)
    z = mm[v, u].astype(np.float64)
    P = np.stack([(u - cx) / fx * z, (v - cy) / fy * z, z])          # (3, N) mm
    Q = cal["R"] @ P + cal["t"][:, None]
    gx, gy, gcx, gcy = cal["Kc"]
    ok = Q[2] > 1
    uc = np.round(gx * Q[0, ok] / Q[2, ok] + gcx).astype(int)
    vc = np.round(gy * Q[1, ok] / Q[2, ok] + gcy).astype(int)
    zc = Q[2, ok]
    h, w = shape
    inside = (uc >= 0) & (uc < w) & (vc >= 0) & (vc < h)
    out = np.full(h * w, np.inf)
    np.minimum.at(out, vc[inside] * w + uc[inside], zc[inside])
    out[np.isinf(out)] = 0
    return (out.reshape(h, w) / 1000.0).astype(np.float32)


class HP60CCamera:
    def __init__(self, device=None, timeout=8.0, flip180=True):
        """flip180: 컬러 · 깊이를 함께 180° 돌린다. 기본 True — 2026-10-14 이 장치를 고정핀대로 바로 세워 두었는데 컬러 · 깊이가
        함께 위아래가 뒤집혀 나왔다 (장치 고유 방향). 보정 R 은 거의 단위행렬 (z 회전 0.8°, 기선 12.2 mm) 이라 둘의 방향은 늘 같다."""
        self.flip180 = flip180
        dev = device if device is not None else _find_video()
        self.cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise SystemExit(f"HP60C ({dev}) 를 못 열었다")
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 642)
        self.cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)                      # JPEG 로 풀지 말고 버퍼 그대로
        got = (int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
        if got != (640, 642):
            raise SystemExit(f"HP60C 가 640x642 MJPG 모드를 안 받는다 ({got})")
        self.cal = None
        self.errors = 0
        self._latest, self._n = None, 0
        self._lock = threading.Lock()
        self._alive = True
        threading.Thread(target=self._loop, daemon=True).start()
        if self.read(timeout=timeout) is None:
            self.close()
            raise SystemExit("HP60C 프레임이 안 온다")
        fx = self.cal["Kc"][0]
        print(f"  HP60C ({dev}): 컬러 fx {fx:.1f} → 화각 {np.degrees(2 * np.arctan(320 / fx)):.1f}°, "
              f"깊이 fx {self.cal['Kd'][0]:.1f} → {np.degrees(2 * np.arctan(320 / self.cal['Kd'][0])):.1f}°, "
              f"기선 {np.linalg.norm(self.cal['t']):.1f} mm")

    def _loop(self):
        while self._alive:
            ok, raw = self.cap.read()
            if not ok or raw is None:
                time.sleep(0.01)
                continue
            try:
                bgr, mm, cal = parse(raw.tobytes())
                depth = register(mm, cal, bgr.shape[:2])
            except Exception:
                self.errors += 1
                continue
            K = cal["Kc"]
            if self.flip180:
                h, w = depth.shape
                bgr, depth = cv2.rotate(bgr, cv2.ROTATE_180), np.ascontiguousarray(depth[::-1, ::-1])
                K = (K[0], K[1], w - 1 - K[2], h - 1 - K[3])
            with self._lock:
                self.cal, self.raw_depth_mm = cal, mm
                self._latest = Frame(bgr, depth, K, time.monotonic())
                self._n += 1

    def read(self, timeout=2.0):
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
        time.sleep(0.2)
        self.cap.release()


def _find_video():
    """이름이 ASJ ZNX_NVT 인 영상 장치 중 캡처되는 첫 번째 (보통 /dev/video0, video1 은 메타데이터)."""
    import glob
    for p in sorted(glob.glob("/sys/class/video4linux/video*")):
        try:
            if "ZNX" in open(p + "/name").read() and open(p + "/index").read().strip() == "0":
                return "/dev/" + p.rsplit("/", 1)[1]
        except OSError:
            pass
    return 0
