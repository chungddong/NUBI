"""AprilTag (36h11) 마커 — 인쇄할 종이 만들기 · 사진에서 찾기.

    python markers.py sheet [--ids 0-11] [--size-mm 160]          → print/apriltag36h11_<크기>mm.pdf (A4 한 장에 하나)
    python markers.py detect <사진 ...> [--size-mm 160] [--hfov 70] 사진마다 찾은 ID · 카메라에서 거리 · 방향

OpenCV aruco (PC 5.0 · Jetson 4.5.4 둘 다 — 옛 함수 / 새 클래스를 가려 쓴다). 크기는 검은 사각 바깥 한 변.
"""

import argparse
import math
import sys
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
A = cv2.aruco
DICT = A.getPredefinedDictionary(A.DICT_APRILTAG_36h11)


def marker_image(i, px):
    if hasattr(A, "generateImageMarker"):
        return A.generateImageMarker(DICT, i, px, borderBits=1)
    return A.drawMarker(DICT, i, px, borderBits=1)


def detect(gray):
    if hasattr(A, "ArucoDetector"):
        p = A.DetectorParameters()
        p.cornerRefinementMethod = A.CORNER_REFINE_SUBPIX
        corners, ids, _ = A.ArucoDetector(DICT, p).detectMarkers(gray)
    else:
        p = A.DetectorParameters_create()
        p.cornerRefinementMethod = A.CORNER_REFINE_SUBPIX
        corners, ids, _ = A.detectMarkers(gray, DICT, parameters=p)
    return [] if ids is None else [(int(i), c.reshape(4, 2)) for i, c in zip(ids.ravel(), corners)]


def tag_pose(corners, size_m, K, dist=None):
    """마커 네 귀퉁이 (왼위 · 오른위 · 오른아래 · 왼아래) → 카메라에서 본 마커 자세 (rvec, tvec)."""
    h = size_m / 2
    obj = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], np.float64)
    ok, rvec, tvec = cv2.solvePnP(obj, corners.astype(np.float64), K, dist if dist is not None else np.zeros(4),
                                  flags=cv2.SOLVEPNP_IPPE_SQUARE)
    return (rvec, tvec) if ok else (None, None)


def parse_ids(s):
    out = []
    for part in s.split(","):
        a, _, b = part.partition("-")
        out += list(range(int(a), int(b or a) + 1))
    return out


def cmd_sheet(a):
    from PIL import Image, ImageDraw, ImageFont
    dpi = 300
    mm = lambda v: round(v / 25.4 * dpi)
    W, H = mm(210), mm(297)
    font = small = None
    for f in (r"C:\Windows\Fonts\malgun.ttf", "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"):
        try:
            font, small = ImageFont.truetype(f, mm(9)), ImageFont.truetype(f, mm(4.2))
            break
        except OSError:
            pass
    pages = []
    for i in parse_ids(a.ids):
        page = Image.new("L", (W, H), 255)
        side = mm(a.size_mm)
        tag = Image.fromarray(marker_image(i, side))
        x0, y0 = (W - side) // 2, mm(25)                   # 둘레 흰 여백 ≥ 마커 한 칸 (160 mm 면 20 mm)
        page.paste(tag, (x0, y0))
        g = ImageDraw.Draw(page)
        y = y0 + side + mm(12)
        g.text((W // 2, y), f"ID {i}", fill=0, font=font, anchor="mm")
        y += mm(14)
        for line in (f"AprilTag 36h11 · 검은 사각 바깥 한 변 {a.size_mm:g} mm — 반드시 실제 크기 (100 %) 로 인쇄",
                     "아래 막대가 정확히 50 mm 인지 자로 확인 (아니면 인쇄 배율을 고친다)",
                     "붙일 때: 이 글자가 아래 · 마커 가운데 바닥에서 50 cm · 구겨지거나 반사되지 않게 (무광)",
                     "붙인 자리를 적는다: 어느 벽 · 가까운 문 기준 몇 cm · 높이"):
            g.text((W // 2, y), line, fill=0, font=small, anchor="mm")
            y += mm(7)
        y += mm(4)
        g.rectangle([W // 2 - mm(25), y, W // 2 + mm(25), y + mm(3)], fill=0)
        g.text((W // 2, y + mm(8)), "50 mm", fill=0, font=small, anchor="mm")
        pages.append(page)
    out = HERE / "print" / f"apriltag36h11_{a.size_mm:g}mm.pdf"
    out.parent.mkdir(exist_ok=True)
    pages = [p.point(lambda v: 255 if v > 127 else 0).convert("1") for p in pages]   # 흑백 1 비트 (JPEG 없이 PDF)
    pages[0].save(out, save_all=True, append_images=pages[1:], resolution=dpi)
    pages[0].convert("L").resize((W // 4, H // 4)).save(out.with_suffix(".png"))         # 첫 장 미리보기
    print(f"→ {out} ({len(pages)} 장)")


def cmd_detect(a):
    for p in a.images:
        img = cv2.imread(p)
        if img is None:
            print(p, "못 읽음")
            continue
        h, w = img.shape[:2]
        f = (w / 2) / math.tan(math.radians(a.hfov) / 2)
        K = np.array([[f, 0, w / 2], [0, f, h / 2], [0, 0, 1.0]])
        found = detect(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY))
        parts = []
        for i, c in found:
            rvec, tvec = tag_pose(c, a.size_mm / 1000, K)
            side = np.mean(np.linalg.norm(np.roll(c, -1, 0) - c, axis=1))
            if tvec is not None:
                R = cv2.Rodrigues(rvec)[0]
                n = R[:, 2]                                   # 마커 앞면 방향 (카메라 좌표)
                ang = math.degrees(math.acos(min(1.0, abs(n[2]))))
                parts.append(f"ID {i}: {np.linalg.norm(tvec):.2f} m, 옆 {tvec[0, 0]:+.2f} m, 비스듬 {ang:.0f}°, 한 변 {side:.0f} px")
        print(Path(p).name, "·", "; ".join(parts) if parts else "없음")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("sheet")
    s.add_argument("--ids", default="0-11")
    s.add_argument("--size-mm", type=float, default=160)
    d = sub.add_parser("detect")
    d.add_argument("images", nargs="+")
    d.add_argument("--size-mm", type=float, default=160)
    d.add_argument("--hfov", type=float, default=70.0, help="가로 화각 (웹캠 2026-10-14 추정 70°)")
    a = ap.parse_args()
    {"sheet": cmd_sheet, "detect": cmd_detect}[a.cmd](a)


if __name__ == "__main__":
    sys.exit(main())
