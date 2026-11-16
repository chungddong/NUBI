"""전시 덱에 넣을 그림을 저장소 원본에서 잘라 images/ 에 만든다.

렌더 왼쪽 위의 버전 라벨, 지도 그림 아래 설명 줄처럼 슬라이드에서 겹치는 글자를 잘라 낸다.
실행: 저장소 루트에서  py docs/presentations/2026-11_eng-fest/prep_images.py
"""
from pathlib import Path

from PIL import Image, ImageOps

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
OUT = HERE / "images"
VEH = ROOT / "hardware/vehicle/models/vehicle"


def save(src: Path, name: str, box=None, width=None):
    im = Image.open(src).convert("RGB")
    if box:
        im = im.crop(box)
    if width and im.width > width:
        im = im.resize((width, round(im.height * width / im.width)), Image.LANCZOS)
    OUT.mkdir(exist_ok=True)
    im.save(OUT / name, quality=90)
    print(f"{name}: {im.size}")


# 차체 렌더 (1600 x 1100) — 위 70 px 라벨을 잘라 낸다
for v in ["v01_base", "v05_floor", "v10_shell", "v22_headlight", "v26_module_rails", "v28_wiring"]:
    save(VEH / v / "images/overall_iso.png", f"{v}_iso.jpg", box=(0, 70, 1600, 1100),
         width=1600 if v in ("v26_module_rails", "v28_wiring") else 800)
save(VEH / "v28_wiring/images/inside_iso.png", "v28_inside_iso.jpg", box=(0, 70, 1600, 1100))

# 휠 마운트 FEA (H 크로스 조립)
save(ROOT / "hardware/vehicle/models/hoverboard_mount/output/fea/h_assembly_hcross/stress_deformed_800N.png",
     "fea_hcross.jpg", box=(100, 120, 1600, 860))

# 깊이 카메라 2종 실측 화면
save(ROOT / "experiments/2026-10-13_vlm-drive/images/camera_compare.jpg", "camera_compare.jpg")

# 실제 주행 경로 지도 — 아래 설명 줄 잘라 냄
save(ROOT / "data/processed/2026-10-15_hallway_robot-map/robot_map_paths.png", "robot_map_paths.jpg",
     box=(0, 0, 1166, 1105))

# 팀 사진 — 사용자가 준 폴더에서 (git 밖 원본). 칸 비율에 맞춰 가운데를 자른다.
PHOTOS = Path.home() / "Downloads/자율주행 플랫폼 관련"


def photo(name: str, out: str, aspect: float, ybias=0.5, xbias=0.5, width=1400, box=None):
    """aspect = 가로/세로. bias 0 = 위(왼쪽)부터, 1 = 아래(오른쪽)까지. box = 먼저 잘라 낼 영역 (원본 비율 0~1)."""
    src = PHOTOS / name
    if not src.exists():
        print(f"건너뜀 (없음): {src}")
        return
    im = ImageOps.exif_transpose(Image.open(src)).convert("RGB")
    if box:
        W0, H0 = im.size
        im = im.crop((round(box[0] * W0), round(box[1] * H0), round(box[2] * W0), round(box[3] * H0)))
    w, h = im.size
    if w / h > aspect:      # 너무 넓다 → 좌우를 자른다
        cw = round(h * aspect)
        x0 = round((w - cw) * xbias)
        im = im.crop((x0, 0, x0 + cw, h))
    else:                   # 너무 높다 → 위아래를 자른다
        ch = round(w / aspect)
        y0 = round((h - ch) * ybias)
        im = im.crop((0, y0, w, y0 + ch))
    if im.width > width:
        im = im.resize((width, round(im.height * width / im.width)), Image.LANCZOS)
    OUT.mkdir(exist_ok=True)
    im.save(OUT / out, quality=88)
    print(f"{out}: {im.size}")


photo("호버보드 분해작업.jpg", "photo_teardown.jpg", 2.85 / 4.6)
photo("펌웨어 재플래쉬.jpg", "photo_reflash.jpg", 2.85 / 4.6, xbias=0.3)
photo("ASUS xtion pro live 카메라 테스트_1.jpg", "photo_obstacle_guard.jpg", 4 / 3, box=(0.17, 0.33, 0.84, 0.86))
photo("차량제작중_프레임제작.jpg", "photo_frame.jpg", 3.85 / 2.6, ybias=0.5)
photo("ASUS xtion pro live 카메라 테스트_2_근접정지 테스트.jpg", "photo_vehicle_xtion.jpg", 3.85 / 2.6, ybias=0.75)

# 제작 일정 장의 사진 카드 (가로 1.88 x 세로 1.45 in)
TL = 1.88 / 1.45
photo("호버보드 분해작업.jpg", "tl_teardown.jpg", TL, ybias=0.3, width=900)
photo("차량제작중_프레임제작.jpg", "tl_frame.jpg", TL, ybias=0.5, width=900)
photo("차량제작중_최소구동테스트.jpg", "tl_wheels.jpg", TL, ybias=0.45, width=900)
photo("차량제작중_실내테스트.jpg", "tl_indoor.jpg", TL, ybias=0.55, width=900)
photo("ASUS xtion pro live 카메라 테스트_1.jpg", "tl_guard.jpg", TL, box=(0.17, 0.33, 0.84, 0.86), width=900)

# source/ — 저장소 밖에서 가져온 자료
SRC = HERE / "source"
# 교내 복도 웹 시뮬레이터 (?env=hallway, 뒤따라가기 시점, v28 차체로 자율주행 중) — 2026-11-05 캡처 1260 x 900
save(SRC / "hallway_sim_follow.jpg", "sim_hallway.jpg", box=(0, 40, 1260, 853))
# 표지 실사풍 렌더 (render_cover.py, Blender Cycles)
_c = Image.open(SRC / "cover_render.png").convert("RGBA")   # 투명 배경 → 슬라이드 배경색(#0D0F14) 위에
_bg = Image.new("RGBA", _c.size, (13, 15, 20, 255))
_bg.alpha_composite(_c)
_bg.convert("RGB").save(OUT / "cover_render.jpg", quality=92)
# RC카 SLAM 지도 (2026-09-11 마지막 세션) — Jetson ~/Downloads/slam_map/session_20260911_100506/really_final.png
save(SRC / "rc_slam_really_final.png", "rc_slam_map.jpg")
# 10-14 13:54 실차 주행을 복도 시뮬레이터로 다시 돌린 영상 (data/logs/hallway-run/2026-10-14_135415/replay_sim.mp4)
# 의 두 장면. 위 왼쪽 = 실제 웹캠, 위 오른쪽 = 찾은 위치에서 그린 3D 지도, 아래 = 지도 위 경로
for t in ("t200", "t275"):
    fr = SRC / f"replay_sim_{t}.png"
    save(fr, f"real_{t}.jpg", box=(0, 0, 640, 360))
    save(fr, f"sim_{t}.jpg", box=(640, 0, 1280, 360))
save(SRC / "replay_sim_t275.png", "replay_map.jpg", box=(0, 455, 810, 660))
im = Image.open(OUT / "rc_slam_map.jpg")  # 일정 카드용
h = round(im.width / TL); y0 = (im.height - h) // 2
im.crop((0, y0, im.width, y0 + h)).save(OUT / "tl_slam.jpg", quality=90)

# SfM 재위치 오차 누적 분포
save(ROOT / "experiments/2026-10-13_sfm-relocalization/error_cdf_jetson_cv.png", "error_cdf.jpg")
