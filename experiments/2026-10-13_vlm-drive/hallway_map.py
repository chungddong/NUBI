"""복도 지도 — hallway-3d 의 scene.json 에서 벽 · 문을 읽어 위에서 본 그림과 글을 만든다.

시뮬레이터 hardware/vehicle/models/vehicle/sim/web/agent.js 의 drawMap · mapText 와 같은 내용 (고정 가구 회색은 빠짐 —
가구 상자는 원본 형상 코드가 정하므로 여기서는 벽 선과 문만 그린다).
좌표: 원본 (x, 위 y, z) → 우리 (x, −z). x = 복도 방향 (입구 방화문 근처 0), y = 왼쪽 +, m.

scene.json 은 저장소에 없다 (학교 건물 자료). 찾는 순서: --scene 인자 → 환경변수 HALLWAY_SCENE →
시뮬레이터가 복사해 둔 hardware/vehicle/models/vehicle/sim/web/hallway/data/scene.json → F:/Develop/hallway-3d/public/data/scene.json
"""

import base64
import io
import json
import math
import os
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
BOUNDS = (-3.5, -13.0, 48.0, 6.0)                 # hallway_env.js 의 bounds
GOAL_R = 1.0
FONTS = [r"C:\Windows\Fonts\malgun.ttf", "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
         "/usr/share/fonts/truetype/nanum/NanumGothic.ttf", "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"]


def find_scene(arg=None):
    for p in (arg, os.environ.get("HALLWAY_SCENE"),
              REPO / "hardware/vehicle/models/vehicle/sim/web/hallway/data/scene.json",
              Path("F:/Develop/hallway-3d/public/data/scene.json")):
        if p and Path(p).exists():
            return Path(p)
    raise SystemExit("복도 scene.json 을 못 찾았다 — --scene 으로 주거나 HALLWAY_SCENE 환경변수")


class HallwayMap:
    def __init__(self, scene_path):
        data = json.loads(Path(scene_path).read_text(encoding="utf-8"))
        self.segs = [(w["a"][0], -w["a"][1], w["b"][0], -w["b"][1]) for w in data["walls"] if not w.get("passable")]
        self.segs += [(a, -b, c, -d) for a, b, c, d in data.get("extraColliders", [])]
        self.doors = []
        for o in data["objects"]:
            if o["type"] != "door" or not (o.get("label") or o.get("zone") == "lounge" or o["pos"][0] > 30):
                continue
            r = o.get("rotY", 0.0)
            nx, ny = math.sin(r), -math.cos(r)
            x, y = o["pos"][0], -o["pos"][2]
            self.doors.append({"id": o["id"], "label": o.get("label"), "name": o["name"], "x": x, "y": y, "nx": nx, "ny": ny,
                               "front": (x + 0.8 * nx, y + 0.8 * ny),
                               "side": ("왼쪽" if ny < 0 else "오른쪽") if abs(ny) > 0.7 else "끝"})
        self.doors.sort(key=lambda d: d["x"])
        self.font = self.font_big = None
        try:
            from PIL import ImageFont
            for f in FONTS:
                if Path(f).exists():
                    self.font, self.font_big = ImageFont.truetype(f, 14), ImageFont.truetype(f, 17)
                    break
        except ImportError:
            pass

    @staticmethod
    def tag(d, ascii_only=False):
        if d["label"]:
            suf = "앞" if "앞문" in d["name"] else "뒤" if "뒷문" in d["name"] else ""
            return d["label"] + ({"앞": "F", "뒤": "B"}.get(suf, "") if ascii_only else suf)
        return "L" if ascii_only else d["name"].split(" 출입문")[0]

    def find(self, key):
        """'310앞' · '310호 (앞문)' · 문 id 로 찾는다."""
        for d in self.doors:
            if key in (d["id"], d["name"], self.tag(d), self.tag(d, True)):
                return d
        raise SystemExit(f"목표 문 '{key}' 를 못 찾았다. 있는 것: " + ", ".join(self.tag(d) for d in self.doors))

    def draw(self, goal, pose=None, width=1280):
        from PIL import Image, ImageDraw
        x0, y0, x1, y1 = BOUNDS
        s = width / (x1 - x0)
        H = round((y1 - y0) * s)
        P = lambda x, y: ((x - x0) * s, (y1 - y) * s)
        im = Image.new("RGB", (width, H), "white")
        g = ImageDraw.Draw(im)
        for x in range(math.ceil(x0 / 5) * 5, int(x1) + 1, 5):
            g.line([P(x, y0), P(x, y1)], fill=(227, 231, 235))
            g.text((P(x, 0)[0] + 2, H - 16), f"x={x}", fill=(125, 134, 143), font=self.font)
        for y in range(math.ceil(y0 / 5) * 5, int(y1) + 1, 5):
            g.line([P(x0, y), P(x1, y)], fill=(227, 231, 235))
            g.text((3, P(0, y)[1] - 16), f"y={y}", fill=(125, 134, 143), font=self.font)
        for ax, ay, bx, by in self.segs:
            g.line([P(ax, ay), P(bx, by)], fill=(31, 35, 40), width=3)
        ascii_only = self.font is None
        for d in self.doors:
            is_goal = d["id"] == goal["id"]
            u, v = P(d["x"], d["y"])
            col = (224, 49, 49) if is_goal else (47, 125, 225)
            g.rectangle([u - 6, v - 6, u + 6, v + 6], fill=col)
            lu, lv = P(d["x"] - 0.6 * d["nx"], d["y"] - 0.6 * d["ny"])     # 벽 바깥쪽
            g.text((lu, lv), self.tag(d, ascii_only), fill=col, anchor="mm" if self.font else None,
                   font=self.font_big if is_goal else self.font)
        gu, gv = P(*goal["front"])
        rr = GOAL_R * s
        for k in range(0, 360, 20):                                       # 점선 원
            g.arc([gu - rr, gv - rr, gu + rr, gv + rr], k, k + 11, fill=(224, 49, 49), width=2)
        g.ellipse([gu - 4, gv - 4, gu + 4, gv + 4], fill=(224, 49, 49))
        if pose:
            u, v = P(pose["x"], pose["y"])
            sr = max(pose.get("sigma", 0) * s, 4)
            g.ellipse([u - sr, v - sr, u + sr, v + sr], fill=(214, 228, 248))
            a, r = -pose["yaw"], 0.45 * s
            pts = [(u + r * math.cos(a), v + r * math.sin(a)), (u + 0.6 * r * math.cos(a + 2.5), v + 0.6 * r * math.sin(a + 2.5)),
                   (u + 0.6 * r * math.cos(a - 2.5), v + 0.6 * r * math.sin(a - 2.5))]
            g.polygon(pts, fill=(28, 91, 176))
        buf = io.BytesIO()
        im.save(buf, format="PNG")
        return "data:image/png;base64," + base64.b64encode(buf.getvalue()).decode("ascii"), im

    def text(self, goal, pose=None):
        L = ["지도 좌표: x = 복도 방향 (입구 방화문 근처 0, 안쪽으로 +), y = 왼쪽 +, 단위 m. 방향 0° = +x (복도 안쪽), 90° = +y (왼쪽).",
             "두 번째 사진이 위에서 본 지도다 (검은 선 = 벽, 파랑 네모 = 문과 호수, 빨강 = 목표 문 · 문 앞 1 m 원"
             + (", 파랑 삼각형 = 추정 위치와 방향, 연한 원 = 추정 오차" if pose else "") + ").",
             f"목표: {goal['name']} — 문 ({goal['x']:.2f}, {goal['y']:.2f}), 문 앞 점 ({goal['front'][0]:.2f}, {goal['front'][1]:.2f}). "
             "이 점에서 1 m 안에 멈추면 성공.",
             "복도 문 (x 순서, 앞 · 뒤 = 앞문 · 뒷문): "
             + ", ".join(f"{self.tag(d)} {d['side']} x={d['x']:.1f}" for d in self.doors if d["x"] < 30)]
        if pose:
            L.append(f"추정 위치: ({pose['x']:.2f}, {pose['y']:.2f}), 방향 {math.degrees(pose['yaw']):.0f}°, "
                     f"오차 약 ±{pose['sigma']:.2f} m (출처: {pose['src']}). 추정이라 틀릴 수 있다 — 사진과 맞춰 보라.")
        else:
            L.append("추정 위치는 없다 — 사진을 지도와 맞춰 지금 어디인지 추론하라.")
        return "\n".join(L)

    def directory(self):
        side = lambda s: " → ".join(d["name"] for d in self.doors if d["side"] == s and d["x"] < 30)
        lounge = ", ".join(d["name"] for d in self.doors if d["x"] >= 30)
        return (f"복도 입구 (방화문) 에서 안쪽으로 들어가며 볼 때, 앞에서부터 차례로\n- 왼쪽 벽: {side('왼쪽')}\n"
                f"- 오른쪽 벽: {side('오른쪽')}\n복도 끝 방화문을 지나면 라운지 ({lounge}). "
                "문 위쪽에 검은 번호판, 몇 방은 벽에서 튀어나온 방 이름 표지판이 있다.")
