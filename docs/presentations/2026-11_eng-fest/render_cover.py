"""표지용 실사풍 렌더 — 웹 시뮬레이터 메시(vehicle.json/bin, v28)를 Blender Cycles로 그린다.

상부 센서 모듈(라이다 · 기둥 판 · GNSS · 상태등)과 참고용 케이블은 빼서 v26처럼 모듈 없는 모습으로.
실행 (저장소 루트에서):
  "C:\Program Files\Blender Foundation\Blender 5.2\blender.exe" -b --factory-startup \
      -P docs/presentations/2026-11_eng-fest/render_cover.py -- [출력.png] [샘플 수]
먼저 sim/web_export.py 로 hardware/vehicle/models/vehicle/sim/web/data 를 만들어 두어야 한다.
"""
import json
import math
import sys
from pathlib import Path

import bmesh
import bpy
import numpy as np
from mathutils import Vector

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
DATA = ROOT / "hardware/vehicle/models/vehicle/sim/web/data"
argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
OUT = Path(argv[0]) if argv else HERE / "source" / "cover_render.png"
SAMPLES = int(argv[1]) if len(argv) > 1 else 256

SKIP = {"lidar", "module_plate", "gnss", "status_light", "cable"}

meta = json.loads((DATA / "vehicle.json").read_text(encoding="utf-8"))
soup = np.fromfile(DATA / "vehicle.bin", dtype=np.float32).reshape(-1, 3)

bpy.ops.wm.read_factory_settings(use_empty=True)
scene = bpy.context.scene


def hex_rgb(h):
    h = h.lstrip("#")
    c = [int(h[i:i + 2], 16) / 255 for i in (0, 2, 4)]
    return [x / 12.92 if x <= 0.04045 else ((x + 0.055) / 1.055) ** 2.4 for x in c] + [1.0]


# ---------- 재질 ----------
def pbr(name, color, metallic=0.0, rough=0.5, bump=None, coat=0.0, aniso=0.0):
    """bump = (종류, 크기, 세기). 종류: brushed(한 방향 헤어라인) · grain(잔 요철) · rubber"""
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    nt = m.node_tree
    b = nt.nodes["Principled BSDF"]
    b.inputs["Base Color"].default_value = color
    b.inputs["Metallic"].default_value = metallic
    b.inputs["Roughness"].default_value = rough
    if coat:
        b.inputs["Coat Weight"].default_value = coat
        b.inputs["Coat Roughness"].default_value = 0.08
    if aniso:
        b.inputs["Anisotropic"].default_value = aniso
    if bump:
        kind, scale, strength = bump
        tc = nt.nodes.new("ShaderNodeTexCoord")
        mp = nt.nodes.new("ShaderNodeMapping")
        nz = nt.nodes.new("ShaderNodeTexNoise")
        bp = nt.nodes.new("ShaderNodeBump")
        nt.links.new(tc.outputs["Object"], mp.inputs["Vector"])
        if kind == "brushed":          # 한 방향으로 길게 늘인 잡음 = 헤어라인
            mp.inputs["Scale"].default_value = (scale * 0.02, scale, scale)
            nz.inputs["Detail"].default_value = 12
        elif kind == "rubber":
            mp.inputs["Scale"].default_value = (scale, scale, scale)
            nz.inputs["Detail"].default_value = 6
        else:
            mp.inputs["Scale"].default_value = (scale, scale, scale)
            nz.inputs["Detail"].default_value = 8
        nz.inputs["Scale"].default_value = 1.0
        nt.links.new(mp.outputs["Vector"], nz.inputs["Vector"])
        nt.links.new(nz.outputs["Fac"], bp.inputs["Height"])
        bp.inputs["Strength"].default_value = strength
        bp.inputs["Distance"].default_value = 0.0005
        nt.links.new(bp.outputs["Normal"], b.inputs["Normal"])
        # 거칠기에도 잡음을 조금 섞어 반사가 고르지 않게
        mr = nt.nodes.new("ShaderNodeMapRange")
        mr.inputs["To Min"].default_value = rough * 0.8
        mr.inputs["To Max"].default_value = min(1.0, rough * 1.25)
        nt.links.new(nz.outputs["Fac"], mr.inputs["Value"])
        nt.links.new(mr.outputs["Result"], b.inputs["Roughness"])
    return m


ALU = pbr("brushed_aluminium", [0.80, 0.81, 0.83, 1], 1.0, 0.28, ("brushed", 900, 0.35), aniso=0.6)
ALU_DARK = pbr("anodized_profile", [0.42, 0.44, 0.47, 1], 1.0, 0.38, ("brushed", 700, 0.3), aniso=0.4)
ACM = pbr("acm_panel", [0.78, 0.80, 0.82, 1], 0.75, 0.3, ("brushed", 1500, 0.7), coat=0.3, aniso=0.7)
BLACK_PLASTIC = pbr("textured_black_plastic", [0.035, 0.038, 0.042, 1], 0.0, 0.55, ("grain", 2500, 0.35))
DARK_PC = pbr("dark_polycarbonate", [0.06, 0.065, 0.07, 1], 0.0, 0.25, ("grain", 3000, 0.08), coat=0.6)
RUBBER = pbr("tire_rubber", [0.012, 0.012, 0.013, 1], 0.0, 0.7, ("rubber", 4000, 0.6))
HUB = pbr("hub_aluminium", [0.75, 0.76, 0.78, 1], 1.0, 0.22, ("grain", 1500, 0.1))
STEEL = pbr("black_oxide_steel", [0.08, 0.08, 0.09, 1], 1.0, 0.35)
GLASS = pbr("lens_glass", [0.02, 0.02, 0.025, 1], 0.0, 0.03, coat=1.0)
mats_by_key = {
    "frame": ALU_DARK, "module_rail": ALU_DARK, "frame_bracket": ALU_DARK, "clamp_oem": ALU_DARK,
    "shell_acm": ACM, "shell_lid": ACM,
    "shell_asa": BLACK_PLASTIC, "shell_rubber": RUBBER, "antenna": BLACK_PLASTIC, "lid_knob": BLACK_PLASTIC,
    "shell_pc": DARK_PC, "camera": DARK_PC,
    "bolt": STEEL, "floor_bolt": STEEL,
}
color_mats = {}


def mat_for(part):
    if part["key"] in mats_by_key:
        return mats_by_key[part["key"]]
    c = part["color"]
    if c not in color_mats:
        color_mats[c] = pbr(f"paint_{c}", hex_rgb(c), 0.0, 0.45, ("grain", 2500, 0.1))
    return color_mats[c]


# ---------- 메시 ----------
# 센서 모듈로 가던 전선처럼 덮개 위로 솟는 것도 뺀다 (레일 · 안테나 · 손나사 · 비상정지는 남김)
LID_TOP = max(p["max"][2] for p in meta["parts"] if p["key"] == "shell_lid")
KEEP_ABOVE = {"module_rail", "antenna", "lid_knob", "estop", "estop_legend", "shell_lid"}
for p in meta["parts"]:
    if p["max"][2] > LID_TOP + 0.004 and p["key"] not in KEEP_ABOVE:
        SKIP.add(p["key"]) if p["key"].startswith("wire") or p["key"] in ("connector", "harness") else None
        p["_skip"] = True

col = bpy.data.collections.new("NUBI")
scene.collection.children.link(col)
wheel_centers = [Vector(w) if isinstance(w, list) else None for w in meta.get("wheels", [])]
for part in meta["parts"]:
    if part["key"] in SKIP or part.get("_skip"):
        continue
    tri = soup[part["start"] * 3:(part["start"] + part["count"]) * 3]   # start · count 는 삼각형 단위
    if len(tri) < 3:
        continue
    me = bpy.data.meshes.new(part["name"])
    bm = bmesh.new()
    verts = [bm.verts.new(v) for v in tri]
    for i in range(0, len(verts), 3):
        try:
            bm.faces.new(verts[i:i + 3])
        except ValueError:
            pass
    bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=1e-5)
    bm.to_mesh(me)
    bm.free()
    ob = bpy.data.objects.new(part["name"], me)
    col.objects.link(ob)
    if part["key"] == "motor":                       # 바퀴: 바깥 고무 + 안쪽 알루미늄 허브
        me.materials.append(RUBBER)
        me.materials.append(HUB)
        pts = np.array([v.co[:] for v in me.vertices])
        c = (pts.min(0) + pts.max(0)) / 2
        r_out = (pts[:, 0].max() - pts[:, 0].min()) / 2
        for f in me.polygons:
            p = f.center - Vector(c)
            r = math.hypot(p.x, p.z)
            f.material_index = 0 if r > r_out * 0.62 else 1
    else:
        me.materials.append(mat_for(part))
# 각진 부품이 뭉개지지 않게 30° 기준으로 매끈 · 모서리 나눔
for ob in col.objects:
    ob.select_set(True)
bpy.context.view_layer.objects.active = next(iter(col.objects))
bpy.ops.object.shade_smooth_by_angle(angle=math.radians(30))

# ---------- 바닥 · 조명 · 카메라 ----------
bb_min = Vector((min(p["min"][i] for p in meta["parts"] if p["key"] not in SKIP) for i in range(3)))
bb_max = Vector((max(p["max"][i] for p in meta["parts"] if p["key"] not in SKIP) for i in range(3)))
ctr = (bb_min + bb_max) / 2

bpy.ops.mesh.primitive_plane_add(size=30, location=(0, 0, 0))
ground = bpy.context.active_object
ground.is_shadow_catcher = True      # 바닥은 그림자만 — 배경은 투명, 슬라이드 색 위에 얹는다

world = bpy.data.worlds.new("studio")
scene.world = world
world.use_nodes = True
wn = world.node_tree
bg = wn.nodes["Background"]
grad = wn.nodes.new("ShaderNodeTexGradient")
tc = wn.nodes.new("ShaderNodeTexCoord")
sep = wn.nodes.new("ShaderNodeSeparateXYZ")
ramp = wn.nodes.new("ShaderNodeValToRGB")
wn.links.new(tc.outputs["Generated"], sep.inputs["Vector"])
wn.links.new(sep.outputs["Z"], ramp.inputs["Fac"])
ramp.color_ramp.elements[0].position = 0.45
ramp.color_ramp.elements[0].color = (0.004, 0.005, 0.007, 1)
ramp.color_ramp.elements[1].position = 0.75
ramp.color_ramp.elements[1].color = (0.06, 0.07, 0.08, 1)
wn.links.new(ramp.outputs["Color"], bg.inputs["Color"])
bg.inputs["Strength"].default_value = 0.35


def area(name, loc, energy, size, color=(1, 1, 1)):
    d = bpy.data.lights.new(name, "AREA")
    d.energy, d.size, d.color = energy, size, color
    o = bpy.data.objects.new(name, d)
    scene.collection.objects.link(o)
    o.location = loc
    o.rotation_euler = (Vector(ctr) - Vector(loc)).to_track_quat("-Z", "Y").to_euler()
    return o


area("key", (2.2, -2.6, 2.6), 160, 2.0)
area("fill", (-1.0, -3.0, 2.4), 25, 2.0, (0.85, 0.92, 1.0))
area("rim_cyan", (-1.8, 2.4, 1.6), 140, 1.5, (0.3, 0.85, 1.0))
area("top", (0, 0, 3.2), 45, 3.0)
strip = area("strip_side", (0.4, -3.4, 1.6), 260, 0.3)      # 옆판에 길게 비치는 띠 반사
strip.data.shape, strip.data.size, strip.data.size_y = "RECTANGLE", 4.0, 0.25
back = area("strip_back", (-2.6, 1.0, 2.4), 220, 0.3, (0.75, 0.9, 1.0))   # 윗모서리 윤곽
back.data.shape, back.data.size, back.data.size_y = "RECTANGLE", 0.25, 4.0

cam_d = bpy.data.cameras.new("cam")
cam_d.lens = 105
cam = bpy.data.objects.new("cam", cam_d)
scene.collection.objects.link(cam)
cam.location = (2.75, -2.95, 1.75)
cam.rotation_euler = (Vector((ctr.x, ctr.y, ctr.z * 0.8)) - cam.location).to_track_quat("-Z", "Y").to_euler()
scene.camera = cam
cam_d.dof.use_dof = True
cam_d.dof.focus_distance = (Vector(ctr) - cam.location).length
cam_d.dof.aperture_fstop = 8

# ---------- 렌더 ----------
scene.render.engine = "CYCLES"
scene.cycles.samples = SAMPLES
scene.cycles.use_denoising = True
scene.render.resolution_x, scene.render.resolution_y = 1800, 1160
scene.render.film_transparent = True
scene.view_settings.exposure = -0.9
scene.view_settings.view_transform = "AgX"
scene.view_settings.look = "AgX - Medium High Contrast"
prefs = bpy.context.preferences.addons["cycles"].preferences
for dev in ("OPTIX", "CUDA", "HIP", "ONEAPI"):
    try:
        prefs.compute_device_type = dev
        prefs.get_devices()
        if any(d.type == dev for d in prefs.devices):
            for d in prefs.devices:
                d.use = True
            scene.cycles.device = "GPU"
            print("GPU:", dev)
            break
    except TypeError:
        continue
scene.render.filepath = str(OUT)
scene.render.image_settings.file_format = "PNG"
scene.render.image_settings.color_mode = "RGBA"
OUT.parent.mkdir(parents=True, exist_ok=True)
bpy.ops.render.render(write_still=True)
print("saved", OUT)
