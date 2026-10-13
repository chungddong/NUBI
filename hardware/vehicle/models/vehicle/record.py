r"""차량 재설계 버전 기록 도구 — 버전마다 같은 규칙으로 이미지와 기록을 만든다.

    .venv/Scripts/python.exe models/vehicle/record.py v01_base
        → v01_base/images/  overall.png (ISO·위·옆·아래), overall_iso.png, thumb.png,
                            change_NN_<그룹>.png (이전/이번 × ISO/위)
        → v01_base/README.md 의 자동 구역, models/vehicle/README.md 의 버전 표
    .venv/Scripts/python.exe models/vehicle/record.py new v02_posts "기둥 배치"
        → 마지막 버전 폴더를 복사해 새 버전을 연다 (VERSION/TITLE/DATE/PREV 를 고쳐 둔다)

변화는 **자동으로 찾는다.** 두 버전의 부품을 `그룹key/부품label` 로 짝지어
  · 이전에만 있음 = 삭제 · 이번에만 있음 = 추가
  · 둘 다 있는데 크기·부피는 같고 자리만 다름 = 이동 / 크기·부피가 다름 = 형상 변경
그래서 부품 label 은 버전 사이에 같은 것이면 같은 이름을 써야 한다.

시간 (2026-10-04 재어 보고 줄였다)
  · 한 번 기록 = **한 프로세스에서 한 번 짓기** → 검증 · STEP · 뷰어 · 메시 · 이미지 · 기록.
    build123d import 만 20 s 라 프로세스를 늘리지 않는다.
  · 코드 해시가 지난 기록과 같으면 짓지 않는다 (캐시로 이미지·기록만, 수 초).
  · 이전 버전은 메시 캐시를 읽는다. 그 버전 코드가 바뀐 경우에만 따로 프로세스로 다시 뜬다
    (버전마다 parameters.py 같은 같은 이름 모듈이 있어 한 프로세스에 두 버전을 못 올린다).
  · 같은 형상을 자리만 옮긴 부품은 한 번만 쪼갠다 (IsPartner).
  · 렌더는 GPU (VTK 오프스크린, 장당 ~0.4 s). 안 되면 numpy raster (iso_render 와 같은 결과).
  · 단계별 소요 시간을 README "기록 소요" 에 남긴다 — 느려지면 어디인지 바로 보인다.
"""

import argparse
import contextlib
import datetime as _dt
import importlib.util
import io
import json
import re
import shutil
import subprocess
import sys
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).parent


class IR:
    """models/iso_render.py 에서 쓰던 두 함수만 옮겨 왔다 — iso_render 를 import 하면
    build123d 까지 딸려 와 (20 s) 캐시만으로 그리는 기록도 느려진다."""

    @staticmethod
    def _unit(v):
        v = np.asarray(v, dtype=float)
        return v / np.linalg.norm(v)

    @staticmethod
    def _ground(tris, light, n=72, scale=1.15):
        """접지면 원판 — 높이는 모델 밑면과 z=0 중 낮은 쪽."""
        rad = np.abs(tris[:, :, :2]).max() * scale
        z0 = min(0.0, float(tris[:, :, 2].min()))
        t = np.linspace(0, 2 * np.pi, n + 1)
        ring = np.stack([rad * np.cos(t), rad * np.sin(t), np.full_like(t, z0)], axis=1)
        g = np.stack([np.tile((0.0, 0.0, z0), (n, 1)), ring[:-1], ring[1:]], axis=1)
        return g, np.tile(IR._unit(light), (n, 3, 1))

TOL = 0.4
BG = (0.933, 0.941, 0.949)                  # #eef0f2
LIGHT = (0.55, -1.0, 0.75)
GROUND_RGB = (0.87, 0.885, 0.9)
RED = (0.85, 0.25, 0.22)                    # 이전 자리 (삭제·이동 전·변경 전)
BLUE = (0.16, 0.48, 0.86)                   # 이번 자리 (추가·이동 후·변경 후)
GHOST = 0.62                                # 안 바뀐 부품을 배경 쪽으로 섞는 비율
VIEWS = {                                   # (물체 → 카메라 방향, 화면 위쪽)
    "ISO": ((1.0, -1.0, 0.55), (0, 0, 1)),
    "위": ((0, 0, 1), (0, 1, 0)),
    "옆 (-y 에서)": ((0, -1, 0), (0, 0, 1)),
    "아래": ((0, 0, -1), (0, -1, 0)),
}
BEGIN = "<!-- record:begin — record.py 가 덮어쓴다. 이 사이는 손으로 고치지 않는다 -->"
END = "<!-- record:end -->"
FONT = r"C:\Windows\Fonts\malgun.ttf"


def _font(px):
    try:
        return ImageFont.truetype(FONT, px)
    except OSError:
        return ImageFont.load_default()


def _hex(c):
    c = c.lstrip("#")
    return tuple(int(c[i:i + 2], 16) / 255 for i in (0, 2, 4))


# ---------------------------------------------------------------- 버전 불러오기
def _purge():
    """버전 폴더 모듈 (parameters · low_clamp · tslot …) 을 sys.modules · sys.path 에서 뺀다.

    버전마다 같은 이름 모듈이 있어서, 빼지 않으면 두 번째 버전이 첫 버전의 parameters 를
    그대로 쓴다. 빼고 나면 한 프로세스에서 여러 버전을 차례로 지을 수 있다 (프로세스를 새로
    띄우면 build123d import 만 20 s)."""
    import os
    vroots = [os.path.normcase(str(p)) for p in ROOT.glob("v[0-9][0-9]_*")]
    for k, m in list(sys.modules.items()):
        f = os.path.normcase(getattr(m, "__file__", None) or "")
        if k.startswith("asm_") or any(f.startswith(v + os.sep) for v in vroots):
            del sys.modules[k]
    sys.path[:] = [p for p in sys.path if os.path.normcase(str(Path(p))) not in vroots]


def load_assembly(name):
    _purge()
    path = ROOT / name / "assembly.py"
    spec = importlib.util.spec_from_file_location(f"asm_{name}", path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod                 # 모듈이 자기 자신을 찾을 수 있게 (sys.modules[__name__])
    spec.loader.exec_module(mod)
    return mod


def code_hash(name):
    """버전 폴더 *.py 내용의 해시 — 같으면 메시 캐시를 다시 쓰고 새로 짓지 않는다."""
    import hashlib
    h = hashlib.sha1()
    for f in sorted((ROOT / name).glob("*.py")):
        h.update(f.name.encode()); h.update(f.read_bytes())
    h.update(str(TOL).encode())
    return h.hexdigest()[:16]


class Timer:
    """단계별 소요 시간 — 콘솔에 찍고 scene.json · README 에 남긴다."""

    def __init__(self):
        import time
        self._now, self.steps = time.perf_counter, []
        self._t = self._t0 = self._now()

    def lap(self, name):
        t = self._now()
        self.steps.append((name, round(t - self._t, 1)))
        print(f"  ⏱ {name} {t - self._t:.1f}s", flush=True)
        self._t = t

    def total(self):
        return round(self._now() - self._t0, 1)


def _trsf(loc):
    t = loc.wrapped.Transformation()
    m = np.array([[t.Value(i, j) for j in range(1, 5)] for i in range(1, 4)])
    return m[:, :3], m[:, 3]


def tessellate(part, cache):
    """부품 메시 (월드 좌표) — 삼각형 (N,3,3) · 꼭짓점 법선 (N,3,3).

    같은 형상을 자리만 옮긴 부품 (IsPartner: 모터 4 개, 볼트 16 개, 같은 길이 프로파일)
    은 원본 좌표에서 **한 번만** 쪼개고 각자의 Location 행렬만 곱한다.
    법선은 iso_render.collect 와 같다 (면 안에서 면적 가중 평균).
    """
    # 주의: build123d 의 located() 는 **깊은 복사**다 (모터 STEP 이면 수 초, 복사본은 더 이상
    # IsPartner 가 아니다 — 처음에 이걸로 원점 사본을 만들었다가 44 부품을 44 번 쪼개 93 s).
    # `Location * 부품` 사본은 형상을 공유한다. 그래서 원본 부품을 제자리에서 쪼개 두고,
    # 사본은 상대 행렬 (사본 Location × 원본 Location⁻¹) 만 곱한다.
    for base, bt, bvn, (R0, T0) in cache:
        if part.wrapped.IsPartner(base.wrapped):
            R1, T1 = _trsf(part.location)
            R = R1 @ R0.T                            # 회전 행렬이라 역행렬 = 전치
            return bt @ R.T + (T1 - R @ T0), bvn @ R.T
    else:
        base = part
        vs, fs = base.tessellate(TOL, 0.2)
        v = np.array([(p.X, p.Y, p.Z) for p in vs])
        f = np.array(fs)
        t = v[f]
        fn = np.cross(t[:, 1] - t[:, 0], t[:, 2] - t[:, 0])
        vn = np.zeros_like(v)
        for k in range(3):
            np.add.at(vn, f[:, k], fn)
        ln = np.linalg.norm(vn, axis=1, keepdims=True)
        bt, bvn = t, np.where(ln > 1e-12, vn / np.where(ln > 0, ln, 1), 0)[f]
        cache.append((base, bt, bvn, _trsf(part.location)))
        return bt, bvn


def viewer_up(port):
    import socket
    with socket.socket() as s:
        s.settimeout(0.2)
        return s.connect_ex(("127.0.0.1", port)) == 0


def build_scene(name, full=True):
    """버전 하나를 **한 번** 짓는다 → 검증 · (STEP · 뷰어) · 메시 캐시. full=False 면 메시만."""
    tm = Timer()
    A = load_assembly(name)
    tm.lap("불러오기")
    groups = A.build()
    tm.lap("짓기")
    bad, verify_lines = None, []
    if full and getattr(A, "verify", None):
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            bad = A.verify(groups)
        verify_lines = buf.getvalue().rstrip().splitlines()
        print("\n".join(verify_lines))
        print("verify:", bad or "OK")
        tm.lap("검증")
    metrics = list(A.metrics(groups)) if getattr(A, "metrics", None) else []
    comps = A.compounds(groups) if hasattr(A, "compounds") else None
    out = ROOT / name / "output"
    out.mkdir(parents=True, exist_ok=True)
    if full and comps:
        from build123d import Compound, export_step
        asm = Compound(children=comps)
        asm.label = A.VERSION
        export_step(asm, str(out / f"{A.VERSION}.step"))
        tm.lap("STEP")
        if hasattr(A, "export"):                     # 버전별 추가 내보내기 (DXF · 도면 …)
            A.export(groups, out)
            tm.lap("내보내기")
        port = getattr(A, "PORT", 3939)
        if viewer_up(port):
            from ocp_vscode import Camera, set_port, show
            set_port(port)
            show(*comps, names=[c.label for c in comps], colors=[d["color"] for d in groups],
                 alphas=[d.get("alpha", 1.0) for d in groups], progress=None,
                 reset_camera=Camera.RESET)
            tm.lap("뷰어 전송")
        else:
            print(f"  (뷰어 {port} 꺼져 있음 — 전송 생략)")

    tris, vns, owner, leaves, seen, cache = [], [], [], [], set(), []
    for d in groups:
        for part in d["parts"]:
            path = f"{d['key']}/{part.label}"
            assert path not in seen, f"부품 이름이 겹친다: {path}"
            seen.add(path)
            t, vn = tessellate(part, cache)
            pts = t.reshape(-1, 3)
            from build123d import CenterOf
            com = part.center(CenterOf.MASS)
            leaves.append(dict(path=path, key=d["key"], group=d["label"], color=d["color"],
                               label=part.label, volume=float(part.volume),
                               area=float(part.area), com=[com.X, com.Y, com.Z],   # B-rep 정확값 (메시 무관)
                               min=pts.min(axis=0).tolist(), max=pts.max(axis=0).tolist(),
                               ntri=len(t)))
            tris.append(t.astype(np.float32))
            vns.append(vn.astype(np.float16))
            owner.append(np.full(len(t), len(leaves) - 1, dtype=np.int32))
    np.savez_compressed(out / "scene.npz", tris=np.concatenate(tris),
                        vn=np.concatenate(vns), owner=np.concatenate(owner))
    tm.lap(f"메시 ({len(cache)} 형상 → {len(leaves)} 부품)")
    old = out / "scene.json"
    if not full and old.exists():                    # 메시만 다시 뜰 땐 옛 검증 결과를 남긴다
        o = json.loads(old.read_text(encoding="utf-8"))
        verify_lines, bad = o.get("verify", []), o.get("bad")
    meta = dict(version=A.VERSION, title=A.TITLE, date=A.DATE, prev=A.PREV, code=code_hash(name),
                summary=A.SUMMARY, wheel_keys=list(getattr(A, "WHEEL_KEYS", ())),
                groups=[dict(key=d["key"], label=d["label"], color=d["color"],
                             count=len(d["parts"])) for d in groups],
                leaves=leaves, metrics=metrics, verify=verify_lines, bad=bad,
                meshed=_dt.datetime.now().isoformat(timespec="seconds"), timing=tm.steps)
    old.write_text(json.dumps(meta, ensure_ascii=False, indent=1), encoding="utf-8")
    return meta


def show_only(name):
    """짓고 뷰어에만 보낸다 — 브라우저 탭을 새로 열었을 때 (열린 뒤에 보내야 그려진다)."""
    tm = Timer()
    A = load_assembly(name)
    groups = A.build()
    tm.lap("짓기")
    comps = A.compounds(groups)
    port = getattr(A, "PORT", 3939)
    if not viewer_up(port):
        print(f"  뷰어 {port} 꺼져 있음")
        return
    from ocp_vscode import Camera, set_port, show
    set_port(port)
    show(*comps, names=[c.label for c in comps], colors=[d["color"] for d in groups],
         alphas=[d.get("alpha", 1.0) for d in groups], progress=None, reset_camera=Camera.RESET)
    tm.lap("뷰어 전송")


def _fresh(name):
    sj = ROOT / name / "output" / "scene.json"
    if not (sj.exists() and (ROOT / name / "output" / "scene.npz").exists()):
        return False
    return json.loads(sj.read_text(encoding="utf-8")).get("code") == code_hash(name)


def load_scene(name, remesh=False):
    """메시 캐시를 읽는다. 그 버전 코드가 바뀌었으면 같은 프로세스에서 메시만 다시 뜬다."""
    out = ROOT / name / "output"
    if remesh or not _fresh(name):
        print(f"  {name} 코드가 캐시와 다르다 → 메시만 다시 뜬다")
        build_scene(name, full=False)
    meta = json.loads((out / "scene.json").read_text(encoding="utf-8"))
    z = np.load(out / "scene.npz")
    return meta, z["tris"].astype(float), z["vn"].astype(float), z["owner"]


# ---------------------------------------------------------------- 렌더
def raster(px, pz, vn, cols, w, h, bg, light, amb=0.34, budget=400_000):
    """iso_render._raster 와 같은 결과를 내는 z-버퍼 래스터 — 덮는 픽셀만 계산한다.

    _raster 는 삼각형마다 바운딩박스 전체를 훑는다. 3030 의 긴 면은 760mm 짜리 가느다란
    삼각형이라 화면에 비스듬히 누우면 박스가 화면만 해진다 — 렌더 한 장에 17.5 억 칸,
    1~3 분이 거기서 나왔다. 여기서는 (삼각형, 행) 마다 무게중심좌표 세 부등식을 풀어
    그 행에서 덮는 x 구간만 구하고, 그 구간의 픽셀만 펼친다 (계산량 ≈ 실제로 칠하는 픽셀).
    포함 판정은 _raster 와 같다 (정수 픽셀에서 b >= -1e-9). 깊이가 같으면 앞 번호가 이긴다.
    """
    zbuf = np.full((h, w), -np.inf)
    img = np.tile(np.asarray(bg, dtype=float), (h, w, 1))
    zf, imf = zbuf.reshape(-1), img.reshape(-1, 3)
    x, y, z = px[:, :, 0], px[:, :, 1], pz
    x0 = np.maximum(np.floor(x.min(1)), 0)
    y0 = np.maximum(np.floor(y.min(1)), 0).astype(np.int64)
    x1 = np.minimum(np.ceil(x.max(1)), w - 1)
    y1 = np.minimum(np.ceil(y.max(1)), h - 1).astype(np.int64)
    den = (y[:, 1] - y[:, 2]) * (x[:, 0] - x[:, 2]) + (x[:, 2] - x[:, 1]) * (y[:, 0] - y[:, 2])
    ok = (x1 >= x0) & (y1 >= y0) & (np.abs(den) >= 1e-9)
    # b0, b1 은 (gx, gy) 의 1 차식: b = A*gx + B*gy + C
    A0 = (y[:, 1] - y[:, 2]) / np.where(ok, den, 1)
    B0 = (x[:, 2] - x[:, 1]) / np.where(ok, den, 1)
    A1 = (y[:, 2] - y[:, 0]) / np.where(ok, den, 1)
    B1 = (x[:, 0] - x[:, 2]) / np.where(ok, den, 1)
    C0 = -A0 * x[:, 2] - B0 * y[:, 2]
    C1 = -A1 * x[:, 2] - B1 * y[:, 2]

    def shade(t, b):
        nn = np.einsum("pk,pkj->pj", b, vn[t])
        ln = np.linalg.norm(nn, axis=1, keepdims=True)
        nn = nn / np.where(ln > 1e-12, ln, 1)
        lam = np.abs(nn @ light)
        return np.clip(cols[t] * (amb + (1 - amb) * lam)[:, None], 0, 1)

    idx = np.nonzero(ok)[0]
    rows = (y1 - y0 + 1)[idx]
    ends = np.cumsum(rows)
    start = 0
    while start < len(idx):                                  # 행 수 budget 만큼씩
        stop = int(np.searchsorted(ends, (ends[start - 1] if start else 0) + budget, "right"))
        stop = max(stop, start + 1)
        T, r = idx[start:stop], rows[start:stop]
        start = stop
        rt = np.repeat(T, r)                                 # (삼각형, 행) 쌍
        gy = (np.repeat(y0[T], r) + np.arange(r.sum()) - np.repeat(np.cumsum(r) - r, r)).astype(float)
        lo, hi = x0[rt].copy(), x1[rt].copy()
        empty = np.zeros(len(rt), bool)
        for A, Bc in ((A0[rt], B0[rt] * gy + C0[rt]), (A1[rt], B1[rt] * gy + C1[rt]),
                      (-A0[rt] - A1[rt], 1.0 - (B0[rt] + B1[rt]) * gy - C0[rt] - C1[rt])):
            with np.errstate(divide="ignore", invalid="ignore"):
                bound = -Bc / A
            pos, neg = A > 1e-15, A < -1e-15
            lo = np.where(pos, np.maximum(lo, np.floor(bound) - 1), lo)   # 1 픽셀 여유 — 판정은 아래서
            hi = np.where(neg, np.minimum(hi, np.ceil(bound) + 1), hi)
            empty |= ~pos & ~neg & (Bc < -1e-6)
        n = np.where(empty, 0, np.maximum(hi - lo + 1, 0)).astype(np.int64)
        if not n.any():
            continue
        pr = np.repeat(np.arange(len(rt)), n)
        gx = np.repeat(lo, n) + (np.arange(n.sum()) - np.repeat(np.cumsum(n) - n, n))
        t, yy = rt[pr], gy[pr]
        X, Y = x[t], y[t]
        b0 = ((Y[:, 1] - Y[:, 2]) * (gx - X[:, 2]) + (X[:, 2] - X[:, 1]) * (yy - Y[:, 2])) / den[t]
        b1 = ((Y[:, 2] - Y[:, 0]) * (gx - X[:, 2]) + (X[:, 0] - X[:, 2]) * (yy - Y[:, 2])) / den[t]
        b2 = 1.0 - b0 - b1
        m = (b0 >= -1e-9) & (b1 >= -1e-9) & (b2 >= -1e-9)
        if not m.any():
            continue
        t, gx, yy, b0, b1, b2 = t[m], gx[m], yy[m], b0[m], b1[m], b2[m]
        zz = b0 * z[t, 0] + b1 * z[t, 1] + b2 * z[t, 2]
        pix = (yy * w + gx).astype(np.int64)
        order = np.lexsort((-t, zz, pix))                    # 픽셀 → 깊이 → (같으면) 앞 번호가 뒤로
        last = order[np.r_[pix[order][1:] != pix[order][:-1], True]]
        win = last[zz[last] > zf[pix[last]]]
        zf[pix[win]] = zz[win]
        imf[pix[win]] = shade(t[win], np.stack([b0[win], b1[win], b2[win]], axis=-1))
    return img


_GPU = {}                                   # VTK 창 하나를 계속 쓴다 (만들 때 1~2 s)
USE_GPU = True


def _gpu():
    if not _GPU:
        from vtkmodules.vtkCommonCore import vtkPoints
        from vtkmodules.vtkCommonDataModel import vtkCellArray, vtkPolyData
        from vtkmodules.vtkRenderingCore import (vtkActor, vtkLight, vtkPolyDataMapper,
                                                 vtkRenderer, vtkRenderWindow,
                                                 vtkWindowToImageFilter)
        import vtkmodules.vtkRenderingOpenGL2  # noqa: F401  OpenGL 백엔드 등록
        from vtkmodules.util import numpy_support as ns
        rw = vtkRenderWindow()
        rw.SetOffScreenRendering(1)
        ren = vtkRenderer()
        rw.AddRenderer(ren)
        ren.AutomaticLightCreationOff()
        _GPU.update(rw=rw, ren=ren, ns=ns, vtkPoints=vtkPoints, vtkCellArray=vtkCellArray,
                    vtkPolyData=vtkPolyData, vtkActor=vtkActor, vtkLight=vtkLight,
                    vtkPolyDataMapper=vtkPolyDataMapper, w2i=vtkWindowToImageFilter)
    return _GPU


def _gpu_actor(G, tris, vn, cols, lit=True):
    ns, n = G["ns"], len(tris)
    pts = G["vtkPoints"]()
    pts.SetData(ns.numpy_to_vtk(tris.reshape(-1, 3).astype(np.float32), deep=True))
    cells = G["vtkCellArray"]()
    cells.SetData(ns.numpy_to_vtkIdTypeArray(np.arange(0, 3 * n + 1, 3, dtype=np.int64), deep=True),
                  ns.numpy_to_vtkIdTypeArray(np.arange(3 * n, dtype=np.int64), deep=True))
    pd = G["vtkPolyData"]()
    pd.SetPoints(pts)
    pd.SetPolys(cells)
    if vn is not None:
        nrm = ns.numpy_to_vtk(vn.reshape(-1, 3).astype(np.float32), deep=True)
        nrm.SetName("Normals")
        pd.GetPointData().SetNormals(nrm)
    rgb = ns.numpy_to_vtk((np.repeat(cols, 3, axis=0) * 255 + 0.5).clip(0, 255).astype(np.uint8),
                          deep=True)
    rgb.SetName("rgb")
    pd.GetPointData().SetScalars(rgb)
    m = G["vtkPolyDataMapper"]()
    m.SetInputData(pd)
    m.SetColorModeToDirectScalars()
    m.ScalarVisibilityOn()
    a = G["vtkActor"]()
    a.SetMapper(m)
    p = a.GetProperty()
    if lit:                                 # _raster 와 같은 음영: 색 x (0.34 + 0.66 |n·l|)
        p.SetAmbient(0.34)
        p.SetDiffuse(0.66)
        p.SetSpecular(0.0)
        p.SetInterpolationToPhong()
    else:
        p.LightingOff()
    return a


def _draw_gpu(tris, vn, cols, ground_tris, d, upv, center, scale, w, h, key=None):
    G = _gpu()
    rw, ren = G["rw"], G["ren"]
    actors = G.setdefault("actors", {})        # 같은 메시·같은 색이면 VTK 객체를 다시 쓴다
    ren.RemoveAllViewProps()
    ren.RemoveAllLights()
    ren.SetBackground(*BG)
    if key is None or key not in actors:
        a = _gpu_actor(G, tris, vn, cols)
        if key is not None:
            actors[key] = a
    ren.AddActor(actors[key] if key is not None else a)
    if ground_tris is not None:
        gk = None if key is None else ("ground", key[0])
        if gk is None or gk not in actors:
            g = _gpu_actor(G, ground_tris, None, np.tile(GROUND_RGB, (len(ground_tris), 1)),
                           lit=False)
            if gk is not None:
                actors[gk] = g
        ren.AddActor(actors[gk] if gk is not None else g)
    # |n·l| = max(0, n·l) + max(0, -n·l) — 반대 방향 평행광을 하나 더 둔다 (감김이 뒤집힌 면도 밝게)
    for sgn in (1, -1):
        L = G["vtkLight"]()
        L.SetLightTypeToSceneLight()
        L.SetPositional(False)
        L.SetFocalPoint(*center)
        L.SetPosition(*(center + sgn * IR._unit(LIGHT) * 1000))
        L.SetIntensity(1.0)
        ren.AddLight(L)
    ren.TwoSidedLightingOff()
    cam = ren.GetActiveCamera()
    cam.SetParallelProjection(True)
    cam.SetFocalPoint(*center)
    cam.SetPosition(*(center + d * 5000))
    cam.SetViewUp(*upv)
    cam.SetParallelScale(scale)
    rw.SetSize(w, h)
    ren.ResetCameraClippingRange()
    rw.Render()
    f = G["w2i"]()
    f.SetInput(rw)
    f.SetInputBufferTypeToRGB()
    f.ReadFrontBufferOff()
    f.Update()
    img = G["ns"].vtk_to_numpy(f.GetOutput().GetPointData().GetScalars())
    return img.reshape(h, w, 3)[::-1].astype(float) / 255


def draw(tris, vn, cols, direction, up, size, window=None, ground=True, ss=2, key=None):
    """삼각형을 그려 PIL 이미지로. window=(min, max) 면 그 상자를 화면에 맞춘다.

    GPU (VTK 오프스크린) 로 그린다 — 장당 1 초 안쪽. 안 되면 numpy raster 로 내려간다.
    """
    global USE_GPU
    d, lt = IR._unit(direction), IR._unit(LIGHT)
    right = IR._unit(np.cross(up, d))
    upv = np.cross(d, right)
    if window is None:
        pts = tris.reshape(-1, 3)
    else:
        lo, hi = np.asarray(window[0]), np.asarray(window[1])
        pts = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1])
                        for z in (lo[2], hi[2])])
    pr, pu = pts @ right, pts @ upv
    c = np.array([(pr.min() + pr.max()) / 2, (pu.min() + pu.max()) / 2])
    hx, hy = max((pr.max() - pr.min()) / 2, 1.0) * 1.1, max((pu.max() - pu.min()) / 2, 1.0) * 1.1
    gt, gn = IR._ground(tris, LIGHT) if ground else (None, None)
    w, h = size[0] * ss, size[1] * ss
    sc = min((w / 2) / hx, (h / 2) / hy)
    img = None
    if USE_GPU:
        try:
            dm = (tris @ d).mean()
            center = c[0] * right + c[1] * upv + dm * d
            img = _draw_gpu(tris, vn, cols, gt, d, upv, center, (h / 2) / sc, w, h, key)
        except Exception as e:                       # noqa: BLE001 — 렌더러가 없으면 CPU 로
            print(f"  GPU 렌더 실패 → CPU 로 그린다: {e}")
            USE_GPU = False
    if img is None:
        if gt is not None:
            tris = np.concatenate([tris, gt])
            vn = np.concatenate([vn, gn])
            cols = np.concatenate([cols, np.tile(GROUND_RGB, (len(gt), 1))])
        px = np.stack([(tris @ right - c[0]) * sc + w / 2,
                       h / 2 - (tris @ upv - c[1]) * sc], axis=-1)
        img = raster(px, tris @ d, vn, cols, w, h, BG, lt)
    img = img.reshape(size[1], ss, size[0], ss, 3).mean(axis=(1, 3))
    return Image.fromarray((np.clip(img, 0, 1) * 255).astype(np.uint8))


def leaf_colors(meta, owner, highlight=None, hcolor=None):
    """부품색 (삼각형별). highlight 집합이 있으면 그 부품만 hcolor, 나머지는 흐리게."""
    base = np.array([_hex(l["color"]) for l in meta["leaves"]])
    if highlight is not None:
        # 흐린 부품은 **무채색**으로 — 제 색을 남기면 파란 H 크로스가 "이번 자리"처럼 보인다
        gray = base @ np.array([0.299, 0.587, 0.114])
        base = np.repeat(gray[:, None], 3, axis=1) * (1 - GHOST) + np.array(BG) * GHOST
        for i, l in enumerate(meta["leaves"]):
            if l["path"] in highlight:
                base[i] = hcolor
    return base[owner]


def label(img, text, xy=(16, 10), px=26, fill=(40, 44, 48)):
    """글자 뒤에 바탕 상자를 깐다 — 위에서 본 그림은 타이어가 모서리까지 온다."""
    d, f = ImageDraw.Draw(img), _font(px)
    x0, y0, x1, y1 = d.textbbox(xy, text, font=f)
    d.rectangle((x0 - 8, y0 - 6, x1 + 8, y1 + 6), fill=(248, 249, 250))
    d.text(xy, text, fill=fill, font=f)
    return img


def grid(cells, cols, header=None, gap=10, bg=(244, 245, 246), head_px=34):
    w, h = cells[0].size
    rows = (len(cells) + cols - 1) // cols
    top = head_px + 24 if header else 0
    canvas = Image.new("RGB", (cols * w + (cols + 1) * gap, top + rows * h + (rows + 1) * gap), bg)
    for i, im in enumerate(cells):
        canvas.paste(im, (gap + (i % cols) * (w + gap), top + gap + (i // cols) * (h + gap)))
    if header:
        label(canvas, header, (gap + 6, 12), head_px)
    return canvas


# ---------------------------------------------------------------- 비교
def leaf_sig(tris, owner, n):
    """부품별 (겉넓이, 겉면 무게중심) — 판 안의 구멍만 옮겨도 바뀐다.

    외곽 크기 · 부피만 보면 앞판 카메라 창이 10 mm 올라간 것 (v15) 을 못 잡았다."""
    e1, e2 = tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0]
    area = 0.5 * np.linalg.norm(np.cross(e1, e2), axis=1)
    cen = tris.mean(axis=1)
    A = np.bincount(owner, area, n)
    C = np.stack([np.bincount(owner, area * cen[:, k], n) for k in range(3)], 1) / np.maximum(A, 1e-9)[:, None]
    return A, C


def diff(prev, cur, psig=None, csig=None):
    P = {l["path"]: l for l in prev["leaves"]}
    C = {l["path"]: l for l in cur["leaves"]}
    pi = {l["path"]: i for i, l in enumerate(prev["leaves"])}
    ci = {l["path"]: i for i, l in enumerate(cur["leaves"])}
    out = []
    for path in list(P) + [p for p in C if p not in P]:
        a, b = P.get(path), C.get(path)
        if a and not b:
            out.append(dict(kind="삭제", path=path, key=a["key"], label=a["label"], a=a, b=None))
        elif b and not a:
            out.append(dict(kind="추가", path=path, key=b["key"], label=b["label"], a=None, b=b))
        else:
            amin, amax, bmin, bmax = map(np.array, (a["min"], a["max"], b["min"], b["max"]))
            same_size = np.allclose(amax - amin, bmax - bmin, atol=0.05)
            # 같은 형상이면 OCC 부피는 거의 똑같이 나온다. 상대 1e-4 로 뒀더니 바닥판 노치 R3
            # (부피 +155 mm³ = 0.009 %) 를 못 잡았다 — 절대 0.5 mm³ 로 본다.
            same_vol = abs(a["volume"] - b["volume"]) <= max(0.5, 1e-7 * a["volume"])
            delta = (bmin + bmax) / 2 - (amin + amax) / 2
            if same_size and same_vol and np.allclose(delta, 0, atol=0.05) and (
                    ("com" in a and "com" in b) or psig is not None):
                if "com" in a and "com" in b:             # B-rep 정확값 — 아주 작은 허용치
                    inner = (abs(a["area"] - b["area"]) > max(0.01, 1e-7 * a["area"])
                             or np.abs(np.array(a["com"]) - np.array(b["com"])).max() > 0.005)
                else:                                      # 옛 캐시 — 메시 값이라 느슨하게 (메시가 매번 조금 다르다)
                    (pa, pc), (ca, cc) = psig, csig
                    i, j = pi[path], ci[path]
                    inner = (abs(pa[i] - ca[j]) > 0.005 * max(pa[i], 1.0)
                             or np.abs(pc[i] - cc[j]).max() > 0.5)
                if inner:
                    out.append(dict(kind="형상 변경", path=path, key=b["key"], label=b["label"], a=a, b=b,
                                    delta=[0, 0, 0], inner=True))
                    continue
            if same_size and same_vol and np.allclose(delta, 0, atol=0.05):
                if a["color"].lower() != b["color"].lower():   # 형상은 같고 색 (재질) 만 바뀜
                    out.append(dict(kind="색·재질 변경", path=path, key=b["key"], label=b["label"],
                                    a=a, b=b, delta=[0, 0, 0]))
                continue
            kind = "이동" if same_size and same_vol else "형상 변경"
            out.append(dict(kind=kind, path=path, key=b["key"], label=b["label"], a=a, b=b,
                            delta=delta.tolist()))
    return out


def describe(ch):
    if ch["kind"] == "이동":
        d = ch["delta"]
        return f"{ch['label']} 이동 Δ({d[0]:+.1f}, {d[1]:+.1f}, {d[2]:+.1f})"
    if ch["kind"] == "색·재질 변경":
        return f"{ch['label']} 색 {ch['a']['color']} → {ch['b']['color']}"
    if ch["kind"] == "형상 변경" and ch.get("inner"):
        return f"{ch['label']} 형상 변경 (외곽 · 부피 같음 — 안쪽 구멍 · 창 자리가 바뀜)"
    if ch["kind"] == "형상 변경":
        return (f"{ch['label']} 형상 변경 (부피 {ch['a']['volume']/1000:.1f} → "
                f"{ch['b']['volume']/1000:.1f} cm³)")
    return f"{ch['label']} {ch['kind']}"


def describe_all(chs):
    """삭제·추가는 이름 줄기(끝 번호 뗀 것)로 묶는다 — '기둥1 삭제 … 기둥6 삭제' → '기둥 ×6 삭제'."""
    out, bunch = [], {}
    for c in chs:
        if c["kind"] in ("삭제", "추가"):
            bunch.setdefault((c["kind"], c["label"].rstrip("0123456789_")), []).append(c)
        else:
            out.append(describe(c))
    for (kind, stem), cs in bunch.items():
        out.append(f"{stem} ×{len(cs)} {kind}" if len(cs) > 1 else describe(cs[0]))
    return out


def _counts(chs):
    kinds = {}
    for c in chs:
        kinds[c["kind"]] = kinds.get(c["kind"], 0) + 1
    return " · ".join(f"{k} {n}" for k, n in kinds.items())


def _window(chs, pad_frac=0.18, pad_min=25.0, half_min=70.0):
    boxes = []
    for c in chs:
        for l in (c["a"], c["b"]):
            if l:
                boxes.append((np.array(l["min"]), np.array(l["max"])))
    lo = np.min([b[0] for b in boxes], axis=0)
    hi = np.max([b[1] for b in boxes], axis=0)
    mid, half = (lo + hi) / 2, (hi - lo) / 2
    half = np.maximum(half + np.maximum(half * 2 * pad_frac, pad_min), half_min)
    return mid - half, mid + half


# ---------------------------------------------------------------- 기록
def record(name, rebuild=False):
    """버전 하나를 기록한다 — 짓기·검증·STEP·뷰어·메시는 **한 프로세스에서 한 번**.

    코드가 지난 기록과 같으면 짓지 않고 메시 캐시로 이미지·기록만 다시 만든다 (rebuild 로 강제).
    """
    tm = Timer()
    if rebuild or not _fresh(name):
        build_steps = build_scene(name)["timing"]
    else:
        print(f"  코드 그대로 → 짓기 생략 (캐시 사용, 다시 지으려면 --rebuild)")
        build_steps = json.loads((ROOT / name / "output" / "scene.json")
                                 .read_text(encoding="utf-8")).get("timing", [])
    tm.lap("짓기 단계 합계")
    out = ROOT / name / "output"
    cur = json.loads((out / "scene.json").read_text(encoding="utf-8"))
    z = np.load(out / "scene.npz")
    ct, cv, co = z["tris"].astype(float), z["vn"].astype(float), z["owner"]
    vdir, img_dir = ROOT / name, ROOT / name / "images"
    img_dir.mkdir(exist_ok=True)
    for old in img_dir.glob("change_*.png"):
        old.unlink()
    title = f"{cur['version']}  {cur['title']}  ({cur['date']})"

    # 전체 모습
    _GPU.get("actors", {}).clear()           # key = (메시 이름, 색 묶음) — 이 기록 안에서만 쓴다
    cols = leaf_colors(cur, co)
    hero = label(draw(ct, cv, cols, *VIEWS["ISO"], (1600, 1100), key=("cur", "base")), title, px=34)
    hero.save(img_dir / "overall_iso.png")
    hero.resize((480, 330), Image.LANCZOS).save(img_dir / "thumb.png")
    cells = [label(draw(ct, cv, cols, d, u, (900, 620), ground=(d[2] >= 0), key=("cur", "base")), v)
             for v, (d, u) in VIEWS.items()]
    grid(cells, 2, header=title).save(img_dir / "overall.png")
    tm.lap("전체 모습 이미지 5 장")

    # 변화
    changes, sections = [], []
    if cur["prev"]:
        prev, pt, pv, po = load_scene(cur["prev"])
        changes = diff(prev, cur, leaf_sig(pt, po, len(prev["leaves"])), leaf_sig(ct, co, len(cur["leaves"])))
        by_key = {}
        for c in changes:
            by_key.setdefault(c["key"], []).append(c)
        glabel = {g["key"]: g["label"] for g in prev["groups"] + cur["groups"]}
        for n, (key, chs) in enumerate(by_key.items(), start=1):
            win = _window(chs)
            red = {c["path"] for c in chs if c["a"]}
            blue = {c["path"] for c in chs if c["b"]}
            cells = []
            ca, cb = leaf_colors(prev, po, red, RED), leaf_colors(cur, co, blue, BLUE)
            for v in ("ISO", "위"):
                d, u = VIEWS[v]
                a = draw(pt, pv, ca, d, u, (900, 620), window=win, key=("prev", n))
                b = draw(ct, cv, cb, d, u, (900, 620), window=win, key=("cur", n))
                cells += [label(a, f"이전 {prev['version']} · {v}", fill=(150, 40, 35)),
                          label(b, f"이번 {cur['version']} · {v}", fill=(25, 80, 150))]
            head = f"변화 {n} — {glabel.get(key, key)}: {_counts(chs)}   (빨강 = 이전 자리, 파랑 = 이번 자리)"
            fn = f"change_{n:02d}_{key}.png"
            grid(cells, 2, header=head, head_px=28).save(img_dir / fn)
            sections.append((n, key, glabel.get(key, key), chs, fn))
            print(f"  변화 {n}: {fn}  ({_counts(chs)})")
        tm.lap(f"변화 이미지 {len(sections)} 장")

    timing = build_steps + tm.steps[1:]
    write_readme(vdir, cur, sections, timing)
    write_index()
    print(f"  기록: {vdir / 'README.md'}   총 {tm.total():.1f}s")
    if cur["bad"]:
        print("  ⚠ 검증 실패:", cur["bad"])


def write_readme(vdir, cur, sections, timing=()):
    readme = vdir / "README.md"
    if readme.exists():
        text = readme.read_text(encoding="utf-8")
    else:
        prev = f"[{cur['prev']}](../{cur['prev']}/README.md)" if cur["prev"] else "없음"
        text = (f"# {cur['version']} — {cur['title']}\n\n"
                f"- 날짜: {cur['date']}\n- 이전 버전: {prev}\n\n"
                "## 무엇을 왜 바꿨나\n\n(요청 · 이유 · 버린 대안)\n\n"
                "## 남은 일 / 다음 버전 후보\n\n- \n\n"
                f"{BEGIN}\n{END}\n")
    stamp = _dt.datetime.now().strftime("%Y-%m-%d %H:%M")
    L = [f"## 기록 (자동 · record.py · {stamp})", "", f"**요약** — {cur['summary']}", "",
         "### 전체 모습", "", "![전체 모습](images/overall.png)", ""]
    if cur["prev"]:
        L += [f"### 이전 버전({cur['prev']}) 대비 변화", ""]
        if not sections:
            L += ["바뀐 부품 없음.", ""]
        else:
            L += ["| # | 그룹 | 변화 | 부품 |", "|---|---|---|---|"]
            for n, key, gl, chs, fn in sections:
                L.append(f"| {n} | {gl} | {_counts(chs)} | " + "<br>".join(describe_all(chs)) + " |")
            L.append("")
            for n, key, gl, chs, fn in sections:
                L += [f"**변화 {n} — {gl}**", "", f"![변화 {n}](images/{fn})", ""]
    lo = np.min([l["min"] for l in cur["leaves"]], axis=0)
    hi = np.max([l["max"] for l in cur["leaves"]], axis=0)
    body = [l for l in cur["leaves"] if l["key"] not in cur["wheel_keys"]]
    clear = min(l["min"][2] for l in body) if body else float("nan")
    L += ["### 수치", "", "| 항목 | 값 |", "|---|---|"]
    L += [f"| {k} | {v} |" for k, v in cur["metrics"]]
    L += ["| 전체 크기 (x × y × z) | " + " × ".join(f"{v:.1f}" for v in hi - lo) + " mm |",
          f"| 최저 지상고 (바퀴 제외, 지면 z=0) | {clear:.1f} mm |", ""]
    L += ["### 부품 구성", "", "| 그룹 | 개수 |", "|---|---|"]
    L += [f"| {g['label']} | {g['count']} |" for g in cur["groups"]]
    L.append("")
    if cur["verify"] or cur["bad"] is not None:
        L += ["### 검증", "", "```"] + cur["verify"] + [
            f"verify: {'OK' if not cur['bad'] else cur['bad']}", "```", ""]
    if timing:
        L += ["### 기록 소요", "", " · ".join(f"{k} {v:.1f}s" for k, v in timing), ""]
    auto = "\n".join(L)
    if BEGIN in text and END in text:
        head, rest = text.split(BEGIN, 1)
        tail = rest.split(END, 1)[1]
        text = f"{head}{BEGIN}\n{auto}\n{END}{tail}"
    else:
        text = f"{text.rstrip()}\n\n{BEGIN}\n{auto}\n{END}\n"
    readme.write_text(text, encoding="utf-8")


def write_index():
    """models/vehicle/README.md 의 버전 표를 폴더에서 다시 만든다."""
    rows = ["| 버전 | 날짜 | 제목 | 요약 | 모습 |", "|---|---|---|---|---|"]
    for v in sorted(p for p in ROOT.iterdir() if re.match(r"v\d\d_", p.name)):
        sj = v / "output" / "scene.json"
        if not sj.exists():
            continue
        m = json.loads(sj.read_text(encoding="utf-8"))
        thumb = f"![]({v.name}/images/thumb.png)" if (v / "images" / "thumb.png").exists() else ""
        rows.append(f"| [{m['version']}]({v.name}/README.md) | {m['date']} | {m['title']} | "
                    f"{m['summary']} | {thumb} |")
    index = ROOT / "README.md"
    text = index.read_text(encoding="utf-8") if index.exists() else f"# 차량 재설계\n\n{BEGIN}\n{END}\n"
    head, rest = text.split(BEGIN, 1)
    tail = rest.split(END, 1)[1]
    index.write_text(f"{head}{BEGIN}\n" + "\n".join(rows) + f"\n{END}{tail}", encoding="utf-8")


# ---------------------------------------------------------------- 새 버전
def new_version(name, title):
    assert re.match(r"v\d\d_[a-z0-9_]+$", name), "이름은 vNN_소문자 (예: v02_posts)"
    dst = ROOT / name
    assert not dst.exists(), f"{dst} 가 이미 있다"
    src = sorted(p for p in ROOT.iterdir() if re.match(r"v\d\d_", p.name)
                 and (p / "assembly.py").exists())[-1]
    dst.mkdir()
    for f in src.glob("*.py"):
        shutil.copy2(f, dst / f.name)
    (dst / "images").mkdir()
    (dst / "output").mkdir()
    asm = dst / "assembly.py"
    text = asm.read_text(encoding="utf-8")
    for k, v in (("VERSION", name), ("TITLE", title), ("PREV", src.name),
                 ("DATE", _dt.date.today().isoformat()), ("SUMMARY", "(작성)")):
        text = re.sub(rf'^{k} = .*$', f'{k} = "{v}"', text, count=1, flags=re.M)
    asm.write_text(text, encoding="utf-8")
    print(f"새 버전 {name} ← {src.name} 복사. assembly.py 의 SUMMARY 와 설명을 고치고 시작한다.")


# ---------------------------------------------------------------- 상주 워커
# build123d import 가 20 s 다 (sympy 9 s 포함). 한 번 import 해 둔 프로세스를 띄워 두고
# 작업만 넘기면 기록 한 번이 그만큼 빨라진다. 워커는 작업마다 record.py 를 **새로 읽어**
# 실행하므로 record.py · 버전 코드를 고쳐도 다시 띄울 필요가 없다.
WORKER_PORT = 3990
WORKER_LOG = ROOT / "worker.log"
_EXIT = "\x00EXIT "


class _Pipe(io.TextIOBase):
    def __init__(self, wfile):
        self.wfile = wfile

    def write(self, s):
        self.wfile.write(s.encode("utf-8", "replace"))
        self.wfile.flush()
        return len(s)


def serve():
    import socketserver
    import threading
    import traceback
    import build123d          # noqa: F401 — 무거운 import 를 미리 해 둔다
    try:
        import ocp_vscode     # noqa: F401
        _gpu()
    except Exception:          # noqa: BLE001
        pass

    class Handler(socketserver.StreamRequestHandler):
        def handle(self):
            line = self.rfile.readline()
            if not line.strip():                  # 준비 확인 (접속만 하고 끊음)
                return
            argv = json.loads(line.decode("utf-8"))["argv"]
            if argv == ["--quit"]:
                threading.Thread(target=self.server.shutdown).start()
                self.wfile.write(f"{_EXIT}0\n".encode())
                return
            pipe, code = _Pipe(self.wfile), 0
            with contextlib.redirect_stdout(pipe), contextlib.redirect_stderr(pipe):
                try:
                    spec = importlib.util.spec_from_file_location("record_job", __file__)
                    job = importlib.util.module_from_spec(spec)
                    spec.loader.exec_module(job)          # 고친 record.py 를 매번 새로 읽는다
                    job.run(argv)
                except SystemExit as e:
                    code = e.code if isinstance(e.code, int) else 1
                except Exception:                         # noqa: BLE001
                    traceback.print_exc()
                    code = 1
            self.wfile.write(f"\n{_EXIT}{code}\n".encode())

    with socketserver.TCPServer(("127.0.0.1", WORKER_PORT), Handler) as srv:
        print(f"record worker ready :{WORKER_PORT}", flush=True)
        srv.serve_forever()


def _client(argv, timeout=1.0):
    """워커에 작업을 넘기고 출력을 그대로 흘려 준다. 워커가 없으면 None."""
    import socket
    try:
        s = socket.create_connection(("127.0.0.1", WORKER_PORT), timeout=timeout)
    except OSError:
        return None
    s.settimeout(None)
    code = 1
    with s, s.makefile("rb") as f:
        s.sendall((json.dumps({"argv": argv}) + "\n").encode())
        for line in f:
            t = line.decode("utf-8", "replace")
            if t.startswith(_EXIT):
                code = int(t[len(_EXIT):])
                break
            sys.stdout.write(t)
            sys.stdout.flush()
    return code


def start_worker(wait=180):
    """워커를 셸과 떨어진 프로세스로 띄우고 준비될 때까지 기다린다."""
    import time
    log = open(WORKER_LOG, "ab")
    base = 0x00000008 | 0x00000200            # DETACHED_PROCESS | CREATE_NEW_PROCESS_GROUP
    for flags in (base | 0x01000000, base):   # + CREATE_BREAKAWAY_FROM_JOB (셸이 닫혀도 산다)
        try:
            subprocess.Popen([sys.executable, str(Path(__file__)), "--serve"], stdout=log,
                             stderr=log, stdin=subprocess.DEVNULL, creationflags=flags,
                             close_fds=True, cwd=str(ROOT.parents[1]))
            break
        except OSError:
            continue
    t0 = time.time()
    while time.time() - t0 < wait:
        if viewer_up(WORKER_PORT):
            print(f"  워커 준비 {time.time() - t0:.0f}s (다음부터는 import 를 건너뛴다)")
            return True
        time.sleep(0.5)
    print("  워커가 안 뜬다 — 이번엔 직접 돌린다. 로그:", WORKER_LOG)
    return False


def run(argv):
    ap = argparse.ArgumentParser()
    ap.add_argument("name")
    ap.add_argument("title", nargs="?")
    ap.add_argument("extra", nargs="?")
    ap.add_argument("--mesh-only", action="store_true", help="이 버전 메시 캐시만")
    ap.add_argument("--rebuild", action="store_true", help="코드가 그대로여도 다시 짓는다")
    ap.add_argument("--show", action="store_true", help="뷰어에만 다시 보낸다 (검증 · 기록 없음)")
    a = ap.parse_args(argv)
    if a.show:
        show_only(a.name)
    elif a.name == "new":
        new_version(a.title, a.extra or a.title)
    elif a.mesh_only:
        build_scene(a.name, full=False)
    else:
        record(a.name, rebuild=a.rebuild)


def main():
    argv = sys.argv[1:]
    if argv == ["--serve"]:
        return serve()
    if argv == ["--stop-worker"]:
        print("워커 종료" if _client(["--quit"]) is not None else "워커 없음")
        return
    local = "--local" in argv                  # 워커를 거치지 않고 이 프로세스에서
    argv = [x for x in argv if x != "--local"]
    if argv and argv[0] != "new" and not local:
        code = _client(argv)
        if code is None:
            print("  (기록 워커가 없다 → 띄운다)")
            if start_worker():
                code = _client(argv)
        if code is not None:
            sys.exit(code)
    run(argv)


if __name__ == "__main__":
    main()
