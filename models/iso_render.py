# 아이소메트릭 셰이딩 렌더 — ocp_vscode 뷰어 없이 PNG 로 확인할 때 쓴다.
#
# 형상을 삼각형으로 쪼갠 뒤 z-버퍼로 직접 래스터한다. 정식 렌더러가 아니라 형상
# 확인용이다(그림자·반사 없음, 램버트 음영만). 대신 앞뒤가 섞이는 일은 없고,
# 꼭짓점 법선을 면 단위로 평균 내 곡면이 각지지 않게 그린다.
#
# 쓰는 법:
#   from iso_render import render
#   render(assembly, "out.png")                                  # 기본 아이소
#   render(assembly, "cut.png", skip=lambda n: n.endswith("우"))  # 반쪽 걷어낸 그림

import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np

from build123d import Location

ISO = (1.0, -1.0, 0.55)          # 기본 시선 (앞-우-위에서 내려다본다)


def _leaves(shape, loc=None):
    """(잎 Shape, 월드 Location) 을 훑는다. Compound 안의 상대 위치를 누적한다."""
    loc = (loc or Location()) * shape.location
    kids = list(getattr(shape, "children", ()) or ())
    if kids:
        for k in kids:
            yield from _leaves(k, loc)
    else:
        yield shape, loc


def _unit(v):
    v = np.asarray(v, dtype=float)
    return v / np.linalg.norm(v)


def collect(assembly, tol=0.4, ang=0.2, skip=None):
    """어셈블리를 삼각형 (N,3,3) · 꼭짓점법선 (N,3,3) · 부품색 (N,3) 으로 만든다."""
    tris, norms, cols = [], [], []
    for leaf, wl in _leaves(assembly):
        if skip and skip(leaf.label or ""):
            continue
        vs, fs = leaf.located(wl).tessellate(tol, ang)
        if not fs:
            continue
        v = np.array([(p.X, p.Y, p.Z) for p in vs])
        f = np.array(fs)
        t = v[f]
        fn = np.cross(t[:, 1] - t[:, 0], t[:, 2] - t[:, 0])     # 면적 가중 면법선
        vn = np.zeros_like(v)
        for k in range(3):                                      # 꼭짓점으로 모은다
            np.add.at(vn, f[:, k], fn)
        ln = np.linalg.norm(vn, axis=1, keepdims=True)
        tris.append(t)
        norms.append(np.where(ln > 1e-12, vn / np.where(ln > 0, ln, 1), 0)[f])
        c = tuple(leaf.color)[:3] if leaf.color else (0.72, 0.74, 0.76)
        cols.append(np.tile(c, (len(f), 1)))
    if not tris:
        raise ValueError("그릴 삼각형이 없다")
    return (np.concatenate(tris), np.concatenate(norms), np.concatenate(cols))


def _ground(tris, light, n=72, scale=1.15):
    """접지면 원판. 법선을 광원 쪽으로 줘서 음영 없이 제 색으로 깔린다.

    높이는 **모델 밑면과 z=0 중 낮은 쪽**이다. z=0 에 고정해 뒀더니, 데이텀이
    지면이 아닌 모델(hoverboard_mount 는 z=0 이 축 중심)에서 원판이 부품 한가운데를
    가로질러 아래쪽 몸체를 통째로 가렸다. 형상이 잘못된 줄 알고 두 번 뒤졌다.
    """
    rad = np.abs(tris[:, :, :2]).max() * scale
    z0 = min(0.0, float(tris[:, :, 2].min()))
    t = np.linspace(0, 2 * np.pi, n + 1)
    ring = np.stack([rad * np.cos(t), rad * np.sin(t), np.full_like(t, z0)], axis=1)
    g = np.stack([np.tile((0.0, 0.0, z0), (n, 1)), ring[:-1], ring[1:]], axis=1)
    return g, np.tile(_unit(light), (n, 3, 1))


def _raster(px, pz, vn, cols, w, h, bg, light, amb=0.34):
    """z-버퍼 래스터 — 삼각형마다 화면 바운딩박스 안을 무게중심좌표로 채운다."""
    zbuf = np.full((h, w), -np.inf)
    img = np.tile(np.asarray(bg, dtype=float), (h, w, 1))
    for i in range(len(px)):
        x, y, z = px[i, :, 0], px[i, :, 1], pz[i]
        x0, y0 = max(int(np.floor(x.min())), 0), max(int(np.floor(y.min())), 0)
        x1, y1 = min(int(np.ceil(x.max())), w - 1), min(int(np.ceil(y.max())), h - 1)
        if x1 < x0 or y1 < y0:
            continue
        den = (y[1] - y[2]) * (x[0] - x[2]) + (x[2] - x[1]) * (y[0] - y[2])
        if abs(den) < 1e-9:
            continue
        gx, gy = np.meshgrid(np.arange(x0, x1 + 1.0), np.arange(y0, y1 + 1.0))
        b0 = ((y[1] - y[2]) * (gx - x[2]) + (x[2] - x[1]) * (gy - y[2])) / den
        b1 = ((y[2] - y[0]) * (gx - x[2]) + (x[0] - x[2]) * (gy - y[2])) / den
        b2 = 1.0 - b0 - b1
        m = (b0 >= -1e-9) & (b1 >= -1e-9) & (b2 >= -1e-9)
        if not m.any():
            continue
        zz = b0 * z[0] + b1 * z[1] + b2 * z[2]
        win = m & (zz > zbuf[y0:y1 + 1, x0:x1 + 1])
        if not win.any():
            continue
        b = np.stack([b0[win], b1[win], b2[win]], axis=-1)      # (P,3)
        nn = b @ vn[i]                                          # 픽셀별 보간 법선
        ln = np.linalg.norm(nn, axis=1, keepdims=True)
        nn = nn / np.where(ln > 1e-12, ln, 1)
        lam = np.abs(nn @ light)                                # 뒤집힌 면은 절대값
        zbuf[y0:y1 + 1, x0:x1 + 1][win] = zz[win]
        img[y0:y1 + 1, x0:x1 + 1][win] = np.clip(
            cols[i] * (amb + (1 - amb) * lam)[:, None], 0, 1)
    return img


def ry(deg):
    """Y축 회전행렬 — build123d 의 Rot(0, deg, 0) 과 같은 부호 규약."""
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def pivot(mat, p):
    """점 p 를 지나는 축 둘레 회전 → (행렬, 이동) 쌍."""
    return mat, np.asarray(p, float) - mat @ np.asarray(p, float)


def chain(*xf):
    """변환 여러 개를 왼쪽부터 겹친다. chain(A, B) 는 A 다음 B 를 적용."""
    m, t = np.eye(3), np.zeros(3)
    for mm, tt in xf:
        m, t = mm @ m, mm @ t + tt
    return m, t


def animate(groups, poses, out, direction=ISO, up=(0, 0, 1), light=(0.55, -1.0, 0.75),
            size=(640, 480), bg="#eef0f2", ss=2, ground=True, fps=12, captions=None,
            margin=1.16):
    """부품군을 프레임마다 변환해 GIF 로 굽는다.

    groups  : {이름: (tris, vn, cols)}  — collect() 결과. 메시는 여기서 한 번만 뜬다.
    poses   : 프레임마다 {이름: (3x3 회전, 이동)}. 빠진 이름은 제자리.
    """
    from PIL import Image, ImageDraw, ImageFont

    d, lt = _unit(direction), _unit(light)
    right = _unit(np.cross(up, d))
    upv = np.cross(d, right)

    # 카메라는 전 프레임 공통 — 각 부품군의 바운딩박스 꼭짓점만 돌려 화면을 맞춘다
    box = {}
    for k, (t, _, _) in groups.items():
        mn, mx = t.reshape(-1, 3).min(0), t.reshape(-1, 3).max(0)
        box[k] = np.array([[a, b, cc] for a in (mn[0], mx[0]) for b in (mn[1], mx[1])
                           for cc in (mn[2], mx[2])])
    seen = np.concatenate([box[k] @ m.T + tv for p in poses for k in groups
                           for m, tv in [p.get(k, (np.eye(3), np.zeros(3)))]])
    flat = np.stack([seen @ right, seen @ upv], -1)
    mn2, mx2 = flat.min(0), flat.max(0)
    c, r = (mn2 + mx2) / 2, (mx2 - mn2).max() / 2 * margin

    w, h = size[0] * ss, size[1] * ss
    sc = (h / 2) / r
    gt, gn = _ground(np.concatenate([g[0] for g in groups.values()]), light) \
        if ground else (None, None)
    gcol = np.tile((0.87, 0.885, 0.9), (len(gt), 1)) if ground else None

    try:
        font = ImageFont.truetype("C:/Windows/Fonts/malgun.ttf", 20)
    except OSError:
        font = ImageFont.load_default()

    frames = []
    for i, pose in enumerate(poses):
        tri, vn, col = [], [], []
        for k, (t, n, cl) in groups.items():
            m, tv = pose.get(k, (np.eye(3), np.zeros(3)))
            tri.append(t @ m.T + tv)
            vn.append(n @ m.T)
            col.append(cl)
        if ground:
            tri.append(gt)
            vn.append(gn)
            col.append(gcol)
        tri, vn, col = (np.concatenate(tri), np.concatenate(vn), np.concatenate(col))
        px = np.stack([(tri @ right - c[0]) * sc + w / 2,
                       h / 2 - (tri @ upv - c[1]) * sc], axis=-1)
        img = _raster(px, tri @ d, vn, col, w, h, matplotlib.colors.to_rgb(bg), lt)
        img = img.reshape(size[1], ss, size[0], ss, 3).mean(axis=(1, 3))
        im = Image.fromarray((np.clip(img, 0, 1) * 255).astype(np.uint8))
        if captions:
            ImageDraw.Draw(im).text((size[0] / 2, size[1] - 28), captions[i],
                                    fill=(58, 63, 70), font=font, anchor="mm")
        frames.append(im.convert("P", palette=Image.ADAPTIVE, colors=128))
        print(f"    프레임 {i + 1}/{len(poses)}", end="\r")

    out = Path(out)
    out.parent.mkdir(parents=True, exist_ok=True)
    frames[0].save(out, save_all=True, append_images=frames[1:],
                   duration=int(1000 / fps), loop=0, optimize=True)
    print(f"  애니메이션 저장: {out}  ({len(frames)} 프레임)")
    return out


def render(assembly, out, direction=ISO, up=(0, 0, 1), light=(0.55, -1.0, 0.75),
           size=(1500, 1150), bg="#eef0f2", tol=0.4, skip=None, ground=True,
           title=None, ss=2):
    """어셈블리를 아이소메트릭 PNG 로 저장하고 경로를 돌려준다 (ss = 수퍼샘플 배율)."""
    tris, vn, cols = collect(assembly, tol=tol, skip=skip)
    d, lt = _unit(direction), _unit(light)                # 물체 → 카메라 / 광원
    right = _unit(np.cross(up, d))
    upv = np.cross(d, right)

    # 화면 맞춤은 모델만 보고 잡는다 (접지면 원판에 끌려가지 않게)
    flat = np.stack([tris @ right, tris @ upv], -1).reshape(-1, 2)
    mn, mx = flat.min(axis=0), flat.max(axis=0)

    if ground:
        gt, gn = _ground(tris, light)
        tris, vn = np.concatenate([tris, gt]), np.concatenate([vn, gn])
        cols = np.concatenate([cols, np.tile((0.87, 0.885, 0.9), (len(gt), 1))])

    w, h = size[0] * ss, size[1] * ss
    c, r = (mn + mx) / 2, (mx - mn).max() / 2 * 1.16
    sc = (h / 2) / r
    px = np.stack([(tris @ right - c[0]) * sc + w / 2,
                   h / 2 - (tris @ upv - c[1]) * sc], axis=-1)
    img = _raster(px, tris @ d, vn, cols, w, h,
                  matplotlib.colors.to_rgb(bg), lt)
    img = img.reshape(size[1], ss, size[0], ss, 3).mean(axis=(1, 3))   # 다운샘플

    fig = plt.figure(figsize=(size[0] / 100, size[1] / 100), dpi=100)
    ax = fig.add_axes([0, 0, 1, 1])
    fig.patch.set_facecolor(bg)
    ax.imshow(np.clip(img, 0, 1))
    ax.axis("off")
    if title:
        ax.text(0.5, 0.955, title, transform=ax.transAxes, ha="center", va="top",
                fontsize=16, color="#3a3f46",
                fontfamily=["Malgun Gothic", "DejaVu Sans"])
    out = Path(out)
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, facecolor=bg)
    plt.close(fig)
    print(f"  렌더 저장: {out}  (삼각형 {len(tris):,})")
    return out
