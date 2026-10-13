"""오프라인 ISO 렌더러 — 뷰어(ocp_vscode) 없이 PNG 로 형상을 확인한다.

직교투영 + Z버퍼 + 평면 셰이딩. 외부 렌더러 없이 numpy/PIL 만 쓴다.
깊이 불연속을 잡아 실루엣 선을 얹기 때문에 CAD 선도처럼 읽힌다.

    from render import render, tile
    render([(part, "#c0392b"), ...], "out.png")
"""

import numpy as np
from PIL import Image

DIR_ISO = (1.0, -1.0, 0.62)     # 카메라가 있는 방향 (물체 → 카메라)
DIR_FRONT = (1.0, 0.0, 0.08)
DIR_TOP = (0.0, -0.001, 1.0)


def _rgb(c):
    c = c.lstrip("#")
    return np.array([int(c[i:i + 2], 16) for i in (0, 2, 4)], float)


def _basis(direction, up):
    """direction = 카메라가 있는 방향. f 는 시선(카메라 → 물체)이라 부호가 반대다."""
    f = -np.array(direction, float)
    f /= np.linalg.norm(f)
    r = np.cross(f, np.array(up, float))
    if np.linalg.norm(r) < 1e-6:                 # 시선이 up 과 나란하면 다른 축을 쓴다
        r = np.cross(f, np.array([0.0, 1.0, 0.0]))
    r /= np.linalg.norm(r)
    return r, np.cross(r, f), f


def render(items, path, direction=DIR_ISO, up=(0, 0, 1), size=900, ss=2,
           tol=0.55, ang=0.3, bg="#f4f5f6", margin=0.05,
           light=(0.45, -0.75, 0.85), edge=0.55, bounds=None):
    """items: [(shape, "#rrggbb"), ...] 를 직교투영해 PNG 로 저장한다.

    bounds: (center3, span) 을 주면 여러 장의 스케일을 맞출 수 있다.
    """
    px = size * ss
    r, u, f = _basis(direction, up)
    L = np.array(light, float)
    L /= np.linalg.norm(L)

    tris = []                                    # (2D 좌표, 깊이, 색)
    allpts = []
    for it in items:
        # (솔리드, 색) 또는 (솔리드, 색, 그 솔리드만의 tol) — 잔 형상이 많은 부품은
        # 따로 거칠게 깎는다. 실측 서보 STEP 한 개가 삼각형의 90% 를 먹는다.
        shape, color, itol = (it if len(it) == 3 else (it[0], it[1], tol))
        verts, faces = shape.tessellate(itol, ang)
        if not faces:
            continue
        V = np.array([(v.X, v.Y, v.Z) for v in verts], float)
        T = np.array(faces, int)
        a, b, c = V[T[:, 0]], V[T[:, 1]], V[T[:, 2]]
        n = np.cross(b - a, c - a)
        ln = np.linalg.norm(n, axis=1)
        keep = ln > 1e-12
        a, b, c, n, ln = a[keep], b[keep], c[keep], n[keep], ln[keep]
        n /= ln[:, None]
        # tessellate 의 삼각형 감김 방향을 믿지 않는다. 법선을 카메라 쪽으로 뒤집고
        # 가림은 Z버퍼에 맡긴다 (뒷면 컬링을 하면 감김이 반대일 때 전부 어두워진다).
        n *= np.where(n @ f > 0, -1.0, 1.0)[:, None]
        shade = 0.22 + 0.78 * np.clip(n @ L, 0, 1)
        tris.append((a, b, c, shade[:, None] * _rgb(color)[None, :]))
        allpts.append(np.vstack([a, b, c]))

    if not tris:
        raise ValueError("그릴 게 없다")
    P = np.vstack(allpts)
    if bounds is None:
        s2 = np.stack([P @ r, P @ u], 1)
        lo, hi = s2.min(0), s2.max(0)
        ctr, span = (lo + hi) / 2, (hi - lo).max()
    else:
        c3, span = bounds
        c3 = np.array(c3, float)
        ctr = np.array([c3 @ r, c3 @ u])
    scale = px * (1 - 2 * margin) / span

    zbuf = np.full((px, px), np.inf)
    img = np.tile(_rgb(bg), (px, px, 1))

    for a, b, c, col in tris:
        xy = [np.stack([(p @ r - ctr[0]) * scale + px / 2,
                        px / 2 - (p @ u - ctr[1]) * scale], 1) for p in (a, b, c)]
        zs = [p @ f for p in (a, b, c)]
        for i in range(len(a)):
            (ax, ay), (bx, by), (cx, cy) = xy[0][i], xy[1][i], xy[2][i]
            x0 = max(int(min(ax, bx, cx)), 0)
            x1 = min(int(max(ax, bx, cx)) + 1, px - 1)
            y0 = max(int(min(ay, by, cy)), 0)
            y1 = min(int(max(ay, by, cy)) + 1, px - 1)
            if x1 < x0 or y1 < y0:
                continue
            den = (by - cy) * (ax - cx) + (cx - bx) * (ay - cy)
            if abs(den) < 1e-9:
                continue
            X, Y = np.meshgrid(np.arange(x0, x1 + 1) + 0.5,
                               np.arange(y0, y1 + 1) + 0.5)
            l1 = ((by - cy) * (X - cx) + (cx - bx) * (Y - cy)) / den
            l2 = ((cy - ay) * (X - cx) + (ax - cx) * (Y - cy)) / den
            l3 = 1.0 - l1 - l2
            m = (l1 >= -1e-9) & (l2 >= -1e-9) & (l3 >= -1e-9)
            if not m.any():
                continue
            z = l1 * zs[0][i] + l2 * zs[1][i] + l3 * zs[2][i]
            sub = zbuf[y0:y1 + 1, x0:x1 + 1]
            m &= z < sub
            if not m.any():
                continue
            sub[m] = z[m]
            img[y0:y1 + 1, x0:x1 + 1][m] = col[i]

    if edge > 0:                                  # 깊이 불연속 → 실루엣/모서리 선
        z = np.where(np.isinf(zbuf), np.nan, zbuf)
        d = np.zeros_like(zbuf)
        for sx, sy in ((1, 0), (0, 1)):
            sh = np.roll(z, sx, 1) if sx else np.roll(z, sy, 0)
            d = np.maximum(d, np.nan_to_num(np.abs(z - sh), nan=0.0))
        thr = span * 0.004
        img[d > thr] *= 1.0 - edge
        img[np.isinf(zbuf) & (np.roll(d, 1, 1) > thr)] = _rgb(bg)   # 배경은 되돌린다

    out = Image.fromarray(img.clip(0, 255).astype(np.uint8))
    if ss > 1:
        out = out.resize((size, size), Image.LANCZOS)
    out.save(path)
    return path


def gif(paths, out, duration=80, boomerang=False, colors=96):
    """PNG 프레임들을 GIF 로 묶는다. boomerang 이면 갔다가 되돌아온다."""
    ims = [Image.open(p).convert("RGB") for p in paths]
    if boomerang and len(ims) > 2:
        ims = ims + ims[-2:0:-1]
    ims = [im.convert("P", palette=Image.ADAPTIVE, colors=colors) for im in ims]
    ims[0].save(out, save_all=True, append_images=ims[1:], duration=duration,
                loop=0, optimize=True, disposal=2)
    return out


def tile(paths, out, cols=None, labels=None, gap=10, bg=(244, 245, 246)):
    """여러 PNG 를 한 장으로 붙인다 (시퀀스 스트립용)."""
    ims = [Image.open(p) for p in paths]
    cols = cols or len(ims)
    rows = (len(ims) + cols - 1) // cols
    w, h = ims[0].size
    canvas = Image.new("RGB", (cols * w + (cols + 1) * gap,
                               rows * h + (rows + 1) * gap), bg)
    for i, im in enumerate(ims):
        canvas.paste(im, (gap + (i % cols) * (w + gap), gap + (i // cols) * (h + gap)))
    if labels:
        from PIL import ImageDraw, ImageFont
        font = None
        for f in (r"C:\Windows\Fonts\malgun.ttf", r"C:\Windows\Fonts\arial.ttf"):
            try:                                   # 기본 폰트는 한글이 네모로 나온다
                font = ImageFont.truetype(f, max(14, h // 26))
                break
            except OSError:
                pass
        d = ImageDraw.Draw(canvas)
        for i, t in enumerate(labels):
            d.text((gap + (i % cols) * (w + gap) + 14,
                    gap + (i // cols) * (h + gap) + 10), t, fill=(40, 44, 48), font=font)
    canvas.save(out)
    return out
