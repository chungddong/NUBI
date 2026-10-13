r"""부품 형상 — 실측 · 데이터시트 치수로 짓는다 (v08).

모든 빌더는 **밑면 z=0, 몸통 중심 x=y=0** 으로 짓는다. 길이 L 은 x, 폭 W 는 y.
배치 (어느 자리 · 어느 방향) 는 parts.py 가 정한다.

세부 형상은 부품을 알아볼 수 있고 간섭 · 배선 자리를 볼 수 있을 만큼만 넣는다
(나사산 · 글자 · 작은 칩은 생략).
"""

from build123d import (Align, Axis, Box, BuildPart, Compound, Cylinder, Locations, Mode,
                       Pos, Rot, fillet)

MIN = (Align.CENTER, Align.CENTER, Align.MIN)


def _box(l, w, h, x=0.0, y=0.0, z=0.0):
    return Pos(x, y, z) * Box(l, w, h, align=MIN)


def _cyl(r, h, x=0.0, y=0.0, z=0.0):
    return Pos(x, y, z) * Cylinder(r, h, align=MIN)


def _comp(parts, label):
    c = Compound(children=list(parts))
    c.label = label
    return c


# ---------------------------------------------------------------- 기판류
def pcb(L, W, t=1.6, holes=(), hole_d=3.2, standoff=0.0, items=()):
    """기판 + (선택) 스탠드오프 + 윗면 부품 상자.

    holes: 기판 중심 기준 구멍 (x, y). standoff > 0 이면 구멍마다 Ø6 육각 대신 원기둥 스탠드오프.
    items: (x, y, l, w, h) — 기판 윗면 위 상자 (커넥터 · 모듈 · 방열판).
    """
    out = []
    with BuildPart() as b:
        _box_part = Box(L, W, t, align=MIN)
        if holes:
            with Locations(*holes):
                Cylinder(hole_d / 2, t, align=MIN, mode=Mode.SUBTRACT)
    board = Pos(0, 0, standoff) * b.part
    out.append(board)
    for (x, y) in holes if standoff > 0 else ():
        out.append(_cyl(3.0, standoff, x, y))
    for (x, y, l, w, h) in items:
        out.append(_box(l, w, h, x, y, standoff + t))
    return out


# ---------------------------------------------------------------- 상자류
def finned_box(L, W, base_h, fin_h=0.0, fin_n=0, fin_t=2.0, fin_axis="x",
               ears=None, items=()):
    """방열 핀이 있는 알루미늄 케이스 (모터 드라이버 · DC-DC).

    ears: dict(l, w, t, holes=[(x, y, d)]) — 몸통 양 끝 (x 방향) 으로 나온 고정 귀. 구멍은 몸통 중심 기준.
    items: (x, y, z, l, w, h) — 단자대 · 커넥터 상자.
    """
    out = [_box(L, W, base_h)]
    if fin_n and fin_h:
        span = W if fin_axis == "x" else L
        pitch = span / fin_n
        for i in range(fin_n):
            c = -span / 2 + pitch * (i + 0.5)
            if fin_axis == "x":
                out.append(_box(L, fin_t, fin_h, 0, c, base_h))
            else:
                out.append(_box(fin_t, W, fin_h, c, 0, base_h))
    if ears:
        with BuildPart() as e:
            for sx in (-1, 1):
                with Locations((sx * (L / 2 + ears["l"] / 2), 0, 0)):
                    Box(ears["l"] + 0.01, ears["w"], ears["t"], align=MIN)
            for (x, y, d) in ears.get("holes", ()):
                with Locations((x, y, 0)):
                    Cylinder(d / 2, ears["t"], align=MIN, mode=Mode.SUBTRACT)
        out.append(e.part)
    for (x, y, z, l, w, h) in items:
        out.append(_box(l, w, h, x, y, z))
    return out


def battery_pack(L, W, H, r=8.0, lead_side=+1, lead_d=8.0, lead_len=40.0):
    """수축 튜브로 감싼 셀 팩 — 모서리 둥글림 + 출력선 2 가닥 (lead_side 쪽 x 끝)."""
    with BuildPart() as b:
        Box(L, W, H, align=MIN)
        try:
            fillet(b.edges().filter_by(Axis.X), radius=min(r, H / 2 - 0.5, W / 2 - 0.5))
        except Exception:                         # noqa: BLE001
            pass
    out = [b.part]
    for dy in (-W / 6, W / 6):
        out.append(Pos(lead_side * (L / 2), dy, H * 0.6) * Rot(0, lead_side * 90, 0)
                   * Cylinder(lead_d / 2, lead_len, align=MIN))
    return out


def fuse_holder(L, W, H, stud_d, stud_h, stud_pitch, holes=(), hole_d=5.5):
    """볼트 고정식 퓨즈 홀더 — 몸통 + 단자 스터드 2 + 고정 구멍."""
    with BuildPart() as b:
        Box(L, W, H, align=MIN)
        for (x, y) in holes:
            with Locations((x, y, 0)):
                Cylinder(hole_d / 2, H, align=MIN, mode=Mode.SUBTRACT)
    out = [b.part]
    for sx in (-1, 1):
        out.append(_cyl(stud_d / 2, stud_h, sx * stud_pitch / 2, 0, H))
    return out


def ssr(L, W, H, ear_holes=(), hole_d=4.5, terminals=()):
    """무접점 릴레이 — 몸통 (밑판 귀 포함) + 나사 단자 4."""
    with BuildPart() as b:
        Box(L, W, H, align=MIN)
        for (x, y) in ear_holes:
            with Locations((x, y, 0)):
                Cylinder(hole_d / 2, H, align=MIN, mode=Mode.SUBTRACT)
    out = [b.part]
    for (x, y) in terminals:
        out.append(_cyl(4.0, 3.0, x, y, H))
    return out


def label(parts, name):
    """조각들을 **하나의 솔리드로 합친다** (fuse).

    Compound(children=…) 로 묶어 두면 build123d 0.11 에서 `a & b` (겹침) 가 엉뚱한 값을 낸다 —
    떨어져 있는 배터리 & 드라이버가 드라이버 부피 전체 (426 cm³) 로 나왔다. 합쳐 두면 v07 의
    단순 상자처럼 정확하다."""
    out = parts[0]
    for p in parts[1:]:
        out = out + p
    out.label = name
    return out
