# T슬롯 알루미늄 프로파일 라이브러리
#
# 단면을 스케치로 그려 길이만큼 압출한 것이 곧 실제 압출재다.
# 프레임 모델에서 `from tslot import bar` 로 가져다 쓴다 (models/ 가 스크립트 경로라 그냥 import 된다).
#
# 주의: 단면 치수는 제조사 공통 대표값이다. 슬롯 규격(입구/방 크기)은 실물과 맞아
#       T너트가 그대로 들어가지만, 내부 리브를 단순화해 단면적은 실물보다 10~15%
#       크게 나온다(2020 실물 약 163 mm²). 특정 브랜드 도면에 맞추려면 PROFILES 만 고친다.

import math

from build123d import *

# 시리즈별 단면 치수 (단위: mm)
#   size=한 변, wall=바깥벽 두께(=슬롯 리브 두께), slot=슬롯 입구 폭,
#   chamber=슬롯 안쪽 폭(T너트가 들어가는 방), boss=중심 보스 지름,
#   bore=중심 구멍 지름, rib=대각 리브 두께, cham=외곽 모따기 R
PROFILES = {
    20: dict(size=20, wall=1.9, slot=6.2, chamber=11.0, boss=8.0, bore=4.2, rib=2.0, cham=1.0),
    30: dict(size=30, wall=2.4, slot=8.2, chamber=16.5, boss=11.0, bore=6.8, rib=2.4, cham=1.5),
    40: dict(size=40, wall=3.0, slot=8.2, chamber=16.5, boss=16.0, bore=10.2, rib=3.0, cham=2.0),
}

ALU_DENSITY = 2.70e-3          # g/mm³ — 중량 계산용

# 빔이 뻗는 방향 (원점 기준 +Z 로 만든 빔을 돌린다)
DIRS = {"X": Rot(0, 90, 0), "Y": Rot(-90, 0, 0), "Z": Rot(0, 0, 0)}


def _outer_square(P):
    """모서리를 둥글린 외곽 정사각 — 바깥벽의 원형이자 최종 트리밍 마스크."""
    with BuildSketch() as m:
        Rectangle(P["size"], P["size"])
        fillet(m.vertices(), radius=P["cham"])
    return m.sketch


def section(series):
    """T슬롯 단면 (원점 중심, XY 평면).

    슬롯을 통째로 파내면 코너 기둥이 중심에서 끊어져 솔리드가 조각난다.
    그래서 실제 압출재처럼 구성 요소를 쌓아 올린다:
    바깥벽(튜브) + 코너 기둥 4 + 중심 보스 + 45° 대각 리브 4.
    슬롯 방은 그 사이에 저절로 생기고, 입구만 바깥벽에서 잘라내면 T 홈이 된다.
    """
    P = PROFILES[series]
    S = P["size"]
    post = (S - P["chamber"]) / 2            # 코너 기둥 한 변 (슬롯 방 폭에서 역산)
    mask = _outer_square(P)
    so, y_lip = P["slot"] / 2, S / 2 - P["wall"]
    with BuildSketch() as sk:
        add(mask)
        Rectangle(S - 2 * P["wall"], S - 2 * P["wall"], mode=Mode.SUBTRACT)   # 바깥벽만 남김
        with Locations(*[(sx * (S - post) / 2, sy * (S - post) / 2)
                         for sx in (-1, 1) for sy in (-1, 1)]):
            Rectangle(post, post)                        # 코너 기둥
        Circle(P["boss"] / 2)                            # 중심 보스
        with Locations(Rot(0, 0, 45), Rot(0, 0, -45)):
            Rectangle(P["rib"], 2 * S)                   # 대각 리브 (마스크로 잘림)
        with Locations(*[Rot(0, 0, a) for a in (0, 90, 180, 270)]):
            Polygon((so, S / 2 + 1), (so, y_lip), (-so, y_lip), (-so, S / 2 + 1),
                    align=None, mode=Mode.SUBTRACT)      # 슬롯 입구
        Circle(P["bore"] / 2, mode=Mode.SUBTRACT)        # 중심 탭 구멍
        add(mask, mode=Mode.INTERSECT)                   # 둥근 모서리 밖으로 삐져나온 것 정리
    return sk.sketch


_BEAMS = {}                 # (series, length) → 토막. v03 부터: 같은 길이는 한 번만 짓는다


def beam(series, length):
    """원점에서 +Z 로 length 만큼 뻗는 압출재 한 토막 (같은 길이는 캐시 — 짓기 0.6 s/본)."""
    key = (series, round(length, 3))
    if key not in _BEAMS:
        with BuildPart() as p:
            with BuildSketch():
                add(section(series))
            extrude(amount=length)
        _BEAMS[key] = p.part
    return _BEAMS[key]


def bar(series, length, start, axis="X"):
    """start(단면 중심)에서 axis(+X/+Y/+Z) 방향으로 length 만큼 뻗는 압출재.

    캐시된 토막을 옮긴 사본이라 같은 길이 부재는 형상을 공유한다 (메시도 한 번만 뜬다)."""
    return Pos(*start) * DIRS[axis] * beam(series, length)


# 국내 프로파일 판매 단위 — **100mm 단위로 올림 과금**이다.
# (770mm 주문 = 100mm 8개 = 800mm 값. 남는 30mm 는 그냥 버리는 값이다.)
# 그래서 설계 길이를 100 배수 근처로 맞추는 게 곧 원가 절감이다.
CUT_UNIT = 100
UNIT_PRICE = 950          # 원 / 100mm (3030, 2026-08 실주문 확인)


def snap_cut(length, unit=CUT_UNIT):
    """절단 길이를 판매 단위로 올린 값 (= 실제로 돈 내는 길이)."""
    return math.ceil(length / unit - 1e-9) * unit


def cut_list(items, series, unit=CUT_UNIT, price=UNIT_PRICE):
    """(이름, 길이) 목록을 길이별로 묶어 출력하고 총 길이·중량을 돌려준다.

    판매 단위(100mm) 올림 수량과 버리는 길이를 같이 낸다 — 설계 길이를 100 배수에
    붙이면 그만큼 그대로 돈이 준다.
    """
    area = section(series).area
    groups = {}
    for name, length in items:
        groups.setdefault(round(length, 1), []).append(name)
    total = billed = 0.0
    units = 0
    # 주문서 그대로 쓸 수 있게 낸다 — 판매처 수량 칸은 **개수가 아니라 단위 개수**다
    # (770mm 1개 = 수량 8, 770mm 5개 = 수량 40).
    print(f"절단 목록 / 주문서 ({series}x{series}, 단면적 {area:.0f} mm², "
          f"{unit}mm 단위 올림 과금):")
    print(f"    구매치수  필요  주문수량   버림      부재")
    for length, names in sorted(groups.items(), reverse=True):
        kinds = sorted({n.rstrip('0123456789') for n in names})
        n, cut = len(names), snap_cut(length, unit)
        u = int(cut / unit) * n
        print(f"   {length:7.1f}  {n:3d}개  {u:6d}   {(cut-length)*n:5.1f} mm   "
              f"{', '.join(kinds)}")
        total += length * n
        billed += cut * n
        units += u
    mass = total * area * ALU_DENSITY / 1000
    print(f"  합계 실길이 {total/1000:.2f} m / 과금 {billed/1000:.2f} m "
          f"({units}단위 = {units*price:,}원) / 버림 {billed-total:.0f} mm "
          f"({(billed-total)/billed*100:.0f}%)")
    print(f"  알루미늄 약 {mass:.2f} kg")
    return total, mass
