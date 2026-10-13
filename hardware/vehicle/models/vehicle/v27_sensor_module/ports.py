r"""뒤판 포트 칸 패널 부품 — 국내 판매처 실물 치수 모델 (v21).

조사 2026-10-05 (research/2026-10-05_ports_fans.md). 표기
  [판매처] 판매 페이지 도면 · 사양 숫자   [사진 비례] 사진으로 잰 값   [추정] 출처 없음 — 실물 받아 고친다

로컬 좌표 (모든 부품 같음): X = 패널 법선, **X=0 = 트레이 안벽의 앞면 (칸 쪽 면)**, 앞 (칸 · 사람 쪽) = +X,
패널 뒤 (차량 안) = -X. 벽 두께 WALL 을 지나 뒤에 너트 · 몸통이 온다. Y 폭, Z 높이.
각 부품은 (형상, 구멍) 을 돌려준다 — 구멍은 트레이 벽에서 뺄 솔리드 (X 방향으로 벽을 관통).
"""

from build123d import Box, BuildPart, Cylinder, Locations, Mode, Pos, RegularPolygon, BuildSketch, Plane, extrude

WALL = 2.5                      # 트레이 안벽 두께 (shell.BAY wall)
AX = dict(rotation=(0, 90, 0))  # 실린더 축 = X


def _cyl(d, x0, x1, y=0.0, z=0.0):
    """X 구간 [x0, x1] 원기둥."""
    return Pos((x0 + x1) / 2, y, z) * Cylinder(d / 2, abs(x1 - x0), **AX)


def _hex(af, x0, x1):
    """맞변 af 육각 너트, X 구간 [x0, x1]."""
    with BuildPart() as p:
        with BuildSketch(Plane.YZ.offset(min(x0, x1))):
            RegularPolygon(af / 2 / 0.8660254, 6, major_radius=True)
        extrude(amount=abs(x1 - x0))
    return p.part


def _dcyl(d, flat, x0, x1):
    """D 컷 원기둥 (지름 d, 맞은편까지 폭 flat — 평면은 +Z 쪽), X 구간 [x0, x1]. 회전 방지 나사부 · 구멍."""
    return _cyl(d, x0, x1) & Pos((x0 + x1) / 2, 0, (flat - d) / 2) * Box(abs(x1 - x0) + 0.02, d + 2, flat)


def _dcut_hole(d, flat):
    return _dcyl(d, flat, -WALL - 1, 1)


def gx16_socket():
    """충전 단자 — GX16 3P 패널 소켓 (알엠파워 GX16 3P set, 1,500).
    구멍 M16 × 1, Ø16 + D 컷 14.6, 앞 플랜지 Ø19 × 1.8 [판매처: 제노몰 HAC5012 도면],
    소켓 전장 20.1 · 패널 뒤 13.1 [판매처: 알엠파워 표기]. 너트 맞변 19 [추정].
    판넬 쪽이 수 (핀) — 배터리 쪽에 쓰면 42 V 핀이 노출된다 → 충전기 쪽 성별 확인 (DECISIONS D-10)."""
    front = 20.1 - 13.1
    s = _cyl(19.0, 0, 1.8) + _cyl(16.0, 1.8, front)                 # 플랜지 + 결합 나사 슬리브
    s = s - _cyl(12.0, front - 5.0, front + 0.01)                  # 플러그 받는 구멍
    s = s + _dcyl(15.8, 14.4, -WALL, 0) + _hex(19.0, -WALL, -WALL - 2.5) + _cyl(12.0, -WALL - 2.5, -13.1)
    return s, _dcut_hole(16.2, 14.8)


def key_has1911():
    """키 스위치 — 2 단 2 핀 메탈 키스위치 ON-OFF 19 mm HAS1911 (제노몰, 6,600).
    구멍 M19, Ø19 + D 컷 16.5, 앞 베젤 Ø21.8 × 6.5, 패널 뒤 33.8 (단자 끝), 몸통 Ø16.3,
    육각 너트 맞변 22, 4 A 28 VDC [판매처 도면]. 키는 ON · OFF 둘 다에서 빠진다 — 주행 중 키를 빼 둔다."""
    s = _cyl(21.8, 0, 6.5) - _cyl(8.0, 5.5, 6.51)                   # 베젤 + 키 홈
    s = s + _dcyl(18.8, 16.3, -WALL, 0) + _dcyl(18.8, 16.3, -WALL, -WALL - 12.0)  # 나사부 (D 컷)
    s = s + _hex(22.0, -WALL - 1.0, -WALL - 4.0)                    # 너트 (와셔 1 뒤)
    s = s + _cyl(16.3, -WALL - 12.0, -30.0)
    for dy in (-4.0, 4.0):                                          # 단자 2
        s = s + Pos(-31.9, dy, 0) * Box(3.8, 0.8, 4.8)
    return s, _dcut_hole(19.2, 16.7)


def voltmeter_round():
    """전압계 — VaneAims LED 원형 DC 4~100 V (FXB16-22DSV, 엘레파츠 6,160 VAT 포함).
    2 선 자체 전원 0.5 W. 앞 Ø29.5, 몸통 Ø20.5, 전장 47.6 [판매처], 구멍 Ø22 (모델명) · 앞 돌출 6 · 플라스틱 너트 [추정].
    컨택터 뒤 (바퀴 쪽) 에 물려 주행 중에만 켜진다 — 세워 둘 때 배터리를 안 먹는다."""
    front = 6.0
    s = _cyl(29.5, 0, front) - _cyl(18.0, front - 0.6, front + 0.01)  # 베젤 + 표시창 오목
    s = s + _cyl(21.8, -WALL, 0) + _cyl(27.0, -WALL, -WALL - 4.0)    # 나사부 + 너트
    s = s + _cyl(20.5, -WALL - 4.0, -(47.6 - front))
    return s, _cyl(22.2, -WALL - 1, 1)


def usbc_mt723():
    """ESP32 USB-C 연장 — MAXTEK MT723 USB 2.0 C 수↔암 판넬용 30 cm (컴퓨존 3,200).
    귀 나사 간격 17, 귀 폭 25, 암 하우징 26 (귀 아래 20) [판매처]. 귀 높이 10 · 두께 2 · 컷아웃 13 × 7 [추정].
    ESP32 보드 쪽 USB 모양에 맞춰 안쪽 끝은 수 C (보드가 마이크로 USB 면 변환 젠더)."""
    s = Pos(1.0, 0, 0) * Box(2.0, 25.0, 10.0)                       # 귀 (칸 쪽, 벽 앞면에 닿음)
    s = s - Pos(2.0, 0, 0) * Box(2.0, 9.0, 3.4)                     # USB-C 암 입구
    for dy in (-8.5, 8.5):
        s = s - _cyl(3.2, -1, 3, dy)
    s = s + Pos(-WALL - 9.0, 0, 0) * Box(18.0 + 0.01, 12.0, 6.5)    # 하우징 (벽 뒤)
    s = s + _cyl(3.8, -WALL - 18.0, -WALL - 48.0)                   # 케이블 토막 30
    hole = Pos(-WALL / 2, 0, 0) * Box(WALL + 2, 13.0, 7.0)
    for dy in (-8.5, 8.5):
        hole = hole + _cyl(3.2, -WALL - 1, 1, dy)
    return s, hole


def bulkhead_round(kind):
    """USB 3.0 / RJ45 원형 벌크헤드 (암-암) — Coms HA079 (USB 3.0, 7,000) · HA084 (RJ45 CAT6, 6,000).
    전장 33 [판매처], 앞 플랜지 Ø28 (MAXTEK 같은 계열 도면) · 나사 약 M22 [사진 비례] · 너트 Ø28 × 4 [추정].
    안쪽은 일반 케이블 (USB 3.0 A 수-수 / 랜 패치) 로 젯슨까지."""
    s = _cyl(28.0, 0, 3.0)
    if kind == "usb":
        s = s - Pos(3.0, 0, 0) * Box(4.0, 12.5, 5.0)                 # USB-A 입구
    else:
        s = s - Pos(3.0, 0, 0) * Box(4.0, 12.0, 10.5)                # RJ45 입구
    s = s + _cyl(21.8, -WALL, 0) + _cyl(28.0, -WALL, -WALL - 4.0) + _cyl(21.8, -WALL - 4.0, -30.0)
    return s, _cyl(22.4, -WALL - 1, 1)


# 칸 안 배치 (트레이 기준 y, z 오프셋) — 칸 안 195 × 55 에 두 줄. 바깥에서 볼 때 +y 가 왼쪽.
#   왼쪽: 전압계 (위) · USB-C (아래) | 가운데: USB-A · USB-A · RJ45 (한 줄) | 오른쪽: 키 (위) · 충전 GX16 (아래)
LAYOUT = [
    ("전압계_DC100V_원형", voltmeter_round, (75.0, 12.0)),
    ("ESP32_USB-C_MT723", usbc_mt723, (75.0, -14.0)),
    ("USB-A_1_HA079", lambda: bulkhead_round("usb"), (38.0, 0.0)),
    ("USB-A_2_HA079", lambda: bulkhead_round("usb"), (0.0, 0.0)),
    ("RJ45_HA084", lambda: bulkhead_round("rj45"), (-38.0, 0.0)),
    ("키스위치_HAS1911", key_has1911, (-75.0, 12.0)),
    ("충전단자_GX16_3P", gx16_socket, (-75.0, -14.0)),
]
