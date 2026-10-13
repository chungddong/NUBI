r"""BOM — 이 버전 형상에서 수량 · 치수를 뽑고, 구매 부품은 아래 표 (링크 · 가격) 를 붙인다 (v16 부터).

원칙
  · 수량 · 길이 · 판 크기 · 볼트 개수는 **형상에서** 센다 — 손으로 적지 않는다.
  · 가격은 **조사로 확인한 것만** 적는다. 모르면 None → "견적 · 확인 필요". 지어내지 않는다.
  · 출력물 (ASA) 은 부피에서 출력 시간을 어림한다 (0.4 노즐 · 유효 토출 10 mm³/s).

assembly.export() 가 부르면 bom.md · bom.csv 를 버전 폴더에 쓴다.
"""

import csv
import math

import parts as PT
import shell as SH
import tslot

PRINT_RATE = 10.0          # mm³/s 유효 토출 (기존 docs/bom.md 와 같은 기준)
ASA_PRICE_G = 30.0         # 원/g (ASA 1 kg ≈ 3 만원 어림) — 바꿀 수 있다
ASA_DENSITY = 1.07         # g/cm³

# ---------------------------------------------------------------- 구매 부품 (조사 근거가 있는 것만 가격)
# (구분, 품목, 규격/모델, 수량 함수 키 또는 숫자, 단가 원 (None=확인 필요), 링크, 비고)
PURCHASE = [
    # v20: 전원 · 구동 계통 확정 — 국내 구매처만 (DECISIONS.md D-03~D-08, research/2026-10-04_domestic_sourcing.md).
    # 단가 0 = 이미 가지고 있는 것 (보유품), None = 견적 · 확인 필요.
    ("구동", "호버보드 (6.5\" 허브모터 2 + 1 세대 메인보드 1)", "바퀴 모터 4 · 모터 드라이버 2 로 쓴다", 2, 0, "",
     "보유품. 메인보드 칩이 STM32F103RCT6 / GD32F103RCT6 인지 확인 (EFeru 펌웨어)"),
    ("구동", "보드 방열판", "알루미늄 3t 175 × 100, 네 귀 M5 · 보드 탭 M4 탭", 2, None, "",
     "PC 바닥판은 단열재 — 보드 판이 B+ · B- · 상선과 절연인지 멀티미터로 확인 후 고정"),
    ("구동", "보드 전원 버튼 릴레이 (또는 포토커플러)", "3.3 V 구동 소형 릴레이 모듈", 2, None, "",
     "ESP32 가 보드 전원 버튼을 누른다 (두 보드 켜기 · 끄기 동기)"),
    ("구동", "ST-Link V2 (펌웨어 굽기)", "STM32 SWD 프로그래머", 1, None, "", "EFeru 펌웨어 굽기 1 회"),
    ("전원", "배터리", "알엠파워 ZENION 삼성 21700 36V 20Ah 10S4P 직사형 + XT-90 (최대 방전 30 A, 최대 충전 10 A, KC)", 1, 553000,
     "https://rmpower.co.kr/shop_view/?idx=514",
     "548,000 + XT-90 옵션 5,000. 95 × 245 × 75, 3 kg. '최대 30 A' 가 연속인지 판매처 확인 (D-04)"),
    ("전원", "충전기", "알엠파워 10S 42V 2A (XT60 단자)", 1, 46000, "https://rmpower.co.kr/1102/?idx=104",
     "DC 단자형 36,000. 포트 칸 GX16 충전 단자로 배선"),
    ("전원", "메인 차단기", "DC 차단기 1P 250VDC 40A (BA49-63 DC, C40) + DIN 레일 100", 1, 7350,
     "https://mega-tec.co.kr/product/detail.html?product_no=504&cate_no=169&display_group=1",
     "7,000 + 레일 350. 극성 확인 (DC MCB 는 극성 지정형이 많다)"),
    ("전원", "비상정지 컨택터", "LS일렉트릭 GPR-M040 DC12V (40 A · 1000 VDC, 코일 12 V 3.5 W)", 1, 41870,
     "https://e-punglim.co.kr/product/detail.html?product_no=16656",
     "주문품 4~6 주. 주접점 극성 — + 단자를 배터리 쪽. 급하면 TE EV200AAANA 170,500 (파워크래프트)"),
    ("전원", "컨택터 코일 커넥터", "GPR-40ST (Tyco 1379658-1 하우징)", 10, 2510,
     "https://e-punglim.co.kr/product/detail.html?product_no=20822", "최소 10 개 단위"),
    ("전원", "코일 서지 보호", "TVS 또는 제너 + 다이오드 (12 V 코일)", 1, None, "",
     "일반 다이오드만 달면 접점이 늦게 떨어진다 (LS 카탈로그)"),
    ("전원", "DC-DC 12V (젯슨 · 코일)", "MEAN WELL RSD-60L-12 (입력 18~72 V → 12 V 5 A)", 1, 74800,
     "https://meanwell.co.kr/product/detail.html?product_no=799&cate_no=52&display_group=1",
     "싸인라이트. 민웰몰 65,990. 젯슨 DC 잭 9~20 V (D-07)"),
    ("전원", "DC-DC 5V", "디오시스템 DKP-55 (입력 8~60 V → 5 V 10 A)", 1, 82000,
     "https://smpskorea.com/product/detail.html?product_no=741&cate_no=126&display_group=1", ""),
    ("전원", "분기 퓨즈 + 홀더", "DC 58 V 이상 정격 5 A (12 V · 5 V 변환기 입력)", 2, None, "",
     "자동차용 32 V 블레이드 퓨즈는 안 된다 — 구매처 확인 필요"),
    ("제어", "메인 컴퓨터", "NVIDIA Jetson Orin Nano (Super) 개발킷", 1, None,
     "https://developer.nvidia.com/embedded/learn/get-started-jetson-orin-nano-devkit", "가격 확인 필요"),
    ("제어", "MCU", "ESP32-DevKitC V4 + 만능기판 받침", 1, None,
     "https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32/esp32-devkitc/user_guide.html",
     "하드웨어 UART 2 개로 두 보드 제어 (3.3 V)"),
    ("센서", "IMU", "ROBOR iAHRS RB-SDA-v1 (35 × 35 × 10, USB)", 1, 198000,
     "https://www.eleparts.co.kr/goods/view?no=9907195", "엘레파츠 VAT 포함 (VAT 별도 180,000). 지자기 보정 끔 (mv=0)"),
    ("안전", "비상정지 버튼", "Autonics SF2ER-E2RB (D40 짧은 헤드, 1NC, 누르면 잠김 · 시계방향 회전 해제)", 1, 23100,
     "https://www.eleparts.co.kr/goods/view?no=13038016", "엘레파츠 VAT 포함 (VAT 별도 21,000), 국내 정품 · 재고 시 2~3 일"),
    ("안전", "비상정지 명판", "Autonics SF2ER-B3 (Ø60, 두께 1, STOP)", 1, 5150,
     "https://e-punglim.co.kr/product/detail.html?product_no=37361", "풍림오토테크 (VAT 표기 없음)"),
    ("배선", "고무 그로밋", "구멍 Ø25 · 판 5 t · 케이블 Ø10", 4, None, "", "바닥판 케이블 구멍"),
    ("배선", "보드 전원 커넥터", "AMASS XT60H 수 + 암", 4, 638,
     "https://www.eleparts.co.kr/goods/view?no=13428798", "수 2 · 암 2 (엘레파츠 VAT 포함). 암 https://www.eleparts.co.kr/goods/view?no=13428797"),
    ("배선", "배터리 커넥터", "XT90 암 (배터리 XT-90 옵션과 짝)", 1, None, "", ""),
    ("외관", "고무 범퍼 띠", "D 형 고무 범퍼 / 고무 몰딩, 높이 30 × 두께 25", "rubber_m", None, "", "앞 · 뒤 판 폭대로 잘라 쓴다"),
    ("외관", "배기 팬", "EVERCOOL EC8025H05B (5 V, 80 × 80 × 25)", 2, 8740,
     "https://www.compuzone.co.kr/product/product_detail.htm?ProductNo=1047506",
     "컴퓨존. 대안 메카솔루션 FNM-S-8025 5V 1,540"),
]


def _profile_rows(groups):
    g = {d["key"]: d["parts"] for d in groups}
    cuts = {}
    for b in g["frame"]:
        L = round(max(tuple(b.bounding_box().size)), 1)
        cuts.setdefault(L, []).append(b.label.rstrip("0123456789"))
    rows = []
    for L, names in sorted(cuts.items(), reverse=True):
        n, cut = len(names), tslot.snap_cut(L)
        price = int(cut / tslot.CUT_UNIT) * tslot.UNIT_PRICE
        rows.append(("프레임", "알루미늄 프로파일 3030", f"L{L:g}", n, price,
                     "", f"100 mm 단위 올림 과금 ({cut:g}) · {', '.join(sorted(set(names)))}"))
    return rows


def _fastener_rows(groups, A):
    g = {d["key"]: d["parts"] for d in groups}
    sh = SH.Shell(A)
    m5 = sum(sum(1 for h in sh.features(k)[2] if h[2] == SH.HOLE) for k in ("side_l", "side_r", "front", "rear"))
    m6_rubber = sum(sum(1 for h in sh.features(k)[2] if h[2] == SH.RUBBER["hole"]) for k in ("front", "rear"))
    rows = [
        ("체결", "둥근머리 렌치볼트", "M6x15", len(g["bolt"]), None, "", "T 클램프 → 레일 · 축크로스 (스프링너트)"),
        ("체결", "둥근머리 렌치볼트 + 와셔", "M6x15", len(g["floor_bolt"]), None, "",
         "바닥판 → 프레임 (스프링너트)"),
        ("체결", "스프링너트 3030", "M6", len(g["bolt"]) + len(g["floor_bolt"]) + m6_rubber, None, "", ""),
        ("체결", "외관판 볼트 + T 너트", "M5", m5, None, "", "옆 · 앞 · 뒤판 → 프로파일 바깥 슬롯 (위쪽은 상부 프레임 뒤)"),
        ("체결", "고무 범퍼 볼트", "M6 (길이는 고무 두께 + 판 + 슬롯)", m6_rubber, None, "", ""),
    ]
    sizes = {}
    for x, y, d, bolt, name in PT.plate_holes():
        sizes.setdefault(bolt.split()[0], 0)
        sizes[bolt.split()[0]] += 1
    for k, n in sorted(sizes.items()):
        rows.append(("체결", "부품 고정 볼트 · 너트 (+ 스탠드오프)", k, n, None, "", "바닥판 부품 고정 구멍에서 셈"))
    return rows


def _sheet_rows(A):
    import floor as FL
    sh = SH.Shell(A)
    rows = [("판재", f"바닥판 {FL.MAT_NAME}", f"{A.BASE_L:.0f} × {A.BASE_W:.0f} × {FL.PLATE_T:g}t", 1, None,
             "https://www.acrylmall.com/src/products/products_list.php?product_category_id=5512",
             "CNC 재단 · 구멍 — output/floor_plate.dxf")]
    for kind, mat in (("side_l", "ACM 3t"), ("side_r", "ACM 3t"), ("front", "UV PC 3t 불투명"), ("rear", "UV PC 3t 불투명")):
        U, V, *_ = sh.features(kind)
        rows.append(("판재", f"외관판 {kind}", f"{mat} {U:.0f} × {V:.0f}", 1, None, "",
                     f"output/shell_{kind}.dxf"))
    return rows


def _printed_rows(groups):
    g = {d["key"]: d["parts"] for d in groups}
    rows = []
    for p in g.get("shell_asa", []) + g.get("cam_bracket", []):
        cm3 = p.volume / 1000
        grams = cm3 * ASA_DENSITY
        hours = p.volume / PRINT_RATE / 3600
        rows.append(("출력", p.label, f"ASA {cm3:.0f} cm³ · {grams:.0f} g · 약 {hours:.1f} h", 1,
                     int(grams * ASA_PRICE_G), "", "부피 꽉 채움 기준 상한 — 실제 인필이면 더 적다"))
    return rows


def rows(groups, A, extra=()):
    out = _profile_rows(groups) + _sheet_rows(A) + _printed_rows(groups) + _fastener_rows(groups, A)
    for cat, name, spec, qty, price, link, note in list(PURCHASE) + list(extra):
        if qty == "rubber_m":
            qty = round(2 * (A.BASE_W + 2 * SH.ACM_T) / 1000, 2)
        out.append((cat, name, spec, qty, price, link, note))
    return out


def write(groups, A, md, csv_path, version, extra=()):
    R = rows(groups, A, extra)
    known = sum(q * p for _, _, _, q, p, _, _ in R if p)
    unknown = [n for _, n, _, _, p, _, _ in R if p is None]     # v20: 0 = 보유품
    with open(csv_path, "w", encoding="utf-8-sig", newline="") as f:
        w = csv.writer(f)
        w.writerow(["구분", "품목", "규격", "수량", "단가(원)", "금액(원)", "링크", "비고"])
        for cat, name, spec, q, p, link, note in R:
            w.writerow([cat, name, spec, q, "" if p is None else p, "" if p is None else q * p, link, note])
    L = [f"# BOM — {version}", "",
         "수량 · 길이 · 판 크기 · 볼트 개수는 이 버전 형상에서 뽑았다. 가격은 조사로 확인한 것만 적었다.", "",
         f"- 확인된 금액 합: **{known:,.0f} 원** (단가가 있는 줄만)",
         f"- 단가 미확인 {len(unknown)} 줄 — 견적 · 확인 필요", "",
         "| 구분 | 품목 | 규격 | 수량 | 단가 | 금액 | 구매처 | 비고 |", "|---|---|---|---:|---:|---:|---|---|"]
    for cat, name, spec, q, p, link, note in R:
        lk = f"[링크]({link})" if link else ""
        L.append(f"| {cat} | {name} | {spec} | {q:g} | {f'{p:,}' if p else '확인 필요'} | "
                 f"{f'{q * p:,.0f}' if p else ''} | {lk} | {note} |")
    L[-len(R):] = [l.replace("| 확인 필요 |", "| 보유 |") if "| 보유품" in l or "보유품." in l else l for l in L[-len(R):]]
    md.write_text("\n".join(L) + "\n", encoding="utf-8", newline="\n")
    return known, len(unknown)
