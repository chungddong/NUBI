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
    ("전원", "배터리", "볼트온 36V 20Ah (LG 셀, BMS, KC)", 1, 480000,
     "https://www.voltonbattery.com/pedelec_36v/?idx=88", "가격은 검색 시점 판매가 — 주문 전 확인"),
    ("전원", "충전기", "42V 2A 10S 리튬이온", 1, None,
     "https://www.aliexpress.us/item/3256806176154920.html", "배터리 판매처 전용 충전기 권장"),
    ("구동", "모터 드라이버", "ZLTECH ZLAC8015D (2 채널, 24–48V, RS485/CAN)", 2, None,
     "https://smotion.co.kr/product/zltech-zlac8015d-canopenrs-485-%EC%A0%9C%EC%96%B4%EC%9A%A9-2%EC%B6%95-wheel-in-motor%EC%9D%B8%ED%9C%A0%EB%AA%A8%ED%84%B0%EC%9A%A9-%EB%93%9C%EB%9D%BC%EC%9D%B4%EB%B2%84/206/",
     "국내 에스모션 (가격 문의). 해외 Alibaba 1–49 개 USD 95–115"),
    ("전원", "메인 퓨즈 + 홀더", "Victron MIDI 40A/58V + MIDI 홀더 CIP000050001", 1, None,
     "https://www.amazon.com/Victron-Energy-MIDI-Fuse-Products/dp/B0DK7S19CX", "58V 정격 — 자동차용 32V 퓨즈는 안 된다"),
    ("전원", "모터 전원 차단 SSR", "Fotek SSR-40DD (출력 5–60VDC 40A)", 1, None,
     "https://www.fotek.com.tw/en-gb/product-category/144", "비상정지 · 키 스위치가 끊는 자리"),
    ("전원", "DC-DC 19V", "30–60V → 19V 5A 이상 (Fidida 등)", 1, None,
     "https://www.amazon.com/Fidida-Converter-30V-60V-Step-Down-Waterproof/dp/B0CDH538MG", "입력 상한 42V 이상 확인"),
    ("전원", "DC-DC 5V", "11–90V → 5V 6A (DROK 200656)", 1, None,
     "https://www.droking.com/", "입력 60V 이상"),
    ("제어", "메인 컴퓨터", "NVIDIA Jetson Orin Nano (Super) 개발킷", 1, None,
     "https://developer.nvidia.com/embedded/learn/get-started-jetson-orin-nano-devkit", "가격 확인 필요"),
    ("제어", "MCU", "ESP32-DevKitC V4 + 만능기판 받침", 1, None,
     "https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32/esp32-devkitc/user_guide.html", ""),
    ("센서", "IMU", "Adafruit BNO085 #4754", 1, None,
     "https://www.adafruit.com/product/4754", "국내 디바이스마트에도 있음 (접속 불가 상태)"),
    ("안전", "비상정지 버튼", "Koino KSEP-2240 (22 파이, 버섯형)", 1, None,
     "https://www.devicemart.co.kr/goods/view?no=15008357", ""),
    ("배선", "케이블 글랜드", "M20 방수 (케이블 Ø6–12)", 4, None, "", "바닥판 글랜드 구멍 Ø20.5"),
    ("배선", "충전 · 전원 커넥터", "Amass XT60H 암수", 2, None,
     "https://www.devicemart.co.kr/goods/view?no=1078005", ""),
    ("외관", "고무 범퍼 띠", "D 형 고무 범퍼 / 고무 몰딩, 높이 30 × 두께 25", "rubber_m", None, "", "앞 · 뒤 판 폭대로 잘라 쓴다"),
    ("외관", "배기 팬", "80 mm DC 팬 (필터 · 빗물막이 포함 권장)", 2, None, "", ""),
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
        ("체결", "둥근머리 렌치볼트 + 와셔 + EPDM 와셔", "M6x15", len(g["floor_bolt"]), None, "",
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
    unknown = [n for _, n, _, _, p, _, _ in R if not p]
    with open(csv_path, "w", encoding="utf-8-sig", newline="") as f:
        w = csv.writer(f)
        w.writerow(["구분", "품목", "규격", "수량", "단가(원)", "금액(원)", "링크", "비고"])
        for cat, name, spec, q, p, link, note in R:
            w.writerow([cat, name, spec, q, p or "", (q * p if p else ""), link, note])
    L = [f"# BOM — {version}", "",
         "수량 · 길이 · 판 크기 · 볼트 개수는 이 버전 형상에서 뽑았다. 가격은 조사로 확인한 것만 적었다.", "",
         f"- 확인된 금액 합: **{known:,.0f} 원** (단가가 있는 줄만)",
         f"- 단가 미확인 {len(unknown)} 줄 — 견적 · 확인 필요", "",
         "| 구분 | 품목 | 규격 | 수량 | 단가 | 금액 | 구매처 | 비고 |", "|---|---|---|---:|---:|---:|---|---|"]
    for cat, name, spec, q, p, link, note in R:
        lk = f"[링크]({link})" if link else ""
        L.append(f"| {cat} | {name} | {spec} | {q:g} | {f'{p:,}' if p else '확인 필요'} | "
                 f"{f'{q * p:,.0f}' if p else ''} | {lk} | {note} |")
    md.write_text("\n".join(L) + "\n", encoding="utf-8", newline="\n")
    return known, len(unknown)
