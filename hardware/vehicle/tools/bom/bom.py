r"""차량 전체 버전 BOM / 예상 견적.

    .venv/Scripts/python.exe tools/bom/bom.py

무엇이 정확하고 무엇이 추정인지 **줄 단위로 구분한다.**

  수량·부피·질량   전부 모델 형상에서 뽑는다 (정확). 형상을 고치면 따라온다.
  프로파일 절단     tslot.cut_list 와 같은 100mm 올림 과금 규칙 (정확).
  단가             아래 PRICE 표. `확인` 태그만 실주문으로 확인된 값이고
                   나머지 `추정` 은 **국내 시장가 눈대중**이다. 견적서로 쓸 게
                   아니라 어디에 돈이 몰리는지 보려고 넣었다.

조달(SRC) 세 가지를 구분한다 — 총액이 두 개 나온다.
  보유   이미 가지고 있다 (프레임·허브모터·기존 클램프·M6 철물·남는 390 프로파일)
  출력   3D 프린터. 필라멘트 값 + 출력 시간만 든다
  구매   새로 사야 한다

만드는 것:
  docs/bom.md    표 + 카테고리 합계 + 가정 목록
  docs/bom.csv   엑셀용
"""

import csv
import math
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT / "models" / "hoverboard_mount"), str(ROOT / "models")]

# vehicle_shell_integration 과 **같은 설정**으로 짓는다. 여기서 갈리면 뷰어에
# 보이는 것과 견적이 달라진다.
os.environ.update(
    DRIVE="skid", MOUNT="low", PAYLOAD="cargo",
    TOP_RAIL_Y="210", SHELL_Y="255", SHELL_WHEEL_Y="263.7",
    WHEEL_ARCH_CLR="12", HUB_MOCK="1", NOSHOW="1", NOEXPORT="1",
)

import swerve_base as V                                       # noqa: E402
import low_clamp as LC                                        # noqa: E402
import parameters as P                                        # noqa: E402
from tslot import ALU_DENSITY, section, snap_cut              # noqa: E402

OUT_MD = ROOT / "docs" / "bom.md"
OUT_CSV = ROOT / "docs" / "bom.csv"

# ---- 단가표 (원). 태그: 확인 = 실주문 확인 / 추정 = 시장가 눈대중 ----
# 고칠 곳은 여기 한 군데다. 실제 견적을 받으면 값과 태그를 같이 갱신한다.
PRICE = {
    # 프로파일 — 100mm 단위 올림 과금
    "prof3030_100": (950, "확인"),        # 2026-08 실주문
    "prof2020_100": (700, "추정"),
    # 프레임 철물
    "brk3025": (360, "확인"),
    "m6x15": (70, "확인"),
    "m6x20": (90, "추정"),
    "spring_nut6": (180, "확인"),
    "tnut6": (350, "추정"),               # 드롭인 T너트 (인발 2271 N 급)
    # 필라멘트 (원/kg)
    "fil_pla": (25000, "추정"),
    "fil_petg": (30000, "추정"),
    "fil_tpu": (45000, "추정"),
    "fil_pps_cf10": (420000, "추정"),     # 참고용 — 기본 견적에는 안 쓴다
    # 판재 (원/m², 3mm)
    "sheet3_pc": (48000, "추정"),
    # 구동부
    "hub_motor": (55000, "추정"),         # 호버보드 6.5" 인휠 (중고 분해가 훨씬 싸다)
    "oem_clamp": (0, "보유"),
    # 전자·제어
    "pi5": (130000, "추정"),
    "hub_driver": (45000, "추정"),
    "battery_36v": (150000, "추정"),
    "breaker": (18000, "추정"),
    "fuse_precharge": (35000, "추정"),
    "estop": (12000, "추정"),
    "wiring": (60000, "추정"),
    # 센서·표시
    "rgbd_cam": (180000, "추정"),         # Xtion 급 RGBD
    "mini_cam": (15000, "추정"),
    "led_bar": (25000, "추정"),
    "tail_light": (12000, "추정"),
    "buzzer": (5000, "추정"),
    "limit_sw": (3000, "추정"),
    "fan60": (8000, "추정"),
    "antenna": (15000, "추정"),
    # 적재함
    "klt_bin": (18000, "추정"),
    "gas_strut": (18000, "추정"),
    "sol_lock": (22000, "추정"),
    "hinge": (8000, "추정"),
    "tie_dring": (4000, "추정"),
    "handle_bar18": (6000, "추정"),       # Ø18 알루미늄 봉 (그립)
}

DENSITY = {"pla": 1.24e-3, "petg": 1.27e-3, "tpu": 1.21e-3}   # g/mm³
FIL_KEY = {"pla": "fil_pla", "petg": "fil_petg", "tpu": "fil_tpu"}
# 출력 시간 눈대중 — 0.4 노즐 / 0.2 층 / 유효 토출 10 mm³/s.
# 벽이 얇은 셸은 이보다 오래(가속 손실), 꽉 찬 클램프는 이보다 짧게 나온다.
FLOW = 10.0
# 실제 가진 프린터 — Creality Ender-3 V3 SE. 베드를 넘는 부품은 **쪼개야 한다**.
# 이게 견적보다 먼저 걸리는 제약이라 표에 같이 낸다.
BED = (220.0, 220.0, 250.0)
BED_NAME = "Ender-3 V3 SE (220x220x250)"
oversize = []      # (품목, 치수, 필요 분할수)


def bed_split(name, part):
    """부품 bbox 를 베드에 넣어 본다. 안 들어가면 축별 분할수를 센다."""
    b = part.bounding_box()
    dims = sorted((b.size.X, b.size.Y, b.size.Z), reverse=True)
    bed = sorted(BED, reverse=True)
    if all(d <= s_ + 1e-6 for d, s_ in zip(dims, bed)):
        return 1
    n = 1
    for d, s_ in zip(dims, bed):
        n *= math.ceil(d / s_ - 1e-6)
    oversize.append((name, f"{dims[0]:.0f}x{dims[1]:.0f}x{dims[2]:.0f}", n))
    return n

rows = []          # (카테고리, 품목, 규격, 수량, 단위, 조달, 단가, 금액, 근거)
PRINT_JOBS = []    # (품목, 총 부피 mm3) — 출력 총량·시간 집계용


def add(cat, name, spec, qty, unit, src, unit_price, basis, print_vol=0.0):
    rows.append([cat, name, spec, qty, unit, src, unit_price,
                 int(round(qty * unit_price)), basis])
    if print_vol:
        PRINT_JOBS.append((name, print_vol))


def p(key):
    return PRICE[key][0]


def tag(key):
    return PRICE[key][1]


def group(label):
    for g in V.GROUPS:
        if g.label == label:
            return list(g.children)
    raise KeyError(f"그룹 없음: {label} (있는 것: {[g.label for g in V.GROUPS]})")


def vol(label):
    """그룹 전체 부피 [mm³]."""
    return sum(k.volume for k in group(label))


def printed(cat, label, mat, src="출력", name=None, spec=None, qty=None):
    """출력 부품 한 줄. 부피 → 질량 → 필라멘트 값. 시간은 근거 칸에 적는다."""
    parts = group(label)
    n = qty if qty is not None else len(parts)
    v = sum(k.volume for k in parts)
    g = v * DENSITY[mat]
    for k in parts:
        bed_split(name or label, k)
    hours = v / FLOW / 3600
    key = FIL_KEY[mat]
    cost = g / 1000 * p(key)
    add(cat, name or label, spec or f"{mat.upper()} 출력, {v/1000:.0f} cm³",
        n, "개", src, int(round(cost / max(n, 1))),
        f"형상 {v/1000:.0f} cm³ → {g:.0f} g → 출력 {hours:.1f} h [{tag(key)}]",
        print_vol=v)


def sheet(cat, label, thickness, name=None):
    """판재 한 줄. 부피/두께 = 면적."""
    parts = group(label)
    v = sum(k.volume for k in parts)
    area = v / thickness / 1e6                       # m²
    add(cat, name or label, f"PC/ABS {thickness:.0f}t, {area:.2f} m²",
        len(parts), "장", "구매", int(round(area * p("sheet3_pc") / len(parts))),
        f"형상 {v/1000:.0f} cm³ / {thickness:.0f}mm = {area:.2f} m² [{tag('sheet3_pc')}]")


def profile_rows(cat, items, series, price_key, src, label):
    """(이름, 길이) 목록 → 길이별 100mm 올림 과금 줄."""
    area = section(series).area
    buckets = {}
    for nm, length in items:
        buckets.setdefault(round(length, 1), []).append(nm)
    tot = mass = 0.0
    for length, names in sorted(buckets.items(), reverse=True):
        n = len(names)
        cut = snap_cut(length)
        units = int(cut / 100) * n
        kinds = sorted({s.rstrip("0123456789") for s in names})
        add(cat, label, f"{series}x{series} L{length:.0f}", n, "본", src,
            int(round(units * p(price_key) / n)),
            f"{units}단위(100mm 올림) · 버림 {(cut-length)*n:.0f}mm · "
            f"{', '.join(kinds)} [{tag(price_key)}]")
        tot += length * n
        mass += length * n * area * ALU_DENSITY / 1000
    return tot, mass


# =====================================================================
# 1. 프레임
# =====================================================================
CAT = "1. 프레임"
frame_len, frame_mass = profile_rows(
    CAT, [(n, l) for n, l, _, _ in V.BEAMS], V.SERIES, "prof3030_100",
    "보유", "알루미늄 프로파일")

_J = V.joints()
n_brk = len(_J)
add(CAT, "다이캐스팅 코너브라켓", "3025", n_brk, "개", "보유", p("brk3025"),
    f"joints() 자동 검출 {n_brk}곳 [{tag('brk3025')}]")

# 프레임 볼트/너트 — 조인트 + 셸 플랜지. 휠마운트 몫은 3장에서 따로 센다.
n_shell_bolt = 2 * len(V.FLANGE_BOLT_X) + 2 * len(V.FLANGE_BOLT_Y)
n_frame_bolt = n_brk * 2 + n_shell_bolt
add(CAT, "둥근머리 렌치볼트", "M6x15", n_frame_bolt, "개", "보유", p("m6x15"),
    f"브라켓 {n_brk*2} + 셸 플랜지 {n_shell_bolt} [{tag('m6x15')}]")
add(CAT, "스프링너트", "306 (M6)", n_frame_bolt, "개", "보유", p("spring_nut6"),
    f"슬롯에 들어가는 수량 = 볼트와 1:1 [{tag('spring_nut6')}]")

# =====================================================================
# 2. 판재
# =====================================================================
CAT = "2. 판재"
sheet(CAT, "판재", V.PANEL_T)
sheet(CAT, "적재함판재", V.CARGO_PT)

# =====================================================================
# 3. 구동부 (Design D)
# =====================================================================
CAT = "3. 구동부"
n_wheel = 4
add(CAT, "호버보드 인휠 허브모터", f'6.5" Ø{V.WHEEL_D:.0f}', n_wheel, "개",
    "보유", p("hub_motor"), f"스키드 4륜 [{tag('hub_motor')}]")
add(CAT, "기존 호버보드 상부 클램프", f"{P.OEM_W}x{P.OEM_H}x{P.OEM_T} 실측",
    n_wheel, "개", "보유", 0, "모터에 달려 나온 것을 재사용한다")

_lc = LC.low_clamp()
bed_split("Design-D 하부 클램프", _lc)
_lcv = _lc.volume
_lcg = _lcv * DENSITY["pla"]
add(CAT, "Design-D 하부 클램프", f"PLA 출력 {_lcv/1000:.0f} cm³ (100% 인필)",
    n_wheel, "개", "출력", int(round(_lcg / 1000 * p("fil_pla"))),
    f"{_lcv/1000:.1f} cm³ → {_lcg:.0f} g → 출력 {_lcv/FLOW/3600:.1f} h/개 "
    f"[{tag('fil_pla')}]", print_vol=_lcv * n_wheel)
add(CAT, "하부 클램프 레일 체결볼트", f"M{P.TNUT_BOLT:.0f}x{P.LOW_BOLT_LEN:.0f}",
    n_wheel * 2, "개", "보유", p("m6x15"),
    f"코너당 {2}개(밑면) x {n_wheel} [실측 길이] [{tag('m6x15')}]")
add(CAT, "하부 클램프 슬롯너트", "스프링너트 306 (M6)", n_wheel * 2, "개",
    "보유", p("spring_nut6"), f"볼트와 1:1 [{tag('spring_nut6')}]")

# 액슬 타이 — 남는 390 프로파일을 쓴다
if P.TIE and P.TIE_KIND == "profile":
    n_tie = 2                                    # 앞뒤 축 1본씩
    add(CAT, "액슬 타이 프로파일", f"3030 L{P.TIE_PROF_LEN:.0f}", n_tie, "본",
        "보유", 0,
        f"남는 390mm 재활용 (390 → {P.TIE_PROF_LEN:.0f} 절단)"
        if P.TIE_PROF_LEN < 390 else "남는 390mm 를 그대로 쓴다")
    _brk = LC.tie_brackets(210.0)
    n_tbrk = len(_brk) * 2                       # 앞뒤 축
    _bv = _brk[0].volume
    _bg = _bv * DENSITY["pla"]
    add(CAT, "타이 코너 브래킷", f"PLA 출력 {_bv/1000:.1f} cm³, L자 6t",
        n_tbrk, "개", "출력", int(round(_bg / 1000 * p("fil_pla"))),
        f"축당 {len(_brk)}개 x 2축 · {_bg:.0f} g/개 [{tag('fil_pla')}]",
        print_vol=_bv * n_tbrk)
    n_tap_bolt = n_tbrk * len(P.TIE_BRK_Z)       # 클램프 탭 쪽
    n_slot_bolt = n_tbrk                         # 프로파일 슬롯 쪽
    add(CAT, "타이 브래킷 볼트 (클램프 탭)", "M6x20", n_tap_bolt, "개", "구매",
        p("m6x20"), f"브래킷 {n_tbrk} x 탭 {len(P.TIE_BRK_Z)}단 [{tag('m6x20')}]")
    add(CAT, "타이 브래킷 볼트 (슬롯)", "M6x15", n_slot_bolt, "개", "보유",
        p("m6x15"), f"브래킷당 1 [{tag('m6x15')}]")
    add(CAT, "타이 브래킷 슬롯너트", "드롭인 T너트 (M6)", n_slot_bolt, "개",
        "구매", p("tnut6"),
        f"인발 2271 N 급 — 스프링너트(933 N)보다 여유 [{tag('tnut6')}]")
elif P.TIE and P.TIE_KIND == "rod":
    add(CAT, "액슬 타이 전산볼트", f"M{P.TIE_ROD:.0f} L{P.TIE_ROD_LEN:.0f} 8.8급",
        4, "개", "구매", 4500,
        "1m 봉 2개에서 4개 (397.4 → 400 절단) [추정]")

# =====================================================================
# 4. 외장·차체
# =====================================================================
CAT = "4. 외장·차체"
printed(CAT, "외장셸", "petg", spec=f"PETG 출력 {V.SHELL_T:.0f}t 벽")
printed(CAT, "카메라브라켓", "petg")
printed(CAT, "배터리브라켓", "petg")
printed(CAT, "범퍼", "tpu", spec=f"TPU 출력 {V.BUMPER[0]}x{V.BUMPER[1]}")
printed(CAT, "핸들", "petg", spec="PETG 출력 (베이스판 + 팔)")
add(CAT, "핸들 그립봉", f"Ø{V.HANDLE_BAR} 알루미늄 봉", len(group("핸들")), "개",
    "구매", p("handle_bar18"), f"봉만 사서 낀다 [{tag('handle_bar18')}]")
_dr = [(str(k.label), k.bounding_box().size.X) for k in group("데크레일")]
profile_rows(CAT, _dr, V.RAIL_SERIES, "prof2020_100", "구매", "데크 모듈 레일")

# =====================================================================
# 5. 전자·제어
# =====================================================================
CAT = "5. 전자·제어"
add(CAT, "라즈베리파이 5", "8GB", 1, "개", "구매", p("pi5"),
    f"메인 컴퓨터 [{tag('pi5')}]")
add(CAT, "인휠 BLDC 드라이버", "홀센서 2채널", len(group("드라이버")), "개",
    "구매", p("hub_driver"), f"모터 4개를 2ch x 2 로 [{tag('hub_driver')}]")
add(CAT, "배터리", f"36V {V.BATT[0]}x{V.BATT[1]}x{V.BATT[2]} 리튬팩", 1, "개",
    "보유", p("battery_36v"), f"호버보드 팩 재사용 [{tag('battery_36v')}]")
add(CAT, "주회로 차단기", "DC 63A", 1, "개", "구매", p("breaker"),
    f"power_box() [{tag('breaker')}]")
add(CAT, "퓨즈 · 프리차지 보드", "-", 1, "개", "구매", p("fuse_precharge"),
    f"power_box() [{tag('fuse_precharge')}]")
add(CAT, "비상정지 스위치", f"Ø{V.ESTOP_HOLE:.0f} 푸시록", 1, "개", "구매",
    p("estop"), f"뒷면 z={V.ESTOP_Z:.0f} [{tag('estop')}]")
add(CAT, "배선 · 커넥터 일체", "전원 + 모터 + 신호", 1, "식", "구매", p("wiring"),
    f"포트베이 커넥터 7종 포함 [{tag('wiring')}]")

# =====================================================================
# 6. 센서·표시
# =====================================================================
CAT = "6. 센서·표시"
add(CAT, "RGBD 카메라", f"Xtion 급 {V.XT_BAR[0]:.0f}x{V.XT_BAR[1]:.0f}", 1, "개",
    "구매", p("rgbd_cam"), f"전면 z={V.CAM_Z:.0f} [{tag('rgbd_cam')}]")
add(CAT, "소형 카메라", "USB 광각", len(group("소형카메라")), "개", "구매",
    p("mini_cam"),
    f"{', '.join(str(k.label) for k in group('소형카메라'))} [{tag('mini_cam')}]")
add(CAT, "전면 LED바", f"{group('LED바')[0].bounding_box().size.Y:.0f}mm", 1, "개",
    "구매", p("led_bar"), f"주행등 [{tag('led_bar')}]")
add(CAT, "후미등", f"{V.TAIL[0]}x{V.TAIL[1]}", len(group("후미등")), "개", "구매",
    p("tail_light"), f"뒷면 z={getattr(V, 'TAIL_Z', 276):.0f} [{tag('tail_light')}]")
add(CAT, "범퍼 리미트 스위치", "-", len(group("범퍼스위치")), "개", "구매",
    p("limit_sw"), f"앞뒤 각 2 [{tag('limit_sw')}]")
add(CAT, "배기팬", "60mm DC", len(group("배기팬")), "개", "구매", p("fan60"),
    f"cooling_cuts() [{tag('fan60')}]")
add(CAT, "부저", "Ø30", 1, "개", "구매", p("buzzer"), f"후진음 [{tag('buzzer')}]")
add(CAT, "안테나", "WiFi/LTE 휩", len(group("안테나")), "개", "구매", p("antenna"),
    f"뒷면 상부 코너 [{tag('antenna')}]")

# =====================================================================
# 7. 적재함
# =====================================================================
CAT = "7. 적재함"
printed(CAT, "적재함", "petg", name="적재함 셸",
        spec=f"PETG 출력 {V.CARGO_T:.0f}t 벽")
printed(CAT, "적재함지붕", "petg", name="장비베이 지붕")
printed(CAT, "적재함뚜껑", "petg", name="뚜껑")
printed(CAT, "슬라이드채널", "petg")
printed(CAT, "장비마운트", "petg")
printed(CAT, "바구니크래들", "petg")
printed(CAT, "리테이너", "petg")
_hw = group("적재함철물")
_strk = [k for k in _hw if "스트라이커" in str(k.label)]
_sv = sum(k.volume for k in _strk)
add(CAT, "잠금 스트라이커", f"PETG 출력 {_sv/1000:.1f} cm³", len(_strk), "개",
    "출력", int(round(_sv * DENSITY["petg"] / 1000 * p("fil_petg") / len(_strk))),
    f"{_sv*DENSITY['petg']:.0f} g [{tag('fil_petg')}]", print_vol=_sv)
add(CAT, "가스 스트럿", "뚜껑 지지", 2, "개", "구매", p("gas_strut"),
    f"뚜껑 자중 보상 [{tag('gas_strut')}]")
add(CAT, "솔레노이드 잠금", "12V", 2, "개", "구매", p("sol_lock"),
    f"원격 개폐 [{tag('sol_lock')}]")
add(CAT, "힌지", "내부 힌지", 2, "개", "구매", p("hinge"),
    f"뚜껑 [{tag('hinge')}]")
add(CAT, "타이다운 D링", f"Ø{V.CARGO_TIE}", len(group("타이다운")), "개", "구매",
    p("tie_dring"), f"바구니 결박 [{tag('tie_dring')}]")
add(CAT, "유로 컨테이너", f"KLT {V.CARGO_BIN[0]}x{V.CARGO_BIN[1]}x"
    f"{V.CARGO_BIN[2]} (26 L)", 1, "개", "구매", p("klt_bin"),
    f"규격품 [{tag('klt_bin')}]")

# =====================================================================
# 집계
# =====================================================================
CATS = []
for r in rows:
    if r[0] not in CATS:
        CATS.append(r[0])

by_cat = {c: sum(r[7] for r in rows if r[0] == c) for c in CATS}
by_src = {}
for r in rows:
    by_src[r[5]] = by_src.get(r[5], 0) + r[7]
total = sum(r[7] for r in rows)
new_money = by_src.get("구매", 0) + by_src.get("출력", 0)

# 출력 총량 — 여기가 실제로 제일 무거운 항목이다
print_vol = sum(v for _, v in PRINT_JOBS) / 1000.0        # cm3
print_hours = sum(v for _, v in PRINT_JOBS) / FLOW / 3600
print_top = sorted(PRINT_JOBS, key=lambda kv: -kv[1])[:5]

bb = V.assembly.bounding_box()

with OUT_CSV.open("w", newline="", encoding="utf-8-sig") as f:
    w = csv.writer(f)
    w.writerow(["카테고리", "품목", "규격", "수량", "단위", "조달", "단가(원)",
                "금액(원)", "근거"])
    w.writerows(rows)
    w.writerow([])
    w.writerow(["", "합계 (전부 신품 기준)", "", "", "", "", "", total, ""])
    w.writerow(["", "지금 추가로 드는 돈 (구매+출력)", "", "", "", "", "",
                new_money, ""])

L = []
A = L.append
A("# 차량 전체 버전 — BOM / 예상 견적")
A("")
A("`tools/bom/bom.py` 가 **모델 형상에서 직접** 뽑는다. 형상을 고치고 다시 돌리면")
A("수량·부피·질량이 따라온다. 손으로 고칠 곳은 스크립트 맨 위 `PRICE` 표뿐이다.")
A("")
A(f"- 구성: `PAYLOAD=cargo` 전체 버전 (프레임 + Design-D 휠마운트 + 외장셸 + 적재함)")
A(f"- 전장 {bb.size.X:.0f} x 전폭 {bb.size.Y:.0f} x 전고 {bb.size.Z:.0f} mm")
A(f"- 뷰어: `.venv/Scripts/python.exe models/hoverboard_mount/vehicle_cargo_integration.py`")
A("")
A("## 요약")
A("")
A("| 구분 | 금액 |")
A("|---|---:|")
for k in ("보유", "출력", "구매"):
    if k in by_src:
        A(f"| {k} | {by_src[k]:,}원 |")
A(f"| **전부 신품으로 살 때 합계** | **{total:,}원** |")
A(f"| **지금 추가로 드는 돈 (출력+구매)** | **{new_money:,}원** |")
A("")
A(f"출력물 총 {print_vol:,.0f} cm³ · 예상 출력 시간 약 **{print_hours:,.0f} 시간**")
A(f"(0.4 노즐 / 0.2 층 / 유효 토출 {FLOW:.0f} mm³/s 기준). 프린터 1대로 붙어 있으면")
A("2주가 넘는다 — **돈보다 이게 병목이다.**")
A("")
A("출력 시간이 큰 것부터:")
A("")
A("| 부품 | 부피 | 출력 시간 |")
A("|---|---:|---:|")
for _nm, _v in print_top:
    A(f"| {_nm} | {_v/1000:,.0f} cm³ | {_v/FLOW/3600:,.0f} h |")
A("")
A("| 카테고리 | 금액 |")
A("|---|---:|")
for c in CATS:
    A(f"| {c} | {by_cat[c]:,}원 |")
A("")
A(f"## 베드에 안 들어가는 부품 — {BED_NAME}")
A("")
if oversize:
    A("돈보다 먼저 걸리는 제약이다. 이 부품들은 **쪼개서 뽑고 붙여야** 한다")
    A("(또는 판재 절곡·진공성형으로 공법을 바꾼다).")
    A("")
    A("| 부품 | 치수 | 최소 분할 |")
    A("|---|---|---:|")
    _seen = {}
    for _nm, _dim, _n in oversize:
        _seen[(_nm, _dim)] = _seen.get((_nm, _dim), 0) + 1
    for (_nm, _dim), _cnt in _seen.items():
        _n = next(n for a, d, n in oversize if (a, d) == (_nm, _dim))
        A(f"| {_nm}{f' x{_cnt}' if _cnt > 1 else ''} | {_dim} | {_n}조각 |")
else:
    A("전부 들어간다.")
A("")
A("## 대안 — 큰 셸을 판재로 바꾸면")
A("")
A("셸 4가지가 출력 시간의 대부분을 먹고, 그러면서 베드에 하나도 안 들어간다.")
A("같은 부피를 벽 두께로 나눠 전개 면적을 내고 3mm 판재로 환산했다.")
A("(굽힘·용접·리벳 공수와 치공구는 안 넣었다 — 면적 값만이다.)")
A("")
A("| 부품 | 벽 | 전개 면적 | 출력 | 판재 | 출력 시간 |")
A("|---|---:|---:|---:|---:|---:|")
_alt_print = _alt_sheet = _alt_hours = 0
for _nm, _lbl, _t, _mat in (("외장셸", "외장셸", V.SHELL_T, "petg"),
                            ("적재함 셸", "적재함", V.CARGO_T, "petg"),
                            ("장비베이 지붕", "적재함지붕", V.CARGO_T, "petg"),
                            ("뚜껑", "적재함뚜껑", V.CARGO_PT, "petg")):
    _v = vol(_lbl)
    _a = _v / _t / 1e6
    _pc = int(round(_v * DENSITY[_mat] / 1000 * p(FIL_KEY[_mat])))
    _sc = int(round(_a * p("sheet3_pc")))
    _h = _v / FLOW / 3600
    _alt_print += _pc
    _alt_sheet += _sc
    _alt_hours += _h
    A(f"| {_nm} | {_t:.0f}mm | {_a:.2f} m² | {_pc:,}원 | {_sc:,}원 | {_h:.0f} h |")
A(f"| **합계** | | | **{_alt_print:,}원** | **{_alt_sheet:,}원** | "
  f"**{_alt_hours:.0f} h** |")
A("")
A(f"판재로 가면 부품값 {_alt_print - _alt_sheet:,}원과 출력 {_alt_hours:.0f}시간이")
A(f"빠진다 (전체 출력 시간의 {_alt_hours/print_hours*100:.0f}%). 추가 지출은")
A(f"{new_money:,} → {new_money - _alt_print + _alt_sheet:,}원.")
A("**대신 굽힘·재단 공수와 이음 철물이 붙는다.** 판재 값만 비교한 수치다.")
A("")
A("## 상세")
A("")
A("| 카테고리 | 품목 | 규격 | 수량 | 조달 | 단가 | 금액 | 근거 |")
A("|---|---|---|---:|:-:|---:|---:|---|")
for c in CATS:
    for r in rows:
        if r[0] != c:
            continue
        A(f"| {r[0]} | {r[1]} | {r[2]} | {r[3]}{r[4]} | {r[5]} | "
          f"{r[6]:,} | {r[7]:,} | {r[8]} |")
A("")
A("## 가정 — 여기가 틀리면 견적이 틀린다")
A("")
A("1. **단가는 `확인` 태그만 실주문 값이다.** 3030 프로파일 950원/100mm,")
A("   다이캐스팅 브라켓 3025 360원, M6x15 70원, 스프링너트 306 180원.")
A("   나머지는 전부 시장가 눈대중이라 실제 견적과 배는 차이날 수 있다.")
A("2. **보유 판정**은 지금까지의 실물 상태를 따랐다 — 프레임 프로파일·철물,")
A("   허브모터 4개, 기존 상부 클램프 4개, M6x15 볼트, 스프링너트, 남는 390mm")
A("   3030 프로파일, 배터리팩. 새로 만드는 거면 '전부 신품' 합계를 봐라.")
A("3. **출력 부품 값은 필라멘트 값뿐이다.** 실패분·서포트·전기·프린터 감가는")
A("   안 넣었다. 실무로는 여기에 30~50% 를 더 얹는다.")
A("4. **판재 두께·재질은 가정이다** (PC/ABS 3mm). 면적은 형상에서 나왔지만")
A("   실제로는 1220x2440 장으로 사서 자르니 남는 값을 더 내게 된다.")
A("5. **외장셸·적재함을 출력으로 잡았다.** 합쳐 11 L 가 넘어서 여기가 전체")
A("   출력 시간의 대부분이다. 진공성형이나 ABS 판재 절곡으로 가면 시간이")
A("   급감하는 대신 금형·치공구가 붙는다.")
A("6. **범퍼를 TPU 로 잡았다.** 고무 범퍼 스트립을 사서 붙이는 게 더 쌀 수 있다.")
A("7. **베드 초과 부품은 분할 비용을 안 넣었다.** 조각을 붙일 다월·접착·보강판이")
A("   추가로 들고, 이음선이 곧 약한 면이 된다.")
A("8. **인건비·배송비·부가세 없음.** 부품값만이다.")
A("")
A("## 다시 만드는 법")
A("")
A("```")
A(".venv/Scripts/python.exe tools/bom/bom.py")
A("```")
A("")
OUT_MD.write_text("\n".join(L), encoding="utf-8", newline="\n")

print(f"BOM {len(rows)}줄")
for c in CATS:
    print(f"  {c:14s} {by_cat[c]:10,}원")
print(f"  {'보유':14s} {by_src.get('보유', 0):10,}원")
print(f"  {'출력':14s} {by_src.get('출력', 0):10,}원")
print(f"  {'구매':14s} {by_src.get('구매', 0):10,}원")
print(f"  {'전부 신품':14s} {total:10,}원")
print(f"  {'추가 지출':14s} {new_money:10,}원")
print(f"  출력 {print_vol:,.0f} cm³ / 약 {print_hours:,.0f} h")
print(f"  베드 초과 {len(oversize)}건 ({BED_NAME})")
print(f"  프레임 {frame_len/1000:.2f} m / {frame_mass:.2f} kg")
print(f"SAVED {OUT_MD.relative_to(ROOT)} / {OUT_CSV.relative_to(ROOT)}")
