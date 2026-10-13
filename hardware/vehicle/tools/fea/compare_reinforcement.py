r"""보강 형태별 FEA 결과를 한 표로 모은다.

    .venv/Scripts/python.exe tools/fea/compare_reinforcement.py

케이스 폴더 이름에 형상 스위치가 접미사로 박혀 있다 (run_clamp_fea.design_variant):
    _side _shelf _tongue _tieprof _tierod
각 케이스마다 그 스위치와 같은 환경변수로 seat_splay / structural_stress 를 **자식
프로세스**로 돌린다 — parameters 가 import 시점에 환경변수를 읽기 때문이다.

지표:
  캠버       축 시트가 x 축 둘레로 기운 각 [mrad] — 바퀴가 "바깥으로 벌어지는" 것
  시트 Uy    축 시트 평균 y 변위 [mm] (+ = 바깥)
  최대변위   |U| 최대 [mm]
  구조응력   강체 고정 구멍 벽 3mm 밖 최대 von Mises [MPa]
  레일볼트   밑면 레일 볼트 2개 반력 합 [N] (인발 성분 = Fz 의 크기)
  추가고정   옆면/선반 볼트 반력 합 [N]
"""

import json
import os
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
FEA = ROOT / "models" / "hoverboard_mount" / "output" / "fea"
PY = ROOT / ".venv" / "Scripts" / "python.exe"
OUT = FEA / "comparison_reinforcement.md"

PETG_YIELD, AL_YIELD = 45.0, 276.0

CASES = [  # (폴더, 표시 이름)
    ("clamp_800N_h3mm_petg_e55mm_up", "보강 없음"),
    ("clamp_800N_h3mm_petg_e55mm_up_side", "옆 날개 4 (부러진 그 형상)"),
    ("clamp_800N_h3mm_petg_e55mm_up_tierod", "M8 전산볼트 타이 ×2"),
    ("clamp_800N_h3mm_petg_e55mm_up_tieprof", "3030 타이 (아래)"),
    ("clamp_800N_h3mm_petg_e55mm_up_shelf", "H 크로스 + 선반"),
    ("clamp_800N_h3mm_petg_e55mm_up_side_shelf", "H 크로스 + 선반 + 날개"),
    ("clamp_800N_h3mm_e55mm_up_shelf", "H 크로스 + 선반 — AL6061"),
    # 혀(tongue) — 혀 볼트 구멍 벽을 강체 고정 / 접지 스프링(z, 볼트당 K N/mm) 으로.
    # _k<K> 케이스는 구멍 벽 x/y 가 자유, _ky<KY> 는 y 에도 접지 스프링을 단 민감도.
    ("clamp_800N_h3mm_petg_e55mm_up_tongue", "H 크로스 + 혀 (조인트 강체)"),
    ("clamp_800N_h3mm_petg_e55mm_up_tongue_k23430", "H 크로스 + 혀 (스프링너트 조인트)"),
    ("clamp_800N_h3mm_petg_e55mm_up_tongue_k98415", "H 크로스 + 혀 (T너트 조인트)"),
    ("clamp_800N_h3mm_petg_e55mm_up_tongue_k23430_ky23430",
     "H 크로스 + 혀 (스프링너트 조인트 + y 스프링 =K)"),
    ("clamp_800N_h3mm_petg_e55mm_up_tongue_k23430_ky10000000",
     "H 크로스 + 혀 (스프링너트 조인트 + y 잠금)"),
    ("clamp_800N_h3mm_e55mm_up_tongue_k23430", "H 크로스 + 혀 — AL6061 (스프링너트 조인트)"),
]


def env_for(folder):
    """폴더 접미사 → run_clamp_fea 와 같은 환경변수.

    _tongue 는 H 크로스 **혀**(LOW_SHELF=1, SHELF_KIND=tongue) 이고, 일체형 T 텅
    (parameters.TONGUE) 은 _ttongue 다 — 예전엔 _tongue 를 TONGUE=1 로 읽어
    LOW_SHELF 를 안 켰고, 그러면 structural_stress 가 혀 구멍 특이점을 구조응력으로 센다.
    """
    e = dict(os.environ)
    e["LOW_SIDE"] = "1" if "_side" in folder else "0"
    e["TONGUE"] = "1" if "_ttongue" in folder else "0"
    if "_tongue" in folder:
        e["LOW_SHELF"], e["SHELF_KIND"] = "1", "tongue"
    elif "_shelf" in folder:
        e["LOW_SHELF"], e["SHELF_KIND"] = "1", "wide"
    else:
        e["LOW_SHELF"] = "0"
    if "_tieprof" in folder:
        e["TIE"], e["TIE_KIND"] = "1", "profile"
    elif "_tierod" in folder:
        e["TIE"], e["TIE_KIND"] = "1", "rod"
    else:
        e["TIE"] = "0"
    m = re.search(r"_k(\d+(?:\.\d+)?)(?:_|$)", folder)
    e["SHELF_JOINT_K"] = m.group(1) if m else "0"
    m = re.search(r"_ky(\d+(?:\.\d+)?)(?:_|$)", folder)
    e["SHELF_JOINT_KY"] = m.group(1) if m else "0"
    return e


def run(script, args, env):
    r = subprocess.run([str(PY), str(ROOT / "tools" / "fea" / script), *args],
                       env=env, capture_output=True, text=True, timeout=600)
    return r.stdout


def reactions(folder):
    txt = (FEA / folder / "summary.txt").read_text(encoding="utf-8")
    groups = dict(re.findall(r"^  (\w+): \[([-\d.e, ]+)\]$", txt, re.M))
    vec = {k: [float(x) for x in v.split(",")] for k, v in groups.items()}
    rail = sum(abs(vec[k][2]) for k in vec if k.startswith("bottom"))
    # 혀 스프링 케이스는 shelf_* 가 스프링 힘(-K/n·ΣUz) — run_clamp_fea 가 RF 대신 적는다.
    extra = sum(abs(vec[k][2]) for k in vec if k.startswith(("side", "shelf")))
    tie = re.search(r"Axle tie: (.+)", txt)
    return rail, extra, (tie.group(1) if tie else "none")


def main():
    rows = []
    for folder, name in CASES:
        if not (FEA / folder).exists() or not list((FEA / folder).glob("*.vtu")):
            rows.append((name, folder, None))
            continue
        env = env_for(folder)
        sp = run("seat_splay.py", [str(FEA / folder)], env).strip().splitlines()[-1].split()
        uy, theta, splay, umax = (float(v) for v in sp[-4:])
        st = run("structural_stress.py", [folder], env)
        m = re.search(r"최대\s+([\d.]+).*?구조\s+([\d.]+) MPa", st)
        smax, sstr = (float(m.group(1)), float(m.group(2))) if m else (float("nan"),) * 2
        rail, extra, tie = reactions(folder)
        yld = AL_YIELD if "_petg" not in folder else PETG_YIELD
        rows.append((name, folder, dict(uy=uy, theta=theta, umax=umax, smax=smax, sstr=sstr,
                                        util=sstr / yld * 100, rail=rail, extra=extra, tie=tie)))

    base = next((r[2] for r in rows if r[1].endswith("_up")), None)
    L = ["# 보강 형태 비교 — Design D 하부 클램프 FEA", "",
         "800 N 상향 · 편심 55 mm · 3 mm C3D10 · PETG 근사 (E 2100, 항복 45) — 이름에 AL6061 이 붙은 줄만 알루미늄.",
         "레일 볼트 구멍 벽 강체 고정, 타이는 접지 스프링, 선반·날개 볼트도 강체 고정.",
         "혀(tongue) 줄의 '조인트' 는 혀 볼트 구멍 벽을 **z 접지 스프링**(볼트당 K: 경제형 스프링너트 23,430 / 드롭인 T너트 98,415 N/mm)",
         "으로 바꾼 것 — H 크로스가 통째로 내려가는 것을 잡는 크로스 양 끝 브라켓 조인트의 무름을 넣는다 (`SHELF_JOINT_K`).",
         "그 줄에서 구멍 벽 x/y 는 자유이고, '+ y 스프링/잠금' 줄은 같은 벽에 y 접지 스프링(`SHELF_JOINT_KY`)을 더한 민감도다.",
         "**캠버**가 실물에서 보이는 '바깥으로 벌어짐'이다 — 축이 옆으로 밀리는 양(시트 Uy)보다 훨씬 크다.", "",
         "| 형태 | 캠버 mrad | 배율 | 시트 Uy mm | 최대변위 mm | 배율 | 구조응력 MPa | 항복 대비 | 레일볼트 N | 추가고정 N |",
         "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|"]
    for name, folder, d in rows:
        if d is None:
            L.append(f"| {name} | (결과 없음: `{folder}`) |||||||||")
            continue
        th_ratio = f"{base['theta'] / d['theta']:.1f}×" if base and d['theta'] else "-"
        u_ratio = f"{base['umax'] / d['umax']:.1f}×" if base else "-"
        L.append(f"| {name} | {d['theta']:+.3f} | {th_ratio} | {d['uy']:+.4f} | {d['umax']:.4f} | {u_ratio} | "
                 f"{d['sstr']:.1f} | {d['util']:.0f}% | {d['rail']:.0f} | {d['extra']:.0f} |")
    L += ["", "배율 = 보강 없음 대비 감소 배수 (클수록 좋다). 추가고정 = 날개/선반 볼트 반력 합 (혀 스프링 케이스는 스프링 힘 -K/n·ΣUz).",
          "구조응력은 강체 고정 구멍 벽 3mm 밖 최대값 (`tools/fea/structural_stress.py`) — 최대 von Mises 는 특이점이라 안 쓴다.", ""]
    # 표 아래 손으로 적어 둔 "## 참고" 절은 지우지 않고 그대로 이어 붙인다.
    if OUT.exists():
        old = OUT.read_text(encoding="utf-8")
        cut = old.find("\n## 참고")
        if cut >= 0:
            L.append(old[cut + 1:].rstrip("\n"))
            L.append("")
    OUT.write_text("\n".join(L), encoding="utf-8", newline="\n")
    print("\n".join(L))
    print(f"SAVED {OUT.relative_to(ROOT)}")


if __name__ == "__main__":
    main()
