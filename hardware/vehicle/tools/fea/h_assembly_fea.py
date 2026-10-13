r"""결합(bonded) 솔리드 조립 FEA — 레일 토막 2 + 하부 클램프 2 (+ 3030 타이 / H 크로스+혀).

    .venv/Scripts/python.exe tools/fea/h_assembly_fea.py --case all
    .venv/Scripts/python.exe tools/fea/h_assembly_fea.py --case hcross --mesh-size 4

단품 해석(run_clamp_fea.py)은 레일 볼트 구멍 벽을 강체로 고정한다. 그 모델로는 "H 크로스
양 끝 브라켓 조인트가 혀와 직렬로 들어간다"는 경로 차이를 볼 수 없어서, 여기서는 클램프와
프로파일을 **하나의 결합 솔리드**로 만들어 레일 토막 양 끝만 고정한다.

  · 차량 좌표 (레일 y=±210, 축 z=79.35, 바퀴 중심 y=±263.7). x=0 이 액슬 라인.
  · 3030 은 **등가 각관** (30x30, 벽 2.15 → I≈3.1e4 mm⁴, 굽힘 강성만 실물과 맞춘다).
  · 클램프 = low_clamp.low_clamp() 를 vehicle_integration.mounted_wheels 와 같은 변환으로.
  · 결합이라 T너트·브라켓·볼트 예압·접촉이 없다 — 조인트 강성을 과대평가한다.
  · 재료는 요소 중심 좌표로 갈라 두 *SOLID SECTION (PETG 클램프 / AL 프로파일).

parameters.py 가 import 시점에 환경변수를 읽으므로 케이스마다 **자식 프로세스**를 띄운다
(--child 는 내부용). run_clamp_fea 의 gmsh/CalculiX/ccx2paraview 호출을 복사했다 — 그
파일은 단품 전용 전역 상태가 많아 import 하지 않는다.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent            # tools/fea
ROOT = HERE.parents[1]
TOOLS_PY = ROOT / "tools" / "fea" / "python"
GMSH_PY = ROOT / "tools" / "fea" / "gmsh_pyenv" / "Lib" / "site-packages"
MODELS = ROOT / "models" / "hoverboard_mount"
for dependency_path in (TOOLS_PY, GMSH_PY):       # run_clamp_fea 와 같은 순서 (gmsh_pyenv 가 앞)
    if str(dependency_path) not in sys.path:
        sys.path.insert(0, str(dependency_path))
for p in (MODELS, ROOT / "models"):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

PY = ROOT / ".venv" / "Scripts" / "python.exe"
CCX = ROOT / "tools" / "fea" / "calculix" / "calculix_2.23_4win" / "ccx_static.exe"
OUT_ROOT = MODELS / "output" / "fea"
STEP_DIR = MODELS / "output" / "step"
TABLE = HERE / "h_assembly_results.md"

MATERIALS = {
    "petg": {"name": "PETG_ISOTROPIC", "label": "PETG isotropic estimate",
             "e": 2_100.0, "nu": 0.38, "yield": 45.0},
    "al":   {"name": "AL_PROFILE", "label": "aluminium 6063-T5 profile",
             "e": 69_000.0, "nu": 0.33, "yield": 145.0},
}

# ---------------------------------------------------------------- 케이스
CASES = {
    "none": dict(
        env=dict(LOW_SHELF="0", TIE="0", LOW_SIDE="0", TONGUE="0"),
        title="no reinforcement", label="보강 없음",
        expected_solids=2,        # 좌우가 안 이어져 있으니 2개가 맞다
    ),
    "tie": dict(
        env=dict(LOW_SHELF="0", TIE="1", TIE_KIND="profile", LOW_SIDE="0", TONGUE="0"),
        title="3030 tie below (373.4 mm, bonded to clamp inner faces)",
        label="3030 타이 (아래, 결합)", expected_solids=1,
    ),
    "hcross": dict(
        env=dict(LOW_SHELF="1", SHELF_KIND="tongue", TIE="0", LOW_SIDE="0", TONGUE="0"),
        title="H cross 3030 x 390 + tongue (bonded)",
        label="H 크로스 + 혀 (결합)", expected_solids=1,
    ),
}

# ---------------------------------------------------------------- 차량 좌표 제원
RAIL_Y = 210.0            # 레일 중심 y
S = 30.0                  # 3030
WALL = 2.15               # 등가 각관 벽 — I = (30^4 - 25.7^4)/12 = 3.11e4 mm^4
RAIL_L = 240.0            # 레일 토막 길이 (x = ±120 고정)
R_WHEEL = None            # parameters 에서 채운다 (자식 프로세스)


def case_env(case: str) -> dict[str, str]:
    e = dict(os.environ)
    e.update(CASES[case]["env"])
    return e


# ================================================================ 자식 프로세스 본체
def child_main(args):
    """환경변수가 맞춰진 자식에서만 부른다."""
    case = args.case
    want = CASES[case]["env"]
    for k, v in want.items():
        if os.environ.get(k) != v:
            raise SystemExit(f"env {k}={os.environ.get(k)!r} != {v!r} — 부모에서 띄워야 한다")

    import gmsh                      # noqa: E402
    import meshio                    # noqa: E402
    import numpy as np               # noqa: E402
    from ccx2paraview import Converter  # noqa: E402
    import low_clamp as LC           # noqa: E402
    import parameters as P           # noqa: E402
    from build123d import Box, BuildPart, Location, Locations, Mode, Pos, Rot  # noqa: E402
    from build123d.exporters3d import export_step  # noqa: E402

    global R_WHEEL
    R_WHEEL = P.WHEEL_DIAMETER / 2
    AXLE_Z = P.AXLE_Z                                 # 79.35
    RAIL_Z = AXLE_Z + LC.RAIL_CZ                      # 108.85
    RAIL_BOT_Z = RAIL_Z - S / 2                       # 93.85 = 다리 윗면 = 혀 윗면
    WHEEL_Y = RAIL_Y - LC.CLAMP_CY                    # 263.7
    CLAMP_IN_Y = RAIL_Y - P.LOW_D / 2                 # 186.7 클램프 안쪽면
    CLAMP_OUT_Y = RAIL_Y + P.LOW_D / 2                # 233.3
    TIE_Z = AXLE_Z + P.TIE_PROF_Z                     # 69.95
    TIE_LEN = 2 * CLAMP_IN_Y                          # 373.4 (브래킷 생략, 안쪽면에 직접)
    CROSS_LEN = 2 * RAIL_Y - S                        # 390 레일 안쪽면 사이
    SEAT_R = P.AXLE_DIAMETER / 2 + P.SADDLE_SEAT_FIT
    TONGUE_IN_Y = CLAMP_IN_Y - P.SHELF_REACH          # 131.7 혀 끝
    ECC = args.eccentricity
    LOAD = args.load
    SIGN = 1.0 if args.load_direction == "upward" else -1.0

    # 기준 4 mm 는 접미사 없이, 수렴 확인용 다른 격자는 _h<size>mm 를 붙인다 (덮어쓰기 방지)
    case_dir = OUT_ROOT / (f"h_assembly_{case}" + ("" if args.mesh_size == 4.0 else f"_h{args.mesh_size:g}mm"))
    case_dir.mkdir(parents=True, exist_ok=True)
    t_start = time.time()

    # ------------------------------------------------------------ 형상
    def tube(length, axis, center):
        with BuildPart() as p:
            with Locations(center):
                if axis == "X":
                    Box(length, S, S)
                    Box(length + 2, S - 2 * WALL, S - 2 * WALL, mode=Mode.SUBTRACT)
                else:
                    Box(S, length, S)
                    Box(S - 2 * WALL, length + 2, S - 2 * WALL, mode=Mode.SUBTRACT)
        return p.part

    clamp0 = LC.low_clamp()
    if len(clamp0.solids()) != 1:
        raise RuntimeError("low_clamp 가 조각났다")
    parts = []
    for sy in (-1, 1):
        parts.append(tube(RAIL_L, "X", (0, sy * RAIL_Y, RAIL_Z)))
        # vehicle_integration.mounted_wheels 와 같은 변환 (x=0 = 액슬 라인)
        tf = Pos(0, sy * WHEEL_Y, AXLE_Z) * (Rot(0, 0, 180) if sy < 0 else Location())
        parts.append(tf * clamp0.moved(Location()))
    if case == "tie":
        parts.append(tube(TIE_LEN, "Y", (0, 0, TIE_Z)))
    elif case == "hcross":
        parts.append(tube(CROSS_LEN, "Y", (0, 0, RAIL_Z)))
    vol_sum = sum(p.volume for p in parts)
    fused = parts[0]
    for p in parts[1:]:
        fused = fused.fuse(p)
    fused = fused.clean()
    n_solids = len(fused.solids())
    if n_solids != CASES[case]["expected_solids"]:
        raise RuntimeError(f"fuse 결과 솔리드 {n_solids}개 (기대 {CASES[case]['expected_solids']})")
    if abs(fused.volume - vol_sum) > 1.0:
        raise RuntimeError(f"fuse 부피 {fused.volume:.1f} != 부품 합 {vol_sum:.1f} — 부품이 겹친다")
    step = STEP_DIR / f"h_assembly_{case}.step"
    STEP_DIR.mkdir(parents=True, exist_ok=True)
    if not export_step(fused, str(step)):
        raise RuntimeError(f"STEP export failed: {step}")
    print(f"[geom] solids={n_solids} volume={fused.volume:.1f} mm3 -> {step}")

    # ------------------------------------------------------------ 메쉬 (run_clamp_fea.create_mesh 복사·수정)
    raw_inp = case_dir / "assembly_mesh_raw.inp"
    msh = case_dir / "assembly_mesh.msh"
    gmsh.initialize()
    try:
        gmsh.option.setNumber("General.Terminal", 1)
        gmsh.option.setNumber("General.Verbosity", 2)
        gmsh.model.add(f"h_assembly_{case}")
        imported = gmsh.model.occ.importShapes(str(step))
        gmsh.model.occ.synchronize()
        volumes = [tag for dim, tag in imported if dim == 3]
        if not volumes:
            volumes = [tag for _, tag in gmsh.model.getEntities(3)]
        if len(volumes) != n_solids:
            raise RuntimeError(f"gmsh volumes {len(volumes)} != solids {n_solids}")
        volume_group = gmsh.model.addPhysicalGroup(3, volumes)
        gmsh.model.setPhysicalName(3, volume_group, "VOLUME_ONLY")
        gmsh.option.setNumber("Mesh.SaveAll", 0)
        gmsh.option.setNumber("Mesh.MeshSizeMin", args.mesh_min)
        gmsh.option.setNumber("Mesh.MeshSizeMax", args.mesh_size)
        gmsh.option.setNumber("Mesh.MeshSizeFromCurvature", 14)
        gmsh.option.setNumber("Mesh.Algorithm3D", 10)          # HXT
        gmsh.model.mesh.generate(3)
        gmsh.model.mesh.optimize("Netgen")
        gmsh.model.mesh.setOrder(2)
        gmsh.model.mesh.optimize("HighOrderElastic", niter=5)
        gmsh.model.mesh.optimize("HighOrder", niter=5)

        tags, xyz, _ = gmsh.model.mesh.getNodes()
        xyz = np.asarray(xyz, float).reshape(-1, 3)
        tags = np.asarray(tags, int)
        coords = {int(t): (float(x), float(y), float(z)) for t, (x, y, z) in zip(tags, xyz)}
        surface: set[int] = set()
        for _, stag in gmsh.model.getEntities(2):
            st, _, _ = gmsh.model.mesh.getNodes(2, stag, includeBoundary=True)
            surface.update(int(t) for t in st)

        element_types, element_tags, element_nodes = gmsh.model.mesh.getElements(3)
        element_ids: list[int] = []
        centroids: list[np.ndarray] = []
        idx = {int(t): i for i, t in enumerate(tags)}
        for etype, etags, enodes in zip(element_types, element_tags, element_nodes):
            name, _, _, nn, _, _ = gmsh.model.mesh.getElementProperties(etype)
            if "Tetrahedron 10" not in name:
                raise RuntimeError(f"Unexpected solid element type: {name}")
            conn = np.asarray(enodes, int).reshape(-1, nn)
            corner = np.vectorize(idx.get)(conn[:, :4])
            centroids.append(xyz[corner].mean(axis=1))
            element_ids.extend(int(t) for t in etags)
        centroid = np.vstack(centroids)
        element_ids_arr = np.asarray(element_ids, int)
        gmsh.write(str(msh))
        gmsh.write(str(raw_inp))
    finally:
        gmsh.finalize()

    # ------------------------------------------------------------ 재료 분할 (요소 중심)
    cx, cy, cz = centroid[:, 0], centroid[:, 1], centroid[:, 2]
    alu = cz > RAIL_BOT_Z + 1e-6                          # 레일·크로스: 클램프는 전부 z<=93.85
    if case == "tie":
        alu |= np.abs(cy) < CLAMP_IN_Y - 1e-6             # 타이: 클램프 안쪽면보다 안쪽
    e_alu = element_ids_arr[alu].tolist()
    e_clamp = element_ids_arr[~alu].tolist()
    if not e_alu or not e_clamp:
        raise RuntimeError("재료 분할 실패")

    # ------------------------------------------------------------ 노드 분류
    tol = max(0.12, args.mesh_size * 0.08)
    fixed: set[int] = set()
    seats = {-1: [], 1: []}
    for t in surface:
        x, y, z = coords[t]
        if abs(abs(x) - RAIL_L / 2) <= tol and abs(y) >= RAIL_Y - S / 2 - tol and z >= RAIL_BOT_Z - tol:
            fixed.add(t)
        for sy in (-1, 1):
            if (abs(math.hypot(x, z - AXLE_Z) - SEAT_R) <= tol and z <= AXLE_Z + 0.15
                    and CLAMP_IN_Y - tol <= sy * y <= CLAMP_OUT_Y + tol):
                seats[sy].append(t)
    if len(fixed) < 100:
        raise RuntimeError(f"레일 끝 고정 노드가 {len(fixed)}개뿐")
    for sy in (-1, 1):
        if len(seats[sy]) < 12:
            raise RuntimeError(f"시트 노드 부족 side {sy}: {len(seats[sy])}")
        if set(seats[sy]) & fixed:
            raise RuntimeError("시트/고정 노드 겹침")

    # ------------------------------------------------------------ 하중 (run_clamp_fea.build_solver_deck 의 편심 커플)
    load_terms = []                    # (set name, nodes, total, per node)
    total_vertical = SIGN * LOAD
    for sy, side in ((-1, "L"), (1, "R")):
        nodes = seats[sy]
        loc_y = {t: sy * coords[t][1] - WHEEL_Y for t in nodes}     # 클램프 로컬 y (+ = 바깥)
        inboard = [t for t in nodes if loc_y[t] < LC.CLAMP_CY]
        outboard = [t for t in nodes if loc_y[t] >= LC.CLAMP_CY]
        if len(inboard) < 6 or len(outboard) < 6:
            raise RuntimeError("편심 커플을 만들 노드가 부족")
        yi = sum(loc_y[t] for t in inboard) / len(inboard)
        yo = sum(loc_y[t] for t in outboard) / len(outboard)
        target = LC.CLAMP_CY + ECC
        f_out = total_vertical * (target - yi) / (yo - yi)
        f_in = total_vertical - f_out
        load_terms.append((f"SEAT_{side}_OUTBOARD", outboard, f_out, f_out / len(outboard)))
        load_terms.append((f"SEAT_{side}_INBOARD", inboard, f_in, f_in / len(inboard)))

    # ------------------------------------------------------------ 덱
    def chunks(values, width=16):
        for s0 in range(0, len(values), width):
            yield values[s0:s0 + width]

    def write_set(stream, kw, name, values):
        stream.write(f"*{kw},{kw}={name}\n")
        for row in chunks(sorted(values)):
            stream.write(",".join(str(v) for v in row) + "\n")

    job = "h_assembly"
    job_inp = case_dir / f"{job}.inp"
    shutil.copyfile(raw_inp, job_inp)
    with job_inp.open("a", encoding="ascii", newline="\n") as st:
        st.write("\n** --- bonded assembly additions ---\n")
        write_set(st, "ELSET", "ECLAMP", e_clamp)
        write_set(st, "ELSET", "EALU", e_alu)
        write_set(st, "NSET", "FIXED_RAIL_ENDS", sorted(fixed))
        for name, nodes, _, _ in load_terms:
            write_set(st, "NSET", name, nodes)
        write_set(st, "NSET", "SEAT_ALL", seats[-1] + seats[1])
        for key in ("petg", "al"):
            m = MATERIALS[key]
            st.write(f"*MATERIAL,NAME={m['name']}\n*ELASTIC\n{m['e']:.1f},{m['nu']:.5f}\n")
        st.write(f"*SOLID SECTION,ELSET=ECLAMP,MATERIAL={MATERIALS['petg']['name']}\n")
        st.write(f"*SOLID SECTION,ELSET=EALU,MATERIAL={MATERIALS['al']['name']}\n")
        st.write("*BOUNDARY\nFIXED_RAIL_ENDS,1,3,0.0\n")
        st.write("*STEP,NLGEOM=NO\n*STATIC\n*CLOAD\n")
        for name, _, _, per in load_terms:
            st.write(f"{name},3,{per:.12g}\n")
        st.write("*NODE FILE\nU,RF\n*EL FILE\nS,E\n")
        st.write("*NODE PRINT,NSET=SEAT_ALL,TOTALS=YES\nU\n")
        st.write("*NODE PRINT,NSET=FIXED_RAIL_ENDS,TOTALS=YES\nRF\n")
        st.write("*END STEP\n")
    n_nodes = len(coords)
    print(f"[mesh] elements={len(element_ids)} (clamp {len(e_clamp)}, alu {len(e_alu)}) "
          f"nodes={n_nodes} fixed={len(fixed)} seatL={len(seats[-1])} seatR={len(seats[1])}")

    # ------------------------------------------------------------ 솔버 (run_clamp_fea.run_solver 복사)
    if not CCX.exists():
        raise FileNotFoundError(CCX)
    env = dict(os.environ)
    env.setdefault("OMP_NUM_THREADS", str(args.threads))
    env.setdefault("CCX_NPROC_STIFFNESS", str(args.threads))
    t0 = time.time()
    proc = subprocess.run([str(CCX), "-i", job], cwd=case_dir, text=True,
                          capture_output=True, check=False, env=env)
    (case_dir / "solver_stdout.log").write_text(proc.stdout, encoding="utf-8")
    (case_dir / "solver_stderr.log").write_text(proc.stderr, encoding="utf-8")
    if proc.returncode != 0:
        raise RuntimeError(f"CalculiX exit {proc.returncode}; see {case_dir / 'solver_stdout.log'}")
    frd = case_dir / f"{job}.frd"
    if not frd.exists() or frd.stat().st_size == 0:
        raise RuntimeError("no FRD")
    solve_s = time.time() - t0
    print(f"[ccx] {solve_s:.0f} s")

    # ------------------------------------------------------------ 변환 (run_clamp_fea.convert_and_summarize 복사)
    orig = np.linalg.eigvals
    np.linalg.eigvals = lambda tensor: np.linalg.eigvalsh(np.asarray(tensor, dtype=float))  # type: ignore
    try:
        Converter(str(frd), ["vtu"]).run()
    finally:
        np.linalg.eigvals = orig  # type: ignore
    vtus = sorted(frd.parent.glob(f"{frd.stem}*.vtu"))
    if not vtus:
        raise RuntimeError("no VTU")
    vtu = vtus[-1]
    mesh = meshio.read(vtu)
    pts = np.asarray(mesh.points)
    U = np.asarray(mesh.point_data["U"])[:, :3]
    SM = np.asarray(mesh.point_data["S_Mises"]).ravel()
    RF = np.asarray(mesh.point_data["RF"])[:, :3]

    # ------------------------------------------------------------ 지표
    x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
    umag = np.linalg.norm(U, axis=1)
    # 노드 영역: 클램프 = z<=93.85, |y|>=186.7, |x|<=클램프 반폭 (경계 노드는 양쪽에 걸린다)
    node_clamp = ((z <= RAIL_BOT_Z + 1e-3) & (np.abs(y) >= CLAMP_IN_Y - 1e-3)
                  & (np.abs(x) <= P.LOW_W / 2 + 1e-3))
    node_alu = ~node_clamp | (np.abs(z - RAIL_BOT_Z) < 1e-3) | (np.abs(np.abs(y) - CLAMP_IN_Y) < 1e-3)
    # 결합면까지의 거리 (클램프 쪽)
    d_bond = np.full(len(pts), np.inf)
    # 다리 윗면 ↔ 레일 밑면: z=93.85, 레일 발자국 |y|∈[195,225]
    dy_out = np.maximum(0.0, np.abs(np.abs(y) - RAIL_Y) - S / 2)
    d_bond = np.minimum(d_bond, np.hypot(RAIL_BOT_Z - z, dy_out))
    if case == "hcross":
        # 혀 윗면 ↔ 크로스 밑면: z=93.85, |x|<=15, |y|∈[131.7,186.7]
        dyo = np.maximum(0.0, np.maximum(TONGUE_IN_Y - np.abs(y), np.abs(y) - CLAMP_IN_Y))
        dxo = np.maximum(0.0, np.abs(x) - S / 2)
        d_bond = np.minimum(d_bond, np.sqrt((RAIL_BOT_Z - z) ** 2 + dyo ** 2 + dxo ** 2))
    if case == "tie":
        # 타이 끝면 ↔ 클램프 안쪽면: |y|=186.7, |x|<=15, z∈[54.95,84.95]
        dxo = np.maximum(0.0, np.abs(x) - S / 2)
        dzo = np.maximum(0.0, np.abs(z - TIE_Z) - S / 2)
        d_bond = np.minimum(d_bond, np.sqrt((np.abs(y) - CLAMP_IN_Y) ** 2 + dxo ** 2 + dzo ** 2))
    far = node_clamp & (d_bond > args.bond_exclude)
    clamp_structural = float(SM[far].max())
    at = pts[far][int(np.argmax(SM[far]))]
    clamp_max = float(SM[node_clamp].max())
    alu_max = float(SM[node_alu].max())
    umax = float(umag.max())
    umax_clamp = float(umag[node_clamp].max())

    seat_tol = 0.3
    sides = {}
    for sy, side in ((-1, "left"), (1, "right")):
        m = ((np.abs(np.hypot(x, z - AXLE_Z) - SEAT_R) <= seat_tol) & (z <= AXLE_Z + 0.15)
             & (sy * y >= CLAMP_IN_Y - seat_tol) & (sy * y <= CLAMP_OUT_Y + seat_tol))
        if m.sum() < 20:
            raise RuntimeError(f"시트 노드 {m.sum()}개 ({side})")
        y_out = sy * y[m]                           # 바깥으로 증가
        uz = U[m, 2]
        theta = float(np.polyfit(y_out, uz, 1)[0])  # dUz/dy_out : 바깥이 위로 들리면 양수
        uy_out = float((sy * U[m, 1]).mean())       # + = 바깥
        sides[side] = dict(n=int(m.sum()), camber_mrad=theta * 1e3, uy_out=uy_out,
                           uz_mean=float(uz.mean()), splay=uy_out - theta * R_WHEEL)
    # 경로 진단 — 캠버가 어디서 오는지: 레일 토막 비틀림 / 다리 윗면 기울기 / 크로스 처짐
    diag = {}
    for sy, side in ((-1, "left"), (1, "right")):
        rb = (np.abs(z - RAIL_BOT_Z) < 0.05) & (np.abs(x) < 3.0)
        inner = rb & (np.abs(sy * y - (RAIL_Y - S / 2)) < 0.6)
        outer = rb & (np.abs(sy * y - (RAIL_Y + S / 2)) < 0.6)
        lt = ((np.abs(z - RAIL_BOT_Z) < 0.05) & (sy * y > CLAMP_IN_Y - 0.1) & (sy * y < CLAMP_OUT_Y + 0.1)
              & (np.abs(x) > P.LOW_POST_X - P.LOW_POST_W / 2 - 0.1) & (np.abs(x) < P.LOW_W / 2 + 0.1))
        d = dict(rail_twist_mrad=float((U[outer, 2].mean() - U[inner, 2].mean()) / S * 1e3),
                 rail_rise_mm=float((U[outer, 2].mean() + U[inner, 2].mean()) / 2),
                 legtop_slope_mrad=float(np.polyfit(sy * y[lt], U[lt, 2], 1)[0] * 1e3))
        if case == "hcross":
            cb = (np.abs(z - RAIL_BOT_Z) < 0.05) & (np.abs(x) < 3.0)
            tip = cb & (np.abs(sy * y - TONGUE_IN_Y) < 1.0)
            ct = (np.abs(z - (RAIL_Z + S / 2)) < 0.05) & (np.abs(x) < 3.0) & (np.abs(y) < 2.0)
            d["cross_uz_at_tongue_tip"] = float(U[tip, 2].mean())
            d["cross_top_mid_uz"] = float(U[ct, 2].mean())
        diag[side] = d

    # 고정 노드는 기하로 다시 고른다 (VTU 점 순서가 gmsh 태그와 같다고 가정하지 않는다)
    fixed_mask = ((np.abs(np.abs(x) - RAIL_L / 2) <= tol) & (np.abs(y) >= RAIL_Y - S / 2 - tol)
                  & (z >= RAIL_BOT_Z - tol))
    if fixed_mask.sum() != len(fixed):
        print(f"[warn] VTU fixed-node count {fixed_mask.sum()} != deck {len(fixed)}")
    rf_total = RF[fixed_mask].sum(axis=0)
    rf_all = RF.sum(axis=0)

    # ------------------------------------------------------------ 그림 (run_clamp_fea.render_result 를 조립체 카메라로)
    png = case_dir / f"stress_deformed_{int(LOAD)}N.png"
    render_case(vtu, png, dict(title=CASES[case]["title"], load_n=LOAD, load_direction=args.load_direction,
                               eccentricity_mm=ECC, clamp_structural_mpa=clamp_structural, umax_mm=umax,
                               camber_mrad_left=sides["left"]["camber_mrad"],
                               camber_mrad_right=sides["right"]["camber_mrad"]))

    metrics = dict(
        case=case, label=CASES[case]["label"], title=CASES[case]["title"], step=str(step),
        folder=str(case_dir), vtu=str(vtu), png=str(png),
        load_n=LOAD, load_direction=args.load_direction, eccentricity_mm=ECC,
        mesh_size=args.mesh_size, mesh_min=args.mesh_min,
        elements=len(element_ids), elements_clamp=len(e_clamp), elements_alu=len(e_alu),
        nodes=n_nodes, fixed_nodes=len(fixed), solids=n_solids,
        camber_mrad_left=sides["left"]["camber_mrad"], camber_mrad_right=sides["right"]["camber_mrad"],
        uy_out_left=sides["left"]["uy_out"], uy_out_right=sides["right"]["uy_out"],
        uz_seat_left=sides["left"]["uz_mean"], uz_seat_right=sides["right"]["uz_mean"],
        splay_left=sides["left"]["splay"], splay_right=sides["right"]["splay"],
        umax_mm=umax, umax_clamp_mm=umax_clamp,
        clamp_structural_mpa=clamp_structural, clamp_structural_at=[float(v) for v in at],
        clamp_max_mpa=clamp_max, alu_max_mpa=alu_max, bond_exclude_mm=args.bond_exclude,
        rf_fixed=[float(v) for v in rf_total], rf_all=[float(v) for v in rf_all],
        applied_z=float(2 * total_vertical), solve_s=solve_s, total_s=time.time() - t_start,
        load_terms=[(n, len(nodes), tot, per) for n, nodes, tot, per in load_terms],
        diag=diag,
    )
    (case_dir / "metrics.json").write_text(json.dumps(metrics, indent=2), encoding="utf-8")
    lines = [
        f"Bonded assembly FEA — {CASES[case]['title']}",
        f"Case: {case}   env: {CASES[case]['env']}",
        f"STEP: {step}   solids after fuse: {n_solids}",
        f"Rail stubs 2 x (30x30 wall {WALL} equivalent tube, L={RAIL_L}) at y=+/-{RAIL_Y}, z={RAIL_Z:.2f}; "
        f"ends x=+/-{RAIL_L/2:.0f} fixed ({len(fixed)} nodes)",
        f"Clamps: low_clamp() at y=+/-{WHEEL_Y}, z=+{AXLE_Z} (-y side rotated 180 deg about z)",
        (f"Tie: 30x30 tube L={TIE_LEN} at z={TIE_Z:.2f}, bonded to clamp inner faces |y|={CLAMP_IN_Y}" if case == "tie" else
         f"H cross: 30x30 tube L={CROSS_LEN} at z={RAIL_Z:.2f}, ends bonded to rail inner faces; tongue top bonded to cross bottom" if case == "hcross" else
         "No reinforcement (two independent halves)"),
        f"Load: {LOAD:.0f} N {args.load_direction} on each axle seat, eccentricity {ECC:g} mm outboard "
        f"(inboard/outboard couple as in run_clamp_fea)",
        "Load terms [set, nodes, total N, per-node N]:",
        *[f"  {n}: {len(nodes)}, {tot:.6f}, {per:.9f}" for n, nodes, tot, per in load_terms],
        f"Mesh: target {args.mesh_size} mm (min {args.mesh_min}), C3D10; elements {len(element_ids)} "
        f"(clamp {len(e_clamp)}, aluminium {len(e_alu)}), nodes {n_nodes}",
        f"Materials: clamps {MATERIALS['petg']['label']} E={MATERIALS['petg']['e']:.0f} nu={MATERIALS['petg']['nu']}; "
        f"profile {MATERIALS['al']['label']} E={MATERIALS['al']['e']:.0f} nu={MATERIALS['al']['nu']}",
        f"Solve: {solve_s:.0f} s (OMP_NUM_THREADS={env['OMP_NUM_THREADS']})",
        "Seat camber (dUz/dy_out, + = outboard rises):",
        *[f"  {side}: {d['camber_mrad']:+.4f} mrad  (n={d['n']}, Uy_out {d['uy_out']:+.4f} mm, "
          f"Uz mean {d['uz_mean']:+.4f} mm, contact splay {d['splay']:+.4f} mm)" for side, d in sides.items()],
        "Load-path diagnostics (rail bottom at x~0; leg tops z=93.85):",
        *[f"  {side}: rail-stub twist {d['rail_twist_mrad']:+.3f} mrad, rail rise {d['rail_rise_mm']:+.4f} mm, "
          f"leg-top slope {d['legtop_slope_mrad']:+.3f} mrad"
          + (f", cross bottom Uz at tongue tip {d['cross_uz_at_tongue_tip']:+.4f} mm, cross top mid Uz {d['cross_top_mid_uz']:+.4f} mm"
             if case == "hcross" else "") for side, d in diag.items()],
        f"Max |U|: {umax:.6f} mm (clamp region {umax_clamp:.6f})",
        f"Clamp structural stress (clamp nodes > {args.bond_exclude} mm from bonded faces): {clamp_structural:.3f} MPa "
        f"at x={at[0]:+.1f} y={at[1]:+.1f} z={at[2]:+.1f}",
        f"Clamp max von Mises incl. bond edges (do not quote): {clamp_max:.3f} MPa",
        f"Aluminium profile max von Mises: {alu_max:.3f} MPa",
        f"Reaction at fixed rail ends [Fx,Fy,Fz]: [{rf_total[0]:.4f}, {rf_total[1]:.4f}, {rf_total[2]:.4f}]  "
        f"(applied Fz total {2*total_vertical:+.1f})",
        f"Sum of all RF: [{rf_all[0]:.4f}, {rf_all[1]:.4f}, {rf_all[2]:.4f}]",
        "Limits: bonded (no T-nut/bracket/bolt compliance, no preload/contact) -> joint stiffness overestimated; "
        "one C3D10 layer through the 2.15 mm tube wall; equivalent square tube matches 3030 bending I only.",
        f"VTU: {vtu}", f"Preview: {png}",
    ]
    (case_dir / "summary.txt").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print("\n".join(lines))


def render_case(vtu: Path, png: Path, m: dict):
    """metrics 딕셔너리(metrics.json 과 같은 키)로 제목 줄을 만들어 render() 를 부른다."""
    lines = [
        f"Bonded assembly: {m['title']}",
        f"PETG clamps / Al equivalent tube, {m['load_n']:.0f} N {m['load_direction']} x2, eccentricity {m['eccentricity_mm']:g} mm",
        f"clamp structural {m['clamp_structural_mpa']:.1f} MPa (colour clipped), max |U| {m['umax_mm']:.4f} mm",
        f"camber L {m['camber_mrad_left']:+.2f} / R {m['camber_mrad_right']:+.2f} mrad (+ = outboard rises)",
    ]
    render(vtu, png, lines, m["clamp_structural_mpa"], m["umax_mm"])


def render(vtu: Path, png: Path, lines: list[str], s_max: float, u_max: float):
    """run_clamp_fea.render_result 와 같은 VTK 오프스크린 렌더 (pyvista 는 .venv 에 없다).

    두 장을 굽는다: <png> 는 등각, <png stem>_front.png 는 x 축을 따라 본 정면(캠버가 보인다).
    """
    from vtkmodules.vtkCommonCore import vtkLookupTable
    from vtkmodules.vtkFiltersGeneral import vtkWarpVector
    from vtkmodules.vtkFiltersGeometry import vtkDataSetSurfaceFilter
    from vtkmodules.vtkIOImage import vtkPNGWriter
    from vtkmodules.vtkIOXML import vtkXMLUnstructuredGridReader
    from vtkmodules.vtkRenderingAnnotation import vtkScalarBarActor
    from vtkmodules.vtkRenderingCore import (vtkActor, vtkDataSetMapper, vtkRenderWindow, vtkRenderer,
                                             vtkTextActor, vtkWindowToImageFilter)
    import vtkmodules.vtkRenderingOpenGL2  # noqa: F401

    reader = vtkXMLUnstructuredGridReader()
    reader.SetFileName(str(vtu))
    reader.Update()
    grid = reader.GetOutput()
    grid.GetPointData().SetActiveVectors("U")
    warp_scale = min(5000.0, 12.0 / max(u_max, 1e-12))
    warp = vtkWarpVector()
    warp.SetInputData(grid)
    warp.SetScaleFactor(warp_scale)
    surf = vtkDataSetSurfaceFilter()
    surf.SetInputConnection(warp.GetOutputPort())
    lut = vtkLookupTable()
    lut.SetHueRange(0.667, 0.0)
    lut.SetNumberOfTableValues(256)
    lut.Build()
    mapper = vtkDataSetMapper()
    mapper.SetInputConnection(surf.GetOutputPort())
    mapper.SetScalarModeToUsePointFieldData()
    mapper.SelectColorArray("S_Mises")
    mapper.SetScalarRange(0.0, s_max)
    mapper.SetLookupTable(lut)
    actor = vtkActor()
    actor.SetMapper(mapper)
    actor.GetProperty().SetInterpolationToPhong()
    osurf = vtkDataSetSurfaceFilter()
    osurf.SetInputData(grid)
    omap = vtkDataSetMapper()
    omap.SetInputConnection(osurf.GetOutputPort())
    omap.ScalarVisibilityOff()
    oact = vtkActor()
    oact.SetMapper(omap)
    oact.GetProperty().SetRepresentationToWireframe()
    oact.GetProperty().SetColor(0.75, 0.78, 0.82)
    oact.GetProperty().SetOpacity(0.10)
    bar = vtkScalarBarActor()
    bar.SetLookupTable(lut)
    bar.SetTitle("von Mises [MPa]")
    bar.SetNumberOfLabels(6)
    bar.SetWidth(0.07)
    bar.SetHeight(0.5)
    bar.SetPosition(0.91, 0.22)
    b = grid.GetBounds()
    c = ((b[0] + b[1]) / 2, (b[2] + b[3]) / 2, (b[4] + b[5]) / 2)
    span = max(b[1] - b[0], b[3] - b[2], b[5] - b[4])
    views = {
        png: ((c[0] + 1.1 * span, c[1] - 0.9 * span, c[2] + 0.75 * span), 1.5),
        png.with_name(png.stem + "_front.png"): ((c[0] + 3.0 * span, c[1], c[2]), 1.35),
    }
    for out, (pos, zoom) in views.items():
        title = vtkTextActor()
        # ASCII 짧은 줄만 쓴다 — 유니코드 대시가 깨졌고, 창 폭을 넘는 줄은 VTK 가 줄을 흩뜨렸다
        title.SetInput("\n".join([*lines, f"deformation display {warp_scale:.0f}x (wireframe = undeformed)"]))
        title.SetPosition(20, 790)
        title.GetTextProperty().SetFontSize(18)
        title.GetTextProperty().SetColor(0.95, 0.96, 0.98)
        ren = vtkRenderer()
        ren.SetBackground(0.055, 0.065, 0.085)
        ren.AddActor(oact)
        ren.AddActor(actor)
        ren.AddViewProp(bar)
        ren.AddViewProp(title)
        cam = ren.GetActiveCamera()
        cam.SetFocalPoint(*c)
        cam.SetPosition(*pos)
        cam.SetViewUp(0.0, 0.0, 1.0)
        ren.ResetCamera()
        cam.Zoom(zoom)
        ren.ResetCameraClippingRange()
        win = vtkRenderWindow()
        win.SetOffScreenRendering(1)
        win.SetSize(1600, 900)
        win.AddRenderer(ren)
        win.Render()
        cap = vtkWindowToImageFilter()
        cap.SetInput(win)
        cap.SetInputBufferTypeToRGB()
        cap.ReadFrontBufferOff()
        cap.Update()
        w = vtkPNGWriter()
        w.SetFileName(str(out))
        w.SetInputConnection(cap.GetOutputPort())
        w.Write()


# ================================================================ 표
SINGLE_PART = [  # comparison_reinforcement.md (단품, 구멍 벽 강체 고정, 3 mm) — 그대로 인용
    ("보강 없음", 46.160, 1.9067, 45.6),
    ("3030 타이 (아래, 접지 스프링)", 7.576, 0.2627, 17.8),
    ("H 크로스 + wide 선반 v2 (선반 볼트 강체 고정)", 11.704, 0.4477, 58.5),
]


def write_table(cases):
    rows = []
    for c in cases:
        mj = OUT_ROOT / f"h_assembly_{c}" / "metrics.json"
        if not mj.exists():
            rows.append((c, None))
            continue
        rows.append((c, json.loads(mj.read_text(encoding="utf-8"))))
    base = next((m for c, m in rows if c == "none" and m), None)
    out = [
        "# 결합 조립 FEA — 레일 토막 + 좌우 클램프 (+타이 / +H 크로스·혀)",
        "",
        "`tools/fea/h_assembly_fea.py` · PETG 클램프(E 2100, ν 0.38) + 알루미늄 등가 각관(E 69000, 30×30 벽 2.15, I≈3.1e4) · "
        "800 N 상향 × 좌우 · 편심 55 mm 바깥 · 4 mm C3D10 (min 1.2) · 레일 토막(240) 양 끝면 전 노드 고정 · **전부 결합(bonded)**.",
        "",
        "| 케이스 | 요소 | 캠버 L mrad | 캠버 R mrad | 배율 | 최대변위 mm | 배율 | 클램프 구조응력 MPa | 항복 대비 | 프로파일 최대 MPa | 고정단 Fz N |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for c, m in rows:
        if not m:
            out.append(f"| {CASES[c]['label']} | 결과 없음 | | | | | | | | | |")
            continue
        cam = (m["camber_mrad_left"] + m["camber_mrad_right"]) / 2
        cam0 = (base["camber_mrad_left"] + base["camber_mrad_right"]) / 2 if base else None
        r1 = f"{cam0 / cam:.1f}×" if base and abs(cam) > 1e-9 else "—"
        r2 = f"{base['umax_mm'] / m['umax_mm']:.1f}×" if base and m["umax_mm"] > 0 else "—"
        out.append(
            f"| {m['label']} | {m['elements']} | {m['camber_mrad_left']:+.3f} | {m['camber_mrad_right']:+.3f} | {r1} | "
            f"{m['umax_mm']:.4f} | {r2} | {m['clamp_structural_mpa']:.1f} | "
            f"{m['clamp_structural_mpa'] / MATERIALS['petg']['yield'] * 100:.0f}% | {m['alu_max_mpa']:.1f} | "
            f"{m['rf_fixed'][2]:+.1f} |")
    out += [
        "",
        "배율 = 보강 없음 대비 감소 배수 (캠버는 좌우 평균). 캠버 = 시트 노드 Uz 를 바깥 방향 y 로 회귀한 기울기 (좌우 모두 **바깥이 위로 들리면 양수**; "
        "left = −y 쪽, right = +y 쪽). "
        "클램프 구조응력 = 클램프 영역 노드 중 결합면에서 2 mm 밖의 최대 von Mises (결합 모서리는 특이점이라 뺀다). "
        "고정단 Fz 는 레일 끝 고정 노드 반력 합 — 입력 −1600 N 과 맞아야 한다.",
        "",
        "## 단품 결과(구멍 벽 강체 고정, 3 mm, comparison_reinforcement.md)와 나란히",
        "",
        "| 형태 | 단품 캠버 mrad | 단품 최대변위 | 단품 구조응력 | 조립 캠버(평균) | 조립 최대변위 | 조립 클램프 구조응력 |",
        "|---|---:|---:|---:|---:|---:|---:|",
    ]
    for (name, cam_s, u_s, s_s), (c, m) in zip(SINGLE_PART, rows):
        if m:
            cam = (m["camber_mrad_left"] + m["camber_mrad_right"]) / 2
            out.append(f"| {name} → {m['label']} | {cam_s:+.2f} | {u_s:.4f} | {s_s:.1f} | {cam:+.2f} | {m['umax_mm']:.4f} | {m['clamp_structural_mpa']:.1f} |")
        else:
            out.append(f"| {name} | {cam_s:+.2f} | {u_s:.4f} | {s_s:.1f} | 결과 없음 | | |")
    out += [
        "",
        "## 경로 진단 — 캠버가 어디서 오나 (right 쪽, x≈0 단면)",
        "",
        "| 케이스 | 레일 토막 비틀림 mrad | 레일 밑면 평균 상승 mm | 다리 윗면 기울기 mrad | 시트 캠버 mrad | 클램프 몸체 몫 (캠버−다리윗면) mrad | 크로스 밑면 Uz @혀 끝 mm | 크로스 윗면 중앙 Uz mm |",
        "|---|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for c, m in rows:
        if not m or "diag" not in m:
            continue
        d = m["diag"]["right"]
        cam = m["camber_mrad_right"]
        fmt = lambda k: f"{d[k]:+.4f}" if k in d else "—"  # noqa: E731
        out.append(f"| {m['label']} | {d['rail_twist_mrad']:+.3f} | {d['rail_rise_mm']:+.4f} | {d['legtop_slope_mrad']:+.3f} | "
                   f"{cam:+.3f} | {cam - d['legtop_slope_mrad']:+.3f} | {fmt('cross_uz_at_tongue_tip')} | {fmt('cross_top_mid_uz')} |")
    out += [
        "",
        "레일 토막 비틀림 = 레일 밑면 안쪽 모서리(|y|=195)와 바깥 모서리(225)의 Uz 차 / 30. 다리 윗면 기울기 = 다리 윗면(z=93.85, 레일에 결합) Uz 의 y 회귀. "
        "둘의 차이가 결합면 밖 레일 밑면의 국부 휨이고, 캠버에서 다리 윗면 기울기를 뺀 것이 클램프 몸체(다리+시트) 자체의 변형 몫이다.",
    ]
    # 메쉬 민감도 — h_assembly_<case>_h<size>mm 폴더가 있으면 4 mm 와 나란히
    conv = []
    for c, m in rows:
        if not m:
            continue
        for d in sorted(OUT_ROOT.glob(f"h_assembly_{c}_h*mm")):
            mj = d / "metrics.json"
            if mj.exists():
                conv.append((m, json.loads(mj.read_text(encoding="utf-8"))))
    if conv:
        out += [
            "",
            "## 메쉬 민감도",
            "",
            "| 케이스 | 격자 mm | 요소 | 캠버 R mrad | 최대변위 mm | 클램프 구조응력 MPa | 프로파일 최대 MPa |",
            "|---|---:|---:|---:|---:|---:|---:|",
        ]
        for m4, mo in conv:
            for m in (m4, mo):
                out.append(f"| {m['label']} | {m['mesh_size']:g} | {m['elements']} | {m['camber_mrad_right']:+.3f} | "
                           f"{m['umax_mm']:.4f} | {m['clamp_structural_mpa']:.1f} | {m['alu_max_mpa']:.1f} |")
            dc = (mo["camber_mrad_right"] / m4["camber_mrad_right"] - 1) * 100
            ds = (mo["clamp_structural_mpa"] / m4["clamp_structural_mpa"] - 1) * 100
            du = (mo["umax_mm"] / m4["umax_mm"] - 1) * 100
            out.append(f"| ↳ {mo['mesh_size']:g} mm 이 4 mm 대비 | | | {dc:+.1f}% | {du:+.1f}% | {ds:+.1f}% | |")
    out += [
        "",
        "## 해석 (2026-09-16 실행, 4 mm 기준)",
        "",
        "- **결합 조립의 절대값은 단품 표와 비교하면 안 된다.** 보강 없음이 단품 46.16 → 결합 10.75 mrad 로 4.3× 작아졌는데, "
        "이는 다리 윗면 24×30 이 레일 밑면에 통째로 용접된 셈이라 볼트 구멍 벽만 잡은 단품보다 조인트가 훨씬 뻣뻣하기 때문이다. "
        "같은 결합 조건 안에서의 **상대 순위**만 읽는다.",
        "- **3030 타이(아래)**: 캠버 6.62 mrad (1.6×), 최대변위 0.241, 클램프 구조응력 16.4 MPa. 단품의 접지 스프링 모델(7.58 / 0.263 / 17.8)과 "
        "거의 같다 — 타이는 좌우 클램프를 직접 이어 벌어짐을 잡는 경로라 프레임 쪽 경계조건에 둔감하다. 다만 결합이라 타이 끝이 모멘트도 "
        "받아 살짝 휜다(타이 최대 19.8 MPa, 끝단 근처). 실제 코너브래킷은 그 모멘트를 못 받으니 실물은 이보다 조금 무르다.",
        "- **H 크로스 + 혀**: 캠버 6.25 mrad (1.7×), 최대변위 0.253, 클램프 구조응력 29.4 MPa (PETG 항복의 65%). 크로스 양 끝을 레일에 "
        "**용접한 상한**에서조차 타이와 같은 수준이다. 이유는 진단표에 있다 — 좌우 혀가 크로스 양 끝을 동시에 아래로 당기면서 모멘트를 "
        "걸어 **크로스 자체가 굽는다** (혀 끝 아래 −0.067 mm, 크로스 윗면 중앙 −0.112 mm). 즉 H 경로는 혀 → 크로스 굽힘 → 레일 비틀림의 "
        "직렬이고, 여기에 실물의 코너브래킷·스프링너트 조인트(볼트당 23,000~130,000 N/mm 추정)가 더 직렬로 들어가면 결합값보다 확실히 "
        "무르다. 타이 경로에는 그 프레임 조인트가 없다.",
        "- 혀의 **응력 29.4 MPa 은 혀 뿌리 포켓 모서리**(x=±10, 클램프 안쪽면, z=몸체 윗면)에 몰린다 — 날카로운 오목 모서리라 3 mm 격자에서 "
        "31.2 (+6%) 로 오르는 메쉬 민감 값이다. 채택하려면 그 모서리에 R 을 줘야 한다. 타이 케이스의 16.4 MPa 은 시트 바깥 모서리(하중 "
        "부여 모서리)로 두 격자에서 ±1% 다.",
        "- 캠버 자체는 두 격자에서 0.1~0.7% 차이로 수렴했다 (메쉬 민감도 표). 고정단 반력 Fz 는 세 케이스 모두 −1600.0 N 으로 입력과 일치.",
        "- 프로파일 응력(30~56 MPa)은 레일 토막 결합 모서리·고정단 값이고 6063-T5 항복 145 이하다. 등가 각관이라 T슬롯 립 국부 응력은 없다.",
        "",
        "## 모델 한계",
        "",
        "- **결합(bonded)** 이라 브라켓·T너트·스프링너트·볼트 예압·접촉이 전부 없다. 크로스 양 끝의 코너브라켓 조인트와 혀 볼트가 "
        "강체 용접처럼 들어가므로 H 크로스 경로의 강성은 **상한**이다 (T슬롯 조인트 축강성 추정 23,000~130,000 N/mm 가 직렬로 빠져 있다). "
        "타이도 브래킷 없이 클램프 안쪽면에 직접 붙였다 (373.4 mm).",
        "- 단품 모델과 달리 레일 볼트가 없다 — 다리 윗면 전체가 레일 밑면(레일 발자국 30 mm)에 결합돼 있다. 레일 볼트 인발·다리 윗면 지압 분리를 못 본다.",
        "- 3030 은 등가 각관(굽힘 I 만 맞춤, 단면적 240 < 실물 406, T슬롯 개단면 비틀림 저하 없음)이고 벽 2.15 에 C3D10 한 층이다 "
        "(4 mm 격자에서 두 층을 만들려면 ≈1 mm 가 필요해 요소가 수백만이 된다). 프로파일은 PETG 보다 33× 뻣뻣해 이 오차가 결과를 좌우하지 않는다.",
        "- 레일 토막 240 양 끝 완전 고정 = 실제 760 프레임보다 뻣뻣하다. 좌우 대칭 하중만 봤다 (양 바퀴 동시 상향).",
        "- 선형 정적·등방성 PETG 근사. 출력 방향/층간 접착/피로 없음.",
        "",
        "실행: `.venv/Scripts/python.exe tools/fea/h_assembly_fea.py --case all --mesh-size 4 --mesh-min 1.2 --load 800 --eccentricity 55 --load-direction upward`",
        "결과 폴더: `models/hoverboard_mount/output/fea/h_assembly_<case>/` (summary.txt, metrics.json, h_assembly.inp/.frd/.vtu, "
        "stress_deformed_800N.png 등각, stress_deformed_800N_front.png 정면). 다른 격자는 `h_assembly_<case>_h<size>mm/`. "
        "조립 STEP: `models/hoverboard_mount/output/step/h_assembly_<case>.step`.",
    ]
    TABLE.write_text("\n".join(out) + "\n", encoding="utf-8")
    print(f"table -> {TABLE}")


# ================================================================ 부모
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--case", default="all", choices=["all", *CASES])
    ap.add_argument("--mesh-size", type=float, default=4.0)
    ap.add_argument("--mesh-min", type=float, default=1.2)
    ap.add_argument("--load", type=float, default=800.0)
    ap.add_argument("--eccentricity", type=float, default=55.0)
    ap.add_argument("--load-direction", choices=("upward", "downward"), default="upward")
    ap.add_argument("--bond-exclude", type=float, default=2.0)
    ap.add_argument("--threads", type=int, default=4)
    ap.add_argument("--child", action="store_true", help=argparse.SUPPRESS)
    ap.add_argument("--table-only", action="store_true")
    ap.add_argument("--render-only", action="store_true", help="기존 metrics.json/VTU 로 그림만 다시 굽는다")
    args = ap.parse_args()
    if args.child:
        child_main(args)
        return
    cases = list(CASES) if args.case == "all" else [args.case]
    if args.render_only:
        for c in cases:
            d = OUT_ROOT / f"h_assembly_{c}"
            m = json.loads((d / "metrics.json").read_text(encoding="utf-8"))
            render_case(Path(m["vtu"]), Path(m["png"]), m)
            print(f"rendered {m['png']}")
        write_table(list(CASES))
        return
    if not args.table_only:
        for c in cases:
            cmd = [str(PY), str(Path(__file__).resolve()), "--child", "--case", c,
                   "--mesh-size", str(args.mesh_size), "--mesh-min", str(args.mesh_min),
                   "--load", str(args.load), "--eccentricity", str(args.eccentricity),
                   "--load-direction", args.load_direction, "--bond-exclude", str(args.bond_exclude),
                   "--threads", str(args.threads)]
            print(f"=== case {c}: {' '.join(cmd[2:])}  env={CASES[c]['env']}", flush=True)
            r = subprocess.run(cmd, env=case_env(c), cwd=ROOT)
            if r.returncode != 0:
                print(f"!!! case {c} failed (exit {r.returncode})", flush=True)
    write_table(list(CASES))


if __name__ == "__main__":
    main()
