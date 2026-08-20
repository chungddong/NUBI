"""Open-source FEA pipeline for the Design-D and legacy U-top wheel mounts.

Pipeline
--------
STEP -> Gmsh (quadratic tetra mesh) -> CalculiX (linear static) ->
ccx2paraview (VTU for ParaView).

Each model isolates one AL6061-T6 mount. Rail-fastener hole walls are treated
as perfectly rigid supports and the wheel load is distributed over nodes on
the axle-seat surface. It is suitable for finding likely stress concentrations
and for same-condition comparisons; it is not a final bolted-contact model.
"""

from __future__ import annotations

import argparse
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys


HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
TOOLS_PY = ROOT / "tools" / "fea" / "python"
GMSH_PY = ROOT / "tools" / "fea" / "gmsh_pyenv" / "Lib" / "site-packages"
for dependency_path in (TOOLS_PY, GMSH_PY):
    if str(dependency_path) not in sys.path:
        sys.path.insert(0, str(dependency_path))
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import gmsh  # noqa: E402
import meshio  # noqa: E402
import numpy as np  # noqa: E402
from ccx2paraview import Converter  # noqa: E402

import low_clamp as LC  # noqa: E402
import parameters as P  # noqa: E402


DESIGN_D_STEP = HERE / "output" / "step" / "low_clamp_design_d.step"
LEGACY_U_STEP = ROOT / "exports" / "cnc" / "휠마운트_cnc_M6.step"
LEGACY_U_NO_GUSSET_STEP = (
    HERE / "output" / "step" / "legacy_u_bracket_no_gussets_open_inboard.step"
)
OUT_ROOT = HERE / "output" / "fea"
CCX = (
    ROOT
    / "tools"
    / "fea"
    / "calculix"
    / "calculix_2.23_4win"
    / "ccx_static.exe"
)
MATERIALS = {
    "al6061": {"name": "AL6061_T6", "label": "AL6061-T6", "e": 68_900.0,
                "nu": 0.33, "yield": 276.0},
    # Isotropic screening approximation only. Printed orientation, infill and
    # temperature can reduce stiffness/strength substantially.
    "petg": {"name": "PETG_ISOTROPIC", "label": "PETG isotropic estimate", "e": 2_100.0,
             "nu": 0.38, "yield": 45.0},
}

MODEL_INFO = {
    "design_d": {
        "step": DESIGN_D_STEP,
        "title": "Design-D lower clamp",
        "case_prefix": "clamp",
        "job": "design_d_lower_clamp",
        "support": ("six bottom/side rail-fastener hole walls" if P.LOW_SIDE
                    else "two bottom rail-fastener hole walls"),
        "seat_center_y": LC.CLAMP_CY,
    },
    "legacy_u": {
        "step": LEGACY_U_STEP,
        "title": "Legacy long U-top bracket",
        "case_prefix": "legacy_u",
        "job": "legacy_u_bracket",
        "support": "four upper/lower U-arm rail-fastener hole walls",
        "seat_center_y": -55.0,
    },
    "legacy_u_nogusset": {
        "step": LEGACY_U_NO_GUSSET_STEP,
        "title": "As-failed legacy U-top bracket (no gussets, opening inboard)",
        "case_prefix": "legacy_u_nogusset",
        "job": "legacy_u_nogusset_bracket",
        "support": "four upper/lower U-arm rail-fastener hole walls",
        "seat_center_y": -55.0,
    },
}


def chunks(values: list[int], width: int = 16):
    for start in range(0, len(values), width):
        yield values[start : start + width]


def write_set(stream, keyword: str, name: str, values: list[int]):
    stream.write(f"*{keyword},{keyword}={name}\n")
    for row in chunks(sorted(values)):
        stream.write(",".join(str(v) for v in row) + "\n")


def surface_node_tags() -> set[int]:
    result: set[int] = set()
    for _, surface_tag in gmsh.model.getEntities(2):
        tags, _, _ = gmsh.model.mesh.getNodes(2, surface_tag, includeBoundary=True)
        result.update(int(tag) for tag in tags)
    return result


def classify_nodes(mesh_size: float, model: str):
    tags, xyz, _ = gmsh.model.mesh.getNodes()
    coords = {
        int(tag): (float(xyz[3 * i]), float(xyz[3 * i + 1]), float(xyz[3 * i + 2]))
        for i, tag in enumerate(tags)
    }
    surface = surface_node_tags()
    tol = max(0.12, mesh_size * 0.08)

    load_nodes: list[int] = []
    fixed_nodes: set[int] = set()
    if model == "design_d":
        # Wheel axle seat: cylindrical cut with its axis parallel to global Y.
        seat_r = P.AXLE_DIAMETER / 2 + P.SADDLE_SEAT_FIT
        for tag in surface:
            x, y, z = coords[tag]
            radial = math.hypot(x, z)
            if (
                abs(radial - seat_r) <= tol
                and z <= 0.15
                and LC.CLAMP_CY - P.LOW_D / 2 - tol
                <= y
                <= LC.CLAMP_CY + P.LOW_D / 2 + tol
            ):
                load_nodes.append(tag)

        # Rigid-bolt approximation at two bottom and four side rail fasteners.
        hole_r = P.TNUT_BOLT * 0.55
        cbore_top = LC.LOW_BOT + P.LOW_CBORE[1]
        for tag in surface:
            x, y, z = coords[tag]
            for sx in (-1, 1):
                if (
                    abs(
                        math.hypot(x - sx * P.LOW_POST_X, y - LC.CLAMP_CY)
                        - hole_r
                    )
                    <= tol
                    and cbore_top - tol <= z <= LC.RAIL_BOT + tol
                ):
                    fixed_nodes.add(tag)

                # 옆면 구멍은 플랜지가 있을 때만 존재한다. LOW_SIDE 를 끄면
                # 이 구멍이 형상에 없으므로 고정 노드를 찾을 수 없다 — 조건을
                # 안 걸면 아래 개수 검사에서 엉뚱하게 죽거나, 더 나쁘게는
                # 바닥 구멍만으로 계속 돌면서 예전과 같은 경계조건인 척한다.
                if P.LOW_SIDE and (
                    abs(math.hypot(x - sx * P.LOW_POST_X, z - LC.BOLT_Z) - hole_r)
                    <= tol
                    and abs(y - LC.CLAMP_CY) >= LC.FLANGE_Y - tol
                ):
                    fixed_nodes.add(tag)
    elif model in {"legacy_u", "legacy_u_nogusset"}:
        # Original wheel_mount.py coordinate system: axle centre z=79.35 mm,
        # seat centre y=-55 mm, and the 3030 rail centre y=-50 mm.
        axle_z = 79.35
        seat_y = -55.0
        seat_r = 8.15
        for tag in surface:
            x, y, z = coords[tag]
            if (
                abs(math.hypot(x, z - axle_z) - seat_r) <= tol
                and z <= axle_z + 0.15
                and -75.0 - tol <= y <= -35.0 + tol
            ):
                load_nodes.append(tag)

        # Four M6 rail holes pass vertically through the lower and upper arms
        # of the U. Fixing their cylindrical walls matches the Design-D rigid
        # bolt-wall comparison while preserving the long cantilever between
        # the rail and axle.
        hole_r = 3.3
        for tag in surface:
            x, y, z = coords[tag]
            for sx in (-1, 1):
                for zc in (165.3, 205.7):
                    if (
                        abs(math.hypot(x - sx * 15.0, y + 50.0) - hole_r) <= tol
                        and zc - 6.0 - tol <= z <= zc + 6.0 + tol
                    ):
                        fixed_nodes.add(tag)
    else:
        raise ValueError(f"Unknown model: {model}")

    if len(load_nodes) < 12:
        raise RuntimeError(f"Too few axle-seat load nodes: {len(load_nodes)}")
    # 액슬 타이(전산볼트) 탭 벽 — 여기에 Y방향 스프링을 건다.
    tie_nodes: set[int] = set()
    if model == "design_d" and getattr(P, "TIE", False):
        tap_r = (P.TIE_ROD - 1.2) / 2
        y0 = LC.CLAMP_CY - P.LOW_D / 2
        for tag in surface:
            x, y, z = coords[tag]
            for sx in ((-1, 1) if P.TIE_N == 2 else (0,)):
                if (abs(math.hypot(x - sx * P.TIE_X, z - P.TIE_Z) - tap_r) <= tol
                        and y0 - tol <= y <= y0 + P.TIE_ENGAGE + tol):
                    tie_nodes.add(tag)
        tie_nodes -= fixed_nodes

    # 구멍 개수가 모델마다 다르다 (Design-D 는 플랜지 유무로 6 또는 2).
    min_fixed = 18 if (model != "design_d" or P.LOW_SIDE) else 6
    if len(fixed_nodes) < min_fixed:
        raise RuntimeError(
            f"Too few rail-fastener support nodes: {len(fixed_nodes)} < {min_fixed}")
    if set(load_nodes) & fixed_nodes:
        raise RuntimeError("Load and support node sets overlap")
    return coords, sorted(fixed_nodes), sorted(load_nodes), sorted(tie_nodes)


def create_mesh(case_dir: Path, mesh_size: float, model: str):
    raw_inp = case_dir / "clamp_mesh_raw.inp"
    msh = case_dir / "clamp_mesh.msh"
    gmsh.initialize()
    try:
        gmsh.option.setNumber("General.Terminal", 1)
        gmsh.option.setNumber("General.Verbosity", 2)
        info = MODEL_INFO[model]
        gmsh.model.add(str(info["job"]))
        imported = gmsh.model.occ.importShapes(str(info["step"]))
        gmsh.model.occ.synchronize()
        volumes = [tag for dim, tag in imported if dim == 3]
        if not volumes:
            volumes = [tag for _, tag in gmsh.model.getEntities(3)]
        if len(volumes) != 1:
            raise RuntimeError(f"Expected one solid volume, found {len(volumes)}")

        # Export only the 3D continuum mesh to Abaqus/CalculiX.  Without a
        # physical volume Gmsh also writes its 1D/2D boundary elements, which
        # CalculiX interprets as beams/shells with missing section properties.
        volume_group = gmsh.model.addPhysicalGroup(3, volumes)
        gmsh.model.setPhysicalName(3, volume_group, "VOLUME_ONLY")
        gmsh.option.setNumber("Mesh.SaveAll", 0)

        gmsh.option.setNumber("Mesh.MeshSizeMin", mesh_size * 0.55)
        gmsh.option.setNumber("Mesh.MeshSizeMax", mesh_size)
        gmsh.option.setNumber("Mesh.MeshSizeFromCurvature", 14)
        gmsh.option.setNumber("Mesh.Algorithm3D", 10)  # HXT tetrahedral mesher
        gmsh.model.mesh.generate(3)
        gmsh.model.mesh.optimize("Netgen")
        gmsh.model.mesh.setOrder(2)
        gmsh.model.mesh.optimize("HighOrderElastic", niter=5)
        gmsh.model.mesh.optimize("HighOrder", niter=5)

        coords, fixed_nodes, load_nodes, tie_nodes = classify_nodes(mesh_size, model)
        element_types, element_tags, _ = gmsh.model.mesh.getElements(3)
        element_ids: list[int] = []
        for element_type, tags in zip(element_types, element_tags):
            name = gmsh.model.mesh.getElementProperties(element_type)[0]
            if "Tetrahedron 10" not in name:
                raise RuntimeError(f"Unexpected solid element type: {name}")
            element_ids.extend(int(tag) for tag in tags)

        gmsh.write(str(msh))
        gmsh.write(str(raw_inp))
        return raw_inp, coords, fixed_nodes, load_nodes, tie_nodes, element_ids
    finally:
        gmsh.finalize()


def build_solver_deck(
    raw_inp: Path,
    job_inp: Path,
    coords: dict[int, tuple[float, float, float]],
    fixed_nodes: list[int],
    load_nodes: list[int],
    tie_nodes: list[int],
    element_ids: list[int],
    load_n: float,
    eccentricity_mm: float,
    seat_center_y: float,
    material: dict[str, float | str],
    load_sign: float,
):
    shutil.copyfile(raw_inp, job_inp)
    total_vertical = load_sign * load_n
    load_terms: list[tuple[str, list[int], float, float]] = []
    if abs(eccentricity_mm) < 1e-9:
        load_terms.append(
            (
                "AXLE_SEAT_LOAD",
                load_nodes,
                total_vertical,
                total_vertical / len(load_nodes),
            )
        )
    else:
        inboard = [tag for tag in load_nodes if coords[tag][1] < seat_center_y]
        outboard = [tag for tag in load_nodes if coords[tag][1] >= seat_center_y]
        if len(inboard) < 6 or len(outboard) < 6:
            raise RuntimeError("Too few nodes to create the eccentric axle load couple")
        yi = sum(coords[tag][1] for tag in inboard) / len(inboard)
        yo = sum(coords[tag][1] for tag in outboard) / len(outboard)
        target_y = seat_center_y + eccentricity_mm
        # Two vertical resultants on the available seat reproduce a remote
        # vertical force whose line of action passes through target_y.
        force_out = total_vertical * (target_y - yi) / (yo - yi)
        force_in = total_vertical - force_out
        load_terms.extend(
            [
                ("AXLE_SEAT_OUTBOARD", outboard, force_out, force_out / len(outboard)),
                ("AXLE_SEAT_INBOARD", inboard, force_in, force_in / len(inboard)),
            ]
        )
    with job_inp.open("a", encoding="ascii", newline="\n") as stream:
        stream.write("\n** --- wheel mount analysis additions ---\n")
        write_set(stream, "ELSET", "EALL", element_ids)
        write_set(stream, "NSET", "FIXED_BOLT_WALLS", fixed_nodes)
        write_set(stream, "NSET", "AXLE_SEAT_LOAD", load_nodes)
        for set_name, nodes, _, _ in load_terms:
            if set_name != "AXLE_SEAT_LOAD":
                write_set(stream, "NSET", set_name, nodes)
        stream.write(f"*MATERIAL,NAME={material['name']}\n")
        stream.write("*ELASTIC\n")
        stream.write(f"{float(material['e']):.1f},{float(material['nu']):.5f}\n")
        stream.write(f"*SOLID SECTION,ELSET=EALL,MATERIAL={material['name']}\n")
        stream.write("*BOUNDARY\n")
        stream.write("FIXED_BOLT_WALLS,1,3,0.0\n")
        if tie_nodes:
            # 액슬 타이(전산볼트)를 **접지 스프링**으로 넣는다. 강체로 묶으면
            # 실제보다 좋게 나온다 — 나사봉은 12,690 N/mm 짜리 탄성체다.
            # 좌우 대칭이라 타이 중점은 안 움직이므로, 클램프 하나가 느끼는
            # 등가 접지강성은 축강성의 2배다 (양끝이 각각 delta 씩 벌어진다).
            _At = {6.0: 20.1, 8.0: 36.6, 10.0: 58.0, 12.0: 84.3}[P.TIE_ROD]
            _L = 2 * (210.0 - P.LOW_D / 2 + P.TIE_ENGAGE)
            _k = 2.0 * 206000.0 * _At * P.TIE_N / _L / len(tie_nodes)  # 강철
            _e0 = max(element_ids) + 1
            stream.write("*ELEMENT,TYPE=SPRING1,ELSET=ETIE\n")
            for _i, _n in enumerate(tie_nodes):
                stream.write(f"{_e0 + _i},{_n}\n")
            stream.write("*SPRING,ELSET=ETIE\n2\n")
            stream.write(f"{_k:.6g}\n")
        stream.write("*STEP,NLGEOM=NO\n")
        stream.write("*STATIC\n")
        stream.write("*CLOAD\n")
        for set_name, _, _, force_per_node in load_terms:
            stream.write(f"{set_name},3,{force_per_node:.12g}\n")
        stream.write("*NODE FILE\n")
        stream.write("U,RF\n")
        stream.write("*EL FILE\n")
        stream.write("S,E\n")
        stream.write("*NODE PRINT,NSET=AXLE_SEAT_LOAD,TOTALS=YES\n")
        stream.write("U\n")
        stream.write("*NODE PRINT,NSET=FIXED_BOLT_WALLS,TOTALS=YES\n")
        stream.write("RF\n")
        if tie_nodes:
            write_set(stream, "NSET", "TIE_WALLS", tie_nodes)
            stream.write("*NODE PRINT,NSET=TIE_WALLS,TOTALS=YES\n")
            stream.write("RF\n")
        stream.write("*END STEP\n")
    return load_terms


def run_solver(case_dir: Path, job_name: str):
    if not CCX.exists():
        raise FileNotFoundError(f"CalculiX solver not found: {CCX}")
    process = subprocess.run(
        [str(CCX), "-i", job_name],
        cwd=case_dir,
        text=True,
        capture_output=True,
        check=False,
    )
    (case_dir / "solver_stdout.log").write_text(process.stdout, encoding="utf-8")
    (case_dir / "solver_stderr.log").write_text(process.stderr, encoding="utf-8")
    if process.returncode != 0:
        raise RuntimeError(
            f"CalculiX failed with exit code {process.returncode}; "
            f"see {case_dir / 'solver_stdout.log'}"
        )
    frd = case_dir / f"{job_name}.frd"
    if not frd.exists() or frd.stat().st_size == 0:
        raise RuntimeError("CalculiX finished without a usable FRD result")
    return frd


def convert_and_summarize(frd: Path):
    # Stress/strain tensors are real and symmetric.  ccx2paraview currently
    # calls the general eigensolver and can receive tiny imaginary round-off
    # components, which Python then refuses to sort.  Use the symmetric solver
    # for this conversion without modifying the vendored open-source package.
    original_eigvals = np.linalg.eigvals
    np.linalg.eigvals = lambda tensor: np.linalg.eigvalsh(  # type: ignore[assignment]
        np.asarray(tensor, dtype=float)
    )
    try:
        Converter(str(frd), ["vtu"]).run()
    finally:
        np.linalg.eigvals = original_eigvals  # type: ignore[assignment]
    candidates = sorted(frd.parent.glob(f"{frd.stem}*.vtu"))
    if not candidates:
        raise RuntimeError("ccx2paraview did not create a VTU result")
    vtu = candidates[-1]
    mesh = meshio.read(vtu)
    point_keys = sorted(mesh.point_data)

    max_mises = None
    max_disp = None
    for key, values in mesh.point_data.items():
        key_upper = key.upper()
        if key_upper == "S_MISES":
            max_mises = float(values.max())
        if key_upper in {"U", "DISP", "DISPLACEMENT"} or "DISP" in key_upper:
            if getattr(values, "ndim", 1) == 2 and values.shape[1] >= 3:
                magnitudes = (values[:, :3] ** 2).sum(axis=1) ** 0.5
                max_disp = float(magnitudes.max())
    return vtu, point_keys, max_mises, max_disp


def support_reaction_summary(vtu: Path, model: str):
    """Sum CalculiX nodal reactions by physical rail-fastener location."""
    mesh = meshio.read(vtu)
    points = np.asarray(mesh.points)
    reactions = np.asarray(mesh.point_data["RF"])
    active = np.linalg.norm(reactions, axis=1) > 1e-12
    groups: list[tuple[str, np.ndarray]] = []
    tol = 0.55

    if model == "design_d":
        for sx, side_name in ((-1, "left"), (1, "right")):
            bottom_radial = np.hypot(
                points[:, 0] - sx * P.LOW_POST_X,
                points[:, 1] - LC.CLAMP_CY,
            )
            bottom = (
                active
                & (np.abs(bottom_radial - P.TNUT_BOLT * 0.55) <= tol)
                & (points[:, 2] >= LC.LOW_BOT + P.LOW_CBORE[1] - tol)
                & (points[:, 2] <= LC.RAIL_BOT + tol)
            )
            groups.append((f"bottom_{side_name}", reactions[bottom].sum(axis=0)))
            for sy, face_name in ((-1, "inner"), (1, "outer")):
                side_radial = np.hypot(
                    points[:, 0] - sx * P.LOW_POST_X,
                    points[:, 2] - LC.BOLT_Z,
                )
                side = (
                    active
                    & (np.abs(side_radial - P.TNUT_BOLT * 0.55) <= tol)
                    & (
                        sy * (points[:, 1] - LC.CLAMP_CY)
                        >= LC.FLANGE_Y - tol
                    )
                )
                groups.append(
                    (f"side_{side_name}_{face_name}", reactions[side].sum(axis=0))
                )
    elif model in {"legacy_u", "legacy_u_nogusset"}:
        for sx, side_name in ((-1, "left"), (1, "right")):
            radial = np.hypot(points[:, 0] - sx * 15.0, points[:, 1] + 50.0)
            hole_wall = active & (np.abs(radial - 3.3) <= tol)
            lower = hole_wall & (points[:, 2] >= 159.3) & (points[:, 2] <= 171.3)
            upper = hole_wall & (points[:, 2] >= 199.7) & (points[:, 2] <= 211.7)
            groups.append((f"lower_arm_{side_name}", reactions[lower].sum(axis=0)))
            groups.append((f"upper_arm_{side_name}", reactions[upper].sum(axis=0)))

    total = sum((force for _, force in groups), np.zeros(3))
    return groups, total


def render_result(
    vtu: Path,
    png: Path,
    title_text: str,
    load_n: float,
    load_direction: str,
    max_mises: float,
    max_disp: float,
):
    """Create a ParaView-compatible VTK stress preview without a GUI."""
    from vtkmodules.vtkCommonCore import vtkLookupTable
    from vtkmodules.vtkFiltersGeneral import vtkWarpVector
    from vtkmodules.vtkFiltersGeometry import vtkDataSetSurfaceFilter
    from vtkmodules.vtkIOImage import vtkPNGWriter
    from vtkmodules.vtkIOXML import vtkXMLUnstructuredGridReader
    from vtkmodules.vtkRenderingAnnotation import vtkScalarBarActor
    from vtkmodules.vtkRenderingCore import (
        vtkActor,
        vtkDataSetMapper,
        vtkRenderWindow,
        vtkRenderer,
        vtkTextActor,
        vtkWindowToImageFilter,
    )
    import vtkmodules.vtkRenderingOpenGL2  # noqa: F401

    reader = vtkXMLUnstructuredGridReader()
    reader.SetFileName(str(vtu))
    reader.Update()
    grid = reader.GetOutput()
    grid.GetPointData().SetActiveVectors("U")

    # Exaggerate only the displayed shape, but cap the visible peak movement at
    # about 8 mm so a more flexible design does not fold across itself.
    # Numerical values in the legend and report remain unscaled.
    warp_scale = min(5000.0, 8.0 / max(max_disp, 1e-12))
    warp = vtkWarpVector()
    warp.SetInputData(grid)
    warp.SetScaleFactor(warp_scale)

    deformed_surface = vtkDataSetSurfaceFilter()
    deformed_surface.SetInputConnection(warp.GetOutputPort())

    lut = vtkLookupTable()
    lut.SetHueRange(0.667, 0.0)
    lut.SetNumberOfTableValues(256)
    lut.Build()

    mapper = vtkDataSetMapper()
    mapper.SetInputConnection(deformed_surface.GetOutputPort())
    mapper.SetScalarModeToUsePointFieldData()
    mapper.SelectColorArray("S_Mises")
    mapper.SetScalarRange(0.0, max_mises)
    mapper.SetLookupTable(lut)

    actor = vtkActor()
    actor.SetMapper(mapper)
    actor.GetProperty().SetInterpolationToPhong()

    original_surface = vtkDataSetSurfaceFilter()
    original_surface.SetInputData(grid)
    original_mapper = vtkDataSetMapper()
    original_mapper.SetInputConnection(original_surface.GetOutputPort())
    original_mapper.ScalarVisibilityOff()
    original_actor = vtkActor()
    original_actor.SetMapper(original_mapper)
    original_actor.GetProperty().SetRepresentationToWireframe()
    original_actor.GetProperty().SetColor(0.75, 0.78, 0.82)
    original_actor.GetProperty().SetOpacity(0.18)

    scalar_bar = vtkScalarBarActor()
    scalar_bar.SetLookupTable(lut)
    scalar_bar.SetTitle("von Mises [MPa]")
    scalar_bar.SetNumberOfLabels(6)
    scalar_bar.SetWidth(0.10)
    scalar_bar.SetHeight(0.55)
    scalar_bar.SetPosition(0.86, 0.22)

    title = vtkTextActor()
    title.SetInput(
        f"{title_text} | {load_n:.0f} N {load_direction}\n"
        f"Max stress {max_mises:.3f} MPa | max displacement {max_disp:.6f} mm\n"
        f"Deformation display: {warp_scale:.0f}x (wireframe = undeformed)"
    )
    title.SetPosition(28, 820)
    title.GetTextProperty().SetFontSize(22)
    title.GetTextProperty().SetColor(0.95, 0.96, 0.98)

    renderer = vtkRenderer()
    renderer.SetBackground(0.055, 0.065, 0.085)
    renderer.AddActor(original_actor)
    renderer.AddActor(actor)
    renderer.AddViewProp(scalar_bar)
    renderer.AddViewProp(title)
    camera = renderer.GetActiveCamera()
    bounds = grid.GetBounds()
    center = (
        (bounds[0] + bounds[1]) / 2,
        (bounds[2] + bounds[3]) / 2,
        (bounds[4] + bounds[5]) / 2,
    )
    span = max(bounds[1] - bounds[0], bounds[3] - bounds[2], bounds[5] - bounds[4])
    # Keep world +Z visibly upright. The previous default VTK view-up made a
    # correctly vertical mount look as if the solve had been run lying down.
    camera.SetFocalPoint(*center)
    camera.SetPosition(center[0] + 1.35 * span, center[1] - 1.8 * span,
                       center[2] + 0.85 * span)
    camera.SetViewUp(0.0, 0.0, 1.0)
    renderer.ResetCameraClippingRange()

    window = vtkRenderWindow()
    window.SetOffScreenRendering(1)
    window.SetSize(1400, 900)
    window.AddRenderer(renderer)
    window.Render()

    capture = vtkWindowToImageFilter()
    capture.SetInput(window)
    capture.SetInputBufferTypeToRGBA()
    capture.ReadFrontBufferOff()
    capture.Update()
    writer = vtkPNGWriter()
    writer.SetFileName(str(png))
    writer.SetInputConnection(capture.GetOutputPort())
    writer.Write()


def run_case(
    load_n: float,
    mesh_size: float,
    model: str = "design_d",
    material_key: str = "al6061",
    eccentricity_mm: float = 0.0,
    load_direction: str = "downward",
):
    info = MODEL_INFO[model]
    material = MATERIALS[material_key]
    suffix = ""
    if material_key != "al6061":
        suffix += f"_{material_key}"
    if abs(eccentricity_mm) > 1e-9:
        suffix += f"_e{eccentricity_mm:g}mm"
    if load_direction == "upward":
        suffix += "_up"
    case_name = f"{info['case_prefix']}_{int(load_n)}N_h{mesh_size:g}mm{suffix}"
    case_dir = OUT_ROOT / case_name
    case_dir.mkdir(parents=True, exist_ok=True)
    raw_inp, coords, fixed_nodes, load_nodes, tie_nodes, element_ids = create_mesh(
        case_dir, mesh_size, model
    )
    job_name = str(info["job"])
    job_inp = case_dir / f"{job_name}.inp"
    load_terms = build_solver_deck(
        raw_inp,
        job_inp,
        coords,
        fixed_nodes,
        load_nodes,
        tie_nodes,
        element_ids,
        load_n,
        eccentricity_mm,
        float(info["seat_center_y"]),
        material,
        1.0 if load_direction == "upward" else -1.0,
    )
    frd = run_solver(case_dir, job_name)
    vtu, point_keys, max_mises, max_disp = convert_and_summarize(frd)
    if max_mises is None or max_disp is None:
        raise RuntimeError("Required S_Mises/U fields are missing from the VTU result")
    preview_png = case_dir / f"stress_deformed_{int(load_n)}N.png"
    render_title = str(info["title"])
    if abs(eccentricity_mm) > 1e-9:
        render_title += f" | {material['label']} | eccentricity {eccentricity_mm:g} mm"
    render_result(
        vtu, preview_png, render_title, load_n, load_direction, max_mises, max_disp
    )
    reaction_groups, reaction_total = support_reaction_summary(vtu, model)

    report = case_dir / "summary.txt"
    lines = [
        f"{info['title']} open-source FEA",
        f"Model key: {model}",
        f"STEP: {info['step']}",
        f"Load: {load_n:.1f} N {load_direction} on axle seat",
        f"Load-line eccentricity from seat centre: {eccentricity_mm:.3f} mm",
        f"Applied bending moment about X: {load_n * eccentricity_mm / 1000:.3f} N m",
        f"Mesh target: {mesh_size:.3f} mm, quadratic C3D10 tetrahedra",
        f"Elements: {len(element_ids)}",
        f"Fixed nodes ({info['support']}): {len(fixed_nodes)}",
        (f"Axle tie: M{P.TIE_ROD:.0f} x{P.TIE_N} as grounded springs, "
         f"{len(tie_nodes)} nodes" if tie_nodes else "Axle tie: none"),
        f"Loaded axle-seat nodes: {len(load_nodes)}",
        "Load terms [set, nodes, total N, per-node N]:",
        *[
            f"  {name}: {len(nodes)}, {total:.6f}, {per_node:.9f}"
            for name, nodes, total, per_node in load_terms
        ],
        f"Material: {material['label']}, E={float(material['e']):.0f} MPa, "
        f"nu={float(material['nu']):.3f}",
        f"Reference yield strength: {float(material['yield']):.1f} MPa",
        f"VTU point fields: {', '.join(point_keys)}",
        f"Max von Mises from VTU: {max_mises if max_mises is not None else 'not parsed'} MPa",
        f"Max displacement from VTU: {max_disp if max_disp is not None else 'not parsed'} mm",
        "Support reactions [Fx,Fy,Fz] N:",
        *[
            f"  {name}: [{force[0]:.6f}, {force[1]:.6f}, {force[2]:.6f}]"
            for name, force in reaction_groups
        ],
        f"  total: [{reaction_total[0]:.6f}, {reaction_total[1]:.6f}, {reaction_total[2]:.6f}]",
        f"Boundary model: rigid support on {info['support']}.",
        "Interpretation: comparative linear-static model; no bolt preload/contact/fatigue yet.",
        f"FRD: {frd}",
        f"VTU: {vtu}",
        f"Preview: {preview_png}",
    ]
    report.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print("\n".join(lines))
    return report


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--load", type=float, default=800.0, help="downward axle-seat load [N]")
    parser.add_argument("--mesh-size", type=float, default=4.0, help="target tetra size [mm]")
    parser.add_argument(
        "--model",
        choices=sorted(MODEL_INFO),
        default="design_d",
        help="mount geometry to analyse",
    )
    parser.add_argument(
        "--material",
        choices=sorted(MATERIALS),
        default="al6061",
        help="isotropic material model",
    )
    parser.add_argument(
        "--eccentricity",
        type=float,
        default=0.0,
        help="outboard offset of vertical load line from axle-seat centre [mm]",
    )
    parser.add_argument(
        "--load-direction",
        choices=("downward", "upward"),
        default="downward",
        help="upward represents the road reaction on a wheel with chassis fixed",
    )
    args = parser.parse_args()
    if args.load <= 0 or args.mesh_size <= 0 or args.eccentricity < 0:
        parser.error("load and mesh-size must be positive")
    run_case(
        args.load,
        args.mesh_size,
        args.model,
        args.material,
        args.eccentricity,
        args.load_direction,
    )


if __name__ == "__main__":
    main()
