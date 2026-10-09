"""One OpenFOAM case: mesh (built once per library, shared by every attitude) and a steady RANS run at one
(alpha, beta) with simpleFoam + k-omega SST.

Frame: the surface STL is in the structural FRD frame (x forward, y right, z down), metres. The aircraft stays
fixed and the air comes at it: the far-field velocity in body axes for angle of attack alpha (nose-up positive) and
sideslip beta (wind from the right positive) is  U = -V (cos a cos b, sin b, sin a cos b).

Forces and moments are reported in FRD, in N and N m, moments about the FRD origin (shifted to any CG later).
Wind-axis quantities: drag along the air velocity, lift = Fx sin a - Fz cos a (up positive), side force = Fy.

OpenFOAM is reached through the Homebrew launcher (`openfoam2412 <app> ...`), or the bare apps when they are on
PATH (Linux). `foam_info()` says which.
"""
from __future__ import annotations

import json
import math
import os
import re
import shutil
import subprocess
import time
from datetime import datetime
from pathlib import Path

import numpy as np

RHO = 1.225          # kg/m3, sea level
NU = 1.5e-5          # m2/s

# mesh / solver settings per quality level. Background cell 1 m; surface levels halve it each time
# (level 5 = 31 mm, 6 = 16 mm, 7 = 8 mm). Layers are off below "fine" (wall functions carry the wall).
QUALITY = {
    "quick":    {"surface_levels": (4, 5), "wake_level": 2, "near_level": 3, "layers": 0, "iterations": 400, "bg_cell": 1.0},
    "standard": {"surface_levels": (5, 6), "wake_level": 3, "near_level": 4, "layers": 0, "iterations": 800, "bg_cell": 1.0},
    "fine":     {"surface_levels": (6, 7), "wake_level": 3, "near_level": 4, "layers": 3, "iterations": 1200, "bg_cell": 1.0},
}


# ----------------------------------------------------------------------------------------------- launcher

def _launcher() -> list[str] | None:
    env = os.environ.get("OPENFOAM_CMD")
    if env:
        return env.split()
    for name in ("openfoam2412", "openfoam2406", "openfoam2312", "openfoam"):
        p = shutil.which(name)
        if p:
            return [p]
    if shutil.which("simpleFoam"):
        return []            # apps directly on PATH
    return None


def foam_info() -> dict:
    """{available, launcher, version, mpirun, cores}"""
    lc = _launcher()
    info = {"available": False, "launcher": " ".join(lc) if lc else None, "version": None, "cores": os.cpu_count() or 1}
    if lc is None:
        return info
    try:
        p = subprocess.run(lc + ["bash", "-c", 'echo "$WM_PROJECT_VERSION"; which mpirun'], capture_output=True, text=True, timeout=60)
        lines = [l for l in p.stdout.splitlines() if l.strip()]
        info["version"] = lines[0] if lines else "?"
        info["mpirun"] = bool(len(lines) > 1 and "mpirun" in lines[1])
        info["available"] = p.returncode == 0
    except Exception as e:  # noqa: BLE001
        info["error"] = str(e)
    return info


def foam_run(args: list[str], case: Path, log_name: str, nproc: int = 1, log=None, env_extra: dict | None = None,
             on_line=None) -> int:
    """Run one OpenFOAM app in the case dir (mpirun -np N ... -parallel when nproc > 1). Output -> case/log.<name>."""
    lc = _launcher()
    if lc is None:
        raise RuntimeError("OpenFOAM not found (brew install gerlero/openfoam/openfoam@2412, or set OPENFOAM_CMD)")
    cmd = list(lc)
    if nproc > 1:
        cmd += ["mpirun", "-np", str(nproc), args[0], "-parallel"] + list(args[1:])
    else:
        cmd += list(args)
    cmd += ["-case", str(Path(case).resolve())]
    env = dict(os.environ)
    env.setdefault("OMPI_MCA_btl_vader_single_copy_mechanism", "none")
    env.setdefault("FOAM_SIGFPE", "")
    if env_extra:
        env.update(env_extra)
    logf = case / f"log.{log_name}"
    if log:
        log(f"[cfd] {' '.join(args)}" + (f" (np {nproc})" if nproc > 1 else ""))
    with open(logf, "w") as f:
        # stdin from /dev/null: mpirun reads the terminal otherwise and a background worker gets SIGTTIN (stopped, state T)
        p = subprocess.Popen(cmd, cwd=str(case), stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=env, bufsize=1)
        _running[case] = p
        try:
            for line in p.stdout:
                f.write(line)
                if on_line:
                    on_line(line)
            p.wait()
        finally:
            _running.pop(case, None)
    return p.returncode


_running: dict[Path, subprocess.Popen] = {}


def kill_running():
    for p in list(_running.values()):
        try:
            p.terminate()
        except Exception:  # noqa: BLE001
            pass


# ----------------------------------------------------------------------------------------------- dictionaries

HEADER = """FoamFile
{{
    version     2.0;
    format      ascii;
    class       {cls};
    object      {obj};
}}
"""


def _dict(cls, obj, body):
    return HEADER.format(cls=cls, obj=obj) + body


def freestream_velocity(V: float, alpha_deg: float, beta_deg: float) -> np.ndarray:
    a, b = math.radians(alpha_deg), math.radians(beta_deg)
    return -V * np.array([math.cos(a) * math.cos(b), math.sin(b), math.sin(a) * math.cos(b)])


def write_mesh_dicts(case: Path, bounds: np.ndarray, q: dict, nproc: int):
    """blockMeshDict, snappyHexMeshDict, decomposeParDict, meshQualityDict for the shared mesh."""
    lo, hi = np.asarray(bounds[0]), np.asarray(bounds[1])
    L = float(np.max(hi - lo))
    c = (lo + hi) / 2
    cell = q["bg_cell"]
    # flow runs toward -x: long box downstream (-x), shorter upstream (+x)
    xmin, xmax = c[0] - 10 * L, c[0] + 6 * L
    ymin, ymax = c[1] - 6 * L, c[1] + 6 * L
    zmin, zmax = c[2] - 6 * L, c[2] + 6 * L
    nx, ny, nz = [max(8, int(round((hi_ - lo_) / cell))) for lo_, hi_ in ((xmin, xmax), (ymin, ymax), (zmin, zmax))]
    bm = f"""
scale 1;
vertices
(
    ({xmin:.3f} {ymin:.3f} {zmin:.3f}) ({xmax:.3f} {ymin:.3f} {zmin:.3f}) ({xmax:.3f} {ymax:.3f} {zmin:.3f}) ({xmin:.3f} {ymax:.3f} {zmin:.3f})
    ({xmin:.3f} {ymin:.3f} {zmax:.3f}) ({xmax:.3f} {ymin:.3f} {zmax:.3f}) ({xmax:.3f} {ymax:.3f} {zmax:.3f}) ({xmin:.3f} {ymax:.3f} {zmax:.3f})
);
blocks ( hex (0 1 2 3 4 5 6 7) ({nx} {ny} {nz}) simpleGrading (1 1 1) );
edges ();
boundary
(
    farfield
    {{
        type patch;
        faces ( (0 4 7 3) (1 2 6 5) (0 1 5 4) (3 7 6 2) (0 3 2 1) (4 5 6 7) );
    }}
);
mergePatchPairs ();
"""
    (case / "system" / "blockMeshDict").write_text(_dict("dictionary", "blockMeshDict", bm))
    s0, s1 = q["surface_levels"]
    pad_near = 0.35 * L
    pad_wake = 0.6 * L
    loc = (xmax - 0.5 * L + 0.0137, c[1] + 0.0371, c[2] + 0.0413)
    layers = q["layers"]
    shm = f"""
castellatedMesh true;
snap            true;
addLayers       {"true" if layers > 0 else "false"};

geometry
{{
    foil.stl {{ type triSurfaceMesh; name foil; }}
    near {{ type searchableBox; min ({lo[0] - pad_near:.3f} {lo[1] - pad_near:.3f} {lo[2] - pad_near:.3f}); max ({hi[0] + pad_near:.3f} {hi[1] + pad_near:.3f} {hi[2] + pad_near:.3f}); }}
    wake {{ type searchableBox; min ({lo[0] - 2.5 * L:.3f} {lo[1] - pad_wake:.3f} {lo[2] - pad_wake:.3f}); max ({hi[0] + pad_wake:.3f} {hi[1] + pad_wake:.3f} {hi[2] + pad_wake:.3f}); }}
}}

castellatedMeshControls
{{
    maxLocalCells 4000000;
    maxGlobalCells 12000000;
    minRefinementCells 10;
    maxLoadUnbalance 0.10;
    nCellsBetweenLevels 3;
    features ();
    refinementSurfaces
    {{
        foil {{ level ({s0} {s1}); patchInfo {{ type wall; inGroups (wall); }} }}
    }}
    resolveFeatureAngle 30;
    refinementRegions
    {{
        near {{ mode inside; levels ((1E15 {q["near_level"]})); }}
        wake {{ mode inside; levels ((1E15 {q["wake_level"]})); }}
    }}
    locationInMesh ({loc[0]:.4f} {loc[1]:.4f} {loc[2]:.4f});
    allowFreeStandingZoneFaces true;
}}

snapControls
{{
    nSmoothPatch 3;
    tolerance 2.0;
    nSolveIter 50;
    nRelaxIter 5;
    nFeatureSnapIter 10;
    implicitFeatureSnap true;
    explicitFeatureSnap false;
    multiRegionFeatureSnap false;
}}

addLayersControls
{{
    relativeSizes true;
    layers {{ foil {{ nSurfaceLayers {max(layers, 1)}; }} }}
    expansionRatio 1.3;
    finalLayerThickness 0.4;
    minThickness 0.1;
    nGrow 0;
    featureAngle 130;
    slipFeatureAngle 30;
    nRelaxIter 5;
    nSmoothSurfaceNormals 1;
    nSmoothNormals 3;
    nSmoothThickness 10;
    maxFaceThicknessRatio 0.5;
    maxThicknessToMedialRatio 0.3;
    minMedialAxisAngle 90;
    nBufferCellsNoExtrude 0;
    nLayerIter 50;
}}

meshQualityControls
{{
    #include "meshQualityDict"
    nSmoothScale 4;
    errorReduction 0.75;
}}

writeFlags ();
mergeTolerance 1e-6;
"""
    (case / "system" / "snappyHexMeshDict").write_text(_dict("dictionary", "snappyHexMeshDict", shm))
    mq = """
maxNonOrtho 65;
maxBoundarySkewness 20;
maxInternalSkewness 4;
maxConcave 80;
minVol 1e-13;
minTetQuality 1e-15;
minArea -1;
minTwist 0.02;
minDeterminant 0.001;
minFaceWeight 0.05;
minVolRatio 0.01;
minTriangleTwist -1;
"""
    (case / "system" / "meshQualityDict").write_text(_dict("dictionary", "meshQualityDict", mq))
    (case / "system" / "decomposeParDict").write_text(_dict("dictionary", "decomposeParDict", f"\nnumberOfSubdomains {nproc};\nmethod scotch;\n"))
    return {"bg_cells": nx * ny * nz, "domain": [[xmin, ymin, zmin], [xmax, ymax, zmax]], "location_in_mesh": list(loc)}


FLOW_SPACING = {"quick": 0.14, "standard": 0.10, "fine": 0.08}   # m, sampling grid for the flow animation


def flow_grid(bounds, spacing: float) -> dict:
    """Regular point grid around the body for the flow animation: upstream +0.6 L, downstream -1.2 L, y symmetric.
    -> {origin, spacing, shape, points Nx3 (x fastest? no: index order i,j,k = x,y,z, C order)}"""
    lo, hi = np.asarray(bounds[0], float), np.asarray(bounds[1], float)
    L = float(np.max(hi - lo))
    xmin, xmax = lo[0] - 1.0 * L, hi[0] + 0.5 * L
    yhalf = max(abs(lo[1]), abs(hi[1])) + 0.25 * L
    zmin, zmax = lo[2] - 0.3 * L, hi[2] + 0.3 * L
    h = float(spacing)
    nx = int(np.ceil((xmax - xmin) / h)) + 1
    ny = 2 * int(np.ceil(yhalf / h)) + 1          # odd count, symmetric about y = 0 (mirroring flips the index)
    nz = int(np.ceil((zmax - zmin) / h)) + 1
    origin = np.array([xmin, -(ny // 2) * h, zmin])
    xs = origin[0] + h * np.arange(nx); ys = origin[1] + h * np.arange(ny); zs = origin[2] + h * np.arange(nz)
    pts = np.stack(np.meshgrid(xs, ys, zs, indexing="ij"), -1).reshape(-1, 3)
    return {"origin": origin.tolist(), "spacing": h, "shape": [nx, ny, nz], "points": pts}


def write_flow_sample_dict(case: Path, grid: dict):
    pts = grid["points"]
    body = "\n".join(f"({x:.4f} {y:.4f} {z:.4f})" for x, y, z in pts)
    (case / "system" / "flowCloud").write_text(_dict("dictionary", "flowCloud", f"""
type            sets;
libs            (sampling);
writeControl    writeTime;
interpolationScheme cellPoint;
setFormat       raw;
fields          (p U);
sets
{{
    flow
    {{
        type    cloud;
        axis    xyz;
        points
        (
{body}
        );
    }}
}}
"""))
    (case / "flow_grid.json").write_text(json.dumps({k: v for k, v in grid.items() if k != "points"}))


def write_case_dicts(case: Path, V: float, alpha: float, beta: float, iterations: int, nproc: int, write_interval: int | None = None,
                     flow: bool = False, flow_include: str = "flowCloud"):
    """controlDict, fvSchemes, fvSolution, transportProperties, turbulenceProperties, 0/ fields."""
    U = freestream_velocity(V, alpha, beta)
    I = 0.01
    k = 1.5 * (I * V) ** 2
    omega = math.sqrt(k) / (0.09 ** 0.25 * 0.1)
    nut = k / omega
    wi = write_interval or iterations
    cd = f"""
application     simpleFoam;
startFrom       latestTime;
startTime       0;
stopAt          endTime;
endTime         {iterations};
deltaT          1;
writeControl    timeStep;
writeInterval   {wi};
purgeWrite      1;
writeFormat     ascii;
writePrecision  8;
writeCompression off;
timeFormat      general;
timePrecision   6;
runTimeModifiable yes;

functions
{{
    forces
    {{
        type            forces;
        libs            (forces);
        patches         (foil);
        rho             rhoInf;
        rhoInf          {RHO};
        CofR            (0 0 0);
        writeControl    timeStep;
        writeInterval   1;
        log             false;
    }}
    {("flowCloud { #include \"" + flow_include + "\" }") if flow else ""}
    cpSurface
    {{
        type            surfaces;
        libs            (sampling);
        writeControl    writeTime;
        surfaceFormat   raw;
        fields          (p);
        interpolationScheme cell;
        surfaces
        (
            foil {{ type patch; patches (foil); interpolate false; }}
        );
    }}
}}
"""
    (case / "system" / "controlDict").write_text(_dict("dictionary", "controlDict", cd))
    fs = """
ddtSchemes      { default steadyState; }
gradSchemes
{
    default         Gauss linear;
    limited         cellLimited Gauss linear 1;
    grad(U)         $limited;
    grad(k)         $limited;
    grad(omega)     $limited;
}
divSchemes
{
    default         none;
    div(phi,U)      bounded Gauss linearUpwindV grad(U);
    div(phi,k)      bounded Gauss limitedLinear 1;
    div(phi,omega)  bounded Gauss limitedLinear 1;
    div((nuEff*dev2(T(grad(U))))) Gauss linear;
}
laplacianSchemes { default Gauss linear limited corrected 0.33; }
interpolationSchemes { default linear; }
snGradSchemes   { default limited corrected 0.33; }
wallDist        { method meshWave; }
"""
    (case / "system" / "fvSchemes").write_text(_dict("dictionary", "fvSchemes", fs))
    fsol = """
solvers
{
    p
    {
        solver          GAMG;
        smoother        GaussSeidel;
        tolerance       1e-7;
        relTol          0.01;
        nPreSweeps      0;
        nPostSweeps     2;
        cacheAgglomeration on;
        agglomerator    faceAreaPair;
        nCellsInCoarsestLevel 100;
        mergeLevels     1;
    }
    "(U|k|omega)"
    {
        solver          smoothSolver;
        smoother        symGaussSeidel;
        tolerance       1e-8;
        relTol          0.1;
        nSweeps         1;
    }
}
SIMPLE
{
    nNonOrthogonalCorrectors 0;
    consistent      yes;
    residualControl { p 1e-5; U 1e-6; "(k|omega)" 1e-6; }
}
potentialFlow { nNonOrthogonalCorrectors 10; }
relaxationFactors
{
    fields    { p 0.5; }
    equations { U 0.7; "(k|omega)" 0.7; }
}
"""
    (case / "system" / "fvSolution").write_text(_dict("dictionary", "fvSolution", fsol))
    (case / "constant" / "transportProperties").write_text(_dict("dictionary", "transportProperties", f"\ntransportModel Newtonian;\nnu {NU};\n"))
    (case / "constant" / "turbulenceProperties").write_text(_dict("dictionary", "turbulenceProperties", """
simulationType RAS;
RAS
{
    RASModel        kOmegaSST;
    turbulence      on;
    printCoeffs     off;
}
"""))
    (case / "system" / "decomposeParDict").write_text(_dict("dictionary", "decomposeParDict", f"\nnumberOfSubdomains {nproc};\nmethod scotch;\n"))
    zero = case / "0"
    zero.mkdir(exist_ok=True)
    Us = f"({U[0]:.6f} {U[1]:.6f} {U[2]:.6f})"
    zero.joinpath("U").write_text(_dict("volVectorField", "U", f"""
dimensions [0 1 -1 0 0 0 0];
internalField uniform {Us};
boundaryField
{{
    farfield {{ type freestreamVelocity; freestreamValue uniform {Us}; }}
    "foil.*" {{ type noSlip; }}
}}
"""))
    zero.joinpath("p").write_text(_dict("volScalarField", "p", """
dimensions [0 2 -2 0 0 0 0];
internalField uniform 0;
boundaryField
{
    farfield { type freestreamPressure; freestreamValue uniform 0; }
    "foil.*" { type zeroGradient; }
}
"""))
    zero.joinpath("k").write_text(_dict("volScalarField", "k", f"""
dimensions [0 2 -2 0 0 0 0];
internalField uniform {k:.6g};
boundaryField
{{
    farfield {{ type freestream; freestreamValue uniform {k:.6g}; }}
    "foil.*" {{ type kqRWallFunction; value uniform {k:.6g}; }}
}}
"""))
    zero.joinpath("omega").write_text(_dict("volScalarField", "omega", f"""
dimensions [0 0 -1 0 0 0 0];
internalField uniform {omega:.6g};
boundaryField
{{
    farfield {{ type freestream; freestreamValue uniform {omega:.6g}; }}
    "foil.*" {{ type omegaWallFunction; value uniform {omega:.6g}; }}
}}
"""))
    zero.joinpath("nut").write_text(_dict("volScalarField", "nut", f"""
dimensions [0 2 -1 0 0 0 0];
internalField uniform {nut:.6g};
boundaryField
{{
    farfield {{ type calculated; value uniform {nut:.6g}; }}
    "foil.*" {{ type nutkWallFunction; value uniform 0; }}
}}
"""))
    return {"U_inf": U.tolist(), "k": k, "omega": omega}


# ----------------------------------------------------------------------------------------------- mesh

def _count_cells(log_text: str) -> int | None:
    m = re.findall(r"cells:\s+(\d+)", log_text)
    if m:
        return int(m[-1])
    m = re.findall(r"nCells:\s+(\d+)", log_text)
    return int(m[-1]) if m else None


def build_mesh(mesh_dir: Path, stl: Path, bounds, quality: str = "standard", nproc: int = 4, log=None, progress=None) -> dict:
    """Shared mesh: blockMesh, snappyHexMesh (parallel), reconstructed into mesh_dir/constant/polyMesh."""
    q = QUALITY.get(quality, QUALITY["standard"])
    mesh_dir = Path(mesh_dir)
    if mesh_dir.exists():
        shutil.rmtree(mesh_dir)
    for d in ("system", "constant/triSurface", "0"):
        (mesh_dir / d).mkdir(parents=True, exist_ok=True)
    shutil.copy(stl, mesh_dir / "constant" / "triSurface" / "foil.stl")
    info = write_mesh_dicts(mesh_dir, np.asarray(bounds), q, nproc)
    # snappy needs a controlDict and fvSchemes/fvSolution present; U/p for the mesh stage only
    write_case_dicts(mesh_dir, 16.67, 0.0, 0.0, 1, nproc)
    t0 = time.time()
    if progress:
        progress("blockMesh")
    if foam_run(["blockMesh"], mesh_dir, "blockMesh", log=log) != 0:
        raise RuntimeError("blockMesh failed, see " + str(mesh_dir / "log.blockMesh"))
    if nproc > 1:
        if foam_run(["decomposePar", "-force"], mesh_dir, "decomposePar", log=log) != 0:
            raise RuntimeError("decomposePar failed")
    stage = {"name": "castellated"}

    def on_line(line):
        if progress:
            if "Morphing phase" in line or "Snapping" in line:
                stage["name"] = "snapping"
                progress("snappyHexMesh: snapping")
            elif "Shrinking and layer addition" in line:
                progress("snappyHexMesh: layers")
            elif "Refinement iteration" in line:
                progress("snappyHexMesh: refinement " + line.strip().split()[-1])
    if progress:
        progress("snappyHexMesh: castellating")
    rc = foam_run(["snappyHexMesh", "-overwrite"], mesh_dir, "snappyHexMesh", nproc=nproc, log=log, on_line=on_line)
    if rc != 0:
        raise RuntimeError("snappyHexMesh failed, see " + str(mesh_dir / "log.snappyHexMesh"))
    if nproc > 1:
        if progress:
            progress("reconstructParMesh")
        if foam_run(["reconstructParMesh", "-constant", "-mergeTol", "1e-6"], mesh_dir, "reconstructParMesh", log=log) != 0:
            raise RuntimeError("reconstructParMesh failed")
        for p in mesh_dir.glob("processor*"):
            shutil.rmtree(p, ignore_errors=True)
    if progress:
        progress("checkMesh")
    foam_run(["checkMesh", "-constant"], mesh_dir, "checkMesh", log=log)
    cm = (mesh_dir / "log.checkMesh").read_text()
    cells = _count_cells(cm)
    m = re.search(r"Mesh OK|Failed (\d+) mesh checks", cm)
    info.update({"quality": quality, "cells": cells, "elapsed_s": round(time.time() - t0, 1), "nproc": nproc, "body_bounds": np.asarray(bounds).tolist(),
                 "mesh_check": m.group(0) if m else "?", "built_at": datetime.now().astimezone().isoformat(timespec="seconds")})
    nonortho = re.search(r"Max non-orthogonality = ([\d.]+)", cm)
    if nonortho:
        info["max_non_orthogonality"] = float(nonortho.group(1))
    (mesh_dir / "mesh.json").write_text(json.dumps(info, indent=1))
    if log:
        log(f"[cfd] mesh: {cells} cells in {info['elapsed_s']} s ({info['mesh_check']})")
    return info


# ----------------------------------------------------------------------------------------------- one case

def write_fan_sources(case: Path, fans: list[dict], rho: float = RHO) -> None:
    """Fans in operation as actuator discs: one short cylinder cell set per fan (system/topoSetDict, run `topoSet`
    before decomposePar) and a fixed momentum source on the air in it (constant/fvOptions, vectorSemiImplicitSource,
    absolute volume mode: the kinematic force F / rho in m4/s2 spread over the set). The force on the AIR points
    along the jet (opposite to the thrust on the aircraft). The wall `forces` function object still reports only
    the pressure and friction on the skin, so the installed aerodynamic effect of the jets is measured and the fan
    thrust itself must be added separately in any trim check.
    fans: [{"name", "centre": [x, y, z] FRD m (in the fluid), "jet_dir": unit vector of the air, "thrust_N",
            "diameter_m", "thickness_m"}]"""
    sets, srcs = [], []
    for k, f in enumerate(fans):
        c = np.asarray(f["centre"], float); d = np.asarray(f["jet_dir"], float); d = d / np.linalg.norm(d)
        t = float(f.get("thickness_m", 0.06)); r = 0.5 * float(f.get("diameter_m", 0.195))
        p1, p2 = c - 0.5 * t * d, c + 0.5 * t * d
        nm = f.get("name", f"fan{k}").replace(" ", "_")
        sets.append(f"""
    {{
        name    {nm};
        type    cellSet;
        action  new;
        source  cylinderToCell;
        point1  ({p1[0]:.4f} {p1[1]:.4f} {p1[2]:.4f});
        point2  ({p2[0]:.4f} {p2[1]:.4f} {p2[2]:.4f});
        radius  {r:.4f};
    }}""")
        F = float(f["thrust_N"]) / rho * d
        srcs.append(f"""
{nm}
{{
    type            vectorSemiImplicitSource;
    active          yes;
    selectionMode   cellSet;
    cellSet         {nm};
    volumeMode      absolute;
    sources
    {{
        U   (({F[0]:.5f} {F[1]:.5f} {F[2]:.5f}) 0);   // {f['thrust_N']:g} N on the air along the jet, kinematic (F / rho)
    }}
}}""")
    (case / "system" / "topoSetDict").write_text(_dict("dictionary", "topoSetDict", "\nactions\n(" + "".join(sets) + "\n);\n"))
    (case / "constant").mkdir(parents=True, exist_ok=True)
    (case / "constant" / "fvOptions").write_text(_dict("dictionary", "fvOptions", "".join(srcs) + "\n"))


def _parse_forces(case: Path, iterations: int):
    """forces function object output -> (F mean, M mean, F std, M std, n iterations seen). Last 30 % averaged."""
    fdir = case / "postProcessing" / "forces"
    rows_f, rows_m = [], []
    for sub in sorted(fdir.glob("*")) if fdir.exists() else []:
        for name, rows in (("force.dat", rows_f), ("moment.dat", rows_m)):
            fp = sub / name
            if not fp.exists():
                continue
            for line in fp.read_text().splitlines():
                if not line.strip() or line.startswith("#"):
                    continue
                nums = [float(x) for x in re.findall(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?", line.replace("(", " ").replace(")", " "))]
                if len(nums) >= 4:
                    rows.append(nums[:4])
    if not rows_f or not rows_m:
        return None
    F = np.array(rows_f)
    M = np.array(rows_m)
    n = len(F)
    w = max(5, int(0.3 * n))
    Fw, Mw = F[-w:, 1:4], M[-w:, 1:4]
    return {"F": Fw.mean(0), "M": Mw.mean(0), "F_std": Fw.std(0), "M_std": Mw.std(0), "n": n,
            "history": {"it": F[:, 0].tolist(), "Fx": F[:, 1].tolist(), "Fy": F[:, 2].tolist(), "Fz": F[:, 3].tolist(),
                        "Mx": M[:, 1].tolist(), "My": M[:, 2].tolist(), "Mz": M[:, 3].tolist()}}


def _parse_surface_raw(case: Path):
    """cpSurface raw output -> (face centres Nx3, p N) from the latest time."""
    sdir = case / "postProcessing" / "cpSurface"
    if not sdir.exists():
        return None
    times = sorted((d for d in sdir.iterdir() if d.is_dir()), key=lambda d: float(d.name) if re.match(r"^[\d.eE+-]+$", d.name) else -1)
    for t in reversed(times):
        fp = t / "p_foil.raw"
        if fp.exists():
            data = np.loadtxt(fp, comments="#")
            if data.ndim == 2 and data.shape[1] >= 4:
                return data[:, :3], data[:, 3]
    return None


def _parse_flow_cloud(case: Path):
    """flowCloud raw output + flow_grid.json -> dict(origin, spacing, shape, U (nx,ny,nz,3) float32, p (nx,ny,nz) float32),
    NaN where a grid point lies inside the body (OpenFOAM drops points outside the mesh)."""
    gj = case / "flow_grid.json"
    sdir = case / "postProcessing" / "flowCloud"
    if not gj.exists() or not sdir.exists():
        return None
    grid = json.loads(gj.read_text())
    times = sorted((d for d in sdir.iterdir() if d.is_dir()), key=lambda d: float(d.name) if re.match(r"^[\d.eE+-]+$", d.name) else -1)
    data = None
    for t in reversed(times):
        for fp in t.glob("*.xy"):
            try:
                import pandas as pd
                data = pd.read_csv(fp, sep=r"\s+", header=None, comment="#").to_numpy(float)
            except Exception:  # noqa: BLE001
                data = np.loadtxt(fp, comments="#")
            break
        if data is not None:
            break
    if data is None or data.ndim != 2 or data.shape[1] < 7:
        return None
    origin = np.asarray(grid["origin"]); h = float(grid["spacing"]); nx, ny, nz = grid["shape"]
    idx = np.rint((data[:, :3] - origin) / h).astype(int)
    ok = np.all((idx >= 0) & (idx < np.array([nx, ny, nz])), axis=1)
    idx = idx[ok]; d = data[ok]
    U = np.full((nx, ny, nz, 3), np.nan, np.float32)
    P = np.full((nx, ny, nz), np.nan, np.float32)
    P[idx[:, 0], idx[:, 1], idx[:, 2]] = d[:, 3]
    U[idx[:, 0], idx[:, 1], idx[:, 2], :] = d[:, 4:7]
    return {"origin": origin.astype(np.float32), "spacing": h, "shape": np.array([nx, ny, nz]), "U": U, "p": P}


def _residuals(log_text: str) -> dict:
    out = {}
    for var in ("Ux", "Uy", "Uz", "p", "k", "omega"):
        m = re.findall(rf"Solving for {var},\s+Initial residual = ([\d.eE+-]+)", log_text)
        if m:
            out[var] = float(m[-1])
    return out


def wind_axes(F: np.ndarray, alpha_deg: float, beta_deg: float) -> dict:
    a, b = math.radians(alpha_deg), math.radians(beta_deg)
    e_air = -np.array([math.cos(a) * math.cos(b), math.sin(b), math.sin(a) * math.cos(b)])
    drag = float(np.dot(F, e_air))
    lift = float(F[0] * math.sin(a) - F[2] * math.cos(a))
    side = float(F[1])
    return {"lift": lift, "drag": drag, "side": side}


def _stl_bounds(stl: Path):
    import trimesh
    m = trimesh.load(str(stl), force="mesh")
    return m.bounds.tolist()


def run_case(case_dir: Path, mesh_dir: Path, alpha: float, beta: float, speed_ms: float, quality: str = "standard",
             nproc: int = 4, log=None, progress=None, iterations: int | None = None, flow: bool = True) -> dict:
    """Run simpleFoam on the shared mesh at one attitude. Writes result.json and surface_p.npz in case_dir."""
    q = QUALITY.get(quality, QUALITY["standard"])
    iters = int(iterations or q["iterations"])
    case_dir = Path(case_dir)
    if case_dir.exists():
        shutil.rmtree(case_dir)
    (case_dir / "system").mkdir(parents=True)
    (case_dir / "constant").mkdir()
    shutil.copytree(mesh_dir / "constant" / "polyMesh", case_dir / "constant" / "polyMesh")
    write_case_dicts(case_dir, speed_ms, alpha, beta, iters, nproc, flow=flow)
    if flow:
        bounds = json.loads((mesh_dir / "mesh.json").read_text()).get("body_bounds")
        if bounds is None:
            bounds = _stl_bounds(mesh_dir / "constant" / "triSurface" / "foil.stl")
        write_flow_sample_dict(case_dir, flow_grid(bounds, FLOW_SPACING.get(quality, 0.08)))
    t0 = time.time()
    state = {"it": 0}

    def on_line(line):
        if line.startswith("Time = "):
            try:
                state["it"] = int(float(line.split("=")[1]))
            except ValueError:
                pass
            if progress and state["it"] % 10 == 0:
                progress(f"simpleFoam {state['it']}/{iters}")
    if nproc > 1:
        if foam_run(["decomposePar", "-force"], case_dir, "decomposePar", log=log) != 0:
            raise RuntimeError("decomposePar failed")
    rc = foam_run(["simpleFoam"], case_dir, "simpleFoam", nproc=nproc, log=log, on_line=on_line)
    log_text = (case_dir / "log.simpleFoam").read_text()
    if rc != 0 and "SIMPLE solution converged" not in log_text:
        raise RuntimeError(f"simpleFoam failed (rc {rc}), see {case_dir / 'log.simpleFoam'}")
    if nproc > 1:
        # the function objects already wrote forces and the surface sample on the master; the fields stay decomposed
        for p in case_dir.glob("processor*"):
            shutil.rmtree(p, ignore_errors=True)
    return postprocess_case(case_dir, alpha, beta, speed_ms, quality, iters, nproc, round(time.time() - t0, 1), flow, log_text, log)


def postprocess_case(case_dir: Path, alpha: float, beta: float, speed_ms: float, quality: str, iters: int, nproc: int,
                     elapsed_s: float, flow: bool, log_text: str = "", log=None, extra: dict | None = None) -> dict:
    """From a finished case directory (local run or imported from the cluster) to result.json, surface_p.npz, flow.npz."""
    case_dir = Path(case_dir)
    if not log_text and (case_dir / "log.simpleFoam").exists():
        log_text = (case_dir / "log.simpleFoam").read_text()
    forces = _parse_forces(case_dir, iters)
    if forces is None:
        raise RuntimeError("no forces output in " + str(case_dir))
    surf = _parse_surface_raw(case_dir)
    F, M = forces["F"], forces["M"]
    conv = "SIMPLE solution converged" in log_text
    rel = float(np.linalg.norm(forces["F_std"]) / max(np.linalg.norm(F), 1e-9))
    res = {
        "alpha_deg": alpha, "beta_deg": beta, "speed_ms": speed_ms, "rho": RHO, "quality": quality, "iterations": forces["n"],
        "iterations_requested": iters, "converged_residuals": conv, "force_scatter": rel,
        "ok": bool(np.all(np.isfinite(F)) and np.all(np.isfinite(M)) and rel < 0.10),
        "F_frd": F.tolist(), "M0_frd": M.tolist(), "F_std": forces["F_std"].tolist(), "M_std": forces["M_std"].tolist(),
        **wind_axes(F, alpha, beta), "residuals": _residuals(log_text),
        "elapsed_s": elapsed_s, "nproc": nproc, "finished_at": datetime.now().astimezone().isoformat(timespec="seconds"),
    }
    if extra:
        res.update(extra)
    if flow:
        try:
            fl = _parse_flow_cloud(case_dir)
            if fl is not None:
                np.savez_compressed(case_dir / "flow.npz", **fl)
                res["flow_points"] = int(np.isfinite(fl["p"]).sum())
                res["flow_shape"] = fl["shape"].tolist()
        except Exception as e:  # noqa: BLE001
            res["flow_error"] = f"{type(e).__name__}: {e}"
        shutil.rmtree(case_dir / "postProcessing" / "flowCloud", ignore_errors=True)   # 25 MB of text per case
    if surf is not None:
        centres, p = surf
        np.savez_compressed(case_dir / "surface_p.npz", centres=centres.astype(np.float32), p=p.astype(np.float32))
        res["surface_faces"] = int(len(p))
    (case_dir / "result.json").write_text(json.dumps(res, indent=1))
    (case_dir / "forces_history.json").write_text(json.dumps(forces["history"]))
    if log:
        log(f"[cfd] a={alpha:+.1f} b={beta:+.1f}: L {res['lift']:.1f} N, D {res['drag']:.1f} N, My0 {M[1]:.1f} N m, "
            f"{forces['n']} it, scatter {rel * 100:.1f} %, {res['elapsed_s']} s")
    return res
