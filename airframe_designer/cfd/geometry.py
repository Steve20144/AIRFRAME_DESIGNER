"""CFD surface preparation: a STEP / STL / OBJ / .blend of the foil -> one watertight STL in the structural frame.

The design surfaces are open skins (the ATLAS v1 STEP is three open shells with gaps up to 5 cm, one of them an
interior deck). snappyHexMesh needs a closed body, so the skins are closed through a generalised winding-number
field (libigl) sampled on a voxel grid and contoured at 0.5 (marching cubes):

  1. tessellate / import every shell as its own mesh, make each shell's normals consistent
  2. orientation: for every sign combination of the shells, build the closure and keep the one whose surface lies
     closest to the input skins (a flipped skin creates a bubble far from any input surface)
  3. interior shells (most of their vertices inside the closure of the others) are dropped and the closure redone
  4. keep the largest body, Taubin-smooth the voxel staircase, decimate to the CFD size and to a display size
  5. CAD frame -> structural FRD (x forward, y right, z down), metres

Outputs in <out_dir>: surface.stl (CFD, FRD metres), display.json (vertices/indices for the browser), meta.json.
"""
from __future__ import annotations

import hashlib
import itertools
import json
import os
import shutil
import subprocess
import sys
import tempfile
import time
from datetime import datetime
from pathlib import Path

import numpy as np

PROJECT_DIR = Path(__file__).resolve().parents[2]
BLENDER = os.environ.get("BLENDER", "/Applications/Blender.app/Contents/MacOS/Blender")

# CAD (Rhino, ATLAS v1: x right, y aft, z up, nose at -y) -> structural FRD, as scripts/atlas_v1_from_masses.py
R_CAD_TO_FRD = np.array([[0.0, -1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
T_CAD_M = np.array([0.0, 0.0, 5.0])


def _log(log, msg):
    line = f"[cfd.geometry] {msg}"
    if log:
        log(line)
    else:
        print(line, flush=True)


def file_hash(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()[:16]


# ----------------------------------------------------------------------------------------------- input readers

def _shells_from_step(path: Path, deflection_mm: float):
    """Each STEP shell/solid as its own (vertices, faces) in the file's units."""
    from ..geometry import cad
    from OCP.STEPControl import STEPControl_Reader
    from OCP.IFSelect import IFSelect_RetDone
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopAbs import TopAbs_SHELL

    r = STEPControl_Reader()
    if r.ReadFile(str(path)) != IFSelect_RetDone:
        raise RuntimeError(f"cannot read STEP {path}")
    r.TransferRoots()
    shape = r.OneShape()
    out = []
    ex = TopExp_Explorer(shape, TopAbs_SHELL)
    while ex.More():
        v, f = cad._mesh_shape(ex.Current(), deflection_mm)
        if len(f):
            out.append((v, f))
        ex.Next()
    if not out:
        v, f = cad._mesh_shape(shape, deflection_mm)
        out.append((v, f))
    return out


def _shells_from_mesh_file(path: Path):
    import trimesh
    m = trimesh.load(str(path), force="mesh")
    parts = m.split(only_watertight=False)
    parts = [p for p in parts if len(p.faces) >= 50] or [m]
    return [(np.asarray(p.vertices), np.asarray(p.faces)) for p in parts]


def blender_export_stl(blend: Path, out_stl: Path, match: list[str] | None = None, log=None) -> str:
    """Export the mesh objects of a .blend to one STL (Blender headless)."""
    if not Path(BLENDER).exists():
        raise RuntimeError(f"Blender not found at {BLENDER} (set BLENDER=...)")
    tools = PROJECT_DIR / "airframe_designer" / "cfd" / "blender_tools.py"
    cmd = [BLENDER, "--background", str(blend), "--python", str(tools), "--", "export", str(out_stl)]
    if match:
        cmd += ["--match", ",".join(match)]
    _log(log, "blender: " + " ".join(cmd[3:]))
    p = subprocess.run(cmd, capture_output=True, text=True, timeout=900)
    msg = [l for l in (p.stdout + p.stderr).splitlines() if "BLENDER_TOOLS" in l]
    if p.returncode != 0 or not out_stl.exists():
        raise RuntimeError("Blender export failed: " + (msg[-1] if msg else (p.stderr or p.stdout)[-800:]))
    return msg[-1] if msg else ""


def load_shells(source: Path, deflection_mm: float = 1.0, match: list[str] | None = None, log=None):
    """-> (list of trimesh shells in metres, CAD frame, notes)"""
    import trimesh
    notes = []
    suf = source.suffix.lower()
    if suf in (".stp", ".step"):
        raw = _shells_from_step(source, deflection_mm)
    elif suf == ".blend":
        tmp = Path(tempfile.mkdtemp(prefix="cfd_blend_")) / "export.stl"
        notes.append(blender_export_stl(source, tmp, match, log))
        raw = _shells_from_mesh_file(tmp)
        shutil.rmtree(tmp.parent, ignore_errors=True)
    elif suf in (".stl", ".obj", ".ply", ".glb", ".gltf"):
        raw = _shells_from_mesh_file(source)
    else:
        raise ValueError(f"unsupported geometry file {source.name}")
    allv = np.vstack([v for v, _ in raw])
    extent = float(np.max(allv.max(0) - allv.min(0)))
    scale = 1e-3 if extent > 100.0 else 1.0        # a 3.8 m aircraft in mm reads as 3800 units
    notes.append(f"{len(raw)} shell(s), extent {extent:.1f} units -> {'mm' if scale < 1 else 'm'}")
    shells = []
    for v, f in raw:
        m = trimesh.Trimesh(np.asarray(v, np.float64) * scale, np.asarray(f, np.int64), process=True)
        trimesh.repair.fix_normals(m)
        shells.append(m)
    return shells, notes


# ----------------------------------------------------------------------------------------------- closure

def _closure(Wsum, axes, lo, h):
    import trimesh
    from skimage import measure
    n = tuple(len(a) for a in axes)
    v, f, _, _ = measure.marching_cubes(Wsum.reshape(n), level=0.5, spacing=(h, h, h))
    v += lo
    m = trimesh.Trimesh(v, f, process=True)
    parts = m.split(only_watertight=False)
    if len(parts) == 0:
        return m
    return max(parts, key=lambda p: abs(p.volume))


def _misfit(closure, shells, h, nsample=4000, rng=np.random.default_rng(0)):
    """Fraction of the closure's surface points farther than 3 voxels from every input skin (lower is better)."""
    import trimesh
    if len(closure.vertices) == 0:
        return 1.0
    idx = rng.choice(len(closure.vertices), size=min(nsample, len(closure.vertices)), replace=False)
    pts = closure.vertices[idx]
    best = np.full(len(pts), np.inf)
    for s in shells:
        _, d, _ = trimesh.proximity.closest_point(s, pts)
        best = np.minimum(best, d)
    return float(np.mean(best > 3.0 * h))


def close_skins(shells, voxel_m: float = 0.010, pad_m: float = 0.05, log=None):
    """Open skins (trimesh list, metres) -> watertight trimesh of the outer body, plus a report dict."""
    import igl
    import trimesh
    from scipy.interpolate import RegularGridInterpolator

    allv = np.vstack([s.vertices for s in shells])
    lo = allv.min(0) - pad_m
    hi = allv.max(0) + pad_m
    h = float(voxel_m)
    n = [int(np.ceil((hi[i] - lo[i]) / h)) + 1 for i in range(3)]
    axes = [lo[i] + h * np.arange(n[i]) for i in range(3)]
    G = np.stack(np.meshgrid(*axes, indexing="ij"), -1).reshape(-1, 3)
    _log(log, f"winding-number grid {n[0]}x{n[1]}x{n[2]} at {h * 1000:.0f} mm ({G.shape[0] / 1e6:.1f} M points)")
    t0 = time.time()
    W = [igl.fast_winding_number(np.asarray(s.vertices, np.float64), np.asarray(s.faces, np.int64), G) for s in shells]
    _log(log, f"winding numbers in {time.time() - t0:.1f} s")

    active = list(range(len(shells)))
    report = {"voxel_m": h, "grid": n, "shells": []}
    for _round in range(len(shells) + 1):
        # orientation: first active shell fixed, the others tried both ways
        cands = []
        for signs in itertools.product([1, -1], repeat=len(active) - 1):
            signs = (1,) + signs
            Wsum = sum(s * W[i] for s, i in zip(signs, active))
            c = _closure(Wsum, axes, lo, h)
            vol = abs(float(c.volume))
            cands.append([signs, c, Wsum, None, vol])
        vmax = max(c[4] for c in cands)
        # a flipped skin either adds a bubble far from every input (high misfit) or cancels the body (tiny volume)
        for c in cands:
            c[3] = _misfit(c[1], [shells[i] for i in active], h) if c[4] > 0.25 * vmax else 1.0
        signs, closure, Wsum, mis, vol = min(cands, key=lambda c: (round(c[3], 3), -c[4]))
        _log(log, f"shells {[i + 1 for i in active]} signs {signs}: volume {vol:.3f} m3, misfit {mis * 100:.1f} %")
        # interior shells: most of their vertices inside the closure of the remaining shells
        interior = None
        if len(active) > 1:
            for k, i in enumerate(active):
                Wo = sum(s * W[j] for s, j in zip(signs, active) if j != i).reshape(n)
                f = RegularGridInterpolator(axes, Wo, bounds_error=False, fill_value=0.0)
                inside = float(np.mean(f(shells[i].vertices) > 0.5))
                if inside > 0.8:
                    interior = i
                    _log(log, f"shell {i + 1}: {inside * 100:.0f} % inside the others -> interior, dropped")
                    break
        if interior is None:
            report["shells"] = [{"index": i + 1, "sign": s, "used": True} for s, i in zip(signs, active)] + \
                               [{"index": i + 1, "used": False} for i in range(len(shells)) if i not in active]
            report["volume_m3"] = vol
            report["misfit"] = mis
            return closure, report
        active = [i for i in active if i != interior]
    raise RuntimeError("closure failed")


# ----------------------------------------------------------------------------------------------- prepare

def _decimate(mesh, target: int):
    """Quadric edge collapse that keeps the surface watertight (pymeshlab; fast_simplification agg=1 as fallback)."""
    import trimesh
    if len(mesh.faces) <= target:
        return mesh
    try:
        import pymeshlab
        ms = pymeshlab.MeshSet()
        ms.add_mesh(pymeshlab.Mesh(np.asarray(mesh.vertices, np.float64), np.asarray(mesh.faces, np.int32)))
        ms.meshing_decimation_quadric_edge_collapse(targetfacenum=int(target), preservetopology=True, preservenormal=True,
                                                    planarquadric=True)
        mm = ms.current_mesh()
        out = trimesh.Trimesh(mm.vertex_matrix(), mm.face_matrix(), process=True)
    except Exception:
        import fast_simplification
        v, f = fast_simplification.simplify(np.asarray(mesh.vertices), np.asarray(mesh.faces, np.int32), target_count=int(target), agg=1)
        out = trimesh.Trimesh(v, f, process=True)
    return out


def planform_metrics(mesh) -> dict:
    """Projected planform area (x-y plane, FRD: half the z-projected area of the closed skin), span, length, chord."""
    nz = np.abs(mesh.face_normals[:, 2])
    area = float(0.5 * np.sum(nz * mesh.area_faces))
    b = mesh.bounds
    span = float(b[1][1] - b[0][1])
    length = float(b[1][0] - b[0][0])
    return {"area_m2": area, "span_m": span, "length_m": length, "mean_chord_m": area / span if span > 0 else length}


def prepare_surface(source: str | Path, out_dir: str | Path, *, voxel_mm: float = 10.0, deflection_mm: float = 1.0,
                    cfd_tris: int = 300_000, display_tris: int = 40_000, smooth_iters: int = 10,
                    match: list[str] | None = None, cad_to_frd: bool = True, log=None) -> dict:
    import trimesh
    source = Path(source).expanduser()
    if not source.is_absolute():
        source = (PROJECT_DIR / source).resolve()
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    t0 = time.time()
    shells, notes = load_shells(source, deflection_mm, match, log)
    for n in notes:
        _log(log, n)
    closed, rep = close_skins(shells, voxel_m=voxel_mm / 1000.0, log=log)
    trimesh.smoothing.filter_taubin(closed, lamb=0.5, nu=-0.53, iterations=smooth_iters)
    closed = _decimate(closed, cfd_tris)
    closed.process(validate=True)
    if not closed.is_watertight:
        trimesh.repair.fill_holes(closed)
    trimesh.repair.fix_normals(closed)
    if closed.volume < 0:
        closed.invert()
    # frame: CAD -> FRD metres
    if cad_to_frd:
        v = (closed.vertices - T_CAD_M) @ R_CAD_TO_FRD.T
        closed = trimesh.Trimesh(v, closed.faces, process=False)
        trimesh.repair.fix_normals(closed)
        if closed.volume < 0:
            closed.invert()
    closed.export(str(out_dir / "surface.stl"))
    disp = _decimate(closed, display_tris)
    disp_payload = {"vertices": np.asarray(disp.vertices, np.float32).round(4).ravel().tolist(),
                    "indices": np.asarray(disp.faces, np.int64).ravel().tolist()}
    (out_dir / "display.json").write_text(json.dumps(disp_payload))
    pm = planform_metrics(closed)
    meta = {
        "source": str(source), "source_hash": file_hash(source), "prepared_at": datetime.now().astimezone().isoformat(timespec="seconds"),
        "frame": "FRD metres (x forward, y right, z down)" if cad_to_frd else "source frame, metres",
        "voxel_mm": voxel_mm, "deflection_mm": deflection_mm, "closure": rep, "notes": notes,
        "cfd_tris": int(len(closed.faces)), "display_tris": int(len(disp.faces)), "watertight": bool(closed.is_watertight),
        "volume_m3": float(closed.volume), "wetted_area_m2": float(closed.area), "bounds_frd": closed.bounds.round(4).tolist(),
        **pm, "elapsed_s": round(time.time() - t0, 1),
    }
    (out_dir / "meta.json").write_text(json.dumps(meta, indent=1))
    _log(log, f"surface ready: {meta['cfd_tris']} tris, watertight {meta['watertight']}, area {pm['area_m2']:.2f} m2, "
              f"span {pm['span_m']:.2f} m, length {pm['length_m']:.2f} m, {meta['elapsed_s']} s")
    return meta


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("source")
    ap.add_argument("out_dir")
    ap.add_argument("--voxel-mm", type=float, default=10.0)
    ap.add_argument("--match", default="", help="Blender object-name substrings, comma separated")
    a = ap.parse_args()
    sys.path.insert(0, str(PROJECT_DIR))
    print(json.dumps(prepare_surface(a.source, a.out_dir, voxel_mm=a.voxel_mm, match=[m for m in a.match.split(",") if m]), indent=1))
