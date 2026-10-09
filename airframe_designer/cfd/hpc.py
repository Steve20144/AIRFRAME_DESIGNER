"""Run the attitude library on the AUTH Aristotelis cluster (OpenFOAM v2506, Slurm) instead of this Mac.

Export: one self-contained bundle (tar.gz) with the closed surface, the mesh template, one case per attitude and
the Slurm scripts: `mesh.sh` builds the shared mesh once (one node), `cases.sh` is a job array that runs every
attitude in parallel (one node each, 20 cores), `submit.sh` chains them, `collect.sh` packs the results.
Import: the results tar.gz goes back through the same post-processing as a local run (`case.postprocess_case`),
into a library of its own (`lib_<kmh>kmh_<batch>`), which the CFD tab reads like any other.

Each bundle carries a manifest with the surface hash, so results are never mixed between geometries.
Bundles live in results/cfd/<surface>/hpc/<batch>/ and the tarballs next to them.
"""
from __future__ import annotations

import json
import re
import shutil
import tarfile
import time
from datetime import datetime
from pathlib import Path

import numpy as np

from . import case as C
from .geometry import PROJECT_DIR

CFD_ROOT = PROJECT_DIR / "results" / "cfd"

# Aristotelis (hpc.it.auth.gr): batch partition, 17 nodes x 20 cores, 7 days; OpenFOAM v2506 module
CLUSTER = {
    "host": "aristotle.it.auth.gr",
    "partition": "batch",
    "cores_per_node": 20,
    "modules": "module load gcc/14.2.0 openmpi/5.0.5 openfoam/2506",
    "bashrc": "source $OPENFOAM_ROOT/etc/bashrc",
}


def _case_key(a, b):
    return f"a{a:+05.1f}_b{b:+05.1f}"


def _batch_root(surface: str) -> Path:
    return CFD_ROOT / surface / "hpc"


def list_batches(surface: str) -> list[dict]:
    out = []
    root = _batch_root(surface)
    if not root.exists():
        return out
    for d in sorted(root.iterdir()):
        mf = d / "manifest.json"
        if mf.exists():
            try:
                m = json.loads(mf.read_text())
            except Exception:  # noqa: BLE001
                continue
            m["tar"] = str(d.with_suffix(".tar.gz").relative_to(PROJECT_DIR)) if d.with_suffix(".tar.gz").exists() else None
            m["results_imported"] = m.get("imported_at") is not None
            out.append(m)
    return out


def export_bundle(surface: str, batch: str, alphas, betas, speed_kmh: float = 60.0, quality: str = "standard",
                  cores_per_node: int | None = None, max_parallel_jobs: int = 10, iterations: int | None = None,
                  walltime_case: str = "03:00:00", walltime_mesh: str = "02:00:00", log=None, fans: list[dict] | None = None) -> dict:
    """Build results/cfd/<surface>/hpc/<batch>/ and <batch>.tar.gz. Returns the manifest."""
    batch = re.sub(r"[^A-Za-z0-9-]+", "-", batch).strip("-") or datetime.now().strftime("b%Y%m%d-%H%M")
    sdir = CFD_ROOT / surface / "surface"
    meta = json.loads((sdir / "meta.json").read_text())
    q = C.QUALITY.get(quality, C.QUALITY["standard"])
    iters = int(iterations or q["iterations"])
    nproc = int(cores_per_node or CLUSTER["cores_per_node"])
    V = float(speed_kmh) / 3.6
    root = _batch_root(surface) / batch
    if root.exists():
        shutil.rmtree(root)
    (root / "template" / "system").mkdir(parents=True)
    (root / "geometry").mkdir()
    (root / "cases").mkdir()
    (root / "mesh" / "system").mkdir(parents=True)
    (root / "mesh" / "constant" / "triSurface").mkdir(parents=True)
    (root / "mesh" / "0").mkdir()
    shutil.copy(sdir / "surface.stl", root / "geometry" / "foil.stl")
    shutil.copy(sdir / "surface.stl", root / "mesh" / "constant" / "triSurface" / "foil.stl")
    bounds = np.asarray(meta["bounds_frd"])
    # the shared mesh (built on the cluster by mesh.sh); its dicts are complete OpenFOAM cases so snappy can run
    C.write_mesh_dicts(root / "mesh", bounds, q, nproc)
    C.write_case_dicts(root / "mesh", V, 0.0, 0.0, 1, nproc)
    # one flow sampling dict for every case
    grid = C.flow_grid(bounds, C.FLOW_SPACING.get(quality, 0.10))
    C.write_flow_sample_dict(root / "template", grid)
    shutil.move(str(root / "template" / "system" / "flowCloud"), str(root / "template" / "flowCloud")) if (root / "template" / "system" / "flowCloud").exists() else None
    shutil.rmtree(root / "template" / "system", ignore_errors=True)
    grid_meta = json.loads((root / "template" / "flow_grid.json").read_text())
    alphas = sorted(set(round(float(a), 1) for a in alphas))
    betas = sorted(set(round(abs(float(b)), 1) for b in betas))
    keys = []
    for b in betas:
        for a in alphas:
            key = _case_key(a, b)
            cdir = root / "cases" / key
            (cdir / "system").mkdir(parents=True)
            (cdir / "constant").mkdir()
            C.write_case_dicts(cdir, V, a, b, iters, nproc, flow=True, flow_include="../../../template/flowCloud")
            if fans:
                C.write_fan_sources(cdir, fans)
            (cdir / "flow_grid.json").write_text(json.dumps(grid_meta))
            keys.append({"key": key, "alpha": a, "beta": b})
    manifest = {
        "batch": batch, "surface": surface, "surface_hash": meta.get("source_hash"), "surface_source": meta.get("source"),
        "speed_kmh": float(speed_kmh), "quality": quality, "iterations": iters, "cores_per_node": nproc, "max_parallel_jobs": int(max_parallel_jobs),
        "fans": fans or [],
        "cases": keys, "flow_spacing_m": C.FLOW_SPACING.get(quality, 0.10), "created_at": datetime.now().astimezone().isoformat(timespec="seconds"),
        "library_quality": f"{quality}-{batch}", "cluster": CLUSTER, "imported_at": None,
    }
    (root / "manifest.json").write_text(json.dumps(manifest, indent=1))
    _write_scripts(root, manifest, walltime_case, walltime_mesh)
    tar = root.with_suffix(".tar.gz")
    with tarfile.open(tar, "w:gz") as tf:
        tf.add(root, arcname=batch)
    manifest["bundle_dir"] = str(root.relative_to(PROJECT_DIR))
    manifest["tar"] = str(tar.relative_to(PROJECT_DIR))
    manifest["tar_mb"] = round(tar.stat().st_size / 1e6, 1)
    if log:
        log(f"[hpc] bundle {batch}: {len(keys)} cases, {manifest['tar_mb']} MB -> {tar}")
    return manifest


def _write_scripts(root: Path, m: dict, walltime_case: str, walltime_mesh: str):
    n = len(m["cases"])
    cores = m["cores_per_node"]
    part = m["cluster"]["partition"]
    mods = m["cluster"]["modules"]
    bashrc = m["cluster"]["bashrc"]
    keys = "\n".join(c["key"] for c in m["cases"])
    (root / "cases.txt").write_text(keys + "\n")
    (root / "mesh.sh").write_text(f"""#!/bin/bash
#SBATCH --job-name={m['batch']}-mesh
#SBATCH --partition={part}
#SBATCH --nodes=1
#SBATCH --ntasks-per-node={cores}
#SBATCH --time={walltime_mesh}
#SBATCH --output=logs/mesh.%j.out
# Shared mesh for every attitude of batch {m['batch']} (snappyHexMesh on one node).
set -e
{mods}
{bashrc}
cd "$SLURM_SUBMIT_DIR/mesh"
blockMesh > log.blockMesh
decomposePar -force > log.decomposePar
srun snappyHexMesh -parallel -overwrite > log.snappyHexMesh
reconstructParMesh -constant -mergeTol 1e-6 > log.reconstructParMesh
rm -rf processor*
checkMesh -constant > log.checkMesh || true
echo "mesh done: $(grep -m1 'cells:' log.checkMesh)"
""")
    (root / "cases.sh").write_text(f"""#!/bin/bash
#SBATCH --job-name={m['batch']}-case
#SBATCH --partition={part}
#SBATCH --nodes=1
#SBATCH --ntasks-per-node={cores}
#SBATCH --time={walltime_case}
#SBATCH --array=0-{n - 1}%{m['max_parallel_jobs']}
#SBATCH --output=logs/case.%A_%a.out
# One attitude per array task: copies the shared mesh, runs simpleFoam ({m['iterations']} iterations) on {cores} cores.
set -e
{mods}
{bashrc}
cd "$SLURM_SUBMIT_DIR"
KEY=$(sed -n "$((SLURM_ARRAY_TASK_ID + 1))p" cases.txt)
echo "attitude $KEY on $(hostname)"
cd "cases/$KEY"
rm -rf constant/polyMesh processor*
cp -r ../../mesh/constant/polyMesh constant/polyMesh
if [ -f system/topoSetDict ]; then topoSet > log.topoSet; fi      # fan cell sets (powered cases)
decomposePar -force > log.decomposePar
START=$(date +%s)
srun simpleFoam -parallel > log.simpleFoam || echo "simpleFoam exited with $?" >> log.simpleFoam
echo "elapsed_s $(( $(date +%s) - START ))" > elapsed.txt
rm -rf processor* constant/polyMesh
echo "done $KEY"
""")
    (root / "submit.sh").write_text(f"""#!/bin/bash
# Submit the whole batch: the mesh first, then every attitude once the mesh is done.
set -e
cd "$(dirname "$0")"
mkdir -p logs
MESH=$(sbatch --parsable mesh.sh)
echo "mesh job $MESH"
CASES=$(sbatch --parsable --dependency=afterok:$MESH cases.sh)
echo "case array $CASES ({n} attitudes, up to {m['max_parallel_jobs']} at a time)"
echo "watch with:  squeue -u $USER      or      ./status.sh"
""")
    (root / "status.sh").write_text(f"""#!/bin/bash
# How far the batch is: finished attitudes have a result in postProcessing/forces and an elapsed.txt
cd "$(dirname "$0")"
DONE=0
for K in $(cat cases.txt); do [ -f "cases/$K/elapsed.txt" ] && DONE=$((DONE+1)); done
echo "$DONE / {n} attitudes finished"
squeue -u $USER -o "%.10i %.18j %.8T %.10M %.6D %R" 2>/dev/null | head -20
""")
    (root / "collect.sh").write_text(f"""#!/bin/bash
# Pack the results (forces, wall pressure, flow samples, logs) for the import into Airframe Designer.
cd "$(dirname "$0")"
OUT="{m['batch']}_results.tar.gz"
tar czf "$OUT" manifest.json cases.txt mesh/log.checkMesh \\
    $(for K in $(cat cases.txt); do for F in postProcessing log.simpleFoam elapsed.txt system/controlDict; do [ -e "cases/$K/$F" ] && echo "cases/$K/$F"; done; done)
ls -la "$OUT"
echo "download with:  scp $USER@{m['cluster']['host']}:$(pwd)/$OUT ."
""")
    for f in ("mesh.sh", "cases.sh", "submit.sh", "status.sh", "collect.sh"):
        (root / f).chmod(0o755)
    (root / "README.md").write_text(f"""# CFD batch {m['batch']} for Aristotelis (hpc.it.auth.gr)

Surface {m['surface']} ({m['surface_hash']}), {m['speed_kmh']:g} km/h, quality {m['quality']}, {n} attitudes,
{m['iterations']} iterations each, {cores} cores per attitude.

## On your Mac
    scp {m['batch']}.tar.gz USER@{m['cluster']['host']}:~/
## On the cluster
    ssh USER@{m['cluster']['host']}
    tar xzf {m['batch']}.tar.gz && cd {m['batch']}
    ./submit.sh          # mesh job, then the attitude array
    ./status.sh          # progress (repeat)
    ./collect.sh         # when every attitude is finished: packs {m['batch']}_results.tar.gz
## Back on your Mac
    scp USER@{m['cluster']['host']}:~/{m['batch']}/{m['batch']}_results.tar.gz ~/Downloads/
then CFD tab -> HPC -> Import results (or: python -m airframe_designer.cfd.hpc import ~/Downloads/{m['batch']}_results.tar.gz)

Case layout: cases/<key>/ (system, constant, 0), shared mesh in mesh/, flow sampling dict in template/.
""")


# ----------------------------------------------------------------------------------------------- import

def import_results(tar_path: str | Path, log=None) -> dict:
    """Unpack a <batch>_results.tar.gz and post-process every attitude into the library lib_<kmh>kmh_<quality>-<batch>."""
    tar_path = Path(tar_path).expanduser()
    if not tar_path.exists():
        raise FileNotFoundError(str(tar_path))
    tmp = Path(shutil.os.path.join(shutil.os.path.dirname(str(tar_path)) or ".", f".import_{int(time.time())}"))
    tmp = CFD_ROOT / "_import_tmp" / f"{int(time.time())}"
    tmp.mkdir(parents=True, exist_ok=True)
    try:
        with tarfile.open(tar_path, "r:gz") as tf:
            tf.extractall(tmp, filter="data")
        mf = next(tmp.glob("**/manifest.json"))
        m = json.loads(mf.read_text())
        base = mf.parent
        surface = m["surface"]
        sdir = CFD_ROOT / surface / "surface"
        smeta = json.loads((sdir / "meta.json").read_text())
        if smeta.get("source_hash") != m.get("surface_hash"):
            raise ValueError(f"the surface has changed since the batch was exported ({m.get('surface_hash')} vs {smeta.get('source_hash')})")
        lib_dir = CFD_ROOT / surface / f"lib_{int(round(m['speed_kmh']))}kmh_{m['library_quality']}"
        (lib_dir / "cases").mkdir(parents=True, exist_ok=True)
        V = float(m["speed_kmh"]) / 3.6
        imported, failed = [], []
        for c in m["cases"]:
            src = base / "cases" / c["key"]
            if not (src / "postProcessing").exists():
                failed.append({"key": c["key"], "error": "no postProcessing (not finished?)"})
                continue
            dst = lib_dir / "cases" / c["key"]
            if dst.exists():
                shutil.rmtree(dst)
            shutil.copytree(src, dst)
            tmpl = base / "template" / "flow_grid.json"
            if not (dst / "flow_grid.json").exists() and tmpl.exists():
                shutil.copy(tmpl, dst / "flow_grid.json")
            el = 0.0
            if (dst / "elapsed.txt").exists():
                try:
                    el = float((dst / "elapsed.txt").read_text().split()[-1])
                except Exception:  # noqa: BLE001
                    pass
            try:
                r = C.postprocess_case(dst, c["alpha"], c["beta"], V, m["quality"], int(m["iterations"]), int(m["cores_per_node"]), el, True,
                                       log=log, extra={"hpc_batch": m["batch"], "computed_on": m["cluster"]["host"]})
                imported.append({"key": c["key"], "lift": r["lift"], "drag": r["drag"], "scatter": r["force_scatter"]})
            except Exception as e:  # noqa: BLE001
                failed.append({"key": c["key"], "error": f"{type(e).__name__}: {e}"})
        # library.json so the tab lists it, mesh.json from the cluster's checkMesh
        cells = None
        cm = base / "mesh" / "log.checkMesh"
        if cm.exists():
            cells = C._count_cells(cm.read_text())
        (lib_dir / "mesh").mkdir(exist_ok=True)
        (lib_dir / "mesh" / "mesh.json").write_text(json.dumps({"quality": m["quality"], "cells": cells, "nproc": m["cores_per_node"], "hpc_batch": m["batch"],
                                                                 "built_at": m.get("created_at"), "mesh_check": "cluster", "body_bounds": smeta["bounds_frd"]}))
        lj = {"speed_kmh": m["speed_kmh"], "quality": m["library_quality"], "grid": {"alphas": sorted(set(c["alpha"] for c in m["cases"])), "betas": sorted(set(c["beta"] for c in m["cases"]))},
              "nproc": m["cores_per_node"], "surface": m.get("surface_source"), "surface_hash": m.get("surface_hash"), "hpc_batch": m["batch"],
              "updated_at": datetime.now().astimezone().isoformat(timespec="seconds"), "cases": []}
        (lib_dir / "library.json").write_text(json.dumps(lj, indent=1))
        # mark the local bundle as imported
        local_mf = _batch_root(surface) / m["batch"] / "manifest.json"
        if local_mf.exists():
            lm = json.loads(local_mf.read_text())
            lm["imported_at"] = datetime.now().astimezone().isoformat(timespec="seconds")
            lm["imported"] = len(imported); lm["failed"] = len(failed)
            local_mf.write_text(json.dumps(lm, indent=1))
        out = {"ok": True, "batch": m["batch"], "library_dir": str(lib_dir.relative_to(PROJECT_DIR)), "speed_kmh": m["speed_kmh"],
               "quality": m["library_quality"], "imported": imported, "failed": failed, "cells": cells}
        if log:
            log(f"[hpc] imported {len(imported)} attitudes ({len(failed)} failed) into {lib_dir.name}")
        return out
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser(description="Export a CFD batch for the AUTH cluster, or import its results.")
    sub = ap.add_subparsers(dest="cmd", required=True)
    e = sub.add_parser("export")
    e.add_argument("surface"); e.add_argument("--batch", required=True)
    e.add_argument("--alphas", default="-12,-9,-6,-3,0,3,6,9,12"); e.add_argument("--betas", default="0,5,10")
    e.add_argument("--speed-kmh", type=float, default=60.0); e.add_argument("--quality", default="standard", choices=list(C.QUALITY))
    e.add_argument("--parallel", type=int, default=10, help="attitudes computed at the same time (array throttle)")
    i = sub.add_parser("import")
    i.add_argument("tar")
    a = ap.parse_args()
    if a.cmd == "export":
        print(json.dumps(export_bundle(a.surface, a.batch, [float(x) for x in a.alphas.split(",")], [float(x) for x in a.betas.split(",")],
                                       a.speed_kmh, a.quality, max_parallel_jobs=a.parallel, log=print), indent=1))
    else:
        r = import_results(a.tar, log=print)
        print(json.dumps({k: v for k, v in r.items() if k != "imported"}, indent=1), f"imported {len(r['imported'])}")
