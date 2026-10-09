"""ATLAS v1 foil candidate (HANDOFF stages S1 to S3, OCP side): place Agent B's reference foils on the full-scale
STEP blender/atlas_v1.stp, measure what changes, loft a SEPARATE candidate lifting-body solid and write its STEP.

The source STEP and the DAT files are never modified (hashes checked before and after). Placement reproduces Agent
B's regional fit (results/reviews/20261005-atlas-v1-step/search_library.py): at each station the plane X = const cuts
the upper shell (shell 1); its max-Y point is the LE, its min-Y point the TE; the normalised profile (LE = stored
min-x point, chord to the mean of the TE endpoints, rigid rotation + uniform scale, no smoothing) is mapped onto that
chord with the upper side towards +Z. Stations and profiles per region are Agent B's diagnostic stations; the left
side (X < 0) is cut and fitted on its own, not mirrored.

Candidate: one solid lofted through the placed profiles from X = -1880 to +1880 mm (29 stations). Between regions the
loft interpolates the profiles (the only blend). The source canopy shell is carried over unchanged; the source upper
and lower shells are replaced by the candidate in the candidate STEP only. Openings (lights, fan inlets) and the tip
beyond |X| = 1880 are NOT in the candidate (reported).

  .venv/bin/python scripts/atlas_v1_foil_candidate.py [--version v001] [--ruled]
Writes results/handoff/T20261005-atlas-v1-foil-references/from_A/<version>/.
"""
from __future__ import annotations
import argparse, hashlib, json, math
from datetime import datetime, timezone
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "blender" / "atlas_v1.stp"
SRC_SHA256 = "72d01577ca0ff11af0075d33d3fdc2d1a39583c2752f6889443af13f1ded13a4"
TASK = "T20261005-atlas-v1-foil-references"
FROM_B = ROOT / "results" / "handoff" / TASK / "from_B"
REGIONS = {   # Agent B's recommended references and diagnostic stations (positive-X CAD mm)
    "centerbody": ("e908.dat", [0, 150, 300, 450]),
    "transition": ("goe05k.dat", [550, 650, 800, 950, 1100]),
    "outer": ("sc20402.dat", [1250, 1400, 1550, 1700, 1800, 1880]),
}
SHELL_NAMES = {0: "canopy", 1: "upper_skin", 2: "lower_skin"}
GRID = np.linspace(0.01, 0.99, 301)          # Agent B's fit grid (fraction of chord)
N_HALF = 81                                  # loft samples per surface, cosine spaced, LE shared


def now() -> str:
    return datetime.now(timezone.utc).isoformat()


def sha256(p: Path) -> str:
    return hashlib.sha256(p.read_bytes()).hexdigest()


# ---------- profiles ----------

def read_profile(path: Path) -> dict:
    """Normalised profile: upper/lower branches as (u, v) sorted LE -> TE, as Agent B normalises (no smoothing)."""
    a = np.array([[float(x) for x in ln.split()] for ln in path.read_text().splitlines()[1:] if len(ln.split()) == 2])
    j = int(np.argmin(a[:, 0]))
    le, te = a[j], (a[0] + a[-1]) / 2
    d = te - le
    c = float(np.linalg.norm(d))
    e = d / c
    n = np.array([-e[1], e[0]])
    uv = np.stack([(a - le) @ e / c, (a - le) @ n / c], axis=1)
    b1, b2 = uv[: j + 1][::-1], uv[j:]                     # both LE -> TE
    upper, lower = (b1, b2) if b1[:, 1].mean() >= b2[:, 1].mean() else (b2, b1)
    return {"name": path.name, "upper": upper, "lower": lower, "points": len(a), "sha256": sha256(path)}


def branch_at(branch: np.ndarray, u: np.ndarray) -> np.ndarray:
    q = branch[np.argsort(branch[:, 0], kind="stable")]
    return np.interp(u, q[:, 0], q[:, 1])


def outline(prof: dict) -> np.ndarray:
    """Closed outline TE(upper) -> LE -> TE(lower), cosine spaced, the same sample count for every profile."""
    u = 0.5 * (1 - np.cos(np.linspace(0, math.pi, N_HALF)))
    up = np.stack([u, branch_at(prof["upper"], u)], axis=1)[::-1]
    lo = np.stack([u, branch_at(prof["lower"], u)], axis=1)[1:]
    return np.concatenate([up, lo])


# ---------- STEP / OCP ----------

def read_shells(path: Path) -> list:
    from OCP.STEPControl import STEPControl_Reader
    from OCP.IFSelect import IFSelect_RetDone
    from OCP.Interface import Interface_Static
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopAbs import TopAbs_SHELL
    r = STEPControl_Reader()
    Interface_Static.SetCVal_s("xstep.cascade.unit", "MM")
    assert r.ReadFile(str(path)) == IFSelect_RetDone
    r.TransferRoots()
    ex, out = TopExp_Explorer(r.OneShape(), TopAbs_SHELL), []
    while ex.More():
        out.append(ex.Current())
        ex.Next()
    return out


def section(shape, x: float) -> np.ndarray:
    """Segments (N, 2, 2) of (Y, Z) mm where the plane X = x cuts the shape."""
    from OCP.BRepAlgoAPI import BRepAlgoAPI_Section
    from OCP.gp import gp_Pln, gp_Pnt, gp_Dir
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopAbs import TopAbs_EDGE
    from OCP.TopoDS import TopoDS
    from OCP.BRepAdaptor import BRepAdaptor_Curve
    from OCP.GCPnts import GCPnts_QuasiUniformDeflection
    op = BRepAlgoAPI_Section(shape, gp_Pln(gp_Pnt(x, 0, 0), gp_Dir(1, 0, 0)), False)
    op.Approximation(False)
    op.Build()
    ex, segs = TopExp_Explorer(op.Shape(), TopAbs_EDGE), []
    while ex.More():
        cu = BRepAdaptor_Curve(TopoDS.Edge(ex.Current()))
        s = GCPnts_QuasiUniformDeflection(cu, 0.025)
        a = np.array([[s.Value(i).Y(), s.Value(i).Z()] for i in range(1, s.NbPoints() + 1)])
        if len(a) > 1:
            segs.append(np.stack([a[:-1], a[1:]], axis=1))
        ex.Next()
    return np.concatenate(segs) if segs else np.zeros((0, 2, 2))


def crossings(segs: np.ndarray, le, e, n, c, grid=GRID) -> tuple[np.ndarray, np.ndarray]:
    """(min v, max v) of the section at each chord fraction (NaN where nothing crosses), as Agent B samples it."""
    u = ((segs - le) @ e) / c
    v = ((segs - le) @ n) / c
    lo, hi = np.full(len(grid), np.nan), np.full(len(grid), np.nan)
    for k, g in enumerate(grid):
        ok = (u.min(1) <= g) & (u.max(1) >= g) & (np.abs(u[:, 1] - u[:, 0]) > 1e-12)
        if ok.any():
            vals = v[ok, 0] + (v[ok, 1] - v[ok, 0]) * (g - u[ok, 0]) / (u[ok, 1] - u[ok, 0])
            lo[k], hi[k] = vals.min(), vals.max()
    return lo, hi


def chord_frame(upper_segs: np.ndarray):
    pts = upper_segs.reshape(-1, 2)
    le, te = pts[np.argmax(pts[:, 0])], pts[np.argmin(pts[:, 0])]       # LE at max Y (nose +Y)
    d = te - le
    c = float(np.linalg.norm(d))
    e = d / c
    n = np.array([-e[1], e[0]])
    if n[1] < 0:
        n = -n
    return le, te, e, n, c


def place(uv: np.ndarray, x: float, le, e, n, c) -> np.ndarray:
    yz = le + c * (uv[:, :1] * e + uv[:, 1:] * n)
    return np.column_stack([np.full(len(uv), x), yz])


def make_wire(p3: np.ndarray):
    """Upper edge TE->LE and lower edge LE->TE (B-splines through the samples; an open TE is part of the lower edge)."""
    from OCP.gp import gp_Pnt
    from OCP.collections import HArray1_gp_Pnt as TColgp_HArray1OfPnt
    from OCP.GeomAPI import GeomAPI_Interpolate
    from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeEdge, BRepBuilderAPI_MakeWire

    def bspline(pts):
        arr = TColgp_HArray1OfPnt(1, len(pts))
        for i, q in enumerate(pts, 1):
            arr.SetValue(i, gp_Pnt(*map(float, q)))
        it = GeomAPI_Interpolate(arr, False, 1e-6)
        it.Perform()
        return it.Curve()

    up, lo = p3[:N_HALF], p3[N_HALF - 1:]
    mw = BRepBuilderAPI_MakeWire()
    mw.Add(BRepBuilderAPI_MakeEdge(bspline(up)).Edge())
    lower = bspline(lo)
    if np.linalg.norm(p3[0] - p3[-1]) > 1e-6:
        # open TE: the TE segment joins the lower curve (exact, C0 at the corner) so every section has two edges,
        # LE and TE(upper) vertices, whatever its TE: the loft needs the same topology across sharp and open TEs
        from OCP.GC import GC_MakeSegment
        from OCP.GeomConvert import GeomConvert, GeomConvert_CompCurveToBSplineCurve
        cc = GeomConvert_CompCurveToBSplineCurve(lower)
        seg = GC_MakeSegment(gp_Pnt(*map(float, p3[-1])), gp_Pnt(*map(float, p3[0]))).Value()
        assert cc.Add(GeomConvert.CurveToBSplineCurve_s(seg), 1e-6)
        lower = cc.BSplineCurve()
    mw.Add(BRepBuilderAPI_MakeEdge(lower).Edge())
    return mw.Wire()


def loft(wires: list, ruled: bool):
    from OCP.BRepOffsetAPI import BRepOffsetAPI_ThruSections
    ts = BRepOffsetAPI_ThruSections(True, ruled, 1e-4)
    ts.CheckCompatibility(True)
    for w in wires:
        ts.AddWire(w)
    ts.Build()
    assert ts.IsDone(), "loft failed"
    return ts.Shape()


def props(shape) -> dict:
    from OCP.GProp import GProp_GProps
    from OCP.BRepGProp import BRepGProp
    from OCP.BRepCheck import BRepCheck_Analyzer
    from OCP.Bnd import Bnd_Box
    from OCP.BRepBndLib import BRepBndLib
    g, a = GProp_GProps(), GProp_GProps()
    BRepGProp.VolumeProperties_s(shape, g)
    BRepGProp.SurfaceProperties_s(shape, a)
    b = Bnd_Box()
    BRepBndLib.AddOptimal_s(shape, b, False, False)
    c = g.CentreOfMass()
    return {"valid": bool(BRepCheck_Analyzer(shape).IsValid()), "volume_m3": g.Mass() * 1e-9,
            "surface_m2": a.Mass() * 1e-6, "volume_centroid_mm": [c.X(), c.Y(), c.Z()],
            "bbox_mm": [b.CornerMin().X(), b.CornerMin().Y(), b.CornerMin().Z(),
                        b.CornerMax().X(), b.CornerMax().Y(), b.CornerMax().Z()]}


def mesh(shape, deflection=0.5) -> tuple[np.ndarray, np.ndarray]:
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.BRep import BRep_Tool
    from OCP.TopLoc import TopLoc_Location
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED
    from OCP.TopoDS import TopoDS
    BRepMesh_IncrementalMesh(shape, deflection, False, 0.2, True)
    vv, ff = [], []
    ex = TopExp_Explorer(shape, TopAbs_FACE)
    while ex.More():
        face = TopoDS.Face(ex.Current())
        loc = TopLoc_Location()
        t = BRep_Tool.Triangulation_s(face, loc)
        if t is not None:
            base = len(vv)
            for j in range(1, t.NbNodes() + 1):
                q = t.Node(j).Transformed(loc.Transformation())
                vv.append([q.X(), q.Y(), q.Z()])
            rev = face.Orientation() == TopAbs_REVERSED
            for j in range(1, t.NbTriangles() + 1):
                a, b, c = t.Triangle(j).Get()
                ff.append([base + a - 1, base + c - 1, base + b - 1] if rev else [base + a - 1, base + b - 1, base + c - 1])
        ex.Next()
    return np.array(vv, float), np.array(ff, int)


def write_step(parts: list[tuple[str, object]], path: Path) -> None:
    from OCP.STEPCAFControl import STEPCAFControl_Writer
    from OCP.STEPControl import STEPControl_AsIs
    from OCP.TDocStd import TDocStd_Document
    from OCP.TCollection import TCollection_ExtendedString
    from OCP.XCAFDoc import XCAFDoc_DocumentTool
    from OCP.TDataStd import TDataStd_Name
    from OCP.Interface import Interface_Static
    from OCP.IFSelect import IFSelect_RetDone
    doc = TDocStd_Document(TCollection_ExtendedString("atlas_v1_candidate"))
    st = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    for name, shape in parts:
        TDataStd_Name.Set_s(st.AddShape(shape, False), TCollection_ExtendedString(name))
    w = STEPCAFControl_Writer()
    Interface_Static.SetCVal_s("write.step.unit", "MM")
    Interface_Static.SetCVal_s("write.step.product.name", "ATLAS_V1_CANDIDATE")
    w.SetNameMode(True)
    assert w.Transfer(doc, STEPControl_AsIs)
    assert w.Write(str(path)) == IFSelect_RetDone


# ---------- main ----------

def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--version", default="v001")
    ap.add_argument("--ruled", action="store_true", help="straight lines between stations instead of a smooth loft")
    args = ap.parse_args()
    out = ROOT / "results" / "handoff" / TASK / "from_A" / args.version
    out.mkdir(parents=True, exist_ok=True)
    stamp = now()

    # S1: frozen inputs
    assert sha256(SRC) == SRC_SHA256, "source STEP hash changed: stop"
    specs = {p["coordinate_file"].split("/")[-1]: p["sha256"]
             for p in json.loads((FROM_B / "profile_specs.json").read_text())["profiles"]}
    profiles = {}
    for region, (dat, _) in REGIONS.items():
        prof = read_profile(FROM_B / "coordinates" / dat)
        assert prof["sha256"] == specs[dat], f"{dat} hash differs from profile_specs.json"
        profiles[region] = prof
    shells = read_shells(SRC)
    assert len(shells) == 3, f"expected 3 shells, found {len(shells)}"

    # S2: place and measure, both sides
    rows, overlays, sections_png = [], [], []
    placed: dict[float, np.ndarray] = {}
    for region, (dat, stations) in REGIONS.items():
        prof = profiles[region]
        ol = outline(prof)
        pu, pl = branch_at(prof["upper"], GRID), branch_at(prof["lower"], GRID)
        for side in (+1, -1):
            for s in stations:
                if side < 0 and s == 0:
                    continue
                x = float(side * s)
                up_segs, lo_segs, can_segs = section(shells[1], x), section(shells[2], x), section(shells[0], x)
                le, te, e, n, c = chord_frame(up_segs)
                _, src_up = crossings(up_segs, le, e, n, c)
                src_lo, _ = crossings(lo_segs, le, e, n, c)
                d_up = (pu - src_up) * c
                d_lo = (pl - src_lo) * c
                okU, okL = np.isfinite(d_up), np.isfinite(d_lo)
                both = okU & okL & (src_up >= src_lo)
                du = GRID[1] - GRID[0]
                src_area = float(np.sum((src_up - src_lo)[both]) * du * c * c)
                cand_area = float(np.sum((pu - pl)[both]) * du * c * c)
                can = {"present": bool(len(can_segs))}
                if len(can_segs):
                    pts = can_segs.reshape(-1, 2)
                    cu, cv = ((pts - le) @ e) / c, ((pts - le) @ n) / c
                    inside = (cu > 0) & (cu < 1)
                    above = (cv[inside] - branch_at(prof["upper"], cu[inside])) * c
                    below = (branch_at(prof["lower"], cu[inside]) - cv[inside]) * c
                    can.update(points=int(inside.sum()),
                               max_above_candidate_upper_mm=float(above.max()) if inside.any() else None,
                               max_below_candidate_lower_mm=float(below.max()) if inside.any() else None,
                               fraction_outside_candidate=float(np.mean((above > 0) | (below > 0))) if inside.any() else None)
                rows.append({
                    "timestamp": stamp, "region": region, "profile": dat, "side": "right" if side > 0 else "left",
                    "cad_x_mm": x, "chord_mm": c, "le_yz_mm": le.tolist(), "te_yz_mm": te.tolist(),
                    "chord_angle_deg": math.degrees(math.atan2(-e[1], -e[0])),     # TE below LE = positive incidence
                    "upper_rms_mm": float(np.sqrt(np.mean(d_up[okU] ** 2))), "upper_max_abs_mm": float(np.abs(d_up[okU]).max()),
                    "upper_coverage": float(okU.mean()),
                    "lower_rms_mm": float(np.sqrt(np.mean(d_lo[okL] ** 2))) if okL.any() else None,
                    "lower_max_abs_mm": float(np.abs(d_lo[okL]).max()) if okL.any() else None,
                    "lower_mean_signed_mm": float(d_lo[okL].mean()) if okL.any() else None,
                    "lower_coverage": float(okL.mean()),
                    "source_max_thickness_mm": float(np.nanmax((src_up - src_lo)[both] * c)) if both.any() else None,
                    "candidate_max_thickness_mm": float(np.max(pu - pl) * c),
                    "source_area_m2": src_area * 1e-6, "candidate_area_m2": cand_area * 1e-6,
                    "area_change_pct": (cand_area / src_area - 1) * 100 if src_area > 0 else None,
                    "area_compared_fraction": float(both.mean()),
                    "canopy": can,
                })
                p3 = place(ol, x, le, e, n, c)
                placed[x] = p3
                overlays.append({"region": region, "profile": dat, "cad_x_mm": x, "points_mm": p3.round(4).tolist()})
                if side > 0:
                    sections_png.append((region, dat, x, up_segs, lo_segs, can_segs, p3[:, 1:]))
                print(f"{region:10s} x {x:+7.0f}  c {c:7.1f}  up rms {rows[-1]['upper_rms_mm']:6.2f}  "
                      f"lo rms {rows[-1]['lower_rms_mm'] or float('nan'):7.2f}  area {rows[-1]['area_change_pct'] or float('nan'):+6.1f} %",
                      flush=True)

    # S2: the candidate solid
    xs = sorted(placed)
    try:
        solid = loft([make_wire(placed[x]) for x in xs], ruled=args.ruled)
        cand = props(solid)
        loft_kind = "ruled" if args.ruled else "smooth"
        if not cand["valid"] and not args.ruled:
            print("smooth loft invalid: retrying ruled", flush=True)
            solid = loft([make_wire(placed[x]) for x in xs], ruled=True)
            cand, loft_kind = props(solid), "ruled (smooth failed BRepCheck)"
    except AssertionError:
        solid = loft([make_wire(placed[x]) for x in xs], ruled=True)
        cand, loft_kind = props(solid), "ruled (smooth failed to build)"
    print("candidate", loft_kind, json.dumps(cand), flush=True)
    src_props = {SHELL_NAMES[i]: props(s) for i, s in enumerate(shells)}

    # S3: STEP (candidate solid + unchanged canopy shell), meshes for Blender
    step = out / f"atlas_v1_cand_{args.version}.step"
    write_step([(f"ATLAS_V1_CAND_{args.version.upper()}_LIFTING_BODY", solid), ("SOURCE_SHELL_0_CANOPY", shells[0])], step)
    meshes = {f"source_{SHELL_NAMES[i]}": mesh(s) for i, s in enumerate(shells)}
    meshes["candidate_lifting_body"] = mesh(solid)
    np.savez_compressed(out / "meshes_mm.npz", **{f"{k}__v": v for k, (v, f) in meshes.items()},
                        **{f"{k}__f": f for k, (v, f) in meshes.items()})
    (out / "overlays_mm.json").write_text(json.dumps(overlays))
    assert sha256(SRC) == SRC_SHA256

    summary = {
        "timestamp": stamp, "task": TASK, "version": args.version, "author": "Agent A",
        "status": "CANDIDATE FOR REVIEW, NOT ACCEPTED; source unchanged",
        "source": {"file": str(SRC.relative_to(ROOT)), "sha256": SRC_SHA256, "shells": src_props},
        "profiles": {r: {"file": p["name"], "sha256": p["sha256"], "points": p["points"]} for r, p in profiles.items()},
        "placement": "Agent B regional fit: shell-1 section LE = max Y, TE = min Y, upper side +Z, rigid + uniform "
                     "scale, no reflection; left side cut and fitted on its own",
        "loft": {"kind": loft_kind, "stations_mm": xs, "samples_per_surface": N_HALF, **cand},
        "step": {"file": step.name, "sha256": sha256(step), "parts": [
            f"ATLAS_V1_CAND_{args.version.upper()}_LIFTING_BODY (solid, lofted)", "SOURCE_SHELL_0_CANOPY (open shell, unchanged)"]},
        "not_in_candidate": ["light openings and any fan openings of the source skins",
                             "tip beyond |X| = 1880 mm (source half span 1902.5 mm)",
                             "source upper/lower shell detail between stations (replaced by the loft)"],
        "stations": rows,
    }
    (out / "fit_metrics.json").write_text(json.dumps(summary, indent=1))
    plot_sections(sections_png, out / "sections_right.png")
    print("wrote", out)


def plot_sections(items, path: Path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    cols = 3
    rows = math.ceil(len(items) / cols)
    fig, axs = plt.subplots(rows, cols, figsize=(cols * 6, rows * 2.6))
    for ax, (region, dat, x, up, lo, can, cand) in zip(axs.ravel(), items):
        for segs, col, lab in ((up, "#1f6feb", "source upper"), (lo, "#8250df", "source lower"), (can, "#bf8700", "canopy")):
            for k, s in enumerate(segs):
                ax.plot(s[:, 0], s[:, 1], color=col, lw=0.8, label=lab if k == 0 else None)
        ax.plot(cand[:, 0], cand[:, 1], color="#cf222e", lw=1.2, label=f"candidate {dat}")
        ax.set_title(f"{region}  X = {x:+.0f} mm", fontsize=9)
        ax.set_aspect("equal")
        ax.invert_xaxis()                     # nose (+Y) on the left
        ax.tick_params(labelsize=7)
    for ax in axs.ravel()[len(items):]:
        ax.axis("off")
    axs.ravel()[0].legend(fontsize=7, loc="lower left")
    fig.suptitle("ATLAS v1 candidate sections (right side): CAD Y (nose left) vs Z, mm", fontsize=11)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


if __name__ == "__main__":
    main()
