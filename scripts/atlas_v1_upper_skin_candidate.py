"""ATLAS v1 candidate v002 (user, 5 Oct 2026, option 1): ONLY the upper skin (source shell 1) is replaced by the foil
upper surfaces; the lower skin (shell 2, with every opening: rear fan inlets/outlets, head and tail light) and the
canopy (shell 0) are carried over unchanged. Shell 1 itself has no openings (one free boundary).

Placement is v001's (Agent B's fit: shell-1 section LE = max Y, TE = min Y, upper side +Z, rigid + uniform scale,
no smoothing, see scripts/atlas_v1_foil_candidate.py), using only each profile's UPPER branch, LE -> TE. Stations
(STATIONS below) include all of Agent B's and are denser so the skin follows the source planform (LE and TE lines):
every 50 mm, every 10 mm through the body-wing junction (450..550, where the chord drops 2.3 -> 1.6 m), every 25 mm
on the tip out to 1900 (source tip |X| 1900.4). In Agent B's gaps (450..550, 1100..1250) the upper ordinate is a
smoothstep blend (3t^2 - 2t^3 of the span fraction) of the two neighbouring profiles: the only blend. A linear
blend creased the skin at both ends of the gap (spanwise curvature ~12x the source's at X 452 / 545). One smooth open loft through all stations,
left and right fitted on their own.

Measured: the fit at every station, the deviation from the source upper skin every 10 mm of span (between stations
too), the seam gaps along the lower skin's outer edge and the canopy's edge where they met the source upper skin,
and the canopy against the new skin.

  .venv/bin/python scripts/atlas_v1_upper_skin_candidate.py [--version v002]
Writes results/handoff/T20261005-atlas-v1-foil-references/from_A/<version>/.
"""
from __future__ import annotations
import argparse, json, math, sys
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import atlas_v1_foil_candidate as A

def smoothstep(t: float) -> float:
    return t * t * (3 - 2 * t)


# (|X| mm, region, profile A, profile B or None, weight of B): Agent B's stations are all in this list
STATIONS = ([(x, "centerbody", "centerbody", None, 0.0) for x in range(0, 451, 50)]
            + [(x, "blend", "centerbody", "transition", smoothstep((x - 450) / 100)) for x in range(460, 541, 10)]
            + [(x, "transition", "transition", None, 0.0) for x in range(550, 1101, 50)]
            + [(x, "blend", "transition", "outer", smoothstep((x - 1100) / 150)) for x in range(1125, 1226, 25)]
            + [(x, "outer", "outer", None, 0.0) for x in list(range(1250, 1701, 50)) + list(range(1725, 1876, 25))
               + [1880, 1890, 1895, 1898, 1900]])
B_STATIONS = {0, 150, 300, 450, 550, 650, 800, 950, 1100, 1250, 1400, 1550, 1700, 1800, 1880}
SEAM_SAMPLES = 600
JOINED_MM = 1.0
FIT_TOL_MM = 0.5          # least-squares B-spline: every placed point within this of the skin           # a boundary point within this of the source upper skin was joined to it


def upper_at(profiles: dict, a: str, b: str | None, w: float, u: np.ndarray) -> np.ndarray:
    v = A.branch_at(profiles[a]["upper"], u)
    return v if b is None else (1 - w) * v + w * A.branch_at(profiles[b]["upper"], u)


def upper_curve(profiles: dict, a: str, b: str | None, w: float) -> np.ndarray:
    u = 0.5 * (1 - np.cos(np.linspace(0, math.pi, A.N_HALF)))
    return np.stack([u, upper_at(profiles, a, b, w, u)], axis=1)            # LE -> TE


def grid_surface(sections: list[np.ndarray]):
    """One B-spline face fitted by least squares to every placed point within FIT_TOL_MM (rows = stations across the
    span, columns = the shared cosine chord samples LE -> TE). A ThruSections loft through ~115 sections did not
    finish in 18 min; exact interpolation rippled spanwise (~1-2 mm at ~10 mm wavelength, +3.9 % area)."""
    from OCP.gp import gp_Pnt
    from OCP.collections import Array2_gp_Pnt
    from OCP.GeomAPI import GeomAPI_PointsToBSplineSurface
    from OCP.GeomAbs import GeomAbs_C2
    from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
    from OCP.BRepBuilderAPI import BRepBuilderAPI_Sewing
    grid = Array2_gp_Pnt(1, len(sections), 1, sections[0].shape[0])
    for i, sec in enumerate(sections, 1):
        for j, q in enumerate(sec, 1):
            grid.SetValue(i, j, gp_Pnt(*map(float, q)))
    api = GeomAPI_PointsToBSplineSurface()
    api.Init(grid, 3, 8, GeomAbs_C2, FIT_TOL_MM)
    assert api.IsDone(), "upper-skin surface fit failed"
    face = BRepBuilderAPI_MakeFace(api.Surface(), 1e-6).Face()
    sew = BRepBuilderAPI_Sewing(1e-3)
    sew.Add(face)
    sew.Perform()
    return sew.SewedShape()


def spanwise_smoothness(src_upper, new_upper, ys=(-300.0, -800.0, -1500.0)) -> list[dict]:
    """Height along spanwise lines (planes Y = const, X 0..1900 every 1 mm): largest second difference of Z (mm per
    mm^2, a ripple detector) for the source and the new skin."""
    from OCP.BRepAlgoAPI import BRepAlgoAPI_Section
    from OCP.gp import gp_Pln, gp_Pnt, gp_Dir
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopAbs import TopAbs_EDGE
    from OCP.TopoDS import TopoDS
    from OCP.BRepAdaptor import BRepAdaptor_Curve
    from OCP.GCPnts import GCPnts_QuasiUniformDeflection

    def zline(shape, y):
        op = BRepAlgoAPI_Section(shape, gp_Pln(gp_Pnt(0, y, 0), gp_Dir(0, 1, 0)), False)
        op.Approximation(False)
        op.Build()
        ex, pts = TopExp_Explorer(op.Shape(), TopAbs_EDGE), []
        while ex.More():
            cu = BRepAdaptor_Curve(TopoDS.Edge(ex.Current()))
            sm = GCPnts_QuasiUniformDeflection(cu, 0.01)
            pts += [[sm.Value(i).X(), sm.Value(i).Z()] for i in range(1, sm.NbPoints() + 1)]
            ex.Next()
        p = np.array(pts)
        return p[np.argsort(p[:, 0])]

    rows = []
    for y in ys:
        a, b = zline(src_upper, y), zline(new_upper, y)
        lo, hi = max(0.0, a[:, 0].min(), b[:, 0].min()), min(1900.0, a[:, 0].max(), b[:, 0].max())
        x = np.arange(math.ceil(lo) + 5, math.floor(hi) - 5, 1.0)
        za, zb = np.interp(x, a[:, 0], a[:, 1]), np.interp(x, b[:, 0], b[:, 1])
        rows.append({"cad_y_mm": y, "x_range_mm": [float(x[0]), float(x[-1])],
                     "source_max_second_diff": float(np.abs(np.diff(za, 2)).max()),
                     "new_max_second_diff": float(np.abs(np.diff(zb, 2)).max()),
                     "height_change_mm": {"mean": float((zb - za).mean()), "max_abs": float(np.abs(zb - za).max())}})
    return rows


def free_boundaries(shape) -> list:
    """Closed free-boundary wires, longest first, as (length mm, wire)."""
    from OCP.ShapeAnalysis import ShapeAnalysis_FreeBounds
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopAbs import TopAbs_WIRE
    from OCP.GProp import GProp_GProps
    from OCP.BRepGProp import BRepGProp
    fb = ShapeAnalysis_FreeBounds(shape, 1e-3, False, False)
    ex, out = TopExp_Explorer(fb.GetClosedWires(), TopAbs_WIRE), []
    while ex.More():
        g = GProp_GProps()
        BRepGProp.LinearProperties_s(ex.Current(), g)
        out.append((g.Mass(), ex.Current()))
        ex.Next()
    return sorted(out, key=lambda t: -t[0])


def sample_wire(wire, n: int) -> np.ndarray:
    from OCP.BRepAdaptor import BRepAdaptor_CompCurve
    from OCP.GCPnts import GCPnts_UniformAbscissa
    from OCP.TopoDS import TopoDS
    cu = BRepAdaptor_CompCurve(TopoDS.Wire(wire))
    ua = GCPnts_UniformAbscissa(cu, n)
    assert ua.IsDone()
    return np.array([[cu.Value(ua.Parameter(i)).X(), cu.Value(ua.Parameter(i)).Y(), cu.Value(ua.Parameter(i)).Z()]
                     for i in range(1, ua.NbPoints() + 1)])


def distances(points: np.ndarray, shape) -> np.ndarray:
    from OCP.BRepExtrema import BRepExtrema_DistShapeShape
    from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeVertex
    from OCP.gp import gp_Pnt
    out = np.empty(len(points))
    for k, p in enumerate(points):
        d = BRepExtrema_DistShapeShape(BRepBuilderAPI_MakeVertex(gp_Pnt(*map(float, p))).Vertex(), shape)
        out[k] = d.Value() if d.IsDone() else np.nan
    return out


def seam(name: str, boundary_pts: np.ndarray, src_upper, new_upper) -> dict:
    d_src = distances(boundary_pts, src_upper)
    joined = d_src < JOINED_MM
    d_new = distances(boundary_pts[joined], new_upper) if joined.any() else np.array([])
    worst = boundary_pts[joined][int(np.argmax(d_new))] if len(d_new) else None
    return {"edge": name, "samples": len(boundary_pts), "joined_to_source_upper": int(joined.sum()),
            "gap_to_new_upper_mm": {"median": float(np.median(d_new)), "p95": float(np.percentile(d_new, 95)),
                                    "max": float(d_new.max())} if len(d_new) else None,
            "worst_point_mm": worst.round(1).tolist() if worst is not None else None,
            "_pts": boundary_pts[joined], "_gap": d_new}


def spanwise_deviation(src_upper, new_upper, xs) -> list[dict]:
    """Per plane X: distance from the source upper-skin section to the new one (points of one to the polyline of the
    other, both ways), mm."""
    from scipy.spatial import cKDTree
    rows = []
    for x in xs:
        a, b = A.section(src_upper, float(x)).reshape(-1, 2), A.section(new_upper, float(x)).reshape(-1, 2)
        if not len(a) or not len(b):
            continue
        d1, _ = cKDTree(b).query(a)
        d2, _ = cKDTree(a).query(b)
        d = np.concatenate([d1, d2])
        rows.append({"cad_x_mm": float(x), "rms_mm": float(np.sqrt(np.mean(d ** 2))), "max_mm": float(d.max())})
    return rows


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--version", default="v002")
    args = ap.parse_args()
    out = A.ROOT / "results" / "handoff" / A.TASK / "from_A" / args.version
    out.mkdir(parents=True, exist_ok=True)
    stamp = A.now()

    assert A.sha256(A.SRC) == A.SRC_SHA256, "source STEP hash changed: stop"
    specs = {p["coordinate_file"].split("/")[-1]: p["sha256"]
             for p in json.loads((A.FROM_B / "profile_specs.json").read_text())["profiles"]}
    assert B_STATIONS <= {x for x, *_ in STATIONS}
    profiles = {}
    for region, (dat, _) in A.REGIONS.items():
        profiles[region] = A.read_profile(A.FROM_B / "coordinates" / dat)
        assert profiles[region]["sha256"] == specs[dat], f"{dat} hash differs from profile_specs.json"
    shells = A.read_shells(A.SRC)
    canopy, src_upper, lower = shells
    assert len(free_boundaries(src_upper)) == 1, "source upper skin has openings: v002 assumes it has none"

    rows, overlays, placed, png = [], [], {}, []
    for s, region, pa, pb, w in STATIONS:
        dat = profiles[pa]["name"] if pb is None else f"{profiles[pa]['name']}+{profiles[pb]['name']}@{w:.2f}"
        curve = upper_curve(profiles, pa, pb, w)
        pu = upper_at(profiles, pa, pb, w, A.GRID)
        for side in (+1, -1):
            if side < 0 and s == 0:
                continue
            x = float(side * s)
            up_segs = A.section(src_upper, x)
            le, te, e, n, c = A.chord_frame(up_segs)
            _, src_up = A.crossings(up_segs, le, e, n, c)
            d_up = (pu - src_up) * c
            ok = np.isfinite(d_up)
            row = {"timestamp": stamp, "region": region, "profile": dat, "side": "right" if side > 0 else "left",
                   "cad_x_mm": x, "agent_b_station": s in B_STATIONS, "chord_mm": c,
                   "chord_angle_deg": math.degrees(math.atan2(-e[1], -e[0])),
                   "upper_rms_mm": float(np.sqrt(np.mean(d_up[ok] ** 2))), "upper_max_abs_mm": float(np.abs(d_up[ok]).max()),
                   "upper_mean_signed_mm": float(d_up[ok].mean()), "upper_coverage": float(ok.mean())}
            can_segs = A.section(canopy, x)
            if len(can_segs):
                pts = can_segs.reshape(-1, 2)
                cu, cv = ((pts - le) @ e) / c, ((pts - le) @ n) / c
                inside = (cu > 0) & (cu < 1)
                if inside.any():
                    above = (cv[inside] - upper_at(profiles, pa, pb, w, cu[inside])) * c
                    row["canopy_max_above_new_upper_mm"] = float(above.max())
            rows.append(row)
            p3 = A.place(curve, x, le, e, n, c)
            placed[x] = p3
            overlays.append({"region": region, "profile": dat, "cad_x_mm": x, "closed": False,
                             "points_mm": p3.round(4).tolist()})
            if side > 0 and s in B_STATIONS | {490, 1900}:
                png.append((region, dat, x, up_segs, A.section(lower, x), can_segs, p3[:, 1:]))
            if side > 0:
                print(f"{region:10s} x {x:+7.0f}  c {c:7.1f}  up rms {row['upper_rms_mm']:6.2f}  max {row['upper_max_abs_mm']:6.2f}", flush=True)

    xs = sorted(placed)
    new_upper = grid_surface([placed[x] for x in xs])
    from OCP.BRepCheck import BRepCheck_Analyzer
    from OCP.GProp import GProp_GProps
    from OCP.BRepGProp import BRepGProp
    valid = bool(BRepCheck_Analyzer(new_upper).IsValid())
    area = {}
    for k, sh in (("source_upper_skin", src_upper), ("candidate_upper_skin", new_upper)):
        g = GProp_GProps()
        BRepGProp.SurfaceProperties_s(sh, g)
        area[k] = g.Mass() * 1e-6
    print("loft valid", valid, "areas m2", area, flush=True)

    dev = spanwise_deviation(src_upper, new_upper, np.arange(-1900, 1901, 10))
    print("spanwise deviation max", max(r["max_mm"] for r in dev), flush=True)
    smooth = spanwise_smoothness(src_upper, new_upper)
    print("smoothness", json.dumps(smooth), flush=True)

    lower_edge = free_boundaries(lower)[0][1]              # the outer edge of the lower skin (openings are shorter)
    canopy_edge = free_boundaries(canopy)[0][1]
    seams = [seam("lower skin outer edge", sample_wire(lower_edge, SEAM_SAMPLES), src_upper, new_upper),
             seam("canopy edge", sample_wire(canopy_edge, SEAM_SAMPLES), src_upper, new_upper)]
    for s in seams:
        print("seam", s["edge"], s["joined_to_source_upper"], s["gap_to_new_upper_mm"], flush=True)

    step = out / f"atlas_v1_cand_{args.version}.step"
    A.write_step([(f"ATLAS_V1_CAND_{args.version.upper()}_UPPER_SKIN", new_upper),
                  ("SOURCE_SHELL_2_LOWER_SKIN", lower), ("SOURCE_SHELL_0_CANOPY", canopy)], step)
    meshes = {f"source_{A.SHELL_NAMES[i]}": A.mesh(s) for i, s in enumerate(shells)}
    meshes["candidate_upper_skin"] = A.mesh(new_upper)
    np.savez_compressed(out / "meshes_mm.npz", **{f"{k}__v": v for k, (v, f) in meshes.items()},
                        **{f"{k}__f": f for k, (v, f) in meshes.items()})
    (out / "overlays_mm.json").write_text(json.dumps(overlays))
    assert A.sha256(A.SRC) == A.SRC_SHA256

    summary = {
        "timestamp": stamp, "task": A.TASK, "version": args.version, "author": "Agent A",
        "status": "CANDIDATE FOR REVIEW (user chose option 1: upper skin only), NOT ACCEPTED; source unchanged",
        "source": {"file": str(A.SRC.relative_to(A.ROOT)), "sha256": A.SRC_SHA256},
        "profiles": {r: {"file": p["name"], "sha256": p["sha256"], "points": p["points"]} for r, p in profiles.items()},
        "replaced": "shell 1 (upper skin) only; shells 0 (canopy) and 2 (lower skin, all openings) unchanged",
        "loft": {"kind": f"one least-squares B-spline surface (C2, degree 3-8) within {FIT_TOL_MM} mm of every placed point "
                         "(all stations x 81 chord samples)", "stations_mm": xs, "valid": valid,
                 "agent_b_stations_mm": sorted(B_STATIONS),
                 "blends": "450..550 E908 -> GOE 5K, 1100..1250 GOE 5K -> SC(2)-0402, upper ordinate, smoothstep in span",
                 **{f"area_{k}_m2": v for k, v in area.items()}},
        "step": {"file": step.name, "sha256": A.sha256(step),
                 "parts": [f"ATLAS_V1_CAND_{args.version.upper()}_UPPER_SKIN (open surface, new)",
                           "SOURCE_SHELL_2_LOWER_SKIN (unchanged)", "SOURCE_SHELL_0_CANOPY (unchanged)"]},
        "seams": [{k: v for k, v in s.items() if not k.startswith("_")} for s in seams],
        "spanwise_deviation": dev,
        "spanwise_smoothness": smooth,
        "stations": rows,
    }
    (out / "fit_metrics.json").write_text(json.dumps(summary, indent=1))
    plot_sections(png, out / "sections_right.png")
    plot_deviation(dev, seams, out / "deviation_and_seams.png")
    print("wrote", out)


def plot_sections(items, path: Path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    cols = 4
    rows = math.ceil(len(items) / cols)
    fig, axs = plt.subplots(rows, cols, figsize=(cols * 5, rows * 2.4))
    for ax, (region, dat, x, up, lo, can, new) in zip(axs.ravel(), items):
        for segs, col, lab in ((up, "#1f6feb", "source upper (replaced)"), (lo, "#8250df", "source lower (kept)"),
                               (can, "#bf8700", "canopy (kept)")):
            for k, s in enumerate(segs):
                ax.plot(s[:, 0], s[:, 1], color=col, lw=0.8, label=lab if k == 0 else None)
        ax.plot(new[:, 0], new[:, 1], color="#cf222e", lw=1.1, label="v002 upper skin")
        ax.set_title(f"{region} {dat.replace('.dat', '')}  X = {x:+.0f} mm", fontsize=8)
        ax.set_aspect("equal")
        ax.invert_xaxis()
        ax.tick_params(labelsize=7)
    for ax in axs.ravel()[len(items):]:
        ax.axis("off")
    axs.ravel()[0].legend(fontsize=7, loc="lower left")
    fig.suptitle("ATLAS v1 candidate v002 sections (right side): CAD Y (nose left) vs Z, mm", fontsize=11)
    fig.tight_layout()
    fig.savefig(path, dpi=105)
    plt.close(fig)


def plot_deviation(dev, seams, path: Path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(14, 4.2))
    x = [r["cad_x_mm"] for r in dev]
    a1.plot(x, [r["max_mm"] for r in dev], color="#cf222e", lw=1.2, label="max")
    a1.plot(x, [r["rms_mm"] for r in dev], color="#1f6feb", lw=1.2, label="RMS")
    for b in (500, 1175):
        for sgn in (1, -1):
            a1.axvline(sgn * b, color="#8c959f", lw=0.6, ls="--")
    a1.set_xlabel("CAD X (span), mm")
    a1.set_ylabel("new vs source upper skin, mm")
    a1.set_title("Upper-skin change along the span (every 10 mm; dashed: region blends)")
    a1.legend()
    a1.grid(alpha=0.3)
    for s, col in zip(seams, ("#8250df", "#bf8700")):
        if len(s["_gap"]):
            a2.scatter(s["_pts"][:, 0], s["_gap"], s=6, color=col, label=f"{s['edge']} ({s['joined_to_source_upper']} pts)")
    a2.set_xlabel("CAD X of the seam point, mm")
    a2.set_ylabel("gap to v002 upper skin, mm")
    a2.set_title("Seam gaps where the kept parts met the old upper skin")
    a2.legend(fontsize=8)
    a2.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


if __name__ == "__main__":
    main()
