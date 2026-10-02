"""Build airframes/atlas_v3_30.json from the STEP export SMALL_SCALE_V3_30.step (ATLAS V3, 30 Sep 2026).

Masses come ONLY from the labels of the TOP-LEVEL component folders ("<name>__M<grams>g", also "_M<grams>g"):
each top-level folder's label is spread by volume over every solid inside it, so a folder's inner labels are not
added on top (the avionics bay's 250 g already covers its Pixhawk, H-FLOW and 1600 mAh pack). Folders without a
label weigh nothing (clamp caps, Four_feet_assembled, the unnamed "=>[0:1:1:189]" solid). Two labelled folders
with the same label at the same place are one part (duplicate). Geometry from the CAD:
  * fans: the nine XFLY folders; they sit exactly where they were in v34, so the v34 axes are kept and checked;
  * foil fans: on the jet exit line at each foil's trailing edge, exit angle ray-traced on this STEP
    (results/v3_30/foil_rays_step.py: rays straight down every 2.5 mm, slope of the last 5 mm of wall);
  * legs: the two rear outboard feet of Four_feet_assembled and the bottom of Front_foot_assembly (the nose skid);
    the aircraft rests on those three points, which sets the parked pitch;
  * H-FLOW and Pixhawk positions from their solids inside the avionics bay (positions only, not weights).
PX4 gains, fan model (36 N, km 0.002, tau 0.12 s), H-FLOW mount (frame-fixed, tilted 20.4 deg), nose-lift settings
and vibration from airframes/atlas_v3_v34_foils_50_65_50_tuned.json; MC_AIRMODE 0.

  python scripts/atlas_v3_30_from_step.py [--step PATH] [--out airframes/atlas_v3_30.json]
"""
from __future__ import annotations
import argparse, json, math, re, shutil
from pathlib import Path
import numpy as np
from airframe_designer.geometry.airframe import Airframe
from airframe_designer.geometry import knobs
from airframe_designer.geometry.cad import import_step, model_from_import
from airframe_designer.geometry.gear import Leg

ROOT = Path(__file__).resolve().parents[1]
RES = ROOT / "results" / "v3_30"
STEP_REL = "airframes/cad/SMALL_SCALE_V3_30.step"
TEMPLATE = ROOT / "airframes" / "atlas_v3_v34_foils_50_65_50_tuned.json"
V34 = ROOT / "airframes" / "atlas_v3_v34.json"
R_CAD = np.array([[0, 0, -1], [1, 0, 0], [0, -1, 0]], float)      # rows: FRD x, y, z in CAD axes (x right, y up, z aft)
ORIGIN = -R_CAD @ np.array([0.0, 0.45, 0.0])                        # FRD origin on the centreline at boom level
LABEL = re.compile(r"_M(\d+(?:\.\d+)?)g", re.I)
JET_HALF_THICKNESS = 0.015     # m, jet centre off the foil wall at the trailing edge (nozzle height not measured)
FRONT_STRUT = 0.10             # m, nose skid modelled as a short stiff strut down to the skid's lowest point


def frd(p_m) -> np.ndarray:
    return R_CAD @ np.asarray(p_m, float) + ORIGIN


def top_level_solids(step: Path) -> list[dict]:
    """Every top-level folder of the STEP: name, and its solids' (volume, centroid, lowest vertex) in CAD metres."""
    from OCP.Interface import Interface_Static
    from OCP.IFSelect import IFSelect_RetDone
    from OCP.STEPControl import STEPControl_Reader
    from OCP.STEPCAFControl import STEPCAFControl_Reader
    from OCP.TDocStd import TDocStd_Document
    from OCP.TCollection import TCollection_ExtendedString
    from OCP.XCAFDoc import XCAFDoc_DocumentTool
    from OCP.collections import Sequence_TDF_Label as Seq
    from OCP.TDataStd import TDataStd_Name
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopAbs import TopAbs_SOLID, TopAbs_VERTEX
    from OCP.TopoDS import TopoDS
    from OCP.GProp import GProp_GProps
    from OCP.BRepGProp import BRepGProp
    from OCP.BRep import BRep_Tool
    STEPControl_Reader(); Interface_Static.SetCVal_s("xstep.cascade.unit", "M")
    doc = TDocStd_Document(TCollection_ExtendedString("t"))
    r = STEPCAFControl_Reader(); r.SetNameMode(True); Interface_Static.SetCVal_s("xstep.cascade.unit", "M")
    if r.ReadFile(str(step)) != IFSelect_RetDone:
        raise RuntimeError(f"cannot read {step}")
    r.Transfer(doc)
    st = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())

    def name(l):
        n = TDataStd_Name()
        return str(n.Get().ToExtString()) if l.FindAttribute(TDataStd_Name.GetID_s(), n) else ""
    free = Seq(); st.GetFreeShapes(free)
    comps = Seq(); st.GetComponents_s(free.Value(1), comps)
    out = []
    for k in range(1, comps.Length() + 1):
        c = comps.Value(k)
        shape = st.GetShape_s(c)
        solids = []
        ex = TopExp_Explorer(shape, TopAbs_SOLID)
        while ex.More():
            s = TopoDS.Solid(ex.Current())
            g = GProp_GProps(); BRepGProp.VolumeProperties_s(s, g); p = g.CentreOfMass()
            low, vx = None, TopExp_Explorer(s, TopAbs_VERTEX)
            while vx.More():
                q = BRep_Tool.Pnt_s(TopoDS.Vertex(vx.Current()))
                if low is None or q.Y() < low[1]:
                    low = (q.X(), q.Y(), q.Z())
                vx.Next()
            solids.append({"volume": abs(g.Mass()), "centroid": [p.X(), p.Y(), p.Z()], "lowest": low})
            ex.Next()
        out.append({"name": name(c), "solids": solids})
    return out


def labelled_folders(folders):
    """{folder name: {"kg", "solids"}} for labelled folders; exact duplicates (same label, same centroid) dropped."""
    parts, seen = {}, set()
    for f in folders:
        m = LABEL.search(f["name"])
        if not m or not f["solids"]:
            continue
        V = sum(s["volume"] for s in f["solids"])
        c = sum(np.array(s["centroid"]) * s["volume"] for s in f["solids"]) / V
        key = (m.group(1), tuple(np.round(c, 3)))
        if key in seen:
            continue
        seen.add(key)
        parts[f["name"]] = {"kg": float(m.group(1)) / 1000.0, "solids": f["solids"], "volume": V, "centroid": c}
    return parts


def build(step: Path, out: Path, log=print) -> Airframe:
    dst = ROOT / STEP_REL
    if not dst.exists() or dst.stat().st_size != step.stat().st_size:
        shutil.copy2(step, dst)
    folders = top_level_solids(dst)
    parts = labelled_folders(folders)
    af = Airframe.load(TEMPLATE)
    v34 = Airframe.load(V34)
    imported = import_step(dst, log=lambda s: None)
    model = model_from_import(imported, STEP_REL, af.cad)
    # each STEP body to the top-level folder solid with the same centroid and volume
    cand = [(np.array(s["centroid"]), s["volume"], k) for k, p in parts.items() for s in p["solids"]]
    C = np.array([c[0] for c in cand]); V = np.array([c[1] for c in cand])
    owner = {}
    for b in model.bodies:
        b.mass = 0.0
        d = np.linalg.norm(C - np.asarray(b.centroid), axis=1)
        j = int(np.argmin(d))
        if d[j] < 2e-3 and abs(V[j] - b.volume) <= 0.03 * max(b.volume, 1e-9):
            owner[b.id] = cand[j][2]
    vol_by_part = {}
    for bid, k in owner.items():
        vol_by_part[k] = vol_by_part.get(k, 0.0) + next(b.volume for b in model.bodies if b.id == bid)
    for b in model.bodies:
        k = owner.get(b.id)
        if k:
            b.mass = parts[k]["kg"] * b.volume / vol_by_part[k]
    missing = [k for k in parts if k not in vol_by_part]
    af.cad = model
    if isinstance(af.design, dict):
        af.design.pop("cad_links", None); af.design.pop("knobs", None)
    af.mass.items = []

    # fans: unchanged from v34 (checked), foil fans moved onto the new exit lines
    fans = {f["name"]: frd(sum(np.array(s["centroid"]) * s["volume"] for s in f["solids"]) / sum(s["volume"] for s in f["solids"]))
            for f in folders if "XFLY" in f["name"] and LABEL.search(f["name"])}
    exit_deg = json.loads((RES / "foil_exit_deg.json").read_text())
    prof = json.loads((RES / "foil_profiles.json").read_text())
    rot = {r.name: r for r in af.rotors}
    v34rot = {r.name: r for r in v34.rotors}
    for name in ("M7", "M8", "M9"):
        rot[name].pos = list(v34rot[name].pos); rot[name].axis = list(v34rot[name].axis)
    # M1/M2 outer L/R, M3/M4 middle, M5/M6 inner (rounded: mirrored fans differ in the last digits)
    foil = sorted(exit_deg, key=lambda k: (-round(abs(fans[k][1]), 3), fans[k][1]))
    for name, k in zip(("M1", "M2", "M3", "M4", "M5", "M6"), foil):
        d = math.radians(exit_deg[k])
        te = frd(np.array(prof[k][-1][1:4]))
        jet = np.array([-math.cos(d), 0.0, math.sin(d)])
        into_jet = np.array([-math.sin(d), 0.0, -math.cos(d)])
        r = rot[name]
        r.pos = np.round(te + JET_HALF_THICKNESS * into_jet, 4).tolist()
        r.axis = np.round(-jet, 6).tolist()
        r.duct_axis = list(v34rot[name].duct_axis)
    # legs: two rear outboard feet and the nose skid
    feet = next(f for f in folders if f["name"].startswith("Four_feet"))
    rear = sorted([s for s in feet["solids"] if s["lowest"][1] < 0.2], key=lambda s: s["lowest"][0])
    skid = min(next(f for f in folders if f["name"].startswith("Front_foot"))["solids"], key=lambda s: s["lowest"][1])
    tl = {l.name: l for l in af.legs}
    props = {q: getattr(tl["L"], q) for q in ("stiffness", "damping", "friction", "enabled")}
    legs = []
    for nm, s in zip(("L", "R"), rear):
        foot = frd(s["lowest"]) - [0.0, 0.0, tl["L"].foot_radius]    # the foot ball's centre, radius above the contact
        legs.append(Leg.from_points(tl[nm].attach, foot.tolist(), name=nm, foot_radius=tl["L"].foot_radius, **props))
    skid_foot = frd(skid["lowest"])
    legs.append(Leg.from_points((skid_foot - [0.0, 0.0, FRONT_STRUT]).tolist(), skid_foot.tolist(), name="N (front foot)",
                                foot_radius=0.0, **props))
    af.legs = legs
    # parked pitch: the line through the rear feet and the skid, then the hover trim for this geometry
    zr, xr = np.mean([l.foot()[2] + l.foot_radius for l in legs[:2]]), np.mean([l.foot()[0] for l in legs[:2]])
    af.landed_pitch_deg = round(-math.degrees(math.atan2(zr - skid_foot[2], skid_foot[0] - xr)), 2)
    # avionics: positions only
    av = next(f for f in folders if f["name"].startswith("avionics_bay"))
    af.resolve_mass()
    trim = af.trim_hover_pitch()
    if trim is not None:
        af.hover_pitch_deg = round(trim, 2)
    af.px4_overrides["MPC_THR_HOVER"] = round(af.hover_thrust_fraction(), 3)
    af.px4_overrides["MC_AIRMODE"] = 0
    for key in ("nose_lift",):
        af.design[key]["target_pitch_deg"] = af.hover_pitch_deg
    af.design["nose_lower"]["target_pitch_deg"] = af.landed_pitch_deg
    af.name = "ATLAS_V3_30"
    af.notes = ("Built by scripts/atlas_v3_30_from_step.py from SMALL_SCALE_V3_30.step (30 Sep 2026). Masses only from the "
                "labels of the top-level component folders, spread by volume over each folder's solids; unlabelled folders "
                f"weigh nothing. Foil exit angles ray-traced on this STEP: {', '.join(f'{v:.1f}' for v in exit_deg.values())} deg. "
                "Legs on the CAD's rear outboard feet and the front foot (nose skid). PX4 gains, fan model, H-FLOW mount "
                "and nose-lift settings from atlas_v3_v34_foils_50_65_50_tuned.json, MC_AIRMODE 0.")
    knobs.setup_defaults(af, link_front_parts=False)
    ms = af.resolve_mass().mass
    log(f"mass {ms.mass:.3f} kg, CG FRD {np.round(ms.cg, 4).tolist()}, inertia {ms.inertia}, products {ms.inertia_products}")
    log(f"parked {af.landed_pitch_deg} deg, hover trim {af.hover_pitch_deg} deg, MPC_THR_HOVER {af.px4_overrides['MPC_THR_HOVER']}")
    for k, p in sorted(parts.items(), key=lambda kv: -kv[1]["kg"]):
        log(f"  {p['kg'] * 1000:6.0f} g  {len(p['solids']):3d} solids  {p['volume'] * 1e6:8.1f} cm3  FRD {np.round(frd(p['centroid']), 3).tolist()}  {k}")
    unl = [f["name"] for f in folders if not LABEL.search(f["name"])]
    log(f"unlabelled (0 g): {unl}")
    if missing:
        log(f"WARNING: no STEP body matched {missing}")
    for name, k in zip(("M1", "M2", "M3", "M4", "M5", "M6"), foil):
        log(f"  {name}: exit {exit_deg[k]:.2f} deg  pos {rot[name].pos}  axis {rot[name].axis}  ({k})")
    for l in legs:
        log(f"  leg {l.name}: attach {np.round(l.attach, 3).tolist()} foot {np.round(l.foot(), 3).tolist()}")
    if out:
        af.save(out)
    return af


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--step", default="/Users/stefanosfragkoulis/Documents/UtopiaLabs/SMALL_SCALE_V3_30.step")
    ap.add_argument("--out", default=str(ROOT / "airframes" / "atlas_v3_30.json"))
    a = ap.parse_args()
    build(Path(a.step), Path(a.out))
