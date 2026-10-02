"""Build airframes/atlas_v3_v34.json from the live Fusion design ATLAS / V3 / SMALL_SCALE_V3 (read through Fusion's MCP).

Inputs (written by the Fusion MCP scripts in results/fusion_v3/, see brain/architecture/atlas-v3-fusion.md):
  tree.json            every occurrence with its bodies (volume, centroid, bounding box), Fusion units cm
  transforms.json      the fans' (and batteries') occurrence matrices
  foil_exit_deg.json   the jet's exit angle below the duct line at each foil fan, ray-traced on the B-rep
  airframes/cad/SMALL_SCALE_V3_v34.step   the same design exported to STEP (meshes, inertia)

Masses come ONLY from the part names: "<name>__M<grams>g" (also "_M<grams>g"). A part without a label, and an
unlabelled copy of a labelled part (the "(Mirror)" duplicates), weighs nothing here. A label covers every body under
that occurrence except bodies under a deeper labelled occurrence (H-FLOW and the 1600 mAh pack in the avionics bay
are their own parts). The label's mass is spread over its bodies by volume, so inertia comes from the real shapes.

Foil fans: the fan pulls air in from rest, so the whole fan + foil force acts along the jet leaving the trailing
edge: the rotor sits on that exit line (jet centre JET_HALF_THICKNESS off the wall) and points against the jet.

  python scripts/atlas_v3_from_fusion.py [--battery-dx M] [--battery-dz M] [--out airframes/atlas_v3_v34.json]
"""
from __future__ import annotations

import argparse, json, re
from pathlib import Path

import numpy as np

from airframe_designer.geometry.airframe import Airframe
from airframe_designer.geometry import knobs
from airframe_designer.geometry.cad import import_step, model_from_import

ROOT = Path(__file__).resolve().parents[1]
FUS = ROOT / "results" / "fusion_v3"
STEP_REL = "airframes/cad/SMALL_SCALE_V3_v34.step"
TEMPLATE = ROOT / "airframes" / "atlas_v3_small_graded.json"   # PX4 gains, fan model, legs, vibration, knobs
R_CAD = np.array([[0, 0, -1], [1, 0, 0], [0, -1, 0]], float)      # rows: FRD x, y, z in CAD axes
ORIGIN = -R_CAD @ np.array([0.0, 0.45, 0.0])                        # FRD origin on the centreline at boom level
LABEL = re.compile(r"_M(\d+(?:\.\d+)?)g", re.I)
JET_HALF_THICKNESS = 0.015     # m, jet centre off the foil wall at the trailing edge (nozzle height not measured)


def frd(p_m) -> np.ndarray:
    return R_CAD @ np.asarray(p_m, float) + ORIGIN


def owner_of(path: str) -> str | None:
    """The deepest labelled occurrence on this path (the part whose label pays for these bodies)."""
    parts = path.split("+")
    own = None
    for i, p in enumerate(parts):
        if LABEL.search(p):
            own = "+".join(parts[:i + 1])
    return own


def labelled_parts(tree: dict) -> dict[str, dict]:
    parts: dict[str, dict] = {}
    for o in tree["occurrences"]:
        own = owner_of(o["path"])
        if own is None:
            continue
        seg = own.split("+")[-1]
        p = parts.setdefault(own, {"label_kg": float(LABEL.search(seg).group(1)) / 1000.0, "bodies": []})
        p["bodies"] += o["bodies"]
    # a labelled part that is an exact copy (same label, same place) of another is a duplicate: keep the first
    seen, out = [], {}
    for k, p in parts.items():
        V = sum(b["volume_cm3"] for b in p["bodies"])
        c = sum(np.array(b["com_cm"]) * b["volume_cm3"] for b in p["bodies"]) / V / 100.0 if V > 0 else None
        p["volume_m3"], p["centroid_cad"] = V * 1e-6, c
        key = (round(p["label_kg"], 4), None if c is None else tuple(np.round(c, 3)))
        if key in seen:
            p["duplicate_of"] = next(kk for kk, pp in out.items() if (round(pp["label_kg"], 4), tuple(np.round(pp["centroid_cad"], 3))) == key)
            continue
        seen.append(key)
        out[k] = p
    return out


def match_bodies(model, parts) -> dict[str, list[str]]:
    """STEP body ids per labelled part: each STEP body goes to the Fusion body with the same centroid and volume."""
    fus = []
    for k, p in parts.items():
        for b in p["bodies"]:
            if b["com_cm"] is not None:
                fus.append((np.array(b["com_cm"]) / 100.0, b["volume_cm3"] * 1e-6, k))
    C = np.array([f[0] for f in fus]); V = np.array([f[1] for f in fus])
    got: dict[str, list[str]] = {k: [] for k in parts}
    for b in model.bodies:
        d = np.linalg.norm(C - np.asarray(b.centroid), axis=1)
        j = int(np.argmin(d + 1e3 * np.abs(V - b.volume) / max(b.volume, 1e-12) * 1e-3))
        if d[j] < 2e-3 and abs(V[j] - b.volume) <= 0.03 * max(b.volume, 1e-9):
            got[fus[j][2]].append(b.id)
    return got


def build(battery_dx=0.0, battery_dz=0.0, out=ROOT / "airframes" / "atlas_v3_v34.json", log=print) -> Airframe:
    tree = json.loads((FUS / "tree.json").read_text())
    tr = json.loads((FUS / "transforms.json").read_text())["transforms"]
    exit_deg = json.loads((FUS / "foil_exit_deg_edge.json").read_text())
    af = Airframe.load(TEMPLATE)
    imported = import_step(ROOT / STEP_REL, log=lambda s: None)
    model = model_from_import(imported, STEP_REL, af.cad)
    parts = labelled_parts(tree)
    ids = match_bodies(model, parts)
    by_id = {b.id: b for b in model.bodies}
    for b in model.bodies:
        b.mass = 0.0
    report = []
    for k, p in parts.items():
        bs = [by_id[i] for i in ids[k]]
        V = sum(b.volume for b in bs)
        for b in bs:
            b.mass = p["label_kg"] * b.volume / V
        report.append((k, p["label_kg"], len(bs), V * 1e6))
        if not bs:
            log(f"WARNING: no STEP body matched {k}")
    af.cad = model
    if isinstance(af.design, dict):          # the template's links point at the old STEP's body ids
        af.design.pop("cad_links", None); af.design.pop("knobs", None)
    af.mass.items = []                   # everything that weighs something is now a labelled CAD part
    # fans
    rot = {r.name: r for r in af.rotors}
    fan = {}
    for k, v in tr.items():
        if "XFLY" in k and "_M330g" in k:
            m = np.array(v["m"]).reshape(4, 4)
            fan[k] = (frd(m[:3, 3] / 100.0), R_CAD @ (m[:3, :3] @ np.array([0.0, 1.0, 0.0])))   # local +y is the fan's thrust axis
    # nose fans: M7 left (y < 0), M8 right, M9 centre
    nose = sorted([k for k in fan if fan[k][0][0] > 0.2], key=lambda k: fan[k][0][1])
    for name, k in zip(("M7", "M9", "M8"), nose):
        rot[name].pos = np.round(fan[k][0], 4).tolist(); rot[name].axis = np.round(fan[k][1], 6).tolist()
    # foil fans: M1/M2 outer L/R, M3/M4 middle, M5/M6 inner
    foil = [k for k in fan if fan[k][0][0] < 0.2]
    order = sorted(foil, key=lambda k: (-abs(fan[k][0][1]), fan[k][0][1]))
    prof = json.loads((FUS / "foil_profiles.json").read_text())
    for name, k in zip(("M1", "M2", "M3", "M4", "M5", "M6"), order):
        d = np.radians(exit_deg[k])
        pts = [h for z, h in prof[k] if h]
        te = frd(np.array(pts[-1][:3]) / 100.0)                       # trailing edge on the wall, FRD
        jet = np.array([-np.cos(d), 0.0, np.sin(d)])                   # leaves aft and down
        into_jet = np.array([-np.sin(d), 0.0, -np.cos(d)])             # wall normal toward the jet (up/aft side)
        r = rot[name]
        r.pos = np.round(te + JET_HALF_THICKNESS * into_jet, 4).tolist()
        r.axis = np.round(-jet, 6).tolist()
        r.duct_axis = np.round(fan[k][1], 6).tolist()
    # battery placement (the six 5200 mAh packs as one group)
    if battery_dx or battery_dz:
        for b in model.bodies:
            if "battery_5200" in b.name and b.mass > 0:
                b.offset = [float(battery_dx), 0.0, float(battery_dz)]
    af.name = "ATLAS_V3_V34"
    af.notes = ("Built by scripts/atlas_v3_from_fusion.py from the Fusion design ATLAS/V3/SMALL_SCALE_V3 version "
                f"{tree.get('version')} (read through Fusion's MCP, 29 Sep 2026). Masses only from the part-name labels "
                "(__M<g>g); unlabelled parts and unlabelled mirror copies weigh nothing. Foil thrust along the jet exit line "
                "at the trailing edge (exit angles ray-traced on the B-rep). Battery group offset "
                f"dx {battery_dx:+.3f} m, dz {battery_dz:+.3f} m.\n\n" + "PX4 gains, fan model (36 N, km 0.002), legs and "
                "vibration from atlas_v3_small_graded.json.")
    knobs.setup_defaults(af, link_front_parts=False)   # jetfoil / nose / battery knobs on the new bodies
    ms = af.resolve_mass().mass
    log(f"mass {ms.mass:.3f} kg, CG FRD {np.round(ms.cg, 4).tolist()}")
    for k, kg, n, v in report:
        log(f"  {kg * 1000:6.0f} g  {n:3d} STEP bodies  {v:8.1f} cm3  {k}")
    if out:
        af.save(out)
    return af


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--battery-dx", type=float, default=0.0); ap.add_argument("--battery-dz", type=float, default=0.0)
    ap.add_argument("--out", default=str(ROOT / "airframes" / "atlas_v3_v34.json"))
    a = ap.parse_args()
    build(a.battery_dx, a.battery_dz, a.out)
