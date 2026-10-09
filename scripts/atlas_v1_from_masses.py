"""Build airframes/atlas_v1_m001.json (ATLAS v1 full scale, candidate v002 skin) from the S4 mass points.

Inputs: the user's clicked masses (`from_A/v002/masses.json`, S4 of brain/HANDOFF.md) and the v002 meshes for the
visual STL and the leg hard points. Everything is a declared assumption except the masses and their positions:

  * FRD = R @ (CAD_m - Q) with R = [[0,1,0],[1,0,0],[0,0,-1]] (physics handoff) and Q = (0, 0, 5.0) m CAD: the FRD
    origin is on the centreline, at the nose station, at the belly level of the centre body.
  * Nine rotors AT THE FAN MASS POINTS (physical centres, not jet exits: no foil geometry exists for v1 yet).
    Wing fans M1..M6: duct axis +x (fan blows aft), jet turned to JET_DEG above the fan axis (assumption, the V3_30
    foils were 50 to 58 deg as drawn, 57.5 to 85 as targets). Tail fans M7..M9: thrust straight up (body -z).
    Schübeler DS-215 class: 215 N max (catalogue 215 to 250 N static), tau 0.3 s, 195 mm. km +-0.01 alternating.
  * The wing group is SYMMETRISED: the right side's three fans and ESCs are mirrored to the left (the clicked left
    side had one fan 0.6 m further forward than its mirror; the user placed "roughly").
  * Lifting body as six strip-theory panels carrying the section polars E908 (body), GOE 5K (transition) and
    SC(2)-0402 (outer wing), planform and anhedral measured on the v002 skin, plus a body drag box for the nacelles,
    canopy and fans. Hidden in the 3D view (visual false). No 60 km/h validation.
  * Legs: the two wing tips (the lowest points of the skin, 1 m below the belly) with 0.15 m pads, and a tail skid.
    The parked pitch is set to the hover trim pitch (the scenario parks at "hover"): with every fan aft of the tip
    feet, no fan can raise the nose from a nose-down park, so the rotate-up concept is not modelled here.
  * PX4 roll/pitch gains from the 2026-10-05 sweep (set A), the rest from atlas_v3_30.json, MPC_THR_HOVER from the trim.

  .venv/bin/python scripts/atlas_v1_from_masses.py [--masses PATH] [--out airframes/atlas_v1_m001.json]
"""
from __future__ import annotations
import argparse, json, math, struct
from datetime import datetime, timezone
from pathlib import Path
import numpy as np
from airframe_designer.geometry.airframe import Airframe
from airframe_designer.geometry.gear import Leg

ROOT = Path(__file__).resolve().parents[1]
CAND = ROOT / "results/handoff/T20261005-atlas-v1-foil-references/from_A/v002"
TEMPLATE = ROOT / "airframes/atlas_v3_30.json"
R = np.array([[0, -1, 0], [-1, 0, 0], [0, 0, -1]], float)   # nose = CAD -Y (user, 2026-10-05): x_frd = -y_cad, y_frd = -x_cad
Q = np.array([0.0, 0.0, 5.0])           # CAD m, the FRD origin
JET_DEG = 75.0                          # wing-fan jet angle above the fan axis (assumption)
MAX_THRUST = 215.0                      # N per fan (catalogue low end)
KM = 0.01                               # m, reaction torque per N (assumption)
TAU = 0.3                               # s
DIAM = 0.195                            # m
PAD = 0.15                              # m, wing-tip foot pad below the skin
NOSE_SKID_CAD = np.array([0.0, -3.05, 5.15])    # bottom of the nose tip
PARK_PITCH_DEG = -10.0                         # nose-down on the wing tips and the nose skid (design intent, brain)


def frd(p_cad_m):
    return (R @ (np.asarray(p_cad_m, float) - Q))


def write_stl(path: Path, meshes: list[tuple[np.ndarray, np.ndarray]]):
    tris = []
    for v, f in meshes:
        tris.append(v[f])                                   # (n, 3, 3)
    T = np.concatenate(tris)
    n = np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0])
    n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)
    with open(path, "wb") as fh:
        fh.write(b"ATLAS v1 candidate v002, FRD metres".ljust(80, b"\0"))
        fh.write(struct.pack("<I", len(T)))
        rec = np.zeros(len(T), dtype=[("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
        rec["n"] = n; rec["v"] = T
        fh.write(rec.tobytes())


def build(masses_path: Path, out: Path, log=print, name: str | None = None) -> Airframe:
    doc = json.loads(masses_path.read_text())
    items = doc["items"]
    fans = [it for it in items if "ducted fan" in it["name"]]
    escs = [it for it in items if "ESC" in it["name"]]
    other = [it for it in items if it not in fans and it not in escs]
    tail = [it for it in fans if it["pos_cad_m"][1] < -2.0]          # "tail" in the code = the trio at CAD y < -2: the NOSE fans
    wing_r = sorted([it for it in fans if it["pos_cad_m"][1] >= -2.0 and it["pos_cad_m"][0] > 0], key=lambda i: i["pos_cad_m"][0])
    esc_tail = [e for e in escs if e["pos_cad_m"][1] < -2.0]
    esc_r = sorted([e for e in escs if e["pos_cad_m"][1] >= -2.0 and e["pos_cad_m"][0] > 0], key=lambda e: e["pos_cad_m"][0])
    assert len(tail) == 3 and len(wing_r) == 3, (len(tail), len(wing_r))

    def mirror(it, name):
        p = list(it["pos_cad_m"]); p[0] = -p[0]
        return {**it, "name": name, "pos_cad_m": p}

    # rotors: M1/M2 inner L/R, M3/M4 middle, M5/M6 outer (wing fans, behind the CG), M7..M9 nose trio (centre first)
    rotors, mass_items = [], []
    jet = math.radians(JET_DEG)
    axis_wing = [round(math.cos(jet), 6), 0.0, round(-math.sin(jet), 6)]
    for k, it in enumerate(wing_r):
        for side, (sgn, src) in enumerate(((-1, mirror(it, f"fan L{k + 1}")), (+1, it))):
            n = 2 * k + 1 + side
            km = KM * (1 if (k + side) % 2 == 0 else -1)
            rotors.append({"name": f"M{n}", "enabled": True, "pos": np.round(frd(src["pos_cad_m"]), 4).tolist(), "axis": axis_wing,
                           "km": km, "max_thrust": MAX_THRUST, "tau": TAU, "tau_down": 0, "diameter": DIAM, "thrust_exponent": 2,
                           "kind": "ducted", "ram_drag": True, "duct_axis": [1.0, 0.0, 0.0], "turn_loss": 0.1})
            mass_items.append({"name": f"fan {'LR'[side]}{k + 1} ({src['bom_name'][:30]})", "mass": src["grams"] / 1000,
                               "pos": np.round(frd(src["pos_cad_m"]), 4).tolist()})
    tail = sorted(tail, key=lambda i: abs(i["pos_cad_m"][0]))
    for k, it in enumerate(tail):
        rotors.append({"name": f"M{7 + k}", "enabled": True, "pos": np.round(frd(it["pos_cad_m"]), 4).tolist(), "axis": [0.0, 0.0, -1.0],
                       "km": KM * (1 if k % 2 == 0 else -1), "max_thrust": MAX_THRUST, "tau": TAU, "tau_down": 0, "diameter": DIAM,
                       "thrust_exponent": 2, "kind": "ducted", "ram_drag": True, "duct_axis": None, "turn_loss": 0.1})
        mass_items.append({"name": f"fan N{k + 1} (nose)", "mass": it["grams"] / 1000, "pos": np.round(frd(it["pos_cad_m"]), 4).tolist()})
    for k, e in enumerate(esc_r):
        for side, src in ((0, mirror(e, "")), (1, e)):
            mass_items.append({"name": f"ESC {'LR'[side]}{k + 1}", "mass": src["grams"] / 1000, "pos": np.round(frd(src["pos_cad_m"]), 4).tolist()})
    for k, e in enumerate(esc_tail):
        mass_items.append({"name": f"ESC N{k + 1} (nose)", "mass": e["grams"] / 1000, "pos": np.round(frd(e["pos_cad_m"]), 4).tolist()})
    for it in other:
        mass_items.append({"name": it["name"], "mass": it["grams"] / 1000, "pos": np.round(frd(it["pos_cad_m"]), 4).tolist()})

    d = Airframe.load(TEMPLATE).to_dict()
    d["name"] = name or out.stem.upper()
    d["cad"] = None
    d["mass"] = {"mass": 0, "cg": [0, 0, 0], "inertia": [1, 1, 1], "inertia_products": [0, 0, 0], "items": mass_items, "from_items": True}
    d["rotors"] = rotors
    # lifting body as six strip-theory panels with the user's section references (polars built by the app from the
    # DAT files in airfoils/: NeuralFoil unless xfoil is installed). Planform measured on the v002 skin (CAD mm
    # slices), the NOSE at CAD y = -3.24: LE y -3.24 at the centre, -1.55 at |x| 0.5, -1.2 at 1.1, -1.0 at 1.9;
    # TE +0.59 / +0.17 / 0.0 / -0.3. The mid-surface drops from z 5.1 (centre) to 5.0 (0.5), 4.6 (1.1), 4.0 (1.9).
    # Incidence 0 (CAD chords horizontal). NOTE for Agent B: the v002 skin was fitted with LE = max Y, i.e. reversed.
    def panel(name, side, pos_cad, span, c_root, c_tip, sweep, dih, foil, n):
        return {"name": name, "enabled": True, "visual": False, "symmetric": False, "side": side,
                "pos": np.round(frd(pos_cad), 3).tolist(), "span": span, "root_chord": c_root, "tip_chord": c_tip,
                "sweep_deg": sweep, "dihedral_deg": dih, "incidence_deg": 0, "pitch_deg": 0, "twist_deg": 0, "panels": n,
                "aspect_ratio": 2.13,    # the whole planform's (3.8^2 / 6.77 m2): a panel is not an isolated wing
                "aero": {"model": "polar", "airfoil_root": foil, "airfoil_tip": foil, "polar_source": "auto", "ncrit": 9.0,
                         "cl_alpha": 6.2832, "cl0": 0.0, "cd0": 0.01, "oswald": 0.8, "stall_deg": 12, "stall_blend_deg": 6,
                         "cd_flat": 1.2, "cm0": 0.0, "vortex_lift": False}}
    wings = []
    for side in (1, -1):
        sg = "R" if side > 0 else "L"
        xc = -side            # FRD +y (right) is CAD -x
        wings.append(panel(f"body {sg} (E908)", side, [0.0, -3.24, 5.1], 0.51, 3.83, 1.72, 73.5, -11.0, "e908", 4))
        ev = {"chord_fraction": 0.25, "max_deg": 25, "span_from": 0.0, "span_to": 1.0, "pitch_gain": 1.0, "roll_gain": 1.0, "deflection_deg": 0.0}
        wings.append({**panel(f"transition {sg} (GOE 5K)", side, [0.5 * xc, -1.55, 5.0], 0.70, 1.72, 1.20, 30.0, -31.0, "goe05k", 4), "elevon": dict(ev)})
        wings.append({**panel(f"outer {sg} (SC(2)-0402)", side, [1.1 * xc, -1.2, 4.6], 1.00, 1.20, 0.70, 14.0, -37.0, "sc20402", 5), "elevon": dict(ev)})
    d["wings"] = wings
    # body drag box: nacelle inlets, canopy, fans at rest (CD A ~ 0.15 m2 head-on): 0.5 rho CD A per axis, acting near the CG
    d["body"] = {"size": [3.8, 1.0, 0.4], "drag_quadratic": [0.1, 0.6, 0.8], "drag_angular": [2.0, 4.0, 4.0], "drag_center": [-1.4, 0, -0.1]}
    # legs: wing tips + tail skid (foot heights set below from the trim pitch)
    z = np.load(CAND / "meshes_mm.npz")
    lower = z["source_lower_skin__v"] / 1000.0
    tips = []
    for lo, hi in ((0.6, 1.95), (-1.95, -0.6)):
        m = (lower[:, 0] > lo) & (lower[:, 0] < hi)
        tips.append(lower[m][np.argmin(lower[m, 2])])
    tip_r, tip_l = tips
    legs = []
    for nm, t in (("L", tip_l), ("R", tip_r)):
        a = frd(t); f = a + [0, 0, PAD]
        legs.append(Leg.from_points(a.tolist(), f.tolist(), name=nm, foot_radius=0.0, stiffness=20000, damping=1500, friction=0.8, enabled=True))
    d["legs"] = [l.to_dict() if hasattr(l, "to_dict") else l.__dict__ for l in legs]
    d["px4_overrides"] = {k: v for k, v in d["px4_overrides"].items()}
    d["px4_overrides"]["MC_AIRMODE"] = 0
    # outdoor-style estimator: GPS fused, baro height (a PX4 instance may carry saved indoor settings, GPS off / range height)
    d["px4_overrides"].update({"EKF2_GPS_CTRL": 7, "EKF2_HGT_REF": 0, "EKF2_RNG_CTRL": 0, "EKF2_OF_CTRL": 0})
    # gain set A of the 2026-10-05 sweep (results/atlas_v1/gains): V3_30's gains rolled the 46 kg vehicle over in 4 cycles
    d["px4_overrides"].update({"MC_ROLLRATE_P": 0.3, "MC_ROLLRATE_I": 0.1, "MC_ROLLRATE_D": 0.02, "MC_ROLL_P": 3.0,
                               "MC_PITCHRATE_P": 0.3, "MC_PITCHRATE_I": 0.1, "MC_PITCHRATE_D": 0.02, "MC_PITCH_P": 3.0})
    des = d["design"]
    des["cruise_speed_kmh"] = 60
    des["battery_wh"] = 14 * 15 * 3.7 * 2 * 9   # 18 x 14S 15 Ah packs, informational
    des["pixhawk_position"] = np.round(frd([0.0, -0.5, 5.45]), 3).tolist()
    des["nose_lift"] = {**des.get("nose_lift", {}), "enabled": True, "motors": [6, 7, 8], "executor": "sim"}   # the nose trio lifts the nose
    des["nose_lower"] = {**des.get("nose_lower", {}), "enabled": True, "motors": [6, 7, 8]}
    des["vibration"] = {**des.get("vibration", {}), "enabled": False, "imu_pos": des["pixhawk_position"]}
    des["flow_sensor"] = {**des.get("flow_sensor", {}), "enabled": False}
    des.pop("knobs", None); des.pop("cad_links", None)
    d["mesh"] = {"file": "atlas_v1_cand_v002.stl", "frame": "frd", "scale": 1.0, "opacity": 0.6}
    af = Airframe.from_dict(d)
    af.resolve_mass()
    trim = af.trim_hover_pitch()
    af.hover_pitch_deg = round(trim, 2) if trim is not None else 0.0
    # nose skid: the CG is ahead of the wing-tip feet, the aircraft rests nose-down on the tips and the nose skid at
    # PARK_PITCH_DEG; body point (x, z) at nose-up theta has world depth d = -x sin(theta) + z cos(theta)
    th = math.radians(PARK_PITCH_DEG)
    tip_feet = np.array([l.foot() for l in af.legs[:2]])
    x_tip, z_tip = tip_feet[:, 0].mean(), tip_feet[:, 2].mean()
    na = frd(NOSE_SKID_CAD)
    d_tip = -x_tip * math.sin(th) + z_tip * math.cos(th)
    z_nose = (d_tip + na[0] * math.sin(th)) / math.cos(th)
    af.legs.append(Leg.from_points(na.tolist(), [na[0], 0.0, z_nose], name="N (nose skid)", foot_radius=0.0, stiffness=20000, damping=1500, friction=0.8, enabled=True))
    af.landed_pitch_deg = PARK_PITCH_DEG
    af.auto_leg_constants()
    af.px4_overrides["MPC_THR_HOVER"] = round(af.hover_thrust_fraction(), 3)
    af.design["nose_lift"]["target_pitch_deg"] = af.hover_pitch_deg
    af.design["nose_lower"]["target_pitch_deg"] = af.landed_pitch_deg
    ms = af.resolve_mass().mass
    af.notes = (f"Built {datetime.now(timezone.utc).astimezone().isoformat(timespec='minutes')} by scripts/atlas_v1_from_masses.py from "
                f"{masses_path.relative_to(ROOT)} (saved {doc.get('updated')}, {len(items)} clicked points, {ms.mass:.3f} kg: the user's "
                "'crucial' parts only, the BOM's airborne total is 149.6 kg). FRD = R (CAD - (0,0,5.0 m)), nose = CAD -Y. Rotors at the fan mass points; "
                f"wing fans jet {JET_DEG:g} deg above a +x fan axis (assumed), tail fans vertical; {MAX_THRUST:g} N, km +-{KM}, tau {TAU} s. "
                "Wing group symmetrised (right side mirrored). Legs: wing tips with 0.15 m pads and a nose skid, parked nose-down at -10 deg; the nose trio M7..M9 lifts the nose. PX4 gains from atlas_v3_30.json.")
    write_stl(ROOT / "airframes/meshes/atlas_v1_cand_v002.stl",
              [((R @ (z[k + "__v"] / 1000.0 - Q).T).T, z[k + "__f"]) for k in ("candidate_upper_skin", "source_lower_skin", "source_canopy")])
    problems = af.validate()
    log(f"mass {ms.mass:.3f} kg, CG FRD {np.round(ms.cg, 4).tolist()}, inertia {ms.inertia}, products {ms.inertia_products}")
    log(f"hover trim {af.hover_pitch_deg} deg, parked {af.landed_pitch_deg} deg, MPC_THR_HOVER {af.px4_overrides['MPC_THR_HOVER']}, "
        f"T/W {af.total_max_thrust() / (ms.mass * 9.81):.2f}")
    for r in af.rotors:
        log(f"  {r.name}: pos {r.pos} axis {r.axis} km {r.km}")
    for l in af.legs:
        log(f"  leg {l.name}: attach {np.round(l.attach, 3).tolist()} foot {np.round(l.foot(), 3).tolist()} len {l.length:.3f}")
    log(f"validate: {problems or 'ok'}")
    log(json.dumps(af.hover_check(), default=float)[:600])
    af.save(out)
    log(f"saved {out}")
    return af


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--masses", default=str(CAND / "masses.json"))
    ap.add_argument("--out", default=str(ROOT / "airframes/atlas_v1_m001.json"))
    ap.add_argument("--name", default=None, help="airframe name (default: the output file stem, upper case)")
    a = ap.parse_args()
    build(Path(a.masses), Path(a.out), name=a.name)
