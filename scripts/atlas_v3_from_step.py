#!/usr/bin/env python
"""Build airframes/atlas_v3_small.json: the small-scale ATLAS V3 from the STEP export SMALL_SCALE_V3.step
(Fusion, 26 Sep 2026), on branch atlas-v5-design.

From the CAD (measured on the imported solids, see brain/architecture/atlas-v3-small.md):
  * nine fans. Six foil fans, three a side, ducts 106 mm OD with a 96 mm rotor disc, all pointing straight aft; the
    jet runs through a rectangular nozzle and then along the curved lower wall (a Coanda foil) that ends 72.7 /
    74.6 / 75.4 deg below horizontal (inner / middle / outer station). Three XFLY 80 mm EDFs in the nose: the two
    side ones canted 30 deg inward, the centre one vertical.
  * the V (anhedral about 37 deg) of the fan line, the booms, the two rear legs (hub on top of the nose to feet aft
    and outboard), part volumes, centroids and inertia.

Assumed, NOT in the CAD (change the constants below and rerun):
  * masses: ATLAS_09B's structure (user, 26 Sep: "the weights from the previous version, remove the two rear
    motors and add one in the front"): avionics 0.5 kg as a point mass at the flight controller; the foils 2.6 kg
    (09B's four JET_FOIL parts) over V3's nozzles and curved foils, and 2.4 kg of other structure (0.2 of it the nose
    carrier shell) over the tubes, clamps, hubs and ESCs in proportion to volume (tube 1.6, printed 1.0 g/cm3).
    Later the same day (user): motors 0.3 kg each and six batteries of BATTERY_EACH_KG (4.6 kg in total). First two
    triangles at the centre (CG 6.2 cm aft, the draft crashed); then (user) one pair near the nose and two pairs in
    the centre: the nose pair side by side in the slide-in carrier, the centre pairs one above the boom plane and one
    below (point masses with a brick's inertia; the BATTERY_* spacings are assumptions, the CAD has no batteries).
  * jet angle: the jet is taken to leave along the foil's exit (full Coanda attachment), i.e. thrust 14.6-17.3 deg
    forward of vertical. Real jets separate early, so the true turning is smaller; the design knobs change it.
  * fan thrust 36 N quadratic, spool 0.12 s, km 0.002 all the same way (the ATLAS_09B figures).
  * motor order: M1/M2 outer foil fans L/R, M3/M4 middle, M5/M6 inner, M7/M8 nose side fans L/R, M9 nose centre.
  * a nose leg: the CAD has only the two rear legs; without a nose support the aircraft tips onto the nose
    carrier. NOSE_LEG_* sets a provisional one that parks it at PARK_PITCH_DEG.

Frames: the CAD is x right, y up, z aft (metres after import); FRD = (-z, x, -y), origin at the boom level
(CAD y = 0.45) on the centreline.
"""
from __future__ import annotations

import copy
import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from airframe_designer.geometry import cad as cadmod, knobs  # noqa: E402
from airframe_designer.geometry.airframe import Airframe  # noqa: E402
from airframe_designer.geometry.gear import Leg  # noqa: E402
from airframe_designer.geometry.propulsion import Rotor  # noqa: E402

STEP = ROOT / "airframes" / "cad" / "SMALL_SCALE_V3.step"
OUT = ROOT / "airframes" / "atlas_v3_small.json"
BASE = ROOT / "airframes" / "atlas_09b.json"        # PX4 gains, fan model, vibration: starting points only

# ATLAS_09B's masses (airframes/atlas_09b.json CAD bodies): 10 fans 3.4, battery 3.225, avionics 0.5, foils 2.6,
# other structure 2.4 = 12.125 kg; V3 has nine fans
BATTERY_EACH_KG = 4.6 / 6      # user, 26 Sep: 4.6 kg for all six (first read as 4.6 kg each: 35.8 kg, cannot fly)
BATTERY_CENTRE = [0.0, 0.0, 0.0]   # FRD: on the centreline at boom level, between the nose and foil fans
BATTERY_PAIR_HALF_SPAN = 0.045     # m, each battery of a pair this far left / right of the centreline
BATTERY_LAYER = 0.06           # m, the centre pairs above (-z) / below (+z) the centre
BATTERY_NOSE_HALF_SPAN = 0.065     # m, the nose pair side by side in the 0.26 m wide carrier
BATTERY_BRICK = (0.16, 0.08, 0.06)   # m, a pack's size for its own inertia
FAN_KG = 0.30                  # user, 26 Sep: "each motor weighs 300grams"
AVIONICS_KG = 0.5
FOIL_KG = 2.6
OTHER_STRUCTURE_KG = 2.4
TUBE_DENSITY = 1600.0          # kg/m3, relative weights inside OTHER_STRUCTURE_KG
PRINT_DENSITY = 1000.0         # kg/m3 of solid volume, likewise
FAN_THRUST_N = 36.0
PARK_PITCH_DEG = 4.0
HOVER_PITCH_DEG = 10.0         # replaced by the trim scan below when it finds a better one

R_CAD = np.array([[0, 0, -1], [1, 0, 0], [0, -1, 0]], float)     # rows: FRD x, y, z in CAD axes
ORIGIN = -R_CAD @ np.array([0.0, 0.45, 0.0])

# body indices in the STEP file order (ids are "<index>:<name>")
FOIL_DUCTS = {"M1": 55, "M2": 15, "M3": 53, "M4": 13, "M5": 54, "M6": 14}   # outer, middle, inner; L then R
FOIL_JET_BELOW_HORIZONTAL = {55: 75.4, 15: 75.4, 53: 74.6, 13: 74.6, 54: 72.7, 14: 72.7}
NOSE_FANS = {"M7": 3, "M8": 43, "M9": 62}
ESC_BIG, ESC_SMALL = (36, 56, 60), (37, 57, 61)
FEET = {"L": 32, "R": 35}
LEG_HUB = 17
CARRIER = 31
FOIL_BODIES = (5, 6, 7, 8, 45, 46, 47, 48)          # nozzles (5-7, 45-47) and curved Coanda foils (8, 48)
PIXHAWK_CAD = [0.0, 0.47, -0.2]


def frd(p) -> np.ndarray:
    return R_CAD @ np.asarray(p, float) + ORIGIN


def body_mass(k: int, b: dict) -> float:
    name, vol = b["name"], b["volume"]
    if k in FOIL_DUCTS.values() or k in NOSE_FANS.values():
        return FAN_KG
    if k in ESC_BIG:
        return 0.05
    if k in ESC_SMALL:
        return 0.02
    if k == CARRIER:
        return 0.2                          # the carrier shell; the batteries are point masses at the centre
    if "carbon_tube" in name:
        return vol * TUBE_DENSITY
    if vol < 5e-6 and "XFLY" in name:
        return 0.0                          # the XFLY's small detail solid
    if k in (10, 11, 12, 50, 51, 52):
        return 0.0                          # rotor discs, counted in FAN_KG
    return vol * PRINT_DENSITY


def main() -> None:
    imported = cadmod.import_step(STEP)
    by_k = {int(b["id"].split(":")[0]): b for b in imported["bodies"]}
    base = Airframe.load(BASE)

    af = copy.deepcopy(base)
    af.name = "ATLAS_V3_SMALL"
    af.cad = cadmod.model_from_import(imported, str(STEP.relative_to(ROOT)).replace("\\", "/"))
    af.cad.rotation_deg = cadmod.euler_deg(R_CAD)
    af.cad.origin = [round(float(v), 6) for v in ORIGIN]
    raw = {b.id: body_mass(int(b.id.split(":")[0]), by_k[int(b.id.split(":")[0])]) for b in af.cad.bodies}
    kidx = lambda bid: int(bid.split(":")[0])
    fixed = set(FOIL_DUCTS.values()) | set(NOSE_FANS.values()) | {CARRIER}
    foil = [i for i in raw if kidx(i) in FOIL_BODIES]
    other = [i for i in raw if kidx(i) not in FOIL_BODIES and kidx(i) not in fixed]
    for group, total in ((foil, FOIL_KG), (other, OTHER_STRUCTURE_KG - 0.2)):
        s = sum(raw[i] for i in group)
        for i in group:
            raw[i] = raw[i] * total / s
    for b in af.cad.bodies:
        b.mass = round(raw[b.id], 5)
    from airframe_designer.geometry.mass import MassItem
    af.mass.items = [MassItem(name="avionics (ATLAS_09B)", mass=AVIONICS_KG, pos=[round(float(v), 4) for v in frd(PIXHAWK_CAD)])]
    a, b, c = BATTERY_BRICK
    own = [BATTERY_EACH_KG * (b * b + c * c) / 12, BATTERY_EACH_KG * (a * a + c * c) / 12, BATTERY_EACH_KG * (a * a + b * b) / 12]
    nose = frd(by_k[CARRIER]["centroid"])
    layout = [("nose", nose, BATTERY_NOSE_HALF_SPAN),
              ("centre upper", np.asarray(BATTERY_CENTRE) + [0.0, 0.0, -BATTERY_LAYER], BATTERY_PAIR_HALF_SPAN),
              ("centre lower", np.asarray(BATTERY_CENTRE) + [0.0, 0.0, BATTERY_LAYER], BATTERY_PAIR_HALF_SPAN)]
    for where, c, half in layout:
        for side, sy in (("L", -1.0), ("R", 1.0)):
            pos = [float(c[0]), float(c[1]) + sy * half, float(c[2])]
            af.mass.items.append(MassItem(name=f"battery {where} {side}", mass=BATTERY_EACH_KG,
                                          pos=[round(v, 4) + 0.0 for v in pos], inertia=[round(v, 6) for v in own]))
    af.mass.from_items = True
    af.mass.manual = None

    proto = base.rotors[0]
    rotors = []
    for name, k in FOIL_DUCTS.items():
        below = math.radians(FOIL_JET_BELOW_HORIZONTAL[k])
        r = copy.deepcopy(proto)
        r.name, r.pos = name, [round(float(v), 4) for v in frd(by_k[k]["centroid"])]
        r.axis = [round(math.cos(below), 6), 0.0, round(-math.sin(below), 6)]    # jet aft-down -> thrust fwd-up
        r.duct_axis, r.diameter, r.max_thrust = [1.0, 0.0, 0.0], 0.096, FAN_THRUST_N
        rotors.append(r)
    for name, k in NOSE_FANS.items():
        b = by_k[k]
        ixx, iyy, izz, ixy, ixz, iyz = b["inertia_unit"]
        w, U = np.linalg.eigh(np.array([[ixx, -ixy, -ixz], [-ixy, iyy, -iyz], [-ixz, -iyz, izz]]))
        i = int(np.argmax([abs(w[j] - np.mean(np.delete(w, j))) for j in range(3)]))
        a = R_CAD @ U[:, i]
        a = a if a[2] < 0 else -a                                               # thrust up (FRD z negative)
        r = copy.deepcopy(proto)
        r.name, r.pos = name, [round(float(v), 4) for v in frd(b["centroid"])]
        r.axis, r.duct_axis, r.diameter, r.max_thrust = [round(float(v), 6) for v in a], None, 0.08, FAN_THRUST_N
        rotors.append(r)
    af.rotors = rotors

    hub = frd(by_k[LEG_HUB]["centroid"])
    legs = []
    for side, k in FEET.items():
        foot = frd(by_k[k]["centroid"])
        legs.append(Leg.from_points(hub + np.array([0.0, 0.03 * (1 if side == "R" else -1), 0.0]), foot, name=side,
                                    foot_radius=0.012, stiffness=1500, damping=80, friction=0.8))
    af.legs = legs
    af.resolve_mass()
    # provisional nose leg: from under the nose carrier to the ground plane that parks the aircraft at PARK_PITCH_DEG
    feet = np.array([l.foot() for l in legs])
    nose_attach = frd([0.0, 0.33, -0.36])
    th = math.radians(PARK_PITCH_DEG)
    # ground plane through the rear feet, tilted so the nose is up by PARK_PITCH_DEG: nose up puts the ground further
    # below the nose, z_ground(x) = z_feet + (x - x_feet) tan th
    xf, zf = feet[:, 0].mean(), feet[:, 2].mean() + 0.012
    z_ground = zf + (nose_attach[0] - xf) * math.tan(th)
    af.legs.append(Leg.from_points(nose_attach, [nose_attach[0], 0.0, z_ground], name="N (not in CAD)",
                                   foot_radius=0.0, stiffness=1500, damping=80, friction=0.8))

    af.landed_pitch_deg = PARK_PITCH_DEG
    af.px4_overrides = {k: v for k, v in base.px4_overrides.items() if not k.startswith("CA_ROTOR")}
    af.design = copy.deepcopy(base.design)
    af.design.pop("groups", None)
    af.design.pop("knobs", None)
    af.design.pop("cad_links", None)
    for key in ("nose_lift", "nose_lower"):
        if key in af.design:
            af.design[key]["motors"] = [6, 7, 8]
    af.design["pixhawk_position"] = [round(float(v), 4) for v in frd(PIXHAWK_CAD)]
    if "vibration" in af.design:
        af.design["vibration"]["imu_pos"] = af.design["pixhawk_position"]

    # hover pitch: the nose-up attitude where the static mix is feasible with the lowest worst-motor share
    scan = []
    for p in np.arange(0.0, 30.01, 0.5):
        af.hover_pitch_deg = float(p)
        hc = af.hover_check()
        u = [x for x in hc.get("hover_utilisation") or [] if x is not None]
        scan.append((float(p), hc["ok"], max(u) if u else float("nan")))
    feasible = [s for s in scan if s[1]]
    af.hover_pitch_deg = af.trim_hover_pitch() or HOVER_PITCH_DEG       # most headroom on every fan
    for nl in ("nose_lift",):
        if nl in af.design:
            af.design[nl]["target_pitch_deg"] = af.hover_pitch_deg
    if "nose_lower" in af.design:
        af.design["nose_lower"]["target_pitch_deg"] = PARK_PITCH_DEG

    af.px4_overrides["MPC_THR_HOVER"] = af.hover_thrust_fraction()   # mid stick hovers (0.26 measured on the 7.8 kg draft)
    knobs.setup_defaults(af)
    knobs.set_ballast(af, [], [i.name for i in af.mass.items if i.name.startswith("battery")])
    af.resolve_mass()
    af.notes = (__doc__ or "").strip() + (f"\n\nBuilt by scripts/atlas_v3_from_step.py. Feasible hover pitch window "
                                          f"{feasible[0][0]:g}-{feasible[-1][0]:g} deg" if feasible else
                                          "\n\nBuilt by scripts/atlas_v3_from_step.py. NO feasible hover pitch in 0-30 deg")
    af.save(OUT)

    print(f"mass {af.mass.mass:.3f} kg, CG {np.round(af.mass.cg, 4).tolist()}, thrust/weight "
          f"{sum(r.effective_max_thrust() for r in af.rotors) / (af.mass.mass * 9.81):.2f}")
    for r in af.rotors:
        print(f"  {r.name}: pos {r.pos} tilt {r.tilt_deg:.1f} cant {r.cant_deg:.1f}")
    for l in af.legs:
        print(f"  leg {l.name}: foot {np.round(l.foot(), 3).tolist()}")
    print("hover scan (pitch, ok, worst share):", [(p, round(u, 2)) for p, ok, u in scan if ok][:60] or "none feasible")
    print(f"hover pitch {af.hover_pitch_deg}; problems:", af.hover_check()["problems"])
    print(f"wrote {OUT.relative_to(ROOT)}")


if __name__ == "__main__":
    main()
