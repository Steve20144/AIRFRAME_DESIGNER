"""Build airframes/atlas_v3_30_cfd.json: the tuned V3_30 with the foil fans' thrust taken from the OpenFOAM CFD run
(user, 1 Oct 2026) instead of the full-attachment jet model. The CFD was run on foils of 75 / 65 / 50 deg (inner /
middle / outer), not on the V3_30 CAD's own 49.5 / 58.2 / 49.6: the airframe is "V3_30 masses with 75/65/50 foils".
The force points stay on the V3_30 trailing edges; with the v34 edges (foils ~71 / 58 / 50, the nearest CAD) the
hover trim and margins move by under 0.3 deg and 0.1 N m.

CFD result, per foil group (the three foil fans of one side together, fans at full thrust, 36 N each = 108 N in):
    F = (-56.12, 46.48, 0.73) N in the CFD frame  x: along the fan axis (pointing aft, with the jet)
                                                  y: up
                                                  z: spanwise (0.73 N, 1 %, sign convention not given: ignored)
  -> 72.87 N out of 108 N in (67.5 %), leaving 39.63 deg above the duct axis (forward and up).
Applied to each of the six foil fans: push direction = the duct axis turned 39.63 deg up (structural FRD:
(cos, 0, -sin)), and the jetfoil turning loss calibrated so each foil fan delivers 72.87 / 3 = 24.29 N at full
command. The force stays on each fan's jet exit line at its foil's trailing edge (the CFD gives the force, not its
line of action). Nose fans unchanged. Then the hover pitch is re-trimmed (PX4 SENS_BOARD_Y_OFF, CA_ROTOR*),
MPC_THR_HOVER recomputed and the nose-lift target moved to the new hover pitch.

  python scripts/atlas_v3_30_cfd.py [--src airframes/atlas_v3_30_tuned.json] [--out airframes/atlas_v3_30_cfd.json]
"""
from __future__ import annotations
import argparse, math
from pathlib import Path
import numpy as np
from airframe_designer.geometry.airframe import Airframe

ROOT = Path(__file__).resolve().parents[1]
CFD_GROUP_N = np.array([-56.12, 46.48, 0.73])     # per foil group, CFD frame (x aft along the fan axis, y up, z span)
FANS_PER_GROUP = 3
FAN_MAX_N = 36.0                                   # the CFD's input: full thrust on every fan
FOIL_FANS = ("M1", "M2", "M3", "M4", "M5", "M6")


def build(src: Path, out: Path, log=print) -> Airframe:
    af = Airframe.load(src)
    fwd, up = -CFD_GROUP_N[0], CFD_GROUP_N[1]
    elev = math.degrees(math.atan2(up, fwd))                       # thrust angle above the duct axis
    per_fan = math.hypot(fwd, up) / FANS_PER_GROUP                 # spanwise part ignored
    eff = per_fan / FAN_MAX_N
    for r in af.rotors:
        if r.name not in FOIL_FANS:
            continue
        e = math.radians(elev)
        r.axis = [round(math.cos(e), 6), 0.0, round(-math.sin(e), 6)]
        r.duct_axis = [1.0, 0.0, 0.0]
        r.max_thrust = FAN_MAX_N
        # effective_max_thrust = max_thrust * (1 - turn_loss * deflection / 90): calibrate turn_loss to the CFD output
        r.turn_loss = round((1.0 - eff) * 90.0 / r.deflection_deg(), 4)
    af.resolve_mass()
    trim = af.trim_hover_pitch()
    if trim is None:
        raise RuntimeError("no hover trim found with the CFD thrust")
    af.hover_pitch_deg = round(trim, 2)
    af.px4_overrides["MPC_THR_HOVER"] = round(af.hover_thrust_fraction(), 3)
    af.design["nose_lift"]["target_pitch_deg"] = af.hover_pitch_deg
    # parked at landed_pitch the board sees itself (hover - landed) nose-down from its level; PX4's failure detector
    # refuses to arm beyond FD_FAIL_P (default 60 deg): with the CFD hover pitch that is 60.3 deg on the legs
    tilt_parked = af.hover_pitch_deg - af.landed_pitch_deg
    if tilt_parked > 55.0:
        af.px4_overrides["FD_FAIL_P"] = int(math.ceil(tilt_parked / 5.0) * 5 + 5)
    af.name = "ATLAS_V3_30_CFD"
    af.notes += (f"\n\nCFD foil thrust (scripts/atlas_v3_30_cfd.py, OpenFOAM, 1 Oct 2026): per foil group "
                 f"{CFD_GROUP_N.tolist()} N (x fan axis aft, y up, z span), foils 75 / 65 / 50 deg, at 3 x {FAN_MAX_N:g} N in -> each foil fan "
                 f"{per_fan:.2f} N ({100 * eff:.1f} %) at {elev:.2f} deg above the duct axis; hover re-trimmed to "
                 f"{af.hover_pitch_deg} deg.")
    T = sum(r.effective_max_thrust() for r in af.active_rotors())
    log(f"foil fans: {elev:.2f} deg above the duct axis, {per_fan:.2f} N each ({100 * eff:.1f} %), turn_loss "
        f"{af.rotors[0].turn_loss}")
    log(f"total max thrust {T:.1f} N, weight {af.mass.mass * 9.80665:.1f} N, T/W {T / (af.mass.mass * 9.80665):.2f}")
    log(f"hover trim {af.hover_pitch_deg} deg, MPC_THR_HOVER {af.px4_overrides['MPC_THR_HOVER']}, parked {af.landed_pitch_deg} deg, "
        f"FD_FAIL_P {af.px4_overrides.get('FD_FAIL_P', 'default 60')}")
    if out:
        af.save(out)
    return af


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", default=str(ROOT / "airframes" / "atlas_v3_30_tuned.json"))
    ap.add_argument("--out", default=str(ROOT / "airframes" / "atlas_v3_30_cfd.json"))
    a = ap.parse_args()
    build(Path(a.src), Path(a.out))
