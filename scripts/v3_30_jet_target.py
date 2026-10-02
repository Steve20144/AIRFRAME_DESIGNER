"""V3_30 airframes for a CFD/CAD target: the foil jets leave steeper than today's CFD (39.6 deg above the fan axis) so
the hover pitch is at most 20 deg, and the legs park the aircraft so the rotation from the ground to hover is at
most 20 deg too (user, 1 Oct 2026). The jet is given directly (what the CFD measures), not as foil angles: per
station the mean jet angle plus the graded split of the 35 / 75 / 70 pick (inner -14.6, middle +8.8, outer +5.8 deg),
thrust kept as in the CFD (67.5 %) unless --eff. Legs re-solved for the parked pitch (same hard points).

  python scripts/v3_30_jet_target.py --jet 60 --park 0 [--eff 0.675] --out airframes/x.json
"""
from __future__ import annotations
import argparse, math, sys
from datetime import datetime
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
import v3_30_foil_headroom as F
from airframe_designer.geometry.airframe import Airframe

SPLIT = {"inner": -14.6, "middle": 8.8, "outer": 5.8}


def build(jet: float, park: float, eff: float, out: Path, log=print) -> Airframe:
    af = Airframe.load(F.SRC)
    for st, off in SPLIT.items():
        for r in af.rotors:
            if r.name in F.STATION[st]:
                e = math.radians(jet + off)
                r.axis = [math.cos(e), 0.0, -math.sin(e)]; r.duct_axis = [1.0, 0.0, 0.0]; r.max_thrust = F.FAN_N
                r.turn_loss = (1 - eff) * 90.0 / r.deflection_deg()
    af.resolve_mass()
    hover = round(F.trim(af), 2)
    af = af.with_attitude(park_pitch_deg=park, hover_pitch_deg=hover)
    af.resolve_mass()
    af.px4_overrides["MPC_THR_HOVER"] = round(float(af.hover_thrust_fraction()), 3)
    af.design["nose_lift"]["target_pitch_deg"] = af.hover_pitch_deg
    af.px4_overrides.pop("FD_FAIL_P", None)          # parked within 20 deg of the hover frame: default 60 is fine
    af.name = f"ATLAS_V3_30_JET{jet:g}_PARK{park:g}"
    ts = datetime.now().astimezone().isoformat(timespec="seconds")
    af.notes += (f"\n\nJet target ({ts}, scripts/v3_30_jet_target.py): foil jets {jet:g} deg above the fan axis "
                 f"(inner {jet + SPLIT['inner']:.1f} / middle {jet + SPLIT['middle']:.1f} / outer {jet + SPLIT['outer']:.1f}), "
                 f"{100 * eff:.1f} % kept; hover {af.hover_pitch_deg} deg, parked {af.landed_pitch_deg} deg.")
    af.save(out)
    h = F.hover(af.copy())
    log(ts, out.name, f"hover {af.hover_pitch_deg} park {af.landed_pitch_deg} rotation {af.hover_pitch_deg - af.landed_pitch_deg:.1f}",
        f"MPC_THR_HOVER {af.px4_overrides['MPC_THR_HOVER']}", h)
    return af


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--jet", type=float, required=True); ap.add_argument("--park", type=float, required=True)
    ap.add_argument("--eff", type=float, default=0.675); ap.add_argument("--out", required=True)
    a = ap.parse_args()
    build(a.jet, a.park, a.eff, Path(a.out))
