"""Sweep the foil jets' exit angle per station (inner / middle / outer, deg above the fan axis: what the CFD measures)
on the V3_30 for a parked pitch and the user's limits (1 Oct 2026): hover pitch <= 20 deg and rotation from the
ground (hover - park) <= 20 deg. Each triple is trimmed (PX4 pseudo-inverse mix) and scored on the busiest fan at
hover and the spare roll / pitch / yaw moment (linear programme, the other axes held), under two thrust-kept
assumptions: the CFD's 67.5 % and a pessimistic 60 % (steeper jets may cost thrust; one CFD point cannot say).

  python scripts/v3_30_jet_sweep.py --park -10 [--lo 40 --hi 90 --step 2.5]
  python scripts/v3_30_jet_sweep.py --park -10 --build 60,80,75 ... --out DIR     (flyable airframes)
Writes <out>/<YYYYmmdd_HHMMSS>/jet_static.json, every row timestamped.
"""
from __future__ import annotations
import argparse, copy, itertools, json, math, sys, time
from datetime import datetime
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
import v3_30_foil_headroom as F
from airframe_designer.geometry.airframe import Airframe

ROOT = Path(__file__).resolve().parents[1]
EFFS = (0.675, 0.60)


def apply_jets(af: Airframe, jets: dict, eff: float) -> Airframe:
    for st, e_deg in jets.items():
        for r in af.rotors:
            if r.name in F.STATION[st]:
                e = math.radians(e_deg)
                r.axis = [math.cos(e), 0.0, -math.sin(e)]; r.duct_axis = [1.0, 0.0, 0.0]; r.max_thrust = F.FAN_N
                r.turn_loss = (1 - eff) * 90.0 / r.deflection_deg()
    af.resolve_mass()
    return af


def build(jets: dict, park: float, eff: float, out: Path, log=print) -> Airframe:
    af = apply_jets(Airframe.load(F.SRC), jets, eff)
    hover = round(F.trim(af), 2)
    af = af.with_attitude(park_pitch_deg=park, hover_pitch_deg=hover)
    af.resolve_mass()
    af.px4_overrides["MPC_THR_HOVER"] = round(float(af.hover_thrust_fraction()), 3)
    af.design["nose_lift"]["target_pitch_deg"] = af.hover_pitch_deg
    af.px4_overrides.pop("FD_FAIL_P", None)
    tag = "_".join(f"{jets[s]:g}" for s in ("inner", "middle", "outer"))
    af.name = f"ATLAS_V3_30_JETS_{tag}_PARK{park:g}"
    ts = F.now()
    af.notes += (f"\n\nJet sweep ({ts}, scripts/v3_30_jet_sweep.py): foil jets inner / middle / outer {tag.replace('_', ' / ')} "
                 f"deg above the fan axis, {100 * eff:.1f} % kept; hover {af.hover_pitch_deg}, parked {af.landed_pitch_deg} deg.")
    af.save(out)
    legs = {l.name: round(l.length, 3) for l in af.active_legs()}
    log(ts, out.name, f"hover {af.hover_pitch_deg} park {af.landed_pitch_deg} rotation {af.hover_pitch_deg - af.landed_pitch_deg:.2f}",
        "legs", legs, "MPC_THR_HOVER", af.px4_overrides["MPC_THR_HOVER"])
    return af


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--park", type=float, required=True)
    ap.add_argument("--max-hover", type=float, default=20.0); ap.add_argument("--max-rotation", type=float, default=20.0)
    ap.add_argument("--lo", type=float, default=40.0); ap.add_argument("--hi", type=float, default=90.0)
    ap.add_argument("--step", type=float, default=2.5)
    ap.add_argument("--out", default=str(ROOT / "results" / "v3_30" / "jet_sweep"))
    ap.add_argument("--build", nargs="*"); ap.add_argument("--eff", type=float, default=0.675)
    a = ap.parse_args()
    if a.build:
        for t in a.build:
            i, m, o = map(float, t.split(","))
            build({"inner": i, "middle": m, "outer": o}, a.park, a.eff, Path(a.out) / f"atlas_v3_30_jets_{t.replace(',', '_')}_park{a.park:g}.json")
        return
    run = Path(a.out) / datetime.now().strftime("%Y%m%d_%H%M%S"); run.mkdir(parents=True, exist_ok=True)
    cap = min(a.max_hover, a.park + a.max_rotation)
    base = Airframe.load(F.SRC)
    grid = [float(x) for x in __import__("numpy").arange(a.lo, a.hi + 1e-9, a.step)]
    rows, t0 = [], time.time()
    print(F.now(), f"park {a.park}, hover cap {cap} deg, {len(grid) ** 3} triples x {len(EFFS)} thrust-kept cases")
    for eff in EFFS:
        for i, m, o in itertools.product(grid, grid, grid):
            jets = {"inner": i, "middle": m, "outer": o}
            af = apply_jets(copy.deepcopy(base), jets, eff)
            # quick screen on the hover pitch alone before the full scoring
            th = af.trim_hover_pitch(step=0.5)
            if th is None or th > cap + 1.0:
                rows.append({"ts": F.now(), "eff": eff, **jets, "status": "hover above cap" if th else "no trim", "hover_coarse": th})
                continue
            h = F.hover(af)
            ok = h is not None and h["hover_pitch_deg"] <= cap
            rows.append({"ts": F.now(), "eff": eff, **jets, "status": "ok" if ok else "hover above cap", **(h or {})})
        print(F.now(), f"eff {eff}: {time.time() - t0:.0f} s")
    meta = {"ts": F.now(), "park_deg": a.park, "hover_cap_deg": cap, "max_hover": a.max_hover, "max_rotation": a.max_rotation,
            "grid": [a.lo, a.hi, a.step], "effs": EFFS, "source": str(F.SRC.relative_to(ROOT))}
    (run / "jet_static.json").write_text(json.dumps({"meta": meta, "rows": rows}, default=float))
    print(run)


if __name__ == "__main__":
    main()
