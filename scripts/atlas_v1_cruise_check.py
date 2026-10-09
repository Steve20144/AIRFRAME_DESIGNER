"""Headless Cruise Test over several speeds: trim, static margin, pitch-plane flight, six-axis flight with and without
the lateral controller. Writes one JSON per speed (full timeseries, replayable) and prints a summary table.

  .venv/bin/python scripts/atlas_v1_cruise_check.py --airframe airframes/atlas_v1_m002.json --speeds 60,90,110 --out results/atlas_v1/m002
"""
import argparse, json, time
from pathlib import Path
from airframe_designer.geometry.airframe import Airframe
from airframe_designer.analysis import cruise

ap = argparse.ArgumentParser()
ap.add_argument("--airframe", required=True); ap.add_argument("--speeds", default="60,90"); ap.add_argument("--out", required=True)
ap.add_argument("--duration", type=float, default=30.0); ap.add_argument("--kick", type=float, default=1.0)
a = ap.parse_args()
out = Path(a.out); out.mkdir(parents=True, exist_ok=True)
af = Airframe.load(a.airframe); af.resolve_mass()
stem = Path(a.airframe).stem
print(f"{stem}: {af.mass.mass:.1f} kg, CG FRD {[round(float(x), 3) for x in af.mass.cg]}")
print("speed | alpha | elevon | L/W | L/D | SM | NP x | pitch-plane | six axes (controlled) | six axes (open)")
for kmh in [float(s) for s in a.speeds.split(",")]:
    t0 = time.time()
    r = cruise.cruise_test(af, speed_kmh=kmh, duration_s=a.duration, perturb_q_deg_s=a.kick)
    (out / f"cruise_{stem}_{int(kmh)}kmh.json").write_text(json.dumps(r, default=float))
    tr = r["sweep"].get("trim") or {}
    sw = r["sweep"]
    f, f6, f6o = r.get("flight"), r.get("flight6"), r.get("flight6_open")
    lab = lambda x: (x["verdict"]["label"] if x else "not run")
    print(f"{kmh:5.0f} | {tr.get('alpha_deg', float('nan')):5.1f} | {tr.get('elevon_deg', float('nan')):+6.1f} | {tr.get('L_over_W', float('nan')):4.2f} | "
          f"{(tr.get('L_over_D') or float('nan')):4.1f} | {100 * (tr.get('static_margin') or float('nan')):5.0f} % | {(tr.get('neutral_point_x') or float('nan')):5.2f} | "
          f"{lab(f)} | {lab(f6)}{(' [' + f6['verdict'].get('lateral', '') + ']') if f6 and f6['verdict'].get('lateral') else ''} | {lab(f6o)}   ({time.time() - t0:.0f} s)")
