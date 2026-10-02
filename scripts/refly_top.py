"""Re-fly a study's top designs on other seeds: writes one `points` study per seed next to the original spec.

  python scripts/refly_top.py studies/<name>.json --top 8 --seeds 2,3 [--min-lean 15] [--tag cad]

--min-lean keeps only designs whose every knob value is at least that (e.g. lean >= 15 deg: jet turning <= 75 deg,
what the CAD foil gives). Prints the spec paths; run each with `airframe_designer study --spec`.
"""
import argparse, json
from pathlib import Path

ap = argparse.ArgumentParser()
ap.add_argument("spec"); ap.add_argument("--top", type=int, default=8); ap.add_argument("--seeds", default="2,3")
ap.add_argument("--min-lean", type=float, default=None); ap.add_argument("--tag", default="")
a = ap.parse_args()
spec = json.loads(Path(a.spec).read_text())
trials = [json.loads(l) for l in (Path("results") / spec["name"] / "trials.jsonl").read_text().splitlines() if l.strip()]
trials = [t for t in trials if t["feasible"] and (a.min_lean is None or min(t["x"]) >= a.min_lean)]
top = sorted(trials, key=lambda t: t["score"])[:a.top]
for t in top:
    print("  ", t["x"], round(t["score"], 3))
for seed in (int(s) for s in a.seeds.split(",")):
    s = dict(spec)
    s["name"] = f"{spec['name']}{'_' + a.tag if a.tag else ''}_s{seed}"
    s["algorithm"] = {"name": "points", "points": [t["x"] for t in top], "budget": len(top), "batch": 8}
    s["sim"] = dict(spec.get("sim") or {}, seed=seed)
    out = Path(a.spec).with_name(s["name"] + ".json")
    out.write_text(json.dumps(s, indent=2))
    print(out)
