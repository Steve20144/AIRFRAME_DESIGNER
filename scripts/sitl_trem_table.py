"""Per-candidate means of the tremble / jolt metrics of a sitl_pack run (tremble from scripts/hitl_pack.tremble).

  python scripts/sitl_trem_table.py results/sitl_pack/X
"""
import json, sys
from collections import defaultdict
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
from hitl_pack import tremble

d = Path(sys.argv[1]); S = json.load(open(d / "summary.json"))["results"]
keys = ["trem_rms", "trem_climb", "trem_hover", "trem_worst1s", "handover_q_rms", "jolt_liftoff_q", "jolt_max", "lower_rate_max", "front_touch_rate", "hover_drift", "td_vz"]
by = defaultdict(list)
for r in S:
    p = d / f"{r['cand']}_n{r['thrust']:g}_s{r['seed']}_ts.json"
    if r["cls"] == "PASS" and p.exists():
        ts = json.load(open(p)); r.update(tremble(ts.get("timeseries", ts)))
    by[r["cand"]].append(r)
print(f"{'cand':10s} pass " + " ".join(f"{k[:13]:>13s}" for k in keys))
for c, rs in by.items():
    ok = [r for r in rs if r["cls"] == "PASS"]
    def m(k):
        v = [r[k] for r in ok if isinstance(r.get(k), (int, float))]
        return f"{sum(v) / len(v):13.3f}" if v else f"{'-':>13s}"
    print(f"{c:10s} {len(ok)}/{len(rs)} " + " ".join(m(k) for k in keys))
