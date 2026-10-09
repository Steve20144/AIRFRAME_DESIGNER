"""Per-candidate, per-thrust table of the jolt / drift targets of a sitl_pack run (mean and worst over seeds).
  python scripts/sitl_pack_table.py results/sitl_pack/X [keys...]"""
import json, math, sys
from collections import defaultdict
d = sys.argv[1]; keys = sys.argv[2:] or ["jolt_max", "drift_total", "hover_drift", "hover_alt_std", "climb_q_rms", "td_vz", "nose_drop_before_lower", "peak_alt"]
s = json.load(open(f"{d}/summary.json")); g = defaultdict(list)
for r in s["results"]: g[(r["cand"], r["thrust"])].append(r)
def stat(rs, k):
    v = [r[k] for r in rs if isinstance(r.get(k), (int, float)) and math.isfinite(r[k])]
    return (sum(v) / len(v), max(v)) if v else (float("nan"), float("nan"))
print(f"{'candidate':22s} {'N':>3s} {'pass':>5s} " + " ".join(f"{k[:16]:>16s}" for k in keys) + "   worst jolt where")
for (c, th), rs in sorted(g.items(), key=lambda x: (x[0][0], -x[0][1])):
    ok = sum(r["cls"] == "PASS" for r in rs)
    cells = []
    for k in keys:
        m, w = stat(rs, k); cells.append(f"{m:7.2f} ({w:6.2f})")
    where = ",".join(sorted({str(r.get('jolt_where')) for r in rs}))
    print(f"{c:22s} {th:3g} {ok}/{len(rs):<3d} " + " ".join(f"{x:>16s}" for x in cells) + f"   {where}")
bad = [r for r in s["results"] if r["cls"] != "PASS"]
for r in bad: print("NOT PASS:", r["cand"], r["thrust"], r["seed"], r["cls"], r.get("failures"))
