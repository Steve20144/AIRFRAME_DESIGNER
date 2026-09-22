"""Run with: python scripts/fw_nose_lift_sweep_tasks.py, then
  python -m airframe_designer batch --tasks results/fw_nose_lift_sweep/tasks.json --workers 6
      --instances 1,2,3,4,5,6 --px4-dir ~/PX4-nl --out results/fw_nose_lift_sweep/results.jsonl
and python scripts/fw_nose_lift_sweep_analyse.py.

Park-pitch sweep of the firmware nose lift: +4 (ATLAS_09B as parked today) down to -8 deg, every scenario."""
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
HERE = ROOT / "results" / "fw_nose_lift_sweep"   # tasks, results and time series
HERE.mkdir(parents=True, exist_ok=True)
tasks = []
for park in (4, 2, 0, -2, -4, -6, -8):
    for name in ("takeoff", "kill", "cancel"):
        sc = json.loads((ROOT / "scenarios" / f"fw_nose_lift_{name}.json").read_text())
        sc["name"] = f"{sc['name']}_park{park:+d}"
        sc["attitude"] = {"park_pitch_deg": park, "hover_pitch_deg": 24}
        for ph in sc["phases"]:
            u = ph.get("until") or {}
            if "pitch_deg" in u:           # "back where it started" follows the park
                u["pitch_deg"] = float(park)
        tid = f"{name}_park{park:+d}"
        tasks.append({"id": tid, "airframe": "airframes/atlas_09b.json", "scenario": sc,
                      "options": {"timeseries_path": str(HERE / f"{tid}_ts.json")}})
(HERE / "tasks.json").write_text(json.dumps(tasks, indent=1))
print(len(tasks), "tasks")
