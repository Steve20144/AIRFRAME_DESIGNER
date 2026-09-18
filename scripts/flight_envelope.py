#!/usr/bin/env python
"""Fly a grid over the inputs nobody has measured, and report where the aircraft still hovers.

An optimiser answers "what is the best design?". This answers a different question: "given that several numbers in
this model were never measured, over how much of their plausible range does the conclusion survive?" That is the
question a first flight actually turns on, and it is the one a single simulation run cannot answer.

Each axis is a parameter path with a list of values to try; every combination is flown as its own closed-loop PX4
run, and the result is a pass/fail grid with the thrust margin at each point. A conclusion that holds across the
whole grid is worth something. One that holds only at the nominal point is worth nothing, and this makes the
difference visible instead of leaving it to be assumed.

    scripts/flight_envelope.py --airframe airframes/atlas_phase01_sym.json --scenario hover \
        --axis "rotors[*].max_thrust=22,26,30,33.3,36" \
        --axis "rotors[*].turn_loss=0.0,0.1,0.2,0.3,0.4" \
        --axis "mass.mass=11.191,11.8,12.5" \
        --fixed "rotors[0,2,4,6].km=0.01" --workers 6
"""
from __future__ import annotations

import argparse
import itertools
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from airframe_designer.batch.runner import run_many  # noqa: E402


def parse_axis(spec: str) -> tuple[str, list]:
    """``path=v1,v2,v3`` into (path, [values]). Values are JSON, so strings and booleans work too."""
    if "=" not in spec:
        raise SystemExit(f"--axis expects path=v1,v2,..., got {spec!r}")
    path, raw = spec.split("=", 1)
    values = []
    for item in raw.split(","):
        try:
            values.append(json.loads(item))
        except json.JSONDecodeError:
            values.append(item)
    return path.strip(), values


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--airframe", required=True)
    ap.add_argument("--scenario", required=True)
    ap.add_argument("--axis", action="append", required=True, metavar="PATH=V1,V2,...",
                    help="one swept parameter (repeatable); every combination is flown")
    ap.add_argument("--fixed", action="append", metavar="PATH=VALUE",
                    help="a parameter held at one value across the whole grid (repeatable)")
    ap.add_argument("--phase", default="hover", help="which scenario phase to report margins from")
    ap.add_argument("--workers", type=int, default=6)
    ap.add_argument("--instances", default=None, help="comma-separated PX4 instances (default: free ones in 1..9)")
    ap.add_argument("--out", default="results/envelope")
    a = ap.parse_args(argv)

    axes = [parse_axis(s) for s in a.axis]
    fixed = dict(parse_axis(s) for s in (a.fixed or []))
    fixed = {k: v[0] for k, v in fixed.items()}

    combos = list(itertools.product(*[v for _, v in axes]))
    tasks = []
    for i, combo in enumerate(combos):
        variables = dict(fixed)
        variables.update({path: value for (path, _), value in zip(axes, combo)})
        tasks.append({"id": f"p{i:04d}", "airframe": a.airframe, "scenario": a.scenario, "variables": variables})

    out_dir = Path(a.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    print(f"{len(tasks)} points over {len(axes)} axes, {a.workers} workers")
    for path, values in axes:
        print(f"  {path}: {values}")
    if fixed:
        print(f"  held fixed: {fixed}")

    def progress(r, done, total):
        print(f"[{done}/{total}] {r.get('id')} {'ok' if r.get('ok') else 'FAILED'}", flush=True)

    instances = [int(x) for x in a.instances.split(",")] if a.instances else None
    results = run_many(tasks, workers=a.workers, instances=instances, progress=progress, speed=0.0)

    rows = []
    for task, r in zip(tasks, results):
        ph = (r.get("metrics", {}).get("phases") or {}).get(a.phase) or {}
        rows.append({
            "variables": task["variables"], "ok": bool(r.get("ok")), "status": r.get("status"),
            "util_max": ph.get("util_max"), "pos_std_xy": ph.get("pos_std_xy"),
            "roll_rms_deg": ph.get("roll_rms_deg"), "alt_mean": ph.get("alt_mean"),
            "failures": r.get("failures") or [],
        })
    (out_dir / "envelope.json").write_text(json.dumps(
        {"airframe": a.airframe, "scenario": a.scenario, "axes": [{"path": p, "values": v} for p, v in axes],
         "fixed": fixed, "points": rows}, indent=2))

    flew = sum(1 for r in rows if r["ok"])
    print(f"\n{flew}/{len(rows)} points hovered  ->  {out_dir / 'envelope.json'}")
    if flew:
        worst = max((r for r in rows if r["ok"] and r["util_max"] is not None),
                    key=lambda r: r["util_max"], default=None)
        if worst:
            print(f"tightest thrust margin among the points that flew: {worst['util_max']:.3f} at {worst['variables']}")
    for r in rows:
        if not r["ok"]:
            print(f"  did not hover: {r['variables']}  {'; '.join(r['failures'])[:90]}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
