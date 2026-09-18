"""Command line: the interactive app, headless runs, batches, studies and static analysis.

  airframe-designer ui        [--airframe X] [--mode sitl|hitl|auto] ...     3D editor + PX4 SITL/HITL + remote
  airframe-designer run       --airframe X --scenario Y [--set path=value] [--out r.json]
  airframe-designer batch     --tasks tasks.json [--workers 4] [--out results.jsonl]
  airframe-designer study     --spec study.json [--workers 4]
  airframe-designer analyse   --airframe X [--speed-kmh 50]
  airframe-designer optimise  --airframe X --spec spec.json           (static, no PX4)
  airframe-designer export    --airframe X [--hitl] [--out file.params]
  airframe-designer vehicle   --manifest vehicle-manifest.json --out airframes/X.json   (CAD -> airframe)
  airframe-designer paths     --airframe X                              (every variable path)
  airframe-designer scenarios                                            (list the bundled scenarios)
  airframe-designer migrate   old.json new.json
"""
from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path

PROJECT_DIR = Path(__file__).resolve().parents[1]


def _parse_set(items: list[str] | None) -> dict:
    out = {}
    for it in items or []:
        if "=" not in it:
            raise SystemExit(f"--set expects path=value, got '{it}'")
        k, v = it.split("=", 1)
        try:
            out[k.strip()] = json.loads(v)
        except json.JSONDecodeError:
            out[k.strip()] = v
    return out


def cmd_run(a) -> int:
    from .batch.worker import run_once
    r = run_once(a.airframe, a.scenario, variables=_parse_set(a.set), px4_dir=a.px4_dir, instance=a.instance, speed=a.speed,
                 rate=a.rate, substeps=a.substeps, noise=not a.no_noise, seed=a.seed, timeout_wall=a.timeout,
                 log=(print if a.verbose else None), quiet=not a.verbose, extra_params=_parse_set(a.param),
                 timeseries_path=a.timeseries, physics=a.physics)
    if not a.keep_airframe:
        r.pop("airframe", None)
    text = json.dumps(r, indent=2)
    if a.out:
        Path(a.out).write_text(text)
        print(f"{'ok' if r['ok'] else 'FAILED'} {r['status']} sim {r.get('timing', {}).get('sim_s')}s wall {r.get('timing', {}).get('wall_s')}s rtf {r.get('timing', {}).get('rtf')} -> {a.out}")
        for f in r.get("failures", []):
            print("  -", f)
    else:
        print(text)
    return 0 if r["ok"] else 1


def cmd_batch(a) -> int:
    from .batch.runner import run_many
    tasks = json.loads(Path(a.tasks).read_text())
    if isinstance(tasks, dict):
        tasks = tasks.get("tasks", [])
    for k, t in enumerate(tasks):
        t.setdefault("id", f"task{k}")
        if "set" in t and "variables" not in t:
            t["variables"] = t.pop("set")
    out = open(a.out, "a") if a.out else None

    def progress(r, done, total):
        tm = r.get("timing", {})
        print(f"[{done}/{total}] {r.get('id')} {'ok' if r.get('ok') else 'FAILED'} {r.get('status')} sim {tm.get('sim_s')}s wall {tm.get('wall_s')}s rtf {tm.get('rtf')}", flush=True)
        if out:
            if not a.keep_airframe:
                r.pop("airframe", None)
            out.write(json.dumps(r) + "\n"); out.flush()

    instances = [int(x) for x in a.instances.split(",")] if a.instances else None
    results = run_many(tasks, workers=a.workers, instances=instances, progress=progress, px4_dir=a.px4_dir, speed=a.speed,
                       rate=a.rate, substeps=a.substeps, noise=not a.no_noise, timeout_wall=a.timeout, physics=a.physics)
    if out:
        out.close()
    else:
        for r in results:
            r.pop("airframe", None)
        print(json.dumps(results, indent=2))
    return 0 if all(r.get("ok") for r in results) else 1


def cmd_compare(a) -> int:
    """Run the same flight on both physics engines and report the differences."""
    from .batch.compare import compare_physics
    r = compare_physics(a.airframe, a.scenario, variables=_parse_set(a.set), px4_dir=a.px4_dir, rate=a.rate, substeps=a.substeps,
                        noise=not a.no_noise, timeout_wall=a.timeout, instances=[int(x) for x in a.instances.split(",")] if a.instances else None,
                        out_path=a.out)
    print(r["report"])
    return 0 if r["ok"] else 1


def cmd_study(a) -> int:
    from .batch.study import run_study
    s = run_study(a.spec, workers=a.workers, out_dir=a.out)
    print(json.dumps({k: v for k, v in s.items() if k != "best"}, indent=2))
    if s.get("best"):
        b = s["best"]
        print("best:", json.dumps({"score": b["score"], "objective": b["objective"], "values": b["values"], "feasible": b["feasible"]}, indent=2))
    return 0


def cmd_analyse(a) -> int:
    from .geometry.airframe import Airframe
    from .analysis.static import analyse
    af = Airframe.load(a.airframe)
    if a.set:
        from .geometry.paths import apply_variables
        af = apply_variables(af, _parse_set(a.set))
    r = analyse(af, (a.speed_kmh / 3.6) if a.speed_kmh else None)
    r["hover_check"] = af.hover_check()
    r["validate"] = af.validate()
    print(json.dumps(r, indent=2, default=float))
    return 0


def cmd_optimise(a) -> int:
    from .geometry.airframe import Airframe
    from .analysis.geometric_optimiser import optimise
    af = Airframe.load(a.airframe)
    spec = json.loads(Path(a.spec).read_text()) if a.spec else {"variables": [], "hover_pitch": [0, 60]}
    r = optimise(af, spec, progress=lambda f, m: print(f"  {m} {f * 100:.0f}%", file=sys.stderr, flush=True))
    for m in r.get("results", []):
        m.pop("airframe", None)
    r.get("current", {}).pop("airframe", None)
    print(json.dumps(r, indent=2, default=float))
    return 0


def cmd_export(a) -> int:
    from .geometry.airframe import Airframe
    af = Airframe.load(a.airframe)
    text = af.px4_params_file(hitl=a.hitl)
    if a.out:
        Path(a.out).write_text(text); print(a.out)
    else:
        print(text)
    return 0


def cmd_paths(a) -> int:
    from .geometry.airframe import Airframe
    from .geometry.paths import list_paths, get_path
    af = Airframe.load(a.airframe)
    for p in list_paths(af):
        try:
            print(f"{p} = {get_path(af, p)}")
        except Exception:
            print(p)
    return 0


def cmd_vehicle(a) -> int:
    """Generate an airframe from the CAD-derived vehicle documents, with its provenance sidecar."""
    from .geometry.airframe import Airframe
    from .vehicle import airframe_from_cad, load_vehicle
    v = load_vehicle(a.manifest, a.geometry)
    d, sidecar = airframe_from_cad(v, name=a.name, turn_loss=a.turn_loss, km_magnitude=a.km,
                                   tau_s=a.tau, leg_clearance_m=a.leg_clearance, leg_splay_deg=a.leg_splay,
                                   symmetrise=a.symmetrise)
    out = Path(a.out or f"airframes/{v.vehicle_id.replace('-', '_')}.json")
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(d, indent=2))
    # Sidecars live in their own directory: everything directly under airframes/ is an airframe, and
    # tests/test_geometry.py enforces that by loading every *.json it finds there.
    side = out.parent / "provenance" / f"{out.stem}.json"
    side.parent.mkdir(parents=True, exist_ok=True)
    side.write_text(json.dumps(sidecar, indent=2))

    af = Airframe.from_dict(d)
    print(f"{v.display_name} rev {v.revision} [{v.declared_state}] -> {out}")
    print(f"  {af.mass.mass:.3f} kg, CG {af.mass.cg}, {len(af.rotors)} rotors, {len(af.legs)} legs, "
          f"hover pitch {af.hover_pitch_deg:g} deg")
    if sidecar.get("symmetry_correction"):
        moved = sidecar["symmetry_correction"]["moved"]
        print(f"  symmetrised {len(moved)} foil fan(s) from the {a.symmetrise} side")
    if v.unweighed:
        print(f"  mass is INCOMPLETE: {len(v.unweighed)} unweighed part(s): {', '.join(v.unweighed)}")
    for w in af.validate():
        print(f"  ! {w}")
    assumed = sorted({f.get("blocker") for r in sidecar["rotors"] for f in r["fields"].values() if f.get("blocker")})
    print(f"  {len(assumed)} blocker(s) govern generated rotor values: {', '.join(assumed)}")
    print(f"  provenance -> {side}")
    return 0


def cmd_scenarios(a) -> int:
    d = PROJECT_DIR / "scenarios"
    for p in sorted(d.glob("*.json")):
        s = json.loads(p.read_text())
        print(f"{p.name:26s} {s.get('description', '')}")
    return 0


def cmd_migrate(a) -> int:
    from .geometry.airframe import Airframe
    af = Airframe.load(a.src)
    af.save(a.dst)
    print(f"{a.src} -> {a.dst} ({af.name}: {len(af.rotors)} rotors, {len(af.wings)} wings, {len(af.legs)} legs)")
    return 0


def cmd_ui(a, argv_rest) -> int:
    from .app import main as app_main
    return app_main(argv_rest)


def main(argv=None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    ap = argparse.ArgumentParser(prog="airframe-designer", description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd")

    def sim_args(p):
        p.add_argument("--px4-dir", default=None)
        p.add_argument("--speed", type=float, default=0.0, help="real-time factor, 0 = as fast as PX4 allows (default)")
        p.add_argument("--rate", type=float, default=250.0, help="sensor rate Hz")
        p.add_argument("--substeps", type=int, default=2, help="physics sub-steps per sensor step")
        p.add_argument("--no-noise", action="store_true")
        p.add_argument("--timeout", type=float, default=600.0, help="wall-clock limit per run, s")
        p.add_argument("--keep-airframe", action="store_true", help="include the full airframe dict in the output")
        p.add_argument("--physics", choices=["python", "jsbsim"], default="python", help="physics engine (default python)")

    p = sub.add_parser("run", help="one headless simulation"); sim_args(p)
    p.add_argument("--airframe", required=True); p.add_argument("--scenario", required=True)
    p.add_argument("--set", action="append", metavar="PATH=VALUE", help="apply a parameter path first (repeatable)")
    p.add_argument("--param", action="append", metavar="NAME=VALUE", help="extra PX4 parameter (repeatable)")
    p.add_argument("--instance", type=int, default=None); p.add_argument("--seed", type=int, default=1)
    p.add_argument("--out", default=None); p.add_argument("--timeseries", default=None, help="write the sampled time series here (JSON)")
    p.add_argument("-v", "--verbose", action="store_true")

    p = sub.add_parser("batch", help="many runs in parallel from a tasks file"); sim_args(p)
    p.add_argument("--tasks", required=True); p.add_argument("--workers", type=int, default=4); p.add_argument("--out", default=None)
    p.add_argument("--instances", default=None, help="comma-separated PX4 instance numbers to use (default: free ones in 1..9)")

    p = sub.add_parser("compare", help="fly a scenario on the Python and JSBSim physics and diff the results"); sim_args(p)
    p.add_argument("--airframe", required=True); p.add_argument("--scenario", required=True)
    p.add_argument("--set", action="append", metavar="PATH=VALUE"); p.add_argument("--instances", default=None)
    p.add_argument("--out", default=None, help="write the full comparison (both results + time series diffs) as JSON")

    p = sub.add_parser("study", help="simulation-driven optimisation study")
    p.add_argument("--spec", required=True); p.add_argument("--workers", type=int, default=None); p.add_argument("--out", default=None)

    p = sub.add_parser("analyse", help="static hover/cruise analysis"); p.add_argument("--airframe", required=True)
    p.add_argument("--speed-kmh", type=float, default=None); p.add_argument("--set", action="append")
    p = sub.add_parser("optimise", help="static geometric optimiser (no PX4)"); p.add_argument("--airframe", required=True); p.add_argument("--spec", default=None)
    p = sub.add_parser("export", help="PX4 .params file"); p.add_argument("--airframe", required=True); p.add_argument("--hitl", action="store_true"); p.add_argument("--out", default=None)
    p = sub.add_parser("vehicle", help="generate an airframe from the CAD-derived vehicle documents")
    p.add_argument("--manifest", required=True, help="path to vehicle-manifest.json")
    p.add_argument("--geometry", default=None, help="applied geometry config (default: the manifest's as-built one)")
    p.add_argument("--out", default=None); p.add_argument("--name", default=None)
    p.add_argument("--turn-loss", type=float, default=0.1, dest="turn_loss", help="thrust lost at 90 deg of jet turning")
    p.add_argument("--km", type=float, default=0.01, help="reaction-torque magnitude per unit thrust, m")
    p.add_argument("--tau", type=float, default=0.12, help="fan spool time constant, s")
    p.add_argument("--leg-clearance", type=float, default=0.10, dest="leg_clearance", help="ground clearance, m")
    p.add_argument("--leg-splay", type=float, default=30.0, dest="leg_splay", help="foot splay from vertical, deg")
    p.add_argument("--symmetrise", choices=["none", "left", "right"], default="none",
                   help="mirror one side's foil-fan positions onto the other (default none: follow the documents)")
    p = sub.add_parser("paths", help="list variable paths"); p.add_argument("--airframe", required=True)
    sub.add_parser("scenarios", help="list bundled scenarios")
    p = sub.add_parser("migrate", help="convert a schema-1 airframe"); p.add_argument("src"); p.add_argument("dst")
    sub.add_parser("ui", help="interactive app (all further arguments go to it)")
    sub.add_parser("mcp", help="MCP server for AI assistants: `mcp` = stdio, `mcp --http [port]` = streamable HTTP")

    if argv and argv[0] == "mcp":
        from .mcp_server import main as mcp_main
        return mcp_main(argv[1:])
    if argv and argv[0] == "ui":
        return cmd_ui(None, argv[1:])
    if not argv:
        argv = ["ui"]
        return cmd_ui(None, [])
    a = ap.parse_args(argv)
    return {"run": cmd_run, "batch": cmd_batch, "compare": cmd_compare, "study": cmd_study, "analyse": cmd_analyse, "optimise": cmd_optimise,
            "export": cmd_export, "vehicle": cmd_vehicle, "paths": cmd_paths, "scenarios": cmd_scenarios, "migrate": cmd_migrate}[a.cmd](a)


if __name__ == "__main__":
    sys.exit(main())
