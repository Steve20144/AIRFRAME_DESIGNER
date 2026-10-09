"""SITL selection sweep (2026-10-08): one firmware build, a list of parameter candidates, each flown with the nose
fans' physical thrust at several values (PX4 and the module still believe 36 N, as on the board). Board parameters
come from results/board_params/params_20261007_board_now_reconstructed.json; candidate params override them.

  python scripts/fw_select_sweep.py --fw A --out results/fw_select_20261008 --cands A_ramp05:NL_AUTO_RAMP=0.5,NL_AUTO_SLEW=1.5 ...
"""
import argparse, json, re, subprocess, sys
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SEED = 0
HOVER_WINDOW = 4.0
AIRFRAME = "airframes/atlas_v3_30_jets_57.5_85_85_park-10.json"
SCENARIO = "scenarios/fw_auto_hop.json"
# flight-relevant board values that differ from the SITL export (EKF2 matched by enabling the simulated H-FLOW)
BOARD_EXTRA = {"COM_DISARM_LAND": -1.0, "COM_RC_OVERRIDE": 0, "MPC_LAND_SPEED": 0.3, "MPC_TKO_SPEED": 2.0}


def board_params(known: set[str]) -> dict:
    b = json.load(open(ROOT / "results/board_params/params_20261007_board_now_reconstructed.json"))["params"]
    out = dict(BOARD_EXTRA)
    for k, v in b.items():
        if (k.startswith("NL_") or re.fullmatch(r"CA_ROTOR[0-8]_\w+", k) or k == "CA_ROTOR_COUNT") and k in known:
            out[k] = v
    return out


def known_params(px4_dir: Path) -> set[str]:
    meta = json.load(open(px4_dir / "build/px4_sitl_default/parameters.json"))
    return {p["name"] for p in meta["parameters"]}


def classify(r: dict) -> str:
    m = r.get("metrics", {})
    if m.get("crashed"):
        return "CRASH"
    if r.get("ok"):
        return "PASS"
    if (m.get("max_alt_m") or 0) < 0.5 and (m.get("max_tilt_deg") or 99) < 30:
        return "NOGO"          # never left the ground, nothing tipped: a safe refusal / abort
    return "PARTIAL"


def one(job):
    fw, name, params, thrust, inst, outdir, px4_dir = job
    tag = f"{fw}_{name}_n{thrust}_s{SEED}"
    out = outdir / f"{tag}.json"; ts = outdir / f"{tag}_ts.json"
    cmd = [sys.executable, "-m", "airframe_designer", "run", "--px4-dir", str(px4_dir), "--instance", str(inst), "--seed", str(SEED),
           "--airframe", AIRFRAME, "--scenario", SCENARIO, "--set", "design.flow_sensor.enabled=true",
           "--set", f"rotors[6:9].max_thrust={thrust}", "--out", str(out), "--timeseries", str(ts), "--timeout", "900"]
    for k, v in params.items():
        cmd += ["--param", f"{k}={v}"]
    r = None
    for attempt in range(3):   # a PX4 start-up that never links to the simulator is retried (not a flight result)
        subprocess.run(cmd, cwd=ROOT, capture_output=True, text=True)
        try:
            r = json.load(open(out))
        except Exception:
            r = None
        if r is not None and r.get("status") != "error" and ts.exists():
            break
    if r is None or r.get("status") == "error" or not ts.exists():
        return dict(fw=fw, cand=name, thrust=thrust, cls="ERROR", err=str((r or {}).get("failures")))
    m = r.get("metrics", {})
    rows = json.load(open(ts))["timeseries"]; cols = rows["columns"]; ix = {c: i for i, c in enumerate(cols)}
    ev = m.get("events", [])
    tk = next((e["t"] for e in ev if e["text"].startswith("wait_takeoff")), None)
    fl = [x for x in rows["rows"] if tk is not None and tk < x[0] < tk + 12]
    hold = [x[ix["util_max"]] for x in rows["rows"] if tk is not None and tk - 2 < x[0] < tk]
    lift = next((float(e["text"].split("after ")[1].rstrip("s")) for e in ev if e["text"].startswith("lift")), None)
    msgs = [l for l in r.get("log", []) if "Nose lift" in l or "nose lift" in l.lower()]
    # hover steadiness: from 4 s after the takeoff to the start of the descent (or 6 s later); touchdown: the
    # vertical speed when the aircraft stops being airborne
    R = rows["rows"]; air = [x for x in R if tk is not None and x[0] > tk and x[ix["airborne"]] > 0.5]
    hov = [x for x in air if tk + 4 < x[0] < tk + 4 + HOVER_WINDOW]
    hstd = hdrift = vtd = None
    if len(hov) > 5:
        import statistics as _st
        alts = [-x[ix["d"]] for x in hov]; hstd = round(_st.pstdev(alts), 3)
        n0, e0 = hov[0][ix["n"]], hov[0][ix["e"]]
        hdrift = round(max(((x[ix["n"]] - n0) ** 2 + (x[ix["e"]] - e0) ** 2) ** 0.5 for x in hov), 2)
    if air:
        t_last = air[-1][0]; before = [x for x in R if t_last - 0.3 < x[0] <= t_last]
        if before: vtd = round(max(x[ix["vd"]] for x in before), 2)
    return dict(fw=fw, cand=name, thrust=thrust, cls=classify(r), lift_s=lift,
                hold_cmd=round(sum(hold) / len(hold), 2) if hold else None,
                min_nose=round(min(abs(x[ix["tilt"]]) for x in fl), 1) if fl else None,
                sat_share=round(sum(1 for x in fl if x[ix["util_max"]] > 0.99) / len(fl), 2) if fl else None,
                fwd=round(max((x[ix["vn"]] ** 2 + x[ix["ve"]] ** 2) ** 0.5 for x in fl), 2) if fl else None,
                max_alt=m.get("max_alt_m"), max_tilt=m.get("max_tilt_deg"), touchdown=m.get("touchdown_speed"),
                failures=r.get("failures"), crash=m.get("crash_reason"), msgs=msgs[-6:],
                hover_alt_std=hstd, hover_drift=hdrift, vz_touchdown=vtd)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--fw", required=True); ap.add_argument("--out", required=True)
    ap.add_argument("--px4-dir", default=str(Path.home() / "PX4-nl"))
    ap.add_argument("--thrusts", default="36,30,28,25,23")
    ap.add_argument("--instances", default="6,7,8,9")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--hover-window", type=float, default=4.0)
    ap.add_argument("--cands", nargs="+", required=True, help="name:K=V,K=V")
    a = ap.parse_args()
    global SEED, HOVER_WINDOW; SEED = a.seed; HOVER_WINDOW = a.hover_window
    px4 = Path(a.px4_dir); outdir = ROOT / a.out; outdir.mkdir(parents=True, exist_ok=True)
    known = known_params(px4); base = board_params(known)
    insts = [int(x) for x in a.instances.split(",")]
    jobs = []
    for c in a.cands:
        name, _, kv = c.partition(":")
        p = dict(base)
        for item in filter(None, kv.split(",")):
            k, v = item.split("="); p[k] = float(v) if "." in v else int(v)
        unknown = [k for k in p if k not in known and k not in BOARD_EXTRA]
        if unknown:
            sys.exit(f"{name}: not on this firmware: {unknown}")
        for t in [float(x) for x in a.thrusts.split(",")]:
            jobs.append([a.fw, name, p, t, None, outdir, px4])
    for i, j in enumerate(jobs):
        j[4] = insts[i % len(insts)]
    res = []
    with ThreadPoolExecutor(len(insts)) as ex:
        # jobs on the same instance must not overlap: run in waves of len(insts)
        for k in range(0, len(jobs), len(insts)):
            res += list(ex.map(one, [tuple(j) for j in jobs[k:k + len(insts)]]))
    summ = outdir / f"summary_{a.fw}.json"
    old = json.load(open(summ)) if summ.exists() else []
    json.dump(old + res, open(summ, "w"), indent=1)
    for r in res:
        print(f"{r['fw']:3s} {r['cand']:28s} {r['thrust']:5.1f}N {r['cls']:8s} lift {r.get('lift_s')!s:6s} hold {r.get('hold_cmd')!s:5s} "
              f"nose_min {r.get('min_nose')!s:5s} fwd {r.get('fwd')!s:5s} alt {r.get('max_alt')!s:5s} td {r.get('touchdown')!s:6s} "
              f"hov_std {r.get('hover_alt_std')!s:6s} hov_drift {r.get('hover_drift')!s:5s} vz_td {r.get('vz_touchdown')!s:5s} {r.get('crash') or ''}")


if __name__ == "__main__":
    main()
