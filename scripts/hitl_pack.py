"""Headless HITL package: the automatic hop flown on the Pixhawk (the app in HITL mode, no browser) once per candidate
parameter set, each flight scored with the SITL package's metrics (oscillation, vibration, jolts, hover, landing).

  python scripts/hitl_pack.py --spec studies/hitl_X.json --out results/hitl_pack/X [--app http://127.0.0.1:8085]

Spec (JSON):
  {"base": {"MC_PITCH_P": 4.0, ...},                 PX4 params every flight starts from
   "candidates": {"name": {"PARAM": value, ...}, ...},
   "repeats": 2, "scenario": "fw_auto_hop_winner", "hover_s": 5}

The board keeps whatever a flight pushed, so every flight sends the base values of every parameter any candidate
touches, then its own. Flights are real time and one at a time (one board): about 1.5 to 2 min each with the reboot.
"""
from __future__ import annotations

import argparse, json, sys, time, urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from sitl_pack import score_run  # noqa: E402


def call(app: str, path: str, body: dict | None = None, timeout: float = 120.0) -> dict:
    req = urllib.request.Request(app + path, data=None if body is None else json.dumps(body).encode(),
                                 headers={"Content-Type": "application/json"}, method="GET" if body is None else "POST")
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return json.loads(r.read())
    except urllib.error.HTTPError as e:
        try:
            return json.loads(e.read())
        except Exception:
            return {"ok": False, "error": f"HTTP {e.code}"}


def fly(app: str, scenario: str, params: dict, name: str, max_s: float = 400.0) -> dict:
    for attempt in range(3):
        r = call(app, "/api/tuning/run", {"live": True, "scenario": scenario, "params": params, "name": name}, timeout=240)
        if r.get("ok"):
            break
        print(f"   start failed ({r.get('error')}), retry {attempt + 1}", flush=True)
        time.sleep(10)
    else:
        return {"cls": "NOSTART", "error": r.get("error")}
    run_id = r["id"]
    t0 = time.time()
    while time.time() - t0 < max_s:
        time.sleep(3)
        s = call(app, "/api/scenario/status")
        if not s.get("running", True) and "result" in s:
            break
    else:
        call(app, "/api/scenario/stop", {})
        time.sleep(3)
        s = call(app, "/api/scenario/status")
    time.sleep(1.5)       # the status call that saw the end also saved the run
    out = {"id": run_id, "status": s.get("status"), "ok": s.get("ok"), "failures": s.get("failures")}
    p = ROOT / "results/tuning" / f"{run_id}.json"
    for d in (ROOT / "results/tuning", Path.home() / ".airframe_designer/tuning"):
        if (d / f"{run_id}.json").exists():
            p = d / f"{run_id}.json"
    out["run_path"] = str(p)
    if p.exists():
        res = json.load(open(p))
        tsp = res.get("timeseries_path")
        out["metrics_app"] = {k: res.get("metrics", {}).get(k) for k in ("max_tilt_deg", "max_alt_m", "touchdown_speed", "crashed")}
        if tsp and Path(tsp).exists():
            out["ts_path"] = tsp
    out["cls"] = "PASS" if s.get("ok") else ("FAIL" if s.get("status") else "NORESULT")
    return out


def tremble(ts: dict) -> dict:
    """The shake the eye sees: roll / pitch rate content between 2 and 10 Hz (deg/s rms), over the whole time in the
    air and per segment (handover + climb, hover, descent), and the worst 1 s window of it."""
    import numpy as np
    c = {k: i for i, k in enumerate(ts["columns"])}
    a = np.array(ts["rows"], float)
    if len(a) < 100:
        return {}
    t = a[:, c["t"]]; dt = float(np.median(np.diff(t)))
    st = a[:, c["nl_state"]]; air = a[:, c["airborne"]] > 0.5

    def band(x):
        f = np.fft.rfftfreq(len(x), dt); X = np.fft.rfft(x - x.mean())
        X[(f < 2.0) | (f > 10.0)] = 0
        return np.fft.irfft(X, len(x))

    out = {}
    fl = np.where(air | (st == 5))[0]
    if len(fl) < 64:
        return out
    i0, i1 = fl[0], fl[-1] + 1
    seg = slice(i0, i1)
    bp = band(np.degrees(a[seg, c["p"]])); bq = band(np.degrees(a[seg, c["q"]]))
    mag = np.sqrt(bp ** 2 + bq ** 2)
    out["trem_rms"] = round(float(np.sqrt(np.mean(mag ** 2))), 3)
    w = max(int(1.0 / dt), 4)
    roll = np.sqrt(np.convolve(mag ** 2, np.ones(w) / w, mode="valid"))
    out["trem_worst1s"] = round(float(roll.max()), 3)
    out["trem_worst_t"] = round(float(t[i0 + int(np.argmax(roll))] - t[i0]), 1)
    alt = -a[seg, c["d"]]; hi = alt > 0.85 * alt.max()
    if hi.sum() > 50:
        out["trem_hover"] = round(float(np.sqrt(np.mean(mag[hi] ** 2))), 3)
    k = np.argmax(hi) if hi.any() else len(mag)
    if k > 20:
        out["trem_climb"] = round(float(np.sqrt(np.mean(mag[:k] ** 2))), 3)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--spec", required=True); ap.add_argument("--out", required=True)
    ap.add_argument("--app", default="http://127.0.0.1:8085")
    a = ap.parse_args()
    spec = json.load(open(a.spec)); out = Path(a.out); out.mkdir(parents=True, exist_ok=True)
    base = dict(spec.get("base") or {}); cands = spec["candidates"]
    touched = {k for c in cands.values() for k in c}
    missing = touched - set(base)
    if missing:
        sys.exit(f"base must give a value for every parameter a candidate touches: {sorted(missing)}")
    reps = int(spec.get("repeats", 1)); scen = spec.get("scenario", "fw_auto_hop_winner"); hover_s = float(spec.get("hover_s", 5))
    order = [(c, r) for r in range(reps) for c in cands]          # interleaved: drift in the board / PC load spreads evenly
    results = []
    sp = out / "summary.json"
    if sp.exists():
        results = json.load(open(sp))["results"]
    done = {(r["cand"], r["rep"]) for r in results if r["cls"] in ("PASS", "FAIL")}
    t_all = time.time()
    for i, (c, rep) in enumerate(order, 1):
        if (c, rep) in done:
            continue
        params = dict(base); params.update(cands[c])
        t0 = time.time()
        r = fly(a.app, scen, params, f"{out.name}:{c}:{rep}")
        r.update({"cand": c, "rep": rep, "params": cands[c], "wall_s": round(time.time() - t0, 1)})
        if r.get("ts_path"):
            try:
                ts = json.load(open(r["ts_path"]))["timeseries"]
                r.update(score_run(ts, hover_s)); r.update(tremble(ts))
            except Exception as e:
                r["score_error"] = str(e)
        results = [x for x in results if (x["cand"], x["rep"]) != (c, rep)] + [r]
        json.dump({"spec": spec, "results": results, "flights": len(results), "wall_s": round(time.time() - t_all),
                   "instances": ["hitl"]}, open(sp, "w"), indent=1)
        g = lambda k: r.get(k)
        print(f"[{i}/{len(order)}] {c:14s} r{rep} {r['cls']:8s} trem {g('trem_rms')} worst1s {g('trem_worst1s')}@{g('trem_worst_t')} "
              f"jolt_max {g('jolt_max')} ({g('jolt_where')}) lower_max {g('lower_rate_max')} front {g('front_touch_rate')} "
              f"climb_q_rms {g('climb_q_rms')} hover_rates_rms {g('hover_rates_rms')} vib_gyro {g('vib_gyro_hover')} "
              f"vib_acc {g('vib_acc_hover')} pitch_osc {g('hover_pitch_osc_deg')} jitter {g('hover_cmd_jitter')} "
              f"drift {g('hover_drift')} td_vz {g('td_vz')} ({r['wall_s']} s) {r.get('failures') or ''}", flush=True)
    print(f"done: {len(results)} flights -> {sp}")


if __name__ == "__main__":
    main()
