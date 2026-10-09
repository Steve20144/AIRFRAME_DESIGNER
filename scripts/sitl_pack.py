"""Headless SITL package for the automatic hop (V3 + the nose-lift firmware): many flights in parallel, each scored for
oscillation, vibration, the smoothness of the nose rotations, the hover and the landing order.

  python scripts/sitl_pack.py --spec studies/hop_pack_X.json --out results/sitl_pack/X

Spec (JSON):
  {"base": {"NL_AUTO_ALT": 1.0, ...},                 PX4 params on top of the board's (reconstructed 7 Oct dump)
   "candidates": {"name": {"PARAM": value, ...}, ...},
   "thrusts": [36, 30], "seeds": [0, 1],             nose-fan physical thrust (PX4 believes 36 N), noise seeds
   "scenario": "scenarios/fw_auto_hop_winner.json", "hover_s": 10,
   "args": ["--accel-bias", "0,0.07,0"], "cand_args": {"name": [...]}}   extra 'run' options (optional)

Speed: one worker per free PX4 instance (TCP 4560+i not in use), each worker runs its jobs back to back so two PX4s
never share an instance (the 8 Oct wave runner sometimes started a job while the previous PX4 on that instance was still
going down: "PX4 never became ready", a spurious 101 deg tip-over); start-up failures are retried.
"""
from __future__ import annotations

import argparse, json, math, queue, re, socket, subprocess, sys, threading, time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
AIRFRAME = "airframes/atlas_v3_30_jets_57.5_85_85_park-10.json"
HOVER_PITCH = 8.5
ST = {"parked": 2, "ramping": 3, "holding": 4, "handover": 5, "flying": 6, "lowering": 7, "aborted": 8}
BOARD_EXTRA = {"COM_DISARM_LAND": -1.0, "COM_RC_OVERRIDE": 0, "MPC_TKO_SPEED": 2.0}


def free_instances(cands=range(1, 10)) -> list[int]:
    out = []
    for i in cands:
        s = socket.socket(); s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            s.bind(("0.0.0.0", 4560 + i)); out.append(i)
        except OSError:
            pass
        finally:
            s.close()
    return out


def known_params(px4: Path) -> set[str]:
    return {p["name"] for p in json.load(open(px4 / "build/px4_sitl_default/parameters.json"))["parameters"]}


def board_params(known: set[str]) -> dict:
    b = json.load(open(ROOT / "results/board_params/params_20261007_board_now_reconstructed.json"))["params"]
    out = dict(BOARD_EXTRA)
    for k, v in b.items():
        if (k.startswith("NL_") or re.fullmatch(r"CA_ROTOR[0-8]_\w+", k) or k == "CA_ROTOR_COUNT") and k in known:
            out[k] = v
    return out


# ------------------------------------------------------------------------------------------------ metrics
def _dom(x: np.ndarray, dt: float) -> tuple[float, float]:
    """dominant frequency (Hz) and its amplitude of a detrended series"""
    if len(x) < 16:
        return float("nan"), float("nan")
    x = x - np.polyval(np.polyfit(np.arange(len(x)), x, 1), np.arange(len(x)))
    f = np.fft.rfftfreq(len(x), dt); P = np.abs(np.fft.rfft(x * np.hanning(len(x)))) * 2 / (0.5 * len(x))
    i = int(np.argmax(P[1:]) + 1)
    return float(f[i]), float(P[i])


def score_run(ts: dict, hover_s: float) -> dict:
    c = {k: i for i, k in enumerate(ts["columns"])}
    a = np.array(ts["rows"], float)
    if len(a) < 10 or "feet_down" not in c:
        return {}
    t = a[:, c["t"]]; dt = float(np.median(np.diff(t)))
    pitch_s = np.degrees(a[:, c["pitch"]]) + HOVER_PITCH          # structural pitch, deg
    q = np.degrees(a[:, c["q"]]); p = np.degrees(a[:, c["p"]]); r = np.degrees(a[:, c["r"]])
    st = a[:, c["nl_state"]]; feet = a[:, c["feet_down"]]; air = a[:, c["airborne"]] > 0.5
    alt = -a[:, c["d"]]; vd = a[:, c["vd"]]
    m: dict = {}

    def rot(mask, target_rate, name):
        if mask.sum() < 10:
            return
        qq = q[mask]; tt = t[mask]
        qf = np.convolve(qq, np.ones(5) / 5, mode="same")                 # ~0.1 s smoothing for the jerk
        jerk = np.diff(qf) / np.maximum(np.diff(tt), 1e-3)
        m[f"{name}_s"] = round(float(tt[-1] - tt[0]), 2)
        m[f"{name}_rate_max"] = round(float(np.max(np.abs(qq))), 2)
        m[f"{name}_rate_dev"] = round(float(np.sqrt(np.mean((np.abs(qf) - target_rate) ** 2))), 2)
        m[f"{name}_jerk_rms"] = round(float(np.sqrt(np.mean(jerk ** 2))), 1)

    # the climb's pitch oscillation ("it trembles when it climbs", 8 Oct): the rate with its slow part (0.5 s mean)
    # removed, in the handover (state 5) and the first 3 s after PX4 has the whole aircraft (state 6)
    def osc_rate(mask, name):
        if mask.sum() < 32:
            return
        x = q[mask] - np.convolve(q[mask], np.ones(25) / 25, mode="same")
        m[f"{name}_q_rms"] = round(float(np.sqrt(np.mean(x ** 2))), 2)
        f, amp = _dom(q[mask], dt); m[f"{name}_q_hz"], m[f"{name}_q_amp"] = round(f, 2), round(amp, 2)
        m[f"{name}_q_max"] = round(float(np.max(np.abs(q[mask]))), 1)
    osc_rate(st == ST["handover"], "handover")
    if (st == ST["flying"]).any():
        t6 = t[st == ST["flying"]][0]
        osc_rate((st == ST["flying"]) & (t < t6 + 3.0), "climb")

    # nose lift (ramping) and the hold
    lift = st == ST["ramping"]
    rot(lift, 3.0, "lift")
    hold = st == ST["holding"]
    if lift.any() or hold.any():
        m["lift_overshoot"] = round(float(np.max(pitch_s[lift | hold]) - HOVER_PITCH), 2)
    m["vib_acc_lift"] = round(float(np.nanmean(a[lift, c["vib_acc"]])), 3) if lift.any() else None
    m["vib_gyro_lift"] = round(float(np.nanmean(a[lift, c["vib_gyro"]])), 4) if lift.any() else None

    # hover window: from the first moment the height is within 10 % of the flight's settled height, hover_s long
    if air.any():
        ta = t[air]; t_take = float(ta[0])
        alt_air = alt[air]
        settled = np.median(alt_air[(ta > t_take + 4) & (ta < t_take + 4 + hover_s)]) if np.any((ta > t_take + 4) & (ta < t_take + 4 + hover_s)) else np.max(alt_air)
        reach = ta[np.argmax(alt_air >= 0.9 * settled)]
        w = air & (t > reach + 1.0) & (t < reach + 1.0 + max(hover_s - 2.0, 2.0))
        if w.sum() > 20:
            ps, rs = a[w, c["pitch_sp"]], a[w, c["roll_sp"]]
            m["hover_alt_std"] = round(float(np.std(alt[w])), 3)
            n0, e0 = a[w, c["n"]][0], a[w, c["e"]][0]
            m["hover_drift"] = round(float(np.max(np.hypot(a[w, c["n"]] - n0, a[w, c["e"]] - e0))), 2)
            m["hover_speed_max"] = round(float(np.max(np.hypot(a[w, c["vn"]], a[w, c["ve"]]))), 2)
            m["hover_pitch_err_rms"] = round(float(np.degrees(np.sqrt(np.nanmean((a[w, c["pitch"]] - ps) ** 2)))), 2)
            m["hover_roll_err_rms"] = round(float(np.degrees(np.sqrt(np.nanmean((a[w, c["roll"]] - rs) ** 2)))), 2)
            m["hover_rates_rms"] = round(float(np.sqrt(np.mean(p[w] ** 2 + q[w] ** 2 + r[w] ** 2))), 2)
            f, amp = _dom(pitch_s[w], dt); m["hover_pitch_osc_hz"], m["hover_pitch_osc_deg"] = round(f, 2), round(amp, 2)
            f, amp = _dom(np.degrees(a[w, c["roll"]]), dt); m["hover_roll_osc_hz"], m["hover_roll_osc_deg"] = round(f, 2), round(amp, 2)
            f, amp = _dom(alt[w], dt); m["hover_alt_osc_hz"], m["hover_alt_osc_m"] = round(f, 2), round(amp, 3)
            m["hover_cmd_jitter"] = round(float(np.mean(a[w, c["cmd_jitter"]])), 4)
            m["vib_acc_hover"] = round(float(np.nanmean(a[w, c["vib_acc"]])), 3)
            m["vib_gyro_hover"] = round(float(np.nanmean(a[w, c["vib_gyro"]])), 4)
        m["peak_alt"] = round(float(np.max(alt_air)), 2)
        # jolts: the largest pitch / roll rate in each segment of the flight (the nose lift's own 3 deg/s rotation is
        # judged by lift_* above); drift: the furthest the aircraft gets from the lift-off point while airborne
        t_end = float(ta[-1])
        segs = {"handover": (st == ST["handover"]) & ~air, "liftoff": air & (t < t_take + 1.0),
                "climb": air & (t >= t_take + 1.0) & (t < reach + 1.0), "hover": w if w.sum() > 20 else air & (t < 0),
                "descent": air & (t >= reach + max(hover_s - 1.0, 2.0)) & (t < t_end - 0.1),
                "touchdown": (t >= t_end - 0.1) & (t < t_end + 0.6), "lowering": (st == ST["lowering"]) & (t > t_end)}
        worst, worst_seg = 0.0, ""
        for sn, sm in segs.items():
            if sm.sum() < 3:
                continue
            jq = float(np.max(np.abs(q[sm]))); jp = float(np.max(np.abs(p[sm])))
            m[f"jolt_{sn}_q"] = round(jq, 1); m[f"jolt_{sn}_p"] = round(jp, 1)
            if max(jq, jp) > worst:
                worst, worst_seg = max(jq, jp), sn
        m["jolt_max"] = round(worst, 1); m["jolt_where"] = worst_seg
        i_off = int(np.argmax(air))
        m["drift_total"] = round(float(np.max(np.hypot(a[air, c["n"]] - a[i_off, c["n"]], a[air, c["e"]] - a[i_off, c["e"]]))), 2)
        # touchdown: the first sample after the flight with a foot on the ground
        t_land = float(ta[-1])
        after = np.where((t > t_land) & (feet >= 1))[0]
        if len(after):
            i = after[0]; j0 = max(i - 3, 0)
            m["td_vz"] = round(float(np.max(vd[j0:i + 1])), 2)
            m["td_vh"] = round(float(np.max(np.hypot(a[j0:i + 1, c["vn"]], a[j0:i + 1, c["ve"]]))), 2)
            m["td_feet_first"] = int(feet[i])
            # landing order: the nose must start down only once the feet are on the ground and still
            low = np.where((t > t_land) & (st == ST["lowering"]))[0]
            if len(low):
                k = low[0]
                m["lower_after_td_s"] = round(float(t[k] - t[i]), 2)
                m["lower_start_feet"] = int(feet[k])
                m["lower_start_vz"] = round(float(abs(vd[k])), 3)
                m["lower_start_alt_rate"] = round(float(np.max(np.abs(vd[max(k - 10, 0):k + 1]))), 3)
                lw = (t >= t[k]) & (st == ST["lowering"])
                rot(lw, 3.0, "lower")
                # the jolt when the front leg comes down: the pitch rate spike right after the 3rd foot touches
                three = np.where((t > t[k]) & (feet >= 3))[0]
                if len(three):
                    k3 = three[0]; win = slice(k3, min(k3 + int(0.4 / dt) + 1, len(t)))
                    m["front_touch_rate"] = round(float(np.max(np.abs(q[win]))), 2)
                # the nose must not fall on its own between the touchdown and the controlled lowering
                m["nose_drop_before_lower"] = round(float(pitch_s[i] - pitch_s[k]), 2)
                m["legs_first"] = bool(m["lower_start_feet"] >= 2 and m["lower_start_alt_rate"] < 0.1
                                       and m["nose_drop_before_lower"] < 2.0)
    return m


# ------------------------------------------------------------------------------------------------ runner
def run_job(job, inst, outdir, px4, scenario) -> dict:
    tag = f"{job['cand']}_n{job['thrust']:g}_s{job['seed']}"
    out = outdir / f"{tag}.json"; ts = outdir / f"{tag}_ts.json"
    cmd = [sys.executable, "-m", "airframe_designer", "run", "--px4-dir", str(px4), "--instance", str(inst),
           "--seed", str(job["seed"]), "--airframe", AIRFRAME, "--scenario", scenario,
           "--set", "design.flow_sensor.enabled=true", "--set", f"rotors[6:9].max_thrust={job['thrust']}",
           # the simulator's own Python nose-lowering (design.nose_lower) arms itself above 1 m and, after the
           # touchdown, takes every motor from the firmware: off, so the flight is the firmware's alone (8 Oct)
           "--set", "design.nose_lower.enabled=false",
           "--out", str(out), "--timeseries", str(ts), "--timeout", "300"]
    for k, v in job["params"].items():
        cmd += ["--param", f"{k}={v}"]
    for sv in job.get("sets", []):
        cmd += ["--set", sv]
    cmd += list(job.get("args", []))            # extra run options, e.g. "--accel-bias", "0,0.07,0"
    r = None
    for attempt in range(3):
        t0 = time.time()
        subprocess.run(cmd, cwd=ROOT, capture_output=True, text=True)
        try:
            r = json.load(open(out))
        except Exception:
            r = None
        bad = (r is None or r.get("status") in ("error", "max_wall_time") or not ts.exists()
               or any("never became ready" in f or "did not connect" in f for f in (r.get("failures") or [])))
        if not bad:
            break
        time.sleep(2)
    res = dict(cand=job["cand"], thrust=job["thrust"], seed=job["seed"], instance=inst, wall_s=round(time.time() - t0, 1))
    if r is None or not ts.exists():
        return dict(res, cls="ERROR")
    m = r.get("metrics", {})
    res.update(cls="PASS" if r.get("ok") else ("CRASH" if m.get("crashed") else
               ("NOGO" if (m.get("max_alt_m") or 0) < 0.5 else "FAIL")),
               failures=r.get("failures"), crash=m.get("crash_reason"), max_tilt=m.get("max_tilt_deg"),
               clipping=m.get("accel_clipping", 0) + m.get("gyro_clipping", 0), sim_s=r.get("sim_time"))
    res.update(score_run(json.load(open(ts))["timeseries"], job.get("hover_s", 10)))
    return res


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--spec", required=True); ap.add_argument("--out", required=True)
    ap.add_argument("--px4-dir", default=str(Path.home() / "PX4-nl"))
    ap.add_argument("--instances", default="auto")
    a = ap.parse_args()
    spec = json.load(open(a.spec)); px4 = Path(a.px4_dir); outdir = ROOT / a.out; outdir.mkdir(parents=True, exist_ok=True)
    known = known_params(px4); base = board_params(known); base.update(spec.get("base", {}))
    insts = free_instances() if a.instances == "auto" else [int(x) for x in a.instances.split(",")]
    scenario = spec.get("scenario", "scenarios/fw_auto_hop_winner.json")
    jobs = queue.Queue(); n = 0
    for name, cp in spec["candidates"].items():
        p = dict(base); p.update(cp)
        unknown = [k for k in p if k not in known and k not in BOARD_EXTRA]
        if unknown:
            sys.exit(f"{name}: not on this firmware: {unknown}")
        for th in spec.get("thrusts", [36]):
            for sd in spec.get("seeds", [0]):
                jobs.put(dict(cand=name, params=p, thrust=th, seed=sd, hover_s=p.get("NL_AUTO_HOV", spec.get("hover_s", 10)),
                              sets=spec.get("sets", []) + spec.get("cand_sets", {}).get(name, []),
                              args=spec.get("args", []) + spec.get("cand_args", {}).get(name, []))); n += 1
    print(f"{n} flights on instances {insts}", flush=True)
    results, lock, t0 = [], threading.Lock(), time.time()

    def worker(inst):
        while True:
            try:
                j = jobs.get_nowait()
            except queue.Empty:
                return
            r = run_job(j, inst, outdir, px4, scenario)
            with lock:
                results.append(r)
                print(f"[{len(results)}/{n}] {r['cand']:22s} {r['thrust']:4g}N s{r['seed']} {r['cls']:6s} "
                      f"JOLT {r.get('jolt_max')}@{r.get('jolt_where')} DRIFT {r.get('drift_total')} climb_q_rms {r.get('climb_q_rms')} hand_q_rms {r.get('handover_q_rms')} hov_alt_std {r.get('hover_alt_std')} drift {r.get('hover_drift')} pitch_osc {r.get('hover_pitch_osc_deg')} "
                      f"td_vz {r.get('td_vz')} legs_first {r.get('legs_first')} lower_rate_max {r.get('lower_rate_max')} "
                      f"front_touch {r.get('front_touch_rate')} drop {r.get('nose_drop_before_lower')} ({r['wall_s']} s)", flush=True)

    th = [threading.Thread(target=worker, args=(i,)) for i in insts]
    [x.start() for x in th]; [x.join() for x in th]
    wall = time.time() - t0
    json.dump({"spec": spec, "wall_s": round(wall, 1), "flights": n, "instances": insts, "results": results},
              open(outdir / "summary.json", "w"), indent=1)
    print(f"done: {n} flights in {wall:.0f} s wall ({60 * n / max(wall, 1):.1f} flights/min) -> {outdir}/summary.json")


if __name__ == "__main__":
    main()
