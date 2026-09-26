"""The nose lift at 20 deg with the balance model re-fitted to the aircraft (SITL, 2026-09-23 night).

On the aircraft (dashboard logs log_20260923_205950 / 210017 / 210055, NL_TGT 24) the nose balances on the rear feet
with the fans off at about 30 deg (it sat at 28-30 for 5-8 s, then crept back or over), and breaks away from the
front leg at 0.63 of full thrust (M9/M10 ~80 %) at 7 deg. The committed model tips at 53-56 deg: the firmware's
balance feed-forward (NL_PIV, NL_A, NL_WEIGHT from that model) asks 2-3x the thrust the aircraft needs near the
target, the nose rose at 5-7 deg/s against NL_RATE 1.5, ran through the 25 deg ceiling on the slow fans and went
over. Here the simulator's CG is moved (up ~0.4 m, aft ~0.07 m) until the model tips at 30 deg with the same
breakaway thrust: whatever the physical cause, that thrust-vs-pitch curve is what the nose lift works against.

  python scripts/nose_lift_refit20.py            # needs the ~/PX4-nl SITL build (the board's firmware)

Conditions: the simulator tipping at 28 / 30 / 32 deg x the nose fans' spin-down 1.5 / 2.0 s (tau up 0.15, the
legs' rocking 0.35 rad/s); the firmware runs the board's NL_* values read over the radio, plus the change on trial.
Results go to results/nose_lift_smoothing/refit20/.
"""
import argparse
import json
import math
import os
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))
from airframe_designer.batch.runner import run_many            # noqa: E402
from airframe_designer.geometry.airframe import Airframe        # noqa: E402
from airframe_designer.geometry.paths import apply_variables    # noqa: E402
from airframe_designer.sim.nose_lift import firmware_params     # noqa: E402
from airframe_designer.sim.scenario import load_scenario        # noqa: E402
from nose_lift_smoothing import analyse                         # noqa: E402

AIRFRAME = ROOT / "airframes" / "atlas_09b.json"
PARK = 7.0
ROCKING = [0.0, 0.35, 0.0]
BREAKAWAY = 0.63            # thrust fraction at which the nose leaves the front leg at PARK (the aircraft, 21:00)

# the board's NL_* values, read over the radio 2026-09-23 ~21:30 (NL_TGT 24 with the C5 gains)
BOARD = {"NL_TGT": 24.0, "NL_KQ": 0.04, "NL_KQI": 0.02, "NL_K_ANG": 0.3, "NL_RATE": 1.5, "NL_LOW_KQ": 0.06,
         "NL_LOW_KQI": 0.03, "NL_CEIL": 1.0, "NL_OVERSHOOT": 12.0, "NL_PIV_X": -0.43565, "NL_PIV_Z": 0.32783,
         "NL_WEIGHT": 118.9056015, "NL_A0": 27.3289394, "NL_A1": 30.7583809, "NL_W0": 1.0921201, "NL_W1": 0.88651,
         "NL_HOV_PITCH": 24.0, "NL_HO_THR": 0.15, "NL_HO_TOUT": 8.0, "NL_TOL": 2.0, "NL_HOLD_S": 0.6, "NL_TOUT": 60.0,
         "NL_HOLD_TOUT": 60.0, "NL_MAX_CMD": 1.0, "NL_EXPO": 2.0, "NL_FADE_S": 2.0, "MPC_THR_HOVER": 0.32}


def refit_pivot(balance=30.0, breakaway=BREAKAWAY, park=PARK):
    """NL_PIV_X/Z that give the board's feed-forward (its own NL_A, NL_W, NL_WEIGHT) the aircraft's curve: zero at
    ``balance`` and ``breakaway`` at the park. An effective pivot, not a measured one."""
    a = BOARD["NL_A0"] * BOARD["NL_W0"] + BOARD["NL_A1"] * BOARD["NL_W1"]
    r = breakaway * a / math.sin(math.radians(balance - park)) / BOARD["NL_WEIGHT"]
    return {"NL_PIV_X": round(-r * math.sin(math.radians(balance)), 5), "NL_PIV_Z": round(r * math.cos(math.radians(balance)), 5)}


def scenario(name):
    if name == "balance24_park7":           # today's flights: NL_TGT 24 from a 7 deg park, armed in Altitude
        sc = scenario("balance24")
        sc["attitude"]["park_pitch_deg"] = PARK
        sc["phases"][1]["mode"] = "altitude"
        return sc
    return json.loads((ROOT / "scenarios" / f"fw_nose_lift_{name}.json").read_text())


def sim_figures(cg, sc):
    base = Airframe.load(AIRFRAME)
    af = apply_variables(base, {"mass.from_items": False, "mass.cg": cg}) if cg else base
    p = firmware_params(load_scenario(sc).apply_attitude(af))
    a = p["NL_A0"] * p["NL_W0"] + p["NL_A1"] * p["NL_W1"]
    bal = math.degrees(math.atan2(-p["NL_PIV_X"], p["NL_PIV_Z"]))
    ff = p["NL_WEIGHT"] * math.hypot(p["NL_PIV_X"], p["NL_PIV_Z"]) * math.sin(math.radians(bal - PARK)) / a
    return af.mass.cg, bal, ff


def fit_cg(balance, sc):
    """The simulator CG (x, z) at which the model tips at ``balance`` and breaks away at BREAKAWAY (Newton)."""
    cg0 = Airframe.load(AIRFRAME).mass.cg
    x, z = cg0[0], cg0[2]

    def err(x, z):
        _, b, f = sim_figures([x, cg0[1], z], sc)
        return np.array([b - balance, f - BREAKAWAY])

    for _ in range(20):
        e = err(x, z)
        if abs(e[0]) < 0.05 and abs(e[1]) < 0.003:
            break
        h = 0.005
        J = np.column_stack([(err(x + h, z) - e) / h, (err(x, z + h) - e) / h])
        x, z = np.array([x, z]) - np.linalg.solve(J, e)
    return [round(float(x), 5), round(float(cg0[1]), 5), round(float(z), 5)]


def nose_by_phase(ts):
    """Nose angle (min/max) and height per phase, for the takeoff: the analyse() phases are the ground ones."""
    ts = ts.get("timeseries", ts)
    c = {k: i for i, k in enumerate(ts["columns"])}
    rows = np.array(ts["rows"], float)
    ph = np.array(ts["phase"])
    out = {}
    for name in dict.fromkeys(ph):
        m = ph == name
        nose = np.degrees(rows[m, c["pitch"]]) + BOARD["NL_HOV_PITCH"]
        out[name] = {"nose_min": round(float(nose.min()), 1), "nose_max": round(float(nose.max()), 1),
                     "nose_end": round(float(nose[-1]), 1), "alt_max": round(float(-rows[m, c["d"]].min()), 2),
                     "airborne": round(float(rows[m, c["airborne"]].mean()), 2)}
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--workers", type=int, default=4)
    ap.add_argument("--instances", default="5,6,7,8")
    ap.add_argument("--px4-dir", default=os.path.expanduser("~/PX4-nl"))
    ap.add_argument("--seeds", default="1")
    ap.add_argument("--analyse", action="store_true", help="only re-read the time series of flights already flown")
    ap.add_argument("--tag", default="")
    a = ap.parse_args()
    out = ROOT / "results" / "nose_lift_smoothing" / ("refit20" + (f"_{a.tag}" if a.tag else ""))
    out.mkdir(parents=True, exist_ok=True)

    piv = refit_pivot()
    proposed = {"NL_TGT": 20.0, **piv, "NL_OVERSHOOT": 8.0}         # the change on trial
    sets = {"board now (TGT 24)": {}, "TGT 20 only": {"NL_TGT": 20.0}, "proposed": proposed}
    ref = scenario("balance20")
    cgs = {b: fit_cg(b, ref) for b in (28.0, 30.0, 32.0)}
    print("effective pivot", piv, "| simulator CGs", cgs)

    tasks = []

    def add(label, sname, fw, tip, down, seed):
        variables = {"rotors[8,9].tau": 0.15, "rotors[8,9].tau_down": down}
        if tip is not None:
            variables = {"mass.from_items": False, "mass.cg": cgs[tip], **variables}
        tid = f"{label} | {sname} | tips {tip or 'model'} | down {down:g}" + (f" | s{seed}" if a.seeds != "1" else "")
        slug = "".join(ch if ch.isalnum() else "_" for ch in tid)
        opts = {"extra_params": {**BOARD, **fw}, "timeseries_path": str(out / f"{slug}_ts.json"), "seed": seed,
                "vibration": ROCKING}
        tasks.append({"id": tid, "airframe": str(AIRFRAME), "scenario": scenario(sname), "variables": variables,
                      "options": opts})

    for seed in (int(v) for v in a.seeds.split(",")):
        # does the re-fitted simulator reproduce today's overshoot with what is on the board?
        for down in (1.5, 2.0):
            add("board now (TGT 24)", "balance24_park7", sets["board now (TGT 24)"], 30.0, down, seed)
        for tip in (28.0, 30.0, 32.0):
            for down in (1.5, 2.0):
                for sname in ("balance20", "midcancel20"):
                    add("proposed", sname, proposed, tip, down, seed)
            add("TGT 20 only", "balance20", sets["TGT 20 only"], tip, 2.0, seed)
        for tip in (30.0, None):          # the handover in Altitude mode; the committed CG for the flight itself
            add("proposed", "takeoff20", proposed, tip, 2.0, seed)

    if a.analyse:
        rs = [json.loads(Path(t["options"]["timeseries_path"]).read_text())["result"] for t in tasks]
    else:
        rs = run_many(tasks, workers=a.workers, instances=[int(i) for i in a.instances.split(",")], px4_dir=a.px4_dir)
    n = len(Airframe.load(AIRFRAME).active_rotors())
    table = []
    for t, r in zip(tasks, rs):
        p = Path(t["options"]["timeseries_path"])
        ts = json.loads(p.read_text()) if p.exists() else None
        table.append({"id": t["id"], "ok": r.get("ok"), "status": r.get("status"), "failures": r.get("failures"),
                      "crashed": (r.get("metrics") or {}).get("crashed"),
                      "events": [e["text"] for e in (r.get("metrics") or {}).get("events", [])][:12],
                      "phases": analyse(ts, n) if ts else {}, "by_phase": nose_by_phase(ts) if ts else {}})
    (out / "summary.json").write_text(json.dumps({"pivot": piv, "cgs": cgs, "runs": table}, indent=1))
    for row in table:
        print(f"\n{row['id']}: {'ok' if row['ok'] else 'FAILED ' + str(row['status'])}"
              f"{' CRASHED' if row['crashed'] else ''} {'; '.join(row['failures'] or [])[:120]}")
        for name, d in row["by_phase"].items():
            if name in ("wait_ready", "arm"):
                continue
            g = row["phases"].get(name, {})
            print(f"   {name:10s} nose {d['nose_min']:5.1f}-{d['nose_max']:5.1f} end {d['nose_end']:5.1f}  alt max "
                  f"{d['alt_max']:5.2f}  airborne {d['airborne']:.2f}"
                  + (f"  flips {g['flips_per_s']:.2f}/s  to 20: {g.get('t_to_20', '-')} s ({g.get('rise_deg_s', '-')} deg/s)" if g else ""))


if __name__ == "__main__":
    main()
