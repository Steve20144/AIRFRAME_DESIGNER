"""The nose fans holding the pitch for the whole flight (NL_FLY_HOLD), against the handover to PX4 (SITL, 2026-09-25).

On the aircraft (dashboard log log_20260925_195551, Altitude mode, NL_TGT 24, park 7.5) the nose held 24 deg until the
throttle went above the middle; the nose lift judged "PX4 took over" and faded, PX4 drove the nose fans at about the
rear fans' level (~1700 us) and the nose fell 24 -> 4 deg in 0.7 s. NL_FLY_HOLD 1 keeps the nose fans on the nose
lift's own pitch loop (NL_F_*) for the flight while PX4 flies the rear fans (thrust, roll, yaw); PX4's pitch rate
gains go to 0 so it does not pitch with the rear fans against them.

  python scripts/nose_hold_study.py            # needs the ~/PX4-nl SITL build with NL_FLY_HOLD

Flights: the 25 Sep sequence (lift to 24, hold, throttle to the middle, above it, climb, hover, descend, land,
throttle down, switch off -> nose lowered -> disarm) and the same with the kill switch in the hover. Conditions:
the committed model and the refit one (CG moved until it tips at 30 deg on the rear feet, as the aircraft does),
the nose fans' spin-down 1.0 / 2.0 s, the legs' rocking. Results go to results/nose_hold/<tag>/.
"""
import argparse
import json
import os
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))
from airframe_designer.batch.runner import run_many            # noqa: E402
from nose_lift_refit20 import fit_cg                            # noqa: E402

AIRFRAME = ROOT / "airframes" / "atlas_09b.json"
BOARD_FILE = ROOT / "results" / "board_params" / "params_20260925_201034.json"   # read over the radio 25 Sep 20:10
PARK, HOVER = 7.5, 24.0
ROCKING = [0.0, 0.35, 0.0]
NO_PX4_PITCH = {"MC_PITCHRATE_P": 0.0, "MC_PITCHRATE_I": 0.0, "MC_PITCHRATE_D": 0.0, "MC_PITCHRATE_FF": 0.0}
HOLD = {"NL_FLY_HOLD": 1, "NL_F_K_ANG": 2.0, "NL_F_RATE": 30.0, "NL_F_KQ": 0.015, "NL_F_KQI": 0.03, "NL_F_FF": 1.5,
        **NO_PX4_PITCH}


def board():
    p = json.loads(BOARD_FILE.read_text())
    p = p.get("params", p)
    keep = ("NL_", "MC_AIRMODE", "MPC_THR_HOVER", "MC_PITCH", "MC_ROLL", "MC_YAW", "CA_METHOD")
    return {k: v for k, v in p.items() if k.startswith(keep) and k != "NL_EN"}


def scenario(kill_in_hover=False):
    """The 25 Sep flight: throttle at zero for the lift, to the middle in the hold, then above it."""
    ph = [
        {"type": "wait_ready", "timeout": 45},
        {"type": "rc", "name": "arm", "channels": {"7": 1500, "9": 1000}, "mode": "altitude", "arm": True,
         "until": {"armed": True, "nl_state": "parked"}, "timeout": 30},
        {"type": "rc", "name": "parked", "duration": 2},
        {"type": "rc", "name": "lift", "channels": {"7": 1000}, "throttle": 0.0,
         "until": {"nl_state": ["holding", "handover", "nosehold"]}, "timeout": 40},
        {"type": "rc", "name": "hold", "duration": 3},
        {"type": "rc", "name": "mid", "throttle": 0.5, "throttle_from": 0.0, "ramp_s": 1.5, "duration": 4},
        {"type": "rc", "name": "liftoff", "throttle": 0.75, "throttle_from": 0.5, "ramp_s": 1.0, "duration": 4},
        {"type": "rc", "name": "hover", "throttle": 0.5, "duration": 10},
    ]
    if kill_in_hover:
        ph.append({"type": "rc", "name": "kill", "channels": {"9": 2000}, "until": {"armed": False}, "timeout": 5,
                   "expect": {"nl_abort": "kill switch"}})
        ph.append({"type": "rc", "name": "after_kill", "duration": 3})
    else:
        ph += [
            {"type": "rc", "name": "descend", "throttle": 0.3, "duration": 10},
            {"type": "rc", "name": "landed", "throttle": 0.0, "ramp_s": 1.0, "duration": 4},
            {"type": "rc", "name": "switch_off", "channels": {"7": 1500}, "until": {"armed": False}, "timeout": 40},
            {"type": "rc", "name": "after", "duration": 2},
        ]
    return {"name": "nose_hold_kill" if kill_in_hover else "nose_hold_flight",
            "description": "25 Sep flight: lift to 24 from 7.5, hold, throttle to the middle, above it, hover, "
                           + ("kill switch in the hover" if kill_in_hover else "descend, land, switch off"),
            "max_time": 160, "attitude": {"park_pitch_deg": PARK, "hover_pitch_deg": HOVER},
            "design": {"nose_lift": {"executor": "firmware", "target_pitch_deg": HOVER, "timeout_s": 60,
                                     "hold_timeout_s": 90}},
            "params": {"COM_RC_IN_MODE": 0, "RC_CHAN_CNT": 18, "RC_MAP_ROLL": 1, "RC_MAP_PITCH": 2,
                       "RC_MAP_THROTTLE": 3, "RC_MAP_YAW": 4, "RC3_TRIM": 1000, "RC_MAP_KILL_SW": 9,
                       "RC_KILLSWITCH_TH": 0.75, "COM_DISARM_LAND": 120.0},
            "abort": {"max_tilt_deg": 70, "max_alt": 8, "crash_speed": 2.5},
            "phases": ph}


def by_phase(ts):
    ts = ts.get("timeseries", ts)
    c = {k: i for i, k in enumerate(ts["columns"])}
    rows = np.array(ts["rows"], float)
    ph = np.array(ts["phase"])
    out = {}
    for name in dict.fromkeys(ph):
        m = ph == name
        x = rows[m]
        nose = np.degrees(x[:, c["pitch"]]) + HOVER
        out[name] = {"s": round(float(x[-1, c["t"]] - x[0, c["t"]]), 1),
                     "nose_min": round(float(nose.min()), 1), "nose_max": round(float(nose.max()), 1),
                     "nose_end": round(float(nose[-1]), 1), "alt_max": round(float(-x[:, c["d"]].min()), 2),
                     "airborne": round(float(x[:, c["airborne"]].mean()), 2),
                     "roll_max": round(float(np.degrees(np.abs(x[:, c["roll"]])).max()), 1),
                     "yaw_span": round(float(np.degrees(np.ptp(np.unwrap(x[:, c["yaw"]])))), 1),
                     "q_max": round(float(np.degrees(np.abs(x[:, c["q"]])).max()), 1)}
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--workers", type=int, default=4)
    ap.add_argument("--instances", default="5,6,7,8")
    ap.add_argument("--px4-dir", default=os.path.expanduser("~/PX4-nl"))
    ap.add_argument("--sets", default="board,hold", help="comma list of: board, hold, or name=JSON of overrides on hold")
    ap.add_argument("--models", default="model,tip30")
    ap.add_argument("--downs", default="1.0,2.0")
    ap.add_argument("--kill", action="store_true", help="also fly the kill-in-hover case")
    ap.add_argument("--analyse", action="store_true", help="only re-read the time series of flights already flown")
    ap.add_argument("--tag", default="run1")
    a = ap.parse_args()
    out = ROOT / "results" / "nose_hold" / a.tag
    out.mkdir(parents=True, exist_ok=True)

    base = board()
    sets = {}
    for s in a.sets.split(","):
        if s == "board":
            sets["board now"] = {}
        elif s == "hold":
            sets["hold"] = dict(HOLD)
        else:
            name, js = s.split("=", 1)
            sets[name] = {**HOLD, **json.loads(js)}
    cg30 = fit_cg(30.0, json.loads((ROOT / "scenarios" / "fw_nose_lift_balance20.json").read_text())) \
        if "tip30" in a.models else None

    tasks = []
    for label, fw in sets.items():
        for model in a.models.split(","):
            for down in (float(v) for v in a.downs.split(",")):
                for kill in ([False, True] if a.kill else [False]):
                    variables = {"rotors[8,9].tau": 0.15, "rotors[8,9].tau_down": down}
                    if model == "tip30":
                        variables = {"mass.from_items": False, "mass.cg": cg30, **variables}
                    tid = f"{label} | {model} | down {down:g}" + (" | kill" if kill else "")
                    slug = "".join(ch if ch.isalnum() else "_" for ch in tid)
                    opts = {"extra_params": {**base, **fw}, "timeseries_path": str(out / f"{slug}_ts.json"),
                            "seed": 1, "vibration": ROCKING}
                    tasks.append({"id": tid, "airframe": str(AIRFRAME), "scenario": scenario(kill),
                                  "variables": variables, "options": opts})

    if a.analyse:
        rs = [json.loads(Path(t["options"]["timeseries_path"]).read_text())["result"] for t in tasks]
    else:
        rs = run_many(tasks, workers=a.workers, instances=[int(i) for i in a.instances.split(",")], px4_dir=a.px4_dir)
    table = []
    for t, r in zip(tasks, rs):
        p = Path(t["options"]["timeseries_path"])
        ts = json.loads(p.read_text()) if p.exists() else None
        m = r.get("metrics") or {}
        table.append({"id": t["id"], "ok": r.get("ok"), "status": r.get("status"), "failures": r.get("failures"),
                      "crashed": m.get("crashed"), "events": [e["text"] for e in m.get("events", [])],
                      "by_phase": by_phase(ts) if ts else {}})
    (out / "summary.json").write_text(json.dumps({"sets": sets, "cg_tip30": cg30, "runs": table}, indent=1))
    for row in table:
        print(f"\n{row['id']}: {'ok' if row['ok'] else 'FAILED ' + str(row['status'])}"
              f"{' CRASHED' if row['crashed'] else ''} {'; '.join(row['failures'] or [])[:140]}")
        for name, d in row["by_phase"].items():
            if name in ("wait_ready", "arm", "parked"):
                continue
            print(f"   {name:10s} {d['s']:5.1f}s nose {d['nose_min']:5.1f}-{d['nose_max']:5.1f} end {d['nose_end']:5.1f}"
                  f"  alt {d['alt_max']:5.2f} air {d['airborne']:.2f}  roll {d['roll_max']:4.1f}  yaw span "
                  f"{d['yaw_span']:5.1f}  q {d['q_max']:5.1f}")


if __name__ == "__main__":
    main()
