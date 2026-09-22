"""Summarise the park-pitch sweep of the firmware nose lift (results.jsonl + the *_ts.json time series)."""
import json
import math
from pathlib import Path

HERE = Path(__file__).resolve().parents[1] / "results" / "fw_nose_lift_sweep"
HOVER = 24.0


def load_ts(tid):
    p = HERE / f"{tid}_ts.json"
    if not p.is_file():
        return None
    ts = json.loads(p.read_text())["timeseries"]
    ix = {c: i for i, c in enumerate(ts["columns"])}
    rows = [dict(t=r[ix["t"]], pitch=math.degrees(r[ix["pitch"]]) + HOVER, alt=-r[ix["d"]], util=r[ix["util_max"]],
                 airborne=r[ix["airborne"]], n=r[ix["n"]], e=r[ix["e"]], phase=ph)
            for r, ph in zip(ts["rows"], ts["phase"])]
    return rows


def phase_rows(rows, *names):
    return [r for r in rows if r["phase"] in names]


res = {}
for line in (HERE / "results.jsonl").read_text().splitlines():
    r = json.loads(line)
    res[r["id"]] = r

parks = (4, 2, 0, -2, -4, -6, -8)
out = {"takeoff": [], "kill": [], "cancel": []}
for park in parks:
    # ---------------- takeoff
    tid = f"takeoff_park{park:+d}"
    r = res.get(tid, {})
    ph = (r.get("metrics") or {}).get("phases", {})
    rows = load_ts(tid) or []
    lift, hold = phase_rows(rows, "lift"), phase_rows(rows, "hold")
    after = phase_rows(rows, "liftoff", "climb")
    peak = max((x["pitch"] for x in lift + hold), default=float("nan"))
    handover_dev = max((abs(x["pitch"] - HOVER) for x in after), default=float("nan"))
    first_air = next((x for x in after if x["airborne"]), None)
    lp = ph.get("lift", {})
    out["takeoff"].append({
        "park": park, "ok": r.get("ok"), "failures": r.get("failures"),
        "lift_s": lp.get("time_to_until"), "lift_rate_max": lp.get("pitch_rate_max_deg_s"),
        "fan_peak": lp.get("util_max"), "slide_m": lp.get("pos_drift"),
        "overshoot": round(peak - HOVER, 2) if peak == peak else None,
        "hold_end": round(hold[-1]["pitch"], 2) if hold else None,
        "handover_s": ph.get("liftoff", {}).get("time_to_until"),
        "handover_dev": round(handover_dev, 2) if handover_dev == handover_dev else None,
        "liftoff_pitch": round(first_air["pitch"], 2) if first_air else None,
        "hover_pitch_rms": ph.get("hover", {}).get("pitch_err_rms_deg"), "hover_drift": ph.get("hover", {}).get("pos_drift"),
        "touchdown": (r.get("metrics") or {}).get("touchdown_speed"), "crashed": (r.get("metrics") or {}).get("crashed"),
    })
    # ---------------- kill
    tid = f"kill_park{park:+d}"
    r = res.get(tid, {})
    ph = (r.get("metrics") or {}).get("phases", {})
    k = {"park": park, "ok": r.get("ok"), "failures": r.get("failures")}
    for tag in ("rising", "held", "handover"):
        k[f"off_ms_{tag}"] = None if ph.get(f"kill_{tag}", {}).get("time_to_until") is None else round(1000 * ph[f"kill_{tag}"]["time_to_until"])
        k[f"disarm_s_{tag}"] = ph.get(f"disarm_{tag}", {}).get("time_to_until")
    out["kill"].append(k)
    # ---------------- cancel
    tid = f"cancel_park{park:+d}"
    r = res.get(tid, {})
    ph = (r.get("metrics") or {}).get("phases", {})
    rows = load_ts(tid) or []
    low = phase_rows(rows, "lowered")
    out["cancel"].append({
        "park": park, "ok": r.get("ok"), "failures": r.get("failures"),
        "lower_s": ph.get("lowered", {}).get("time_to_until"), "lower_rate_max": ph.get("lowered", {}).get("pitch_rate_max_deg_s"),
        "end_pitch": round(low[-1]["pitch"], 2) if low else None, "min_pitch": round(min(x["pitch"] for x in low), 2) if low else None,
        "radio_loss_to_disarm_s": ph.get("after_radio_loss", {}).get("time_to_until"),
    })
(HERE / "summary.json").write_text(json.dumps(out, indent=1))
for k, rows in out.items():
    print(f"== {k}")
    for row in rows:
        print("  ", {kk: v for kk, v in row.items() if kk != "failures" or v})
