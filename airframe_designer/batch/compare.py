"""Fly the same scenario on both physics engines (Python rigid body vs JSBSim) and compare: per-phase metrics
side by side with deltas, and RMS differences of the 50 Hz state histories (altitude, speed, roll/pitch, rates)
over the flight. Both runs use the same airframe, PX4 parameters, scenario and sensor noise seed."""
from __future__ import annotations

import json
import math
import tempfile
from pathlib import Path

import numpy as np

from .runner import run_many

KEYS = ("duration", "alt_mean", "alt_std", "pos_std_xy", "pos_drift", "speed_mean", "speed_max", "roll_rms_deg", "pitch_rms_deg",
        "pitch_mean_deg", "yaw_drift_deg", "rates_rms_deg_s", "tilt_max_deg", "util_max", "util_mean", "power_mean", "lift_share_mean",
        "time_to_alt", "time_to_pitch", "overshoot_deg", "vel_err_rms", "pos_err_final")


def _series_diff(a: dict, b: dict) -> dict:
    """RMS differences between two time series dicts ({"columns", "rows"}) on the common time span (nearest samples)."""
    if not a or not b or not a.get("rows") or not b.get("rows"):
        return {}
    ca = {c: i for i, c in enumerate(a["columns"])}
    A = np.array(a["rows"], float); B = np.array(b["rows"], float)
    ta, tb = A[:, 0], B[:, 0]
    t_end = min(ta[-1], tb[-1])
    t = np.arange(0.0, t_end, 0.1)
    out = {}
    for name, cols, conv in (("altitude_m", ("d",), lambda v: -v), ("speed_m_s", ("vn", "ve", "vd"), None),
                             ("roll_deg", ("roll",), np.degrees), ("pitch_deg", ("pitch",), np.degrees), ("yaw_deg", ("yaw",), np.degrees),
                             ("rates_deg_s", ("p", "q", "r"), None), ("thrust_N", ("thrust",), None)):
        def sample(M, tt):
            idx = [ca[c] for c in cols]
            vals = np.array([np.interp(t, tt, M[:, i]) for i in idx])
            if len(idx) > 1:
                v = np.sqrt((vals ** 2).sum(axis=0))
                return np.degrees(v) if name == "rates_deg_s" else v
            v = vals[0]
            return conv(v) if conv else v
        va, vb = sample(A, ta), sample(B, tb)
        d = va - vb
        out[name] = {"rms_diff": round(float(np.sqrt((d ** 2).mean())), 4), "max_diff": round(float(np.abs(d).max()), 4),
                     "python_mean": round(float(va.mean()), 4), "jsbsim_mean": round(float(vb.mean()), 4)}
    out["common_time_s"] = round(float(t_end), 2)
    return out


def compare_physics(airframe, scenario, variables=None, instances=None, out_path=None, **kw) -> dict:
    td = Path(tempfile.mkdtemp(prefix="afd_compare_"))
    tasks = [{"id": "python", "airframe": airframe, "scenario": scenario, "variables": variables or {},
              "options": {"physics": "python", "timeseries_path": str(td / "python.json")}},
             {"id": "jsbsim", "airframe": airframe, "scenario": scenario, "variables": variables or {},
              "options": {"physics": "jsbsim", "timeseries_path": str(td / "jsbsim.json")}}]
    kw.pop("physics", None)
    rp, rj = run_many(tasks, workers=2, instances=instances, **kw)
    ts = {}
    for name in ("python", "jsbsim"):
        p = td / f"{name}.json"
        ts[name] = json.loads(p.read_text())["timeseries"] if p.is_file() else {}
    series = _series_diff(ts["python"], ts["jsbsim"])
    lines = [f"physics comparison: {rp.get('airframe_name')} / {rp.get('scenario')}",
             f"  python : {'ok' if rp.get('ok') else 'FAILED'} {rp.get('status')} {rp.get('failures', [])[:1]}  rtf {rp.get('timing', {}).get('rtf')}",
             f"  jsbsim : {'ok' if rj.get('ok') else 'FAILED'} {rj.get('status')} {rj.get('failures', [])[:1]}  rtf {rj.get('timing', {}).get('rtf')}", ""]
    phases = {}
    for name in sorted(set(rp.get("metrics", {}).get("phases", {})) | set(rj.get("metrics", {}).get("phases", {}))):
        pp = rp["metrics"]["phases"].get(name, {}); pj = rj["metrics"]["phases"].get(name, {})
        rows = {}
        lines.append(f"  [{name}]  {'metric':22s} {'python':>10s} {'jsbsim':>10s} {'delta':>10s}")
        for k in KEYS:
            if k in pp or k in pj:
                a, b = pp.get(k), pj.get(k)
                d = (b - a) if isinstance(a, (int, float)) and isinstance(b, (int, float)) else None
                rows[k] = {"python": a, "jsbsim": b, "delta": d}
                fa = f"{a:10.3f}" if isinstance(a, (int, float)) else f"{str(a):>10s}"
                fb = f"{b:10.3f}" if isinstance(b, (int, float)) else f"{str(b):>10s}"
                fd = f"{d:+10.3f}" if d is not None else f"{'':>10s}"
                lines.append(f"           {k:22s} {fa} {fb} {fd}")
        phases[name] = rows
    if series:
        lines.append(""); lines.append(f"  state history over the common {series['common_time_s']} s (RMS / max difference, jsbsim - python):")
        for k, v in series.items():
            if isinstance(v, dict):
                lines.append(f"           {k:14s} rms {v['rms_diff']:8.3f}  max {v['max_diff']:8.3f}   (means {v['python_mean']:.3f} vs {v['jsbsim_mean']:.3f})")
    report = "\n".join(lines)
    result = {"ok": bool(rp.get("ok")) and bool(rj.get("ok")), "python": {k: v for k, v in rp.items() if k not in ("airframe", "log")},
              "jsbsim": {k: v for k, v in rj.items() if k not in ("airframe", "log")}, "phases": phases, "series": series, "report": report}
    if out_path:
        Path(out_path).write_text(json.dumps(result, indent=2))
    return result
