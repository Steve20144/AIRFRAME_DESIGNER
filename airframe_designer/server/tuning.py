"""Helpers for the Tuning tab: a library of flights (headless, live and sweep trials) with their time series, the
default tuning objective, and compact per-run summaries the UI can list and chart.

Runs live in results/tuning/<id>.json (result) + <id>_ts.json (time series). Sweeps are ordinary studies whose
trials keep their time series under results/<study>/ts/.
"""
from __future__ import annotations

import json
import math
import time
from pathlib import Path

PROJECT_DIR = Path(__file__).resolve().parents[2]
RESULTS_DIR = PROJECT_DIR / "results"
TUNING_DIR = RESULTS_DIR / "tuning"

# What "good" means for a Stabilized takeoff / hover / landing: attitude tracking against PX4's own setpoint,
# quiet rates, small attitude excursions on the way up and down, a soft touchdown, no saturation.
TUNING_OBJECTIVE = ("phases.hover.pitch_err_rms_deg + phases.hover.roll_err_rms_deg + 0.05 * phases.hover.rates_rms_deg_s"
                    " + 0.2 * (phases.liftoff.pitch_max_deg + phases.liftoff.roll_max_deg)"
                    " + 0.2 * (phases.landing.pitch_max_deg + phases.landing.roll_max_deg)"
                    " + 2 * phases.nose_lower.touchdown_speed + 5 * metrics.saturation_fraction")
TUNING_CONSTRAINTS = ["ok", "not metrics.crashed", "phases.nose_lower.touchdown_speed < 0.8",
                      "abs(phases.hover.yaw_drift_deg) < 10", "phases.hover.pos_drift < 2"]

HOVER_NAMES = ("hover", "hold", "pilot_stabilized")
LIFTOFF_NAMES = ("liftoff", "takeoff")
LANDING_NAMES = ("landing", "land")


def _first(ph: dict, names) -> dict:
    for n in names:
        if n in ph:
            return ph[n] or {}
    return {}


def brief(result: dict) -> dict:
    """The dozen numbers a tuning table shows for one flight."""
    m = (result or {}).get("metrics") or {}
    ph = m.get("phases") or {}
    hover, lift, land, nl = _first(ph, HOVER_NAMES), _first(ph, LIFTOFF_NAMES), _first(ph, LANDING_NAMES), ph.get("nose_lower") or {}
    return {
        "ok": bool(result.get("ok")), "status": result.get("status"), "failures": (result.get("failures") or [])[:3],
        "hover_pitch_err": hover.get("pitch_err_rms_deg", hover.get("pitch_rms_deg")),
        "hover_roll_err": hover.get("roll_err_rms_deg", hover.get("roll_rms_deg")),
        "hover_rates": hover.get("rates_rms_deg_s"), "hover_yaw_drift": hover.get("yaw_drift_deg"),
        "hover_drift": hover.get("pos_drift"), "hover_alt": hover.get("alt_mean"),
        "liftoff_max": None if not lift else max(float(lift.get("pitch_max_deg") or 0), float(lift.get("roll_max_deg") or 0)),
        "landing_max": None if not land else max(float(land.get("pitch_max_deg") or 0), float(land.get("roll_max_deg") or 0)),
        "touchdown": nl.get("touchdown_speed", m.get("touchdown_speed")), "nose_down_s": nl.get("lower_duration"),
        "saturation": m.get("saturation_fraction"), "max_tilt": m.get("max_tilt_deg"),
        "sim_s": (result.get("timing") or {}).get("sim_s"), "wall_s": (result.get("timing") or {}).get("wall_s"),
    }


def decimate(ts: dict | None, max_points: int = 1500) -> dict | None:
    """Thin the time series to at most ``max_points`` rows and make it strict-JSON safe (NaN -> null: the setpoint
    columns are NaN until PX4's first ATTITUDE_TARGET, and FastAPI refuses to serialise NaN)."""
    if not ts or not ts.get("rows"):
        return ts
    rows = ts["rows"]
    k = max(1, math.ceil(len(rows) / max_points))
    clean = [[(None if isinstance(v, float) and (math.isnan(v) or math.isinf(v)) else v) for v in r] for r in rows[::k]]
    return {"columns": ts["columns"], "rows": clean, "phase": (ts.get("phase") or [])[::k]}


def new_id(prefix: str = "run") -> str:
    return f"{prefix}{int(time.time() * 1000) % 10_000_000_000}"


def save_run(run_id: str, result: dict, timeseries: dict | None, meta: dict | None = None) -> Path:
    TUNING_DIR.mkdir(parents=True, exist_ok=True)
    r = {k: v for k, v in (result or {}).items() if k not in ("airframe", "log")}
    r.update(meta or {})
    r["id"] = run_id
    r.setdefault("saved", time.time())
    if timeseries:
        (TUNING_DIR / f"{run_id}_ts.json").write_text(json.dumps({"timeseries": timeseries}))
        r["timeseries_path"] = str(TUNING_DIR / f"{run_id}_ts.json")
    (TUNING_DIR / f"{run_id}.json").write_text(json.dumps(r))
    return TUNING_DIR / f"{run_id}.json"


def list_runs() -> list[dict]:
    out = []
    if not TUNING_DIR.is_dir():
        return out
    for p in sorted(TUNING_DIR.glob("*.json"), key=lambda q: q.stat().st_mtime, reverse=True):
        if p.name.endswith("_ts.json"):
            continue
        try:
            r = json.loads(p.read_text())
        except Exception:
            continue
        out.append({"id": r.get("id", p.stem), "name": r.get("name") or r.get("id", p.stem), "kind": r.get("kind", "headless"),
                    "scenario": r.get("scenario"), "airframe_name": r.get("airframe_name"), "saved": r.get("saved"),
                    "params": r.get("params") or {}, "variables": r.get("variables") or {}, "physics": r.get("physics"),
                    "has_ts": (TUNING_DIR / f"{r.get('id', p.stem)}_ts.json").is_file(), "brief": brief(r)})
    return out


def load_run(run_id: str, max_points: int = 1500) -> dict | None:
    p = TUNING_DIR / f"{run_id}.json"
    if not p.is_file():
        return None
    r = json.loads(p.read_text())
    tsp = TUNING_DIR / f"{run_id}_ts.json"
    ts = None
    if tsp.is_file():
        try:
            ts = json.loads(tsp.read_text()).get("timeseries")
        except Exception:
            ts = None
    r["timeseries"] = decimate(ts, max_points)
    r["brief"] = brief(r)
    return r


def delete_run(run_id: str) -> bool:
    ok = False
    for p in (TUNING_DIR / f"{run_id}.json", TUNING_DIR / f"{run_id}_ts.json"):
        if p.is_file():
            p.unlink(); ok = True
    return ok


# ------------------------------------------------------------------ sweeps (studies with time series)
def list_sweeps() -> list[dict]:
    """Every study output directory under results/ that has trials, newest first, with a brief per trial."""
    out = []
    if not RESULTS_DIR.is_dir():
        return out
    for d in RESULTS_DIR.iterdir():
        tp = d / "trials.jsonl"
        if not d.is_dir() or not tp.is_file():
            continue
        try:
            spec = json.loads((d / "study.json").read_text()) if (d / "study.json").is_file() else {}
            trials = []
            for k, line in enumerate(tp.read_text().splitlines()):
                if not line.strip():
                    continue
                t = json.loads(line)
                metrics = t.get("metrics") or {}
                first = next(iter(metrics.values()), {}) if isinstance(metrics, dict) else {}
                trials.append({"k": k, "values": t.get("values"), "score": t.get("score"), "objective": t.get("objective"),
                               "feasible": t.get("feasible"), "violations": [v for v in (t.get("violations") or []) if not str(v).startswith("objective")],
                               "failures": (t.get("failures") or [])[:2], "generation": t.get("generation"),
                               "brief": brief({"ok": t.get("ok"), "status": t.get("status"), "failures": t.get("failures"), "metrics": first}),
                               "has_ts": bool(t.get("timeseries_path")) and Path(t["timeseries_path"]).is_file()})
            summary = json.loads((d / "summary.json").read_text()) if (d / "summary.json").is_file() else None
            out.append({"name": d.name, "mtime": tp.stat().st_mtime, "description": spec.get("description", ""),
                        "scenario": spec.get("scenario") or spec.get("scenarios"), "variables": spec.get("variables", []),
                        "objective": spec.get("objective"), "constraints": spec.get("constraints", []), "finished": summary is not None,
                        "trials": trials, "best_k": (min(range(len(trials)), key=lambda i: trials[i]["score"]) if trials else None)})
        except Exception:
            continue
    out.sort(key=lambda s: s["mtime"], reverse=True)
    return out


def load_trial(study: str, k: int, max_points: int = 1500) -> dict | None:
    d = RESULTS_DIR / study
    tp = d / "trials.jsonl"
    if not tp.is_file():
        return None
    lines = [l for l in tp.read_text().splitlines() if l.strip()]
    if not 0 <= k < len(lines):
        return None
    t = json.loads(lines[k])
    metrics = t.get("metrics") or {}
    first = next(iter(metrics.values()), {}) if isinstance(metrics, dict) else {}
    ts = None
    tsp = t.get("timeseries_path")
    if tsp and Path(tsp).is_file():
        try:
            ts = json.loads(Path(tsp).read_text()).get("timeseries")
        except Exception:
            ts = None
    r = {"id": f"{study}#{k}", "name": f"{study} trial {k + 1}", "kind": "sweep", "values": t.get("values"), "score": t.get("score"),
         "feasible": t.get("feasible"), "ok": t.get("ok"), "status": t.get("status"), "failures": t.get("failures"),
         "metrics": first, "timeseries": decimate(ts, max_points)}
    r["brief"] = brief(r)
    return r


def sweep_spec(airframe: dict, scenario: str, variables: list[dict], name: str | None = None, workers: int = 4,
               base_variables: dict | None = None, objective: str | None = None, constraints: list[str] | None = None,
               algorithm: dict | None = None) -> dict:
    """A study spec from the Tuning tab's form. ``variables``: [{"param": "MC_ROLLRATE_P", "min", "max", "levels"}]
    (or {"path": ..., "range": [...]})."""
    vs, levels = [], []
    for v in variables:
        path = v.get("path") or ("px4." + str(v.get("param", "")).strip())
        lo, hi = (v.get("range") or [v.get("min"), v.get("max")])
        vs.append({"path": path, "range": [float(lo), float(hi)]} | ({"type": "int"} if v.get("type") == "int" else {}))
        levels.append(max(1, int(v.get("levels", 3))))
    budget = 1
    for l in levels:
        budget *= l
    alg = algorithm or {"name": "grid", "levels": levels, "budget": budget, "batch": workers}
    scenario_path = scenario if scenario.endswith(".json") or "/" in scenario else f"scenarios/{scenario}.json"
    return {"name": name or f"tune_{int(time.time()) % 1000000}", "description": "Tuning tab sweep",
            "airframe": airframe, "base_variables": base_variables or {}, "scenario": scenario_path, "variables": vs,
            "objective": objective or TUNING_OBJECTIVE, "constraints": constraints or TUNING_CONSTRAINTS, "penalty": 100,
            "algorithm": alg, "workers": workers, "timeseries": True,
            "sim": {"rate": 250, "substeps": 2, "noise": True, "speed": 0}}
