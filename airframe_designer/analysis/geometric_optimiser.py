"""Fast geometric optimiser: searches design variables against the static model (hover margin vs cruise power).
Variables are parameter paths (see geometry.paths), plus the legacy rotor-group tilt/cant form the UI uses.
No PX4 involved, thousands of evaluations per minute; use the batch optimiser for closed-loop objectives."""
from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Callable

import numpy as np

from ..geometry.airframe import Airframe
from ..geometry.frames import axis_to_tilt_cant
from ..geometry.paths import apply_variables
from .static import analyse


@dataclass
class Variable:
    path: str
    lo: float
    hi: float
    kind: str = ""       # legacy UI: "tilt" | "cant" | "hover_pitch"
    group: str = ""


def _variables_from_spec(af: Airframe, spec: dict) -> list[Variable]:
    vars_: list[Variable] = []
    for v in spec.get("variables", []) or []:
        vars_.append(Variable(v["path"], float(v["range"][0]), float(v["range"][1])))
    for gname, g in (spec.get("groups") or {}).items():
        idx = ",".join(str(i) for i in g["rotors"])
        if g.get("tilt"):
            vars_.append(Variable(f"rotors[{idx}].tilt_deg", *g["tilt"], kind="tilt", group=gname))
        if g.get("cant"):
            vars_.append(Variable(f"rotors[{idx}].cant_deg", *g["cant"], kind="cant", group=gname))
    if spec.get("hover_pitch"):
        vars_.append(Variable("hover_pitch_deg", *spec["hover_pitch"], kind="hover_pitch"))
    return vars_


def _score(m: dict, weight: float, tilt_limit: float) -> float:
    h, c = m["hover"], m["cruise"]
    s = weight * (h["max_util"] + h.get("waste", 0.0)) + (1 - weight) * (c.get("power_ratio") or 5.0)
    if not h["ok"]:
        s += 5.0 + 2.0 * len(h.get("negative", []))
    if not c.get("converged", False):
        s += 5.0
    s += 2.0 * len(c.get("saturated", [])) + 2.0 * len(c.get("negative", []))
    over = abs(c.get("px4_pitch_deg", 0.0)) - tilt_limit
    if over > 0:
        s += over / 10.0
    if any("stalled" in p for p in c.get("problems", [])):
        s += 1.0
    return float(s)


def nelder_mead(f: Callable[[np.ndarray], float], x0: np.ndarray, lo: np.ndarray, hi: np.ndarray, step: np.ndarray,
                max_eval: int) -> tuple[np.ndarray, float]:
    n = len(x0)
    pts = [np.clip(x0, lo, hi)]
    for i in range(n):
        p = x0.copy(); p[i] = np.clip(p[i] + step[i], lo[i], hi[i])
        if abs(p[i] - x0[i]) < 1e-9:
            p[i] = np.clip(x0[i] - step[i], lo[i], hi[i])
        pts.append(p)
    vals = [f(p) for p in pts]
    evals = n + 1
    while evals < max_eval:
        order = np.argsort(vals)
        pts = [pts[i] for i in order]; vals = [vals[i] for i in order]
        if vals[-1] - vals[0] < 1e-5 and max(np.abs(pts[-1] - pts[0])) < 0.05:
            break
        centroid = np.mean(pts[:-1], axis=0)
        xr = np.clip(centroid + (centroid - pts[-1]), lo, hi); fr = f(xr); evals += 1
        if fr < vals[0]:
            xe = np.clip(centroid + 2 * (centroid - pts[-1]), lo, hi); fe = f(xe); evals += 1
            pts[-1], vals[-1] = (xe, fe) if fe < fr else (xr, fr)
        elif fr < vals[-2]:
            pts[-1], vals[-1] = xr, fr
        else:
            xc = np.clip(centroid + 0.5 * (pts[-1] - centroid), lo, hi); fc = f(xc); evals += 1
            if fc < vals[-1]:
                pts[-1], vals[-1] = xc, fc
            else:
                for i in range(1, len(pts)):
                    pts[i] = np.clip(pts[0] + 0.5 * (pts[i] - pts[0]), lo, hi); vals[i] = f(pts[i]); evals += 1
    i = int(np.argmin(vals))
    return pts[i], vals[i]


def optimise(af: Airframe, spec: dict, progress: Callable[[float, str], None] | None = None) -> dict:
    """spec = {"variables": [{"path": "rotors[0,1].tilt_deg", "range": [0, 60]}, ...]   (or the legacy "groups"
    / "hover_pitch" form), "weight": 0..1 (1 = hover only), "speed_kmh": 50, "samples": 300, "refine": 6,
    "tilt_limit_deg": 45, "seed": 1}"""
    speed = float(spec.get("speed_kmh", af.design.get("cruise_speed_kmh", 50.0))) / 3.6
    weight = float(np.clip(spec.get("weight", 0.5), 0.0, 1.0))
    tilt_limit = float(spec.get("tilt_limit_deg", 45.0))
    variables = _variables_from_spec(af, spec)
    if not variables:
        return {"ok": False, "error": "nothing to optimise: give at least one variable"}
    lo = np.array([v.lo for v in variables]); hi = np.array([v.hi for v in variables])
    cache: dict[tuple, dict] = {}

    def candidate(x) -> Airframe:
        return apply_variables(af, {v.path: float(val) for v, val in zip(variables, x)})

    def evaluate(x: np.ndarray) -> dict:
        key = tuple(np.round(x, 2))
        if key in cache:
            return cache[key]
        time.sleep(0.0005)
        m = analyse(candidate(np.array(key)), speed, tilt_limit)
        m["score"] = _score(m, weight, tilt_limit)
        m["x"] = [float(v) for v in key]
        cache[key] = m
        return m

    from ..geometry.paths import get_path
    x_now = []
    for v in variables:
        cur = get_path(af, v.path)
        x_now.append(float(cur[0] if isinstance(cur, list) else cur))
    x_now = np.clip(np.array(x_now, float), lo, hi)
    n_samples = min(int(spec.get("samples", 300)), 2000)
    rng = np.random.default_rng(int(spec.get("seed", 1)))
    samples = [x_now] + [lo + (hi - lo) * rng.random(len(variables)) for _ in range(n_samples)]
    if len(variables) <= 2:
        grid = np.linspace(0, 1, 13)
        for a in grid:
            if len(variables) == 1:
                samples.append(lo + (hi - lo) * a)
            else:
                for b in grid:
                    samples.append(lo + (hi - lo) * np.array([a, b]))
    results = []
    for k, x in enumerate(samples):
        results.append(evaluate(x))
        if progress and k % 20 == 0:
            progress(0.7 * k / len(samples), f"sampling {k}/{len(samples)}")
    results.sort(key=lambda m: m["score"])
    n_refine = int(spec.get("refine", 6))
    step = (hi - lo) * 0.08
    for k, start in enumerate(results[:n_refine]):
        if progress:
            progress(0.7 + 0.3 * k / max(1, n_refine), f"refining {k + 1}/{n_refine}")
        nelder_mead(lambda x: evaluate(x)["score"], np.array(start["x"]), lo, hi, step, max_eval=90)
    allres = sorted(cache.values(), key=lambda m: m["score"])
    feasible = [m for m in allres if m["hover"]["ok"] and m["hover"]["max_util"] < 1.0
                and m["cruise"].get("converged") and not m["cruise"].get("saturated")]
    pareto = []
    for m in sorted(feasible, key=lambda m: (m["hover"]["max_util"], m["cruise"]["power_ratio"] or 9)):
        pr = m["cruise"]["power_ratio"] or 9
        if all(pr < (p["cruise"]["power_ratio"] or 9) for p in pareto):
            pareto.append(m)
    if len(pareto) > 8:
        idx = sorted({int(round(i)) for i in np.linspace(0, len(pareto) - 1, 8)})
        pareto = [pareto[i] for i in idx]
    pareto_keys = {tuple(m["x"]) for m in pareto}

    def pack(m):
        cand = candidate(np.array(m["x"]))
        return {"x": m["x"], "values": {v.path: xv for v, xv in zip(variables, m["x"])}, "score": m["score"],
                "pareto": tuple(m["x"]) in pareto_keys,
                "hover": {k: m["hover"][k] for k in ("ok", "max_util", "power", "authority", "problems", "waste", "total_thrust")},
                "cruise": {k: m["cruise"].get(k) for k in ("ok", "converged", "pitch_deg", "px4_pitch_deg", "max_util", "total_thrust",
                                                            "power", "power_ratio", "lift_share", "alpha_deg", "problems")},
                "hover_pitch_deg": cand.hover_pitch_deg, "axes": [r.axis for r in cand.rotors], "airframe": cand.to_dict()}

    seen: set[tuple] = set()
    top = []
    for m in allres:
        key = tuple(int(round(v / 2.0)) for v in m["x"])
        if key in seen:
            continue
        seen.add(key); top.append(pack(m))
        if len(top) >= 6:
            break
    for m in pareto:
        key = tuple(int(round(v / 2.0)) for v in m["x"])
        if key not in seen:
            seen.add(key); top.append(pack(m))
    if progress:
        progress(1.0, "done")
    return {"ok": True, "variables": [{"path": v.path, "kind": v.kind, "group": v.group, "lo": v.lo, "hi": v.hi} for v in variables],
            "evaluated": len(cache), "feasible": len(feasible), "results": top, "current": pack(evaluate(x_now))}


def default_groups(af: Airframe) -> dict:
    """Rotors that share a tilt and cant (to the degree) form one group, named A, B, C... front to back."""
    keys: dict[tuple, list[int]] = {}
    for i, r in enumerate(af.rotors):
        tilt, cant = axis_to_tilt_cant(r.axis, r.side)
        keys.setdefault((round(tilt), round(cant)), []).append(i)
    ordered = sorted(keys.values(), key=lambda idx: -max(af.rotors[i].pos[0] for i in idx))
    return {chr(65 + k): {"rotors": idx} for k, idx in enumerate(ordered)}
