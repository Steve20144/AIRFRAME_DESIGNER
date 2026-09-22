"""Self-contained HTML report for one or more headless runs (and, optionally, a study's trials).

    python scripts/report_runs.py --out results/stab/report.html results/stab/*.json
    python scripts/report_runs.py --out results/stab/report.html --study results/stab_tune results/stab/base_*.json

Every run that has a ``timeseries_path`` (written with ``run --timeseries``) gets a panel: a summary line, then charts
of height, roll and pitch against PX4's own setpoint, yaw, horizontal position, motor utilisation and the throttle
hand, with the scenario phases shaded. Runs without a time series get the summary line only. Pure Python, inline
SVG, no dependencies: the file opens anywhere.
"""
from __future__ import annotations

import argparse
import glob
import html
import json
import math
import os
from pathlib import Path

W, H, PAD_L, PAD_R, PAD_T, PAD_B = 880, 170, 52, 12, 14, 22
MAX_POINTS = 700
PHASE_COLOURS = ["#f4f1ea", "#e9f1f7", "#f1eef7", "#eef7ee", "#f7efe9", "#efeff2"]
LINE = {"a": "#1f4e79", "b": "#c0392b", "c": "#2e8b57", "d": "#8e6c1f", "sp": "#999999", "e": "#6a1b9a"}


def _load(path: str) -> dict:
    with open(path) as f:
        d = json.load(f)
    ts = None
    tsp = d.get("timeseries_path")
    cands = [tsp] if tsp else []
    cands.append(str(Path(path).with_name(Path(path).stem + "_ts.json")))
    for c in cands:
        if c and os.path.isfile(c):
            with open(c) as f:
                ts = json.load(f).get("timeseries")
            break
    if ts and ts.get("rows") and len(ts["rows"]) > MAX_POINTS:
        # keep the page light: a chart cannot show more than a few hundred points anyway
        k = math.ceil(len(ts["rows"]) / MAX_POINTS)
        ts = {"columns": ts["columns"], "rows": ts["rows"][::k], "phase": (ts.get("phase") or [])[::k]}
    d["_ts"] = ts
    d["_path"] = path
    return d


def _series(ts: dict, name: str) -> list[float]:
    i = ts["columns"].index(name)
    return [r[i] for r in ts["rows"]]


def _nice_ticks(lo: float, hi: float, n: int = 5) -> list[float]:
    if not math.isfinite(lo) or not math.isfinite(hi) or hi <= lo:
        return [lo]
    raw = (hi - lo) / n
    mag = 10 ** math.floor(math.log10(raw))
    step = min((s * mag for s in (1, 2, 2.5, 5, 10)), key=lambda s: abs(s - raw))
    t0 = math.ceil(lo / step) * step
    out = []
    v = t0
    while v <= hi + 1e-9:
        out.append(round(v, 10))
        v += step
    return out


def chart(title: str, t: list[float], curves: list[tuple[str, list[float], str, bool]], phases: list[str] | None,
          unit: str = "", ylim: tuple[float, float] | None = None, hline: float | None = None) -> str:
    """curves: (label, values, colour, dashed). NaN values break the line."""
    if not t:
        return ""
    t0, t1 = t[0], t[-1]
    vals = [v for _, ys, _, _ in curves for v in ys if v is not None and math.isfinite(v)]
    if not vals:
        return ""
    lo, hi = (min(vals), max(vals)) if ylim is None else ylim
    if hline is not None:
        lo, hi = min(lo, hline), max(hi, hline)
    if hi - lo < 1e-6:
        lo, hi = lo - 1, hi + 1
    m = 0.06 * (hi - lo)
    lo, hi = lo - m, hi + m
    iw, ih = W - PAD_L - PAD_R, H - PAD_T - PAD_B
    sx = lambda x: PAD_L + (x - t0) / max(t1 - t0, 1e-9) * iw
    sy = lambda y: PAD_T + (hi - y) / (hi - lo) * ih
    parts = [f'<svg viewBox="0 0 {W} {H}" width="100%" xmlns="http://www.w3.org/2000/svg" font-family="system-ui,sans-serif" font-size="11">']
    # phase bands
    if phases:
        i0 = 0
        k = 0
        for i in range(1, len(phases) + 1):
            if i == len(phases) or phases[i] != phases[i0]:
                x0, x1 = sx(t[i0]), sx(t[i - 1])
                parts.append(f'<rect x="{x0:.1f}" y="{PAD_T}" width="{max(x1 - x0, 0.5):.1f}" height="{ih}" fill="{PHASE_COLOURS[k % len(PHASE_COLOURS)]}"/>')
                if x1 - x0 > 30:
                    parts.append(f'<text x="{(x0 + x1) / 2:.1f}" y="{PAD_T + 11}" text-anchor="middle" fill="#777">{html.escape(phases[i0])}</text>')
                i0 = i
                k += 1
    # axes and ticks
    parts.append(f'<rect x="{PAD_L}" y="{PAD_T}" width="{iw}" height="{ih}" fill="none" stroke="#bbb"/>')
    for yv in _nice_ticks(lo + m, hi - m):
        y = sy(yv)
        parts.append(f'<line x1="{PAD_L}" x2="{W - PAD_R}" y1="{y:.1f}" y2="{y:.1f}" stroke="#e3e3e3"/>')
        parts.append(f'<text x="{PAD_L - 4}" y="{y + 3.5:.1f}" text-anchor="end" fill="#555">{yv:g}</text>')
    for xv in _nice_ticks(t0, t1, 8):
        x = sx(xv)
        parts.append(f'<text x="{x:.1f}" y="{H - 6}" text-anchor="middle" fill="#555">{xv:g}</text>')
    if hline is not None:
        y = sy(hline)
        parts.append(f'<line x1="{PAD_L}" x2="{W - PAD_R}" y1="{y:.1f}" y2="{y:.1f}" stroke="#888" stroke-dasharray="2,3"/>')
    # curves
    for label, ys, colour, dashed in curves:
        seg: list[str] = []
        pts: list[str] = []
        for x, y in zip(t, ys):
            if y is None or not math.isfinite(y):
                if pts:
                    seg.append(" ".join(pts)); pts = []
                continue
            pts.append(f"{sx(x):.1f},{sy(min(max(y, lo), hi)):.1f}")
        if pts:
            seg.append(" ".join(pts))
        dash = ' stroke-dasharray="4,3"' if dashed else ""
        for s in seg:
            parts.append(f'<polyline points="{s}" fill="none" stroke="{colour}" stroke-width="1.4"{dash}/>')
    # legend
    x = PAD_L + 6
    for label, _, colour, dashed in curves:
        dash = ' stroke-dasharray="4,3"' if dashed else ""
        parts.append(f'<line x1="{x}" x2="{x + 16}" y1="{H - 9}" y2="{H - 9}" stroke="{colour}" stroke-width="2"{dash}/>')
        parts.append(f'<text x="{x + 20}" y="{H - 5}" fill="#333">{html.escape(label)}</text>')
        x += 30 + 6.2 * len(label)
    parts.append(f'<text x="{W - PAD_R}" y="{PAD_T - 3}" text-anchor="end" fill="#333" font-weight="600">{html.escape(title)}{" [" + unit + "]" if unit else ""}</text>')
    parts.append("</svg>")
    return "".join(parts)


def _fmt(v, nd=2):
    if v is None:
        return "-"
    if isinstance(v, float):
        return f"{v:.{nd}f}"
    return str(v)


def run_panel(d: dict, hover_names=("hover", "hold", "pilot_stabilized")) -> str:
    m = d.get("metrics") or {}
    ph = m.get("phases") or {}
    hover = next((ph[n] for n in hover_names if n in ph), None)
    liftoff = ph.get("liftoff") or ph.get("takeoff")
    landing = ph.get("landing") or ph.get("land")
    nl = ph.get("nose_lower")
    status = ("ok" if d.get("ok") else "FAILED") + " " + str(d.get("status"))
    fails = "; ".join(d.get("failures") or [])
    name = Path(d["_path"]).stem
    vars_ = d.get("variables") or {}
    head = f"<h2 id='{html.escape(name)}'>{html.escape(name)} <span class='st {'ok' if d.get('ok') else 'bad'}'>{html.escape(status)}</span></h2>"
    sub = f"<div class='sub'>{html.escape(d.get('airframe_name', ''))} / {html.escape(str(d.get('scenario')))}"
    if vars_:
        sub += " / " + html.escape(", ".join(f"{k}={v}" for k, v in vars_.items()))
    sub += "</div>"
    if fails:
        sub += f"<div class='fail'>{html.escape(fails)}</div>"
    cells = []
    if hover:
        cells += [("hover pitch RMS", _fmt(hover.get("pitch_rms_deg")) + " deg"), ("hover roll RMS", _fmt(hover.get("roll_rms_deg")) + " deg"),
                  ("pitch err RMS", _fmt(hover.get("pitch_err_rms_deg")) + " deg"), ("roll err RMS", _fmt(hover.get("roll_err_rms_deg")) + " deg"),
                  ("yaw drift", _fmt(hover.get("yaw_drift_deg"), 1) + " deg"), ("rates RMS", _fmt(hover.get("rates_rms_deg_s"), 1) + " deg/s"),
                  ("drift", _fmt(hover.get("pos_drift")) + " m in " + _fmt(hover.get("duration"), 0) + " s"),
                  ("alt", _fmt(hover.get("alt_min")) + " to " + _fmt(hover.get("alt_max")) + " m"),
                  ("util max", _fmt(hover.get("util_max")))]
    if liftoff:
        cells += [("liftoff pitch max", _fmt(liftoff.get("pitch_max_deg")) + " deg"), ("liftoff roll max", _fmt(liftoff.get("roll_max_deg")) + " deg"),
                  ("liftoff rates RMS", _fmt(liftoff.get("rates_rms_deg_s"), 1) + " deg/s")]
    if landing:
        cells += [("landing pitch max", _fmt(landing.get("pitch_max_deg")) + " deg"), ("landing drift", _fmt(landing.get("pos_drift")) + " m"),
                  ("time to ground", _fmt(landing.get("time_to_ground"), 1) + " s")]
    if nl:
        cells += [("touchdown", _fmt(nl.get("touchdown_speed")) + " m/s"), ("nose down in", _fmt(nl.get("lower_duration"), 1) + " s"),
                  ("final pitch", _fmt(nl.get("final_pitch_deg"), 1) + " deg")]
    elif m.get("touchdown_speed") is not None:
        cells += [("touchdown", _fmt(m.get("touchdown_speed")) + " m/s")]
    cells += [("saturation", _fmt(m.get("saturation_fraction"), 3)), ("max tilt", _fmt(m.get("max_tilt_deg"), 1) + " deg")]
    table = "<div class='kv'>" + "".join(f"<div><span>{html.escape(k)}</span><b>{html.escape(v)}</b></div>" for k, v in cells) + "</div>"
    out = [head, sub, table]
    ts = d.get("_ts")
    if ts and ts.get("rows"):
        t = _series(ts, "t")
        phases = ts.get("phase")
        deg = lambda xs: [math.degrees(v) if v is not None and math.isfinite(v) else float("nan") for v in xs]
        alt = [-v for v in _series(ts, "d")]
        cols = ts["columns"]
        has_sp = "roll_sp" in cols
        out.append(chart("height", t, [("altitude", alt, LINE["a"], False)], phases, "m"))
        cur = [("roll", deg(_series(ts, "roll")), LINE["a"], False), ("pitch", deg(_series(ts, "pitch")), LINE["b"], False)]
        if has_sp:
            cur += [("roll sp", deg(_series(ts, "roll_sp")), LINE["sp"], True), ("pitch sp", deg(_series(ts, "pitch_sp")), "#c98a86", True)]
        out.append(chart("attitude, hover frame", t, cur, phases, "deg", hline=0.0))
        # the same, for the flown part only (from arming to the end of the landing) and clamped to a few degrees,
        # so hover tracking is readable next to the 20 degree nose lift
        flight = {"arm", "liftoff", "takeoff", "hover", "hold", "landing", "land", "cut", "headwind", "crosswind", "recover1", "recover2",
                  "pilot_stabilized", "pilot_position"}
        idx = [i for i, p in enumerate(phases or []) if p in flight]
        if len(idx) > 10:
            i0, i1 = idx[0], idx[-1] + 1
            sub = lambda ys: ys[i0:i1]
            out.append(chart("attitude, flight only (clamped to 5 deg)", t[i0:i1], [(l, sub(ys), c_, d_) for l, ys, c_, d_ in cur],
                             phases[i0:i1], "deg", ylim=(-5.0, 5.0), hline=0.0))
            out.append(chart("body rates, flight only (clamped to 30 deg/s)", t[i0:i1],
                             [("p", sub(deg(_series(ts, "p"))), LINE["a"], False), ("q", sub(deg(_series(ts, "q"))), LINE["b"], False),
                              ("r", sub(deg(_series(ts, "r"))), LINE["c"], False)], phases[i0:i1], "deg/s", ylim=(-30.0, 30.0), hline=0.0))
        yaw = deg(_series(ts, "yaw"))
        # unwrap for readability
        uy = []
        prev = None
        off = 0.0
        for v in yaw:
            if prev is not None and math.isfinite(v) and math.isfinite(prev):
                if v - prev > 180: off -= 360
                elif v - prev < -180: off += 360
            uy.append(v + off if math.isfinite(v) else v)
            prev = v
        ycur = [("yaw", uy, LINE["c"], False)]
        if has_sp:
            ysp = deg(_series(ts, "yaw_sp"))
            usp = []
            prev = None; off = 0.0
            for v in ysp:
                if prev is not None and math.isfinite(v) and math.isfinite(prev):
                    if v - prev > 180: off -= 360
                    elif v - prev < -180: off += 360
                usp.append(v + off if math.isfinite(v) else v)
                prev = v
            # align the setpoint's wrap offset with the actual at the first finite sample
            k0 = next((i for i, (a, b) in enumerate(zip(uy, usp)) if math.isfinite(a) and math.isfinite(b)), None)
            if k0 is not None:
                shift = round((uy[k0] - usp[k0]) / 360.0) * 360.0
                usp = [v + shift if math.isfinite(v) else v for v in usp]
            ycur.append(("yaw sp", usp, LINE["sp"], True))
        out.append(chart("yaw", t, ycur, phases, "deg"))
        out.append(chart("body rates", t, [("p", deg(_series(ts, "p")), LINE["a"], False), ("q", deg(_series(ts, "q")), LINE["b"], False),
                                            ("r", deg(_series(ts, "r")), LINE["c"], False)], phases, "deg/s", hline=0.0))
        out.append(chart("horizontal position", t, [("north", _series(ts, "n"), LINE["a"], False), ("east", _series(ts, "e"), LINE["b"], False)], phases, "m", hline=0.0))
        ucur = [("util max", _series(ts, "util_max"), LINE["d"], False), ("cmd mean", _series(ts, "cmd_mean"), LINE["e"], False)]
        if has_sp:
            ucur.append(("thrust sp", _series(ts, "thr_sp"), LINE["sp"], True))
        out.append(chart("motors", t, ucur, phases, "0..1", ylim=(0.0, 1.0)))
    else:
        out.append("<div class='sub'>no time series for this run (use --timeseries)</div>")
    return "\n".join(out)


def study_panel(study_dir: str) -> str:
    tp = Path(study_dir) / "trials.jsonl"
    if not tp.is_file():
        return f"<h2>study {html.escape(study_dir)}</h2><div class='fail'>no trials.jsonl</div>"
    trials = [json.loads(l) for l in tp.read_text().splitlines() if l.strip()]
    if not trials:
        return ""
    spec = {}
    sp = Path(study_dir) / "study.json"
    if sp.is_file():
        spec = json.loads(sp.read_text())
    names = list(trials[0]["values"].keys())
    trials.sort(key=lambda t: t["score"])
    out = [f"<h2>study {html.escape(Path(study_dir).name)}</h2>",
           f"<div class='sub'>objective: <code>{html.escape(str(spec.get('objective', '')))}</code> &nbsp; constraints: <code>{html.escape('; '.join(spec.get('constraints', [])))}</code> &nbsp; {len(trials)} trials, {sum(1 for t in trials if t.get('feasible'))} feasible</div>"]
    # score vs each variable, small scatter charts
    charts = []
    for n in names:
        xs = [t["values"][n] for t in trials]
        ys = [t["objective"] if t.get("feasible") and t.get("objective") is not None else None for t in trials]
        pts = [(x, y) for x, y in zip(xs, ys) if y is not None and math.isfinite(y)]
        bad = [x for x, y in zip(xs, ys) if y is None or not math.isfinite(y)]
        if not pts:
            continue
        lo, hi = min(p[0] for p in pts + [(b, 0) for b in bad]), max(p[0] for p in pts + [(b, 0) for b in bad])
        ylo, yhi = min(p[1] for p in pts), max(p[1] for p in pts)
        if hi - lo < 1e-9: hi = lo + 1
        if yhi - ylo < 1e-9: yhi = ylo + 1
        w, h = 280, 150
        sx = lambda x: 40 + (x - lo) / (hi - lo) * (w - 50)
        sy = lambda y: 10 + (yhi - y) / (yhi - ylo) * (h - 34)
        svg = [f'<svg viewBox="0 0 {w} {h}" width="{w}" xmlns="http://www.w3.org/2000/svg" font-family="system-ui,sans-serif" font-size="10">',
               f'<rect x="40" y="10" width="{w - 50}" height="{h - 34}" fill="#fafafa" stroke="#bbb"/>']
        for x in bad:
            svg.append(f'<line x1="{sx(x):.1f}" x2="{sx(x):.1f}" y1="10" y2="{h - 24}" stroke="#e57373" stroke-dasharray="2,2"/>')
        for x, y in pts:
            svg.append(f'<circle cx="{sx(x):.1f}" cy="{sy(y):.1f}" r="3" fill="{LINE["a"]}" fill-opacity="0.7"/>')
        bx, by = pts[0] if trials[0].get("feasible") else (None, None)
        if bx is not None:
            svg.append(f'<circle cx="{sx(bx):.1f}" cy="{sy(by):.1f}" r="5" fill="none" stroke="{LINE["b"]}" stroke-width="2"/>')
        svg.append(f'<text x="{w / 2}" y="{h - 12}" text-anchor="middle" fill="#333">{html.escape(n)}</text>')
        svg.append(f'<text x="38" y="14" text-anchor="end" fill="#555">{yhi:.3g}</text><text x="38" y="{h - 24}" text-anchor="end" fill="#555">{ylo:.3g}</text>')
        svg.append(f'<text x="{sx(lo):.1f}" y="{h - 1}" fill="#555">{lo:g}</text><text x="{sx(hi):.1f}" y="{h - 1}" text-anchor="end" fill="#555">{hi:g}</text>')
        svg.append("</svg>")
        charts.append("".join(svg))
    out.append("<div class='row'>" + "".join(charts) + "</div><div class='sub'>objective (feasible trials) against each variable; red ring = best, red dashes = infeasible trials</div>")
    # table of the best trials
    hdr = "".join(f"<th>{html.escape(n.replace('px4.', ''))}</th>" for n in names)
    rows = []
    for t in trials[:25]:
        mm = next(iter((t.get("metrics") or {}).values()), {}) if isinstance(t.get("metrics"), dict) else {}
        phs = (mm or {}).get("phases", {})
        hov = phs.get("hover") or phs.get("hold") or {}
        nl = phs.get("nose_lower") or {}
        vals = "".join(f"<td>{t['values'][n]:.4g}</td>" for n in names)
        rows.append(f"<tr class='{'ok' if t.get('feasible') else 'bad'}'><td>{t['score']:.3f}</td><td>{'yes' if t.get('feasible') else 'no'}</td>{vals}"
                    f"<td>{_fmt(hov.get('pitch_rms_deg'))}</td><td>{_fmt(hov.get('roll_rms_deg'))}</td><td>{_fmt(hov.get('yaw_drift_deg'), 1)}</td>"
                    f"<td>{_fmt(hov.get('pos_drift'))}</td><td>{_fmt(nl.get('touchdown_speed'))}</td><td>{html.escape('; '.join((t.get('violations') or []) + (t.get('failures') or []))[:90])}</td></tr>")
    out.append(f"<table><tr><th>score</th><th>feasible</th>{hdr}<th>hover pitch RMS</th><th>roll RMS</th><th>yaw drift</th><th>drift m</th><th>touchdown m/s</th><th>notes</th></tr>{''.join(rows)}</table>")
    return "\n".join(out)


CSS = """
body{font-family:system-ui,-apple-system,Segoe UI,sans-serif;margin:0 auto;max-width:960px;padding:16px;color:#222;background:#fff}
h1{font-size:20px;margin:0 0 4px} h2{font-size:16px;margin:26px 0 2px;border-top:1px solid #ddd;padding-top:12px}
.sub{color:#666;font-size:12px;margin:2px 0 6px} .fail{color:#b71c1c;font-size:12px;margin:2px 0 6px}
.st{font-size:12px;padding:1px 6px;border-radius:3px;margin-left:6px} .st.ok{background:#e8f5e9;color:#256029} .st.bad{background:#fdecea;color:#b71c1c}
.kv{display:flex;flex-wrap:wrap;gap:6px 14px;font-size:12px;margin:4px 0 8px} .kv div span{color:#666;margin-right:4px}
svg{display:block;margin:4px 0} .row{display:flex;flex-wrap:wrap;gap:8px}
table{border-collapse:collapse;font-size:11px;margin-top:8px} th,td{border:1px solid #ddd;padding:2px 5px;text-align:right} th{background:#f3f3f3}
tr.bad td{color:#9e9e9e} code{font-size:11px}
nav a{font-size:12px;margin-right:10px}
"""


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("runs", nargs="*", help="result JSON files (globs ok); *_ts.json siblings are picked up")
    ap.add_argument("--study", action="append", default=[], help="a study output directory (results/<name>) to summarise")
    ap.add_argument("--title", default="AIRFRAME_DESIGNER runs")
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    files: list[str] = []
    for r in a.runs:
        files += sorted(glob.glob(r)) if any(ch in r for ch in "*?[") else [r]
    files = [f for f in files if not f.endswith("_ts.json")]
    runs = [_load(f) for f in files]
    body = [f"<h1>{html.escape(a.title)}</h1>", "<nav>" + " ".join(f"<a href='#{html.escape(Path(r['_path']).stem)}'>{html.escape(Path(r['_path']).stem)}</a>" for r in runs) + "</nav>"]
    for sd in a.study:
        body.append(study_panel(sd))
    for r in runs:
        body.append(run_panel(r))
    doc = f"<!doctype html><html><head><meta charset='utf-8'><title>{html.escape(a.title)}</title><style>{CSS}</style></head><body>{''.join(body)}</body></html>"
    Path(a.out).parent.mkdir(parents=True, exist_ok=True)
    Path(a.out).write_text(doc, encoding="utf-8")
    print(f"{a.out}: {len(runs)} run(s), {len(a.study)} study(ies), {len(doc) // 1024} kB")


if __name__ == "__main__":
    main()
