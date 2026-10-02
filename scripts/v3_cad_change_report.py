"""CAD change notice for SMALL_SCALE_V3 v34: what the designer changes in Fusion so the model matches the simulation.

Draws a side and a top view from the v34 STEP (grey silhouette) with redline markup (battery packs moved, H-FLOW
mount, CG before / after) and a floor plot of the jet impingement against the H-FLOW view, then writes one page.

  python scripts/v3_cad_change_report.py [out.html]      (default docs/v3_views/v34/V3_CAD_change_notice.html)
"""
import base64, io, json, math, os, sys
import numpy as np
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
from airframe_designer.geometry.airframe import Airframe
from airframe_designer.geometry.cad import import_step

OUT = sys.argv[1] if len(sys.argv) > 1 else "docs/v3_views/v34/V3_CAD_change_notice.html"
S = 700.0                                                     # px per metre in the drawings
ASIS = Airframe.load("airframes/atlas_v3_v34_asis.json").resolve_mass()
REC = Airframe.load("airframes/atlas_v3_v34_recommended.json").resolve_mass()
TRIM = REC.trim_hover_pitch()
imp = import_step(REC.cad.file, log=lambda s: None)
Rm = REC.cad.matrix(); O = np.asarray(REC.cad.origin)
BODIES = [(b["name"], np.asarray(b["vertices"], float).reshape(-1, 3) @ Rm.T + O, np.asarray(b["indices"], int).reshape(-1, 3)) for b in imp["bodies"]]


def fus_mm(p):
    """Structural FRD (m) -> Fusion design coordinates (mm): X = y, Y = 450 - z*1000, Z = -x."""
    p = np.asarray(p, float)
    return np.array([p[1] * 1000, 450 - p[2] * 1000, -p[0] * 1000])


VIEWS = {"side": (lambda p: p[..., 0], lambda p: -p[..., 2], (-0.62, 0.52), (-0.44, 0.19)),
         "top": (lambda p: p[..., 0], lambda p: -p[..., 1], (-0.62, 0.52), (-0.74, 0.74))}


def render(view):
    hf, vf, (h0, h1), (v0, v1) = VIEWS[view]
    w, h = (h1 - h0) * S, (v1 - v0) * S
    fig = plt.figure(figsize=(w / 100, h / 100), dpi=100); ax = fig.add_axes([0, 0, 1, 1]); ax.axis("off")
    for name, v, t in BODIES:
        if "battery_5200" in name or "H-FLOW" in name:
            continue                                            # drawn as markup
        P = np.stack([hf(v), vf(v)], axis=1)[t]
        ax.add_collection(PolyCollection(P, facecolor="#8793a1", edgecolor="none", alpha=0.26))
    ax.set_xlim(h0, h1); ax.set_ylim(v0, v1)
    buf = io.BytesIO(); fig.savefig(buf, format="png", transparent=True, dpi=100); plt.close(fig)
    return base64.b64encode(buf.getvalue()).decode(), w, h


def px(view, p):
    hf, vf, (h0, h1), (v0, v1) = VIEWS[view]
    p = np.asarray(p, float)
    return float((hf(p) - h0) * S), float((v1 - vf(p)) * S)


def batt_boxes(af):
    out = []
    for b in af.cad.bodies:
        if "battery_5200" in b.name and b.mass > 0:
            c = np.round(af.cad.body_pos(b), 4)
            if not any(np.allclose(c, o) for o in out):
                out.append(c)
    return out


PACK = np.array([0.16, 0.05, 0.05])                            # FRD extents of one 5200 mAh pack


def pack_rect(view, c, cls):
    hf, vf, *_ = VIEWS[view]
    half = PACK / 2
    corners = np.array([c + s * half for s in (np.array([-1, -1, -1]), np.array([1, 1, 1]))])
    (x1, y1), (x2, y2) = px(view, corners[0]), px(view, corners[1])
    return f'<rect class="{cls}" x="{min(x1, x2):.1f}" y="{min(y1, y2):.1f}" width="{abs(x2 - x1):.1f}" height="{abs(y2 - y1):.1f}" rx="2"/>'


def cg_mark(view, cg, cls, label, dy):
    x, y = px(view, cg)
    return (f'<g class="{cls}"><circle cx="{x:.1f}" cy="{y:.1f}" r="8"/><path d="M{x - 8:.1f},{y:.1f} A8,8 0 0,1 {x:.1f},{y - 8:.1f} L{x:.1f},{y:.1f} Z '
            f'M{x + 8:.1f},{y:.1f} A8,8 0 0,1 {x:.1f},{y + 8:.1f} L{x:.1f},{y:.1f} Z"/></g>'
            f'<text class="lab {cls}-t" x="{x + 12:.1f}" y="{y + dy:.1f}">{label}</text>')


def arrow(view, p, d, L, cls, marker):
    q = np.asarray(p) + L * np.asarray(d)
    (x1, y1), (x2, y2) = px(view, p), px(view, q)
    return f'<line class="{cls}" x1="{x1:.1f}" y1="{y1:.1f}" x2="{x2:.1f}" y2="{y2:.1f}" marker-end="url(#{marker})"/>'


th = math.radians(TRIM)
DOWN_H = np.array([-math.sin(th), 0.0, math.cos(th)])          # hover vertical, structural frame
HF_CAD = np.array([-0.015, 0.0, 0.062]); HF_NOSE = np.array([0.30, 0.0, 0.077])


def drawing(view):
    png, w, h = render(view)
    g = [f'<image href="data:image/png;base64,{png}" x="0" y="0" width="{w:.0f}" height="{h:.0f}"/>']
    old, new = batt_boxes(ASIS), batt_boxes(REC)
    for c in old:
        g.append(pack_rect(view, c, "old"))
    for c in new:
        if not any(np.allclose(c, o) for o in old):
            g.append(pack_rect(view, c, "new"))
            oc = min(old, key=lambda o: abs(o[1] - c[1]) + 10 * (o[0] > 0) + abs(o[2] - c[2]))
            (x1, y1), (x2, y2) = px(view, oc - [0.08, 0, 0]), px(view, c + [0.085, 0, 0])
            if view == "side" or abs(c[1]) < 0.05:
                g.append(f'<line class="move" x1="{x1:.1f}" y1="{y1:.1f}" x2="{x2:.1f}" y2="{y2:.1f}" marker-end="url(#m-red)"/>')
    # CG before / after
    g.append(cg_mark(view, ASIS.mass.cg, "cgold", "CG as drawn", -12 if view == "side" else -12))
    g.append(cg_mark(view, REC.mass.cg, "cgnew", "CG after change", 24 if view == "side" else 22))
    # H-FLOW
    if view == "side":
        g.append(arrow(view, HF_CAD, [0, 0, 1], 0.12, "hfold", "m-grey"))
        g.append(arrow(view, HF_CAD, DOWN_H, 0.14, "hfnew", "m-red"))
        x, y = px(view, HF_CAD + 0.14 * DOWN_H)
        g.append(f'<text class="lab red-t" x="{x - 150:.1f}" y="{y + 18:.1f}">H-FLOW lens, tilted {TRIM:.1f}° aft</text>')
        g.append(arrow(view, HF_NOSE, DOWN_H, 0.10, "hfalt", "m-red"))
        x, y = px(view, HF_NOSE + 0.10 * DOWN_H)
        g.append(f'<text class="lab alt-t" x="{x - 6:.1f}" y="{y + 18:.1f}">option B: nose</text>')
        # hover vertical reference
        a, b = REC.mass.cg - 0.20 * DOWN_H, REC.mass.cg + 0.30 * DOWN_H
        (x1, y1), (x2, y2) = px(view, a), px(view, b)
        g.append(f'<line class="vert" x1="{x1:.1f}" y1="{y1:.1f}" x2="{x2:.1f}" y2="{y2:.1f}"/>'
                 f'<text class="lab muted-t" x="{x1 + 6:.1f}" y="{y1 + 4:.1f}">vertical in hover</text>')
    else:
        for p, cls in ((HF_CAD, "hfdot"), (HF_NOSE, "hfdotalt")):
            x, y = px(view, p)
            g.append(f'<circle class="{cls}" cx="{x:.1f}" cy="{y:.1f}" r="6"/>')
        x, y = px(view, HF_NOSE)
        g.append(f'<text class="lab alt-t" x="{x + 10:.1f}" y="{y - 10:.1f}">H-FLOW option B</text>')
    # Fusion axes key
    if view == "side":
        o = px(view, [-0.40, 0, -0.08])
        g.append(f'<g class="axes"><line x1="{o[0]:.1f}" y1="{o[1]:.1f}" x2="{o[0] - 50:.1f}" y2="{o[1]:.1f}" marker-end="url(#m-grey)"/>'
                 f'<line x1="{o[0]:.1f}" y1="{o[1]:.1f}" x2="{o[0]:.1f}" y2="{o[1] - 50:.1f}" marker-end="url(#m-grey)"/>'
                 f'<text class="lab muted-t" x="{o[0] - 76:.1f}" y="{o[1] + 16:.1f}">Fusion +Z (aft)</text>'
                 f'<text class="lab muted-t" x="{o[0] + 6:.1f}" y="{o[1] - 40:.1f}">+Y (up)</text></g>')
    else:
        o = px(view, [-0.40, -0.66, 0])
        g.append(f'<g class="axes"><line x1="{o[0]:.1f}" y1="{o[1]:.1f}" x2="{o[0] - 50:.1f}" y2="{o[1]:.1f}" marker-end="url(#m-grey)"/>'
                 f'<line x1="{o[0]:.1f}" y1="{o[1]:.1f}" x2="{o[0]:.1f}" y2="{o[1] + 50:.1f}" marker-end="url(#m-grey)"/>'
                 f'<text class="lab muted-t" x="{o[0] - 76:.1f}" y="{o[1] - 8:.1f}">Fusion +Z (aft)</text>'
                 f'<text class="lab muted-t" x="{o[0] + 6:.1f}" y="{o[1] + 46:.1f}">+X</text></g>')
    return f'<svg viewBox="0 0 {w:.0f} {h:.0f}" role="img" aria-label="{view} view with the changes marked">{"".join(g)}</svg>'


def floor_plot():
    """Floor under the hover (hover frame, CG above the origin): jet impingement points and the H-FLOW view circles."""
    Ry = np.column_stack([np.array([math.cos(th), 0, math.sin(th)]), [0, 1, 0], DOWN_H]).T
    cg = np.asarray(REC.mass.cg)
    W, H, sc = 620, 360, 150.0
    ox, oy = 250, 180
    P = lambda xy: (ox + xy[0] * sc, oy + xy[1] * sc)
    g = [f'<rect class="floor" x="0" y="0" width="{W}" height="{H}"/>']
    for gx in np.arange(-1.5, 2.6, 0.5):
        x, _ = P([gx, 0]); g.append(f'<line class="grid" x1="{x:.1f}" y1="0" x2="{x:.1f}" y2="{H}"/>')
        g.append(f'<text class="tick" x="{x + 3:.1f}" y="{H - 6}">{gx:+.1f} m</text>')
    for gy in np.arange(-1.0, 1.01, 0.5):
        _, y = P([0, gy]); g.append(f'<line class="grid" x1="0" y1="{y:.1f}" x2="{W}" y2="{y:.1f}"/>')
    for Hh, cls in ((1.0, "h10"), (1.5, "h15")):
        def fp(p, d):
            ph = Ry @ (np.asarray(p) - cg); dh = Ry @ np.asarray(d); t = (Hh - ph[2]) / dh[2]; return (ph + t * dh)[:2]
        for p, name in ((HF_CAD, "belly"), (HF_NOSE, "nose")):
            c = fp(p, DOWN_H); lens_h = Hh - (Ry @ (p - cg))[2]; r = lens_h * math.tan(math.radians(21))
            x, y = P(c)
            g.append(f'<circle class="view {cls} {name}" cx="{x:.1f}" cy="{y:.1f}" r="{r * sc:.1f}"/>')
        for rr in REC.rotors:
            j = fp(rr.pos, -np.asarray(rr.axis) / np.linalg.norm(rr.axis)); x, y = P(j)
            g.append(f'<circle class="jet {cls}" cx="{x:.1f}" cy="{y:.1f}" r="5"/>')
            if cls == "h15":
                g.append(f'<text class="tick" x="{x + 7:.1f}" y="{y + 4:.1f}">{rr.name}</text>')
    x, y = P([0, 0]); g.append(f'<path class="cgx" d="M{x - 6},{y} L{x + 6},{y} M{x},{y - 6} L{x},{y + 6}"/><text class="tick" x="{x + 8}" y="{y - 8}">below CG</text>')
    g.append(f'<text class="tick" x="8" y="16">nose → right · floor seen from above · dashed: hover at 1.0 m · solid: 1.5 m</text>')
    return f'<svg viewBox="0 0 {W} {H}" role="img" aria-label="floor plot of jet impingement and H-FLOW view">{"".join(g)}</svg>'


def rows_from(path, variant):
    rs = [json.loads(l) for l in open(path)]
    return [r for r in rs if r["id"].startswith(variant + "|") and r.get("status") != "error"]


def stab_stats(path, variant):
    import statistics as st
    rs = rows_from(path, variant)
    H = [r["metrics"]["phases"]["hover"] for r in rs if "hover" in r["metrics"].get("phases", {})]
    ok = sum(1 for r in rs if r.get("ok") and not r["metrics"].get("crashed"))
    return {"n": len(rs), "ok": ok, "drift_med": st.median(h["pos_drift"] for h in H), "drift_max": max(h["pos_drift"] for h in H),
            "pitch": st.mean(h["pitch_err_rms_deg"] for h in H), "util": max(h["util_max"] for h in H),
            "td": max(r["metrics"].get("touchdown_speed") or 0 for r in rs)}


STAB = {k: stab_stats("results/fusion_v3/stab_all2.jsonl", k) for k in ("asis", "recommended", "group13", "all_rear_aft045", "frontpair_back")}
FLOW = {}
for r in map(json.loads, open("results/fusion_v3/final2.jsonl")):
    if r["id"].startswith("flow_"):
        FLOW.setdefault(r["id"].split("|")[0][5:], []).append(r["metrics"]["phases"]["pos_hold"])
import statistics as st
FLOWS = {k: (st.median(x["pos_drift"] for x in v), max(x["pos_drift"] for x in v), st.mean(x["alt_std"] for x in v), len(v)) for k, v in FLOW.items()}


def lp(path):
    import subprocess
    out = subprocess.check_output([sys.executable, "results/fusion_v3/authority_lp.py", path], text=True, stderr=subprocess.DEVNULL)
    return json.loads(out.split(" ", 1)[1])


AUTH = lp("airframes/atlas_v3_v34_recommended.json")
AUTH0 = lp("airframes/atlas_v3_v34_asis.json")

old_rear = [c for c in batt_boxes(ASIS) if c[0] < 0]
fmm = lambda p: "(" + ", ".join(f"{v:.0f}" for v in fus_mm(p)) + ")"
occ = {(-0.03,): "battery_5200mAh__M760g:3", (-0.085,): "battery_5200mAh__M760g:5", (0.03,): "battery_5200mAh__M760g(Mirror):2", (0.085,): "battery_5200mAh__M760g(Mirror):3"}
battery_rows = ""
for c in sorted(old_rear, key=lambda c: c[1]):
    name = occ[(round(float(c[1]), 3),)]
    new = c + np.array([-0.195, 0, 0.01 if abs(c[1]) > 0.06 else 0.0])
    d = fus_mm(new) - fus_mm(c)
    battery_rows += (f"<tr><td class='mono'>{name}</td><td class='num'>{fmm(c)}</td><td class='num red'>{fmm(new)}</td>"
                     f"<td class='num'>Z {d[2]:+.0f}" + (f", Y {d[1]:+.0f}" if abs(d[1]) > 0.5 else "") + "</td></tr>")

cg0, cg1 = np.asarray(ASIS.mass.cg), np.asarray(REC.mass.cg)


def static_loads(af):
    tp = af.trim_hover_pitch(); q = af.copy(); q.hover_pitch_deg = tp
    E = q.effectiveness(); u = np.linalg.pinv(E) @ np.array([0, 0, 0, 0, 0, -1.0]); up = -(E[5] @ u)
    tmax = np.array([r.effective_max_thrust() for r in q.active_rotors()]); util = u * (q.mass.mass * 9.80665 / up) / tmax
    return tp, util.max() * 100, util.min() * 100


TP0, U0MAX, U0MIN = static_loads(ASIS); TP1, U1MAX, U1MIN = static_loads(REC)
LEVER = np.linalg.norm(HF_CAD - cg1) * 1000
side_svg, top_svg, floor_svg = drawing("side"), drawing("top"), floor_plot()

CSS = """
/* Layout: a single reading column like a drawing's notes sheet, title block on top, drawings full width. */
:root { --paper:#f6f8f9; --sheet:#ffffff; --ink:#18212b; --muted:#5c6874; --rule:#d5dce2; --red:#c8262d; --alt:#b0620f; --grey:#7d8894; --ok:#23794a;
  --display:'Barlow Semi Condensed', 'Arial Narrow', Arial, sans-serif; --body:'Source Sans 3', 'Segoe UI', Helvetica, Arial, sans-serif; --mono:'JetBrains Mono', ui-monospace, Menlo, Consolas, monospace; }
@media (prefers-color-scheme: dark) { :root:not([data-theme="light"]) { --paper:#11161b; --sheet:#171d23; --ink:#e3e8ec; --muted:#9aa6b1; --rule:#2c353e; --red:#ff6b6b; --alt:#f0a352; --grey:#8f9aa5; --ok:#5fcf8e; color-scheme:dark } }
:root[data-theme="dark"] { --paper:#11161b; --sheet:#171d23; --ink:#e3e8ec; --muted:#9aa6b1; --rule:#2c353e; --red:#ff6b6b; --alt:#f0a352; --grey:#8f9aa5; --ok:#5fcf8e; color-scheme:dark }
body { background:var(--paper); color:var(--ink); font:16px/1.55 var(--body); }
main { max-width:1040px; margin:0 auto; padding-inline:16px; padding-block:24px 56px; display:grid; gap:28px; }
h1, h2, h3 { font-family:var(--display); font-weight:600; line-height:1.15; text-wrap:balance; margin:0; }
h1 { font-size:2.1rem; letter-spacing:.01em; } h2 { font-size:1.45rem; } h3 { font-size:1.1rem; }
p { margin:0; max-width:68ch; } section { display:grid; gap:14px; min-width:0; }
.titleblock { display:grid; grid-template-columns:repeat(auto-fit, minmax(170px, 1fr)); border:1.5px solid var(--ink); background:var(--sheet); }
.titleblock > div { padding:8px 12px; border-right:1px solid var(--rule); border-bottom:1px solid var(--rule); min-width:0; }
.titleblock .k { font:600 .7rem/1.2 var(--display); letter-spacing:.09em; text-transform:uppercase; color:var(--muted); }
.titleblock .v { font:500 .95rem/1.3 var(--mono); overflow-wrap:anywhere; }
.lede { font-size:1.08rem; }
.changes { display:grid; gap:10px; counter-reset:c; padding:0; margin:0; list-style:none; }
.changes li { display:grid; grid-template-columns:auto 1fr; gap:4px 14px; padding:12px 14px; background:var(--sheet); border:1px solid var(--rule); }
.changes .tag { font:700 .8rem/1.6 var(--mono); color:var(--red); border:1.5px solid var(--red); padding:0 6px; height:fit-content; }
.changes .tag.keep { color:var(--ok); border-color:var(--ok); }
.changes b { font-weight:600; } .changes .out { grid-column:2; color:var(--muted); font-size:.95rem; }
.wrap { overflow-x:auto; border:1px solid var(--rule); background:var(--sheet); }
table { border-collapse:collapse; width:100%; font-size:.93rem; font-variant-numeric:tabular-nums; }
th, td { text-align:left; padding:7px 10px; border-bottom:1px solid var(--rule); vertical-align:top; }
th { font:600 .72rem/1.3 var(--display); letter-spacing:.08em; text-transform:uppercase; color:var(--muted); }
td.num, th.num { text-align:right; white-space:nowrap; } .mono { font-family:var(--mono); font-size:.84rem; overflow-wrap:anywhere; }
.red { color:var(--red); font-weight:600; } .good { color:var(--ok); font-weight:600; }
ol.steps { margin:0; padding-left:1.3em; display:grid; gap:6px; max-width:72ch; }
figure { margin:0; display:grid; gap:6px; } figure svg { display:block; width:100%; height:auto; min-width:560px; }
figcaption { color:var(--muted); font-size:.88rem; }
.note { border-left:3px solid var(--alt); padding:8px 12px; background:var(--sheet); max-width:72ch; }
svg .old { fill:none; stroke:var(--grey); stroke-width:1.5; stroke-dasharray:5 4; }
svg .new { fill:var(--red); fill-opacity:.16; stroke:var(--red); stroke-width:2; }
svg .move { stroke:var(--red); stroke-width:1.6; stroke-dasharray:3 3; }
svg .cgold circle { fill:var(--sheet); stroke:var(--grey); stroke-width:1.6; } svg .cgold path { fill:var(--grey); }
svg .cgnew circle { fill:var(--sheet); stroke:var(--red); stroke-width:1.8; } svg .cgnew path { fill:var(--red); }
svg .hfold { stroke:var(--grey); stroke-width:2.4; } svg .hfnew { stroke:var(--red); stroke-width:2.6; } svg .hfalt { stroke:var(--alt); stroke-width:2.2; stroke-dasharray:6 4; }
svg .hfdot { fill:var(--red); } svg .hfdotalt { fill:none; stroke:var(--alt); stroke-width:2; }
svg .vert { stroke:var(--muted); stroke-width:1; stroke-dasharray:2 4; } svg .axes line { stroke:var(--grey); stroke-width:1.3; }
svg .lab { font:600 13px var(--body); paint-order:stroke; stroke:var(--sheet); stroke-width:4px; }
svg .cgold-t, svg .muted-t { fill:var(--muted); } svg .cgnew-t, svg .red-t { fill:var(--red); } svg .alt-t { fill:var(--alt); }
svg .mk-red { fill:var(--red); } svg .mk-grey { fill:var(--grey); }
svg .floor { fill:var(--sheet); } svg .grid { stroke:var(--rule); stroke-width:1; } svg .tick { fill:var(--muted); font:500 11px var(--mono); }
svg .view { fill:none; stroke-width:1.8; } svg .view.belly { stroke:var(--red); } svg .view.nose { stroke:var(--alt); }
svg .view.h10 { stroke-dasharray:5 4; } svg .jet { fill:var(--ink); fill-opacity:.55; } svg .jet.h10 { fill-opacity:.2; }
svg .cgx { stroke:var(--ink); stroke-width:1.5; }
.legend { display:flex; flex-wrap:wrap; gap:6px 18px; color:var(--muted); font-size:.86rem; }
.legend span::before { content:""; display:inline-block; width:18px; height:0; border-top:2px solid currentColor; margin-right:6px; vertical-align:middle; }
.legend .r { color:var(--red); } .legend .g { color:var(--grey); } .legend .a { color:var(--alt); }
"""

MARK = ('<svg width="0" height="0" style="position:absolute" aria-hidden="true"><defs>'
        '<marker id="m-red" viewBox="0 0 10 10" refX="8" refY="5" markerWidth="6" markerHeight="6" orient="auto-start-reverse"><path d="M0,0 L10,5 L0,10 z" class="mk-red"/></marker>'
        '<marker id="m-grey" viewBox="0 0 10 10" refX="8" refY="5" markerWidth="6" markerHeight="6" orient="auto-start-reverse"><path d="M0,0 L10,5 L0,10 z" class="mk-grey"/></marker>'
        '</defs></svg>')

a0, a1 = STAB["asis"], STAB["recommended"]
hf_cad, hf_nose = fus_mm(HF_CAD - [0, 0, 0.012]), fus_mm(HF_NOSE)
fl = FLOWS
html = f"""<title>V3 CAD Change Notice</title>
<link rel="preconnect" href="https://fonts.googleapis.com"><link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=Barlow+Semi+Condensed:wght@500;600;700&family=JetBrains+Mono:wght@400;500;700&family=Source+Sans+3:wght@400;600&display=swap">
<style>{CSS}</style>
{MARK}
<main>
<header style="display:grid;gap:14px">
  <h1>SMALL_SCALE_V3 · CAD change notice</h1>
  <div class="titleblock">
    <div><div class="k">Design</div><div class="v">ATLAS / V3 / SMALL_SCALE_V3</div></div>
    <div><div class="k">Base version</div><div class="v">v34 (read 29 Sep 2026)</div></div>
    <div><div class="k">Verified by</div><div class="v">PX4 v1.17 SITL, {154} flights</div></div>
    <div><div class="k">Units / axes</div><div class="v">mm, Fusion design axes</div></div>
  </div>
  <p class="lede">The model was read from Fusion, weighed from the part-name labels only, and flown in the simulator. As drawn it flies ({a0['ok']}/{a0['n']} hover flights, no crashes), but its CG sits {abs(cg1[0] - cg0[0]) * 1000:.0f} mm ahead of the point where the nine fans share the hover load evenly, and the H-FLOW looks {TRIM:.0f}° off vertical in hover. Three changes fix that; everything else stays as drawn.</p>
</header>

<section>
  <h2>Changes at a glance</h2>
  <ol class="changes">
    <li><span class="tag">C1</span><div><b>Move the rear battery block 195 mm aft</b> (Fusion +Z); lower its two outer packs 10 mm.</div>
      <div class="out">Hover drift {a0['drift_med']:.2f} → {a1['drift_med']:.2f} m (median), pitch error {a0['pitch']:.2f}° → {a1['pitch']:.2f}°, busiest fan {a0['util'] * 100:.0f} → {a1['util'] * 100:.0f} %.</div></li>
    <li><span class="tag">C2</span><div><b>Tilt the H-FLOW mount {TRIM:.1f}°</b> so the lens points straight down while hovering.</div>
      <div class="out">Position-hold drift on flow only {fl['CAD_flat'][0]:.2f} → {fl['CAD_tilted'][0]:.2f} m (median), height noise {fl['CAD_flat'][2] * 100:.1f} → {fl['CAD_tilted'][2] * 100:.1f} cm.</div></li>
    <li><span class="tag">C3</span><div><b>Fix the weight labels</b>: label 6 real parts, remove 9 unlabelled duplicates, reconcile the two jetfoils.</div>
      <div class="out">Unlabelled parts weigh nothing in the model; the real aircraft is heavier than the {REC.mass.mass:.2f} kg simulated.</div></li>
    <li><span class="tag keep">KEEP</span><div><b>Jetfoil exit angles and nose fans as drawn.</b></div>
      <div class="out">Measured on the B-rep: jet leaves at 69.0° / 59.3° / 49.4° (inner / middle / outer). A steeper inner foil (72°, 75°) flew no better or worse.</div></li>
  </ol>
</section>

<section>
  <h2>Axes used below</h2>
  <p>All positions are Fusion design coordinates in mm, as shown in the Inspect panel of the open design: X to the aircraft's right, Y up, Z aft (the nose is at negative Z). The simulator's frame is x forward, y right, z down with its origin on the centreline at Y = 450 mm; the notice converts everything for you.</p>
</section>

<section>
  <h2>C1 · Battery placement</h2>
  <p>The six 5200 mAh packs (760 g each, 4.56 kg together, 39 % of the labelled mass) set the CG. Only the rear block of four moves; the front pair stays where it is. A single 130 mm shift of all six packs balances the fans as well, but puts the front pair into the flight controller stack (V6X / RC08 parts), so it is not used.</p>
  <ol class="steps">
    <li>Select the four rear packs listed below (the ones at Z = +150 mm).</li>
    <li>Move them together +195 mm along Z.</li>
    <li>Move the two outer ones (X = ±85 mm) a further −10 mm along Y; at Y 435 they touch the All_mounts bodies near Z 285 to 342 mm.</li>
    <li>Rework the battery tray or straps for the new position; leave the front pair (Z = −150 mm) and the slide-in carrier unchanged.</li>
  </ol>
  <div class="wrap"><table>
    <thead><tr><th>Occurrence</th><th class="num">Centre now (X, Y, Z)</th><th class="num">Centre after</th><th class="num">Move</th></tr></thead>
    <tbody>{battery_rows}</tbody></table></div>
  <div class="wrap"><table>
    <thead><tr><th></th><th class="num">CG, Fusion (X, Y, Z) mm</th><th class="num">Hover pitch (trim)</th><th class="num">Fan load, busiest / idlest</th><th class="num">Hover drift, median / worst</th></tr></thead>
    <tbody>
      <tr><td>As drawn</td><td class="num">{fmm(cg0)}</td><td class="num">{TP0:.1f}°</td><td class="num">{U0MAX:.0f} % / {U0MIN:.0f} %</td><td class="num">{a0['drift_med']:.2f} / {a0['drift_max']:.2f} m</td></tr>
      <tr><td class="red">After C1</td><td class="num red">{fmm(cg1)}</td><td class="num red">{TRIM:.1f}°</td><td class="num red">{U1MAX:.0f} % / {U1MIN:.0f} %</td><td class="num red">{a1['drift_med']:.2f} / {a1['drift_max']:.2f} m</td></tr>
    </tbody></table></div>
  <figure><div class="wrap">{side_svg}</div>
    <figcaption>Side view, nose right. Grey dashed: packs as drawn; red: packs after C1; the red arrow at the belly is the H-FLOW lens after C2, the grey one its current direction.</figcaption></figure>
  <figure><div class="wrap">{top_svg}</div>
    <figcaption>Top view, nose right, aircraft's right side at the bottom.</figcaption></figure>
  <div class="legend"><span class="g">as drawn</span><span class="r">after the change</span><span class="a">H-FLOW option B</span></div>
  <p class="note">"Middle" here means the point over which the fans share the hover load evenly. The geometric middle of the airframe (half-way between nose and tail, Z = +47 mm) is where the CG already is; the thrust of the tilted foils acts along the jets leaving the trailing edges, far aft and low, so the balance point sits about 50 mm further aft.</p>
</section>

<section>
  <h2>C2 · H-FLOW optical flow sensor</h2>
  <p>The aircraft hovers {TRIM:.1f}° nose-up. PX4 treats the flow sensor as looking straight down in its hover frame, so a sensor mounted flat to the structure sees tilted flow and a slant range (6 % long). Tilting the mount is the part of the placement that matters most in the simulation.</p>
  <ol class="steps">
    <li><b>Option A (recommended): keep the current spot</b> under the avionics bay, lens centre at about {fmm(HF_CAD)} mm. Rotate the mount {TRIM:.1f}° about the X axis so the lens normal moves from −Y toward +Z (it looks down and slightly aft with the aircraft level on the bench). The view stays unobstructed; the lens is {LEVER:.0f} mm from the new CG.</li>
    <li><b>Option B: under the nose</b>, lens centre about {fmm(HF_NOSE)} mm, same {TRIM:.1f}° tilt. Use it if the aircraft will hover above about 1.1 m: at the belly spot the inner foil jets (M5 / M6) land inside the sensor's view from that height, at the nose all jets stay outside it up to about 1.8 m. The nose ESC (ESC__M200g:2) sits directly above this spot; mount on fixed structure beside it, not on the slide-in carrier.</li>
    <li>Keep the lens at least 80 mm above the floor when parked (simulated: 386 mm at the belly, 482 mm at the nose) and keep the 42° cone below the lens clear of legs, straps and cables.</li>
    <li>If the hover pitch changes in a later revision, the tilt follows it: tilt = hover pitch.</li>
  </ol>
  <figure><div class="wrap">{floor_svg}</div>
    <figcaption>Floor seen from above while hovering. Dots: where each fan's jet reaches the floor. Circles: what the H-FLOW sees (42° cone), red for the belly spot, orange for the nose; dashed at 1.0 m hover height, solid at 1.5 m.</figcaption></figure>
  <div class="wrap"><table>
    <thead><tr><th>Mount (on the C1 airframe)</th><th class="num">Position-hold drift, median / worst</th><th class="num">Height noise (std)</th><th class="num">Flights</th></tr></thead>
    <tbody>
      <tr><td>Current spot, flat (as drawn)</td><td class="num">{fl['CAD_flat'][0]:.2f} / {fl['CAD_flat'][1]:.2f} m</td><td class="num">{fl['CAD_flat'][2] * 100:.1f} cm</td><td class="num">{fl['CAD_flat'][3]}</td></tr>
      <tr><td class="red">Current spot, tilted {TRIM:.1f}° (A)</td><td class="num red">{fl['CAD_tilted'][0]:.2f} / {fl['CAD_tilted'][1]:.2f} m</td><td class="num red">{fl['CAD_tilted'][2] * 100:.1f} cm</td><td class="num">{fl['CAD_tilted'][3]}</td></tr>
      <tr><td>Nose, tilted {TRIM:.1f}° (B)</td><td class="num">{fl['nose_tilted'][0]:.2f} / {fl['nose_tilted'][1]:.2f} m</td><td class="num">{fl['nose_tilted'][2] * 100:.1f} cm</td><td class="num">{fl['nose_tilted'][3]}</td></tr>
    </tbody></table></div>
  <p>Flights: lift off, 20 s hands-off in Position mode on flow and range only (GPS off), a 3 s stick push, stop and hold. For reference, the same hold on GPS drifts 0.07 m.</p>
</section>

<section>
  <h2>C3 · Weight labels</h2>
  <p>The simulator weighs a part only when its name ends in a label such as <span class="mono">__M760g</span>. Parts without one weigh nothing, and exact unlabelled copies were ignored as duplicates.</p>
  <div class="wrap"><table>
    <thead><tr><th>Part(s)</th><th>Problem</th><th>Action</th></tr></thead>
    <tbody>
      <tr><td class="mono">All_mounts (14 bodies, 909 cm³)</td><td>No label: the largest unweighed item</td><td>Weigh and add __M&lt;g&gt;g</td></tr>
      <tr><td class="mono">Four_feet_assembled</td><td>No label</td><td>Add label</td></tr>
      <tr><td class="mono">Clamp_cap_1, Clamp_cap_3 and mirrors</td><td>No label</td><td>Add label</td></tr>
      <tr><td class="mono">Open CASCADE STEP translator 7.9 1.9(Mirror), 1.11(Mirror)</td><td>Right-hand copies of Jetfoil_mount_1 / _2 without label: 280 g missing on the right</td><td>Rename to Jetfoil_mount_1__M120g(Mirror), Jetfoil_mount_2__M160g(Mirror)</td></tr>
      <tr><td class="mono">JET_FOIL_V3__M920g / JET_FOIL_V3__M800g(Mirror)</td><td>Same geometry (1329 cm³), different labels: 120 g left-right difference</td><td>Weigh both, correct the labels</td></tr>
      <tr><td class="mono">16x1000mm_carbon_tube(Mirror), (Mirror) (1), 16x500mm_carbon_tube(Mirror), XFLY 80mm EDF(Mirror), battery_5200mAh(Mirror) ×3, PDB_XT90PW-F_CASE(Mirror)</td><td>Unlabelled duplicates sitting exactly on labelled parts; a mass-properties export counts them twice (9 batteries instead of 6)</td><td>Delete, or exclude from the design</td></tr>
      <tr><td class="mono">XFLY 80mm EDF_M330g(Mirror) ×4</td><td>Single underscore label</td><td>Rename to __M330g for consistency</td></tr>
    </tbody></table></div>
  <p>With the labels as they are, the lateral CG sits 4 mm to the left (Fusion −X); the missing right-hand mounts and the 920 / 800 g foils cause it. Fix the labels rather than moving batteries sideways.</p>
</section>

<section>
  <h2>Verification</h2>
  <div class="wrap"><table>
    <thead><tr><th>Battery layout</th><th class="num">Flights OK</th><th class="num">Hover drift, median / worst</th><th class="num">Pitch error</th><th class="num">Busiest fan</th><th class="num">Touchdown</th></tr></thead>
    <tbody>
      <tr><td>As drawn</td><td class="num">{a0['ok']}/{a0['n']}</td><td class="num">{a0['drift_med']:.2f} / {a0['drift_max']:.2f} m</td><td class="num">{a0['pitch']:.2f}°</td><td class="num">{a0['util'] * 100:.0f} %</td><td class="num">{a0['td']:.2f} m/s</td></tr>
      <tr><td class="red">C1: rear block +195 mm Z</td><td class="num">{a1['ok']}/{a1['n']}</td><td class="num red">{a1['drift_med']:.2f} / {a1['drift_max']:.2f} m</td><td class="num red">{a1['pitch']:.2f}°</td><td class="num">{a1['util'] * 100:.0f} %</td><td class="num">{a1['td']:.2f} m/s</td></tr>
      <tr><td>All six packs +130 mm Z (collides)</td><td class="num">{STAB['group13']['ok']}/{STAB['group13']['n']}</td><td class="num">{STAB['group13']['drift_med']:.2f} / {STAB['group13']['drift_max']:.2f} m</td><td class="num">{STAB['group13']['pitch']:.2f}°</td><td class="num">{STAB['group13']['util'] * 100:.0f} %</td><td class="num">{STAB['group13']['td']:.2f} m/s</td></tr>
      <tr><td>Front pair into the rear block, block +45 mm Z</td><td class="num">{STAB['all_rear_aft045']['ok']}/{STAB['all_rear_aft045']['n']}</td><td class="num">{STAB['all_rear_aft045']['drift_med']:.2f} / {STAB['all_rear_aft045']['drift_max']:.2f} m</td><td class="num">{STAB['all_rear_aft045']['pitch']:.2f}°</td><td class="num">{STAB['all_rear_aft045']['util'] * 100:.0f} %</td><td class="num">{STAB['all_rear_aft045']['td']:.2f} m/s</td></tr>
      <tr><td>Front pair into the rear block only</td><td class="num">{STAB['frontpair_back']['ok']}/{STAB['frontpair_back']['n']}</td><td class="num">{STAB['frontpair_back']['drift_med']:.2f} / {STAB['frontpair_back']['drift_max']:.2f} m</td><td class="num">{STAB['frontpair_back']['pitch']:.2f}°</td><td class="num">{STAB['frontpair_back']['util'] * 100:.0f} %</td><td class="num">{STAB['frontpair_back']['td']:.2f} m/s</td></tr>
    </tbody></table></div>
  <p>Each layout: Stabilized mode, lift off, 12 s hands-off hover at 2.5 m, land; flown at its trim pitch and ±0.5°, three seeds each.</p>
  <div class="wrap"><table>
    <thead><tr><th>Control authority at hover (C1 airframe)</th><th class="num">Roll</th><th class="num">Pitch</th><th class="num">Yaw</th></tr></thead>
    <tbody>
      <tr><td>Largest moment the fans can add while holding the aircraft up</td><td class="num">±{min(AUTH['roll+_Nm'], AUTH['roll-_Nm']):.1f} N·m</td><td class="num">±{min(AUTH['pitch+_Nm'], AUTH['pitch-_Nm']):.1f} N·m</td><td class="num">±{min(AUTH['yaw+_Nm'], AUTH['yaw-_Nm']):.1f} N·m</td></tr>
      <tr><td>As angular acceleration</td><td class="num">{min(AUTH['roll+_deg_s2'], AUTH['roll-_deg_s2']):.0f} °/s²</td><td class="num">{min(AUTH['pitch+_deg_s2'], AUTH['pitch-_deg_s2']):.0f} °/s²</td><td class="num">{min(AUTH['yaw+_deg_s2'], AUTH['yaw-_deg_s2']):.0f} °/s²</td></tr>
      <tr><td>Flown: half stick (6° commanded) / full yaw stick (60 °/s)</td><td class="num">reaches ±5.6°</td><td class="num">reaches ±5.5°</td><td class="num">38 °/s right, 33 °/s left</td></tr>
      <tr><td>4 m/s crosswind / headwind</td><td class="num" colspan="2">attitude held within 2.5°, fans at 62 %</td><td class="num">heading swings 28 to 31°</td></tr>
    </tbody></table></div>
  <p>Roll and pitch have ample margin. Yaw is the weak axis (about 6 times weaker than roll) because all nine fans spin the same way; the aircraft weathervanes in a crosswind. Counter-rotating fan pairs would fix it and are the next design question, not part of this notice.</p>
</section>

<section>
  <h2>Mass model</h2>
  <div class="wrap"><table>
    <thead><tr><th>Group (labelled parts)</th><th class="num">Count</th><th class="num">Mass</th></tr></thead>
    <tbody>
      <tr><td>XFLY 80 mm EDF</td><td class="num">9</td><td class="num">2.970 kg</td></tr>
      <tr><td>Battery 5200 mAh</td><td class="num">6</td><td class="num">4.560 kg</td></tr>
      <tr><td>JET_FOIL_V3 (920 + 800 g)</td><td class="num">2</td><td class="num">1.720 kg</td></tr>
      <tr><td>ESC 200 g · PDB 175 g</td><td class="num">3 · 3</td><td class="num">1.125 kg</td></tr>
      <tr><td>Carbon tubes (1000 / 500 / 250 mm)</td><td class="num">8</td><td class="num">0.410 kg</td></tr>
      <tr><td>Slide-in carrier · jetfoil mounts (left only)</td><td class="num">1 · 2</td><td class="num">0.680 kg</td></tr>
      <tr><td>Avionics bay 250 g + H-FLOW 15 g + 1600 mAh pack 100 g</td><td class="num">3</td><td class="num">0.365 kg</td></tr>
      <tr><td><b>Total</b></td><td class="num"></td><td class="num"><b>{REC.mass.mass:.3f} kg</b></td></tr>
    </tbody></table></div>
</section>

<section>
  <h2>Assumptions and limits</h2>
  <ul style="margin:0;padding-left:1.2em;display:grid;gap:4px;max-width:72ch">
    <li>Fan thrust 36 N each, reaction torque 0.002 N·m/N, all spinning the same way (ATLAS_09B figures, not measured on V3).</li>
    <li>The jet stays attached to the foil to its trailing edge; the thrust acts along the exit line there. Real jets separate early, which lowers the turning.</li>
    <li>Only labelled masses are included (C3). Heavier unlabelled parts, especially All_mounts, will move the CG; rerun after relabelling.</li>
    <li>The flow simulation has a perfect textured floor. It does not model dust or surface disturbance from the jets, lighting or lens vibration; the jet-in-view check covers dust only geometrically. H-FLOW field of view taken as 42°.</li>
    <li>Legs are the simulator's provisional ones, parked at the hover pitch.</li>
  </ul>
</section>
</main>"""
os.makedirs(os.path.dirname(OUT), exist_ok=True)
open(OUT, "w", encoding="utf-8").write(html)
print("wrote", OUT, len(html) // 1024, "kB")
