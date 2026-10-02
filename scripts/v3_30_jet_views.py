"""Side, front and top view of ATLAS V3_30 (CAD render as the base) with switchable layers: the target foil jets of an
airframe (e.g. 57.5 / 85 / 85 deg above the fan axis), today's CFD-measured jet (39.6 deg per foil group), nose fans,
CG and the hover horizon. Made from scripts/v3_foil_views.py for the V3_30 CAD.

Writes one self-contained HTML page (each view has a Save PNG button; Save all PNGs saves the three) and the three
views as PNGs beside it in the light theme: through headless Edge on Windows (called from WSL), else through resvg
(pip install resvg-py; macOS and Linux).

  python scripts/v3_foil_views.py [out.html] [airframe.json] [label]
      defaults docs/v3_views/jets_57.5_85_85/v3_30_jet_angles.html, airframes/atlas_v3_30_jets_57.5_85_85_park-10.json, "Target jets"
"""
import base64, io, json, math, os, shutil, subprocess, sys
import numpy as np
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
from airframe_designer.geometry.airframe import Airframe

OUT = sys.argv[1] if len(sys.argv) > 1 else "docs/v3_views/jets_57.5_85_85/v3_30_jet_angles.html"
S = 800.0                                    # px per metre
R_CAD = np.array([[0, 0, -1], [1, 0, 0], [0, -1, 0]], float); ORIGIN = np.array([0.0, 0.0, 0.45])
frd = lambda p: R_CAD @ np.asarray(p, float) + ORIGIN

cad = json.load(open("airframes/cad/SMALL_SCALE_V3_30.step.bodies.json"))
best = Airframe.load(sys.argv[2] if len(sys.argv) > 2 else "airframes/atlas_v3_30_jets_57.5_85_85_park-10.json")
best.resolve_mass()
NAME_LABEL = sys.argv[3] if len(sys.argv) > 3 else "Target jets"
JET = {r.name: math.degrees(math.atan2(-r.axis[2], r.axis[0])) for r in best.rotors if r.name in ("M1", "M2", "M3", "M4", "M5", "M6")}
BEST_LABEL = " / ".join(f"{JET[m]:g}" for m in ("M5", "M3", "M1")) + "° in / mid / out"
FOIL = {m: 39.63 for m in JET}       # today's CFD (75 / 65 / 50 foils): each group's jet leaves 39.6 deg above the fan axis
NAME = {"M1": "outer", "M3": "middle", "M5": "inner"}

VIEWS = {  # name: (screen-right from an FRD point, screen-up from an FRD point, extents h0 h1, v0 v1, title)
    "side": (lambda p: p[0], lambda p: -p[2], (-1.15, 0.6), (-0.56, 0.2), "Side view · nose right · x forward →, up = −z"),
    "front": (lambda p: -p[1], lambda p: -p[2], (-0.74, 0.74), (-0.56, 0.17), "Front view · looking aft at the nose · drone's right (+y) on the left"),
    "top": (lambda p: p[0], lambda p: -p[1], (-0.75, 1.15), (-0.78, 0.78), "Top view · looking down · nose right, drone's right (+y) at the bottom · foil arrows drawn 2x long"),
}


def render(view):
    hf, vf, (h0, h1), (v0, v1), _ = VIEWS[view]
    w, h = (h1 - h0) * S, (v1 - v0) * S
    fig = plt.figure(figsize=(w / 100, h / 100), dpi=100); ax = fig.add_axes([0, 0, 1, 1]); ax.axis("off")
    for b in cad["bodies"]:
        v = np.asarray(b["vertices"]).reshape(-1, 3) @ R_CAD.T + ORIGIN
        t = np.asarray(b["indices"]).reshape(-1, 3)
        P = np.stack([hf(v.T), vf(v.T)], axis=1)[t]
        ax.add_collection(PolyCollection(P, facecolor="#9aa3ad", edgecolor="none", alpha=0.22))
    ax.set_xlim(h0, h1); ax.set_ylim(v0, v1)
    buf = io.BytesIO(); fig.savefig(buf, format="png", transparent=True); plt.close(fig)
    return base64.b64encode(buf.getvalue()).decode(), w, h


def px(view, p):
    hf, vf, (h0, h1), (v0, v1), _ = VIEWS[view]
    return (hf(p) - h0) * S, (v1 - vf(p)) * S


def arrow(view, p, d, length, cls, label=None, lab_off=(6, -6)):
    """An arrow from FRD point p along FRD direction d (projected), length in metres."""
    q = np.asarray(p, float) + length * np.asarray(d, float)
    (x1, y1), (x2, y2) = px(view, p), px(view, q)
    s = f'<line class="{cls}" x1="{x1:.1f}" y1="{y1:.1f}" x2="{x2:.1f}" y2="{y2:.1f}" marker-end="url(#arr-{cls.replace(' ', '-')})"/>'
    if label:
        s += f'<text class="{cls} lab" x="{x2 + lab_off[0]:.1f}" y="{y2 + lab_off[1]:.1f}">{label}</text>'
    return s


def layer_svg(view):
    L = {k: [] for k in ("mass", "horizon", "asbuilt", "best", "nose")}
    rot = {r.name: r for r in best.rotors}
    for m in ("M1", "M2", "M3", "M4", "M5", "M6"):
        r = rot[m]; p = np.asarray(r.pos)
        if view == "side" and r.pos[1] < 0:
            continue                                           # left and right overlap in the side view
        k_len = 2.0 if view == "top" else 1.0
        te = p.copy()
        for key, delta in (("asbuilt", FOIL[m]), ("best", JET[m])):
            dl = math.radians(delta)
            jet = np.array([-math.cos(dl), 0.0, math.sin(dl)])      # aft and down
            thr = -jet                                             # forward and up
            lab_j = f"{delta:.1f}°" if view == "side" else None
            L[key].append(arrow(view, p, thr, 0.12 * k_len, key, None))
            L[key].append(arrow(view, te, jet, 0.09 * k_len, key + " jet", lab_j, (6, 4) if key == "best" else (-66, 4)))
        if view == "top" and r.pos[1] > 0:
            x, y = px(view, p)
            tag = {"M2": "M1/M2 outer", "M4": "M3/M4 middle", "M6": "M5/M6 inner"}[m]
            L["best"].append(f'<text class="tag" x="{x + 70:.1f}" y="{y + 5:.1f}">{tag} · jet '
                             f'<tspan class="asbuilt-t">{FOIL[m]:.1f}°</tspan> → <tspan class="best-t">{JET[m]:.1f}°</tspan></text>')
        if view == "side":
            x, y = px(view, p)
            tag = {"M2": "M1/M2 outer", "M4": "M3/M4 middle", "M6": "M5/M6 inner"}[m]
            L["best"].append(f'<text class="tag" x="{x - 12:.1f}" y="{y + 22:.1f}" text-anchor="end">{tag} · jet '
                             f'<tspan class="asbuilt-t">{FOIL[m]:.1f}°</tspan> → <tspan class="best-t">{JET[m]:.1f}°</tspan></text>')
    for m in ("M7", "M8", "M9"):
        r = rot[m]; p = np.asarray(r.pos)
        if view == "side" and m == "M8":
            continue
        lab = None
        if view == "front":
            lab = {"M7": "M7 30° in", "M8": "M8 30° in", "M9": "M9 upright"}[m]
        elif view == "top":
            lab = {"M7": "M7 leans right (inward)", "M8": "M8 leans left (inward)", "M9": "M9 upright (points at you)"}[m]
        elif m in ("M7", "M9"):
            lab = {"M7": "M7/M8 (30° inward, side-on)", "M9": "M9"}[m]
        off = ({"M7": (10, 4), "M8": (-92, 4), "M9": (-34, -12)}[m] if view == "front" else
               {"M7": (60, -46), "M8": (60, 56), "M9": (14, 5)}[m] if view == "top" else ((6, -4) if m != "M7" else (-150, -10)))
        if view == "top" and m == "M9":
            x9, y9 = px(view, p)
            L["nose"].append(f'<circle class="nosedot" cx="{x9:.1f}" cy="{y9:.1f}" r="6"/><text class="nose lab" x="{x9 + 12:.1f}" y="{y9 + 5:.1f}">{lab}</text>')
            continue
        L["nose"].append(arrow(view, p, r.axis, 0.14, "nose", lab, off))
    cg = best.mass.cg
    x, y = px(view, cg)
    L["mass"].append(f'<g class="cg"><circle cx="{x:.1f}" cy="{y:.1f}" r="9"/><path d="M{x - 9:.1f},{y:.1f} A9,9 0 0,1 {x:.1f},{y - 9:.1f} L{x:.1f},{y:.1f} Z M{x + 9:.1f},{y:.1f} A9,9 0 0,1 {x:.1f},{y + 9:.1f} L{x:.1f},{y:.1f} Z"/></g>'
                     f'<text class="mass lab" x="{x + 14:.1f}" y="{y + 26:.1f}">CG ({cg[0]:+.3f}, {cg[1]:+.3f}, {cg[2]:+.3f}) m</text>')
    for it in []:
        bx, by = px(view, it.pos)
        wdt = (0.08 if view == "front" else 0.16) * S; hgt = (0.08 if view == "top" else 0.06) * S
        L["mass"].append(f'<rect class="batt" x="{bx - wdt / 2:.1f}" y="{by - hgt / 2:.1f}" width="{wdt:.1f}" height="{hgt:.1f}" rx="3"/>')
    if view == "side":
        # the world horizontal while hovering nose-up: in the body frame it slopes down toward the nose
        t = math.radians(best.hover_pitch_deg)
        d = np.array([math.cos(t), 0.0, math.sin(t)])
        a, b = np.asarray(cg) - 0.6 * d, np.asarray(cg) + 0.55 * d
        (x1, y1), (x2, y2) = px(view, a), px(view, b)
        L["horizon"].append(f'<line class="horizon" x1="{x1:.1f}" y1="{y1:.1f}" x2="{x2:.1f}" y2="{y2:.1f}"/>'
                            f'<text class="horizon lab" x="{x2 - 250:.1f}" y="{y2 + 18:.1f}">world horizontal in hover (nose-up {best.hover_pitch_deg:.1f}°)</text>')
    # body axes
    o = px(view, [0, 0, 0])
    ax = []
    if view in ("side", "top"):
        ax.append(arrow(view, [0, 0, 0], [1, 0, 0], 0.08, "axis", "+x fwd", (4, 4)))
    if view in ("front", "top"):
        ax.append(arrow(view, [0, 0, 0], [0, 1, 0], 0.08, "axis", "+y right", (-60, -6) if view == "front" else (6, 12)))
    if view != "top":
        ax.append(arrow(view, [0, 0, 0], [0, 0, 1], 0.08, "axis", "+z down", (4, 12)))
    return L, "".join(ax)


PARTS = {}


def parts(view):
    """CAD image and overlay layers of one view (rendered once, used by the page and the PNG export)."""
    if view not in PARTS:
        png, w, h = render(view)
        L, axes = layer_svg(view)
        PARTS[view] = (png, w, h, L, axes)
    return PARTS[view]


def svg_body(view):
    png, w, h, L, axes = parts(view)
    groups = "".join(f'<g class="layer" data-layer="{k}">{"".join(v)}</g>' for k, v in L.items())
    return (f'<image href="data:image/png;base64,{png}" x="0" y="0" width="{w:.0f}" height="{h:.0f}" class="layer" data-layer="cad"/>'
            f'<g class="layer" data-layer="axes">{axes}</g>{groups}')


def view_html(view):
    _, w, h, _, _ = parts(view)
    title = VIEWS[view][4]
    return (f'<figure data-view="{view}"><figcaption><span>{title}</span><button class="save" data-save="{view}" '
            f'title="save this view as a PNG, with the layers as shown">Save PNG</button></figcaption>\n'
            f'<svg viewBox="0 0 {w:.0f} {h:.0f}" role="img" aria-label="{title}">{svg_body(view)}</svg></figure>')


LIGHT = {"bg": "#fbfbfa", "fg": "#1d1f22", "muted": "#666c73", "card": "#ffffff", "line": "#e3e5e8", "asbuilt": "#d9731a", "best": "#1f6fd1",
         "nose": "#8b4fc4", "mass": "#1d1f22", "batt": "#2e9e5b", "horizon": "#7a8290", "axis": "#666c73"}
DARK = {"bg": "#16181b", "fg": "#e8eaed", "muted": "#a0a6ad", "card": "#1f2226", "line": "#33373c", "asbuilt": "#f0923f", "best": "#5aa2ff",
        "nose": "#b98af0", "mass": "#e8eaed", "batt": "#4cc47f", "horizon": "#9aa3ad", "axis": "#a0a6ad"}


def VARS(d):
    return "; ".join(f"--{k}:{v}" for k, v in d.items())


# the drawing's own rules: shared by the page and every exported SVG
SVG_CSS = """line { stroke-width:3; stroke-linecap:round; }
line.asbuilt, line.asbuilt.jet { stroke:var(--asbuilt); stroke-dasharray:9 6; }
line.best, line.best.jet { stroke:var(--best); }
line.jet { stroke-width:2.2; }
line.nose { stroke:var(--nose); } line.axis { stroke:var(--axis); stroke-width:1.6; }
line.horizon { stroke:var(--horizon); stroke-width:1.5; stroke-dasharray:4 5; }
text { font:600 15px system-ui, "Segoe UI", sans-serif; paint-order:stroke; stroke:var(--card); stroke-width:4px; }
text.asbuilt { fill:var(--asbuilt); } text.best { fill:var(--best); } text.nose { fill:var(--nose); }
text.mass { fill:var(--mass); font-weight:500; } text.horizon { fill:var(--horizon); font-weight:500; font-size:13px; }
text.axis { fill:var(--axis); font-weight:500; font-size:13px; } text.tag { fill:var(--fg); font-weight:500; font-size:14px; }
tspan.asbuilt-t { fill:var(--asbuilt); font-weight:700; } tspan.best-t { fill:var(--best); font-weight:700; }
.nosedot { fill:var(--nose); }
.cg circle { fill:var(--card); stroke:var(--mass); stroke-width:2; } .cg path { fill:var(--mass); }
.batt { fill:var(--batt); fill-opacity:.35; stroke:var(--batt); stroke-width:1.5; }
.hidden { display:none; }
.marker-asbuilt { fill:var(--asbuilt); } .marker-best { fill:var(--best); } .marker-nose { fill:var(--nose); } .marker-axis { fill:var(--axis); }
text.title { font-size:15px; font-weight:600; fill:var(--fg); stroke:none; }
text.legend { font-size:12px; font-weight:500; fill:var(--muted); stroke:none; }"""
MARKERS = "".join(f'<marker id="arr-{c.replace(" ", "-")}" viewBox="0 0 10 10" refX="8" refY="5" markerWidth="5" markerHeight="5" orient="auto-start-reverse">'
                  f'<path d="M0,0 L10,5 L0,10 z" class="marker-{v}"/></marker>'
                  for c, v in (("asbuilt", "asbuilt"), ("asbuilt jet", "asbuilt"), ("best", "best"), ("best jet", "best"), ("nose", "nose"), ("axis", "axis")))
LEGEND = f"Blue: {NAME_LABEL.lower()} ({BEST_LABEL}) · dashed orange: today's CFD jet 39.6° · purple: nose fans · angles above the fan axis"
PAD_TOP, PAD_BOT = 44, 12


def standalone_svg(view, theme=LIGHT, hidden=("horizon",)):
    """One view as a self-contained SVG document with a title and legend (what the PNG export renders)."""
    _, w, h, _, _ = parts(view)
    body = svg_body(view)
    for layer in hidden:
        body = body.replace(f'class="layer" data-layer="{layer}"', f'class="layer hidden" data-layer="{layer}"')
    title = VIEWS[view][4]
    H = h + PAD_TOP + PAD_BOT
    return (f'<svg xmlns="http://www.w3.org/2000/svg" width="{w:.0f}" height="{H:.0f}" viewBox="0 {-PAD_TOP} {w:.0f} {H:.0f}">'
            f'<style>svg {{ {VARS(theme)} }} {SVG_CSS}</style><defs>{MARKERS}</defs>'
            f'<rect x="0" y="{-PAD_TOP}" width="{w:.0f}" height="{H:.0f}" fill="{theme["card"]}"/>'
            f'<text class="title" x="12" y="-24">ATLAS V3_30 · {title}</text><text class="legend" x="12" y="-7">{LEGEND}</text>{body}</svg>')


rows = "".join(f"<tr><td>{n}</td><td>{m}</td><td>{FOIL[a]:.1f}°</td><td class='b'>{JET[a]:.1f}°</td><td class='b'>{90 - JET[a]:.1f}°</td></tr>"
               for n, m, a in (("outer", "M1 M2", "M1"), ("middle", "M3 M4", "M3"), ("inner", "M5 M6", "M5")))
PAGE_CSS = (":root { " + VARS(LIGHT) + " }\n"
            "@media (prefers-color-scheme: dark) { :root:not([data-theme=\"light\"]) { " + VARS(DARK) + " } }\n"
            ":root[data-theme=\"dark\"] { " + VARS(DARK) + " }\n" + """
body { margin:0; background:var(--bg); color:var(--fg); font:15px/1.45 system-ui, -apple-system, "Segoe UI", sans-serif; }
main { max-width:1200px; margin:0 auto; padding:20px 16px 40px; }
h1 { font-size:20px; margin:0 0 4px; } .sub { color:var(--muted); margin:0 0 14px; }
.controls { display:flex; flex-wrap:wrap; align-items:center; gap:8px 18px; padding:10px 12px; background:var(--card); border:1px solid var(--line); border-radius:10px; position:sticky; top:0; z-index:2; }
.controls label { display:flex; align-items:center; gap:6px; cursor:pointer; user-select:none; }
.spacer { flex:1; }
.sw { width:22px; height:4px; border-radius:2px; display:inline-block; }
button.save, button.saveall { font:600 13px system-ui, sans-serif; color:var(--fg); background:var(--bg); border:1px solid var(--line); border-radius:8px; padding:5px 10px; cursor:pointer; white-space:nowrap; }
button.save:hover, button.saveall:hover { border-color:var(--best); color:var(--best); }
figure { margin:16px 0 0; background:var(--card); border:1px solid var(--line); border-radius:10px; padding:10px; }
figcaption { display:flex; justify-content:space-between; align-items:center; gap:12px; color:var(--muted); font-size:13px; margin:0 0 6px; }
figure svg { width:100%; height:auto; display:block; }
table { border-collapse:collapse; margin:16px 0 0; background:var(--card); border:1px solid var(--line); border-radius:10px; overflow:hidden; font-size:14px; }
th, td { padding:7px 12px; border-bottom:1px solid var(--line); text-align:right; } th:first-child, td:first-child, td:nth-child(2) { text-align:left; }
th { color:var(--muted); font-weight:600; } td.b { color:var(--best); font-weight:600; }
.note { color:var(--muted); font-size:13px; margin-top:12px; max-width:900px; }
.wrap { overflow-x:auto; }
""")
SCRIPT = ("""
document.querySelectorAll('[data-toggle]').forEach(cb => {
  const apply = () => document.querySelectorAll(`[data-layer="${cb.dataset.toggle}"]`).forEach(el => el.classList.toggle('hidden', !cb.checked));
  cb.addEventListener('change', apply); apply();
});
const NS = 'http://www.w3.org/2000/svg';
const VARNAMES = """ + json.dumps(list(LIGHT)) + """;
const LEGEND = """ + json.dumps(LEGEND) + """;
const PAD_TOP = """ + str(PAD_TOP) + """, PAD_BOT = """ + str(PAD_BOT) + """;
// A view as a PNG, drawn the way it is shown now (layers on or off, light or dark theme), at 2x.
async function savePng(fig) {
  const src = fig.querySelector('svg'), vb = src.viewBox.baseVal, W = vb.width, H = vb.height + PAD_TOP + PAD_BOT;
  const svg = src.cloneNode(true);
  svg.setAttribute('xmlns', NS); svg.setAttribute('width', W); svg.setAttribute('height', H);
  svg.setAttribute('viewBox', `0 ${-PAD_TOP} ${W} ${H}`);
  svg.removeAttribute('style');
  const cs = getComputedStyle(document.documentElement);
  const vars = VARNAMES.map(v => `--${v}:${cs.getPropertyValue('--' + v).trim()}`).join(';');
  const style = document.createElementNS(NS, 'style');
  style.textContent = `svg{${vars}} ` + document.getElementById('svg-css').textContent;
  const defs = document.getElementById('markers').querySelector('defs').cloneNode(true);
  const bg = document.createElementNS(NS, 'rect');
  bg.setAttribute('x', 0); bg.setAttribute('y', -PAD_TOP); bg.setAttribute('width', W); bg.setAttribute('height', H);
  bg.setAttribute('fill', cs.getPropertyValue('--card').trim());
  const t = document.createElementNS(NS, 'text'); t.setAttribute('class', 'title'); t.setAttribute('x', 12); t.setAttribute('y', -24);
  t.textContent = 'ATLAS V3_30 · ' + fig.querySelector('figcaption span').textContent;
  const lg = document.createElementNS(NS, 'text'); lg.setAttribute('class', 'legend'); lg.setAttribute('x', 12); lg.setAttribute('y', -7);
  lg.textContent = LEGEND;
  svg.prepend(style, defs, bg, t, lg);
  const url = URL.createObjectURL(new Blob([new XMLSerializer().serializeToString(svg)], {type: 'image/svg+xml'}));
  const img = new Image();
  await new Promise((ok, fail) => { img.onload = ok; img.onerror = () => fail(new Error('the SVG did not load')); img.src = url; });
  const k = 2, c = document.createElement('canvas');
  c.width = W * k; c.height = H * k;
  const g = c.getContext('2d'); g.scale(k, k); g.drawImage(img, 0, 0, W, H);
  URL.revokeObjectURL(url);
  const blob = await new Promise(ok => c.toBlob(ok, 'image/png'));
  const a = document.createElement('a');
  a.href = URL.createObjectURL(blob); a.download = `v3_30_${fig.dataset.view}.png`;
  document.body.appendChild(a); a.click(); a.remove();
  setTimeout(() => URL.revokeObjectURL(a.href), 2000);
  return blob.size;
}
window.savePng = savePng;
document.querySelectorAll('button.save').forEach(b => b.addEventListener('click', () => savePng(b.closest('figure')).catch(e => alert('PNG export failed: ' + e.message))));
document.querySelector('button.saveall').addEventListener('click', async () => {
  for (const f of document.querySelectorAll('figure[data-view]')) {
    await savePng(f).catch(e => alert('PNG export failed: ' + e.message));
    await new Promise(r => setTimeout(r, 400));
  }
});
""")

html = f"""<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>V3_30 Jet Angles</title>
<style>{PAGE_CSS}</style><style id="svg-css">{SVG_CSS}</style></head><body><main>
<h1>ATLAS V3_30 · target foil jets 57.5 / 85 / 85 in the drone's frame</h1>
<p class="sub">Base: render of SMALL_SCALE_V3_30.step (14.40 kg). Frame FRD: x forward, y right, z down, origin on the centreline at boom level. Jet angle = how far above the fan axis (−x, the duct line) the air leaves the foil group, i.e. how far the foil turns the jet. Parked at −10°, hovers at 8.5° nose-up.</p>
<div class="controls">
  <label><input type="checkbox" data-toggle="cad" checked> CAD</label>
  <label><input type="checkbox" data-toggle="asbuilt" checked><span class="sw" style="background:var(--asbuilt)"></span> Today's CFD jet (39.6°)</label>
  <label><input type="checkbox" data-toggle="best" checked><span class="sw" style="background:var(--best)"></span> {NAME_LABEL} ({BEST_LABEL})</label>
  <label><input type="checkbox" data-toggle="nose" checked><span class="sw" style="background:var(--nose)"></span> Nose fans</label>
  <label><input type="checkbox" data-toggle="mass" checked><span class="sw" style="background:var(--mass)"></span> CG</label>
  <label><input type="checkbox" data-toggle="horizon"><span class="sw" style="background:var(--horizon)"></span> Hover horizon</label>
  <label><input type="checkbox" data-toggle="axes" checked> Axes</label>
  <span class="spacer"></span>
  <button class="saveall" title="save the three views as PNGs, with the layers as shown">Save all PNGs</button>
</div>
{view_html("side")}
{view_html("front")}
{view_html("top")}
<div class="wrap"><table><thead><tr><th>Station</th><th>Fans</th><th>Today's CFD jet</th><th>Target jet</th><th>Target thrust lean from body-up</th></tr></thead><tbody>{rows}</tbody></table></div>
<p class="note">Jet angles are what the CFD measures (the air leaving the foils), not foil bend angles: today's foils turn the jet only ~63 % as far as the foil. Long arrows at the fans: thrust direction. Short arrows at the foil trailing edges: jet leaving the foil. Solid blue = {NAME_LABEL.lower()}, dashed orange = today's CFD-measured jet (39.6° above the fan axis for every group). In the front view the foil thrust has no sideways part, so both layers point straight up there and differ only in length (the vertical share of the thrust); the nose fans' 30° inward cant shows only in that view. In the top view the foil arrows are drawn twice as long: their forward share is small. Side view shows the right-hand fans (the left ones sit behind them). Masses from the V3_30 CAD folders (14.40 kg), CG as listed.</p>
</main>
<svg id="markers" width="0" height="0" style="position:absolute"><defs>{MARKERS}</defs></svg>
<script>{SCRIPT}</script>
</body></html>"""
out_dir = os.path.dirname(OUT) or "."
os.makedirs(out_dir, exist_ok=True)
open(OUT, "w", encoding="utf-8").write(html)
print("wrote", OUT, len(html) // 1024, "kB")

# PNGs: Edge (headless, on Windows) renders each standalone SVG at its own size, 2x
EDGE = "/mnt/c/Program Files (x86)/Microsoft/Edge/Application/msedge.exe"


def win(path):
    return subprocess.check_output(["wslpath", "-w", os.path.abspath(path)], text=True).strip()


def inline_vars(svg, theme):
    """resvg does not resolve CSS custom properties: write the theme's colours into the rules."""
    for k, v in theme.items():
        svg = svg.replace(f"var(--{k})", v)
    return svg.replace("system-ui, \"Segoe UI\", sans-serif", "\"Helvetica Neue\", Helvetica, Arial, sans-serif")


if os.path.exists(EDGE) and shutil.which("wslpath"):
    for view in ("side", "front", "top"):
        _, w, h, _, _ = parts(view)
        svg_path = os.path.join(out_dir, f"v3_30_{view}.svg")
        png_path = os.path.join(out_dir, f"v3_30_{view}.png")
        with open(svg_path, "w", encoding="utf-8") as f:
            f.write(standalone_svg(view))
        profile = win("/mnt/c/Users/Public") + "\\v3_views_edge_profile"
        subprocess.run([EDGE, "--headless=new", "--disable-gpu", "--hide-scrollbars", "--no-first-run", f"--user-data-dir={profile}",
                        "--force-device-scale-factor=2", f"--window-size={int(w)},{int(h + PAD_TOP + PAD_BOT)}", f"--screenshot={win(png_path)}",
                        "file:///" + win(svg_path).replace("\\", "/")], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=120)
        os.remove(svg_path)
        size = os.path.getsize(png_path) // 1024 if os.path.exists(png_path) else None
        print("wrote", png_path, f"{size} kB" if size is not None else "MISSING")
else:
    try:
        import resvg_py
    except ImportError:
        resvg_py = None
        print("neither Edge nor resvg found: PNGs not written (pip install resvg-py, or use the page's Save PNG buttons)")
    for view in (("side", "front", "top") if resvg_py else ()):
        png_path = os.path.join(out_dir, f"v3_30_{view}.png")
        data = resvg_py.svg_to_bytes(svg_string=inline_vars(standalone_svg(view), LIGHT), zoom=2.0)
        open(png_path, "wb").write(bytes(data))
        print("wrote", png_path, f"{os.path.getsize(png_path) // 1024} kB")
