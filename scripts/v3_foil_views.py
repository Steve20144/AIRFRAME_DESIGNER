"""Side, front and top view of ATLAS V3 small (CAD render as the base) with switchable layers: the CAD foil's jet angles,
the best configuration's angles, nose fans, CG and batteries. Writes one self-contained HTML file."""
import base64, io, json, math, sys
import numpy as np
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
from airframe_designer.geometry.airframe import Airframe

OUT = sys.argv[1]
S = 800.0                                    # px per metre
R_CAD = np.array([[0, 0, -1], [1, 0, 0], [0, -1, 0]], float); ORIGIN = np.array([0.0, 0.0, 0.45])
frd = lambda p: R_CAD @ np.asarray(p, float) + ORIGIN

cad = json.load(open("airframes/cad/SMALL_SCALE_V3.step.bodies.json"))
best = Airframe.load(sys.argv[2] if len(sys.argv) > 2 else "airframes/atlas_v3_small_best.json")
BEST_LABEL = " / ".join(f"{round(r.tilt_deg, 1):g}" for r in best.rotors[0:6:2]) + f", nose {round(abs(best.rotors[6].cant_deg), 1):g}°"
FOIL = {"M1": 75.4, "M2": 75.4, "M3": 74.6, "M4": 74.6, "M5": 72.7, "M6": 72.7}     # CAD foil exit, deg below the duct line
TE = {0.364: frd([0.364, 0.056, 0.466]), 0.282: frd([0.282, 0.117, 0.452]), 0.2: frd([0.2, 0.178, 0.44])}   # foil trailing edges (right side)
STATION = {"M1": 0.364, "M2": 0.364, "M3": 0.282, "M4": 0.282, "M5": 0.2, "M6": 0.2}
NAME = {"M1": "outer", "M3": "middle", "M5": "inner"}

VIEWS = {  # name: (screen-right from an FRD point, screen-up from an FRD point, extents h0 h1, v0 v1, title)
    "side": (lambda p: p[0], lambda p: -p[2], (-0.66, 0.56), (-0.56, 0.17), "Side view · nose right · x forward →, up = −z"),
    "front": (lambda p: -p[1], lambda p: -p[2], (-0.74, 0.74), (-0.56, 0.17), "Front view · looking aft at the nose · drone's right (+y) on the left"),
    "top": (lambda p: p[0], lambda p: -p[1], (-0.66, 0.86), (-0.76, 0.76), "Top view · looking down · nose right, drone's right (+y) at the bottom · foil arrows drawn 2x long"),
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
        te = TE[STATION[m]].copy(); te[1] = math.copysign(te[1], r.pos[1])
        for key, delta in (("asbuilt", FOIL[m]), ("best", 90.0 - r.tilt_deg)):
            dl = math.radians(delta)
            jet = np.array([-math.cos(dl), 0.0, math.sin(dl)])      # aft and down
            thr = -jet                                             # forward and up
            lab_j = f"δ {delta:.1f}°" if view == "side" else None
            L[key].append(arrow(view, p, thr, 0.12 * k_len, key, None))
            L[key].append(arrow(view, te, jet, 0.09 * k_len, key + " jet", lab_j, (6, 4) if key == "best" else (-66, 4)))
        if view == "top" and r.pos[1] > 0:
            x, y = px(view, p)
            tag = {"M2": "M1/M2 outer", "M4": "M3/M4 middle", "M6": "M5/M6 inner"}[m]
            L["best"].append(f'<text class="tag" x="{x + 70:.1f}" y="{y + 5:.1f}">{tag} · θ '
                             f'<tspan class="asbuilt-t">{90 - FOIL[m]:.1f}°</tspan> → <tspan class="best-t">{r.tilt_deg:.1f}°</tspan></text>')
        if view == "side":
            x, y = px(view, p)
            tag = {"M2": "M1/M2 outer", "M4": "M3/M4 middle", "M6": "M5/M6 inner"}[m]
            L["best"].append(f'<text class="tag" x="{x - 12:.1f}" y="{y + 22:.1f}" text-anchor="end">{tag} · θ '
                             f'<tspan class="asbuilt-t">{90 - FOIL[m]:.1f}°</tspan> → <tspan class="best-t">{r.tilt_deg:.1f}°</tspan></text>')
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
    for it in best.mass.items:
        if not it.name.startswith("battery"):
            continue
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


def view_html(view):
    png, w, h = render(view)
    L, axes = layer_svg(view)
    title = VIEWS[view][4]
    groups = "".join(f'<g class="layer" data-layer="{k}">{"".join(v)}</g>' for k, v in L.items())
    return f'''<figure><figcaption>{title}</figcaption>
<svg viewBox="0 0 {w:.0f} {h:.0f}" role="img" aria-label="{title}">
  <image href="data:image/png;base64,{png}" x="0" y="0" width="{w:.0f}" height="{h:.0f}" class="layer" data-layer="cad"/>
  <g class="layer" data-layer="axes">{axes}</g>{groups}
</svg></figure>'''


rows = "".join(f"<tr><td>{n}</td><td>{m}</td><td>{90 - d:.1f}°</td><td>{d:.1f}°</td><td class='b'>{best_t:.1f}°</td><td class='b'>{90 - best_t:.1f}°</td></tr>"
               for n, m, d, best_t in (("outer", "M1 M2", 75.4, 17.5), ("middle", "M3 M4", 74.6, 25.0), ("inner", "M5 M6", 72.7, 25.0)))
rows = "".join(f"<tr><td>{n}</td><td>{m}</td><td>{90 - d:.1f}°</td><td>{d:.1f}°</td><td class='b'>{bt:.1f}°</td><td class='b'>{90 - bt:.1f}°</td></tr>"
               for n, m, d, bt in (("outer", "M1 M2", 75.4, best.rotors[0].tilt_deg), ("middle", "M3 M4", 74.6, best.rotors[2].tilt_deg), ("inner", "M5 M6", 72.7, best.rotors[4].tilt_deg)))
html = f'''<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>V3 Foil Angles</title>
<style>
:root {{ --bg:#fbfbfa; --fg:#1d1f22; --muted:#666c73; --card:#ffffff; --line:#e3e5e8;
  --asbuilt:#d9731a; --best:#1f6fd1; --nose:#8b4fc4; --mass:#1d1f22; --batt:#2e9e5b; --horizon:#7a8290; --axis:#666c73; }}
@media (prefers-color-scheme: dark) {{ :root:not([data-theme="light"]) {{ --bg:#16181b; --fg:#e8eaed; --muted:#a0a6ad; --card:#1f2226; --line:#33373c;
  --asbuilt:#f0923f; --best:#5aa2ff; --nose:#b98af0; --mass:#e8eaed; --batt:#4cc47f; --horizon:#9aa3ad; --axis:#a0a6ad; }} }}
body {{ margin:0; background:var(--bg); color:var(--fg); font:15px/1.45 system-ui, -apple-system, "Segoe UI", sans-serif; }}
main {{ max-width:1200px; margin:0 auto; padding:20px 16px 40px; }}
h1 {{ font-size:20px; margin:0 0 4px; }} .sub {{ color:var(--muted); margin:0 0 14px; }}
.controls {{ display:flex; flex-wrap:wrap; gap:8px 18px; padding:10px 12px; background:var(--card); border:1px solid var(--line); border-radius:10px; position:sticky; top:0; z-index:2; }}
.controls label {{ display:flex; align-items:center; gap:6px; cursor:pointer; user-select:none; }}
.sw {{ width:22px; height:4px; border-radius:2px; display:inline-block; }}
figure {{ margin:16px 0 0; background:var(--card); border:1px solid var(--line); border-radius:10px; padding:10px; }}
figcaption {{ color:var(--muted); font-size:13px; margin:0 0 6px; }}
svg {{ width:100%; height:auto; display:block; }}
line {{ stroke-width:3; stroke-linecap:round; }}
line.asbuilt, line.asbuilt.jet {{ stroke:var(--asbuilt); stroke-dasharray:9 6; }}
line.best, line.best.jet {{ stroke:var(--best); }}
line.jet {{ stroke-width:2.2; }}
line.nose {{ stroke:var(--nose); }} line.axis {{ stroke:var(--axis); stroke-width:1.6; }}
line.horizon {{ stroke:var(--horizon); stroke-width:1.5; stroke-dasharray:4 5; }}
text {{ font:600 15px system-ui, sans-serif; paint-order:stroke; stroke:var(--card); stroke-width:4px; }}
text.asbuilt {{ fill:var(--asbuilt); }} text.best {{ fill:var(--best); }} text.nose {{ fill:var(--nose); }}
text.mass {{ fill:var(--mass); font-weight:500; }} text.horizon {{ fill:var(--horizon); font-weight:500; font-size:13px; }}
text.axis {{ fill:var(--axis); font-weight:500; font-size:13px; }} text.tag {{ fill:var(--fg); font-weight:500; font-size:14px; }}
tspan.asbuilt-t {{ fill:var(--asbuilt); font-weight:700; }} tspan.best-t {{ fill:var(--best); font-weight:700; }}
.nosedot {{ fill:var(--nose); }}
.cg circle {{ fill:var(--card); stroke:var(--mass); stroke-width:2; }} .cg path {{ fill:var(--mass); }}
.batt {{ fill:var(--batt); fill-opacity:.35; stroke:var(--batt); stroke-width:1.5; }}
.hidden {{ display:none; }}
table {{ border-collapse:collapse; margin:16px 0 0; background:var(--card); border:1px solid var(--line); border-radius:10px; overflow:hidden; font-size:14px; }}
th, td {{ padding:7px 12px; border-bottom:1px solid var(--line); text-align:right; }} th:first-child, td:first-child, td:nth-child(2) {{ text-align:left; }}
th {{ color:var(--muted); font-weight:600; }} td.b {{ color:var(--best); font-weight:600; }}
.note {{ color:var(--muted); font-size:13px; margin-top:12px; max-width:900px; }}
.wrap {{ overflow-x:auto; }}
</style></head><body><main>
<h1>ATLAS V3 small · jetfoil angles in the drone's frame</h1>
<p class="sub">Base: render of SMALL_SCALE_V3.step. Frame FRD: x forward, y right, z down, origin on the centreline at boom level. θ = thrust lean forward of body-up (−z); δ = 90° − θ = how far the foil turns the jet down from the duct line (−x).</p>
<div class="controls">
  <label><input type="checkbox" data-toggle="cad" checked> CAD</label>
  <label><input type="checkbox" data-toggle="asbuilt" checked><span class="sw" style="background:var(--asbuilt)"></span> CAD foil (as built)</label>
  <label><input type="checkbox" data-toggle="best" checked><span class="sw" style="background:var(--best)"></span> Best config ({BEST_LABEL})</label>
  <label><input type="checkbox" data-toggle="nose" checked><span class="sw" style="background:var(--nose)"></span> Nose fans</label>
  <label><input type="checkbox" data-toggle="mass" checked><span class="sw" style="background:var(--batt)"></span> CG + batteries</label>
  <label><input type="checkbox" data-toggle="horizon"><span class="sw" style="background:var(--horizon)"></span> Hover horizon</label>
  <label><input type="checkbox" data-toggle="axes" checked> Axes</label>
</div>
{view_html("side")}
{view_html("front")}
{view_html("top")}
<div class="wrap"><table><thead><tr><th>Station</th><th>Fans</th><th>CAD θ</th><th>CAD δ</th><th>Best θ</th><th>Best δ</th></tr></thead><tbody>{rows}</tbody></table></div>
<p class="note">Long arrows at the fans: thrust direction. Short arrows at the foil trailing edges: jet leaving the foil. Solid blue = best configuration, dashed orange = the CAD foil as drawn (jet fully attached). In the front view the foil thrust has no sideways part, so both layers point straight up there and differ only in length (the vertical share of the thrust); the nose fans' 30° inward cant shows only in that view. Side view shows the right-hand fans (the left ones sit behind them). Battery sizes and spacing are assumed; masses 12.8 kg, CG as listed.</p>
</main>
<script>
document.querySelectorAll('[data-toggle]').forEach(cb => {{
  const apply = () => document.querySelectorAll(`[data-layer="${{cb.dataset.toggle}}"]`).forEach(el => el.classList.toggle('hidden', !cb.checked));
  cb.addEventListener('change', apply); apply();
}});
</script>
<svg width="0" height="0" style="position:absolute"><defs>
{"".join(f'<marker id="arr-{c.replace(' ', '-')}" viewBox="0 0 10 10" refX="8" refY="5" markerWidth="5" markerHeight="5" orient="auto-start-reverse"><path d="M0,0 L10,5 L0,10 z" style="fill:var(--{v})"/></marker>' for c, v in (("asbuilt", "asbuilt"), ("asbuilt jet", "asbuilt"), ("best", "best"), ("best jet", "best"), ("nose", "nose"), ("axis", "axis")))}
</defs></svg>
</body></html>'''
open(OUT, "w").write(html)
print("wrote", OUT, len(html) // 1024, "kB")
