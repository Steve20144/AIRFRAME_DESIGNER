"""Engineering report (PDF) for SMALL_SCALE_V3 v34: simulation review and CAD corrections, in plain terms.

Reads the airframes and flight results written on 29 Sep 2026 (results/fusion_v3/, airframes/atlas_v3_v34_*.json)
and writes docs/v3_views/v34/V3_simulation_review.pdf. Fonts: Montserrat TTFs in --fonts (static files).

  python scripts/v3_report_pdf.py --fonts <dir with Montserrat-*.ttf> --logo <airy-logo-black.png> [--out file.pdf]
"""
import argparse, io, json, math, statistics as st, subprocess, sys
from pathlib import Path

import numpy as np
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.collections import PolyCollection
from matplotlib.patches import Rectangle, Circle, FancyArrowPatch

from reportlab.lib.pagesizes import A4
from reportlab.lib.units import mm
from reportlab.lib import colors
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib.enums import TA_LEFT
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import (BaseDocTemplate, PageTemplate, Frame, Paragraph, Spacer, Table, TableStyle, Image,
                                KeepTogether, PageBreak, Flowable)

from airframe_designer.geometry.airframe import Airframe
from airframe_designer.geometry.cad import import_step

ap = argparse.ArgumentParser()
ap.add_argument("--fonts", required=True); ap.add_argument("--logo", required=True)
ap.add_argument("--out", default="docs/v3_views/v34/V3_simulation_review.pdf")
A = ap.parse_args()

GREEN, GRAY, RED, INK, INK2, RULE, CALLOUT = "#1E9255", "#A4A4A4", "#D70400", "#000000", "#6B6B6B", "#DDDDDD", "#F5F5F5"
for w in ("Regular", "Medium", "SemiBold", "Bold"):
    pdfmetrics.registerFont(TTFont(f"Mont-{w}", str(Path(A.fonts) / f"Montserrat-{w}.ttf")))
    font_manager.fontManager.addfont(str(Path(A.fonts) / f"Montserrat-{w}.ttf"))
pdfmetrics.registerFontFamily("Mont-Regular", normal="Mont-Regular", bold="Mont-SemiBold", italic="Mont-Regular", boldItalic="Mont-SemiBold")
plt.rcParams.update({"font.family": "Montserrat", "font.size": 7.5, "axes.edgecolor": GRAY, "axes.labelcolor": INK2,
                     "xtick.color": INK2, "ytick.color": INK2, "axes.spines.top": False, "axes.spines.right": False})

# ------------------------------------------------------------------ data
ASIS = Airframe.load("airframes/atlas_v3_v34_asis.json").resolve_mass()
REC = Airframe.load("airframes/atlas_v3_v34_recommended.json").resolve_mass()


def static_loads(af):
    tp = af.trim_hover_pitch(); q = af.copy(); q.hover_pitch_deg = tp
    E = q.effectiveness(); u = np.linalg.pinv(E) @ np.array([0, 0, 0, 0, 0, -1.0]); up = -(E[5] @ u)
    tmax = np.array([r.effective_max_thrust() for r in q.active_rotors()]); util = u * (q.mass.mass * 9.80665 / up) / tmax
    return tp, util.max() * 100, util.min() * 100


TP0, U0MAX, U0MIN = static_loads(ASIS); TRIM, U1MAX, U1MIN = static_loads(REC)
cg0, cg1 = np.asarray(ASIS.mass.cg), np.asarray(REC.mass.cg)
fus = lambda p: np.array([p[1] * 1000, 450 - p[2] * 1000, -p[0] * 1000])     # FRD m -> Fusion mm (X right, Y up, Z aft)
ftxt = lambda p: "{:.0f}, {:.0f}, {:.0f}".format(*fus(p))


def stab(variant):
    rs = [json.loads(l) for l in open("results/fusion_v3/stab_all2.jsonl")]
    rs = [r for r in rs if r["id"].startswith(variant + "|") and r.get("status") != "error"]
    H = [r["metrics"]["phases"]["hover"] for r in rs if "hover" in r["metrics"].get("phases", {})]
    return dict(n=len(rs), ok=sum(1 for r in rs if r.get("ok") and not r["metrics"].get("crashed")),
                med=st.median(h["pos_drift"] for h in H), mx=max(h["pos_drift"] for h in H),
                pitch=st.mean(h["pitch_err_rms_deg"] for h in H), util=max(h["util_max"] for h in H) * 100,
                td=max(r["metrics"].get("touchdown_speed") or 0 for r in rs))


LAYOUTS = [("As drawn", "asis"), ("Rear block +195 mm aft (recommended)", "recommended"), ("All six packs +130 mm aft (does not fit)", "group13"),
           ("Front pair into rear block, block +45 mm", "all_rear_aft045"), ("Front pair into rear block", "frontpair_back")]
S = {k: stab(k) for _, k in LAYOUTS}
FLOW = {}
for r in map(json.loads, open("results/fusion_v3/final2.jsonl")):
    if r["id"].startswith("flow_"):
        FLOW.setdefault(r["id"].split("|")[0][5:], []).append(r["metrics"]["phases"]["pos_hold"])
FL = {k: dict(med=st.median(x["pos_drift"] for x in v), mx=max(x["pos_drift"] for x in v), alt=st.mean(x["alt_std"] for x in v) * 100, n=len(v)) for k, v in FLOW.items()}
AUTHF = {}
for r in map(json.loads, open("results/fusion_v3/final2.jsonl")):
    if "|v3_authority|" in r["id"] or "|v3_gust|" in r["id"]:
        for n, h in r["metrics"]["phases"].items():
            AUTHF.setdefault(n, []).append(h)
yaw_r = st.mean(h["yaw_drift_deg"] for h in AUTHF["yaw_right"]) / 5.0
yaw_l = -st.mean(h["yaw_drift_deg"] for h in AUTHF["yaw_left"]) / 5.0
gust_yaw = [abs(h["yaw_drift_deg"]) for h in AUTHF["gust_east"]]
LP = json.loads(subprocess.check_output([sys.executable, "results/fusion_v3/authority_lp.py", "airframes/atlas_v3_v34_recommended.json"],
                                        text=True, stderr=subprocess.DEVNULL).split(" ", 1)[1])
NFLIGHTS = 154

# ------------------------------------------------------------------ geometry for the figures
imp = import_step(REC.cad.file, log=lambda s: None)
Rm = REC.cad.matrix(); O = np.asarray(REC.cad.origin)
BODIES = [(b["name"], np.asarray(b["vertices"], float).reshape(-1, 3) @ Rm.T + O, np.asarray(b["indices"], int).reshape(-1, 3)) for b in imp["bodies"]]
th = math.radians(TRIM)
DOWN = np.array([-math.sin(th), 0.0, math.cos(th)])
HF_CAD, HF_NOSE = np.array([-0.015, 0.0, 0.062]), np.array([0.30, 0.0, 0.077])
PACK = np.array([0.16, 0.05, 0.05])


def packs(af):
    out = []
    for b in af.cad.bodies:
        if "battery_5200" in b.name and b.mass > 0:
            c = np.round(af.cad.body_pos(b), 4)
            if not any(np.allclose(c, o) for o in out):
                out.append(c)
    return out


def silhouette(ax, hf, vf):
    for name, v, t in BODIES:
        if "battery_5200" in name or "H-FLOW" in name:
            continue
        P = np.stack([hf(v), vf(v)], axis=1)[t]
        ax.add_collection(PolyCollection(P, facecolor="#B9BEC4", edgecolor="none", alpha=0.35))


def to_img(fig, width_mm):
    buf = io.BytesIO(); fig.savefig(buf, format="png", dpi=260, bbox_inches="tight", facecolor="white"); plt.close(fig)
    buf.seek(0)
    img = Image(buf); r = img.imageHeight / img.imageWidth
    img.drawWidth = width_mm * mm; img.drawHeight = width_mm * mm * r
    return img


def cg_marker(ax, x, y, color, label, dy):
    ax.add_patch(Circle((x, y), 0.011, facecolor="white", edgecolor=color, lw=1.2, zorder=6))
    ax.add_patch(matplotlib.patches.Wedge((x, y), 0.011, 90, 180, facecolor=color, edgecolor="none", zorder=7))
    ax.add_patch(matplotlib.patches.Wedge((x, y), 0.011, 270, 360, facecolor=color, edgecolor="none", zorder=7))
    ax.text(x + 0.018, y + dy, label, color=color, fontsize=7, fontweight="semibold", zorder=8)


def arrow(ax, p, q, color, ls="-", lw=1.6):
    ax.add_patch(FancyArrowPatch(p, q, arrowstyle="-|>", mutation_scale=9, color=color, lw=lw, linestyle=ls, zorder=6))


def fig_side():
    fig, ax = plt.subplots(figsize=(7.2, 3.9)); ax.set_aspect("equal"); ax.axis("off")
    hf, vf = (lambda v: v[..., 0]), (lambda v: -v[..., 2])
    silhouette(ax, hf, vf)
    old, new = packs(ASIS), packs(REC)
    for c in old:
        ax.add_patch(Rectangle((c[0] - PACK[0] / 2, -c[2] - PACK[2] / 2), PACK[0], PACK[2], fill=False, ec=GRAY, lw=1.1, ls=(0, (4, 3)), zorder=4))
    for c in new:
        if not any(np.allclose(c, o) for o in old):
            ax.add_patch(Rectangle((c[0] - PACK[0] / 2, -c[2] - PACK[2] / 2), PACK[0], PACK[2], fc=GREEN, alpha=0.28, ec=GREEN, lw=1.3, zorder=5))
    arrow(ax, (-0.235, 0.050), (-0.43, 0.050), GREEN)
    ax.text(-0.43, 0.075, "rear block +195 mm aft", color=GREEN, fontsize=7, fontweight="semibold")
    cg_marker(ax, cg0[0], -cg0[2], GRAY, "", 0)
    cg_marker(ax, cg1[0], -cg1[2], GREEN, "", 0)
    ax.annotate("CG as drawn", (cg0[0], -cg0[2]), (cg0[0] + 0.11, -cg0[2] + 0.11), color=INK2, fontsize=7, fontweight="semibold", bbox=dict(fc="white", ec="none", pad=1.2, alpha=0.9), arrowprops=dict(arrowstyle="-", color=GRAY, lw=0.7))
    ax.annotate("CG after change", (cg1[0], -cg1[2]), (cg1[0] - 0.12, 0.13), color=GREEN, fontsize=7, fontweight="semibold", bbox=dict(fc="white", ec="none", pad=1.2, alpha=0.9), arrowprops=dict(arrowstyle="-", color=GREEN, lw=0.7))
    arrow(ax, (HF_CAD[0], -HF_CAD[2]), (HF_CAD[0], -HF_CAD[2] - 0.12), GRAY)
    q = HF_CAD + 0.14 * DOWN; arrow(ax, (HF_CAD[0], -HF_CAD[2]), (q[0], -q[2]), GREEN)
    ax.text(q[0] - 0.02, -q[2] - 0.035, f"H-FLOW view, tilted {TRIM:.1f}°", color=GREEN, fontsize=7, fontweight="semibold", ha="right")
    ax.text(HF_CAD[0] + 0.012, -HF_CAD[2] - 0.12, "as drawn", color=INK2, fontsize=6.5)
    q = HF_NOSE + 0.10 * DOWN; arrow(ax, (HF_NOSE[0], -HF_NOSE[2]), (q[0], -q[2]), INK2, ls=(0, (3, 2)), lw=1.3)
    ax.text(q[0] + 0.01, -q[2] - 0.025, "option B", color=INK2, fontsize=6.5)
    ax.text(0.47, 0.16, "NOSE", color=INK2, fontsize=6.5, ha="right"); ax.text(-0.60, 0.16, "TAIL", color=INK2, fontsize=6.5)
    ax.set_xlim(-0.62, 0.52); ax.set_ylim(-0.42, 0.19)
    return fig


def fig_top():
    fig, ax = plt.subplots(figsize=(4.4, 5.6)); ax.set_aspect("equal"); ax.axis("off")
    hf, vf = (lambda v: v[..., 0]), (lambda v: -v[..., 1])
    silhouette(ax, hf, vf)
    old, new = packs(ASIS), packs(REC)
    for c in old:
        ax.add_patch(Rectangle((c[0] - PACK[0] / 2, -c[1] - PACK[1] / 2), PACK[0], PACK[1], fill=False, ec=GRAY, lw=1.1, ls=(0, (4, 3)), zorder=4))
    for c in new:
        if not any(np.allclose(c, o) for o in old):
            ax.add_patch(Rectangle((c[0] - PACK[0] / 2, -c[1] - PACK[1] / 2), PACK[0], PACK[1], fc=GREEN, alpha=0.28, ec=GREEN, lw=1.3, zorder=5))
    cg_marker(ax, cg0[0], -cg0[1], GRAY, "", 0)
    cg_marker(ax, cg1[0], -cg1[1], GREEN, "", 0)
    ax.annotate("CG as drawn", (cg0[0], -cg0[1]), (cg0[0] + 0.05, 0.20), color=INK2, fontsize=6.5, fontweight="semibold", bbox=dict(fc="white", ec="none", pad=1.2, alpha=0.9), arrowprops=dict(arrowstyle="-", color=GRAY, lw=0.7))
    ax.annotate("CG after", (cg1[0], -cg1[1]), (cg1[0] - 0.12, -0.20), color=GREEN, fontsize=6.5, fontweight="semibold", bbox=dict(fc="white", ec="none", pad=1.2, alpha=0.9), arrowprops=dict(arrowstyle="-", color=GREEN, lw=0.7))
    ax.add_patch(Circle((HF_CAD[0], 0), 0.012, fc=GREEN, ec="none", zorder=7))
    ax.annotate("H-FLOW (A)", (HF_CAD[0], 0), (HF_CAD[0] + 0.10, -0.26), color=GREEN, fontsize=6.5, fontweight="semibold", bbox=dict(fc="white", ec="none", pad=1.2, alpha=0.9), arrowprops=dict(arrowstyle="-", color=GREEN, lw=0.7))
    ax.add_patch(Circle((HF_NOSE[0], 0), 0.012, fc="white", ec=INK2, lw=1.1, zorder=7))
    ax.annotate("option B", (HF_NOSE[0], 0), (HF_NOSE[0] - 0.02, 0.24), color=INK2, fontsize=6.5, bbox=dict(fc="white", ec="none", pad=1.2, alpha=0.9), arrowprops=dict(arrowstyle="-", color=GRAY, lw=0.7))
    ax.text(0.47, 0.70, "NOSE", color=INK2, fontsize=6.5, ha="right"); ax.text(-0.60, 0.70, "TAIL", color=INK2, fontsize=6.5)
    ax.text(-0.60, -0.72, "aircraft's right side at the bottom", color=INK2, fontsize=6.5)
    ax.set_xlim(-0.62, 0.52); ax.set_ylim(-0.74, 0.74)
    return fig


def fig_layout_chart():
    fig, ax = plt.subplots(figsize=(7.2, 2.3))
    names = [n for n, _ in LAYOUTS][::-1]; keys = [k for _, k in LAYOUTS][::-1]
    y = np.arange(len(keys))
    for i, k in enumerate(keys):
        col = GREEN if k == "recommended" else GRAY
        ax.barh(i, S[k]["med"], height=0.5, color=col)
        ax.plot([S[k]["med"], S[k]["mx"]], [i, i], color=col, lw=1)
        ax.plot(S[k]["mx"], i, "|", color=col, ms=7)
        ax.text(S[k]["mx"] + 0.05, i, f"{S[k]['med']:.2f} m  (worst {S[k]['mx']:.2f})", va="center", fontsize=7, color=INK)
    ax.set_yticks(y); ax.set_yticklabels(names, fontsize=7.2); ax.tick_params(length=0)
    ax.spines["left"].set_visible(False); ax.set_xlim(0, 3.4); ax.set_xlabel("hands-off hover drift, median over 9 flights (bar) and worst (tick), m")
    return fig


def fig_flow_chart():
    fig, ax = plt.subplots(figsize=(7.2, 1.7))
    rows = [("Current spot, flat (as drawn)", "CAD_flat", GRAY), (f"Current spot, tilted {TRIM:.1f}° (A)", "CAD_tilted", GREEN), (f"Nose, tilted {TRIM:.1f}° (B)", "nose_tilted", INK2)][::-1]
    for i, (n, k, c) in enumerate(rows):
        ax.barh(i, FL[k]["med"], height=0.5, color=c); ax.plot([FL[k]["med"], FL[k]["mx"]], [i, i], color=c, lw=1); ax.plot(FL[k]["mx"], i, "|", color=c, ms=7)
        ax.text(FL[k]["mx"] + 0.02, i, f"{FL[k]['med']:.2f} m  (worst {FL[k]['mx']:.2f})", va="center", fontsize=7)
    ax.set_yticks(range(len(rows))); ax.set_yticklabels([r[0] for r in rows], fontsize=7.2); ax.tick_params(length=0)
    ax.spines["left"].set_visible(False); ax.set_xlim(0, 0.95); ax.set_xlabel("position-hold drift on the optical sensor only, median over 6 flights (bar) and worst (tick), m")
    return fig


def jet_points(H):
    Ry = np.column_stack([np.array([math.cos(th), 0, math.sin(th)]), [0, 1, 0], DOWN]).T
    def fp(p, d):
        ph = Ry @ (np.asarray(p) - cg1); dh = Ry @ np.asarray(d); t = (H - ph[2]) / dh[2]; return (ph + t * dh)[:2]
    jets = [(r.name, fp(r.pos, -np.asarray(r.axis) / np.linalg.norm(r.axis))) for r in REC.rotors]
    views = {}
    for nm, p in (("belly", HF_CAD), ("nose", HF_NOSE)):
        lens_h = H - (Ry @ (p - cg1))[2]
        views[nm] = (fp(p, DOWN), lens_h * math.tan(math.radians(21)))
    return jets, views


def clear_height(name):
    for H in np.arange(0.5, 3.0, 0.05):
        jets, views = jet_points(H); c, r = views[name]
        if min(np.linalg.norm(j - c) for _, j in jets) < r:
            return H - 0.05
    return 3.0


H_BELLY, H_NOSE = clear_height("belly"), clear_height("nose")


def fig_floor():
    fig, axs = plt.subplots(1, 2, figsize=(7.2, 2.9))
    for ax, H in zip(axs, (1.0, 1.5)):
        jets, views = jet_points(H)
        ax.set_aspect("equal"); ax.set_xlim(-1.0, 1.4); ax.set_ylim(-1.3, 1.3); ax.invert_yaxis()
        for s in ("left", "bottom"): ax.spines[s].set_color(RULE)
        for nm, j in jets:
            ax.plot(j[0], j[1], "o", color=INK, ms=4); ax.text(j[0] + 0.05, j[1] + 0.03, nm, fontsize=6, color=INK2)
        (cb, rb), (cn, rn) = views["belly"], views["nose"]
        ax.add_patch(Circle(cb, rb, fill=False, ec=GREEN, lw=1.5)); ax.add_patch(Circle(cn, rn, fill=False, ec=INK2, lw=1.2, ls=(0, (3, 2))))
        ax.plot(0, 0, "+", color=INK, ms=7)
        ax.set_title(f"hover height {H:.1f} m", fontsize=7.5, color=INK, loc="left")
        ax.set_xlabel("forward of the CG, m"); ax.tick_params(labelsize=6.5)
    axs[0].set_ylabel("to the right, m")
    return fig


# ------------------------------------------------------------------ document
W, H = A4
M = 18 * mm
logo = Image(A.logo); lr = logo.imageHeight / logo.imageWidth


def on_page(c, doc):
    c.saveState()
    c.drawImage(A.logo, W - M - 20 * mm, H - 9 * mm - 20 * mm * lr, width=20 * mm, height=20 * mm * lr, mask="auto")
    c.setFont("Mont-Bold", 7); c.setFillColor(colors.HexColor(INK))
    c.drawString(M, H - 12.5 * mm, "V3 SMALL SCALE"); c.setFont("Mont-Regular", 7); c.setFillColor(colors.HexColor(INK2))
    c.drawString(M + 24 * mm, H - 12.5 * mm, "SIMULATION REVIEW AND CAD CORRECTIONS")
    c.setStrokeColor(colors.HexColor(RULE)); c.setLineWidth(0.5); c.line(M, 14 * mm, W - M, 14 * mm)
    c.setFont("Mont-Regular", 7); c.setFillColor(colors.HexColor(INK2))
    c.drawString(M, 10 * mm, "V3-SIM-2026-09-29 · Rev A · 29 Sep 2026")
    c.drawRightString(W - M, 10 * mm, f"Page {doc.page}")
    c.restoreState()


st_body = ParagraphStyle("body", fontName="Mont-Regular", fontSize=9.4, leading=14, textColor=colors.HexColor(INK), spaceAfter=6)
st_small = ParagraphStyle("small", parent=st_body, fontSize=7.8, leading=11, textColor=colors.HexColor(INK2))
st_cap = ParagraphStyle("cap", parent=st_small, spaceBefore=3, spaceAfter=10)
st_bul = ParagraphStyle("bul", parent=st_body, leftIndent=12, bulletIndent=2, spaceAfter=3)
st_num = ParagraphStyle("num", parent=st_body, leftIndent=16, bulletIndent=0, spaceAfter=4)
st_cell = ParagraphStyle("cell", fontName="Mont-Regular", fontSize=8, leading=10.5, textColor=colors.HexColor(INK))
st_cellb = ParagraphStyle("cellb", parent=st_cell, fontName="Mont-SemiBold")
st_head = ParagraphStyle("head", fontName="Mont-Bold", fontSize=6.8, leading=9, textColor=colors.HexColor(INK2))


class Heading(Flowable):
    """Two-tone uppercase heading with a thin green rule under it."""
    def __init__(self, bold, rest="", size=13, rule=True, before=10):
        super().__init__(); self.bold, self.rest, self.size, self.rule, self.before = bold.upper(), rest.upper(), size, rule, before
    def wrap(self, aw, ah):
        self.aw = aw; return aw, self.size + (7 if self.rule else 3) + self.before
    def draw(self):
        c = self.canv; y = 5 if self.rule else 1
        c.setFont("Mont-Bold", self.size); c.setFillColor(colors.HexColor(INK)); c.drawString(0, y + 2, self.bold)
        wbold = pdfmetrics.stringWidth(self.bold + " ", "Mont-Bold", self.size)
        c.setFont("Mont-Regular", self.size); c.setFillColor(colors.HexColor(INK2)); c.drawString(wbold, y + 2, self.rest)
        if self.rule:
            c.setStrokeColor(colors.HexColor(GREEN)); c.setLineWidth(0.75); c.line(0, 0, self.aw, 0)


class Stats(Flowable):
    """Big numerals with small labels, in a row."""
    def __init__(self, items):
        super().__init__(); self.items = items
    def wrap(self, aw, ah):
        self.aw = aw; return aw, 30 * mm
    def draw(self):
        c = self.canv; n = len(self.items); w = self.aw / n
        for i, (big, unit, label) in enumerate(self.items):
            x = i * w
            c.setFont("Mont-Bold", 26); c.setFillColor(colors.HexColor(GREEN)); c.drawString(x, 13 * mm, big)
            bw = pdfmetrics.stringWidth(big, "Mont-Bold", 26)
            c.setFont("Mont-SemiBold", 9); c.setFillColor(colors.HexColor(INK)); c.drawString(x + bw + 3, 13 * mm, unit)
            p = Paragraph(label, ParagraphStyle("sl", parent=st_small, fontSize=7.6, leading=10))
            pw, ph = p.wrap(w - 8 * mm, 20 * mm); p.drawOn(c, x, 11 * mm - ph)


def P(t, s=st_body): return Paragraph(t, s)
def bullets(items, style=st_bul): return [Paragraph(t, style, bulletText="•") for t in items]
def numbered(items): return [Paragraph(t, st_num, bulletText=f"{i}.") for i, t in enumerate(items, 1)]


def table(rows, widths, head=True, bold_rows=(), green_rows=()):
    data = []
    for i, r in enumerate(rows):
        cells = []
        for j, v in enumerate(r):
            if head and i == 0:
                cells.append(Paragraph(str(v).upper(), st_head))
            else:
                sty = st_cellb if i in bold_rows else st_cell
                if i in green_rows:
                    sty = ParagraphStyle("g", parent=sty, textColor=colors.HexColor(GREEN), fontName="Mont-SemiBold")
                cells.append(Paragraph(str(v), sty))
        data.append(cells)
    t = Table(data, colWidths=[w * mm for w in widths], hAlign="LEFT", repeatRows=1 if head else 0)
    t.setStyle(TableStyle([("LINEBELOW", (0, 0), (-1, -1), 0.5, colors.HexColor(RULE)), ("VALIGN", (0, 0), (-1, -1), "TOP"),
                           ("TOPPADDING", (0, 0), (-1, -1), 4), ("BOTTOMPADDING", (0, 0), (-1, -1), 4), ("LEFTPADDING", (0, 0), (-1, -1), 3)]))
    return t


def callout(flow_items):
    t = Table([[flow_items]], colWidths=[W - 2 * M - 2 * mm], hAlign="LEFT")
    t.setStyle(TableStyle([("BACKGROUND", (0, 0), (-1, -1), colors.HexColor(CALLOUT)), ("LINEBEFORE", (0, 0), (0, -1), 1.6, colors.HexColor(GREEN)),
                           ("LEFTPADDING", (0, 0), (-1, -1), 9), ("RIGHTPADDING", (0, 0), (-1, -1), 9), ("TOPPADDING", (0, 0), (-1, -1), 7), ("BOTTOMPADDING", (0, 0), (-1, -1), 3)]))
    return t


a0, a1 = S["asis"], S["recommended"]
drift_cut = (1 - a1["med"] / a0["med"]) * 100
flow_cut = (1 - FL["CAD_tilted"]["med"] / FL["CAD_flat"]["med"]) * 100
story = []

# ---- cover block
story += [Spacer(1, 6 * mm), Heading("V3 Small Scale", "", size=24, rule=False, before=0), Heading("Simulation review", "and CAD corrections", size=15, rule=True, before=2), Spacer(1, 5 * mm)]
story.append(table([["Item", "Detail"],
                    ["Document", "V3-SIM-2026-09-29, revision A, 29 September 2026"],
                    ["Design reviewed", "Fusion 360: ATLAS / V3 / SMALL_SCALE_V3, version 34"],
                    ["Method", f"Airframe Designer flight simulator with the PX4 v1.17 flight controller in the loop, {NFLIGHTS} simulated flights"],
                    ["Prepared for", "CAD design, to apply the corrections in section 4"],
                    ["Prepared by", "S. Fragkoulis"]], [38, 136]))
story += [Spacer(1, 6 * mm), Heading("1  Summary", "")]
story.append(P(f"We read the current CAD model, worked out the weight and balance of the aircraft from the part labels, and flew it in the simulator. "
               f"The aircraft <b>flies as drawn</b>: all {a0['n']} test hovers were stable and none crashed. Two things hold it back, and both are simple CAD changes."))
story += bullets([f"<b>The batteries sit too far forward.</b> The balance point (centre of gravity, CG) is {abs(cg1[0] - cg0[0]) * 1000:.0f} mm ahead of where the nine fans share the work evenly. "
                  f"The nose fans work harder than the rest and the aircraft wanders more in a hands-off hover.",
                  f"<b>The optical flow sensor (H-FLOW) points the wrong way in flight.</b> The aircraft hovers {TRIM:.0f}° nose-up, so a sensor mounted flat looks {TRIM:.0f}° off vertical. "
                  "That makes the indoor position hold less accurate."])
story.append(Stats([(f"{drift_cut:.0f}", "%", f"less wander in a hands-off hover after moving the batteries ({a0['med']:.2f} to {a1['med']:.2f} m)"),
                    (f"{U1MAX:.0f}", "%", f"load on the busiest fan after the change, down from {U0MAX:.0f} %"),
                    (f"{flow_cut:.0f}", "%", f"less drift when holding position on the optical sensor, after tilting its mount")]))
story.append(callout([P("<b>Required CAD changes</b>", st_body)] + numbered([
    "<b>C1</b>: move the rear block of four batteries 195 mm towards the tail, and lower the two outer ones 10 mm.",
    f"<b>C2</b>: tilt the H-FLOW mount {TRIM:.1f}° so it looks straight down while hovering.",
    "<b>C3</b>: correct the weight labels on the parts (missing labels, duplicates, one inconsistent pair).",
    "<b>No change</b>: jetfoil angles and nose fans stay as drawn."])))
story.append(PageBreak())

# ---- method
story += [Heading("2  Background", "and method")]
story.append(P("The simulator is a digital copy of the aircraft: its geometry and weights, the thrust of every fan, the jet turned by the jetfoils, the legs on the ground. "
               "It is connected to the same flight-control software that runs on the real aircraft (PX4), which flies it exactly as it would fly the real one. The work followed five steps:"))
story += numbered(["<b>Read the CAD.</b> The open Fusion design was read directly: every part, its position and its shape. The angle at which each jetfoil releases its jet was measured on the model surface: "
                   "69.0° (inner), 59.3° (middle) and 49.4° (outer) below the fan axis.",
                   "<b>Weigh it.</b> Only parts whose name carries a weight label (for example <font name='Mont-SemiBold'>battery_5200mAh__M760g</font> = 760 g) were counted. Parts without a label, and unlabelled copies that sit exactly on top of a labelled part, count as zero.",
                   "<b>Find the balance point.</b> From the weights and positions we computed the CG and, for each battery position, how hard each fan must work to hover.",
                   "<b>Fly it.</b> Each design was flown hands-off: take off, hover 12 s, land. Every case was flown three times with different sensor noise, and at its hover angle and 0.5° either side, "
                   "so a result does not depend on one lucky flight.",
                   "<b>Test the optical sensor.</b> A model of the H-FLOW (camera that tracks the floor plus a distance sensor) was added, GPS was switched off, and the aircraft was asked to hold its position indoors."])
story += [Heading("3  Findings", ""), Heading("3.1", "Weight and balance", size=10.5, rule=False, before=4)]
story.append(P(f"The labelled parts add up to <b>{REC.mass.mass:.2f} kg</b>. The six 5200 mAh batteries are the heaviest group (4.56 kg, 39 %), so where they sit decides the balance."))
story.append(table([["Group", "Count", "Mass"], ["XFLY 80 mm fans", "9", "2.97 kg"], ["Batteries 5200 mAh", "6", "4.56 kg"], ["Jetfoils (920 g + 800 g)", "2", "1.72 kg"],
                    ["ESCs and power boards", "3 + 3", "1.13 kg"], ["Carbon tubes", "8", "0.41 kg"], ["Nose carrier, jetfoil mounts (left side only)", "1 + 2", "0.68 kg"],
                    ["Avionics bay, H-FLOW, 1600 mAh pack", "3", "0.37 kg"], ["Total (labelled parts only)", "", f"{REC.mass.mass:.2f} kg"]], [110, 25, 39], bold_rows=(8,)))
story.append(Spacer(1, 3 * mm))
story.append(P(f"As drawn, the CG sits at the geometric middle of the airframe. That is not where it should be for this aircraft. The jetfoils push along the jets leaving their trailing edges, "
               f"which are far back and low, so the point over which the fans share the load evenly lies about <b>{abs(cg1[0] - cg0[0]) * 1000:.0f} mm further back</b>. "
               f"With the CG there, the busiest fan drops from {U0MAX:.0f} % to {U1MAX:.0f} % of full power and the least loaded one rises from {U0MIN:.0f} % to {U1MIN:.0f} %: more even, more reserve."))
story.append(KeepTogether([to_img(fig_side(), 165), P(f"Figure 1. Side view, nose right. Grey dashed: batteries and CG as drawn. Green: after change C1, and the H-FLOW view direction after change C2 ({TRIM:.1f}° tilt).", st_cap)]))
story.append(KeepTogether([to_img(fig_top(), 96), P("Figure 2. Top view, nose right. The two front batteries stay; the rear block of four moves towards the tail.", st_cap)]))

story.append(KeepTogether([Heading("3.2", "Stability in hover", size=10.5, rule=False, before=4),
    P("Several battery layouts were flown. \"Drift\" is how far the aircraft wanders during a 12 s hover with nobody touching the sticks: a direct measure of how settled it is."),
    to_img(fig_layout_chart(), 174), P("Figure 3. Hover drift for each battery layout. Every layout flew all 9 of its flights without a crash.", st_cap)]))
story.append(table([["Battery layout", "Flights OK", "Drift median / worst", "Pitch hold error", "Busiest fan", "Touchdown speed"]] +
                   [[n, f"{S[k]['ok']}/{S[k]['n']}", f"{S[k]['med']:.2f} / {S[k]['mx']:.2f} m", f"{S[k]['pitch']:.2f}°", f"{S[k]['util']:.0f} %", f"{S[k]['td']:.2f} m/s"] for n, k in LAYOUTS],
                   [62, 18, 28, 22, 20, 24], green_rows=(2,)))
story.append(Spacer(1, 2 * mm))
story.append(P("Moving all six batteries back together would work as well, but the front pair would then sit inside the flight controller. Moving only the rear block gives the best result and fits."))

story += [Heading("3.3", "Control authority", size=10.5, rule=False, before=4)]
story.append(P("Authority is the turning force the fans have in reserve while they hold the aircraft up. It decides how firmly the aircraft can correct itself and follow the pilot."))
story.append(table([["Axis", "Reserve moment", "As acceleration", "Flown test result"],
                    ["Roll (tilt left/right)", f"±{min(LP['roll+_Nm'], LP['roll-_Nm']):.1f} N·m", f"{min(LP['roll+_deg_s2'], LP['roll-_deg_s2']):.0f} °/s²", "reaches the commanded 6° in under 2 s"],
                    ["Pitch (nose up/down)", f"±{min(LP['pitch+_Nm'], LP['pitch-_Nm']):.1f} N·m", f"{min(LP['pitch+_deg_s2'], LP['pitch-_deg_s2']):.0f} °/s²", "reaches the commanded 6° in under 2 s"],
                    ["Yaw (turn on the spot)", f"±{min(LP['yaw+_Nm'], LP['yaw-_Nm']):.1f} N·m", f"{min(LP['yaw+_deg_s2'], LP['yaw-_deg_s2']):.0f} °/s²", f"turns {yaw_r:.0f} °/s right, {yaw_l:.0f} °/s left of 60 °/s asked"],
                    ["4 m/s side wind", "", "", f"attitude held within 2.5°; heading swings {min(gust_yaw):.0f} to {max(gust_yaw):.0f}°"]], [40, 30, 30, 74]))
story.append(Spacer(1, 2 * mm))
story.append(P("Roll and pitch have plenty in reserve. <b>Yaw is the weak axis</b>, about six times weaker than roll, because all nine fans spin the same way. The aircraft weathervanes in a side wind. "
               "This is a known design limit (counter-rotating fan pairs would remove it) and is outside the scope of this report; the battery change does not make it worse."))

story += [Heading("3.4", "Optical flow sensor (H-FLOW)", size=10.5, rule=False, before=4)]
story.append(P("Indoors there is no GPS, so the aircraft holds its position by watching the floor with the H-FLOW camera and measuring its height with the sensor's rangefinder. "
               f"The flight controller assumes the sensor looks straight down while hovering. The aircraft hovers {TRIM:.1f}° nose-up, so the mount must be tilted by the same angle."))
story.append(KeepTogether([to_img(fig_flow_chart(), 174), P("Figure 4. Position-hold drift using only the optical sensor (20 s hands-off hold, then a stick push and stop). For comparison, the same hold on GPS drifts 0.07 m.", st_cap)]))
story.append(table([["Mount", "Drift median / worst", "Height noise", "Flights"],
                    ["Current spot, flat (as drawn)", f"{FL['CAD_flat']['med']:.2f} / {FL['CAD_flat']['mx']:.2f} m", f"{FL['CAD_flat']['alt']:.1f} cm", str(FL['CAD_flat']['n'])],
                    [f"Current spot, tilted {TRIM:.1f}° (option A, recommended)", f"{FL['CAD_tilted']['med']:.2f} / {FL['CAD_tilted']['mx']:.2f} m", f"{FL['CAD_tilted']['alt']:.1f} cm", str(FL['CAD_tilted']['n'])],
                    [f"Under the nose, tilted {TRIM:.1f}° (option B)", f"{FL['nose_tilted']['med']:.2f} / {FL['nose_tilted']['mx']:.2f} m", f"{FL['nose_tilted']['alt']:.1f} cm", str(FL['nose_tilted']['n'])]],
                   [84, 38, 28, 24], green_rows=(2,)))
story.append(Spacer(1, 2 * mm))
story.append(P(f"Tilting the mount is what matters: it cuts the drift by {flow_cut:.0f} % and halves the height noise. Where the sensor sits (under the avionics bay or under the nose) makes no measurable difference in the simulator. "
               "One practical difference remains: the jets. Their air stirs up dust where it hits the floor, and dust in the camera's view can confuse it."))
story.append(KeepTogether([to_img(fig_floor(), 174), P(f"Figure 5. The floor seen from above while hovering. Black dots: where each fan's jet reaches the floor. Green circle: what the H-FLOW sees from its current spot. "
                                                        f"Grey dashed circle: from under the nose. The current spot keeps all jets out of view up to about {H_BELLY:.1f} m hover height; the nose spot up to about {H_NOSE:.1f} m.", st_cap)]))
story.append(PageBreak())

# ---- changes
story += [Heading("4  Required", "CAD changes")]
story.append(P("All positions are in the Fusion design coordinates of the open model, in millimetres: <b>X</b> to the aircraft's right, <b>Y</b> up, <b>Z</b> towards the tail (the nose is at negative Z)."))
story += [Heading("C1", "Battery placement", size=10.5, rule=False, before=4)]
story += numbered(["Select the four rear batteries (the ones at Z = +150 mm), listed below.",
                   "Move them together <b>+195 mm along Z</b> (towards the tail).",
                   "Move the two outer ones (X = ±85 mm) a further <b>-10 mm along Y</b>. Without this they touch the mount parts (All_mounts) near Z 285 to 342 mm.",
                   "Rework the battery tray or straps for the new position. Leave the two front batteries (Z = -150 mm) and the slide-in carrier unchanged."])
occ = {-0.03: "battery_5200mAh__M760g:3", -0.085: "battery_5200mAh__M760g:5", 0.03: "battery_5200mAh__M760g(Mirror):2", 0.085: "battery_5200mAh__M760g(Mirror):3"}
rows = [["Part (occurrence)", "Centre now X, Y, Z", "Centre after", "Move"]]
for c in sorted([c for c in packs(ASIS) if c[0] < 0], key=lambda c: c[1]):
    new = c + np.array([-0.195, 0, 0.01 if abs(c[1]) > 0.06 else 0.0])
    d = fus(new) - fus(c)
    rows.append([occ[round(float(c[1]), 3)], ftxt(c), ftxt(new), f"Z {d[2]:+.0f}" + (f", Y {d[1]:+.0f}" if abs(d[1]) > 0.5 else "")])
story.append(table(rows, [66, 36, 36, 36]))
story.append(Spacer(1, 2 * mm))
story.append(table([["", "CG X, Y, Z (mm)", "Hover angle", "Busiest / least loaded fan"],
                    ["As drawn", ftxt(cg0), f"{TP0:.1f}°", f"{U0MAX:.0f} % / {U0MIN:.0f} %"],
                    ["After C1", ftxt(cg1), f"{TRIM:.1f}°", f"{U1MAX:.0f} % / {U1MIN:.0f} %"]], [40, 50, 34, 50], green_rows=(2,)))

story += [Heading("C2", "H-FLOW mount", size=10.5, rule=False, before=8)]
story += numbered([f"<b>Option A (recommended): keep the current position</b> under the avionics bay, lens centre at about X {fus(HF_CAD)[0]:.0f}, Y {fus(HF_CAD - [0, 0, 0.012])[1]:.0f}, Z {fus(HF_CAD)[2]:.0f} mm. "
                   f"Rotate the mount <b>{TRIM:.1f}° about the X axis</b> so that the lens turns from pointing straight down (-Y) towards the tail (+Z). On the bench the lens then looks slightly backwards; in the hover it looks straight down.",
                   f"<b>Option B, only if the aircraft will hover higher than about {H_BELLY:.1f} m:</b> under the nose, lens centre at about X 0, Y {fus(HF_NOSE)[1]:.0f}, Z {fus(HF_NOSE)[2]:.0f} mm, same {TRIM:.1f}° tilt. "
                   "The nose ESC (ESC__M200g:2) sits directly above this spot: mount the sensor on fixed structure next to it, not on the removable slide-in carrier.",
                   "In both cases keep a clear 42° cone below the lens (no legs, straps or cables in view) and at least 80 mm between the lens and the floor when parked (currently 386 mm).",
                   "If a later revision changes the hover angle, the tilt changes with it: tilt = hover angle."])

story += [Heading("C3", "Weight labels", size=10.5, rule=False, before=8)]
story.append(P("The simulator only counts labelled parts. Until these are fixed, the real aircraft is heavier than the model and slightly unbalanced to the left (4 mm)."))
story.append(table([["Part", "Problem", "Action"],
                    ["All_mounts (14 bodies, 909 cm³)", "No weight label; the largest part not counted", "Weigh and add __M&lt;grams&gt;g"],
                    ["Four_feet_assembled", "No weight label", "Add label"],
                    ["Clamp_cap_1, Clamp_cap_3 and their mirrors", "No weight label", "Add label"],
                    ["Open CASCADE STEP translator 7.9 1.9(Mirror) and 1.11(Mirror)", "Right-hand jetfoil mounts without labels: 280 g missing on the right", "Rename to Jetfoil_mount_1__M120g(Mirror) and Jetfoil_mount_2__M160g(Mirror)"],
                    ["JET_FOIL_V3__M920g and JET_FOIL_V3__M800g(Mirror)", "Same shape, different weights: 120 g difference left to right", "Weigh both and correct"],
                    ["16x1000mm_carbon_tube(Mirror) ×2, 16x500mm_carbon_tube(Mirror), XFLY 80mm EDF(Mirror), battery_5200mAh(Mirror) ×3, PDB_XT90PW-F_CASE(Mirror)",
                     "Unlabelled duplicates on top of labelled parts; a mass report from Fusion counts them twice (9 batteries instead of 6)", "Delete, or exclude from the design"],
                    ["XFLY 80mm EDF_M330g(Mirror) ×4", "Label written with a single underscore", "Rename to __M330g"]], [62, 60, 52]))

story += [Heading("Keep", "as drawn", size=10.5, rule=False, before=8)]
story.append(P("<b>Jetfoil angles (69° / 59° / 49°) and the nose fans.</b> A steeper inner jetfoil was tested (72° and 75°); it flew no better at 72° and wandered twice as much at 75°."))

story += [Heading("5  Limitations", "")]
story += bullets(["Fan thrust (36 N each) and fan torque are taken from the ATLAS_09B aircraft, not measured on V3.",
                  "The jet is assumed to follow each jetfoil all the way to its trailing edge. A real jet may leave the surface earlier and turn less.",
                  "Only labelled parts are weighed (change C3). After relabelling, the CG will move; the study should be rerun before building.",
                  "The optical sensor model sees an ideal, textured floor. Dust, lighting and vibration of the lens are not simulated; the jet check in figure 5 is geometric only. The sensor's field of view is taken as 42°.",
                  "The legs are the simulator's provisional ones, parked at the hover angle."])

story += [Heading("Appendix A", "Terms used", size=11)]
story.append(table([["Term", "Meaning"],
                    ["CG (centre of gravity)", "The balance point of the aircraft: where all its weight acts."],
                    ["Hover angle (trim pitch)", f"The nose-up angle the aircraft holds when hovering, set by its geometry and CG ({TRIM:.1f}° after C1)."],
                    ["Fan load", "How hard a fan works in the hover, in % of its full power. Lower and more even means more reserve."],
                    ["Drift", "How far the aircraft wanders while hovering with nobody touching the sticks."],
                    ["Pitch hold error", "How far the nose angle strays from the one asked for, on average."],
                    ["Authority", "The turning force the fans can spare while holding the aircraft up."],
                    ["Position hold", "Flight mode in which the aircraft stays in place by itself; indoors it relies on the H-FLOW."],
                    ["Weathervaning", "The aircraft turning its nose into or away from the wind because it cannot resist the wind's twisting force."],
                    ["SITL", "Software in the loop: the real flight-control software flying the simulated aircraft."]], [44, 130]))
story += [Heading("Appendix B", "Coordinates", size=11)]
story.append(P("The simulator uses x forward, y right, z down, with the origin on the centreline at Fusion Y = 450 mm. Conversion to Fusion (mm): "
               "Fusion X = 1000·y, Fusion Y = 450 - 1000·z, Fusion Z = -1000·x. All numbers in section 4 are already converted.", st_small))

Path(A.out).parent.mkdir(parents=True, exist_ok=True)
doc = BaseDocTemplate(A.out, pagesize=A4, leftMargin=M, rightMargin=M, topMargin=27 * mm, bottomMargin=20 * mm,
                      title="V3 Small Scale: Simulation review and CAD corrections", author="S. Fragkoulis", subject="SMALL_SCALE_V3 v34")
doc.addPageTemplates([PageTemplate(id="p", frames=[Frame(M, 20 * mm, W - 2 * M, H - 47 * mm, id="f", leftPadding=0, rightPadding=0, topPadding=0, bottomPadding=0)], onPage=on_page)])
doc.build(story)
print("wrote", A.out)
