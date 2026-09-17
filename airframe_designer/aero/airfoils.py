"""Airfoil sections and their polars (CL, CD, CM against angle of attack and Reynolds number).

Coordinates come from, in order: a local ``airfoils/<name>.dat`` (Selig or Lednicer format, e.g. copied from the
UIUC database), a NACA 4- or 5-digit name generated analytically (``naca2412``, ``naca23006``), or a download
from the UIUC Airfoil Coordinates Database (https://m-selig.ae.illinois.edu/ads/coord/<name>.dat), cached locally.

Polars come from XFOIL when an ``xfoil`` binary is on the PATH, otherwise from NeuralFoil (a neural surrogate
trained on XFOIL results, pip ``neuralfoil``; no binary needed). Either way the result is a table over a fine
angle-of-attack grid and a set of Reynolds numbers, stored as ``airfoils/polars/<name>.json`` and looked up at
run time by bilinear interpolation, so the wing sees the section's real lift, drag and moment at the angle of
attack and speed it is flying at. Outside the range XFOIL/NeuralFoil is valid for (roughly -20..+25 degrees) the
coefficients blend into a flat plate, as the simpler models do.
"""
from __future__ import annotations

import json
import math
import os
import re
import shutil
import subprocess
import tempfile
import time
import urllib.request
from pathlib import Path

import numpy as np

AIRFOIL_DIR = Path(__file__).resolve().parents[2] / "airfoils"
POLAR_DIR = AIRFOIL_DIR / "polars"
UIUC_URL = "https://m-selig.ae.illinois.edu/ads/coord/{name}.dat"
NU_AIR = 1.46e-5                       # kinematic viscosity of air at sea level, m^2/s
DEFAULT_RE = [5e4, 1e5, 2e5, 5e5, 1e6, 2e6, 5e6]
ALPHA_STEP = 0.5
POLAR_ALPHA_RANGE = (-24.0, 28.0)      # range asked from XFOIL / NeuralFoil; beyond it a flat plate takes over


def normalise_name(name: str) -> str:
    return re.sub(r"[^a-z0-9_-]", "", name.strip().lower().replace(" ", ""))


# --------------------------------------------------------------------- NACA generators
def _naca_thickness(x: np.ndarray, t: float) -> np.ndarray:
    return 5 * t * (0.2969 * np.sqrt(x) - 0.1260 * x - 0.3516 * x ** 2 + 0.2843 * x ** 3 - 0.1015 * x ** 4)


def naca_coordinates(name: str, n: int = 100) -> np.ndarray | None:
    """Selig-ordered coordinates (upper TE -> LE -> lower TE) for NACA 4-digit (2412) and 5-digit (23012) names."""
    m = re.fullmatch(r"naca(\d{4,5})", name)
    if not m:
        return None
    d = m.group(1)
    beta = np.linspace(0.0, math.pi, n)
    x = 0.5 * (1 - np.cos(beta))                  # cosine spacing
    if len(d) == 4:
        mm, p, t = int(d[0]) / 100, int(d[1]) / 10, int(d[2:]) / 100
        yc = np.where(x < p, mm / max(p * p, 1e-9) * (2 * p * x - x * x), mm / max((1 - p) ** 2, 1e-9) * ((1 - 2 * p) + 2 * p * x - x * x)) if p > 0 else np.zeros_like(x)
        dyc = np.where(x < p, 2 * mm / max(p * p, 1e-9) * (p - x), 2 * mm / max((1 - p) ** 2, 1e-9) * (p - x)) if p > 0 else np.zeros_like(x)
    else:
        L, P, Q, t = int(d[0]), int(d[1]), int(d[2]), int(d[3:]) / 100
        # standard 5-digit camber line (Q = 0): tabulated r and k1 for P = 1..5 (2xx: L=2 gives CL_design 0.3)
        table = {1: (0.0580, 361.4), 2: (0.1260, 51.64), 3: (0.2025, 15.957), 4: (0.2900, 6.643), 5: (0.3910, 3.230)}
        if Q != 0 or P not in table:
            return None
        r, k1 = table[P]
        k1 = k1 * L / 2.0                          # scale the design lift with the first digit
        yc = np.where(x < r, k1 / 6 * (x ** 3 - 3 * r * x * x + r * r * (3 - r) * x), k1 * r ** 3 / 6 * (1 - x))
        dyc = np.where(x < r, k1 / 6 * (3 * x * x - 6 * r * x + r * r * (3 - r)), np.full_like(x, -k1 * r ** 3 / 6))
    yt = _naca_thickness(x, t)
    th = np.arctan(dyc)
    xu, yu = x - yt * np.sin(th), yc + yt * np.cos(th)
    xl, yl = x + yt * np.sin(th), yc - yt * np.cos(th)
    upper = np.column_stack([xu, yu])[::-1]       # TE -> LE
    lower = np.column_stack([xl, yl])[1:]         # LE -> TE
    return np.vstack([upper, lower])


# --------------------------------------------------------------------- coordinate files
def parse_dat(text: str) -> np.ndarray:
    """Selig (single loop from TE over the top to the LE and back) or Lednicer (two surfaces LE->TE) format."""
    lines = [l.strip() for l in text.splitlines() if l.strip()]
    pts = []
    for l in lines[1:] if not re.match(r"^\s*-?\d", lines[0]) else lines:
        parts = l.replace(",", " ").split()
        try:
            pts.append((float(parts[0]), float(parts[1])))
        except (ValueError, IndexError):
            continue
    a = np.array(pts, float)
    if len(a) and a[0, 0] > 1.5:                   # Lednicer header line: point counts
        n_up = int(a[0, 0]); a = a[1:]
        upper, lower = a[:n_up], a[n_up:]
        return np.vstack([upper[::-1], lower[1:]])
    return a


def load_coordinates(name: str, allow_download: bool = True) -> np.ndarray:
    name = normalise_name(name)
    local = AIRFOIL_DIR / f"{name}.dat"
    if local.is_file():
        return parse_dat(local.read_text())
    gen = naca_coordinates(name)
    if gen is not None:
        return gen
    if allow_download:
        AIRFOIL_DIR.mkdir(parents=True, exist_ok=True)
        url = UIUC_URL.format(name=name)
        try:
            text = None
            for attempt in range(2):
                try:
                    with urllib.request.urlopen(url, timeout=30) as r:
                        text = r.read().decode("latin-1")
                    break
                except Exception as e:
                    if attempt == 1:
                        raise
        except Exception as e:
            raise FileNotFoundError(f"airfoil '{name}': no local file, not a NACA 4/5-digit name, and the UIUC download failed ({e})")
        pts = parse_dat(text)
        if len(pts) < 10:
            raise FileNotFoundError(f"airfoil '{name}': UIUC returned no usable coordinates")
        local.write_text(text)
        return pts
    raise FileNotFoundError(f"airfoil '{name}' not found")


def list_airfoils() -> dict:
    files = sorted(p.stem for p in AIRFOIL_DIR.glob("*.dat")) if AIRFOIL_DIR.is_dir() else []
    polars = sorted(p.stem for p in POLAR_DIR.glob("*.json")) if POLAR_DIR.is_dir() else []
    return {"coordinates": files, "polars": polars, "xfoil": shutil.which("xfoil") is not None}


# --------------------------------------------------------------------- polar generation
def _neuralfoil_polar(coords: np.ndarray, alphas: np.ndarray, re_list: list[float], ncrit: float) -> dict:
    import neuralfoil as nf
    cl, cd, cm, conf = [], [], [], []
    for Re in re_list:
        out = nf.get_aero_from_coordinates(coords, alpha=alphas, Re=float(Re), n_crit=ncrit, model_size="xlarge")
        cl.append(np.asarray(out["CL"], float)); cd.append(np.asarray(out["CD"], float)); cm.append(np.asarray(out["CM"], float))
        conf.append(np.asarray(out.get("analysis_confidence", np.ones_like(alphas)), float))
    return {"cl": cl, "cd": cd, "cm": cm, "confidence": conf, "source": "neuralfoil"}


def _xfoil_polar(coords: np.ndarray, alphas: np.ndarray, re_list: list[float], ncrit: float) -> dict:
    """Run the XFOIL binary (pacc polars), one run per Reynolds number; missing angles (non-converged) are
    filled by interpolation. Used automatically when ``xfoil`` is on the PATH."""
    exe = shutil.which("xfoil")
    cl, cd, cm, conf = [], [], [], []
    with tempfile.TemporaryDirectory() as td:
        cfile = Path(td) / "foil.dat"
        cfile.write_text("foil\n" + "\n".join(f"{x:.6f} {y:.6f}" for x, y in coords))
        for Re in re_list:
            pfile = Path(td) / f"polar_{int(Re)}.txt"
            a0, a1 = float(alphas[0]), float(alphas[-1])
            script = f"load {cfile}\npane\noper\nvisc {Re:.0f}\nvpar\nn {ncrit}\n\niter 200\npacc\n{pfile}\n\naseq 0 {a1} {ALPHA_STEP}\naseq 0 {a0} {-ALPHA_STEP}\npacc\n\nquit\n"
            try:
                subprocess.run([exe], input=script, capture_output=True, text=True, timeout=180, cwd=td)
            except Exception:
                pass
            pts = {}
            if pfile.is_file():
                for line in pfile.read_text().splitlines():
                    parts = line.split()
                    try:
                        a, l, d, _, m = (float(v) for v in parts[:5])
                        pts[round(a, 2)] = (l, d, m)
                    except (ValueError, IndexError):
                        continue
            if len(pts) < 5:
                raise RuntimeError(f"XFOIL produced no polar at Re={Re:.0f}")
            xs = np.array(sorted(pts)); ys = np.array([pts[a] for a in xs])
            cl.append(np.interp(alphas, xs, ys[:, 0])); cd.append(np.interp(alphas, xs, ys[:, 1])); cm.append(np.interp(alphas, xs, ys[:, 2]))
            conf.append(np.where((alphas >= xs[0]) & (alphas <= xs[-1]), 1.0, 0.0))
    return {"cl": cl, "cd": cd, "cm": cm, "confidence": conf, "source": "xfoil"}


def build_polar(name: str, re_list: list[float] | None = None, ncrit: float = 9.0, source: str = "auto",
                force: bool = False) -> dict:
    """Build (or load from cache) the polar table of an airfoil. Returns the table dict."""
    name = normalise_name(name)
    POLAR_DIR.mkdir(parents=True, exist_ok=True)
    cache = POLAR_DIR / f"{name}.json"
    if cache.is_file() and not force:
        d = json.loads(cache.read_text())
        if d.get("ncrit") == ncrit and (source == "auto" or d.get("source") == source):
            return d
    coords = load_coordinates(name)
    re_list = [float(r) for r in (re_list or DEFAULT_RE)]
    alphas = np.arange(POLAR_ALPHA_RANGE[0], POLAR_ALPHA_RANGE[1] + ALPHA_STEP / 2, ALPHA_STEP)
    use_xfoil = (source == "xfoil") or (source == "auto" and shutil.which("xfoil") is not None)
    t0 = time.time()
    if use_xfoil:
        try:
            res = _xfoil_polar(coords, alphas, re_list, ncrit)
        except Exception as e:
            if source == "xfoil":
                raise
            res = _neuralfoil_polar(coords, alphas, re_list, ncrit)
            res["note"] = f"xfoil failed ({e}); NeuralFoil used"
    else:
        res = _neuralfoil_polar(coords, alphas, re_list, ncrit)
    # valid angle range per Re: where the analysis is trusted (confidence) and before the post-stall collapse
    # valid angle range per Re: from alpha = 0 outwards up to the stall, i.e. the first clear drop of CL after its
    # local maximum (XFOIL/NeuralFoil data beyond that is not trustworthy: the surrogate can even show CL rising
    # again at 25 degrees). Past the stall the section blends into a flat plate.
    valid = []
    for k in range(len(re_list)):
        cl_k = np.asarray(res["cl"][k]); c = np.asarray(res["confidence"][k])
        i0 = int(np.argmin(np.abs(alphas)))

        def edge(step):
            i = i0; best = cl_k[i0] * (1 if step > 0 else -1); i_best = i0
            while 0 <= i + step < len(alphas):
                i += step
                v = cl_k[i] * (1 if step > 0 else -1)
                if v > best:
                    best, i_best = v, i
                elif best - v > 0.04 or c[i] < 0.3:        # dropped clearly below the peak: stalled
                    break
            return i_best
        hi = float(alphas[edge(+1)]) + 1.0
        lo = float(alphas[edge(-1)]) - 1.0
        valid.append([max(lo, float(alphas[0])), min(hi, float(alphas[-1]))])
    table = {"name": name, "source": res["source"], "ncrit": ncrit, "re": re_list, "alpha": alphas.tolist(),
             "cl": [np.round(v, 5).tolist() for v in res["cl"]], "cd": [np.round(v, 6).tolist() for v in res["cd"]],
             "cm": [np.round(v, 5).tolist() for v in res["cm"]], "valid": valid, "built_s": round(time.time() - t0, 1),
             "n_points": int(len(coords))}
    if res.get("note"):
        table["note"] = res["note"]
    cache.write_text(json.dumps(table))
    return table


def polar_summary(table: dict, Re: float = 5e5) -> dict:
    """Headline numbers at one Reynolds number: CL max and its angle, CL at 0, zero-lift angle, min CD, L/D max."""
    p = Polar(table)
    a = np.arange(-15.0, 25.0, 0.25)
    cl, cd, cm = p.lookup(np.radians(a), np.full_like(a, Re))
    i = int(np.argmax(cl)); ld = cl / np.maximum(cd, 1e-6); j = int(np.argmax(ld))
    zero = float(np.interp(0.0, cl[:i], a[:i])) if i > 1 else None
    return {"re": Re, "cl_max": round(float(cl[i]), 3), "alpha_cl_max_deg": float(a[i]), "cl0": round(float(np.interp(0.0, a, cl)), 3),
            "alpha_zero_lift_deg": None if zero is None else round(zero, 2), "cd_min": round(float(cd.min()), 5),
            "ld_max": round(float(ld[j]), 1), "alpha_ld_max_deg": float(a[j]), "cm0": round(float(np.interp(0.0, a, cm)), 4),
            "source": table.get("source"), "valid_deg": table.get("valid", [[None, None]])[min(len(table.get("re", [])) - 1, 3)]}


# --------------------------------------------------------------------- run-time lookup
class Polar:
    """Bilinear interpolation of a polar table in (alpha, log Re). Beyond the valid angle range the section blends
    into a flat plate over 15 degrees (CL = 1.2 sin a cos a, CD = CD_min + 1.2 sin^2 a), like the simple models."""

    def __init__(self, table: dict):
        self.name = table["name"]
        self.alpha = np.radians(np.asarray(table["alpha"], float))
        self.log_re = np.log(np.asarray(table["re"], float))
        self.cl = np.asarray(table["cl"], float); self.cd = np.asarray(table["cd"], float); self.cm = np.asarray(table["cm"], float)
        self.valid = np.radians(np.asarray(table.get("valid") or [[self.alpha[0], self.alpha[-1]]] * len(self.log_re), float))
        self.cd_min = float(self.cd.min())
        self.da = float(self.alpha[1] - self.alpha[0])
        self.blend = math.radians(15.0)

    def prepare(self, Re: np.ndarray) -> tuple:
        """Reynolds-number interpolation weights and valid angle range for a set of strips (once per step)."""
        Re = np.asarray(Re, float)
        lre = np.clip(np.log(np.maximum(Re, 1.0)), self.log_re[0], self.log_re[-1])
        if len(self.log_re) > 1:
            j = np.clip(np.searchsorted(self.log_re, lre) - 1, 0, len(self.log_re) - 2)
            fr = (lre - self.log_re[j]) / (self.log_re[j + 1] - self.log_re[j])
            lo = (1 - fr) * self.valid[j, 0] + fr * self.valid[j + 1, 0]
            hi = (1 - fr) * self.valid[j, 1] + fr * self.valid[j + 1, 1]
        else:
            j = np.zeros(len(Re), int); fr = np.zeros(len(Re)); lo = np.full(len(Re), self.valid[0, 0]); hi = np.full(len(Re), self.valid[0, 1])
        return j, fr, lo, hi

    def lookup(self, alpha: np.ndarray, Re: np.ndarray, prep: tuple | None = None) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        alpha = np.asarray(alpha, float)
        j, fr, lo, hi = prep if prep is not None else self.prepare(Re)
        a_c = np.minimum(np.maximum(alpha, lo), hi)                 # section data frozen at the edge of validity
        fi = np.clip((a_c - self.alpha[0]) / self.da, 0, len(self.alpha) - 1.001)
        i = fi.astype(int); fa = fi - i
        w0 = 1.0 - fa
        if len(self.log_re) > 1:
            j1 = j + 1; f0 = 1.0 - fr
            cl = f0 * (self.cl[j, i] * w0 + self.cl[j, i + 1] * fa) + fr * (self.cl[j1, i] * w0 + self.cl[j1, i + 1] * fa)
            cd = f0 * (self.cd[j, i] * w0 + self.cd[j, i + 1] * fa) + fr * (self.cd[j1, i] * w0 + self.cd[j1, i + 1] * fa)
            cm = f0 * (self.cm[j, i] * w0 + self.cm[j, i + 1] * fa) + fr * (self.cm[j1, i] * w0 + self.cm[j1, i + 1] * fa)
        else:
            cl = self.cl[j, i] * w0 + self.cl[j, i + 1] * fa; cd = self.cd[j, i] * w0 + self.cd[j, i + 1] * fa; cm = self.cm[j, i] * w0 + self.cm[j, i + 1] * fa
        # flat-plate blend beyond the valid range (only computed when some strip is outside it)
        over = np.maximum(alpha - hi, lo - alpha)
        if over.max() > 0.0:
            t = np.clip(over / self.blend, 0.0, 1.0)
            sa, ca = np.sin(alpha), np.cos(alpha)
            cl = (1 - t) * cl + t * (1.2 * sa * ca)
            cd = (1 - t) * cd + t * (self.cd_min + 1.2 * sa * sa)
            cm = (1 - t) * cm
        return cl, cd, cm


_POLARS: dict[str, Polar] = {}


def get_polar(name: str, ncrit: float = 9.0, source: str = "auto") -> Polar:
    key = f"{normalise_name(name)}:{ncrit}:{source}"
    if key not in _POLARS:
        _POLARS[key] = Polar(build_polar(name, ncrit=ncrit, source=source))
    return _POLARS[key]


def ensure_polars(airframe, log=None) -> list[str]:
    """Build/cache the polars every 'polar' wing of an airframe needs (call before creating the physics)."""
    built = []
    for w in getattr(airframe, "wings", []):
        a = w.aero
        if getattr(a, "model", "") == "polar":
            for n in (a.airfoil_root, a.airfoil_tip or a.airfoil_root):
                if n:
                    t0 = time.time()
                    get_polar(n, a.ncrit, a.polar_source)
                    if time.time() - t0 > 0.5 and log:
                        log(f"[airfoil] polar for {n} built in {time.time() - t0:.1f} s")
                    built.append(n)
    return built
