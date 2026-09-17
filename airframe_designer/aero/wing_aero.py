"""Strip-theory wing aerodynamics.

Every wing is cut into spanwise strips (geometry.wings.wing_panels). For each strip the local relative wind
(vehicle velocity through the air plus the rotation-induced velocity at the strip) is projected onto the strip's
chordwise / normal axes, giving its own angle of attack. Lift, drag and the section pitching moment act at the
strip's quarter-chord point. Summing strips gives, for free: dihedral effect (rolling moment from sideslip),
roll damping, pitch damping of a tailplane, weathercock stability of a fin, and stalled tips before roots on a
twisted wing.

Section models
--------------
linear     CL = cl0 + a3d * alpha, a3d = Helmbold(cl_alpha, AR); CD = cd0 + CL^2 / (pi e AR);
           past the stall the coefficients blend into a flat plate over ``stall_blend_deg``.
polhamus   sharp-edged delta: CL = Kp sin(a) cos^2(a) + Kv cos(a) sin^2(a), CD = cd0 + CL tan(a),
           Kp = Helmbold slope, Kv = pi (vortex lift). Same flat-plate blend past the stall.
"""
from __future__ import annotations

import math

import numpy as np

from ..geometry.wings import Wing, wing_panels
from .atmosphere import RHO

NU_AIR = 1.46e-5


def normalise(name: str) -> str:
    return name.strip().lower().replace(" ", "")


def helmbold_slope(cl_alpha_2d: float, ar: float) -> float:
    """3-D lift-curve slope of a finite wing (1/rad)."""
    if ar <= 1e-6:
        return 0.0
    k = cl_alpha_2d / (math.pi * ar)
    return cl_alpha_2d / (math.sqrt(1.0 + k * k) + k)


def section_coefficients(alpha: np.ndarray, model: str, a3d, cl0, cd0, oswald, ar, stall, blend, cd_flat, kv) -> tuple[np.ndarray, np.ndarray]:
    """Vectorised CL, CD for an array of angles of attack (rad). The per-strip coefficient arguments may be
    scalars or arrays of alpha's shape."""
    alpha = np.asarray(alpha, float)
    sgn = np.where(alpha >= 0, 1.0, -1.0)
    a = np.abs(alpha)
    a_s = np.minimum(a, stall)                     # pre-stall value frozen at the stall angle
    if model == "polhamus":
        def pre(x):
            cl = a3d * np.sin(x) * np.cos(x) ** 2 + kv * np.cos(x) * np.sin(x) ** 2
            return cl, cd0 + cl * np.tan(np.minimum(x, math.radians(89.0)))
        cl_pre, cd_pre = pre(a)
        cl_s, cd_s = pre(a_s)
        cl_pre, cl_s = sgn * cl_pre, sgn * cl_s
    else:
        def pre(x):
            cl = cl0 + a3d * x
            return cl, cd0 + cl * cl / (math.pi * np.maximum(oswald, 1e-3) * np.maximum(ar, 1e-3))
        cl_pre, cd_pre = pre(alpha)
        cl_s, cd_s = pre(sgn * a_s)
    t = np.clip((a - stall) / np.maximum(blend, 1e-6), 0.0, 1.0)
    cl_flat = cd_flat * np.sin(alpha) * np.cos(alpha)
    cd_flat_v = cd0 + cd_flat * np.sin(a) ** 2
    cl = np.where(a <= stall, cl_pre, (1 - t) * cl_s + t * cl_flat)
    cd = np.where(a <= stall, cd_pre, (1 - t) * cd_s + t * cd_flat_v)
    return cl, cd


class WingSet:
    """All enabled wings of an airframe, flattened into strip arrays. Per-strip CL/CD tables over the angle of
    attack are built once (0.25 degree resolution, linear interpolation) so a force evaluation is a handful of
    vector operations."""

    TABLE_STEP_DEG = 0.25

    def __init__(self, wings: list[Wing], cg):
        from .fastmath import skew_stack
        cg = np.asarray(cg, float)
        parts = [wing_panels(w) for w in wings]
        self.n = sum(len(p["area"]) for p in parts)
        self.wings = wings
        if self.n == 0:
            return
        self.cg = cg
        self.r = np.concatenate([p["pos"] for p in parts]) - cg           # strip quarter-chord relative to the CG
        self.e_c = np.concatenate([p["e_c"] for p in parts])
        self.e_n = np.concatenate([p["e_n"] for p in parts])
        self.e_d = -self.e_n
        self.pitch_axis = np.concatenate([p["pitch_axis"] for p in parts])
        self.area = np.concatenate([p["area"] for p in parts])
        self.chord = np.concatenate([p["chord"] for p in parts])
        self.wing_index = np.concatenate([np.full(len(p["area"]), i) for i, p in enumerate(parts)])

        def per(fn):
            return np.concatenate([np.full(len(p["area"]), fn(w)) for w, p in zip(wings, parts)])
        self.model_polhamus = per(lambda w: 1.0 if w.aero.model == "polhamus" else 0.0) > 0.5
        self.model_polar = per(lambda w: 1.0 if w.aero.model == "polar" else 0.0) > 0.5
        self.eta = np.concatenate([p["eta"] for p in parts])              # spanwise station 0 (root) .. 1 (tip)
        self.polar_wings: list[dict] = []                                  # per polar wing: strips, polars, AR, e
        if self.model_polar.any():
            from .airfoils import get_polar
            for i, w in enumerate(wings):
                if w.aero.model != "polar":
                    continue
                m = self.wing_index == i
                root = w.aero.airfoil_root or "naca0012"
                tip = w.aero.airfoil_tip or root
                self.polar_wings.append({"mask": m, "idx": np.nonzero(m)[0], "root": get_polar(root, w.aero.ncrit, w.aero.polar_source),
                                         "tip": get_polar(tip, w.aero.ncrit, w.aero.polar_source), "same": normalise(root) == normalise(tip),
                                         "ar": max(w.ar, 0.5), "e": max(w.aero.oswald, 0.3), "cl_prev": 0.0, "name": w.name})
        self.a3d = per(lambda w: helmbold_slope(w.aero.cl_alpha, w.ar))
        self.cl0 = per(lambda w: w.aero.cl0)
        self.cd0 = per(lambda w: w.aero.cd0)
        self.oswald = per(lambda w: w.aero.oswald)
        self.ar = per(lambda w: w.ar)
        self.stall = per(lambda w: math.radians(w.aero.stall_deg))
        self.blend = per(lambda w: math.radians(max(w.aero.stall_blend_deg, 0.1)))
        self.cd_flat = per(lambda w: w.aero.cd_flat)
        self.cm0 = per(lambda w: w.aero.cm0)
        self.kv = per(lambda w: math.pi if w.aero.vortex_lift else 0.0)
        self.mixed = bool(self.model_polhamus.any() and not self.model_polhamus.all())
        # precomputed products for the force/moment assembly
        self.S = skew_stack(self.r)                                  # r x (.)
        self.RC = np.cross(self.r, self.e_c)                          # r x e_c
        self.RD = np.cross(self.r, self.e_d)                          # r x e_d
        self.cm_term = self.cm0 * self.chord                          # cm0 * c (times q at run time)
        self.half_rho_area = 0.5 * RHO * self.area
        self.rows = np.arange(self.n)
        # area-weighted alpha per wing
        W = np.zeros((len(wings), self.n))
        for i in range(len(wings)):
            m = self.wing_index == i
            if self.area[m].sum() > 0:
                W[i, m] = self.area[m] / self.area[m].sum()
        self.alpha_weights = W
        # coefficient tables
        step = math.radians(self.TABLE_STEP_DEG)
        self.table_alpha = np.arange(-math.pi, math.pi + step / 2, step)
        K = len(self.table_alpha)
        self.inv_step = 1.0 / step
        self.K = K
        self.cl_tab = np.zeros((self.n, K)); self.cd_tab = np.zeros((self.n, K))
        for i in range(self.n):
            model = "polhamus" if self.model_polhamus[i] else "linear"
            cl, cd = section_coefficients(self.table_alpha, model, self.a3d[i], self.cl0[i], self.cd0[i], self.oswald[i],
                                          self.ar[i], self.stall[i], self.blend[i], self.cd_flat[i], self.kv[i])
            self.cl_tab[i], self.cd_tab[i] = cl, cd

    def coefficients(self, alpha: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Per-strip CL, CD by table lookup (alpha in rad, array of length n)."""
        fi = (np.asarray(alpha, float) + math.pi) * self.inv_step
        i0 = np.clip(fi.astype(int), 0, self.K - 2)
        fr = fi - i0
        cl = self.cl_tab[self.rows, i0] * (1.0 - fr) + self.cl_tab[self.rows, i0 + 1] * fr
        cd = self.cd_tab[self.rows, i0] * (1.0 - fr) + self.cd_tab[self.rows, i0 + 1] * fr
        return cl, cd

    def coefficients_exact(self, alpha: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """The analytic coefficients the tables are built from (for tests / plots)."""
        cl = np.zeros_like(alpha, dtype=float); cd = np.zeros_like(alpha, dtype=float)
        for model, mask in (("polhamus", self.model_polhamus), ("linear", ~self.model_polhamus)):
            if mask.any():
                c1, c2 = section_coefficients(alpha[mask], model, self.a3d[mask], self.cl0[mask], self.cd0[mask],
                                              self.oswald[mask], self.ar[mask], self.stall[mask], self.blend[mask],
                                              self.cd_flat[mask], self.kv[mask])
                cl[mask], cd[mask] = c1, c2
        return cl, cd

    def polar_coefficients(self, alpha: np.ndarray, speed: np.ndarray, cl: np.ndarray, cd: np.ndarray, cm: np.ndarray) -> None:
        """Fill cl/cd/cm for the strips of 'polar' wings from their root/tip section data at the strip's angle of
        attack and Reynolds number. Finite-wing effect: the wing's lift induces a downwash angle
        alpha_i = CL_wing / (pi e AR), which reduces every section's effective angle and adds induced drag
        cl * alpha_i; one warm-started fixed-point iteration per call."""
        for pw in self.polar_wings:
            idx = pw["idx"]
            a_s = alpha[idx]; Re = speed[idx] * self.chord[idx] / NU_AIR
            # sub-steps 1 ms apart rarely change the state enough to matter: reuse the last lookup while every
            # strip's angle moved less than 0.05 deg and its speed less than 0.5%
            last = pw.get("last")
            if last is not None and np.abs(a_s - last[0]).max() < 8.7e-4 and np.abs(speed[idx] - last[1]).max() <= 0.005 * max(float(last[1].max()), 1e-3):
                cl[idx], cd[idx], cm[idx] = last[2], last[3], last[4]
                continue
            area = self.area[idx]
            k = 1.0 / (math.pi * pw["e"] * pw["ar"])
            cl_wing = pw["cl_prev"]
            wts = area / area.sum() if area.sum() > 0 else np.full(len(idx), 1.0 / max(len(idx), 1))
            damp = 1.0 / (1.0 + 2.0 * math.pi * k)      # Newton-like damping: the plain fixed point diverges for low AR
            eta = self.eta[idx]
            prep_r = pw["root"].prepare(Re)
            prep_t = None if pw["same"] else pw["tip"].prepare(Re)
            for _ in range(2):                            # warm-started from the previous step: two passes converge
                a_eff = a_s - cl_wing * k
                if pw["same"]:
                    cls_, cds_, cms_ = pw["root"].lookup(a_eff, Re, prep_r)
                else:
                    clr, cdr, cmr = pw["root"].lookup(a_eff, Re, prep_r)
                    clt, cdt, cmt = pw["tip"].lookup(a_eff, Re, prep_t)
                    cls_, cds_, cms_ = clr * (1 - eta) + clt * eta, cdr * (1 - eta) + cdt * eta, cmr * (1 - eta) + cmt * eta
                cl_new = float(cls_ @ wts)
                cl_wing = cl_wing + damp * (cl_new - cl_wing)
            pw["cl_prev"] = cl_wing
            cl[idx] = cls_
            cd[idx] = cds_ + cls_ * (cl_wing * k)          # profile + induced
            cm[idx] = cms_
            pw["last"] = (a_s.copy(), speed[idx].copy(), cl[idx].copy(), cd[idx].copy(), cm[idx].copy())

    def forces(self, v_air_body: np.ndarray, rates: np.ndarray, detail: bool = True) -> tuple[np.ndarray, np.ndarray, dict]:
        """Total force and moment about the CG (structural frame) from the vehicle velocity through the air
        ``v_air_body`` and body rates. Also returns a breakdown (lift, drag, per-wing alpha) when ``detail``."""
        if self.n == 0:
            return np.zeros(3), np.zeros(3), {"lift": 0.0, "drag": 0.0, "alpha": [], "stalled": False, "wings": []}
        V = v_air_body - np.einsum("nij,j->ni", self.S, rates)            # v + w x r  (w x r = -(r x w))
        u = np.einsum("ij,ij->i", V, self.e_c)
        w = np.einsum("ij,ij->i", V, self.e_d)
        v2 = u * u + w * w
        alpha = np.arctan2(w, u)
        speed = np.sqrt(v2)
        cl, cd = self.coefficients(alpha)
        cm = self.cm0
        if self.polar_wings:
            cl = cl.copy(); cd = cd.copy(); cm = self.cm0.copy()
            self.polar_coefficients(alpha, speed, cl, cd, cm)
        qv = self.half_rho_area * speed                                     # q / V
        a = qv * (cl * w - cd * u)
        b = -qv * (cl * u + cd * w)
        F = a @ self.e_c + b @ self.e_d
        M = a @ self.RC + b @ self.RD + (cm * self.chord * qv * speed) @ self.pitch_axis
        if not detail:
            return F, M, None
        q = qv * speed
        lift = float((q * cl).sum()); drag = float((q * cd).sum())
        alphas = (self.alpha_weights @ alpha).tolist()
        stall_lim = self.stall.copy()
        for pw in self.polar_wings:
            stall_lim[pw["idx"]] = float(pw["root"].valid[-1, 1])
        stalled = bool(np.any((v2 > 1e-4) & (np.abs(alpha) > stall_lim)))
        # per wing: resultant force (structural frame) at the area-weighted strip centre, for the 3D view
        Fs = a[:, None] * self.e_c + b[:, None] * self.e_d
        per_wing = []
        W = self.alpha_weights
        for i in range(len(self.wings)):
            m = W[i] > 0
            Fi = Fs[m].sum(axis=0)
            pos = (W[i][m] @ self.r[m]) + self.cg
            per_wing.append({"F": Fi.tolist(), "lift": float((q * cl)[m].sum()), "pos": pos.tolist(), "alpha": alphas[i]})
        return F, M, {"lift": lift, "drag": drag, "alpha": alphas, "stalled": stalled, "wings": per_wing}
