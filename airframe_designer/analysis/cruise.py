"""Cruise test: how the lifting body behaves in forward flight, from the section polars (XFOIL class).

Two parts, both without PX4 and without the fans' thrust vectoring (the fans count only as mass; the thrust that
balances drag is an ideal force along the body x axis through the CG):

1. ``alpha_sweep``: the whole aircraft's CL, CD, CM against angle of attack at one airspeed, from the strip-theory
   wing panels (section polars) and the body drag box. Gives the trim (CM = 0), the lift there against the weight,
   the speed that would carry the weight at that trim, the static margin (neutral point from dCM/dCL) and which
   panels are stalled.
2. ``free_flight``: the rigid body released in level flight at the trim with a small pitch-rate kick, integrated
   with the project's own physics, no controller: does the pitch oscillation die out (statically and dynamically
   stable), persist, or grow (unstable)?

Conventions: structural frame FRD. Alpha positive nose-up (flow from below), CM nose-up positive about the CG,
reference area = the panels' total area, reference chord = area / span (mean chord).
"""
from __future__ import annotations

import csv
import math
from pathlib import Path

import numpy as np

from ..aero import WingSet, BodyAero, RHO
from ..aero.airfoils import get_polar, build_polar, normalise_name
from ..dynamics.quaternion import q_from_euler, q_to_rotmat, q_to_euler
from ..geometry.airframe import Airframe
from ..geometry.propulsion import Rotor

G = 9.81
XFOIL_POINTS = Path(__file__).resolve().parents[2] / "results/reviews/20261005-atlas-v1-step/xfoil-screen-699/polar_points.csv"


def _settled_forces(ws, v, rates=None, passes: int = 8):
    """Wing forces with the polar wings' warm-started induced-angle iteration converged and the lookup-reuse
    band bypassed (both make a single call path-dependent, which breaks finite differences)."""
    rates = np.zeros(3) if rates is None else rates
    out = None
    for _ in range(passes):
        for pw in ws.polar_wings:
            pw.pop("last", None)
        out = ws.forces(v, rates)
    return out


def _wind_axes(alpha: float):
    d = np.array([math.cos(alpha), 0.0, math.sin(alpha)])      # flow direction in the body (velocity of the aircraft)
    l = np.array([math.sin(alpha), 0.0, -math.cos(alpha)])     # lift direction (up at alpha 0)
    return d, l


def alpha_sweep(af: Airframe, speed_ms: float, alphas_deg: np.ndarray, elevon_deg: float | None = None) -> dict:
    af.resolve_mass()
    cg = af.cg
    wings = af.active_wings()
    ws = WingSet(wings, cg)
    if elevon_deg is not None and ws.has_elevons:
        ws.set_elevons(elevon_deg)
    body = BodyAero(af.body, cg)
    S = float(ws.area.sum()) if ws.n else 1.0
    span = 2.0 * max((abs(float(p[1])) for w in wings for half in __import__("airframe_designer.geometry.wings", fromlist=["wing_outline"]).wing_outline(w) for p in half), default=1.0)
    c_ref = S / span if span > 0 else 1.0
    q = 0.5 * RHO * speed_ms ** 2
    W = float(af.mass.mass) * G
    rows = []
    if len(alphas_deg):          # the polar wings' induced-angle iteration is warm-started: settle it before the first row
        d0, _ = _wind_axes(math.radians(float(alphas_deg[0])))
        for _ in range(6):
            ws.forces(speed_ms * d0, np.zeros(3))
    for a_deg in alphas_deg:
        a = math.radians(float(a_deg))
        d, l = _wind_axes(a)
        v = speed_ms * d
        F_w, M_w, wb = _settled_forces(ws, v, passes=4)
        F_b, M_b, body_drag = body.forces(v, np.zeros(3))
        F = F_w + F_b
        M = M_w + M_b
        L = float(F @ l); D = float(-F @ d)
        rows.append({"alpha_deg": float(a_deg), "L": L, "D": D, "CL": L / (q * S), "CD": D / (q * S),
                     "CM": float(M[1]) / (q * S * c_ref), "M": float(M[1]), "L_over_W": L / W,
                     "wing_lift": wb["lift"], "wing_drag": wb["drag"], "body_drag": float(body_drag),
                     "panel_alpha_deg": [math.degrees(x) for x in wb["alpha"]], "stalled": bool(wb["stalled"])})
    cl = np.array([r["CL"] for r in rows]); cm = np.array([r["CM"] for r in rows]); al = np.array([r["alpha_deg"] for r in rows])
    lw = np.array([r["L_over_W"] for r in rows]); My = np.array([r["M"] for r in rows])
    stalled = np.array([r["stalled"] for r in rows])

    def at(i, t, key):
        v = np.array([r[key] for r in rows]); return float(v[i] + t * (v[i + 1] - v[i]))

    def slopes(i):
        h = max(1, int(round(1.0 / max(1e-9, float(al[1] - al[0]))))) if len(al) > 2 else 1
        i0, i1 = max(0, i - h), min(len(al) - 1, i + 1 + h)
        dcm = float((cm[i1] - cm[i0]) / max(1e-9, (al[i1] - al[i0]))); dcl = float((cl[i1] - cl[i0]) / max(1e-9, (al[i1] - al[i0])))
        dcm_dcl = dcm / dcl if abs(dcl) > 1e-9 else None
        sm = (-dcm_dcl) if dcm_dcl is not None else None
        return dcm, dcl, dcm_dcl, sm

    # 1. natural trim: CM crosses zero (prefer the stable sense, nose-up to nose-down with rising alpha)
    crossings = [i for i in range(len(al) - 1) if cm[i] * cm[i + 1] <= 0 and cm[i] != cm[i + 1]]
    stable_cross = [i for i in crossings if cm[i] > cm[i + 1]]
    # 2. otherwise the alpha that carries the weight (L = W), trimmed by an ideal pitching moment
    weight_cross = [i for i in range(len(al) - 1) if (lw[i] - 1) * (lw[i + 1] - 1) <= 0 and lw[i] != lw[i + 1]]
    trim = None
    if crossings or weight_cross:
        natural = bool(crossings)
        i = (stable_cross or crossings)[0] if natural else weight_cross[0]
        y = cm if natural else (lw - 1)
        t = float(y[i] / (y[i] - y[i + 1]))
        a_trim = float(al[i] + t * (al[i + 1] - al[i]))
        L_trim, D_trim, CL_trim, M_trim = at(i, t, "L"), at(i, t, "D"), at(i, t, "CL"), at(i, t, "M")
        dcm, dcl, dcm_dcl, sm = slopes(i)
        trim_moment = 0.0 if natural else -M_trim                       # N m nose-up needed to hold this alpha
        trim = {"alpha_deg": a_trim, "L": L_trim, "D": D_trim, "CL": CL_trim, "L_over_W": L_trim / W,
                "L_over_D": (L_trim / D_trim) if D_trim > 1e-9 else None,
                "speed_for_weight_ms": (speed_ms * math.sqrt(W / L_trim)) if L_trim > 1e-6 else None,
                "natural": natural, "trim_moment_Nm": trim_moment,
                "CM_untrimmed": at(i, t, "CM"),
                # a CG shift that would trim here instead of the moment (aft positive): M = L * dx -> dx = -M / L
                "cg_shift_to_trim_m": (-M_trim / L_trim) if (not natural and abs(L_trim) > 1e-6) else 0.0,
                "dCM_dalpha_per_deg": dcm, "dCL_dalpha_per_deg": dcl, "dCM_dCL": dcm_dcl, "static_margin": sm,
                "neutral_point_x": (float(cg[0]) - sm * c_ref) if sm is not None else None,   # x forward: NP behind the CG when stable
                "statically_stable": (dcm < 0), "stalled": bool(stalled[i] or stalled[i + 1])}
        if not natural and sm is not None:
            # moving the CG aft by cg_shift would leave margin sm - shift/c_ref
            trim["static_margin_after_cg_shift"] = sm - trim["cg_shift_to_trim_m"] / c_ref
    return {"speed_ms": speed_ms, "q": q, "S": S, "span": span, "c_ref": c_ref, "weight": W, "cg": [float(x) for x in cg],
            "rows": rows, "trim": trim, "elevon_deg": elevon_deg, "has_elevons": bool(ws.has_elevons),
            "panels": [{"name": w.name, "airfoil": getattr(w.aero, "airfoil_root", ""), "area": w.area, "span": w.span,
                        "root_chord": w.root_chord, "tip_chord": w.tip_chord, "sweep_deg": w.sweep_deg, "dihedral_deg": w.dihedral_deg,
                        "incidence_deg": w.incidence_deg, "pos": list(w.pos)} for w in wings]}


def trim_elevons(af: Airframe, speed_ms: float, alpha0_deg: float = 2.0) -> dict | None:
    """Solve alpha and the elevon deflection for lift = weight and zero pitching moment (Newton, 2 unknowns)."""
    af.resolve_mass()
    cg = af.cg
    ws = WingSet(af.active_wings(), cg)
    if not ws.has_elevons:
        return None
    body = BodyAero(af.body, cg)
    W = float(af.mass.mass) * G

    def f(a_deg, d_deg):
        ws.set_elevons(d_deg)
        a = math.radians(a_deg); d, l = _wind_axes(a); v = speed_ms * d
        F_w, M_w, _ = _settled_forces(ws, v)
        F_b, M_b, _ = body.forces(v, np.zeros(3))
        F = F_w + F_b; M = M_w + M_b
        return np.array([float(F @ l) - W, float(M[1])]), float(F @ l), float(-F @ d)

    x = np.array([alpha0_deg, 0.0])
    r, L, D = f(*x)
    for _ in range(40):
        J = np.zeros((2, 2))
        for j, h in enumerate((0.5, 1.0)):
            xp = x.copy(); xp[j] += h
            J[:, j] = (f(*xp)[0] - r) / h
        try:
            dx = -np.linalg.solve(J, r)
        except np.linalg.LinAlgError:
            break
        dx = np.clip(dx, -3.0, 3.0)
        x = x + dx
        x[1] = float(np.clip(x[1], -ws.elevon_wings[0]["max"] * 180 / math.pi, ws.elevon_wings[0]["max"] * 180 / math.pi))
        r, L, D = f(*x)
        if abs(r[0]) < 0.05 * W * 1e-2 and abs(r[1]) < 0.5:
            break
    ok = abs(r[0]) < 0.02 * W and abs(r[1]) < 5.0
    return {"ok": bool(ok), "alpha_deg": float(x[0]), "elevon_deg": float(x[1]), "L": float(L), "D": float(D), "residual_N": float(r[0]), "residual_Nm": float(r[1]),
            "at_limit": bool(abs(abs(x[1]) - ws.elevon_wings[0]["max"] * 180 / math.pi) < 1e-6)}


LATERAL_DEFAULT = {"roll_p": 1.5, "roll_d": 0.4, "roll_limit_deg": 15.0,      # elevon roll hold: deg per deg, deg per deg/s
                   # yaw channels act on SIDESLIP (keep the nose in the relative wind, as a weathercock would) plus yaw-rate damping;
                   # an optional heading term (yaw_p) is off by default because it fights the weathercock in a crosswind
                   "yaw_arm_m": 0.75, "yaw_p": 0.0, "yaw_beta": 0.10, "yaw_r": 0.10, "yaw_limit": 0.6,   # thrust split per deg of sideslip / heading, per deg/s
                   # split drag rudders at the wing tips (Northrop US2412646A: differentially opened tip flaps), one per side:
                   # drag area CD*A at full opening, acting at the outer elevon's mid-span trailing edge; opened 0..1 by the sideslip loop
                   "rudder_cda_m2": 0.25, "rudder_p": 0.0, "rudder_beta": 0.15, "rudder_r": 0.15,
                   # pitch hold on the elevons (the attitude loop PX4 runs in reality; the pitch-plane 'flight' stays uncontrolled)
                   "pitch_p": 1.0, "pitch_d": 0.3, "pitch_limit_deg": 8.0,
                   # allocation: the pitch channel (trim + hold) may use the elevon travel only up to max - roll_reserve, so roll always keeps this much
                   "roll_reserve_deg": 10.0}


def _tip_rudder_positions(af: Airframe) -> list[tuple[str, list[float]]]:
    """Mid-span trailing edge of the outermost elevon panel on each side (FRD), where a split drag rudder would sit."""
    out = []
    for side in (-1, 1):
        cands = [w for w in af.wings if getattr(w, "elevon", None) and not getattr(w, "symmetric", False) and int(getattr(w, "side", 0)) == side]
        if not cands:
            continue
        w = max(cands, key=lambda w: abs(float(w.pos[1])))
        sw, dh = math.radians(float(w.sweep_deg)), math.radians(float(w.dihedral_deg))
        le = np.array(w.pos, float) + 0.5 * np.array([-w.span * math.tan(sw), side * w.span * math.cos(dh), -w.span * math.sin(dh)])
        te = le - np.array([0.5 * (float(w.root_chord) + float(w.tip_chord)), 0.0, 0.0])
        out.append(("L" if side < 0 else "R", [float(x) for x in te]))
    return out


def free_flight(af: Airframe, speed_ms: float, alpha_deg: float, drag_N: float, duration_s: float = 30.0,
                perturb_q_deg_s: float = 5.0, dt: float = 0.002, sample_s: float = 0.05, altitude_m: float = 100.0,
                trim_moment_Nm: float = 0.0, longitudinal_only: bool = False, elevon_deg: float | None = None,
                lateral_control: dict | None = None, perturb_beta_deg: float = 0.0) -> dict:
    """Release at level flight (pitch = alpha, flight path 0) with an ideal forward thrust equal to the trim drag,
    kick the pitch rate, integrate the rigid body. Pitch is never controlled (the foil's own longitudinal behaviour).
    With `lateral_control` (see LATERAL_DEFAULT) a roll hold acts through the elevons (differential deflection on
    top of the trim) and a heading hold / yaw damper splits the forward thrust between a left and a right engine
    at +-yaw_arm_m (what the wing fans give by differential thrust); otherwise no controller at all."""
    from ..dynamics.rigid_body import RigidBody
    test = af.copy()
    test.resolve_mass()
    cg = [float(x) for x in test.cg]
    tmax = max(2.0 * float(drag_N), 1.0)
    lat = None if longitudinal_only else lateral_control
    if lat:
        arm = float(lat.get("yaw_arm_m", LATERAL_DEFAULT["yaw_arm_m"]))
        test.rotors = [Rotor(name=f"ideal thrust {nm}", enabled=True, pos=[cg[0], cg[1] + sy * arm, cg[2]], axis=[1.0, 0.0, 0.0], km=0.0,
                             max_thrust=tmax, tau=0.01, diameter=0.1, thrust_exponent=2.0, kind="prop", ram_drag=False, duct_axis=None, turn_loss=0.0)
                       for nm, sy in (("L", -1.0), ("R", 1.0))]
        cmds = [math.sqrt(max(0.0, float(drag_N)) / 2.0 / tmax)] * 2
        cda = float(lat.get("rudder_cda_m2", LATERAL_DEFAULT["rudder_cda_m2"]))
        rud_pos = _tip_rudder_positions(test)
        if cda > 0 and rud_pos:
            q0 = 0.5 * RHO * speed_ms ** 2
            for nm, pos in rud_pos:          # an ideal drag force (thrust pointing aft) at each tip, max CD*A*q at the release speed
                test.rotors.append(Rotor(name=f"drag rudder {nm}", enabled=True, pos=pos, axis=[-1.0, 0.0, 0.0], km=0.0, max_thrust=max(1.0, cda * q0),
                                         tau=0.05, diameter=0.05, thrust_exponent=1.0, kind="prop", ram_drag=False, duct_axis=None, turn_loss=0.0))
                cmds.append(0.0)
    else:
        test.rotors = [Rotor(name="ideal thrust", enabled=True, pos=cg, axis=[1.0, 0.0, 0.0], km=0.0, max_thrust=tmax, tau=0.01,
                             diameter=0.1, thrust_exponent=2.0, kind="prop", ram_drag=False, duct_axis=None, turn_loss=0.0)]
        cmds = [math.sqrt(max(0.0, float(drag_N)) / tmax)]
    n_thr = len(cmds)                       # engines (1 or 2) plus the two drag rudders when present
    n_rud = 2 if (lat and len(cmds) == 4) else 0
    n_eng = n_thr - n_rud
    if abs(trim_moment_Nm) > 1e-9:    # (the trim couple sits after the thrust engines in cmds)
        # an ideal constant trim couple (what an elevon or reflex would give): +-T at +-1 m, net force zero, no aero
        T = abs(trim_moment_Nm) / 2.0; up = -1.0 if trim_moment_Nm > 0 else 1.0        # nose-up: lift ahead of the CG
        test.rotors.append(Rotor(name="trim couple fwd", enabled=True, pos=[cg[0] + 1.0, cg[1], cg[2]], axis=[0.0, 0.0, up], km=0.0,
                                 max_thrust=T, tau=0.01, diameter=0.05, thrust_exponent=1.0, kind="prop", ram_drag=False, duct_axis=None, turn_loss=0.0))
        test.rotors.append(Rotor(name="trim couple aft", enabled=True, pos=[cg[0] - 1.0, cg[1], cg[2]], axis=[0.0, 0.0, -up], km=0.0,
                                 max_thrust=T, tau=0.01, diameter=0.05, thrust_exponent=1.0, kind="prop", ram_drag=False, duct_axis=None, turn_loss=0.0))
        cmds += [1.0, 1.0]
    test.design = dict(test.design or {}); test.design["vibration"] = {"enabled": False}
    rb = RigidBody(test)
    if elevon_deg is not None and rb.wings.has_elevons:
        rb.wings.set_elevons(elevon_deg)         # held at the trim deflection, no controller
    rb.reset(pos_ned=[0.0, 0.0, -float(altitude_m)])
    a = math.radians(alpha_deg)
    rb.q = q_from_euler(0.0, a, 0.0)
    rb.vel = np.array([speed_ms, 0.0, 0.0])
    if abs(perturb_beta_deg) > 1e-9:        # a sharp-edged side gust: the air suddenly comes perturb_beta_deg from the side (from the right when positive)
        bb = math.radians(perturb_beta_deg)
        rb.vel = q_to_rotmat(rb.q) @ np.array([speed_ms * math.cos(bb), speed_ms * math.sin(bb), 0.0]) + np.array([0.0, 0.0, 0.0])
        rb.vel = np.array([speed_ms * math.cos(bb), speed_ms * math.sin(bb), 0.0])
    rb.rates = np.array([0.0, math.radians(perturb_q_deg_s), 0.0])
    rb.omega = np.array(cmds); rb.cmd = np.array(cmds)
    rb.on_ground = False
    n = int(round(duration_s / dt)); every = max(1, int(round(sample_s / dt)))
    out = {"t": [], "speed": [], "alpha_deg": [], "pitch_deg": [], "alt": [], "climb": [], "q_deg_s": [], "roll_deg": [], "lift": [], "stalled": [],
           "pos": [], "q": [], "wing_forces": [], "yaw_deg": [], "beta_deg": [], "roll_cmd_deg": [], "thrust_split": [], "rudder": [], "pitch_cmd_deg": []}   # pos NED m, quaternion (w, x, y, z) for the 3D replay
    yaw0 = 0.0; pitch0 = a
    if lat:
        g = {**LATERAL_DEFAULT, **lat}
    ended = None
    v0sq = max(1e-6, speed_ms ** 2)
    R0 = q_to_rotmat(rb.q)
    for _ in range(6):            # settle the warm-started induced-angle iteration at the release state
        rb.wings.forces(R0.T @ rb.vel, rb.rates)
    for k in range(n):
        for pw in rb.wings.polar_wings:
            pw.pop("last", None)   # exact lookup every step: the 0.5 % reuse band hides slow drifts in a free flight
        if len(cmds) > n_thr:    # the trim couple is aerodynamic: it scales with dynamic pressure like an elevon or reflex would
            rb.cmd[n_thr:] = min(4.0, float(rb.vel @ rb.vel) / v0sq)
        roll_cmd = 0.0; split = 0.0; rud = 0.0; pitch_cmd = 0.0
        if lat:
            ph, th, ps = q_to_euler(rb.q)
            roll_cmd = float(np.clip(g["roll_p"] * math.degrees(ph) + g["roll_d"] * math.degrees(rb.rates[0]), -g["roll_limit_deg"], g["roll_limit_deg"]))
            pitch_cmd = float(np.clip(g["pitch_p"] * math.degrees(th - pitch0) + g["pitch_d"] * math.degrees(rb.rates[1]), -g["pitch_limit_deg"], g["pitch_limit_deg"]))
            if rb.wings.has_elevons:        # TE down positive = nose-down: a nose-up error gets TE down. Right wing TE down rolls left.
                ev_max = math.degrees(rb.wings.elevon_wings[0]["max"]); lim = max(0.0, ev_max - g["roll_reserve_deg"])
                pitch_total = float(np.clip((elevon_deg or 0.0) + pitch_cmd, -lim, lim))
                rb.wings.set_controls(pitch_total, float(np.clip(roll_cmd, -g["roll_reserve_deg"], g["roll_reserve_deg"])))
            yaw_err = math.degrees((ps - yaw0 + math.pi) % (2 * math.pi) - math.pi)
            vb_now = q_to_rotmat(rb.q).T @ rb.vel
            beta_now = math.degrees(math.atan2(vb_now[1], max(1e-6, vb_now[0])))
            # positive command = nose-left moment. Sideslip from the right (beta > 0) needs the nose to go right: negative.
            split = float(np.clip(g["yaw_p"] * yaw_err - g["yaw_beta"] * beta_now + g["yaw_r"] * math.degrees(rb.rates[2]), -g["yaw_limit"], g["yaw_limit"]))
            T = max(0.0, float(drag_N))                                 # more right thrust yaws the nose left
            rb.cmd[0] = math.sqrt(T * (1.0 - split) / 2.0 / tmax); rb.cmd[1] = math.sqrt(T * (1.0 + split) / 2.0 / tmax)
            if n_rud:                                                   # drag on the left tip yaws the nose left (negative)
                u = float(np.clip(g["rudder_p"] * yaw_err - g["rudder_beta"] * beta_now + g["rudder_r"] * math.degrees(rb.rates[2]), -1.0, 1.0)) * min(4.0, float(rb.vel @ rb.vel) / v0sq)
                rb.cmd[n_eng] = min(1.0, max(0.0, u)); rb.cmd[n_eng + 1] = min(1.0, max(0.0, -u))   # [L, R]: the left tip opens for a nose-right error (drag there yaws the nose left)
                rud = u
        rb.step(dt, detail=(k % every == 0))
        if longitudinal_only:    # keep the motion in the pitch plane: no roll, no yaw, no sideslip (the foil's own pitch stability)
            rb.rates[0] = 0.0; rb.rates[2] = 0.0; rb.vel[1] = 0.0
            _, pth, _ = q_to_euler(rb.q); rb.q = q_from_euler(0.0, pth, 0.0)
        if k % every == 0:
            R = q_to_rotmat(rb.q)
            vb = R.T @ rb.vel
            alpha = math.degrees(math.atan2(vb[2], vb[0])) if abs(vb[0]) > 1e-6 else 0.0
            roll, pitch, _ = q_to_euler(rb.q)
            out["t"].append(round(rb.t, 3)); out["speed"].append(float(np.linalg.norm(rb.vel))); out["alpha_deg"].append(alpha)
            out["pitch_deg"].append(math.degrees(pitch)); out["roll_deg"].append(math.degrees(roll)); out["alt"].append(float(-rb.pos[2]))
            out["climb"].append(float(-rb.vel[2])); out["q_deg_s"].append(math.degrees(rb.rates[1]))
            out["lift"].append(float(rb.breakdown.get("lift", 0.0))); out["stalled"].append(bool(rb.breakdown.get("stalled", False)))
            out["pos"].append([round(float(x), 4) for x in rb.pos]); out["q"].append([round(float(x), 6) for x in rb.q])
            _, _, yaw = q_to_euler(rb.q)
            beta = math.degrees(math.atan2(vb[1], max(1e-6, vb[0])))
            out["yaw_deg"].append(math.degrees(yaw)); out["beta_deg"].append(beta); out["roll_cmd_deg"].append(roll_cmd); out["thrust_split"].append(split); out["rudder"].append(rud); out["pitch_cmd_deg"].append(pitch_cmd)
            out["wing_forces"].append([{"F": [round(float(x), 2) for x in w["F"]], "pos": [round(float(x), 3) for x in w["pos"]]}
                                       for w in rb.breakdown.get("wing_forces", [])])
            if not rb.is_sane() or abs(math.degrees(pitch)) > 80 or rb.pos[2] > 0:
                ended = "tumbled" if abs(math.degrees(pitch)) > 80 else ("hit the ground" if rb.pos[2] > 0 else "diverged")
                break
    verdict = _classify(out, ended, duration_s)
    out["ended"] = ended; out["verdict"] = verdict; out["longitudinal_only"] = longitudinal_only
    out["lateral_control"] = ({**LATERAL_DEFAULT, **lat} if lat else None); out["perturb_beta_deg"] = float(perturb_beta_deg)
    if lat and not ended:
        out["verdict"]["lateral"] = (f"pitch held within {max(abs(x - out['pitch_deg'][0]) for x in out['pitch_deg']):.1f} deg (elevon up to {max(abs(x) for x in out['pitch_cmd_deg']):.1f} deg), roll held within {max(abs(x) for x in out['roll_deg']):.1f} deg, heading within "
                                     f"{max(abs(x) for x in out['yaw_deg']):.1f} deg, sideslip within {max(abs(x) for x in out['beta_deg']):.1f} deg; "
                                     f"elevon roll command up to {max(abs(x) for x in out['roll_cmd_deg']):.1f} deg, thrust split up to {max(abs(x) for x in out['thrust_split']) * 100:.0f} %, tip drag rudder up to {max(abs(x) for x in out['rudder']) * 100:.0f} %")
    return out


def _classify(ts: dict, ended, duration_s: float) -> dict:
    t = np.array(ts["t"]); p = np.array(ts["pitch_deg"]); al = np.array(ts["alpha_deg"])
    if ended:
        roll_max = float(np.max(np.abs(np.array(ts["roll_deg"])))) if ts["roll_deg"] else 0.0
        return {"stable": False, "label": f"unstable: {ended} at t = {t[-1]:.1f} s" + (f" after rolling to {roll_max:.0f} deg" if roll_max > 30 else ""), "ratio": None}
    if len(t) < 20:
        return {"stable": False, "label": "too short", "ratio": None}
    x = p - np.mean(p[len(p) // 2:])
    half = len(x) // 2
    a1 = float(np.max(np.abs(x[:half]))); a2 = float(np.max(np.abs(x[half:])))
    ratio = a2 / a1 if a1 > 1e-6 else 0.0
    drift = float(abs(al[-1] - al[0]))
    alt_change = float(ts["alt"][-1] - ts["alt"][0])
    roll = np.array(ts["roll_deg"]); roll_max = float(np.max(np.abs(roll)))
    if drift > 3.0 or abs(float(np.max(p) - np.min(p))) > 45.0 or alt_change < -0.3 * duration_s * 2.0 or roll_max > 30.0:
        how = f"rolled off to {roll_max:.0f} deg" if roll_max > 30.0 else f"alpha drifted {drift:.1f} deg, pitch swung {float(np.max(p) - np.min(p)):.0f} deg"
        label = f"unstable: departed from the trim ({how}, altitude {alt_change:+.0f} m)"
        return {"stable": False, "label": label, "ratio": ratio, "alpha_drift_deg": drift, "alt_change_m": alt_change,
                "speed_change_ms": float(ts["speed"][-1] - ts["speed"][0])}
    if ratio < 0.5 and drift < 2.0:
        label = "stable: the pitch oscillation dies out"
    elif ratio < 0.9:
        label = "lightly damped: the oscillation decays slowly"
    elif ratio <= 1.2 and drift < 2.0:
        label = "neutral: the oscillation neither grows nor dies"
    else:
        label = "unstable: the oscillation grows"
    return {"stable": ratio < 0.9 and drift < 2.0, "label": label, "ratio": ratio, "alpha_drift_deg": drift,
            "alt_change_m": float(ts["alt"][-1] - ts["alt"][0]), "speed_change_ms": float(ts["speed"][-1] - ts["speed"][0])}


def xfoil_points(name: str) -> list[dict]:
    """Agent B's XFOIL 6.99 points for a section (natural transition runs), for comparison with the app's polar."""
    name = normalise_name(name)
    if not XFOIL_POINTS.is_file():
        return []
    out = []
    with open(XFOIL_POINTS) as fh:
        for r in csv.DictReader(fh):
            if normalise_name(r["profile"]) == name and r["mode"] == "natural":
                out.append({"speed_kmh": float(r["speed_kmh"]), "Re": float(r["Re"]), "alpha": float(r["alpha"]),
                            "CL": float(r["CL"]), "CD": float(r["CD"]), "CM": float(r["CM"])})
    return out


def section_comparison(af: Airframe) -> list[dict]:
    """Per section: the app's polar (NeuralFoil or XFOIL, whichever built it) at the Reynolds numbers of Agent B's
    XFOIL points, against those points."""
    seen = {}
    for w in af.active_wings():
        a = w.aero
        if getattr(a, "model", "") != "polar" or not a.airfoil_root or a.airfoil_root in seen:
            continue
        pol = get_polar(a.airfoil_root, a.ncrit, a.polar_source)
        table = build_polar(a.airfoil_root, ncrit=a.ncrit, source=a.polar_source)
        pts = xfoil_points(a.airfoil_root)
        comp = []
        for p in pts:
            cl, cd, cm = pol.lookup(np.array([math.radians(p["alpha"])]), np.array([p["Re"]]))
            comp.append({**p, "app_CL": float(cl[0]), "app_CD": float(cd[0]), "app_CM": float(cm[0])})
        alphas = np.arange(-10, 16.01, 0.5)
        re_show = sorted({p["Re"] for p in pts})[:3] or [1e6]
        curves = []
        for Re in re_show:
            cl, cd, cm = pol.lookup(np.radians(alphas), np.full(len(alphas), Re))
            curves.append({"Re": float(Re), "alpha": alphas.tolist(), "CL": cl.tolist(), "CD": cd.tolist(), "CM": cm.tolist()})
        rms = math.sqrt(np.mean([(c["app_CL"] - c["CL"]) ** 2 for c in comp])) if comp else None
        seen[a.airfoil_root] = {"airfoil": a.airfoil_root, "source": table.get("source"), "note": table.get("note"),
                                "valid_alpha_deg": table.get("valid"), "re_list": table.get("re"),
                                "xfoil_points": comp, "curves": curves, "rms_dCL_vs_xfoil": rms, "n_xfoil": len(comp)}
    return list(seen.values())


def with_cg_shift(af: Airframe, dx_m: float) -> Airframe:
    """A copy with the CG moved dx forward (what-if: ballast or batteries moved), inertia unchanged."""
    t = af.copy(); t.resolve_mass()
    t.mass.from_items = False; t.mass.manual = None
    t.mass.cg = [float(t.mass.cg[0]) + float(dx_m), float(t.mass.cg[1]), float(t.mass.cg[2])]
    return t


def cruise_test(af: Airframe, speed_kmh: float = 60.0, alpha_min: float = -6.0, alpha_max: float = 16.0, step: float = 0.5,
                duration_s: float = 30.0, perturb_q_deg_s: float = 5.0, altitude_m: float = 100.0, cg_shift_m: float = 0.0,
                lateral_control: bool = True, gust_beta_deg: float = 10.0) -> dict:
    from ..aero.airfoils import ensure_polars
    ensure_polars(af)
    if abs(cg_shift_m) > 1e-9:
        af = with_cg_shift(af, cg_shift_m)
    V = float(speed_kmh) / 3.6
    et = trim_elevons(af, V)
    sweep = alpha_sweep(af, V, np.arange(alpha_min, alpha_max + step / 2, step), elevon_deg=(et["elevon_deg"] if et else None))
    if et and sweep["trim"] is not None:
        tr0 = sweep["trim"]; W = sweep["weight"]
        tr0.update({"elevon_deg": et["elevon_deg"], "elevon_ok": et["ok"], "elevon_at_limit": et["at_limit"], "natural": True, "trim_moment_Nm": 0.0})
        if et["ok"]:      # the solver's point (lift = weight and zero moment together) rather than the grid interpolation
            tr0.update({"alpha_deg": et["alpha_deg"], "L": et["L"], "D": et["D"], "L_over_W": et["L"] / W,
                        "L_over_D": (et["L"] / et["D"]) if et["D"] > 1e-9 else None, "speed_for_weight_ms": V})
            rows = sweep["rows"]
            k = int(np.argmin([abs(r["alpha_deg"] - et["alpha_deg"]) for r in rows]))
            tr0["stalled"] = bool(rows[k]["stalled"])
    res = {"ok": True, "speed_kmh": float(speed_kmh), "cg_shift_m": float(cg_shift_m), "sweep": sweep, "sections": section_comparison(af), "flight": None, "flight6": None,
           "notes": ["Fans are mass only; the thrust that balances drag is an ideal force along x at the CG.",
                     "Section polars from the app's airfoil store (see each section's source); Agent B's XFOIL 6.99 points overlaid.",
                     "Strip theory: no spanwise induced-flow coupling between panels, no fuselage or canopy lift, no fan inlet flow."]}
    tr = sweep["trim"]
    if tr is not None:
        a0 = tr["alpha_deg"]
        kw = dict(duration_s=duration_s, perturb_q_deg_s=perturb_q_deg_s, altitude_m=altitude_m, trim_moment_Nm=tr.get("trim_moment_Nm", 0.0))
        kw["elevon_deg"] = tr.get("elevon_deg")
        res["flight"] = free_flight(af, V, a0, tr["D"], longitudinal_only=True, **kw)     # the foil's pitch stability
        if lateral_control:      # all six degrees of freedom: roll hold on the elevons, heading hold by differential thrust (pitch still free)
            res["flight6"] = free_flight(af, V, a0, tr["D"], longitudinal_only=False, lateral_control=dict(LATERAL_DEFAULT), perturb_beta_deg=gust_beta_deg, **kw)
            res["flight6_open"] = free_flight(af, V, a0, tr["D"], longitudinal_only=False, **kw)   # the same with no controller at all
            res["notes"].insert(0, f"Six-axis flight 'flight6': released into a {gust_beta_deg:g} deg side gust plus the pitch kick, with an attitude loop as PX4 "
                                   "would run it: pitch and roll held by the elevons (P 1 to 1.5 deg/deg, D 0.3 to 0.4 deg per deg/s, 10 deg of travel reserved for roll), "
                                   "sideslip zeroed by splitting the forward thrust left/right at +-0.75 m (the wing fans) and by split drag rudders at the "
                                   "wing-tip trailing edges (Northrop US2412646A, CD A 0.25 m2 each), plus yaw-rate damping; 'flight6_open' is the pitch-kick "
                                   "release with no controller at all; the pitch-plane 'flight' is the foil alone, uncontrolled.")
        else:
            res["flight6"] = free_flight(af, V, a0, tr["D"], longitudinal_only=False, **kw)   # all six degrees of freedom, no control at all
        if tr.get("elevon_deg") is not None:
            res["notes"].insert(0, f"Elevons (plain flaps, thin-airfoil effectiveness) trimmed to {tr['elevon_deg']:+.1f} deg (TE down positive) and held there; no pitch controller.")
        if not tr.get("natural") and tr.get("elevon_deg") is None:
            res["notes"].append("CM never crosses zero: the free flight is trimmed at the lift-equals-weight alpha by an ideal constant "
                                "pitching moment (what reflex, an elevon or a CG shift would provide).")
    else:
        res["notes"].append("No trim: CM never crosses zero and the lift never equals the weight in this alpha range; the free flight was not run.")
    return res
