"""Free flight on the CFD library: the rigid body released in forward flight, aerodynamic forces and moments taken
from the computed attitudes (interpolated in alpha and beta, scaled with dynamic pressure), with wind.

What the library cannot give is rate damping (pitch, roll and yaw damping come from the motion, and the library
holds static attitudes only). That part is taken from the strip-theory wing model the app already has: the moment
change per unit body rate is measured once at the release speed by finite differences and scaled with airspeed.
Static forces: CFD. Damping: strip theory. Thrust: an ideal forward force through the CG equal to the CFD drag at
release, so the speed is roughly held. No controller, no control surfaces.

Conventions as the rest of the package: body FRD, alpha nose-up positive, beta wind from the right positive.
Wind is given in NED; the aircraft starts heading north, so a crosswind "from the right" blows toward -y (west).
"""
from __future__ import annotations

import math
from pathlib import Path

import numpy as np

from ..analysis.cruise import _classify
from ..dynamics.quaternion import q_from_euler, q_to_rotmat, q_to_euler
from ..dynamics.rigid_body import RigidBody
from ..geometry.airframe import Airframe
from ..geometry.propulsion import Rotor
from . import case as C

G = 9.81


class _Table:
    """Fast (alpha, beta) -> force, moment about the origin (N, N m at the library speed), with mirroring."""

    def __init__(self, lib):
        pts, F, M = [], [], []
        for k, r in lib.results.items():
            f = np.array(r["F_frd"], float); m = np.array(r["M0_frd"], float)
            if not (np.all(np.isfinite(f)) and np.all(np.isfinite(m))):
                continue
            a, b = float(r["alpha_deg"]), float(r["beta_deg"])
            pts.append((a, b)); F.append(f); M.append(m)
            if abs(b) > 1e-9:
                pts.append((a, -b)); F.append(f * np.array([1, -1, 1])); M.append(m * np.array([-1, 1, -1]))
        if not pts:
            raise ValueError("the library has no computed attitude")
        self.P = np.array(pts); self.F = np.array(F); self.M = np.array(M)
        self.amin, self.amax = self.P[:, 0].min(), self.P[:, 0].max()
        self.bmin, self.bmax = self.P[:, 1].min(), self.P[:, 1].max()
        self.V = lib.speed_kmh / 3.6
        self.mode = "2d"
        if len(pts) == 1:
            self.mode = "const"
        elif np.ptp(self.P[:, 1]) < 1e-9 or np.ptp(self.P[:, 0]) < 1e-9:
            self.mode = "1d"
            self.axis = 0 if np.ptp(self.P[:, 1]) < 1e-9 else 1
            o = np.argsort(self.P[:, self.axis]); self.x1 = self.P[o, self.axis]; self.F1 = self.F[o]; self.M1 = self.M[o]
        else:
            from scipy.interpolate import LinearNDInterpolator, NearestNDInterpolator
            vals = np.hstack([self.F, self.M])
            try:
                self.lin = LinearNDInterpolator(self.P, vals)
            except Exception:  # noqa: BLE001
                self.lin = None
            self.near = NearestNDInterpolator(self.P, vals)
        self.clipped = 0

    def __call__(self, alpha_deg: float, beta_deg: float):
        a = min(max(alpha_deg, self.amin), self.amax); b = min(max(beta_deg, self.bmin), self.bmax)
        if a != alpha_deg or b != beta_deg:
            self.clipped += 1
        if self.mode == "const":
            return self.F[0], self.M[0]
        if self.mode == "1d":
            x = a if self.axis == 0 else b
            F = np.array([np.interp(x, self.x1, self.F1[:, i]) for i in range(3)])
            M = np.array([np.interp(x, self.x1, self.M1[:, i]) for i in range(3)])
            return F, M
        v = self.lin(a, b) if self.lin is not None else None
        if v is None or np.any(np.isnan(v)):
            v = self.near(a, b)
        v = np.asarray(v).ravel()
        return v[:3], v[3:6]


def _outside_increment(alpha_deg, beta_deg, table: _Table, q: float, S: float, S_side: float, r_ac: np.ndarray):
    """Beyond the computed range the table is held at its edge; this adds a flat-plate style increment so large
    angles still produce a growing normal / side force (and its moment about the CG through the planform centre).
    Rough physics, flagged in the result: it keeps a departed aircraft from flying on an edge value forever."""
    a = math.radians(alpha_deg); b = math.radians(beta_deg)
    ae = math.radians(min(max(alpha_deg, table.amin), table.amax)); be = math.radians(min(max(beta_deg, table.bmin), table.bmax))
    F = np.zeros(3)
    if a != ae:
        dz = math.sin(a) - math.sin(ae)
        F[2] += -q * S * 1.2 * dz * abs(math.cos(a)) * 1.0          # extra normal force (down for extra nose-up)
        F[0] += q * S * 1.2 * dz * abs(math.sin(a)) * (-1.0 if a > 0 else 1.0)   # and its drag-wise part along the body
    if b != be:
        dy = math.sin(b) - math.sin(be)
        F[1] += -q * S_side * 1.2 * dy * abs(math.cos(b))
    M = np.cross(r_ac, F)
    return F, M


class _CfdBody(RigidBody):
    """RigidBody whose wing and body aerodynamics are replaced by the CFD table (plus strip-theory rate damping)."""

    def __init__(self, airframe: Airframe, table: _Table, damping: np.ndarray, V0: float, cg: np.ndarray, ref: dict):
        super().__init__(airframe)
        self.table = table
        self.S = float(ref["S"]); self.S_side = float(ref["S_side"]); self.r_ac = np.asarray(ref["r_ac"], float) - np.asarray(cg, float)
        self.outside_steps = 0
        self.M_ctrl = np.zeros(3)
        self.D = damping          # 3x3: body moment per unit body rate at V0 (N m s / rad), negative diagonal = damping
        self.V0 = V0
        self.cg_vec = np.asarray(cg, float)
        self.last_aero = {}

    def _forces(self, pos, vel, q, rates, omega, detail: bool = True, dt: float | None = None):
        R = q_to_rotmat(q)
        v_air = R.T @ (vel - self.wind_ned)
        F_r, M_r, thrust, ram = self.rotors.forces(omega, v_air, rates, detail)
        V = float(np.sqrt(v_air @ v_air))
        if V > 0.5:
            alpha = math.degrees(math.atan2(v_air[2], v_air[0]))
            beta = math.degrees(math.asin(max(-1.0, min(1.0, v_air[1] / V))))
            F0, M0 = self.table(alpha, beta)
            s = (V / self.table.V) ** 2
            F_a = F0 * s
            M_a = (M0 - np.cross(self.cg_vec, F0)) * s        # library moments are about the FRD origin: move to the CG
            if alpha < self.table.amin or alpha > self.table.amax or beta < self.table.bmin or beta > self.table.bmax:
                Fo, Mo = _outside_increment(alpha, beta, self.table, 0.5 * C.RHO * V * V, self.S, self.S_side, self.r_ac)
                F_a = F_a + Fo; M_a = M_a + Mo
                self.outside_steps += 1
            M_d = (self.D @ rates) * (V / self.V0)
        else:
            alpha = beta = 0.0; F_a = np.zeros(3); M_a = np.zeros(3); M_d = np.zeros(3)
        F_body = F_r + F_a
        M_body = M_r + M_a + M_d + self.M_ctrl
        F_ned = R @ F_body
        F_ned[2] += self.mass * G
        F_c, M_c, on_ground, feet = self.legs.forces(pos, vel, R, rates, dt, self.mass, self.I_inv)
        F_ned = F_ned + F_c
        M_body = M_body + M_c
        bd = None
        if detail:
            wa = C.wind_axes(F_a, alpha, beta) if V > 0.5 else {"lift": 0.0, "drag": 0.0, "side": 0.0}
            bd = {"thrust": float(thrust.sum()), "lift": wa["lift"], "wing_drag": wa["drag"], "side": wa["side"], "ram_drag": ram,
                  "body_drag": 0.0, "alpha": alpha, "beta": beta, "stalled": False, "airspeed": V, "power": self.rotors.ideal_power(thrust),
                  "M_aero": M_a.tolist(), "M_damp": M_d.tolist(),
                  "wing_forces": [{"F": F_a.tolist(), "pos": self.cg_vec.tolist()}]}
        return F_ned, M_body, thrust, on_ground, feet, R, bd


def damping_matrix(af: Airframe, speed_ms: float, alpha_deg: float, drate: float = 0.3) -> np.ndarray:
    """dM/d(rates) of the strip-theory wings at the release state, by central differences (N m per rad/s)."""
    rb = RigidBody(af)
    a = math.radians(alpha_deg)
    v = speed_ms * np.array([math.cos(a), 0.0, math.sin(a)])
    D = np.zeros((3, 3))
    for j in range(3):
        r = np.zeros(3); r[j] = drate
        for pw in rb.wings.polar_wings:
            pw.pop("last", None)
        _, Mp, _ = rb.wings.forces(v, r)
        for pw in rb.wings.polar_wings:
            pw.pop("last", None)
        _, Mm, _ = rb.wings.forces(v, -r)
        D[:, j] = (np.asarray(Mp) - np.asarray(Mm)) / (2 * drate)
    return D


class WindSchedule:
    """Wind in NED from a list of events [{t, head, cross, up, ramp}] (m/s; head = blowing against a north-bound
    aircraft, cross = from the right, up = updraft; ramp = seconds to blend in) plus optional turbulence: a
    low-pass filtered random component with the given RMS (m/s) on each axis (fixed seed, repeatable)."""

    def __init__(self, events, turbulence_rms: float = 0.0, turbulence_s: float = 1.5, seed: int = 1):
        ev = sorted([dict(e) for e in (events or [])], key=lambda e: float(e.get("t", 0.0)))
        if not ev or float(ev[0].get("t", 0.0)) > 0.0:
            ev.insert(0, {"t": 0.0, "head": 0.0, "cross": 0.0, "up": 0.0, "ramp": 0.0})
        self.events = ev
        self.rms = float(turbulence_rms)
        self.tau = max(0.2, float(turbulence_s))
        self.rng = np.random.default_rng(int(seed))
        self.gust = np.zeros(3)

    def steady(self, t: float) -> np.ndarray:
        cur = self.events[0]
        for e in self.events:
            if float(e.get("t", 0.0)) <= t:
                cur = e
            else:
                break
        w = np.array([-float(cur.get("head", 0.0)), -float(cur.get("cross", 0.0)), -float(cur.get("up", 0.0))])
        ramp = float(cur.get("ramp", 0.0) or 0.0)
        if ramp > 0 and cur is not self.events[0]:
            i = self.events.index(cur)
            prev = self.events[i - 1]
            wp = np.array([-float(prev.get("head", 0.0)), -float(prev.get("cross", 0.0)), -float(prev.get("up", 0.0))])
            f = min(1.0, (t - float(cur.get("t", 0.0))) / ramp)
            w = wp + (w - wp) * f
        return w

    def step(self, t: float, dt: float) -> np.ndarray:
        w = self.steady(t)
        if self.rms > 0:
            a = dt / self.tau
            self.gust += a * (self.rng.normal(0.0, self.rms, 3) * math.sqrt(2.0 * self.tau / dt) - self.gust)
            w = w + self.gust
        return w


class IdealPilot:
    """Holds altitude, heading and airspeed with bounded control moments (what elevons / differential fans could give)
    and the fans' thrust along the body x axis. Pitch is the altitude control, roll holds wings level, yaw holds
    heading. Gains from the inertia tensor (0.5 Hz, damping 0.8); the pilot is deliberately simple and ideal."""

    def __init__(self, I: np.ndarray, m_max: np.ndarray, thrust_max: float, V_target: float, alt_target: float,
                 pitch_trim_deg: float, drag_est: float):
        wn, z = 2 * math.pi * 0.5, 0.8
        Ii = np.diag(I)
        self.kp = Ii * wn * wn; self.kd = 2 * z * wn * Ii
        self.m_max = np.asarray(m_max, float)
        self.thrust_max = float(thrust_max)
        self.V_t = V_target; self.alt_t = alt_target
        self.pitch_trim = math.radians(pitch_trim_deg)
        self.thrust = drag_est; self.i_speed = 0.0
        self.sat = np.zeros(4)      # saturation counters: roll, pitch, yaw, thrust

    def update(self, rb, dt: float):
        roll, pitch, yaw = q_to_euler(rb.q)
        alt = -rb.pos[2]; climb = -rb.vel[2]
        # outer loop: pitch target from the altitude error and climb rate, around the trim pitch, within +-10 deg
        pitch_t = self.pitch_trim + max(-0.17, min(0.17, 0.01 * (self.alt_t - alt) - 0.03 * climb))
        yaw_err = (0.0 - yaw + math.pi) % (2 * math.pi) - math.pi
        err = np.array([0.0 - roll, pitch_t - pitch, yaw_err])
        M = self.kp * err - self.kd * rb.rates
        for i in range(3):
            if abs(M[i]) > self.m_max[i]:
                self.sat[i] += 1
        M = np.clip(M, -self.m_max, self.m_max)
        rb.M_ctrl = M
        V = float(rb.breakdown.get("airspeed", np.linalg.norm(rb.vel))) if rb.breakdown else float(np.linalg.norm(rb.vel))
        e = self.V_t - V
        self.i_speed = max(-200.0, min(200.0, self.i_speed + e * dt * 20.0))
        T = self.thrust + 60.0 * e + self.i_speed
        if T > self.thrust_max or T < 0:
            self.sat[3] += 1
        T = max(0.0, min(self.thrust_max, T))
        rb.cmd[0] = math.sqrt(T / rb.rotors_tmax) if rb.rotors_tmax > 0 else 0.0
        return pitch_t, T


def trim_pitch_for_weight(table: _Table, V: float, weight: float, S_unused=None) -> tuple[float | None, float]:
    """alpha (deg, beta 0) where the table's lift equals the weight at speed V, by bisection inside the table range.
    Returns (alpha or None when the table never reaches the weight, lift at the top of the range)."""
    def lift(a):
        F, _ = table(a, 0.0)
        return C.wind_axes(F * (V / table.V) ** 2, a, 0.0)["lift"]
    lo, hi = table.amin, table.amax
    l_hi = lift(hi); l_lo = lift(lo)
    if (l_hi - weight) * (l_lo - weight) > 0:
        return None, max(l_hi, l_lo)
    for _ in range(40):
        mid = 0.5 * (lo + hi)
        if (lift(mid) - weight) * (l_lo - weight) <= 0:
            hi = mid
        else:
            lo = mid; l_lo = lift(lo)
    return 0.5 * (lo + hi), lift(0.5 * (lo + hi))


def cfd_free_flight(af: Airframe, lib, speed_kmh: float | None = None, pitch0_deg: float = -8.0, crosswind_ms: float = 0.0,
                    headwind_ms: float = 0.0, duration_s: float = 30.0, dt: float = 0.002, sample_s: float = 0.05,
                    altitude_m: float = 100.0, mass_kg: float | None = None, cg: list[float] | None = None,
                    damping_scale: float = 1.0, perturb_q_deg_s: float = 0.0, controller: str = "none",
                    wind_events: list | None = None, turbulence_rms: float = 0.0, thrust_fraction: float = 1.0,
                    control_moments: list[float] | None = None) -> dict:
    table = _Table(lib)
    V = (speed_kmh / 3.6) if speed_kmh else table.V
    test = af.copy()
    test.resolve_mass()
    if mass_kg or cg:
        test.mass.manual = dict(test.mass.manual or {})
        if mass_kg:
            test.mass.mass = float(mass_kg)
        if cg:
            test.mass.cg = [float(x) for x in cg]
    cgv = np.array([float(x) for x in test.cg])
    # strip-theory damping at the release state (from the original airframe's wings), then drop the wings and the body box
    D = damping_matrix(af, V, pitch0_deg) * float(damping_scale)
    test.wings = []
    try:
        test.body.drag = [0.0, 0.0, 0.0]          # the CFD force includes the body
    except Exception:  # noqa: BLE001
        pass
    fans_max = float(sum(float(r.max_thrust) for r in af.rotors if getattr(r, "enabled", True))) or 1.0
    weight = float(test.mass.mass) * G
    pitch_trim, lift_top = trim_pitch_for_weight(table, V, weight)
    use_pilot = controller == "hold"
    if use_pilot and pitch_trim is None:
        pitch_trim_used = table.amax if lift_top < weight else table.amin
    else:
        pitch_trim_used = pitch_trim if pitch_trim is not None else pitch0_deg
    F0, _ = table(pitch0_deg, 0.0)
    drag0 = max(0.0, C.wind_axes(F0, pitch0_deg, 0.0)["drag"] * (V / table.V) ** 2)
    tmax = max(2.0 * drag0, 1.0) if not use_pilot else max(fans_max * float(thrust_fraction), 1.0)
    test.rotors = [Rotor(name="ideal thrust", enabled=True, pos=cgv.tolist(), axis=[1.0, 0.0, 0.0], km=0.0, max_thrust=tmax, tau=0.01,
                         diameter=0.1, thrust_exponent=2.0, kind="prop", ram_drag=False, duct_axis=None, turn_loss=0.0)]
    test.design = dict(test.design or {}); test.design["vibration"] = {"enabled": False}
    meta = lib.meta
    b = np.asarray(meta.get("bounds_frd", [[0, 0, 0], [0, 0, 0]]), float)
    ref = {"S": meta.get("area_m2", 6.5), "S_side": 0.5 * meta.get("length_m", 3.7) * float(b[1][2] - b[0][2] or 1.6),
           "r_ac": [0.5 * (b[0][0] + b[1][0]), 0.0, 0.5 * (b[0][2] + b[1][2])]}
    rb = _CfdBody(test, table, D, V, cgv, ref)
    rb.rotors_tmax = tmax
    rb.reset(pos_ned=[0.0, 0.0, -float(altitude_m)])
    rb.q = q_from_euler(0.0, math.radians(pitch0_deg), 0.0)
    rb.vel = np.array([V, 0.0, 0.0])                         # flight path level, heading north: alpha = pitch at release
    events = list(wind_events or [])
    if not events:
        events = [{"t": 0.0, "head": float(headwind_ms), "cross": float(crosswind_ms), "up": 0.0, "ramp": 0.0}]
    wind = WindSchedule(events, turbulence_rms)
    rb.wind_ned = wind.step(0.0, dt)
    pilot = None
    if use_pilot:
        mm = control_moments or [300.0, 400.0, 150.0]
        pilot = IdealPilot(test.mass.tensor(), mm, tmax, V, float(altitude_m), pitch_trim_used, drag0)
    rb.rates = np.array([0.0, math.radians(perturb_q_deg_s), 0.0])
    cmd = math.sqrt(drag0 / tmax)
    rb.omega = np.array([cmd]); rb.cmd = np.array([cmd])
    rb.on_ground = False
    n = int(round(duration_s / dt)); every = max(1, int(round(sample_s / dt)))
    out = {"t": [], "speed": [], "airspeed": [], "alpha_deg": [], "beta_deg": [], "pitch_deg": [], "roll_deg": [], "yaw_deg": [], "alt": [], "climb": [],
           "q_deg_s": [], "p_deg_s": [], "r_deg_s": [], "lift": [], "drag": [], "side": [], "stalled": [], "pos": [], "q": [], "wing_forces": [],
           "east": [], "wind_head": [], "wind_cross": [], "wind_up": [], "thrust": [], "pitch_target_deg": [], "M_ctrl": []}
    ended = None
    pitch_t = math.radians(pitch0_deg); T_now = drag0
    for k in range(n):
        rb.wind_ned = wind.step(rb.t, dt)
        if pilot is not None and rb.breakdown:
            pitch_t, T_now = pilot.update(rb, dt)
        rb.step(dt, detail=(k % every == 0))
        if k % every == 0:
            roll, pitch, yaw = q_to_euler(rb.q)
            bd = rb.breakdown
            out["t"].append(round(rb.t, 3)); out["speed"].append(float(np.linalg.norm(rb.vel))); out["airspeed"].append(bd["airspeed"])
            out["alpha_deg"].append(bd["alpha"]); out["beta_deg"].append(bd["beta"])
            out["pitch_deg"].append(math.degrees(pitch)); out["roll_deg"].append(math.degrees(roll)); out["yaw_deg"].append(math.degrees(yaw))
            out["alt"].append(float(-rb.pos[2])); out["climb"].append(float(-rb.vel[2])); out["east"].append(float(rb.pos[1]))
            out["q_deg_s"].append(math.degrees(rb.rates[1])); out["p_deg_s"].append(math.degrees(rb.rates[0])); out["r_deg_s"].append(math.degrees(rb.rates[2]))
            out["lift"].append(bd["lift"]); out["drag"].append(bd["wing_drag"]); out["side"].append(bd["side"]); out["stalled"].append(False)
            w = rb.wind_ned
            out["wind_head"].append(float(-w[0])); out["wind_cross"].append(float(-w[1])); out["wind_up"].append(float(-w[2]))
            out["thrust"].append(float(bd["thrust"])); out["pitch_target_deg"].append(math.degrees(pitch_t) if pilot else None)
            out["M_ctrl"].append([round(float(x), 1) for x in rb.M_ctrl])
            out["pos"].append([round(float(x), 4) for x in rb.pos]); out["q"].append([round(float(x), 6) for x in rb.q])
            out["wing_forces"].append([{"F": [round(float(x), 2) for x in w["F"]], "pos": [round(float(x), 3) for x in w["pos"]]} for w in bd["wing_forces"]])
            if not rb.is_sane() or abs(math.degrees(pitch)) > 80 or abs(math.degrees(roll)) > 120 or rb.pos[2] > 0:
                ended = "tumbled" if (abs(math.degrees(pitch)) > 80 or abs(math.degrees(roll)) > 120) else ("hit the ground" if rb.pos[2] > 0 else "diverged")
                break
    verdict = _classify(out, ended, duration_s)
    if pilot is not None and not ended:
        alt_dev = float(np.max(np.abs(np.array(out["alt"]) - float(altitude_m))))
        sat = pilot.sat / max(1, n)
        held = alt_dev < 15.0 and float(np.max(np.abs(out["roll_deg"]))) < 45.0
        label = (f"held: altitude within {alt_dev:.0f} m, heading within {float(np.max(np.abs(out['yaw_deg']))):.0f} deg, "
                 f"thrust {float(np.mean(out['thrust'])):.0f} N average ({100 * float(np.mean(out['thrust'])) / tmax:.0f} % of the fans)"
                 if held else f"not held: altitude off by {alt_dev:.0f} m, max roll {float(np.max(np.abs(out['roll_deg']))):.0f} deg")
        sat_txt = ", ".join(f"{nm} {100 * v:.0f} %" for nm, v in zip(("roll", "pitch", "yaw", "thrust"), sat) if v > 0.01)
        verdict = {"stable": held, "label": label + (f"; control at its limit: {sat_txt}" if sat_txt else ""), "ratio": None,
                   "alt_dev_m": alt_dev, "saturation": sat.tolist()}
    out["ended"] = ended; out["verdict"] = verdict; out["longitudinal_only"] = False
    out["setup"] = {"speed_kmh": V * 3.6, "pitch0_deg": pitch0_deg, "crosswind_ms": crosswind_ms, "headwind_ms": headwind_ms,
                    "controller": controller, "wind_events": events, "turbulence_rms": turbulence_rms, "fans_max_N": fans_max,
                    "thrust_max_N": tmax, "pitch_trim_deg": pitch_trim, "pitch_trim_used_deg": pitch_trim_used, "lift_at_range_top_N": lift_top,
                    "weight_N": weight, "control_moments_Nm": (control_moments or [300.0, 400.0, 150.0]) if use_pilot else None,
                    "mass_kg": float(test.mass.mass), "cg": cgv.tolist(), "thrust_N": drag0, "duration_s": duration_s,
                    "damping_Nms_per_rad": np.diag(D).tolist(), "library": str(lib.dir.name), "attitudes": int(len(lib.results)),
                    "table_range": {"alpha": [table.amin, table.amax], "beta": [table.bmin, table.bmax]},
                    "clipped_steps": table.clipped, "outside_steps": rb.outside_steps, "steps": n,
                    "note": "static forces from the CFD table (interpolated in alpha and beta, scaled with dynamic pressure); rate damping from the strip-theory wings at release; beyond the computed range a flat-plate style increment is added (rough)"}
    out["summary"] = {
        "max_roll_deg": float(np.max(np.abs(out["roll_deg"]))) if out["roll_deg"] else None,
        "heading_change_deg": float(out["yaw_deg"][-1] - out["yaw_deg"][0]) if out["yaw_deg"] else None,
        "lateral_drift_m": float(out["east"][-1]) if out["east"] else None,
        "alt_change_m": float(out["alt"][-1] - out["alt"][0]) if out["alt"] else None,
        "pitch_range_deg": [float(np.min(out["pitch_deg"])), float(np.max(out["pitch_deg"]))] if out["pitch_deg"] else None,
        "beta_range_deg": [float(np.min(out["beta_deg"])), float(np.max(out["beta_deg"]))] if out["beta_deg"] else None,
    }
    return out
