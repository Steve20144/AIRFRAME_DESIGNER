"""Nose-lift ground sequence for aircraft that hover nose-up.

An airframe that parks nose-down (or level) but hovers at ``hover_pitch_deg`` cannot simply be armed: PX4 treats the
hover attitude as level, so on the ground it would demand full pitch torque from every motor at once. Instead the
simulator raises the nose *before* PX4 is armed, using only the chosen (front) motors, pivoting about the rear feet:

  1. ramping   the pitch target rises from the parked attitude at ``rate_deg_s``; a PID on pitch drives the chosen
               motors (as a per-motor *floor* under PX4's commands, which are zero while disarmed)
  2. holding   the nose is at the target; the app/scenario now arms PX4 and requests takeoff
  3. handover  PX4 is armed and spooling: the floor stays until PX4's own commands for those motors reach it (or
               a timeout), then fades out over ``fade_s``
  4. done / failed (nose overshoot, vehicle left the ground early, timeout)

PX4 needs no changes: while disarmed it only sees its attitude change, exactly as if the aircraft were tilted by hand.
"""
from __future__ import annotations

import math

import numpy as np


class NoseLift:
    def __init__(self, motors: list[int], target_pitch_deg: float, rate_deg_s: float = 8.0, kp: float = 0.0,
                 ki: float = 0.0, kd: float = 0.0, k_ang: float = 1.0, kq: float = 0.02, kqi: float = 0.012, max_cmd: float = 1.0, tolerance_deg: float = 2.0,
                 hold_s: float = 0.6, fade_s: float = 2.0, handover_timeout_s: float = 8.0, timeout_s: float = 25.0,
                 assist_motors: list[int] | None = None, assist_cmd: float = 0.0, k_rate: float = 3.0):
        self.k_rate = float(k_rate)
        self.split = None
        self.motors = [int(m) for m in motors]
        self.assist_motors = [int(m) for m in (assist_motors or [])]   # e.g. rear jets at a low idle to carry some weight
        self.assist_cmd = float(assist_cmd)
        self.target = float(target_pitch_deg)
        self.rate = float(rate_deg_s)
        self.kp, self.ki, self.kd = kp, ki, kd          # kept for API compatibility (unused by the rate loop)
        self.k_ang, self.kq, self.kqi = float(k_ang), float(kq), float(kqi)
        self.max_cmd = float(max_cmd)
        self.tol = float(tolerance_deg)
        self.hold_s, self.fade_s = hold_s, fade_s
        self.handover_timeout = handover_timeout_s
        self.timeout = timeout_s
        self.state = "ramping"
        self.reason = ""
        self.t0 = None
        self.ramp_target = None
        self.integral = 0.0
        self.cmd = 0.0
        self.hold_since = None
        self.handover_since = None
        self.fade_start = None
        self.fade_from = 0.0
        self.pitch = 0.0
        self._t = 0.0
        self.history: list[tuple[float, float, float]] = []   # (t, pitch, cmd)

    # ------------------------------------------------------------ helpers
    def floor(self, n: int) -> np.ndarray | None:
        if self.state in ("done", "failed"):
            return None
        f = np.zeros(n)
        split = getattr(self, "split", None)
        for k, m in enumerate(self.motors):
            if 0 <= m < n:
                f[m] = self.cmd * (split[k] if split is not None and k < len(split) else 1.0)
        scale = 1.0 if self.fade_start is None else max(0.0, 1.0 - (self._t - self.fade_start) / max(self.fade_s, 1e-3))
        for m in self.assist_motors:
            if 0 <= m < n:
                f[m] = max(f[m], self.assist_cmd * scale)
        return f

    def _finish(self, state: str, reason: str = "") -> None:
        self.state, self.reason, self.cmd = state, reason, 0.0

    # --------------------------------------------------------------- hook
    def _update_split(self, s, rates) -> None:
        """Per-motor thrust multipliers so that the lifting motors produce no net yaw or roll moment about the CG
        (motors at different stations with opposite cants would otherwise twist the aircraft on its feet), with
        yaw/roll rate damping on top. Multipliers are normalised to a mean of 1 so the pitch feed-forward holds."""
        ms = [m for m in self.motors if m < s.rotors.n]
        if len(ms) < 2:
            self.split = None
            return
        A = np.array([[float(np.cross(s.rotors.r[m], s.rotors.axis[m])[0] - s.rotors.km[m] * s.rotors.axis[m][0]),
                       float(np.cross(s.rotors.r[m], s.rotors.axis[m])[2] - s.rotors.km[m] * s.rotors.axis[m][2])] for m in ms])   # roll, yaw per unit thrust
        T = np.array([s.rotors.tmax[m] * s.rotors.scale[m] for m in ms])
        # minimum-deviation weights w (thrust = w * T_i * frac) with zero net yaw moment (and zero roll moment
        # too when there are enough motors to satisfy both without switching any off)
        M = A * T[:, None]                       # moments per unit weight: [:, 0] roll, [:, 1] yaw
        w = np.ones(len(ms))
        cols = [1] if len(ms) < 3 else [0, 1]
        Mc = M[:, cols]
        try:
            lam = np.linalg.lstsq(Mc.T @ Mc + 1e-9 * np.eye(len(cols)), Mc.T @ w, rcond=None)[0]
            w = w - Mc @ lam
        except np.linalg.LinAlgError:
            pass
        # rate damping: push against roll (p) and yaw (r) rates through the same moment map
        p, r = float(rates[0]), float(rates[2])
        w = w - self.k_rate * (M[:, 0] * p + M[:, 1] * r) / max(float(np.abs(M).max()), 1e-9)
        w = np.clip(w, 0.2, 1.8)
        self.split = (w / w.mean()).tolist()

    def _thrust_to_cmd(self, s, frac: float) -> float:
        """Thrust fraction -> motor command for the lifting motors (thrust = max * cmd^n)."""
        n = float(np.mean([s.rotors.exponent[m] for m in self.motors if m < s.rotors.n])) if self.motors else 2.0
        return float(np.clip(max(frac, 0.0), 0.0, self.max_cmd) ** (1.0 / max(n, 1e-3)))

    def _balance_fraction(self, s) -> float:
        """Thrust fraction on the lifting motors that exactly balances the weight about the rear feet at the
        current attitude (static moment balance, computed from the geometry every step)."""
        R = s.rotmat
        feet = s.legs.r if s.legs.n else np.zeros((0, 3))
        rear = feet[feet[:, 0] < 0.0] if len(feet) else feet
        if len(rear) == 0:
            return 0.5
        piv = (rear.mean(axis=0)) @ R.T                       # pivot point relative to the CG, NED
        axis = R[:, 1]                                         # body pitch axis in NED
        W = s.mass * 9.80665
        tau_g = float(np.cross(-piv, np.array([0.0, 0.0, W])) @ axis)   # gravity moment about the pivot
        a = 0.0
        split = self.split or [1.0] * len(self.motors)
        for k, m in enumerate(self.motors):
            if m < s.rotors.n:
                r_ned = s.rotors.r[m] @ R.T
                d_ned = s.rotors.axis[m] @ R.T
                a += float(np.cross(r_ned - piv, d_ned) @ axis) * s.rotors.tmax[m] * s.rotors.scale[m] * split[k]
        if abs(a) < 1e-9:
            return 0.5
        return float(np.clip(-tau_g / a, 0.0, 2.0))

    def __call__(self, simr) -> None:
        if self.state in ("done", "failed"):
            return
        s = simr.sim
        t = simr.t
        self._t = t
        dt = 1.0 / simr.sensor_rate
        self.pitch = math.degrees(s.euler[1])
        q = math.degrees(s.rates[1])
        self._update_split(s, s.rates)
        if self.t0 is None:
            self.t0 = t
            self.ramp_target = self.pitch
            self.pitch0 = self.pitch
            self.ff = 0.0            # thrust fraction that just starts moving the nose (found by the slow ramp)
            self.frac = 0.0
            self.moving = False
        if simr.step_count % 10 == 0:
            self.history.append((round(t, 3), round(self.pitch, 2), round(self.cmd, 3)))
            if len(self.history) > 4000:
                self.history = self.history[-4000:]
        link = simr.link
        armed = bool(link is not None and getattr(link, "actuator_armed", False))
        px4_cmd = np.asarray(getattr(link, "actuators", [0.0] * 16), float) if link is not None else np.zeros(16)

        if self.state == "ramping":
            if t - self.t0 > self.timeout:
                self._finish("failed", f"nose did not reach {self.target:g} deg in {self.timeout:g} s (at {self.pitch:.1f})"); return
            if self.pitch > self.target + 12.0:
                self._finish("failed", f"nose overshot to {self.pitch:.1f} deg"); return
            if not s.on_ground and self.pitch < self.target - 5.0:
                self._finish("failed", "vehicle left the ground before the nose was up (front motors too strong or CG too far back)"); return
            self.ff = self._balance_fraction(s)
            if self.ff > self.max_cmd + 0.02 and t - self.t0 > 2.0 and self.pitch - self.pitch0 < 1.0:
                self._finish("failed", f"the chosen motors cannot hold the nose here (need {self.ff * 100:.0f}% of their thrust); "
                                       f"add an idle on other motors, move the CG aft or the rear feet forward"); return
            # rate loop: the nose is asked to rotate at ``rate`` deg/s (easing in over 1.5 s, decelerating
            # proportionally over the last degrees), and the thrust regulates the measured pitch rate around that
            # on top of the geometric balance feed-forward
            ease = min(1.0, (t - self.t0) / 1.5)
            q_des = float(np.clip(self.k_ang * (self.target - self.pitch), -self.rate, self.rate * ease))
            eq = q_des - q
            self.integral = float(np.clip(self.integral + eq * dt, -40.0, 40.0))
            self.frac = ease * self.ff + self.kq * eq + self.kqi * self.integral
            self.ramp_target = self.pitch
            at_target = abs(self.pitch - self.target) < self.tol and abs(q) < 3.0
            if at_target:
                self.hold_since = self.hold_since or t
                if t - self.hold_since >= self.hold_s:
                    self.state = "holding"
            else:
                self.hold_since = None
            self.cmd = self._thrust_to_cmd(s, self.frac)
            if armed:
                self.state = "handover"; self.handover_since = t
            return
        if self.state == "holding":
            self.ff = self._balance_fraction(s)
            q_des = float(np.clip(self.k_ang * (self.target - self.pitch), -self.rate, self.rate))
            eq = q_des - q
            self.integral = float(np.clip(self.integral + eq * dt, -40.0, 40.0))
            self.frac = self.ff + self.kq * eq + self.kqi * self.integral
            self.cmd = self._thrust_to_cmd(s, self.frac)
            if armed:
                self.state = "handover"; self.handover_since = t
            return
        if self.state == "handover":
            # keep holding the nose with the same loop until PX4 itself drives the lifting motors at least as hard
            # as the hold (leaving the ground is not enough: PX4's spool-up would otherwise drop the nose), or
            # after the handover timeout; then fade out
            q_des = float(np.clip(self.k_ang * (self.target - self.pitch), -self.rate, self.rate))
            eq = q_des - q
            self.integral = float(np.clip(self.integral + eq * dt, -40.0, 40.0))
            hold = self._thrust_to_cmd(s, self._balance_fraction(s) + self.kq * eq + self.kqi * self.integral)
            px4_share = min((px4_cmd[m] if m < len(px4_cmd) else 0.0) for m in self.motors) if self.motors else 0.0
            taken_over = px4_share >= 0.95 * hold
            if self.fade_start is None and (taken_over or t - self.handover_since > self.handover_timeout or not armed):
                self.fade_start, self.fade_from = t, hold
            if self.fade_start is None:
                self.cmd = hold
            else:
                k = (t - self.fade_start) / max(self.fade_s, 1e-3)
                self.cmd = float(self.fade_from * max(0.0, 1.0 - k))
                if k >= 1.0:
                    self._finish("done", "PX4 took over" if armed else "disarmed during handover"); return

    def status(self) -> dict:
        return {"state": self.state, "reason": self.reason, "pitch_deg": round(self.pitch, 2), "target_deg": self.target,
                "ramp_target_deg": None if self.ramp_target is None else round(self.ramp_target, 2), "cmd": round(self.cmd, 3),
                "motors": self.motors}


def uses_firmware(design: dict | None) -> bool:
    """True when design.nose_lift runs on the flight controller (the nose_lift PX4 module, firmware/px4_ext) instead
    of in the simulator: ``"executor": "firmware"``. The simulator's lift and the app's switch watcher then stand
    down, and the airframe export carries the NL_* parameters."""
    nl = (design or {}).get("nose_lift") or {}
    return bool(nl.get("enabled", True)) and str(nl.get("executor", "sim")).lower() == "firmware"


def firmware_params(airframe) -> dict[str, float | int]:
    """NL_* parameters for the nose_lift PX4 module, from the same geometry the simulator's NoseLift uses: the
    pitch moment of each lifting motor about the rear feet, the static roll/yaw-cancelling thrust weights and the
    moments the rate damping works through (structural frame, relative to the CG). Plus the PX4 settings the
    sequence needs: a kill switch that disarms at once, and no pre-takeoff auto-disarm during a slow lift."""
    from ..dynamics.rigid_body import RigidBody
    d = ((getattr(airframe, "design", None) or {}).get("nose_lift") or {})
    body = RigidBody(airframe)
    rot, legs = body.rotors, body.legs
    motors = sorted({int(m) for m in d.get("motors") or [] if 0 <= int(m) < rot.n})
    if not motors:
        raise ValueError("design.nose_lift.motors is empty: choose the lifting motors")
    if len(motors) > 4:
        raise ValueError(f"the nose_lift module drives at most 4 lifting motors, not {len(motors)}")
    rear = legs.r[legs.r[:, 0] < 0.0] if legs.n else np.zeros((0, 3))
    if len(rear) == 0:
        raise ValueError("no rear feet (legs aft of the CG) to pivot about")
    piv = rear.mean(axis=0)
    T = np.array([rot.tmax[m] * rot.scale[m] for m in motors])
    A_ry = np.array([[float(np.cross(rot.r[m], rot.axis[m])[0] - rot.km[m] * rot.axis[m][0]),
                      float(np.cross(rot.r[m], rot.axis[m])[2] - rot.km[m] * rot.axis[m][2])] for m in motors])
    M = A_ry * T[:, None]                                    # roll, yaw moment at full thrust
    w = np.ones(len(motors))
    if len(motors) >= 2:                                     # NoseLift._update_split at zero rates
        cols = [1] if len(motors) < 3 else [0, 1]
        Mc = M[:, cols]
        lam = np.linalg.lstsq(Mc.T @ Mc + 1e-9 * np.eye(len(cols)), Mc.T @ w, rcond=None)[0]
        w = w - Mc @ lam
    A = [float(np.cross(rot.r[m] - piv, rot.axis[m])[1]) * float(T[k]) for k, m in enumerate(motors)]
    p: dict[str, float | int] = {
        "NL_EN": 1,
        "NL_MOT_MSK": int(sum(1 << m for m in motors)),
        "NL_HOV_PITCH": round(float(airframe.hover_pitch_deg), 3),
        "NL_TGT": round(float(d.get("target_pitch_deg", airframe.hover_pitch_deg)), 3),
        "NL_RATE": float(d.get("rate_deg_s", 8.0)),
        "NL_K_ANG": float(d.get("k_ang", 1.0)),
        "NL_KQ": float(d.get("kq", 0.02)),
        "NL_KQI": float(d.get("kqi", 0.012)),
        "NL_K_RATE": float(d.get("k_rate", 3.0)),
        "NL_MAX_CMD": float(d.get("max_cmd", 1.0)),
        "NL_TOL": float(d.get("tolerance_deg", 2.0)),
        "NL_HOLD_S": float(d.get("hold_s", 0.6)),
        "NL_FADE_S": float(d.get("fade_s", 2.0)),
        "NL_HO_TOUT": float(d.get("handover_timeout_s", 8.0)),
        "NL_TOUT": float(d.get("timeout_s", 25.0)),
        "NL_EXPO": round(float(np.mean([rot.exponent[m] for m in motors])), 4),
        "NL_WEIGHT": round(float(body.mass) * 9.80665, 4),
        "NL_PIV_X": round(float(piv[0]), 5), "NL_PIV_Y": round(float(piv[1]), 5), "NL_PIV_Z": round(float(piv[2]), 5),
        "NL_RC_CH": int(d.get("rc_channel", 0) or 0),
        "NL_RC_TH": int(d.get("rc_threshold", 1500)),
        "NL_RC_LOW": int(bool(d.get("rc_active_low", False))),
        # a kill (the transmitter's push button) disarms at once, so letting go of it never restarts the motors
        "COM_KILL_DISARM": 0.0,
        # PX4 disarms 11 s after arming without a takeoff; the lift and hold take longer
        "COM_DISARM_PRFLT": 120.0,
    }
    for k in range(4):
        p[f"NL_A{k}"] = round(A[k], 5) if k < len(motors) else 0.0
        p[f"NL_W{k}"] = round(float(w[k]), 5) if k < len(motors) else 1.0
        p[f"NL_MR{k}"] = round(float(M[k, 0]), 5) if k < len(motors) else 0.0
        p[f"NL_MY{k}"] = round(float(M[k, 1]), 5) if k < len(motors) else 0.0
    for key, name in (("handover_throttle", "NL_HO_THR"), ("hold_timeout_s", "NL_HOLD_TOUT"),
                      ("roll_max_deg", "NL_ROLL_MAX"), ("liftoff_dz_m", "NL_LIFT_DZ")):
        if key in d:
            p[name] = float(d[key])
    return p


class NoseLower(NoseLift):
    """The takeoff sequence run backwards, for landing. The aircraft touches down at its hover attitude (rear feet
    first); at that moment the sequence takes over *all* motor commands: every motor except the chosen (front) ones
    is cut, and the chosen ones lower the nose to ``target_pitch_deg`` (the parked attitude) at ``rate_deg_s`` with
    the same balance-plus-rate loop as the lift, then fade out. States:

      waiting   armed in the air, no influence on the motors; triggers when the vehicle has been airborne above
                ``min_airborne_alt`` and touches the ground (or immediately with ``wait_touchdown=False``)
      lowering  overriding PX4: rear motors 0, front motors on the loop, nose descending at the rate
      settling  at the target: fade the front motors out over ``fade_s``
      done / failed

    PX4 still believes it is flying when the takeover happens, so the simulator force-disarms it when the sequence
    finishes; a real aircraft needs the same logic in firmware.
    """

    def __init__(self, motors: list[int], target_pitch_deg: float, rate_deg_s: float = 3.0, wait_touchdown: bool = True,
                 min_airborne_alt: float = 0.8, wait_timeout_s: float = 180.0, timeout_s: float = 40.0, fade_s: float = 2.0,
                 tolerance_deg: float = 1.5, takeover_boost: float = 1.15, kq: float = 0.3, kqi: float = 0.1,
                 rear_fade_s: float = 0.0, **kw):
        super().__init__(motors, target_pitch_deg, rate_deg_s=rate_deg_s, timeout_s=timeout_s, fade_s=fade_s,
                         tolerance_deg=tolerance_deg, kq=kq, kqi=kqi, **kw)
        self.kind = "lower"
        self.rear_fade_s = float(rear_fade_s)           # the other motors fade from their PX4 command to 0 over this
                                                        # (0 = cut at once; a fade pushes the tail up and the nose down)
        self.rear_cmd0 = None
        self.takeover_boost = float(takeover_boost)     # front thrust above static balance at the takeover instant,
                                                        # so cutting the rear motors does not drop the nose
        self.wait_touchdown = bool(wait_touchdown)
        self.min_airborne_alt = float(min_airborne_alt)
        self.wait_timeout = float(wait_timeout_s)
        self.override_all = not self.wait_touchdown          # the simulator replaces PX4's commands while True
        self.state = "waiting" if self.wait_touchdown else "lowering"
        self.was_airborne = not self.wait_touchdown
        self.touchdown_t = None
        self.touchdown_speed = None
        self.touchdown_pitch = None
        self.touchdown_alt = None
        self.t_wait0 = None
        self.rate_max = 0.0

    def floor(self, n: int) -> np.ndarray | None:
        if self.state in ("done", "failed"):
            return None
        f = np.zeros(n)
        if not self.override_all:
            return f                                          # waiting: max(PX4, 0) leaves PX4 alone
        if self.rear_cmd0 is not None and self.t0 is not None:
            k = 1.0 - (self._t - self.t0) / max(self.rear_fade_s, 1e-3)
            if k > 0.0:
                f[:] = np.asarray(self.rear_cmd0, float)[:n] * k
        split = getattr(self, "split", None)
        for k, m in enumerate(self.motors):
            if 0 <= m < n:
                f[m] = self.cmd * (split[k] if split is not None and k < len(split) else 1.0)
        return f

    def __call__(self, simr) -> None:
        if self.state in ("done", "failed"):
            return
        s = simr.sim
        t = simr.t
        self._t = t
        dt = 1.0 / simr.sensor_rate
        self.pitch = math.degrees(s.euler[1])
        q = math.degrees(s.rates[1])
        alt = float(-s.pos[2])
        if self.rear_cmd0 is None and self.state == "waiting":
            link = simr.link
            px4 = np.asarray(getattr(link, "actuators", [0.0] * 16), float) if link is not None else np.zeros(16)
            self._px4_last = np.array(px4[: s.rotors.n], float)
        self._update_split(s, s.rates)
        if self.t_wait0 is None:
            self.t_wait0 = t
        if simr.step_count % 10 == 0:
            self.history.append((round(t, 3), round(self.pitch, 2), round(self.cmd, 3)))
            if len(self.history) > 4000:
                self.history = self.history[-4000:]

        if self.state == "waiting":
            if not s.on_ground and alt > self.min_airborne_alt:
                self.was_airborne = True
            if self.was_airborne and s.on_ground:
                self.touchdown_t, self.touchdown_speed = t, float(np.linalg.norm(s.vel))
                self.touchdown_pitch, self.touchdown_alt = self.pitch, alt
                self.state, self.override_all, self.t0, self.integral = "lowering", True, t, 0.0
                self.rear_cmd0 = getattr(self, "_px4_last", np.zeros(s.rotors.n)).copy()
                for m in self.motors:
                    if 0 <= m < len(self.rear_cmd0):
                        self.rear_cmd0[m] = 0.0
                self.ff = self._balance_fraction(s)
                self.cmd = self._thrust_to_cmd(s, self.ff * self.takeover_boost)
                self.integral = (self.takeover_boost - 1.0) * self.ff / max(self.kqi, 1e-6)   # loop starts where the boost is
            elif t - self.t_wait0 > self.wait_timeout:
                self._finish("failed", f"no touchdown within {self.wait_timeout:g} s")
            return
        if self.t0 is None:
            self.t0, self.integral = t, 0.0
            self.touchdown_pitch, self.touchdown_alt = self.pitch, alt
        if self.state == "lowering":
            self.rate_max = max(self.rate_max, abs(q))
            if t - self.t0 > self.timeout:
                self._finish("failed", f"nose did not come down to {self.target:g} deg in {self.timeout:g} s (at {self.pitch:.1f})"); return
            if self.pitch < self.target - 10.0:
                self._finish("failed", f"nose dropped to {self.pitch:.1f} deg (target {self.target:g})"); return
            if self.touchdown_alt is not None and alt > self.touchdown_alt + 0.6:
                self._finish("failed", "vehicle lifted off again during the nose lower"); return
            self.ff = self._balance_fraction(s)
            ease = min(1.0, (t - self.t0) / 1.0)
            q_des = float(np.clip(self.k_ang * (self.target - self.pitch), -self.rate * ease, self.rate))
            eq = q_des - q
            self.integral = float(np.clip(self.integral + eq * dt, -40.0, 40.0))
            self.frac = self.ff + self.kq * eq + self.kqi * self.integral
            self.cmd = self._thrust_to_cmd(s, self.frac)
            if abs(self.pitch - self.target) < self.tol and abs(q) < 3.0:
                self.hold_since = self.hold_since or t
                if t - self.hold_since >= self.hold_s:
                    self.state, self.fade_start, self.fade_from = "settling", t, self.cmd
            else:
                self.hold_since = None
            return
        if self.state == "settling":
            k = (t - self.fade_start) / max(self.fade_s, 1e-3)
            self.cmd = float(self.fade_from * max(0.0, 1.0 - k))
            if k >= 1.0:
                self._finish("done", f"nose down at {self.pitch:.1f} deg, {t - self.t0:.1f} s after touchdown")

    def status(self) -> dict:
        d = super().status()
        d.update({"kind": "lower", "touchdown_speed": None if self.touchdown_speed is None else round(self.touchdown_speed, 2),
                  "touchdown_pitch_deg": None if self.touchdown_pitch is None else round(self.touchdown_pitch, 1),
                  "rate_max_deg_s": round(self.rate_max, 1)})
        return d
