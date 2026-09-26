"""Fan vibration at the IMU: what spinning, imperfect fans do to the accelerometer and the gyro.

Sources, per active rotor (settings in ``design.vibration``, see docs/SCHEMA.md):
  * 1P imbalance: a residual unbalance U (g mm) spinning with the fan puts a force U w^2 rotating in the fan's disc
    plane, w = the fan speed in rad/s (the rotor's normalised speed times ``rpm_max``).
  * blade pass: a thrust ripple (a fraction of the fan's thrust) along the thrust axis at ``blades`` x the fan speed.

Path to the sensor:
  1. the rigid body about the CG: F/m plus the angular acceleration I^-1 (r x F) acting on the IMU's lever arm; the
     gyro sees the integral of that angular acceleration;
  2. optional frame resonances between the fans and the flight controller, and the flight controller's isolator
     (soft mount, base-excitation transmissibility);
  3. the sampler: each HIL_SENSOR carries the average over its interval, as PX4's integrated IMU samples do (a box
     filter: sinc attenuation and half an interval of lag). Tones above half the sensor rate therefore alias into
     the band PX4 sees, as they do through a real integrating IMU; the alias frequencies depend on the sensor rate.

Every tone is a phasor evaluated in closed form at the fan's current phase: a few 3-vectors per rotor per sensor
step, and nothing enters the rigid-body integration. At hundreds of hertz the forces move the vehicle by microns;
they matter only through the sensors (PX4's vibration metrics, clipping, the rate loop's D term, and the EKF height
drift that vibration rectification causes, see brain/lessons/fan-vibration-drifts-ekf-height.md).

Frequency responses use the signed tone frequency (a fan spinning the other way is a negative frequency), so the
phase of every response stays consistent for both spin directions.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field, fields

import numpy as np

G = 9.80665


@dataclass
class VibrationConfig:
    enabled: bool = False
    rpm_max: float | list[float] = 30000.0   # fan speed at full command (normalised speed 1), rpm; scalar or per rotor
    imbalance_gmm: float | list[float] = 1.0  # residual unbalance per fan, g mm (ISO 21940 G6.3 on a 100 g rotor at
                                              # 30000 rpm is 0.2 g mm; a hobby EDF off the shelf is often 1 to 5)
    blades: int | list[int] = 12
    blade_ripple: float | list[float] = 0.0   # thrust ripple amplitude at blade pass, fraction of the fan's thrust
    imu_pos: list[float] | None = None        # the flight controller's IMU, structural frame, m (None: at the CG)
    mount_hz: float = 0.0                     # isolator natural frequency, Hz (0: hard-mounted)
    mount_damping: float = 0.1                # isolator damping ratio
    frame_modes: list[dict] = field(default_factory=list)  # [{"hz", "damping", "gain"}]: frame resonances; the peak
                                              # amplification is about gain / (2 damping) at hz
    accel_rectification: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])  # DC accel bias per body
                                              # axis, m/s^2 per g^2 of total vibration (rms, at the IMU, unsampled)

    @classmethod
    def from_dict(cls, d: dict | None) -> "VibrationConfig":
        d = dict(d or {})
        return cls(**{k: v for k, v in d.items() if k in {f.name for f in fields(cls)}})


def _per_rotor(v, n: int, name: str) -> np.ndarray:
    a = np.atleast_1d(np.asarray(v, float))
    if a.size == 1:
        return np.full(n, float(a[0]))
    if a.size != n:
        raise ValueError(f"design.vibration.{name}: {a.size} values for {n} active rotors")
    return a


def _skew(v) -> np.ndarray:
    x, y, z = (float(c) for c in v)
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def _disc_basis(axis) -> tuple[np.ndarray, np.ndarray]:
    """Two unit vectors spanning the disc plane with u x v = axis."""
    a = np.asarray(axis, float); a = a / np.linalg.norm(a)
    ref = np.array([1.0, 0.0, 0.0]) if abs(a[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    u = np.cross(ref, a); u /= np.linalg.norm(u)
    return u, np.cross(a, u)


class VibrationModel:
    """Vibration added to the IMU samples, from an airframe and its ``design.vibration`` block."""

    def __init__(self, airframe, cfg: VibrationConfig | dict | None = None, seed: int = 1):
        self.cfg = cfg if isinstance(cfg, VibrationConfig) else VibrationConfig.from_dict(
            cfg if cfg is not None else (getattr(airframe, "design", None) or {}).get("vibration"))
        c = self.cfg
        airframe.resolve_mass()
        rotors = airframe.active_rotors()
        n = self.n = len(rotors)
        m = float(airframe.mass.mass)
        I_inv = np.linalg.inv(airframe.mass.tensor())
        cg = np.asarray(airframe.cg, float)
        p = (np.asarray(c.imu_pos, float) - cg) if c.imu_pos is not None else np.zeros(3)
        self.rpm_max = _per_rotor(c.rpm_max, n, "rpm_max")
        self.unbalance = _per_rotor(c.imbalance_gmm, n, "imbalance_gmm") * 1e-6      # kg m
        self.blades = _per_rotor(c.blades, n, "blades")
        self.ripple = _per_rotor(c.blade_ripple, n, "blade_ripple")
        self.spin = np.array([1.0 if r.km >= 0 else -1.0 for r in rotors])
        # per rotor: accel at the IMU and angular acceleration per unit force, applied to the imbalance's complex
        # direction (u - j v: Re{(u - j v) e^{j theta}} = u cos theta + v sin theta) and to the thrust axis
        self.acc_imb = np.zeros((n, 3), complex); self.alp_imb = np.zeros((n, 3), complex)
        self.acc_ax = np.zeros((n, 3)); self.alp_ax = np.zeros((n, 3))
        for i, r in enumerate(rotors):
            arm = np.asarray(r.pos, float) - cg
            alpha = I_inv @ _skew(arm)                           # angular acceleration per unit force
            acc = np.eye(3) / m - _skew(p) @ alpha               # F/m + alpha x p
            u, v = _disc_basis(r.duct_axis or r.axis)
            cdir = u - 1j * v
            self.acc_imb[i] = acc @ cdir; self.alp_imb[i] = alpha @ cdir
            ax = np.asarray(r.axis, float); ax = ax / np.linalg.norm(ax)
            self.acc_ax[i] = acc @ ax; self.alp_ax[i] = alpha @ ax
        rng = np.random.default_rng(seed + 7919)
        self.theta = rng.uniform(0.0, 2.0 * np.pi, n)          # fan angles, rad
        self.bp_phase = rng.uniform(0.0, 2.0 * np.pi, n)       # blade-pass ripple phase offsets
        self.rect = np.asarray(c.accel_rectification, float)
        self._t_prev: int | None = None
        self.last_rms_g = 0.0

    # ------------------------------------------------------------ responses
    def path_gain(self, f: np.ndarray) -> np.ndarray:
        """Frame resonances times the isolator, at signed frequencies ``f`` (Hz)."""
        f = np.asarray(f, float)
        h = np.ones(f.shape, complex)
        if not self.cfg.frame_modes and self.cfg.mount_hz <= 0:
            return h
        for mode in self.cfg.frame_modes:
            r = f / float(mode["hz"]); z = float(mode.get("damping", 0.05))
            h = h * (1.0 + float(mode.get("gain", 0.0)) * r * r / (1.0 - r * r + 2j * z * r))
        if self.cfg.mount_hz > 0:
            r = f / float(self.cfg.mount_hz); z = float(self.cfg.mount_damping)
            h = h * (1.0 + 2j * z * r) / (1.0 - r * r + 2j * z * r)
        return h

    @staticmethod
    def sample_gain(f: np.ndarray, dt: float) -> np.ndarray:
        """Average over the preceding interval dt: sinc attenuation and dt/2 of lag."""
        x = np.pi * np.asarray(f, float) * dt
        with np.errstate(divide="ignore", invalid="ignore"):
            sinc = np.where(x != 0, np.sin(x) / np.where(x != 0, x, 1.0), 1.0)
        return sinc * np.exp(-1j * x)

    def sources(self, omega_norm, thrust) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Every tone at the current fan speeds, before the path and the sampler: signed frequency (Hz), accel
        phasor (m/s^2, per axis), gyro phasor (rad/s, per axis) and phase (rad). The 1P rows come first (one per
        rotor), then the blade-pass rows when a ripple is set."""
        om = np.clip(np.asarray(omega_norm, float)[: self.n], 0.0, 1.0)
        f1 = self.spin * om * self.rpm_max / 60.0                # signed rev/s
        w1 = 2.0 * np.pi * f1
        nz = w1 != 0
        inv_jw1 = np.zeros(self.n, complex)
        inv_jw1[nz] = 1.0 / (1j * w1[nz])
        F = self.unbalance * w1 * w1                             # imbalance force amplitude, N
        f, xa, xg, ph = f1, self.acc_imb * F[:, None], self.alp_imb * (F * inv_jw1)[:, None], self.theta
        if self.ripple.any():
            T = self.ripple * np.asarray(thrust, float)[: self.n] * np.exp(1j * self.bp_phase)
            f = np.concatenate([f, self.blades * f1])
            xa = np.vstack([xa, self.acc_ax * T[:, None]])
            xg = np.vstack([xg, self.alp_ax * (T * inv_jw1 / self.blades)[:, None]])
            ph = np.concatenate([ph, self.blades * self.theta])
        return f, xa, xg, ph

    # --------------------------------------------------------------- sample
    def sample(self, sim, time_usec: int) -> tuple[np.ndarray, np.ndarray]:
        """Accel (m/s^2) and gyro (rad/s) vibration for the HIL sample ending at ``time_usec``."""
        dt = (time_usec - self._t_prev) * 1e-6 if self._t_prev is not None and time_usec > self._t_prev else 0.004
        self._t_prev = time_usec
        omega = getattr(sim, "omega", None)
        if omega is None or len(omega) < self.n or self.n == 0:
            return np.zeros(3), np.zeros(3)
        om = np.clip(np.asarray(omega, float)[: self.n], 0.0, 1.0)
        self.theta = (self.theta + (2.0 * np.pi * dt / 60.0) * self.spin * om * self.rpm_max) % (2.0 * np.pi)
        f, xa, xg, ph = self.sources(om, getattr(sim, "thrust", np.zeros(self.n)))
        h = self.path_gain(f)
        e = (h * np.exp(1j * ph) * self.sample_gain(f, dt))[:, None]
        acc = (xa * e).real.sum(axis=0)
        gyr = (xg * e).real.sum(axis=0)
        xh = xa * h[:, None]
        self.last_rms_g = math.sqrt(0.5 * float((xh.real ** 2 + xh.imag ** 2).sum())) / G
        if self.rect.any():
            acc = acc + self.rect * self.last_rms_g ** 2
        return acc, gyr


def vibration_from_airframe(airframe, seed: int = 1) -> VibrationModel | None:
    """The model for ``airframe`` when its ``design.vibration`` is enabled, else None."""
    d = (getattr(airframe, "design", None) or {}).get("vibration")
    if not d or not d.get("enabled"):
        return None
    return VibrationModel(airframe, d, seed=seed)


class VibrationMetrics:
    """PX4's own vibration metrics (VehicleIMU: 0.99 / 0.01 filtered length of the sample-to-sample change) and its
    clip counters, on the samples we send. In SITL each HIL_SENSOR is one PX4 sample, so these equal what PX4
    computes; the aircraft computes them at its own IMU rate, so compare against a log at the same sensor rate."""

    ACCEL_CLIP = 16.0 * G * 0.999                  # SimulatorMavlink's FIFO ranges and PX4Accelerometer's clip limit
    GYRO_CLIP = math.radians(2000.0) * 0.999

    def __init__(self):
        self.reset()

    def reset(self) -> None:
        self.accel = 0.0; self.gyro = 0.0
        self.accel_clipping = 0; self.gyro_clipping = 0
        self._a = None; self._g = None

    def update(self, accel: np.ndarray, gyro: np.ndarray) -> None:
        if self._a is not None:
            da = accel - self._a; dg = gyro - self._g
            self.accel = 0.99 * self.accel + 0.01 * math.sqrt(float(da @ da))
            self.gyro = 0.99 * self.gyro + 0.01 * math.sqrt(float(dg @ dg))
        self._a = accel.copy(); self._g = gyro.copy()
        self.accel_clipping += int((np.abs(accel) >= self.ACCEL_CLIP).sum())
        self.gyro_clipping += int((np.abs(gyro) >= self.GYRO_CLIP).sum())

    def status(self) -> dict:
        return {"accel_metric": round(self.accel, 4), "gyro_metric": round(self.gyro, 5),
                "accel_clipping": self.accel_clipping, "gyro_clipping": self.gyro_clipping}


def bench(airframe, omega, rate: float = 250.0, seconds: float = 10.0, seed: int = 1, cfg: dict | None = None,
          noise=None) -> dict:
    """Hold the fans at normalised speeds ``omega`` (no PX4, no flight) and report what the IMU would see at
    ``rate`` Hz: PX4's vibration metrics (with the default sensor noise on top unless ``noise`` is False), the rms
    vibration at the IMU, clipping, and each tone's frequency and where it aliases to."""
    from .models import SensorNoise
    d = dict((getattr(airframe, "design", None) or {}).get("vibration") or {})
    d.update(cfg or {})
    d["enabled"] = True
    vm = VibrationModel(airframe, d, seed=seed)
    om = np.clip(np.asarray(omega, float), 0.0, 1.0)
    rotors = airframe.active_rotors()
    if om.size != vm.n:
        raise ValueError(f"{om.size} fan speeds for {vm.n} active rotors")
    sim = type("BenchState", (), {})()
    sim.omega = om
    sim.thrust = np.array([r.effective_max_thrust() for r in rotors]) * om ** np.array([r.thrust_exponent for r in rotors])
    nz = SensorNoise() if noise is None else (noise or None)
    rng = np.random.default_rng(seed)
    clean, noisy = VibrationMetrics(), VibrationMetrics()
    gravity = np.array([0.0, 0.0, -G])                         # on the ground, level: the IMU also reads gravity
    for k in range(max(2, int(round(seconds * rate)))):
        a, g = vm.sample(sim, int(round((k + 1) * 1e6 / rate)))
        a = a + gravity
        clean.update(a, g)
        if nz is not None:
            noisy.update(a + rng.normal(0.0, nz.accel, 3), g + rng.normal(0.0, nz.gyro, 3))
    f, xa, xg, _ = vm.sources(om, sim.thrust)
    h = vm.path_gain(f)
    tones = []
    for i in range(len(f)):
        if abs(f[i]) < 1e-9:
            continue
        alias = abs(abs(f[i]) - rate * round(abs(f[i]) / rate))
        tones.append({"rotor": i % vm.n + 1, "source": "1P" if i < vm.n else "blade pass",
                      "hz": round(abs(float(f[i])), 1), "alias_hz": round(float(alias), 1),
                      "accel_amp": round(float(np.linalg.norm(np.abs(xa[i] * h[i]))), 4),
                      "gyro_amp": round(float(np.linalg.norm(np.abs(xg[i] * h[i]))), 5),
                      "sampled_gain": round(float(abs(vm.sample_gain(f[i:i + 1], 1.0 / rate)[0])), 4)})
    m = noisy if nz is not None else clean
    return {"rate_hz": rate, "seconds": seconds, "omega": [round(float(x), 4) for x in om], "rms_g": round(vm.last_rms_g, 4),
            "accel_metric": round(m.accel, 4), "gyro_metric": round(m.gyro, 5),
            "accel_metric_vibration_only": round(clean.accel, 4), "gyro_metric_vibration_only": round(clean.gyro, 5),
            "accel_clipping": clean.accel_clipping, "gyro_clipping": clean.gyro_clipping,
            "config": dict(vars(vm.cfg)), "tones": tones}
