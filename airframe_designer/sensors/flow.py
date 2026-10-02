"""Downward optical flow + ToF range sensor (Holybro H-FLOW style) -> HIL_OPTICAL_FLOW and DISTANCE_SENSOR.

Configured per airframe in ``design.flow_sensor``:
  {"enabled": true, "pos": [x, y, z] structural FRD m (the lens), "mount": "hover" | "structure",
   "extra_pitch_deg": 0, "rate": 50, "flow_noise": 0.02 rad/s per reading, "range_noise": 0.01 m, "range_min": 0.08,
   "range_max": 8.0, "fov_deg": 42}

"mount": "hover" points the optical axis along the hover frame's down (PX4's body z after SENS_BOARD_Y_OFF, which
is what EKF2 assumes); "structure" mounts it flat to the airframe (optical axis = structural +z), so in hover it looks
``hover_pitch_deg`` aft of vertical and PX4, which still assumes body z, gets tilted flow and a slant range.

What the sensor reports is the image-centre flow of a pinhole camera over a flat floor at NED z = 0, in PX4's sensor
convention (SimulatorMavlink / VehicleOpticalFlow): pixel_x = dtheta_x - v_y dt / d, pixel_y = dtheta_y + v_x dt / d,
with v the lens velocity and dtheta the rotation, both in the sensor frame, d the distance along the optical axis.
The integrated gyro fields carry the sensor-frame rotation over the same window (RH, like pixel_flow), so PX4
compensates with them directly. Left NaN, PX4's VehicleOpticalFlow integrates the raw sensor_gyro instead, which is
in the flight controller's mounting frame, NOT rotated by SENS_BOARD_*: with the board at SENS_BOARD_Y_OFF 23.85 a
yaw rate then leaks ~40 % into the x compensation, every turn reads as a sideways slide, and EKF2 nudges its
heading on each flow update (yaw std 1.6 vs 0.1 deg in hover).
"""
from __future__ import annotations

import math

import numpy as np


def sensor_axes(airframe, cfg: dict) -> np.ndarray:
    """Columns: the sensor's x, y, z (optical axis) in the structural frame."""
    th = math.radians((airframe.hover_pitch_deg if cfg.get("mount", "hover") == "hover" else 0.0) + float(cfg.get("extra_pitch_deg", 0.0)))
    x = np.array([math.cos(th), 0.0, math.sin(th)])       # hover-level forward, seen from the structure
    z = np.array([-math.sin(th), 0.0, math.cos(th)])      # hover-level down
    return np.column_stack([x, np.array([0.0, 1.0, 0.0]), z])


class FlowSensor:
    def __init__(self, airframe, cfg: dict, rng: np.random.Generator):
        self.cfg = cfg
        self.C = sensor_axes(airframe, cfg)
        cg = np.asarray(airframe.mass.cg, float)
        self.r = np.asarray(cfg["pos"], float) - cg            # lens from the CG, structural frame
        self.rate = float(cfg.get("rate", 50.0))
        self.flow_noise = float(cfg.get("flow_noise", 0.02))
        self.range_noise = float(cfg.get("range_noise", 0.01))
        self.rmin, self.rmax = float(cfg.get("range_min", 0.08)), float(cfg.get("range_max", 8.0))
        self.half_fov = math.radians(float(cfg.get("fov_deg", 42.0)) / 2)
        self.rng = rng
        self._acc = np.zeros(2); self._gyro_acc = np.zeros(3); self._t0 = None; self.last_range = float("nan")

    def geometry(self, sim):
        """Lens velocity and rotation in the sensor frame, and the distance along the optical axis to the floor."""
        R = sim.rotmat
        w = np.asarray(sim.rates, float)
        v_b = R.T @ np.asarray(sim.vel, float) + np.cross(w, self.r)
        p = np.asarray(sim.pos, float) + R @ self.r
        a_w = R @ self.C[:, 2]
        h = -p[2]
        d = h / a_w[2] if a_w[2] > 1e-3 else float("inf")
        return self.C.T @ v_b, self.C.T @ w, d, math.acos(max(-1.0, min(1.0, a_w[2])))

    def step(self, sim, dt: float):
        """Accumulate flow over dt; returns nothing (the messages are built in ``messages``)."""
        v, w, d, _ = self.geometry(sim)
        if not math.isfinite(d) or d <= 1e-3:
            return
        self._acc += np.array([w[0] - v[1] / d, w[1] + v[0] / d]) * dt
        self._gyro_acc += w * dt

    def messages(self, sim, time_usec: int, dt_int: float) -> tuple[dict, dict]:
        v, w, d, tilt = self.geometry(sim)
        # flow_noise is the rate noise of one reading (rad/s): the integrated angle gets flow_noise * dt. (It used to
        # be scaled by sqrt(dt) as a density, which at 50 Hz made each reading 7x noisier, 0.14 rad/s: ~0.37 m/s of
        # apparent motion at 2.6 m, and EKF2's heading jumped 1-2 deg on every flow update.)
        n = self.rng.normal(0.0, self.flow_noise, 2) * dt_int if self.flow_noise > 0 else np.zeros(2)
        flow = self._acc + n
        gyro = self._gyro_acc
        self._acc = np.zeros(2); self._gyro_acc = np.zeros(3)
        dist = d + (self.rng.normal(0.0, self.range_noise + 0.005 * d) if self.range_noise > 0 else 0.0)
        valid = math.isfinite(dist) and self.rmin <= dist <= self.rmax and tilt < math.pi / 2 - 0.05
        self.last_range = dist if valid else float("nan")
        quality = 255 if valid else 0
        of = dict(time_usec=int(time_usec), sensor_id=0, integration_time_us=int(round(dt_int * 1e6)),
                  integrated_x=float(flow[0]), integrated_y=float(flow[1]),
                  integrated_xgyro=float(gyro[0]), integrated_ygyro=float(gyro[1]), integrated_zgyro=float(gyro[2]),
                  temperature=2500, quality=quality, time_delta_distance_us=int(round(dt_int * 1e6)),
                  distance=float(dist) if valid else -1.0)
        ds = dict(time_boot_ms=int(time_usec // 1000), min_distance=int(round(self.rmin * 100)), max_distance=int(round(self.rmax * 100)),
                  current_distance=int(round(max(0.0, dist) * 100)) if valid else int(round(self.rmax * 100 + 1)),
                  type=0, id=1, orientation=25, covariance=int(round((self.range_noise + 0.005 * max(d, 0)) ** 2 * 1e4)) or 1,
                  horizontal_fov=float(2 * self.half_fov), vertical_fov=float(2 * self.half_fov), signal_quality=100 if valid else 0)
        return of, ds


def flow_from_airframe(airframe, rng) -> FlowSensor | None:
    cfg = (getattr(airframe, "design", None) or {}).get("flow_sensor") if isinstance(getattr(airframe, "design", None), dict) else None
    if not cfg or not cfg.get("enabled", True):
        return None
    return FlowSensor(airframe, cfg, rng)


def ekf2_params(airframe, cfg: dict) -> dict:
    """PX4 parameters for flow + range navigation without GPS, the sensor offsets in the hover frame from the CG."""
    th = math.radians(airframe.hover_pitch_deg)
    Ry = np.array([[math.cos(th), 0, math.sin(th)], [0, 1, 0], [-math.sin(th), 0, math.cos(th)]])   # structure -> hover frame
    r = Ry @ (np.asarray(cfg["pos"], float) - np.asarray(airframe.mass.cg, float))
    # the optical axis seen from PX4's body (hover) frame: a "structure" mount tilted by extra_pitch_deg looks
    # (hover_pitch - extra_pitch) off body down; PX4 takes that for the range sensor, positive towards the nose
    z = Ry @ sensor_axes(airframe, cfg)[:, 2]
    rng_pitch = math.atan2(float(z[0]), float(z[2]))
    return {"EKF2_RNG_PITCH": round(rng_pitch, 4), "EKF2_GPS_CTRL": 0, "EKF2_OF_CTRL": 1, "EKF2_RNG_CTRL": 1, "EKF2_HGT_REF": 2,
            "EKF2_OF_POS_X": round(float(r[0]), 3), "EKF2_OF_POS_Y": round(float(r[1]), 3), "EKF2_OF_POS_Z": round(float(r[2]), 3),
            "EKF2_RNG_POS_X": round(float(r[0]), 3), "EKF2_RNG_POS_Y": round(float(r[1]), 3), "EKF2_RNG_POS_Z": round(float(r[2]), 3),
            "EKF2_IMU_POS_X": 0.0, "EKF2_IMU_POS_Y": 0.0, "EKF2_IMU_POS_Z": 0.0,
            "SENS_FLOW_ROT": 0, "SENS_FLOW_MINHGT": float(cfg.get("range_min", 0.08)), "SENS_FLOW_MAXHGT": float(cfg.get("range_max", 8.0)),
            "COM_ARM_WO_GPS": 1, "EKF2_OF_DELAY": 0.0, "EKF2_RNG_DELAY": 0.0}
