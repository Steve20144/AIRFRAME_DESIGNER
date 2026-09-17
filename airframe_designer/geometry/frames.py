"""Small vector helpers shared by the geometry segments (pure Python + numpy, no simulation state)."""
from __future__ import annotations

import math

import numpy as np


def unit(v, fallback=(0.0, 0.0, -1.0)) -> list[float]:
    n = math.sqrt(sum(float(c) * float(c) for c in v))
    if n < 1e-9:
        return list(fallback)
    return [float(c) / n for c in v]


def rot_x(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def rot_y(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def rot_z(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def rodrigues(axis, angle: float) -> np.ndarray:
    """Rotation matrix for a rotation of ``angle`` radians about ``axis`` (right-hand rule)."""
    k = np.asarray(unit(axis, (1.0, 0.0, 0.0)), float)
    K = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]])
    return np.eye(3) + math.sin(angle) * K + (1.0 - math.cos(angle)) * (K @ K)


def tilt_cant_to_axis(tilt_deg: float, cant_deg: float, side: float = 1.0) -> list[float]:
    """Thrust axis from a forward tilt (from vertical, negative = backward) and a sideways cant.

    Cant is measured *outward* from the centreline: positive leans the axis away from the vehicle's
    centreline on the side the rotor sits (``side`` = sign of its y position; +1 on the centreline)."""
    t = math.radians(tilt_deg)
    c = math.radians(cant_deg) * (1.0 if side >= 0 else -1.0)
    return [round(math.sin(t), 6), round(math.sin(c) * math.cos(t), 6), round(-math.cos(c) * math.cos(t), 6)]


def axis_to_tilt_cant(axis, side: float = 1.0) -> tuple[float, float]:
    """Inverse of tilt_cant_to_axis (degrees)."""
    a = unit(axis)
    tilt = math.degrees(math.asin(max(-1.0, min(1.0, a[0]))))
    cant = 0.0 if math.hypot(a[1], a[2]) < 1e-9 else math.degrees(math.atan2(a[1], -a[2]))
    return tilt, cant * (1.0 if side >= 0 else -1.0)
