"""Bluff-body drag of the fuselage: quadratic per body axis, acting at the drag centre, plus quadratic
rotational damping."""
from __future__ import annotations

import numpy as np

from ..geometry.body import Body


class BodyAero:
    def __init__(self, body: Body, cg):
        from .fastmath import skew
        self.drag_q = np.asarray(body.drag_quadratic, float)
        self.drag_ang = np.asarray(body.drag_angular, float)
        self.r = np.asarray(body.drag_center, float) - np.asarray(cg, float)
        self.S = skew(self.r)
        self.offset = bool(np.any(np.abs(self.r) > 1e-12))

    def forces(self, v_air_body: np.ndarray, rates: np.ndarray, detail: bool = True) -> tuple[np.ndarray, np.ndarray, float]:
        v = (v_air_body - self.S @ rates) if self.offset else v_air_body
        F = -self.drag_q * v * np.abs(v)
        M = -self.drag_ang * rates * np.abs(rates)
        if self.offset:
            M = M + self.S @ F
        return F, M, (float(np.sqrt(F @ F)) if detail else 0.0)
