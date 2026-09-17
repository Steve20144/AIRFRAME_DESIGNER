"""Ground contact at the feet: spring-damper normal force with Coulomb friction, per leg with its own stiffness,
damping and friction. Vectorised over all legs. The ground is the plane z = 0 (NED, down positive)."""
from __future__ import annotations

import numpy as np

from ..geometry.gear import Leg


class LegContacts:
    def __init__(self, legs: list[Leg], cg):
        from ..aero.fastmath import skew_stack
        cg = np.asarray(cg, float)
        self.n = len(legs)
        self.r = np.array([l.foot() for l in legs], float).reshape(self.n, 3) - cg     # feet relative to the CG
        self.radius = np.array([l.foot_radius for l in legs], float)
        self.k = np.array([l.stiffness for l in legs], float)
        self.c = np.array([l.damping for l in legs], float)
        self.mu = np.array([l.friction for l in legs], float)
        self.S = skew_stack(self.r)

    def rest_height(self, R: np.ndarray) -> float:
        """CG height above the ground (positive up) with the lowest foot touching, for attitude R (body->NED)."""
        if self.n == 0:
            return 0.0
        z = (self.r @ R.T)[:, 2] + self.radius
        return float(np.max(z))

    def effective_mass(self, mass: float, I_inv: np.ndarray, R: np.ndarray) -> np.ndarray:
        """Mass each foot 'sees' for a vertical push: 1 / (1/m + (r x n) I^-1 (r x n)), n = up in the body frame."""
        n_body = R.T @ np.array([0.0, 0.0, -1.0])
        rxn = np.cross(self.r, n_body)                                  # (n,3)
        return 1.0 / (1.0 / mass + np.einsum("ni,ij,nj->n", rxn, I_inv, rxn))

    def forces(self, pos_ned: np.ndarray, vel_ned: np.ndarray, R: np.ndarray, rates: np.ndarray,
               dt: float | None = None, mass: float | None = None, I_inv: np.ndarray | None = None) -> tuple[np.ndarray, np.ndarray, bool, int]:
        """Force (NED), moment about the CG (body), on_ground flag, number of feet in contact.

        With ``dt``, ``mass`` and ``I_inv`` the damping (and the viscous part of the friction) is clamped per foot
        to what an explicit step of ``dt`` can integrate: a damping force may remove at most half of a foot's
        velocity per step. Otherwise a light, low-inertia airframe on stiff-damped legs jitters at the sampling
        rate, and that jitter reaches the simulated gyro and accelerometer (and PX4's estimator)."""
        if self.n == 0:
            return np.zeros(3), np.zeros(3), False, 0
        pen = (self.r @ R[2]) + pos_ned[2] + self.radius                 # foot depth below ground (> 0: contact)
        touching = pen >= 0.0
        if not touching.any():
            return np.zeros(3), np.zeros(3), False, 0
        vb = -np.einsum("nij,j->ni", self.S, rates)                     # w x r, body frame
        V = vel_ned + vb @ R.T                                          # foot velocities, NED
        c = self.c
        c_lim = None
        if dt is not None and mass is not None and I_inv is not None:
            c_lim = 0.5 * self.effective_mass(mass, I_inv, R) / dt
            c = np.minimum(c, c_lim)
        fn = np.maximum(self.k * np.maximum(pen, 0.0) + c * np.maximum(V[:, 2], 0.0), 0.0) * touching
        vh = V[:, :2]
        sp = np.sqrt(vh[:, 0] ** 2 + vh[:, 1] ** 2)
        visc = self.mu * fn / np.maximum(sp, 0.2)                         # friction, viscous below 0.2 m/s
        if c_lim is not None:
            visc = np.minimum(visc, c_lim)
        scale = np.where(touching & (sp > 1e-4), -visc, 0.0)
        f = np.empty((self.n, 3))
        f[:, 0] = scale * vh[:, 0]; f[:, 1] = scale * vh[:, 1]; f[:, 2] = -fn
        F = f.sum(axis=0)
        M = np.einsum("nij,nj->i", self.S, f @ R)                       # r x (R^T f)
        return F, M, True, int(touching.sum())
