"""Small-vector helpers that avoid numpy's per-call overhead on 3-vectors (np.cross costs ~10 us each)."""
from __future__ import annotations

import numpy as np


def cross3(a, b) -> np.ndarray:
    a0, a1, a2 = float(a[0]), float(a[1]), float(a[2])
    b0, b1, b2 = float(b[0]), float(b[1]), float(b[2])
    return np.array([a1 * b2 - a2 * b1, a2 * b0 - a0 * b2, a0 * b1 - a1 * b0])


def skew(v) -> np.ndarray:
    """[v]x such that [v]x @ w == v x w."""
    x, y, z = float(v[0]), float(v[1]), float(v[2])
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def skew_stack(V: np.ndarray) -> np.ndarray:
    """(n,3) -> (n,3,3) stack of skew matrices, so that einsum('nij,j->ni', S, w) == rows x w."""
    V = np.asarray(V, float).reshape(-1, 3)
    n = len(V)
    S = np.zeros((n, 3, 3))
    S[:, 0, 1] = -V[:, 2]; S[:, 0, 2] = V[:, 1]
    S[:, 1, 0] = V[:, 2]; S[:, 1, 2] = -V[:, 0]
    S[:, 2, 0] = -V[:, 1]; S[:, 2, 1] = V[:, 0]
    return S
