"""Ask/tell optimisers over a box [lo, hi]^n, so a study can evaluate a whole batch of candidates in parallel.

  random        uniform samples (with an optional Latin-hypercube layout)
  grid          full factorial (levels per variable)
  cmaes         a compact CMA-ES (Hansen); population evaluated in parallel each generation
  nelder_mead   sequential simplex (one candidate at a time), best for cheap objectives or refinement
"""
from __future__ import annotations

import math

import numpy as np


class Optimizer:
    def __init__(self, lo, hi, budget: int, seed: int = 1, **kw):
        self.lo = np.asarray(lo, float); self.hi = np.asarray(hi, float)
        self.n = len(self.lo)
        self.budget = int(budget)
        self.rng = np.random.default_rng(seed)
        self.evaluated = 0
        self.best_x = None
        self.best_f = math.inf

    def ask(self) -> list[np.ndarray]:
        raise NotImplementedError

    def tell(self, xs: list[np.ndarray], fs: list[float]) -> None:
        for x, f in zip(xs, fs):
            self.evaluated += 1
            if f < self.best_f:
                self.best_f, self.best_x = f, np.asarray(x, float).copy()

    @property
    def done(self) -> bool:
        return self.evaluated >= self.budget

    def clip(self, x):
        return np.clip(np.asarray(x, float), self.lo, self.hi)


class RandomSearch(Optimizer):
    def __init__(self, lo, hi, budget, seed=1, batch: int = 8, include_center: bool = True, x0=None, **kw):
        super().__init__(lo, hi, budget, seed)
        self.batch = max(1, int(batch))
        self.x0 = None if x0 is None else self.clip(x0)
        self._first = True

    def ask(self):
        k = min(self.batch, self.budget - self.evaluated)
        if k <= 0:
            return []
        xs = []
        if self._first and self.x0 is not None:
            xs.append(self.x0.copy()); k -= 1
        self._first = False
        # Latin hypercube for the rest of this batch
        if k > 0:
            cut = np.linspace(0, 1, k + 1)
            u = self.rng.random((k, self.n))
            pts = cut[:k, None] + u * (cut[1:k + 1, None] - cut[:k, None])
            for j in range(self.n):
                self.rng.shuffle(pts[:, j])
            xs += [self.lo + (self.hi - self.lo) * p for p in pts]
        return xs


class GridSearch(Optimizer):
    def __init__(self, lo, hi, budget, seed=1, levels: int | list[int] = 5, batch: int = 8, **kw):
        super().__init__(lo, hi, budget, seed)
        lv = [levels] * self.n if isinstance(levels, int) else list(levels)
        axes = [np.linspace(l, h, max(1, int(k))) for l, h, k in zip(self.lo, self.hi, lv)]
        mesh = np.meshgrid(*axes, indexing="ij")
        self.points = [np.array(p) for p in zip(*[m.ravel() for m in mesh])]
        self.budget = min(self.budget, len(self.points))
        self.batch = max(1, int(batch))
        self.i = 0

    def ask(self):
        xs = self.points[self.i:self.i + self.batch]
        self.i += len(xs)
        return xs


class CMAES(Optimizer):
    """Minimal CMA-ES in normalised coordinates [0, 1]^n (bounds handled by clipping the sampled points)."""

    def __init__(self, lo, hi, budget, seed=1, population: int | None = None, sigma: float = 0.3, x0=None, **kw):
        super().__init__(lo, hi, budget, seed)
        n = self.n
        self.lam = int(population or (4 + int(3 * math.log(max(n, 1)))))
        self.mu = self.lam // 2
        w = np.log(self.mu + 0.5) - np.log(np.arange(1, self.mu + 1))
        self.w = w / w.sum()
        self.mueff = 1.0 / (self.w ** 2).sum()
        self.cc = (4 + self.mueff / n) / (n + 4 + 2 * self.mueff / n)
        self.cs = (self.mueff + 2) / (n + self.mueff + 5)
        self.c1 = 2 / ((n + 1.3) ** 2 + self.mueff)
        self.cmu = min(1 - self.c1, 2 * (self.mueff - 2 + 1 / self.mueff) / ((n + 2) ** 2 + self.mueff))
        self.damps = 1 + 2 * max(0, math.sqrt((self.mueff - 1) / (n + 1)) - 1) + self.cs
        self.chiN = math.sqrt(n) * (1 - 1 / (4 * n) + 1 / (21 * n * n))
        self.m = np.full(n, 0.5) if x0 is None else (self.clip(x0) - self.lo) / np.maximum(self.hi - self.lo, 1e-12)
        self.sigma = float(sigma)
        self.C = np.eye(n); self.pc = np.zeros(n); self.ps = np.zeros(n)
        self.B = np.eye(n); self.D = np.ones(n)
        self.gen = 0
        self._z: list[np.ndarray] = []

    def _decompose(self):
        C = (self.C + self.C.T) / 2
        vals, vecs = np.linalg.eigh(C)
        vals = np.maximum(vals, 1e-20)
        self.B, self.D = vecs, np.sqrt(vals)

    def ask(self):
        self._decompose()
        k = min(self.lam, self.budget - self.evaluated)
        xs, zs = [], []
        for _ in range(max(0, k)):
            z = self.rng.standard_normal(self.n)
            y = self.B @ (self.D * z)
            xn = np.clip(self.m + self.sigma * y, 0.0, 1.0)
            xs.append(self.lo + (self.hi - self.lo) * xn); zs.append(xn)
        self._z = zs
        return xs

    def tell(self, xs, fs):
        super().tell(xs, fs)
        if len(fs) < 2:
            return
        n = self.n
        xn = np.array([(np.asarray(x) - self.lo) / np.maximum(self.hi - self.lo, 1e-12) for x in xs])
        order = np.argsort(fs)
        mu = min(self.mu, len(order))
        w = self.w[:mu] / self.w[:mu].sum()
        m_old = self.m.copy()
        self.m = (w[:, None] * xn[order[:mu]]).sum(axis=0)
        y = (self.m - m_old) / self.sigma
        invsqrtC = self.B @ np.diag(1.0 / self.D) @ self.B.T
        self.ps = (1 - self.cs) * self.ps + math.sqrt(self.cs * (2 - self.cs) * self.mueff) * (invsqrtC @ y)
        self.gen += 1
        hsig = np.linalg.norm(self.ps) / math.sqrt(1 - (1 - self.cs) ** (2 * self.gen)) / self.chiN < 1.4 + 2 / (n + 1)
        self.pc = (1 - self.cc) * self.pc + hsig * math.sqrt(self.cc * (2 - self.cc) * self.mueff) * y
        ys = (xn[order[:mu]] - m_old) / self.sigma
        rank_mu = sum(wi * np.outer(yi, yi) for wi, yi in zip(w, ys))
        self.C = (1 - self.c1 - self.cmu) * self.C + self.c1 * (np.outer(self.pc, self.pc) + (1 - hsig) * self.cc * (2 - self.cc) * self.C) + self.cmu * rank_mu
        self.sigma *= math.exp((self.cs / self.damps) * (np.linalg.norm(self.ps) / self.chiN - 1))
        self.sigma = float(min(self.sigma, 1.0))


class NelderMead(Optimizer):
    """Sequential simplex with bound clipping; one candidate per ask()."""

    def __init__(self, lo, hi, budget, seed=1, x0=None, step: float = 0.25, **kw):
        super().__init__(lo, hi, budget, seed)
        x0 = self.clip(x0) if x0 is not None else (self.lo + self.hi) / 2
        self.pts = [x0.copy()]
        for i in range(self.n):
            p = x0.copy(); p[i] = np.clip(p[i] + step * (self.hi[i] - self.lo[i]), self.lo[i], self.hi[i])
            if abs(p[i] - x0[i]) < 1e-9:
                p[i] = np.clip(x0[i] - step * (self.hi[i] - self.lo[i]), self.lo[i], self.hi[i])
            self.pts.append(p)
        self.vals: list[float] = []
        self._pending = None
        self._stage = "init"
        self._trial: dict = {}

    def ask(self):
        if self.done:
            return []
        if len(self.vals) < len(self.pts):
            self._pending = self.pts[len(self.vals)]
            return [self._pending]
        order = np.argsort(self.vals)
        self.pts = [self.pts[i] for i in order]; self.vals = [self.vals[i] for i in order]
        centroid = np.mean(self.pts[:-1], axis=0)
        worst = self.pts[-1]
        if self._stage in ("init", "reflect"):
            x = self.clip(centroid + (centroid - worst)); self._stage = "reflect_eval"
        elif self._stage == "expand":
            x = self.clip(centroid + 2 * (centroid - worst)); self._stage = "expand_eval"
        elif self._stage == "contract":
            x = self.clip(centroid + 0.5 * (worst - centroid)); self._stage = "contract_eval"
        else:  # shrink: re-evaluate points one by one
            i = self._trial.get("shrink_i", 1)
            self.pts[i] = self.clip(self.pts[0] + 0.5 * (self.pts[i] - self.pts[0]))
            x = self.pts[i]; self._stage = "shrink_eval"
        self._pending = x
        return [x]

    def tell(self, xs, fs):
        super().tell(xs, fs)
        f = fs[0]; x = np.asarray(xs[0], float)
        if len(self.vals) < len(self.pts):
            self.vals.append(f); return
        if self._stage == "reflect_eval":
            if f < self.vals[0]:
                self._trial = {"xr": x, "fr": f}; self._stage = "expand"
            elif f < self.vals[-2]:
                self.pts[-1], self.vals[-1] = x, f; self._stage = "reflect"
            else:
                self._trial = {"xr": x, "fr": f}; self._stage = "contract"
        elif self._stage == "expand_eval":
            if f < self._trial["fr"]:
                self.pts[-1], self.vals[-1] = x, f
            else:
                self.pts[-1], self.vals[-1] = self._trial["xr"], self._trial["fr"]
            self._stage = "reflect"
        elif self._stage == "contract_eval":
            if f < self.vals[-1]:
                self.pts[-1], self.vals[-1] = x, f; self._stage = "reflect"
            else:
                self._trial = {"shrink_i": 1}; self._stage = "shrink"
        elif self._stage == "shrink_eval":
            i = self._trial["shrink_i"]; self.vals[i] = f
            if i + 1 < len(self.pts):
                self._trial["shrink_i"] = i + 1; self._stage = "shrink"
            else:
                self._stage = "reflect"


def make_optimizer(name: str, lo, hi, budget: int, seed: int = 1, **kw) -> Optimizer:
    name = (name or "random").lower().replace("-", "_")
    cls = {"random": RandomSearch, "lhs": RandomSearch, "grid": GridSearch, "cmaes": CMAES, "cma_es": CMAES, "cma": CMAES,
           "nelder_mead": NelderMead, "simplex": NelderMead}.get(name)
    if cls is None:
        raise ValueError(f"unknown optimiser '{name}' (random, grid, cmaes, nelder_mead)")
    return cls(lo, hi, budget, seed=seed, **kw)
