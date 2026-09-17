"""Objective expressions, scoring and the ask/tell optimisers (no simulation)."""
from __future__ import annotations

import math

import numpy as np
import pytest

from airframe_designer.batch.objective import evaluate_expression, score_result
from airframe_designer.batch.optimizers import CMAES, GridSearch, NelderMead, RandomSearch, make_optimizer

RESULT = {"ok": True, "status": "done", "failures": [],
          "metrics": {"crashed": False, "energy_wh": 1.5, "max_tilt_deg": 12.0,
                      "phases": {"hover": {"pos_std_xy": 0.25, "roll_rms_deg": 2.0, "time_to_alt": 6.5},
                                 "takeoff": {"time_to_alt": 6.5}}},
          "scenarios": {"hover": {"metrics": {"energy_wh": 1.5}}}}


# --------------------------------------------------------------- objective
def test_evaluate_expression_basic():
    assert evaluate_expression("phases.hover.pos_std_xy + 2", RESULT) == pytest.approx(2.25)
    assert evaluate_expression("metrics.phases.hover.pos_std_xy + 2", RESULT) == pytest.approx(2.25)
    assert evaluate_expression("not metrics.crashed", RESULT) is True
    assert evaluate_expression("not crashed", RESULT) is True
    assert evaluate_expression("ok", RESULT) is True
    assert evaluate_expression("phases.takeoff.time_to_alt < 12", RESULT) is True
    assert evaluate_expression("0 < phases.takeoff.time_to_alt < 5", RESULT) is False
    assert evaluate_expression("phases['hover']['pos_std_xy'] * 4", RESULT) == pytest.approx(1.0)
    assert evaluate_expression("scenarios.hover.metrics.energy_wh", RESULT) == 1.5
    assert evaluate_expression("max(energy_wh, 2) + sqrt(4) - abs(-1) + clip(10, 0, 3)", RESULT) == pytest.approx(6.0)
    assert evaluate_expression("energy_wh if crashed else -energy_wh", RESULT) == -1.5
    assert evaluate_expression("ok and not crashed or False", RESULT) is True
    assert evaluate_expression("2 ** 3 % 5 // 2", RESULT) == 1
    assert evaluate_expression("None", RESULT) is None


def test_evaluate_expression_missing_values_give_none():
    assert evaluate_expression("phases.landing.time", RESULT) is None
    assert evaluate_expression("phases.hover.nope + 1", RESULT) is None
    assert evaluate_expression("phases.hover.nope < 1", RESULT) is None
    assert evaluate_expression("phases['x']", RESULT) is None
    assert evaluate_expression("energy_wh", {}) is None
    assert evaluate_expression("touchdown_speed + 1", {"metrics": {"touchdown_speed": None}}) is None


@pytest.mark.parametrize("expr", ["lambda: 1", "[1, 2]", "{1: 2}", "(1, 2)", "abs.__call__(1)", "[1] if not crashed else 1",
                                  "energy_wh @ energy_wh", "f'{1}'", "energy_wh << 1", "sqrt(*phases)"])
def test_evaluate_expression_rejects_disallowed_constructs(expr):
    with pytest.raises(ValueError):
        evaluate_expression(expr, RESULT)


def test_evaluate_expression_never_executes_unknown_callables():
    import sys
    sys.modules.pop("this", None)
    with pytest.raises(ValueError):
        evaluate_expression("__import__('this')", RESULT)
    assert "this" not in sys.modules
    with pytest.raises(ValueError):
        evaluate_expression("open('/etc/hosts')", RESULT)
    assert evaluate_expression("metrics.__class__", RESULT) is None


def test_unknown_function_call_raises_value_error():
    with pytest.raises(ValueError):
        evaluate_expression("__import__('os')", RESULT)


def test_score_result_feasible():
    s = score_result(RESULT, "phases.hover.pos_std_xy", ["not metrics.crashed", "ok", "phases.takeoff.time_to_alt < 12"])
    assert s == {"score": 0.25, "objective": 0.25, "violations": [], "feasible": True}


def test_score_result_penalises_violations():
    s = score_result(RESULT, "phases.hover.pos_std_xy", ["phases.takeoff.time_to_alt < 5", "metrics.crashed"], penalty=100.0)
    assert s["feasible"] is False
    assert s["violations"] == ["phases.takeoff.time_to_alt < 5", "metrics.crashed"]
    assert s["score"] == pytest.approx(0.25 + 200.0)
    assert s["objective"] == 0.25


def test_score_result_maximize_and_undefined():
    s = score_result(RESULT, "phases.hover.pos_std_xy", maximize=True)
    assert s["score"] == pytest.approx(-0.25) and s["feasible"]
    s = score_result(RESULT, "phases.hover.missing", ["ok"])
    assert s["objective"] is None and not s["feasible"]
    assert s["violations"] == ["objective 'phases.hover.missing' undefined"]
    assert s["score"] == 1000.0 * 2
    s = score_result({"ok": False, "status": "error", "metrics": {}}, "1.0", [])
    assert s["violations"] == ["run error"] and s["score"] == pytest.approx(1001.0)
    s = score_result({"ok": True, "status": "done", "metrics": {"x": float("nan")}}, "x", [])
    assert s["objective"] is None and "undefined" in s["violations"][0]


# -------------------------------------------------------------- optimisers
def _drive(opt, f):
    while not opt.done:
        xs = opt.ask()
        if not xs:
            break
        opt.tell(xs, [f(x) for x in xs])
    return opt


def quad2(x):
    return (x[0] - 1.0) ** 2 + (x[1] + 2.0) ** 2


def test_random_search_in_bounds_and_budget():
    lo, hi = [-1.0, 0.0, 10.0], [1.0, 5.0, 20.0]
    opt = RandomSearch(lo, hi, budget=21, seed=3, batch=8, x0=[0.0, 2.5, 99.0])
    seen = []
    while not opt.done:
        xs = opt.ask()
        assert 1 <= len(xs) <= 8
        seen += xs
        opt.tell(xs, [float(np.sum(np.square(x))) for x in xs])
    assert len(seen) == 21 and opt.evaluated == 21
    assert opt.ask() == []
    A = np.array(seen)
    assert np.all(A >= np.array(lo)) and np.all(A <= np.array(hi))
    assert A[0].tolist() == [0.0, 2.5, 20.0]                # x0 first, clipped into the box
    assert len({tuple(x) for x in seen}) == 21
    assert opt.best_f == min(float(np.sum(np.square(x))) for x in seen)
    # deterministic in the seed
    again = RandomSearch(lo, hi, budget=21, seed=3, batch=8, x0=[0.0, 2.5, 99.0])
    assert np.allclose(again.ask(), seen[:8])


def test_random_search_latin_hypercube_layout():
    opt = RandomSearch([0.0], [1.0], budget=10, seed=1, batch=10, include_center=False)
    xs = np.array(opt.ask()).ravel()
    assert sorted(int(x * 10) for x in xs) == list(range(10))    # one sample per decile


def test_grid_search_enumerates_levels():
    opt = GridSearch([0.0, 10.0], [1.0, 20.0], budget=1000, levels=[3, 2], batch=4)
    pts = []
    while not opt.done:
        xs = opt.ask(); pts += xs
        opt.tell(xs, [0.0] * len(xs))
    assert opt.budget == 6 and len(pts) == 6
    assert sorted(map(tuple, pts)) == sorted([(a, b) for a in (0.0, 0.5, 1.0) for b in (10.0, 20.0)])
    opt = GridSearch([0.0, 0.0], [1.0, 1.0], budget=4, levels=5)
    assert len(opt.points) == 25 and opt.budget == 4
    assert len(_drive(opt, quad2).ask()) == 0 or opt.done


def test_cmaes_converges_on_2d_quadratic():
    opt = CMAES([-5.0, -5.0], [5.0, 5.0], budget=200, seed=1, population=8)
    assert opt.lam == 8 and opt.mu == 4
    _drive(opt, quad2)
    assert opt.evaluated == 200
    assert np.linalg.norm(opt.best_x - np.array([1.0, -2.0])) < 0.1
    assert opt.best_f < 0.01
    assert opt.sigma < 0.3


def test_cmaes_respects_bounds_and_x0():
    lo, hi = [0.0, 0.0], [1.0, 1.0]
    opt = CMAES(lo, hi, budget=40, seed=2, population=8, sigma=0.5, x0=[0.9, 0.1])
    assert np.allclose(opt.m, [0.9, 0.1])
    pts = []
    while not opt.done:
        xs = opt.ask(); pts += xs
        opt.tell(xs, [quad2(x) for x in xs])
    A = np.array(pts)
    assert np.all(A >= 0.0) and np.all(A <= 1.0) and len(A) == 40
    assert np.linalg.norm(opt.best_x - np.array([1.0, 0.0])) < 0.15   # the optimum is the nearest corner


@pytest.mark.parametrize("seed", [1, 2, 3])
def test_nelder_mead_1d(seed):
    opt = NelderMead([-5.0], [5.0], budget=60, seed=seed)
    _drive(opt, lambda x: (x[0] - 1.5) ** 2 + 0.5)
    assert opt.evaluated <= 60 and opt.done
    assert opt.best_x[0] == pytest.approx(1.5, abs=1e-2)
    assert opt.best_f == pytest.approx(0.5, abs=1e-3)


def test_nelder_mead_2d():
    opt = NelderMead([-5.0, -5.0], [5.0, 5.0], budget=60, x0=[3.0, 3.0])
    assert len(opt.pts) == 3 and np.allclose(opt.pts[0], [3.0, 3.0])
    seen = []
    while not opt.done:
        xs = opt.ask()
        assert len(xs) == 1
        seen += xs
        opt.tell(xs, [quad2(x) for x in xs])
    assert len(seen) == 60
    assert np.linalg.norm(opt.best_x - np.array([1.0, -2.0])) < 0.02
    assert np.all(np.array(seen) >= -5.0) and np.all(np.array(seen) <= 5.0)


def test_nelder_mead_shrinks_when_stuck_at_bound():
    opt = NelderMead([0.0], [1.0], budget=30, x0=[1.0])
    _drive(opt, lambda x: (x[0] - 2.0) ** 2)             # optimum outside the box: converges to the bound
    assert opt.best_x[0] == pytest.approx(1.0, abs=1e-3)


def test_make_optimizer_names_and_unknown():
    assert isinstance(make_optimizer("random", [0], [1], 5), RandomSearch)
    assert isinstance(make_optimizer("LHS", [0], [1], 5), RandomSearch)
    assert isinstance(make_optimizer("grid", [0], [1], 5, levels=3), GridSearch)
    assert isinstance(make_optimizer("cma-es", [0], [1], 5), CMAES)
    assert isinstance(make_optimizer("cmaes", [0], [1], 5, population=6), CMAES)
    assert isinstance(make_optimizer("nelder_mead", [0], [1], 5), NelderMead)
    assert isinstance(make_optimizer("simplex", [0], [1], 5), NelderMead)
    assert isinstance(make_optimizer(None, [0], [1], 5), RandomSearch)
    with pytest.raises(ValueError, match="unknown optimiser"):
        make_optimizer("bayes", [0], [1], 5)
    # unknown keyword arguments are tolerated (study specs carry the algorithm dict through)
    make_optimizer("random", [0], [1], 5, sigma=0.1, population=4)
