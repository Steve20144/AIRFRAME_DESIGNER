"""Objectives and constraints as small expressions over the result dict, e.g.

    "phases.hold.pos_std_xy + 0.05 * phases.hold.roll_rms_deg"
    "metrics.energy_wh"      (the leading "metrics." is optional)
    constraints: ["not metrics.crashed", "phases.takeoff.time_to_alt < 12", "ok"]

Only arithmetic, comparisons, boolean logic, attribute/index access and a few math functions are allowed.
A missing value evaluates to None, which fails the expression (a penalty is applied by score_result)."""
from __future__ import annotations

import ast
import math
from typing import Any

_FUNCS = {"abs": abs, "min": min, "max": max, "sqrt": math.sqrt, "exp": math.exp, "log": math.log, "log10": math.log10,
          "pow": pow, "round": round, "float": float, "int": int, "clip": lambda v, lo, hi: max(lo, min(hi, v)),
          "hypot": math.hypot, "tanh": math.tanh}


class _Missing(Exception):
    pass


def _lookup(scope: Any, name: str):
    if isinstance(scope, dict):
        if name in scope:
            return scope[name]
        raise _Missing(name)
    if hasattr(scope, name):
        return getattr(scope, name)
    raise _Missing(name)


class _Eval(ast.NodeVisitor):
    def __init__(self, root: dict):
        self.root = root

    def visit(self, node):
        m = getattr(self, "visit_" + type(node).__name__, None)
        if m is None:
            raise ValueError(f"expression element not allowed: {type(node).__name__}")
        return m(node)

    def visit_Expression(self, n): return self.visit(n.body)
    def visit_Constant(self, n): return n.value

    def visit_Name(self, n):
        if n.id in ("True", "False", "None"):
            return {"True": True, "False": False, "None": None}[n.id]
        if n.id in _FUNCS:
            return _FUNCS[n.id]
        try:
            return _lookup(self.root, n.id)
        except _Missing:
            m = self.root.get("metrics") if isinstance(self.root, dict) else None
            if isinstance(m, dict):
                return _lookup(m, n.id)
            raise

    def visit_Attribute(self, n):
        return _lookup(self.visit(n.value), n.attr)

    def visit_Subscript(self, n):
        v = self.visit(n.value); k = self.visit(n.slice)
        try:
            return v[k]
        except (KeyError, IndexError, TypeError):
            raise _Missing(str(k))

    def visit_UnaryOp(self, n):
        v = self.visit(n.operand)
        if isinstance(n.op, ast.USub): return -v
        if isinstance(n.op, ast.UAdd): return +v
        if isinstance(n.op, ast.Not): return not v
        raise ValueError("operator not allowed")

    def visit_BinOp(self, n):
        a, b = self.visit(n.left), self.visit(n.right)
        if a is None or b is None:
            raise _Missing("None in arithmetic")
        ops = {ast.Add: lambda: a + b, ast.Sub: lambda: a - b, ast.Mult: lambda: a * b, ast.Div: lambda: a / b,
               ast.Pow: lambda: a ** b, ast.Mod: lambda: a % b, ast.FloorDiv: lambda: a // b}
        if type(n.op) not in ops:
            raise ValueError("operator not allowed")
        return ops[type(n.op)]()

    def visit_BoolOp(self, n):
        vals = [self.visit(v) for v in n.values]
        return all(vals) if isinstance(n.op, ast.And) else any(vals)

    def visit_Compare(self, n):
        left = self.visit(n.left)
        for op, comp in zip(n.ops, n.comparators):
            right = self.visit(comp)
            if left is None or right is None:
                raise _Missing("None in comparison")
            ok = {ast.Lt: left < right, ast.LtE: left <= right, ast.Gt: left > right, ast.GtE: left >= right,
                  ast.Eq: left == right, ast.NotEq: left != right}.get(type(op))
            if ok is None:
                raise ValueError("comparison not allowed")
            if not ok:
                return False
            left = right
        return True

    def visit_IfExp(self, n):
        return self.visit(n.body) if self.visit(n.test) else self.visit(n.orelse)

    def visit_Call(self, n):
        if isinstance(n.func, ast.Name) and n.func.id not in _FUNCS:
            raise ValueError(f"function not allowed: {n.func.id}")
        f = self.visit(n.func)
        if f not in _FUNCS.values():
            raise ValueError("function not allowed")
        return f(*[self.visit(a) for a in n.args])


def evaluate_expression(expr: str, result: dict):
    """Evaluate ``expr`` against a run result. Returns None when a referenced value is missing."""
    tree = ast.parse(expr, mode="eval")
    try:
        return _Eval(result).visit(tree)
    except _Missing:
        return None


def score_result(result: dict, objective: str, constraints: list[str] | None = None, penalty: float = 1000.0,
                 maximize: bool = False) -> dict:
    """Objective value (lower is better after sign handling) plus which constraints failed."""
    failed = []
    if not result.get("ok", False) and result.get("status") == "error":
        failed.append("run error")
    for c in constraints or []:
        v = evaluate_expression(c, result)
        if not v:
            failed.append(c)
    val = evaluate_expression(objective, result)
    if val is None or (isinstance(val, float) and (math.isnan(val) or math.isinf(val))):
        failed.append(f"objective '{objective}' undefined")
        val = None
    raw = val
    if val is None:
        score = penalty * (1 + len(failed))
    else:
        score = (-float(val) if maximize else float(val)) + penalty * len(failed)
    return {"score": float(score), "objective": raw, "violations": failed, "feasible": not failed}
