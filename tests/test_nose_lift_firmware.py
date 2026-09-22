"""The NL_* export for the nose_lift PX4 module (firmware/px4_ext) must describe the same lift as the simulator's."""
import copy
import json
import math
from pathlib import Path

import numpy as np

from airframe_designer.dynamics.rigid_body import RigidBody
from airframe_designer.geometry.airframe import Airframe
from airframe_designer.sim.nose_lift import NoseLift, firmware_params, uses_firmware
from airframe_designer.sim.scenario import Scenario

ROOT = Path(__file__).resolve().parents[1]


def _atlas(executor: str | None = "firmware") -> Airframe:
    """ATLAS_09B with the nose lift on ``executor`` ("sim", "firmware", or None for no executor key at all)."""
    af = Airframe.from_dict(json.loads((ROOT / "airframes" / "atlas_09b.json").read_text()))
    af.design["nose_lift"].pop("executor", None)
    if executor is not None:
        af.design["nose_lift"]["executor"] = executor
    return af


def _firmware_balance(p: dict, R: np.ndarray, split: list[float]) -> float:
    """NoseLift::balance_fraction() in NoseLift.cpp, from the exported parameters."""
    piv = R @ np.array([p["NL_PIV_X"], p["NL_PIV_Y"], p["NL_PIV_Z"]])
    tau = float(np.cross(-piv, [0.0, 0.0, p["NL_WEIGHT"]]) @ R[:, 1])
    a = sum(p[f"NL_A{k}"] * split[k] for k in range(len(split)))
    return min(2.0, max(0.0, -tau / a))


def _firmware_split(p: dict, rates, n: int) -> list[float]:
    """NoseLift::update_split() in NoseLift.cpp."""
    w0 = [p[f"NL_W{k}"] for k in range(n)]
    mr = [p[f"NL_MR{k}"] for k in range(n)]
    my = [p[f"NL_MY{k}"] for k in range(n)]
    mmax = max(max(abs(x) for x in mr), max(abs(x) for x in my))
    w = [min(1.8, max(0.2, w0[k] - p["NL_K_RATE"] * (mr[k] * rates[0] + my[k] * rates[2]) / mmax)) for k in range(n)]
    return [x / float(np.mean(w)) for x in w]


def test_export_only_with_the_firmware_executor():
    assert not uses_firmware(_atlas(None).design)
    assert "NL_EN" not in _atlas(None).px4_params()
    assert "NL_EN" not in _atlas("sim").px4_params()
    p = _atlas("firmware").px4_params()
    assert p["NL_EN"] == 1
    assert p["NL_MOT_MSK"] == (1 << 8) | (1 << 9)          # motors 9 and 10
    assert p["COM_KILL_DISARM"] == 0.0                       # a tap of the kill button disarms
    assert p["COM_DISARM_PRFLT"] >= 60.0                     # no pre-takeoff auto-disarm during the lift
    assert p["NL_RC_CH"] == 7 and p["NL_RC_LOW"] == 1


def test_airframe_overrides_win():
    af = _atlas("firmware")
    af.px4_overrides["COM_KILL_DISARM"] = 2.0
    assert af.px4_params()["COM_KILL_DISARM"] == 2.0


def test_exported_geometry_matches_the_simulator_lift():
    af = _atlas("firmware")
    p = firmware_params(af)
    body = RigidBody(af)
    nl = NoseLift(af.design["nose_lift"]["motors"], 24.0)
    for pitch in (4.0, 12.0, 24.0):
        th = math.radians(pitch)
        body.q = np.array([math.cos(th / 2), 0.0, math.sin(th / 2), 0.0])
        rates = np.array([0.05, 0.0, -0.03])
        nl._update_split(body, rates)
        split = _firmware_split(p, rates, 2)
        assert np.allclose(split, nl.split, atol=1e-4)
        assert abs(_firmware_balance(p, body.rotmat, split) - nl._balance_fraction(body)) < 1e-4


def test_scenario_design_block_switches_the_executor():
    af = _atlas(None)
    sc = Scenario.from_dict({"design": {"nose_lift": {"executor": "firmware"}}})
    out = sc.apply_attitude(af)
    assert uses_firmware(out.design)
    assert not uses_firmware(af.design)                      # the caller's airframe is untouched
    assert out.design["nose_lift"]["motors"] == af.design["nose_lift"]["motors"]
