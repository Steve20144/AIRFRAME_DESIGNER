"""JSBSim backend: same airframe, same interface, same rest state and hover response as the Python body."""
import math

import numpy as np
import pytest

from airframe_designer.geometry.airframe import Airframe, quad_x
from airframe_designer.dynamics import RigidBody
from airframe_designer.dynamics.quaternion import q_to_rotmat

jsbsim = pytest.importorskip("jsbsim")
from airframe_designer.dynamics.jsbsim_backend import JSBSimBody, generate_model, FT, LBF, LBFT  # noqa: E402


def test_generated_model_loads_and_rests_like_python(tmp_path):
    af = quad_x()
    py = RigidBody(af); py.reset()
    jb = JSBSimBody(af); jb.reset()
    for _ in range(1500):
        py.step(0.001, False); jb.step(0.001, False)
    assert jb.on_ground and jb.feet_down >= 1
    assert abs(py.pos[2] - jb.pos[2]) < 0.01
    assert abs(math.degrees(py.euler[1]) - math.degrees(jb.euler[1])) < 0.2
    assert np.allclose(jb.accel_body, [0, 0, -9.80665], atol=0.05)


def test_hover_thrust_climb_matches_python():
    af = quad_x()
    hc = af.hover_check(); ht = np.array(hc["hover_thrust"]); tmax = np.array([r.effective_max_thrust() for r in af.active_rotors()])
    cmd = np.sqrt(np.clip(ht / tmax, 0, 1)) * 1.03
    py = RigidBody(af); py.reset(); jb = JSBSimBody(af); jb.reset()
    for _ in range(500):
        py.step(0.001, False); jb.step(0.001, False)
    py.set_motor_commands(cmd); jb.set_motor_commands(cmd)
    for k in range(2000):
        py.step(0.001, k % 4 == 3); jb.step(0.001, k % 4 == 3)
    assert not py.on_ground and not jb.on_ground
    assert abs(py.pos[2] - jb.pos[2]) < 0.05                 # same climb within 5 cm after 2 s
    assert abs(py.thrust.sum() - jb.thrust.sum()) < 1e-6      # identical rotor model
    assert np.abs(np.degrees(jb.euler)).max() < 0.5


def test_soft_legs_settle_to_spring_equilibrium():
    """The Python body used to hover above its spring equilibrium on soft legs (a damping hack); JSBSim exposed it."""
    af = Airframe.load("airframes/atlas_pivot16.json")
    py = RigidBody(af); py.reset(); jb = JSBSimBody(af); jb.reset()
    for _ in range(4000):
        py.step(0.001, False); jb.step(0.001, False)
    assert abs(py.pos[2] - jb.pos[2]) < 0.02
    assert abs(math.degrees(py.euler[1]) - math.degrees(jb.euler[1])) < 0.3


# ------------------------------------------------ static equivalence: same state in, same force and moment out
def _place(jb, euler_deg=(0.0, 0.0, 0.0), uvw=(0.0, 0.0, 0.0), vned=None, pqr_deg=(0.0, 0.0, 0.0), alt_ft=300.0):
    """Hold the JSBSim body at a known attitude, velocity and body rate, well clear of the ground."""
    f = jb.fdm
    f["ic/h-agl-ft"] = alt_ft
    f["ic/phi-deg"], f["ic/theta-deg"] = float(euler_deg[0]), float(euler_deg[1])
    f["ic/psi-true-deg"] = float(euler_deg[2]) % 360.0
    f["ic/p-rad_sec"], f["ic/q-rad_sec"], f["ic/r-rad_sec"] = np.radians(pqr_deg)
    if vned is None:
        f["ic/u-fps"], f["ic/v-fps"], f["ic/w-fps"] = (float(v) * FT for v in uvw)
    else:
        f["ic/vn-fps"], f["ic/ve-fps"], f["ic/vd-fps"] = (float(v) * FT for v in vned)
    f.run_ic()
    jb._read_state()


def _jsbsim_force_moment(jb):
    """What JSBSim applied to the body this step, less the wing coefficient tables. Those are sampled from the
    strip-theory model at one reference speed and scaled by qbar, so they are not expected to match it term for
    term. What is left is the part that must: rotor thrust at its moment arms, and everything this project
    injects (ram drag, reaction torque, body drag)."""
    f = jb.fdm
    F = np.array([f["forces/fbx-total-lbs"], f["forces/fby-total-lbs"], f["forces/fbz-total-lbs"]]) / LBF
    M = np.array([f["moments/l-total-lbsft"], f["moments/m-total-lbsft"], f["moments/n-total-lbsft"]]) / LBFT
    F -= np.array([f["aero/force/x-wing"], f["aero/force/y-beta"], f["aero/force/z-wing"]]) / LBF
    M -= np.array([f["aero/moment/roll-beta"] + f["aero/moment/roll-damp"],
                   f["aero/moment/pitch-wing"] + f["aero/moment/pitch-damp"],
                   f["aero/moment/yaw-beta"] + f["aero/moment/yaw-damp"]]) / LBFT
    return F, M


STATIC_STATES = [
    dict(),
    dict(euler_deg=(4.0, -7.0, 30.0)),
    dict(euler_deg=(-12.0, 20.0, 200.0), uvw=(3.0, -1.5, 0.8), pqr_deg=(10.0, -8.0, 6.0)),
    dict(euler_deg=(0.0, 15.0, 95.0), uvw=(-1.0, 2.5, -0.6), pqr_deg=(-20.0, 5.0, -14.0)),
]


@pytest.mark.parametrize("path", ["tests/data/atlas_08.json",          # CG at the origin, wings, fans canted +-30
                                  "tests/data/atlas_phase01_legs.json"])  # CG 17 cm off the origin, products of inertia
def test_static_force_and_moment_match_python(path):
    """No flying, no PX4: one airframe, one known state, one known set of rotor commands, and the total body
    force and moment JSBSim ends up with must be the ones RigidBody computes. This is what pins the frame and
    sign conventions -- the thrust directions written raw in FRD, the locations converted to JSBSim's structural
    frame, and the moments injected through the ROLL/PITCH/YAW axes."""
    af = Airframe.load(path)
    jb = JSBSimBody(af); py = RigidBody(af)
    cmd = 0.35 + 0.45 * np.random.default_rng(7).random(jb.rotors.n)   # deliberately asymmetric
    for st in STATIC_STATES:
        _place(jb, **st)
        q0, vel0, rates0 = jb.q.copy(), jb.vel.copy(), jb.rates.copy()
        jb.set_motor_commands(cmd); jb.omega = cmd.copy()              # already spooled up: step() holds it
        jb.step(1e-6, detail=False)                                    # a step short enough not to move anything
        assert jb.fdm["forces/fbz-gear-lbs"] == 0.0                    # airborne: nothing from the feet
        F_js, M_js = _jsbsim_force_moment(jb)
        v_air = q_to_rotmat(q0).T @ vel0
        F_r, M_r, _, _ = py.rotors.forces(cmd, v_air, rates0, False)
        F_b, M_b, _ = py.body.forces(v_air, rates0, False)
        assert np.allclose(F_js, F_r + F_b, atol=2e-3), f"{st}: force {F_js} vs {F_r + F_b}"
        assert np.allclose(M_js, M_r + M_b, atol=2e-3), f"{st}: moment {M_js} vs {M_r + M_b}"


def test_horizontal_position_is_a_signed_displacement():
    """pos must be the displacement from the reset point with its sign, because the GPS handed to PX4 is built
    from it. Reading JSBSim's unsigned distance-from-start instead reported every metre flown west as a metre
    east, so PX4's correction to the west grew the reported easting and asked for more: a runaway, not a hover."""
    af = Airframe.load("tests/data/atlas_08.json")
    jb = JSBSimBody(af)
    for vned in ((4.0, 3.0), (-4.0, -3.0), (0.0, -5.0), (-5.0, 0.0)):
        _place(jb, alt_ft=500.0, vned=(vned[0], vned[1], 0.0))
        p0 = jb.pos.copy()
        travelled = np.zeros(2)
        for _ in range(200):
            v = jb.vel[:2].copy()
            jb.step(0.01, detail=False)
            travelled += 0.5 * (v + jb.vel[:2]) * 0.01                 # trapezoid over the reported velocity
        moved = jb.pos[:2] - p0[:2]
        assert np.allclose(moved, travelled, atol=0.05), f"{vned}: pos moved {moved}, velocity says {travelled}"
        # the blunt version of the same thing: flying south has to be reported as flying south
        for axis, v in enumerate(vned):
            if v:
                name = "north" if axis == 0 else "east"
                assert np.sign(moved[axis]) == np.sign(v), f"{vned}: {name} came back as {moved[axis]:+.2f} m"
                assert abs(moved[axis]) > 1.5 * abs(v), f"{vned}: only {moved[axis]:+.2f} m of {name} in 2 s"
