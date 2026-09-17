"""6-DOF rigid body with leg contact, and the quaternion helpers."""
from __future__ import annotations

import math

import numpy as np
import pytest

from airframe_designer.dynamics import G, LegContacts, RigidBody, q_deriv, q_from_euler, q_normalize, q_to_euler, q_to_rotmat
from airframe_designer.geometry.airframe import quad_x

DT = 0.002


def _run(rb: RigidBody, seconds: float, dt: float = DT) -> None:
    for _ in range(int(round(seconds / dt))):
        rb.step(dt)


# ------------------------------------------------------------ quaternions
@pytest.mark.parametrize("euler", [(0.0, 0.0, 0.0), (0.3, -0.2, 1.0), (-1.2, 0.4, -2.5), (0.0, 1.0, 3.0)])
def test_euler_quaternion_round_trip(euler):
    q = q_from_euler(*euler)
    assert np.linalg.norm(q) == pytest.approx(1.0)
    assert q_to_euler(q) == pytest.approx(euler, abs=1e-9)
    R = q_to_rotmat(q)
    assert np.allclose(R @ R.T, np.eye(3)) and np.linalg.det(R) == pytest.approx(1.0)


def test_rotmat_convention_body_to_ned():
    q = q_from_euler(0.0, 0.0, math.pi / 2)            # yaw 90: body x points east
    assert np.allclose(q_to_rotmat(q) @ np.array([1.0, 0.0, 0.0]), [0.0, 1.0, 0.0], atol=1e-12)
    q = q_from_euler(0.0, math.pi / 4, 0.0)            # nose up 45: body x points up (negative down)
    assert (q_to_rotmat(q) @ np.array([1.0, 0.0, 0.0]))[2] == pytest.approx(-math.sin(math.pi / 4))


def test_q_deriv_and_normalize():
    q = q_from_euler(0.0, 0.0, 0.0)
    dq = q_deriv(q, np.array([0.0, 0.0, 1.0]))
    q2 = q_normalize(q + dq * 0.01)
    assert q_to_euler(q2)[2] == pytest.approx(0.01, abs=1e-6)
    assert np.linalg.norm(q_normalize(np.array([2.0, 0.0, 0.0, 0.0]))) == pytest.approx(1.0)


# ----------------------------------------------------------- rigid body
def test_rest_on_legs(quad):
    rb = RigidBody(quad)
    rest = rb.legs.rest_height(rb.rotmat)
    assert rest == pytest.approx(0.12)
    assert rb.pos[2] == pytest.approx(-rest)
    assert rb.on_ground is True
    _run(rb, 0.5)
    assert rb.feet_down == 4 and rb.on_ground is True
    assert rb.pos[2] == pytest.approx(-rest, abs=0.01)
    assert rb.pos[2] > -rest                              # the springs compress a little under the weight
    assert np.linalg.norm(rb.vel) < 1e-3
    assert np.linalg.norm(rb.rates) < 1e-3
    assert abs(rb.pos[0]) < 1e-6 and abs(rb.pos[1]) < 1e-6
    assert rb.tilt_deg == pytest.approx(0.0, abs=1e-6)


def test_accel_body_is_minus_g_at_rest(quad):
    rb = RigidBody(quad)
    assert rb.accel_body == pytest.approx([0.0, 0.0, -G])
    _run(rb, 0.5)
    assert rb.accel_body == pytest.approx([0.0, 0.0, -G], abs=0.05)


def test_hover_command_holds_position(quad):
    rb = RigidBody(quad)
    rb.reset(pos_ned=[0.0, 0.0, -2.0])
    assert rb.on_ground is True and rb.t == 0.0            # reset() leaves the flag for the first step to clear
    w = math.sqrt(rb.mass * G / (4 * 8.0))                 # thrust = tmax * omega^2 per rotor
    rb.set_motor_commands([w] * 4)
    rb.omega[:] = w
    _run(rb, 2.0)
    assert rb.on_ground is False and rb.feet_down == 0
    assert np.linalg.norm(rb.pos - np.array([0.0, 0.0, -2.0])) < 0.03
    assert np.linalg.norm(rb.vel) < 0.05
    assert rb.tilt_deg < 0.5
    assert rb.breakdown["thrust"] == pytest.approx(rb.mass * G, rel=1e-6)
    assert rb.breakdown["power"] > 0
    assert rb.t == pytest.approx(2.0)


def test_free_fall_from_height():
    af = quad_x()
    af.body.drag_quadratic = [0.0, 0.0, 0.0]
    rb = RigidBody(af)
    rb.reset(pos_ned=[0.0, 0.0, -10.0])
    _run(rb, 0.5)
    assert rb.vel[2] == pytest.approx(G * 0.5, rel=1e-6)
    assert rb.pos[2] == pytest.approx(-10.0 + 0.5 * G * 0.25, rel=0.01)    # semi-implicit Euler: one dt of bias
    assert rb.on_ground is False
    assert rb.accel_body == pytest.approx([0.0, 0.0, 0.0], abs=1e-9)         # free fall: the accelerometer reads zero
    # with the preset body drag the fall is noticeably slower
    rb2 = RigidBody(quad_x())
    rb2.reset(pos_ned=[0.0, 0.0, -10.0])
    _run(rb2, 0.5)
    assert 0.8 * G * 0.5 < rb2.vel[2] < G * 0.5


def test_tilt_deg(quad):
    rb = RigidBody(quad)
    assert rb.tilt_deg == pytest.approx(0.0)
    rb.q = q_from_euler(math.pi / 2, 0.0, 0.0)
    assert rb.tilt_deg == pytest.approx(90.0)
    rb.q = q_from_euler(0.0, -math.pi / 6, 0.0)
    assert rb.tilt_deg == pytest.approx(30.0)
    rb.q = q_from_euler(0.0, 0.0, 1.0)                     # yaw alone is not a tilt
    assert rb.tilt_deg == pytest.approx(0.0, abs=1e-9)


def test_landed_pitch_sets_initial_attitude_and_rest_height():
    from airframe_designer.geometry import generate_legs
    af = quad_x()
    af.landed_pitch_deg = -20.0
    af.legs = generate_legs(0.12, 0.10, 0.10, landed_pitch_deg=-20.0)   # feet coplanar in the landed attitude
    rb = RigidBody(af)
    assert math.degrees(rb.euler[1]) == pytest.approx(-20.0)
    assert rb.tilt_deg == pytest.approx(20.0)
    assert rb.pos[2] == pytest.approx(-rb.legs.rest_height(rb.rotmat))
    _run(rb, 0.5)
    assert rb.on_ground and rb.feet_down == 4
    assert rb.tilt_deg == pytest.approx(20.0, abs=0.5)       # stands nose-down on its legs
    assert np.linalg.norm(rb.vel) < 0.05


def test_hover_frame_euler():
    af = quad_x()
    af.hover_pitch_deg = 25.0
    rb = RigidBody(af)
    rb.q = q_from_euler(0.0, math.radians(25.0), 0.0)      # structural frame pitched to the hover attitude
    r, p, y = rb.hover_frame_euler()
    assert abs(r) < 1e-9 and p == pytest.approx(0.0, abs=1e-9) and abs(y) < 1e-9
    rb.q = q_from_euler(0.0, 0.0, 0.0)
    assert math.degrees(rb.hover_frame_euler()[1]) == pytest.approx(-25.0)


def test_set_rotor_health_scales_thrust(quad):
    rb = RigidBody(quad)
    rb.set_rotor_health([0.5, 1.0, 1.0, 0.0])
    assert rb.rotors.scale.tolist() == [0.5, 1.0, 1.0, 0.0]
    assert rb.rotors.thrust(np.ones(4)).tolist() == pytest.approx([4.0, 8.0, 8.0, 0.0])
    rb.set_rotor_health([0.25])                            # short lists leave the rest at 1
    assert rb.rotors.scale.tolist() == [0.25, 1.0, 1.0, 1.0]


def test_motor_commands_clipped_and_spool(quad):
    rb = RigidBody(quad)
    rb.set_motor_commands([2.0, -1.0, 0.5])
    assert rb.cmd.tolist() == [1.0, 0.0, 0.5, 0.0]
    rb.reset(pos_ned=[0, 0, -5.0])
    rb.set_motor_commands([1.0] * 4)
    rb.step(DT)
    assert 0 < rb.omega[0] < 1.0                           # first-order spool-up
    _run(rb, 0.5)
    assert rb.omega[0] == pytest.approx(1.0, abs=1e-4)


def test_set_airframe_keeps_state_size(quad):
    rb = RigidBody(quad)
    rb.set_motor_commands([0.3] * 4)
    hexa = quad_x(); hexa.rotors.append(hexa.rotors[0].__class__(name="M5", pos=[0, 0.3, 0]))
    rb.set_airframe(hexa)
    assert len(rb.cmd) == 5 and rb.rotors.n == 5


def test_leg_contacts_rest_height_and_forces():
    from airframe_designer.geometry import Leg
    legs = [Leg(attach=[0.1, 0.1, 0], length=0.2), Leg(attach=[-0.1, -0.1, 0], length=0.2, foot_radius=0.05)]
    lc = LegContacts(legs, [0.0, 0.0, 0.0])
    R = np.eye(3)
    assert lc.rest_height(R) == pytest.approx(0.25)        # the foot ball raises the rest height
    F, M, on_ground, feet = lc.forces(np.array([0, 0, -0.3]), np.zeros(3), R, np.zeros(3))
    assert not on_ground and feet == 0 and np.all(F == 0)
    F, M, on_ground, feet = lc.forces(np.array([0, 0, -0.24]), np.zeros(3), R, np.zeros(3))
    assert on_ground and feet == 1 and F[2] < 0             # one foot 1 cm into the ground, pushing up
    assert F[2] == pytest.approx(-3000.0 * 0.01)
