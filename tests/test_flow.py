"""The simulated optical flow must decode, through PX4's own VehicleOpticalFlow formula, to the true velocity."""
import math
from types import SimpleNamespace

import numpy as np

from airframe_designer.sensors.flow import FlowSensor


def _af(hover_pitch=0.0):
    return SimpleNamespace(hover_pitch_deg=hover_pitch, mass=SimpleNamespace(cg=[0.0, 0.0, 0.0]))


def _sim(vel_ned, rates=(0, 0, 0), pitch_deg=0.0, height=2.0):
    th = math.radians(pitch_deg)
    R = np.array([[math.cos(th), 0, math.sin(th)], [0, 1, 0], [-math.sin(th), 0, math.cos(th)]])   # body -> NED, nose up
    return SimpleNamespace(rotmat=R, rates=np.array(rates, float), vel=np.array(vel_ned, float), pos=np.array([0, 0, -height]))


def _px4_velocity(of, dt, rates_body):
    """VehicleOpticalFlow: flow_xy = -pixel, gyro = -delta_angle, comp = flow - gyro, v_x = -r comp_y/dt, v_y = r comp_x/dt."""
    flow = -np.array([of["integrated_x"], of["integrated_y"]]); gyro = -np.asarray(rates_body)[:2] * dt
    comp = flow - gyro; r = of["distance"]
    return np.array([-r * comp[1] / dt, r * comp[0] / dt])


def _measure(fs, sim, dt=0.02):
    fs.step(sim, dt)
    of, _ = fs.messages(sim, 1_000_000, dt)
    return of


def test_translation_decodes_to_velocity():
    fs = FlowSensor(_af(), {"pos": [0, 0, 0], "flow_noise": 0, "range_noise": 0}, np.random.default_rng(1))
    for v in ([1.0, 0, 0], [0, -0.5, 0], [0.3, 0.4, 0]):
        of = _measure(fs, _sim(v))
        assert abs(of["distance"] - 2.0) < 1e-9
        assert np.allclose(_px4_velocity(of, 0.02, [0, 0, 0]), v[:2], atol=1e-9)


def test_rotation_is_compensated_by_the_gyro():
    fs = FlowSensor(_af(), {"pos": [0, 0, 0], "flow_noise": 0, "range_noise": 0}, np.random.default_rng(1))
    of = _measure(fs, _sim([0, 0, 0], rates=(0.3, -0.2, 0)))
    assert np.allclose(_px4_velocity(of, 0.02, [0.3, -0.2, 0]), [0, 0], atol=1e-9)


def test_hover_mount_looks_straight_down_at_the_hover_pitch():
    af = _af(hover_pitch=20.0)
    hover = FlowSensor(af, {"pos": [0, 0, 0], "mount": "hover", "flow_noise": 0, "range_noise": 0}, np.random.default_rng(1))
    flat = FlowSensor(af, {"pos": [0, 0, 0], "mount": "structure", "flow_noise": 0, "range_noise": 0}, np.random.default_rng(1))
    sim = _sim([0, 0, 0], pitch_deg=20.0)
    assert abs(_measure(hover, sim)["distance"] - 2.0) < 1e-9                        # vertical: true height
    assert abs(_measure(flat, sim)["distance"] - 2.0 / math.cos(math.radians(20))) < 1e-9   # slant range


def test_structure_mount_tilt_gives_the_range_pitch_and_offsets():
    """A sensor fixed to the frame tilted 20.4 deg on an airframe hovering at 23.85 looks 3.45 deg towards the nose
    of PX4's body down; its offset from the CG is expressed in the hover frame."""
    from airframe_designer.sensors.flow import ekf2_params
    af = _af(23.85)
    p = ekf2_params(af, {"pos": [0.1, 0.0, 0.0], "mount": "structure", "extra_pitch_deg": 20.4})
    assert math.isclose(math.degrees(p["EKF2_RNG_PITCH"]), 3.45, abs_tol=0.01)
    th = math.radians(23.85)
    assert math.isclose(p["EKF2_OF_POS_X"], round(0.1 * math.cos(th), 3)) and math.isclose(p["EKF2_OF_POS_Z"], round(-0.1 * math.sin(th), 3))
    assert ekf2_params(af, {"pos": [0, 0, 0], "mount": "hover"})["EKF2_RNG_PITCH"] == 0.0


def test_message_carries_its_own_gyro_so_turns_cancel_on_a_tilted_mount():
    """PX4 takes the flow message's integrated gyro when it is finite (else the raw, unrotated board gyro). On the
    as-built mount (frame-fixed, 20.4 deg, hover 23.85) a pure turn, yaw included, must decode to zero velocity."""
    fs = FlowSensor(_af(23.85), {"pos": [0, 0, 0], "mount": "structure", "extra_pitch_deg": 20.4,
                                 "flow_noise": 0, "range_noise": 0}, np.random.default_rng(1))
    of = _measure(fs, _sim([0, 0, 0], rates=(0.2, -0.1, 0.5), pitch_deg=23.85))
    gyro = [of["integrated_xgyro"], of["integrated_ygyro"], of["integrated_zgyro"]]
    assert all(math.isfinite(g) for g in gyro)
    assert np.allclose(_px4_velocity(of, 0.02, np.array(gyro) / 0.02), [0, 0], atol=1e-9)
    # the sensor-frame rotation, not the body rates: the 20.4 deg mount sees part of the yaw rate on its x axis
    assert abs(gyro[0] / 0.02 - 0.2) > 1e-3


def test_flow_noise_is_the_rate_noise_of_one_reading():
    fs = FlowSensor(_af(), {"pos": [0, 0, 0], "flow_noise": 0.02, "range_noise": 0}, np.random.default_rng(3))
    rates = []
    for _ in range(4000):
        of = _measure(fs, _sim([0, 0, 0]), dt=0.02)
        rates.append(of["integrated_x"] / 0.02)
    assert abs(float(np.std(rates)) - 0.02) < 0.002
