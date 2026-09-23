"""Fan vibration at the IMU (sensors/vibration.py): closed-form checks against the rigid-body physics it stands for."""
import numpy as np
import pytest

from airframe_designer.geometry.paths import get_path, set_path
from airframe_designer.sensors import SensorSuite
from airframe_designer.sensors.vibration import VibrationMetrics, VibrationModel, bench, vibration_from_airframe


class State:
    def __init__(self, n, on=(0,), omega=0.8):
        self.omega = np.zeros(n); self.omega[list(on)] = omega
        self.thrust = 8.0 * self.omega ** 2
        self.rates = np.zeros(3); self.accel_body = np.array([0.0, 0.0, -9.80665])
        self.rotmat = np.eye(3); self.pos = np.zeros(3)


def run(vm, st, rate, seconds):
    out = [vm.sample(st, int(round((k + 1) * 1e6 / rate))) for k in range(int(rate * seconds))]
    return np.array([a for a, _ in out]), np.array([g for _, g in out])


def test_off_unless_enabled(quad):
    assert vibration_from_airframe(quad) is None
    quad.design["vibration"] = {"enabled": False, "imbalance_gmm": 5}
    assert vibration_from_airframe(quad) is None
    quad.design["vibration"]["enabled"] = True
    assert isinstance(vibration_from_airframe(quad), VibrationModel)


def test_imbalance_is_a_rotating_force_over_the_mass(quad):
    """One fan, IMU at the CG, sampled far above the fan speed: |accel| is the constant U w^2 / m."""
    vm = VibrationModel(quad, {"enabled": True, "rpm_max": 12000, "imbalance_gmm": 20})
    st = State(4, on=(0,), omega=0.5)
    acc, _ = run(vm, st, 50000.0, 0.05)
    w = 2 * np.pi * 0.5 * 12000 / 60
    expect = 20e-6 * w * w / quad.mass.mass
    norms = np.linalg.norm(acc, axis=1)
    assert norms.mean() == pytest.approx(expect, rel=0.01) and norms.std() < 0.01 * expect
    ax = np.asarray(quad.active_rotors()[0].axis, float)
    assert np.abs(acc @ ax).max() < 1e-9 * expect + 1e-12        # in the disc plane only


def test_gyro_is_the_integral_of_the_angular_acceleration(quad):
    vm = VibrationModel(quad, {"enabled": True, "rpm_max": 12000, "imbalance_gmm": 20})
    st = State(4, on=(1,), omega=0.6)
    rate = 50000.0
    acc, gyr = run(vm, st, rate, 0.02)
    acc, gyr = acc[1:], gyr[1:]                                  # the first sample has no interval behind it
    r = np.asarray(quad.active_rotors()[1].pos, float) - np.asarray(quad.cg, float)
    F = acc * quad.mass.mass                                     # IMU at the CG: accel = F / m
    alpha = (np.linalg.inv(quad.mass.tensor()) @ np.cross(r, F).T).T
    dgyr = np.diff(gyr, axis=0) * rate
    mid = 0.5 * (alpha[1:] + alpha[:-1])
    assert np.abs(dgyr - mid).max() < 0.01 * np.abs(mid).max()


def test_imu_lever_arm_adds_the_tangential_acceleration(quad):
    base = VibrationModel(quad, {"enabled": True, "imbalance_gmm": 5})
    arm = VibrationModel(quad, {"enabled": True, "imbalance_gmm": 5, "imu_pos": list(np.asarray(quad.cg) + [0.2, 0.0, 0.0])})
    st = State(4, on=(0,), omega=0.7)
    run(base, st, 250.0, 0.1); run(arm, st, 250.0, 0.1)
    assert arm.last_rms_g != pytest.approx(base.last_rms_g, rel=1e-3)


def test_soft_mount_isolates_above_its_frequency(quad):
    st = State(4, on=(0,), omega=1.0)                            # 500 Hz at the default 30000 rpm
    hard = VibrationModel(quad, {"enabled": True})
    soft = VibrationModel(quad, {"enabled": True, "mount_hz": 50, "mount_damping": 0.1})
    run(hard, st, 250.0, 0.05); run(soft, st, 250.0, 0.05)
    r, z = 10.0, 0.1
    expect = abs(1 + 2j * z * r) / abs(1 - r * r + 2j * z * r)
    assert soft.last_rms_g / hard.last_rms_g == pytest.approx(expect, rel=1e-6)


def test_frame_mode_amplifies_at_resonance(quad):
    st = State(4, on=(0,), omega=0.5)                            # 250 Hz
    plain = VibrationModel(quad, {"enabled": True})
    mode = VibrationModel(quad, {"enabled": True, "frame_modes": [{"hz": 250, "damping": 0.05, "gain": 0.5}]})
    run(plain, st, 1000.0, 0.02); run(mode, st, 1000.0, 0.02)
    assert mode.last_rms_g / plain.last_rms_g == pytest.approx(abs(1 + 0.5 / (2j * 0.05)), rel=1e-6)


def test_the_sampler_averages_each_interval(quad):
    """A tone at exactly the sensor rate averages to zero in every sample (the box filter's null)."""
    vm = VibrationModel(quad, {"enabled": True, "rpm_max": 15000})
    acc, gyr = run(vm, State(4, on=(0,), omega=1.0), 250.0, 0.2)  # 250 Hz tone, 250 Hz sensors
    assert np.abs(acc).max() < 1e-9 and vm.last_rms_g > 0.01


def test_rectification_biases_the_accel(quad):
    vm = VibrationModel(quad, {"enabled": True, "rpm_max": 15000, "accel_rectification": [0.0, 0.0, 2.0]})
    acc, _ = run(vm, State(4, on=(0,), omega=1.0), 250.0, 0.2)
    assert acc[:, 2].mean() == pytest.approx(2.0 * vm.last_rms_g ** 2, rel=1e-6)


def test_blade_pass_ripple_is_axial(quad):
    vm = VibrationModel(quad, {"enabled": True, "imbalance_gmm": 0.0, "blade_ripple": 0.05, "blades": 3, "rpm_max": 6000})
    acc, _ = run(vm, State(4, on=(2,), omega=0.9), 20000.0, 0.05)
    ax = np.asarray(quad.active_rotors()[2].axis, float)
    along = acc @ ax
    assert np.abs(along).max() == pytest.approx(0.05 * 8.0 * 0.81 / quad.mass.mass, rel=0.02)
    assert np.abs(acc - np.outer(along, ax)).max() < 1e-9


def test_metric_matches_px4_formula_and_scales_linearly(quad):
    om = [1.0, 0.8, 0.0, 0.0]
    a = bench(quad, om, rate=400.0, seconds=5.0, noise=False, cfg={"imbalance_gmm": 1.0})
    b = bench(quad, om, rate=400.0, seconds=5.0, noise=False, cfg={"imbalance_gmm": 2.0})
    assert b["accel_metric"] == pytest.approx(2 * a["accel_metric"], rel=1e-6)
    assert {t["rotor"] for t in a["tones"]} == {1, 2}
    m = VibrationMetrics()
    for k in range(3000):
        m.update(np.array([np.sin(k), 0.0, 0.0]), np.zeros(3))
    ref = np.mean(np.abs(np.diff(np.sin(np.arange(3000)))))
    assert m.accel == pytest.approx(ref, rel=0.05)


def test_clipping_counts_samples_at_the_range():
    m = VibrationMetrics()
    m.update(np.array([0.0, 0.0, -170.0]), np.zeros(3))
    m.update(np.array([0.0, 0.0, -9.8]), np.array([40.0, 0.0, 0.0]))
    assert m.accel_clipping == 1 and m.gyro_clipping == 1


def test_sensor_suite_adds_vibration_when_enabled(quad):
    st = State(4, on=(0, 1, 2, 3), omega=0.6)
    plain = SensorSuite(seed=3); plain.noise.enabled = False
    plain.set_airframe(quad)
    quad.design["vibration"] = {"enabled": True, "imbalance_gmm": 10, "rpm_max": 20000}
    shaken = SensorSuite(seed=3); shaken.noise.enabled = False
    shaken.set_airframe(quad)
    for k in range(1, 200):
        p = plain.hil_sensor(st, k * 4000); s = shaken.hil_sensor(st, k * 4000)
    assert p["zacc"] == pytest.approx(-9.80665)
    assert abs(s["xacc"]) + abs(s["yacc"]) + abs(s["zacc"] + 9.80665) > 1e-3
    assert shaken.vibration_status()["enabled"] and shaken.vibration_status()["accel_metric"] > plain.vibration_status()["accel_metric"]


def test_paths_create_the_vibration_block(quad):
    assert "vibration" not in quad.design
    set_path(quad, "design.vibration.mount_hz", 40)
    assert get_path(quad, "design.vibration.mount_hz") == 40
    with pytest.raises(KeyError):
        get_path(quad, "design.nothing.here")
