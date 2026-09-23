# Experiment: mitigating fan vibration on ATLAS_09B (SITL, Tuning tab)

Date: 2026-09-23. App on port 8091 (`app-nose-lift-fw`, ~/PX4-nl, instance 4), attempts from the Tuning tab
(`R1 ...`, `R2 ...` in `results/tuning/`). ATLAS_09B with the committed `design.vibration` (bench log 164
calibration: 12.63 g mm, IMU at the Pixhawk, rectification -0.0067 on z). Seed 1 only: one run per row.

New metrics for this (`sim/metrics.py`, Tuning columns `vib`, `jitter`, `h lift`, `h err`): hover accel
vibration metric; motor command jitter (filtered mean |change| per step, x1000); PX4's height estimate
(LOCAL_POSITION_NED) minus truth, less its offset at rest, on the legs during the lift and over the flight.

## Round 1: `fw_nose_lift_takeoff` (GPS on)

| attempt | vib | jitter | h lift | pitch/roll err | drift |
|---|---|---|---|---|---|
| vibration off | 0.04 | 0.51 | 0.13 | 0.15/0.39 | 2.9 |
| baseline | 5.3 | 0.97 | 0.22 | 0.01/0.34 | 2.8 |
| soft mount 50 Hz (damping 0.15) | 0.28 | 0.52 | 0.13 | 0.16/0.38 | 2.9 |
| fans balanced to 3 g mm | 1.26 | 0.54 | 0.17 | 0.12/0.43 | 3.2 |
| IMU_GYRO_CUTOFF 25, IMU_DGYRO_CUTOFF 15 | 5.6 | 0.41 | 0.18 | 0.05/0.36 | 2.9 |

## Round 2: `fw_nose_lift_takeoff_indoor` (EKF2_HGT_REF 0, EKF2_GPS_CTRL 0, as on the aircraft)

| attempt | vib | jitter | h lift | h err | pitch/roll err | drift | touch |
|---|---|---|---|---|---|---|---|
| vibration off | 0.0 | 0.52 | 0.06 | 0.21 | 0.18/0.46 | 3.35 | 0.62 |
| baseline | 5.4 | 0.99 | 0.14 | 0.43 | 0.36/0.59 | 4.99 | 0.69 |
| soft mount 50 Hz | 0.3 | 0.52 | 0.22 | 0.40 | 0.16/0.45 | 3.20 | 0.61 |
| EKF2_ACC_B_NOISE 0.03 | 5.5 | 0.98 | 0.10 | 0.39 | 0.36/0.59 | 4.99 | 0.69 |
| EKF2_BARO_NOISE 1.0 | 5.4 | 0.95 | 0.09 | 0.38 | 0.40/0.57 | 4.92 | 0.69 |
| both | 5.4 | 0.96 | 0.08 | 0.37 | 0.38/0.57 | 4.92 | 0.69 |

## Round 3: indoor scenario, seeds 2-4 (mean +- sd over 3)

| attempt | vib | jitter | h lift | pitch/roll err | PX4 pitch est bias | drift | touch |
|---|---|---|---|---|---|---|---|
| baseline | 5.4 | 0.96+-0.01 | 1.09+-0.34 | 0.38/0.66 | -0.16 | 5.45+-0.24 | 0.73 |
| soft mount 50 Hz | 0.3 | 0.51+-0.01 | 0.65+-0.44 | 0.20/0.55 | -0.02 | 3.90+-0.34 | 0.66 |
| gyro LPF 25 / D 15 | 5.5 | 0.43+-0.00 | 0.97+-0.48 | 0.38/0.67 | -0.16 | 5.50+-0.57 | 0.74 |
| mount + LPF | 0.3 | 0.25+-0.00 | 0.89+-0.35 | 0.22/0.54 | -0.03 | 3.93+-0.43 | 0.65 |

(no vibration, seed 1: pitch est bias +0.01, pitch err 0.18.) The filters halve the motor jitter but leave the
attitude error and drift exactly where they were: the vibration biases PX4's pitch *estimate* by -0.16 deg
(indoors, without GPS, the EKF's tilt leans on the accelerometer), and the gyro / D-term filters sit only in the
control path. The height errors are ~1 m with or without vibration on these seeds: baro-only height noise, not a
vibration measure. V4 will fly without a damper (user, 2026-09-23): the open lever is estimator-side.

## Round 4: estimator settings on top of the filters (indoor, seeds 2-4)

All with IMU_GYRO_CUTOFF 25 / IMU_DGYRO_CUTOFF 15 and the vibration on, except the reference.

| attempt | jitter | pitch est bias | pitch/roll err | drift | touch |
|---|---|---|---|---|---|
| no vibration (2 seeds; s4 hit the SITL arming flake) | 0.51 | -0.01 | 0.19/0.57 | 3.96 | 0.66 |
| filters only (round 3) | 0.43 | -0.16 | 0.38/0.67 | 5.50 | 0.74 |
| EKF2_GRAV_NOISE 3 | 0.42 | -0.09 | 0.31/0.88 | 6.40 | 0.85 |
| EKF2_GRAV_NOISE 10 | 0.43 | -0.07 | 0.28/0.93 | 6.64 | 0.89 |
| gravity fusion off (EKF2_IMU_CTRL 3) | 0.43 | +0.10 | 0.09/1.00 | 6.70 | 0.92 |
| EKF2_ACC_NOISE 1.0 | 0.42 | -0.19 | 0.42/0.57 | 4.99 | 0.69 |

Down-weighting the accelerometer's gravity cuts the pitch estimate bias but hands the error to roll, drifts
further and lands harder; ACC_NOISE 1.0 is neutral. No estimator setting recovers the clean flight: for a V4
without a damper keep the EKF defaults, take the filters (jitter) and remove the vibration at the source (fan
balancing, round 1: 3 g mm gave jitter 0.54 and nearly clean numbers).

## Reading

- Vibration doubles the motor command jitter (gyro vibration through the D term); indoors it also doubles the
  hover attitude error and adds 1.6 m of hands-off drift. A 50 Hz soft mount or balancing removes all of it;
  lower gyro / D-term cutoffs remove the jitter (round 1, not yet tried indoors).
- The EKF changes do nothing for jitter, attitude or drift; they trim the on-leg height error 0.14 -> 0.08 to 0.10.
- The height errors are noisy at 0.1-0.2 m (the soft mount's 0.22 on the legs has no vibration behind it): judge
  them over seeds before believing any EKF gain.
- Without GPS PX4's arming summary never names a position mode (it arrives as an undecoded number), so
  `wait_ready` cannot pass; the indoor scenario waits 20 s instead.
- The Tuning tab handed the app's own PX4 instance (4, the ~/PX4-nl build, whose lock file reads as free) to an
  attempt; the reservation now skips `state.conn.px4_instance`. `/api/tuning/runs` answers in >20 s while six
  attempts run.
