# Fan vibration model

Added 2026-09-23 (`airframe_designer/sensors/vibration.py`, schema in docs/SCHEMA.md "Fan vibration").

## What it does

`design.vibration` on the airframe (off unless `enabled`) adds the fans' vibration to the simulated IMU samples:
1P imbalance (`imbalance_gmm`, a force U w^2 rotating in each fan's disc plane, `rpm_max` maps the normalised fan
speed to rpm) and optional blade-pass thrust ripple. Path: rigid body about the CG (F/m, I^-1 (r x F) on the
`imu_pos` lever arm, gyro = its integral), optional `frame_modes` and soft mount (`mount_hz`), then the HIL
sampler as a box average over each sample (PX4's integrated samples), so tones alias as through a real IMU.
Closed-form phasors per sensor step, nothing enters the rigid-body integration. Costs ~31 us per sensor step when
on (the physics is ~130 us per sub-step), nothing when off.

PX4's own formulas are applied to what we send: accel / gyro vibration metric (VehicleIMU: 0.99/0.01 filtered
|sample - previous|) and clip counts (16 g / 2000 deg/s, SimulatorMavlink's ranges). They show in the status bar
(`Vib` pill, red above 3 or on clipping), in `run_once` metrics (`accel_vibration_mean/max` and `gyro_*` per phase,
`accel_clipping`, `gyro_clipping`) and in time series (`vib_acc`, `vib_gyro`). Not cross-checked against PX4's
VIBRATION message yet (same formula on the same samples, so they should agree in SITL).

`--set design.vibration.<key>=...` works on any airframe: `set_path` now creates missing dict levels.
The "Fan vibration" checkbox (Flight tab) writes `design.vibration.enabled` into the airframe.

## No-PX4 bench

`python -m airframe_designer vibration --airframe X --motors 9=1,10=0.82 --rate 400 [--target-metric 3.8]
[--set design.vibration.mount_hz=40] [--json]` holds fans at fixed speeds, prints rms at the IMU, the metric, each
tone and its alias; `--target-metric` scales `imbalance_gmm` to a logged metric (linear when 1P dominates).

## Numbers so far (defaults: 30000 rpm, 12 blades, 1 g mm, hard mount, IMU at CG)

- ATLAS_OG_FLAT, bench log 164 case (M9 100 %, M10 82 %): 0.1 g rms, metric 0.18 at 250 Hz, 0.21 at 400 Hz,
  1.18 at 1 kHz. At 250 Hz a full-speed 500 Hz tone is a sampler null (averages to zero).
- Reproducing the logged 3.8 at 400 Hz needs an effective 18 g mm: more likely a frame resonance amplifying a
  smaller imbalance than 18 g mm of true imbalance. Real rpm_max, the board's accel rate and a frame mode are unknown.
- `stab_lab` on ATLAS_OG_FLAT (SITL, seed 1): hover metric 0.04 off, 1.3 at 5 g mm, 4.8 at 18 g mm; all three fly and
  pass with unchanged attitude and altitude tracking, no clipping. Zero-mean vibration alone does not disturb the
  EKF; the bench height drift needs `accel_rectification` (DC bias per g^2 rms), not calibrated yet.
- Motor override of both nose fans at full with nobody flying flipped the airframe on its legs (3 samples > 16 g
  counted as clipping): contact spikes also show up in the clip counter.

## ATLAS_09B end to end in the app (2026-09-23, SITL, ~/PX4-nl, `fw_nose_lift_takeoff`, park +4)

Calibrated to bench log 164 at the app's 250 Hz: `imbalance_gmm` 12.63 with `imu_pos` [0.15, 0, -0.13] (the
Pixhawk, `design.pixhawk_position`), `accel_rectification` [0, 0, -0.0067] (0.03 m/s^2 at the bench's 2.1 g rms,
the lesson's "few hundredths"). Set on the live airframe only, not saved in `atlas_09b.json`.
Metric 3.8 at the start of the lift (the log's value), 5.0 to 6.7 in hover with all ten fans. Both runs pass:
lift 7.21 s vs 7.24 s without vibration, handover 4.14 s both, touchdown 0.48 vs 0.54 m/s, no clipping, no false
liftoff abort (the baro check holds with the rectification bias). End position 6.7 m vs 8.2 m from the start and
yaw 32 vs 20 deg: the drift is the hands-off yaw-authority drift, present without vibration too.

## Next

Measure fan rpm at full command and the board's accel sample rate; calibrate `imbalance_gmm` (and a frame mode) to
log 164 at that rate; fit `accel_rectification` to the 0.85 m / 6.4 s height drift and check that the firmware
nose lift's baro liftoff check holds in SITL with it; try `mount_hz` 30 to 60 to size the soft mount.
