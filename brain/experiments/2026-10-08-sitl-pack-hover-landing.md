# 2026-10-08 Headless SITL package: oscillation, vibration, rotation smoothness, hover, landing order

Tools: `scripts/sitl_pack.py --spec studies/X.json --out results/sitl_pack/X` (one worker per free PX4 instance,
back-to-back jobs per instance, start-up retries) and `scripts/sitl_pack_report.py DIR [--ref NAME]` (ratio-to-reference
score). Time series now carry `feet_down`, `nl_state`, `nl_cmd` (sim/metrics.py; px4/link.py `state_i`).
Speed: 7.6 flights/min one at a time -> 21 to 29 flights/min on 8 instances (10-core Mac, CPU bound).
Metrics per flight: lift/lowering rate max, deviation from 3 deg/s, jerk, overshoot; hover (window from 90 % of the
settled height): height std, drift, speed, attitude error vs setpoint, rates rms, dominant pitch/roll/height oscillation
(Hz, amplitude), motor jitter, PX4 accel/gyro vibration; touchdown vz/vh; lowering start: feet down, climb rate, nose
fall since touchdown, wait; front-leg touch rate.

Findings:
- tune1 (52 flights, one change at a time, 36/30 N): MC_*RATE_D lower = oscillation (rates x3, 2/4 failed): keep D.
  Z velocity gains up = height std halved; LNDMC_TRIG_TIME 0.5 shortens the wait; NL_LOW_KQ/KQI change nothing; gyro
  low-pass does not lower the vibration metrics (they are on the raw IMU; the source is the fan imbalance model, 12.6
  g mm): vibration is mechanical, not a parameter.
- THE LANDING DEFECT: under PX4 Land the rear feet touch at 0.2 m/s, then PX4 winds the thrust down on two feet and the
  nose falls forward uncontrolled at ~2.9 deg/s for 5 s (8.5 -> -6 deg) until "landed"; only then did the module lower.
  Fix (firmware): NL_AUTO_TD stage at which the module takes over (0 landed, 1 maybe landed, 2 ground contact, 3 own
  cue: below 0.6 m and |vz| < 0.06 m/s for 0.15 s, or the nose 2 deg under the target while |vz| < 0.15); at touchdown
  under PX4 Land the module lowers at once (no 0.5 s cut). Nose fall before the controlled lowering 13.5 -> 0.3 to 1.0
  deg, wait 5.2 -> 0.4 to 0.5 s, lowering jerk 2.5 -> 1.3, front-leg touch 0.95 deg/s.
- PX4 hover in Position mode (NL_AUTO_HOLD 1, Auto Loiter is refused without GPS) + PX4 Land at 0.2 m/s with the crawl
  0.15 below 0.7 m: height std 1.1 to 1.5 cm at 36 N (2 to 8 cm at 30 to 28 N), drift 0.12 to 0.14 m at 36 N, 0.4 to
  0.6 at 32, 0.7 to 0.9 at 30, 1.1 to 1.7 at 28; touchdown vz 0.15 to 0.20.
Final package: 12/12 PASS (36/32/30/28 N x 3 seeds), switch-off 4/4, kill 4/4. Two flights flagged "legs first: no"
by the metric only because a 0.10 to 0.12 m/s bounce at touchdown sits in its 0.2 s window (at the lowering start: 2
feet, vz 0.02, nose fall < 0.6 deg).
Final firmware + params for HITL: `results/sitl_pack/final/` (image, sources, final_params.json). Not done: HITL.

## Climb tremble (user: "it trembles when it climbs") - climb1 (66) and climb2 (72 flights)
New metric: pitch-rate oscillation (0.5 s mean removed) in the handover and the first 3 s of PX4 control. Baseline:
~2 Hz pitch-rate oscillation, rms 2.7 / 9.1 / 11.1 deg/s at 36 / 30 / 28 N (worse with weaker nose fans; roll clean).
One-at-a-time: MC_PITCH_P 6 -> 4.5 halves it; MC_PITCHRATE_D 0.012, NL_AUTO_VUP 0.3, MC_PITCHRATE_P 0.55 (1 fail)
help less; NL_FADE_S 4 worse. Combination chosen: **MC_PITCH_P 4.0 + MC_PITCHRATE_D 0.012**: climb osc 0.67 / 3.1 / 3.7,
36 N climb max 9.1 -> 5.8 deg/s, hover rates 2.3 -> 1.3, drift unchanged at 36 N (0.14 m) but 1.3 / 2.9 m at 30 / 28 N
(was 0.8 / 1.4): softer pitch costs position hold when the nose fans are weak. Saved in final_params.json. Single
lift-off kicks of 24 to 30 deg/s remain at 30 / 28 N.
