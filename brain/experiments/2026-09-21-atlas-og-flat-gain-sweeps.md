# Experiment: Gain sweeps and confirmation, ATLAS_OG flat brackets, Stabilized lab sequence

Date: 2026-09-21

## Question

Which PID set gives a stable hover, smooth takeoff and smooth landing in `stab_lab`, and where are the margins?

## Setup

SITL, `stab_lab` (park 8, hover 26, nose lift 3 deg/s, arm in Stabilized, throttle hand to 2.5 m, hover 12 s,
descend 0.4 m/s, nose lower). Grid studies: `stab_rate_p` (rate P 0.1 to 0.4, 4x4), `stab_att_p` (att P 3 to 9,
4x4), `stab_rate_d_yaw` (rate D 0 to 0.01, yaw rate P 0.05 / 0.15, 18), then from the Tuning tab `seq_pid_1`
(rate P 0.25 to 0.35 x pitch att P 4 to 6, 27) and `seq_pid_2_integrators` (rate I 0.1 to 0.3, 9). Confirmation:
seeds 1 to 3, decisive liftoff, JSBSim physics, Stabilized gust test, a live flight. Objective and constraints as in
`server/tuning.py`. Results: `results/stab*/`, `results/seq_pid_*`, `results/stab/report_final.html`.

## Results

- 16 of 16 rate-P combinations flew; best at rate P 0.3 to 0.4; 0.1 sluggish (0.14 deg error, 1.4 deg liftoff).
- Attitude P: 5 best; 7 degrades (0.09 to 0.14 deg, rates 0.8 to 1.0 deg/s); **pitch P 9 oscillates at 8 deg
  with 14 percent saturation and cannot land**. Roll P benign to 7. Pitch is the weak axis.
- Rate D 0 to 0.01 all flew; roll D 0.01 gave 2 deg liftoff excursions. Yaw rate P 0.05 vs 0.15: small.
- Sequence sweeps: 35 of 36 met every acceptance target; landscape flat; integrators indifferent.
- Chosen: rate P 0.3, rate D 0.005, attitude P 5, integrators 0.2, yaw unchanged. Confirmation over three seeds,
  JSBSim and gusts: hover tracking 0.03 to 0.17 deg RMS, liftoff within 1.7 deg, landing within 1.1 deg, touchdown
  0.38 to 0.41 m/s, nose down 9.6 s, saturation 0.1 percent; gusts within 5 deg (yaw weathercocks 60 to 90 deg in
  crosswind: authority, not gains). Live flight at real time: touchdown 0.41 m/s, roll 0.5 deg RMS, 2.7 deg
  liftoff excursion (real-time pacing plus the one-leg liftoff skid).

## Conclusion

`airframes/atlas_og_flat.json` (+ `.params`) is ready for HITL. Watch pitch first on the board; lower MC_PITCH_P
before touching the rate loop if it wobbles.

## Follow-up

HITL comparison against these numbers; H-Flow or external vision if hands-off position hold is wanted indoors.
