# Experiment: ATLAS_09B rounds 4 and 5, the yaw loop and the hands-off drift

Date: 2026-09-21 (after the CAD integration; app on `airframes/atlas_09b.json`, scenario `stab_lab_native_fast`)

## Question

Can gains remove the standing roll bias and the hands-off drift that the first 09B tune left (0.25 deg, 0.2 to
0.3 m/s), and what is the mechanism?

## Setup

Sweeps from the Tuning tab (4 workers): `09b_yaw_4` (MC_YAW_WEIGHT 0.2/0.4/0.6 x MC_YAWRATE_P 0.1/0.225/0.35 x
MC_ROLL_P 3/5, 18 trials) and `09b_yaw_5` (MC_YAWRATE_P 0.03/0.065/0.1 x MC_YAWRATE_I 0/0.05/0.1 x MC_ROLL_P 5/7,
18 trials). Single attempts in `results/tuning/` named `09B ...`: baseline (committed gains), hover 23 and 22,
yaw soft / softer / minimal, sets A to D, set A over seeds 2 to 4 and on JSBSim. A first `09b_yaw_4` ran on a
leaked live stance (hover 26 / park -5) and was deleted; see lessons/environment-gotchas.md.

## Results

| set (rest as committed) | hover roll bias | 12 s drift | heading wander | touchdown |
|---|---|---|---|---|
| committed: yaw P 2.8, yaw rate P 0.2 / I 0.1, roll P 3 | 0.40 deg | 3.7 m | 1.8 deg | 0.51 m/s |
| yaw rate P 0.1, roll P 5 (best of round 4) | 0.26 | 2.7 | 2.1 | 0.46 |
| yaw rate P 0.065, I 0, roll P 5 (round 5) | 0.14 to 0.23 | 1.7 to 2.6 | 1.3 to 1.7 | 0.43 |
| **set A**: yaw P 1.0, yaw rate max 30, rate P 0.065 / I 0.02, roll P 5 | 0.11 to 0.17 | 1.8 to 2.2 | 0.4 to 1.4 | 0.41 to 0.42 |
| yaw minimal: yaw P 0.5, rate P 0.02 / I 0 | 0.03 | 1.6 | 9.2 | 0.40 |
| hover 23 or 22 (park 4) | 0.4 | 15 | 6 | 1.4 (pitch bias 1.5 deg) |

- Every trial of both rounds completes the sequence: liftoff pitch excursion 0.9 deg, landing 3 deg, touchdown
  0.40 to 0.51 m/s, saturation 4 percent. The only failing criterion is the 2 m hands-off drift.
- Trends are monotonic: lower yaw rate P, lower yaw attitude P and a yaw rate cap all reduce the roll bias and the
  drift; MC_YAW_WEIGHT has no effect; MC_ROLL_P 5 beats 3, 7 adds nothing; yaw rate I does not matter.
- Mechanism (time series): at liftoff the heading swings 10 to 15 deg uncommanded while the yaw estimate lags truth
  by 3 deg. The heading-hold demand saturates the allocator (yaw authority about 0.3 Nm one way), PX4 freezes the
  roll and pitch integrators under saturation, and the estimate settles 0.15 deg off the setpoint in both axes.
  The aircraft picks up 0.2 to 0.3 m/s in that transient and, without a position loop, keeps it. Ram drag balances
  a residual 0.18 deg tilt at that speed.
- Run-to-run variation is bimodal with the seed fixed: two liftoff outcomes (truth pitch 0.00 vs -0.18 deg at hover)
  appear in every sweep, and a repeat of a passing trial fell into the other mode. Judge sets on their trend and
  over several seeds, not on a single score.
- Live on the app's PX4 (real time, parameters pushed at runtime, two flights): both complete the sequence; roll bias
  -0.45 and -0.15 deg, drift 4.0 and 1.6 m, heading wander 8 and 5 deg, touchdown 0.47 and 0.39 m/s. Same bimodal
  spread as headless, heading wander larger live.
- JSBSim with set A: roll bias -0.10 deg (opposite sign), drift 2.4 m, touchdown 0.45 m/s: same order, so the bias
  is not an artefact of one engine.

## Conclusion

Set A is adopted in `airframes/atlas_09b.json` (and `.params`). It halves the standing roll bias and the hands-off
drift against the committed gains at the cost of 1 to 2 deg of heading wander in 12 s. No gain removes the rest:
the cause is yaw authority with ten same-spin fans. Counter-rotating fan pairs (or the measured km being smaller)
would remove it; a pilot trims 0.2 m/s with a stick touch.

## Follow-up

Measure km; consider counter-rotating fans; HITL with `atlas_09b.params`; the drift criterion is OG-grade, for 09B
report the number rather than gate on it.
