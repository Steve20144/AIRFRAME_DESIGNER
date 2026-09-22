# Experiment: ATLAS_09B from import to the tuned Stabilized sequence

Date: 2026-09-21

## Question

Does the upstream ATLAS_09B model fly the indoor sequence in Stabilized, and what does it take?

## Setup

SITL from the Tuning tab, `stab_lab_native` / `stab_lab_native_fast` (the sequence at the airframe's own park and
hover pitch). Diagnostics as single attempts (names `09B H1` to `H13` in `results/tuning/`), sweeps `09b_pitch_1`
(18), `09b_rates_1` (9), `09b_roll_2` (12). Offline allocator analysis: PX4-like pseudo-inverse hover mix from the
exported effectiveness, its clipping residual and the per-axis torque authority left. Report:
`results/stab/report_atlas_09b.html`.

## Results

| step | hover pitch err | drift 12 s | liftoff | landing | outcome |
|---|---|---|---|---|---|
| as imported (upstream gains) | 0.23 deg | 16.7 m | 5.4 deg kick | crashed (tail-over at 1.2 m/s) | no ground sequences |
| + nose lift / nose lower blocks | 0.23 | 16.6 | 5.4 | crashed | kick is PX4's: parking at hover pitch reproduced it |
| + THR_MDL_FAC 1, MPC_THR_HOVER 0.40 | 1.0 | 8.3 | 4.4 | 16 deg, 0.88 m/s | completes; steady nose-down bias appears |
| + park +4 (lift margin 19 percent) | 1.2 to 1.7 | 10 to 13 | 1.2 to 1.6 | 21 deg, 1.1 to 1.3 m/s | bias remains |
| + M7 axis fix, km 0.002, hover 24 | 0.08 to 0.17 | 3.6 to 4.6 | 0.6 to 1.2 | 2.7 to 4.5 deg, 0.48 to 0.58 m/s | flies |
| + gains (roll rate P 0.6, pitch rate P 0.45, rate I 0.3, pitch att P 5) | 0.02 to 0.22 | 2.4 to 3.9 | 0.6 to 1.0 | 1 to 4.8 deg, 0.46 to 0.54 m/s | confirmed, live flight too |

Findings behind the steps:

- PX4's estimate matched truth to 0.3 deg (`pitch_est_bias_deg`), so the 1.7 deg bias was the controller's: the
  pseudo-inverse hover mix at km 0.004 / hover 25 pinned foil fan 1 at idle, leaving -0.14 Nm nose-up authority
  against 4.6 Nm nose-down. With km 0.002 at hover 24: +2.7 / -2.7 Nm pitch, +15.8 / -3.5 roll, **+0.3 / -2.6 yaw**.
- km 0.004 also stalled the nose lift from -12: the lift's yaw-nulling split shifted thrust to the shorter-arm fan
  and the 5 percent margin vanished.
- Roll rate P from 0.3 to 0.6 cut the standing roll bias from 0.68 to 0.21 deg and drift from 6 to 2.8 m; 0.8 to 1.0
  and rate I 0.6 brought nothing more. The bias is the yaw axis at saturation coupling into roll.
- CA_METHOD 1, airmode 2, wider integrator limits, a 20 percent idle assist during the lift and a lift target 10 deg
  below hover (upstream's trick) did not help; the last made the handover worse here.

## Conclusion

The imported model needed four corrections before gains mattered. It now flies the full sequence with a 0.25 deg
roll bias and 0.2 to 0.3 m/s drift that no gain removes: yaw authority is 0.3 Nm because the canted nose fans are
the only yaw source and ten same-spin fans' reaction torque consumes it. The pilot trims that.

## Follow-up

Measure km on a thrust stand. Decide between the OG (measured fan positions) and 09B (nominal) geometries. HITL.
