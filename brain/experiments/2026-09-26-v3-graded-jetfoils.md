# 2026-09-26 ATLAS V3 small: graded jetfoils (steep at the body, gentle at the foil end)

First sweep run on macOS. Study `studies/v3_jetfoil_graded.json` on `airframes/atlas_v3_small.json` (nose-pair battery
layout, 12.8 kg, nose side fans 30 deg as built). User's request: the jet turned steepest next to the body and least
at the end of the foil. In knob terms, thrust lean inner < middle < outer (δ = 90 - lean, so δ inner > middle > outer).

- Family: every strictly graded triple on 5..35 deg in 2.5 steps (286), then the outer station extended to 37.5-45
  (42 more) because the leaders sat at outer 35. New `points` optimiser (`batch/optimizers.py`) flies an explicit list.
- Same score as `v3_jetfoil_sweep_batt`: mean over trim, trim -0.5 and trim +0.5 deg. 292 / 328 fly all three.
- Top 10 re-flown on seeds 2 and 3 (`studies/v3_jetfoil_graded_s2/_s3.json`, made by `scripts/refly_top.py`).

**Best: outer 40 / middle 30 / inner 15 deg lean (jet turned 50 / 60 / 75)** -> `airframes/atlas_v3_small_graded.json`
(saved parked at trim). 9/9 flights, mean 1.32, worst 1.42, hover drift median 0.36 m, worst 0.51 m, touchdown
0.38 m/s. Trim 26.9 deg (the previous best trimmed at 17.0), MPC_THR_HOVER 0.439, busiest hover fan 69 % (was 59),
nose fans 14 % (was 21). The inner station's 75 deg is what the CAD foil gives; outer and middle turn less than it.

- Beats the previous best 25/10/25 (mean 1.98, drift median 1.2 m) under the same objective. The whole graded family
  flies well: the top 10 all drift under 1 m on every seed.
- More outer/middle lean -> higher trim pitch, foil fans carry more and the nose less. The 45/40/x designs looked best
  on seed 1 (1.16) but fell to 1.43 over three seeds: the optimum is interior, not a range artefact.
- Designs with inner 5-10 deg (85-80 deg of turning) are in the top band too; the ranking does not need them.

Views: `docs/v3_views/graded/` (`python scripts/v3_foil_views.py docs/v3_views/graded/v3_foil_angles.html
airframes/atlas_v3_small_graded.json "Graded config"`), PNGs through resvg.

Watched live in the app (SITL, `app-mac` launch config): drift 0.88 m, pitch error 0.23, roll 0.10, touchdown 0.51,
but only on a freshly booted PX4 ([lesson](../lessons/live-app-needs-fresh-px4-per-flight.md)).

Caveats as before: fan thrust 36 N, km 0.002 same spin, full jet attachment, battery spacing assumed. A 27 deg hover
is 10 deg more nose-up than the previous best; the fans at 69 % leave less margin for gusts and the landing.
