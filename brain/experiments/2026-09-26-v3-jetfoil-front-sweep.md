# 2026-09-26 ATLAS V3 small: jetfoil angle x front tilt sweep

Study `studies/v3_jetfoil_front_sweep.json` on `airframes/atlas_v3_small.json` (assumed masses, 7.8 kg, full jet
attachment): the three foil stations' thrust lean (10 / 17.5 / 25 deg forward of vertical, each) x the nose side
fans' sideways tilt (0-40 deg in 10s), 135 flights of `scenarios/v3_stab_nolift.json` (Stabilized, lift, 12 s hover,
land). Every candidate flies at its own trim: `"hover_pitch_deg": "trim"` (`Airframe.trim_hover_pitch`, the pitch
where PX4's pseudo-inverse mix keeps its weakest fan strongest) and `"hover_thrust": "trim"`
(`Airframe.hover_thrust_fraction`, weight over the fans' vertical full thrust; 0.26 predicted = 0.26 measured).
Score: hover pitch + roll error + 0.05 x |heading drift| + 0.5 x position drift + liftoff/landing upsets + 2 x
touchdown speed + saturation. Top 8 re-flown on seeds 2 and 3.

## Result

Best, and the most consistent over three seeds: **outer 10, middle 25, inner 10 deg, nose fans upright (0 deg)**,
trim 10.0 deg, MPC_THR_HOVER 0.253 (`airframes/atlas_v3_small_best.json`). Mean score 1.36 (1.31-1.40); heading
drift 0.0 deg, position drift 0.17-0.21 m, pitch/roll error 0.05/0.01-0.03 deg, touchdown 0.34-0.36 m/s, fans at
29 %. The draft (14.6 / 15.4 / 17.3, nose fans 30) on the same flights: score 4.4-4.6, heading 17-21 deg, drift 2.5-2.9
m, fans 46 %. The next seven are within 0.2 of the best: a tie band, not a ranking.

## What the sweep says

- Stations at different angles beat equal ones. All three equal fails or scores 4-14 (at 10/10/10 and 17.5/17.5/17.5
  it mostly fails): different leans give the allocator independent fore-aft and yaw handles (differential thrust of
  forward-leaning fans at +-y is yaw authority, which the same-spin fans lack).
- More turning is better: median score 1.5 at 10 deg on a station against 2.5 at 25.
- The nose fans' sideways tilt barely matters (best 1.30 at every tilt 0-40): the foil fans carry yaw. Upright nose
  fans are the simplest part and are as good.

## Caveats

10 deg lean = the jet turned 80 deg; the CAD's foil exit is 73-75 deg and real jets separate before it, so the
winner asks for more turning than this foil gives. Masses, fan thrust and km are assumptions. The first run of the
sweep (fixed MPC_THR_HOVER 0.26, `results/v3_jetfoil_front_sweep_fixedthr`) ranked similarly but many candidates
failed to lift off on other seeds: a geometry sweep must match the hover throttle per candidate.
