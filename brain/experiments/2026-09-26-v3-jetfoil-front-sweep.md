# 2026-09-26 ATLAS V3 small: jetfoil angle x front tilt sweep

Study `studies/v3_jetfoil_front_sweep.json` on `airframes/atlas_v3_small.json`: the three foil stations' thrust lean
(10 / 17.5 / 25 deg forward of vertical, each) x the nose side fans' sideways tilt (0-40 deg in 10s), 135 flights of
`scenarios/v3_stab_nolift.json` (Stabilized, lift, 12 s hover, land). Every candidate flies at its own trim:
`"hover_pitch_deg": "trim"` (`Airframe.trim_hover_pitch`: the pitch where PX4's pseudo-inverse mix leaves every fan
furthest from both zero and full) and `"hover_thrust": "trim"` (`Airframe.hover_thrust_fraction`: weight over the
fans' vertical full thrust; predicted 0.26 = measured 0.26 on the light draft; ATLAS_09B trims at 24.4, flown at 24).
Score: hover pitch + roll error + 0.05 x |heading drift| + 0.5 x position drift + liftoff/landing upsets + 2 x
touchdown speed + saturation. Top 8 re-flown on seeds 2 and 3.

## Masses: ATLAS_09B's (the user, 26 Sep: "remove the two rear motors and add one in the front")

11.785 kg: 9 fans x 0.34, battery 3.225 in the nose carrier, avionics 0.5 at the flight controller, foils 2.6 and
2.4 kg other structure spread by volume. CG 2.4 cm ahead of the origin, thrust/weight 2.65.

**Best (mean over three seeds): outer 17.5, middle 25, inner 25 deg lean, nose side fans 30 deg inward (as built)**,
trim 16.15 deg, MPC_THR_HOVER 0.395 (`airframes/atlas_v3_small_best.json`). Mean score 1.52 (1.44-1.60), heading
drift 0.2-0.4 deg, position drift 0.36-0.56 m, touchdown 0.34-0.35 m/s, busiest fan 60 %. It needs at most 72.5 deg
of jet turning, inside the 73-75 deg the CAD foil gives. Runner-up 10 / 10 / 25, nose upright: 1.55, drift 0.28-0.46
m, busiest fan 47 %, but 80 deg of turning at two stations. The top six are within 0.05: a tie band.

The draft (14.6 / 15.4 / 17.3, nose 30) fails at these masses on all three seeds: heading 83-158 deg, drift 48-59 m,
hits the ground at 2.2-2.5 m/s. Static trim already showed one fan at 85 % there.

- Different angles per station beat equal ones (as at 7.8 kg): the lean differences give the allocator yaw and
  fore-aft handles that the same-spin fans lack.
- The nose fans' sideways tilt still barely matters (best 1.45-1.55 at every tilt).

## Superseded: the 7.8 kg guess

With guessed masses (7.8 kg, `results/v3_jetfoil_front_sweep_7kg`) the best was 10 / 25 / 10, nose upright (score
1.36). At the real masses it scores 1.57, still in the top band.

## Bugs found on the way

- The rigid body (and the JSBSim, Gazebo, static, vibration, CLI and worker paths) called `mass.resolve()`, which
  ignores CAD bodies; with a mass item present (V3's avionics) the aircraft became a 0.5 kg point mass (LinAlgError).
  All now call `Airframe.resolve_mass()`.
- A fixed MPC_THR_HOVER across a geometry sweep made good candidates fail to lift off on other seeds.

## Caveats

Battery position (nose carrier), fan thrust 36 N, km 0.002 and full jet attachment are still assumptions.

## CG sweep on the best configuration (same day)

Battery (3.225 kg) moved fore/aft -0.20..+0.10 m and down/up +-0.04 m (`studies/v3_best_cg_sweep_s1..3.json`,
three seeds), then per fore/aft point five hover pitches (trim -1..+1 deg, `results/v3_cg_pitch`). CG x -3.1..+5.1 cm.

- **No CG optimum in the flights.** Best-over-pitch scores 1.39-1.99 at every CG, medians 2.3-3.4, no trend.
- **The flights are dominated by a sensitivity to the exact hover pitch**: at one CG, 0.5 deg of hover pitch swings the
  hover drift between 0.5 and 6 m (pitch tracking error 0.03 vs 0.67 deg), periodic with about 1.5 deg, identical
  across seeds. A pitch-rate integrator (0.2) makes it worse, so it is not the missing MC_PITCHRATE_I. Cause found the same day: the yaw authority limit of the same-spin fans
  ([lesson](../lessons/v3-hover-pitch-sensitivity-is-yaw-authority.md)); with km 0 it disappears.
- Static (reliable): forward CG lowers the busiest hover fan (0.60 at +2.4 cm, 0.48 at +5.1) and the trim pitch
  (about 0.46 deg per cm); aft CG raises trim to 19 deg. Recommendation: keep +2.4 cm or move up to 2-3 cm forward.

## Jetfoil resweep on the nose-pair battery layout (12.8 kg, same day)

`studies/v3_jetfoil_sweep_batt.json`: the three stations at 10/15/20/25 deg lean (64 designs), nose fans 30 as built,
each flown at trim and trim +-0.5 deg (`scenarios/v3_stab_nolift_m05/_p05`, `hover_pitch_offset_deg`), scored on the
mean of the three so a design cannot win on one lucky pitch. (The study's objective expression first missed the
`metrics.` level of multi-scenario results, so every trial scored 200; the trials were re-scored from the saved
flights and the spec fixed.) 42 / 64 fly all three pitches. Top 5 re-flown on seeds 2 and 3 (9 flights each).

**Best: outer 25 / middle 10 / inner 25 deg lean, nose 30** (`airframes/atlas_v3_small_best.json`, trim 17.0 deg,
MPC_THR_HOVER 0.431): 9/9 flights, mean score 1.98, worst 2.23, drift median 1.2 m, worst 1.55 m; hover fans 46-59 %,
nose fans 21 %. Runner-up 20 / 10 / 25: 9/9, drift worst 1.52 m. The middle station at 10 deg means the jet turned
80 deg there, more than the CAD foil's 74.6; outer and inner at 65 deg are gentler than the CAD foil.
Views: `scripts/v3_foil_views.py` writes `docs/v3_views/v3_foil_angles.html` (side, front and top views with the CAD
foil and best layers, Save PNG buttons) and, through headless Edge on Windows, `docs/v3_views/v3_{side,front,top}.png`.
