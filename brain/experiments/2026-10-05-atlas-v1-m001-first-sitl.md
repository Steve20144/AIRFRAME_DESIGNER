# ATLAS v1 full scale, first mass placement (M001): import and first SITL hover

2026-10-05 (evening, PDT). Agent A, on the user's instruction "import this weighted model and let's see how it behaves in SITL".

## Inputs

- Masses: the user's S4 clicks on candidate v002 (`from_A/v002/masses.json`, saved 19:12 PDT): 24 points, 45.994 kg,
  the "crucial" parts only (9 DS-215 fans 4.3 kg, 9 ESCs 0.6 kg, 2 avionics batteries, 2 power modules, 2 BECs).
  The BOM's airborne total is 149.6 kg; the 18 flight packs (42.4 kg), structure, canopy and the rest are not placed.
- Builder: `scripts/atlas_v1_from_masses.py` -> `airframes/atlas_v1_m001.json` (+ `airframes/meshes/atlas_v1_cand_v002.stl`
  for the app's 3D view). FRD = R (CAD - (0, 0, 5.0 m)), R = [[0,1,0],[1,0,0],[0,0,-1]].
- S5 (`from_A/v002/cg.json`): 45.994 kg, CG FRD (-1.369, -0.001, 0.134) m = CAD (-0.001, -1.369, 4.866) m;
  Ixx 18.3, Iyy 56.5, Izz 66.8 kg m2 (point masses only, no structure: an underestimate).

## Assumptions declared in the model (all open for the user to change)

- Rotors at the fan mass points. The user placed 3 fans in each wing nacelle (right side mirrored to the left: the
  clicked left side had one fan 0.6 m further forward) and 3 at the tail (CAD y -2.84 to -3.05, nose is +Y).
  Wing fans M1 to M6: fan axis +x, jet turned 75 deg above it (no v1 foil geometry exists). Tail fans M7 to M9: vertical.
  215 N per fan (catalogue low end), tau 0.3 s, km +-0.01 alternating. T/W 4.05 at 46 kg.
- Lifting body as one linear-model wing (span 3.8 m, root chord 1.7, tip 0.7, LE sweep 24 deg, dihedral -12) plus a body drag box.
- Legs: the wing tips are the lowest points of the skin (1 m below the belly) -> two 0.15 m pads there, plus a tail
  skid (0.84 m) set so the aircraft STANDS AT THE HOVER PITCH. Reason: every fan is aft of the tip feet, so no fan can
  raise the nose from a nose-down park about those feet; the park-nose-down / rotate-up concept is not modelled.
  Hover trim 10.15 deg nose-up (within the 20 deg limit), MPC_THR_HOVER 0.249.
- PX4: V3_30's parameter set, roll/pitch gains replaced by sweep set A (below). MC_AIRMODE 0, no nose lift, no H-FLOW.

## What happened

1. Tail skid first built with the wrong sign (foot 1.8 m long): the aircraft stood 20 deg nose-down from the hover
   frame, PX4 drove the motors to 88 % at arm against the attitude error with zero throttle, jumped, abort. Fixed.
2. With V3_30's gains (rate P 0.6/0.75, attitude P 5/6) and 0.3 s fans: clean liftoff, then a roll oscillation of
   about 2 s period grows over 4 cycles to 57 deg and the aircraft hits the ground (both seeds).
3. Gain sweep, seed 1, `results/atlas_v1/gains/` (roll = pitch gains):

| Set | rate P / I / D | att P | Result |
|---|---|---|---|
| A | 0.3 / 0.1 / 0.02 | 3 | full flight, hover roll rms 0.19 deg, pitch rms 1.4, touchdown 0.59 m/s |
| B | 0.2 / 0.05 / 0.03 | 2.5 | crash in hover (10.6 m/s) |
| C | 0.15 / 0.05 / 0.01 | 2 | crash in hover (10.9 m/s) |
| D | 0.3 / 0.1 / 0.04 | 4 | full flight, pitch rms 2.0, touchdown 0.84 m/s |
| E | 0.4 / 0.12 / 0.03 | 4 | full flight, roll rms 0.12, pitch rms 1.5, touchdown 0.82 m/s |
| F | 0.3 / 0.1 / 0.06 | 5 | pitch oscillation 12 deg, crash at the cut |

   Set A chosen (baked into the airframe). Softer sets lose the aircraft; the D term on the 0.3 s fans hurts.
4. Confirmation, 3 seeds, `results/atlas_v1/m001/` (scenario `scenarios/2026-10-05_atlas_v1_m001_hover.json`:
   arm in Stabilized, lift off, 60 s hands-off hover, descent, landing, cut):

| Seed | Flight | Hover alt | Drift in 60 s | Roll rms | Pitch rms / max | Pitch mean | Touchdown | Energy |
|---|---|---|---|---|---|---|---|---|
| 1 | complete | 3.30 m | 41.5 m | 0.22 deg | 1.40 / 3.35 deg | -1.21 deg | 0.60 m/s | 198 Wh |
| 2 | complete | 3.29 m | 42.6 m | 0.29 deg | 1.44 / 3.56 deg | -1.22 deg | 0.59 m/s | 198 Wh |
| 3 | complete | 3.30 m | 41.3 m | 0.16 deg | 1.41 / 3.40 deg | -1.21 deg | 0.55 m/s | 198 Wh |

   Hover thrust 454 N (= weight), 8.4 kW, busiest fan 31 %, no saturation, max tilt 12 deg (the 10 deg hover pitch + 2).

## Findings

- The 46 kg model flies the whole profile in SITL with set A. Attitude is quiet. Behaviour is dominated by the
  0.3 s fan lag: the loop gains must be about half of the small-scale ones, and softer than set A diverges.
- Steady forward drift of 0.68 m/s (41 m in 60 s hands-off): the aircraft holds a steady -1.2 deg pitch error
  against a zero setpoint in Stabilized. Not an estimator bias (0.2 deg). Candidates: the rate-loop integrator
  limit (MC_PR_INT_LIM 0.3) at the torque the layout needs, or the trim. V3_30 drifted 0.5 to 6 m. To investigate.
- Hover altitude settles 0.8 m above the pilot's 2.5 m target (the scripted pilot's altitude hold vs the thrust curve).
- Mass realism: 46 kg is a third of the BOM. With 150 kg the hover share would be ~75 % and T/W 1.3; the same gains
  will not transfer. Place the flight packs (18 x 2.36 kg) and the structure before any conclusion about margins.
- Park/rotate: with the user's fan placement the aircraft can only lower its nose about the tip feet (tail fans
  lift the tail), it cannot raise it. A nose-down park needs fans ahead of the feet or feet aft of the CG.

## Commands

```
.venv/bin/python scripts/atlas_v1_from_masses.py
.venv/bin/python -m airframe_designer run --airframe airframes/atlas_v1_m001.json --scenario scenarios/2026-10-05_atlas_v1_m001_hover.json --instance 1 --seed 1 --out r.json --timeseries r_ts.json
```
App: launch entry `app-atlas-v1-mac` (port 8082, PX4 instance 5).

## Addendum, same evening: GUI takeoff, estimator trap, and the switch to the cruise test

- GUI session on PX4 instance 5 could not arm for 100 s: the instance directory carried saved parameters from a
  1 Oct indoor session (EKF2_GPS_CTRL 0, EKF2_HGT_REF 2). Headless runs seed their own parameters and never saw
  it. Fix: the builder now writes EKF2_GPS_CTRL 7, EKF2_HGT_REF 0, EKF2_RNG_CTRL 0, EKF2_OF_CTRL 0 into the
  airframe's px4_overrides; the stale `parameters.bson` of instance 5 was deleted. Lesson: a PX4 instance remembers.
- PX4 Takeoff mode (position control) rolled the aircraft over: with the 0.3 s fans the attitude loop lags a
  moving setpoint by about a second; the position loop then demands 36 deg of pitch. Four softer position-loop
  sets (results/atlas_v1/takeoff/) also failed; an inner-loop batch sweep was started and stopped when the user
  changed direction: the fans are placed roughly and their vectoring is not meaningful yet. Stabilized hover
  (set A) remains the only validated flight.
- Direction change: fans are mass only; the foil's forward-flight stability is the question. See
  brain/architecture/cruise-test-tab.md: the CG sits 1.0 m behind the neutral point, statically unstable.
