# 2026-10-01 V3_30 from STEP (SMALL_SCALE_V3_30.step): masses from top-level labels, SITL, pitch tuning

Build: `scripts/atlas_v3_30_from_step.py` -> `airframes/atlas_v3_30.json` (as drawn) and `atlas_v3_30_tuned.json`.
STEP copied to `airframes/cad/SMALL_SCALE_V3_30.step`. Helpers in `results/v3_30/` (gitignored): `step_tree.py`
(assembly tree, volume/centroid per occurrence), `foil_rays_step.py` (OCP ray-traced exit angles; reproduces the
Fusion v34 angles within 1.7 deg), `foot_points.py`, `fly.py` (parallel headless runs, instances 1..N, brief + ground
bounce), `diag.py` (hover offset vs wobble, estimate vs truth).

Mass rule (user, 1 Oct): ONLY the 53 top-level folders' labels, spread by volume over each folder's solids (the
avionics bay's 250 g covers its Pixhawk / H-FLOW / 1600 mAh; v34 counted those on top). Unlabelled = 0 g: clamp caps,
Four_feet_assembled, and an unnamed 304 cm3 solid "=>[0:1:1:189]" at CAD (0.52..0.65, 0..0.21, 0.30..0.45): the
lowest point of the model, right side only, no mirror (ask the CAD designer; ignored for weight and contact).
Result 14.40 kg (v34 11.83): 9 ESCs x 200 g (v34 had 3 labelled), All_mounts 591 g and Battery_mounts 370 g newly
labelled, front foot 152 g, two more jetfoil mounts. Foils still labelled 920 / 800 g. CG FRD (-0.114, 0.003, 0.089),
I (0.87, 1.11, 1.73), Ixz -0.128. T/W 2.20.
Geometry changes vs v34: rear battery block of four moved 195 mm aft (the v34 recommendation) and all packs 10 mm
lower; avionics bay 47 mm aft; side PDBs moved; fans unchanged. INNER foil redrawn: exit 49.5 deg (was ~70), trailing
edge 2 cm further aft; middle 58.2, outer 49.6. Legs now from the CAD: rear outboard feet FRD (-0.432, +-0.697,
0.396) and the new Front_foot_assembly skid bottom (0.5, 0, 0.10): parks at -17.6 geometric, rests -18.2 on the
springs. Hover trim 27.15 deg, MPC_THR_HOVER 0.492; static mix 40-59 % per fan, L/R uneven (foil labels, CG y).

SITL, `scenarios/2026-10-01_1511_v3_30_cad_rotate_hover.json` (rotate, 60 s Stabilized, land), 3 seeds:
| gains | pitch err | drift | lift-off | touchdown |
|---|---|---|---|---|
| start (50/65/50 tuned: rate P 0.75, I 0.1) | 0.30 | 13.4 m | 2.85 | 0.28 |
| **rate P 0.9, I 0.05 (tuned)** | 0.18 | 8.4 m | 2.47 | 0.20 |
| rate P 0.75, I 0 | 0.15 | 7.0 m | 2.89 | 0.19 (no integrator: not recommended) |
The hover error is a steady nose-down offset present from the start of the hover, fading slowly, and larger with MORE
pitch-rate I (0.2: 0.42); MC_PR_INT_LIM 0.6 and yaw gains change nothing; MC_PITCH_P 9 oscillates (12 deg/s).
Mechanism not identified. Position hold on H-FLOW (`2026-10-01_1523_v3_30_cad_rotate_poshold_hflow.json`, 1.5 m):
drift 0.12 / 0.19 m tuned vs 0.16 / 0.22 old gains, touchdown 0.27 vs 0.35.

## CFD foil thrust (OpenFOAM, user 1 Oct): V3_30 cannot hold a hover in SITL

CFD per foil group (3 fans, 3 x 36 N in; FOILS 75 / 65 / 50 deg inner/middle/outer, not the V3_30 CAD 50/58/50): (-56.12, 46.48, 0.73) N, x fan axis (aft), y up, z span -> 72.87 N
(67.5 %) at 39.63 deg above the duct axis (the CAD walls average ~52: the real jets turn ~13 deg less, i.e. they
separate). `scripts/atlas_v3_30_cfd.py` -> `airframes/atlas_v3_30_cfd.json` (each foil fan 24.29 N at 39.63 deg,
turn_loss calibrated to 0.7387; spanwise 0.73 N ignored, sign not given). Trim moves to 42.7 deg nose-up
(trim_hover_pitch range widened to 80 deg; no other airframe's trim changes), MPC_THR_HOVER 0.652, T/W 1.80.
Parked -17.6 -> PX4 sees -60.3 deg on the legs: "Attitude failure (pitch)" blocks arming (FD_FAIL_P default 60);
the script sets FD_FAIL_P 70 (also needed on the real board). Rotation to 42.7 works (nose fans 6 % at the end: the
CG is nearly above the rear feet). Lift-off with the 3 s ramp pitches nose-down at 79 deg/s (rear-foot friction
against the forward jet push, below and behind the CG); with 0.5-1 s ramps it gets airborne but saturates 60-70 % of
the time and crashes or slides ~100 m (roll offset 1.3 deg). Static margins at hover: pitch 1.5 N m (old model 7.4),
roll 5.9, yaw much larger (the side groups push mostly forward: differential thrust). What-ifs (efficiency 80-90 %,
or the foil thrust 45-55 deg up) bring pitch margin to only 2-3 N m: the layout around the CG limits pitch authority.
Not tuned: saturation is not a gain problem.
Force points on the v34 edges (~71/58/50) instead of V3_30: trim 43.0, pitch margin 1.42 N m (same conclusion).
