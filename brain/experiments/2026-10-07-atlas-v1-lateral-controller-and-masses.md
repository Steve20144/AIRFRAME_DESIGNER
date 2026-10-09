# 2026-10-07 Atlas v1: sidewind fix without fins (controller + mass placement), Blender review scene

User request: "make the changes needed to fly stably, fix the instability, show the model in Blender"; constraints
from the day before: no rear fins, the exterior unchanged. Geometry was not touched.

## Changes made
1. **Lateral controller in the Cruise Test** (`analysis/cruise.py`, `LATERAL_DEFAULT`, `free_flight(lateral_control=...)`):
   pitch and roll held by the elevons (P 1.0 / 1.5 deg per deg, D 0.3 / 0.4 deg per deg/s; the pitch channel may use
   the travel only up to 25 - 10 deg so roll always keeps 10 deg); **sideslip** zeroed by (a) splitting the forward
   thrust left/right at +-0.75 m (the wing fans, Ullman US20170190436) and (b) **split drag rudders at the wing-tip
   trailing edges** (Northrop US2412646A), modelled as an ideal aft force of CD A 0.25 m2 x q per side at the outer
   elevon's mid-span TE, opened 0..1 by -0.15 per deg of sideslip + 0.15 per deg/s of yaw rate; plus yaw-rate damping.
   `cruise_test()` now releases 'flight6' into a 10 deg side gust (`gust_beta_deg`) on top of the pitch kick and keeps
   'flight6_open' (no controller) for comparison; the pitch-plane 'flight' is unchanged (foil alone).
   - LESSON: a **heading hold fights the weathercock** in a crosswind and keeps the sideslip alive; every heading-hold
     variant rolled off. Feeding back sideslip (beta) instead holds the aircraft at once.
   - LESSON: tip drag below the CG (anhedral tips) pitches the nose down; without the pitch hold the rudders caused a
     dive at the 1 % static margin of M001 / 90 km/h.
   - Thrust split alone is weak in cruise (cruise thrust is only 135 N, so +-60 % at 0.75 m is 60 N m); the drag
     rudders (about 100 N at 1.5 m) do the work.
2. **Flight packs placed** (`scripts/atlas_v1_place_packs.py`): 18 x 2.356 kg in a 2 x 3 x 3 block inside the centre
   body ahead of the wing plus 9 BMS; `masses_m002.json` (CG 0.20 m ahead of M001's) and `masses_m002b.json`
   (0.05 m ahead), untouched `masses.json`. Builder `atlas_v1_from_masses.py` gained `--name`.
   Airframes: `airframes/atlas_v1_m002.json` (92.0 kg, CG FRD x 1.573), `atlas_v1_m002b.json` (CG 1.423).
3. **Blender review scene** `scripts/blender/atlas_v1_stability_blend.py` -> `results/atlas_v1/m002/atlas_v1_stability_review.blend`
   (+ PNGs blender_*.png): locked source skins, M001 points, packs and BMS as true-size boxes, CG of each set,
   neutral point, elevon strips, differential-thrust arrows, split drag rudders at the tips, legend. Opened in Blender 5.2.2.

## Results (Cruise Test, 30 s, kick 1 deg/s, flight6 = 10 deg side gust + controller)
| Set | kg | CG x | km/h | alpha | elevon trim | L/D | SM | pitch plane | six axes, controlled | open loop |
|---|---|---|---|---|---|---|---|---|---|---|
| M001 | 46 | 1.37 | 60 | 11.6 | -12.9 | 5.0 | 8 % | lightly damped | **stable** (roll <= 5 deg, sideslip gone in ~8 s) | rolls off, ground at 7.8 s |
| M001 | 46 | 1.37 | 90 | 5.8 | -12.8 | 3.3 | 1 % | stable | **stable** (roll <= 4.9 deg) | ground at 7.2 s |
| M002b | 92 | 1.42 | 90 | 11.0 | -14.4 | 4.8 | 7 % | stable | **stable** (roll <= 5.1 deg) | tumbles at 7.0 s |
| M002b | 92 | 1.42 | 110 | 7.9 | -14.0 | 4.0 | 2 % | departs (slow phugoid) | **stable** with the pitch hold | ground at 7.3 s |
| M002 | 92 | 1.57 | 90 | 12.9 | -19.4 | 4.1 | 15 % | stable | FAILS (elevon travel used up by the trim) | tumbles |
| M002 | 92 | 1.57 | 110 | 9.0 | -16.8 | 3.6 | 11 % | stable | FAILS | tumbles |
At 92 kg and 60 km/h the body cannot carry the weight (L/W 0.49): cruise is 90 km/h and up.
The elevon budget is the limit: cambered sections need 13 to 19 deg of up-elevon to trim, and 10 deg must stay for
roll. A CG more than ~0.1 m ahead of the M001 CG costs more elevon than it buys in margin. Reflex in the sections
(Agent B) or nose-fan trim in cruise would relax this.

## Hover regression found (not caused today)
`airframes/atlas_v1_m001.json` was rebuilt 2026-10-05 20:47 (nose flip) AFTER the 3/3 hover runs of 19:29. Today the
same hover scenario on the current M001 (and on M002, M002b, with set A, softer gains, or 273 N nose fans) ends the
same way: lifts off, hovers at 3.5 m with pitch held, then from about 8 s into the hover the pitch error grows
(-5 deg) while the aircraft drifts forward (3 to 10 m/s), and it tumbles ~10 s after the hover starts (hit the ground
9 to 11 m/s). PX4's pitch setpoint is 0 and its estimate tracks the truth, so the loop sees the error and does not
produce the torque: suspect the post-flip rotor geometry / allocation (CA_ROTOR0_PX changed sign), not the gains.
Runs: results/atlas_v1/m002/hover_*.json. To be investigated before any SITL claim for v1.

## CFD
Local standard-mesh sideslip run (alpha 3, beta 0/5/10) stopped on the user's instruction (11:49); the lateral
numbers will come from Aristotelis: bundles `results/cfd/atlas_v1/hpc/b02-sidewind-60kmh.tar.gz` (35 cases),
`b03-sidewind-90kmh.tar.gz` (35), `b04-sidewind-60kmh-fine.tar.gz` (20), Greek instructions in
`results/cfd/atlas_v1/hpc/ARISTOTELIS_READ_ME_EL.md`. The quick library's side force at beta 5 was 9 N +- 7.6 N: noise.

## Commands
    .venv/bin/python scripts/atlas_v1_place_packs.py --cg-fwd 0.05 --out .../masses_m002b.json
    .venv/bin/python scripts/atlas_v1_from_masses.py --masses <abs path> --out <abs path>/airframes/atlas_v1_m002b.json
    .venv/bin/python scripts/atlas_v1_cruise_check.py --airframe airframes/atlas_v1_m002b.json --speeds 90,110 --out results/atlas_v1/m002
    /Applications/Blender.app/Contents/MacOS/Blender --background --factory-startup --python scripts/blender/atlas_v1_stability_blend.py -- --out results/atlas_v1/m002 ... --render
