# Confirmed nine-unit layout and illustrative EDF reference

2026-10-05T22:44:21.725958+00:00

## Resolved user facts

Nine propulsion units:3 rear-right,3 rear-left,3 front, grouped like the small-scale model. Fans run during forward flight. This resolves count and group topology only. Exact full-scale centres, inclinations, diameters, spin directions and performance are not implied. Do not ask fan count again.

## Which small-scale version means what

- `brain/architecture/atlas-v3-small.md` and `airframes/atlas_v3_small.json`: initial SMALL_SCALE_V3 reference, six foil fans arranged inner/middle/outer on each side; front lateral pair and centre unit. The nominal rear lateral offsets are±.200/.282/.364m, foil exit angles about72.7/74.6/75.4°. Historical assumptions include36N and km=.002; these do not become full-scale facts.
- `brain/architecture/atlas-v3-fusion.md`, `airframes/atlas_v3_v34.json`: later live Fusion extraction, rear lateral offsets±.2078/.3104/.413m. Rear rotor entries are placed on the **jet EXIT line** for force application. Front pair FRD positions(.335,±.078,.057)m; centre(.425,0,.060)m. These are prototype coordinates only.
- `brain/experiments/2026-10-01-v3-30-step-import-and-tune.md`, `airframes/atlas_v3_30.json`, and read-only inspection of `scripts/atlas_v3_30_from_step.py`: fan hardware layout retained fromv34; inner foil redrawn and exit force points/angles changed. Actual as-drawn foil angles≈49.5/58.2/49.6°, not the initial values. Neither force points nor net-force vectors identify physical rotor discs.
- `atlas_v3_30_jets_57.5_85_85_park-10.json` is a target jet-angle scenario. Its85°/57.5° net-force vectors are not hardware shaft orientation or a demonstrated full-scale flow condition. Earlier ATLAS_OG/09B ten-fan configurations are not the newly confirmed nine-unit topology.

## Axis semantics and full-scale mapping proposal

Small-scale FRD is x forward,y right,z down; the older CAD frame is x right,y up,z aft. The confirmed atlas_v1 frame has nose+Y; with its existing+Z-up convention, a **direction** maps from FRD to full-scale CAD as(FRD_y,FRD_x,−FRD_z). This is an orientation correspondence, not a position transform or scaling rule; origin and physical centres remain undefined.

All six rear prototype `duct_axis` values are approximately(+1,0,0) FRD, representing forward force before jet turning. Raw airflow/exhaust is opposite, aft. Qualitatively this corresponds to+Y force/−Y raw exhaust in atlas_v1. The `axis` field is the tilted net force after jetfoil turning; do not use it as the EDF shaft or flow boundary normal.

The front prototype thrust vectors are left(0,+.5,−.866), right(0,−.5,−.866), centre(0,0,−1): the pair thrusts upward and inward at30° from vertical, centre upward. Exhaust is opposite, downward/outward for the pair. In atlas_v1 direction notation these would be(+.5,0,+.866), (−.5,0,+.866), (0,0,+1) **only if the prototype inclinations are explicitly retained**. They are not confirmed full-scale angles.

Reviewable group map (no CAD placement performed):

| Full-scale slot | Prototype identifier | Qualitative anchor |
|---|---|---|
| Rear left, outer/middle/inner | M1/M3/M5 | Three left-side rear flow-opening candidates, order from tip toward centre |
| Rear right, outer/middle/inner | M2/M4/M6 | Mirrored right-side candidates |
| Front left/right | M7/M8 | A lateral pair ahead of rear groups |
| Front centre | M9 | Centreline; prototype places it ahead of lateral pair |

Candidate openings visible in the correct STEP are review anchors, not certified rotor centre planes. The file remains three open shells with no labelled full-scale fan assembly. Exported PX4 motor numbers are not automatically physical controller channel assignments; historical mappings changed.

## Manufacturer reference, not a selected installation

Official page: https://www.schubeler.com/product/ds-215-dia-hst/ . Tech specs identify DS-215-DIA HST with DSM10066-290:195mm casing inside diameter,215cm² swept area,3.4kg including motor,215–250N static thrust,9.8–15.6kW electrical,12000–14000rpm,84–98m/s exhaust,12–14S20Ah recommendation. CW/CCW options exist. The chart link uses295 while the specification uses290; revision is unconfirmed and no chart values were digitized.

If nine identical reference units were used, arithmetic reference totals would be30.6kg EDF mass,88.2–140.4kW electrical input range, and1935–2250N scalar static thrust sum. These are separate catalogue aggregates, not paired operating points, installed net/vertical force, actual ATLAS mass, endurance or forward-flight performance. Do not multiply selected RPM, exhaust velocity, swept area and thrust endpoints into an invented operating point. The actual selected fans are only described as similar to this class.

## Smallest remaining decisions / bounded next step

1. **Geometry:** identify or mark the nine physical rotor centre planes and axes on the existing full-scale geometry, with final diameter/clearance. If proposing prototype-like inclinations, explicitly review the rear longitudinal shafts, front pair30° inward thrust and central vertical unit. Do not scale the rear JSON force points into fan centres. A dimensioned assembly or nine-row centre/axis/diameter table resolves this; a layout annotation can be prepared first without modifying CAD.
2. **Operating reference:** confirm the actual fan/motor revision or allow a stated provisional class model, then choose a documented operating condition per group and obtain its pressure/flow or thrust-versus-inflow characteristic. The catalogue ranges alone do not define powered boundaries at100km/h or across0–150km/h. Spin direction is needed if torque/swirl is included; no all-same-spin assumption.

No new question about fan count, nose, scale, canopy or lighting gaps is needed. No CFD, aircraft geometry changes, upload or Agent A handoff was performed. Existing XFOIL results remain isolated references.
