# Decision: Corrections applied to the imported ATLAS_09B model

Date: 2026-09-21
Status: accepted (each item reversible, all recorded in the file's notes)

## Context

`airframes/atlas_09b.json` was pulled from the upstream repository (rasmushauschild/AIRFRAME_DESIGNER, commit
83ba06a) as the newer physical model: CAD-body mass properties, tricycle stand, nominal rotor positions, canted
nose fans, km 0.01, hover 23, park -12, upstream firmware parameters. As imported it crashed every landing in the
Stabilized lab sequence (experiments/2026-09-21-atlas-09b-sequence-tuning.md).

## Decision

1. Remove `NLF_*` (upstream's custom firmware module) and `PWM_*_FUNC*` (this export sets output functions).
2. Add `design.nose_lift` / `design.nose_lower` (motors M9 M10), which the model lacked.
3. `THR_MDL_FAC 1.0`, `MPC_THR_HOVER 0.40`: the fans are quadratic.
4. Rotor M7's axis set equal to its mirror M8 (imported: 30 vs 40 deg).
5. `km 0.002` on all fans (imported 0.01), hover pitch 24 (imported 23).
6. Parked pitch +4 (imported -12), stand re-solved under the same hard points.
7. Gains: roll rate P 0.6, pitch rate P 0.45, rate I 0.3, D 0.012, roll att P 3, pitch att P 5.

## Reasoning

Items 1 to 4 are mismatches with this checkout or evident defects. Item 5 rests on the allocator analysis: with
km 0.01 the pseudo-inverse hover mix needs negative thrust on three fans; with 0.004 it pins a foil fan at idle and
PX4 has no nose-up authority (hovers 1.7 deg nose-down, drifts 13 m); with 0.002 at hover 24 pitch authority is
2.7 Nm each way. 0.002 is also the ATLAS_OG file value. Item 6: the nose fans have 5 percent moment margin over
gravity about the rear feet at -12 and the lift stalls; 19 percent at +4.

## Alternatives Considered

- Keep km 0.01 and tune around it: no gain set flew the sequence.
- Alternate spin directions: would give +0.5 Nm yaw authority instead of +0.3, but the aircraft is built with all
  fans clockwise.

## Consequences

The model flies the full sequence (hover 0.02 to 0.2 deg pitch, 0.3 roll, touchdown 0.46 m/s, nose down 9.7 s)
with a residual 0.25 deg roll bias and 0.2 to 0.3 m/s drift from yaw saturation. **The reaction-torque coefficient
must be measured on a thrust stand before any of this is trusted on hardware.** The `cad` block is kept but not
rendered here (no STEP viewer).
