# 2026-10-07 Atlas v1: patents for sidewind (crosswind) stability of the anhedral wing, no rear fins

Source geometry: `blender/atlas_v1.stp` (Rhino 8, 2026-09-29). CAD frame x right, y AFT (nose at y -3.24 m), z up, mm.
CORRECTION (same day): a first version of this note read the STEP in the wrong frame (took z as length) and concluded
the tips were ahead of the CG. Wrong. Measured again in the right frame:

- Span 3.81 m, length 3.82 m (nose y -3.24, tail hub y +0.59), body 1.6 m tall plus legs down to z 0.
- Plan: the outer wing (|x| > 0.9 m) spans y -1.0 to -0.3: **0.4 to 1.1 m BEHIND the CG** (CAD y -1.37 for M001).
  The leading edge sweeps back (y -3.24 at the centre, -1.55 at |x| 0.5, -1.2 at 1.1, -1.0 at 1.9).
- Anhedral: the wing drops from z 5.1 (centre) to 4.0 (tips): the -31 / -37 deg outboard V of the strip model.

## Patents (closest first)
1. **US2406506A, Northrop, "All-wing airplane" (1946)**: no fin; wing tips drooped about 30 deg, swept back behind the
   CG, no toe-in. In sideslip the drooped tips give outward side forces whose couple, proportional to the yaw angle,
   restores the heading. Atlas v1 already has this arrangement (drooped tips 0.4 to 1.1 m behind the CG); the strip
   model gives Cn_beta +0.0017/deg (weathercock stable) from it.
2. **US2412646A, Northrop and Sears, "Tailless aircraft" (1946)**: sweepback 20 to 25 deg for directional stability and
   **differentially opened wing-tip rudders** (split drag rudders) for yaw control without a fin. This is the yaw
   effector adopted in the cruise-test controller (see the experiment note).
3. **US20170190436 (Ullman and Homer, granted 2018, active to 2037), "Distributed electric ducted fan wing"**: roll and
   yaw by differential power to EDFs along the span; crosswind compensation by the spanwise lift distribution, no
   control surfaces. The differential thrust of the six wing fans is the second yaw effector in the controller.
4. US20160009380A1 (Boeing split winglet, lower winglet 15 to 30 deg anhedral): passive nose-down tip moment in gusts.
5. US10814973 (Textron M-wing / gull wing): the V root section itself gives yaw stability without a tail.

## What the aerodynamics says (strip model, M001, 90 km/h; CFD quick library too noisy to use)
- Cl_beta **+0.0051/deg** (anhedral: a sideslip rolls the aircraft INTO the wind, destabilising), Cn_beta +0.0017/deg
  (stabilising), CY_beta -0.010/deg. Without roll control every six-axis release rolls past 90 deg within 4 s
  (time constant about 0.4 s: the roll inertia is only 18 kg m2 against 500 N m of roll moment per 10 deg of sideslip).
- Elevon roll authority (25 % chord flaps on transition + outer panels): about 670 N m per +-6 deg, 1670 per +-12.
  Enough, as long as the pitch trim does not eat the travel (trim -12.8 deg at M001, -19.4 deg at 92 kg / 90 km/h).
See brain/experiments/2026-10-07-atlas-v1-lateral-controller-and-masses.md for the fix and the numbers.
