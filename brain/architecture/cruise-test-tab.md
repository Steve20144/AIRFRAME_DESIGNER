# Cruise Test tab (forward-flight stability of the lifting body)

Added 2026-10-05 by Agent A on the user's instruction: "forget the EDFs, keep them as weight; create a cruising
scenario to check the stability of the foil from XFOIL; new tab Cruise Test".

## What it does

`airframe_designer/analysis/cruise.py`, endpoint `POST /api/cruise/test` (app.py), tab `#tab-cruise` (index.html,
app.js `renderCruise`, small SVG charts `svgChart`). Runs on the loaded airframe, no PX4, no fan thrust vectoring:

1. **Alpha sweep** at one airspeed: whole-aircraft CL, CD, CM (about the CG, nose-up positive) from the strip-theory
   wing panels with section polars plus the body drag box. Trim = CM crossing zero; lift there vs weight; the speed
   that would carry the weight at that trim; dCM/dalpha, dCM/dCL, static margin (fraction of the mean chord, positive
   = stable), neutral point (x forward: behind the CG when stable); per-panel alpha and stall flags.
2. **Free flight**: the rigid body released level at the trim, ideal thrust = trim drag along x through the CG, a
   pitch-rate kick (default 5 deg/s), integrated with the project's RigidBody for N seconds with no controller.
   Verdict from the pitch amplitude of the second half vs the first (damped / lightly damped / neutral / growing),
   or tumbled / hit the ground.
3. **Sections**: each section's polar as the app built it (NeuralFoil, since xfoil is not on this Mac) at the
   Reynolds numbers of Agent B's XFOIL 6.99 points (`results/reviews/20261005-atlas-v1-step/xfoil-screen-699/
   polar_points.csv`, natural transition), with those points overlaid and the RMS CL difference.

## Replay in the 3D view

"Play in 3D" (added the same evening on the user's request) replays the free flight in the app's 3D view: the free
flight records position, quaternion and per-panel forces every 50 ms; app.js interpolates them in real time at
the chosen rate (0.1 to 1x), feeds `scene.updateState` directly (the live simulator's state messages are ignored
while the replay is active), switches the camera to Follow, draws the panel lift arrows, and restores the live
view 1.5 s after the last frame. The release point is drawn 4 m above the grid.

## Aero engine decision

The user offered OpenFOAM. Not chosen for this tab: a 3D CFD case needs a watertight surface, a mesh and hours per
operating point, which does not fit a sweep-and-release stability check that must run in seconds in the app. The
tab uses the section polars (the XFOIL-class data the user asked for) in the existing strip-theory solver; XFOIL
itself is used when installed (`polar_source: auto`), NeuralFoil otherwise (installed in `.venv` on 2026-10-05).
NeuralFoil vs Agent B's XFOIL 6.99 points: RMS CL difference 0.013 (E908), 0.015 (GOE 5K), 0.008 (SC(2)-0402).
OpenFOAM stays the right tool for a later validation of a single trimmed case (installed flow, nacelle inlets).

## Model behind it (ATLAS v1 M001, `scripts/atlas_v1_from_masses.py`), nose = CAD -Y

- Frame: FRD = R (CAD - (0, 0, 5.0 m)), R = [[0,-1,0],[-1,0,0],[0,0,-1]]. The nose is the long slender end (CAD
  y = -3.24 m, FRD x = +3.24); the user corrected this on 2026-10-05 after seeing the replay fly backwards. Agent B's
  "nose +Y" and its LE-at-max-Y section fit are reversed (reported in HANDOFF.md).
- Six single-side panels (new `Wing.side`, `visual: false`): body E908 (0.51 m per side, chord 3.83 -> 1.72, sweep
  73.5, dihedral -11), transition GOE 5K (0.70 m, 1.72 -> 1.20, sweep 30, dihedral -31), outer SC(2)-0402 (1.00 m,
  1.20 -> 0.70, sweep 14, dihedral -37); incidence 0; `aspect_ratio` 2.13 on every panel (the planform's: a panel
  treated as an isolated wing of AR 0.7 got an absurd induced angle). Area 6.77 m2, mean chord 1.78 m. Body drag
  box 0.1 / 0.6 / 0.8 N per (m/s)^2.
- Trim: the sections are cambered (cm about -0.13), so CM is negative everywhere: no natural trim. The test trims
  at the lift-equals-weight alpha with an ideal pitching moment that scales with dynamic pressure (an elevon or
  reflex), reports that moment and the CG shift that would trim instead (and the margin left after it).
- Free flight: pitch-plane-only (roll, yaw and sideslip zeroed each step: the foil's own longitudinal behaviour) and
  all six axes (no roll control). The polar wings' warm-started induced-angle iteration is settled before release
  and the 0.5 % lookup-reuse band is bypassed every step (it hid slow drifts). Default kick 1 deg/s: with 5 deg/s
  the pitch-plane flight departs slowly after about 15 s even at 37 % static margin (a weakly damped speed/path
  mode with constant thrust; to be studied with the attitude loop in the loop, not a static-stability verdict).
- Limits: strip theory (no spanwise coupling, no fuselage/canopy lift, no fan inflow), polars valid only inside each
  table's alpha window (GOE 5K tops out at 8.5 deg on NeuralFoil).

## Elevons (added 2026-10-05 on the user's request)

`Wing.elevon` = {chord_fraction, max_deg, span_from, span_to, pitch_gain, roll_gain, deflection_deg}: a plain
trailing-edge flap on a panel, thin-airfoil theory in `WingSet` (`flap_tau`, `flap_cm`, `delta` per strip,
`set_elevons`, `set_controls(pitch, roll)`): the section behaves at alpha + tau * delta (tau 0.61 for a 25 % flap)
and gets dcm/ddelta -0.65 per rad about the quarter chord, plus a small drag rise. ATLAS v1 M001 carries them
on the transition and outer panels (25 % chord, +-25 deg). `cruise.trim_elevons` solves alpha and the deflection
for lift = weight and zero moment (Newton; the force evaluations are "settled": lookup cache bypassed and the
induced-angle iteration converged, otherwise finite differences are garbage); the free flights hold that
deflection, no controller. Not yet drawn in the 3D view, not yet driven by PX4.

## Results (2026-10-05, 46 kg, kick 1 deg/s, elevons trimmed)

| Case | alpha | elevon | L/D | static margin | pitch plane | six axes |
|---|---|---|---|---|---|---|
| 60 km/h, CG as placed | 11.6 deg | -12.9 deg (up) | 5.0 | 8 % | lightly damped | rolls off |
| 90 km/h, CG as placed | 5.8 deg | -12.8 deg | 3.3 | 1 % | stable | rolls off |
| 90 km/h, CG +0.6 m fwd | 9.4 deg | -21.5 deg | 2.3 | 34 % | stable | rolls off |

Up-elevon kills lift, so the body flies 4 to 8 deg steeper than without, at the edge of the GOE 5K polar's valid
range at 60 km/h, and L/D halves with the CG forward: the cambered sections and the nose-heavy CG cost a lot of
trim drag. Reflex in the sections or a less nose-heavy CG would cut the elevon angle. The six-axis flights still
roll off (no roll control yet; the elevons have roll_gain 1 but nothing commands them).

## Results before the elevons (2026-10-05, 46 kg, kick 1 deg/s, ideal trim moment)

| Case | alpha for L = W | L/D | static margin | trim moment | pitch plane | six axes |
|---|---|---|---|---|---|---|
| 60 km/h, CG as placed | 6.5 deg | 5.8 | -2 % (NP at the CG) | 196 N m nose-up | departs | rolls off |
| 90 km/h, CG as placed | 1.0 deg | 4.9 | +3 % | 418 N m | departs slowly | rolls off |
| 90 km/h, CG +0.6 m forward | 1.0 deg | 4.9 | +37 % | 689 N m | stable | rolls off |

The neutral point sits about 1.1 to 1.4 m behind the datum (1.9 to 2.1 m behind the nose tip), within 0.25 m of
the clicked-parts CG: the margin is decided by where the flight packs go. Every six-axis flight rolls off without
roll control: the anhedral (-31 / -37 deg outboard) gives negative effective dihedral; a controller or dihedral is
needed before judging lateral behaviour.

## First result before the flip (superseded)

With the nose taken as CAD +Y the body looked statically unstable (NP 1.0 m ahead of the CG). That was the frame
error, not the sections.
