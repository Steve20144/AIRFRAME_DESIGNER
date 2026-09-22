# Research: the upstream repository and the ATLAS_09B model

## Question

Where does the "more accurate physical drone model" live, and what is in it?

## Findings

- This checkout grew from **rasmushauschild/AIRFRAME_DESIGNER** (remote `upstream`, added 2026-09-21). `origin`
  (Steve20144/AIRFRAME_DESIGNER) is the user's remote and is behind this checkout. rasmushauschild/AIRFRAME_SIMULATOR
  is the older ancestor; its last commit (2026-09-17) only added the 13 kg placeholder `atlas_07c`.
- Upstream at commit 83ba06a (2026-09-19) is 46 commits past the common ancestor 01d3315: native PX4 ground
  sequences in a custom firmware module (`NLF_*`, `firmware/atlas/`), `geometry/cad.py` (places the PHASE_0_V4
  STEP, derives mass from 89 bodies), motor test, real-flight conversion, pilot handover, board-vs-SITL parameter
  sync, a Pixhawk marker in the 3D view. Only the model was taken.
- **ATLAS_09B** (`airframes/atlas_09b.json` here): 12.125 kg from CAD bodies, CG [0.009, 0, -0.006] (origin near the
  CG), inertia [0.61, 0.65, 1.07] with Ixz -0.116, tricycle stand (nose leg at x 0.77), no wing model, nominal
  rotor positions (M1 [-0.3, -0.37, 0.19] ... M10 [0.56, 0, -0.1]), nose fans canted +-30, 36 N fans, imported km
  0.01, hover 23, park -12, gains rate P 0.5 / 0.45, D 0.012, att P 3. Corrections applied here: see
  decisions/2026-09-21-atlas-09b-model-corrections.md.
- **ATLAS_OG** (`airframes/atlas_og.json`, from the Fusion design PHASE_0_V4 via MCP, 2026-09-18): weighed
  12.09 kg, CG [-0.078, 0, 0.123], measured fan stations (up to 10 cm from 09B's), 33.3 N fans, foil wing model,
  four-leg stand at +8, hover 26. The two models do not transfer results to each other.

## CAD integration (2026-09-21)

Upstream's CAD segment was ported: `geometry/cad.py`, `tests/test_cad.py`, mass resolve from items + bodies,
the `cad` field on the airframe, the three server routes, the CAD card in the Geometry tab and the body meshes in
the 3D view (drag a body to move its mass point). `airframes/cad/PHASE_0_V4.step` is the user's local STEP
(byte-identical to upstream's). ATLAS_09B carries the block with `from_items: true` (12.125 kg from 28 massed
bodies of 86). ATLAS_OG and ATLAS_OG_FLAT carry the same bodies with origin 0 (their structural frame is the
Fusion frame; the nose EDF centroids land within 5 mm of the OG rotor positions), `from_items: false` so the
weighed mass stays. Note: the eight foil-fan bodies sit at x -0.05 to -0.12 while both models place those rotors
at x -0.22 to -0.40, because the rotor position is the jet exit after the foil bend, not the fan; the nose fans
coincide. Lesson: `Airframe.save` before this port dropped the `cad` block silently (unknown keys are ignored).

## Implications

Pick one geometry as the truth for the aircraft that will fly, and reconcile fan positions against the CAD. Upstream's
native ground-sequence firmware is the alternative to this repo's simulator-side nose lift for HITL; not merged.

## Remaining uncertainties

Which rotor positions match the build; the reaction-torque coefficient (lessons/reaction-torque-km.md).
