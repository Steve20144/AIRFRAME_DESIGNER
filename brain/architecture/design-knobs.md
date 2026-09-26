# Design knobs (branch atlas-v5-design, 2026-09-26)

Named geometry values for the next ATLAS version: jetfoil angles, front jets, battery (CG). Each knob moves the
rotors and the CAD parts linked to them, so the physics, the 3D view and the CG stay in step.
Code: `airframe_designer/geometry/knobs.py`; tests `tests/test_knobs.py`.

## Where it lives

- Geometry tab, **Design knobs** card: *Set up knobs* builds the defaults; each row has a value, a sweep range and
  **→ sweep** (adds `knobs.<name>` to the Tuning tab's sweep).
- *Selected parts → Link*: CAD parts picked in the 3D view (shift-click) are tied to a rotor (they turn and move
  with it), set as the battery (moved by the CG knobs), or unlinked.
- API: `GET /api/knobs`, `POST /api/knobs/setup`, `POST /api/knobs/set {values}`, `POST /api/knobs/link {bodies, target}`.
- Paths: `knobs.foil_1_deg` etc. work in studies, the Tuning sweep (a name with a dot or bracket is a path, a bare
  name a PX4 parameter) and the MCP tools.

## Defaults (ATLAS layout)

- `foil_1_deg` .. `foil_4_deg`: one per mirror pair of foil fans (outer first), value = thrust angle from vertical
  (ATLAS_09B: 25/30/35/40). The foil surface is NOT turned: a new angle is a new foil shape.
- `front_cant_deg` (sideways, magnitude, per-rotor signs kept) and `front_tilt_deg` (fore/aft) on the nose fans.
  The nose fans' CAD bodies (nearest within 0.6 fan diameters) are linked.
- `cg_dx`, `cg_dz`: move the battery bodies (name contains BATT, mass > 0) by metres; with no CAD bodies they shift
  `mass.cg` from `base_cg`.

## How it works

- Rotor knobs read their value off the rotors (no stored value), so hand edits and knobs never disagree.
- `design.cad_links[rotor] = {bodies, pos0, axis0}`; on every `resolve_mass` `update_poses` turns each linked body
  by the rotation axis0 -> axis about pos0 and carries it to the rotor's position; `CadBody.rot/pivot/shift` hold
  the pose (derived, not edited). Inertia is rotated too. The 3D view reads `pose_pos` and `rot`.

## Checked

2-trial sweep of `knobs.front_cant_deg` on stab_lab (ATLAS_09B, seed 1): 0 deg flew (score 101.7), 30 deg failed
(700), agreeing with the 2026-09-17 bracket study. A check of the chain, not a result.

## Open

- V3 imported 2026-09-26 ([atlas-v3-small](atlas-v3-small.md)); a front-cant sign of 0 keeps a centre nose fan upright.
