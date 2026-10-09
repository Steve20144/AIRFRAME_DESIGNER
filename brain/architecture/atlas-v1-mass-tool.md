# ATLAS v1 mass tool (S4 of the handoff workflow)

Created 2026-10-05 by Agent A. Script: `scripts/atlas_v1_mass_tool.py`. Launch entry: `mass-tool-mac` (port 8099).

```
.venv/bin/python scripts/atlas_v1_mass_tool.py        # http://127.0.0.1:8099
```

## What it is

A local page (three.js from jsdelivr, no app dependency) that shows the candidate v002 geometry from the Blender
working copy's meshes (`from_A/v002/meshes_mm.npz`: new upper skin, kept lower skin, kept canopy, CAD frame, mm,
served in metres) and lets the user click mass points onto it:

- click a surface: new point; name and grams in the list; per-surface visibility toggles
- move (re-pick) or delete a point; "mirror the next point" drops a second point at x -> -x
- live total mass and point-mass CG (CAD frame) with a CG marker in the view; inertia is S5's job, not here
- Save writes `from_A/v002/masses.json` (atomic write); Reload re-reads it; the page warns on leaving with unsaved edits

## BOM parts palette (added 2026-10-05, user request)

The side panel lists the airborne lines of the sourcing BOM (`--bom`, default `~/Downloads/ATLAS_Sourcing_BOM_7.xlsx`,
sheet `BOM`, re-read on every page load; a copy of the parsed lines goes to `from_A/v002/bom_airborne.json` with the
xlsx SHA-256). Airborne = numeric unit weight (the sheet marks ground equipment with the word `ground`) and amount
above zero; ALTERNATE lines carry amount 0 and drop out. 73 lines, 149.61 kg, equal to the sheet's own total.
Select a part, then every click places one instance: name `<short part name> #n`, grams = the unit weight (editable
per click; `all` puts a whole line such as 33 ft of cable on one point). The palette shows placed / quantity per
line (green when complete, amber when over). A click with no part selected places a free point as before.
`masses.json` items from a part carry `bom_row`, `bom_name`, `subsystem`.

## masses.json

```
{"schema": 1, "candidate": "<npz path>", "frames": {...}, "updated": iso,
 "items": [{"name", "grams", "pos_cad_m", "pos_frd_m", "picked_on_body", "timestamp"}]}
```

- `pos_cad_m`: STEP frame (x right, y AFT, z up: the nose is CAD -Y, user 2026-10-05), metres, STEP origin.
- `pos_frd_m` = R @ pos_cad_m with R = [[0,-1,0],[-1,0,0],[0,0,-1]] and a **zero translation** (declared choice). The S5 CG step and the airframe builder subtract the CG; nothing is CG-relative here.

## Decisions and limits

- Geometry is candidate v002 (the user's option 1), not the untouched source: the point is where masses sit on the
  design that will be built. Switch with `--npz` if another candidate is chosen (same npz layout).
- Points are on the skin surface where the click lands. For masses inside the body (battery, pilot), place the
  point on the skin above or below and note it in the name, or edit `pos_cad_m` in the JSON; the tool does not
  offset into the volume.
- The page serves itself from the script, so a code change needs a server restart (preview_stop / start).
- Navigation (fixed 2026-10-05 after the user could not rotate): the side panel overflowed at small pane heights so
  the page itself scrolled under the drag. Now `html` is overflow hidden, the panel fits the viewport (palette and
  placed list share the height), OrbitControls has damping, zoom to cursor, shift+drag pans, view presets (Iso,
  Top, Front, Side, Below), double click recentres the orbit on the clicked spot. A click's add is deferred 260 ms
  and cancelled by a double click (PointerEvent.detail is always 0, so the click count cannot be read there).
- Lesson: rebuilding the list DOM on an input's change event destroys the field that Tab moves to; markers and
  list are rendered separately for that reason. CSS2D labels must be removed from the DOM explicitly before
  `group.clear()`, or they linger.
