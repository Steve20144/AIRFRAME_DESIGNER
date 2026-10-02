# ATLAS V3 from the live Fusion design (Fusion MCP, 2026-09-29)

Fusion 360 ships a local MCP server at `http://127.0.0.1:27182/mcp` (JSON-RPC over HTTP; needs the `MCP-Session-Id`
header from `initialize` and a `notifications/initialized` before any call). It is NOT registered with Claude Code;
`scripts/fusion_mcp.py` talks to it directly (`tools`, `call <tool> '<json>'`). To register it for native tool use:
`claude mcp add --transport http fusion http://127.0.0.1:27182/mcp` (then restart the session).

Tools: `fusion_mcp_read` (documents, projects, API docs, screenshot), `fusion_mcp_execute` (`featureType: "script"`
runs a Python `run(context)` in Fusion; pass `readOnly: true` for inspection; also document open), `fusion_mcp_update`
(undo/redo). Fusion API internal units are cm.

## Pipeline (all in `results/fusion_v3/`, gitignored, regenerate with the scripts)

1. `read_tree.py` via execute: every occurrence path, bodies, volume, centroid, bbox -> `tree.json`.
2. `export.py`: STEP export of the open design to `airframes/cad/SMALL_SCALE_V3_v34.step` + fan/battery matrices
   (`transforms.json`). The XFLY component's local +Y is its thrust axis.
3. `foil_rays.py`: `root.findBRepUsingRay` straight down through each foil fan's jet plane every 2.5 mm, keeping hits
   on JET_FOIL bodies -> wall profile; exit angle = slope of the last 0.5 cm before the trailing edge.
4. `scripts/atlas_v3_from_fusion.py` -> `airframes/atlas_v3_v34.json`: masses only from `_M<g>g` labels (deepest
   labelled occurrence owns its bodies; mass spread by volume over STEP bodies matched by centroid+volume; unlabelled
   parts and unlabelled "(Mirror)" duplicates weigh 0); foil rotors on the jet EXIT line at the trailing edge (the
   fan draws air from rest, so the net force is along the outgoing jet); nose fans from the transforms; PX4 gains,
   legs, fan model from `atlas_v3_small_graded.json`; knobs re-set up on the new bodies.

Frames: CAD x right, y up, z aft (the Fusion design axes); FRD = (-z, x, 0.45 - y) m.

## v34 facts

- 11.830 kg labelled; CG FRD (-0.045, -0.004, +0.078) as drawn. Foil fans at FRD y ±0.208/0.310/0.413 (the older
  STEP had ±0.20/0.28/0.36). Foil exit 69.0 / 59.3 / 49.4 deg (inner/middle/outer): already graded.
- Six 5200 mAh packs: front pair Fusion Z -150, rear block of four Z +150 (X ±30, ±85), Y 435 mm.
- H-FLOW__M15g in the avionics bay at FRD (-0.015, 0, 0.05), flat to the structure.

Flow sensor model: [sensors/flow.py](../../airframe_designer/sensors/flow.py) (`design.flow_sensor`), see the
[experiment](../experiments/2026-09-29-v3-v34-fusion-cg-hflow.md).
