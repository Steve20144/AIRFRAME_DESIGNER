# SUPERSEDED: WRONG SOURCE FILE, DO NOT APPLY

User correction (2026-10-05T21:05:14.903476+00:00): exterior_modified.3dm was the WRONG aircraft file. The authoritative input is blender/atlas_v1.stp. All earlier selections, placements, fit metrics and implementation instructions derived from the Rhino file are historical only and must not be applied to the STEP aircraft. E908/GOE5K/SC(2)-0403 selection is not a current recommendation. New findings require user review before any new Agent A handoff or implementation.

---

# Agent B: unchanged-design library fit screen

Created: 2026-10-05T20:33:30.810965+00:00
Task: T20261005-library-fit-screen
Status: S0 PARTIAL. No airfoil approved, no aircraft geometry changed.

## Source and identity
The user-selected new directory contains exactly one model, `blender/exterior_modified.3dm`, not a Blender file. Header identifies Rhino 8. rhino3dm 8.35.0 read it as data without executing embedded scripts. File SHA-256: e08de19b2fe6b3c919440c5fc727cca2e20bcfe146b677337ced330bf0ab75c6. Units are millimetres; document absolute tolerance is 0.001 mm (not an authorized design deviation). There are 577 objects, 572 Breps named JET_FOIL_SHOW-MODEL on engines, and five SubD objects. Actual object contents and generated geometry views establish an aircraft assembly with central body and lateral engine/wing shapes.

Two overlapping exterior SubDs are both visible: object 575 `body_developed`, UUID e9c62947-01df-4ddc-bfdb-8c48dd45a17e; and object 576 `wing`, UUID 13aef682-b831-4ea3-bbe6-8fd226f57d6c. Object 574 on `Layer 01` is a central-body surface. All five SubDs are open (`IsSolid=False`, control mesh not closed). No choice among exterior versions has been made. A user clarification is pending. This ambiguity prevents an approved foil substitution.

Raw CAD axes appear to be lateral X, longitudinal Y, vertical Z from the shapes; forward sign and transform into project FRD remain unconfirmed. Do not apply the older STEP coordinate transform automatically. Body_developed's conservative SubD bounding box is X [-1933.75,1933.75], Y [-3276.14,602.47], Z [3951.42,5321.11] mm. Wing bounds are X [-1931.42,1931.42], Y [-3276.14,602.47], Z [3954.31,5688.47] mm. These are bounding-box bounds, not closed-volume clearances. Their roughly 3.9 m span/length must not be silently equated to the small V3_30 prototype. Source identity, operating scale and station mapping need confirmation.

## Measurements and method
`geometry_inventory.py` reads immutable geometry, duplicates SubDs in memory and applies four subdivision levels for a control-net approximation of the limit surface. `sections.py` triangulates this mesh and intersects X=600,900,1200,1500 mm. These are diagnostic span cuts, NOT identified inner/middle/outer fan centres or chosen placement stations. `geometry.png` and `sections.png` show both candidate exterior surfaces. `objects.json` preserves object IDs, layers and bounds; `section_dimensions.json` preserves the measured cuts.

| CAD X, mm | body_developed projected Y extent, mm | wing projected Y extent, mm |
|---:|---:|---:|
|600|1433.5|1435.9|
|900|1168.8|1168.1|
|1200|1039.9|1051.4|
|1500|951.3|954.3|

The wing sections are single open curves, not closed upper/lower airfoil outlines. Body_developed cuts include duct openings and multiple disconnected branches. A projected chord or overall bounding box cannot establish a usable airfoil envelope. No closed-section containment, attachment clearance, fan clearance or full-span fit has been demonstrated.

Subdivision 4 versus 5 changes wing section Z by at most 0.132 mm at 2001 common Y samples per cut (excluding 1 mm at endpoints); see convergence.json. This checks tessellation sensitivity, not exact NURBS certification or a permitted fit tolerance.

## Traceable candidates and geometric screening
Coordinates are the unchanged downloaded UIUC files, with source URLs and hashes in candidate_sources.json:
- NACA 0012: symmetric 12% baseline.
- NACA 2412: modest-camber 12% baseline.
- Eppler E387: thinner, cambered low-Re comparator.
- Selig S1223: strongly cambered high-lift comparator.
These represent distinct geometry families, not a declared aerodynamic ranking. Public sources: https://m-selig.ae.illinois.edu/ads/coord_database.html and the direct coordinate URLs below.

`compare_sections.py` keeps each library profile unchanged, uniformly scales it to the existing wing-cut endpoint chord and rigidly aligns its endpoints. Both endpoint directions, profile sides and normal signs are checked because forward/skin convention is unresolved. For each station it retains the smallest RMS error against the existing skin at 199 points from 1% to 99% chord. No offsets, camber warping, nonuniform scaling or aircraft movement are permitted. This is a deliberately permissive single-skin mismatch test, not an accepted placement or volumetric fit.

| Candidate | t/c from coordinates | Required full thickness over tested chords, mm | Best-side RMS skin mismatch across cuts, mm | Largest point mismatch across those fits, mm |
|---|---:|---:|---:|---:|
|NACA 0012|12.003%|115.0-173.2|24.53-33.20|46.73|
|NACA 2412|12.000%|115.0-173.1|12.02-19.52|34.23|
|E387|9.070%|86.9-130.9|7.49-41.36|59.73|
|S1223|12.139%|116.3-175.2|22.98-29.66|43.69|

Result: none of these four reproduces the existing wing skin under the tested fixed-endpoint transformations. All are rejected as currently verified, unchanged-skin drop-in replacements. NACA 2412's smaller aggregate mismatch does NOT make it approved. There is no evidence that the other side fits ducts or structure. This is not an exhaustive proof that no library profile can work, nor a verdict on an alternative unconfirmed region of body_developed. If the user intends to preserve this exact outer skin, its existing sections are the reference geometry; replacing them with a different named profile would change the design.

## Aerodynamics and missing evidence
No polars, lift/drag numbers, installed forces or jet angles were produced. The official XFOIL primer describes isolated subsonic sections and lists massive separation and unsteady flow among limitations: https://web.mit.edu/drela/Public/web/xfoil/xfoil_doc.txt . Independent-section analysis would require confirmed chord, speed, air properties, transition/roughness and incidence. It cannot validate the assembled duct/fan/jetfoil, attachment of a deflected jet, interference, swirl or thrust retention.

The prior brain has one 3x36 N CFD point for a different specified foil configuration and a k~0.63 turning approximation. It is not evidence for these four profiles or this Rhino assembly. Current station geometry and operating scale are not yet mapped. No Reynolds range is invented, because even the operating velocity and scale are unresolved.

Project goals retained from HANDOFF: hover <=20 deg, ground rotation <=20 deg; for park -10, hover <=10; jet-angle build tolerance +/-2.5 deg with busiest fan below about 90%. These are downstream targets, not evidence that a candidate achieves them.

## Required next steps and Agent A contract
1. User identifies authoritative exterior layer/assembly and precise foil region. Original is preserved.
2. Identify fixed duct axes, attachment interfaces, leading/trailing boundaries and a closed allowed volume (or an explicitly allowed side/thickness of the skin). Confirm axes and model scale.
3. Repeat library comparison over actual station planes and full span, checking containment and interfaces without deforming aircraft or profile. If none fits, report none.
4. Only for geometrically admissible candidates, run documented independent-section analysis where applicable; obtain installed CFD with fan boundary conditions, mesh/domain convergence and per-station force vectors/jet angles.
5. Agent A may integrate only after an approved fit and suitable installed evidence. Current manifest uses null values for unavailable aero fields and remains S0 partial. No controller actions or physical tests are authorized.

## Sources
- Original Rhino file, objects.json and generated section/fit measurements.
- brain/HANDOFF.md; brain/experiments/2026-10-01-v3-30-step-import-and-tune.md; brain/experiments/2026-10-01-v3-30-foil-headroom-sweep.md.
- https://m-selig.ae.illinois.edu/ads/coord/n0012.dat
- https://m-selig.ae.illinois.edu/ads/coord/naca2412.dat
- https://m-selig.ae.illinois.edu/ads/coord/e387.dat
- https://m-selig.ae.illinois.edu/ads/coord/s1223.dat
- https://web.mit.edu/drela/Public/web/xfoil/xfoil_doc.txt


Payload: `results/handoff/T20261005-library-fit-screen/from_B/`.


## Later user scope clarification
The user clarified the target as the entire body and wings forming one lifting surface. The region question is resolved. See [whole-body follow-up](2026-10-05-wholebody-foil-screen.md) for 1,658-profile screening and current outcome. Previous discrete-jetfoil assumptions are not the current task.
