# SUPERSEDED: WRONG SOURCE FILE, DO NOT APPLY

User correction (2026-10-05T21:05:14.903476+00:00): exterior_modified.3dm was the WRONG aircraft file. The authoritative input is blender/atlas_v1.stp. All earlier selections, placements, fit metrics and implementation instructions derived from the Rhino file are historical only and must not be applied to the STEP aircraft. E908/GOE5K/SC(2)-0403 selection is not a current recommendation. New findings require user review before any new Agent A handoff or implementation.

---

# 2026-10-05 Whole-body lifting-surface library search

Timestamp: 2026-10-05T20:49:50.360970+00:00
Task: T20261005-wholebody-foil-screen; Agent B; S0 partial.

## User clarification and immutable constraint
The target is the entire existing aircraft body and wings acting as one continuous lifting surface. It is not a discrete fan jetfoil replacement. Original shape, ducts, mounting interfaces and envelope must stay unchanged. The region question has been answered; do not ask it again. The older three-station S0 manifest is not a sufficient definition of this new scope.

## Evidence and outcome
Source is blender/exterior_modified.3dm, Rhino 8 in mm, unchanged SHA-256 e08de19b2fe6b3c919440c5fc727cca2e20bcfe146b677337ced330bf0ab75c6. The wing, body_developed and Layer 01 SubDs remain separate. Their intended assembly relationship is not assumed. 14 span cuts (X=0 to1850 mm) cover centrebody, transition and outer wing. All source components in those cuts are open; no complete approved section contour was available. A conditional upper-wing/lower-body projection has missing intervals and multiple body branches. See topology, seam gaps and original section CSV.

A broad screen parsed 1,658 profiles from 1,665 DAT entries in the [UIUC archive](https://m-selig.ae.illinois.edu/ads/archives/coord_seligFmt.zip); seven exclusions are logged. Fixed endpoint chord, rigid normalization, uniform scale only; both directions/reflections tested. 301 chord samples per cut, 14 cuts. The best same-profile global upper-skin match, YS-915, still has 28.49 mm RMS / 82.60 mm maximum sampled difference. Regional references with the same provisional forward convention and no reflection: E908 centrebody 24.40/37.71 mm; Gottingen5K transition 5.97/14.15 mm; SC(2)-0403 outer 4.01/8.06 mm. These are geometric comparisons, not approved airfoils. YS/E908 are catalogued as hydrofoils; nearby MH120 is a propeller-tip profile. Similarity does not establish aircraft suitability.

Conditional two-sheet matching is substantially worse: best global NACA5-H-20 reflected, 107.58 mm RMS /390.53 mm maximum. It is a proxy, NOT full-contour certification. Abrupt switches among local profile choices create upper-branch differences up to52.88 mm and18.98 mm at example common chords. No loft, closure or blending was performed. No verified library replacement satisfies exact preservation. Enclosing the aircraft in a larger profile would change the envelope and is not authorized.

## Deliverables and next step
All files local: results/handoff/T20261005-wholebody-foil-screen/from_B/. Start with AGENT_A_BRIEF.md and REPORT.md. Includes raw source-coordinate sections, 48 candidate coordinate files with provenance, original public archive/hash, all rankings, local-only images, methods and uncertainty maps. Original aircraft file and existing log entries were preserved. No geometry uploaded.

Next useful work is evaluation of the unchanged custom lifting body after identifying how the existing sheets form the actual outer boundary and confirming operating scale/axes and speed/incidence/CG/fan boundary conditions. Full-aircraft3D treatment is required for lift, trim, stability and interference. [XFOIL](https://web.mit.edu/drela/Public/web/xfoil/xfoil_doc.txt) is only an isolated-section screen; no polars, installed forces or jet angles were inferred. No controller writes or physical tests.

Previous four-profile study remains historical. This update supersedes its discrete-jetfoil scope and pending-region question.


## User selection update (2026-10-05T20:55:50.330659+00:00)
The user selected E908, Gottingen 5K and SC(2)-0403 as regional geometric references for Agent A. See [decision](../decisions/2026-10-05-user-selected-wholebody-references.md). This updates the next step, not the measured fit or aerodynamic validation. S0 remains partial and design changes require explicit approval.
