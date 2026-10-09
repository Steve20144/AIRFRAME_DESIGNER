# ATLAS v1, nine EDF provisional placement

Timestamp: 2026-10-05T22:58:19.831466+00:00
Status: **PROPOSED NOT ACCEPTED**. User approved preparing this separate review proposal only, no implementation.

Review: [proposal and decisions](../../results/reviews/20261005-atlas-v1-step/nine-edf-provisional/REVIEW.md)
Coordinates and axes: [layout.json](../../results/reviews/20261005-atlas-v1-step/nine-edf-provisional/layout.json)
Images: overlay_plan.png, overlay_front.png, overlay_side.png, rear_anchor_detail.png in that folder.

Authoritative source blender/atlas_v1.stp, full 3.805 m span, nose +Y, canopy retained, intentional light openings preserved. SHA256 before and after: 72d01577ca0ff11af0075d33d3fdc2d1a39583c2752f6889443af13f1ded13a4. 38 STEP/profile/config files verified unchanged. No previous full placement proposal existed; prior powered-forward-flight notes are topology/reference inputs.

Rear physical rotor-reference centers are proposed from full-scale shell-C rim geometry, moved +150 mm in Y from rounded rim references. No scaling of small-model M1..M6 jetfoil exit force points. CAD mm: outer (±980,-711,4660), middle (±755,-847,4850), inner (±530,-977,5040), shaft force +Y and raw exhaust -Y. Front pair (±450,-400,5050), proposed inward 30 degrees from vertical; central (0,-100,5270), vertical. These are hypotheses, not accepted fan assembly locations or controller channels.

Schübeler reference DS-215-DIA HST / DSM10066-290: 195 mm is INSIDE casing. Actual external diameter, intake lip, full motor length and mounting envelope remain unknown from accessible official product page/flyer; manufacturer manual is required. No photo scaling. D200/220/250 x L100/200/300 mm cylinders are arbitrary sensitivity probes, not Schübeler dimensions or guaranteed conservative envelopes.

81 exact B-rep cylinder-to-shell checks. At D220 L200, minimum shell gaps: outer 50.43 mm, middle 27.31, inner 32.71, front pair 23.02, front center 38.50. 36 pair gaps, minimum 74.49 mm. Larger probes fail: D250 L300 intersects middle rear and front pair; central L300 fails at all tested diameters. Open shells and unmodelled hardware prevent installation certification. Unsuitable initial positions recorded, including a left-canopy interference missed by right-side-only inspection, both sides now checked.

**Front direct flow paths BLOCKED**: pair inlet axes meet canopy at ~359/360 mm, exhaust meets lower shell at ~465 mm; central inlet canopy at 204.8 mm, exhaust lower shell at 176.3 mm from center. Local cylinder clearance does not establish an inlet or outlet. Front rows remain spatial proposals only; no direct installation in unchanged STEP is accepted. Rear centerline rays show no intersections within 4 m, not a full-aperture or powered-flow validation.

Review decisions: retain rear 3+3 as conditional geometry candidate; review proposed front axes/locations with the flow obstruction explicitly unresolved. Obtain real dimensioned hardware envelope and established inlet/outlet routing before acceptance. Do not modify aircraft/canopy/light openings to force fit. No CFD, uploads, deployment, source/profile edits or Agent A handoff.
