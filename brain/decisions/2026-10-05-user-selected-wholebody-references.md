# SUPERSEDED: WRONG SOURCE FILE, DO NOT APPLY

User correction (2026-10-05T21:05:14.903476+00:00): exterior_modified.3dm was the WRONG aircraft file. The authoritative input is blender/atlas_v1.stp. All earlier selections, placements, fit metrics and implementation instructions derived from the Rhino file are historical only and must not be applied to the STEP aircraft. E908/GOE5K/SC(2)-0403 selection is not a current recommendation. New findings require user review before any new Agent A handoff or implementation.

---

# User-selected whole-body geometric references

Timestamp: 2026-10-05T20:55:50.330659+00:00

The user explicitly selected the nearest regional references for Agent A to use in the next integration assessment: **E908 centrebody, Gottingen 5K body-wing transition, SC(2)-0403 outer wing**. Target remains the whole existing aircraft body and wings. This is a user selection of geometric references, not a fit or aerodynamic validation. Existing design, ducts, mounts and envelope remain unchanged.

Authoritative next-step instruction: `results/handoff/T20261005-wholebody-foil-screen/from_B/USER_SELECTED_REFERENCES.md`; machine-readable selection: `user_selected_references.json`. These contain exact UIUC URLs, files/hashes, station mapping and comparison transform. Existing diagnostic X stations: centrebody 0/200/400/500 mm; transition 600/750/900/1050; outer 1200/1350/1500/1650/1750/1850. This is not a panel-boundary or old fan-station prescription.

RMS/maximum upper-skin deviations remain E908 24.40/37.71 mm, Gottingen 5K 5.97/14.15 mm, SC(2)-0403 4.01/8.06 mm. Complete boundary identity/closure, full-contour fit and aerodynamic evidence are missing; existing transition comparisons show substantial discontinuities if profiles are switched abruptly. S0 remains PARTIAL and missing force/jet-angle fields stay null.

Agent A may use the selected references in separate comparison overlays/working copies and assess full integration without changing the aircraft. Any integration that requires reshaping, blending or other design changes must first be reported with quantified changes for user approval. No implementation, aircraft parameter deployment or physical test was started by this update.
