# 2026-10-05 ATLAS v1 foil candidate v001 (HANDOFF S1 to S3)

Agent A, task T20261005-atlas-v1-foil-references. Payload `results/handoff/T20261005-atlas-v1-foil-references/from_A/v001/`
(gitignored; `fit_report.md` first). Scripts `scripts/atlas_v1_foil_candidate.py` (OCP) and
`scripts/blender/atlas_v1_build_blend.py` (Blender 5.2.2, headless).

- Source `blender/atlas_v1.stp` (full scale, span 3.805 m, three open shells: 0 canopy, 1 upper skin, 2 lower skin;
  CAD +X right, +Y nose, +Z up, origin far from the aircraft: Z 4.0 to 5.6 m). Never modified; hash checked each run.
- Agent B's regional references (E908 X 0..450, GOE 5K 550..1100, SC(2)-0402 1250..1880 mm) placed with B's fit
  (shell-1 section LE = max Y, TE = min Y) reproduce B's RMS exactly; the source is left/right symmetric to 1e-6 mm.
- **As full sections the thin foils do not fit the aircraft**: upper skin within 2 to 42 mm, but 45 to 54 % of the
  centre-body section and 87 to 91 % of the transition section are lost (rear fan nacelles ~400 mm deep, with
  inlet/exhaust openings), and the cabin is cut (67 % of the canopy section outside at X = 0). Body thickness is
  ~19 % of chord at the centre, up to ~40 % in the transition.
- Fitted chord lines are nose-down vs CAD horizontal: 0 at the centre, -3 at X 450, -5.3 to -6.2 in the transition,
  -5.8 to -3.4 outboard. Panel incidences / polars must state this reference.
- Lofting gotcha: OCP ThruSections fails between a sharp-TE section (GOE 5K) and an open-TE one (SC(2)-0402);
  fixed by joining the TE segment into the lower edge so every section has two edges.
- OCP gotcha: `xstep.cascade.unit` is process-global and also scales STEP *writing*; set it before reading or
  writing (a test failed only after another test had set it to M).
- `import_step` now loads open shells as massless surface bodies (`"surface": true`, volume 0), so surface-only
  STEPs such as atlas_v1.stp open in the app.

## v002, upper skin only (user's option 1, same day)

`scripts/atlas_v1_upper_skin_candidate.py`, payload `from_A/v002/`. Lower skin (all openings) and canopy kept; the
source upper skin has no openings. 115 stations (B's 15 + denser: the junction TE sweeps 650 mm aft in 100 mm of
span, so B's 450/550 stations alone gave a 126 mm deviation between them); smoothstep blends in B's gaps; one
least-squares C2 B-spline within 0.5 mm of the placed points. Upper-skin change max 42.6 mm (E908 at X 450),
<= 11.5 mm outboard of 550; seams to the kept parts max 1.8 mm; area +0.8 %.
- Surface gotchas: ThruSections through ~115 sections never finished (18 min); exact interpolation
  (GeomAPI_PointsToBSplineSurface.Interpolate) rippled 1-2 mm at ~10 mm wavelength (+3.9 % area); least squares
  (Init, tol 0.5 mm) fixed it. A linear profile blend creases the skin at both ends of the gap: use smoothstep.
- Ripple detector: second difference of Z along planes Y = const every 1 mm (source ~0.003 mm/mm^2).
- Open (user): accept the 1.8 mm seams or edit the kept edges; junction spanwise curvature still 7-14x the source.
