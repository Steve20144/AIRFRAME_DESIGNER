# Primary leading-edge coordinate audit: SC(2)-0402 and0403

2026-10-05T22:17:30.754170+00:00

Primary source: Charles D. Harris, NASA TP-2969 (1990), https://ntrs.nasa.gov/api/citations/19900007394/downloads/19900007394.pdf . TablesII/III are printed pages17/18 (PDF pages19/20). Relevant complete scans were visually inspected and retained.

The first eight x stations from0 through0.05 chord were manually transcribed for both upper and lower surfaces. All32 ordinates match the unchanged UIUC input coordinates exactly at published precision. In particular x/c=.002 gives y/c=±.00135 for0402 and±.00210 for0403; x/c=.005 gives±.00205/±.00320. NASA itself has no additional table stations between0 and.002. No missing-point or transcription correction is justified by this check. This is a leading-edge audit, not a claim of full-table verification.

The report text on printed page12 explains finite printed coordinate precision, a common reference line rather than the conventional LE-to-TE chord, and a design assumption of transition at3% chord and Re=10million for thickness below6%. Figure30 (printed page66) shows a parabolic radius/thickness trend; it does not supply an exact analytic nose contour or a higher-resolution point set for these two sections. These are design assumptions, not evidence of natural-transition accuracy in our low-Mach cases. Our5% trip was a sensitivity case, not a reproduction of NASA's3% design condition.

The published reference-line convention matters: the tabulated LE-to-mean-TE chord angles are−0.26069° for0402 and−0.14324° for0403. Thus equal XFOIL input alpha compares the published common reference line, not identical conventional chord incidence; at input alpha0 the conventional incidences are approximately+.26069°/+.14324°. Previous reports correctly labelled alpha as library-axis incidence. The earlier2.7% drag difference at equal input alpha must remain a single reference-line case, not an equal-chord-incidence or universal superiority claim.

Conclusion: **primary-coordinate provenance is resolved; numerical sensitivity is not caused by a demonstrated bad UIUC leading-edge transcription.** Existing tests establish panel/transition coupling. The original tabulation does not uniquely establish sub-grid curvature at the very thin nose, so sampling/spline representation can contribute, but cannot be proven the sole cause. Increasing panels samples the same spline and does not add original geometric information. No additional solver run, smoothing, invented radius, coordinate replacement or geometry change was warranted. Retain controlled-transition bounds and withhold natural-transition drag ranking where it is grid-sensitive.

Files: NASA_TP_2969.pdf; original scan images; leading_edge_comparison.csv; verification.json. The aircraft STEP remains unchanged. No3D analysis or handoff.
