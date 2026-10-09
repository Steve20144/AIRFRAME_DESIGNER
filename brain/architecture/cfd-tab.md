# CFD tab (OpenFOAM attitude library on the clean foil)

Added 2026-10-06 by Agent A on the user's request: "an engine inside Airframe Designer, in its own tab, that uses
OpenFOAM to examine the foil surfaces aerodynamically; I want to see if it is stable in cruise, how it behaves when
it moves in 3D, and to move it in 3D and have it simulate the airflow as in normal flight". Answers the user gave
to the plain-language questions: stability = "returns on its own" (hands-off); workflow = library of attitudes
computed once + live turning; air sees the foil only (no fans, no jets); display = pressure colours on the body;
speed 60 km/h; attitude range "calm flight" (alpha -6..12, beta 0..10); install OpenFOAM via Homebrew; start with
the current shape (blender/atlas_v1.stp), the modified .blend comes later.

## Why a library, not live CFD

One steady RANS solution of the 3.7 m body takes minutes (quick mesh) to tens of minutes (fine) on the M5. The tab
therefore computes a grid of attitudes in the background and interpolates between them while the user drags the
aircraft; the pressure field, forces and moments shown come from the same interpolation weights.

## Pieces

- `airframe_designer/cfd/geometry.py`: STEP / STL / OBJ / .blend -> one watertight STL in FRD metres
  (`results/cfd/<name>/surface/`). The ATLAS v1 STEP is three **open** shells with gaps up to 5 cm; shell 1 is an
  interior deck. Closure: generalised winding number (libigl `fast_winding_number`) on a 10 mm voxel grid
  (27 M points, 5 s), marching cubes at 0.5, shells oriented by trying every sign combination and keeping the
  closure whose surface lies closest to the input skins (a flipped skin either cancels the body or adds a bubble far
  from any input); interior shells (>80 % of vertices inside the others' closure) are dropped automatically.
  Then Taubin smoothing and pymeshlab quadric decimation (keeps it watertight; trimesh/fast_simplification at
  agg>=5 did not) to 300 k triangles (CFD) and 40 k (browser). Result for atlas_v1: planform 6.57 m2, span 3.71 m,
  length 3.74 m, mean chord 1.77 m, volume 1.90 m3, 18 s. Blender's voxel remesh was tried first and rejected: on
  open skins it builds a thin double shell (318 bodies, 0.01 m3), and the import scale must be baked into the
  vertices before remeshing or the voxel size is in mm and Blender runs out of memory (exit 137).
  .blend input: `blender_tools.py export` runs inside Blender headless (5.2.2 works headless on this Mac), exports
  the mesh objects (optionally filtered by name substring) to STL, then the same closure.
- `airframe_designer/cfd/case.py`: OpenFOAM case writer and runner. Launcher `openfoam2412` (Homebrew cask
  gerlero/openfoam/openfoam@2412, a disk image mounted at /Volumes/OpenFOAM-v2412; deprecated upstream, disabled
  2027-01-04, so plan to move to a newer cask). Domain box 16 L x 12 L x 12 L, flow toward -x (aircraft fixed, air
  comes from the nose side), one `farfield` patch with `freestreamVelocity` / `freestreamPressure` so any alpha and
  beta is just a different far-field vector; `foil` wall from snappyHexMesh (castellated + snap, layers only at
  "fine"); simpleFoam, kOmegaSST, wall functions, SIMPLEC, bounded linearUpwindV. Function objects: `forces`
  (every iteration, CofR at the FRD origin) and `surfaces` in `raw` format (wall pressure per face centre). The
  mesh is built **once per library** and shared by every attitude (only 0/ changes), so the wall face centres are
  identical across cases and one nearest-face map carries Cp onto the display mesh.
  Quality levels: quick (levels 4-5, ~0.31 M cells, 29 s mesh on 8 cores), standard (5-6), fine (6-7 + 3 layers).
- `airframe_designer/cfd/library.py`: `Library` (grid, background runner thread, status, interpolation, stability
  verdicts), `Manager` (what the server holds). Symmetry: only beta >= 0 is computed; negative beta mirrors
  Fy, Mx, Mz and reads Cp at the y-mirrored vertex. Interpolation: Delaunay barycentric weights on the (alpha, beta)
  points (nearest outside the hull, 1-D when only one line of points exists). Stability: straight-line fits of
  Cm(alpha), CL(alpha) at beta 0 -> Cm_alpha, static margin -Cm_alpha/CL_alpha (only when CL grows with alpha),
  neutral point x_cg - SM c, trim (Cm = 0 crossing) with lift vs weight and the speed that would carry the weight,
  alpha for L = W and the residual pitching moment there; Cn_beta, Cl_beta at the alpha nearest the trim. Verdict
  text in plain words (stable / unstable / no trim / weathercock / dihedral effect). Headless:
  `python -m airframe_designer.cfd.library atlas_v1 --quality standard --alphas ... --betas ...` writes the same
  files the tab reads (the server rescans result.json files when idle).
- Server: `/api/cfd/status|surface/prepare|library/select|library/start|library/stop|library/run_one|mesh|query|
  stability` in app.py. `query` returns the Cp field as base64 float32 per display vertex.
- UI: `ui/cfd_view.js` (three.js: body coloured by Cp, red pushes / blue pulls, aircraft turned by alpha about the
  body y axis and beta about the vertical while the wind arrows stay fixed; force arrows lift/drag/side/weight at
  the CG and a pitching-moment arc; "Turn aircraft by dragging" mode; camera presets), tab `#tab-cfd` in index.html,
  `cfd*` functions at the end of app.js. Launch config `app-cfd-mac` (port 8083, no PX4) for working on it.

## The moving air (added the same evening, user request)

- Each case also samples U and p on a regular point grid around the body (`flowCloud`, a `sets`/`cloud` function
  object with `interpolationScheme cellPoint`, raw format, written at the final iteration; grid from
  `case.flow_grid`: box from 1.0 L downstream to 0.5 L upstream, y symmetric with an odd point count so the mirror
  is an index flip; spacing quick 0.14 m, standard 0.10 m (95 x 57 x 40 = 217 k points, 3.5 MB float32), fine 0.08).
  Points inside the body are NaN. `flow.npz` per case; `Library.flow()` interpolates with the same weights
  (mirrored cases flip the y index and the v component); `GET /api/cfd/flow` returns a JSON header line plus the
  raw float32 arrays (the browser copies past the header before making typed arrays: alignment).
- The library reruns a case whose `flow.npz` is missing (the two standard cases computed before this existed).
- In the view (`cfd_view.js`): air particles (default 14 k, midpoint advection through the trilinear field,
  respawned when they leave the box or enter the skin) and optional "air threads" (fat dashed lines, `Line2`
  from three/addons/lines, vendored under `ui/vendor/three/lines/`; the dash distance is time of flight x V so the
  dashes run fast where the air is fast). Seeding: smoke sheets (horizontal through the CG + vertical on the
  centreline), one sheet, or the whole volume. Colour: the skin palette on a narrower Cp range (default 0.3 x the
  skin range, since the air away from the body sits at Cp ~ 0; the user could not see any colour at the full range).
  The user tried threads and preferred the particles ("more and with colour").
- Live progress: `Library.live_progress()` reads the case directory of the attitude being computed (works for the
  headless CLI too): iteration / total from controlDict, lift and drag history from force.dat; the tab shows a
  second bar and a small chart, polls every 4 s while anything runs.

## Free flight on the CFD model (`airframe_designer/cfd/flight.py`, same evening, user request)

User: "test crosswinds, see it move in the 3D sim with this foil, start with a pitch of -8, run automatically when
the CFD finishes and play it". `cfd_free_flight(af, lib, pitch0_deg, crosswind_ms, headwind_ms, ...)`: a RigidBody
subclass whose wing and body aerodynamics are the library table (LinearND interpolation in alpha, beta with
mirroring, forces scaled by (V/V_lib)^2, moments moved from the FRD origin to the CG), plus rate damping measured
once from the strip-theory wings by central differences at the release state (dM/d rates, scaled with V), an
ideal thrust through the CG equal to the CFD drag at release, NED wind (`rb.wind_ned`; crosswind "from the right"
= wind toward -y with the aircraft heading north). Beyond the computed alpha/beta range the table is held at its
edge plus a rough flat-plate increment through the planform centre (so a departed aircraft does not fly on an edge
value forever); the result reports the fraction of steps outside the range. Replay reuses the Cruise Test replay
(`cruiseReplay` in app.js, main 3D view, follow camera). Verdict via `analysis.cruise._classify` plus max roll,
heading change, lateral drift, altitude change.
UI: "Free flight on the CFD model" in the CFD tab, `POST /api/cfd/flight`, and an **auto-run** checkbox: when the
selected library has every planned attitude and nothing is computing, the tab runs the flight once and plays it.
First try on the quick library (7 attitudes, alpha -3..6): meaningless (80 % of the steps outside the range); the
standard library was extended to alphas -12..12 (27 cases, restarted 20:05, about 5.5 h) so a -8 deg release is
inside the table. Expect the clean foil to sink at 60 km/h (lift ~ 120-200 N vs 451 N weight).

### Long flights with an ideal pilot and a wind programme (user request, same evening)

User: "study it in flight under realistic conditions as if the motors were there, for a long horizon, choosing side
and head winds each time". `flight.py` gained `IdealPilot` (holds altitude through pitch, wings level, heading
through yaw, airspeed through the fans' thrust: gains from the inertia tensor at 0.5 Hz / damping 0.8, pitch
target within +-10 deg of the trim pitch found by bisection on the table for lift = weight, control moments
bounded, default 300 / 400 / 150 N m roll / pitch / yaw, thrust bounded by the airframe's fans (9 x 215 N) times a
fraction), `WindSchedule` (events {t, head, cross, up, ramp} in m/s, NED, plus low-pass turbulence of a given RMS,
fixed seed) and per-sample records of wind, thrust, pitch target and control moments. Verdict for a piloted
flight: "held / not held" with the altitude deviation and the share of time each control sat at its limit. The
server switches to dt 0.004 above 120 s. UI: pilot select, thrust fraction, turbulence, wind programme textarea
(one event per line). Test at 90 km/h on the quick table: held within 17 m with 5 m/s cross and head gusts.

## Background work panel (user request: "one table of what runs, pause and stop, all in one place")

The library always computes in a **worker process** (this module's CLI with `--worker`, spawned by the app with
`start_new_session`, `runner.json` {pid, cmd, started_at} in the library dir, `runner.log`). `Library.start()` and
`run_one()` spawn it; `start_thread()` / `run_one_thread()` are what the worker itself runs. Pause = SIGSTOP and
resume = SIGCONT on the whole process tree (pgrep -P walk: python, mpirun, solver ranks); stop = SIGCONT + SIGTERM
(the worker's handler sets the stop flag and kills the solver; the half-done case directory is removed so it is
redone later). `Manager.jobs()` lists the surface preparation and every library worker found by `*/lib_*/runner.json`
with state (ps STAT letter: T = paused), progress (done / total, ETA, current attitude and iteration) and controls;
`POST /api/cfd/jobs/action {id, action}`. The panel sits at the top of the CFD tab and polls every 4 s while a job
exists. The worker started by hand at 20:05 (pid 66495) was registered by writing its runner.json.

## HPC path: the library on the AUTH Aristotelis cluster (`airframe_designer/cfd/hpc.py`, user request)

Docs (hpc.it.auth.gr): OpenFOAM v2506 (`module load gcc/14.2.0 openmpi/5.0.5 openfoam/2506`, `srun <solver>
-parallel`), ANSYS Fluent 2026R1 also present (licence pool, first 4 cores free), partition `batch` 17 nodes x 20
cores x 128 GB, 7 days; `rome` 17 x 128 cores, 2 days; login/transfer host aristotle.it.auth.gr; accounts for AUTH
members (2FA, myaccount.auth.gr) and "collaborators of AUTH members". Chosen: OpenFOAM on the cluster, same cases.
`export_bundle(surface, batch, alphas, betas, speed, quality, ...)` -> results/cfd/<surface>/hpc/<batch>/ +
<batch>.tar.gz (16 MB for 27 cases): geometry/foil.stl, mesh/ (complete snappy case for 20 cores), template/flowCloud
(one 217 k point sampling dict, each case includes it by relative path), cases/<key>/ (system, constant, 0),
mesh.sh (1 node: blockMesh, decomposePar, srun snappyHexMesh, reconstructParMesh, checkMesh), cases.sh (job array
0..N-1 % parallel, copies the shared polyMesh, decomposePar, srun simpleFoam, keeps postProcessing + log + elapsed),
submit.sh (mesh, then the array with afterok), status.sh, collect.sh (packs <batch>_results.tar.gz), README.md.
`import_results(tar)` checks the surface hash, copies each case, runs `case.postprocess_case` (same files as a local
run) into a library of its own `lib_<kmh>kmh_<quality>-<batch>`, writes mesh.json / library.json, marks the local
manifest imported. Round trip tested with two local cases faked as cluster output. UI: "HPC" section (Build bundle
shows the exact scp / ssh / submit / status / collect / scp / import steps; Import results; batch list), quality
box lists imported libraries. Replay with the real air: `cruiseReplay.onFrame` hook; during a CFD free-flight
replay the CFD view turns the aircraft to the instantaneous alpha, beta and bank and reloads the air field when
the angles move by more than 1 deg (at most every 0.7 s). `setAttitude(a, b, roll)` gained the bank angle.
Iteration loop: new batch name, Build bundle, upload, submit, collect, download, import, free flight, repeat.

## Lesson: a background worker must not let mpirun touch the terminal (2026-10-06, 3 h lost)

The standard worker started from the shell with `nohup ... &` froze (ps state T) at its second case: prterun/mpirun
reads stdin, a background process reading the terminal gets SIGTTIN and is stopped. It looked "paused" in the jobs
panel and a SIGCONT only lasted until the next launch. Fix: every solver launch and every worker spawn now has
`stdin=DEVNULL` (`case.foam_run`, `Library.spawn`); by hand use `< /dev/null`. The jobs panel shows a stuck worker
as "paused" with no iteration progress and the live row goes "stale": that is the symptom.

## Conventions

- Far-field velocity in body axes: U = -V (cos a cos b, sin b, sin a cos b); alpha nose-up positive, beta wind
  from the right positive. Lift = Fx sin a - Fz cos a, drag = F . e_air, side = Fy. Moments about the FRD origin in
  the files, shifted to the CG at query time (M_cg = M0 - r_cg x F), so a CG change never needs a rerun.
- Coefficients with S = planform area, c = mean chord (S / span), b = span of the prepared surface.
- A case is "doubtful" when the force scatter over the last 30 % of iterations exceeds 10 %.

## First numbers (quick mesh, not converged: see convergence below)

The 250-iteration quick cases are useless for verdicts: lift at alpha 3 drifted from 198 N (150 it) to 125 N
(250 it) and the lift vs alpha table was non-monotonic. See the convergence section before trusting any grid.

## Convergence (1500 iterations at alpha 3, beta 0, quick mesh, 8 cores, 2026-10-06)

The steady solver never reaches the residual targets on this body: after about 250 iterations the forces settle
into a bounded oscillation (the separated under-body cup) and stay there. Fz -126 +-6 N (5 %), Fx -149 +-0.7 N,
My about the origin 205 +-3.6 N m; the means over the last 100, 200 and 400 iterations agree within 1.5 N.
Hence: quick 400 it, standard 800 it, fine 1200 it, forces averaged over the last 30 % of iterations, a case is
flagged "doubtful" above 10 % scatter. At 60 km/h and alpha 3 the clean foil gives only 118 N of lift (CL 0.11,
weight 451 N) and 155 N of drag (CD 0.14) on the quick mesh; the standard library will say how much of that drag is
mesh. Quick case: 29 s mesh, 0.26 s per iteration on 8 cores (0.17 s when the machine is otherwise idle).

## Limits

Steady RANS with wall functions on a voxel-closed skin (trailing edges at least one voxel thick); clean foil, no
fans, nacelles, jets or control surfaces; static tendencies only (no damping, no dynamic modes). The under-body
"cup" region (fan ducts) is massively separated, which is what makes convergence slow. For the damped motion the
Cruise Test free flight could take this library as its aero table (not done).
