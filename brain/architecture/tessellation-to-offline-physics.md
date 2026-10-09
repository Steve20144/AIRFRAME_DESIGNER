# Tessellation to offline physics: current execution path

Audit2026-10-07. Goal G-af1034164247. This extends the existing physics handoff; source STEP and existing roles remain canonical. No Blender installation or security change is needed for the next engineering stages.

## Available evidence and executable path

Verified v4 source_mesh.json contains three open surface meshes in CAD metres, nose -Y. The goal worktree has a hash-verified copy under analysis/source_preparation_v4_evidence_01. Existing corrected-minus-Y cross sections, candidate deviations and current bounded XFOIL results can be consumed as JSON/NumPy data. A .blend is a representation artifact, not a required input to Airframe.from_dict, WingSet, VehicleModel or RigidBody.

1. After ASTRA centrebody assessment, Claude builds a separately versioned geometry/input contract from those retained meshes and sections. Store source hashes, CAD-to-FRD rotation [[0,-1,0],[-1,0,0],[0,0,-1]], an explicit reference translation, measured span/chords/incidence/area, chosen regional profile interfaces and full-contour deviations. Keep baseline and proposed geometry separate. Do not reuse the historical v002 altered upper skin or hard-coded M001 planform silently. Zero mesh volume/inertia values are not mass properties.
2. Create a reduced-order JSON model and validate save/readback, frames and force signs. Mesh-derived planform alone is not aerodynamics: use explicit section tables with coordinate/solver/Re/transition provenance and only their supported domain. Disable implicit airfoils.py auto generation/download: ensure_polars/get_polar currently may write caches or select another solver. Prepare private pinned assets and a fail-closed table/domain reader in the candidate bundle. The present -1..+1 section points do not justify broad alpha or Reynolds interpolation.
3. First run mass-independent force/moment comparisons at60km/h and declared incidences, including fixed common reference moments and per-region contributions. This produces genuine reduced-order aerodynamic calculations without inventing aircraft weight. Geometry/polar interpolation assumptions and strip-theory limitations must be explicit. New solver execution belongs to a separately bounded reviewed invocation, not the retired STEP/Blender grant.
4. Then static trim and short RigidBody integration for a labelled hypothetical loading bracket, with explicit total mass/CG/positive-definite tensor and sensitivity bounds. Agent engineering assumptions are permitted when clearly hypothetical; they must not be promoted to installed inputs. Run only after load/static balance gates, fixed seed,5seconds at dt.002 and dt.001, full-six-axis residuals, retained failure traces and no hardware/PX4. This is offline physics, not SITL. Real predictive trim awaits complete loading and installed propulsion/control data.
5. Add/control-test elevon command plumbing before any cruise-navigation SITL claim. Simulator.step currently passes HIL actuator values to set_motor_commands only. WingSet.set_controls and set_elevons exist, but no PX4 surface channel adapter calls them. Define explicit motor versus servo output mapping, signed/saturated deflections, allocator/airframe firmware configuration, failsafe and round-trip tests. Existing auto-hop firmware is a prototype ground/hover sequence, not this cruise controller. Private PX4 SITL is a later gated run using a separate instance and verified effective model/parameters.

## Rechecked missing physical inputs

Current from_A/v002/masses.json remains24points/45.994kg, updated2026-10-05T19:12:30-07:00. It includes nine roughly placed fan entries (duplicate #9 label), nine ESCs and six avionics/power items. It lacks the main flight battery loading, structure/canopy, mounts/wiring and payload/loading configuration. Needed for predictive rigid-body flight: total loaded mass, CG and full inertia tensor or justified component approximations. No new mass placement was found.

Nine-fan topology is known; actual net-force locations/directions, physical fan centres, installed thrust/inflow/turn-loss/power model and controller channels remain unresolved. Historical builder assumes75deg turn,215N/max,km.01,tau.3 and mirrors placements; these are not measured truth. Its source docstring still shows old +Y transform, while executable R is corrected -Y; new contract must derive from explicit tested values. Candidate body drag and sectional control effectiveness are also modelling assumptions.

Current cruise.py replaces propulsion with ideal forward thrust at CG, optionally adds ideal pitch couples, and holds elevons fixed. Longitudinal-only mode clamps roll/yaw/sideslip. Even a successful output from that helper is not autonomous navigation or actual EDF-powered stability. Six-axis and longitudinal modes must remain distinguished.

No immediate human choice is required to finish geometry extraction, numerical section comparison or explicitly hypothetical offline sensitivity. Actual aircraft prediction ultimately requires the owner/physical measurements to supply the complete loading case and accepted installation, not a generic request to choose an airfoil.

## Blender research boundary

Installed only /Applications/Blender.app:5.2.2LTS, build d13f752e3b9c,2026-09-15, release branch. No standalone bpy distribution was found in project .venv. Bundled scripts/modules/bpy imports native _bpy and is not a separate headless Python runtime.

Official source main now includes nil-device guard referring to issue163272: https://raw.githubusercontent.com/blender/blender/main/source/blender/gpu/metal/mtl_backend.mm . The inspected blender-v5.2-release file does not contain that guard: https://raw.githubusercontent.com/blender/blender/blender-v5.2-release/source/blender/gpu/metal/mtl_backend.mm . Thus a main-branch fix is confirmed; a released replacement containing it is NOT verified. Official download/corrective-release retrieval was unavailable and issue page inaccessible. Installed CLI exposes only Metal; no supported same-rights GPU-free startup found. No alternative binary installed, no sandbox change, no rerun. Continue numerical physics independently of .blend.


## Implemented isolated preparation scaffold

Versioned root: results/factory/physics-prep-v1/G-af1034164247. geometry_contract.json pins the actual three-body mesh and measured per-body bounds; panel/candidate mapping is explicitly pending Claude. input_boundary.py implements local hash-pinned JSON, traversal/symlink/duplicate-key/nonfinite rejection, exact Reynolds/Mach/transition matching, bounded interpolation with failed points breaking support, and hypothetical mass/CG/inertia validation including physical triangle inequalities. run_offline.py is a fixed isolated-mode entrypoint that refuses absent/draft review or incomplete geometry before outputs. Report creation is exclusive, with no replay or deletion. Ten boundary tests passed; synthetic tests are not model results. prep_manifest_revision_03.json is the latest development manifest; earlier manifests are retained historical drafts.

CLAUDE_TASK.md specifies model/physics implementation under the existing role. execution_scope.DRAFT.json separates file-only implementation from a later independently reviewed fixed execution:120seconds,one invocation,<=6forcecases,<=9hypothetical loadings,zero retries,no network/shell/subprocess/hardware/PX4/deletion. Dynamic integration is deliberately a subsequent static/domain-gated stage, not silently simulated on inadequate tables. No capability or new broker rights have been activated.

Once mass-independent work is exhausted, the smallest owner question for an installed-aircraft prediction is: which complete loading configuration should be represented (including main packs, structure and payload), and its measured/estimated component masses and CAD positions? Until supplied, retain explicitly hypothetical brackets; do not ask merely to choose arbitrary numeric assumptions.


### Actual Claude file-only dispatch

Implementation subgoal G-fafbd7efe376, task T-1e5789fc0672, actual invocation8c39b53f51eb4add8b1f488caf4c4c61/session82126c0c-9bba-492f-bebe-af5f0a53980d. Separate Git worktree results/factory/worktrees/G-fafbd7efe376, writes constrained to physics/ by CLI tool rules and OS sandbox. Exact canonical instruction reads approved for this invocation; no shell, solver, model run, deletion or new broker capability. Actual centrebody comparison/report and hashed measured geometry inputs supplied. Agent instructed to return a structured handoff to dots for independent review; normal handoff persists even when Dots is disconnected. All source/scaffold artifacts retained.

Primary-goal ASTRA regional task T-873caeda707f queued paused awaiting actual worker:18solutions/6continuation paths,20seconds/path,120seconds aggregate,zero retries. Transition GOE5K/NACA16-006 and outboardSC0403, existingSC0402 comparator with initialization mismatch explicit. No direct full-contour replacement implied by upper-sheet fit. Goal workspace isolated from Claude implementation. Global concurrency remains1, so dispatcher serializes active invocations.


### Concrete eventual PX4 control changes (not executed)

Current WingSet.set_controls inputs are degrees, not normalized sticks: positive pitch input means trailing-edge down and nose-down; positive roll means right trailing-edge down. Implement an explicit surface-channel map in the airframe schema with neutral, sign, min/max degrees, slew and stale-command handling. Validate unique nonoverlapping motor/surface indices; do not assume a generic HIL control slot is a surface.

PX4Link currently stores all16 HIL_ACTUATOR_CONTROLS values and sequence/time. Simulator.step forwards the vector to RigidBody.set_motor_commands, which truncates to rotor count; surface channels are unused. Add a pure validated demultiplexer that preserves motors and maps configured servo values to set_elevons (per surface) before substeps. Preserve motor_override/nose_lift semantics without replacing servo commands. Tests: zero/positive/negative signed commands, left/right moment signs, saturation/slew, invalid/missing/NaN frames, disarmed/stale policies and zero effect on existing motor-only models.

Current Airframe.px4_params forces CA_AIRFRAME=0 and exports rotors; the new cruise airframe needs an explicitly reviewed compatible allocation/output function configuration, not guessed servo channels in the multicopter export. Verify installed firmware parameters and HIL output mapping in a private instance before navigation tests. A software damper can be an explicitly hypothetical offline controller but must not be presented as PX4 or navigation.


### Transport recovery and real file-only continuation

Regional invocation ae479b88b8cc43e6be6018adb13c4372 was orphaned after transport loss. Parent confirmed worker stopped; two process audits found no regional Python or XFOIL executor. Paths0/1 were retained with6attempts4converged8.935291790927295seconds. Historical path2 remains EXECUTION_UNKNOWN despite no directory; never replay it. Operator reconciliation closed the expired mailbox without renewing its lease and left the partial task paused. Recovery manifest SHA256 d8a8b15e7a7741c1a947428fa09bea74e78e5a258daf3a973b415eaefa0df29a.

Fresh continuation T-d3bdb3d52a92 permits only paths3,4,5 (NACA16-006 forced,SC0403 natural/forced),9solutions/60seconds/zero retries. Original campaign accounting reserves3unknown attempts/20seconds: total18attempts maximum and88.935291791seconds worst known+reserved+new, below120seconds. Fresh actual worker attachment is required; old heartbeat is stale and disconnected.

Claude first file-only call ended at20turns with six implementation files and no tests/docs/handoff; no physics was executed. It was resumed using the existing session and same bounded file-only authorization as invocation0ef6654ebb594ad687daf88c044fa474, taskT-1e5789fc0672. Deadline01:16:31UTC. Remaining scope fixes the independent localRe/Mach issue, writes tests and README, then structured handoff. Current server13461 configured existing max_parallel_agents=2 only at a verified quiescent restart; per-agent and per-goal writer locks remain. No broker grant or model execution rights.
