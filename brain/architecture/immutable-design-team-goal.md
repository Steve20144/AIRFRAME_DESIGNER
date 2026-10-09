# Larger-airfoil project goal, immutable exterior

## Current user instruction

The user asks the existing team to work on AIRFRAME_DESIGNER's "bigger airfoil version": ChatGPT finds the best matching airfoil; Claude prepares the Blender/model and evidence-based mass properties, then AIRFRAME_DESIGNER SITL; both judge results and decide a bounded next iteration. The exterior design must stay untouched, and cruise must be stable and self-correcting. This latest immutable-design constraint supersedes historical permission to reshape upper skin. Existing global role files are not rewritten.

"Changing Blender" is scoped to a separate working copy's non-geometric association, material/physics metadata and measured mass representation. It does not authorize changing exterior vertices/topology, planform, sections, canopy or intentional openings. Do not add reflex, dihedral or control-surface geometry as an unapproved workaround. If no matching foil/available control configuration can satisfy the requirements, report that conflict rather than silently changing the design.

## Baseline evidence and unresolved identity

The current documented source is `blender/atlas_v1.stp`, SHA256 `72d01577ca0ff11af0075d33d3fdc2d1a39583c2752f6889443af13f1ded13a4` (verified unchanged). It is not `exterior_modified.3dm`, which was superseded as the wrong source.

A different historical candidate exists at `results/handoff/T20261005-atlas-v1-foil-references/from_A/v002/atlas_v1_work_v002.blend` and `atlas_v1_cand_v002.step`. Candidate v002 replaced upper skin, so it cannot be assumed to represent an unchanged original exterior. Resolved by the latest explicit user confirmation: the authoritative bigger-version input is `blender/atlas_v1.stp` and its nose must be treated as CAD -Y. The original STEP exterior is locked for this goal. Candidate v002 is historical comparison evidence only, not the baseline. Read-only fit rechecking can proceed without another source/orientation question. Never promote old E908/GOE5K/SC(2)-0402 rankings to a current optimum without rechecking the actual locked shape and corrected nose direction.

The latest explicit user confirmation establishes nose CAD -Y, independently of contradictory historical notes. Current cruise documentation also uses CAD -Y. Some old source script docstrings and Blender metadata still say +Y; audit frame transforms before any reuse. `scripts/atlas_v1_from_masses.py` uses the corrected numeric matrix but contains stale prose and assumptions. Do not execute it blindly.

## Mass and simulation provenance gates

`from_A/v002/masses.json` contains 24 user-placed points, total 45.994 kg, timestamp 2026-10-05T19:12:30-07:00, tied to candidate v002 meshes. Existing notes call them "crucial parts only"; BOM airborne total is documented as 149.61 kg. These are different inventories, not interchangeable estimates. Confirm complete installed components, their physical positions and distributed inertia, and mapping to the locked model. No invented ballast, CG relocation or automatic replacement of real weights to obtain stability.

The historical builder symmetrizes roughly placed motors, assumes 215 N per rotor, 75-degree jet turn, time constant and reaction torque. None is installed-flow validation. Current Cruise Test is strip-theory/free-flight analysis, explicitly **not PX4 SITL**. Its six-axis results roll off, and the documented elevons are not yet driven by PX4. Simulator/controller/actuator coupling therefore needs explicit engineering validation before claiming self-correcting cruise. See the canonical cruise-test and mass-tool notes for detail.

## Bounded workflow

1. Dots: read-only baseline/provenance inventory and airfoil-fit research, respecting exact fixed geometry. Separate observations, assumptions, candidate ranking and missing inputs. Use allowed research tools under the active session's existing permissions, no new network/service credentials.
2. Before model writes: verify the confirmed baseline hash and complete mass/control provenance; approve a goal-scoped execution capability bundle. The runtime's generic per-goal dispatch gate keeps Claude from running while this setup is unresolved. Dots may produce useful analysis first.
3. Claude: create an isolated, versioned simulation representation that preserves the locked exterior; assign only sourced masses; establish a declared frame and source hashes. No canonical model overwrite.
4. Analysis and later private SITL: static/trim/short six-axis checks first, then a validated cruise controller/actuator scenario on a reserved nonzero PX4 instance. Never use hardware or disturb port 8080/instance 0.
5. Both agents review evidence before the next bounded iteration. Default runtime limits remain one active writer, at most 12 transitions, 3 change cycles, 2 retries, 900 seconds per invocation; no unlimited research/simulation loops. Keep a reviewed finite experiment matrix.

The previous 60 km/h target is a provisional benchmark pending current user confirmation. Before running, agents must record quantitative acceptance thresholds for trim residuals, valid polar envelope, actuator headroom, signed perturbation recovery in all six axes, steady attitude/rate/airspeed error, oscillation growth and saturation duration. Include multiple seeds and time-step refinement/repeatability. Separate passive aerodynamic stability from active controller correction. A pitch-only constrained run, ideal trim couple, or one successful hover is not a six-axis cruise pass. Agents should propose measurable thresholds in their test plan, not invent evidence or silently declare them user-approved aircraft requirements. Simulation results are not flight-safety certification.

## Workspace and capability limitations

The original checkout contains extensive dirty/untracked user work, including the source CAD, cruise module, mass builder and airframe. The runtime currently creates worktrees from committed HEAD only. Before Claude uses them, prepare a reviewed, content-hashed overlay of the selected current project files and reference inputs into the isolated goal worktree; do not silently run obsolete HEAD code or copy the whole dirty checkout. Canonical originals remain unchanged/read-only.

Current Claude adapter has file tools only; it cannot run Blender, Python tests or SITL. No previous smoke read allowance is still active. The proposed goal-specific approval bundle is exact canonical instruction/context/input reads, writes confined to the goal worktree and private output directory, and a reviewed argv-based command runner for installed Python, Blender and the existing PX4 binary. Do not enable arbitrary Bash, arbitrary Python -c/scripts, directory-wide external writes, hardware/serial tools or network access beyond the explicit local SITL links. Pin reviewed execution scripts and re-review changed executable inputs. No persistent permission grant or launch-at-login service.

## Command inventory for the scoped execution review

Installed binaries were verified present: `/Applications/Blender.app/Contents/MacOS/Blender`, the repository `.venv/bin/python`, and `~/PX4-Autopilot/build/px4_sitl_default/bin/px4`. Existing CLI help supports:

```
.venv/bin/python -m airframe_designer run --airframe <goal airframe> --scenario <reviewed cruise scenario> --instance <reserved 1..9> --seed <reviewed seed> --timeout <bounded seconds> --out <goal output> --timeseries <goal output>
```

This is an interface inventory, not permission to execute an arbitrary scenario. The existing runner writes PX4 instance parameter files and uses `/tmp/px4_lock-N`; its work directory must be redirected/contained in the goal's private runtime area, and its instance must be atomically reserved. Do not grant write access to the shared PX4 source/build or existing instance directories. Use only the reserved loopback HIL/control ports, not serial/hardware or remote network.

Blender's existing project pattern is `Blender --background --factory-startup --python <reviewed script> -- <goal output directory>`. The present `scripts/blender/atlas_v1_build_blend.py` consumes historical candidate meshes and writes +Y metadata. It is not an approved immutable/-Y job. Similarly `scripts/atlas_v1_from_masses.py --masses ... --out ...` still hardcodes v002 meshes and also writes an STL at a fixed project path. These must be adapted/reviewed in the isolated workspace before exact command approval; merely changing `--out` is insufficient.

For validation, reuse existing Python tests, starting with a reviewed selection such as `tests/test_cad.py` and `tests/test_sim.py`, plus new meaningful frame/geometry-preservation/mass/cruise tests. `pytest -m 'not px4'` is the documented non-PX4 suite, but pin/review the executable test inputs first. Cruise analysis currently exposes the Python `cruise_test` function, not a verified standalone cruise-SITL command. A bounded, reviewed entrypoint and actuator mapping are required before claiming that capability.

Approval bundle to present once commands/scripts are concrete: exact canonical instruction and selected input reads; writes only to this goal's isolated workspace/output/private temporary paths; reviewed executable/script hashes and fixed argv for Blender/model validation/tests/private SITL; bounded time/process counts, reserved nonzero instance and loopback ports; no credentials, arbitrary shell/eval, source overwrites, auto-push, hardware or persistent access. Until then the generic Claude dispatch gate remains in force. This setup work does not perform the Dots airfoil research or fabricate a model/control implementation.

## Reviewed preparation checkpoint

Separate preparation worktree: `results/factory/preparation/G-af1034164247`, branch `factory/prep-G-af1034164247`. See `execution_prep/README.md`, `overlay.json` and `permission_bundle.json` there for exact hashes and bounded source-only tessellation/Blender argv. Seven synthetic safety tests passed. No actual CAD/Blender/PX4 execution or permission expansion occurred; real Dots task worktree was not modified. Mass/controller/SITL stages remain gated. The bundle is review evidence and requires caller-enforced OS isolation before any execution.

## One-shot runner integration ready for review

The operator-only `ClaudeAdapter.approve_preparation_once` hook and `team_factory/scoped_runner.py` / `scoped_bridge.py` are implemented but inactive. See `results/factory/preparation/G-af1034164247/execution_prep/APPROVAL.md` for exact commands/write scope and bundle digest. Claude keeps its existing filesystem sandbox; a private stdio-to-Unix-socket bridge reaches an orchestrator-owned broker, which applies the narrower payload sandbox. This avoids unsupported nested macOS sandbox initialization. Grants are task/goal-bound, expire, are consumed before spawn and are never loaded from agent results/API/config. No model payload or permission grant has occurred. Dots expired-lease artifacts were independently hash-verified and operator-salvaged; task is assigned to Claude, team paused, execution/mass/control gates intact.

## Current user direction and no-deletion revision

The owner now requests roughly the same overall design, explicit quantified candidate deviations, cruise at 60 km/h, a whole-body regional foil combination and eventual navigation. Proposed deviations must remain visible; the original STEP stays the baseline. User approved the two source-only preparation operations but prohibits deletion, including temporary files. Version v2 at `results/factory/preparation-v2/G-af1034164247/execution_prep/REVIEW.md` retains old artifacts/temp/socket and denies filesystem unlink. Independent review precedes activation; no extra owner decision is requested for this narrowing. ASTRA evidence task T-62f272ed20ab completed and preserved its new matrix; a bounded isolated-section XFOIL pilot can use existing authorized worker tools, separate from the Claude preparation grant and future flight simulation.


### Reviewed v4 source preparation: partial success, retained native crash

Actual Claude invocation `61da93d368f14cc29233df7acd0ce764` used approved bundle `42ff60563f57c2e7c473b9d398f85d6e0f9ea7bdd84b3d9b849fca9d31cbccc5`, one attempt per operation. STEP tessellation succeeded with three bodies; original and private-copy SHA-256 remain `72d01577ca0ff11af0075d33d3fdc2d1a39583c2752f6889443af13f1ded13a4`. Mesh SHA-256 is `d40fcfe03e1f3cdbbf1e22dc414700956416d1154aa537f90fd4b20d88df3875`. Blender exited SIGSEGV (-11) during native Metal backend detection (`supports_barycentric_whitelist`, `WM_init`), before Python payload. The accompanying rmdir denial is retained; it is not established as the crash cause. Installed Blender help lists only Metal backend. No successful .blend output, permission expansion, deletion or replay. Grant retired (event157); preparation task paused for engineering diagnosis and existing assessment task remains paused.

Audit: `results/factory/verification/claude-preparation-v4-audit-1b3083d2bf164060b96748364bb8024f.json`. This also preserves an earlier invocation result that remained in task state after an adapter exception; that stale summary is not v4 evidence and was cleared from the current task view.

ASTRA diagnostic task `T-31204981554f` completed before this invocation. Its four bounded continuation solutions converged; both zero-alpha outcomes agreed at printed precision (CL .1148, CD .00859, Cm -.0336). Original independent-start pilot failure remains unchanged; this is initialization-sensitivity evidence, not whole-aircraft cruise or stability proof.


### Partial preparation unblocks finite centrebody comparison

`T-036132d414c8` now depends on the completed pilot and initialization diagnostic, not a successful Blender file. Its explicit scope is verified mesh assessment, read-only IEEE warning-origin inspection and E908 versus GOE6K at60km/h: four continuation paths of three solutions (-1,0,+1 chord-alpha), natural Ncrit9 versus5%top/bottomtrip,320panels,T=.02,VACC0,300iterations;20seconds per path,80aggregate,12attempts,zero automatic retries. Exact flow_inputs Reynolds/Mach remain authoritative. No54-point sweep, whole-aircraft claim or further automatic solver batch. Evidence copied additively into `analysis/source_preparation_v4_evidence_01` in the goal worktree; manifest SHA256 `a8ca1f11d348f0238582438e2969224f79d78f81df1f409159b8bce945055b50`. Its three bodies are surfaces with zero volume/inertia metadata; do not infer masses.

Result lifecycle correction: a new claim archives the preceding result with its invocation ID in append-only events and retains separately labelled previous_result context; current result/error are cleared before execution. Adapter failures can no longer display old outcomes as current. Ten targeted regression tests passed, including actual runtime exception and restart/history coverage. Normal factory server restarted with this fix and no preparation grants.

Blender technical blocker: local5.2.2 help offers only Metal, no null/CPU backend. Background mode was already used. CPU Cycles flags govern rendering and are not a verified workaround for pre-Python Metal detection. Upstream source now checks for absent Metal devices with a sandbox/non-GUI note referencing issue163272: https://raw.githubusercontent.com/blender/blender/main/source/blender/gpu/metal/mtl_backend.mm . This supports a null-device explanation for the retained strstr/backtrace but does not prove the installed binary contains the fix or establish exact OS entitlement needed. Upstream CLI source https://raw.githubusercontent.com/blender/blender/main/source/creator/creator_args.cc has a newer debug GPU no-fallback option absent from installed help; it was not used. No supported same-permission workaround verified, no new binary installed, no GPU/Mach permissions broadened, and no preparation retry performed. Full read-only diagnosis and .blend remain separate from section science.
