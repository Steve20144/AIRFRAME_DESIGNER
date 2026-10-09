# SUPERSEDED: WRONG SOURCE FILE, DO NOT APPLY

User correction (2026-10-05T21:05:14.903476+00:00): exterior_modified.3dm was the WRONG aircraft file. The authoritative input is blender/atlas_v1.stp. All earlier selections, placements, fit metrics and implementation instructions derived from the Rhino file are historical only and must not be applied to the STEP aircraft. E908/GOE5K/SC(2)-0403 selection is not a current recommendation. New findings require user review before any new Agent A handoff or implementation.

---

# Project Brain Index

This file is the entry point to the project's persistent knowledge base.

**Multi-agent work**: read [HANDOFF.md](HANDOFF.md) first. It is the coordination channel between agents
(roles, the foil -> Blender -> STEP -> masses -> airframe -> sweep -> SITL -> HITL workflow, status board, message log).

Claude should read this file before exploring the rest of the brain, then open at most the three to five notes the
current task needs.

---

## Product

- [Vision](product/vision.md): a PX4-in-the-loop design simulator for the ATLAS ten-fan nose-up hoverer.
- [Requirements](product/requirements.md): what a flight test must show before hardware.
- [User Experience](product/user-experience.md): the app's tabs and the indoor flight flow.

## Architecture

- [System Overview](architecture/overview.md): segments, frames, data flow, PX4 coupling.
- [How to run](architecture/how-to-run.md): macOS (native arm64 PX4 build, SDKROOT gotcha) and WSL environments, commands, ports, tests, conventions.
- [Scenario runner and ground sequences](architecture/scenarios-and-ground-sequences.md): phases, the scripted pilot,
  the attitude block, nose lift and nose lower.
- [Tuning tab](architecture/tuning-tab.md): attempts, sweeps, flight library, charts, API.
- [V3 minimum parameter set](architecture/v3-minimum-parameter-set.md): the 18 dials + 4 switches per phase for
  rotate, handover, climb 1.5 m, hover, descend, lower; what no parameter fixes.
- [Telemetry radio and throttle dashboard](architecture/telemetry-dashboard.md): SiK on TELEM3, the live dashboard,
  per-run records, log download over the radio, PX4 stream quirks.
- [Design knobs](architecture/design-knobs.md): jetfoil angles, front jets, battery/CG as named values that move
  rotors and linked CAD parts; Geometry tab card, `knobs.*` paths for sweeps (branch atlas-v5-design).
- [ATLAS V3 small draft](architecture/atlas-v3-small.md): SMALL_SCALE_V3.step as an airframe; what is CAD, what is
  assumed; flies Stabilized at 10.5 deg in SITL but the heading turns 52 deg in 12 s.
- [ATLAS V3 from the live Fusion design](architecture/atlas-v3-fusion.md): Fusion's local MCP server
  (127.0.0.1:27182/mcp, `scripts/fusion_mcp.py`), labelled masses only, foil exit angles ray-traced on the B-rep,
  `scripts/atlas_v3_from_fusion.py` -> `airframes/atlas_v3_v34*.json`; optical flow sensor model `sensors/flow.py`.
- [Cruise Test tab](architecture/cruise-test-tab.md): forward-flight stability from the section polars (alpha sweep, trim,
  static margin, free flight without a controller); nose = CAD -Y (corrected 2026-10-05); M001 is nose-heavy with the NP within 0.25 m of the CG, no natural trim
  (cambered sections), rolls off without roll control.
- [CFD tab](architecture/cfd-tab.md): OpenFOAM attitude library on the voxel-closed clean foil (geometry closure, case
  writer, interpolation, static-stability verdicts, three.js Cp view); 2026-10-06.
- [ATLAS v1 mass tool](architecture/atlas-v1-mass-tool.md): S4 of the handoff, click mass points onto candidate v002
  in the browser (`scripts/atlas_v1_mass_tool.py`, port 8099) -> `masses.json` with CAD and FRD positions.
- [ATLAS agents team plan](architecture/atlas-agents-team-plan.md): PROPOSAL 2026-10-08, chief + 5 teams (engineer + reviewer each) across 6 Macs, new `atlas-agents` repo, GitHub as the shared bus; first mission V3_30 stable hover.
- [Fan vibration model](architecture/vibration-model.md): `design.vibration` (imbalance, blade pass, mount, frame
  modes) at the IMU, PX4's vibration metrics, the `vibration` bench command, first numbers.

## Decisions

`decisions/` holds one record per decision:

- [2026-09-17 Gazebo attaches as a physics backend](decisions/2026-09-17-gazebo-as-physics-backend.md)
- [2026-09-21 Fly indoors in Stabilized, on the flat brackets](decisions/2026-09-21-stabilized-indoor-flat-brackets.md)
- [2026-09-21 The scenario owns the parked and hover pitch](decisions/2026-09-21-scenario-owns-attitude.md)
- [2026-09-21 Tuning lives in the app](decisions/2026-09-21-tuning-tab-in-app.md)
- [2026-09-21 ATLAS_09B model corrections](decisions/2026-09-21-atlas-09b-model-corrections.md)
- [2026-09-22 The nose lift runs on the flight controller](decisions/2026-09-22-nose-lift-on-the-flight-controller.md)

## Experiments

`experiments/`:

- [2026-09-18 ATLAS_OG board tune (HITL)](experiments/2026-09-18-atlas-og-hitl-board-tune.md)
- [2026-09-21 Stabilized baseline, flat vs as built](experiments/2026-09-21-stabilized-baseline-flat-vs-asbuilt.md)
- [2026-09-21 ATLAS_OG flat gain sweeps and confirmation](experiments/2026-09-21-atlas-og-flat-gain-sweeps.md)
- [2026-09-22 Firmware nose lift, park +4 to -8](experiments/2026-09-22-fw-nose-lift-park-sweep.md)
- [2026-09-21 ATLAS_09B: from import to the tuned sequence](experiments/2026-09-21-atlas-09b-sequence-tuning.md)
- [2026-09-21 ATLAS_09B rounds 4 and 5: the yaw loop and the hands-off drift](experiments/2026-09-21-atlas-09b-yaw-rounds.md)
- [2026-09-21 First piloted HITL session](experiments/2026-09-21-hitl-pilot-session.md): RC setup, integrator wind-up on the legs, hover thrust, stick scale
- [2026-09-23 Firmware nose lift on the real aircraft](experiments/2026-09-23-aircraft-nose-lift-bench-runs.md): two
  fans, props on; false aborts fixed, sim gains failed, G4 set, rate filter rejected, fan response still unmodelled
- [2026-09-23 The nose-lift fan bursts](experiments/2026-09-23-nose-lift-fan-bursts.md): 4 Hz off/full switching
  from the rate gain on the legs' rocking; filters tumble it; KQ 0.03 / K_ANG 0.4 cuts it 6x; the hold check could
  never pass on the rocking frame (fixed: 1 Hz settled rate), so holding and handover work in SITL
- [2026-09-23 Nose-lift gains for 24 deg](experiments/2026-09-23-nose-lift-24deg-gains.md): the 1 s spin-down model
  was optimistic; judged over a bracket of fan models, C5 (KQ .04/.02, K_ANG .3, RATE 1.5, CEIL 1) never tips
- [2026-09-23 Mitigating fan vibration (Tuning tab)](experiments/2026-09-23-vibration-tuning-rounds.md): vibration
  doubles motor jitter and, indoors, attitude error and drift; soft mount or balance fixes it, gyro filters fix jitter
- [2026-09-25 Nose hold flights, ULog findings](experiments/2026-09-25-nose-hold-flights.md): no liftoff on 25 Sep; first liftoff 00:21 on the front-fix build (0.8 m, nose held 26-30); M9 clips at full while M10 sits at half (split wastes pitch authority); forward push uncertain
- [2026-09-26 V3 jetfoil x front tilt sweep](experiments/2026-09-26-v3-jetfoil-front-sweep.md): at ATLAS_09B's masses
  (11.785 kg) best outer 17.5 / middle 25 / inner 25 deg lean, nose fans 30 as built; the draft fails; mixed station
  angles win, nose tilt barely matters; mass.resolve() ignored CAD bodies (fixed). CG sweep: no optimum, flights
  swing 0.5-6 m drift with 0.5 deg of hover pitch (cause: yaw authority, see lessons)
- [2026-10-05 ATLAS v1 M001: first mass placement in SITL](experiments/2026-10-05-atlas-v1-m001-first-sitl.md): 46 kg of
  clicked parts, nine 215 N fans at the fan points, hover trim 10 deg; V3_30 gains roll it over, set A (rate 0.3/0.1/0.02,
  att 3) flies 3/3 seeds; 0.7 m/s forward drift open; no fan can raise the nose from a nose-down park
- [2026-10-01 V3_30 STEP import + tune](experiments/2026-10-01-v3-30-step-import-and-tune.md) — top-level-label masses (14.4 kg), CAD legs, pitch-rate P 0.9 / I 0.05
- [2026-10-01 V3_30 foil headroom sweep](experiments/2026-10-01-v3-30-foil-headroom-sweep.md): foils redistribute, do not add thrust; pick 35/75/70 doubles roll spare; 20 deg hover+rotation needs a 60 deg jet and a level park; park -10 needs jets 57.5/85/85
- [2026-09-29 V3 v34 from Fusion: CG, authority, H-FLOW](experiments/2026-09-29-v3-v34-fusion-cg-hflow.md): 11.83 kg
  labelled; rear battery block +195 mm aft -> drift 1.86 -> 0.53 m; yaw ~6x weaker than roll; H-FLOW must be tilted to
  the hover frame (20.4 deg), belly or nose equal in SITL; CAD change notice published
- [2026-10-05 ATLAS v1 foil candidate v001](experiments/2026-10-05-atlas-v1-foil-candidate-v001.md): full-scale
  STEP + Agent B's E908/GOE5K/SC(2)-0402 lofted as a review candidate; foils fit the upper skin (2-42 mm) but as
  full sections cut the cabin and the rear nacelles (45-91 % of section area); import_step now loads open shells.
  v002 (user's choice) replaces the upper skin only: max change 42.6 mm, seams 1.8 mm, area +0.8 %
- [2026-10-06 V3 first automatic hop, killed at 1.5 m](experiments/2026-10-06-v3-first-auto-hop.md): the aircraft was
  in POSITION mode (no flight-mode channel), so the throttle override was a climb-rate command and the position
  controller flew the attitude; fixed: the module commands Stabilized and refuses otherwise, stick-scaled feed-forward,
  throttle slew limit; kill worked in 0.1 s
- [2026-10-06 Automatic takeoff, hover and landing on the board (NL_AUTO)](experiments/2026-10-06-auto-hop-firmware.md):
  the nose-lift module flies the throttle after the hold (baro height loop, rc_update throttle hook), roll/pitch/yaw
  stay with the pilot, kill switch latches; SITL passes on V3 (the board's airframe) and 09B; needs COM_DISARM_LAND -1 and NL_FLY_HOLD 0
- [2026-10-07 Board readout before the second automatic hop](experiments/2026-10-07-board-readout-before-second-hop.md): build Oct 6 16:06 (the Stabilized fix) confirmed on the board, NL_AUTO_ALT 0.8 / HOV 5, COM_DISARM_LAND -1, backup script stops at 260 params.
- [2026-10-07 V3 second hop: nose stuck 8.5 s at full command, breakaway, overshoot abort](experiments/2026-10-07-v3-second-hop-nose-stuck-overshoot.md): integral windup during a stuck nose, no stuck detection, abort cut every motor at +20 deg.
- [2026-10-07 Nose lift ramp: anti-windup, NL_I_MAX, NL_STUCK_S](experiments/2026-10-07-nose-lift-antiwindup-stuck-abort.md): built after the stuck-nose attempts, not simulated; Oct 6 image not kept.
- [2026-10-07 SITL board state, nose-fan thrust sweep](experiments/2026-10-07-sitl-board-state-nose-thrust-sweep.md): flashed code + board params pass at 36..28 N, crash at 25 N, no lift at 21 N; real aircraft sits at 28 to 30 N equivalent; hold cmd > 0.70 = do not take off.
- [2026-10-08 SITL firmware + parameter selection](experiments/2026-10-08-sitl-firmware-selection.md): winner C (takeoff gate + handover nose-drop guard, module hover, ramp 2 s), PASS to 28 N, safe NOGO below, no crash; PX4 Hold path broken; corrects the 7 Oct cmd/thrust mapping.
- [2026-10-08 SITL package: hover, landing order, smooth rotation](experiments/2026-10-08-sitl-pack-hover-landing.md): scripts/sitl_pack.py (21-29 flights/min), module takes over at touchdown (NL_AUTO_TD 3), Position hover + PX4 Land, nose fall 13.5 -> <1 deg; final firmware for HITL in results/sitl_pack/final.
- [2026-10-09 HITL: tremble and jolts](experiments/2026-10-09-hitl-tremble-and-jolts.md): rc_update 300 ms throttle steps (tremble), legs-aware attitude setpoint (lift-off kicks), NL_LOW_VMIN, roll gains for USB delay; HITL final3 tremble 0.12-0.27, jolts <= 4.6 deg/s
- [Lesson: sim Python nose-lower overrides the firmware](lessons/sim-python-nose-lower-overrides-firmware.md): design.nose_lower arms above 1 m CG and takes every motor after touchdown even with executor firmware; disable it in firmware SITL.
- [HITL board conditions](lessons/hitl-board-conditions.md): arm switch ch 8, CRSF owns input_rc 0, sim-sensor auto-calibration; HITL-only values and the flight restore
- [2026-10-08 Jolts and drift](experiments/2026-10-08-jolts-and-drift.md): Position climb from the ramp end, range-finder height, Position descent 0.05 m/s, bumpless lowering, floor to 6 cm off the legs; jolts ~5 deg/s; remaining drift 0.25 m = H-FLOW mount 20.4 deg (re-angle to 8.5: 0.03 m).
- [2026-09-26 V3 graded jetfoils](experiments/2026-09-26-v3-graded-jetfoils.md): jet steepest at the body, gentlest
  at the foil end; best 40 / 30 / 15 lean (turned 50 / 60 / 75), trim 26.9, 9/9 flights, drift worst 0.51 m (previous
  best 1.55); `airframes/atlas_v3_small_graded.json`, views in `docs/v3_views/graded/`; first sweep on macOS

## Research

`research/`:

- [Indoor positioning options](research/indoor-positioning-options.md): H-Flow, GPS repeaters, external vision.
- [Upstream repository and the ATLAS_09B model](research/upstream-repository.md)

## Lessons

`lessons/`:

- [PX4 parameter seeding drifts with the firmware version](lessons/px4-param-seeding-version-drift.md)
- [Quadratic fans need THR_MDL_FAC 1](lessons/thr-mdl-fac-quadratic-fans.md)
- [The pseudo-inverse mix and the idle fan](lessons/pseudo-inverse-idle-fan-authority.md)
- [Stands, pivots and lift margin](lessons/stands-and-lift-margin.md)
- [Reaction torque decides controllability](lessons/reaction-torque-km.md)
- [Environment and process gotchas](lessons/environment-gotchas.md)
- [A PX4 module must read the clock after copying its messages](lessons/px4-module-clock-before-copy.md)
- [Fan vibration drifts the EKF height on the ground](lessons/fan-vibration-drifts-ekf-height.md)
- [V3's hover-pitch sensitivity is the yaw authority limit](lessons/v3-hover-pitch-sensitivity-is-yaw-authority.md):
  same-spin reaction torque keeps yaw at its limit, it leaks into pitch, hands-off Stabilized turns that into drift;
  km 0 removes it; judge V3 sweeps over a km bracket or with counter-rotating fans
- [A live app flight needs a fresh PX4, parked at the scenario's pitch](lessons/live-app-needs-fresh-px4-per-flight.md):
  reboot-only IMU filter params, re-parking under a running EKF and a second flight's roll bias gave 17-21 m drift
- [The default ULog cannot identify the nose lift](lessons/ulog-default-profile-misses-nose-lift.md): outputs at
  10 Hz, `nose_lift_output` not logged, no fan speed anywhere

## Inbox

`inbox/` holds unprocessed notes; see [open items](inbox/open-items.md).

---

# Current Project State

## Current Goal

Fly ATLAS indoors in Stabilized mode: rotate from the parked pitch to the hover pitch, hover, land at the hover
pitch, rotate back; first in SITL, then HITL on the Pixhawk 6X Pro, then the real aircraft.

## Current Phase

**The real aircraft is V3** (user, 2026-10-06): nine fans, three nose fans on motors 7-9, as the board is configured
(`airframes/atlas_v3_30_jets_57.5_85_85_park-10.json`, park -10, hover 8.5 deg, pushed 2 Oct by
`results/board_params/push_jets_park10.py`); the board's parameters are the truth, see
[NL_AUTO note](experiments/2026-10-06-auto-hop-firmware.md) for the 6 Oct readout. Earlier (to 2 Oct) the brain
called the flying aircraft ATLAS_09B / V4 (ten fans, 24 deg hover); the notes below from September are about that
configuration. Bench runs of the
firmware nose lift on the real aircraft (two nose fans, props on) with a live dashboard that logs every flight arm
to disarm ([telemetry dashboard](architecture/telemetry-dashboard.md)). State at the end of 2026-09-23
([fan bursts note](experiments/2026-09-23-nose-lift-fan-bursts.md)):

- The fans burst off <-> full at ~4 Hz with the G4 gains: the rate loop amplifies the legs' 5-10 Hz rocking. Gentle
  gains raised smoothly but over-sped and, cancelled mid-rise, tipped the nose back to 45-48 deg; G4 restored.
- The hold check could never pass on the rocking frame (no Holding, no handover, no lowering fade). The board now
  runs `~/firmware_backups/FLASH_nearest_board_holdfix_ceiling.px4`, which fixes that and adds NL_CEIL: above target +
  ceiling the nose is brought back like the lowering. SITL: smooth 12 deg balance, never above 14.
- What the board ran was a 22 Sep 22:49 PDT build not in git ([board firmware](experiments/2026-09-23-nose-lift-fan-bursts.md)).
  Parameters are set over the radio with `scripts/board_params.py`; flashing needs USB.
- The simulator models fan vibration (calibrated on the bench, on by default for ATLAS_09B) and the legs' rocking;
  the aircraft-like model (`scripts/nose_lift_smoothing.py --fans 0.15,1.0 --rocking 0.35`) reproduces the bursts.
- Earlier: ATLAS_OG_FLAT tuned hands-off in Stabilized; ATLAS_09B flies the full SITL sequence with yaw set A.

## Current Priorities

0. Fly the 12 deg balance on the NL_CEIL firmware (flashed 23 Sep 18:26, build Sep 23 18:15:30; parameters set and
   read back: NL_TGT 12, NL_CEIL 1, NL_KQ 0.03 / KQI 0.015, NL_K_ANG 0.4, lowering G4 0.06 / 0.03, NL_TOUT 60) on a
   charged battery. Check each log for flips, the peak (must stay <= 14), Holding reached, and what SB off does.
1. Calibrate the firmware's fan figures (NL_A0/1) from the logged lift-off command; every new gain set must pass
   the SITL balance and mid-rise-cancel scenarios on the aircraft-like model before it flies.
2. Pull a flight's ULog over USB (full-rate gyro) to measure the rocking and the fans' real response; balance the fans.
3. Make the app's export carry the board's NL gains (it still writes 0.02/0.012 and no lowering gains).
4. Measure km on a thrust stand; counter-rotating fan pairs would remove the yaw-authority limit.

## Important Constraints

- The repo runs from WSL Ubuntu-24.04 on Windows or natively on macOS ([how to run](architecture/how-to-run.md)); port 8080 / PX4 instance 0 is the interactive app, never kill a session
  you did not start. Batch and tuning use instances 1 to 9.
- Structural frame FRD, CG in `mass.cg`; PX4's level is the hover frame (`hover_pitch_deg`, SENS_BOARD_Y_OFF).
- The simulation loop must never block on MAVLink in lockstep; scenario phases are non-blocking state machines.
- `run_once` results must stay JSON-serialisable; metrics are the contract for studies and the Tuning tab.

## Open Questions

- Which reaction-torque coefficient is real (0.002 to 0.01 span the difference between flyable and not).
- Which rotor geometry is the built aircraft: Fusion-measured (OG) or nominal (09B); they differ by up to 10 cm.
- Indoor position source for the real flights: pilot only, H-Flow, or external vision.


## Agent B research update, 2026-10-05

- [Unchanged-design library fit screen](experiments/2026-10-05-library-fit-screen.md): Rhino exterior identified; ambiguous open exterior surfaces; four UIUC sections screened, none approved as unchanged-skin replacement; S0 partial pending authoritative geometry and installed-flow evidence.

- [Whole-body lifting-surface library search](experiments/2026-10-05-wholebody-foil-screen.md): user clarified entire body+wings; 1,658 library profiles over 14 cuts; closest regional skin references identified, none approved under exact shape preservation; S0 partial, existing custom sections delivered.

- [User-selected whole-body references](decisions/2026-10-05-user-selected-wholebody-references.md): E908 centrebody, Gottingen 5K transition, SC(2)-0403 outer wing selected for Agent A integration assessment; immutable design and S0 partial retained.


## Corrected STEP review pending user review

- Authoritative input: `blender/atlas_v1.stp`. Fresh review: `results/reviews/20261005-atlas-v1-step/REVIEW.md`. 15 section positions, 1,658 UIUC entries plus 20 PDAS tables. No current selected profiles and no new Agent A handoff; user review required. Prior Rhino instructions remain superseded.

- Corrected STEP blocker register: `results/reviews/20261005-atlas-v1-step/blockers/BLOCKERS.md`. Units/tolerance confirmed; shell-role, opening, nose/scale and operating-case clarification pending. No new handoff.

- Corrected STEP user confirmations: central A is pilot canopy (retain), nose +Y, forward-flight assessment. `results/reviews/20261005-atlas-v1-step/blockers/confirmed-answers/UPDATE.md`. Scale, speed and opening/interface information pending; no handoff.


## Confirmed full-scale operating envelope

2026-10-05T21:46:34.976810+00:00: Source atlas_v1.stp preserved; canopy retained; nose +Y; actual span 3.805 m; forward flight 0–150 km/h; intentional head/tail-light gaps preserved. Rankings unchanged. Re/Mach calculated, no XFOIL polars. Bounded-task input questions resolved; user review before any handoff. See `results/reviews/20261005-atlas-v1-step/blockers/operating-envelope/UPDATE.md`.


- [XFOIL isolated-reference screening](experiments/2026-10-05-atlas-v1-xfoil-screen.md): official6.99 local build, four library sections, Re/Mach by local chords, documented convergence failures and transition/panel sensitivity. No installed-aircraft validation or Agent A handoff.

- [XFOIL numeric diagnosis](experiments/2026-10-05-atlas-v1-xfoil-numeric-followup.md):34/36 targeted solutions, usable low-angle evidence, panel/transition sensitivity isolated; no whole-aircraft validation or handoff.

- [SC primary-coordinate audit](experiments/2026-10-05-sc-primary-coordinate-audit.md):NASA TP2969 leading-edge ordinates match; natural-transition uncertainty remains, common-reference incidence clarified.
- [Full-scale powered inputs](research/2026-10-05-atlas-v1-powered-inputs.md):fans-on confirmed; full-scale fan geometry mapping and operating loading still needed; no small-prototype transfer or CFD.

- [Nine-fan full-scale topology](research/2026-10-05-atlas-v1-nine-fan-layout.md):user confirms3 rear-right+3 rear-left+3 front; small-scale version/force-axis distinctions recorded; Schuebeler195mm EDF retained as illustrative class only. No full-scale coordinates/performance transfer or handoff.

- [2026-10-05 ATLAS v1 EDF provisional placement](research/2026-10-05-atlas-v1-edf-provisional-placement.md): PROPOSED NOT ACCEPTED; nine centers/axes, local overlays, envelope checks; front direct flow paths blocked, real hardware envelope unknown.

- [Correct-STEP foil reference handover](research/2026-10-05-atlas-v1-foil-reference-handover.md): E908/GOE5K/SC0402 recommended, YS915 alternate; specs and byte-identical DAT delivered for versioned candidate review, cruise60km/h. Not final user selection or aerodynamic validation.


- [Current ATLAS v1 physics handoff (2026-10-06)](../results/handoff/T20261005-atlas-v1-foil-references/from_B/handoff.md): next-agent entrypoint; source/frames, reused foil references, unresolved nine-fan installation/mass/aero inputs, bounded model/static/trim/physics/SITL checks. Handoff complete; physics model not ready. Supersedes historical foil-only next scope for this task.


## Team runtime — 2026-10-06

- [Project Factory orchestration](architecture/team-factory.md): persistent goals/tasks/events, existing role sources, adapters, worktree safety, dashboard and current live integration blockers. This adds software coordination only; aircraft source corrections and simulation/hardware gates above remain in force.

- [Current immutable-design team goal](architecture/immutable-design-team-goal.md): user confirms bigger-version source `blender/atlas_v1.stp`, nose CAD -Y; untouched exterior, matching-foil research then sourced masses and validated six-axis cruise/SITL evidence. Historical +Y fits and shape-changing candidates are not valid for this run.
- research/2026-10-07-atlas-v1-sidewind-patent-search.md: sidewind patents for the anhedral wing (Northrop US2406506A drooped tips behind the CG, US2412646A split tip rudders, Ullman EDF differential thrust); strip-model Cl_beta +0.005/deg (anhedral, destabilising), Cn_beta +0.0017/deg
- experiments/2026-10-07-atlas-v1-lateral-controller-and-masses.md: sideslip-feedback controller (elevons + tip drag rudders + thrust split) holds M001/M002b through a 10 deg side gust; flight packs placed (M002, M002b); elevon trim budget is the limit; HOVER REGRESSION since the 10-05 nose flip; Blender review scene; Aristotelis CFD bundles b02-b04
- experiments/2026-10-07-atlas-v1-aristotelis-b02-sidewind.md: first cluster CFD library (35 attitudes, 60 km/h): clean body pitch-unstable (SM -22 %), CL 0.14-0.26 (needs 89 km/h at 46 kg), weak weathercock, no dihedral effect; CFD free flights: 5 m/s crosswind held by the ideal pilot, 10 m/s + turbulence tumbles
