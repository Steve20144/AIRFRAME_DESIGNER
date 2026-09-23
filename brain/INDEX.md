# Project Brain Index

This file is the entry point to the project's persistent knowledge base.

Claude should read this file before exploring the rest of the brain, then open at most the three to five notes the
current task needs.

---

## Product

- [Vision](product/vision.md): a PX4-in-the-loop design simulator for the ATLAS ten-fan nose-up hoverer.
- [Requirements](product/requirements.md): what a flight test must show before hardware.
- [User Experience](product/user-experience.md): the app's tabs and the indoor flight flow.

## Architecture

- [System Overview](architecture/overview.md): segments, frames, data flow, PX4 coupling.
- [How to run](architecture/how-to-run.md): WSL environment, commands, ports, tests, conventions.
- [Scenario runner and ground sequences](architecture/scenarios-and-ground-sequences.md): phases, the scripted pilot,
  the attitude block, nose lift and nose lower.
- [Tuning tab](architecture/tuning-tab.md): attempts, sweeps, flight library, charts, API.
- [Telemetry radio and throttle dashboard](architecture/telemetry-dashboard.md): SiK on TELEM3, the live dashboard,
  per-run records, log download over the radio, PX4 stream quirks.

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

## Inbox

`inbox/` holds unprocessed notes; see [open items](inbox/open-items.md).

---

# Current Project State

## Current Goal

Fly ATLAS indoors in Stabilized mode: rotate from the parked pitch to the hover pitch, hover, land at the hover
pitch, rotate back; first in SITL, then HITL on the Pixhawk 6X Pro, then the real aircraft.

## Current Phase

SITL tuning complete for two models; piloted HITL started 2026-09-21 (see the HITL session note). Bench runs of the
firmware nose lift on the real aircraft started 2026-09-22 (two nose fans, props on, see the aircraft-runs note):
the nose rises from a +2 deg park with the G4 gains, but the fans' slow response makes the rise and the lowering
jerky, and the model does not reproduce it yet.

- ATLAS_OG with flat nose brackets (`airframes/atlas_og_flat.json`): tuned and confirmed hands-off in Stabilized.
- ATLAS_09B (imported from upstream, corrected, `airframes/atlas_09b.json`): flies the full sequence with the softened
  yaw loop (set A); 0.12 deg roll bias and 1.8 m hands-off drift in 12 s remain, a yaw-authority limit the pilot trims.
- The PHASE_0_V4 STEP is integrated (CAD bodies with masses, meshes in the 3D view) on ATLAS_09B, ATLAS_OG and
  ATLAS_OG_FLAT; STEP import needs OpenCascade (cadquery-ocp) in the venv.
- ATLAS_OG as built (canted +-30 nose brackets): cannot hover hands-off in Stabilized, no gain fixes it.

## Current Priorities

0. Aircraft nose lift: measure the nose fans' step response on the bench, fit the model (`tau`, `tau_down`), re-tune
   the lift gains in SITL; fix the lowering fade; make the app export carry the board's gains. Keep raises short
   (SB off by ~10 deg) until then.
1. HITL session on the board with `atlas_og_flat.params` (or `atlas_09b.params`), comparing against the SITL
   confirmation numbers in the experiments notes.
2. Measure the fans' reaction-torque coefficient on a thrust stand; it decides ATLAS_09B's controllability. Ask
   whether counter-rotating fan pairs are possible: they would remove the yaw-authority limit outright.
3. Decide which model represents the aircraft that will fly (OG from Fusion vs 09B from upstream).
4. Remote-switch takeoff and land sequences for Stabilized in HITL (see inbox).

## Important Constraints

- The repo runs only from WSL Ubuntu-24.04; port 8080 / PX4 instance 0 is the interactive app, never kill a session
  you did not start. Batch and tuning use instances 1 to 9.
- Structural frame FRD, CG in `mass.cg`; PX4's level is the hover frame (`hover_pitch_deg`, SENS_BOARD_Y_OFF).
- The simulation loop must never block on MAVLink in lockstep; scenario phases are non-blocking state machines.
- `run_once` results must stay JSON-serialisable; metrics are the contract for studies and the Tuning tab.

## Open Questions

- Which reaction-torque coefficient is real (0.002 to 0.01 span the difference between flyable and not).
- Which rotor geometry is the built aircraft: Fusion-measured (OG) or nominal (09B); they differ by up to 10 cm.
- Indoor position source for the real flights: pilot only, H-Flow, or external vision.
