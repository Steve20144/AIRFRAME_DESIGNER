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
- [Design knobs](architecture/design-knobs.md): jetfoil angles, front jets, battery/CG as named values that move
  rotors and linked CAD parts; Geometry tab card, `knobs.*` paths for sweeps (branch atlas-v5-design).
- [ATLAS V3 small draft](architecture/atlas-v3-small.md): SMALL_SCALE_V3.step as an airframe; what is CAD, what is
  assumed; flies Stabilized at 10.5 deg in SITL but the heading turns 52 deg in 12 s.
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
  angles win, nose tilt barely matters; mass.resolve() ignored CAD bodies (fixed)

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

The aircraft that flies is ATLAS_09B (V4, no vibration damper; the Load menu shows only it). Bench runs of the
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

- The repo runs only from WSL Ubuntu-24.04; port 8080 / PX4 instance 0 is the interactive app, never kill a session
  you did not start. Batch and tuning use instances 1 to 9.
- Structural frame FRD, CG in `mass.cg`; PX4's level is the hover frame (`hover_pitch_deg`, SENS_BOARD_Y_OFF).
- The simulation loop must never block on MAVLink in lockstep; scenario phases are non-blocking state machines.
- `run_once` results must stay JSON-serialisable; metrics are the contract for studies and the Tuning tab.

## Open Questions

- Which reaction-torque coefficient is real (0.002 to 0.01 span the difference between flyable and not).
- Which rotor geometry is the built aircraft: Fusion-measured (OG) or nominal (09B); they differ by up to 10 cm.
- Indoor position source for the real flights: pilot only, H-Flow, or external vision.
