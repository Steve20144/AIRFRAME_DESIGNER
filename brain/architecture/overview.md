# System Architecture

## Overview

A Python 3.12 simulator with PX4 (v1.17) in the loop over MAVLink HIL messages, a FastAPI server with a websocket,
and a vanilla-JS three.js front end. PX4 only advances when the simulator sends sensors (lockstep), so the same code
runs a live session at real time or a headless batch as fast as PX4 allows.

## Components

```
airframe_designer/
  geometry/   mass (CG, inertia, products; resolve from items + CAD bodies), propulsion (rotors: pos, axis, duct
              axis, km, thrust curve), wings, gear (legs, stands: generate_legs, legs_for_park_pitch), body,
              cad (STEP import via OpenCascade: solids -> bodies with volume, centroid, unit inertia, cached
              meshes; placement rotation/origin/scale; body masses -> mass items), airframe (composition,
              schema migration, with_attitude, resolve_mass, PX4 export), paths (parameter-path addressing)
  aero/       strip-theory wings, rotor thrust / torque / ram drag, body drag, fastmath
  dynamics/   quaternion, per-leg contact (damping clamped to the integrator's stable range), rigid body about
              the CG; jsbsim_backend and gazebo_backend as alternative engines
  sensors/    IMU / mag / baro / GPS -> HIL_SENSOR, HIL_GPS
  px4/        link (MAVLink SITL/HITL; records ATTITUDE, ATTITUDE_TARGET, params, events), sitl (process, BSON
              param seeding, instances), connection (runtime SITL<->HITL), events, param_meta
  sim/        simulator loop (hooks), scenario (phases, scripted pilot, attitude block), metrics (time series,
              per-phase statistics), nose_lift (NoseLift / NoseLower ground hooks)
  analysis/   static hover / cruise model, geometric optimiser (no PX4)
  batch/      worker (run_once), runner (parallel over PX4 instances), objective, optimizers, study
  server/     app (routes, websocket), tuning (flight library, sweep specs, briefs), cli
ui/           index.html app.js scene.js style.css vendor/three
airframes/    schema-2 JSON (+ .params exports, meshes/);  scenarios/  studies/  results/ (generated)
scripts/      report_runs.py (HTML reports), atlas_og_from_fusion.py, build_hitl_firmware.sh
```

## Data Flow

1. Airframe JSON -> `Airframe` -> PX4 parameters (`px4_params`: CA_ROTORn_* about the CG in the hover frame,
   SENS_BOARD_Y_OFF = hover pitch, output functions, overrides). Headless: seeded into `parameters.bson` before
   boot. Live: pushed over MAVLink.
2. Each sensor step: PX4 actuator outputs -> rigid body (rotor thrust, aero, contact) -> hooks (metrics, scenario,
   nose lift) -> sensors -> HIL messages -> PX4. The scenario hook streams MANUAL_CONTROL for manual modes and
   sets modes, params and wind.
3. Metrics sample 50 Hz rows: position, velocity, hover-frame Euler angles, rates, tilt, thrust, power, lift,
   airspeed, motor utilisation, mean command, airborne flag, PX4 attitude setpoint (roll_sp ...), PX4 attitude
   estimate (roll_est ...). `summary()` gives per-phase statistics including `*_err_rms_deg` (vs setpoint) and
   `*_est_bias_deg` (estimate vs truth).
4. Studies: `run_study` builds tasks per candidate, `run_many` spreads them over PX4 instances, trials.jsonl holds
   values, score, metrics and (with `timeseries: true`) a time-series path. The Tuning tab reads these directories.

## Interfaces

- CAD routes: `POST /api/cad/import` (STEP upload -> airframes/cad/, bodies attached), `GET /api/cad/mesh`
  (meshes in the structural frame for the 3D view), `GET /api/cad/totals`.
- REST + websocket in `server/app.py`; the Tuning tab's routes: `/api/tuning/run` (attempt, `live: true` for the
  app's PX4), `/api/tuning/sweep` (grid study), `/api/tuning/runs`, `/api/tuning/run/<id>`, `/api/tuning/sweeps`,
  `/api/tuning/sweep/<name>/trial/<k>`, `/api/tuning/defaults`.
- MCP server (`airframe_designer mcp`) exposes edit, simulate and study tools to other assistants.
- CLI subcommands: `ui`, `run`, `batch`, `study`, `compare`, `analyse`, `optimise`, `export`, `vehicle`, `mcp`.

## External Dependencies

- cadquery-ocp (OpenCascade) for STEP import, installed in the WSL venv 2026-09-21.
- PX4-Autopilot v1.17 SITL build (`~/PX4-Autopilot/build/px4_sitl_default`), a HITL firmware build for the
  Pixhawk 6X Pro (`~/PX4-hitl`, `scripts/build_hitl_firmware.sh`); pymavlink; FastAPI/uvicorn; numpy; JSBSim
  (optional engine); Gazebo Harmonic (optional, prototype backend).

## Constraints

- Frames: structural FRD with the CG explicit; all force models take positions relative to the CG; PX4 sees the
  hover frame. Legs and rotors are edited in the structural frame.
- Lockstep: never block on MAVLink replies inside the loop; phases are state machines polled every step.
- Ground contact damping is clamped per step, otherwise a light airframe jitters and PX4 refuses to arm.
- Physics cost matters (~85 us per sub-step); use `aero/fastmath` instead of `np.cross` on 3-vectors.
