# Decision: the nose lift runs on the flight controller, with a latching kill switch

Date: 2026-09-22. Status: built and passing in SITL; not yet flashed, not yet run in HITL or on the aircraft.

## Why

The simulator's nose lift (`sim/nose_lift.py`) only exists in the app: on the physical aircraft SB did nothing, and
PX4's kill switch could not stop it (it drives the simulated rotors while PX4 is disarmed). Agreed with the user:
port it to the Pixhawk 6X Pro, arm first and then SB, landing rotation later, the kill switch is the momentary push
button on the small Radiomaster.

## What

- `firmware/px4_ext`: out-of-tree PX4 module `nose_lift` (+ messages `NoseLiftOutput`, `NoseLiftFeedback`), the
  Python loop ported one to one (balance feed-forward about the rear feet, pitch-rate loop, roll/yaw split).
- `firmware/patches`: ControlAllocator applies the module's output just before `actuator_motors` is published
  (override mask, floor mask, stale output with an override = all motors stopped); `rc.mc_apps` starts the module.
  Because this is upstream of the output driver, PX4's kill and disarm cut the motors in every state.
- `scripts/build_nose_lift_firmware.sh sitl|board`: dedicated worktree `~/PX4-nl` (v1.17.0, patches stay applied).
  Board image `px4_fmu-v6x_multicopter` with pwm_out_sim: 96.3 % flash.
- States: Disarmed and Parked hold every motor stopped (held already while disarmed, see below); SB edge starts
  Ramping; Holding; throttle above `NL_HO_THR` starts Handover (floor until PX4 reaches it); Flying is passive.
  SB off, radio lost, hold or lift timeout = Lowering (nose back to where it started, then disarm). Kill, roll,
  overshoot, liftoff, attitude lost = Aborted (motors stopped, force disarm, latched until disarm).
- App: `design.nose_lift.executor: "firmware"` (Flight tab "Runs on") adds the `NL_*` parameters to the export
  (computed from the same RigidBody geometry, `tests/test_nose_lift_firmware.py`), `COM_KILL_DISARM 0`,
  `COM_DISARM_PRFLT 120`; the app's SB watcher and manual lift stand down; the card shows the module's state from
  DEBUG_VECT "NLIFT". Scenarios gained an `rc` phase (RC_CHANNELS_OVERRIDE -> input_rc) and a `design` block.

## Findings on the way

- `actuator_motors` is normalised thrust; PX4's output stage applies `THR_MDL_FAC` (1.0 on 09B) after the allocator.
  Publishing motor commands there square-rooted them twice: 12.6 deg/s and a 2.3 deg overshoot instead of 3 deg/s.
  The module now publishes `f c^2 + (1 - f) c` for the command c it wants.
- Arming nose-down in airmode sent motors 3 to 10 to up to 100 % for one control cycle before the module switched
  to Parked. The override is now held while disarmed, so it is in place at the instant of arming (probe: 0.0 on
  every motor from boot to SB).
- Stock PX4 disarms 11 s after arming without a takeoff (`COM_DISARM_PRFLT` 10 + spool-up): mid-hold.
- SITL needs `RC_CHAN_CNT` > 0 before `rc_update` accepts override RC as calibrated.
- Transmitter lost on the ground: PX4 enters Hold for 5 s; the module lowers the nose and disarms.

## Evidence (SITL, ATLAS_09B, `results/fw_nose_lift/`)

`fw_nose_lift_takeoff`: 4 -> 24 deg at about 3.5 deg/s like the Python lift, handover "PX4 took over", hover
2.75 m, touchdown 0.40 m/s. `fw_nose_lift_kill`: kill while rising, holding and in the handover, all motors 0 within
0.048 s, abort latched, releasing the switch restarts nothing. `fw_nose_lift_cancel`: SB off lowers to 4.5 deg and
disarms; radio loss lowers and disarms.

## On the board (HITL, same day)

Flashed from the app's new Flash tab (the board had been on PX4 1.18.0 beta; downgraded to this 1.17.0 image, the
airframe parameters survived). Kill on SE, the T8L's back latching switch (channel 5, `RC_KILLSWITCH_TH` -0.5), set
with the new Controller tab; arm on channel 8 (`RC_ARMSWITCH_TH` 0.25); SB on channel 7 starts the lift. Kill test
card (`px4/killtest.py`, the board's own HIL_ACTUATOR_CONTROLS): a kill while the nose was rising took the motors
from 0.91 to all off in 17 ms and the outputs to disarmed in 27 ms; nothing came back after the release. The user
accepted the results.

## On the aircraft (2026-09-22/23)

Two front fans, props on: see [aircraft runs](../experiments/2026-09-23-aircraft-nose-lift-bench-runs.md). Firmware
fixes flashed: the clock is read after the uORB copies (a false "attitude lost"), and "left the ground" also needs
the low-passed baro to rise `NL_LIFT_DZ` (fan vibration drifted the EKF height). The sim-tuned gains failed (15 deg/s
rise coasting to 43 deg; the lowering slammed the fans): G4 set on the board (`NL_KQ` 0.10, `NL_KQI` 0.05,
`NL_LOW_KQ` 0.06, `NL_LOW_KQI` 0.03). The app's export still writes `NL_KQ` 0.02 / `NL_KQI` 0.012 and does not export
the lowering gains; do not re-export to the board until `design.nose_lift` carries G4. `COM_DISARM_PRFLT` is -1 on
the board (the export writes 120). New, off by default: `NL_Q_LPF` (a rate low-pass that SITL rejected).

## Open

Kill tests while holding and in the handover on the board; a flight-mode switch (channel 5 now carries the kill;
with none mapped PX4 boots in Position and wants a GPS fix); props-off bench with real ESCs. Landing rotation not
ported. The feed-forward uses modelled thrust; on hardware the loop and `NL_MAX_CMD` carry it until measured.
