# Lesson: a live app flight matches headless only on a freshly booted PX4, parked at the scenario's pitch

Found 2026-09-26 flying `v3_stab_nolift` on ATLAS_V3_SMALL_GRADED in the interactive app (macOS). Headless: drift
0.3-0.6 m. Live: 17-21 m, roll error 2.1-2.5 deg, touchdown up to 1.66 m/s. Three separate causes:

1. **Reboot-only parameters.** The app boots a stock PX4 and pushes the airframe export at scenario start;
   `IMU_GYRO_CUTOFF`, `IMU_DGYRO_CUTOFF`, `IMU_INTEG_RATE` need a reboot (`rebootRequired` in parameters.json), so the
   first live flight ran stock gyro filtering. PX4 saves the pushed values; the next boot has them.
2. **Re-parking under a running estimator.** A scenario with an `attitude` block re-solves the legs and `sim.reset`
   puts the aircraft down at the new pitch (4 -> 26.9 deg here) while EKF2 keeps running. Fix: launch the app on the
   airframe already parked at the scenario's pitch (`sc.apply_attitude(af)` saved), so the reset changes nothing.
3. **A second flight in the same PX4 session.** After a flight the reset teleports the aircraft back to the origin;
   EKF2 then carries a roll bias (`roll_est_bias_deg` 2.4 vs -0.02 headless) and the hover slides sideways.

Rule: for a flight meant to compare with headless results, restart the app (fresh PX4) before each flight, on the
parked airframe. Check `roll_est_bias_deg` / `pitch_est_bias_deg` in the result: near 0 means the estimator was clean.
The physics rate (app 1000 Hz vs batch 500 Hz) and real-time pacing do not matter (headless at 1000 Hz: 0.6 m).

Also: the 3D view camera is Static by default and loses the aircraft above ~2 m; the Camera button cycles to Follow.

## "Battery unhealthy" blocks arming in long live sessions (fixed 2026-09-30)

After some minutes of a live app session (seen at 449 s sim time) PX4 SITL logged "Preflight Fail: Battery unhealthy"
and `wait_ready` timed out: commander flags the battery unhealthy when battery_status is older than 5 s (the
battery_simulator's status went stale; not a flat battery, it resets to 100 % while disarmed). The simulation has no
battery model, so `Airframe.px4_params_sitl()` now sets `CBRK_SUPPLY_CHK = 894281` (supply checks bypassed, no reboot
needed; pushed live at scenario start, seeded in batch runs). SITL only: the HITL / board export keeps the checks.

## HITL: reboot the board after loading an airframe with another parked pitch (2026-10-01)

Loading the -16 deg parked airframe while the board ran (estimator had settled at 23.85) left EKF2 with a lateral
accel bias at its 0.4 m/s^2 limit: "High Accelerometer Bias", and Stabilized dropped out of "can arm" (manual / acro
only). Rebooting the board with the aircraft already resting at the new pitch: bias 0.004, stable over 2 min,
Stabilized armable again. Same rule as SITL: re-park first, then a fresh estimator.

## Automatic since 2026-10-01: every live scenario starts on a fresh PX4

`POST /api/scenario/start` (Fly live) now: params pushed -> vehicle reset to the scenario's rest -> `conn.fresh_px4()`
(SITL: our PX4 relaunched, up to 3 tries of 25 s because rcS sometimes hangs after the GCS MAVLink instance and
14540+i never opens; HITL: board reboot) -> params re-checked -> flight. `fresh_px4: false` in the body skips it.
The app's default-mode (Takeoff) selection is held off meanwhile. Verified: two live flights of the -16 batch back to
back both OK (drift 0.57 / 1.88 m, touchdown 0.16). Do not press Reset during a live flight: it restarts EKF2 in the
air (seen: landing never touched down).

## Ground bounce when armed on the legs: MC_AIRMODE (2026-10-01)

-16 deg rotate batch, armed at idle on the legs for 4 s: pitch rocked up to 54 deg/s (2 of 3 seeds), lift-off wobble
5.8 deg. MC_AIRMODE 1 keeps full roll/pitch authority and an unreset rate integrator at idle on the ground, which
fights the spring legs and the nose-lift hold. MC_AIRMODE 0 plus arm phase `until: {armed: true}` (climb as soon as
armed): 6.8 deg/s, wobble 2.05, all seeds. Board set to MC_AIRMODE 0 on 2026-10-01 (verified after a reboot); the v3_upload JSON still says 1.

## Saved magnetic declination in the reused SITL instance (2026-10-01)

The app's PX4 (instance 0, `~/.airframe_designer/px4_instance_0`) had EKF2_MAG_DECL 5.04 saved from earlier GPS
sessions (EKF2_DECL_TYPE 3 = take from WMM via GPS and save). The simulated field has ZERO declination, so in a
GPS-less (H-FLOW) live flight every compass reading was 5 deg off: heading error 4.7 deg already at boot, roll estimate
-1.1 deg, 34 m drift, yaw drift 17 deg. Headless runs boot a fresh rootfs (declination 0) and never saw it. Fix:
`Airframe.px4_params_sitl` pins EKF2_DECL_TYPE 0 and EKF2_MAG_DECL 0 (test_sitl_export_pins_zero_declination).
Live V3_30 tuned after the fix: yaw drift 2.9 deg, roll err 0.41, drift 22 m (headless 1.0 / 0.08 / 8.4 m).
Still open: in the live run PX4's pitch estimate steps from -0.01 to -0.23 deg off truth at the moment it arms on the
legs (nose fans holding 27 deg) and keeps that offset in the hover; headless stays near -0.07. Not explained yet.
HITL boards need the same two parameters set for HITL flights (they keep their own declination otherwise).

## Fly live: relaunch again when EKF2 comes up starved; Batch list reloads (2026-10-01 19:05)

Now and then a relaunched SITL PX4 boots with EKF2 starved ("ekf2 missing data", "No valid attitude estimate",
can arm only manual|acro) and wait_ready timed out after 45 s. `conn.estimator_up(since, wait=20)` checks the
arming summaries after each fresh start; `_scenario_start` relaunches up to 3 times (`estimator_tries`) and returns
a clear 503 if it never starts. The Batch and Tuning tabs now reload the scenario list each time they open (a
scenario saved after the page loaded used to be missing until a page reload). Verified: Fly live from the Batch tab
on the 1851 poshold scenario, ok.

## Root cause of the live drift: PX4 booted at a large lockstep time (2026-10-01 20:00)

Symptom: live SITL flights drifted backwards in Stabilized / Altitude (12-45 m a minute vs 3-6 m headless): PX4's
estimate lost ~5 % of the on-ground nose-lift rotation (a ~1 deg step ~5 s into rotate_up) and then held a steady
0.3-0.4 deg nose-down est-vs-truth offset. Ruled out: pacing, 1000 Hz physics, multi-EKF / logging / gyro params,
pre-flight settling, fan vibration, home, the reused instance's saved params (a clean seeded boot alone did not fix
it). Cause: the app's simulator clock keeps running across flights, and in lockstep a freshly booted PX4 takes its
clock from the first HIL_SENSOR. Reproduced headless by starting the clock late: 0 s -> 0.11 deg / 6 m, 100 s ->
0.45 deg / 22 m, 400 s -> 1.66 deg / 77 m. Fix: `Simulator.restart_clock()` before every SITL PX4 launch
(connect_sitl). Verified live, two Altitude > Position > Altitude flights back to back: rotate tracked 100.3 / 100.1 %,
est-truth 0.00 deg, Altitude drift 0.3-0.7 m (was 1.1-3.2), Position 0.02-0.07 m (was 0.36-0.40), touchdown 0.24.
Kept: live relaunch boots a clean, seeded working dir (`seed_params`, like run_once). HITL is NOT affected by the
clock (the board timestamps on receipt); its est offset (board log "Found 0 compass", 2.4 deg off at rest) is open.
