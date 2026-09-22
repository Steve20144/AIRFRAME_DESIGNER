# Experiment: firmware nose lift, park pitch +4 down to -8 deg (ATLAS_09B, SITL)

Date: 2026-09-22. Scripts: `scripts/fw_nose_lift_sweep_tasks.py`, `scripts/fw_nose_lift_sweep_analyse.py`; output in `results/fw_nose_lift_sweep/` (summary.json; run1 = first
firmware, run2 = liftoff fix, top level = final). Each park angle re-stands the aircraft (scenario attitude block,
hover 24) and runs `fw_nose_lift_takeoff`, `_kill` and `_cancel` on the nose_lift PX4 module.

## Static margin (lifting fans' thrust fraction that balances the nose on the rear feet at the park)

| park | +4 | +2 | 0 | -2 | -4 | -6 | -8 |
|---|---|---|---|---|---|---|---|
| balance | 0.848 | 0.859 | 0.871 | 0.883 | 0.895 | 0.908 | 0.922 |
| margin | 15.2 % | 14.1 % | 12.9 % | 11.7 % | 10.5 % | 9.2 % | 7.8 % |

## Final results (21 of 21 pass)

| park | lift s | peak fan cmd | handover dip deg | liftoff pitch | kill off ms (rise/hold/handover) | cancel end pitch |
|---|---|---|---|---|---|---|
| +4 | 7.2 | 0.966 | 0.5 | 24.3 | 40 / 32 / 52 | 4.05 |
| +2 | 7.8 | 0.975 | 0.5 | 24.0 | 32 / 44 / 44 | 2.04 |
| 0 | 8.5 | 0.984 | 0.7 | 23.6 | 44 / 48 / 36 | 0.02 |
| -2 | 9.2 | 0.991 | 1.2 | 23.4 | 40 / 36 / 40 | -1.99 |
| -4 | 9.9 | 0.995 | 1.7 | 23.2 | 44 / 48 / 52 | -4.00 |
| -6 | 10.5 | 0.999 | 2.1 | 23.0 | 56 / 52 / 52 | -6.01 |
| -8 | 11.2 | 1.000 | 2.5 | 22.9 | 44 / 40 / 56 | -8.02 |

Everywhere: overshoot none (holds at 23.6 to 23.8), hover pitch RMS under 0.14 deg, touchdown 0.50 to 0.54 m/s, the
feet slide 0.35 to 0.38 m during the lift (model friction; independent of the angle), lowering peaks 3.4 to 3.6 deg/s.

## Reading

- The lift works to -8 in the model, but the headroom goes: the fans peak at 97 % at +4 and are pinned at 100 % at
  -6 and -8 during the breakaway. Real fans giving a few percent less than modelled (thrust estimate, battery sag)
  would stall the lift there (the module then aborts with "motors cannot hold" before the nose moves, or lowers
  after its timeout). Practical limit about -4 until the fans' thrust is measured.
- Handover dip grows with the steeper stand: the nose sags on the legs while PX4 takes the throttle (21.7 deg at -8),
  recovering within 0.8 s after liftoff. Likely the re-solved stand (rear feet closer under the CG); not proven.
- The kill is phase- and angle-independent: all motors off 32 to 56 ms after the switch frame (includes the 20 ms
  scripted RC frame), outputs disarmed every time, nothing restarts.

## Fixed during the sweep (firmware)

- False "left the ground" abort: the EKF height drifted more than NL_LIFT_DZ after two kills and re-arms (true
  height +0.14 m), the module cut the motors from the hold and the nose fell 25 deg in 0.5 s. Now needs the height
  gain and a climb over 0.3 m/s, and follows estimator height resets.
- Cancel lowering dipped at 8 deg/s at the start (full-rate demand through the stiffer lowering gain) and dropped the
  last 2 deg at 10 deg/s (fade started 2 deg above the leg). Now eased in over 1 s and fades only after 0.6 s within
  0.5 deg of the start pitch: 3.4 to 3.6 deg/s peak, 4 to 5 s longer.
- One run (cancel at -2) failed once at arming with "Battery unhealthy" from SITL; did not repeat.
