# 2026-10-06 15:37 V3: first automatic takeoff (NL_AUTO), killed at 1.5 m

Dashboard run `results/telemetry_runs/run_20261006_153712.json` / `log_20261006_153712.csv` (radio, 10 Hz);
ULog pulled over USB: `results/board_logs/ulog_320_20261006_223648.ulg` (2.8 MB, `nose_lift_output` is logged).
Video `~/Downloads/IMG_3581 2.MOV` (4K60, 212 s; the flight is at 46 to 50 s). Board: NL_AUTO 1, NL_AUTO_HOV 5, the rest default
(ALT 1.5, RAMP 2, KP 0.25, KV 0.35, MAX 0.9), MPC_THR_CURVE 0, MPC_THR_HOVER 0.592, kill on channel 5.

Timeline (s after arming): 0.3 nose rising from -13.9; 8.0 holding at 9.6 deg (target 8.5); 11.0 automatic takeoff
starts, handover to PX4; 11.0-13.0 the rear fans sit at 1100-1140 us (idle) for the whole 2 s throttle ramp;
13.1 "Takeoff detected"; 13.3-13.6 rear fans jump 1130 -> 1550 -> 1800 us within 0.5 s; 13.6-15.0 structural pitch
8 -> 20-21 deg (PX4 pitch +12 to +13, nose up against its level), roll to -6, heading 169 -> 137 deg; 15.05 "PX4
took over" (floor faded); 15.4-17.4 flying, pitch 14-21, heading 123 -> 103 -> 156 -> 177, rear fans 1500-1900 us;
17.1 "hovering at 1.2 m" (baro) while the H-FLOW range read 1.7-2.0 m (takeoff 0.46 m, so 1.5 m above the start);
17.5 kill: every motor off in 0.1 s, abort latched "kill switch", disarmed at 17.7.

## Root cause (ULog): the aircraft was in POSITION mode, not Stabilized

`vehicle_status.nav_state` = POSCTL from arming to the kill (no flight-mode channel: RC_MAP_FLTMODE 0, the board
stays in whatever mode it was left in; the dashboard showed "Position" at arming). In Position mode the throttle
stick is a climb-rate command, not thrust, and the position controller flies the attitude:
- the override's 2 s ramp (stick 0 -> 0.52) produced zero thrust (`vehicle_thrust_setpoint` 0.00) because a
  stick below mid in Position mode on the ground is "do not take off"; at stick 0.47 PX4 declared "Takeoff
  detected" and the position controller's own takeoff ramp opened the thrust: the step;
- the stick at 0.6 to 0.8 then meant "climb 0.3 to 1 m/s" (`vz_sp` -0.5 to -1.1), the module's height loop and
  PX4's altitude loop chained: 1.7 m above the start in 4 s;
- `vehicle_attitude_setpoint` pitch of +7 to +12 deg (nose up) and the heading swings came from the position
  controller holding x/y on the H-FLOW against the forward push; the sticks were centred throughout (RC 1500/1500/
  1001/1500), so none of it was the pilot.
Video (46 to 50 s): lifts, slides left and banks within a second, 2 m up at 48 s over the people's heads, kill,
drops. Same picture.

## Fixes in the firmware (built and SITL-passed 6 Oct evening, not yet flashed)

1. At the hold with NL_AUTO the module commands Stabilized itself (DO_SET_MODE main mode 7) and starts the takeoff
   only once `nav_state` reads STAB; if it does not within the wait plus 2 s, "NL_AUTO refused, not in
   Stabilized" and the pilot flies. SITL: armed in Position (`fw_auto_hop_posmode`) flies the same profile as
   armed in Stabilized.
2. Feed-forward is a stick position: mid stick (0.5) with MPC_THR_CURVE 0 or 2, MPC_THR_HOVER only with curve 1.
3. `NL_AUTO_SLEW` 0.3 per second on the automatic throttle (ramp and cut excluded). V3 SITL peak 2.62 -> 2.43 m.

## Findings from the dashboard log (before the ULog)

1. **No ramp on the aircraft.** PX4 commanded the rear fans idle for the 2 s of the override's ramp and then stepped
   to 0.65+ thrust at "Takeoff detected". In SITL the same ramp was smooth. Cause not yet shown (the ULog has
   manual_control_setpoint.throttle and vehicle_thrust_setpoint; pull it over USB).
2. **Feed-forward 14 % too high.** With MPC_THR_CURVE 0 the stick is rescaled so mid stick = the hover thrust;
   the override sent MPC_THR_HOVER (0.592) *as a stick value*, which maps to ~0.67 thrust. The right feed-forward
   stick is 0.5 (NL_AUTO_THR semantics must be "stick", default 0.5 under curve 0 / 2).
3. **Height loop too hot for a lagging barometer under fan wash**: rear fans up to 1900 us (0.9 throttle) while
   climbing; the baro read 1.2 m when the rangefinder read 1.5 m above the start. Needs a throttle slew limit,
   lower gains, lower NL_AUTO_MAX; the climb reached the commanded 1.5 m above the start in 4 s, which is what
   "so high" was.
4. **Pitch ran 12 deg nose-up and PX4 never brought it back** during the handover: the nose fans are floored at the
   hold command (handover design) so PX4 cannot lower the nose, while the rear jets at high thrust pitch the nose
   up; after the fade the pitch stayed 10-13 deg up (authority/forward push). Heading swung 70 deg in 3 s (the
   known same-spin yaw limit). That is the "not stable".
5. Kill switch: engaged 17.49, "motors stopped" 17.58, disarmed 17.69. As designed.

## H-FLOW health in the flight (ULog, EKF aid sources and flags)

- Rangefinder: online all along, signal quality 100, 0.40 m on the legs -> 1.92 m at the kill, fused as the height
  reference (EKF2_HGT_REF 2, cs_rng_hgt 100 %, range kinematically consistent 100 %). Oddity: local_position
  dist_bottom_valid stayed 0 although dist_bottom tracked the sensor.
- Optical flow: fused 100 % on the legs and in the hover, innovation test ratio 0.03 to 0.16 (rejection is 1.0),
  never rejected. BUT during the climb (144.3 to 146.5 s) only 20 % of the samples fused: cs_opt_flow dropped,
  cs_inertial_dead_reckoning on 144.5 to 146.4 s, cs_gnd_effect on from 144.5. For two seconds after lift-off the
  EKF had no velocity aiding, and that is exactly when the position controller (Position mode) slid the aircraft
  left. Flow quality itself is not in this log (vehicle_optical_flow not in the default logging profile).
- Compass: cs_mag_field_disturbed 133 to 146 s (fans on), heading consistent only 58 % of the log, "heading
  estimate not stable" after the kill: the heading the position controller rotated its velocity by was poor.
- "Preflight Fail: Distance Sensor 0 missing" appeared on the dashboard at 15:49, twelve minutes after this flight,
  not during it: the sensor drops off at times (CAN / power); check before relying on it.
Conclusion: the sensor was online and healthy; the EKF's use of it failed in the first two seconds of the climb,
plus a disturbed compass. A Position-mode hover needs flow aiding proven through the climb, or a switch only once
cs_opt_flow is back and the heading is consistent.
