# 2026-10-06 Automatic takeoff, hover and landing on the flight controller (NL_AUTO)

Requested by the user for the real aircraft, **V3**: the nine-fan aircraft as configured on the board on 2 Oct
(`airframes/atlas_v3_30_jets_57.5_85_85_park-10.json`: three nose fans on motors 7-9, `NL_MOT_MSK` 448, park -10,
hover 8.5 deg, `MPC_THR_HOVER` 0.592; user confirmed 2026-10-06 that the board is the truth). The nose-lift
switch starts the rotation; at the hold wait 3 s; take off and hover 1.5 m; hover 30 s; land (descend, then rotate
the nose down). The kill switch must stop every motor at any time. Nothing else on the aircraft changes.

## What was built (firmware only, `firmware/`)

- `nose_lift` module: an `AutoPhase` sub-machine (Wait, Climb, Hover, Descend, Cut, Done) on top of the existing
  states. At Holding with `NL_AUTO 1` it waits `NL_AUTO_WAIT` (3 s), then starts the handover itself and asks for
  the throttle: ramp to the hover feed-forward (`NL_AUTO_THR`, or `MPC_THR_HOVER` when 0) over `NL_AUTO_RAMP` 2 s,
  then a height loop on the 0.5 s low-passed barometer (height above the takeoff point) damped on the estimator's
  climb rate: climb at `NL_AUTO_VUP` 0.5 m/s to `NL_AUTO_ALT` 1.5 m, hover `NL_AUTO_HOV` 30 s, descend at
  `NL_AUTO_VDN` 0.3 m/s (0.2 in the last metre), touchdown = the target 0.1 m below the aircraft with the baro
  rate under 0.25 m/s for 1 s (or PX4 landed), throttle cut over 0.5 s, then the existing Lowering (nose back to
  the parked pitch, PX4 disarmed). Gains `NL_AUTO_KP` 0.25 /m, `NL_AUTO_KV` 0.35 s/m, `NL_AUTO_KI` 0.08 with the
  integral capped at 0.2 of throttle and the target never more than 0.5 m ahead of the aircraft (without that cap
  the loop wound up on the legs, leapt to 2.5 m and fell at 3 m/s).
- `NoseLiftOutput.msg` gained `throttle` (NaN = none) and `auto_phase`; patch 0003 makes `rc_update` put that
  throttle in place of the pilot's throttle stick while it is finite and fresh (200 ms). Everything downstream
  (attitude control, land detector) sees it as the pilot's; the kill switch and disarm act in the output driver as
  before. Roll, pitch and yaw stay on the pilot's sticks (Stabilized), so the pilot still covers the forward push.
- Ways out: the kill switch (every phase; the module latches Aborted, PX4 disarms); the nose-lift switch off in the
  air lands at once; the throttle stick raised above `NL_AUTO_PILOT` (1400 us raw) hands the throttle back to the
  pilot for the rest of the flight; `NL_AUTO 0` restores the old behaviour exactly.
- Not supported: `NL_FLY_HOLD 1` (the nose fans' own flight loop multiplies every throttle change through
  `NL_F_FF`; the height loop oscillated 0.2 to 0.76 of throttle in SITL). The module says so at the hold and leaves
  the throttle to the pilot. The board has `NL_FLY_HOLD 0` since 2 Oct.

## SITL (`scenarios/fw_auto_hop*.json`, results in `results/fw_auto_hop/`)

On V3 (the board's airframe, files `v3_*`): full sequence passes, hover 1.5 m above the takeoff point, touchdown
0.20 m/s, fan saturation 0.2 %, nose lowered, disarmed; kill: motors off in 40 ms, "kill switch" latched, PX4
disarmed 0.8 s later; switch off in the hover: lands, lowers, disarms. First developed and tuned on ATLAS_09B
(ten fans, 24 deg hover), where the same three pass as well:

- `fw_auto_hop`: lift 7.2 s to the hold, takeoff 3.0 s later, hover 1.5 m above the takeoff point (peak overshoot
  0.25 m), 30 s, descent, legs touch at about 0.2 m/s vertical (the runner's touchdown figure, 0.08 to 0.72 m/s,
  includes the hands-off forward drift), nose lowered, disarmed 73 s after arming. Seeds 0 and 7 pass.
- `fw_auto_hop_kill`: kill 14 s after takeoff: motors off within 50 ms, abort "kill switch" latched, PX4 disarmed
  0.7 s later; the aircraft falls from 1.5 m (4.9 m/s), which is what a kill means.
- `fw_auto_hop_switchoff`: switch off during the hover: descent at once, touchdown, lowering, disarm.
- Scenario note: with `COM_RC_IN_MODE 0` the scripted transmitter must stream before `wait_ready` (an `rc` phase
  first), or PX4 never lists Stabilized as armable.

## Board as read on 2026-10-06 14:25 (backup `params_20261006_142533_before_nl_auto.json`)

PX4 v1.17.0 release, `NL_EN` 1, `NL_FLY_HOLD` 0, `NL_TGT` 8.5, `NL_RC_CH` 7 (active low, 1300), `NL_HO_THR` 0.65,
`NL_KQ/KQI` 0.02/0.012, lowering 0.06/0.03, `NL_CEIL` 0, `COM_DISARM_LAND` 2, `COM_DISARM_PRFLT` 120, kill on
channel 5 (threshold -0.5), arm on 8, `RC_MAP_THROTTLE` 3, `MC_AIRMODE` 0. `NL_AUTO` absent (not flashed yet).

## Before the aircraft

- `COM_DISARM_LAND` must be -1 (board had 2): PX4 would otherwise disarm 2 s after the throttle cut, in the
  middle of the nose lowering, and drop the nose onto the legs. The module disarms itself after the lowering.
- `MPC_THR_HOVER` (board 0.592) is the feed-forward; the integral covers up to 0.2 of throttle either way.
- The barometer reads the fans' wash (about 0.45 m on the aircraft); the height is relative to the reading at the
  takeoff, so a steady bias cancels and a bias that changes with thrust moves the hover height by that much.
- The forward push (1.6 to 2.7 m/s^2 on 26 Sep) is the pilot's to cover with the pitch stick; 30 s hands off walks
  tens of metres.
