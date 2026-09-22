# Experiment: first piloted HITL session on ATLAS_09B (Pixhawk 6X Pro, transmitter in hand)

Date: 2026-09-21. Recordings: `results/hitl/attempt1.jsonl`, `attempt2.jsonl` (`scripts/record_hitl.py`: board
MAVLink via the app's udp 14550 proxy + sim truth over the websocket).

## Setup findings (before any flight)

- The board still had COM_RC_IN_MODE 3 (MAVLink sticks only) from scripted runs and no mode switch mapped
  (RC_MAP_FLTMODE 0): the transmitter did nothing. Now 0 and channel 5 (up Stabilized, down Position).
- Arming is the switch on channel 8 (RC_MAP_ARM_SW 8); a kill switch on another channel blocked the first arm.
- Nose-lift switch: channel 7 active low; the other position of the same switch now runs the landing rotation
  (`rc_switch_loop` in `server/app.py`, `rc_land_threshold` mirrors the takeoff threshold). `rc_takeoff` false in
  the 09B file: arming with the switch down no longer requests Takeoff mode.
- usbipd: the board re-enumerates on every reboot (parameter push with a rotation change, estimator restart) and
  comes back as /dev/ttyACM0 or ACM1; keep `usbipd attach --wsl --busid <id> --auto-attach` running and reconnect
  on the new port. After a sim reset the board's EKF keeps the old attitude; restart the estimator (reboot).
- ATTITUDE_TARGET is not streamed on USB by default: `mavlink stream -d /dev/ttyACM0 -s ATTITUDE_TARGET -r 50`.

## Flights

| attempt | result | what the recording showed |
|---|---|---|
| 1 (set A yaw loop) | two flips: at landing, then on full yaw stick | attitude tracked the stick within 3 deg; heading spun at 31 deg/s hands-off (set A yaw loop too soft under a pilot's tilts); inputs 60 to 85 percent stick = 22 to 30 deg tilt, 8.5 m/s, 19 m |
| 2 (yaw P 2.8 / rate P 0.15 / max 60, MPC_MAN_TILT_MAX 12, MPC_MAN_Y_MAX 60) | landed at 3.8 m/s, "could not hover, it drifts" | heading held; nose sat 5 to 6 deg above setpoint for the whole flight with the rate setpoint at -25 deg/s and no motor saturated; mid stick climbed at 4 m/s to 33 m |

## Root causes found in SITL afterwards

- **Pitch integrator wind-up on the legs** (`scenarios/stab_slowlift_probe.json`, a 4 s pilot-style throttle ramp):
  the pitch offset equals the integrator limit (5 deg at MC_PR_INT_LIM 0.3, 2.7 at 0.1, 0 with MC_PITCHRATE_I 0)
  and never unwinds airborne; a brisk liftoff never builds it. MC_PITCHRATE_I 0 adopted; the brisk sequence still
  tracks to 0.24 deg. Roll I 0 also removed the standing roll bias in that probe (not adopted yet).
- **Hover thrust**: zero vertical speed at thrust setpoint 0.323 in the flight data; MPC_THR_HOVER 0.40 to 0.32 so
  mid stick hovers instead of climbing.
- **Stick scale**: 35 deg at full stick is outdoor scale; 12 deg indoors.
- HIL output mapping (HIL_ACT_FUNC1..10 = motors 1..10) verified correct on the board; SITL and HITL exports differ
  only in output function names.

## Pilot procedure that follows

SB down and hold until the nose holds; sticks centred; arm; advance the throttle briskly through liftoff (do not
sit at hover thrust on the legs); hover at mid stick; corrections of 30 to 50 percent held for two seconds;
descend under 1 m/s; disarm; SB up.
