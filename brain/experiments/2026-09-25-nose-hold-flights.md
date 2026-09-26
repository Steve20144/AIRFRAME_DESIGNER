# 2026-09-25 evening: nose hold (NL_FLY_HOLD) flights, ULog findings

ULogs pulled over USB to `results/board_logs/2026-09-25_<dashboard time>_<ulog>.ulg`; analysis script kept in the
session scratchpad (ulog_takeoff.py: throttle-up = rear mean thrust > 0.15; nose = PX4 pitch + 24; a_up = world
vertical specific force in g; a_fwd = horizontal specific force along the heading).

Flights: 19:55 (old handover), 20:18 (nose hold, 24), 21:05 (nose hold + 26 step, NL_F_FF 1.5), 21:20 (same,
NL_F_FF 1.0, throttle raised mid-raise).

- **None left the ground.** a_up stays 0.95-1.05 g through every throttle-up, baro within +-0.45 m (fan wash),
  EKF height flat. Rear fans reached 0.30-0.55 mean thrust (21:05: 0.55) with the nose fans at 0.3-1.0: not enough.
- **Nose-fan split wastes authority.** Whenever the pitch loop asks for full, M9 (control[8]) sits at 1.0 and M10
  (control[9]) at 0.45-0.65: the roll/yaw split (NL_W0/W1 1.09/0.89 + NL_K_RATE damping) clips M9 and M10 never
  takes the rest. 21:20: nose fell 25 -> 2.5 with M9 1.0, M10 0.5-0.67. Candidate fix: pitch priority (give the
  clipped share to the other fan). Not done.
- **On the rear feet the rear fans pitch the nose up** (20:18, 21:05: 24 -> 28-31 with the nose fans still
  0.6-0.9); when the nose falls it falls all the way (19:55 PX4 in control, nose fans only 0.55; 21:20 saturated).
- **Forward push:** a_fwd +1 to +2 m/s2 whenever the rear fans run (up to 3-4 while the nose falls), not smaller at
  27-31 deg than at 24-25 per unit rear thrust. Uncertain: after the kills the same measure reads -1.5 to -2.9 at
  rest (attitude estimate off after the fast rotation), and fan vibration biases the accelerometer.
- No horizontal velocity source on the board (no flow/GPS data; EKF constant-position mode).

## 2026-09-26 00:00-00:21, front-fix build (pitch priority), thrust-scaled angle 24 -> 26, NL_F_FF 1.0

All 14 ULogs of 2026-09-26 (SD folder) pulled to `results/board_logs/sd0926_*.ulg`; `sess129/log104-106` are old
HITL sessions (SYS_HITL 1, old firmware, old motor map), not the aircraft. Flights: ULog 07_01_23 (dashboard
00:02:41), 07_17_49 (00:17:50), 07_21_03 (00:21:05); the rest are arm/disarm only. All three ended on the kill.

- **00:21 left the ground.** Baro rose steadily 0.2 -> 0.84 m over 1.5-2.7 s after the throttle-up, vertical
  specific force 1.03-1.05 g sustained, EKF vz 0.4-0.8 m/s (the EKF also drifts +0.2-0.3 m/s sitting still, so
  it is not evidence alone). Killed at ~0.8 m. Rear fans 0.55-0.59 mean thrust.
- **The nose now holds in the air.** 00:21: 30 -> 26.3 deg while airborne with the nose fans at 0.4-0.66 (not
  saturated). Both nose fans now share the load (00:17: 0.6-0.8 each; before the fix M10 sat at ~0.5).
- **The overshoot happens at the ground-to-air transition.** Whenever the rear fans pass ~0.4, the nose runs 3-4 deg
  over the target (29-30) while the rear feet are still loaded, then comes back once airborne.
- **00:17:** rear 0.22-0.30 held the nose 23.3-26.3 for 5 s (target 25.6-26), baro +0.1-0.3 m (light on the legs at
  most); rear to 0.43 -> nose 29.5, kill.
- **Throttle already at mid before the hold** (00:21: 0.50 during the raise) makes the rear fans jump at once.
  In the flight hold with the rear fans idle the nose sagged 23.4 -> 20.7 (flight gains, ground).
- **Forward push:** a_fwd +1.6 to +2.7 m/s2 during the 00:21 climb (nose 26-30); near 0 (-0.4..+0.8) in 00:17 while
  the nose sat at 24-26 with the rear at 0.22-0.30. Same caveats as above.
- Heading drift 2-3 deg/s (00:17: +20 deg over 6.4 s). "radio link lost" 3.5 s after the 00:21 kill.
