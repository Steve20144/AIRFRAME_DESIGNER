# Experiment: firmware nose lift on the real aircraft (ATLAS_09B, 2026-09-22/23)

First runs of the staged takeoff on the aircraft itself: Pixhawk 6X, only the two nose fans wired (outputs 9/10 =
motors M10/M9, `NL_MOT_MSK` 768), props on, fans on their own battery (the board's power module reads avionics only,
~0.4 A throughout). Logs pulled over the SiK radio (see [telemetry dashboard](../architecture/telemetry-dashboard.md));
analysis scripts were scratch, the numbers are below. Board clock runs about 3 to 7 min behind the PC (no GPS).

## Runs

| log | park | what happened | cause |
|---|---|---|---|
| tlog 21:34 | -8.3 | fans ramp 0.4 s, "aborted (attitude lost)" | clock read before the uORB copies ([lesson](../lessons/px4-module-clock-before-copy.md)); fixed |
| 164 | -8 | M9 100 %, M10 82 %, nose never moves, "left the ground" after 6.4 s | EKF height drift from fan vibration ([lesson](../lessons/fan-vibration-drifts-ekf-height.md)); fixed |
| (dashboard) | -8 | same 92 / 82 % | the yaw split: M9 x1.10, M10 x0.90 of one command capped at 1 (`NL_W0/1`) |
| 174 | +1.6 | rise speeds up to 15 deg/s, fans cut to 0 at 26.6 deg, coasts to 42.8, "overshoot" | raise gain far too weak for the real fans (`NL_KQ` 0.02) |
| 177 | +2.1 | SB off at 9.8 deg while rising 9 deg/s: fans 0 <-> 100 % every ~0.1-0.4 s, pitch rate +-14 deg/s | lowering gain 15x the raise gain (`NL_LOW_KQ` 0.3) |
| 184 | +1.7 | G4 gains: rise ~3.5 deg/s average but commands still burst 0 <-> 0.6-1.0; SB off at ~17 deg, nose kept rising to 27 with commands near 0, then down, last 17 -> 2 deg at -20 deg/s | fans spin down slowly (thrust persists 1-2 s after the command falls) and the loop fights that |
| 185 | -1.7 | full command 7.6 s, nose never moves | lift margin: the cut-off lies between -1.7 and +1.7 deg park |

At 27 deg the nose came back down on its own: gravity still holds it nose-down near the hover pitch, only weakly (it
barely brakes a rotation there, which is why an over-speeding rise coasts far past the target).

## Gain study in SITL (cancel while rising, park +2)

Reproduced the aircraft only with the firmware underrating the fans (`NL_A0/1` x0.85 or x0.9) and a slower fan
(`tau` 0.3 to 0.5 s). Worst rise / worst lowering drop, fan lag 0.5 s, x0.85:

| gains (KQ/KQI, LOW_KQ/LOW_KQI) | rise | lowering |
|---|---|---|
| 0.02/0.012, 0.3/0.1 (sim tuning) | 20.8 deg/s | -54 deg/s |
| G4 0.10/0.05, 0.06/0.03 | 8.9 | -2.4 |
| G5 0.15/0.06, 0.06/0.03 | 7.4 | -2.6 |

G4 set on the board 2026-09-23 (the softer of the two: SITL proved optimistic about stiff gains, 0.3 never
bang-banged in SITL but did on the aircraft). Standard takeoff/kill/cancel pass at +2 and -8 (~4 deg/s rise).

## Rate filter: rejected

The 5-10 Hz shake of the frame on its legs (+-20 deg/s pitch rate, attitude within +-0.3 deg, vibration metric 3-5)
looked like the burst driver. A first-order low-pass on the module's rates (`NL_Q_LPF`) with the sim's new slow
spin-down (`tau_down` 1.0) made it worse by its lag: 2 Hz raised the worst rise 9 -> 14 deg/s and the lowering drop
-4 -> -33; 1 Hz 20 / -51. The bursts on the aircraft repeat at ~1 Hz (0.4 raising, 1.2 lowering), not 10 Hz. The
filter stays in the firmware, default 0 (off).

## Still open

- The sim does not yet reproduce the ~1 Hz bursting even with `tau_down` 1.0 and the gyro shake: measure the fans'
  real step response (spin-up, spin-down, dead time) on the bench, then fit `tau`/`tau_down` and re-tune.
- Lowering never fades: after the rotation the pitch estimate reads ~0.5 deg high (2.85 vs 2.4 true on the leg), so
  "within 0.5 deg of the start" is never met and thrust bleeds off until the 25 s lowering timeout.
- "Strong magnetic interference" after every run with the fans on (fan current near the compass).
- SITL flake: twice in ~40 runs PX4 armed while the module stayed "disarmed"; the retry passes.
