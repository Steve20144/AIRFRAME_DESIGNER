# 2026-10-07 15:13 V3 second NL_AUTO attempt: nose stuck 8.5 s at full command, breakaway, overshoot abort, tipped

Dashboard run `results/telemetry_runs/run_20261007_151340.json` (radio). ULog on the SD card: `2026-10-07/22_13_15.ulg`
(not pulled yet). Board: build Oct 6 16:06, NL_AUTO 1, ALT 0.8, HOV 5, mode Position at arming (the module would have
switched to Stabilized at the hold; it never got there).

Timeline (s after arming, structural pitch): 0 parked at -16.2 (6 Oct: -13.9); 0 to 2 the nose fans ramp 0.16 -> 1.0;
2.0 to 8.2 **cmd 0.9 to 1.0 (M7/M9/M8 at 1900 us) and the nose does not move** (-16.3 to -15.7), yaw drifts -109 ->
-117 (the aircraft twists on the ground); 8.2 to 9.0 the nose starts creeping (-15.6 -> -13); 9.0 to 11.3 it runs away:
-12 -> +10.5 at 12 to 35 deg/s (the limit for the loop is 3 deg/s) while the command still reads 0.8 to 0.97, because the
integral (cap +-40 deg, KQI 0.012 = up to 0.48 of thrust) wound up during the stuck phase and unwinds at only
10 deg/s-equivalent per second; 11.3 to 12.0 command 0.09 / 0.6 / 0 (the loop finally cuts), pitch 10.5 -> 17;
12.08 **abort "overshoot"** (pitch > 8.5 + NL_OVERSHOOT 12 = 20.5, `abort(Overshoot, false)` = every motor stopped at
once, no lowering), 12.23 disarmed by the module, pitch peaks 23.8 at 12.6 and falls back (-14 deg/s at 12.8, log
ends 13.0). The user reports the aircraft tipped over (the dashboard log ends 1 s after the disarm; the ULog has the
rest). Rear fans at 1000 us throughout: it never reached the hold, never took off.

## Cause

1. The nose did not break free at full nose-fan thrust for 6 s (on 6 Oct it rose within 0.3 s at 87 %). Candidates:
   the aircraft sat 2.3 deg lower than on 6 Oct (nose skid or legs caught, ground), or a nose fan not delivering
   (esc_status in the ULog decides).
2. No anti-windup / no stuck detection in the Ramping loop: full integral when the nose let go, so the loop pushed
   the nose through the target at 4 to 10 times the allowed rate and could not pull it back (the fans only push up).
3. The overshoot abort cuts all motors instead of anything softer; from +20 deg nose-up with nothing running the
   aircraft fell back onto the nose.

## To do before the next attempt

- Pull the ULog over USB (attitude after 13 s, esc_status / actuator_outputs of the three nose fans, the ground angle).
- Firmware: freeze the integral while the command is saturated and reset it when the nose starts moving (or cap the
  integral at ~0.1 of thrust); abort (lower, motors off) if cmd > 0.9 for more than 2 s with the nose not moving;
  reconsider the overshoot threshold and what the abort does.
- Aircraft: check that the nose rests free (skid, legs, ground) and the parked angle before arming.

## ULog `results/board_logs/ulog_20261007_151340_nose_stuck.ulg` (pulled over USB 15:25)

- Three nose fans commanded identically (actuator_motors control[6..8] all 1.0 from 2.5 to 6.5 s, PWM 1900); no
  esc_status in the log, so whether all three spun is not provable from the board. 6 Oct for comparison: the nose
  started moving at 2.5 s with the command at 0.62 to 0.76 and rose at about 5 deg/s; today 1.0 for 6 s moved nothing.
- Structural pitch (PX4 pitch + 8.5): peak 24.2 deg at 13.3 s, 16.8 and falling at 13.8 s, log ends 14.05 s (logger
  stops 1 s after the disarm). No backward flip in the log: the aircraft was falling forward onto the nose from 24 deg
  with every motor off when the record ends; whatever followed is not on the board.
- Pitch at arming -16.2 (6 Oct -13.9): sat 2.3 deg lower.
- Parameters changed between the 6 Oct flight and today (full 1246-parameter dump
  `params_20261007_full_before_revert.json`, filtered by component): NL_AUTO_ALT 1.5 -> 0.8 (reverted to 1.5 at 15:20
  on the user's order to repeat the 6 Oct flight), COM_DISARM_LAND 2 -> -1 (kept: needed for the lowering after an
  automatic landing), CAL_ACC0_PRIO -1 -> 50 and CAL_BARO0_OFF (PX4 internal, not set by hand). Firmware: the 6 Oct
  flight ran a pre-fix NL_AUTO build that no longer exists as a file; the 16:06 build on the board has the same nose
  lift loop, the differences are all after the hold (Stabilized, feed-forward, slew).

## 15:28 third attempt (run_20261007_152841, ULog 22_28_15.ulg on the SD, not pulled): same stuck nose, switch off at the target, tipped back

Parked at -10.8 this time (the aircraft sits differently every time: -13.9, -16.2, -10.8). 0.9 to 5.0 s nose fans 0.8 to
1.0 and the nose does not move (stuck 4 s); 5.0 to 8.4 s it breaks free and accelerates to 12 deg/s (limit 3); at 8.3
deg the user switched the nose lift off: the module went to Lowering (command 0, fans idle 1100 us) but the momentum
carried the nose to 27.2 deg at 10.4 s; it then sat at 19 to 22 deg for 25 s with the fans idle (resting beyond the
tipping point, on the rear; gravity could not bring the nose down and the nose fans only push up), lowering loop
bursts of 0.4 to 0.9 did nothing, "lowering timeout" at 33.5 s, motors stopped, disarmed; pitch read 0.6 a half second
later (estimator reset or the fall). Rear fans never ran. Conclusion unchanged: integral windup on a stuck nose, no
stuck detection, nothing can stop an overshoot once the fans are the only actuator. The aircraft must not be tried
again until (a) the nose rests free and the reason it sticks is found, and (b) the Ramping loop gets anti-windup and
a stuck abort.

## 15:38 fourth attempt (run_20261007_153845, ULog 22_38_19.ulg on the SD, not pulled; still the Oct 6 firmware)

Parked -10.2. Stuck 0 to 4.5 s at cmd 0.85 to 0.95 (PWM 1800 to 1870); breakaway 4.5 s, 8 to 10 deg/s, overshoot to
17.5 at 11.5 s (under the 20.5 abort), the loop brought it back, **holding at 6.8 to 8.9 deg from 17.8 s with cmd mean
0.64**: identical to every earlier hold (3 Oct 0.58 to 0.65, 6 Oct 0.62), so the nose fans and the nose weight are
what they always were. **Stabilized switch worked** (18.0 s), automatic takeoff 20.7 s, handover 20.8 s. During the
handover the structural pitch fell 8.8 -> 3.8 (22.0 s) -> -28.5 (23.3 s) **with the nose fans at 1900 us (max) from
22.0 s and the rear fans at 1100 to 1300 us** (PX4 cutting the rear to raise the nose); range 0.49 -> 0.33 m (nose
toward the ground); at 24.7 s rear 1843, pitch -29, kill at 25.0. Never left the ground: tail lifted, nose on the
ground. Handover history: nose UP to +21/22 on 3 Oct 17:29 and 6 Oct, nose DOWN to -23 on 3 Oct 17:57 and 18:18 and
today; it never stayed at 8.5.
Signature today, twice: full command on the nose fans produced no usable thrust (stuck phase, handover) while
0.64 held the nose. Candidates: voltage collapse of the pack at full command (dashboard battery 7.48 V today vs 8.04
on 6 Oct and 7.96 on 3 Oct; which pack feeds the nose fans and the current under load are in the ULog
battery_status), or an ESC cutoff at full throttle. To check: ULog over USB, the pack's charge state, a bench spin of
the three nose fans at 100 % with a voltmeter on the pack.

### ULogs pulled 15:44 (`ulog_20261007_153845_handover_nose_down.ulg`, `ulog_20261007_152841_switchoff_tail.ulg`)

- battery_status has only the board's own 2S supply (7.48 V, 0.4 A, flat; 6 Oct 8.04 V): the fan packs are not
  measured, so the voltage-collapse idea is neither confirmed nor refuted from the board.
- Handover 15:38 from actuator_motors: nose fans 0.77 to 1.0 from 23.0 s, rear fans mean only 0.19 to 0.39, and the
  structural pitch fell 2 -> -26 deg in 1.5 s; the tail lifted about the nose skid at rear 0.3 (on 26 Sep the rear
  needed 0.55 to leave the ground at all). 6 Oct handover: nose fans 0.47 to 0.82, rear 0.31 to 0.84, nose went UP.
- Pattern of the whole day: nose fans at 0.78 to 1.0 move nothing (stuck phases, 3 attempts) or let the nose fall
  (handover), nose fans at 0.47 to 0.64 hold it at 7 to 9 deg. 3 Oct nose-down handovers also had 85 to 90 % peaks.
  Hypothesis: the nose fans lose thrust above about 80 % command (PWM above about 1800 us of a 1100 to 1900 range):
  ESC endpoint / current limit / stall / pack sag. Outputs: nose fans on MAIN8 (M7, 1100-1900), AUX2 (M9, 1100-1900),
  AUX4 (M8, 1100-1900); rear on MAIN1-6 (1100-1900), 400 Hz PWM, no ESC telemetry.
- Test before any flight: bench spin of each nose fan at 60, 80, 100 % with a voltmeter on its pack, listening for
  a drop at the top. Cheap in-flight guard if confirmed: NL_MAX_CMD 0.8 (the hold needs 0.64).

## Handover authority check (ULogs: attitude setpoint, torque setpoint, control_allocator_status)
- 6 Oct (Position): the structural pitch followed PX4's own setpoint within 1 to 2 deg (sp 12 -> 18 -> 20 -> 13 -> 17,
  pitch 6.7 -> 20.4 -> 21.4 -> 13.3 -> 18.4), unallocated pitch torque 0 throughout, nose fans 0.33 to 0.8 (not
  saturated), rear 0.5 to 0.85 within 0.5 s. The "nose up to 20" was COMMANDED by the position controller (leaning
  back about 12 deg against the forward push of 1.6 to 2.7 m/s^2), not lost control.
- 15:38 today (Stabilized, sp 8.4 constant): nose fans at 1.0 from 22.0 s (weak packs), rear only 0.05 -> 0.45 over
  2.5 s (slow ramp), pitch torque demand 0.3 -> 3.4, unallocated up to 3.1 Nm: PX4 ran out of pitch authority.
- Consequences: with charged packs, the fast ramp and a constant Stabilized setpoint the nose should stay near 8.5;
  in PX4 Hold the position controller will lean the nose up about 10 to 12 deg against the push, as on 6 Oct.
