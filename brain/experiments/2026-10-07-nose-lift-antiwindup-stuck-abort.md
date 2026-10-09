# 2026-10-07 Nose lift ramp: anti-windup, integral cap, stuck-nose abort (built, not simulated, user's order)

After the three 7 Oct attempts (nose held 4 to 8 s at full command, breakaway at 12 deg/s, 24 to 27 deg, aircraft on
its tail), `firmware/px4_ext/src/modules/nose_lift` Ramping state:

1. The integral is frozen while the command sits at NL_MAX_CMD and the rate error still asks for more.
2. The integral cap is `NL_I_MAX` (new, default 10 deg = 0.12 of thrust at NL_KQI 0.012; was a fixed 40 = 0.48).
   The Holding state keeps its 40 deg cap.
3. When the measured pitch rate exceeds the wanted rate by a full NL_RATE (3 deg/s), a positive integral is dropped
   to zero at once; the rate term and the balance feed-forward then brake.
4. `NL_STUCK_S` (new, default 2 s): full command for that long with less than 1 deg of motion aborts with the
   controlled lowering ("nose stuck"). 0 disables. New Abort::Stuck.

Build: Oct 7 (~15:45) `~/PX4-nl/build/px4_fmu-v6x_multicopter/`, copy
`results/board_firmware/px4_fmu-v6x_multicopter_noselift_antiwindup_stuck_20261007.px4`. The user asked for no SITL
run before flashing. NOTE: the Oct 6 16:06 image (what the board runs) was overwritten in the build dir and has no
copy; it is reproducible by reverting this change in `firmware/px4_ext` and rebuilding.

What was NOT changed: NL_KQ 0.02 (the rate brake stays weak: 9 deg/s over = 18 % less thrust), the overshoot abort
(motors off above target + 12), the Holding and Lowering loops. The physical cause of the sticking is still unknown.

## Flashed 2026-10-07 15:50 over USB (px_uploader, 26 s, verify OK)

`ver all`: build Oct 7 2026 15:37:38, git d6f12ad1c4. Parameters kept across the flash; NL_I_MAX 10 and NL_STUCK_S 2
present at their defaults; **NL_MAX_CMD set 1.0 -> 0.8** (the nose fans move nothing above ~0.8, see the 7 Oct flight
note; the hold needs 0.64). Flight configuration for the next attempt = the 6 Oct profile, automatic: NL_AUTO 1,
ALT 1.5, HOV 5, WAIT 3, THR 0 (mid stick), SLEW 0.3, MAX 0.9, PILOT 1400, TGT 8.5, COM_DISARM_LAND -1, kill ch 5.
After the flash `nose_lift status` read roll 8.1 deg on the ground (NL_ROLL_MAX 8.0: the lift would abort "roll
limit" at once), pitch -23.8, switch on: the aircraft was not sitting level, told the user.

## 16:00 NL_AUTO_HOLD: PX4 Hold then PX4 Land (built, flashed, not simulated; user's order)

New phases HoldWait / HoldPX4 / LandPX4 in `step_auto`. At Hover, after 1 s with `_lpos` xy, v_xy and z valid, the
module sends DO_SET_MODE AUTO/LOITER; nav_state AUTO_LOITER within 2 s -> HoldPX4 (NL_AUTO_HOV runs from there),
else "Hold refused", Stabilized back, own hover. After the hover: DO_SET_MODE AUTO/LAND -> LandPX4 until the land
detector says landed (then Stabilized, Cut from 0, nose lowering), 60 s timeout; PX4 leaving Hold/Land for anything
else -> Stabilized + own landing. Switch off in Hold -> PX4 Land. Pilot throttle above NL_AUTO_PILOT -> Stabilized
first, then the pilot. The module's throttle stays at the mid stick under PX4 (a manual-mode fall-back holds height).
Build Oct 7 15:53:54, copy `results/board_firmware/px4_fmu-v6x_multicopter_noselift_px4hold_20261007.px4`, flashed
16:00. Params: NL_AUTO_HOLD 1, COM_RC_OVERRIDE 1 -> 0 (a stick touch would drop Hold), MPC_LAND_SPEED 0.7 -> 0.3
(MPC_LAND_CRWL 0.3 unchanged). Risks stated to the user: Hold needs flow aiding through the climb (6 Oct: 2 s of
dead reckoning after lift-off) and a consistent heading (compass disturbed by the fans); PX4 Land has no flare of
its own beyond the land speed.

## 16:10 parameter changes (over the dashboard relay)
- NL_MAX_CMD 0.8 -> **0.64**: the cap is a thrust fraction before NL_EXPO 2, so 0.8 was motor command 0.89 (PWM ~1815);
  0.64 = command 0.8 (PWM ~1740). Applies only to the module's own command (lift, hold, handover floor), not to
  PX4's allocation on the nose fans once PX4 flies them.
- NL_AUTO_RAMP 2 -> **0.5 s**, NL_AUTO_SLEW 0.3 -> **1.5 /s**: the 6 Oct handover worked because the rear fans went
  idle -> 0.66 in 0.5 s (the Position-mode step) and the aircraft left the ground at once; today's slow ramp (0.05 ->
  0.39 over 2.5 s) unloaded the rear feet without flying, the aircraft stood on its nose and the nose fell. Repeat
  the 6 Oct speed, in Stabilized.
- 16:15 NL_STUCK_S 2 -> 10 s (user: 2 s is not enough for the nose to break free; NL_TOUT 25 s is the outer limit).
- 16:20 run_20261007_160157: abort 'motors cannot hold the nose' at 2.4 s with the nose fans at 1740 us (cmd 0.80): the pre-existing CannotHold check fires when the balance fraction at the parked angle (-11 deg, model ~0.7) exceeds NL_MAX_CMD + 0.02; with the cap at 0.64 the model says the nose cannot be lifted at all. NL_MAX_CMD back to 1.0. Note: the model's balance fraction at the parked angle (~0.7 thrust = command ~0.85) matches what every breakaway today needed; the 'fans lose thrust above 80 %' idea is no longer the simplest explanation for the stuck starts, marginal authority plus friction at the parked angle is.

## 16:30 NL_Q_FC: the loops damp on a 2 Hz low-passed rate; stuck test on the 0.5 s averaged command

16:06 attempt (run_20261007_160614, ULog 23_04_44.ulg not pulled): nose stuck 14 s at -9.5 deg with the command
chopping 0.5 <-> 1.0 at the frame's rocking frequency (10 to 15 Hz, +-20 deg/s on the legs with the fans on; the same
rocking is in every log back to 6 Oct, std 4 to 9 deg/s); KQ 0.02 x 20 deg/s = +-0.4 thrust. Fans average ~0.85, never
a sustained 1.0, and the NL_STUCK_S test (instantaneous command at the limit) never fired. Change: `_q_f` (NL_Q_FC,
default 2 Hz) replaces the raw rate in every loop's damping term (ramp, hold, lowering, nose hold); the stuck test
uses `_cmd_avg` (0.5 s) above 0.9 of NL_MAX_CMD. Build Oct 7 16:28, copy
`results/board_firmware/px4_fmu-v6x_multicopter_noselift_qfilter_20261007.px4`. Not simulated (user's order).
Open: yesterday the nose moved at a mean command of 0.60, today 0.85 moves nothing; the aircraft, not the loop.
- Flashed 16:40 (build datetime Oct 7 16:11:04; first two uploader runs failed because the dashboard in --port auto had fallen back to the USB port: stop the dashboard before a flash). NL_Q_FC 2 present, all flight params unchanged.

## 17:41 first try on the 16:11 build with charged fan packs (run_20261007_174117): lifted easily, overshoot abort

Parked -5.6. The nose moved at once (cmd 0.58 to 0.81, no stuck phase: the charged packs fixed the sticking, so
the stuck starts were low fan-pack voltage). It rose at 5 to 8 deg/s (wanted 3), passed 8.5 at 4.1 s with cmd 0.57,
kept rising with cmd 0.36 to 0.66 (10.5 at 4.6 s, 14 at 5.2 s, 19 at 6.0 s), overshoot abort at 20.5, motors off,
fell back to -11. Board supply 7.27 V and "Low battery" all along (the board's own 2S, not the fans).
Cause: the balance feed-forward over-estimates the thrust. Model (NL_WEIGHT 141, PIV -0.349/0.279, A 24.9/24.9/32.0,
sum 81.7): fraction 0.65 at -5.6, 0.52 at 8.5 (cmd 0.72), 0.47 at 14 (cmd 0.68), 0.40 at 20 (cmd 0.63); the measured
hold was cmd 0.62 at 9 deg on 6 Oct (fraction 0.38) and 0.64 at 8.2 today on weak packs, less on fresh ones. Until
today the integral (cap 40 = -0.48 of thrust) absorbed that error; the new NL_I_MAX 10 (symmetric) leaves only
-0.12, and with KQ 0.02 the loop's floor at 10 to 14 deg was cmd ~0.45 to 0.5, above what holds the nose. The model's
balance point (gravity moment zero) is at 51 deg structural, so the nose was never past balance; it was pushed.
Proposed: NL_WEIGHT 141 -> 105 (matches the 6 Oct hold), and in firmware an asymmetric integral cap (small up,
40 down) so the integral can always brake.
- 17:55 NL_WEIGHT 141.24 -> 105 over the radio (step 1 of 2; the asymmetric integral cap only if this is not enough). Expected loop command with it: ~0.69 at -5.6, ~0.62 at 8.5, ~0.59 at 14 deg, +-0.12 thrust from the integral.

## 18:02 and 18:03 with NL_WEIGHT 105: the nose no longer lifts, the loop sits at a ceiling of cmd 0.81
- 18:02 (parked -7.3): command climbs to 0.81 by 4.9 s and stays at exactly 0.81 for 12 s; nose -7.3 -> -5.7; the
  user disarmed at 17.5 s. 0.81 is the loop's own ceiling: ff (W 105) 0.48 + integral cap NL_I_MAX 10 x KQI 0.012 =
  0.12 + rate term 3 x 0.02 = 0.06 -> 0.66 thrust -> cmd 0.81. The stuck test never fired (0.81 < 0.9 x NL_MAX_CMD).
  With W 141 the same ceiling was ~0.88 at -6 deg, enough on charged packs (17:41 lifted at 0.58 to 0.81).
- 18:03 (parked -2.6) and 16:15 (parked -4.3): cmd 0.75 to 0.94, the nose did not rise, then dropped to -16/-17
  within 2 s while the fans pushed (range 0.45 -> 0.39); the aircraft was resting higher than its natural parked
  angle (~-16) and slid back down; that angle needs more thrust still.
- So the 10 deg integral cap is wrong both ways: too small up once the feed-forward is lowered, too small down when
  it is high. Fix options: NL_I_MAX ~25 (0.30 thrust both ways; windup bounded by the freeze-at-max and the
  zero-on-overspeed rules); stuck test against the loop's real ceiling, not 0.9 x NL_MAX_CMD.

## 18:05 (ULog sd1008_01_03_20.ulg; all of today's ULogs pulled: sd1007_23_04_44, sd1007_23_13_09, sd1008_00_39_12,
## sd1008_00_59_56, sd1008_01_01_15, sd1008_01_03_20)
Parked -0.6. Lifted slowly (1.5 to 2 deg/s, cmd 0.73 to 0.75) to 6.5 at 6 s, then wandered 1.7 to 8.5 (mean 6.2) for
8 s with cmd 0.48 <-> 0.86 at 4.5 Hz and one nose fan at a time jumping to 1.0 (roll split); never 0.6 s inside
6.5..10.5 settled, so no Holding, no takeoff. From 13.5 s the nose sank (3.8 -> -1.7 -> -9.6 -> -16 at 17 s) with
cmd 0.83 to 1.0 (PWM 1790 to 1840), killed at 18.7 s.
Compared holds: 6 Oct and 15:38 (raw rate) pitch p2p 1.3 to 2.5 deg, cmd std 0.10 to 0.11 at 8 to 10 Hz; 18:05 (2 Hz
filter) p2p 6.8 deg, mean 2.3 deg under the target. Causes: (1) the loop's authority, NL_I_MAX 10 x KQI = 0.12 with
W 105: ceiling ~cmd 0.75 to 0.8, a steady error under the target (same as 18:02); (2) the 2 Hz rate filter's lag made
the hold looser and slower; (3) the late sink at cmd 0.83 to 1.0 matches 16:15 and 18:03: thrust fading with time on
the nose-fan pack (not measured; 17:41 on fresh packs lifted at 0.58 to 0.81).
Proposed (params only): NL_I_MAX 30, NL_Q_FC 5; charge and measure the nose-fan pack before every attempt.
The earlier "asymmetric cap, small up" plan is wrong: today's failures are a too-small UP authority.

## 18:20 back to the 6 Oct loop + the safe additions (user: "make whatever changes, but it must fly stably")
- BUG found: the overspeed reset (integral -> <= 0 when the pitch rate exceeds the wanted rate by NL_RATE) read the
  RAW rate; on the legs it rocks +-20 deg/s at 10 to 15 Hz, so the test kept firing and kept emptying the integral:
  a large part of the evening's missing authority. Now it reads `_q_settled` (1 Hz).
- Params: NL_I_MAX 40 (the old fixed cap), NL_Q_FC 50 (practically the raw rate, as on 6 Oct), NL_WEIGHT 141.24 (as
  on 6 Oct). Kept: integral frozen while the command is at NL_MAX_CMD, overspeed reset (filtered), stuck abort 10 s
  on the 0.5 s average, NL_AUTO_HOLD 1, RAMP 0.5, SLEW 1.5, LAND_SPEED 0.3. NL_AUTO_ALT 1.5 -> 1.0: SITL showed a 0.5 m
  climb overshoot with the fast ramp and Hold freezes the height where it is (~2.0 m with ALT 1.5).
- SITL V3 (`results/fw_auto_hop/v3_hold_20261007*.json`, board settings, ALT 1.5): full pass, lift 7.2 s, takeoff,
  peak 2.06 m (est.), flat 24 to 28 s (PX4 Hold), constant 0.33 m/s descent (PX4 Land), touchdown 0.25 m/s, nose
  lowered, disarmed; max tilt 11.6. SITL does not model the legs' rocking nor the fan-pack sag.
- Build Oct 7 18:13:59 (`results/board_firmware/px4_fmu-v6x_multicopter_noselift_overspeedfix_20261007.px4`),
  flashed 18:20, parameters read back.

## 19:55 REVERTED to the pre-first-test firmware (user's order)
All four of today's module changes undone in `firmware/px4_ext` (anti-windup/stuck abort, PX4 Hold/Land, rate filter,
overspeed fix); `git diff --stat` of the module back to 333/57/164 (551+, 3-), exactly the session-start state.
Today's last sources kept in `results/board_firmware/src_overspeedfix_20261007/`. Rebuilt image
`results/board_firmware/px4_fmu-v6x_multicopter_noselift_oct6state_rebuilt_20261007.px4` (1762112 B vs 1762108 B for
the lost Oct 6 16:06 image; none of NL_I_MAX / NL_STUCK_S / NL_Q_FC / NL_AUTO_HOLD in it), flashed 19:55, build
Oct 7 19:50:39. Parameters NOT reverted (user asked for the firmware only): NL_AUTO_ALT 1.0 (first test 0.8),
RAMP 0.5 (2), SLEW 1.5 (0.3), COM_RC_OVERRIDE 0 (1), MPC_LAND_SPEED 0.3 (0.7); full first-test dump in
`results/board_params/params_20261007_full_before_revert.json`. Board read roll -59.7 deg after the flash (lying
on its side or out of the aircraft).
