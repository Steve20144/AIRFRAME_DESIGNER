# 2026-10-09 HITL on the Pixhawk 6X Pro: tremble and jolts of the automatic hop

## Question

The final SITL firmware (final_v2) on the real board in HITL: does it fly the automatic hop
(`scenarios/fw_auto_hop_winner.json`) without tremble and with minimal jolts? Iterate until it does.

## Setup

- App in HITL mode, headless: launch entry `app-winnerC-hitl-mac` (port 8085, `/dev/cu.usbmodem01`), airframe
  `airframes/atlas_v3_30_board_winnerC_sitl.json`. Runner: `scripts/hitl_pack.py --spec studies/hitl_*.json`
  (one board, real time, ~1.5 min per flight incl. the reboot; uses `/api/tuning/run` live + sitl_pack metrics).
- New metric `tremble()` (hitl_pack.py): p/q rate content 2-10 Hz, rms over the flight, worst 1 s window.
- SITL for screening in between (`scripts/sitl_pack.py`, new `args`/`cand_args`, new `run --accel-bias X,Y,Z`).
- Never run SITL packs or builds during HITL flights: the HITL sim is real time and the CPU load spoils it.

## What was wrong (in order found)

1. HITL would not arm (3 board conditions SITL does not have, see lessons/hitl-board-conditions.md).
2. Nose fell onto its leg at 12 deg/s at the end of the lowering: the target was PX4's pre-lift pitch estimate,
   1-2 deg off a few seconds after a reboot. Fix: NL_LOW_VMIN (keep lowering >= 1 deg/s, fade on leg contact).
3. Tremble in the handover/climb (also in SITL, the "it trembles when it climbs" of 8 Oct): rc_update only
   processes manual control when an RC value changes or every 300 ms; with steady sticks the module's throttle
   ramp reached the motors as 0.05 steps at 3.3 Hz. Fix: patch 0003 processes every RC frame while the nose lift
   gives the throttle. Tremble halved in SITL (0.43 -> 0.24 deg/s) and HITL.
4. HITL roll ringing 2-4 Hz (0.2-0.35 deg/s, SITL 0.05): USB delay on a roll loop near its margin.
   Fix: MC_ROLL_P 5 -> 4, MC_ROLLRATE_P 0.6 -> 0.45 (also better in SITL).
5. Roll kick at lift-off 3-7 deg/s: PX4 corrects a small tilt estimate error (0.1-0.5 deg) against the legs and
   releases it when they leave the ground. Fix: patch 0004 (mc_att_control): while nose_lift_output.on_legs the
   roll/pitch setpoint follows the estimate (NL_LEGS_ATT 3), offset fades over NL_LEGS_FADE 0.5 s off the legs.
   SITL with 0.4/0.8 deg tilt error: 5 -> <= 1.2 deg/s. HITL with 0.40 deg error: 1.3 deg/s.
6. Touchdown pitch kick (rear legs hit first): NL_AUTO_VTD 0.05 -> 0.03 m/s: 4.7 -> 3.0 deg/s, +3 s of flight.
7. Lift-off pitch: NL_KQI 0.012 -> 0.024 and MC_PITCH_P 4 -> 3. Tried and rejected: NL_HO_FF > 0.05 (crashes at
   30 N), NL_K_ANG 2 with NL_LEGS_ATT 3 (crashes at 30 N), NL_HO_KANG >= 3 (unstable; kept at 0 = off).

## Results (fw_auto_hop_winner)

| | HITL start (h0) | HITL final3 (h6, 4 flights) | SITL final3 36/32/30/28 N |
|---|---|---|---|
| tremble rms deg/s | 0.53-0.62 | 0.12-0.27 | ~0.13-0.18 |
| worst 1 s tremble | 1.2-1.4 | 0.22-0.84 | ~0.5 |
| worst jolt deg/s | 6.2-12.4 | 3.1-4.6 | 3.2-4.1 at 36/30 N, 5-7 at 32/28 N (climb) |
| lowering max / front-leg touch | 12.4 / 8.4 | 3.1-3.3 / 0.6-0.8 | 3.2 / ~1.2 |

The one 4.6 in HITL was a single bad optical-flow sample over USB (EKF velocity step 0.05 m/s), HITL only.
Accel vibration (6-7.6 m/s2) is the simulator's fan-imbalance model: identical in SITL and HITL, not a control
effect, not reducible by tuning. Gyro vibration 0.012-0.015 rad/s.
Switch-off 4/4 and kill 4/4 in SITL with final3.

## Files

- Board image: `results/hitl_pack/firmware/px4_fmu-v6x_multicopter_final3_20261009.px4` (flashed 9 Oct, build
  18:46), sources + all 4 patches in `src_final3/`. Params: `results/hitl_pack/final3_params.json`.
- Back to the real aircraft: `results/hitl_pack/board_restore_for_flight.json` (pre-HITL backup + final3; lists the
  HITL-only values to undo). Not applied yet.
- Build on this Mac: `SDKROOT=/Library/Developer/CommandLineTools/SDKs/MacOSX26.5.sdk VENV_BIN=~/.venvs/px4-build/bin
  bash scripts/build_nose_lift_firmware.sh sitl|board` (the default SDK 27.0 cannot link; a failed configure makes
  PX4's Makefile delete the build directory).
