# 2026-10-07 13:06 Board readout before the second automatic hop (USB, /dev/cu.usbmodem01)

- Firmware on the board: PX4 Release 1.17.0, git d6f12ad1c4 (PX4-nl HEAD), **build Oct 6 2026 16:06:00**, variant
  multicopter. That is the 6 Oct evening build with the three fixes (Stabilized forced before the automatic takeoff,
  feed-forward = mid stick, NL_AUTO_SLEW): `NL_AUTO_SLEW` exists on the board and the strings "switching to
  Stabilized" / "NL_AUTO refused, not in Stabilized" are in the image. Sources identical: `~/PX4-nl/airframe_designer_ext`
  == `firmware/px4_ext` (diff -rq clean). The flash itself was not logged in the brain (done between 6 Oct 16:06 and
  7 Oct).
- Parameters (get): NL_EN 1, NL_AUTO 1, **NL_AUTO_ALT 0.8** (was 1.5 on the 6 Oct flight), **NL_AUTO_HOV 5**,
  NL_AUTO_WAIT 3, VUP 0.5, VDN 0.3, THR 0 (= mid stick), SLEW 0.3, MAX 0.9, RAMP 2, KP/KV/KI 0.25/0.35/0.08,
  NL_AUTO_PILOT 1400, NL_FLY_HOLD 0, NL_TGT 8.5, NL_HOV_PITCH 8.5, SENS_BOARD_Y_OFF 8.5, NL_MOT_MSK 448,
  NL_RC_CH 7 (active low, 1300), NL_HO_THR 0.65, NL_HO_TOUT 15, NL_CEIL 0, MPC_THR_HOVER 0.592, MPC_THR_CURVE 0,
  **COM_DISARM_LAND -1** (was 2 on 6 Oct, now right), COM_DISARM_PRFLT 120, RC_MAP_FLTMODE 0 (still no mode switch),
  kill ch 5, arm ch 8, throttle ch 3, COM_RC_IN_MODE 0, COM_RC_LOSS_T 0.5, NAV_RCL_ACT 2 (RC loss = Land),
  COM_LOW_BAT_ACT 0, EKF2_HGT_REF 2 (range), EKF2_OF_CTRL 1, EKF2_RNG_CTRL 1, MC_AIRMODE 0, CA_ROTOR_COUNT 9,
  MC_PITCH_P 6, MC_PITCHRATE_P 0.9, MC_YAW_P 2.8, MC_YAWRATE_P 0.15.
- `commander status`: disarmed, navigation mode **Hold** (not Position this time; irrelevant now, the module sets
  Stabilized itself). `nose_lift status`: disarmed, pitch -22.6 deg on the bench, switch "no RC" (transmitter off).
- Backup `results/board_params/params_20261007_130636_pre_flight_20261007.json`. BUG: `board_params.py backup` stops
  at 260 of 1243 parameters: a second MAVLink component answers the list request with its own param_count 55 and the
  loop takes that count. The 6 Oct backup has the same gap. Use `get` for anything beyond CA_ROTOR6.
