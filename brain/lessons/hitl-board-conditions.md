# HITL on the 6X Pro needs board conditions SITL does not have

Found 9 Oct 2026 (brain/experiments/2026-10-09-hitl-tremble-and-jolts.md). Set them for HITL only, undo for flight
(`results/hitl_pack/board_restore_for_flight.json`):

- RC_MAP_ARM_SW 8 (the real radio's arm switch): the simulator never sends channel 8, arming is refused.
  HITL: RC_MAP_ARM_SW 0.
- RC_CRSF_PRT_CFG 102 (ExpressLRS on TELEM2): the CRSF driver owns input_rc instance 0 and rc_update reads ONLY
  instance 0, so the simulator's RC_CHANNELS_OVERRIDE (instance 1) is ignored ("No manual control input").
  HITL: RC_CRSF_PRT_CFG 0 (reboot). PX4IO was not the cause.
- MBE_ENABLE 1 / SENS_IMU_AUTOCAL 1 learn offsets in flight and save them for the SIMULATED sensors
  (CAL_MAG3/CAL_GYRO3, device ids 197388/1310988); without lockstep they learn wrong values and every boot starts
  with a different heading/tilt error (1-4 deg yaw, 0.2-0.85 deg roll). HITL: both 0, CAL_*3 offsets 0.
- The winner scenarios now also set RC3_MIN 1000 (the board's calibrated 1001 vs the scenario's RC3_TRIM 1000 failed
  the RC check); they also move the kill switch to channel 9 (board: 5) - restore for flight.
- Do not write files with long `echo` lines through the MAVLink shell: it hung the board (USB replug needed).
