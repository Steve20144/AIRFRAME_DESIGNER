# Real-flight parameter set: ATLAS V3_30 with the CFD foils (2026-10-01)

File: `atlas_v3_30_cfd_real_flight_20261001.params` (175 parameters, QGroundControl format).
Made by `scripts/real_flight_params.py` from `airframes/atlas_v3_30_cfd.json`, diffed against a read-only dump of
the board (`results/board_params/params_20261001_211932_readonly_decoded.json`): 68 values change, listed in
`atlas_v3_30_cfd_real_flight_20261001.diff.json`. **Nothing has been written to the board.**

## Assumption: which aircraft this is

The real aircraft = V3_30 CAD masses (14.40 kg) with foils **75 / 65 / 50 deg** and the thrust the OpenFOAM CFD
measured (each foil group 72.9 N at 39.6 deg above the fan axis). If the foils on the aircraft are different, this
file is wrong for it: say which foils are fitted and it is regenerated.

## What it sets

| Area | Value | Effect |
|---|---|---|
| HITL off | `SYS_HITL 0` | real sensors, real motor outputs |
| Transmitter | `COM_RC_IN_MODE 0` | RC only: arm switch ch 8, kill switch ch 5 work |
| Disarm, idle on the ground | `COM_DISARM_PRFLT 10` | armed without taking off: disarms after 10 s |
| Disarm, after landing | `COM_DISARM_LAND 2` | disarms 2 s after landing |
| Nose lift | `NL_EN 1`, `NL_TGT / NL_HOV_PITCH 42.7`, RC ch 7 | firmware lifts the nose from the legs to the hover pitch |
| Geometry | `CA_ROTOR*`, `SENS_BOARD_Y_OFF 42.7`, `MPC_THR_HOVER 0.652` | the CFD airframe, hover 42.7 deg |
| Arming tilt | `FD_FAIL_P 70` | needed to arm parked at -17.6 deg (60.3 deg from the hover frame); it also means pitch failure detection only trips at 70 deg |
| H-FLOW | `UAVCAN_SUB_FLOW 1`, `UAVCAN_SUB_RNG 1`, offsets, `EKF2_RNG_PITCH 0.391 rad` | real sensor on, mount 20.3 deg |
| Flow gyro | `EKF2_OF_GYR_SRC 1` | flow compensated with the autopilot's rotated gyro |
| Estimator | `EKF2_MULTI_IMU 3`, `SENS_IMU_MODE 0`, `SENS_IMU_AUTOCAL 1`, `EKF2_DECL_TYPE 3` | 6X defaults back |
| Gains | `MC_PITCHRATE_P 0.9 / I 0.05`, roll / yaw as tuned, `MC_AIRMODE 0` | SITL-tuned |

Not included on purpose: output functions (motors 1-8 on MAIN 1-8 and motor 9 on AUX 4 stay as wired), sensor
calibrations, `CBRK_SUPPLY_CHK` (stays 0).

## Decide or check before loading

1. **Safety switch: still disabled** (`CBRK_IO_SAFETY 22027`, kept as the board has it). Arming then means motors
   spin within 1 s, no button. If the GPS/safety-button module is plugged in and you want it, set
   `CBRK_IO_SAFETY 0`: the button must then be pressed before it can arm.
2. **Battery:** `BAT1_N_CELLS` is 2 on the board, which does not match the 5200 mAh packs. Set your real cell count
   (and the voltage divider / power module), otherwise battery % is meaningless. Low battery action is warning only
   (`COM_LOW_BAT_ACT 0`).
3. **Flight-mode channel:** `RC_MAP_FLTMODE 0` (none). Map your mode switch, or it arms in whatever mode it is in.
4. **H-FLOW angle:** at a 42.7 deg hover a sensor fixed at 20.3 deg looks **22.4 deg off vertical**. PX4 corrects the
   range for that tilt but not the flow, so position hold on flow will be poor. Re-angle the mount to ~43 deg for
   this airframe (or to the hover pitch of whichever design flies).
5. **GPS:** `EKF2_GPS_CTRL 0` (flow and range only), as in the sim. Outdoors with a GPS fitted, set it to 7.
6. **Flow driver params** `SENS_FLOW_MAXHGT / MINHGT / ROT` only appear once the H-FLOW is subscribed: QGC may skip
   them on the first load; load the file again after the reboot.
7. **Reboot** after loading (`SYS_HITL`, IMU filters and multi-IMU need it), then re-check that motor 9 is on AUX 4.

## Props-off test sequence

1. Props off, battery in, transmitter on. Kill switch (ch 5) engaged: arming must be refused.
2. Kill off, arm on ch 8: all 9 motors spin at idle within 1 s. Check each motor's position and direction
   (Motor Test in QGC; `COM_MOT_TEST_EN 1`).
3. Leave it armed, do nothing: it must disarm by itself after 10 s.
4. Arm, engage kill: motors stop at once.
5. Nose lift: with the aircraft on its legs, the NL RC switch (ch 7) starts the lift; the nose fans should raise it
   towards 42.7 deg and hand over to PX4.
6. Switch the transmitter off while armed (props off): it must go to Land (`NAV_RCL_ACT 2`) after 0.5 s.

## How it will fly (from SITL, honest)

This airframe has very little thrust headroom: the busiest fan sits at ~91-93 % of full thrust in hover, and the live
SITL flight with it saturated the fans at lift-off, rolled 12 deg and drifted (that flight also had the sim-clock
bug, since fixed, so it was somewhat worse than the airframe alone). Expect sluggish control and little margin for
gusts. The designs that hover at 20 deg or less (jet60 park0, jets 57.5/85/85 park -10) need foils that do not exist
yet.
