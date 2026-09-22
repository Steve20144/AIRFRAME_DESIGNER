# Research: indoor positioning options for ATLAS

## Question

How to give PX4 a position source indoors (no GPS), and whether the two Holybro GPS modules on hand can help.

## Findings

- **Stabilised vs positioned.** Rate and attitude loops need only the IMU; altitude needs a baro or rangefinder;
  only the position loop needs a position source. A quad flies indoors in Stabilized with the pilot as position
  loop. ATLAS could not until the flat brackets (decision 2026-09-21).
- **Holybro H-Flow** (PAA3905 flow + AFBR-S50 ToF + ICM-42688, DroneCAN, 0.08 to 30 m, 42 deg FoV, 7.4 rad/s,
  above 5 lux, 15 g). PX4 v1.17 supports it (fmu-v6x build includes DroneCAN; it starts outside the HITL branch).
  On ATLAS it must be mounted 26 deg nose-down on the structure so it points straight down in hover: EKF2 rejects
  range and flow beyond 45 deg from vertical and SENS_FLOW_ROT is yaw-only. Parameters: UAVCAN_ENABLE 2,
  UAVCAN_SUB_FLOW/RNG 1, UAVCAN_RNG_MIN 0.08 / MAX 30, EKF2_OF_CTRL 1, EKF2_RNG_CTRL 1, EKF2_GPS_CTRL 0,
  EKF2_HGT_REF 2, EKF2_RNG_A_HMAX 10, EKF2_RNG_QLTY_T 0.2, SENS_FLOW_MINHGT 0.08, SENS_FLOW_MAXHGT = ceiling,
  SENS_FLOW_MAXR 7.4, EKF2_OF_POS_* / EKF2_RNG_POS_* from the mount point (same CG-relative hover-frame rotation as
  the rotor export). Needs floor texture under the exhaust of ten EDFs; velocity-only, so the hold drifts. Bench
  HITL must turn the subscriptions off. The simulator needs HIL_OPTICAL_FLOW + DISTANCE_SENSOR emulation (PX4's
  simulator bridge accepts both) before EKF2_GPS_CTRL 0 can go into the airframe overrides.
- **Two GPS receivers** cannot form an indoor system: passive receivers, satellites attenuated below tracking
  indoors; moving-baseline RTK is outdoor-only. **A GPS repeater** (roof antenna, LNA, indoor patch) gives every
  receiver the roof antenna's fixed position and zero Doppler: in Position mode that is a flyaway, not a hold.
  Useful only for bench checks of the GPS driver, compass, arming and timestamps; licence-restricted (illegal for
  civilians in the US, Ofcom licence in the UK, per-country in the EU). Keep the modules: their magnetometers are
  a heading source away from ESC currents.
- **What PX4 v1.17 accepts as external position:** ATT_POS_MOCAP, VISION_POSITION_ESTIMATE, ODOMETRY via
  EKF2_EV_CTRL. Not GPS_INPUT (ArduPilot's fake-GPS message). HIL_GPS only in HITL mode.
- **Commercial indoor GPS** (Marvelmind ultrasound beacons, NMEA over UART into the GPS port, ~2 cm, 8 to 16 Hz):
  the literal "mock GPS"; ask about tolerance to ten EDFs' noise.
- **External vision** (fixed camera + AprilTag on the airframe, or downward camera + tag mat, companion computer
  publishing ODOMETRY): most accurate, gives yaw, most work.

## Implications

Order of preference for ATLAS: pilot in Stabilized (now possible on flat brackets) -> H-Flow for hover / takeoff /
landing work -> external vision for repeatable trajectories and absolute comparison with the simulator.

## Sources

Holybro H-Flow overview and setup guide (docs.holybro.com), PX4 optical flow guide (docs.px4.io), PX4 v1.17 source
(`mavlink_receiver.cpp`, `ekf2/EKF/common.h` range_cos_max_tilt 0.7071, `rcS` uavcan start).

## Remaining uncertainties

Floor texture and downwash effects on flow quality; Marvelmind acoustics next to EDFs; whether the team fits flat
brackets or a position source first.
