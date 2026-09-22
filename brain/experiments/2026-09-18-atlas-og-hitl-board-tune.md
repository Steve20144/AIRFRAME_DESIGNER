# Experiment: ATLAS_OG hover tune on the Pixhawk 6X Pro (HITL)

Date: 2026-09-18

## Question

Why did ATLAS_OG wobble in Position mode on the board, and which parameters hover it cleanly?

## Setup

HITL over USB, Position mode, nose lift + PX4 Takeoff + 20 s hover started from the app (`/api/scenario/start`).
Baseline: rate P 0.10, stock allocator (CA_METHOD 0).

## Results

- Baseline wobbled at 0.5 Hz, pitch 3.7 to 6 deg RMS, sometimes diverged. Cause: the only yaw authority is the
  two canted nose fans; the pseudo-inverse allocator with airmode saturated motors on yaw demand (one fan at idle,
  others at 80 to 87 percent); yaw ran away and dragged roll and pitch. Lowering rate P made it worse.
- CA_METHOD 1 (sequential desaturation) alone: hover pitch 1.0 deg RMS.
- Final set (in `atlas_og.json` overrides): CA_METHOD 1, MC_YAW_WEIGHT 0.1, MC_YAW_P 1.5, MC_YAWRATE_P 0.05,
  MC_YAWRATE_I 0.03, MC_YR_INT_LIM 0.1, MC_YAWRATE_MAX 30, rate P 0.20, rate D 0.004, LNDMC_TRIG_TIME 0.5.
  Board hover: pitch 0.19 deg RMS, roll 0.19, rates 1.0 deg/s, position 2 to 4 cm. Saved to flash.
- A manual cruise round showed 9 to 13 deg attitude tracking error on stick inputs and a tip-over after a
  9.6 m/s push; position-loop softening (MPC_VEL_MANUAL 3, MPC_ACC_HOR 1.5, ACC_HOR_MAX 2, JERK_MAX 4) is in the
  file as a lab ceiling. MC_PITCH_P 3 made it worse.
- Board attitude vs sim truth differs by SENS_BOARD_Y_OFF (26 deg) in pitch, otherwise agrees to about 1 deg.

## Conclusion

Yaw saturation in the allocator, not rate gains, was the wobble. Landing on the +8 stand tipped the aircraft
(PX4 holds 26 deg until land detection); the nose-lower sequence was built for it the same day.

## Follow-up

Rate P above 0.20 (0.30, 0.40) was queued for the board and later chosen in SITL (0.30). See
experiments/2026-09-21-atlas-og-flat-gain-sweeps.md.
