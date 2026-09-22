/**
 * Nose lift ground sequence: parameters. The AIRFRAME_DESIGNER app computes the geometry ones (moments about the
 * rear feet, the roll/yaw split) from the airframe and pushes them with the rest of the airframe export; the
 * structural frame is the airframe's own, PX4's level is that frame pitched nose-up by NL_HOV_PITCH.
 */

/**
 * Enable the nose lift
 *
 * With this set, arming on the ground holds every motor stopped until the nose-lift switch (NL_RC_CH) is moved
 * to on; the lifting motors then raise the nose to NL_TGT, hold it, and hand over to PX4 when the throttle is
 * raised above NL_HO_THR.
 *
 * @boolean
 * @group Nose Lift
 */
PARAM_DEFINE_INT32(NL_EN, 0);

/**
 * Lifting motors
 *
 * Bit i is motor i+1. At most four.
 *
 * @min 0
 * @max 4095
 * @group Nose Lift
 */
PARAM_DEFINE_INT32(NL_MOT_MSK, 0);

/**
 * Hover pitch of the structural frame
 *
 * PX4's level attitude; the module adds it to PX4's pitch to get the structural pitch.
 *
 * @unit deg
 * @min -90
 * @max 90
 * @decimal 2
 * @group Nose Lift
 */
PARAM_DEFINE_FLOAT(NL_HOV_PITCH, 0.0f);

/**
 * Target structural pitch
 *
 * @unit deg
 * @min -90
 * @max 90
 * @decimal 2
 * @group Nose Lift
 */
PARAM_DEFINE_FLOAT(NL_TGT, 0.0f);

/**
 * Nose rotation rate
 *
 * @unit deg/s
 * @min 0.5
 * @max 30
 * @decimal 1
 * @group Nose Lift
 */
PARAM_DEFINE_FLOAT(NL_RATE, 3.0f);

/**
 * Pitch error to rate gain
 *
 * @unit 1/s
 * @min 0.1
 * @max 10
 * @decimal 2
 * @group Nose Lift
 */
PARAM_DEFINE_FLOAT(NL_K_ANG, 1.0f);

/**
 * Pitch rate error gain (lift)
 *
 * Thrust fraction per deg/s of pitch-rate error.
 *
 * @min 0
 * @max 2
 * @decimal 3
 * @group Nose Lift
 */
PARAM_DEFINE_FLOAT(NL_KQ, 0.02f);

/**
 * Pitch rate integral gain (lift)
 *
 * @min 0
 * @max 2
 * @decimal 3
 * @group Nose Lift
 */
PARAM_DEFINE_FLOAT(NL_KQI, 0.012f);

/**
 * Pitch rate error gain (controlled lowering after a cancel)
 *
 * @min 0
 * @max 2
 * @decimal 3
 * @group Nose Lift
 */
PARAM_DEFINE_FLOAT(NL_LOW_KQ, 0.3f);

/**
 * Pitch rate integral gain (controlled lowering after a cancel)
 *
 * @min 0
 * @max 2
 * @decimal 3
 * @group Nose Lift
 */
PARAM_DEFINE_FLOAT(NL_LOW_KQI, 0.1f);

/**
 * Roll and yaw rate damping in the thrust split
 *
 * @min 0
 * @max 20
 * @decimal 2
 * @group Nose Lift
 */
PARAM_DEFINE_FLOAT(NL_K_RATE, 3.0f);

/**
 * Maximum thrust fraction of the lifting motors
 *
 * @min 0
 * @max 1
 * @decimal 2
 * @group Nose Lift
 */
PARAM_DEFINE_FLOAT(NL_MAX_CMD, 1.0f);

/**
 * Pitch tolerance at the target
 *
 * @unit deg
 * @min 0.2
 * @max 10
 * @decimal 1
 * @group Nose Lift
 */
PARAM_DEFINE_FLOAT(NL_TOL, 2.0f);

/**
 * Time at the target before holding
 *
 * @unit s
 * @min 0
 * @max 10
 * @decimal 1
 * @group Nose Lift
 */
PARAM_DEFINE_FLOAT(NL_HOLD_S, 0.6f);

/**
 * Fade-out time of the lifting motors
 *
 * @unit s
 * @min 0.1
 * @max 10
 * @decimal 1
 * @group Nose Lift
 */
PARAM_DEFINE_FLOAT(NL_FADE_S, 2.0f);

/**
 * Handover timeout
 *
 * After this long in the handover the hold fades out even if PX4 has not reached it.
 *
 * @unit s
 * @min 1
 * @max 30
 * @decimal 1
 * @group Nose Lift
 */
PARAM_DEFINE_FLOAT(NL_HO_TOUT, 8.0f);

/**
 * Lift timeout
 *
 * @unit s
 * @min 5
 * @max 120
 * @decimal 0
 * @group Nose Lift
 */
PARAM_DEFINE_FLOAT(NL_TOUT, 25.0f);

/**
 * Hold timeout
 *
 * Holding longer than this without a takeoff lowers the nose again.
 *
 * @unit s
 * @min 5
 * @max 300
 * @decimal 0
 * @group Nose Lift
 */
PARAM_DEFINE_FLOAT(NL_HOLD_TOUT, 60.0f);

/**
 * Throttle that starts the handover
 *
 * Throttle stick (0 = minimum, 1 = maximum) above which the held nose is handed to PX4.
 *
 * @min 0.02
 * @max 1
 * @decimal 2
 * @group Nose Lift
 */
PARAM_DEFINE_FLOAT(NL_HO_THR, 0.15f);

/**
 * Highest throttle at which the lift may start
 *
 * @min 0
 * @max 0.5
 * @decimal 2
 * @group Nose Lift
 */
PARAM_DEFINE_FLOAT(NL_START_THR, 0.05f);

/**
 * Roll limit during the lift
 *
 * @unit deg
 * @min 1
 * @max 45
 * @decimal 1
 * @group Nose Lift
 */
PARAM_DEFINE_FLOAT(NL_ROLL_MAX, 8.0f);

/**
 * Overshoot limit above the target
 *
 * @unit deg
 * @min 1
 * @max 45
 * @decimal 1
 * @group Nose Lift
 */
PARAM_DEFINE_FLOAT(NL_OVERSHOOT, 12.0f);

/**
 * Height gain that counts as leaving the ground during the lift
 *
 * @unit m
 * @min 0.05
 * @max 5
 * @decimal 2
 * @group Nose Lift
 */
PARAM_DEFINE_FLOAT(NL_LIFT_DZ, 0.5f);

/**
 * Nose-lift switch channel
 *
 * 1-based RC channel of the switch; 0 disables the switch.
 *
 * @min 0
 * @max 18
 * @group Nose Lift
 */
PARAM_DEFINE_INT32(NL_RC_CH, 0);

/**
 * Nose-lift switch threshold
 *
 * @unit us
 * @min 900
 * @max 2100
 * @group Nose Lift
 */
PARAM_DEFINE_INT32(NL_RC_TH, 1500);

/**
 * Nose-lift switch is on below the threshold
 *
 * @boolean
 * @group Nose Lift
 */
PARAM_DEFINE_INT32(NL_RC_LOW, 0);

/**
 * Thrust exponent of the lifting motors
 *
 * thrust = max * command^NL_EXPO
 *
 * @min 1
 * @max 3
 * @decimal 2
 * @group Nose Lift
 */
PARAM_DEFINE_FLOAT(NL_EXPO, 2.0f);

/**
 * Weight
 *
 * @unit N
 * @min 0
 * @max 2000
 * @decimal 2
 * @group Nose Lift
 */
PARAM_DEFINE_FLOAT(NL_WEIGHT, 0.0f);

/**
 * Pivot (mean of the rear feet) relative to the CG, structural x
 *
 * @unit m
 * @min -10
 * @max 10
 * @decimal 4
 * @group Nose Lift
 */
PARAM_DEFINE_FLOAT(NL_PIV_X, 0.0f);

/**
 * Pivot relative to the CG, structural y
 *
 * @unit m
 * @min -10
 * @max 10
 * @decimal 4
 * @group Nose Lift
 */
PARAM_DEFINE_FLOAT(NL_PIV_Y, 0.0f);

/**
 * Pivot relative to the CG, structural z
 *
 * @unit m
 * @min -10
 * @max 10
 * @decimal 4
 * @group Nose Lift
 */
PARAM_DEFINE_FLOAT(NL_PIV_Z, 0.0f);

/**
 * Lifting motor 1: pitch moment about the pivot at full thrust
 *
 * Lifting motors are numbered in NL_MOT_MSK bit order.
 *
 * @unit Nm
 * @decimal 4
 * @group Nose Lift
 */
PARAM_DEFINE_FLOAT(NL_A0, 0.0f);

/**
 * Lifting motor 2: pitch moment about the pivot at full thrust
 *
 * @unit Nm
 * @decimal 4
 * @group Nose Lift
 */
PARAM_DEFINE_FLOAT(NL_A1, 0.0f);

/**
 * Lifting motor 3: pitch moment about the pivot at full thrust
 *
 * @unit Nm
 * @decimal 4
 * @group Nose Lift
 */
PARAM_DEFINE_FLOAT(NL_A2, 0.0f);

/**
 * Lifting motor 4: pitch moment about the pivot at full thrust
 *
 * @unit Nm
 * @decimal 4
 * @group Nose Lift
 */
PARAM_DEFINE_FLOAT(NL_A3, 0.0f);

/**
 * Lifting motor 1: static thrust weight
 *
 * Weight that cancels the net yaw (and roll, with three or more motors) moment of the lifting motors.
 *
 * @decimal 4
 * @group Nose Lift
 */
PARAM_DEFINE_FLOAT(NL_W0, 1.0f);

/**
 * Lifting motor 2: static thrust weight
 *
 * @decimal 4
 * @group Nose Lift
 */
PARAM_DEFINE_FLOAT(NL_W1, 1.0f);

/**
 * Lifting motor 3: static thrust weight
 *
 * @decimal 4
 * @group Nose Lift
 */
PARAM_DEFINE_FLOAT(NL_W2, 1.0f);

/**
 * Lifting motor 4: static thrust weight
 *
 * @decimal 4
 * @group Nose Lift
 */
PARAM_DEFINE_FLOAT(NL_W3, 1.0f);

/**
 * Lifting motor 1: roll moment about the CG at full thrust
 *
 * @unit Nm
 * @decimal 4
 * @group Nose Lift
 */
PARAM_DEFINE_FLOAT(NL_MR0, 0.0f);

/**
 * Lifting motor 2: roll moment about the CG at full thrust
 *
 * @unit Nm
 * @decimal 4
 * @group Nose Lift
 */
PARAM_DEFINE_FLOAT(NL_MR1, 0.0f);

/**
 * Lifting motor 3: roll moment about the CG at full thrust
 *
 * @unit Nm
 * @decimal 4
 * @group Nose Lift
 */
PARAM_DEFINE_FLOAT(NL_MR2, 0.0f);

/**
 * Lifting motor 4: roll moment about the CG at full thrust
 *
 * @unit Nm
 * @decimal 4
 * @group Nose Lift
 */
PARAM_DEFINE_FLOAT(NL_MR3, 0.0f);

/**
 * Lifting motor 1: yaw moment about the CG at full thrust
 *
 * @unit Nm
 * @decimal 4
 * @group Nose Lift
 */
PARAM_DEFINE_FLOAT(NL_MY0, 0.0f);

/**
 * Lifting motor 2: yaw moment about the CG at full thrust
 *
 * @unit Nm
 * @decimal 4
 * @group Nose Lift
 */
PARAM_DEFINE_FLOAT(NL_MY1, 0.0f);

/**
 * Lifting motor 3: yaw moment about the CG at full thrust
 *
 * @unit Nm
 * @decimal 4
 * @group Nose Lift
 */
PARAM_DEFINE_FLOAT(NL_MY2, 0.0f);

/**
 * Lifting motor 4: yaw moment about the CG at full thrust
 *
 * @unit Nm
 * @decimal 4
 * @group Nose Lift
 */
PARAM_DEFINE_FLOAT(NL_MY3, 0.0f);
