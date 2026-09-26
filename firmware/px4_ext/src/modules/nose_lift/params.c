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
 * Nose fans hold the pitch for the whole flight
 *
 * 0: the throttle-up hands the nose fans over to PX4 (NL_HO_*). 1: they never go to PX4. From the hold, the
 * throttle-up starts the flight hold: the nose fans keep NL_TGT on their own loop (NL_F_*) while PX4 flies the
 * rear fans (thrust, roll, yaw). Set PX4's pitch rate gains to 0 with it, or PX4 pitches with the rear fans
 * against the nose fans. After landing, the switch off with the throttle at minimum lowers the nose. The kill
 * switch stops every motor as before.
 *
 * @boolean
 * @group Nose Lift
 */
PARAM_DEFINE_INT32(NL_FLY_HOLD, 0);

/**
 * Flight hold: pitch angle gain
 *
 * Pitch rate asked per degree of pitch error.
 *
 * @unit 1/s
 * @min 0
 * @max 10
 * @decimal 2
 * @group Nose Lift
 */
PARAM_DEFINE_FLOAT(NL_F_K_ANG, 2.0f);

/**
 * Flight hold: pitch rate limit
 *
 * @unit deg/s
 * @min 1
 * @max 90
 * @decimal 1
 * @group Nose Lift
 */
PARAM_DEFINE_FLOAT(NL_F_RATE, 30.0f);

/**
 * Flight hold: pitch rate gain
 *
 * Nose-fan thrust fraction per deg/s of pitch rate error.
 *
 * @min 0
 * @max 0.2
 * @decimal 4
 * @group Nose Lift
 */
PARAM_DEFINE_FLOAT(NL_F_KQ, 0.015f);

/**
 * Flight hold: pitch rate integral gain
 *
 * @min 0
 * @max 0.5
 * @decimal 4
 * @group Nose Lift
 */
PARAM_DEFINE_FLOAT(NL_F_KQI, 0.03f);

/**
 * Flight hold: nose-fan thrust per rear-fan thrust
 *
 * Feed-forward: the nose fans' mean thrust fraction is this times the rear fans' mean thrust (as PX4 commands
 * them), the ratio at which their pitch moments about the centre of gravity cancel. The integral covers the error.
 *
 * @min 0
 * @max 5
 * @decimal 3
 * @group Nose Lift
 */
PARAM_DEFINE_FLOAT(NL_F_FF, 1.5f);

/**
 * Flight hold: nose angle at liftoff thrust
 *
 * In the flight hold the nose angle follows the rear fans' mean thrust: NL_TGT with them idle, rising in
 * proportion to their thrust to this angle at NL_F_TGT_THR and held there above it, moving at most NL_F_TGT_RATE
 * (tilting the rear fans' forward push back as it builds; down again as the throttle comes down).
 *
 * @unit deg
 * @min 0
 * @max 60
 * @decimal 1
 * @group Nose Lift
 */
PARAM_DEFINE_FLOAT(NL_F_TGT, 26.0f);

/**
 * Flight hold: rear-fan thrust that counts as liftoff
 *
 * Mean thrust fraction PX4 commands to the rear fans (low-passed over 0.3 s) at which the nose reaches NL_F_TGT.
 * About the hover thrust (MPC_THR_HOVER); with THR_MDL_FAC 1, 0.30 is ~1540 us on a rear fan.
 *
 * @min 0
 * @max 1
 * @decimal 2
 * @group Nose Lift
 */
PARAM_DEFINE_FLOAT(NL_F_TGT_THR, 0.30f);

/**
 * Flight hold: rate of the nose's move between NL_TGT and NL_F_TGT
 *
 * @unit deg/s
 * @min 0.5
 * @max 20
 * @decimal 1
 * @group Nose Lift
 */
PARAM_DEFINE_FLOAT(NL_F_TGT_RATE, 4.0f);

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
 * Ceiling above the target
 *
 * Above NL_TGT + NL_CEIL the raise and the hold bring the nose back the way the lowering does (down at NL_RATE on
 * the lowering gains NL_LOW_KQ / NL_LOW_KQI) until it is back at NL_TGT, then hold it there again. 0 disables it.
 * NL_OVERSHOOT still cuts the motors further up.
 *
 * @unit deg
 * @min 0
 * @max 20
 * @decimal 1
 * @group Nose Lift
 */
PARAM_DEFINE_FLOAT(NL_CEIL, 0.0f);

/**
 * Height gain that counts as leaving the ground during the lift
 *
 * Both the height estimate (climbing faster than 0.3 m/s) and the low-passed barometric altitude must have risen
 * this much since the lift started; the estimate alone when no baro is available.
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
