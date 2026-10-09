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
 * Integral cap of the nose lift loop while raising
 *
 * The integral (deg of rate error times s) may not exceed this either way; times NL_KQI it is the thrust
 * fraction the integral can add. 7 Oct: the old fixed cap of 40 deg (0.48 of thrust) wound up on a stuck
 * nose and threw it through the target. The integral is also frozen while the command sits at its limit
 * and dropped to zero when the nose runs faster than the wanted rate by NL_RATE.
 *
 * @unit deg
 * @min 0
 * @max 40
 * @decimal 1
 * @group Nose Lift
 */
PARAM_DEFINE_FLOAT(NL_I_MAX, 10.0f);

/**
 * Low-pass corner of the pitch rate the nose lift loops damp on
 *
 * On its legs the frame rocks at 10 to 15 Hz (+-20 deg/s); damping on the raw rate chopped the nose command
 * between 0.5 and 1.0 at that frequency (7 Oct). 2 Hz cuts the rocking five times and delays a 3 deg/s rotation
 * by about 80 ms. Higher = less filtering.
 *
 * @unit Hz
 * @min 0.5
 * @max 50
 * @decimal 1
 * @group Nose Lift
 */
PARAM_DEFINE_FLOAT(NL_Q_FC, 2.0f);

/**
 * Stuck nose abort while raising
 *
 * Full command (NL_MAX_CMD) for this long with less than 1 deg of pitch motion cancels the lift (controlled
 * lowering, "nose stuck"): something holds the nose and pushing on only stores up a breakaway. 0 disables.
 *
 * @unit s
 * @min 0
 * @max 10
 * @decimal 1
 * @group Nose Lift
 */
PARAM_DEFINE_FLOAT(NL_STUCK_S, 2.0f);

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

/**
 * Automatic takeoff, hover and landing
 *
 * 0: the pilot's throttle flies. 1: once the nose lift reaches Holding the module waits NL_AUTO_WAIT, then flies
 * the throttle itself: a baro height loop climbs to NL_AUTO_ALT at NL_AUTO_VUP, hovers NL_AUTO_HOV, descends at
 * NL_AUTO_VDN to touchdown, cuts, and lowers the nose. The pilot's roll, pitch and yaw sticks stay live
 * (Stabilized); the nose-lift switch off in the air lands at once; the throttle stick above NL_AUTO_PILOT hands the
 * throttle back to the pilot; the kill switch stops every motor at any time.
 *
 * @boolean
 * @group Nose Lift
 */
PARAM_DEFINE_INT32(NL_AUTO, 0);

/**
 * Hand the hover and the landing to PX4 (Position hold, then Land)
 *
 * 0: the module hovers and lands on its own baro height loop (Stabilized throughout). 1: once the module's climb
 * reaches NL_AUTO_ALT, it has hovered 1 s there and the estimator has a valid horizontal position and velocity
 * (H-FLOW), it asks PX4 for Hold (position and height held by PX4); after NL_AUTO_HOV in Hold it asks for Land
 * (PX4 descends at MPC_LAND_SPEED to the land detector), then cuts and lowers the nose. If Hold is not granted
 * within 2 s, or PX4 leaves Hold or Land for anything else, the module asks for Stabilized back and lands on its
 * own loop. The module keeps a mid-stick throttle under PX4 the whole time so a fall-back into a manual mode
 * holds height rather than descending. Set COM_RC_OVERRIDE 0, or a stick touch drops PX4 out of Hold.
 *
 * @boolean
 * @group Nose Lift
 */
PARAM_DEFINE_INT32(NL_AUTO_HOLD, 0);

/**
 * Takeoff gate: the highest nose command that may hold the nose before an automatic takeoff
 *
 * At the end of NL_AUTO_WAIT the nose command averaged over the last second is compared with this; above it the
 * automatic takeoff is refused and the nose is lowered ("nose fans too weak"): the nose fans have too little margin
 * left for the handover. Units: the module's motor command (SITL 8 Oct: pass at 0.82, crash at 0.85). 0 disables.
 *
 * @min 0
 * @max 1
 * @decimal 2
 * @group Nose Lift
 */
PARAM_DEFINE_FLOAT(NL_AUTO_HCMD, 0.83f);

/**
 * Takeoff guard: nose drop that cancels the automatic takeoff in the handover
 *
 * In the first 3 s of the automatic climb and below 0.3 m above the takeoff point, a nose more than this below
 * the target cuts the automatic throttle (0.5 s) and lowers the nose (7 Oct 15:38: 8.8 -> -26 deg on the legs with
 * the nose fans at full). 0 disables.
 *
 * @unit deg
 * @min 0
 * @max 20
 * @decimal 1
 * @group Nose Lift
 */
PARAM_DEFINE_FLOAT(NL_AUTO_HDROP, 5.0f);

/**
 * Flare height of the module's own landing
 *
 * Below this height above the takeoff point the descent slows to NL_AUTO_VFL.
 *
 * @unit m
 * @min 0
 * @max 3
 * @decimal 2
 * @group Nose Lift
 */
PARAM_DEFINE_FLOAT(NL_AUTO_HFL, 0.7f);

/**
 * Flare descent speed of the module's own landing
 *
 * 30 Sep SITL: 0.15 m/s from 0.7 m gave 0.17 m/s touchdowns, without a flare 0.39 m/s and a jolt at the feet.
 *
 * @unit m/s
 * @min 0.05
 * @max 1
 * @decimal 2
 * @group Nose Lift
 */
PARAM_DEFINE_FLOAT(NL_AUTO_VFL, 0.15f);

/**
 * Touchdown stage at which the module takes over the landing
 *
 * The land detector stage, below 0.6 m above the takeoff point, at which the automatic flight cuts the throttle and
 * the module lowers the nose under control: 0 landed (PX4's last stage, ~5 s after the feet touch, and PX4 lets
 * the nose fall forward meanwhile), 1 maybe landed, 2 ground contact (the first stage, still 2.5 to 3.5 s late in
 * SITL), 3 the module's own cue: still (|vz| < 0.06 m/s) for 0.25 s, or the nose 2 deg under the target while
 * hardly descending; the earlier stages still count.
 *
 * @min 0
 * @max 3
 * @value 0 landed
 * @value 1 maybe landed
 * @value 2 ground contact
 * @value 3 the module's own cue
 * @group Nose Lift
 */
PARAM_DEFINE_INT32(NL_AUTO_TD, 3);

/**
 * Climb in PX4 Position mode from the lift-off
 *
 * 2: hand over to Position as soon as the throttle ramp reaches the hover thrust, still on the legs.
 * With NL_AUTO_HOLD 1: as soon as the aircraft is off its legs (estimator height > 6 cm and climbing > 0.1 m/s)
 * PX4 Position mode takes over and the module climbs it with a climb-rate stick (NL_AUTO_PKZ, NL_AUTO_PACC, at most
 * NL_AUTO_VUP) to NL_AUTO_ALT; the hover clock starts there. 0: the module climbs in Stabilized and PX4 Position
 * takes over at the hover (8 Oct SITL: 3.7 m of drift in the climb and a 20 to 38 deg/s braking jolt).
 *
 * @min 0
 * @max 2
 * @group Nose Lift
 */
PARAM_DEFINE_INT32(NL_AUTO_PCLB, 1);

/**
 * Take off in PX4 Position mode from the legs
 *
 * With NL_AUTO_HOLD 1 and NL_AUTO_PCLB 1: at the hold the module asks for Position instead of Stabilized and the
 * automatic takeoff is a climb-rate stick eased in from 0 (PX4's own takeoff ramp, MPC_TKO_RAMP_T, lifts it off
 * with the position held from the start). 8 Oct SITL: from Stabilized the aircraft slid forward on its legs and
 * after the lift-off (0.5 m/s at 30 N) and PX4 Position's braking was the worst remaining jolt. 0: Stabilized
 * takeoff, Position from the lift-off.
 *
 * @boolean
 * @group Nose Lift
 */
PARAM_DEFINE_INT32(NL_AUTO_PTKO, 0);

/**
 * Keep the nose-fan floor until the lift-off (automatic flights)
 *
 * 1: in an automatic flight the handover's floor under the nose fans is faded only once the aircraft is off its
 * legs (range finder 3 cm above its reading at the takeoff) and PX4 has taken over; 0: as soon as PX4 has taken over.
 *
 * @boolean
 * @group Nose Lift
 */
PARAM_DEFINE_INT32(NL_HO_LIFT, 1);

/**
 * Lift-off height for NL_HO_LIFT
 *
 * @unit m
 * @min 0.01
 * @max 0.5
 * @decimal 2
 * @group Nose Lift
 */
PARAM_DEFINE_FLOAT(NL_HO_LIFT_H, 0.06f);

/**
 * Rear-thrust feed-forward to the nose fans in the handover
 *
 * Added to the nose fans' thrust fraction per unit of the rear fans' mean thrust (PX4's), while the nose is still
 * held by the module on its legs. 0: none.
 *
 * @min 0
 * @max 2
 * @decimal 2
 * @group Nose Lift
 */
PARAM_DEFINE_FLOAT(NL_HO_FF, 0.0f);

/**
 * Climb-rate gain of the Position climb
 *
 * Climb rate asked = this x (NL_AUTO_ALT - height), at most NL_AUTO_VUP.
 *
 * @unit 1/s
 * @min 0.1
 * @max 5
 * @decimal 2
 * @group Nose Lift
 */
PARAM_DEFINE_FLOAT(NL_AUTO_PKZ, 1.0f);

/**
 * Climb-rate change limit of the Position climb
 *
 * @unit m/s^2
 * @min 0.05
 * @max 5
 * @decimal 2
 * @group Nose Lift
 */
PARAM_DEFINE_FLOAT(NL_AUTO_PACC, 0.5f);

/**
 * Descend in PX4 Position mode (instead of PX4 Land)
 *
 * After the hover the module descends in Position with a climb-rate stick: NL_AUTO_VDN easing (NL_AUTO_PKZ) to
 * NL_AUTO_VTD at NL_AUTO_HTD above the takeoff point, then NL_AUTO_VTD to the touchdown (the module's own cue
 * below 0.15 m), then the nose lowering. 0: PX4 Land.
 *
 * @boolean
 * @group Nose Lift
 */
PARAM_DEFINE_INT32(NL_AUTO_PDSC, 1);

/**
 * Height of the Position climb and descent from the range finder
 *
 * 1: the height above the takeoff point is the range finder's reading minus its reading at the takeoff (8 Oct
 * SITL: the estimator's height crept up 0.3 to 0.4 m in a flight). 0 (or no fresh range): the estimator's height.
 *
 * @boolean
 * @group Nose Lift
 */
PARAM_DEFINE_INT32(NL_AUTO_RNG, 1);

/**
 * Touchdown speed of the Position descent
 *
 * @unit m/s
 * @min 0.02
 * @max 0.5
 * @decimal 2
 * @group Nose Lift
 */
PARAM_DEFINE_FLOAT(NL_AUTO_VTD, 0.1f);

/**
 * Height above the takeoff point where the Position descent reaches NL_AUTO_VTD
 *
 * @unit m
 * @min 0
 * @max 2
 * @decimal 2
 * @group Nose Lift
 */
PARAM_DEFINE_FLOAT(NL_AUTO_HTD, 0.25f);

/**
 * Rear-fan fade-out at the lowering after a flight
 *
 * The rear fans keep the thrust PX4 last gave them and fade out over this time while the nose fans lower the nose
 * (0: off at once, the old behaviour).
 *
 * @unit s
 * @min 0
 * @max 5
 * @decimal 2
 * @group Nose Lift
 */
PARAM_DEFINE_FLOAT(NL_LOW_RRAMP, 1.0f);

/**
 * Lowering: least nose-down rate until the front leg stops the nose
 *
 * The lowering aims at the pitch PX4 estimated before the lift; that estimate can be 1 to 2 deg off (9 Oct HITL: the
 * nose stopped above its leg, the fade cut the fans and it fell at 12 deg/s). With a value above 0 the nose keeps
 * coming down at least this fast and the motors fade once the leg holds it still (or 3 deg below the start pitch).
 * 0: the old behaviour (fade at the start pitch).
 *
 * @unit deg/s
 * @min 0
 * @max 3
 * @decimal 2
 * @group Nose Lift
 */
PARAM_DEFINE_FLOAT(NL_LOW_VMIN, 1.0f);

/**
 * Attitude errors PX4 does not hold against the legs
 *
 * While the aircraft stands on its legs under the ground sequence (raise, hold, handover until it is off its legs,
 * lowering) a roll or pitch error cannot be corrected: PX4's attitude loop only loads the legs and the load comes out
 * as a jolt when they leave the ground (9 Oct HITL: 0.4 deg of estimated tilt gave a 5-6 deg/s roll kick). With the
 * bit set, mc_att_control holds that setpoint at the estimated angle there (offset up to 3 deg) and fades the offset
 * out over NL_LEGS_FADE once the aircraft is off its legs.
 *
 * @bit 0 roll
 * @bit 1 pitch
 * @min 0
 * @max 3
 * @group Nose Lift
 */
PARAM_DEFINE_INT32(NL_LEGS_ATT, 1);

/**
 * Off the legs: fade time of the attitude setpoint offset (NL_LEGS_ATT)
 *
 * @unit s
 * @min 0.2
 * @max 5
 * @decimal 1
 * @group Nose Lift
 */
PARAM_DEFINE_FLOAT(NL_LEGS_FADE, 1.0f);

/**
 * Handover: angle gain of the nose hold
 *
 * Wanted nose rate per degree of pitch error while PX4 takes the aircraft over on its legs (0: NL_K_ANG). The rear
 * fans' thrust rising through the throttle ramp takes the nose down; a stiffer hold keeps it at NL_TGT so the aircraft
 * leaves the ground level and does not slide forward.
 *
 * @unit 1/s
 * @min 0
 * @max 10
 * @decimal 1
 * @group Nose Lift
 */
PARAM_DEFINE_FLOAT(NL_HO_KANG, 0.0f);

/**
 * Automatic flight: wait at the hold before the takeoff
 *
 * @unit s
 * @min 0
 * @max 30
 * @decimal 1
 * @group Nose Lift
 */
PARAM_DEFINE_FLOAT(NL_AUTO_WAIT, 3.0f);

/**
 * Automatic flight: hover height above the takeoff point (barometric)
 *
 * @unit m
 * @min 0.3
 * @max 10
 * @decimal 2
 * @group Nose Lift
 */
PARAM_DEFINE_FLOAT(NL_AUTO_ALT, 1.5f);

/**
 * Automatic flight: hover time
 *
 * @unit s
 * @min 1
 * @max 300
 * @decimal 1
 * @group Nose Lift
 */
PARAM_DEFINE_FLOAT(NL_AUTO_HOV, 30.0f);

/**
 * Automatic flight: climb rate
 *
 * @unit m/s
 * @min 0.1
 * @max 2
 * @decimal 2
 * @group Nose Lift
 */
PARAM_DEFINE_FLOAT(NL_AUTO_VUP, 0.5f);

/**
 * Automatic flight: descent rate
 *
 * @unit m/s
 * @min 0.1
 * @max 2
 * @decimal 2
 * @group Nose Lift
 */
PARAM_DEFINE_FLOAT(NL_AUTO_VDN, 0.3f);

/**
 * Automatic flight: hover throttle feed-forward (stick position)
 *
 * Throttle stick position (0..1) at which the aircraft hovers. 0: mid stick (0.5) with MPC_THR_CURVE 0 or 2, where
 * PX4 rescales the stick so that mid stick is the hover thrust, or MPC_THR_HOVER with curve 1. The height loop
 * corrects around it.
 *
 * @min 0
 * @max 1
 * @decimal 3
 * @group Nose Lift
 */
PARAM_DEFINE_FLOAT(NL_AUTO_THR, 0.0f);

/**
 * Automatic flight: throttle slew limit
 *
 * The automatic throttle moves at most this much per second (not the takeoff ramp, not the cut).
 *
 * @unit 1/s
 * @min 0.05
 * @max 2
 * @decimal 2
 * @group Nose Lift
 */
PARAM_DEFINE_FLOAT(NL_AUTO_SLEW, 0.3f);

/**
 * Automatic flight: throttle ceiling
 *
 * @min 0.3
 * @max 1
 * @decimal 2
 * @group Nose Lift
 */
PARAM_DEFINE_FLOAT(NL_AUTO_MAX, 0.9f);

/**
 * Automatic flight: throttle ramp from zero to the hover throttle at the takeoff
 *
 * @unit s
 * @min 0.5
 * @max 10
 * @decimal 1
 * @group Nose Lift
 */
PARAM_DEFINE_FLOAT(NL_AUTO_RAMP, 2.0f);

/**
 * Automatic flight: height gain
 *
 * Throttle per metre of height error.
 *
 * @min 0
 * @max 1
 * @decimal 3
 * @group Nose Lift
 */
PARAM_DEFINE_FLOAT(NL_AUTO_KP, 0.25f);

/**
 * Automatic flight: climb-rate gain
 *
 * Throttle per m/s of climb-rate error (the barometric rate).
 *
 * @min 0
 * @max 1
 * @decimal 3
 * @group Nose Lift
 */
PARAM_DEFINE_FLOAT(NL_AUTO_KV, 0.35f);

/**
 * Automatic flight: height integral gain
 *
 * @min 0
 * @max 0.5
 * @decimal 3
 * @group Nose Lift
 */
PARAM_DEFINE_FLOAT(NL_AUTO_KI, 0.08f);

/**
 * Automatic flight: pilot takeover threshold on the raw throttle channel
 *
 * The throttle channel (RC_MAP_THROTTLE) above this many microseconds hands the throttle back to the pilot's stick
 * for the rest of the flight. 0 disables the takeover.
 *
 * @unit us
 * @min 0
 * @max 2100
 * @group Nose Lift
 */
PARAM_DEFINE_INT32(NL_AUTO_PILOT, 1400);
