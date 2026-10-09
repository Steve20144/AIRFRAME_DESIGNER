/**
 * @file NoseLift.hpp
 *
 * Nose-lift ground sequence for an aircraft that parks nose-down but hovers nose-up (AIRFRAME_DESIGNER).
 *
 * PX4 treats the hover attitude as level, so armed on its legs at the parked attitude it would demand full pitch
 * torque from every motor at once. This module owns the motors on the ground instead:
 *
 *   Parked    armed on the ground: every motor stopped
 *   Ramping   the nose-lift switch went on: the lifting motors (NL_MOT_MSK) raise the nose to NL_TGT at NL_RATE,
 *             on a pitch-rate loop over a static balance feed-forward about the rear feet, the other motors stopped
 *   Holding   at the target: the nose is held; raising the throttle above NL_HO_THR starts the handover
 *   Handover  PX4 drives every motor, the hold stays as a floor under the lifting motors until PX4's own commands
 *             reach it (or NL_HO_TOUT), then fades out over NL_FADE_S
 *   Flying    passive
 *   Lowering  a cancel (switch off, radio lost, hold or lift timeout): the nose is lowered back to where it started
 *             at NL_RATE, then the motors stop and PX4 is disarmed
 *   Aborted   every motor stopped until PX4 is disarmed (kill switch, attitude lost, roll, overshoot, liftoff)
 *
 * NL_AUTO adds an automatic takeoff, hover and landing on top of these states (AutoPhase): at Holding it waits
 * NL_AUTO_WAIT, then asks for the throttle itself (nose_lift_output.throttle, which rc_update puts in place of the
 * pilot's throttle stick): a barometric height loop climbs to NL_AUTO_ALT, hovers NL_AUTO_HOV, descends to
 * touchdown, cuts, then the nose is lowered (Lowering) and PX4 disarmed. Roll, pitch and yaw stay with the pilot's
 * sticks (Stabilized). The switch off in the air lands at once; the throttle stick raised above NL_AUTO_PILOT hands
 * the throttle back to the pilot; the kill switch stops every motor in every phase, as before.
 *
 * The module never drives an output itself: it publishes nose_lift_output, and the control allocator applies it
 * to actuator_motors. PX4's kill switch and disarm act after that, in the output driver, so they cut the motors in
 * every state; a kill also latches Aborted here, so releasing the switch never resumes the sequence.
 */

#pragma once

#include <drivers/drv_hrt.h>
#include <lib/mathlib/mathlib.h>
#include <lib/matrix/matrix/math.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <systemlib/mavlink_log.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/debug_vect.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/input_rc.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/nose_lift_feedback.h>
#include <uORB/topics/nose_lift_output.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_air_data.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>

using namespace time_literals;

class NoseLift : public ModuleBase<NoseLift>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	NoseLift();
	~NoseLift() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();
	int print_status() override;

	enum class State : uint8_t {
		Disabled = 0,
		Disarmed,
		Parked,
		Ramping,
		Holding,
		Handover,
		Flying,
		Lowering,
		Aborted,
		NoseHold,	// NL_FLY_HOLD: the nose fans hold the pitch for the whole flight, PX4 flies the rest
	};

	enum class AutoPhase : uint8_t {
		Off = 0,
		Wait,		// at Holding, NL_AUTO_WAIT before the takeoff
		Climb,		// throttle ramp, then the height target rises at NL_AUTO_VUP to NL_AUTO_ALT
		Hover,		// NL_AUTO_HOV at NL_AUTO_ALT
		HoldWait,	// NL_AUTO_HOLD: PX4 Hold requested at the hover height, waiting for the mode
		HoldPX4,	// PX4 Hold flies (position and height), NL_AUTO_HOV from here
		LandPX4,	// PX4 Land flies down to the land detector
		Descend,	// height target falls at NL_AUTO_VDN through the ground until the aircraft sits still
		Cut,		// throttle to zero over 0.5 s, then the nose lowering
		Done,		// the pilot's stick (or nothing) flies the throttle again
	};

	enum class Abort : uint8_t {
		None = 0,
		Kill,
		SwitchOff,
		RcLost,
		AttitudeLost,
		Roll,
		Overshoot,
		Liftoff,
		LiftTimeout,
		CannotHold,
		HoldTimeout,
		LowerTimeout,
		Stuck,
		Weak,
	};

private:
	static constexpr int MAX_LIFT = 4;
	static constexpr int NUM_MOTORS = sizeof(nose_lift_output_s::control) / sizeof(nose_lift_output_s::control[0]);

	void Run() override;

	void update_attitude(hrt_abstime now);
	void update_switch(hrt_abstime now);
	void update_lift_motors();
	void update_split();
	float balance_fraction() const;
	bool over_ceiling(hrt_abstime now);
	float rear_thrust() const;
	void start_nose_hold(hrt_abstime now);
	void lower_from_nose_hold(hrt_abstime now);
	float thrust_to_cmd(float frac) const;
	float motor_thrust(float command) const;
	float motor_command(float thrust) const;
	float throttle() const;
	bool lifted_off(hrt_abstime now);
	void update_baro();

	void step_auto(hrt_abstime now, float dt, bool kill);
	bool in_stabilized() const;
	void request_stabilized(hrt_abstime now);
	void request_px4_mode(float main_mode, float sub_mode, const char *name, hrt_abstime now);
	bool in_px4_hold() const;
	bool ptko() const;
	bool touched_down(float h, float h_max) const;
	void touchdown_lower(hrt_abstime now, const char *why);
	void update_touchdown(float h, hrt_abstime now);
	float ekf_height(float h_baro) const;
	bool range_ok() const;
	float ekf_climb_rate() const;
	float stick_for_climb(float v_up) const;
	bool in_px4_land() const;
	bool px4_position_ok() const;
	void start_auto_flight(hrt_abstime now);
	void auto_land_now(const char *why, hrt_abstime now);
	void end_auto(hrt_abstime now);
	void lower_from_flight(hrt_abstime now);
	int pilot_throttle_us() const;
	bool auto_flying() const
	{
		return _auto == AutoPhase::Climb || _auto == AutoPhase::Hover || _auto == AutoPhase::HoldWait || _auto == AutoPhase::HoldPX4
		       || _auto == AutoPhase::LandPX4 || _auto == AutoPhase::Descend || _auto == AutoPhase::Cut;
	}
	static const char *auto_name(AutoPhase a);

	void try_start(hrt_abstime now);
	void step_sequence(hrt_abstime now, float dt, bool kill);
	void abort(Abort reason, bool controlled, hrt_abstime now);
	void set_state(State s, hrt_abstime now);
	void request_disarm(hrt_abstime now);
	void publish(hrt_abstime now);

	static const char *state_name(State s);
	static const char *abort_name(Abort a);
	static bool is_sequence(State s) { return s == State::Ramping || s == State::Holding || s == State::Handover || s == State::Lowering || s == State::NoseHold; }

	uORB::Publication<nose_lift_output_s> _output_pub{ORB_ID(nose_lift_output)};
	uORB::Publication<debug_vect_s> _debug_pub{ORB_ID(debug_vect)};
	uORB::Publication<vehicle_command_s> _command_pub{ORB_ID(vehicle_command)};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::Subscription _attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _armed_sub{ORB_ID(actuator_armed)};
	uORB::Subscription _dist_sub{ORB_ID(distance_sensor)};
	uORB::Subscription _act_sub{ORB_ID(actuator_motors)};
	actuator_motors_s _act{};		// PX4's motor outputs (the bumpless start of the lowering after a flight)
	distance_sensor_s _dist{};
	float _auto_rng0{NAN};		// [m] range-finder reading at the start of the automatic flight
	uORB::Subscription _land_sub{ORB_ID(vehicle_land_detected)};
	uORB::Subscription _manual_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription _rc_sub{ORB_ID(input_rc)};
	uORB::Subscription _lpos_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _air_sub{ORB_ID(vehicle_air_data)};
	uORB::Subscription _status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _feedback_sub{ORB_ID(nose_lift_feedback)};

	vehicle_attitude_s _attitude{};
	vehicle_angular_velocity_s _angvel{};
	actuator_armed_s _armed{};
	vehicle_land_detected_s _land{};
	manual_control_setpoint_s _manual{};
	input_rc_s _rc{};
	vehicle_local_position_s _lpos{};
	vehicle_air_data_s _air{};
	vehicle_status_s _status{};
	nose_lift_feedback_s _feedback{};

	orb_advert_t _mavlink_log_pub{nullptr};
	perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};

	State _state{State::Disabled};
	Abort _abort_reason{Abort::None};
	hrt_abstime _last_run{0};
	hrt_abstime _state_since{0};
	hrt_abstime _last_disarm_request{0};
	hrt_abstime _last_debug{0};

	// attitude in the structural frame
	matrix::Dcmf _R{};		// structural body -> NED
	float _pitch{0.f};		// [deg]
	float _roll{0.f};		// [deg]
	float _q{0.f};			// [deg/s] pitch rate
	float _q_f{0.f};		// [deg/s] pitch rate low-passed at NL_Q_FC: what the loops damp on
	hrt_abstime _q_f_t{0};
	float _cmd_avg{0.f};
	float _hold_cmd_avg{0.f};	// nose command averaged ~1 s while holding (the takeoff gate)		// nose command averaged over 0.5 s (the stuck test)
	float _q_settled{0.f};		// [deg/s] pitch rate low-passed at 1 Hz: only for the "has it settled" checks
	hrt_abstime _q_settled_t{0};	// sample time of the last rate it took
	float _p_rad{0.f};		// [rad/s] roll rate
	float _r_rad{0.f};		// [rad/s] yaw rate
	bool _att_ok{false};

	// nose-lift switch
	bool _rc_ok{false};
	bool _sb_known{false};
	bool _sb_on{false};
	bool _sb_rose{false};
	bool _sb_fell{false};

	// lifting motors (NL_MOT_MSK bit order) and their split
	int _lift_count{0};
	int _lift_motor[MAX_LIFT] {};
	float _split[MAX_LIFT] {1.f, 1.f, 1.f, 1.f};

	// sequence
	hrt_abstime _t0{0};
	hrt_abstime _hold_since{0};
	hrt_abstime _fade_start{0};
	float _pitch0{0.f};		// [deg] where the lift started (the lowering goes back there)
	float _target{0.f};		// [deg] current target
	float _integral{0.f};
	bool _hold_refused_told{false};
	bool _hold_refused{false};	// PX4 refused the Position hold once this flight: do not ask again
	hrt_abstime _auto_hover_t0{0};
	bool _pclimb{false};
	bool _pdesc{false};		// descending in PX4 Position with a climb-rate stick (NL_AUTO_PDSC)
	hrt_abstime _pdesc_t0{0};
	bool _ho_off_legs{false};		// handover: the aircraft is off its legs (NL_HO_LIFT), for nose_lift_output.on_legs
	bool _rear_ramp{false};		// lowering after a flight: the rear fans fade out from _rear_from
	hrt_abstime _rear_t0{0};
	float _rear_from[NUM_MOTORS] {};		// climbing in PX4 Position with a climb-rate stick (NL_AUTO_PCLB)
	float _pclimb_v{0.f};		// [m/s] the climb rate the stick asks for
	float _auto_z0{NAN};		// [m] estimator z at the start of the automatic flight
	hrt_abstime _td_still_since{0};	// touchdown cue: since when near the ground and not descending
	float _td_vz{0.f};		// [m/s] up positive, the last climb rate the touchdown cue saw	// start of the hover (NL_AUTO_HOV counts from here)
	hrt_abstime _sat_since{0};	// ramp: since when the command sits at its limit (0 = it does not)
	float _sat_pitch{0.f};		// [deg] pitch when it got there (stuck = no motion since)
	bool _ceiling{false};		// above NL_TGT + NL_CEIL: being brought back (lowering gains)
	float _cmd{0.f};
	float _fade_from{0.f};
	float _z0{NAN};
	uint8_t _z_reset_counter{0};
	float _baro_f{NAN};		// [m] low-passed barometric altitude
	float _baro0{NAN};		// [m] ... at the start of the lift
	hrt_abstime _baro_t{0};		// sample time of the last baro update
	bool _fading{false};
	const char *_fade_reason{""};
	bool _low_thr_told{false};	// NoseHold: "lower the throttle first" said for this switch-off
	float _rear_f{0.f};		// NoseHold: rear fans' mean thrust, low-passed
	float _baro_vz{0.f};		// [m/s] rate of the low-passed baro altitude, low-passed again (0.5 s)

	// automatic takeoff, hover and landing (NL_AUTO)
	AutoPhase _auto{AutoPhase::Off};
	hrt_abstime _auto_since{0};	// start of the current phase
	float _auto_h0{NAN};		// [m] baro altitude at the takeoff
	float _auto_h_t{0.f};		// [m] height target above the takeoff
	float _auto_vz_t{0.f};		// [m/s] climb-rate target
	float _auto_int{0.f};		// [m s] height error integral
	float _auto_thr{NAN};		// [-] throttle asked for (NaN = none)
	float _auto_cut_from{0.f};
	hrt_abstime _auto_still_since{0};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::NL_EN>) _param_nl_en,
		(ParamInt<px4::params::NL_MOT_MSK>) _param_nl_mot_msk,
		(ParamFloat<px4::params::NL_HOV_PITCH>) _param_nl_hov_pitch,
		(ParamFloat<px4::params::NL_TGT>) _param_nl_tgt,
		(ParamFloat<px4::params::NL_RATE>) _param_nl_rate,
		(ParamFloat<px4::params::NL_K_ANG>) _param_nl_k_ang,
		(ParamFloat<px4::params::NL_KQ>) _param_nl_kq,
		(ParamFloat<px4::params::NL_KQI>) _param_nl_kqi,
		(ParamFloat<px4::params::NL_LOW_KQ>) _param_nl_low_kq,
		(ParamFloat<px4::params::NL_LOW_KQI>) _param_nl_low_kqi,
		(ParamFloat<px4::params::NL_K_RATE>) _param_nl_k_rate,
		(ParamFloat<px4::params::NL_MAX_CMD>) _param_nl_max_cmd,
		(ParamFloat<px4::params::NL_TOL>) _param_nl_tol,
		(ParamFloat<px4::params::NL_HOLD_S>) _param_nl_hold_s,
		(ParamFloat<px4::params::NL_FADE_S>) _param_nl_fade_s,
		(ParamFloat<px4::params::NL_HO_TOUT>) _param_nl_ho_tout,
		(ParamFloat<px4::params::NL_TOUT>) _param_nl_tout,
		(ParamFloat<px4::params::NL_HOLD_TOUT>) _param_nl_hold_tout,
		(ParamFloat<px4::params::NL_HO_THR>) _param_nl_ho_thr,
		(ParamInt<px4::params::NL_FLY_HOLD>) _param_nl_fly_hold,
		(ParamFloat<px4::params::NL_F_K_ANG>) _param_nl_f_k_ang,
		(ParamFloat<px4::params::NL_F_RATE>) _param_nl_f_rate,
		(ParamFloat<px4::params::NL_F_KQ>) _param_nl_f_kq,
		(ParamFloat<px4::params::NL_F_KQI>) _param_nl_f_kqi,
		(ParamFloat<px4::params::NL_F_FF>) _param_nl_f_ff,
		(ParamFloat<px4::params::NL_F_TGT>) _param_nl_f_tgt,
		(ParamFloat<px4::params::NL_F_TGT_THR>) _param_nl_f_tgt_thr,
		(ParamFloat<px4::params::NL_F_TGT_RATE>) _param_nl_f_tgt_rate,
		(ParamFloat<px4::params::NL_START_THR>) _param_nl_start_thr,
		(ParamFloat<px4::params::NL_ROLL_MAX>) _param_nl_roll_max,
		(ParamFloat<px4::params::NL_OVERSHOOT>) _param_nl_overshoot,
		(ParamFloat<px4::params::NL_I_MAX>) _param_nl_i_max,
		(ParamFloat<px4::params::NL_Q_FC>) _param_nl_q_fc,
		(ParamFloat<px4::params::NL_STUCK_S>) _param_nl_stuck_s,
		(ParamFloat<px4::params::NL_CEIL>) _param_nl_ceil,
		(ParamFloat<px4::params::NL_LIFT_DZ>) _param_nl_lift_dz,
		(ParamInt<px4::params::NL_RC_CH>) _param_nl_rc_ch,
		(ParamInt<px4::params::NL_RC_TH>) _param_nl_rc_th,
		(ParamInt<px4::params::NL_RC_LOW>) _param_nl_rc_low,
		(ParamFloat<px4::params::NL_EXPO>) _param_nl_expo,
		(ParamFloat<px4::params::NL_WEIGHT>) _param_nl_weight,
		(ParamFloat<px4::params::NL_PIV_X>) _param_nl_piv_x,
		(ParamFloat<px4::params::NL_PIV_Y>) _param_nl_piv_y,
		(ParamFloat<px4::params::NL_PIV_Z>) _param_nl_piv_z,
		(ParamFloat<px4::params::NL_A0>) _param_nl_a0,
		(ParamFloat<px4::params::NL_A1>) _param_nl_a1,
		(ParamFloat<px4::params::NL_A2>) _param_nl_a2,
		(ParamFloat<px4::params::NL_A3>) _param_nl_a3,
		(ParamFloat<px4::params::NL_W0>) _param_nl_w0,
		(ParamFloat<px4::params::NL_W1>) _param_nl_w1,
		(ParamFloat<px4::params::NL_W2>) _param_nl_w2,
		(ParamFloat<px4::params::NL_W3>) _param_nl_w3,
		(ParamFloat<px4::params::NL_MR0>) _param_nl_mr0,
		(ParamFloat<px4::params::NL_MR1>) _param_nl_mr1,
		(ParamFloat<px4::params::NL_MR2>) _param_nl_mr2,
		(ParamFloat<px4::params::NL_MR3>) _param_nl_mr3,
		(ParamFloat<px4::params::NL_MY0>) _param_nl_my0,
		(ParamFloat<px4::params::NL_MY1>) _param_nl_my1,
		(ParamFloat<px4::params::NL_MY2>) _param_nl_my2,
		(ParamFloat<px4::params::NL_MY3>) _param_nl_my3,
		(ParamInt<px4::params::NL_AUTO>) _param_nl_auto,
		(ParamFloat<px4::params::NL_AUTO_WAIT>) _param_nl_auto_wait,
		(ParamFloat<px4::params::NL_AUTO_ALT>) _param_nl_auto_alt,
		(ParamFloat<px4::params::NL_AUTO_HOV>) _param_nl_auto_hov,
		(ParamInt<px4::params::NL_AUTO_HOLD>) _param_nl_auto_hold,
		(ParamFloat<px4::params::NL_AUTO_HCMD>) _param_nl_auto_hcmd,
		(ParamFloat<px4::params::NL_AUTO_HDROP>) _param_nl_auto_hdrop,
		(ParamFloat<px4::params::NL_AUTO_HFL>) _param_nl_auto_hfl,
		(ParamFloat<px4::params::NL_AUTO_VFL>) _param_nl_auto_vfl,
		(ParamInt<px4::params::NL_AUTO_TD>) _param_nl_auto_td,
		(ParamInt<px4::params::NL_AUTO_PCLB>) _param_nl_auto_pclb,
		(ParamInt<px4::params::NL_AUTO_PTKO>) _param_nl_auto_ptko,
		(ParamInt<px4::params::NL_HO_LIFT>) _param_nl_ho_lift,
		(ParamFloat<px4::params::NL_HO_LIFT_H>) _param_nl_ho_lift_h,
		(ParamFloat<px4::params::NL_HO_FF>) _param_nl_ho_ff,
		(ParamInt<px4::params::NL_AUTO_PDSC>) _param_nl_auto_pdsc,
		(ParamInt<px4::params::NL_AUTO_RNG>) _param_nl_auto_rng,
		(ParamFloat<px4::params::NL_AUTO_VTD>) _param_nl_auto_vtd,
		(ParamFloat<px4::params::NL_AUTO_HTD>) _param_nl_auto_htd,
		(ParamFloat<px4::params::NL_LOW_RRAMP>) _param_nl_low_rramp,
		(ParamFloat<px4::params::NL_LOW_VMIN>) _param_nl_low_vmin,
		(ParamInt<px4::params::NL_LEGS_ATT>) _param_nl_legs_att,
		(ParamFloat<px4::params::NL_LEGS_FADE>) _param_nl_legs_fade,
		(ParamFloat<px4::params::NL_HO_KANG>) _param_nl_ho_kang,
		(ParamFloat<px4::params::NL_AUTO_PKZ>) _param_nl_auto_pkz,
		(ParamFloat<px4::params::NL_AUTO_PACC>) _param_nl_auto_pacc,
		(ParamFloat<px4::params::MAN_DEADZONE>) _param_man_deadzone,
		(ParamFloat<px4::params::MPC_Z_VEL_MAX_UP>) _param_mpc_z_vel_max_up,
		(ParamFloat<px4::params::MPC_Z_VEL_MAX_DN>) _param_mpc_z_vel_max_dn,
		(ParamFloat<px4::params::NL_AUTO_VUP>) _param_nl_auto_vup,
		(ParamFloat<px4::params::NL_AUTO_VDN>) _param_nl_auto_vdn,
		(ParamFloat<px4::params::NL_AUTO_THR>) _param_nl_auto_thr,
		(ParamFloat<px4::params::NL_AUTO_MAX>) _param_nl_auto_max,
		(ParamFloat<px4::params::NL_AUTO_RAMP>) _param_nl_auto_ramp,
		(ParamFloat<px4::params::NL_AUTO_KP>) _param_nl_auto_kp,
		(ParamFloat<px4::params::NL_AUTO_KV>) _param_nl_auto_kv,
		(ParamFloat<px4::params::NL_AUTO_KI>) _param_nl_auto_ki,
		(ParamInt<px4::params::NL_AUTO_PILOT>) _param_nl_auto_pilot,
		(ParamFloat<px4::params::MPC_THR_HOVER>) _param_mpc_thr_hover,
		(ParamInt<px4::params::MPC_THR_CURVE>) _param_mpc_thr_curve,
		(ParamFloat<px4::params::NL_AUTO_SLEW>) _param_nl_auto_slew,
		(ParamInt<px4::params::RC_MAP_THROTTLE>) _param_rc_map_throttle,
		(ParamInt<px4::params::CA_ROTOR_COUNT>) _param_ca_rotor_count,
		(ParamFloat<px4::params::THR_MDL_FAC>) _param_thr_mdl_fac
	)
};
