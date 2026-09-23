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
#include <uORB/topics/debug_vect.h>
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
	float thrust_to_cmd(float frac) const;
	float motor_thrust(float command) const;
	float motor_command(float thrust) const;
	float throttle() const;
	bool lifted_off(hrt_abstime now);
	void update_baro();

	void try_start(hrt_abstime now);
	void step_sequence(hrt_abstime now, float dt, bool kill);
	void abort(Abort reason, bool controlled, hrt_abstime now);
	void set_state(State s, hrt_abstime now);
	void request_disarm(hrt_abstime now);
	void publish(hrt_abstime now);

	static const char *state_name(State s);
	static const char *abort_name(Abort a);
	static bool is_sequence(State s) { return s == State::Ramping || s == State::Holding || s == State::Handover || s == State::Lowering; }

	uORB::Publication<nose_lift_output_s> _output_pub{ORB_ID(nose_lift_output)};
	uORB::Publication<debug_vect_s> _debug_pub{ORB_ID(debug_vect)};
	uORB::Publication<vehicle_command_s> _command_pub{ORB_ID(vehicle_command)};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::Subscription _attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _armed_sub{ORB_ID(actuator_armed)};
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
	float _cmd{0.f};
	float _fade_from{0.f};
	float _z0{NAN};
	uint8_t _z_reset_counter{0};
	matrix::Vector3f _w_f{};	// [rad/s] low-passed body rates in the structural frame
	hrt_abstime _rate_t{0};		// sample time of the last rate used
	float _baro_f{NAN};		// [m] low-passed barometric altitude
	float _baro0{NAN};		// [m] ... at the start of the lift
	hrt_abstime _baro_t{0};		// sample time of the last baro update
	bool _fading{false};
	const char *_fade_reason{""};

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
		(ParamFloat<px4::params::NL_START_THR>) _param_nl_start_thr,
		(ParamFloat<px4::params::NL_ROLL_MAX>) _param_nl_roll_max,
		(ParamFloat<px4::params::NL_OVERSHOOT>) _param_nl_overshoot,
		(ParamFloat<px4::params::NL_LIFT_DZ>) _param_nl_lift_dz,
		(ParamFloat<px4::params::NL_Q_LPF>) _param_nl_q_lpf,
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
		(ParamInt<px4::params::CA_ROTOR_COUNT>) _param_ca_rotor_count,
		(ParamFloat<px4::params::THR_MDL_FAC>) _param_thr_mdl_fac
	)
};
