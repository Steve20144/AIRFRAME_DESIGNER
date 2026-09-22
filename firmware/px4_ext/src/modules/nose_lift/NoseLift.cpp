/**
 * @file NoseLift.cpp
 *
 * Port of AIRFRAME_DESIGNER's airframe_designer/sim/nose_lift.py (NoseLift) to the flight controller. The loop
 * and its numbers are the same; what differs is the order (PX4 is armed first, the switch starts the lift) and
 * the ways out (see NoseLift.hpp).
 */

#include "NoseLift.hpp"

using matrix::Dcmf;
using matrix::Eulerf;
using matrix::Quatf;
using matrix::Vector3f;

NoseLift::NoseLift() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	update_lift_motors();
}

NoseLift::~NoseLift()
{
	perf_free(_loop_perf);
}

bool NoseLift::init()
{
	ScheduleOnInterval(5_ms);	// 200 Hz
	return true;
}

// ------------------------------------------------------------------------------------------------- inputs

void NoseLift::update_attitude(hrt_abstime now)
{
	_att_ok = _attitude.timestamp > 0 && _angvel.timestamp > 0
		  && now - _attitude.timestamp < 100_ms && now - _angvel.timestamp < 100_ms;

	// PX4's body frame is the structural frame pitched nose-up by the hover pitch: R_px4 = R_s * Rh^T
	const float h = math::radians(_param_nl_hov_pitch.get());
	const float c = cosf(h), s = sinf(h);
	Dcmf Rh;
	Rh(0, 0) = c;   Rh(0, 1) = 0.f; Rh(0, 2) = s;
	Rh(1, 0) = 0.f; Rh(1, 1) = 1.f; Rh(1, 2) = 0.f;
	Rh(2, 0) = -s;  Rh(2, 1) = 0.f; Rh(2, 2) = c;

	_R = Dcmf(Quatf(_attitude.q)) * Rh;
	const Eulerf e(_R);
	_roll = math::degrees(e.phi());
	_pitch = math::degrees(e.theta());

	const Vector3f w_s = Rh.transpose() * Vector3f(_angvel.xyz);
	_p_rad = w_s(0);
	_q = math::degrees(w_s(1));
	_r_rad = w_s(2);
}

void NoseLift::update_switch(hrt_abstime now)
{
	const int ch = _param_nl_rc_ch.get();
	_rc_ok = ch >= 1 && ch <= input_rc_s::RC_INPUT_MAX_CHANNELS && ch <= _rc.channel_count
		 && _rc.timestamp_last_signal > 0 && now - _rc.timestamp_last_signal < 500_ms
		 && !_rc.rc_lost && !_rc.rc_failsafe;
	_sb_rose = _sb_fell = false;

	if (!_rc_ok) {
		return;
	}

	const int v = _rc.values[ch - 1];
	const bool on = _param_nl_rc_low.get() ? v < _param_nl_rc_th.get() : v > _param_nl_rc_th.get();

	if (_sb_known) {
		_sb_rose = on && !_sb_on;
		_sb_fell = !on && _sb_on;
	}

	_sb_on = on;
	_sb_known = true;
}

void NoseLift::update_lift_motors()
{
	const int mask = _param_nl_mot_msk.get();
	_lift_count = 0;

	for (int i = 0; i < NUM_MOTORS && _lift_count < MAX_LIFT; i++) {
		if (mask & (1 << i)) {
			_lift_motor[_lift_count++] = i;
		}
	}
}

float NoseLift::throttle() const
{
	// manual_control_setpoint.throttle is -1..1; 0..1 here, and "unknown" reads as full so nothing starts on it
	return _manual.valid ? 0.5f * (_manual.throttle + 1.f) : 1.f;
}

bool NoseLift::lifted_off()
{
	if (!PX4_ISFINITE(_z0) || !_lpos.z_valid || !_lpos.v_z_valid) {
		return false;
	}

	// the estimator shifted its height (a reset): shift the reference with it rather than read it as a climb
	if (_lpos.z_reset_counter != _z_reset_counter) {
		_z0 += _lpos.delta_z;
		_z_reset_counter = _lpos.z_reset_counter;
	}

	// a real premature liftoff climbs; the height estimate on the legs can drift by tenths of a metre within seconds
	// (after a kill drops the nose, for one), and a false alarm here cuts the motors and drops the nose from the hold
	const bool high = (_z0 - _lpos.z) > _param_nl_lift_dz.get();
	const bool climbing = -_lpos.vz > 0.3f;
	return high && climbing;
}

// ------------------------------------------------------------------------------------------------- the loop

void NoseLift::update_split()
{
	// thrust weights that cancel the net yaw (and roll) moment of the lifting motors, with roll/yaw rate damping,
	// normalised to a mean of 1 so the pitch feed-forward holds (nose_lift.py NoseLift._update_split)
	if (_lift_count < 2) {
		for (int k = 0; k < MAX_LIFT; k++) { _split[k] = 1.f; }

		return;
	}

	const float w0[MAX_LIFT] {_param_nl_w0.get(), _param_nl_w1.get(), _param_nl_w2.get(), _param_nl_w3.get()};
	const float mr[MAX_LIFT] {_param_nl_mr0.get(), _param_nl_mr1.get(), _param_nl_mr2.get(), _param_nl_mr3.get()};
	const float my[MAX_LIFT] {_param_nl_my0.get(), _param_nl_my1.get(), _param_nl_my2.get(), _param_nl_my3.get()};

	float mmax = 1e-9f;

	for (int k = 0; k < _lift_count; k++) {
		mmax = math::max(mmax, math::max(fabsf(mr[k]), fabsf(my[k])));
	}

	float w[MAX_LIFT] {};
	float sum = 0.f;

	for (int k = 0; k < _lift_count; k++) {
		w[k] = w0[k] - _param_nl_k_rate.get() * (mr[k] * _p_rad + my[k] * _r_rad) / mmax;
		w[k] = math::constrain(w[k], 0.2f, 1.8f);
		sum += w[k];
	}

	const float mean = sum / _lift_count;

	for (int k = 0; k < _lift_count; k++) {
		_split[k] = w[k] / mean;
	}
}

float NoseLift::balance_fraction() const
{
	// thrust fraction on the lifting motors that balances the weight about the rear feet at this attitude
	// (nose_lift.py NoseLift._balance_fraction); the lifting moment is body-fixed, gravity's is not
	const Vector3f piv = _R * Vector3f(_param_nl_piv_x.get(), _param_nl_piv_y.get(), _param_nl_piv_z.get());
	const Vector3f axis = _R.col(1);
	const float tau_g = (-piv).cross(Vector3f(0.f, 0.f, _param_nl_weight.get())).dot(axis);

	const float A[MAX_LIFT] {_param_nl_a0.get(), _param_nl_a1.get(), _param_nl_a2.get(), _param_nl_a3.get()};
	float a = 0.f;

	for (int k = 0; k < _lift_count; k++) {
		a += A[k] * _split[k];
	}

	if (fabsf(a) < 1e-9f) {
		return 0.5f;
	}

	return math::constrain(-tau_g / a, 0.f, 2.f);
}

float NoseLift::thrust_to_cmd(float frac) const
{
	const float f = math::constrain(math::max(frac, 0.f), 0.f, _param_nl_max_cmd.get());
	return powf(f, 1.f / math::max(_param_nl_expo.get(), 1e-3f));
}

// actuator_motors carries normalised thrust: the output driver turns it into a motor command through THR_MDL_FAC
// (command = the root of f c^2 + (1 - f) c = thrust). The loop works in motor commands, like the simulator's, so
// it publishes the thrust whose command is the one it wants, and reads PX4's thrust back as a command.
float NoseLift::motor_thrust(float command) const
{
	const float f = math::constrain(_param_thr_mdl_fac.get(), 0.f, 1.f);
	return f * command * command + (1.f - f) * command;
}

float NoseLift::motor_command(float thrust) const
{
	const float f = math::constrain(_param_thr_mdl_fac.get(), 0.f, 1.f);

	if (f < 1e-3f) {
		return thrust;
	}

	return (-(1.f - f) + sqrtf((1.f - f) * (1.f - f) + 4.f * f * math::max(thrust, 0.f))) / (2.f * f);
}

// ------------------------------------------------------------------------------------------------- states

void NoseLift::try_start(hrt_abstime now)
{
	const char *why = nullptr;

	if (!_att_ok) {
		why = "no attitude estimate";

	} else if (_lift_count == 0) {
		why = "no lifting motors (NL_MOT_MSK)";

	} else if (!_land.landed) {
		why = "not landed";

	} else if (throttle() > _param_nl_start_thr.get()) {
		why = "throttle not at minimum";

	} else if (fabsf(_roll) > _param_nl_roll_max.get()) {
		why = "rolled on the ground";
	}

	if (why) {
		mavlink_log_critical(&_mavlink_log_pub, "Nose lift refused: %s\t", why);
		return;
	}

	_t0 = now;
	_pitch0 = _pitch;
	_target = _param_nl_tgt.get();
	_integral = 0.f;
	_cmd = 0.f;
	_hold_since = 0;
	_fading = false;
	_z0 = _lpos.z_valid ? _lpos.z : NAN;
	_z_reset_counter = _lpos.z_reset_counter;
	_abort_reason = Abort::None;
	set_state(State::Ramping, now);
	mavlink_log_info(&_mavlink_log_pub, "Nose lift: raising the nose from %.1f to %.1f deg\t", (double)_pitch0,
			 (double)_target);
}

void NoseLift::step_sequence(hrt_abstime now, float dt, bool kill)
{
	if (kill) {
		abort(Abort::Kill, false, now);
		return;
	}

	if (!_att_ok) {
		abort(Abort::AttitudeLost, false, now);
		return;
	}

	const bool lifting = _state == State::Ramping || _state == State::Holding;

	if (lifting || _state == State::Lowering) {
		if (fabsf(_roll) > _param_nl_roll_max.get()) { abort(Abort::Roll, false, now); return; }

		if (lifted_off()) { abort(Abort::Liftoff, false, now); return; }
	}

	if (lifting) {
		if (_pitch > _target + _param_nl_overshoot.get()) { abort(Abort::Overshoot, false, now); return; }

		if (!_rc_ok) { abort(Abort::RcLost, true, now); return; }

		if (_sb_fell) { abort(Abort::SwitchOff, true, now); return; }
	}

	update_split();
	const float ff = balance_fraction();
	const float el = (now - _t0) * 1e-6f;
	const float rate = _param_nl_rate.get();
	const float k_ang = _param_nl_k_ang.get();
	const float tol = _param_nl_tol.get();

	switch (_state) {
	case State::Ramping: {
			if (el > _param_nl_tout.get()) { abort(Abort::LiftTimeout, true, now); return; }

			if (ff > _param_nl_max_cmd.get() + 0.02f && el > 2.f && _pitch - _pitch0 < 1.f) {
				abort(Abort::CannotHold, false, now);
				return;
			}

			// the nose is asked to rotate at NL_RATE (easing in over 1.5 s, slowing proportionally over the last
			// degrees); the thrust regulates the pitch rate around that on top of the balance feed-forward
			const float ease = math::min(1.f, el / 1.5f);
			const float q_des = math::constrain(k_ang * (_target - _pitch), -rate, rate * ease);
			const float eq = q_des - _q;
			_integral = math::constrain(_integral + eq * dt, -40.f, 40.f);
			_cmd = thrust_to_cmd(ease * ff + _param_nl_kq.get() * eq + _param_nl_kqi.get() * _integral);

			if (fabsf(_pitch - _target) < tol && fabsf(_q) < 3.f) {
				if (_hold_since == 0) { _hold_since = now; }

				if ((now - _hold_since) * 1e-6f >= _param_nl_hold_s.get()) {
					set_state(State::Holding, now);
					mavlink_log_info(&_mavlink_log_pub, "Nose lift: holding at %.1f deg, raise the throttle to take off\t",
							 (double)_pitch);
				}

			} else {
				_hold_since = 0;
			}

			break;
		}

	case State::Holding: {
			if ((now - _state_since) * 1e-6f > _param_nl_hold_tout.get()) { abort(Abort::HoldTimeout, true, now); return; }

			const float q_des = math::constrain(k_ang * (_target - _pitch), -rate, rate);
			const float eq = q_des - _q;
			_integral = math::constrain(_integral + eq * dt, -40.f, 40.f);
			_cmd = thrust_to_cmd(ff + _param_nl_kq.get() * eq + _param_nl_kqi.get() * _integral);

			if (throttle() > _param_nl_ho_thr.get()) {
				set_state(State::Handover, now);
				_fading = false;
				mavlink_log_info(&_mavlink_log_pub, "Nose lift: handing over to PX4\t");
			}

			break;
		}

	case State::Handover: {
			// keep holding the nose until PX4 itself drives the lifting motors at least as hard as the hold (leaving
			// the ground is not enough: the spool-up would otherwise drop the nose), or the timeout; then fade out
			const float q_des = math::constrain(k_ang * (_target - _pitch), -rate, rate);
			const float eq = q_des - _q;
			_integral = math::constrain(_integral + eq * dt, -40.f, 40.f);
			const float hold = thrust_to_cmd(ff + _param_nl_kq.get() * eq + _param_nl_kqi.get() * _integral);

			float px4_share = 0.f;

			if (now - _feedback.timestamp < 100_ms) {
				px4_share = 1e9f;

				for (int k = 0; k < _lift_count; k++) {
					const float c = _feedback.allocator_control[_lift_motor[k]];
					px4_share = math::min(px4_share, PX4_ISFINITE(c) ? motor_command(c) : 0.f);
				}
			}

			const bool taken_over = px4_share >= 0.95f * hold;

			if (!_fading && (taken_over || (now - _state_since) * 1e-6f > _param_nl_ho_tout.get() || !_rc_ok)) {
				_fading = true;
				_fade_start = now;
				_fade_from = hold;
				_fade_reason = taken_over ? "PX4 took over" : (!_rc_ok ? "radio lost" : "handover timed out");
			}

			if (!_fading) {
				_cmd = hold;

			} else {
				const float k = (now - _fade_start) * 1e-6f / math::max(_param_nl_fade_s.get(), 1e-3f);
				_cmd = _fade_from * math::max(0.f, 1.f - k);

				if (k >= 1.f) {
					_cmd = 0.f;
					set_state(State::Flying, now);
					mavlink_log_info(&_mavlink_log_pub, "Nose lift: done, %s\t", _fade_reason);
				}
			}

			break;
		}

	case State::Lowering: {
			// a cancel: bring the nose back to where the lift started, then stop the motors and disarm
			if ((now - _state_since) * 1e-6f > _param_nl_tout.get()) { abort(Abort::LowerTimeout, false, now); return; }

			if (!_fading) {
				// ease the descent in over 1 s: a full-rate demand at once, through the stiffer lowering gain,
				// dips the thrust and drops the nose for a moment (nose_lift.py NoseLower eases the same way)
				const float ease = math::min(1.f, (now - _state_since) * 1e-6f / 1.f);
				const float q_des = math::constrain(k_ang * (_target - _pitch), -rate * ease, rate);
				const float eq = q_des - _q;
				_integral = math::constrain(_integral + eq * dt, -40.f, 40.f);
				_cmd = thrust_to_cmd(ff + _param_nl_low_kq.get() * eq + _param_nl_low_kqi.get() * _integral);

				// fade only once the nose sits on its front leg again: fading from further up drops the last degrees
				// (one-sided: legs that compress a little more than before may leave it slightly below where it started)
				if (_pitch - _target < math::min(tol, 0.5f) && fabsf(_q) < 1.f) {
					if (_hold_since == 0) { _hold_since = now; }

					if ((now - _hold_since) * 1e-6f >= _param_nl_hold_s.get()) {
						_fading = true;
						_fade_start = now;
						_fade_from = _cmd;
					}

				} else {
					_hold_since = 0;
				}

			} else {
				const float k = (now - _fade_start) * 1e-6f / math::max(_param_nl_fade_s.get(), 1e-3f);
				_cmd = _fade_from * math::max(0.f, 1.f - k);

				if (k >= 1.f) {
					_cmd = 0.f;
					set_state(State::Aborted, now);
					mavlink_log_info(&_mavlink_log_pub, "Nose lift: nose lowered to %.1f deg, disarming\t", (double)_pitch);
				}
			}

			break;
		}

	default:
		break;
	}
}

void NoseLift::abort(Abort reason, bool controlled, hrt_abstime now)
{
	_abort_reason = reason;

	if (controlled && (_state == State::Ramping || _state == State::Holding)) {
		// same integrator contribution under the lowering gains, so the thrust does not jump
		_integral *= _param_nl_kqi.get() / math::max(_param_nl_low_kqi.get(), 1e-6f);
		_target = _pitch0;
		_fading = false;
		_hold_since = 0;
		set_state(State::Lowering, now);
		mavlink_log_critical(&_mavlink_log_pub, "Nose lift cancelled (%s): lowering the nose to %.1f deg\t",
				     abort_name(reason), (double)_target);

	} else {
		_cmd = 0.f;
		set_state(State::Aborted, now);
		mavlink_log_critical(&_mavlink_log_pub, "Nose lift aborted (%s): motors stopped\t", abort_name(reason));
	}
}

void NoseLift::set_state(State s, hrt_abstime now)
{
	_state = s;
	_state_since = now;
}

void NoseLift::request_disarm(hrt_abstime now)
{
	if (!_armed.armed || now - _last_disarm_request < 1_s) {
		return;
	}

	_last_disarm_request = now;
	vehicle_command_s cmd{};
	cmd.command = vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM;
	cmd.param1 = 0.f;
	cmd.param2 = 21196.f;	// force: PX4 may not consider an aircraft on two feet "landed"
	cmd.target_system = _status.system_id;
	cmd.target_component = _status.component_id;
	cmd.source_system = _status.system_id;
	cmd.source_component = _status.component_id;
	cmd.from_external = false;
	cmd.timestamp = hrt_absolute_time();
	_command_pub.publish(cmd);
}

// ------------------------------------------------------------------------------------------------- output

void NoseLift::publish(hrt_abstime now)
{
	nose_lift_output_s out{};
	out.timestamp = now;
	out.state = static_cast<uint8_t>(_state);
	out.abort_reason = static_cast<uint8_t>(_abort_reason);
	out.pitch_deg = _pitch;
	out.target_deg = _target;
	out.cmd = _cmd;

	for (int i = 0; i < NUM_MOTORS; i++) {
		out.control[i] = NAN;
	}

	const int n = math::constrain(static_cast<int>(_param_ca_rotor_count.get()), 1, NUM_MOTORS);
	const uint16_t all = static_cast<uint16_t>((1u << n) - 1u);
	uint16_t lift_mask = 0;

	for (int k = 0; k < _lift_count; k++) {
		lift_mask |= static_cast<uint16_t>(1u << _lift_motor[k]);
	}

	switch (_state) {
	case State::Disarmed:	// held already while disarmed, so the override is in place at the instant PX4 arms
	case State::Parked:	// (armed nose-down in airmode, PX4 would otherwise send every motor towards full)
	case State::Aborted:
		out.active = true;
		out.override_mask = all;
		break;

	case State::Ramping:
	case State::Holding:
	case State::Lowering:
	case State::Handover:
		out.active = true;

		if (_state == State::Handover) {
			out.floor_mask = lift_mask;

		} else {
			out.override_mask = all;
		}

		for (int k = 0; k < _lift_count; k++) {
			out.control[_lift_motor[k]] = motor_thrust(math::constrain(_cmd * _split[k], 0.f, 1.f));
		}

		break;

	default:
		out.active = false;
		break;
	}

	_output_pub.publish(out);

	if (now - _last_debug > 100_ms) {
		_last_debug = now;
		debug_vect_s dbg{};
		dbg.timestamp = now;
		strncpy(dbg.name, "NLIFT", sizeof(dbg.name));
		dbg.x = static_cast<float>(_state) + 0.01f * static_cast<float>(_abort_reason);
		dbg.y = _pitch;
		dbg.z = _cmd;
		_debug_pub.publish(dbg);
	}
}

// ------------------------------------------------------------------------------------------------- Run

void NoseLift::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	if (_parameter_update_sub.updated()) {
		parameter_update_s pu;
		_parameter_update_sub.copy(&pu);
		updateParams();
		update_lift_motors();
	}

	const hrt_abstime now = hrt_absolute_time();
	const float dt = _last_run > 0 ? math::constrain((now - _last_run) * 1e-6f, 1e-4f, 0.05f) : 0.005f;
	_last_run = now;

	_attitude_sub.update(&_attitude);
	_angular_velocity_sub.update(&_angvel);
	_armed_sub.update(&_armed);
	_land_sub.update(&_land);
	_manual_sub.update(&_manual);
	_rc_sub.update(&_rc);
	_lpos_sub.update(&_lpos);
	_status_sub.update(&_status);
	_feedback_sub.update(&_feedback);

	if (_param_nl_en.get() == 0) {
		if (_state != State::Disabled) {
			set_state(State::Disabled, now);
			publish(now);	// one inactive message so the allocator lets go
		}

		perf_end(_loop_perf);
		return;
	}

	if (_state == State::Disabled) {
		set_state(State::Disarmed, now);
	}

	update_attitude(now);
	update_switch(now);

	const bool armed = _armed.armed;
	const bool kill = _armed.kill || _armed.termination;

	if (!armed && _state != State::Disarmed) {
		if (is_sequence(_state)) {
			mavlink_log_critical(&_mavlink_log_pub, "Nose lift: disarmed while %s\t", state_name(_state));
		}

		_cmd = 0.f;
		set_state(State::Disarmed, now);
	}

	switch (_state) {
	case State::Disarmed:
		if (armed) {
			if (_land.landed) {
				set_state(State::Parked, now);
				_abort_reason = Abort::None;
				mavlink_log_info(&_mavlink_log_pub, "Nose lift: armed on the ground, motors stopped until the switch\t");

			} else {
				set_state(State::Flying, now);
			}

		} else if (_sb_rose) {
			mavlink_log_info(&_mavlink_log_pub, "Nose lift: arm first, then the switch\t");
		}

		break;

	case State::Parked:
		if (kill) {
			abort(Abort::Kill, false, now);

		} else if (_sb_rose) {
			try_start(now);
		}

		break;

	case State::Ramping:
	case State::Holding:
	case State::Handover:
	case State::Lowering:
		step_sequence(now, dt, kill);
		break;

	case State::Aborted:
		request_disarm(now);
		break;

	default:
		break;
	}

	publish(now);
	perf_end(_loop_perf);
}

// ------------------------------------------------------------------------------------------------- module

const char *NoseLift::state_name(State s)
{
	switch (s) {
	case State::Disabled: return "disabled";

	case State::Disarmed: return "disarmed";

	case State::Parked: return "parked";

	case State::Ramping: return "raising the nose";

	case State::Holding: return "holding";

	case State::Handover: return "handing over";

	case State::Flying: return "flying";

	case State::Lowering: return "lowering the nose";

	case State::Aborted: return "aborted";
	}

	return "?";
}

const char *NoseLift::abort_name(Abort a)
{
	switch (a) {
	case Abort::None: return "none";

	case Abort::Kill: return "kill switch";

	case Abort::SwitchOff: return "switch off";

	case Abort::RcLost: return "radio lost";

	case Abort::AttitudeLost: return "attitude lost";

	case Abort::Roll: return "roll limit";

	case Abort::Overshoot: return "overshoot";

	case Abort::Liftoff: return "left the ground";

	case Abort::LiftTimeout: return "lift timeout";

	case Abort::CannotHold: return "motors cannot hold the nose";

	case Abort::HoldTimeout: return "hold timeout";

	case Abort::LowerTimeout: return "lowering timeout";
	}

	return "?";
}

int NoseLift::print_status()
{
	PX4_INFO("state: %s, abort: %s", state_name(_state), abort_name(_abort_reason));
	PX4_INFO("pitch %.2f deg (target %.2f), rate %.2f deg/s, roll %.2f deg, cmd %.3f", (double)_pitch,
		 (double)_target, (double)_q, (double)_roll, (double)_cmd);
	PX4_INFO("switch: %s%s, lifting motors: %d", _rc_ok ? "" : "no RC, ", _sb_on ? "on" : "off", _lift_count);
	perf_print_counter(_loop_perf);
	return 0;
}

int NoseLift::task_spawn(int argc, char *argv[])
{
	NoseLift *instance = new NoseLift();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;
	return PX4_ERROR;
}

int NoseLift::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int NoseLift::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Nose-lift ground sequence (AIRFRAME_DESIGNER): with NL_EN set, arming on the ground holds every motor stopped;
the switch on NL_RC_CH raises the nose on the NL_MOT_MSK motors to NL_TGT and holds it until the throttle hands
it to PX4. The kill switch cuts everything in every state.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("nose_lift", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int nose_lift_main(int argc, char *argv[])
{
	return NoseLift::main(argc, argv);
}
