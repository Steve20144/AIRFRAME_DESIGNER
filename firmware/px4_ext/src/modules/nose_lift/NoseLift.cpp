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

	// the hold and fade decisions ask whether the nose has settled; on its legs the frame never stops rocking (+-20
	// deg/s at 5-10 Hz), so against the raw rate "under 3 deg/s" was never true and the lift could not reach
	// Holding (no handover to PX4) nor the lowering fade. A decision can wait: 1 Hz, and only for these checks.
	if (_q_settled_t == 0) {
		_q_settled = _q;

	} else if (_angvel.timestamp_sample != _q_settled_t) {
		const float dt = math::constrain((_angvel.timestamp_sample - _q_settled_t) * 1e-6f, 0.f, 0.05f);
		_q_settled += (_q - _q_settled) * (dt / (dt + 1.f / (2.f * M_PI_F * 1.f)));
	}

	// the loops' damping term: the same rocking (+-20 deg/s at 10 to 15 Hz) chopped the nose command between 0.5 and
	// 1.0 at that frequency (7 Oct 16:06, the nose stuck, the fans averaging 0.85), so the rate the loops damp on is
	// low-passed at NL_Q_FC (default 2 Hz: 10 Hz rocking cut 5 times, a 3 deg/s rotation seen with 80 ms of lag)
	if (_q_f_t == 0) {
		_q_f = _q;

	} else if (_angvel.timestamp_sample != _q_f_t) {
		const float dt = math::constrain((_angvel.timestamp_sample - _q_f_t) * 1e-6f, 0.f, 0.05f);
		const float fc = math::max(_param_nl_q_fc.get(), 0.1f);
		_q_f += (_q - _q_f) * (dt / (dt + 1.f / (2.f * M_PI_F * fc)));
	}

	_q_f_t = _angvel.timestamp_sample;
	_q_settled_t = _angvel.timestamp_sample;
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

void NoseLift::update_baro()
{
	// the raw barometric altitude, low-passed over 0.5 s against its sample noise (tenths of a metre)
	if (_air.timestamp_sample == 0 || _air.timestamp_sample == _baro_t || !PX4_ISFINITE(_air.baro_alt_meter)) {
		return;
	}

	const float dt = _baro_t > 0 ? math::constrain((_air.timestamp_sample - _baro_t) * 1e-6f, 0.f, 0.5f) : 0.f;
	_baro_t = _air.timestamp_sample;
	const float prev = _baro_f;
	_baro_f = PX4_ISFINITE(_baro_f) ? _baro_f + dt / (0.5f + dt) * (_air.baro_alt_meter - _baro_f) : _air.baro_alt_meter;

	// its rate, low-passed the same way: the automatic flight damps its height loop on it (the estimator's vz drifts
	// by tenths of a m/s on the ground under fan vibration, the baro does not)
	if (PX4_ISFINITE(prev) && dt > 1e-4f) {
		_baro_vz += dt / (0.3f + dt) * ((_baro_f - prev) / dt - _baro_vz);
	}
}

int NoseLift::pilot_throttle_us() const
{
	// the pilot's throttle stick, raw, straight from the receiver: manual_control_setpoint carries the override
	const int ch = _param_rc_map_throttle.get();

	if (ch < 1 || ch > _rc.channel_count || _rc.timestamp_last_signal == 0 || _rc.rc_lost) {
		return 0;
	}

	return _rc.values[ch - 1];
}

bool NoseLift::lifted_off(hrt_abstime now)
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

	// and the barometer must agree: on the aircraft (log 164) fan vibration (accel vibration metric 0.02 -> 3.8)
	// biased the accelerometer, and the estimate climbed a smooth 0.85 m at 0.3 m/s in 6 s with the nose not
	// moving and the raw baro within 0.2 m. Without a recent baro the estimate decides alone, as before.
	const bool baro_ok = PX4_ISFINITE(_baro0) && PX4_ISFINITE(_baro_f) && now - _air.timestamp < 500_ms;
	const bool baro_high = !baro_ok || (_baro_f - _baro0) > _param_nl_lift_dz.get();
	return high && climbing && baro_high;
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
	_sat_since = 0;
	_sat_pitch = _pitch;
	_cmd_avg = 0.f;
	_cmd = 0.f;
	_ceiling = false;
	_hold_since = 0;
	_fading = false;
	_z0 = _lpos.z_valid ? _lpos.z : NAN;
	_z_reset_counter = _lpos.z_reset_counter;
	_baro0 = _baro_f;
	_abort_reason = Abort::None;
	set_state(State::Ramping, now);
	mavlink_log_info(&_mavlink_log_pub, "Nose lift: raising the nose from %.1f to %.1f deg\t", (double)_pitch0,
			 (double)_target);
}

// NL_CEIL: above NL_TGT + NL_CEIL the raise and the hold bring the nose back the way the lowering does (down at
// NL_RATE on the lowering gains) until it is back at the target, then balance again. On the aircraft a gentle loop
// cannot take the lift-off surplus away fast enough and the nose ran through the target (23 Sep: 45-48 deg after a
// cancel); the stronger lowering gains catch it. The integrator's contribution is kept across the gain change.
bool NoseLift::over_ceiling(hrt_abstime now)
{
	const float ceil = _param_nl_ceil.get();

	if (ceil <= 0.f) {
		_ceiling = false;
		return false;
	}

	if (!_ceiling && _pitch > _target + ceil) {
		_ceiling = true;
		_integral *= _param_nl_kqi.get() / math::max(_param_nl_low_kqi.get(), 1e-6f);
		mavlink_log_info(&_mavlink_log_pub, "Nose lift: above %.1f deg, bringing it back to %.1f	", (double)(_target + ceil),
				 (double)_target);

	} else if (_ceiling && _pitch <= _target) {
		_ceiling = false;
		_integral *= _param_nl_low_kqi.get() / math::max(_param_nl_kqi.get(), 1e-6f);
	}

	return _ceiling;
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

		if (lifted_off(now)) { abort(Abort::Liftoff, false, now); return; }
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
			const bool down = over_ceiling(now);
			const float q_des = down ? -rate : math::constrain(k_ang * (_target - _pitch), -rate, rate * ease);
			const float eq = q_des - _q_f;

			// 7 Oct: the nose sat still for 4 to 8 s with the fans at full command (something held it), the
			// integral filled to its cap (40 deg = 0.48 of thrust) and, once the nose broke free, drove it
			// through the target at 12 deg/s instead of 3; the fans cannot pull the nose back down, so it ran
			// to 24-27 deg and the aircraft sat on its tail. Three guards:
			//  1. no integration while the command is at its limit and still asking for more (anti-windup);
			//  2. the integral may not exceed NL_I_MAX (default 10 deg = 0.12 of thrust);
			//  3. once the nose runs faster than the wanted rate by a full NL_RATE, whatever the integral
			//     stored is wrong: drop it to zero at once (the rate and feed-forward terms then brake).
			// And a stuck nose aborts (NL_STUCK_S): full command for that long with less than 1 deg of motion
			// starts the controlled lowering instead of waiting for the breakaway.
			const bool saturated = _cmd >= _param_nl_max_cmd.get() - 0.01f;
			const float i_max = math::max(_param_nl_i_max.get(), 0.f);
			// the stuck test looks at the command averaged over 0.5 s (the instantaneous one never sat at the limit
			// through the 7 Oct rocking, so the 10 s test never fired): stuck = the average above 0.9 of the limit
			_cmd_avg += (_cmd - _cmd_avg) * math::min(1.f, dt / 0.5f);
			const bool saturated_avg = _cmd_avg >= 0.9f * _param_nl_max_cmd.get();

			if (saturated_avg) {
				if (_sat_since == 0) {
					_sat_since = now;
					_sat_pitch = _pitch;

				} else if (_param_nl_stuck_s.get() > 0.f && (now - _sat_since) * 1e-6f > _param_nl_stuck_s.get()
					   && _pitch - _sat_pitch < 1.f) {
					abort(Abort::Stuck, true, now);
					return;
				}

			} else {
				_sat_since = 0;
			}

			// the overspeed test reads the 1 Hz rate: on the legs the raw rate rocks +-20 deg/s at 10 to 15 Hz and
			// tripped this test all the time, which kept emptying the integral (7 Oct evening: no authority left)
			if (_q_settled > q_des + rate) {
				_integral = math::min(_integral, 0.f);

			} else if (!(saturated && eq > 0.f)) {
				_integral = math::constrain(_integral + eq * dt, -i_max, i_max);
			}

			_cmd = down ? thrust_to_cmd(ff + _param_nl_low_kq.get() * eq + _param_nl_low_kqi.get() * _integral)
			       : thrust_to_cmd(ease * ff + _param_nl_kq.get() * eq + _param_nl_kqi.get() * _integral);

			if (!_ceiling && fabsf(_pitch - _target) < tol && fabsf(_q_settled) < 3.f) {
				if (_hold_since == 0) { _hold_since = now; }

				if ((now - _hold_since) * 1e-6f >= _param_nl_hold_s.get()) {
					_hold_cmd_avg = _cmd;
					set_state(State::Holding, now);

					if (_param_nl_auto.get() && _param_nl_fly_hold.get()) {
						// the nose fans' own flight loop multiplies every throttle change (NL_F_FF): the height
						// loop oscillated 0.2-0.76 of throttle in SITL. Not tuned for it: the pilot flies.
						mavlink_log_critical(&_mavlink_log_pub, "Nose lift: NL_AUTO needs NL_FLY_HOLD 0, the pilot flies the throttle\t");
					}

					if (_param_nl_auto.get() && !_param_nl_fly_hold.get()) {
						_auto = AutoPhase::Wait;
						_auto_since = now;

						if (ptko()) {
							request_px4_mode(3.f, 0.f, "Position (takeoff)", now);

						} else {
							request_stabilized(now);
						}
						mavlink_log_info(&_mavlink_log_pub, "Nose lift: holding at %.1f deg, automatic takeoff in %.0f s\t",
								 (double)_pitch, (double)_param_nl_auto_wait.get());

					} else {
						mavlink_log_info(&_mavlink_log_pub, "Nose lift: holding at %.1f deg, raise the throttle to take off\t",
								 (double)_pitch);
					}
				}

			} else {
				_hold_since = 0;
			}

			break;
		}

	case State::Holding: {
			if ((now - _state_since) * 1e-6f > _param_nl_hold_tout.get()) { abort(Abort::HoldTimeout, true, now); return; }

			const bool down = over_ceiling(now);
			const float q_des = down ? -rate : math::constrain(k_ang * (_target - _pitch), -rate, rate);
			const float eq = q_des - _q_f;
			_integral = math::constrain(_integral + eq * dt, -40.f, 40.f);
			_cmd = down ? thrust_to_cmd(ff + _param_nl_low_kq.get() * eq + _param_nl_low_kqi.get() * _integral)
			       : thrust_to_cmd(ff + _param_nl_kq.get() * eq + _param_nl_kqi.get() * _integral);

			// the nose command averaged over about 1 s while holding: what the nose fans need to keep the nose up
			_hold_cmd_avg += (_cmd - _hold_cmd_avg) * math::min(1.f, dt / 1.f);

			bool auto_go = _auto == AutoPhase::Wait && (now - _auto_since) * 1e-6f >= _param_nl_auto_wait.get();

			if (auto_go && _param_nl_auto_hcmd.get() > 0.f && _hold_cmd_avg > _param_nl_auto_hcmd.get()) {
				// 7 Oct SITL sweep: the takeoff passes while the nose holds at a command up to ~0.67 (nose fans worth
				// 28 N and more) and crashes from 0.75 (25 N: the nose fans saturate in the handover, the nose drops,
				// the aircraft slides forward); a sagging pack moved the aircraft across that line within one session
				mavlink_log_critical(&_mavlink_log_pub, "Nose lift: NL_AUTO refused, nose fans at %.0f%% to hold (limit %.0f%%): too weak, lowering\t",
						     (double)(100.f * _hold_cmd_avg), (double)(100.f * _param_nl_auto_hcmd.get()));
				abort(Abort::Weak, true, now);
				return;
			}

			if (auto_go && (ptko() ? !in_px4_hold() : !in_stabilized())) {
				// 6 Oct: armed in Position mode (no flight-mode channel), the throttle override became a climb-rate
				// command and the position controller flew the attitude: 1 m/s climbs, 12 deg nose-up. Never again in
				// the thrust-stick flight; NL_AUTO_PTKO takes off in Position on purpose, with a climb-rate stick
				auto_go = false;

				if ((now - _auto_since) * 1e-6f >= _param_nl_auto_wait.get() + 2.f) {
					_auto = AutoPhase::Off;
					mavlink_log_critical(&_mavlink_log_pub, "Nose lift: NL_AUTO refused, not in %s (mode %d); the pilot flies\t",
							     ptko() ? "Position" : "Stabilized", (int)_status.nav_state);
				}
			}

			if (auto_go) {
				start_auto_flight(now);
			}

			if (auto_go || throttle() > _param_nl_ho_thr.get()) {
				if (_param_nl_fly_hold.get()) {
					start_nose_hold(now);

				} else {
					set_state(State::Handover, now);
					_fading = false;
					_ho_off_legs = false;
					mavlink_log_info(&_mavlink_log_pub, "Nose lift: handing over to PX4\t");
				}
			}

			break;
		}

	case State::Handover: {
			// keep holding the nose until PX4 itself drives the lifting motors at least as hard as the hold (leaving
			// the ground is not enough: the spool-up would otherwise drop the nose), or the timeout; then fade out
			// NL_HO_KANG: a stiffer angle hold here than the raise's NL_K_ANG (9 Oct SITL 32 N: with 1.0 the nose sagged
			// 8.6 -> 7.7 deg over the throttle ramp; the aircraft then slid forward and PX4 braked at 5-7 deg/s)
			const float k_ho = _param_nl_ho_kang.get() > 0.f ? _param_nl_ho_kang.get() : k_ang;
			const float q_des = math::constrain(k_ho * (_target - _pitch), -rate, rate);
			const float eq = q_des - _q_f;
			_integral = math::constrain(_integral + eq * dt, -40.f, 40.f);
			// NL_HO_FF: the rear fans' thrust rising on the legs takes the nose down (8 Oct SITL 36 N: 8.5 -> 7.1 deg
			// over the ramp, then PX4 snapped it back at the lift-off); feed it forward to the nose fans
			const float hold = thrust_to_cmd(ff + _param_nl_ho_ff.get() * rear_thrust() + _param_nl_kq.get() * eq
							 + _param_nl_kqi.get() * _integral);

			float px4_share = 0.f;

			if (now - _feedback.timestamp < 100_ms) {
				px4_share = 1e9f;

				for (int k = 0; k < _lift_count; k++) {
					const float c = _feedback.allocator_control[_lift_motor[k]];
					px4_share = math::min(px4_share, PX4_ISFINITE(c) ? motor_command(c) : 0.f);
				}
			}

			// judged against at least the balance thrust: the rate term on the legs' rocking can take the hold to 0 for
			// a moment, and "PX4 above 0" then passed at once and faded from 0 (23 Sep 23:25: nose 22 -> 9 deg in 0.3 s)
			const float ref = math::max(hold, thrust_to_cmd(ff));
			const bool taken_over = px4_share >= 0.95f * ref;

			// NL_HO_LIFT: in an automatic flight the floor is kept until the aircraft is off its legs (the range finder
			// NL_HO_LIFT_H above its takeoff reading; the legs' springs alone extend ~3 cm as the rear fans unload them). 8 Oct SITL 36 N: PX4 held the pitch poorly on the legs, the nose sagged 2.2 deg
			// during the fade and PX4 snapped it back at 8.9 deg/s at the lift-off
			const bool off_legs = !(_param_nl_ho_lift.get() && auto_flying() && range_ok() && PX4_ISFINITE(_auto_rng0))
					      || _dist.current_distance - _auto_rng0 > _param_nl_ho_lift_h.get();

			_ho_off_legs = off_legs;

			if (!_fading && ((taken_over && off_legs) || (now - _state_since) * 1e-6f > _param_nl_ho_tout.get() || !_rc_ok)) {
				_fading = true;
				_fade_start = now;
				_fade_from = ref;
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

	case State::NoseHold: {
			// the nose fans hold NL_TGT on their own loop for the whole flight; PX4 flies the rear fans. The radio,
			// roll, overshoot and liftoff checks of the ground sequence do not apply: cutting the nose fans in the air
			// drops the nose. The kill switch (above) still stops everything.
			if (_sb_fell) { _low_thr_told = false; }

			if (!_sb_on && _rc_ok) {
				if (throttle() <= _param_nl_start_thr.get()) {
					lower_from_nose_hold(now);
					return;

				} else if (!_low_thr_told) {
					_low_thr_told = true;
					mavlink_log_critical(&_mavlink_log_pub, "Nose lift: land and lower the throttle first, then the switch\t");
				}
			}

			// the rear fans' thrust leans forward and pushes the aircraft along the floor in proportion to it: the nose
			// angle follows their thrust, NL_TGT with them idle up to NL_F_TGT at NL_F_TGT_THR (liftoff), tilting that
			// push back as it builds, and down again as the throttle comes down for the landing
			const float rear = rear_thrust();
			_rear_f += (rear - _rear_f) * (dt / (dt + 0.3f));
			const float frac = math::constrain(_rear_f / math::max(_param_nl_f_tgt_thr.get(), 1e-3f), 0.f, 1.f);
			const float goal = _param_nl_tgt.get() + frac * (_param_nl_f_tgt.get() - _param_nl_tgt.get());
			const float step = _param_nl_f_tgt_rate.get() * dt;
			_target += math::constrain(goal - _target, -step, step);

			const float f_rate = _param_nl_f_rate.get();
			const float q_des = math::constrain(_param_nl_f_k_ang.get() * (_target - _pitch), -f_rate, f_rate);
			const float eq = q_des - _q_f;
			const float f = _param_nl_f_ff.get() * rear + _param_nl_f_kq.get() * eq
					+ _param_nl_f_kqi.get() * _integral;

			// no integration into a saturated command
			if (!((f >= _param_nl_max_cmd.get() && eq > 0.f) || (f <= 0.f && eq < 0.f))) {
				_integral = math::constrain(_integral + eq * dt, -40.f, 40.f);
			}

			_cmd = thrust_to_cmd(f);
			break;
		}

	case State::Lowering: {
			// a cancel: bring the nose back to where the lift started, then stop the motors and disarm
			if ((now - _state_since) * 1e-6f > _param_nl_tout.get()) { abort(Abort::LowerTimeout, false, now); return; }

			if (!_fading) {
				// ease the descent in over 1 s: a full-rate demand at once, through the stiffer lowering gain,
				// dips the thrust and drops the nose for a moment (nose_lift.py NoseLower eases the same way)
				// (not above the ceiling: a cancel while the nose is still running up must brake at once)
				const float ceil = _param_nl_ceil.get();
				const bool high = ceil > 0.f && _pitch > _param_nl_tgt.get() + ceil;
				const float ease = high ? 1.f : math::min(1.f, (now - _state_since) * 1e-6f / 1.f);
				// 9 Oct HITL: the target is the pitch PX4 estimated before the lift; a few seconds after a reboot that
				// estimate is 1 to 2 deg off, the nose stopped 1.4 deg above its front leg, the fade cut the fans and
				// the nose fell onto the leg at 12 deg/s. With NL_LOW_VMIN the nose keeps coming down at least that
				// fast until the leg stops it, and the fade starts on that contact, not on an estimated angle
				const float vmin = math::max(_param_nl_low_vmin.get(), 0.f);
				float q_des = math::constrain(k_ang * (_target - _pitch), -rate * ease, rate);

				if (vmin > 0.f && !high) {
					q_des = math::min(q_des, -vmin * ease);
				}

				const float eq = q_des - _q_f;
				_integral = math::constrain(_integral + eq * dt, -40.f, 40.f);
				_cmd = thrust_to_cmd(ff + _param_nl_low_kq.get() * eq + _param_nl_low_kqi.get() * _integral);

				// fade only once the nose sits on its front leg again: fading from further up drops the last degrees
				// (one-sided: legs that compress a little more than before may leave it slightly below where it started).
				// With NL_LOW_VMIN: on the leg = still asked down but standing still near the start pitch, or well
				// below it (no leg found)
				const bool at_target = vmin > 0.f
						       ? ((_pitch - _target < tol && fabsf(_q_settled) < 0.3f * vmin && ease >= 1.f)
							  || _pitch < _target - 3.f)
						       : (_pitch - _target < math::min(tol, 0.5f) && fabsf(_q_settled) < 1.f);

				if (at_target) {
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

// mean thrust PX4 commands to the motors the nose lift does not own (the rear fans); 0 without fresh feedback
float NoseLift::rear_thrust() const
{
	if (hrt_elapsed_time(&_feedback.timestamp) > 100_ms) {
		return 0.f;
	}

	const int n = math::constrain(static_cast<int>(_param_ca_rotor_count.get()), 1, NUM_MOTORS);
	const int mask = _param_nl_mot_msk.get();
	float sum = 0.f;
	int count = 0;

	for (int i = 0; i < n; i++) {
		if (!(mask & (1 << i))) {
			const float c = _feedback.allocator_control[i];
			sum += PX4_ISFINITE(c) ? math::max(c, 0.f) : 0.f;
			count++;
		}
	}

	return count > 0 ? sum / count : 0.f;
}

void NoseLift::start_nose_hold(hrt_abstime now)
{
	// bumpless: the integral starts where the flight loop gives the thrust the hold had
	const float f_prev = powf(math::max(_cmd, 0.f), math::max(_param_nl_expo.get(), 1e-3f));
	const float f_rate = _param_nl_f_rate.get();
	const float eq = math::constrain(_param_nl_f_k_ang.get() * (_target - _pitch), -f_rate, f_rate) - _q;
	const float rest = f_prev - _param_nl_f_ff.get() * rear_thrust() - _param_nl_f_kq.get() * eq;
	_integral = math::constrain(rest / math::max(_param_nl_f_kqi.get(), 1e-6f), -40.f, 40.f);
	_low_thr_told = false;
	_rear_f = rear_thrust();
	set_state(State::NoseHold, now);
	mavlink_log_info(&_mavlink_log_pub, "Nose lift: nose fans hold %.1f deg for the flight, PX4 flies the rear fans\t",
			 (double)_target);
}

void NoseLift::lower_from_nose_hold(hrt_abstime now)
{
	// back on the ground with the throttle down: the cancel's lowering, from the thrust the flight loop had, with the
	// liftoff check's height references taken here (the estimate has moved since the lift started)
	const float f_prev = powf(math::max(_cmd, 0.f), math::max(_param_nl_expo.get(), 1e-3f));
	const float rest = f_prev - balance_fraction() - _param_nl_low_kq.get() * (0.f - _q);
	_integral = math::constrain(rest / math::max(_param_nl_low_kqi.get(), 1e-6f), -40.f, 40.f);
	_target = _pitch0;
	_fading = false;
	_hold_since = 0;
	_z0 = _lpos.z_valid ? _lpos.z : NAN;
	_z_reset_counter = _lpos.z_reset_counter;
	_baro0 = _baro_f;
	_abort_reason = Abort::SwitchOff;
	set_state(State::Lowering, now);
	mavlink_log_info(&_mavlink_log_pub, "Nose lift: landed, lowering the nose to %.1f deg\t", (double)_target);
}

// ------------------------------------------------------------------------------------------------- automatic flight

bool NoseLift::in_stabilized() const
{
	return _status.nav_state == vehicle_status_s::NAVIGATION_STATE_STAB;
}

void NoseLift::touchdown_lower(hrt_abstime now, const char *why)
{
	request_stabilized(now);
	mavlink_log_info(&_mavlink_log_pub, "Nose lift: touchdown under %s (%s), lowering\t", why,
			 _land.landed ? "landed" : (_land.maybe_landed ? "maybe landed" : (_land.ground_contact ? "ground contact" : "still")));
	_auto_thr = 0.f;
	end_auto(now);

	if (_state == State::Flying || _state == State::NoseHold) {
		if (_state == State::NoseHold) { lower_from_nose_hold(now); } else { lower_from_flight(now); }

	} else {
		abort(Abort::SwitchOff, true, now);
	}
}

bool NoseLift::range_ok() const
{
	return _dist.timestamp > 0 && hrt_elapsed_time(&_dist.timestamp) < 200_ms && _dist.signal_quality != 0
	       && PX4_ISFINITE(_dist.current_distance) && _dist.current_distance > _dist.min_distance
	       && _dist.current_distance < _dist.max_distance;
}

float NoseLift::ekf_height(float h_baro) const
{
	// height above the takeoff point. 8 Oct SITL: the estimator's height crept up 0.3 to 0.4 m during the flight
	// (fan vibration), so the "1 m" hover sat at 0.64 m and on the ground it still read 0.4 m: the touchdown cue never
	// fired. First choice: the range finder against its reading at the takeoff (same attitude, on the legs at the
	// hover pitch); then the estimator against its height at the takeoff; then the baro
	if (range_ok() && PX4_ISFINITE(_auto_rng0) && _param_nl_auto_rng.get()) {
		return _dist.current_distance - _auto_rng0;
	}

	if (_lpos.z_valid && PX4_ISFINITE(_auto_z0) && hrt_elapsed_time(&_lpos.timestamp) < 200_ms) {
		return _auto_z0 - _lpos.z;
	}

	return h_baro;
}

float NoseLift::ekf_climb_rate() const
{
	return _lpos.v_z_valid && hrt_elapsed_time(&_lpos.timestamp) < 200_ms ? -_lpos.vz : _baro_vz;
}

float NoseLift::stick_for_climb(float v_up) const
{
	// the throttle stick that makes PX4's Position mode climb at v_up (m/s, negative = descend): the inverse of
	// FlightTaskManualAltitude::_scaleSticks, v = MPC_Z_VEL_MAX_UP/DN * expo_deadzone(2 s - 1, 0.6, MAN_DEADZONE)
	if (fabsf(v_up) < 1e-3f) {
		return 0.5f;
	}

	const float vmax = v_up > 0.f ? _param_mpc_z_vel_max_up.get() : _param_mpc_z_vel_max_dn.get();
	const float frac = math::constrain(fabsf(v_up) / math::max(vmax, 0.1f), 0.f, 1.f);
	const float e = 0.6f;
	float lo = 0.f, hi = 1.f;

	for (int i = 0; i < 24; i++) {
		const float x = 0.5f * (lo + hi);

		if ((1.f - e) * x + e * x * x * x < frac) { lo = x; } else { hi = x; }
	}

	const float dz = math::constrain(_param_man_deadzone.get(), 0.f, 0.9f);
	const float z = dz + (1.f - dz) * 0.5f * (lo + hi);
	return math::constrain(0.5f + (v_up > 0.f ? 0.5f : -0.5f) * z, 0.f, 1.f);
}

void NoseLift::update_touchdown(float h, hrt_abstime now)
{
	// the module's own touchdown cue (NL_AUTO_TD 3): near the ground and no longer descending. SITL 8 Oct: the
	// vertical speed reaches zero 0.3 s after the rear feet touch, PX4's ground-contact stage only 2.5 to 3.5 s later
	const float vz = _lpos.v_z_valid && hrt_elapsed_time(&_lpos.timestamp) < 200_ms ? -_lpos.vz : _baro_vz;
	_td_vz = vz;

	// stillness below half the creep speed: a creep slower than 0.12 m/s must not read as "still" (8 Oct SITL: a 0.06
	// m/s creep fired the cue 10 cm up and dropped the aircraft)
	const float still = math::min(0.06f, 0.5f * math::max(_param_nl_auto_vtd.get(), 0.02f));

	if (h < 0.6f && fabsf(vz) < still) {
		if (_td_still_since == 0) { _td_still_since = now; }

	} else {
		_td_still_since = 0;
	}
}

bool NoseLift::touched_down(float h, float h_max) const
{
	// 8 Oct SITL: PX4 declares "landed" ~5 s after the rear feet touch; meanwhile it winds the thrust down on two feet
	// and the nose falls forward uncontrolled at ~3 deg/s (8.5 -> -6 deg). NL_AUTO_TD picks the land detector stage
	// at which the module takes the motors and lowers the nose itself: 0 landed, 1 maybe landed, 2 ground contact.
	// Only near the ground (h above the takeoff point), so an early stage cannot fire in the air.
	const int stage = _param_nl_auto_td.get();
	const bool flag = _land.landed || (stage >= 1 && _land.maybe_landed) || (stage >= 2 && _land.ground_contact);
	// stage 3: still for 0.25 s near the ground, or the nose already tipping forward while hardly descending
	const bool own = stage >= 3 && h < h_max
			 && ((_td_still_since != 0 && hrt_elapsed_time(&_td_still_since) > 150_ms)
			     || (_pitch < _target - 2.f && fabsf(_td_vz) < 0.15f));
	return own || (flag && (_land.landed || h < h_max));
}

bool NoseLift::ptko() const
{
	return _param_nl_auto_ptko.get() && _param_nl_auto_hold.get() && _param_nl_auto_pclb.get();
}

bool NoseLift::in_px4_hold() const
{
	return _status.nav_state == vehicle_status_s::NAVIGATION_STATE_POSCTL
	       || _status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER;
}

bool NoseLift::in_px4_land() const
{
	return _status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_LAND;
}

bool NoseLift::px4_position_ok() const
{
	return _lpos.xy_valid && _lpos.v_xy_valid && _lpos.z_valid && hrt_elapsed_time(&_lpos.timestamp) < 500_ms;
}

void NoseLift::request_px4_mode(float main_mode, float sub_mode, const char *name, hrt_abstime now)
{
	vehicle_command_s cmd{};
	cmd.command = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
	cmd.param1 = 1.f;	// MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
	cmd.param2 = main_mode;
	cmd.param3 = sub_mode;
	cmd.target_system = _status.system_id;
	cmd.target_component = _status.component_id;
	cmd.source_system = _status.system_id;
	cmd.source_component = _status.component_id;
	cmd.from_external = false;
	cmd.timestamp = now;
	_command_pub.publish(cmd);
	mavlink_log_info(&_mavlink_log_pub, "Nose lift: asking PX4 for %s\t", name);
}

void NoseLift::request_stabilized(hrt_abstime now)
{
	// the automatic flight is a throttle hand in Stabilized: ask PX4 for that mode (PX4_CUSTOM_MAIN_MODE_STABILIZED
	// = 7); the takeoff waits for the mode to show in vehicle_status and is refused if it never does
	if (in_stabilized()) {
		return;
	}

	vehicle_command_s cmd{};
	cmd.command = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
	cmd.param1 = 1.f;	// MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
	cmd.param2 = 7.f;	// PX4_CUSTOM_MAIN_MODE_STABILIZED
	cmd.param3 = 0.f;
	cmd.target_system = _status.system_id;
	cmd.target_component = _status.component_id;
	cmd.source_system = _status.system_id;
	cmd.source_component = _status.component_id;
	cmd.from_external = false;
	cmd.timestamp = now;
	_command_pub.publish(cmd);
	mavlink_log_info(&_mavlink_log_pub, "Nose lift: switching to Stabilized for the automatic flight\t");
}

void NoseLift::start_auto_flight(hrt_abstime now)
{
	_auto = AutoPhase::Climb;
	_auto_since = now;
	_auto_h0 = _baro_f;
	_auto_z0 = _lpos.z_valid ? _lpos.z : NAN;
	_auto_rng0 = range_ok() ? _dist.current_distance : NAN;
	_pclimb = false;
	_pdesc = false;
	_pclimb_v = 0.f;
	_auto_h_t = 0.f;
	_auto_vz_t = 0.f;
	_auto_int = 0.f;
	_auto_thr = 0.f;
	_auto_still_since = 0;
	_hold_refused_told = false;
	_hold_refused = false;
	_auto_hover_t0 = now;

	if (ptko()) {
		// NL_AUTO_PTKO: PX4 Position takes off from the legs (its own takeoff ramp, the position held from the
		// start); the module asks for the climb with the stick, eased in from 0 at NL_AUTO_PACC
		_pclimb = true;
		_auto = AutoPhase::HoldPX4;
	}

	mavlink_log_info(&_mavlink_log_pub, "Nose lift: automatic takeoff to %.1f m, hover %.0f s, then landing\t",
			 (double)_param_nl_auto_alt.get(), (double)_param_nl_auto_hov.get());
}

void NoseLift::auto_land_now(const char *why, hrt_abstime now)
{
	_auto = AutoPhase::Descend;
	_auto_since = now;
	_auto_still_since = 0;
	mavlink_log_info(&_mavlink_log_pub, "Nose lift: landing (%s) from %.1f m\t", why,
			 (double)(PX4_ISFINITE(_baro_f) && PX4_ISFINITE(_auto_h0) ? _baro_f - _auto_h0 : 0.f));
}

void NoseLift::end_auto(hrt_abstime now)
{
	_auto = AutoPhase::Done;
	_auto_since = now;
	_auto_thr = NAN;
}

void NoseLift::lower_from_flight(hrt_abstime now)
{
	// landed after a handover flight (NL_FLY_HOLD 0): take every motor back, the rear fans stop, the nose fans
	// lower the nose from the balance thrust (the cancel's lowering, as nose_lift.py NoseLower cuts the others)
	// 8 Oct SITL: taking the motors from zero (rear fans off at once, nose fans from _cmd 0 through the lowering
	// loop) jolted the aircraft at +-14 deg/s right after the touchdown. Bumpless: the nose fans start from PX4's
	// last command (the integral is set so the loop gives it), the rear fans keep their thrust and fade out over
	// NL_LOW_RRAMP
	_integral = 0.f;
	_cmd = 0.f;
	_rear_ramp = false;

	// PX4's own motor outputs (actuator_motors, always published; 8 Oct SITL: nose_lift_feedback only comes while
	// the module overrides, so it was stale here and the lowering started every motor from zero)
	const bool act_fresh = _act.timestamp > 0 && hrt_elapsed_time(&_act.timestamp) < 100_ms;
	const bool fb_fresh = hrt_elapsed_time(&_feedback.timestamp) < 100_ms;

	if (act_fresh || fb_fresh) {
		float sum = 0.f;

		for (int k = 0; k < _lift_count; k++) {
			const float cf = act_fresh ? _act.control[_lift_motor[k]] : _feedback.allocator_control[_lift_motor[k]];
			sum += PX4_ISFINITE(cf) ? motor_command(math::max(cf, 0.f)) : 0.f;
		}

		_cmd = _lift_count > 0 ? sum / _lift_count : 0.f;
		const float f_prev = powf(math::max(_cmd, 0.f), math::max(_param_nl_expo.get(), 1e-3f));
		const float rest = f_prev - balance_fraction() + _param_nl_low_kq.get() * _q_f;	// eq = 0 - q at the start
		_integral = math::constrain(rest / math::max(_param_nl_low_kqi.get(), 1e-6f), -40.f, 40.f);

		const int n = math::constrain(static_cast<int>(_param_ca_rotor_count.get()), 1, NUM_MOTORS);
		const int mask = _param_nl_mot_msk.get();

		for (int i = 0; i < n; i++) {
			const float cf = act_fresh ? _act.control[i] : _feedback.allocator_control[i];
			_rear_from[i] = (!(mask & (1 << i)) && PX4_ISFINITE(cf)) ? math::max(cf, 0.f) : 0.f;
		}

		_rear_ramp = _param_nl_low_rramp.get() > 0.f;
		_rear_t0 = now;
	}

	_target = _pitch0;
	_fading = false;
	_hold_since = 0;
	_z0 = _lpos.z_valid ? _lpos.z : NAN;
	_z_reset_counter = _lpos.z_reset_counter;
	_baro0 = _baro_f;
	_abort_reason = Abort::None;
	set_state(State::Lowering, now);
	mavlink_log_info(&_mavlink_log_pub, "Nose lift: landed, lowering the nose to %.1f deg\t", (double)_target);
}

void NoseLift::step_auto(hrt_abstime now, float dt, bool kill)
{
	if (!auto_flying()) {
		return;
	}

	if (kill) {
		// PX4's kill has stopped every motor already; nothing resumes
		end_auto(now);

		if (_state == State::Flying) {
			abort(Abort::Kill, false, now);
		}

		return;
	}

	if (_state != State::Handover && _state != State::Flying && _state != State::NoseHold) {
		// the sequence left the flight (an abort, a lowering): the automatic throttle has nothing to fly
		end_auto(now);
		return;
	}

	const int pilot = _param_nl_auto_pilot.get();

	const bool under_px4 = _auto == AutoPhase::HoldWait || _auto == AutoPhase::HoldPX4 || _auto == AutoPhase::LandPX4;

	if (pilot > 0 && pilot_throttle_us() > pilot) {
		mavlink_log_critical(&_mavlink_log_pub, "Nose lift: throttle stick raised, the pilot flies the throttle\t");

		if (under_px4) {
			// no flight-mode switch on the transmitter: give the pilot a manual mode to fly in
			request_stabilized(now);
		}

		end_auto(now);
		return;
	}

	if (_sb_fell && (_auto == AutoPhase::Climb || _auto == AutoPhase::Hover)) {
		auto_land_now("switch off", now);
	}

	if (_sb_fell && _auto == AutoPhase::HoldPX4 && _param_nl_auto_pdsc.get() && in_px4_hold()) {
		// switch off under Position: the module's own gentle descent from here (NL_AUTO_VTD touchdown), as at the
		// end of the hover (PX4 Land put the feet down at 0.2 m/s)
		_pclimb = false;

		if (!_pdesc) {
			_pdesc = true;
			_pdesc_t0 = now;
			_pclimb_v = math::min(_pclimb_v, 0.f);
			_td_still_since = 0;
			mavlink_log_info(&_mavlink_log_pub, "Nose lift: switch off, descending in PX4 Position\t");
		}

	} else if (_sb_fell && (_auto == AutoPhase::HoldWait || _auto == AutoPhase::HoldPX4)) {
		request_px4_mode(4.f, 6.f, "Land (switch off)", now);	// PX4_CUSTOM_MAIN_MODE_AUTO, SUB_MODE_AUTO_LAND
		_auto = AutoPhase::LandPX4;
		_auto_since = now;
	}

	const float el = (now - _auto_since) * 1e-6f;
	const float h = PX4_ISFINITE(_baro_f) && PX4_ISFINITE(_auto_h0) ? _baro_f - _auto_h0 : 0.f;
	const float alt = _param_nl_auto_alt.get();
	const float vup = _param_nl_auto_vup.get();
	const float vdn = _param_nl_auto_vdn.get();
	// the feed-forward is a STICK position: with MPC_THR_CURVE 0 or 2 PX4 rescales the stick so that mid stick is
	// the hover thrust (6 Oct: MPC_THR_HOVER sent as a stick read 14 % above hover); only curve 1 maps the stick
	// straight to thrust
	const float ff = _param_nl_auto_thr.get() > 0.f ? _param_nl_auto_thr.get()
			 : (_param_mpc_thr_curve.get() == 1 ? _param_mpc_thr_hover.get() : 0.5f);

	if ((_auto == AutoPhase::Climb || (_auto == AutoPhase::HoldPX4 && _pclimb && ptko())) && _param_nl_auto_hdrop.get() > 0.f
	    && el < 3.f && ekf_height(h) < 0.3f
	    && _pitch < _target - _param_nl_auto_hdrop.get()) {
		// 7 Oct 15:38: in the handover the nose fans went to full while the rear fans were still spooling, the nose
		// fell from 8.8 to -26 deg with the aircraft still on its legs. Still near the ground and early in the
		// takeoff: cut the automatic throttle (0.5 s) and lower the nose rather than carry on
		mavlink_log_critical(&_mavlink_log_pub, "Nose lift: takeoff cancelled, nose %.1f deg under the target in the handover\t",
				     (double)(_target - _pitch));
		_auto = AutoPhase::Cut;
		_auto_since = now;
		_auto_cut_from = PX4_ISFINITE(_auto_thr) ? _auto_thr : 0.f;
		return;
	}

	switch (_auto) {
	case AutoPhase::Climb: {
			const float ramp = math::max(_param_nl_auto_ramp.get(), 0.1f);

			// NL_AUTO_PCLB 1: at the lift-off; 2: as soon as the throttle ramp has the hover thrust, still on the legs
			// (8 Oct SITL: Position from the lift-off still braked a forward slide that began on the legs)
			const bool at_liftoff = !_land.landed && ekf_height(h) > 0.06f && ekf_climb_rate() > 0.1f;
			const bool at_ramp_end = _param_nl_auto_pclb.get() >= 2 && el >= ramp;

			if (_param_nl_auto_hold.get() && _param_nl_auto_pclb.get() && !_hold_refused && px4_position_ok()
			    && (at_liftoff || at_ramp_end)) {
				// 8 Oct SITL: climbing in Stabilized nobody holds the position: up to 1 m/s forward and 3.7 m of
				// drift before PX4 Position took over at the hover, and its braking was the flight's worst jolt
				// (20 to 38 deg/s). NL_AUTO_PCLB: PX4 Position takes over as soon as the aircraft is off its legs
				// and the module climbs it with a climb-rate stick
				_pclimb = true;
				_pclimb_v = math::constrain(ekf_climb_rate(), 0.f, vup);
				_auto_since = now;	// the climb timeout and the handover guard count from here
				request_px4_mode(3.f, 0.f, "Position (climb)", now);	// PX4_CUSTOM_MAIN_MODE_POSCTL
				_auto = AutoPhase::HoldWait;
				_auto_since = now;
				_auto_thr = stick_for_climb(_pclimb_v);
				return;
			}

			if (el < ramp) {
				// the throttle ramps to the hover feed-forward with the height target at the ground: no loop yet
				_auto_thr = ff * el / ramp;
				_auto_h_t = 0.f;
				_auto_vz_t = 0.f;
				return;
			}

			// the target climbs at NL_AUTO_VUP but never runs more than 0.5 m ahead of the aircraft: while it still
			// sits on its legs the loop would otherwise wind up and leap off (SITL: 2.5 m and a 3 m/s fall)
			_auto_h_t = math::min(math::min(_auto_h_t + vup * dt, alt), h + 0.5f);
			_auto_vz_t = _auto_h_t < alt ? vup : 0.f;

			if (_auto_h_t >= alt && h > alt - 0.3f) {
				_auto = AutoPhase::Hover;
				_auto_since = now;
				_auto_hover_t0 = now;
				mavlink_log_info(&_mavlink_log_pub, "Nose lift: hovering at %.1f m for %.0f s\t", (double)h,
						 (double)_param_nl_auto_hov.get());

			} else if (el > ramp + alt / math::max(vup, 0.05f) + 15.f) {
				_auto = AutoPhase::Hover;
				_auto_since = now;
				_auto_hover_t0 = now;
				mavlink_log_critical(&_mavlink_log_pub, "Nose lift: climb timed out at %.1f m, hovering here\t", (double)h);
			}

			break;
		}

	case AutoPhase::Hover: {
		_auto_vz_t = 0.f;
		const float hov_el = (now - _auto_hover_t0) * 1e-6f;

		if (_param_nl_auto_hold.get() && !_hold_refused && el >= 1.f && px4_position_ok()) {
			// the module's loop has the aircraft at the height and settled: PX4 Position mode holds the position (on
			// the H-FLOW) and, with the module's stick at mid, the height (NL_AUTO_HOLD). 8 Oct SITL: Auto Loiter is
			// refused without a global position, and the old code asked again every 3 s with the hover clock
			// restarted, so it hovered for ever. One request; a refusal falls back to this hover for the rest of it
			request_px4_mode(3.f, 0.f, "Position", now);	// PX4_CUSTOM_MAIN_MODE_POSCTL
			_auto = AutoPhase::HoldWait;
			_auto_since = now;
			return;
		}

		if (_param_nl_auto_hold.get() && el >= 3.f && !px4_position_ok() && !_hold_refused_told) {
			_hold_refused_told = true;
			mavlink_log_critical(&_mavlink_log_pub, "Nose lift: no valid position for Hold, hovering on the module\t");
		}

		if (hov_el >= _param_nl_auto_hov.get()) {
			auto_land_now("hover done", now);
		}

		break;
	}

	case AutoPhase::HoldWait:
		// mid stick under PX4 (or the climb rate while climbing in Position): a manual-mode fall-back holds height
		_auto_thr = _pclimb ? stick_for_climb(_pclimb_v) : ff;

		if (in_px4_hold()) {
			_auto = AutoPhase::HoldPX4;
			_auto_since = now;
			mavlink_log_info(&_mavlink_log_pub, "Nose lift: PX4 Position hold at %.1f m, then PX4 Land\t", (double)h);

		} else if (el > 2.f) {
			mavlink_log_critical(&_mavlink_log_pub, "Nose lift: Position refused (mode %d), hovering on the module\t",
					     (int)_status.nav_state);
			_hold_refused = true;
			request_stabilized(now);

			if (_pclimb) {
				// refused at lift-off: the module's own climb carries on (past its ramp)
				_pclimb = false;
				_auto = AutoPhase::Climb;
				_auto_since = now - (hrt_abstime)(1e6f * math::max(_param_nl_auto_ramp.get(), 0.1f));
				_auto_h_t = h;

			} else {
				_auto = AutoPhase::Hover;
				_auto_since = now;
			}
		}

		return;

	case AutoPhase::HoldPX4:
		_auto_thr = ff;

		if (_pclimb && in_px4_hold()) {
			// climb in Position: the climb rate eases into the target height, its change limited to NL_AUTO_PACC
			const float hz = ekf_height(h);
			const float v_t = math::constrain(_param_nl_auto_pkz.get() * (alt - hz), 0.f, vup);
			const float acc = math::max(_param_nl_auto_pacc.get(), 0.05f);
			_pclimb_v += math::constrain(v_t - _pclimb_v, -acc * dt, acc * dt);
			_auto_thr = stick_for_climb(_pclimb_v);

			if ((hz > alt - 0.05f && _pclimb_v < 0.05f) || el > alt / math::max(vup, 0.05f) + 20.f) {
				_pclimb = false;
				_auto_hover_t0 = now;
				_auto_thr = 0.5f;
				mavlink_log_info(&_mavlink_log_pub, "Nose lift: hovering at %.2f m (PX4 Position) for %.0f s\t", (double)hz,
						 (double)_param_nl_auto_hov.get());
			}

			return;
		}

		if (!in_px4_hold()) {
			mavlink_log_critical(&_mavlink_log_pub, "Nose lift: PX4 left Position (mode %d), landing on the module\t",
					     (int)_status.nav_state);
			request_stabilized(now);
			auto_land_now("hold lost", now);

		} else if (_pdesc) {
			// descent in Position (NL_AUTO_PDSC): eases from NL_AUTO_VDN down to NL_AUTO_VTD at NL_AUTO_HTD, then
			// creeps until the feet are down (8 Oct SITL: PX4 Land put the feet down at 0.20 m/s, its crawl never
			// engaged, and the rear-feet impact pitched the nose down at ~15 deg/s)
			const float hz = ekf_height(h);
			const float vtd = math::max(_param_nl_auto_vtd.get(), 0.02f);
			const float v_t = -(vtd + math::min(math::max(vdn - vtd, 0.f),
							    _param_nl_auto_pkz.get() * math::max(hz - _param_nl_auto_htd.get(), 0.f)));
			const float acc = math::max(_param_nl_auto_pacc.get(), 0.05f);
			_pclimb_v += math::constrain(v_t - _pclimb_v, -acc * dt, acc * dt);
			_auto_thr = stick_for_climb(_pclimb_v);
			update_touchdown(hz, now);

			// with the range finder the height is good to a centimetre or two: the feet are down below 6 cm
			if (touched_down(hz, range_ok() && PX4_ISFINITE(_auto_rng0) && _param_nl_auto_rng.get() ? 0.06f : 0.15f)) {
				_pdesc = false;
				touchdown_lower(now, "Position descent");

			} else if ((now - _pdesc_t0) * 1e-6f > 60.f) {
				_pdesc = false;
				mavlink_log_critical(&_mavlink_log_pub, "Nose lift: Position descent timed out, PX4 Land\t");
				request_px4_mode(4.f, 6.f, "Land", now);
				_auto = AutoPhase::LandPX4;
				_auto_since = now;
			}

		} else if (!_pclimb && (now - _auto_hover_t0) * 1e-6f >= _param_nl_auto_hov.get()) {
			if (_param_nl_auto_pdsc.get()) {
				_pdesc = true;
				_pdesc_t0 = now;
				_pclimb_v = 0.f;
				_td_still_since = 0;
				mavlink_log_info(&_mavlink_log_pub, "Nose lift: descending in PX4 Position from %.2f m\t", (double)ekf_height(h));

			} else {
				request_px4_mode(4.f, 6.f, "Land", now);
				_auto = AutoPhase::LandPX4;
				_auto_since = now;
			}
		}

		return;

	case AutoPhase::LandPX4:
		_auto_thr = ff;
		update_touchdown(h, now);

		if (touched_down(h, 0.6f)) {
			// the feet are down: back to Stabilized and the module lowers the nose under control at once (no 0.5 s
			// cut: PX4 Land already has the thrust down, and the nose fell ~2 deg in that half second in SITL)
			touchdown_lower(now, "PX4 Land");

		} else if (!in_px4_land() && el > 2.f) {
			mavlink_log_critical(&_mavlink_log_pub, "Nose lift: PX4 left Land (mode %d), landing on the module\t",
					     (int)_status.nav_state);
			request_stabilized(now);
			auto_land_now("land lost", now);

		} else if (el > 60.f) {
			mavlink_log_critical(&_mavlink_log_pub, "Nose lift: PX4 Land timed out, landing on the module\t");
			request_stabilized(now);
			auto_land_now("land timeout", now);
		}

		return;

	case AutoPhase::Descend: {
			update_touchdown(h, now);
			// the target runs on below the ground: the loop winds the thrust down until the aircraft sits still
			// flare: at most 0.2 m/s in the last metre; the target leads the aircraft by at most 0.15 m down so a lagging loop does not add to the descent (SITL: 0.8 m/s touchdown without this)
			// 30 Sep: 0.4 m/s to 0.7 m then 0.15 m/s gave 0.17 m/s touchdowns; 0.39 m/s and a 7-9 deg/s jolt without
			const float v = h < _param_nl_auto_hfl.get() ? math::min(vdn, _param_nl_auto_vfl.get()) : vdn;
			_auto_h_t = math::max(_auto_h_t - v * dt, h - 0.15f);
			_auto_vz_t = -v;
			const bool low = _auto_h_t <= h - 0.1f && h < 0.8f && fabsf(_baro_vz) < 0.25f;

			if (low) {
				if (_auto_still_since == 0) { _auto_still_since = now; }

			} else {
				_auto_still_since = 0;
			}

			const bool down = (_auto_still_since != 0 && now - _auto_still_since > 1_s) || _auto_h_t <= -2.f
					  || (_land.landed && _auto_h_t <= 0.f) || (_param_nl_auto_td.get() > 0 && touched_down(h, 0.6f));

			if (down) {
				_auto = AutoPhase::Cut;
				_auto_since = now;
				_auto_cut_from = PX4_ISFINITE(_auto_thr) ? _auto_thr : 0.f;
				mavlink_log_info(&_mavlink_log_pub, "Nose lift: touchdown (throttle %.2f), cutting\t", (double)_auto_cut_from);
				return;
			}

			break;
		}

	case AutoPhase::Cut: {
			const float k = el / 0.5f;
			_auto_thr = _auto_cut_from * math::max(0.f, 1.f - k);

			if (k >= 1.f) {
				_auto_thr = 0.f;
				end_auto(now);

				if (_state == State::NoseHold) {
					lower_from_nose_hold(now);

				} else if (_state == State::Flying) {
					lower_from_flight(now);

				} else {
					// still handing over (never took off properly): the cancel's lowering
					abort(Abort::SwitchOff, true, now);
				}
			}

			return;
		}

	default:
		return;
	}

	// the height loop: hover feed-forward plus height, climb-rate and integral terms, on the barometer
	// climb rate: the estimator's in the air (it drifts by tenths of a m/s on the ground under fan vibration, which
	// matters little once airborne), the barometer's rate when it has none
	const float vz = _lpos.v_z_valid && hrt_elapsed_time(&_lpos.timestamp) < 200_ms ? -_lpos.vz : _baro_vz;
	const float eh = _auto_h_t - h;
	const float ev = _auto_vz_t - vz;
	const float thr_max = _param_nl_auto_max.get();
	const float ki = _param_nl_auto_ki.get();
	const float thr = ff + _param_nl_auto_kp.get() * eh + _param_nl_auto_kv.get() * ev + ki * _auto_int;

	// the integral covers the hover-throttle error only: at most +-0.2 of throttle
	if (!((thr >= thr_max && eh > 0.f) || (thr <= 0.f && eh < 0.f))) {
		const float lim = ki > 1e-6f ? 0.2f / ki : 0.f;
		_auto_int = math::constrain(_auto_int + eh * dt, -lim, lim);
	}

	// the throttle may not move faster than NL_AUTO_SLEW per second: a lagging barometer under fan wash made the
	// loop ask for 0.9 within half a second (6 Oct)
	const float step = math::max(_param_nl_auto_slew.get(), 0.01f) * dt;
	const float prev = PX4_ISFINITE(_auto_thr) ? _auto_thr : 0.f;
	_auto_thr = math::constrain(math::constrain(thr, prev - step, prev + step), 0.f, thr_max);
}

const char *NoseLift::auto_name(AutoPhase a)
{
	switch (a) {
	case AutoPhase::Off: return "off";

	case AutoPhase::Wait: return "waiting";

	case AutoPhase::Climb: return "climbing";

	case AutoPhase::Hover: return "hovering";

	case AutoPhase::HoldWait: return "asking PX4 Position";

	case AutoPhase::HoldPX4: return "PX4 Position hold";

	case AutoPhase::LandPX4: return "PX4 Land";

	case AutoPhase::Descend: return "descending";

	case AutoPhase::Cut: return "cutting";

	case AutoPhase::Done: return "done";
	}

	return "?";
}

void NoseLift::abort(Abort reason, bool controlled, hrt_abstime now)
{
	_rear_ramp = false;
	_abort_reason = reason;
	_auto = AutoPhase::Off;
	_auto_thr = NAN;

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
	out.throttle = auto_flying() ? _auto_thr : NAN;
	out.auto_phase = static_cast<uint8_t>(_auto);
	out.on_legs = _state == State::Ramping || _state == State::Holding || _state == State::Lowering
		      || (_state == State::Handover && !_ho_off_legs);
	out.legs_hold = static_cast<uint8_t>(math::constrain(static_cast<int>(_param_nl_legs_att.get()), 0, 3));
	out.legs_fade_s = _param_nl_legs_fade.get();

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
	case State::NoseHold:
		out.active = true;

		if (_state == State::Handover) {
			out.floor_mask = lift_mask;

		} else if (_state == State::NoseHold) {
			out.override_mask = lift_mask;	// the nose fans only: PX4 keeps the rear fans

		} else {
			out.override_mask = all;
		}

		if (_state == State::NoseHold) {
			// pitch before the roll/yaw split: a fan the split pushes past full hands the thrust it cannot give to
			// the others (25 Sep: M9 held at full while M10 sat at half and the nose fell, a quarter of the nose
			// fans' thrust unused)
			float t[MAX_LIFT] {};
			float excess = 0.f;

			for (int k = 0; k < _lift_count; k++) {
				t[k] = motor_thrust(math::max(_cmd * _split[k], 0.f));

				if (t[k] > 1.f) {
					excess += t[k] - 1.f;
					t[k] = 1.f;
				}
			}

			for (int k = 0; k < _lift_count; k++) {
				const float add = math::min(1.f - t[k], excess);
				t[k] += add;
				excess -= add;
				out.control[_lift_motor[k]] = t[k];
			}

		} else {
			for (int k = 0; k < _lift_count; k++) {
				out.control[_lift_motor[k]] = motor_thrust(math::constrain(_cmd * _split[k], 0.f, 1.f));
			}

			if (_state == State::Lowering && _rear_ramp) {
				const float k = (now - _rear_t0) * 1e-6f / math::max(_param_nl_low_rramp.get(), 1e-3f);

				if (k >= 1.f) {
					_rear_ramp = false;

				} else {
					const int mask = _param_nl_mot_msk.get();

					for (int i = 0; i < n; i++) {
						if (!(mask & (1 << i))) { out.control[i] = _rear_from[i] * (1.f - k); }
					}
				}
			}
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

	_attitude_sub.update(&_attitude);
	_angular_velocity_sub.update(&_angvel);
	_armed_sub.update(&_armed);
	_land_sub.update(&_land);
	_manual_sub.update(&_manual);
	_rc_sub.update(&_rc);
	_lpos_sub.update(&_lpos);
	_dist_sub.update(&_dist);
	_act_sub.update(&_act);
	_air_sub.update(&_air);
	_status_sub.update(&_status);
	_feedback_sub.update(&_feedback);

	// the clock is read after the copies: on hardware the gyro and estimator threads preempt this one, and a
	// message published between an earlier clock read and its copy is newer than now, so now - timestamp
	// wrapped to a huge age ("attitude lost" 0.4 s into the first lift on the aircraft; lockstep SITL cannot)
	const hrt_abstime now = hrt_absolute_time();
	const float dt = _last_run > 0 ? math::constrain((now - _last_run) * 1e-6f, 1e-4f, 0.05f) : 0.005f;
	_last_run = now;

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
	update_baro();

	const bool armed = _armed.armed;
	const bool kill = _armed.kill || _armed.termination;

	if (!armed && _state != State::Disarmed) {
		if (is_sequence(_state)) {
			mavlink_log_critical(&_mavlink_log_pub, "Nose lift: disarmed while %s\t", state_name(_state));
		}

		_cmd = 0.f;
		_auto = AutoPhase::Off;
		_auto_thr = NAN;
		set_state(State::Disarmed, now);
	}

	if (armed) {
		step_auto(now, dt, kill);
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
	case State::NoseHold:
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

	case State::NoseHold: return "holding the nose";
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

	case Abort::Stuck: return "nose stuck";

	case Abort::Weak: return "nose fans too weak for the takeoff";
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
