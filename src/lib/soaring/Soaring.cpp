/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "Soaring.hpp"

#include <cmath>
#include <math.h>

namespace soaring
{

static constexpr float kG = 9.80665f;
static constexpr float kDeg = 0.01745329f;
static constexpr float kPi = 3.14159265f;
static constexpr float kTwoPi = 6.2831853f;
static constexpr float kAspdExcess = 2.5f;	// [m/s] pitot TAS above the GPS turn estimate that ends soaring
// the restart check mirrors the FD_FW_TL_THR, FD_FW_TL_RATE and FD_FW_TL_TIME defaults: 2 s to throttle 0.7, -0.8 m/s, 2 s
static constexpr float kRestartGraceS = 2.f;
static constexpr float kRestartRate = -0.8f;
static constexpr float kRestartS = 2.f;

static float clampf(float v, float lo, float hi)
{
	return v < lo ? lo : (v > hi ? hi : v);
}

static float lpf(float y, float x, float dt, float tc)
{
	return y + (x - y) * dt / (tc + dt);
}

static float finite(float v)
{
	return std::isfinite(v) ? v : 0.f;
}

// hypotf links the double hypot from newlib
static float norm(float x, float y)
{
	return sqrtf(x * x + y * y);
}

float Soaring::sink(const Params &p, float cas, float tas_over_cas, float bank)
{
	const float v = fmaxf(cas, 5.f);
	const float c = cosf(fminf(fabsf(bank), 60.f * kDeg));
	return (p.cd0 * v * v * v / p.k + p.b * p.k / (v * c * c)) * tas_over_cas;
}

float Soaring::updraft(const float s[4])
{
	return s[0] * expf(-(s[2] * s[2] + s[3] * s[3]) / (s[1] * s[1]));
}

void Soaring::jacobian(const float s[4], float H[4])
{
	const float w = s[0];
	const float r2 = s[1] * s[1];
	const float d2 = s[2] * s[2] + s[3] * s[3];
	const float e = expf(-d2 / r2);
	H[0] = e;
	H[1] = 2.f * w * d2 * e / (r2 * s[1]);
	H[2] = -2.f * w * s[2] * e / r2;
	H[3] = -2.f * w * s[3] * e / r2;
}

void Soaring::setState(State state, uint8_t reason, float height)
{
	if (state != _state) {
		if (_state == State::THERMAL) {
			_last_gain = height - _entry_height;
			_last_time = _state_s;
			_height_gained += _last_gain;
		}

		// a motor climb leaves its energy in the entry filter
		if (state == State::GLIDE && _state != State::THERMAL) {
			_netto_slow = 0.f;
		}

		_state = state;
		_state_s = 0.f;
	}

	_reason = reason;
}

void Soaring::enterThermal(const Input &in, const Params &p, float climb_expected)
{
	const float an = in.vn - in.wind_n;
	const float ae = in.ve - in.wind_e;
	const float a = sqrtf(an * an + ae * ae);

	_s[0] = fmaxf(_netto_slow, 0.5f);
	_s[1] = 70.f;
	_s[2] = a > 1.f ? p.dist_ahead * an / a : p.dist_ahead;
	_s[3] = a > 1.f ? p.dist_ahead * ae / a : 0.f;

	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			_P[i][j] = 0.f;
		}
	}

	_P[0][0] = 1.f;
	_P[1][1] = 30.f * 30.f;
	_P[2][2] = 50.f * 50.f;
	_P[3][3] = 50.f * 50.f;

	_update_s = 0.f;
	_fault = false;
	_cx = in.x + _s[2];
	_cy = in.y + _s[3];
	_ccw = fabsf(in.roll) > 5.f * kDeg ? in.roll < 0.f : p.nav_loiter_rad < 0.f;
	_climb_avg = climb_expected;
	_entry_height = in.height;
	_thermal_count++;
	setState(State::THERMAL, REASON_NONE, in.height);
}

void Soaring::thermalStep(const Input &in, const Params &p, float dt, float netto, float te, float radius)
{
	_s[2] -= (in.vn - in.wind_n) * dt;
	_s[3] -= (in.ve - in.wind_e) * dt;
	_update_s += dt;

	if (_update_s >= 0.2f) {
		const float q1 = p.q1 * p.q1 * _update_s;
		const float q2 = p.q2 * p.q2 * _update_s;
		_update_s = 0.f;
		_P[0][0] += q1;
		_P[1][1] += q2;
		_P[2][2] += q2;
		_P[3][3] += q2;

		float H[4];
		jacobian(_s, H);
		const float innovation = netto - updraft(_s);

		float ph[4];
		float s = p.r * p.r;

		for (int i = 0; i < 4; i++) {
			ph[i] = _P[i][0] * H[0] + _P[i][1] * H[1] + _P[i][2] * H[2] + _P[i][3] * H[3];
			s += H[i] * ph[i];
		}

		for (int i = 0; i < 4; i++) {
			_s[i] += ph[i] / s * innovation;

			for (int j = 0; j < 4; j++) {
				_P[i][j] -= ph[i] * ph[j] / s;
			}
		}

		for (int i = 0; i < 4; i++) {
			for (int j = i + 1; j < 4; j++) {
				_P[i][j] = _P[j][i] = 0.5f * (_P[i][j] + _P[j][i]);
			}
		}

		_s[0] = clampf(_s[0], 0.f, 10.f);
		_s[1] = clampf(_s[1], 20.f, 200.f);
		_s[2] = clampf(_s[2], -300.f, 300.f);
		_s[3] = clampf(_s[3], -300.f, 300.f);
	}

	for (int i = 0; i < 4; i++) {
		if (!std::isfinite(_s[i]) || !std::isfinite(_P[i][i])) {
			_fault = true;
		}
	}

	// the commanded centre drifts with the air mass, then closes on the estimate
	_cx += in.wind_n * dt;
	_cy += in.wind_e * dt;
	const float ex = in.x + _s[2] - _cx;
	const float ey = in.y + _s[3] - _cy;
	const float d = sqrtf(ex * ex + ey * ey);
	const float step = 5.f * dt;
	const float f = d > step ? step / d : 1.f;

	if (std::isfinite(f)) {
		_cx += ex * f;
		_cy += ey * f;
	}

	_climb_avg = lpf(_climb_avg, te, dt, kTwoPi * radius / fmaxf(in.tas, 5.f));
}

bool Soaring::turnCheck(const Input &in, float tas, float roll)
{
	bool evaluated = false;

	// over a full turn of the air velocity the spread of the ground velocity about its mean is the TAS, whatever the wind
	if (fabsf(_turn_net) >= kTwoPi) {
		const float mn = _turn_sum[2] / _turn_sum[0];
		const float me = _turn_sum[3] / _turn_sum[0];
		_aspd_turn = sqrtf(fmaxf(_turn_sum[4] / _turn_sum[0] - mn * mn - me * me, 0.f));
		_aspd_excess = _turn_sum[1] / _turn_sum[0] - _aspd_turn;
		_turn_ok = fabsf(_aspd_excess) <= kAspdExcess;
		evaluated = true;
	}

	// heading from yaw, not g tan(roll) / TAS: an over-reading pitot would stretch the turn
	float d = in.yaw - _yaw_prev;
	d = d > kPi ? d - kTwoPi : (d < -kPi ? d + kTwoPi : d);
	_yaw_prev = in.yaw;
	const bool step = !std::isfinite(d) || fabsf(d) > 0.5f;

	// a reversal restarts the turn since S-turns cover headings unevenly; a step is an estimator yaw reset
	if (evaluated || !in.armed || step || (fabsf(roll) > 5.f * kDeg && d * _turn_net < 0.f)) {
		_turn_net = 0.f;

		for (int i = 0; i < 5; i++) {
			_turn_sum[i] = 0.f;
		}
	}

	if (step) {
		d = 0.f;
	}

	// weight by heading change, not time: a ground-fixed orbit in wind spends longer upwind
	const float w = fabsf(d);
	const float vn = finite(in.vn);
	const float ve = finite(in.ve);
	_turn_net += d;
	_turn_sum[0] += w;
	_turn_sum[1] += w * tas;
	_turn_sum[2] += w * vn;
	_turn_sum[3] += w * ve;
	_turn_sum[4] += w * (vn * vn + ve * ve);
	return evaluated;
}

void Soaring::update(const Input &in, const Params &p, Output &out)
{
	const float dt = (_last_us > 0 && in.time_us > _last_us) ? fminf((in.time_us - _last_us) * 1e-6f, 0.1f) : 0.f;
	_last_us = in.time_us;

	const float cas = finite(in.cas);
	const float tas = finite(in.tas);
	const float roll = clampf(finite(in.roll), -60.f * kDeg, 60.f * kDeg);
	const float ratio = (cas > 5.f && tas > 5.f) ? tas / cas : 1.f;

	if (dt > 0.f) {
		_climb_f = lpf(_climb_f, -finite(in.vz), dt, 0.5f);
		_tas_dot = lpf(_tas_dot, (tas - _tas_prev) / dt, dt, 0.5f);
		_accel_f = lpf(_accel_f, finite(in.accel_x), dt, 0.5f);
	}

	_tas_prev = tas;
	const float te = _climb_f + tas * _tas_dot / kG;
	const float sink_now = sink(p, cas, ratio, roll);
	const float netto = te + sink_now;

	if (dt > 0.f) {
		_netto_slow = lpf(_netto_slow, netto, dt, p.vario_tc);
	}

	const float bank = clampf(p.thml_bank, 10.f * kDeg, 60.f * kDeg);
	const float vth = p.thml_aspd > 0.f ? p.thml_aspd : p.airspd_min / sqrtf(cosf(bank));
	const float radius = clampf(vth * ratio * vth * ratio / (kG * tanf(bank)), 25.f, 150.f);
	const float climb_expected = _netto_slow - sink(p, vth, ratio, bank);

	const float alt_max = p.gf_max_ver > 0.f ? fminf(p.alt_max, p.gf_max_ver - 30.f) : p.alt_max;
	const float r_soar = p.gf_max_hor > 0.f ? fminf(p.max_rad, p.gf_max_hor - 100.f) : p.max_rad;
	const float dist_home = norm(in.x - in.home_x, in.y - in.home_y);
	const float hold_dist = norm(in.hold_x - in.home_x, in.hold_y - in.home_y);
	const float cruise_rad = (p.crse_rad > 0.f && hold_dist + p.crse_rad <= r_soar) ? p.crse_rad : in.nav_radius;
	const float cruise_cas = p.crse_aspd > 0.f ? p.crse_aspd : p.airspd_min;
	const float cas_min = p.airspd_min * sqrtf(1.f / cosf(roll)) - 2.f;

	// glide reach home at FW_AIRSPD_MIN in the wind along the track home, with a 1.3 margin
	const float wind_home = dist_home > 1.f ? (in.wind_n * (in.home_x - in.x) + in.wind_e * (in.home_y - in.y)) / dist_home : 0.f;
	const float alt_floor = fmaxf(p.alt_min, p.alt_pat + 1.3f * dist_home * sink(p, p.airspd_min, ratio, 0.f)
				  / fmaxf(p.airspd_min * ratio + wind_home, 1.f));

	if (!in.armed || !p.enabled) {
		_latch = REASON_NONE;
	}

	if (!in.armed) {
		_turn_ok = false;
	}

	if (turnCheck(in, tas, roll) && p.enabled && _aspd_excess > kAspdExcess && _latch == REASON_NONE) {
		_latch = REASON_AIRSPEED;
	}

	_state_s += dt;
	_glide_bar_s = fmaxf(_glide_bar_s - dt, 0.f);
	const bool unpowered = _state == State::GLIDE || _state == State::THERMAL;
	_aspd_low_s = (unpowered && cas < cas_min) ? _aspd_low_s + dt : 0.f;
	const bool aspd_low = _aspd_low_s >= 1.f;
	const bool active = _state >= State::CLIMB;
	_restart_low_s = (_state == State::CLIMB && _state_s >= kRestartGraceS && te < kRestartRate) ? _restart_low_s + dt : 0.f;

	uint8_t inhibit = REASON_NONE;

	if (_latch != REASON_NONE) {
		inhibit = _latch;

	} else if (in.operator_cmd && active) {
		inhibit = REASON_OPERATOR;

	} else if (_restart_low_s >= kRestartS) {
		// a restart that adds no energy ends soaring and leaves the motor failure to the stock failsafe
		inhibit = REASON_MOTOR;

	} else if (!in.allowed) {
		inhibit = in.inhibit_reason != REASON_NONE ? in.inhibit_reason : (uint8_t)REASON_MODE;

	} else if (!in.wind_valid) {
		inhibit = REASON_WIND;

	} else if (p.alt_cut < p.alt_min + 30.f || alt_max < p.alt_cut + 20.f) {
		inhibit = REASON_BAND;

	} else if (hold_dist + in.nav_radius > r_soar) {
		inhibit = REASON_RADIUS;
	}

	if (p.enabled && active && (inhibit == REASON_OPERATOR || inhibit == REASON_MODE || inhibit == REASON_FAILSAFE
				    || inhibit == REASON_MOTOR)) {
		_latch = inhibit;
	}

	if (!p.enabled) {
		setState(State::OFF, REASON_NONE, in.height);
		_good_s = 0.f;

	} else if (inhibit != REASON_NONE) {
		setState(State::INHIBIT, inhibit, in.height);
		_good_s = 0.f;

	} else {
		switch (_state) {
		case State::OFF:
		case State::INHIBIT:
			setState(State::INHIBIT, REASON_NONE, in.height);
			_good_s += dt;

			if (_good_s >= 2.f && _turn_ok) {
				setState(in.height > alt_floor ? State::GLIDE : State::CLIMB, REASON_NONE, in.height);
			}

			break;

		case State::CLIMB:
			if (in.height >= p.alt_cut && alt_floor <= p.alt_cut - 30.f && _glide_bar_s <= 0.f) {
				setState(State::GLIDE, REASON_NONE, in.height);
			}

			break;

		case State::GLIDE:
			if (in.height <= alt_floor) {
				setState(State::CLIMB, REASON_FLOOR, in.height);

			} else if (aspd_low) {
				setState(State::CLIMB, REASON_AIRSPEED_LOW, in.height);
				_glide_bar_s = 20.f;

			} else if (climb_expected >= p.vspeed && _state_s >= p.min_crse_s
				   && in.height >= alt_floor + 20.f && in.height <= alt_max - 30.f
				   && dist_home + 2.f * radius <= r_soar) {
				enterThermal(in, p, climb_expected);
			}

			break;

		case State::THERMAL:
			thermalStep(in, p, dt, netto, te, radius);

			if (in.height <= alt_floor) {
				setState(State::CLIMB, REASON_FLOOR, in.height);

			} else if (aspd_low) {
				setState(State::CLIMB, REASON_AIRSPEED_LOW, in.height);
				_glide_bar_s = 20.f;

			} else if (_fault) {
				setState(State::GLIDE, REASON_ESTIMATOR, in.height);

			} else if (in.height + 5.f * te >= alt_max) {
				setState(State::GLIDE, REASON_CEILING, in.height);

			} else if (norm(_cx - in.home_x, _cy - in.home_y) + radius > r_soar) {
				setState(State::GLIDE, REASON_RADIUS, in.height);

			} else if (_state_s >= p.min_thml_s && _climb_avg < p.vspeed) {
				setState(State::GLIDE, REASON_WEAK_LIFT, in.height);
			}

			break;
		}
	}

	const bool thermal = _state == State::THERMAL;
	// held down to the ceiling less 20 m so the zoom on slowing (23 to 14 m/s returns 17 m) stays under it
	_speed_hold = in.height > alt_max || (_speed_hold && in.height >= alt_max - 20.f);

	out.state = _state;
	out.reason = _reason;
	out.alt_sp = fmaxf(p.alt_cut + 5.f, in.height);
	out.airspeed_sp = thermal ? vth : ((_state == State::GLIDE && _speed_hold) ? p.airspd_max : cruise_cas);
	out.loiter_radius = thermal ? radius : cruise_rad;
	out.loiter_ccw = _ccw;
	out.centre_x = _cx;
	out.centre_y = _cy;

	out.vario_te = te;
	out.vario_te_imu = _climb_f + tas * _accel_f / kG;
	out.vario_netto = netto;
	out.vario_netto_slow = _netto_slow;
	out.sink_polar = sink_now;
	out.climb_expected = climb_expected;
	out.climb_avg = thermal ? _climb_avg : NAN;
	out.thermal_w = thermal ? _s[0] : NAN;
	out.thermal_r = thermal ? _s[1] : NAN;
	out.thermal_x = thermal ? in.x + _s[2] : NAN;
	out.thermal_y = thermal ? in.y + _s[3] : NAN;

	for (int i = 0; i < 4; i++) {
		out.thermal_var[i] = thermal ? _P[i][i] : NAN;
	}

	out.alt_min = p.alt_min;
	out.alt_floor = alt_floor;
	out.alt_cut = p.alt_cut;
	out.alt_max = alt_max;
	out.dist_home = dist_home;
	out.cas_min = cas_min;
	out.aspd_excess = _aspd_excess;
	out.aspd_turn = _aspd_turn;
	out.thermal_count = _thermal_count;
	out.height_gained = _height_gained;
	out.last_gain = _last_gain;
	out.last_time = _last_time;
}

} // namespace soaring
