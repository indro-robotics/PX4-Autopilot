/*
*  ArduSoar support function
*
*  Peter Braswell, Samuel Tabor, Andrey Kolobov, and Iain Guilliard
*/

// Source: ArduPilot libraries/AP_Soaring/AP_Soaring.cpp, ArduPlane/soaring.cpp and ArduPlane/mode_thermal.cpp, commit 74f0f83.
// Licence: GPLv3 or later, see COPYING.

#include "SoaringController.hpp"

#include <mathlib/math/Limits.hpp>
#include <px4_platform_common/defines.h>

namespace soaring
{

static constexpr float GRAVITY_MSS = 9.80665f;
static constexpr float kAspdExcess = 2.5f;	// [m/s] pitot TAS above the GPS turn estimate that ends soaring
// a single reversed yaw sample in turbulence must not restart the turn
static constexpr float kTurnReverse = 30.f * M_PI_F / 180.f;
static constexpr float kCutBand = 50.f;	// [m] under SOAR_ALT_CUTOFF where CLIMB flies FW_AIRSPD_TRIM
static constexpr float kCentreSpeed = 5.f;	// [m/s] commanded centre closing on the estimate

static float lpf(float y, float x, float dt, float tc)
{
	return y + (x - y) * dt / (tc + dt);
}

static float finite(float v)
{
	return PX4_ISFINITE(v) ? v : 0.f;
}

// hypotf links the double hypot from newlib
static float norm(float x, float y)
{
	return sqrtf(x * x + y * y);
}

SoaringController::SoaringController() :
	_vario(_polarParams),
	_speedToFly(_polarParams)
{
}

matrix::Vector2f SoaringController::drift_step(const matrix::Vector2f &wind, float dt, float climb, float strength,
		float drift)
{
	// climb over strength held to 0 to 1: a strength near 0 moves the estimate metres per step, and 0 gives NaN
	const float lean = climb <= 0.f ? 0.f : (climb >= strength ? 1.f : climb / strength);
	// param set does not enforce the yaml range
	const float k = math::constrain(drift, 0.f, 1.f);
	// a base carried at k times the wind leaves the rest of the wind to lean the column
	return wind * (dt * (lean * (1.f - k) + k));
}

bool SoaringController::check_thermal_criteria(const Params &p, float alt_floor, float alt_max) const
{
	return (_state_s > p.min_crse_s
		&& (_vario.get_trigger_value() - _vario.get_exp_thermalling_sink()) > p.vspeed
		&& _vario.alt < alt_max
		&& _vario.alt > alt_floor);
}

void SoaringController::init_thermalling(const Input &in, const Params &p)
{
	// Calc filter matrices - so that changes to parameters can be updated by switching in and out of thermal mode
	float r      = powf(p.r,  2); // Measurement noise
	float cov_q1 = powf(p.q1, 2); // Process noise for strength
	float cov_q2 = powf(p.q2, 2); // Process noise for position and radius

	const float init_q[4] = {cov_q1,
				 cov_q2,
				 cov_q2,
				 cov_q2
				};

	const matrix::SquareMatrix<float, 4> q = matrix::diag(matrix::Vector<float, 4> {init_q});

	const float init_p[4] = {INITIAL_STRENGTH_COVARIANCE,
				 INITIAL_RADIUS_COVARIANCE,
				 INITIAL_POSITION_COVARIANCE,
				 INITIAL_POSITION_COVARIANCE
				};

	const matrix::SquareMatrix<float, 4> p_init = matrix::diag(matrix::Vector<float, 4> {init_p});

	// New state vector filter will be reset. Thermal location is placed in front of a/c
	const float init_xr[4] = {_vario.get_trigger_value(),
				  INITIAL_THERMAL_RADIUS,
				  in.x + p.dist_ahead * cosf(in.yaw),
				  in.y + p.dist_ahead * sinf(in.yaw)
				 };

	const matrix::Vector<float, 4> xr{init_xr};

	// Also reset covariance matrix p so filter is not affected by previous data
	_ekf.reset(xr, p_init, q, r);

	_thermal_start_pos = matrix::Vector3f{in.x, in.y, -in.height};

	_vario.reset_climb_filter(0.0f);

	_cx = _ekf.X(2);
	_cy = _ekf.X(3);
	_fault = false;
	_ccw = p.nav_loiter_rad < 0.f;
	_thermal_count++;
	setState(State::THERMAL, REASON_NONE, in.height);
}

void SoaringController::update_thermalling(const Input &in, const Params &p, float dt, float radius)
{
	const matrix::Vector2f wind{in.wind_n, in.wind_e};
	const matrix::Vector2f wind_drift = drift_step(wind, dt, _vario.get_filtered_climb(), _ekf.X(0), p.drift);

	// update the filter
	_ekf.update(_vario.reading, in.x, in.y, wind_drift(0), wind_drift(1));

	_thermalability = (_ekf.X(0) * expf(-powf(radius / _ekf.X(1), 2))) - _vario.get_exp_thermalling_sink();

	for (int i = 0; i < 4; i++) {
		if (!PX4_ISFINITE(_ekf.X(i)) || !PX4_ISFINITE(_ekf.P(i, i))) {
			_fault = true;
		}
	}

	// the circle centre moves by the estimate's own drift step: a 6.8 s smoothing trails a 6 m/s carried centre by 41 m
	_cx += wind_drift(0);
	_cy += wind_drift(1);
	const float ex = _ekf.X(2) - _cx;
	const float ey = _ekf.X(3) - _cy;
	const float d = norm(ex, ey);
	const float step = kCentreSpeed * dt;
	const float f = d > step ? step / d : 1.f;

	if (PX4_ISFINITE(f)) {
		_cx += ex * f;
		_cy += ey * f;
	}
}

void SoaringController::update_cruising(const Input &in, const Params &p)
{
	// Calculate the optimal airspeed for the current conditions of wind along current direction,
	// expected lift in next thermal and filtered sink rate.

	const float wx = in.wind_n * cosf(in.yaw) + in.wind_e * sinf(in.yaw);

	const float wz = _vario.get_stf_value();

	// Constraints on the airspeed calculation.
	const float CLmin = _polarParams.K / (p.airspd_max * p.airspd_max);
	const float CLmax = _polarParams.K / (p.airspd_min * p.airspd_min);

	// Update the calculation.
	_speedToFly.update(wx, wz, p.vspeed, CLmin, CLmax);
}

bool SoaringController::check_drift(const Params &p) const
{
	// Check for -1 (disabled)
	if (p.max_drift < 0) {
		return false;
	}

	// Check against the estimated thermal.
	const matrix::Vector2f position(_ekf.X(2), _ekf.X(3));

	const matrix::Vector2f start_pos(_thermal_start_pos(0), _thermal_start_pos(1));

	// Simple check of distance from initial start point.
	return (position - start_pos).length() > p.max_drift;
}

float SoaringController::get_cruising_target_airspeed(const Params &p, float cruise_cas)
{
	if (p.crse_arspd < 0) {
		// an unsolved speed to fly, NaN included, reads -1
		const float stf = _speedToFly.speed_to_fly();
		return stf > 0.f ? stf : cruise_cas;
	}

	return cruise_cas;
}

void SoaringController::setState(State state, uint8_t reason, float height)
{
	if (state != _state) {
		if (_state == State::THERMAL) {
			_last_gain = height + _thermal_start_pos(2);
			_last_time = _state_s;
			_height_gained += _last_gain;
		}

		_state = state;
		_state_s = 0.f;
	}

	_reason = reason;
}

bool SoaringController::turnCheck(const Input &in, float tas)
{
	bool evaluated = false;

	// over a full turn of the air velocity the spread of the ground velocity about its mean is the TAS, whatever the wind
	if (fabsf(_turn_net) >= M_TWOPI_F) {
		const float mn = _turn_sum[2] / _turn_sum[0];
		const float me = _turn_sum[3] / _turn_sum[0];
		_aspd_turn = sqrtf(fmaxf(_turn_sum[4] / _turn_sum[0] - mn * mn - me * me, 0.f));
		_aspd_excess = _turn_sum[1] / _turn_sum[0] - _aspd_turn;
		_turn_ok = fabsf(_aspd_excess) <= kAspdExcess;
		evaluated = true;
	}

	// heading from yaw, not g tan(roll) / TAS: an over-reading pitot would stretch the turn
	float d = in.yaw - _yaw_prev;
	d = d > M_PI_F ? d - M_TWOPI_F : (d < -M_PI_F ? d + M_TWOPI_F : d);
	_yaw_prev = in.yaw;
	const bool step = !PX4_ISFINITE(d) || fabsf(d) > 0.5f;

	// a reversal restarts the turn since S-turns cover headings unevenly; a step is an estimator yaw reset
	if (evaluated || !in.armed || step || _turn_peak - fabsf(_turn_net + d) >= kTurnReverse) {
		_turn_net = 0.f;
		_turn_peak = 0.f;

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
	_turn_peak = fmaxf(_turn_peak, fabsf(_turn_net));
	_turn_sum[0] += w;
	_turn_sum[1] += w * tas;
	_turn_sum[2] += w * vn;
	_turn_sum[3] += w * ve;
	_turn_sum[4] += w * (vn * vn + ve * ve);
	return evaluated;
}

void SoaringController::update(const Input &in, const Params &p, Output &out)
{
	const float dt = (_last_us > 0 && in.time_us > _last_us) ? fminf((in.time_us - _last_us) * 1e-6f, 0.1f) : 0.f;
	// an external mode runs no update, and the Hold after it is a mode change like any other
	const bool gap = _last_us > 0 && in.time_us > _last_us + 1000000;
	_last_us = in.time_us;

	// a local position reset moves the frame under the estimate, the circle and the entry point
	if (in.xy_reset_counter != _xy_reset_counter) {
		_xy_reset_counter = in.xy_reset_counter;
		_ekf.X(2) += in.delta_x;
		_ekf.X(3) += in.delta_y;
		_thermal_start_pos(0) += in.delta_x;
		_thermal_start_pos(1) += in.delta_y;
		_cx += in.delta_x;
		_cy += in.delta_y;
	}

	_polarParams = {p.polar_k, p.polar_cd0, p.polar_b};

	const float cas = finite(in.cas);
	const float tas = finite(in.tas);
	const float roll = math::constrain(finite(in.roll), math::radians(-60.f), math::radians(60.f));
	const float ratio = (cas > 5.f && tas > 5.f) ? tas / cas : 1.f;
	// ArduPilot's range: a bank of 0 gives an infinite radius and time constant, which stops every filter
	const float bank_deg = math::constrain(p.thml_bank, 20.f, 50.f);
	const float bank = math::radians(bank_deg);
	const float vth = p.thml_arspd > 0.f ? p.thml_arspd : p.airspd_min / sqrtf(cosf(bank));

	// one non-finite sample stays in the bias filter for the rest of the flight
	const bool cas_ok = in.cas_valid && PX4_ISFINITE(in.cas);
	const bool vz_ok = in.vz_valid && PX4_ISFINITE(in.vz);
	const Variometer::Input vario_in{in.time_us, in.height, cas, cas_ok, p.airspd_trim, finite(in.accel_x), in.vz, vz_ok, roll};
	_vario.update(vario_in, bank_deg, vth);

	if (dt > 0.f) {
		_climb_f = lpf(_climb_f, -finite(in.vz), dt, 0.5f);
	}

	// Thermalling radius is controlled by parameter SOAR_THML_BANK and true target airspeed.
	const float radius = (vth * ratio) * (vth * ratio) / (GRAVITY_MSS * tanf(bank));
	const float sink_now = _vario.calculate_aircraft_sinkrate(roll, _vario.get_airspeed());
	const float climb_expected = _vario.get_trigger_value() - _vario.get_exp_thermalling_sink();

	const float alt_max = p.gf_max_ver > 0.f ? fminf(p.alt_max, p.gf_max_ver - 30.f) : p.alt_max;
	float r_soar = p.max_radius >= 0.f ? p.max_radius : INFINITY;

	if (p.gf_max_hor > 0.f) {
		r_soar = fminf(r_soar, p.gf_max_hor - 100.f);
	}

	const float dist_home = norm(in.x - in.home_x, in.y - in.home_y);
	const float hold_dist = norm(in.hold_x - in.home_x, in.hold_y - in.home_y);
	const float cruise_rad = (p.crse_rad > 0.f && hold_dist + p.crse_rad <= r_soar) ? p.crse_rad : in.nav_radius;
	const float cruise_cas = p.crse_arspd > 0.f ? p.crse_arspd : p.airspd_min;
	const float cruise_sink = _vario.calculate_aircraft_sinkrate(0.f, cruise_cas) * ratio;
	const float cas_min = p.airspd_min * sqrtf(1.f / cosf(roll)) - 2.f;

	// glide reach home at FW_AIRSPD_MIN in the wind along the track home, with a 1.3 margin
	const float wind_home = dist_home > 1.f ? (in.wind_n * (in.home_x - in.x) + in.wind_e * (in.home_y - in.y)) / dist_home : 0.f;
	const float alt_floor = fmaxf(p.alt_min, p.alt_pat + 1.3f * dist_home * _vario.calculate_aircraft_sinkrate(0.f,
				      p.airspd_min) * ratio / fmaxf(p.airspd_min * ratio + wind_home, 1.f));

	if (!in.armed || !p.enable) {
		_latch = REASON_NONE;
	}

	if (turnCheck(in, tas) && p.enable && _aspd_excess > kAspdExcess && _latch == REASON_NONE) {
		_latch = REASON_AIRSPEED;
	}

	// a verdict from before a pitot dropout says nothing about the pitot after it
	if (!in.armed || (!in.allowed && in.inhibit_reason == REASON_AIRSPEED)) {
		_turn_ok = false;
	}

	const State prev = _state;
	_state_s += dt;
	_glide_bar_s = fmaxf(_glide_bar_s - dt, 0.f);
	const bool unpowered = _state == State::GLIDE || _state == State::THERMAL;
	_aspd_low_s = (unpowered && cas < cas_min) ? _aspd_low_s + dt : 0.f;
	const bool aspd_low = _aspd_low_s >= 1.f;
	const bool active = _state >= State::CLIMB;

	uint8_t inhibit = REASON_NONE;

	if (gap) {
		inhibit = REASON_MODE;

	} else if (!in.allowed) {
		inhibit = in.inhibit_reason != REASON_NONE ? in.inhibit_reason : (uint8_t)REASON_MODE;

	} else if (!in.wind_valid) {
		// a wind flagged invalid drives a wrong drift step: with no wind estimate in 6 m/s of wind the estimate moves 1.86 m/s wrong
		inhibit = REASON_WIND;

	} else if (p.alt_cutoff < p.alt_min + 30.f || alt_max < p.alt_cutoff + 20.f) {
		inhibit = REASON_BAND;

	} else if (hold_dist + in.nav_radius > r_soar) {
		inhibit = REASON_RADIUS;
	}

	// a mode command is how Hold is selected; any other command overrides soaring that waits on an inhibit
	const bool waiting = _state == State::INHIBIT && inhibit != REASON_MODE;

	if (_latch != REASON_NONE) {
		inhibit = _latch;

	} else if ((active && in.operator_cmd) || (waiting && (in.operator_cmd & OPERATOR_NAV))) {
		inhibit = REASON_OPERATOR;
	}

	if (p.enable && (inhibit == REASON_OPERATOR || (active && (inhibit == REASON_MODE || inhibit == REASON_FAILSAFE
			 || inhibit == REASON_MOTOR || inhibit == REASON_TERRAIN)))) {
		_latch = inhibit;
	}

	if (!p.enable) {
		// THERMAL ends too: left in its orbit with the motor on, the aircraft descends toward the home height
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
			if (in.height > p.alt_cutoff && alt_floor <= p.alt_cutoff - 30.f && _glide_bar_s <= 0.f) {
				// Start glide
				setState(State::GLIDE, REASON_NONE, in.height);

				// Reset the filtered vario rate - it is currently elevated due to the climb rate and would otherwise take a while to fall again,
				// leading to false positives.
				_vario.reset_trigger_filter(0.0f);
			}

			break;

		case State::GLIDE:
			if (in.height < alt_floor) {
				// Time to throttle up
				setState(State::CLIMB, REASON_FLOOR, in.height);

			} else if (aspd_low) {
				setState(State::CLIMB, REASON_AIRSPEED_LOW, in.height);
				_glide_bar_s = 20.f;

			} else {
				// Cruising modes update.
				update_cruising(in, p);

				// Test for switch into THERMAL mode
				if (check_thermal_criteria(p, alt_floor, alt_max) && dist_home + 2.f * radius <= r_soar) {
					init_thermalling(in, p);
				}
			}

			break;

		case State::THERMAL:
			// Update thermal estimate and check for switch back to AUTO
			update_thermalling(in, p, dt, radius);

			if (in.height < alt_floor) {
				setState(State::CLIMB, REASON_FLOOR, in.height);

			} else if (aspd_low) {
				setState(State::CLIMB, REASON_AIRSPEED_LOW, in.height);
				_glide_bar_s = 20.f;

			} else if (_fault) {
				setState(State::GLIDE, REASON_ESTIMATOR, in.height);

			} else if (norm(_cx - in.home_x, _cy - in.home_y) + radius > r_soar) {
				setState(State::GLIDE, REASON_RADIUS, in.height);

			} else if (in.height + 5.f * _vario.get_filtered_climb() > alt_max) {
				setState(State::GLIDE, REASON_CEILING, in.height);

			} else if (_state_s > p.min_thml_s) {
				// McCready(alt) is SOAR_VSPEED
				if (_thermalability < p.vspeed) {
					setState(State::GLIDE, REASON_WEAK_LIFT, in.height);

				} else if (_vario.alt < (-_thermal_start_pos(2)) || _vario.get_filtered_climb() < 0.0f) {
					setState(State::GLIDE, REASON_HEIGHT_LOST, in.height);

				} else if (check_drift(p)) {
					setState(State::GLIDE, REASON_DRIFT, in.height);
				}
			}

			break;
		}
	}

	// latched so that lift met in CLIMB never carries the setpoint toward the fence
	if (_state == State::CLIMB && prev != State::CLIMB) {
		_climb_sp = fminf(fmaxf(p.alt_cutoff + 5.f, in.height), alt_max);
	}

	// the path angle change to the glide is the pitch change a cut needs: TECS alone takes 30 s to find it
	if (_state == State::GLIDE && prev != State::GLIDE && prev != State::THERMAL) {
		_cut_count++;
		// only a soaring CLIMB leaves its motor climb in the pitch integrator, and lift is no part of it
		const float climb = prev == State::CLIMB ? fminf(_climb_f, p.clmb_max) : 0.f;
		_cut_pitch_step = math::constrain(-climb / fmaxf(tas, 5.f) - cruise_sink / fmaxf(cruise_cas * ratio, 5.f), -0.5f, 0.f);
	}

	const bool thermal = _state == State::THERMAL;
	// held down to the ceiling less 20 m so the zoom on slowing (23 to 14 m/s returns 17 m) stays under it
	_speed_hold = in.height > alt_max || (_speed_hold && in.height >= alt_max - 20.f);

	out.state = _state;
	out.reason = _reason;
	out.alt_sp = _climb_sp;
	// a dead motor in CLIMB then costs height, not airspeed
	out.speed_weight = _state == State::CLIMB ? 2.f : NAN;
	// zero thrust at the cut decelerates by g sin(path angle) plus drag: trim speed flattens the path and adds speed in hand
	const bool cut_near = _state == State::CLIMB && in.height >= p.alt_cutoff - kCutBand;

	if (thermal) {
		out.airspeed_sp = vth;

	} else if (_state == State::GLIDE) {
		out.airspeed_sp = _speed_hold ? p.airspd_max : get_cruising_target_airspeed(p, cruise_cas);

	} else {
		out.airspeed_sp = cut_near ? fminf(p.airspd_trim, p.airspd_max) : cruise_cas;
	}

	out.loiter_radius = thermal ? radius : cruise_rad;
	out.loiter_ccw = _ccw;
	out.centre_x = _cx;
	out.centre_y = _cy;

	out.vario_te = _vario.reading - sink_now;
	out.vario_netto = _vario.reading;
	out.vario_netto_slow = _vario.get_trigger_value();
	out.sink_polar = sink_now;
	out.climb_expected = climb_expected;
	out.climb_avg = thermal ? _vario.get_filtered_climb() : NAN;
	out.thermal_climb = thermal ? _thermalability : NAN;
	out.thermal_w = thermal ? _ekf.X(0) : NAN;
	out.thermal_r = thermal ? _ekf.X(1) : NAN;
	out.thermal_x = thermal ? _ekf.X(2) : NAN;
	out.thermal_y = thermal ? _ekf.X(3) : NAN;

	for (int i = 0; i < 4; i++) {
		out.thermal_var[i] = thermal ? _ekf.P(i, i) : NAN;
	}

	out.alt_min = p.alt_min;
	out.alt_floor = alt_floor;
	out.alt_cut = p.alt_cutoff;
	out.alt_max = alt_max;
	out.dist_home = dist_home;
	out.cas_min = cas_min;
	out.aspd_excess = _aspd_excess;
	out.aspd_turn = _aspd_turn;
	out.thermal_count = _thermal_count;
	out.height_gained = _height_gained;
	out.last_gain = _last_gain;
	out.last_time = _last_time;
	out.cut_count = _cut_count;
	out.cut_pitch_step = _cut_pitch_step;
}

} // namespace soaring
