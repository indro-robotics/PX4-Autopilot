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

#include "FixedWingModeManager.hpp"

#include <px4_platform_common/events.h>
#include <string.h>
#include <uORB/topics/battery_status.h>

using SoarState = soaring::State;

static_assert(soaring_status_s::STATE_THERMAL == (uint8_t)SoarState::THERMAL, "soaring state numbering");
static_assert(soaring_status_s::REASON_CONFIG == soaring::REASON_CONFIG, "soaring reason numbering");

void
FixedWingModeManager::soar_params_update()
{
	// a table of handles is smaller in flash than one Param object per field
	static constexpr struct {
		px4::params id;
		float soaring::Params::*field;
	} kParams[] = {
		{px4::params::SOAR_ALT_MIN, &soaring::Params::alt_min},
		{px4::params::SOAR_ALT_PAT, &soaring::Params::alt_pat},
		{px4::params::SOAR_ALT_CUT, &soaring::Params::alt_cut},
		{px4::params::SOAR_ALT_MAX, &soaring::Params::alt_max},
		{px4::params::SOAR_MAX_RAD, &soaring::Params::max_rad},
		{px4::params::SOAR_VSPEED, &soaring::Params::vspeed},
		{px4::params::SOAR_THML_BANK, &soaring::Params::thml_bank},
		{px4::params::SOAR_THML_ASPD, &soaring::Params::thml_aspd},
		{px4::params::SOAR_CRSE_ASPD, &soaring::Params::crse_aspd},
		{px4::params::SOAR_CRSE_RAD, &soaring::Params::crse_rad},
		{px4::params::SOAR_MIN_THML_S, &soaring::Params::min_thml_s},
		{px4::params::SOAR_MIN_CRSE_S, &soaring::Params::min_crse_s},
		{px4::params::SOAR_VARIO_TC, &soaring::Params::vario_tc},
		{px4::params::SOAR_POLAR_CD0, &soaring::Params::cd0},
		{px4::params::SOAR_POLAR_B, &soaring::Params::b},
		{px4::params::SOAR_POLAR_K, &soaring::Params::k},
		{px4::params::SOAR_Q1, &soaring::Params::q1},
		{px4::params::SOAR_Q2, &soaring::Params::q2},
		{px4::params::SOAR_R, &soaring::Params::r},
		{px4::params::SOAR_DIST_AHEAD, &soaring::Params::dist_ahead},
		{px4::params::GF_MAX_VER_DIST, &soaring::Params::gf_max_ver},
		{px4::params::GF_MAX_HOR_DIST, &soaring::Params::gf_max_hor},
	};

	for (const auto &e : kParams) {
		param_set_used(param_handle(e.id));
		param_get(param_handle(e.id), &(_soar_p.*e.field));
	}

	int32_t enabled = 0;
	param_set_used(param_handle(px4::params::SOAR_EN));
	param_get(param_handle(px4::params::SOAR_EN), &enabled);
	_soar_p.enabled = enabled == 1;
	param_get(param_handle(px4::params::COM_ACT_FAIL_ACT), &_soar_act_fail_act);
	_soar_p.thml_bank = math::radians(math::min(_soar_p.thml_bank, _param_fw_r_lim.get() - 15.f));
	_soar_p.airspd_min = _param_fw_airspd_min.get();
	_soar_p.airspd_max = _param_fw_airspd_max.get();
	_soar_p.nav_loiter_rad = _param_nav_loiter_rad.get();
}

void
FixedWingModeManager::soar_update(const hrt_abstime now)
{
	_home_position_sub.update(&_home);

	failsafe_flags_s failsafe_flags;

	if (_failsafe_flags_sub.update(&failsafe_flags)) {
		_battery_warning = failsafe_flags.battery_warning;
	}

	const position_setpoint_s &cur = _pos_sp_triplet.current;
	const bool hold = _vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER
			  && _control_mode_current == FW_POSCTRL_MODE_AUTO
			  && (cur.type == position_setpoint_s::SETPOINT_TYPE_LOITER || cur.type == position_setpoint_s::SETPOINT_TYPE_POSITION)
			  && _position_setpoint_current_valid && _global_local_proj_ref.isInitialized()
			  && !_landed && _vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING;

	soaring::Input in;
	in.time_us = now;

	if (!hold) {
		in.inhibit_reason = soaring::REASON_MODE;

	} else if (_soar_act_fail_act != 3) {
		// a failed motor restart needs the motor failure action Return
		in.inhibit_reason = soaring::REASON_CONFIG;

	} else if (_vehicle_status.failsafe) {
		in.inhibit_reason = soaring::REASON_FAILSAFE;

	} else if (_vehicle_status.failure_detector_status & vehicle_status_s::FAILURE_MOTOR) {
		in.inhibit_reason = soaring::REASON_MOTOR;

	} else if (_battery_warning >= battery_status_s::WARNING_LOW) {
		in.inhibit_reason = soaring::REASON_BATTERY;

	} else if (hrt_elapsed_time(&_time_soar_airspeed) > 1_s) {
		in.inhibit_reason = soaring::REASON_AIRSPEED;

	} else if (!_home.valid_alt || !_home.valid_hpos || !_home.valid_lpos) {
		in.inhibit_reason = soaring::REASON_HOME;

	} else {
		in.inhibit_reason = soaring::REASON_NONE;
	}

	in.armed = _control_mode.flag_armed;
	in.allowed = in.inhibit_reason == soaring::REASON_NONE;
	in.operator_cmd = _soar_operator_cmd;
	_soar_operator_cmd = false;
	in.x = _local_pos.x;
	in.y = _local_pos.y;
	in.vn = _local_pos.vx;
	in.ve = _local_pos.vy;
	in.vz = _local_pos.vz;
	in.height = _current_altitude - _home.alt;
	in.cas = _airspeed_eas;
	in.tas = _airspeed_tas;
	in.roll = _roll;
	in.yaw = _yaw;
	in.accel_x = _body_acceleration_x;
	in.wind_n = _wind_vel(0);
	in.wind_e = _wind_vel(1);
	in.wind_valid = _wind_valid;
	in.home_x = _home.x;
	in.home_y = _home.y;

	if (hold) {
		const Vector2f hold_centre = _global_local_proj_ref.project(cur.lat, cur.lon);
		in.hold_x = hold_centre(0);
		in.hold_y = hold_centre(1);
	}

	in.nav_radius = fabsf(cur.loiter_radius) > FLT_EPSILON ? fabsf(cur.loiter_radius) : fabsf(_param_nav_loiter_rad.get());

	const SoarState prev_state = _soar_out.state;
	const uint8_t prev_reason = _soar_out.reason;
	_soaring.update(in, _soar_p, _soar_out);
	soar_events(prev_state, prev_reason, in.height);

	const soaring::Output &o = _soar_out;
	const bool changed = o.state != prev_state || o.reason != prev_reason;

	if (changed || now - _time_soar_status >= 200_ms) {
		_time_soar_status = now;
		soaring_status_s status{};
		status.timestamp = hrt_absolute_time();
		status.state = (uint8_t)o.state;
		status.reason = o.reason;
		status.vario_te = o.vario_te;
		status.vario_te_imu = o.vario_te_imu;
		status.vario_netto = o.vario_netto;
		status.vario_netto_slow = o.vario_netto_slow;
		status.sink_polar = o.sink_polar;
		status.climb_expected = o.climb_expected;
		status.climb_avg = o.climb_avg;
		status.thermal_w = o.thermal_w;
		status.thermal_r = o.thermal_r;
		status.thermal_x = o.thermal_x;
		status.thermal_y = o.thermal_y;

		for (int i = 0; i < 4; i++) {
			status.thermal_var[i] = o.thermal_var[i];
		}

		status.loiter_radius = o.state >= SoarState::CLIMB ? o.loiter_radius : NAN;
		status.alt_rel = in.height;
		status.alt_min = o.alt_min;
		status.alt_floor = o.alt_floor;
		status.alt_cut = o.alt_cut;
		status.alt_max = o.alt_max;
		status.dist_home = o.dist_home;
		status.aspd_excess = o.aspd_excess;
		status.thermal_count = o.thermal_count;
		status.height_gained = o.height_gained;
		_soaring_status_pub.publish(status);
	}

	if (o.state != prev_state || now - _time_soar_debug >= 1_s) {
		_time_soar_debug = now;
		debug_key_value_s debug{};
		debug.timestamp = hrt_absolute_time();
		strncpy(debug.key, "SOAR_ST", sizeof(debug.key));
		debug.value = (float)o.state;
		_debug_key_value_pub.publish(debug);
	}
}

void
FixedWingModeManager::soar_events(const SoarState prev_state, const uint8_t prev_reason, const float height)
{
	const soaring::Output &o = _soar_out;
	const bool new_inhibit = o.state == SoarState::INHIBIT && (prev_state != SoarState::INHIBIT || prev_reason != o.reason);

	if (new_inhibit && o.reason == soaring::REASON_CONFIG) {
		events::send<int32_t>(events::ID("soaring_refused_config"), events::Log::Warning,
				      "soaring refused - COM_ACT_FAIL_ACT {1}, expected 3", _soar_act_fail_act);
		return;
	}

	if (new_inhibit && o.reason == soaring::REASON_AIRSPEED && o.aspd_excess > 2.5f) {
		events::send<float, float>(events::ID("soaring_off_airspeed_turn"), events::Log::Warning,
					   "soaring off  airspeed {1:.1} m/s, turn estimate {2:.1}", o.aspd_turn + o.aspd_excess, o.aspd_turn);
		return;
	}

	if (new_inhibit && o.reason == soaring::REASON_BAND) {
		if (o.alt_max < o.alt_cut + 20.f) {
			events::send<float, float>(events::ID("soaring_refused_ceiling"), events::Log::Warning,
						   "soaring refused - ceiling {1:.0} m under SOAR_ALT_CUT {2:.0} m. Raise GF_MAX_VER_DIST", o.alt_max, o.alt_cut);

		} else {
			events::send<float, float>(events::ID("soaring_refused_band"), events::Log::Warning,
						   "soaring refused - SOAR_ALT_CUT {1:.0} m under SOAR_ALT_MIN {2:.0} m plus 30 m", o.alt_cut, o.alt_min);
		}

		return;
	}

	if (o.state == prev_state) {
		return;
	}

	const bool was_on = prev_state >= SoarState::CLIMB;
	const bool on = o.state >= SoarState::CLIMB;

	if (prev_state == SoarState::THERMAL && on) {
		switch (o.reason) {
		case soaring::REASON_FLOOR:
			events::send<float, float>(events::ID("soaring_thermal_left_floor"), events::Log::Info,
						   "soaring thermal left  floor, gained {1:.0} m in {2:.0} s", o.last_gain, o.last_time);
			break;

		case soaring::REASON_WEAK_LIFT:
			events::send<float, float>(events::ID("soaring_thermal_left_weak_lift"), events::Log::Info,
						   "soaring thermal left  weak lift, gained {1:.0} m in {2:.0} s", o.last_gain, o.last_time);
			break;

		case soaring::REASON_CEILING:
			events::send<float, float>(events::ID("soaring_thermal_left_ceiling"), events::Log::Info,
						   "soaring thermal left  ceiling, gained {1:.0} m in {2:.0} s", o.last_gain, o.last_time);
			break;

		case soaring::REASON_RADIUS:
			events::send<float, float>(events::ID("soaring_thermal_left_radius"), events::Log::Info,
						   "soaring thermal left  radius, gained {1:.0} m in {2:.0} s", o.last_gain, o.last_time);
			break;

		case soaring::REASON_ESTIMATOR:
			events::send<float, float>(events::ID("soaring_thermal_left_estimator"), events::Log::Info,
						   "soaring thermal left  estimator, gained {1:.0} m in {2:.0} s", o.last_gain, o.last_time);
			break;

		case soaring::REASON_AIRSPEED_LOW:
			events::send<float, float>(events::ID("soaring_thermal_left_airspeed_low"), events::Log::Info,
						   "soaring thermal left  airspeed low, gained {1:.0} m in {2:.0} s", o.last_gain, o.last_time);
			break;
		}
	}

	if (!was_on && on) {
		events::send<float, float, float>(events::ID("soaring_on"), events::Log::Info,
						  "soaring on  band {1:.0} to {2:.0} m, ceiling {3:.0} m", o.alt_min, o.alt_cut, o.alt_max);

	} else if (was_on && !on) {
		switch (o.reason) {
		case soaring::REASON_MODE:
			events::send(events::ID("soaring_off_mode"), events::Log::Info, "soaring off  mode");
			break;

		case soaring::REASON_FAILSAFE:
			events::send(events::ID("soaring_off_failsafe"), events::Log::Info, "soaring off  failsafe");
			break;

		case soaring::REASON_AIRSPEED:
			events::send(events::ID("soaring_off_airspeed"), events::Log::Info, "soaring off  airspeed");
			break;

		case soaring::REASON_BATTERY:
			events::send(events::ID("soaring_off_battery"), events::Log::Info, "soaring off  battery");
			break;

		case soaring::REASON_HOME:
			events::send(events::ID("soaring_off_home"), events::Log::Info, "soaring off  home");
			break;

		case soaring::REASON_RADIUS:
			events::send(events::ID("soaring_off_radius"), events::Log::Info, "soaring off  radius");
			break;

		case soaring::REASON_OPERATOR:
			events::send(events::ID("soaring_off_operator"), events::Log::Warning,
				     "soaring off  operator command. SOAR_EN 0 then 1 restarts");
			break;

		case soaring::REASON_MOTOR:
			events::send(events::ID("soaring_off_motor"), events::Log::Warning, "soaring off  motor failure");
			break;

		case soaring::REASON_WIND:
			events::send(events::ID("soaring_off_wind"), events::Log::Warning, "soaring off  wind");
			break;

		default:
			events::send(events::ID("soaring_off_disabled"), events::Log::Info, "soaring off  SOAR_EN 0");
			break;
		}

	} else if (prev_state == SoarState::CLIMB && o.state == SoarState::GLIDE) {
		events::send<float>(events::ID("soaring_motor_off"), events::Log::Info, "soaring motor off  {1:.0} m", height);

	} else if (o.state == SoarState::CLIMB && o.reason == soaring::REASON_FLOOR) {
		events::send<float>(events::ID("soaring_motor_on_floor"), events::Log::Info, "soaring motor on  floor {1:.0} m", o.alt_floor);

	} else if (o.state == SoarState::CLIMB && o.reason == soaring::REASON_AIRSPEED_LOW) {
		events::send<float, float>(events::ID("soaring_motor_on_airspeed"), events::Log::Warning,
					   "soaring motor on  airspeed {1:.1} m/s, expected {2:.1}", _airspeed_eas, o.cas_min + 2.f);

	} else if (o.state == SoarState::THERMAL) {
		events::send<float, float>(events::ID("soaring_thermal"), events::Log::Info,
					   "soaring thermal  climb {1:.1} m/s at {2:.0} m", o.climb_expected, height);
	}
}

void
FixedWingModeManager::soar_apply(position_setpoint_s &current_sp)
{
	if (_soar_out.state == SoarState::CLIMB) {
		current_sp.alt = _home.alt + _soar_out.alt_sp;

	} else if (_soar_out.state == SoarState::GLIDE) {
		current_sp.alt = _current_altitude;
		current_sp.gliding_enabled = true;

	} else {
		return;
	}

	current_sp.cruising_speed = _soar_out.airspeed_sp;
	current_sp.loiter_radius = _soar_out.loiter_radius;
}

void
FixedWingModeManager::control_soar_thermal(const Vector2f &ground_speed)
{
	const Vector2f centre{_soar_out.centre_x, _soar_out.centre_y};
	const Vector2f position{_local_pos.x, _local_pos.y};
	const DirectionalGuidanceOutput sp = navigateLoiter(centre, position, _soar_out.loiter_radius, _soar_out.loiter_ccw,
					     ground_speed - _wind_vel, Vector2f{0.f, 0.f});

	const fixed_wing_lateral_setpoint_s lateral_ctrl_sp = {
		.timestamp = hrt_absolute_time(),
		.course = NAN,
		.airspeed_direction = sp.course_setpoint,
		.lateral_acceleration = sp.lateral_acceleration_feedforward
	};
	_lateral_ctrl_sp_pub.publish(lateral_ctrl_sp);

	const fixed_wing_longitudinal_setpoint_s longitudinal_ctrl_sp = {
		.timestamp = hrt_absolute_time(),
		.altitude = _current_altitude,
		.height_rate = NAN,
		.equivalent_airspeed = _soar_out.airspeed_sp,
		.pitch_direct = NAN,
		.throttle_direct = NAN
	};
	_longitudinal_ctrl_sp_pub.publish(longitudinal_ctrl_sp);

	_ctrl_configuration_handler.setThrottleMin(0.f);
	_ctrl_configuration_handler.setThrottleMax(0.f);
	_ctrl_configuration_handler.setSpeedWeight(2.f);

	position_setpoint_s orbit = _pos_sp_triplet.current;
	_global_local_proj_ref.reproject(centre(0), centre(1), orbit.lat, orbit.lon);
	orbit.alt = _current_altitude;
	orbit.loiter_radius = _soar_out.loiter_radius;
	orbit.loiter_direction_counter_clockwise = _soar_out.loiter_ccw;
	publishOrbitStatus(orbit);
	publishLocalPositionSetpoint(orbit);
}
