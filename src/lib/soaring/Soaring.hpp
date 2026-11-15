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

/**
 * @file Soaring.hpp
 * Thermal soaring in Hold: total-energy variometer, thermal estimator and state machine.
 */

#pragma once

#include <math.h>
#include <stdint.h>

namespace soaring
{

enum class State : uint8_t {
	OFF = 0,
	INHIBIT,
	CLIMB,
	GLIDE,
	THERMAL
};

enum Reason : uint8_t {
	REASON_NONE = 0,
	REASON_MODE,
	REASON_FAILSAFE,
	REASON_AIRSPEED,
	REASON_BATTERY,
	REASON_HOME,
	REASON_BAND,
	REASON_RADIUS,
	REASON_FLOOR,
	REASON_WEAK_LIFT,
	REASON_CEILING,
	REASON_ESTIMATOR = 12,		// 11 stays unused so every logged reason keeps one meaning
	REASON_AIRSPEED_LOW,
	REASON_OPERATOR,
	REASON_MOTOR,
	REASON_WIND,
	REASON_CONFIG,
	REASON_TERRAIN
};

enum Operator : uint8_t {
	OPERATOR_MODE = 1,	// DO_SET_MODE
	OPERATOR_NAV = 2	// DO_REPOSITION, DO_CHANGE_ALTITUDE, DO_ORBIT, DO_CHANGE_SPEED
};

struct Params {
	bool enabled{false};
	float alt_min{100.f};		// [m] above home
	float alt_pat{80.f};		// [m] height to arrive home with after the glide back
	float alt_cut{250.f};		// [m] above home
	float alt_max{400.f};		// [m] above home
	float max_rad{350.f};		// [m]
	float vspeed{0.5f};		// [m/s]
	float thml_bank{0.6109f};	// [rad]
	float thml_aspd{0.f};		// [m/s] CAS, 0 uses airspd_min / sqrt(cos bank)
	float crse_aspd{0.f};		// [m/s] CAS, 0 uses airspd_min
	float crse_rad{150.f};		// [m]
	float min_thml_s{30.f};		// [s]
	float min_crse_s{30.f};		// [s]
	float vario_tc{2.f};		// [s]
	float cd0{0.0297f};
	float b{0.0702f};
	float k{80.2f};			// [m2/s2]
	float q1{0.02f};		// [m/s per sqrt(s)]
	float q2{3.f};			// [m per sqrt(s)]
	float r{0.5f};			// [m/s]
	float dist_ahead{15.f};		// [m]
	float airspd_min{14.f};		// [m/s] FW_AIRSPD_MIN
	float airspd_trim{18.f};	// [m/s] FW_AIRSPD_TRIM
	float airspd_max{23.f};		// [m/s] FW_AIRSPD_MAX
	float gf_max_ver{0.f};		// [m] GF_MAX_VER_DIST, 0 off
	float gf_max_hor{0.f};		// [m] GF_MAX_HOR_DIST, 0 off
	float nav_loiter_rad{60.f};	// [m] NAV_LOITER_RAD, negative counter-clockwise
};

struct Input {
	uint64_t time_us{0};
	bool armed{false};
	bool allowed{false};
	uint8_t operator_cmd{0};	// Operator bits seen since the last update
	uint8_t inhibit_reason{REASON_MODE};
	float x{0.f};			// [m] local north
	float y{0.f};			// [m] local east
	float vn{0.f};			// [m/s] ground velocity north
	float ve{0.f};			// [m/s] ground velocity east
	float vz{0.f};			// [m/s] down
	float height{0.f};		// [m] above home
	float cas{0.f};			// [m/s]
	float tas{0.f};			// [m/s]
	float roll{0.f};		// [rad]
	float yaw{0.f};			// [rad]
	float accel_x{0.f};		// [m/s2] kinematic acceleration along the body x axis
	float wind_n{0.f};		// [m/s]
	float wind_e{0.f};		// [m/s]
	bool wind_valid{false};
	float home_x{0.f};		// [m] local north
	float home_y{0.f};		// [m] local east
	float hold_x{0.f};		// [m] Hold centre, local north
	float hold_y{0.f};		// [m] Hold centre, local east
	float nav_radius{60.f};		// [m] Hold radius
};

struct Output {
	State state{State::OFF};
	uint8_t reason{REASON_NONE};

	float alt_sp{0.f};		// [m] above home, CLIMB
	float speed_weight{NAN};	// TECS speed weight, NAN keeps the stock value
	float airspeed_sp{0.f};		// [m/s] CAS
	float loiter_radius{0.f};	// [m]
	bool loiter_ccw{false};		// THERMAL
	float centre_x{0.f};		// [m] commanded THERMAL centre, local north
	float centre_y{0.f};		// [m] commanded THERMAL centre, local east

	float vario_te{0.f};
	float vario_te_imu{0.f};
	float vario_netto{0.f};
	float vario_netto_slow{0.f};
	float sink_polar{0.f};
	float climb_expected{0.f};
	float climb_avg{0.f};
	float thermal_w{0.f};
	float thermal_r{0.f};
	float thermal_x{0.f};		// [m] estimated centre, local north
	float thermal_y{0.f};		// [m] estimated centre, local east
	float thermal_var[4] {};
	float alt_min{0.f};
	float alt_floor{0.f};		// [m] motor restart and lowest thermalling height
	float alt_cut{0.f};
	float alt_max{0.f};		// [m] effective ceiling
	float dist_home{0.f};
	float cas_min{0.f};		// [m/s] airspeed-low threshold
	float aspd_excess{NAN};		// [m/s] pitot TAS less the GPS estimate over the last full turn
	float aspd_turn{NAN};		// [m/s] GPS estimate of TAS over the last full turn
	uint16_t thermal_count{0};
	float height_gained{0.f};	// [m] sum over thermals
	float last_gain{0.f};		// [m] last thermal
	float last_time{0.f};		// [s] last thermal
	uint16_t cut_count{0};		// motor cuts: entries to GLIDE from powered flight
	float cut_pitch_step{0.f};	// [rad] path angle change of the last motor cut, nose down negative
};

class Soaring
{
public:
	void update(const Input &in, const Params &p, Output &out);

	// [m/s] still-air sink, positive down
	static float sink(const Params &p, float cas, float tas_over_cas, float bank);

	// [W, R, x, y]: strength m/s, radius m, centre north and east of the aircraft m
	static float updraft(const float s[4]);

	// a motor that stays dead at the restart is flagged by the thrust loss check alone, and only Return acts on its flag
	static bool configOk(int32_t act_fail_act, float tl_time) { return act_fail_act == 3 && tl_time > 0.f; }
	static void jacobian(const float s[4], float H[4]);

private:
	void setState(State state, uint8_t reason, float height);
	void enterThermal(const Input &in, const Params &p, float climb_expected);
	void thermalStep(const Input &in, const Params &p, float dt, float netto, float te, float radius);
	bool turnCheck(const Input &in, float tas);

	State _state{State::OFF};
	uint8_t _reason{REASON_NONE};
	uint64_t _last_us{0};
	float _state_s{0.f};
	float _good_s{0.f};
	float _aspd_low_s{0.f};
	float _glide_bar_s{0.f};
	uint8_t _latch{REASON_NONE};
	bool _speed_hold{false};
	float _climb_sp{0.f};
	uint16_t _cut_count{0};
	float _cut_pitch_step{0.f};

	float _climb_f{0.f};
	float _tas_prev{0.f};
	float _tas_dot{0.f};
	float _accel_f{0.f};
	float _netto_slow{0.f};
	float _climb_avg{0.f};

	float _s[4] {};
	float _P[4][4] {};
	float _update_s{0.f};
	bool _fault{false};
	float _cx{0.f};
	float _cy{0.f};
	bool _ccw{false};
	float _entry_height{0.f};

	float _yaw_prev{NAN};
	float _turn_net{0.f};		// [rad] signed heading change
	float _turn_peak{0.f};		// [rad] largest absolute heading change of this turn
	float _turn_sum[5] {};		// weight, weighted TAS, v north, v east, v.v
	float _aspd_excess{NAN};
	float _aspd_turn{NAN};
	bool _turn_ok{false};

	uint16_t _thermal_count{0};
	float _height_gained{0.f};
	float _last_gain{0.f};
	float _last_time{0.f};
};

} // namespace soaring
