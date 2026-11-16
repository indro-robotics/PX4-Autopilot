/*
  Soaring Controller class by Samuel Tabor

  Provides a layer between the thermal centring algorithm and the main
  code for managing navigation targets, data logging, tuning parameters,
  algorithm inputs and eventually other soaring strategies such as
  speed-to-fly. AP_TECS library used for reference.
*/

// Source: ArduPilot libraries/AP_Soaring/AP_Soaring.h, commit 74f0f83. Licence: GPLv3 or later, see COPYING.

#pragma once

#include "ExtendedKalmanFilter.hpp"
#include "SpeedToFly.hpp"
#include "Variometer.hpp"

#include <math.h>
#include <stdint.h>

namespace soaring
{

static constexpr float INITIAL_THERMAL_RADIUS = 80.0;
static constexpr float INITIAL_STRENGTH_COVARIANCE = 0.0049;
static constexpr float INITIAL_RADIUS_COVARIANCE = 400.0;
static constexpr float INITIAL_POSITION_COVARIANCE = 400.0;

class SoaringController
{
public:
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
		REASON_TERRAIN,
		REASON_HEIGHT_LOST,
		REASON_DRIFT
	};

	enum Operator : uint8_t {
		OPERATOR_MODE = 1,	// DO_SET_MODE
		OPERATOR_NAV = 2	// DO_REPOSITION, DO_CHANGE_ALTITUDE, DO_ORBIT, DO_CHANGE_SPEED
	};

	struct Params {
		bool enable{false};		// SOAR_ENABLE
		float vspeed{0.7f};		// [m/s] SOAR_VSPEED
		float q1{0.001f};		// SOAR_Q1, per update
		float q2{0.03f};		// SOAR_Q2, per update
		float r{0.45f};			// SOAR_R
		float dist_ahead{5.f};		// [m] SOAR_DIST_AHEAD
		float min_thml_s{20.f};		// [s] SOAR_MIN_THML_S
		float min_crse_s{10.f};		// [s] SOAR_MIN_CRSE_S
		float polar_cd0{0.027f};	// SOAR_POLAR_CD0
		float polar_b{0.031f};		// SOAR_POLAR_B
		float polar_k{25.6f};		// [m2/s2] SOAR_POLAR_K
		float alt_max{350.f};		// [m] SOAR_ALT_MAX, above home
		float alt_min{50.f};		// [m] SOAR_ALT_MIN, above home
		float alt_cutoff{250.f};	// [m] SOAR_ALT_CUTOFF, above home
		float max_drift{-1.f};		// [m] SOAR_MAX_DRIFT, -1 off
		float max_radius{-1.f};		// [m] SOAR_MAX_RADIUS, -1 off
		float thml_bank{30.f};		// [deg] SOAR_THML_BANK
		float thml_arspd{0.f};		// [m/s] SOAR_THML_ARSPD, 0 flies FW_AIRSPD_MIN at the bank's lift coefficient
		float crse_arspd{0.f};		// [m/s] SOAR_CRSE_ARSPD, 0 flies FW_AIRSPD_MIN, -1 speed to fly
		float alt_pat{80.f};		// [m] SOAR_ALT_PAT, height to arrive home with after the glide back
		float crse_rad{150.f};		// [m] SOAR_CRSE_RAD
		float drift{0.f};		// SOAR_DRIFT, share of the wind the thermal's base travels with
		float airspd_min{14.f};		// [m/s] FW_AIRSPD_MIN
		float airspd_trim{18.f};	// [m/s] FW_AIRSPD_TRIM
		float airspd_max{23.f};		// [m/s] FW_AIRSPD_MAX
		float clmb_max{3.f};		// [m/s] FW_T_CLMB_MAX
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
		uint8_t xy_reset_counter{0};
		float delta_x{0.f};		// [m] local north step of the last position reset
		float delta_y{0.f};		// [m] local east step of the last position reset
		float vn{0.f};			// [m/s] ground velocity north
		float ve{0.f};			// [m/s] ground velocity east
		float vz{0.f};			// [m/s] down
		bool vz_valid{true};
		float height{0.f};		// [m] above home
		float cas{0.f};			// [m/s]
		bool cas_valid{true};
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

		float vario_te{0.f};		// [m/s] total energy climb
		float vario_netto{0.f};		// [m/s] air rise
		float vario_netto_slow{0.f};	// [m/s] air rise through the entry filter
		float sink_polar{0.f};
		float climb_expected{0.f};	// [m/s] entry filter less the thermalling sink
		float climb_avg{0.f};		// [m/s] climb through the orbit filter
		float thermal_climb{0.f};	// [m/s] estimated climb on the thermalling circle
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

	SoaringController();

	void update(const Input &in, const Params &p, Output &out);

	// [m] estimate shift of one update: the travel of the base plus the lean in the wind it leaves
	static matrix::Vector2f drift_step(const matrix::Vector2f &wind, float dt, float climb, float strength, float drift);

	// a motor that stays dead at the restart is flagged by the thrust loss check alone, and only Return acts on its flag
	static bool configOk(int32_t act_fail_act, float tl_time) { return act_fail_act == 3 && tl_time > 0.f; }

private:
	Variometer::PolarParams _polarParams{};
	ExtendedKalmanFilter _ekf{};
	Variometer _vario;
	SpeedToFly _speedToFly;

	// store position thermal was entered as a backup check
	matrix::Vector3f _thermal_start_pos;

	float _thermalability{0.f};

	bool check_thermal_criteria(const Params &p, float alt_floor, float alt_max) const;
	void init_thermalling(const Input &in, const Params &p);
	void update_thermalling(const Input &in, const Params &p, float dt, float radius);
	void update_cruising(const Input &in, const Params &p);
	bool check_drift(const Params &p) const;
	float get_cruising_target_airspeed(const Params &p, float cruise_cas);

	void setState(State state, uint8_t reason, float height);
	bool turnCheck(const Input &in, float tas);

	State _state{State::OFF};
	uint8_t _reason{REASON_NONE};
	uint64_t _last_us{0};
	uint8_t _xy_reset_counter{0};
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

	bool _fault{false};
	float _cx{0.f};
	float _cy{0.f};
	bool _ccw{false};

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
