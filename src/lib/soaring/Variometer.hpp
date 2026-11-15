
/* Variometer class by Samuel Tabor

Manages the estimation of aircraft total energy, drag and vertical air velocity.
*/

// Source: ArduPilot libraries/AP_Soaring/Variometer.h, commit 74f0f83. Licence: GPLv3 or later, see COPYING.

#pragma once

#include <stdint.h>

namespace soaring
{

// first order, time constant 1/(2 pi cutoff)
class LowPassFilter
{
public:
	explicit LowPassFilter(float cutoff_freq) : _cutoff_freq(cutoff_freq) {}
	void set_cutoff_frequency(float cutoff_freq) { _cutoff_freq = cutoff_freq; }
	float apply(float sample, float dt);
	void reset(float value) { _output = value; }
	float get() const { return _output; }

private:
	float _cutoff_freq;
	float _output{0.f};
};

// mean of the last five samples, fewer until five arrive
class AverageFilterFloat_Size5
{
public:
	float apply(float sample);

private:
	float _samples[5] {};
	uint8_t _count{0};
	uint8_t _next{0};
};

class Variometer
{

	// store time of last update
	uint64_t _prev_update_time{0};

	float _aspd_filt_constrained{0};

	float _expected_thermalling_sink{0};

	// declares a 5point average filter using floats
	AverageFilterFloat_Size5 _vdot_filter;

	AverageFilterFloat_Size5 _sp_filter;

	/*
	 low pass filters for various purposes.
	 */
	// Climb rate filter for monitoring progress in thermal.
	LowPassFilter _climb_filter{1 / 60.0f};

	// Fast filter for mavlink/audio vario output.
	LowPassFilter _audio_filter{1 / 0.71f};

	// Slower filter for deciding to enter THERMAL mode.
	LowPassFilter _trigger_filter{1 / 4.06f};

	// Longitudinal acceleration bias filter.
	LowPassFilter _vdotbias_filter{1 / 60.0f};

	// Speed to fly vario filter.
	LowPassFilter _stf_filter{1 / 20.0f};

public:
	struct PolarParams {
		float K;
		float CD0;
		float B;
	};

	// one sample of the vehicle state
	struct Input {
		uint64_t time_us;
		float height;           // [m] above home
		float eas;              // [m/s]
		bool eas_valid;
		float airspeed_cruise;  // [m/s] used without a valid airspeed
		float accel_x;          // [m/s2] kinematic acceleration along the body x axis
		float vz;               // [m/s] down
		bool vz_valid;
		float roll;             // [rad]
	};

	Variometer(const PolarParams &polarParams);

	float alt{0};
	float reading{0};
	float tau{0};

	void update(const Input &in, const float thermal_bank, const float thermal_airspeed);
	float calculate_aircraft_sinkrate(float phi, float aspd) const;

	void reset_climb_filter(float value) { _climb_filter.reset(value);}

	void reset_trigger_filter(float value) { _trigger_filter.reset(value);}

	float get_airspeed(void) const {return _aspd_filt_constrained;};

	float get_displayed_value(void) const {return _audio_filter.get();};

	float get_filtered_climb(void) const {return _climb_filter.get();};

	float get_trigger_value(void) const {return _trigger_filter.get();};

	float get_stf_value(void) const {return _stf_filter.get();};

	float get_exp_thermalling_sink(void) const {return _expected_thermalling_sink;};

	float calculate_circling_time_constant(const float thermal_bank) const;

private:
	const PolarParams &_polarParams;
};

} // namespace soaring
