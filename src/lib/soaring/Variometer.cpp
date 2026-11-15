/* Variometer class by Samuel Tabor

Manages the estimation of aircraft total energy, drag and vertical air velocity.
*/

// Source: ArduPilot libraries/AP_Soaring/Variometer.cpp, commit 74f0f83. Licence: GPLv3 or later, see COPYING.

#include "Variometer.hpp"

#include <math.h>
#include <mathlib/math/Limits.hpp>
#include <px4_platform_common/defines.h>

namespace soaring
{

static constexpr float GRAVITY_MSS = 9.80665f;

float LowPassFilter::apply(float sample, float dt)
{
	_output += (sample - _output) * dt / (dt + 1.f / (M_TWOPI_F * _cutoff_freq));
	return _output;
}

float AverageFilterFloat_Size5::apply(float sample)
{
	_samples[_next] = sample;
	_next = (_next + 1) % 5;
	_count = _count < 5 ? _count + 1 : 5;
	float sum = 0.f;

	for (uint8_t i = 0; i < _count; i++) {
		sum += _samples[i];
	}

	return sum / _count;
}

Variometer::Variometer(const PolarParams &polarParams) :
	_polarParams(polarParams)
{
}

void Variometer::update(const Input &in, const float thermal_bank, const float thermal_airspeed)
{
	alt = in.height;

	float aspd = in.eas;

	if (!in.eas_valid) {
		aspd = in.airspeed_cruise;
	}

	float aspd_filt = _sp_filter.apply(aspd);

	// Constrained airspeed.
	const float minV = sqrtf(_polarParams.K / 1.5f);
	_aspd_filt_constrained = aspd_filt > minV ? aspd_filt : minV;

	tau = calculate_circling_time_constant(math::radians(thermal_bank));

	float dt = (float)(in.time_us - _prev_update_time) / 1e6f;

	// Logic borrowed from AP_TECS.cpp
	// Update and average speed rate of change
	// Calculate speed rate of change
	float temp = in.accel_x;
	// take 5 point moving average
	float dsp = _vdot_filter.apply(temp);

	// Now we need to high-pass this signal to remove bias.
	_vdotbias_filter.set_cutoff_frequency(1 / (20 * tau));
	float dsp_bias = _vdotbias_filter.apply(temp, dt);

	float dsp_cor = dsp - dsp_bias;


	float raw_climb_rate = 0.0f;

	if (in.vz_valid) {
		// if possible use the EKF vertical velocity
		raw_climb_rate = -in.vz;
	}

	_climb_filter.set_cutoff_frequency(1 / (3 * tau));
	float smoothed_climb_rate = _climb_filter.apply(raw_climb_rate, dt);

	// Compute still-air sinkrate
	float roll = in.roll;
	float sinkrate = calculate_aircraft_sinkrate(roll, _aspd_filt_constrained);

	reading = raw_climb_rate + dsp_cor * _aspd_filt_constrained / GRAVITY_MSS + sinkrate;

	// Update filters.

	float filtered_reading = _trigger_filter.apply(reading, dt);

	_audio_filter.apply(reading, dt);

	_stf_filter.apply(reading, dt);

	_prev_update_time = in.time_us;

	// the turn is flown at the thermalling airspeed: at the X8's 14.0 m/s glide the sink reads 1.62 against 1.91 m/s
	_expected_thermalling_sink = calculate_aircraft_sinkrate(math::radians(thermal_bank), thermal_airspeed);

	(void)filtered_reading;
	(void)smoothed_climb_rate;
}

float Variometer::calculate_aircraft_sinkrate(float phi, float aspd) const
{
	// Remove aircraft sink rate
	float CL0;  // CL0 = 2*W/(rho*S*V^2)
	float C1;   // C1 = CD0/CL0
	float C2;   // C2 = CDi0/CL0 = B*CL0
	CL0 = _polarParams.K / (aspd * aspd);

	C1 = _polarParams.CD0 / CL0;  // constant describing expected angle to overcome zero-lift drag
	C2 = _polarParams.B * CL0;    // constant describing expected angle to overcome lift induced drag at zero bank

	// exact cosine to 60 deg: the two-term series reads 2.6 m/s high at 70 deg and 15 m/s, and divides by zero at 81 deg
	float cosphi = cosf(math::min(fabsf(phi), math::radians(60.f)));

	return aspd * (C1 + C2 / (cosphi * cosphi));
}

float Variometer::calculate_circling_time_constant(float thermal_bank) const
{
	// Calculate a time constant to use to filter quantities over a full thermal orbit.
	// This is used for rejecting variation in e.g. climb rate, or estimated climb rate
	// potential, as the aircraft orbits the thermal.
	// Use the time to circle - variations at the circling frequency then have a gain of 25%
	// and the response to a step input will reach 64% of final value in three orbits.
	return 2 * M_PI_F * _aspd_filt_constrained / (GRAVITY_MSS * tanf(thermal_bank));
}

} // namespace soaring
