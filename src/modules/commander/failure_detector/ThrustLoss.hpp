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

#pragma once

#include <float.h>
#include <math.h>
#include <stdint.h>

class ThrustLoss
{
public:
	struct Param {
		float time_s;
		float throttle;
		float rate;
	};

	// energy_rate is tecs_status.total_energy_rate in m^2/s^3; flying is fixed wing and not landed; low is under FW_TL_LVL_ALT
	void update(uint64_t t_us, bool armed, bool flying, bool low, float throttle, float energy_rate, const Param &p)
	{
		if (!armed || p.time_s <= 0.f) {
			*this = ThrustLoss{};
			return;
		}

		if (t_us <= _t_prev) {
			return;
		}

		if (t_us > _t_prev + 500000) {
			_t_change = 0;
		}

		_t_prev = t_us;

		// total_energy_rate is already low-passed by FW_T_STE_R_TC; a second filter delays a climb-out loss past 5 s
		const float e = energy_rate / 9.81f;
		// a motor commanded off, as in a soaring glide, loses energy at any threshold
		const bool thrust_expected = throttle > 0.f && throttle >= p.throttle;
		// TECS lowers the throttle once thrust is back, so the energy gain alone clears the flag
		// rising air near the ground would clear the flag over a dead motor and bank the glide toward its setpoint
		const bool change = _lost ? (e > -p.rate && !(low && flying)) : (thrust_expected && flying && e < p.rate);

		if (!change) {
			_t_change = 0;

		} else if (_t_change == 0) {
			_t_change = t_us;
		}

		if (_t_change != 0 && t_us >= _t_change + static_cast<uint64_t>(p.time_s * 1e6f)) {
			_lost = !_lost;
			_t_change = 0;
		}
	}

	bool lost() const { return _lost; }

	static constexpr float kAccelBank30 = 9.80665f * 0.57735f;

	// an orbit left above the height its final needs lands long: the distance to the touchdown point over the glide ratio, plus 10 m
	static bool orbitEnds(bool touchdown_next, float height, float distance, float glide, float course_change,
			      float airspeed)
	{
		distance += turnArc(course_change, airspeed);
		return !touchdown_next || !(height > distance / (glide > 1.f ? glide : 1.f) + 10.f);
	}

	// the turn onto the final is flown at 30 deg of bank before the glide to the touchdown point, and its arc costs height like the rest of the path
	static float turnArc(float course_change, float airspeed)
	{
		const float arc = acosf(cosf(course_change)) * airspeed * airspeed / kAccelBank30;
		return __builtin_isfinite(arc) ? arc : 0.f;
	}

	// the glide arrives under the altitude an orbit item holds to its ring, so the altitude test would keep the item for good
	static bool orbitRing(bool on_ring, bool at_altitude, bool flagged)
	{
		return on_ring && (at_altitude || flagged);
	}

	// wind changes the ground covered per metre of height: ground speed over airspeed, held to 0.5 to 1.2; a crosswind costs the crab
	static float glideOverGround(float glide, float airspeed, float wind_along, float wind_cross)
	{
		const float scale = (sqrtf(fmaxf(airspeed * airspeed - wind_cross * wind_cross, 0.f)) + wind_along) / airspeed;
		return glide * (__builtin_isfinite(scale) ? fminf(fmaxf(scale, 0.5f), 1.2f) : 1.f);
	}

	// the final leaves an orbit on its tangent, turned from the bearing of the centre to the touchdown point by asin(radius / range)
	static float finalCourse(float bearing, float radius, float range, bool counter_clockwise)
	{
		const float turn = asinf(fminf(fabsf(radius) / fmaxf(range, 1.f), 1.f));
		return counter_clockwise ? bearing - turn : bearing + turn;
	}

	// an orbit and a circular landing are reached at the circle: the distance to the centre would level the wings on it
	static float levelHeight(float level_alt, float distance, bool circle, float radius, float glide)
	{
		const float reach = (circle && __builtin_isfinite(radius)) ? fmaxf(distance - fabsf(radius), 0.f) : distance;
		return level_alt + (__builtin_isfinite(reach) ? reach : 0.f) / fmaxf(glide, 1.f);
	}

	// without a range the height is above the estimator origin: over lower ground the floor would hold the nose up at stall speed
	static bool pitchFloor(bool takeoff, bool landing, bool range_valid, float range, float flare_alt)
	{
		return takeoff || (range_valid && range < 2.f * flare_alt && !landing);
	}

	// the guidance minimum keeps the ground speed floor into wind; a pilot mode keeps the airspeed its stick commands
	static float glideAirspeed(bool auto_mode, float setpoint, float airspeed_min, float guidance_min, float airspeed_max)
	{
		return auto_mode ? fminf(fmaxf(airspeed_min, guidance_min), airspeed_max) : setpoint;
	}

	// a dead motor pays for bank with height: 30 deg raises the sink rate by a quarter, 50 deg nearly doubles it
	// the wing strike rule limits the bank only just above FW_WING_HEIGHT, too low to level out of 30 deg at glide sink: 30 deg at 15 m of range down to 10 deg at 10 m
	static float lateralAccelMax(bool flagged, bool auto_mode, float accel_max, bool range_valid, float range)
	{
		const float bank_deg = 10.f + 20.f * fminf(fmaxf((range - 10.f) / 5.f, 0.f), 1.f);
		const float limit = range_valid ? 9.80665f * tanf(bank_deg * 0.0174533f) : kAccelBank30;
		return (flagged && auto_mode) ? fminf(accel_max, limit) : accel_max;
	}

	// a motor that runs again would fly the rest of the leg at the orbit height: NAN for no write, else the altitude of the orbit setpoint
	static float returnLegAltitude(bool flagged, bool before_ring, float setpoint_alt, float orbit_alt, float altitude,
				       float return_alt, bool &lowered)
	{
		if (flagged) {
			const bool lower = fabsf(setpoint_alt - orbit_alt) >= FLT_EPSILON;
			lowered = lowered || lower;
			return lower ? orbit_alt : NAN;
		}

		const bool restore = lowered && before_ring;
		lowered = false;
		return restore ? fmaxf(altitude, return_alt) : NAN;
	}

private:
	uint64_t _t_prev{0};
	uint64_t _t_change{0};
	bool _lost{false};
};
