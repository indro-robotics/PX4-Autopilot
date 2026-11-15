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

#include <mathlib/mathlib.h>

namespace flare_law
{

// the cap belongs to the time-to-touchdown trigger: there a momentary downdraft would start the flare high
inline float triggerHeight(float ttd, float time, float vz, float flalt)
{
	return (ttd > FLT_EPSILON && flalt > FLT_EPSILON) ? math::constrain(ttd * vz, flalt, 2.f * flalt) :
	       math::max((ttd > FLT_EPSILON ? ttd : time) * vz, flalt);
}

// a switch between the landing types in the flare would start the flare again at full throttle
inline bool landingReset(bool landing, bool mode_changed, bool flaring)
{
	return !landing || (mode_changed && !flaring);
}

// a stale sample means TECS was not flying the pitch, so the measured pitch seeds the law
inline float entryPitch(bool tecs_valid, uint64_t now_us, uint64_t tecs_us, float tecs_pitch, float pitch)
{
	return (tecs_valid && now_us < tecs_us + 200000) ? tecs_pitch : pitch;
}

// a nose-down entry pitch would lower the whole flare, so the seed rises to the flare floor over the ramp
inline float seed(float pitch_entry, float pmin, float time, float ramp_time)
{
	return pitch_entry + (math::max(pitch_entry, pmin) - pitch_entry) * math::min(time / ramp_time, 1.f);
}

// without terrain use the landing height is not range, so only the time ramp is valid there
inline bool rangeLatch(float kp, bool range_valid, int use_terrain)
{
	return kp > FLT_EPSILON && range_valid && use_terrain > 0;
}

// starts at the entry pitch so the first flare command is not clipped, never under pmax
inline float ceiling(float pitch_entry, float pmax, float time, float ramp_time)
{
	return math::max(pmax, pitch_entry + (pmax - pitch_entry) * math::min(time / ramp_time, 1.f));
}

// height rate setpoint, positive up: the entry rate at the entry height, -sink on the ground
inline float rangeSchedule(float height, float height_entry, float height_rate_entry, float sink)
{
	const float ramp = 1.f - math::constrain(height / height_entry, 0.f, 1.f);
	return ramp * (-sink) + (1.f - ramp) * height_rate_entry;
}

// pitch setpoint in rad; vz is positive down, gains in rad per m/s, integ carries the integrator between calls
inline float pitch(float height_rate_sp, float height_rate_entry, float pitch_entry, float vz, float speed, float kp,
		   float ki, float pitch_min, float pitch_max, float dt, float &integ)
{
	const float error = height_rate_sp + vz;
	const float u = pitch_entry + (height_rate_sp - height_rate_entry) / speed + kp * error + integ;

	if ((u < pitch_max || error < 0.f) && (u > pitch_min || error > 0.f)) {
		integ += ki * error * dt;
	}

	return math::constrain(u, pitch_min, pitch_max);
}

} // namespace flare_law
