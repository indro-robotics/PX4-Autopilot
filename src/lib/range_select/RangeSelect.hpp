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

#include <math.h>
#include <stdint.h>
#include <uORB/topics/distance_sensor.h>

namespace range_select
{

struct Selection {
	int used{-1};          // newest sample of the selected group, else the newest fresh downward sample, -1 when none
	int count{0};          // samples averaged
	float distance{NAN};   // group mean, else the used sample's distance
	uint64_t timestamp{0}; // group mean sample time, else the used sample's
	float range_max{0.f};  // longest max_distance among fresh downward samples
};

// long_floor is hysteresis state: one per caller, 0 before the first call
inline Selection selectDownwardRange(const distance_sensor_s *samples, int n, uint64_t stale_before, float &long_floor)
{
	const auto fresh = [&](const distance_sensor_s & s) {
		return (s.timestamp > stale_before) && (s.orientation == distance_sensor_s::ROTATION_DOWNWARD_FACING);
	};

	const auto good = [&](const distance_sensor_s & s) {
		// a shorter sensor rejoins only under 0.9 x its range, so the selection does not toggle at its limit
		const float limit = (s.max_distance < long_floor) ? 0.9f * s.max_distance : s.max_distance;
		return fresh(s) && (s.signal_quality != 0) && (s.current_distance >= s.min_distance) && (s.current_distance <= limit);
	};

	Selection sel{};
	float shortest = INFINITY;

	for (int i = 0; i < n; i++) {
		if (fresh(samples[i])) {
			if ((sel.used < 0) || (samples[i].timestamp > samples[sel.used].timestamp)) {
				sel.used = i;
			}

			sel.range_max = fmaxf(sel.range_max, samples[i].max_distance);

			if (good(samples[i])) {
				shortest = fminf(shortest, samples[i].max_distance);
			}
		}
	}

	float sum = 0.f;
	uint64_t time_sum = 0;
	int newest_good = -1;
	bool shorter_left_out = false;

	for (int i = 0; i < n; i++) {
		const distance_sensor_s &s = samples[i];

		if (good(s) && (s.max_distance <= 1.2f * shortest)) {
			sum += s.current_distance;
			time_sum += s.timestamp;
			sel.count++;

			if ((newest_good < 0) || (s.timestamp > samples[newest_good].timestamp)) {
				newest_good = i;
			}

		} else if (fresh(s) && (s.max_distance < shortest)) {
			shorter_left_out = true;
		}
	}

	if (sel.count > 0) {
		sel.used = newest_good;
		sel.distance = sum / sel.count;
		sel.timestamp = time_sum / sel.count;
		long_floor = shorter_left_out ? shortest : 0.f;

	} else {
		long_floor = 0.f;

		if (sel.used >= 0) {
			sel.distance = samples[sel.used].current_distance;
			sel.timestamp = samples[sel.used].timestamp;
		}
	}

	return sel;
}

} // namespace range_select
