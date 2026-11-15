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

	// an orbit left above the height its final needs lands long: the distance to the touchdown point over the glide ratio, plus 10 m
	static bool orbitEnds(bool touchdown_next, float height, float distance, float glide)
	{
		return !touchdown_next || !(height > distance / (glide > 1.f ? glide : 1.f) + 10.f);
	}

private:
	uint64_t _t_prev{0};
	uint64_t _t_change{0};
	bool _lost{false};
};
