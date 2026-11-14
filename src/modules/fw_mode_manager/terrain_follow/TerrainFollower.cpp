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


#include "TerrainFollower.hpp"

#include <mathlib/math/Limits.hpp>

static constexpr uint64_t kLossTime = 500000;
static constexpr uint64_t kReacquireTime = 2000000;
static constexpr uint64_t kLowTime = 200000;
static constexpr uint64_t kRecoverTime = 1000000;
static constexpr uint64_t kReturnTime = 6000000;
static constexpr float kFilterTime = 1.f;
static constexpr float kRateMax = 5.f;
static constexpr float kRangeUse = 0.75f;
static constexpr float kUplift = 1.5f;
static constexpr float kUpliftExit = 2.f;
static constexpr float kFaultClimb = 40.f;
static constexpr float kCosTiltMax = 0.766f;

float TerrainFollower::rawHagl(float range, float cos_tilt)
{
	return cos_tilt > kCosTiltMax ? range * cos_tilt : NAN;
}

bool TerrainFollower::latchFollow(bool bits_set, bool airborne)
{
	if (!airborne || !bits_set) {
		_latched = false;

	} else if (!_bits_prev) {
		_latched = true;
	}

	_bits_prev = bits_set;
	return _latched;
}

TerrainFollower::Output TerrainFollower::update(const Input &in, const Params &p)
{
	if (in.raw_valid) {
		if (_t_valid_since == 0) {
			_t_valid_since = in.now_us;
		}

		_t_last_valid = in.now_us;
		_range_max = in.range_max;

	} else if (in.now_us > _t_last_valid + kLossTime) {
		_t_valid_since = 0;
	}

	_range_ok = _t_valid_since != 0 && (_range_ok || in.now_us >= _t_valid_since + kReacquireTime);

	const bool have_used = in.raw_valid || _have_terrain;
	float used = NAN;

	if (in.raw_valid) {
		used = (in.ekf_valid && in.hagl_ekf < in.hagl_raw) ? in.hagl_ekf : in.hagl_raw;

	} else if (_have_terrain) {
		used = in.alt_amsl - _terrain;
	}

	if (have_used) {
		if (_have_terrain && in.dt > 0.f) {
			// a step reaches the follow law through the height error, not as a rate spike
			const float closure = math::constrain((_used - used) / in.dt, -kRateMax, kRateMax);
			const float a = in.dt / (kFilterTime + in.dt);
			_closure += a * (closure - _closure);
			_terrain_rate += a * (in.vz_up + closure - _terrain_rate);
		}

		_used = used;
		_terrain = in.alt_amsl - used;
		_have_terrain = true;
	}

	const float terrain_rate = math::constrain(_terrain_rate, -kRateMax, kRateMax);
	const float hagl_min = p.floor + p.sink * p.ttg + 1.f;
	const float hagl_cmd = math::max(hagl_min, math::min(p.hagl, kRangeUse * _range_max * cosf(p.bank)));
	const float uplift = kUplift * p.floor;
	const bool active = in.eligible && (in.follow_wanted || in.guard);
	const bool lost = !_range_ok;

	Output out{};

	if (_state != FAULT && !active) {
		_state = OFF;
	}

	switch (_state) {
	case OFF:
		if (active) {
			_state = STANDBY;
		}

		break;

	case STANDBY:
		if (in.follow_wanted && !lost) {
			_state = FOLLOW;

		} else if (in.guard && !in.follow_wanted && !lost && in.nav_alt < _terrain + uplift) {
			_state = UPLIFT;
		}

		break;

	case FOLLOW:
	case UPLIFT: {
			const bool done = (_state == FOLLOW) ? !in.follow_wanted
					  : (in.follow_wanted || in.nav_alt > _terrain + uplift + kUpliftExit);

			if (done) {
				_state = STANDBY;

			} else if (lost) {
				_loss_alt = in.alt_amsl;
				_state = RANGE_LOST;
			}
		}
		break;

	case RANGE_LOST:
		if (!lost) {
			_state = in.follow_wanted ? FOLLOW : STANDBY;
		}

		break;

	case PULL_UP: {
			const bool under = used < p.floor;

			if (under && in.alt_amsl > _pullup_alt + kFaultClimb) {
				_state = FAULT;
				break;
			}

			if (!under || _t_under == 0) {
				_t_under = under ? in.now_us : 0;
			}

			if (under && !in.in_return && !_return_sent && in.now_us >= _t_under + kReturnTime) {
				out.request_return = true;
				_return_sent = true;
			}

			if (used >= uplift && _closure <= 0.f) {
				if (_t_recover == 0) {
					_t_recover = in.now_us;
				}

				if (in.now_us >= _t_recover + kRecoverTime) {
					_state = _resume;
				}

			} else {
				_t_recover = 0;
			}
		}
		break;

	default:
		break;
	}

	if (_state != STANDBY && _state != FOLLOW && _state != UPLIFT) {
		_t_low = 0;

	} else if (in.raw_valid) {
		if (used >= p.floor) {
			_t_low = 0;

		} else if (_t_low == 0) {
			_t_low = in.now_us;
		}

		const bool low = _t_low != 0 && in.now_us >= _t_low + kLowTime;
		const bool closing = p.ttg > 0.f && _closure > 0.f && used >= p.floor && (used - p.floor) < _closure * p.ttg;

		if (low || closing) {
			_resume = _state;
			_pullup_alt = in.alt_amsl;
			_t_recover = 0;
			_t_under = used < p.floor ? in.now_us : 0;
			_t_low = 0;
			_state = PULL_UP;
		}
	}

	out.state = _state;
	out.height_rate = NAN;
	out.altitude = NAN;
	out.hagl_cmd = hagl_cmd;
	out.hagl_used = used;
	out.terrain_rate = terrain_rate;
	out.cap_bank = ownsAltitude(_state);

	if (_state == FOLLOW || _state == UPLIFT) {
		const float target = (_state == FOLLOW) ? hagl_cmd : uplift;
		const float rate = math::constrain(terrain_rate + (target - used) / p.tc, -p.sink, p.climb_max);
		out.use_rate = true;
		out.height_rate = in.raw_valid ? rate : math::max(rate, 0.f);

	} else if (_state == PULL_UP) {
		out.use_rate = true;
		out.height_rate = p.climb_max;
		out.min_airspeed = true;

	} else if (_state == RANGE_LOST) {
		out.altitude = math::max(in.nav_alt, _loss_alt + p.loss_up);
	}

	return out;
}
