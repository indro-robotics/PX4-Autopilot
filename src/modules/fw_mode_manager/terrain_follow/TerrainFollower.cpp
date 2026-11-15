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
static constexpr float kCosTiltMax = 0.766f;
static constexpr uint64_t kReturnLossTime = 30000000;
static constexpr double kSameCircle = 1e-7;
static constexpr float kJump = 0.2f;
static constexpr float kClearanceMin = 7.f;
static constexpr uint64_t kSetpointGap = 500000;
static constexpr float kLimitBand = 1.f;

float TerrainFollower::rawHagl(float range, float cos_tilt)
{
	return cos_tilt > kCosTiltMax ? range * cos_tilt : NAN;
}

bool TerrainFollower::ApproachLatch::update(bool past_land_start, bool fresh, bool close, double sp_lat, double sp_lon)
{
	// a loiter republished with a new altitude is the same circle
	if (!(fabs(sp_lat - lat) <= kSameCircle && fabs(sp_lon - lon) <= kSameCircle)) {
		lat = sp_lat;
		lon = sp_lon;
		// a loiter that starts around the aircraft is a Return climb or a Hold, not the approach
		started_inside = close;
	}

	// a setpoint older than the mission index belongs to the item before it
	latched = past_land_start && (latched || (close && fresh && !started_inside));
	return latched;
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
		// a jump during acquisition marks a sensor that has not settled, so the 2 s count starts again
		if (_t_valid_since == 0 || (!_range_ok && fabsf(in.hagl_raw - _raw_prev) > kJump * in.range_max)) {
			_t_valid_since = in.now_us;
		}

		_t_last_valid = in.now_us;
		_raw_prev = in.hagl_raw;
		_range_max = in.range_max;

	} else if (in.now_us > _t_last_valid + kLossTime) {
		_t_valid_since = 0;
	}

	_range_ok = _t_valid_since != 0 && (_range_ok || in.now_us >= _t_valid_since + kReacquireTime);

	float used = NAN;
	float interval = 0.f;
	bool sampled = false;

	if (in.raw_valid) {
		used = (in.ekf_valid && in.hagl_ekf < in.hagl_raw) ? in.hagl_ekf : in.hagl_raw;

		// per sample: a per-cycle derivative of a 20 Hz sensor reads at most 2/5 of the clip
		if (in.range_time_us != _t_sample) {
			interval = (_t_sample != 0 && in.range_time_us > _t_sample) ? (in.range_time_us - _t_sample) * 1e-6f : 0.f;
			_t_sample = in.range_time_us;
			sampled = true;
		}

	} else if (_have_terrain) {
		used = in.alt_amsl - _terrain;
		interval = (_t_sample == 0) ? in.dt : 0.f;
		_t_sample = 0;
		sampled = true;
	}

	if (sampled) {
		if (_have_terrain && interval > 0.f) {
			// a step reaches the follow law through the height error, not as a rate spike
			const float closure = math::constrain((_used - used) / interval, -kRateMax, kRateMax);
			const float a = interval / (kFilterTime + interval);
			_closure += a * (closure - _closure);
			_terrain_rate += a * (in.vz_up + closure - _terrain_rate);
		}

		_used = used;
		_have_terrain = true;
	}

	if (_have_terrain) {
		_terrain = in.alt_amsl - used;
	}

	const float uplift = kUplift * p.floor;

	if (!in.in_hold) {
		_hold_alt = -INFINITY;
	}

	// a Hold keeps the height its pull-up reached, or the next lap pulls up over the same crest
	const float nav_alt = (_hold_alt > in.nav_alt) ? _hold_alt : in.nav_alt;

	if (in.landing) {
		_released = false;

	} else if (_range_ok && used >= uplift + kUpliftExit) {
		_released = true;
	}

	const float terrain_rate = math::constrain(_terrain_rate, -kRateMax, kRateMax);
	const float hagl_min = p.floor + math::max(p.sink * p.ttg + 1.f, kClearanceMin);
	const float hagl_cmd = math::max(hagl_min, math::min(p.hagl, kRangeUse * _range_max * cosf(p.bank)));
	if (!in.no_setpoint) {
		_t_setpoint = in.now_us;
	}

	// a mode switch publishes an IDLE setpoint for a few cycles, which must not end RANGE_LOST, UPLIFT or PULL_UP
	const bool eligible = in.eligible || (in.no_setpoint && in.now_us < _t_setpoint + kSetpointGap);
	// a pull-up runs to its recovery in the Return it requested, which wants neither follow nor guard
	const bool active = eligible && !in.landing && (in.follow_wanted || in.guard || _state == PULL_UP);
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
		if (_released && in.follow_wanted && !lost) {
			_state = FOLLOW;

		} else if (_released && in.guard && !in.follow_wanted && !lost && nav_alt < _terrain + uplift) {
			_state = UPLIFT;
		}

		break;

	case FOLLOW:
	case UPLIFT: {
			const bool done = (_state == FOLLOW) ? !in.follow_wanted
					  : (in.follow_wanted || nav_alt > _terrain + uplift + kUpliftExit);
			// a guard that lost the ground from above the band it holds flies the plan, not the loss climb
			const bool above_band = _state == UPLIFT && _raw_prev > uplift + kUpliftExit;

			if (done || (lost && above_band)) {
				_state = STANDBY;

			} else if (lost) {
				_loss_alt = in.alt_amsl;
				// range bursts shorter than the reacquire time move the last valid sample, not the loss
				_t_lost = _t_last_valid;
				_state = RANGE_LOST;
			}
		}
		break;

	case RANGE_LOST:
		if (!lost) {
			_state = in.follow_wanted ? FOLLOW : STANDBY;

		} else if (in.in_return && in.now_us >= _t_lost + kReturnLossTime && _raw_prev >= p.floor) {
			// the loss climb holds the Return above its approach altitude, so the landing would never start
			_state = FAULT;
		}

		break;

	case PULL_UP: {
			const bool under = used < p.floor;

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

					if (in.in_hold) {
						_hold_alt = math::max(_hold_alt, in.alt_amsl);
					}
				}

			} else {
				_t_recover = 0;
			}
		}
		break;

	default:
		break;
	}

	const bool checked = _state == STANDBY || _state == FOLLOW || _state == UPLIFT || _state == RANGE_LOST;

	if (!checked || !_released) {
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
		const float target = (_state == FOLLOW) ? hagl_cmd : math::max(uplift, nav_alt - _terrain);
		float rate = terrain_rate + (target - used) / p.tc;

		if (_state == FOLLOW) {
			// a wrongly long range cannot take the aircraft further than FW_TF_MAXDN under the plan
			const float limit = (_hold_alt > in.nav_alt - p.max_down) ? _hold_alt : in.nav_alt - p.max_down;
			const float bound = (limit - in.alt_amsl) / p.tc;
			const bool at_limit = in.alt_amsl <= limit + kLimitBand;

			if (bound > rate) {
				rate = bound;
				// one Warning per arrival at the limit altitude, not one per cycle the bound slows the descent
				out.descent_limited = at_limit && !_limit_held;
				_limit_held |= at_limit;
			}

			_limit_held &= at_limit;
		}

		rate = math::constrain(rate, -p.sink, p.climb_max);
		out.use_rate = true;
		out.height_rate = in.raw_valid ? rate : math::max(rate, 0.f);

	} else if (_state == PULL_UP) {
		out.use_rate = true;
		out.height_rate = p.climb_max;
		out.min_airspeed = true;

	} else if (_state == RANGE_LOST) {
		out.altitude = math::max(in.nav_alt, _loss_alt + p.loss_up);

		if (in.raw_valid) {
			out.altitude = math::max(out.altitude, in.alt_amsl - used + (in.follow_wanted ? hagl_cmd : uplift));
		}

	} else if (_state == STANDBY && nav_alt > in.nav_alt) {
		out.altitude = nav_alt;
	}

	// a noisy low reading would climb without bound; above the ceiling the bound holds the height, never descends
	if (out.use_rate) {
		const float bound = math::max((in.ceiling - in.alt_amsl) / p.tc, 0.f);

		if (out.height_rate > bound && in.alt_amsl >= in.ceiling - kLimitBand && !in.in_return && !_return_sent) {
			out.request_return = true;
			_return_sent = true;
		}

		out.height_rate = math::min(out.height_rate, bound);
	}

	return out;
}
