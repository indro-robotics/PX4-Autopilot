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

class TerrainFollower
{
public:
	enum State : uint8_t { OFF, STANDBY, FOLLOW, UPLIFT, PULL_UP, RANGE_LOST, FAULT };
	enum Nav : uint8_t { NAV_OTHER, NAV_HOLD, NAV_MISSION, NAV_RETURN };

	struct Input  { uint64_t now_us; float dt; bool eligible, follow_wanted, guard, in_return;
			float alt_amsl, vz_up, nav_alt; bool raw_valid; float hagl_raw, range_max;
			bool ekf_valid; float hagl_ekf; };
	struct Params { float hagl, floor, tc, sink, climb_max, loss_up, ttg, bank; };
	struct Output { uint8_t state; bool use_rate; float height_rate, altitude; bool min_airspeed,
			cap_bank, request_return; float hagl_cmd, hagl_used, terrain_rate; };

	Output update(const Input &in, const Params &p);

	void reset() { *this = TerrainFollower{}; }

	bool latchFollow(bool bits_set, bool airborne);

	bool rangeOk() const { return _range_ok; }

	static bool ownsAltitude(uint8_t state) { return state >= FOLLOW && state <= RANGE_LOST; }

	static bool eligible(bool flying_auto, bool landing_item, Nav nav, bool past_land_start)
	{
		return flying_auto && !landing_item && nav != NAV_OTHER && !(nav == NAV_MISSION && past_land_start);
	}

	static bool followBit(int32_t en, Nav nav)
	{
		return (nav == NAV_HOLD && (en & 1)) || (nav == NAV_MISSION && (en & 2));
	}

	static float rawHagl(float range, float cos_tilt);

private:
	uint8_t _state{OFF};
	uint8_t _resume{STANDBY};
	bool _range_ok{false};
	bool _have_terrain{false};
	bool _latched{false};
	bool _bits_prev{false};
	bool _return_sent{false};
	uint64_t _t_valid_since{0};
	uint64_t _t_last_valid{0};
	uint64_t _t_low{0};
	uint64_t _t_recover{0};
	uint64_t _t_under{0};
	float _range_max{0.f};
	float _terrain{0.f};
	float _used{0.f};
	float _terrain_rate{0.f};
	float _closure{0.f};
	float _loss_alt{0.f};
	float _pullup_alt{0.f};
};
