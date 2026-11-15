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
#include <lib/range_select/StuckCheck.hpp>

class TerrainFollower
{
public:
	enum State : uint8_t { OFF, STANDBY, FOLLOW, UPLIFT, PULL_UP, RANGE_LOST, FAULT };
	enum Nav : uint8_t { NAV_OTHER, NAV_HOLD, NAV_MISSION, NAV_RETURN };

	struct Input  { uint64_t now_us, range_time_us; float dt; bool eligible, landing, follow_wanted, guard, in_return, in_hold,
			no_setpoint; float alt_amsl, vz_up, nav_alt, ceiling; bool raw_valid; float hagl_raw, range_max;
			bool ekf_valid; float hagl_ekf; };
	struct Params { float hagl, floor, tc, sink, climb_max, loss_up, ttg, bank, max_down; };
	struct Output { uint8_t state; bool use_rate; float height_rate, altitude; bool min_airspeed,
			cap_bank, request_return, descent_limited; float hagl_cmd, hagl_used, terrain_rate; };

	// [m] above home: the climb bound without a fence ceiling
	static constexpr float kCeilingNoFence = 120.f;

	using StuckCheck = range_select::StuckCheck;

	// The approach circle and every leg after it while the mission index stays past DO_LAND_START.
	struct ApproachLatch {
		double lat{NAN};
		double lon{NAN};
		bool started_inside{false};
		bool latched{false};
		bool update(bool past_land_start, bool fresh, bool close, double sp_lat, double sp_lon);
	};

	Output update(const Input &in, const Params &p);

	void reset() { *this = TerrainFollower{}; }

	// an estimator height reset moves the altitude estimate, not the aircraft, the ground or the loss point
	void heightReset(float delta)
	{
		_terrain += delta;
		_loss_alt += delta;
		_hold_alt += delta;
	}

	// a new origin altitude moves z and the origin together, so the altitude above sea level holds still
	static float resetShift(float delta_z, float ref_alt, float ref_alt_prev)
	{
		return fabsf(ref_alt - ref_alt_prev) > 0.f ? 0.f : -delta_z;
	}

	bool latchFollow(bool bits_set, bool airborne);

	bool rangeOk() const { return _range_ok; }

	static bool ownsAltitude(uint8_t state) { return state >= FOLLOW && state <= RANGE_LOST; }

	// soaring flies its own height band in Hold: a follow at FW_TF_HAGL would hold it under its floor
	static bool followBit(int32_t en, Nav nav, bool soaring = false)
	{
		return !soaring && ((nav == NAV_HOLD && (en & 1)) || (nav == NAV_MISSION && (en & 2)));
	}

	static bool guardActs(uint8_t state) { return state == UPLIFT || state == PULL_UP; }

	// LAND after a loiter is the landing only from its circle on: the pattern orbit before it is the transit
	static bool landingContext(bool land_current, bool land_next, bool loiter_current, bool on_circle)
	{
		return land_current || (land_next && (!loiter_current || on_circle));
	}

	static float rawHagl(float range, float cos_tilt);

private:
	uint8_t _state{OFF};
	uint8_t _resume{STANDBY};
	bool _range_ok{false};
	bool _released{false};
	bool _have_terrain{false};
	bool _latched{false};
	bool _bits_prev{false};
	bool _return_sent{false};
	bool _limit_held{false};
	uint64_t _t_setpoint{0};
	uint64_t _t_sample{0};
	uint64_t _t_valid_since{0};
	uint64_t _t_last_valid{0};
	uint64_t _t_lost{0};
	uint64_t _t_low{0};
	uint64_t _t_recover{0};
	uint64_t _t_under{0};
	float _range_max{0.f};
	float _raw_prev{0.f};
	float _terrain{0.f};
	float _used{0.f};
	float _terrain_rate{0.f};
	float _closure{0.f};
	float _loss_alt{0.f};
	float _hold_alt{-INFINITY};
};
