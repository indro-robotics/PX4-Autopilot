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


#include <gtest/gtest.h>
#include <math.h>

#include "TerrainFollower.hpp"

using TF = TerrainFollower;

static constexpr float kDt = 0.02f;
static constexpr float kTerrain = 100.f;

class TerrainFollowerTest : public ::testing::Test
{
protected:
	void SetUp() override
	{
		in.now_us = 1000000;
		in.dt = kDt;
		in.eligible = true;
		in.follow_wanted = true;
		in.guard = false;
		in.in_return = false;
		in.ceiling = INFINITY;
		setHeight(30.f);
		in.nav_alt = kTerrain + 30.f;
		in.range_max = 50.f;
		in.ekf_valid = true;
	}

	void setHeight(float h, float terrain = kTerrain)
	{
		in.alt_amsl = terrain + h;
		in.raw_valid = true;
		in.hagl_raw = h;
		in.hagl_ekf = h;
	}

	// one cycle with a new 50 Hz range sample
	TF::Output step()
	{
		in.now_us += static_cast<uint64_t>(kDt * 1e6f);
		in.range_time_us = in.now_us;
		out = tf.update(in, p);
		return out;
	}

	TF::Output run(float seconds)
	{
		const int n = static_cast<int>(seconds / kDt + 0.5f);

		for (int i = 0; i < n; i++) {
			step();
		}

		return out;
	}

	void follow()
	{
		run(2.1f);
		ASSERT_EQ(out.state, TF::FOLLOW);
	}

	TF tf;
	TF::Params p{25.f, 8.f, 4.f, 2.f, 3.f, 10.f, 3.f, 35.f * static_cast<float>(M_PI) / 180.f, 10.f};
	TF::Input in{};
	TF::Output out{};
};

TEST_F(TerrainFollowerTest, IneligibleStaysOff)
{
	in.eligible = false;
	EXPECT_EQ(run(5.f).state, TF::OFF);
	EXPECT_FALSE(out.use_rate);
	EXPECT_TRUE(isnan(out.altitude));
	EXPECT_FALSE(out.cap_bank);
}

TEST_F(TerrainFollowerTest, NeitherFollowNorGuardStaysOff)
{
	in.follow_wanted = false;
	EXPECT_EQ(run(5.f).state, TF::OFF);
}

TEST_F(TerrainFollowerTest, FollowAfterTwoSecondsOfRange)
{
	EXPECT_EQ(step().state, TF::STANDBY);
	EXPECT_EQ(run(1.9f).state, TF::STANDBY);
	EXPECT_EQ(run(0.2f).state, TF::FOLLOW);
	EXPECT_TRUE(out.use_rate);
	EXPECT_TRUE(out.cap_bank);
	EXPECT_TRUE(isnan(out.altitude));
	// 30 m over a 25 m command with TC 4 s: (25 - 30) / 4 = -1.25 m/s
	EXPECT_NEAR(out.height_rate, -1.25f, 0.05f);
}

TEST_F(TerrainFollowerTest, IneligibleTurnsFollowOff)
{
	follow();
	in.eligible = false;
	EXPECT_EQ(step().state, TF::OFF);
	EXPECT_FALSE(out.use_rate);
}

TEST_F(TerrainFollowerTest, FollowClearedWithGuardGoesStandby)
{
	follow();
	in.follow_wanted = false;
	in.guard = true;
	EXPECT_EQ(step().state, TF::STANDBY);
	EXPECT_FALSE(out.use_rate);
}

TEST_F(TerrainFollowerTest, FollowClearedWithoutGuardGoesOff)
{
	follow();
	in.follow_wanted = false;
	EXPECT_EQ(step().state, TF::OFF);
}

TEST_F(TerrainFollowerTest, NoRangeNoDescent)
{
	setHeight(45.f);
	in.raw_valid = false;
	in.hagl_raw = NAN;
	in.ekf_valid = false;
	EXPECT_EQ(run(10.f).state, TF::STANDBY);
	EXPECT_FALSE(out.use_rate);
	EXPECT_TRUE(isnan(out.altitude));
}

TEST_F(TerrainFollowerTest, GuardUpliftAndExit)
{
	in.follow_wanted = false;
	in.guard = true;
	// released at 15 m, over 1.5 x 8 + 2 = 14 m
	setHeight(15.f);
	in.nav_alt = kTerrain + 10.f;
	run(2.1f);
	ASSERT_EQ(out.state, TF::UPLIFT);
	setHeight(10.f);
	run(3.f);
	ASSERT_EQ(out.state, TF::UPLIFT);
	// target 1.5 x 8 = 12 m from 10 m with TC 4 s: 0.5 m/s
	EXPECT_NEAR(out.height_rate, 0.5f, 0.05f);
	in.nav_alt = kTerrain + 13.9f;
	EXPECT_EQ(step().state, TF::UPLIFT);
	in.nav_alt = kTerrain + 14.1f;
	EXPECT_EQ(step().state, TF::STANDBY);
}

TEST_F(TerrainFollowerTest, NoUpliftAboveClearance)
{
	in.follow_wanted = false;
	in.guard = true;
	in.nav_alt = kTerrain + 12.1f;
	EXPECT_EQ(run(5.f).state, TF::STANDBY);
}

TEST_F(TerrainFollowerTest, UpliftYieldsToFollow)
{
	in.follow_wanted = false;
	in.guard = true;
	setHeight(15.f);
	in.nav_alt = kTerrain + 10.f;
	run(2.1f);
	ASSERT_EQ(out.state, TF::UPLIFT);
	in.follow_wanted = true;
	run(0.1f);
	EXPECT_EQ(out.state, TF::FOLLOW);
}

TEST_F(TerrainFollowerTest, RangeLostWithinHalfSecond)
{
	follow();
	in.raw_valid = false;
	in.hagl_raw = NAN;
	EXPECT_EQ(run(0.48f).state, TF::FOLLOW);
	EXPECT_EQ(run(0.04f).state, TF::RANGE_LOST);
	EXPECT_FALSE(out.use_rate);
	EXPECT_TRUE(out.cap_bank);
	// max(nav 130 m, 130 m at loss + 10 m) = 140 m
	EXPECT_NEAR(out.altitude, kTerrain + 40.f, 0.1f);
}

TEST_F(TerrainFollowerTest, RangeLostAboveNavAltitude)
{
	setHeight(25.f, kTerrain + 30.f);
	follow();
	in.raw_valid = false;
	run(0.6f);
	ASSERT_EQ(out.state, TF::RANGE_LOST);
	// aircraft 155 m, nav 130 m: max(130, 155 + 10) = 165 m
	EXPECT_NEAR(out.altitude, kTerrain + 65.f, 0.1f);
}

TEST_F(TerrainFollowerTest, RangeLostBelowNavAltitude)
{
	in.nav_alt = kTerrain + 60.f;
	follow();
	in.raw_valid = false;
	run(0.6f);
	ASSERT_EQ(out.state, TF::RANGE_LOST);
	EXPECT_NEAR(out.altitude, kTerrain + 60.f, 0.1f);
}

TEST_F(TerrainFollowerTest, RangeReturnsFollowAfterTwoSeconds)
{
	follow();
	in.raw_valid = false;
	run(0.6f);
	ASSERT_EQ(out.state, TF::RANGE_LOST);
	setHeight(30.f);
	EXPECT_EQ(run(1.9f).state, TF::RANGE_LOST);
	EXPECT_EQ(run(0.2f).state, TF::FOLLOW);
}

TEST_F(TerrainFollowerTest, RangeReturnsStandbyWhenFollowCleared)
{
	follow();
	in.raw_valid = false;
	run(0.6f);
	in.follow_wanted = false;
	in.guard = true;
	setHeight(30.f);
	EXPECT_EQ(run(2.1f).state, TF::STANDBY);
}

TEST_F(TerrainFollowerTest, SingleDroppedSampleNoDescent)
{
	follow();
	EXPECT_LT(out.height_rate, 0.f);
	in.raw_valid = false;
	EXPECT_EQ(step().state, TF::FOLLOW);
	EXPECT_GE(out.height_rate, 0.f);
}

TEST_F(TerrainFollowerTest, SingleLowSampleNoPullUp)
{
	follow();
	setHeight(5.f);
	step();
	setHeight(30.f);
	EXPECT_EQ(run(1.f).state, TF::FOLLOW);
}

TEST_F(TerrainFollowerTest, LowForPersistencePullsUp)
{
	p.ttg = 0.f;
	follow();
	setHeight(7.f);
	EXPECT_EQ(run(0.18f).state, TF::FOLLOW);
	EXPECT_EQ(run(0.04f).state, TF::PULL_UP);
	EXPECT_TRUE(out.use_rate);
	EXPECT_FLOAT_EQ(out.height_rate, p.climb_max);
	EXPECT_TRUE(out.min_airspeed);
	EXPECT_TRUE(out.cap_bank);
}

TEST_F(TerrainFollowerTest, StepUpActsInOneSample)
{
	follow();
	// a tree line: the lidar reads 22 m while the estimator still holds 30 m
	in.hagl_raw = 22.f;
	step();
	EXPECT_FLOAT_EQ(out.hagl_used, 22.f);
}

TEST_F(TerrainFollowerTest, DitchIgnoredWhileEstimatorHolds)
{
	follow();
	in.hagl_raw = 35.f;
	step();
	EXPECT_FLOAT_EQ(out.hagl_used, 30.f);
}

TEST_F(TerrainFollowerTest, ClosingPullsUpBeforeFloor)
{
	in.follow_wanted = false;
	in.guard = true;
	in.nav_alt = kTerrain + 200.f;
	run(2.1f);
	ASSERT_EQ(out.state, TF::STANDBY);
	float h = 30.f;
	float entry = NAN;

	for (int i = 0; i < 1000 && out.state != TF::PULL_UP; i++) {
		h -= 3.f * kDt;
		in.hagl_raw = h;
		in.hagl_ekf = h;
		step();
		entry = h;
	}

	ASSERT_EQ(out.state, TF::PULL_UP);
	// closing at up to 3 m/s with 3 s to go: entry above the 8 m floor
	EXPECT_GT(entry, p.floor + 1.f);
}

TEST_F(TerrainFollowerTest, PullUpRecoversToPreviousState)
{
	p.ttg = 0.f;
	follow();
	setHeight(7.f);
	run(0.3f);
	ASSERT_EQ(out.state, TF::PULL_UP);
	setHeight(13.f);
	EXPECT_EQ(run(0.9f).state, TF::PULL_UP);
	run(2.f);
	EXPECT_EQ(out.state, TF::FOLLOW);
}

TEST_F(TerrainFollowerTest, PullUpHoldsBelowUplift)
{
	p.ttg = 0.f;
	follow();
	setHeight(7.f);
	run(0.3f);
	setHeight(11.f);
	EXPECT_EQ(run(5.f).state, TF::PULL_UP);
}

TEST_F(TerrainFollowerTest, ReturnRequestedOnceAfterSixSeconds)
{
	p.ttg = 0.f;
	follow();
	setHeight(5.f);
	run(0.22f);
	ASSERT_EQ(out.state, TF::PULL_UP);
	int requests = 0;

	for (int i = 0; i < 280; i++) {
		requests += step().request_return;
	}

	EXPECT_EQ(requests, 0);

	for (int i = 0; i < 500; i++) {
		requests += step().request_return;
	}

	EXPECT_EQ(requests, 1);
	EXPECT_EQ(out.state, TF::PULL_UP);
}

TEST_F(TerrainFollowerTest, NoReturnRequestInReturn)
{
	p.ttg = 0.f;
	in.in_return = true;
	follow();
	setHeight(5.f);
	int requests = 0;

	for (int i = 0; i < 600; i++) {
		requests += step().request_return;
	}

	EXPECT_EQ(requests, 0);
}

TEST_F(TerrainFollowerTest, StuckLowSensorClimbsToTheCeilingOnly)
{
	// Mission under a ceiling 100 m over the ground: a reading stuck at 5 m holds the pull-up, and no fault ends it
	p.ttg = 0.f;
	in.ceiling = kTerrain + 100.f;
	follow();
	setHeight(5.f);
	run(0.3f);
	ASSERT_EQ(out.state, TF::PULL_UP);
	int requests = 0;

	for (int i = 0; i < 4000; i++) {
		in.alt_amsl += (out.use_rate ? out.height_rate : 0.f) * kDt;
		requests += step().request_return;
		ASSERT_EQ(out.state, TF::PULL_UP) << in.alt_amsl;
		ASSERT_GE(out.height_rate, 0.f) << in.alt_amsl;
	}

	// 80 s after the entry at 105 m: the climb has stopped at the ceiling, and the 6 s request was the one Return
	EXPECT_GT(in.alt_amsl, kTerrain + 98.f);
	EXPECT_LE(in.alt_amsl, kTerrain + 100.f);
	EXPECT_LT(out.height_rate, 0.1f);
	EXPECT_EQ(requests, 1);
}

TEST_F(TerrainFollowerTest, MissionPullUpClimbsPastFortyMetres)
{
	p.ttg = 0.f;
	follow();
	setHeight(5.f);
	run(0.3f);
	ASSERT_EQ(out.state, TF::PULL_UP);
	const float entry = in.alt_amsl;

	while (in.alt_amsl < entry + 60.f) {
		in.alt_amsl += 3.f * kDt;
		step();
	}

	EXPECT_EQ(out.state, TF::PULL_UP);
	EXPECT_FLOAT_EQ(out.height_rate, p.climb_max);
}

TEST_F(TerrainFollowerTest, CeilingHoldsTheGuardAndRequestsOneReturn)
{
	// Mission with the guard under a ceiling 20 m over the ground: UPLIFT wants 12 m over a crest 9 m high
	in.follow_wanted = false;
	in.guard = true;
	in.nav_alt = kTerrain + 5.f;
	in.ceiling = kTerrain + 20.f;
	setHeight(15.f);
	run(2.1f);
	ASSERT_EQ(out.state, TF::UPLIFT);
	int requests = 0;

	// 0.5 m under the ceiling and descending to 12 m over the plain: the bound does not hold, so no request
	setHeight(19.5f);

	for (int i = 0; i < 25; i++) {
		requests += step().request_return;
	}

	ASSERT_EQ(out.state, TF::UPLIFT);
	EXPECT_LT(out.height_rate, 0.f);
	EXPECT_EQ(requests, 0);
	// 2 m under the ceiling over a crest at 109 m: the bound slows the climb to 0.5 m/s, still short of the ceiling
	setHeight(9.f, kTerrain + 9.f);

	for (int i = 0; i < 5; i++) {
		requests += step().request_return;
	}

	ASSERT_EQ(out.state, TF::UPLIFT);
	EXPECT_NEAR(out.height_rate, 0.5f, 1e-3f);
	EXPECT_EQ(requests, 0);
	// 10.5 m over the crest and 0.5 m under the ceiling: (12 - 10.5) / 4 = 0.375 m/s wanted, (20 - 19.5) / 4 = 0.125 m/s
	setHeight(10.5f, kTerrain + 9.f);

	for (int i = 0; i < 150; i++) {
		requests += step().request_return;
	}

	EXPECT_EQ(out.state, TF::UPLIFT);
	EXPECT_NEAR(out.height_rate, 0.125f, 1e-3f);
	EXPECT_EQ(requests, 1);
	// in Return the bound holds the same way, with no request
	tf.reset();
	in.in_return = true;
	setHeight(15.f);
	run(2.1f);
	setHeight(10.5f, kTerrain + 9.f);
	requests = 0;

	for (int i = 0; i < 150; i++) {
		requests += step().request_return;
	}

	EXPECT_EQ(out.state, TF::UPLIFT);
	EXPECT_NEAR(out.height_rate, 0.125f, 1e-3f);
	EXPECT_EQ(requests, 0);
}

TEST_F(TerrainFollowerTest, PullUpFliesTheMinimumAirspeedAndTheClimbLimit)
{
	p.ttg = 0.f;
	follow();
	EXPECT_FALSE(out.min_airspeed);
	setHeight(5.f);
	run(0.3f);
	ASSERT_EQ(out.state, TF::PULL_UP);
	// FW_AIRSPD_MIN and FW_T_CLMB_MAX
	EXPECT_TRUE(out.min_airspeed);
	EXPECT_TRUE(out.use_rate);
	EXPECT_FLOAT_EQ(out.height_rate, p.climb_max);
	setHeight(13.f);
	run(1.1f);
	ASSERT_EQ(out.state, TF::FOLLOW);
	EXPECT_FALSE(out.min_airspeed);
}

TEST_F(TerrainFollowerTest, HoldKeepsThePullUpHeight)
{
	// guard in a Hold 20 m over the plain; one lap crosses a crest that leaves 7 m
	in.follow_wanted = false;
	in.guard = true;
	in.in_hold = true;
	in.nav_alt = kTerrain + 20.f;
	setHeight(20.f);
	run(2.1f);
	ASSERT_EQ(out.state, TF::STANDBY);
	EXPECT_TRUE(isnan(out.altitude));
	setHeight(7.f, kTerrain + 13.f);
	run(0.3f);
	ASSERT_EQ(out.state, TF::PULL_UP);
	// recovered 13 m over the crest at 126 m, inside the band UPLIFT holds
	setHeight(13.f, kTerrain + 13.f);
	run(1.1f);
	ASSERT_EQ(out.state, TF::UPLIFT);
	// the plain on the rest of the lap: the plan stays at the pull-up height
	setHeight(26.f);
	run(30.f);
	EXPECT_EQ(out.state, TF::STANDBY);
	EXPECT_NEAR(out.altitude, kTerrain + 26.f, 1e-3f);
	// the next crest: the guard counts from the kept height, 13 m over it and over the 1.5 x 8 = 12 m that lifts
	setHeight(13.f, kTerrain + 13.f);
	run(5.f);
	EXPECT_EQ(out.state, TF::STANDBY);
	// a Mission leg after the Hold flies its own altitude
	in.in_hold = false;
	setHeight(26.f);
	run(0.1f);
	EXPECT_TRUE(isnan(out.altitude));
}

TEST_F(TerrainFollowerTest, HoldHeightRaisesTheGuardTargets)
{
	// guard in a Hold 5 m over the plain; a pull-up from UPLIFT recovers 13 m over the ground
	in.follow_wanted = false;
	in.guard = true;
	in.in_hold = true;
	in.nav_alt = kTerrain + 5.f;
	setHeight(15.f);
	run(2.1f);
	ASSERT_EQ(out.state, TF::UPLIFT);
	setHeight(7.f);
	run(0.3f);
	ASSERT_EQ(out.state, TF::PULL_UP);
	setHeight(13.f);
	run(1.1f);
	ASSERT_EQ(out.state, TF::UPLIFT);
	// UPLIFT holds the kept 113 m, not the 112 m of 1.5 x 8 over the ground, once the 1 s terrain rate filter settles
	run(3.f);
	ASSERT_EQ(out.state, TF::UPLIFT);
	EXPECT_NEAR(out.height_rate, 0.f, 0.01f);
}

TEST_F(TerrainFollowerTest, FollowInHoldKeepsThePullUpHeight)
{
	// follow in a Hold planned at 130 m; a pull-up over a crest at 122 m recovers at 135 m, over the 120 m FW_TF_MAXDN allows
	p.ttg = 0.f;
	in.in_hold = true;
	follow();
	setHeight(7.f, kTerrain + 22.f);
	run(0.3f);
	ASSERT_EQ(out.state, TF::PULL_UP);
	setHeight(13.f, kTerrain + 22.f);
	run(1.1f);
	ASSERT_EQ(out.state, TF::FOLLOW);
	// over the plain the follow wants 25 m, 10 m down, and the kept height holds it level
	setHeight(35.f);
	run(0.5f);
	ASSERT_EQ(out.state, TF::FOLLOW);
	EXPECT_GE(out.height_rate, -0.01f);
}

TEST_F(TerrainFollowerTest, UpliftLossAboveTheBandFliesThePlan)
{
	// a guard climbing down from a soaring height: the beam leaves the ground 40 m under, over the 12 to 14 m UPLIFT holds
	in.follow_wanted = false;
	in.guard = true;
	in.nav_alt = kTerrain + 10.f;
	setHeight(15.f);
	run(2.1f);
	ASSERT_EQ(out.state, TF::UPLIFT);
	setHeight(40.f, kTerrain + 2.f);
	step();
	ASSERT_EQ(out.state, TF::UPLIFT);
	in.raw_valid = false;
	in.hagl_raw = NAN;
	EXPECT_EQ(run(0.6f).state, TF::STANDBY);
	EXPECT_FALSE(out.use_rate);
	EXPECT_TRUE(isnan(out.altitude));
	// inside the band the loss climbs
	tf.reset();
	setHeight(15.f);
	run(2.1f);
	ASSERT_EQ(out.state, TF::UPLIFT);
	setHeight(13.f);
	step();
	in.raw_valid = false;
	in.hagl_raw = NAN;
	EXPECT_EQ(run(0.6f).state, TF::RANGE_LOST);
}

TEST_F(TerrainFollowerTest, ReturnLossUnderTheFloorNeverFaults)
{
	// the last reading 7 m under the aircraft, then no range: no fault hands the altitude back to the Return
	in.follow_wanted = false;
	in.guard = true;
	in.in_return = true;
	in.nav_alt = kTerrain + 10.f;
	setHeight(15.f);
	run(2.1f);
	setHeight(13.f);
	run(0.1f);
	ASSERT_EQ(out.state, TF::UPLIFT);
	setHeight(7.f);
	step();
	in.raw_valid = false;
	in.hagl_raw = NAN;
	EXPECT_EQ(run(0.6f).state, TF::RANGE_LOST);
	EXPECT_EQ(run(60.f).state, TF::RANGE_LOST);
	EXPECT_GT(out.altitude, kTerrain + 10.f);
}

TEST_F(TerrainFollowerTest, OriginAltitudeChangeShiftsNothing)
{
	// a height reset of 5 m, then an origin moved 20 m up with z, the altitude above sea level unchanged
	EXPECT_FLOAT_EQ(TF::resetShift(5.f, 92.5f, 92.5f), -5.f);
	EXPECT_FLOAT_EQ(TF::resetShift(20.f, 112.5f, 92.5f), 0.f);
}

TEST_F(TerrainFollowerTest, SinkLimited)
{
	setHeight(80.f);
	follow();
	EXPECT_FLOAT_EQ(out.height_rate, -p.sink);
}

TEST_F(TerrainFollowerTest, CommandFloorWithTtgZero)
{
	p.ttg = 0.f;
	p.hagl = 10.f;
	follow();
	// floor 8 + max(2 x 0 + 1, 7) = 15 m
	EXPECT_GE(out.hagl_cmd, p.floor + 7.f);
}

TEST_F(TerrainFollowerTest, CommandClamps)
{
	follow();
	EXPECT_FLOAT_EQ(out.hagl_cmd, 25.f);
	p.hagl = 40.f;
	step();
	// 0.75 x 50 m x cos(35 deg) = 30.72 m
	EXPECT_NEAR(out.hagl_cmd, 30.72f, 0.01f);
	p.hagl = 10.f;
	step();
	// floor 8 + sink 2 x ttg 3 + 1 = 15 m
	EXPECT_FLOAT_EQ(out.hagl_cmd, 15.f);
}

TEST_F(TerrainFollowerTest, TenPercentRampSettlesWithinOneMetre)
{
	in.ekf_valid = false;
	setHeight(25.f);
	follow();
	float terrain = kTerrain;
	float v = 0.f;
	float worst = 0.f;

	// 18 m/s over a 10 percent ramp: terrain rises 1.8 m/s; the TECS response modelled as a 1.5 s lag
	for (int i = 0; i < static_cast<int>(60.f / kDt); i++) {
		terrain += 1.8f * kDt;
		in.hagl_raw = in.alt_amsl - terrain;
		in.vz_up = v;
		step();
		v += (out.height_rate - v) * kDt / 1.5f;
		in.alt_amsl += v * kDt;

		if (i * kDt > 40.f) {
			worst = fmaxf(worst, fabsf(in.alt_amsl - terrain - 25.f));
		}
	}

	EXPECT_EQ(out.state, TF::FOLLOW);
	EXPECT_LT(worst, 1.f);
}

TEST_F(TerrainFollowerTest, BitsSetOnGroundWaitForSwitch)
{
	EXPECT_FALSE(tf.latchFollow(true, false));
	EXPECT_FALSE(tf.latchFollow(true, true));
	EXPECT_FALSE(tf.latchFollow(false, true));
	EXPECT_TRUE(tf.latchFollow(true, true));
	EXPECT_TRUE(tf.latchFollow(true, true));
	EXPECT_FALSE(tf.latchFollow(true, false));
	EXPECT_FALSE(tf.latchFollow(true, true));
}

TEST_F(TerrainFollowerTest, FollowBits)
{
	EXPECT_TRUE(TF::followBit(1, TF::NAV_HOLD));
	EXPECT_FALSE(TF::followBit(2, TF::NAV_HOLD));
	EXPECT_TRUE(TF::followBit(2, TF::NAV_MISSION));
	EXPECT_FALSE(TF::followBit(1, TF::NAV_MISSION));
	EXPECT_FALSE(TF::followBit(7, TF::NAV_RETURN));
}

TEST_F(TerrainFollowerTest, SoaringSuspendsFollowAndKeepsTheGuard)
{
	// FW_TF_EN 7 in Hold and in Mission: soaring in CLIMB, GLIDE or THERMAL takes the follow bits, nothing else
	EXPECT_FALSE(TF::followBit(7, TF::NAV_HOLD, true));
	EXPECT_FALSE(TF::followBit(7, TF::NAV_MISSION, true));
	EXPECT_TRUE(TF::followBit(7, TF::NAV_HOLD, false));

	p.ttg = 0.f;
	in.guard = true;
	follow();
	in.follow_wanted = TF::followBit(7, TF::NAV_HOLD, true);
	EXPECT_EQ(step().state, TF::STANDBY);
	EXPECT_FALSE(out.use_rate);
	EXPECT_FALSE(out.cap_bank);
	EXPECT_FALSE(TF::guardActs(out.state));

	// the glide sinks to 7 m over rising ground: the guard pulls up after 0.2 s under the 8 m floor
	setHeight(7.f);
	EXPECT_EQ(run(0.18f).state, TF::STANDBY);
	EXPECT_EQ(run(0.04f).state, TF::PULL_UP);
	EXPECT_TRUE(TF::guardActs(out.state));
	EXPECT_FLOAT_EQ(out.height_rate, p.climb_max);

	// soaring ended: back over 1.5 x 8 = 12 m for 1 s the follower resumes by its own rule
	in.follow_wanted = TF::followBit(7, TF::NAV_HOLD, false);
	setHeight(20.f);
	run(1.1f);
	EXPECT_EQ(run(0.1f).state, TF::FOLLOW);
	EXPECT_FALSE(TF::guardActs(out.state));
}

TEST_F(TerrainFollowerTest, GuardActsInUpliftAndPullUpOnly)
{
	EXPECT_TRUE(TF::guardActs(TF::UPLIFT));
	EXPECT_TRUE(TF::guardActs(TF::PULL_UP));

	// FOLLOW must not end soaring: it is the state soaring starts from with a follow bit set
	for (uint8_t state : {TF::OFF, TF::STANDBY, TF::FOLLOW, TF::RANGE_LOST, TF::FAULT}) {
		EXPECT_FALSE(TF::guardActs(state)) << (int)state;
	}
}

TEST_F(TerrainFollowerTest, TiltCorrected)
{
	const float c30 = cosf(30.f * static_cast<float>(M_PI) / 180.f);
	// 30 deg bank 25 m over flat ground: the beam reads 25 / cos 30 = 28.87 m
	EXPECT_NEAR(TF::rawHagl(25.f / c30, c30), 25.f, 0.01f);
	EXPECT_TRUE(isnan(TF::rawHagl(30.f, cosf(41.f * static_cast<float>(M_PI) / 180.f))));
}

TEST_F(TerrainFollowerTest, FrozenSensorDropped)
{
	follow();
	TF::StuckCheck stuck;
	const float frozen = 30.f;
	const float alt_first = in.alt_amsl - 3.f * kDt;
	float t_dropped = -1.f;

	// a 10 Hz reading frozen while the follower descends at 3 m/s: 1 m of altitude passes in under 10 samples
	for (int i = 0; i < 200 && out.state != TF::RANGE_LOST; i++) {
		in.alt_amsl -= 3.f * kDt;
		in.raw_valid = stuck.live(frozen, in.alt_amsl, 1000000 + static_cast<uint64_t>(i / 5) * 100000);
		in.hagl_raw = in.raw_valid ? frozen : NAN;
		in.ekf_valid = false;

		if (!in.raw_valid && t_dropped < 0.f) {
			t_dropped = i * kDt;
			// dropped once the altitude has moved more than 1.0 m from the first sample, within one 0.06 m cycle
			EXPECT_GT(alt_first - in.alt_amsl, 1.f);
			EXPECT_LE(alt_first - in.alt_amsl, 1.f + 3.f * kDt + 1e-3f);
		}

		step();
	}

	ASSERT_GE(t_dropped, 0.f);
	ASSERT_EQ(out.state, TF::RANGE_LOST);
	// RANGE_LOST 0.5 s after the last live sample, within one cycle
	EXPECT_NEAR(static_cast<float>(in.now_us - 1000000) * 1e-6f - 2.1f - t_dropped, 0.5f, 2.f * kDt);
}

TEST_F(TerrainFollowerTest, FrozenLevelSensorDropped)
{
	TF::StuckCheck stuck;

	// level flight, each 50 Hz sample read on two cycles: the 10th bit-identical new sample is dropped
	for (int k = 1; k <= 10; k++) {
		const uint64_t t = 5000000 + static_cast<uint64_t>(k) * 20000;
		const bool first = stuck.live(25.f, 125.f, t);
		const bool again = stuck.live(25.f, 125.f, t);
		EXPECT_EQ(first, k < 10) << "sample " << k;
		EXPECT_EQ(again, k < 10) << "sample " << k;
	}

	EXPECT_FALSE(stuck.live(25.f, 125.f, 5000000 + 11 * 20000));
	EXPECT_TRUE(stuck.live(25.01f, 125.f, 5000000 + 12 * 20000));
}

TEST_F(TerrainFollowerTest, StuckReferenceFollowsDistance)
{
	TF::StuckCheck stuck;
	float alt = 130.f;

	// 3 s down at 1.8 m/s with a changing reading, then one repeat 0.036 m of altitude after its first sample
	for (int i = 0; i < 150; i++) {
		alt -= 1.8f * kDt;
		ASSERT_TRUE(stuck.live(25.f + 0.01f * i, alt, 1000000 + static_cast<uint64_t>(i) * 20000));
	}

	alt -= 1.8f * kDt;
	EXPECT_TRUE(stuck.live(25.f + 0.01f * 149, alt, 1000000 + 150 * 20000));
}

TEST_F(TerrainFollowerTest, LiveSensorOnTrackedSlope)
{
	TF::StuckCheck stuck;
	float alt = 130.f;
	uint32_t seed = 1;
	int dropped = 0;

	// 18 m/s down a 10 percent ramp at a constant 25 m: 1.8 m/s of altitude change, 0.02 m of noise per sample
	for (int i = 0; i < 3000; i++) {
		alt -= 1.8f * kDt;
		seed = seed * 1664525u + 1013904223u;
		const float noise = 0.02f * (static_cast<float>(seed >> 8) / 8388608.f - 1.f);
		dropped += !stuck.live(25.f + noise, alt, 1000000 + static_cast<uint64_t>(i) * 20000);
	}

	EXPECT_EQ(dropped, 0);
}

TEST_F(TerrainFollowerTest, DescentBounded)
{
	in.ekf_valid = false;
	in.nav_alt = kTerrain + 30.f;
	p.hagl = 22.f;
	// plan 30 m less FW_TF_MAXDN 10 m
	const float limit = 30.f - 10.f;
	float h = 30.f;
	float v = 0.f;
	int warnings = 0;
	float highest_warned = -INFINITY;
	bool bounded_above_band = false;
	float lowest_settled = INFINITY;

	// the range reads long_by m long; TECS modelled as a 1.5 s lag on the height rate
	auto fly = [&](float long_by, float seconds) {
		for (int i = 0; i < static_cast<int>(seconds / kDt); i++) {
			in.alt_amsl = kTerrain + h;
			in.hagl_raw = h + long_by;
			in.vz_up = v;
			step();
			bounded_above_band |= h > limit + 1.5f && fabsf(out.height_rate - (limit - h) / p.tc) < 0.01f;

			if (out.descent_limited) {
				warnings++;
				highest_warned = fmaxf(highest_warned, h);
			}

			v += ((out.use_rate ? out.height_rate : 0.f) - v) * kDt / 1.5f;
			h += v * kDt;

			if (i * kDt > 60.f) {
				lowest_settled = fminf(lowest_settled, h);
			}
		}
	};

	fly(15.f, 90.f);
	ASSERT_EQ(out.state, TF::FOLLOW);
	EXPECT_GE(lowest_settled, limit - 0.1f);
	// the bound sets the rate more than 1.5 m above the limit, the Warning fires within 1 m of it
	EXPECT_TRUE(bounded_above_band);
	EXPECT_EQ(warnings, 1);
	EXPECT_LE(highest_warned, limit + 1.f);
	// a true range climbs to the 22 m command, then the long range brings the aircraft back to the limit
	fly(0.f, 30.f);
	EXPECT_GT(h, limit + 1.f);
	EXPECT_EQ(warnings, 1);
	fly(15.f, 60.f);
	EXPECT_EQ(warnings, 2);
	EXPECT_LE(highest_warned, limit + 1.f);
}

TEST_F(TerrainFollowerTest, NotReleasedAfterReset)
{
	in.follow_wanted = false;
	in.guard = true;
	setHeight(7.f);
	in.nav_alt = kTerrain + 7.f;
	run(3.1f);
	EXPECT_EQ(out.state, TF::STANDBY);
	EXPECT_FALSE(out.cap_bank);
	// 1.5 x 8 + 2 = 14 m releases the guard
	setHeight(14.f);
	run(0.1f);
	setHeight(7.f);
	run(0.3f);
	EXPECT_EQ(out.state, TF::PULL_UP);
}

TEST_F(TerrainFollowerTest, LandingContextCancels)
{
	in.follow_wanted = false;
	in.guard = true;
	in.nav_alt = kTerrain + 200.f;
	float h = 30.f;
	setHeight(h);
	run(2.1f);
	ASSERT_EQ(out.state, TF::STANDBY);

	// closing at 1.5 m/s down to 13 m: released, between the 14 m release and the 12.5 m closing trip
	while (h > 13.f) {
		h -= 1.5f * kDt;
		setHeight(h);
		step();
	}

	ASSERT_EQ(out.state, TF::STANDBY);
	in.landing = true;
	EXPECT_EQ(step().state, TF::OFF);
	in.landing = false;
	// the go-around Hold at 10 m
	in.nav_alt = kTerrain + 10.f;
	bool pulled = false;
	bool lifted = false;
	bool capped = false;

	while (h > 9.f) {
		h -= 1.5f * kDt;
		setHeight(h);
		step();
		pulled |= out.state == TF::PULL_UP;
		lifted |= out.state == TF::UPLIFT;
		capped |= out.cap_bank;
	}

	EXPECT_EQ(out.state, TF::STANDBY);
	EXPECT_FALSE(pulled);
	EXPECT_FALSE(lifted);
	EXPECT_FALSE(capped);

	while (h < 14.5f) {
		h += 1.5f * kDt;
		setHeight(h);
		step();
	}

	run(0.1f);
	// nav 10 m under 1.5 x 8 = 12 m: the guard lifts again
	EXPECT_EQ(out.state, TF::UPLIFT);
}

TEST_F(TerrainFollowerTest, RangeLostLowSample)
{
	follow();
	in.raw_valid = false;
	run(0.6f);
	ASSERT_EQ(out.state, TF::RANGE_LOST);
	setHeight(7.f);
	run(0.3f);
	EXPECT_FALSE(tf.rangeOk());
	EXPECT_EQ(out.state, TF::PULL_UP);
}

TEST_F(TerrainFollowerTest, RangeLostClosing)
{
	follow();
	in.raw_valid = false;
	run(0.6f);
	ASSERT_EQ(out.state, TF::RANGE_LOST);
	float h = 16.f;

	// rising ground under the regained beam, closing at 3 m/s above the floor
	for (int i = 0; i < 90 && out.state != TF::PULL_UP; i++) {
		h -= 3.f * kDt;
		setHeight(h);
		step();
	}

	EXPECT_FALSE(tf.rangeOk());
	EXPECT_EQ(out.state, TF::PULL_UP);
	EXPECT_GE(h, p.floor);
}

TEST_F(TerrainFollowerTest, RangeLostTargetRises)
{
	follow();
	in.raw_valid = false;
	run(0.6f);
	ASSERT_EQ(out.state, TF::RANGE_LOST);
	// climbed to 136 m during the loss; the beam returns at 15 m over ground at 121 m
	setHeight(15.f, kTerrain + 21.f);
	step();
	ASSERT_EQ(out.state, TF::RANGE_LOST);
	// max(nav 130, loss 130 + 10, 136 - 15 + 25 = 146)
	EXPECT_NEAR(out.altitude, kTerrain + 46.f, 0.1f);
}

TEST_F(TerrainFollowerTest, AcquisitionJumpRestarts)
{
	run(1.5f);
	// 0.2 x 50 m = 10 m
	setHeight(42.f);
	EXPECT_EQ(run(1.98f).state, TF::STANDBY);
	EXPECT_EQ(run(0.1f).state, TF::FOLLOW);
}

TEST_F(TerrainFollowerTest, SingleLowSampleWhileClosing)
{
	in.follow_wanted = false;
	in.guard = true;
	in.nav_alt = kTerrain + 200.f;
	float h = 30.f;
	run(2.1f);
	ASSERT_EQ(out.state, TF::STANDBY);

	// closing at 1 m/s trips the closing rule under 8 + 1 x 3 = 11 m, so stop at 15 m
	for (int i = 0; h > 15.f; i++) {
		h -= 1.f * kDt;
		setHeight(i == 600 ? 5.f : h);
		step();
		ASSERT_NE(out.state, TF::PULL_UP);
	}
}

TEST_F(TerrainFollowerTest, RateFromSampleInterval)
{
	in.eligible = false;
	in.ekf_valid = false;
	float hagl = 40.f;
	uint64_t t_sample = 0;
	uint64_t t_next = in.now_us;

	// level flight, ground rising 3.6 m/s, samples at 20 Hz over 50 Hz cycles
	for (int i = 0; i < static_cast<int>(4.f / kDt); i++) {
		in.now_us += static_cast<uint64_t>(kDt * 1e6f);

		if (in.now_us >= t_next) {
			t_sample = t_next;
			t_next += 50000;
			hagl = 40.f - 3.6f * static_cast<float>(t_sample - 1000000) * 1e-6f;
		}

		in.range_time_us = t_sample;
		in.raw_valid = true;
		in.hagl_raw = hagl;
		in.vz_up = 0.f;
		out = tf.update(in, p);
	}

	EXPECT_NEAR(out.terrain_rate, 3.6f, 0.3f);
}

TEST_F(TerrainFollowerTest, NotReleasedDuringAcquisition)
{
	in.follow_wanted = false;
	in.guard = true;
	in.nav_alt = kTerrain + 7.f;
	// 15 m for 1 s, under the 2 s that make range_ok, then 7 m: the 14 m release needs range_ok
	setHeight(15.f);
	run(1.f);
	setHeight(7.f);
	run(3.f);
	EXPECT_EQ(out.state, TF::STANDBY);
	EXPECT_FALSE(out.cap_bank);
}

TEST_F(TerrainFollowerTest, RangeLostGuardTargetIsUplift)
{
	in.follow_wanted = false;
	in.guard = true;
	in.nav_alt = kTerrain + 10.f;
	setHeight(15.f);
	run(2.1f);
	// lost inside the 12 to 14 m band UPLIFT holds
	setHeight(13.f);
	step();
	ASSERT_EQ(out.state, TF::UPLIFT);
	in.raw_valid = false;
	run(0.6f);
	ASSERT_EQ(out.state, TF::RANGE_LOST);
	// lost at 113 m; the beam returns at 10 m over ground at 120 m
	setHeight(10.f, kTerrain + 20.f);
	step();
	ASSERT_EQ(out.state, TF::RANGE_LOST);
	// max(nav 110, loss 113 + 10, 130 - 10 + 1.5 x 8 = 132)
	EXPECT_NEAR(out.altitude, kTerrain + 32.f, 0.1f);
}

TEST_F(TerrainFollowerTest, AcquisitionJumpDownRestarts)
{
	run(1.5f);
	// 30 m to 18 m: a 12 m drop over the 0.2 x 50 m = 10 m jump
	setHeight(18.f);
	EXPECT_EQ(run(1.98f).state, TF::STANDBY);
	EXPECT_EQ(run(0.1f).state, TF::FOLLOW);
}

TEST_F(TerrainFollowerTest, ClosingEntryReturnsAfterSixSecondsUnder)
{
	in.follow_wanted = false;
	in.guard = true;
	in.nav_alt = kTerrain + 200.f;
	run(2.1f);
	ASSERT_EQ(out.state, TF::STANDBY);
	float h = 30.f;

	// closing at 3 m/s enters PULL_UP above the floor
	while (out.state != TF::PULL_UP && h > p.floor) {
		h -= 3.f * kDt;
		setHeight(h);
		step();
	}

	ASSERT_EQ(out.state, TF::PULL_UP);
	ASSERT_GE(h, p.floor);
	setHeight(5.f);
	int requests = 0;

	// 6 s under the floor counts from the first cycle under it
	for (int i = 0; i < 295; i++) {
		requests += step().request_return;
	}

	EXPECT_EQ(requests, 0);

	for (int i = 0; i < 15; i++) {
		requests += step().request_return;
	}

	EXPECT_EQ(requests, 1);
}

TEST_F(TerrainFollowerTest, PullUpHoldsWhileClosing)
{
	p.ttg = 0.f;
	follow();
	setHeight(7.f);
	run(0.3f);
	ASSERT_EQ(out.state, TF::PULL_UP);
	float h = 20.f;
	setHeight(h);
	step();

	// above 1.5 x 8 = 12 m for 3 s, still closing at 1 m/s
	for (int i = 0; i < 150; i++) {
		h -= 1.f * kDt;
		setHeight(h);
		step();
	}

	EXPECT_EQ(out.state, TF::PULL_UP);
}

TEST_F(TerrainFollowerTest, UpliftKeepsPlannedHeight)
{
	in.follow_wanted = false;
	in.guard = true;
	setHeight(15.f);
	in.nav_alt = kTerrain + 10.f;
	run(2.1f);
	ASSERT_EQ(out.state, TF::UPLIFT);
	setHeight(10.f);
	// plan 13.5 m above ground, inside the 12 to 14 m band UPLIFT holds: (13.5 - 10) / 4 = 0.875 m/s
	in.nav_alt = kTerrain + 13.5f;
	run(3.f);
	ASSERT_EQ(out.state, TF::UPLIFT);
	EXPECT_NEAR(out.height_rate, 0.875f, 0.05f);
}

TEST_F(TerrainFollowerTest, ReturnRangeLossFaultsAfterThirtySeconds)
{
	in.follow_wanted = false;
	in.guard = true;
	in.in_return = true;
	in.nav_alt = kTerrain + 10.f;
	setHeight(15.f);
	run(2.1f);
	// lost inside the 12 to 14 m band UPLIFT holds
	setHeight(13.f);
	step();
	ASSERT_EQ(out.state, TF::UPLIFT);
	in.raw_valid = false;
	in.hagl_raw = NAN;
	EXPECT_EQ(run(0.6f).state, TF::RANGE_LOST);
	// 30 s from the last valid sample
	EXPECT_EQ(run(29.3f).state, TF::RANGE_LOST);
	EXPECT_EQ(run(0.2f).state, TF::FAULT);
	EXPECT_FALSE(out.use_rate);
	EXPECT_TRUE(isnan(out.altitude));
	EXPECT_FALSE(out.cap_bank);
	// range back: off for the rest of the Return
	setHeight(15.f);
	EXPECT_EQ(run(5.f).state, TF::FAULT);
}

TEST_F(TerrainFollowerTest, RangeLostBeforeReturnFaultsInReturn)
{
	in.follow_wanted = false;
	in.guard = true;
	in.nav_alt = kTerrain + 10.f;
	setHeight(15.f);
	run(2.1f);
	// lost inside the 12 to 14 m band UPLIFT holds
	setHeight(13.f);
	step();
	ASSERT_EQ(out.state, TF::UPLIFT);
	in.raw_valid = false;
	in.hagl_raw = NAN;
	EXPECT_EQ(run(10.f).state, TF::RANGE_LOST);
	// the Hold to Return switch: 0.06 s of IDLE setpoint, with no altitude
	in.eligible = false;
	in.no_setpoint = true;
	in.in_return = true;
	in.nav_alt = NAN;
	EXPECT_EQ(run(0.06f).state, TF::RANGE_LOST);
	in.eligible = true;
	in.no_setpoint = false;
	in.nav_alt = kTerrain + 10.f;
	// 30 s from the last valid sample: 10 + 0.06 + 19.84 = 29.9 s
	EXPECT_EQ(run(19.84f).state, TF::RANGE_LOST);
	EXPECT_EQ(run(0.2f).state, TF::FAULT);
}

TEST_F(TerrainFollowerTest, UpliftAndPullUpHoldThroughSetpointGap)
{
	in.follow_wanted = false;
	in.guard = true;
	in.nav_alt = kTerrain + 10.f;
	setHeight(15.f);
	run(2.1f);
	ASSERT_EQ(out.state, TF::UPLIFT);
	in.eligible = false;
	in.no_setpoint = true;
	in.nav_alt = NAN;
	EXPECT_EQ(run(0.1f).state, TF::UPLIFT);
	in.eligible = true;
	in.no_setpoint = false;
	in.nav_alt = kTerrain + 10.f;
	EXPECT_EQ(step().state, TF::UPLIFT);
	setHeight(7.f);
	run(0.3f);
	ASSERT_EQ(out.state, TF::PULL_UP);
	in.eligible = false;
	in.no_setpoint = true;
	in.nav_alt = NAN;
	EXPECT_EQ(run(0.1f).state, TF::PULL_UP);
}

TEST_F(TerrainFollowerTest, SetpointGapOfHalfSecondTurnsOff)
{
	follow();
	in.eligible = false;
	in.no_setpoint = true;
	EXPECT_EQ(run(0.48f).state, TF::FOLLOW);
	EXPECT_EQ(run(0.04f).state, TF::OFF);
}

TEST_F(TerrainFollowerTest, MissionRangeLossHoldsPastThirtySeconds)
{
	follow();
	in.raw_valid = false;
	in.hagl_raw = NAN;
	EXPECT_EQ(run(60.f).state, TF::RANGE_LOST);
}

TEST_F(TerrainFollowerTest, ApproachCircleFlownIntoLatches)
{
	TF::ApproachLatch latch;
	// the approach loiter appears far away, then the aircraft reaches its circle
	EXPECT_FALSE(latch.update(true, true, false, 45.309, -75.747));
	EXPECT_TRUE(latch.update(true, true, true, 45.309, -75.747));
	// the final waypoint leg outside any circle stays a landing context
	EXPECT_TRUE(latch.update(true, true, false, 45.3092, -75.749));
	// the mission index back at or below DO_LAND_START ends it
	EXPECT_FALSE(latch.update(false, true, false, 45.3092, -75.749));
	EXPECT_FALSE(latch.update(true, true, false, 45.3092, -75.749));
}

TEST_F(TerrainFollowerTest, LoiterAroundTheAircraftDoesNotLatch)
{
	TF::ApproachLatch latch;
	EXPECT_FALSE(latch.update(true, true, false, 45.309, -75.747));
	// a Return climb loiter centred where the aircraft is
	for (int i = 0; i < 100; i++) {
		EXPECT_FALSE(latch.update(true, true, true, 45.312, -75.7546));
	}

	// its altitude republished on the same centre
	EXPECT_FALSE(latch.update(true, true, true, 45.312, -75.7546));
	// the approach loiter flown into afterwards latches
	EXPECT_FALSE(latch.update(true, true, false, 45.309, -75.747));
	EXPECT_TRUE(latch.update(true, true, true, 45.309, -75.747));
}

TEST_F(TerrainFollowerTest, SetpointOlderThanMissionIndexDoesNotLatch)
{
	TF::ApproachLatch latch;
	// the Hold circle flown into, then the mission index moves past DO_LAND_START before the new setpoint arrives
	EXPECT_FALSE(latch.update(false, true, false, 45.312, -75.755));
	EXPECT_FALSE(latch.update(false, true, true, 45.312, -75.755));
	EXPECT_FALSE(latch.update(true, false, true, 45.312, -75.755));
	EXPECT_FALSE(latch.update(true, true, false, 45.309, -75.747));
}

TEST_F(TerrainFollowerTest, LandNextIsTheLandingFromTheCircleOn)
{
	// the firmware pattern: the orbit current, LAND next, kilometres from the circle or on it
	EXPECT_FALSE(TF::landingContext(false, true, true, false));
	EXPECT_TRUE(TF::landingContext(false, true, true, true));
	// a final waypoint ahead of LAND, and LAND itself
	EXPECT_TRUE(TF::landingContext(false, true, false, false));
	EXPECT_TRUE(TF::landingContext(true, false, false, false));
	EXPECT_FALSE(TF::landingContext(false, false, true, true));
	EXPECT_FALSE(TF::landingContext(false, false, false, false));
}

TEST_F(TerrainFollowerTest, PullUpRunsToRecoveryInItsReturn)
{
	// follow in Hold without the guard bit: the Return the pull-up requests wants neither follow nor guard
	p.ttg = 0.f;
	follow();
	setHeight(5.f);
	run(0.3f);
	ASSERT_EQ(out.state, TF::PULL_UP);
	in.follow_wanted = false;
	in.in_return = true;
	EXPECT_EQ(run(2.f).state, TF::PULL_UP);
	EXPECT_FLOAT_EQ(out.height_rate, p.climb_max);
	// 1.5 x 8 = 12 m without closure for 1 s recovers, and the Return then flies its own altitude
	setHeight(13.f);
	run(1.1f);
	EXPECT_EQ(out.state, TF::OFF);
	EXPECT_FALSE(out.use_rate);
}

TEST_F(TerrainFollowerTest, ReturnLossFaultsThroughRangeBursts)
{
	in.follow_wanted = false;
	in.guard = true;
	in.in_return = true;
	in.nav_alt = kTerrain + 10.f;
	setHeight(15.f);
	run(2.1f);
	// lost inside the 12 to 14 m band UPLIFT holds
	setHeight(13.f);
	step();
	ASSERT_EQ(out.state, TF::UPLIFT);
	in.raw_valid = false;
	in.hagl_raw = NAN;
	EXPECT_EQ(run(0.6f).state, TF::RANGE_LOST);

	// a lidar at the edge of its range: 1.5 s of valid samples every 10 s, under the 2 s that reacquire
	for (int i = 0; i < 2; i++) {
		run(8.5f);
		setHeight(15.f);
		run(1.5f);
		in.raw_valid = false;
		in.hagl_raw = NAN;
	}

	// 0.6 + 2 x 10 + 9.3 = 29.9 s from the last sample before the loss
	EXPECT_EQ(run(9.3f).state, TF::RANGE_LOST);
	EXPECT_EQ(run(0.2f).state, TF::FAULT);
}

TEST_F(TerrainFollowerTest, ReturnPullUpClimbsPastFortyMetres)
{
	// a ridge rising under the Return 5 m below the aircraft: FAULT would hand the altitude back under the crest
	p.ttg = 0.f;
	in.follow_wanted = false;
	in.guard = true;
	in.in_return = true;
	in.nav_alt = kTerrain + 10.f;
	setHeight(15.f);
	run(2.1f);
	ASSERT_EQ(out.state, TF::UPLIFT);
	setHeight(5.f);
	run(0.3f);
	ASSERT_EQ(out.state, TF::PULL_UP);
	const float entry = in.alt_amsl;

	while (in.alt_amsl < entry + 60.f) {
		in.alt_amsl += 3.f * kDt;
		step();
	}

	EXPECT_EQ(out.state, TF::PULL_UP);
	EXPECT_FLOAT_EQ(out.height_rate, p.climb_max);
}

TEST_F(TerrainFollowerTest, FenceCeilingBoundsTheClimbInEveryMode)
{
	p.ttg = 0.f;
	in.follow_wanted = false;
	in.guard = true;
	in.in_return = true;
	in.nav_alt = kTerrain + 10.f;
	in.ceiling = kTerrain + 30.f;
	setHeight(15.f);
	run(2.1f);
	ASSERT_EQ(out.state, TF::UPLIFT);
	setHeight(5.f);
	run(0.3f);
	ASSERT_EQ(out.state, TF::PULL_UP);
	// 10 m under the ceiling: (130 - 120) / 4 = 2.5 m/s, under the 3 m/s climb
	setHeight(5.f, kTerrain + 15.f);
	step();
	EXPECT_NEAR(out.height_rate, 2.5f, 1e-3f);
	// at the ceiling the climb stops and the guard stays
	setHeight(5.f, kTerrain + 25.f);
	EXPECT_EQ(run(1.f).state, TF::PULL_UP);
	EXPECT_NEAR(out.height_rate, 0.f, 1e-3f);
	// in Mission and Hold the bound holds the same way
	in.in_return = false;
	EXPECT_EQ(step().state, TF::PULL_UP);
	EXPECT_NEAR(out.height_rate, 0.f, 1e-3f);
}

TEST_F(TerrainFollowerTest, HeightResetKeepsTheLossClimb)
{
	follow();
	in.raw_valid = false;
	in.hagl_raw = NAN;
	run(0.6f);
	ASSERT_EQ(out.state, TF::RANGE_LOST);
	// lost at 130 m, target 140 m; a 5 m reset raises the estimate, not the aircraft
	in.alt_amsl += 5.f;
	tf.heightReset(5.f);
	step();
	EXPECT_NEAR(out.altitude, kTerrain + 45.f, 0.01f);
}

TEST_F(TerrainFollowerTest, HeightResetKeepsTheHoldHeight)
{
	in.follow_wanted = false;
	in.guard = true;
	in.in_hold = true;
	in.nav_alt = kTerrain + 20.f;
	setHeight(20.f);
	run(2.1f);
	setHeight(7.f, kTerrain + 13.f);
	run(0.3f);
	ASSERT_EQ(out.state, TF::PULL_UP);
	setHeight(13.f, kTerrain + 13.f);
	run(1.1f);
	setHeight(26.f);
	run(1.f);
	ASSERT_NEAR(out.altitude, kTerrain + 26.f, 1e-3f);
	// a 5 m reset raises the estimate, not the aircraft
	in.alt_amsl += 5.f;
	tf.heightReset(5.f);
	step();
	EXPECT_NEAR(out.altitude, kTerrain + 31.f, 1e-3f);
}

TEST_F(TerrainFollowerTest, HeightResetKeepsTheHeightWithoutRange)
{
	follow();
	in.raw_valid = false;
	in.hagl_raw = NAN;
	step();
	ASSERT_FLOAT_EQ(out.hagl_used, 30.f);
	in.alt_amsl += 5.f;
	tf.heightReset(5.f);
	step();
	EXPECT_FLOAT_EQ(out.hagl_used, 30.f);
}

TEST_F(TerrainFollowerTest, CeilingBoundNeverDescendsInAPullUp)
{
	// Return under a 130 m ceiling, UPLIFT held at the ceiling over ground at 118 m
	p.ttg = 0.f;
	in.follow_wanted = false;
	in.guard = true;
	in.in_return = true;
	in.nav_alt = kTerrain + 10.f;
	in.ceiling = kTerrain + 30.f;
	setHeight(15.f);
	run(2.1f);
	ASSERT_EQ(out.state, TF::UPLIFT);
	setHeight(12.f, kTerrain + 18.f);
	run(1.f);
	ASSERT_EQ(out.state, TF::UPLIFT);
	// range lost at 130 m; lift then carries the aircraft to 140 m, 10 m over the ceiling
	in.raw_valid = false;
	in.hagl_raw = NAN;
	run(0.6f);
	ASSERT_EQ(out.state, TF::RANGE_LOST);
	in.alt_amsl = kTerrain + 40.f;
	run(1.f);
	// range back over a 133 m crest: 7 m under the aircraft, under the 8 m floor
	setHeight(7.f, kTerrain + 33.f);
	run(0.3f);
	ASSERT_EQ(out.state, TF::PULL_UP);
	EXPECT_GE(out.height_rate, 0.f);
}

TEST_F(TerrainFollowerTest, CeilingBoundKeepsTheSinkLimit)
{
	p.ttg = 0.f;
	in.follow_wanted = false;
	in.guard = true;
	in.in_return = true;
	in.nav_alt = kTerrain + 10.f;
	in.ceiling = kTerrain + 30.f;
	setHeight(15.f);
	run(2.1f);
	ASSERT_EQ(out.state, TF::UPLIFT);
	// 20 m over the ceiling with the ground 18 m under: UPLIFT wants -1.5 m/s, FW_TF_SINK is 2
	setHeight(18.f, kTerrain + 32.f);
	run(0.5f);
	ASSERT_EQ(out.state, TF::UPLIFT);
	EXPECT_GE(out.height_rate, -p.sink);
}

TEST_F(TerrainFollowerTest, LossClimbEndsAtTheCeiling)
{
	// Return under a 130 m ceiling: UPLIFT at 126 m over ground at 116 m, 2 m under its 12 m target
	p.ttg = 0.f;
	in.follow_wanted = false;
	in.guard = true;
	in.in_return = true;
	in.nav_alt = kTerrain + 10.f;
	in.ceiling = kTerrain + 30.f;
	setHeight(15.f);
	run(2.1f);
	ASSERT_EQ(out.state, TF::UPLIFT);
	setHeight(10.f, kTerrain + 16.f);
	run(1.f);
	ASSERT_EQ(out.state, TF::UPLIFT);
	// range lost at 126 m: FW_TF_LOSS_UP 10 asks for 136 m, the ceiling gives 130 m
	in.raw_valid = false;
	in.hagl_raw = NAN;
	run(0.6f);
	ASSERT_EQ(out.state, TF::RANGE_LOST);
	EXPECT_NEAR(out.altitude, kTerrain + 30.f, 0.01f);
	// lift carries the aircraft to 133 m: the setpoint is that height, no descent
	in.alt_amsl = kTerrain + 33.f;
	step();
	EXPECT_NEAR(out.altitude, kTerrain + 33.f, 0.01f);
	// a plan at 135 m, above the ceiling, is not lowered
	in.alt_amsl = kTerrain + 26.f;
	in.nav_alt = kTerrain + 35.f;
	step();
	EXPECT_NEAR(out.altitude, kTerrain + 35.f, 0.01f);
	// under a 150 m ceiling the 136 m of the loss climb stand
	in.nav_alt = kTerrain + 10.f;
	in.ceiling = kTerrain + 50.f;
	step();
	EXPECT_NEAR(out.altitude, kTerrain + 36.f, 0.01f);
}

TEST_F(TerrainFollowerTest, LossClimbIntoTheCeilingFliesTheGuardsRate)
{
	// Mission follow at 30 m over the plain, 130 m, under a 150 m ceiling; FW_TF_LOSS_UP 20 asks for 150 m
	p.loss_up = 20.f;
	in.ceiling = kTerrain + 50.f;
	follow();
	in.raw_valid = false;
	in.hagl_raw = NAN;
	run(0.6f);
	ASSERT_EQ(out.state, TF::RANGE_LOST);
	// (150 - 130) / 4 = 5 m/s, held to FW_T_CLMB_MAX 3
	EXPECT_TRUE(out.use_rate);
	EXPECT_FLOAT_EQ(out.height_rate, p.climb_max);
	// for 60 s the rate is the pull-up's, FW_T_CLMB_MAX under the bound (ceiling - height) / FW_TF_TC, and no Return is asked
	float worst = 0.f;
	float peak = in.alt_amsl;
	bool returned = false;

	for (int i = 0; i < 3000; i++) {
		in.alt_amsl += (out.use_rate ? out.height_rate : 0.f) * kDt;
		peak = fmaxf(peak, in.alt_amsl);
		step();
		worst = fmaxf(worst, fabsf(out.height_rate - fminf(p.climb_max, fmaxf((in.ceiling - in.alt_amsl) / p.tc, 0.f))));
		returned |= out.request_return;
	}

	EXPECT_EQ(out.state, TF::RANGE_LOST);
	EXPECT_LT(worst, 1e-4f);
	EXPECT_LE(peak, in.ceiling);
	EXPECT_GT(peak, in.ceiling - 0.5f);
	EXPECT_FALSE(returned);
}

TEST_F(TerrainFollowerTest, LossClimbUnderTheCeilingFliesTheRateOnlyNearIt)
{
	in.ceiling = kTerrain + 40.5f;
	follow();
	in.raw_valid = false;
	in.hagl_raw = NAN;
	run(0.6f);
	ASSERT_EQ(out.state, TF::RANGE_LOST);
	// 140 m, 0.5 m under the ceiling, is inside its slow-down of 3 x 4 = 12 m: (140 - 130) / 4 = 2.5 m/s
	EXPECT_TRUE(out.use_rate);
	EXPECT_NEAR(out.height_rate, 2.5f, 1e-3f);
	// 140 m under a 152.5 m ceiling is outside it: TECS flies the altitude
	in.ceiling = kTerrain + 52.5f;
	step();
	EXPECT_FALSE(out.use_rate);
	EXPECT_NEAR(out.altitude, kTerrain + 40.f, 0.01f);
	// a plan above the ceiling is the altitude, not the loss climb
	in.ceiling = kTerrain + 35.f;
	in.nav_alt = kTerrain + 45.f;
	step();
	EXPECT_FALSE(out.use_rate);
	EXPECT_NEAR(out.altitude, kTerrain + 45.f, 0.01f);
}
