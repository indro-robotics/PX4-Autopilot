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

	TF::Output step()
	{
		in.now_us += static_cast<uint64_t>(kDt * 1e6f);
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
	TF::Params p{25.f, 8.f, 4.f, 2.f, 3.f, 10.f, 3.f, 35.f * static_cast<float>(M_PI) / 180.f};
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
	setHeight(10.f);
	in.nav_alt = kTerrain + 10.f;
	run(2.1f);
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
	setHeight(10.f);
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

TEST_F(TerrainFollowerTest, StuckLowSensorFaultsAfterFortyMetres)
{
	p.ttg = 0.f;
	follow();
	setHeight(5.f);
	run(0.3f);
	ASSERT_EQ(out.state, TF::PULL_UP);
	const float entry = in.alt_amsl;

	while (in.alt_amsl < entry + 39.f) {
		in.alt_amsl += 3.f * kDt;
		step();
	}

	EXPECT_EQ(out.state, TF::PULL_UP);

	while (in.alt_amsl < entry + 41.f) {
		in.alt_amsl += 3.f * kDt;
		step();
	}

	EXPECT_EQ(out.state, TF::FAULT);
	EXPECT_FALSE(out.use_rate);
	EXPECT_FALSE(out.cap_bank);
	in.eligible = false;
	EXPECT_EQ(step().state, TF::FAULT);
	tf.reset();
	EXPECT_EQ(step().state, TF::OFF);
}

TEST_F(TerrainFollowerTest, SinkLimited)
{
	setHeight(80.f);
	follow();
	EXPECT_FLOAT_EQ(out.height_rate, -p.sink);
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

TEST_F(TerrainFollowerTest, Eligibility)
{
	EXPECT_TRUE(TF::eligible(true, false, TF::NAV_HOLD, false));
	EXPECT_TRUE(TF::eligible(true, false, TF::NAV_MISSION, false));
	EXPECT_TRUE(TF::eligible(true, false, TF::NAV_RETURN, false));
	EXPECT_TRUE(TF::eligible(true, false, TF::NAV_HOLD, true));
	EXPECT_FALSE(TF::eligible(false, false, TF::NAV_HOLD, false));
	EXPECT_FALSE(TF::eligible(true, true, TF::NAV_HOLD, false));
	EXPECT_FALSE(TF::eligible(true, false, TF::NAV_MISSION, true));
	EXPECT_FALSE(TF::eligible(true, false, TF::NAV_OTHER, false));
}

TEST_F(TerrainFollowerTest, FollowBits)
{
	EXPECT_TRUE(TF::followBit(1, TF::NAV_HOLD));
	EXPECT_FALSE(TF::followBit(2, TF::NAV_HOLD));
	EXPECT_TRUE(TF::followBit(2, TF::NAV_MISSION));
	EXPECT_FALSE(TF::followBit(1, TF::NAV_MISSION));
	EXPECT_FALSE(TF::followBit(7, TF::NAV_RETURN));
}

TEST_F(TerrainFollowerTest, TiltCorrected)
{
	const float c30 = cosf(30.f * static_cast<float>(M_PI) / 180.f);
	// 30 deg bank 25 m over flat ground: the beam reads 25 / cos 30 = 28.87 m
	EXPECT_NEAR(TF::rawHagl(25.f / c30, c30), 25.f, 0.01f);
	EXPECT_TRUE(isnan(TF::rawHagl(30.f, cosf(41.f * static_cast<float>(M_PI) / 180.f))));
}
