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
#include <gtest/gtest-spi.h>
#include <lib/range_select/StuckCheck.hpp>

using range_select::StuckCheck;

static constexpr uint64_t kStart = 5'000'000;
static constexpr uint64_t kPeriod = 20'000;

TEST(StuckCheckTest, TenthNewSampleOfOneDistanceIsFrozen)
{
	StuckCheck stuck;

	// level flight, each 50 Hz sample read on two cycles: only a new sample time counts
	for (int k = 1; k <= 12; k++) {
		const uint64_t t = kStart + k * kPeriod;
		EXPECT_EQ(stuck.live(25.f, 125.f, t), k < 10) << "sample " << k;
		EXPECT_EQ(stuck.live(25.f, 125.f, t), k < 10) << "sample " << k;
	}
}

TEST(StuckCheckTest, DistanceHeldAcrossMoreThanOneMetreOfAltitudeIsFrozen)
{
	StuckCheck stuck;
	// a 10 Hz reading held through a 3 m/s descent: 0.9 m under its first sample it is live, 1.2 m under it is frozen
	EXPECT_TRUE(stuck.live(30.f, 100.f, kStart));
	EXPECT_TRUE(stuck.live(30.f, 99.7f, kStart + 100'000));
	EXPECT_TRUE(stuck.live(30.f, 99.4f, kStart + 200'000));
	EXPECT_TRUE(stuck.live(30.f, 99.1f, kStart + 300'000));
	EXPECT_FALSE(stuck.live(30.f, 98.8f, kStart + 400'000));
	// a climb counts as a descent does
	StuckCheck up;
	EXPECT_TRUE(up.live(30.f, 100.f, kStart));
	EXPECT_FALSE(up.live(30.f, 101.2f, kStart + 100'000));
}

TEST(StuckCheckTest, ChangedDistanceIsLiveAtOnceAndCountsFromOne)
{
	StuckCheck stuck;

	for (int k = 1; k <= 15; k++) {
		stuck.live(25.f, 125.f, kStart + k * kPeriod);
	}

	EXPECT_FALSE(stuck.live(25.f, 125.f, kStart + 16 * kPeriod));

	// one step of a 16 bit float at 25 m is 1/64 m
	for (int k = 1; k <= 12; k++) {
		EXPECT_EQ(stuck.live(25.015625f, 125.f, kStart + (16 + k) * kPeriod), k < 10) << "sample " << k;
	}
}

TEST(StuckCheckTest, ReadingThatStepsWithinTenSamplesStaysLive)
{
	StuckCheck stuck;

	// flat ground under level flight: nine samples on one step, then one on the next, for 20 s
	for (int k = 0; k < 1000; k++) {
		const float d = (k % 10 == 9) ? 25.015625f : 25.f;
		ASSERT_TRUE(stuck.live(d, 125.f, kStart + k * kPeriod)) << "sample " << k;
	}
}

TEST(StuckCheckTest, AltitudeReferenceFollowsTheDistance)
{
	StuckCheck stuck;
	float alt = 130.f;

	// 3 s down at 1.8 m/s with a changing reading, then one repeat 0.036 m of altitude after its first sample
	for (int i = 0; i < 150; i++) {
		alt -= 1.8f * 0.02f;
		ASSERT_TRUE(stuck.live(25.f + 0.01f * i, alt, kStart + i * kPeriod));
	}

	alt -= 1.8f * 0.02f;
	EXPECT_TRUE(stuck.live(25.f + 0.01f * 149, alt, kStart + 150 * kPeriod));
}

TEST(StuckCheckCanary, FrozenReadingCalledLiveIsCaught)
{
	StuckCheck stuck;

	for (int k = 1; k <= 20; k++) {
		stuck.live(25.f, 125.f, kStart + k * kPeriod);
	}

	EXPECT_NONFATAL_FAILURE(EXPECT_TRUE(stuck.live(25.f, 125.f, kStart + 21 * kPeriod)), "");
}
