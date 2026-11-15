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

#include "ThrustLoss.hpp"

namespace
{

// FD_FW_TL_TIME 2.0 s, FD_FW_TL_THR 0.7, FD_FW_TL_RATE -0.8 m/s
constexpr ThrustLoss::Param kParam{2.f, 0.7f, -0.8f};
constexpr float kStep{0.02f};

class FailureDetectorThrustTest : public ::testing::Test
{
protected:
	// feeds tecs_status at 50 Hz with a specific energy rate in m/s; returns the seconds to the first flag change, else -1
	float fly(float seconds, float throttle, float energy_rate, bool armed = true, bool flying = true,
		  const ThrustLoss::Param &p = kParam)
	{
		const bool before = _tl.lost();
		float changed = -1.f;

		for (int k = 1; k <= lroundf(seconds / kStep); k++) {
			_t_us += 20000;
			_tl.update(_t_us, armed, flying, _low, throttle, energy_rate * 9.81f, p);

			if (changed < 0.f && _tl.lost() != before) {
				changed = k * kStep;
			}
		}

		return changed;
	}

	void cruise() { fly(10.f, 0.5f, 0.f); }

	void lose()
	{
		cruise();
		fly(3.5f, 0.9f, -1.4f);
		ASSERT_TRUE(_tl.lost());
	}

	ThrustLoss _tl{};
	uint64_t _t_us{1000000};
	bool _low{false};
};

TEST_F(FailureDetectorThrustTest, DeficitAtHighThrottleSets)
{
	cruise();
	// the 2 s timer starts at the first sample below -0.8 m/s
	const float t = fly(3.5f, 0.9f, -1.4f);
	EXPECT_TRUE(_tl.lost());
	EXPECT_NEAR(t, 2.f, 0.03f);
}

TEST_F(FailureDetectorThrustTest, LowThrottleDoesNotSet)
{
	cruise();
	fly(10.f, 0.3f, -1.4f);
	EXPECT_FALSE(_tl.lost());
}

TEST_F(FailureDetectorThrustTest, ThrottleAtTheThresholdSets)
{
	// FD_FW_TL_THR is the throttle at or above which the motor is expected to add energy
	cruise();
	EXPECT_NEAR(fly(3.5f, 0.7f, -1.4f), 2.f, 0.03f);
	EXPECT_TRUE(_tl.lost());
}

TEST_F(FailureDetectorThrustTest, MotorCommandedOffNeverSets)
{
	// a soaring glide sinks 1.4 m/s at zero throttle for minutes, at the shipped threshold and at its minimum of 0
	for (float threshold : {0.7f, 0.f}) {
		_tl = ThrustLoss{};
		cruise();
		EXPECT_LT(fly(120.f, 0.f, -1.4f, true, true, {2.f, threshold, -0.8f}), 0.f) << threshold;
		EXPECT_FALSE(_tl.lost()) << threshold;
		// the motor restart at the floor that adds no energy is flagged 2 s after the throttle is back
		EXPECT_NEAR(fly(3.5f, 1.f, -1.4f, true, true, {2.f, threshold, -0.8f}), 2.f, 0.03f) << threshold;
	}
}

TEST_F(FailureDetectorThrustTest, OneSecondDeficitDoesNotSet)
{
	cruise();
	fly(1.f, 0.9f, -1.4f);
	fly(10.f, 0.9f, 0.f);
	EXPECT_FALSE(_tl.lost());
}

TEST_F(FailureDetectorThrustTest, HoldsThroughThirtySecondsOfIdle)
{
	lose();
	fly(15.f, 0.f, -1.4f);
	fly(15.f, 0.f, 0.f);
	EXPECT_TRUE(_tl.lost());
}

TEST_F(FailureDetectorThrustTest, EnergyGainClears)
{
	lose();
	fly(5.f, 0.f, 0.f);
	const float t = fly(3.f, 0.9f, 1.f);
	EXPECT_FALSE(_tl.lost());
	EXPECT_NEAR(t, 2.f, 0.03f);
}

TEST_F(FailureDetectorThrustTest, GainAtLowThrottleClears)
{
	lose();
	fly(3.f, 0.3f, 1.f);
	EXPECT_FALSE(_tl.lost());
}

TEST_F(FailureDetectorThrustTest, ShortGainDoesNotClear)
{
	lose();
	fly(1.5f, 0.9f, 1.f);
	fly(10.f, 0.9f, -1.4f);
	EXPECT_TRUE(_tl.lost());
}

TEST_F(FailureDetectorThrustTest, DisarmClears)
{
	lose();
	fly(kStep, 0.f, 0.f, false);
	EXPECT_FALSE(_tl.lost());
	fly(10.f, 0.5f, 0.f);
	EXPECT_FALSE(_tl.lost());
}

TEST_F(FailureDetectorThrustTest, LandedNeverSets)
{
	fly(10.f, 0.9f, -1.4f, true, false);
	EXPECT_FALSE(_tl.lost());
}

TEST_F(FailureDetectorThrustTest, TimeZeroDisables)
{
	cruise();
	fly(10.f, 0.9f, -1.4f, true, true, {0.f, 0.7f, -0.8f});
	EXPECT_FALSE(_tl.lost());
}

TEST_F(FailureDetectorThrustTest, StaleSampleResetsTimerKeepsState)
{
	cruise();
	fly(1.5f, 0.9f, -1.4f);
	_t_us += 600000;
	// the timer restarts at the first sample after the gap, so 1.5 s more does not set and 0.5 s after that does
	fly(1.5f, 0.9f, -1.4f);
	EXPECT_FALSE(_tl.lost());
	fly(0.6f, 0.9f, -1.4f);
	EXPECT_TRUE(_tl.lost());
	_t_us += 600000;
	fly(kStep, 0.9f, 1.f);
	EXPECT_TRUE(_tl.lost());
}

TEST_F(FailureDetectorThrustTest, GainUnderTheClearRateDoesNotClear)
{
	lose();
	// the clear rate is the negative of FD_FW_TL_RATE, +0.8 m/s: lift of +0.5 m/s is not thrust
	fly(10.f, 0.9f, 0.5f);
	EXPECT_TRUE(_tl.lost());
	EXPECT_NEAR(fly(3.f, 0.9f, 0.9f), 2.f, 0.03f);
}

TEST_F(FailureDetectorThrustTest, UnderTheLevelHeightGainDoesNotClear)
{
	lose();
	_low = true;
	fly(10.f, 0.9f, 0.5f);
	EXPECT_TRUE(_tl.lost());
	// 1.8 m/s of lift reads about +0.85 m/s, above the clear rate
	fly(10.f, 0.9f, 0.85f);
	fly(10.f, 0.9f, 3.f);
	EXPECT_TRUE(_tl.lost());
	_low = false;
	EXPECT_NEAR(fly(3.f, 0.9f, 3.f), 2.f, 0.03f);
}

TEST_F(FailureDetectorThrustTest, GainClearsWhileLanded)
{
	lose();
	_low = true;
	// a landed state read in the air must not hold the flag over a motor that pulls again
	EXPECT_NEAR(fly(3.f, 0.9f, 1.f, true, false), 2.f, 0.03f);
	EXPECT_FALSE(_tl.lost());
}

TEST_F(FailureDetectorThrustTest, SampleOlderThanTheLastIsDropped)
{
	cruise();
	fly(1.9f, 0.9f, -1.4f);
	// an older sample without the deficit would restart the 2 s timer; kept, it ends 2.0 - 1.9 + 0.02 s into this span
	_tl.update(_t_us - 1000000, true, true, false, 0.9f, 0.f, kParam);
	EXPECT_NEAR(fly(0.2f, 0.9f, -1.4f), 0.12f, 0.03f);
	EXPECT_TRUE(_tl.lost());
}

} // namespace
