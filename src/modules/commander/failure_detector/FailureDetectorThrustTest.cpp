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

TEST(ThrustLossOrbit, ContinuesAboveTheGlideToTheTouchdownPlusTenMetres)
{
	// FW_TL_GLIDE 8 and 254 m of final: 254 / 8 + 10 = 41.75 m above the touchdown point
	EXPECT_FALSE(ThrustLoss::orbitEnds(true, 52.f, 254.f, 8.f, 0.f, 14.f));
	EXPECT_FALSE(ThrustLoss::orbitEnds(true, 41.85f, 254.f, 8.f, 0.f, 14.f));
	EXPECT_TRUE(ThrustLoss::orbitEnds(true, 41.65f, 254.f, 8.f, 0.f, 14.f));
	EXPECT_TRUE(ThrustLoss::orbitEnds(true, 30.f, 254.f, 8.f, 0.f, 14.f));
	// the far side of a 60 m ring is 120 m further out and ends 120 / 8 = 15 m higher
	EXPECT_FALSE(ThrustLoss::orbitEnds(true, 56.85f, 374.f, 8.f, 0.f, 14.f));
	EXPECT_TRUE(ThrustLoss::orbitEnds(true, 56.65f, 374.f, 8.f, 0.f, 14.f));
	// a glide ratio of 12 reaches the same 254 m from 254 / 12 + 10 = 31.17 m
	EXPECT_FALSE(ThrustLoss::orbitEnds(true, 31.3f, 254.f, 12.f, 0.f, 14.f));
	EXPECT_TRUE(ThrustLoss::orbitEnds(true, 31.f, 254.f, 12.f, 0.f, 14.f));
}

TEST(ThrustLossOrbit, EndsAtTheRingWithoutATouchdownPointOrAHeight)
{
	// with no landing item after the orbit there is no final to save height for
	EXPECT_TRUE(ThrustLoss::orbitEnds(false, 120.f, 254.f, 8.f, 0.f, 14.f));
	// a height or a distance that does not read must not hold a glide on the circle
	EXPECT_TRUE(ThrustLoss::orbitEnds(true, NAN, 254.f, 8.f, 0.f, 14.f));
	EXPECT_TRUE(ThrustLoss::orbitEnds(true, 120.f, NAN, 8.f, 0.f, 14.f));
}

TEST(ThrustLossOrbit, GlideRatioUnderOneCountsAsOne)
{
	// FW_TL_GLIDE 0 would hold the orbit to the ground: 100 / 1 + 10 = 110 m
	EXPECT_TRUE(ThrustLoss::orbitEnds(true, 109.f, 100.f, 0.f, 0.f, 14.f));
	EXPECT_FALSE(ThrustLoss::orbitEnds(true, 111.f, 100.f, 0.f, 0.f, 14.f));
	EXPECT_TRUE(ThrustLoss::orbitEnds(true, 109.f, 100.f, 0.5f, 0.f, 14.f));
	EXPECT_FALSE(ThrustLoss::orbitEnds(true, 111.f, 100.f, 0.5f, 0.f, 14.f));
}

TEST(ThrustLossOrbit, TurnOntoTheFinalAddsItsArcToTheDistance)
{
	const float pi = static_cast<float>(M_PI);
	// 30 deg of bank at 14 m/s turns on 14^2 / (9.80665 * tan(30 deg)) = 34.62 m: a half turn is 108.8 m, a quarter 54.4 m
	EXPECT_NEAR(ThrustLoss::turnArc(pi, 14.f), 108.8f, 0.1f);
	EXPECT_NEAR(ThrustLoss::turnArc(pi / 2.f, 14.f), 54.4f, 0.1f);
	// the radius grows with the square of the airspeed: at 18 m/s 57.22 m, a half turn 179.8 m
	EXPECT_NEAR(ThrustLoss::turnArc(pi, 18.f), 179.8f, 0.1f);
	// a turn to the left costs what a turn to the right costs, and 350 deg is 10 deg the other way: 34.62 * 0.1745 = 6.04 m
	EXPECT_NEAR(ThrustLoss::turnArc(-pi / 2.f, 14.f), 54.4f, 0.1f);
	EXPECT_NEAR(ThrustLoss::turnArc(350.f * pi / 180.f, 14.f), 6.04f, 0.05f);
	EXPECT_NEAR(ThrustLoss::turnArc(-350.f * pi / 180.f, 14.f), 6.04f, 0.05f);
	EXPECT_NEAR(ThrustLoss::turnArc(0.f, 14.f), 0.f, 0.02f);
	// a course or an airspeed that does not read adds nothing: the orbit ends by the distance alone
	EXPECT_FLOAT_EQ(ThrustLoss::turnArc(NAN, 14.f), 0.f);
	EXPECT_FLOAT_EQ(ThrustLoss::turnArc(pi, NAN), 0.f);
	EXPECT_FLOAT_EQ(ThrustLoss::turnArc(pi, 0.f), 0.f);

	// flying away from the touchdown point with 254 m of final: (254 + 108.8) / 8 + 10 = 55.35 m, against 41.75 m pointing at it
	EXPECT_FALSE(ThrustLoss::orbitEnds(true, 55.5f, 254.f, 8.f, pi, 14.f));
	EXPECT_TRUE(ThrustLoss::orbitEnds(true, 55.2f, 254.f, 8.f, pi, 14.f));
	EXPECT_TRUE(ThrustLoss::orbitEnds(true, 55.2f, 254.f, 8.f, -pi, 14.f));
	// a quarter turn: (254 + 54.4) / 8 + 10 = 48.55 m
	EXPECT_FALSE(ThrustLoss::orbitEnds(true, 48.7f, 254.f, 8.f, pi / 2.f, 14.f));
	EXPECT_TRUE(ThrustLoss::orbitEnds(true, 48.4f, 254.f, 8.f, -pi / 2.f, 14.f));
	// a 6 m/s headwind on the final scales the turn with the distance: (254 + 108.8) / 4.571 + 10 = 89.4 m
	const float glide = ThrustLoss::glideOverGround(8.f, 14.f, -6.f, 0.f);
	EXPECT_FALSE(ThrustLoss::orbitEnds(true, 89.7f, 254.f, glide, pi, 14.f));
	EXPECT_TRUE(ThrustLoss::orbitEnds(true, 89.1f, 254.f, glide, pi, 14.f));
	// an unread course leaves the rule of the distance: 41.75 m
	EXPECT_FALSE(ThrustLoss::orbitEnds(true, 41.85f, 254.f, 8.f, NAN, 14.f));
	EXPECT_TRUE(ThrustLoss::orbitEnds(true, 41.65f, 254.f, 8.f, NAN, 14.f));
	// no landing item after the orbit: no final and no turn to save height for
	EXPECT_TRUE(ThrustLoss::orbitEnds(false, 120.f, 254.f, 8.f, pi, 14.f));
}

TEST(ThrustLossOrbit, RingCountsUnderTheHeldAltitudeWithTheFlag)
{
	// a glide from 45 m reaches the ring 23 m under the altitude the item holds, far outside NAV_FW_ALTL_RAD 2 m
	EXPECT_TRUE(ThrustLoss::orbitRing(true, false, true));
	EXPECT_TRUE(ThrustLoss::orbitRing(true, true, false));
	// with the motor running the stock altitude test stands
	EXPECT_FALSE(ThrustLoss::orbitRing(true, false, false));
	// the flag ends no orbit the aircraft has not reached
	EXPECT_FALSE(ThrustLoss::orbitRing(false, false, true));
	EXPECT_FALSE(ThrustLoss::orbitRing(false, true, true));
}

TEST(ThrustLossWind, GlideRatioFollowsGroundSpeedOverAirspeedWithinBounds)
{
	EXPECT_FLOAT_EQ(ThrustLoss::glideOverGround(8.f, 14.f, 0.f, 0.f), 8.f);
	// 4 m/s headwind at 14 m/s: 8 * 10 / 14 = 5.714
	EXPECT_NEAR(ThrustLoss::glideOverGround(8.f, 14.f, -4.f, 0.f), 5.714f, 0.001f);
	// 6 m/s headwind: 8 * 8 / 14 = 4.571, above the 0.5 bound of 4.0
	EXPECT_NEAR(ThrustLoss::glideOverGround(8.f, 14.f, -6.f, 0.f), 4.571f, 0.001f);
	// 10 m/s headwind would give 8 * 4 / 14 = 2.29: held at 8 * 0.5
	EXPECT_FLOAT_EQ(ThrustLoss::glideOverGround(8.f, 14.f, -10.f, 0.f), 4.f);
	// 2 m/s tailwind: 8 * 16 / 14 = 9.143; 6 m/s would give 11.4: held at 8 * 1.2
	EXPECT_NEAR(ThrustLoss::glideOverGround(8.f, 14.f, 2.f, 0.f), 9.143f, 0.001f);
	EXPECT_FLOAT_EQ(ThrustLoss::glideOverGround(8.f, 14.f, 6.f, 0.f), 9.6f);
	// no airspeed or no wind component leaves the still-air ratio
	EXPECT_FLOAT_EQ(ThrustLoss::glideOverGround(8.f, 0.f, 0.f, 0.f), 8.f);
	EXPECT_FLOAT_EQ(ThrustLoss::glideOverGround(8.f, 14.f, NAN, 0.f), 8.f);
	EXPECT_FLOAT_EQ(ThrustLoss::glideOverGround(8.f, NAN, -4.f, 0.f), 8.f);
}

TEST(ThrustLossWind, CrosswindCostsTheCrab)
{
	// 6 m/s across the track at 14 m/s: 8 * sqrt(14^2 - 6^2) / 14 = 7.228; either side
	EXPECT_NEAR(ThrustLoss::glideOverGround(8.f, 14.f, 0.f, 6.f), 7.228f, 0.001f);
	EXPECT_NEAR(ThrustLoss::glideOverGround(8.f, 14.f, 0.f, -6.f), 7.228f, 0.001f);
	// with 4 m/s against: 8 * (12.649 - 4) / 14 = 4.942
	EXPECT_NEAR(ThrustLoss::glideOverGround(8.f, 14.f, -4.f, 6.f), 4.942f, 0.001f);
	// a crosswind at the airspeed leaves no ground speed: held at 8 * 0.5
	EXPECT_FLOAT_EQ(ThrustLoss::glideOverGround(8.f, 14.f, 0.f, 15.f), 4.f);
	// 250 m of final: the orbit ends at 250 / 7.228 + 10 = 44.6 m, not the 41.25 m of still air
	EXPECT_FALSE(ThrustLoss::orbitEnds(true, 44.7f, 250.f, ThrustLoss::glideOverGround(8.f, 14.f, 0.f, 6.f), 0.f, 14.f));
	EXPECT_TRUE(ThrustLoss::orbitEnds(true, 44.5f, 250.f, ThrustLoss::glideOverGround(8.f, 14.f, 0.f, 6.f), 0.f, 14.f));
}

TEST(ThrustLossWind, HeadwindOnTheFinalEndsTheOrbitHigher)
{
	// 254 m of final in a 6 m/s headwind at 14 m/s: 254 / 4.571 + 10 = 65.6 m, against 41.75 m in still air
	const float glide = ThrustLoss::glideOverGround(8.f, 14.f, -6.f, 0.f);
	EXPECT_FALSE(ThrustLoss::orbitEnds(true, 66.f, 254.f, glide, 0.f, 14.f));
	EXPECT_TRUE(ThrustLoss::orbitEnds(true, 65.f, 254.f, glide, 0.f, 14.f));
	EXPECT_FALSE(ThrustLoss::orbitEnds(true, 42.f, 254.f, 8.f, 0.f, 14.f));
}

TEST(ThrustLossWind, FinalCourseIsTheTangentFromTheOrbit)
{
	// firmware pattern: radius 60 m, centre sqrt(200^2 + 60^2) = 208.8 m from the touchdown, asin(60 / 208.8) = 16.70 deg
	const float turn = 16.70f * static_cast<float>(M_PI) / 180.f;
	EXPECT_NEAR(ThrustLoss::finalCourse(1.f, 60.f, 208.8f, false), 1.f + turn, 0.001f);
	EXPECT_NEAR(ThrustLoss::finalCourse(1.f, 60.f, 208.8f, true), 1.f - turn, 0.001f);
	EXPECT_NEAR(ThrustLoss::finalCourse(1.f, -60.f, 208.8f, false), 1.f + turn, 0.001f);
	// a touchdown point inside the circle has no tangent: a quarter turn, never NAN
	EXPECT_NEAR(ThrustLoss::finalCourse(0.f, 60.f, 30.f, false), static_cast<float>(M_PI_2), 0.001f);
	EXPECT_NEAR(ThrustLoss::finalCourse(0.f, 60.f, 0.f, false), static_cast<float>(M_PI_2), 0.001f);
}

TEST(ThrustLossLevel, HeightRisesByTheReachToTheSetpoint)
{
	// FW_TL_LVL_ALT 20 m, 300 m to a waypoint at 8:1: 20 + 300 / 8 = 57.5 m
	EXPECT_FLOAT_EQ(ThrustLoss::levelHeight(20.f, 300.f, false, NAN, 8.f), 57.5f);
	// an orbit of 60 m is reached 60 m earlier: 20 + 240 / 8 = 50 m
	EXPECT_FLOAT_EQ(ThrustLoss::levelHeight(20.f, 300.f, true, 60.f, 8.f), 50.f);
	EXPECT_FLOAT_EQ(ThrustLoss::levelHeight(20.f, 300.f, true, -60.f, 8.f), 50.f);
	// on the circle of a circular landing the height is FW_TL_LVL_ALT, not 20 + 60 / 8 = 27.5 m
	EXPECT_FLOAT_EQ(ThrustLoss::levelHeight(20.f, 60.f, true, 60.f, 8.f), 20.f);
	EXPECT_FLOAT_EQ(ThrustLoss::levelHeight(20.f, 40.f, true, 60.f, 8.f), 20.f);
	// a waypoint 60 m away is not a circle
	EXPECT_FLOAT_EQ(ThrustLoss::levelHeight(20.f, 60.f, false, 60.f, 8.f), 27.5f);
	// a 6 m/s headwind at 14 m/s halves the reach: 20 + 240 / 4.571 = 72.5 m
	EXPECT_NEAR(ThrustLoss::levelHeight(20.f, 300.f, true, 60.f, ThrustLoss::glideOverGround(8.f, 14.f, -6.f, 0.f)), 72.5f, 0.1f);
	// a distance or a radius that does not read, and a ratio under 1
	EXPECT_FLOAT_EQ(ThrustLoss::levelHeight(20.f, NAN, false, NAN, 8.f), 20.f);
	EXPECT_FLOAT_EQ(ThrustLoss::levelHeight(20.f, 300.f, true, NAN, 8.f), 57.5f);
	EXPECT_FLOAT_EQ(ThrustLoss::levelHeight(20.f, 80.f, false, NAN, 0.f), 100.f);
}

TEST(ThrustLossFloor, NeedsARangeToTheGroundOutsideTakeoff)
{
	// FW_LND_FLALT 1.5 m: the floor acts under 3 m of range
	EXPECT_TRUE(ThrustLoss::pitchFloor(false, false, true, 2.9f, 1.5f));
	EXPECT_FALSE(ThrustLoss::pitchFloor(false, false, true, 3.1f, 1.5f));
	// 3 m above the estimator origin over a valley 70 m deep is no height above ground
	EXPECT_FALSE(ThrustLoss::pitchFloor(false, false, false, 2.9f, 1.5f));
	EXPECT_FALSE(ThrustLoss::pitchFloor(false, false, false, 0.f, 1.5f));
	// a landing flies its own flare
	EXPECT_FALSE(ThrustLoss::pitchFloor(false, true, true, 1.f, 1.5f));
	// Takeoff keeps the floor at any height and without a range
	EXPECT_TRUE(ThrustLoss::pitchFloor(true, false, false, 40.f, 1.5f));
	EXPECT_TRUE(ThrustLoss::pitchFloor(true, false, true, 40.f, 1.5f));
}

TEST(ThrustLossAirspeed, GuidanceMinimumRaisesThePinInAutoOnly)
{
	// FW_AIRSPD_MIN 14 and FW_AIRSPD_MAX 23: guidance asks 16 m/s to hold its ground speed floor into wind
	EXPECT_FLOAT_EQ(ThrustLoss::glideAirspeed(true, 18.f, 14.f, 16.f, 23.f), 16.f);
	EXPECT_FLOAT_EQ(ThrustLoss::glideAirspeed(true, 18.f, 14.f, 0.f, 23.f), 14.f);
	EXPECT_FLOAT_EQ(ThrustLoss::glideAirspeed(true, 18.f, 14.f, 30.f, 23.f), 23.f);
	// Position and Altitude mode keep the airspeed the stick commands
	EXPECT_FLOAT_EQ(ThrustLoss::glideAirspeed(false, 18.f, 14.f, 16.f, 23.f), 18.f);
	EXPECT_FLOAT_EQ(ThrustLoss::glideAirspeed(false, 21.f, 14.f, 0.f, 23.f), 21.f);
}

TEST(ThrustLossBank, AutoModesBankThirtyDegreesWithTheFlag)
{
	// 9.80665 * tan(30 deg) = 5.662 m/s^2; FW_R_LIM 50 deg is 11.69 m/s^2
	EXPECT_NEAR(ThrustLoss::lateralAccelMax(true, true, 11.69f, true, 40.f), 5.662f, 0.001f);
	EXPECT_NEAR(atanf(ThrustLoss::lateralAccelMax(true, true, 11.69f, true, 40.f) / 9.80665f) * 180.f / static_cast<float>(M_PI), 30.f, 0.01f);
	// a lower limit of the mode stands: wings level under the level height, the wing strike limit near the ground
	EXPECT_FLOAT_EQ(ThrustLoss::lateralAccelMax(true, true, 0.f, true, 40.f), 0.f);
	EXPECT_FLOAT_EQ(ThrustLoss::lateralAccelMax(true, true, 3.f, true, 40.f), 3.f);
	// the pilot and a running motor keep the full bank
	EXPECT_FLOAT_EQ(ThrustLoss::lateralAccelMax(true, false, 11.69f, true, 40.f), 11.69f);
	EXPECT_FLOAT_EQ(ThrustLoss::lateralAccelMax(false, true, 11.69f, true, 40.f), 11.69f);
}

TEST(ThrustLossBank, TenDegreesWithAValidRangeUnderTenMetres)
{
	// 9.80665 * tan(10 deg) = 1.729 m/s^2
	EXPECT_NEAR(ThrustLoss::lateralAccelMax(true, true, 11.69f, true, 9.9f), 1.729f, 0.001f);
	EXPECT_NEAR(atanf(ThrustLoss::lateralAccelMax(true, true, 11.69f, true, 3.f) / 9.80665f) * 180.f / static_cast<float>(M_PI), 10.f, 0.01f);
	// from 10 m up the 30 deg of the glide stand
	EXPECT_NEAR(ThrustLoss::lateralAccelMax(true, true, 11.69f, true, 10.f), 5.662f, 0.001f);
	EXPECT_NEAR(ThrustLoss::lateralAccelMax(true, true, 11.69f, true, 10.1f), 5.662f, 0.001f);
	// a range that is not valid is no height above the ground, whatever it reads
	EXPECT_NEAR(ThrustLoss::lateralAccelMax(true, true, 11.69f, false, 3.f), 5.662f, 0.001f);
	EXPECT_NEAR(ThrustLoss::lateralAccelMax(true, true, 11.69f, false, 0.f), 5.662f, 0.001f);
	// a lower limit of the mode stands: wings level under FW_WING_HEIGHT
	EXPECT_FLOAT_EQ(ThrustLoss::lateralAccelMax(true, true, 0.f, true, 3.f), 0.f);
	EXPECT_FLOAT_EQ(ThrustLoss::lateralAccelMax(true, true, 1.f, true, 3.f), 1.f);
	// the pilot and a running motor keep the full bank at any height
	EXPECT_FLOAT_EQ(ThrustLoss::lateralAccelMax(true, false, 11.69f, true, 3.f), 11.69f);
	EXPECT_FLOAT_EQ(ThrustLoss::lateralAccelMax(false, true, 11.69f, true, 3.f), 11.69f);
}

TEST(ThrustLossReturnLeg, TakesTheLegHeightBackOnceAfterAClearBeforeTheRing)
{
	// Return from 120 m: orbit at home + 22 m, RTL_RETURN_ALT 30 m, altitudes above home
	bool lowered = false;
	EXPECT_FLOAT_EQ(ThrustLoss::returnLegAltitude(true, true, 120.f, 22.f, 118.f, 30.f, lowered), 22.f);
	// the setpoint holds the orbit height while the flag stays
	EXPECT_TRUE(isnan(ThrustLoss::returnLegAltitude(true, true, 22.f, 22.f, 110.f, 30.f, lowered)));
	// the flag clears at 104 m, 1 km before the ring: the leg holds 104 m, as a Return entered there would
	EXPECT_FLOAT_EQ(ThrustLoss::returnLegAltitude(false, true, 22.f, 22.f, 104.f, 30.f, lowered), 104.f);
	// once: the next cycles write nothing, so the ring can hand the orbit its own height
	EXPECT_TRUE(isnan(ThrustLoss::returnLegAltitude(false, true, 104.f, 22.f, 104.f, 30.f, lowered)));
	EXPECT_TRUE(isnan(ThrustLoss::returnLegAltitude(false, false, 22.f, 22.f, 30.f, 30.f, lowered)));
}

TEST(ThrustLossReturnLeg, ClimbsToTheReturnAltitudeWhenTheClearComesBelowIt)
{
	// cut in the Hold at 45 m, clear at 26 m: the leg climbs to RTL_RETURN_ALT 30 m and leaves the orbit's 22 m
	bool lowered = false;
	EXPECT_FLOAT_EQ(ThrustLoss::returnLegAltitude(true, true, 45.f, 22.f, 44.f, 30.f, lowered), 22.f);
	EXPECT_FLOAT_EQ(ThrustLoss::returnLegAltitude(false, true, 22.f, 22.f, 26.f, 30.f, lowered), 30.f);
}

TEST(ThrustLossReturnLeg, WritesNothingOnTheRingOrWithoutALoweredLeg)
{
	// a clear on the ring leaves the orbit at its own height
	bool lowered = false;
	EXPECT_FLOAT_EQ(ThrustLoss::returnLegAltitude(true, true, 45.f, 22.f, 44.f, 30.f, lowered), 22.f);
	EXPECT_TRUE(isnan(ThrustLoss::returnLegAltitude(false, false, 22.f, 22.f, 26.f, 30.f, lowered)));
	// and the ring ends the restore: leaving the ring later writes nothing
	EXPECT_TRUE(isnan(ThrustLoss::returnLegAltitude(false, true, 22.f, 22.f, 26.f, 30.f, lowered)));
	// a Return that never saw the flag keeps its setpoint
	bool never = false;
	EXPECT_TRUE(isnan(ThrustLoss::returnLegAltitude(false, true, 60.f, 22.f, 60.f, 30.f, never)));
	// a flag set on the orbit at its height lowers nothing, so its clear restores nothing
	EXPECT_TRUE(isnan(ThrustLoss::returnLegAltitude(true, false, 22.f, 22.f, 21.f, 30.f, never)));
	EXPECT_TRUE(isnan(ThrustLoss::returnLegAltitude(false, true, 22.f, 22.f, 21.f, 30.f, never)));
}

} // namespace
