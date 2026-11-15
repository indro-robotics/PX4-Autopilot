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

#include "landing_pattern.h"

TEST(LandingPattern, DefaultsAndTheRangeEdgesPass)
{
	// MIS_LP_DIST 200, MIS_LP_ALT 22, MIS_LP_OFFSET 6
	EXPECT_TRUE(landingPatternInRange(200.f, 22.f, 6.f));
	EXPECT_TRUE(landingPatternInRange(50.f, 5.f, 5.f));
	EXPECT_TRUE(landingPatternInRange(50.f, 5.f, -5.f));
	EXPECT_TRUE(landingPatternInRange(400.f, 50.f, -50.f));
}

TEST(LandingPattern, FinalTowardTheLauncherIsRefused)
{
	// a negative length puts the orbit beyond the touchdown point and the final 146.6 deg off the launch heading
	EXPECT_FALSE(landingPatternInRange(-200.f, 22.f, 6.f));
	EXPECT_FALSE(landingPatternInRange(0.f, 22.f, 6.f));
	EXPECT_FALSE(landingPatternInRange(49.9f, 22.f, 6.f));
}

TEST(LandingPattern, OrbitUnderFiveMetresIsRefused)
{
	EXPECT_FALSE(landingPatternInRange(200.f, 4.9f, 6.f));
	EXPECT_FALSE(landingPatternInRange(200.f, 0.f, 6.f));
	EXPECT_FALSE(landingPatternInRange(200.f, -5.f, 6.f));
}

TEST(LandingPattern, TouchdownLineOnTheLauncherIsRefused)
{
	EXPECT_FALSE(landingPatternInRange(200.f, 22.f, 0.f));
	EXPECT_FALSE(landingPatternInRange(200.f, 22.f, 4.9f));
	EXPECT_FALSE(landingPatternInRange(200.f, 22.f, -4.9f));
}

TEST(LandingPattern, ValueThatDoesNotReadIsRefused)
{
	EXPECT_FALSE(landingPatternInRange(NAN, 22.f, 6.f));
	EXPECT_FALSE(landingPatternInRange(200.f, NAN, 6.f));
	EXPECT_FALSE(landingPatternInRange(200.f, 22.f, NAN));
}

TEST(LandingPattern, OrbitMoreThanOneDegreeUnderTheLandingSlopeIsRefused)
{
	// FW_LND_ANG 6.3: the bound is 5.3 deg, 200 x tan(5.3 deg) = 18.55 m at MIS_LP_DIST 200
	EXPECT_TRUE(landingPatternSlopeFits(200.f, 22.f, 6.3f));
	EXPECT_TRUE(landingPatternSlopeFits(200.f, 18.6f, 6.3f));
	EXPECT_FALSE(landingPatternSlopeFits(200.f, 18.5f, 6.3f));
	// MIS_LP_ALT 5 at MIS_LP_DIST 200 is 1.43 deg
	EXPECT_FALSE(landingPatternSlopeFits(200.f, 5.f, 6.3f));
	// the stock FW_LND_ANG 5.0: the bound is 4.0 deg, 13.99 m at 200 m
	EXPECT_TRUE(landingPatternSlopeFits(200.f, 14.1f, 5.f));
	EXPECT_FALSE(landingPatternSlopeFits(200.f, 13.9f, 5.f));
	// the length counts: 22 m over 300 m is 4.19 deg
	EXPECT_FALSE(landingPatternSlopeFits(300.f, 22.f, 6.3f));
	// the steep side is the mission check's: 30 m over 200 m, 8.53 deg, fits here
	EXPECT_TRUE(landingPatternSlopeFits(200.f, 30.f, 6.3f));
	// a landing angle that does not read judges nothing
	EXPECT_TRUE(landingPatternSlopeFits(200.f, 5.f, NAN));
}
