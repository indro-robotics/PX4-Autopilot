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

#include "flare_law.hpp"

using math::radians;

namespace
{
constexpr float HR_ENTRY = -1.6f;
constexpr float HR_GROUND = -0.5f;
constexpr float SPEED = 13.f;
constexpr float DT = 0.02f;
const float KP = radians(4.5f);
const float KI = radians(2.5f);
const float PITCH_ENTRY = radians(1.f);
const float PITCH_MIN = radians(0.f);
const float PITCH_MAX = radians(4.5f);

float flarePitch(float height_rate_sp, float vz, float &integ, float kp = KP, float pitch_max = PITCH_MAX)
{
	return flare_law::pitch(height_rate_sp, HR_ENTRY, PITCH_ENTRY, vz, SPEED, kp, KI, PITCH_MIN, pitch_max, DT, integ);
}
}

TEST(FlareLawTest, ZeroErrorReturnsEntryPitch)
{
	float integ = 0.f;

	for (int i = 0; i < 50; i++) {
		EXPECT_FLOAT_EQ(flarePitch(HR_ENTRY, -HR_ENTRY, integ), PITCH_ENTRY);
	}

	EXPECT_FLOAT_EQ(integ, 0.f);
}

TEST(FlareLawTest, ExcessSinkHoldsCeilingAndFreezesIntegrator)
{
	float integ = 0.f;

	// 2 m/s over the setpoint asks 1 + 4.85 + 4.5 x 2 = 14.85 deg against the 4.5 deg ceiling
	for (int i = 0; i < 50; i++) {
		EXPECT_FLOAT_EQ(flarePitch(HR_GROUND, 2.5f, integ), PITCH_MAX);
	}

	EXPECT_FLOAT_EQ(integ, 0.f);
}

TEST(FlareLawTest, BalloonLowersPitchNotUnderFloor)
{
	float integ = 0.f;
	const float level = flarePitch(HR_GROUND, -HR_GROUND, integ, KP, radians(10.f));

	// climbing 0.5 m/s against a 0.5 m/s sink setpoint: 4.85 + 1 - 4.5 x 1 = 1.35 deg
	EXPECT_NEAR(flarePitch(HR_GROUND, -0.5f, integ, KP, radians(10.f)), radians(1.35f), radians(0.01f));
	EXPECT_LT(radians(1.35f), level);

	// climbing 1.5 m/s asks 4.85 + 1 - 4.5 x 2 = -3.15 deg, under the 0 deg floor
	integ = 0.f;

	for (int i = 0; i < 50; i++) {
		EXPECT_FLOAT_EQ(flarePitch(HR_GROUND, -1.5f, integ), PITCH_MIN);
	}

	EXPECT_FLOAT_EQ(integ, 0.f);
}

TEST(FlareLawTest, ScheduleFeedforwardAtThirteenMetresPerSecond)
{
	float integ = 0.f;

	// (-0.5 - -1.6) / 13 rad = 4.848 deg over the entry pitch
	const float pitch = flarePitch(HR_GROUND, -HR_GROUND, integ, KP, radians(10.f));
	EXPECT_NEAR(pitch - PITCH_ENTRY, radians(4.85f), radians(0.05f));
}

TEST(FlareLawTest, IntegratorRunsInsideLimitsAndBackFromThem)
{
	float integ = 0.f;

	// 0.1 m/s excess sink for 1 s inside the limits: 2.5 x 0.1 x 1 = 0.25 deg
	for (int i = 0; i < 50; i++) {
		flarePitch(HR_ENTRY, -HR_ENTRY + 0.1f, integ);
	}

	EXPECT_NEAR(integ, radians(0.25f), radians(0.001f));

	// at the ceiling a climb error still unwinds it
	integ = radians(10.f);
	flarePitch(HR_ENTRY, -HR_ENTRY - 0.2f, integ);
	EXPECT_LT(integ, radians(10.f));
}

TEST(FlareLawTest, ScheduleEndsAtEntrySinkAndFlareSink)
{
	EXPECT_FLOAT_EQ(flare_law::rangeSchedule(1.5f, 1.5f, HR_ENTRY, -HR_GROUND), HR_ENTRY);
	EXPECT_FLOAT_EQ(flare_law::rangeSchedule(0.f, 1.5f, HR_ENTRY, -HR_GROUND), HR_GROUND);
	EXPECT_FLOAT_EQ(flare_law::rangeSchedule(0.75f, 1.5f, HR_ENTRY, -HR_GROUND), 0.5f * (HR_ENTRY + HR_GROUND));
	EXPECT_FLOAT_EQ(flare_law::rangeSchedule(2.5f, 1.5f, HR_ENTRY, -HR_GROUND), HR_ENTRY);
	EXPECT_FLOAT_EQ(flare_law::rangeSchedule(-0.2f, 1.5f, HR_ENTRY, -HR_GROUND), HR_GROUND);
}

TEST(FlareLawTest, CanaryFlippedGainMissesCeiling)
{
	float integ = 0.f;
	EXPECT_NONFATAL_FAILURE(EXPECT_FLOAT_EQ(flarePitch(HR_GROUND, 2.5f, integ, -KP), PITCH_MAX), "PITCH_MAX");
}
