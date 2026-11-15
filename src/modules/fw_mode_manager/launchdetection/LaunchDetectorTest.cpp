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
#include <px4_platform_common/param.h>

#include "LaunchDetector.h"

using launchdetection::LaunchDetector;
using Status = launch_detection_status_s;

static constexpr float kDt = 0.01f;
static constexpr float kShotAccel = 30.f;
static constexpr float kGate = 10.5f;
static constexpr float kKnock = 2.f;

class LaunchDetectorTest : public ::testing::Test
{
public:
	void SetUp() override
	{
		param_control_autosave(false);
		set("FW_LAUN_AC_THLD", 15.f);
		set("FW_LAUN_AC_T", 0.1f);
		set("FW_LAUN_MOT_DEL", 0.4f);
		set("FW_LAUN_SPD_MIN", kGate);
	}

	static void set(const char *name, float value)
	{
		param_set(param_find(name), &value);
	}

	static void run(LaunchDetector &detector, float seconds, float accel_x, float speed, float roll = 0.f,
			float pitch = 0.f)
	{
		for (float t = 0.f; t < seconds; t += kDt) {
			detector.update(kDt, accel_x, speed, roll, pitch);
		}
	}

	// A catapult stroke long enough to pass FW_LAUN_AC_T, then the motor delay at the given speed and attitude.
	static uint shot(LaunchDetector &detector, float speed, float roll = 0.f, float pitch = 0.f)
	{
		run(detector, 0.15f, kShotAccel, 0.f);
		EXPECT_EQ(detector.getLaunchDetected(), Status::STATE_LAUNCH_DETECTED_DISABLED_MOTOR);
		run(detector, 0.5f, 0.f, speed, roll, pitch);
		return detector.getLaunchDetected();
	}

	static float deg(float d) { return d * static_cast<float>(M_PI) / 180.f; }
};

TEST_F(LaunchDetectorTest, GateOffIsStock)
{
	set("FW_LAUN_SPD_MIN", 0.f);
	LaunchDetector detector{nullptr};
	EXPECT_EQ(shot(detector, 0.f), Status::STATE_FLYING);
	EXPECT_FALSE(detector.aborted());
}

TEST_F(LaunchDetectorTest, MotorHeldUntilDelay)
{
	LaunchDetector detector{nullptr};
	run(detector, 0.15f, kShotAccel, 0.f);
	run(detector, 0.3f, 0.f, 12.f);
	EXPECT_EQ(detector.getLaunchDetected(), Status::STATE_LAUNCH_DETECTED_DISABLED_MOTOR);
}

TEST_F(LaunchDetectorTest, ShotAboveGateFlies)
{
	LaunchDetector detector{nullptr};
	EXPECT_EQ(shot(detector, kGate + 0.5f), Status::STATE_FLYING);
	EXPECT_FALSE(detector.aborted());
}

TEST_F(LaunchDetectorTest, PeakSpeedDecides)
{
	LaunchDetector detector{nullptr};
	run(detector, 0.15f, kShotAccel, 0.f);
	run(detector, 0.1f, 0.f, kGate + 0.5f);
	run(detector, 0.4f, 0.f, kGate - 1.f);
	EXPECT_EQ(detector.getLaunchDetected(), Status::STATE_FLYING);
}

TEST_F(LaunchDetectorTest, ShotUnderGateAbortsAndNeverFlies)
{
	LaunchDetector detector{nullptr};
	EXPECT_EQ(shot(detector, 9.f), Status::STATE_LAUNCH_DETECTED_DISABLED_MOTOR);
	EXPECT_TRUE(detector.aborted());

	// A second stroke or a dive past the gate must not start the motor until disarm.
	run(detector, 10.f, kShotAccel, 2.f * kGate);
	EXPECT_EQ(detector.getLaunchDetected(), Status::STATE_LAUNCH_DETECTED_DISABLED_MOTOR);
	EXPECT_TRUE(detector.aborted());
}

TEST_F(LaunchDetectorTest, StrokeStillAcceleratingAtTheDelayCanFly)
{
	LaunchDetector detector{nullptr};
	run(detector, 0.15f, kShotAccel, 0.f);
	run(detector, 0.5f, 8.f, kGate - 1.5f);
	EXPECT_EQ(detector.getLaunchDetected(), Status::STATE_LAUNCH_DETECTED_DISABLED_MOTOR);
	EXPECT_FALSE(detector.aborted());

	run(detector, 0.05f, 8.f, kGate + 0.5f);
	EXPECT_EQ(detector.getLaunchDetected(), Status::STATE_FLYING);
}

TEST_F(LaunchDetectorTest, SlowShotAbortsWhenTheStrokeEnds)
{
	LaunchDetector detector{nullptr};
	run(detector, 0.15f, kShotAccel, 0.f);
	run(detector, 0.5f, 8.f, kGate - 1.5f);
	EXPECT_FALSE(detector.aborted());

	run(detector, 0.02f, -2.f, kGate - 1.5f);
	EXPECT_TRUE(detector.aborted());
}

TEST_F(LaunchDetectorTest, KnockReturnsToWaiting)
{
	LaunchDetector detector{nullptr};
	EXPECT_EQ(shot(detector, 0.f), Status::STATE_WAITING_FOR_LAUNCH);
	EXPECT_FALSE(detector.aborted());

	// The re-armed wait takes the real shot.
	EXPECT_EQ(shot(detector, kGate + 0.5f), Status::STATE_FLYING);
}

TEST_F(LaunchDetectorTest, KnockBoundIsTwoMetresPerSecond)
{
	LaunchDetector below{nullptr};
	EXPECT_EQ(shot(below, 1.9f), Status::STATE_WAITING_FOR_LAUNCH);

	LaunchDetector above{nullptr};
	shot(above, 2.1f);
	EXPECT_TRUE(above.aborted());
}

TEST_F(LaunchDetectorTest, InvalidSpeedFlies)
{
	LaunchDetector detector{nullptr};
	EXPECT_EQ(shot(detector, NAN), Status::STATE_FLYING);
	EXPECT_FALSE(detector.aborted());
}

TEST_F(LaunchDetectorTest, ResetClearsAbort)
{
	LaunchDetector detector{nullptr};
	shot(detector, 9.f);
	ASSERT_TRUE(detector.aborted());

	detector.reset();
	EXPECT_EQ(detector.getLaunchDetected(), Status::STATE_WAITING_FOR_LAUNCH);
	EXPECT_FALSE(detector.aborted());
	EXPECT_EQ(shot(detector, kGate + 0.5f), Status::STATE_FLYING);
}

TEST_F(LaunchDetectorTest, StrokeUnderKnockSpeedStillAcceleratingWaits)
{
	// A real stroke can read under 2 m/s at FW_LAUN_MOT_DEL; only the end of the acceleration may reset it.
	LaunchDetector detector{nullptr};
	run(detector, 0.15f, kShotAccel, 0.f);
	run(detector, 0.5f, 20.f, 1.f);
	EXPECT_EQ(detector.getLaunchDetected(), Status::STATE_LAUNCH_DETECTED_DISABLED_MOTOR);
	EXPECT_FALSE(detector.aborted());

	run(detector, 0.05f, 20.f, kGate + 0.5f);
	EXPECT_EQ(detector.getLaunchDetected(), Status::STATE_FLYING);
}

TEST_F(LaunchDetectorTest, KnockResetsWhenTheAccelerationEnds)
{
	LaunchDetector detector{nullptr};
	EXPECT_EQ(shot(detector, 1.f), Status::STATE_WAITING_FOR_LAUNCH);
	EXPECT_FALSE(detector.aborted());
}

TEST_F(LaunchDetectorTest, RolledShotAbortsAndNeverFlies)
{
	for (const float roll : {deg(35.f), deg(-35.f)}) {
		LaunchDetector detector{nullptr};
		EXPECT_EQ(shot(detector, kGate + 0.5f, roll), Status::STATE_LAUNCH_DETECTED_DISABLED_MOTOR);
		EXPECT_TRUE(detector.aborted());

		run(detector, 5.f, 0.f, 2.f * kGate);
		EXPECT_EQ(detector.getLaunchDetected(), Status::STATE_LAUNCH_DETECTED_DISABLED_MOTOR);
	}
}

TEST_F(LaunchDetectorTest, RollInsideTheLimitFlies)
{
	for (const float roll : {deg(25.f), deg(-25.f)}) {
		LaunchDetector detector{nullptr};
		EXPECT_EQ(shot(detector, kGate + 0.5f, roll), Status::STATE_FLYING);
		EXPECT_FALSE(detector.aborted());
	}
}

TEST_F(LaunchDetectorTest, PitchOutsideTheLimitAborts)
{
	for (const float pitch : {deg(50.f), deg(-35.f)}) {
		LaunchDetector detector{nullptr};
		shot(detector, kGate + 0.5f, 0.f, pitch);
		EXPECT_TRUE(detector.aborted());
	}

	for (const float pitch : {deg(40.f), deg(-25.f)}) {
		LaunchDetector detector{nullptr};
		EXPECT_EQ(shot(detector, kGate + 0.5f, 0.f, pitch), Status::STATE_FLYING);
	}
}

TEST_F(LaunchDetectorTest, AttitudeIgnoredWithGateOff)
{
	set("FW_LAUN_SPD_MIN", 0.f);
	LaunchDetector detector{nullptr};
	EXPECT_EQ(shot(detector, 0.f, deg(35.f), deg(50.f)), Status::STATE_FLYING);
	EXPECT_FALSE(detector.aborted());
}

TEST_F(LaunchDetectorTest, PublishedWaitingUntilKnockSpeed)
{
	LaunchDetector detector{nullptr};
	run(detector, 0.15f, kShotAccel, 0.f);
	EXPECT_EQ(detector.getPublishedState(), Status::STATE_WAITING_FOR_LAUNCH);
	run(detector, 0.05f, kShotAccel, kKnock - 0.1f);
	EXPECT_EQ(detector.getPublishedState(), Status::STATE_WAITING_FOR_LAUNCH);
	run(detector, 0.05f, kShotAccel, kKnock + 0.1f);
	EXPECT_EQ(detector.getPublishedState(), Status::STATE_LAUNCH_DETECTED_DISABLED_MOTOR);

	set("FW_LAUN_SPD_MIN", 0.f);
	LaunchDetector stock{nullptr};
	run(stock, 0.15f, kShotAccel, 0.f);
	EXPECT_EQ(stock.getPublishedState(), Status::STATE_LAUNCH_DETECTED_DISABLED_MOTOR);
}
