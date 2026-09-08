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
#include "GotoSession.hpp"

using matrix::Vector3f;

namespace
{

constexpr uint64_t kStamp = 10000000; // 10 s

const Vector3f kGoto{4.f, -2.f, -3.f};
const Vector3f kVehicle{1.f, 0.5f, -1.5f};
constexpr float kYaw = 0.7f;
constexpr uint64_t kNav = 5000000; // navigator current setpoint stamp
constexpr uint64_t kNavNext = 5100000;

GotoSession::Input freshInput(const uint64_t stamp = kStamp)
{
	GotoSession::Input in;
	in.timestamp = stamp;
	in.position = kGoto;
	in.control_heading = true;
	in.heading = 1.2f;
	in.set_max_horizontal_speed = true;
	in.max_horizontal_speed = 0.25f;
	in.set_max_vertical_speed = true;
	in.max_vertical_speed = 0.4f;
	return in;
}

void expectVector(const Vector3f &actual, const Vector3f &expected)
{
	for (int i = 0; i < 3; i++) {
		EXPECT_FLOAT_EQ(actual(i), expected(i)) << "axis " << i;
	}
}

} // namespace

TEST(GotoSession, NoStreamNeverGoverns)
{
	GotoSession session;
	GotoSession::Input none;

	for (uint64_t now = 0; now < 5 * kStamp; now += kStamp / 4) {
		EXPECT_FALSE(session.update(now, none, true, false, kNav, kVehicle, kYaw));
		EXPECT_FALSE(session.governs());
	}

	session.shiftTarget({1.f, 1.f, 1.f});
	session.shiftHeading(1.f);
	EXPECT_FALSE(session.governs());
}

TEST(GotoSession, FreshStreamSetsTargetHeadingAndCaps)
{
	GotoSession session;
	EXPECT_FALSE(session.update(kStamp + 1000, freshInput(), true, false, kNav, kVehicle, kYaw));

	EXPECT_TRUE(session.governs());
	EXPECT_TRUE(session.fresh());
	expectVector(session.target(), kGoto);
	EXPECT_FLOAT_EQ(session.heading(), 1.2f);
	EXPECT_FLOAT_EQ(session.maxHorizontalSpeed(), 0.25f);
	EXPECT_FLOAT_EQ(session.maxVerticalSpeed(), 0.4f);
}

TEST(GotoSession, UnsetOptionalFieldsReadNaN)
{
	GotoSession session;
	GotoSession::Input in = freshInput();
	in.control_heading = false;
	in.set_max_horizontal_speed = false;
	in.set_max_vertical_speed = false;
	session.update(kStamp + 1000, in, true, false, kNav, kVehicle, kYaw);

	EXPECT_TRUE(session.governs());
	EXPECT_TRUE(std::isnan(session.heading()));
	EXPECT_TRUE(std::isnan(session.maxHorizontalSpeed()));
	EXPECT_TRUE(std::isnan(session.maxVerticalSpeed()));

	// A set flag with a non-finite value counts as unset.
	in.control_heading = true;
	in.heading = NAN;
	in.set_max_horizontal_speed = true;
	in.max_horizontal_speed = INFINITY;
	session.update(kStamp + 2000, in, true, false, kNav, kVehicle, kYaw);
	EXPECT_TRUE(std::isnan(session.heading()));
	EXPECT_TRUE(std::isnan(session.maxHorizontalSpeed()));
}

TEST(GotoSession, StaleStreamHoldsVehicleStateOnce)
{
	GotoSession session;
	session.update(kStamp + 1000, freshInput(), true, false, kNav, kVehicle, kYaw);

	// One cycle past the window: the transition is reported once and the vehicle state is the target.
	const uint64_t stale = kStamp + GotoSession::TIMEOUT_US;
	EXPECT_TRUE(session.update(stale, freshInput(), true, false, kNav, kVehicle, kYaw));
	EXPECT_TRUE(session.governs());
	EXPECT_FALSE(session.fresh());
	expectVector(session.target(), kVehicle);
	EXPECT_FLOAT_EQ(session.heading(), kYaw);
	EXPECT_TRUE(std::isnan(session.maxHorizontalSpeed()));
	EXPECT_TRUE(std::isnan(session.maxVerticalSpeed()));

	// The vehicle moves on; the held target does not follow it.
	const Vector3f later{3.f, 3.f, -3.f};
	EXPECT_FALSE(session.update(stale + kStamp, freshInput(), true, false, kNav, later, 2.f));
	EXPECT_TRUE(session.governs());
	expectVector(session.target(), kVehicle);
	EXPECT_FLOAT_EQ(session.heading(), kYaw);
}

TEST(GotoSession, FreshnessWindowMatchesGotoControl)
{
	// GotoControl stops publishing 500 ms after the last setpoint; the task takes over on the same cycle.
	EXPECT_EQ(GotoSession::TIMEOUT_US, 500000u);

	GotoSession session;
	session.update(kStamp + 499999, freshInput(), true, false, kNav, kVehicle, kYaw);
	EXPECT_TRUE(session.fresh());

	session.update(kStamp + 500000, freshInput(), true, false, kNav, kVehicle, kYaw);
	EXPECT_FALSE(session.fresh());
	EXPECT_TRUE(session.governs());
}

TEST(GotoSession, NonFinitePositionIsNotFresh)
{
	GotoSession session;
	GotoSession::Input in = freshInput();
	in.position(1) = NAN;
	EXPECT_FALSE(session.update(kStamp + 1000, in, true, false, kNav, kVehicle, kYaw));
	EXPECT_FALSE(session.governs());

	// Mid-session the same message ends the fresh phase and holds the vehicle state.
	session.update(kStamp + 1000, freshInput(), true, false, kNav, kVehicle, kYaw);
	EXPECT_TRUE(session.update(kStamp + 2000, in, true, false, kNav, kVehicle, kYaw));
	expectVector(session.target(), kVehicle);
}

TEST(GotoSession, HeldWithoutVehicleStateKeepsGotoTarget)
{
	GotoSession session;
	session.update(kStamp + 1000, freshInput(), true, false, kNav, kVehicle, kYaw);
	const Vector3f unknown{NAN, NAN, NAN};
	EXPECT_TRUE(session.update(kStamp + GotoSession::TIMEOUT_US, freshInput(), true, false, kNav, unknown, kYaw));
	expectVector(session.target(), kGoto);
}

TEST(GotoSession, NavigatorHorizontalTargetEndsSession)
{
	GotoSession session;
	session.update(kStamp + 1000, freshInput(), true, false, kNav, kVehicle, kYaw);
	session.update(kStamp + GotoSession::TIMEOUT_US, freshInput(), true, false, kNav, kVehicle, kYaw);
	EXPECT_TRUE(session.governs());

	EXPECT_FALSE(session.update(kStamp + GotoSession::TIMEOUT_US + 1000, freshInput(), true, true, kNav, kVehicle, kYaw));
	EXPECT_FALSE(session.governs());

	// A later target without horizontal position does not revive a stale session.
	EXPECT_FALSE(session.update(kStamp + GotoSession::TIMEOUT_US + 2000, freshInput(), true, false, kNav, kVehicle, kYaw));
	EXPECT_FALSE(session.governs());
}

TEST(GotoSession, UngovernedTargetTypeEndsSession)
{
	GotoSession session;
	session.update(kStamp + 1000, freshInput(), true, false, kNav, kVehicle, kYaw);
	EXPECT_TRUE(session.fresh());

	// A takeoff target keeps its own climb target even while the stream is fresh.
	EXPECT_FALSE(session.update(kStamp + 2000, freshInput(), false, false, kNav, kVehicle, kYaw));
	EXPECT_FALSE(session.governs());

	// Governance resumes only while the stream is fresh.
	EXPECT_FALSE(session.update(kStamp + 3000, freshInput(), true, false, kNav, kVehicle, kYaw));
	EXPECT_TRUE(session.fresh());
}

TEST(GotoSession, FreshStreamResumesFromHeld)
{
	GotoSession session;
	session.update(kStamp + 1000, freshInput(), true, false, kNav, kVehicle, kYaw);
	session.update(kStamp + GotoSession::TIMEOUT_US, freshInput(), true, false, kNav, kVehicle, kYaw);
	EXPECT_FALSE(session.fresh());

	GotoSession::Input resumed = freshInput(kStamp + 2 * GotoSession::TIMEOUT_US);
	resumed.position = Vector3f{9.f, 9.f, -9.f};
	EXPECT_FALSE(session.update(resumed.timestamp + 1000, resumed, true, false, kNav, kVehicle, kYaw));
	EXPECT_TRUE(session.fresh());
	expectVector(session.target(), resumed.position);
	EXPECT_FLOAT_EQ(session.maxHorizontalSpeed(), 0.25f);
}

TEST(GotoSession, EstimatorResetShiftsHeldTargetOnly)
{
	GotoSession session;
	session.update(kStamp + 1000, freshInput(), true, false, kNav, kVehicle, kYaw);

	// Fresh: the stream owns the target, a reset does not move it.
	session.shiftTarget({1.f, 2.f, 3.f});
	expectVector(session.target(), kGoto);

	session.update(kStamp + GotoSession::TIMEOUT_US, freshInput(), true, false, kNav, kVehicle, kYaw);
	session.shiftTarget({1.f, 2.f, NAN});
	expectVector(session.target(), kVehicle + Vector3f{1.f, 2.f, 0.f});
	session.shiftTarget({NAN, NAN, -0.5f});
	expectVector(session.target(), kVehicle + Vector3f{1.f, 2.f, -0.5f});

	// Heading follows a heading reset and stays wrapped.
	session.shiftHeading(3.f);
	EXPECT_NEAR(session.heading(), matrix::wrap_pi(kYaw + 3.f), 1e-6f);
	EXPECT_LE(session.heading(), float(M_PI));
	EXPECT_GE(session.heading(), -float(M_PI));
}

TEST(GotoSession, ClearEndsSessionUntilNextFreshStream)
{
	GotoSession session;
	session.update(kStamp + 1000, freshInput(), true, false, kNav, kVehicle, kYaw);
	session.update(kStamp + GotoSession::TIMEOUT_US, freshInput(), true, false, kNav, kVehicle, kYaw);
	session.clear();
	EXPECT_FALSE(session.governs());

	EXPECT_FALSE(session.update(kStamp + GotoSession::TIMEOUT_US + 1000, freshInput(), true, false, kNav, kVehicle, kYaw));
	EXPECT_FALSE(session.governs());

	session.update(2 * kStamp + 1000, freshInput(2 * kStamp), true, false, kNav, kVehicle, kYaw);
	EXPECT_TRUE(session.fresh());
}

TEST(GotoSession, NewNavigatorTargetWhileHeldEndsSession)
{
	GotoSession session;
	session.update(kStamp + 1000, freshInput(), true, false, kNav, kVehicle, kYaw);
	session.update(kStamp + 500000, freshInput(), true, false, kNav, kVehicle, kYaw);
	EXPECT_TRUE(session.governs());

	// Same stamp: the navigator republished the same target, the hold stays.
	EXPECT_FALSE(session.update(kStamp + 600000, freshInput(), true, false, kNav, kVehicle, kYaw));
	EXPECT_TRUE(session.governs());
	expectVector(session.target(), kVehicle);

	// New stamp with a stale stream: the navigator target takes over.
	EXPECT_FALSE(session.update(kStamp + 700000, freshInput(), true, false, kNavNext, kVehicle, kYaw));
	EXPECT_FALSE(session.governs());
	EXPECT_FALSE(session.update(kStamp + 800000, freshInput(), true, false, kNavNext, kVehicle, kYaw));
	EXPECT_FALSE(session.governs());
}

TEST(GotoSession, NewNavigatorTargetWhileFreshRestartsFresh)
{
	GotoSession session;
	session.update(kStamp + 1000, freshInput(), true, false, kNav, kVehicle, kYaw);
	EXPECT_TRUE(session.fresh());

	// The same cycle carries a fresh stream: no fallback to the navigator target, session restarts.
	GotoSession::Input moved = freshInput(kStamp + 2000);
	moved.position = Vector3f{7.f, 7.f, -7.f};
	EXPECT_FALSE(session.update(kStamp + 3000, moved, true, false, kNavNext, kVehicle, kYaw));
	EXPECT_TRUE(session.fresh());
	expectVector(session.target(), moved.position);

	// The new stamp is the recorded one: it holds on a stale stream, the old stamp ends the session.
	EXPECT_TRUE(session.update(kStamp + 2000 + 500000, moved, true, false, kNavNext, kVehicle, kYaw));
	EXPECT_TRUE(session.governs());
	EXPECT_FALSE(session.update(kStamp + 2000 + 600000, moved, true, false, kNav, kVehicle, kYaw));
	EXPECT_FALSE(session.governs());
}
