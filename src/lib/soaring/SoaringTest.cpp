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

#include <algorithm>
#include <cmath>
#include <math.h>
#include <random>
#include <vector>

#include "Soaring.hpp"

using namespace soaring;

static constexpr float kG = 9.80665f;
static constexpr float kDeg = 0.01745329f;
static constexpr float kDt = 0.02f;
static constexpr float kPi = 3.14159265f;

// X8 still-air sink by true airspeed, wings level, from the simulation model's polar
static const float kTableV[] = {12.f, 13.f, 14.f, 15.f, 16.f, 17.f, 18.f, 19.f, 20.f};
static const float kTableSink[] = {1.10f, 1.24f, 1.41f, 1.61f, 1.86f, 2.13f, 2.45f, 2.81f, 3.21f};

static float tableSink(float v)
{
	int i = 0;

	while (i < 7 && v > kTableV[i + 1]) {
		i++;
	}

	const float f = (v - kTableV[i]) / (kTableV[i + 1] - kTableV[i]);
	return kTableSink[i] + f * (kTableSink[i + 1] - kTableSink[i]);
}

// same lift coefficient in a turn: speed scales by sqrt(n), sink by n^1.5
static float tableSinkTurn(float v, float bank)
{
	const float c = cosf(bank);
	return tableSink(v * sqrtf(c)) / (c * sqrtf(c));
}

// Allen 2006 bell updraft, equations 11 to 23 without the environment sink
struct Allen {
	float wstar{4.08f};
	float zi{2819.f};

	float operator()(float r, float z) const
	{
		static const float rows[7][5] = {
			{0.14f, 1.5352f, 2.5826f, -0.0113f, 0.0008f},
			{0.25f, 1.5265f, 3.6054f, -0.0176f, 0.0005f},
			{0.36f, 1.4866f, 4.8354f, -0.0320f, 0.0001f},
			{0.47f, 1.2042f, 7.7904f, 0.0848f, 0.0001f},
			{0.58f, 0.8816f, 13.9720f, 0.3404f, 0.0001f},
			{0.69f, 0.7067f, 23.9940f, 0.5689f, 0.0002f},
			{0.80f, 0.6189f, 42.7965f, 0.7157f, 0.0001f},
		};
		const float zr = std::min(z / zi, 0.5f);
		const float c = cbrtf(zr);
		const float wbar = wstar * c * (1.f - 1.1f * zr);
		const float r2 = std::max(10.f, 0.102f * c * (1.f - 0.25f * zr) * zi);
		const float ratio = r2 < 600.f ? 0.0011f * r2 + 0.14f : 0.8f;
		const float r1 = ratio * r2;
		const float wpeak = 3.f * wbar * (r2 * r2 * r2 - r2 * r2 * r1) / (r2 * r2 * r2 - r1 * r1 * r1);
		int k = 0;

		for (int i = 1; i < 7; i++) {
			if (fabsf(rows[i][0] - ratio) < fabsf(rows[k][0] - ratio)) {
				k = i;
			}
		}

		const float x = r / r2;
		return wpeak * (1.f / (1.f + powf(fabsf(rows[k][1] * x + rows[k][3]), rows[k][2])) + rows[k][4] * x);
	}
};

// Steps the library at 50 Hz with a point-mass aircraft in the air w (m/s up); it circles 60 m about the origin.
struct Rig {
	Soaring soar;
	Params p;
	Input in;
	Output out;
	float t{0.f};
	float w{0.f};
	float speed{14.f};
	float accel{0.f};
	float psi{0.f};
	float rate{14.f / 60.f};
	bool motor{true};		// CLIMB adds energy for a climb_powered climb
	float climb_powered{3.f};	// [m/s]
	float throttle{1.f};		// TECS throttle setpoint in CLIMB

	Rig(bool clean_turn = true)
	{
		p.enabled = true;
		p.gf_max_ver = 450.f;
		p.gf_max_hor = 500.f;
		in.armed = true;
		in.allowed = true;
		in.inhibit_reason = REASON_NONE;
		in.height = 200.f;
		in.wind_valid = true;
		in.hold_x = 0.f;
		in.hold_y = 0.f;
		in.nav_radius = 60.f;
		in.x = 0.f;
		in.y = -60.f;

		// soaring starts only after one full turn with a pitot that agrees with GPS
		if (clean_turn) {
			p.enabled = false;
			step(2.f * kPi / rate + 0.1f);
			p.enabled = true;
			in.height = 200.f;
			in.x = 0.f;
			in.y = -60.f;
		}
	}

	void step(float seconds)
	{
		const int n = (int)lroundf(seconds / kDt);

		for (int i = 0; i < n; i++) {
			t += kDt;
			psi += rate * kDt;
			speed += accel * kDt;
			const float roll = atanf(speed * rate / kG);
			const float powered = (motor && out.state == State::CLIMB) ? tableSinkTurn(speed, roll) + climb_powered : 0.f;
			const float climb = w + powered - tableSinkTurn(speed, roll) - speed * accel / kG;
			in.time_us = (uint64_t)lroundf(t * 1e6f);
			in.vn = speed * cosf(psi) + in.wind_n;
			in.ve = speed * sinf(psi) + in.wind_e;
			in.x += in.vn * kDt;
			in.y += in.ve * kDt;
			in.vz = -climb;
			in.height += climb * kDt;
			in.cas = speed;
			in.tas = speed;
			in.roll = roll;
			in.yaw = psi;
			in.throttle = out.state == State::CLIMB ? throttle : 0.f;
			soar.update(in, p, out);
		}
	}

	void toGlide()
	{
		step(2.1f);
		ASSERT_EQ(out.state, State::GLIDE);
	}

	// lift with netto 3 m/s: expected climb 3 less the 1.91 m/s thermalling sink
	void toThermal()
	{
		toGlide();
		w = 3.f;
		const float t_end = t + p.min_crse_s + 3.f;

		while (out.state == State::GLIDE && t < t_end) {
			step(kDt);
		}

		ASSERT_EQ(out.state, State::THERMAL);
	}
};

static State holdHeight(Rig &rig, float height, float seconds)
{
	for (int i = 0; i < (int)lroundf(seconds / kDt); i++) {
		rig.in.height = height;
		rig.step(kDt);
	}

	return rig.out.state;
}

TEST(SoaringPolar, MatchesStudyTable)
{
	Params p;

	for (int i = 2; i <= 6; i++) {
		EXPECT_NEAR(Soaring::sink(p, kTableV[i], 1.f, 0.f), kTableSink[i], 0.03) << kTableV[i];
	}
}

TEST(SoaringPolar, BankAtThermallingSpeed)
{
	Params p;
	const float bank = 35.f * kDeg;
	const float v = 14.f / sqrtf(cosf(bank));
	// table 1.41 m/s at 14 m/s, times n^1.5 = 1.349: 1.902 m/s at 15.47 m/s
	EXPECT_NEAR(Soaring::sink(p, v, 1.f, bank), tableSinkTurn(v, bank), 0.03);
}

TEST(SoaringPolar, TasOverCas)
{
	Params p;
	EXPECT_NEAR(Soaring::sink(p, 15.f, 1.1f, 0.f), 1.1f * Soaring::sink(p, 15.f, 1.f, 0.f), 1e-5);
}

TEST(SoaringEstimator, JacobianMatchesNumericDerivative)
{
	const float states[3][4] = {{2.5f, 70.f, 30.f, -20.f}, {1.f, 40.f, -50.f, 10.f}, {4.f, 120.f, 5.f, 80.f}};
	const float step[4] = {1e-3f, 1e-2f, 1e-2f, 1e-2f};

	for (const auto &s0 : states) {
		float H[4];
		Soaring::jacobian(s0, H);

		for (int i = 0; i < 4; i++) {
			float a[4] = {s0[0], s0[1], s0[2], s0[3]};
			float b[4] = {s0[0], s0[1], s0[2], s0[3]};
			a[i] += step[i];
			b[i] -= step[i];
			const float numeric = (Soaring::updraft(a) - Soaring::updraft(b)) / (2.f * step[i]);
			EXPECT_NEAR(H[i], numeric, 1e-3f + 1e-3f * fabsf(numeric)) << i;
		}
	}
}

TEST(SoaringVario, NettoZeroInStillAirGlide)
{
	const float cases[3][2] = {{14.f, 0.f}, {18.f, 0.f}, {14.f / sqrtf(cosf(35.f * kDeg)), 35.f * kDeg}};

	for (const auto &c : cases) {
		Soaring soar;
		Params p;
		Input in;
		Output out;
		in.cas = in.tas = c[0];
		in.roll = c[1];
		in.vz = tableSinkTurn(c[0], c[1]);

		for (int i = 1; i <= 500; i++) {
			in.time_us = (uint64_t)i * 20000;
			soar.update(in, p, out);
		}

		EXPECT_NEAR(out.vario_netto, 0.f, 0.05) << c[0];
		EXPECT_NEAR(out.vario_netto_slow, 0.f, 0.05) << c[0];
	}
}

TEST(SoaringVario, NettoZeroWhileTradingSpeedForHeight)
{
	// energy holds: climb = -sink - V dV/dt / g; 18 to 14 m/s over 16 s gives 0.4 m/s of exchange
	Soaring soar;
	Params p;
	Input in;
	Output out;
	const float dvdt = -0.25f;
	float v = 18.f;

	for (int i = 1; i <= 800; i++) {
		in.time_us = (uint64_t)i * 20000;
		in.cas = in.tas = v;
		in.vz = tableSink(v) + v * dvdt / kG;
		in.accel_x = dvdt;
		soar.update(in, p, out);

		if (i > 100) {
			ASSERT_NEAR(out.vario_netto, 0.f, 0.05) << v;
			ASSERT_NEAR(out.vario_te_imu, out.vario_te, 0.05) << v;
		}

		v += dvdt * kDt;
	}
}

TEST(SoaringStates, OffWhenDisabled)
{
	Rig rig;
	rig.toGlide();
	rig.p.enabled = false;
	rig.step(0.02f);
	EXPECT_EQ(rig.out.state, State::OFF);
}

TEST(SoaringStates, InhibitWhenNotAllowed)
{
	Rig rig;
	rig.toGlide();
	rig.in.allowed = false;
	rig.in.inhibit_reason = REASON_BATTERY;
	rig.step(0.02f);
	EXPECT_EQ(rig.out.state, State::INHIBIT);
	EXPECT_EQ(rig.out.reason, REASON_BATTERY);
}

TEST(SoaringStates, InhibitBandUnderSiteCeiling)
{
	Rig rig;
	rig.p.gf_max_ver = 60.f;
	rig.in.height = 45.f;
	rig.step(5.f);
	EXPECT_EQ(rig.out.state, State::INHIBIT);
	EXPECT_EQ(rig.out.reason, REASON_BAND);
	EXPECT_FLOAT_EQ(rig.out.alt_max, 30.f);
}

TEST(SoaringStates, InhibitBandCutUnderFloor)
{
	Rig rig;
	rig.p.alt_cut = rig.p.alt_min + 29.f;
	rig.step(5.f);
	EXPECT_EQ(rig.out.reason, REASON_BAND);
}

TEST(SoaringStates, InhibitRadiusWhenHoldOutsideCircle)
{
	Rig rig;
	// R_soar min(350, 500 - 100) = 350; Hold 300 m out plus 60 m radius
	rig.in.hold_x = 300.f;
	rig.step(5.f);
	EXPECT_EQ(rig.out.reason, REASON_RADIUS);
	rig.in.hold_x = 280.f;
	rig.step(2.1f);
	EXPECT_EQ(rig.out.state, State::GLIDE);
}

TEST(SoaringStates, GoodForTwoSecondsBeforeGlide)
{
	Rig rig;
	rig.step(1.9f);
	EXPECT_EQ(rig.out.state, State::INHIBIT);
	rig.step(0.2f);
	EXPECT_EQ(rig.out.state, State::GLIDE);
}

TEST(SoaringStates, ClimbFromInhibitAtFloorThenGlideAtCut)
{
	Rig rig;
	rig.in.height = 100.f;
	rig.step(2.1f);
	EXPECT_EQ(rig.out.state, State::CLIMB);
	EXPECT_FLOAT_EQ(rig.out.alt_sp, 255.f);
	rig.in.height = 249.f;
	rig.step(0.02f);
	EXPECT_EQ(rig.out.state, State::CLIMB);
	rig.in.height = 250.1f;
	rig.step(0.02f);
	EXPECT_EQ(rig.out.state, State::GLIDE);
}

TEST(SoaringStates, GlideToClimbAtFloor)
{
	Rig rig;
	rig.toGlide();
	rig.in.height = 100.f;
	rig.step(0.02f);
	EXPECT_EQ(rig.out.state, State::CLIMB);
	EXPECT_EQ(rig.out.reason, REASON_FLOOR);
}

TEST(SoaringStates, ThermalEntryWaitsForMinimumGlide)
{
	Rig rig;
	rig.toGlide();
	rig.w = 3.f;
	rig.step(rig.p.min_crse_s - 0.2f);
	EXPECT_EQ(rig.out.state, State::GLIDE);
	EXPECT_GT(rig.out.climb_expected, rig.p.vspeed);
	rig.step(0.3f);
	EXPECT_EQ(rig.out.state, State::THERMAL);
	EXPECT_EQ(rig.out.thermal_count, 1);
}

TEST(SoaringStates, NoThermalEntryInClimb)
{
	Rig rig;
	rig.in.height = 90.f;
	rig.step(2.1f);
	ASSERT_EQ(rig.out.state, State::CLIMB);
	rig.w = 6.f;
	EXPECT_EQ(holdHeight(rig, 150.f, 40.f), State::CLIMB);
}

TEST(SoaringStates, NoThermalEntryOutsideBand)
{
	Rig low;
	low.toGlide();
	low.w = 3.f;
	EXPECT_EQ(holdHeight(low, 119.f, 40.f), State::GLIDE);

	Rig high;
	high.toGlide();
	high.w = 3.f;
	EXPECT_EQ(holdHeight(high, 371.f, 40.f), State::GLIDE);

	Rig far;
	far.in.x = 345.f;
	far.toGlide();
	far.w = 3.f;
	EXPECT_EQ(holdHeight(far, 200.f, 40.f), State::GLIDE);
}

TEST(SoaringStates, WeakLiftExitOnlyAfterMinimumThermalTime)
{
	Rig rig;
	rig.toThermal();
	rig.w = 0.f;
	rig.step(rig.p.min_thml_s - 0.5f);
	EXPECT_EQ(rig.out.state, State::THERMAL);
	rig.step(1.f);
	EXPECT_EQ(rig.out.state, State::GLIDE);
	EXPECT_EQ(rig.out.reason, REASON_WEAK_LIFT);
}

TEST(SoaringStates, CeilingExitFiveSecondsAhead)
{
	Rig rig;
	rig.toThermal();
	// climbing 1.5 m/s 4 m under the 400 m ceiling: the 5 s lead reaches it
	rig.in.height = 396.f;
	rig.step(0.04f);
	EXPECT_GT(rig.out.vario_te, 0.9f);
	EXPECT_EQ(rig.out.state, State::GLIDE);
	EXPECT_EQ(rig.out.reason, REASON_CEILING);
}

TEST(SoaringStates, GlideHoldsMaximumAirspeedDownToCeilingLess20)
{
	Rig rig;
	rig.in.height = 380.f;
	rig.toGlide();
	EXPECT_FLOAT_EQ(rig.out.airspeed_sp, rig.p.airspd_min);
	// ceiling min(400, 450 - 30) = 400
	const float heights[] = {401.f, 399.f, 390.f, 380.5f};

	for (float h : heights) {
		EXPECT_EQ(holdHeight(rig, h, 0.04f), State::GLIDE) << h;
		EXPECT_FLOAT_EQ(rig.out.airspeed_sp, rig.p.airspd_max) << h;
	}

	holdHeight(rig, 379.5f, 0.04f);
	EXPECT_FLOAT_EQ(rig.out.airspeed_sp, rig.p.airspd_min);
	holdHeight(rig, 395.f, 0.04f);
	EXPECT_FLOAT_EQ(rig.out.airspeed_sp, rig.p.airspd_min);
}

TEST(SoaringStates, ClimbHoldsHeightAboveTheCut)
{
	Rig rig;
	rig.in.height = 380.f;
	rig.toGlide();
	rig.speed = 11.5f;
	float entry = NAN;

	for (int i = 0; i < 60 && rig.out.state == State::GLIDE; i++) {
		entry = rig.in.height;
		rig.step(kDt);
	}

	ASSERT_EQ(rig.out.state, State::CLIMB);
	ASSERT_EQ(rig.out.reason, REASON_AIRSPEED_LOW);
	// the cut setpoint would be 255 m; the entry height, within one 50 Hz sample, is latched
	EXPECT_NEAR(rig.out.alt_sp, entry, 0.1f);
	rig.speed = 14.f;
	rig.step(5.f);
	ASSERT_EQ(rig.out.state, State::CLIMB);
	EXPECT_GT(rig.in.height, entry + 10.f);
	EXPECT_NEAR(rig.out.alt_sp, entry, 0.1f);
}

TEST(SoaringStates, ClimbSetpointCappedAtTheCeiling)
{
	// ceiling min(400, 450 - 30) = 400; an airspeed-low entry at 410 m climbs to 400 m, not 410 m
	Rig rig;
	rig.in.height = 410.f;
	rig.toGlide();
	rig.speed = 11.5f;
	rig.step(1.1f);
	ASSERT_EQ(rig.out.state, State::CLIMB);
	EXPECT_FLOAT_EQ(rig.out.alt_sp, 400.f);
}

TEST(SoaringStates, ClimbSetpointIgnoresLiftWhileTheCutGateIsClosed)
{
	// floor 222.9 m above the cut less 30 m: CLIMB stays on and must hold cut + 5 m, not the height reached in lift
	Rig rig;
	rig.p.alt_pat = 215.f;
	rig.in.height = 200.f;
	rig.step(2.1f);
	ASSERT_EQ(rig.out.state, State::CLIMB);
	EXPECT_FLOAT_EQ(rig.out.alt_sp, 255.f);
	rig.w = 3.f;
	rig.step(30.f);
	ASSERT_EQ(rig.out.state, State::CLIMB);
	EXPECT_GT(rig.in.height, 300.f);
	EXPECT_FLOAT_EQ(rig.out.alt_sp, 255.f);
}

TEST(SoaringStates, ClimbFliesSpeedPriorityThroughoutAndTrimSpeedInTheLast50m)
{
	// a dead motor in CLIMB must cost height, not airspeed; the cut must start with trim speed in hand, not FW_AIRSPD_MIN
	Rig rig;
	rig.in.height = 100.f;
	rig.step(2.1f);
	ASSERT_EQ(rig.out.state, State::CLIMB);
	const float entry = rig.in.height;
	int cruise = 0;
	int trim = 0;

	while (rig.out.state == State::CLIMB && rig.t < 120.f) {
		ASSERT_FLOAT_EQ(rig.out.speed_weight, 2.f) << rig.in.height;

		if (rig.in.height < 250.f - 50.f) {
			ASSERT_FLOAT_EQ(rig.out.airspeed_sp, rig.p.airspd_min) << rig.in.height;
			cruise++;

		} else {
			ASSERT_FLOAT_EQ(rig.out.airspeed_sp, rig.p.airspd_trim) << rig.in.height;
			trim++;
		}

		rig.step(kDt);
	}

	// at 3 m/s and 50 Hz: entry to 200 m at FW_AIRSPD_MIN, then 200 m to the 250 m cut, 16.7 s, at FW_AIRSPD_TRIM
	EXPECT_NEAR(cruise, (200.f - entry) / 3.f * 50.f, 5.f);
	EXPECT_NEAR(trim, 50.f / 3.f * 50.f, 5.f);
	ASSERT_EQ(rig.out.state, State::GLIDE);
	EXPECT_TRUE(std::isnan(rig.out.speed_weight));
	EXPECT_FLOAT_EQ(rig.out.airspeed_sp, rig.p.airspd_min);

	// a trim above FW_AIRSPD_MAX flies FW_AIRSPD_MAX; the 222.9 m floor holds CLIMB from 200 m
	Rig capped;
	capped.p.airspd_trim = 25.f;
	capped.p.alt_pat = 215.f;
	capped.step(2.1f);
	ASSERT_EQ(capped.out.state, State::CLIMB);
	capped.step(10.f);
	ASSERT_EQ(capped.out.state, State::CLIMB);
	ASSERT_GE(capped.in.height, 200.f);
	EXPECT_FLOAT_EQ(capped.out.airspeed_sp, capped.p.airspd_max);
}

TEST(SoaringStates, RadiusExitWhenCircleLeavesFence)
{
	Rig rig;
	rig.in.x = 180.f;
	rig.toThermal();
	// the air mass carries the circle north at 8 m/s toward R_soar 350 m
	rig.in.wind_n = 8.f;
	float radius = 0.f;

	for (int i = 0; i < 30 * 50 && rig.out.state == State::THERMAL; i++) {
		radius = rig.out.loiter_radius;
		rig.step(kDt);
	}

	EXPECT_EQ(rig.out.state, State::GLIDE);
	EXPECT_EQ(rig.out.reason, REASON_RADIUS);
	EXPECT_NEAR(hypotf(rig.out.centre_x, rig.out.centre_y) + radius, 350.f, 1.f);
}

TEST(SoaringStates, HoldPointMoveKeepsThermal)
{
	Rig rig;
	rig.toThermal();
	rig.in.hold_x = 150.f;
	rig.step(1.f);
	EXPECT_EQ(rig.out.state, State::THERMAL);
}

TEST(SoaringStates, EstimatorFaultExit)
{
	Rig rig;
	rig.toThermal();
	rig.in.wind_n = NAN;
	rig.soar.update(rig.in, rig.p, rig.out);
	rig.in.time_us += 20000;
	rig.soar.update(rig.in, rig.p, rig.out);
	EXPECT_EQ(rig.out.state, State::GLIDE);
	EXPECT_EQ(rig.out.reason, REASON_ESTIMATOR);
}

TEST(SoaringStates, ThermalToClimbAtFloor)
{
	Rig rig;
	rig.in.height = 125.f;
	rig.toThermal();
	rig.in.height = 99.f;
	rig.step(0.02f);
	EXPECT_EQ(rig.out.state, State::CLIMB);
	EXPECT_EQ(rig.out.reason, REASON_FLOOR);
}

TEST(SoaringStates, AirspeedLowRestartsMotorAndBarsGlide)
{
	Rig rig;
	rig.in.height = 300.f;
	rig.toGlide();
	// threshold 14 sqrt(n) - 2 = 12 m/s wings level
	rig.speed = 11.5f;
	rig.step(0.9f);
	EXPECT_EQ(rig.out.state, State::GLIDE);
	rig.step(0.2f);
	EXPECT_EQ(rig.out.state, State::CLIMB);
	EXPECT_EQ(rig.out.reason, REASON_AIRSPEED_LOW);
	rig.speed = 14.f;
	rig.step(19.5f);
	EXPECT_EQ(rig.out.state, State::CLIMB);
	rig.step(1.f);
	EXPECT_EQ(rig.out.state, State::GLIDE);
}

TEST(SoaringStates, ThermalLeftRecordsGain)
{
	Rig rig;
	rig.toThermal();
	const float h0 = rig.in.height;
	rig.step(10.f);
	rig.in.allowed = false;
	rig.step(0.02f);
	EXPECT_EQ(rig.out.state, State::INHIBIT);
	EXPECT_NEAR(rig.out.last_gain, rig.in.height - h0, 0.1);
	EXPECT_NEAR(rig.out.last_time, 10.f, 0.05);
	EXPECT_NEAR(rig.out.height_gained, rig.out.last_gain, 1e-3);
}

// TAS over CAS 1.0243 makes FW_AIRSPD_MIN 14 m/s a 14.34 m/s TAS with a 1.452 m/s polar sink
static float floorAt(float north, float wind_n, float alt_pat = 80.f, float east = 0.f, float wind_e = 0.f)
{
	Soaring soar;
	Params p;
	Input in;
	Output out;
	p.alt_pat = alt_pat;
	in.armed = true;
	in.x = north;
	in.y = east;
	in.cas = 14.f;
	in.tas = 14.34f;
	in.wind_n = wind_n;
	in.wind_e = wind_e;
	in.wind_valid = true;
	soar.update(in, p, out);
	return out.alt_floor;
}

TEST(SoaringFloor, GlideReachHome)
{
	// reach term 80 + 1.3 x 150 x 1.452 / 14.34 = 99.7 m, under SOAR_ALT_MIN
	EXPECT_NEAR(floorAt(150.f, 0.f), 100.f, 0.05f);
	EXPECT_NEAR(floorAt(350.f, 0.f), 80.f + 1.3f * 350.f * 1.4528f / 14.34f, 0.1f);
	EXPECT_NEAR(floorAt(350.f, 0.f), 126.1f, 0.1f);
	// home lies south: a wind toward the north is a headwind on the way home, 14.34 - 8 = 6.34 m/s
	EXPECT_NEAR(floorAt(350.f, 8.f), 184.3f, 0.1f);
	EXPECT_NEAR(floorAt(350.f, -8.f), 109.6f, 0.1f);
}

TEST(SoaringFloor, GlideReachHomeEastOfHome)
{
	// home lies west: a wind toward the east is the same 6.34 m/s headwind, a wind toward the north is a crosswind
	EXPECT_NEAR(floorAt(0.f, 0.f, 80.f, 350.f, 8.f), 184.3f, 0.1f);
	EXPECT_NEAR(floorAt(0.f, 0.f, 80.f, 350.f, -8.f), 109.6f, 0.1f);
	EXPECT_NEAR(floorAt(0.f, 8.f, 80.f, 350.f, 0.f), 126.1f, 0.1f);
	// north-east at 247.5 m each way, 350 m out, wind 8 m/s toward the north-east: the full 8 m/s against
	EXPECT_NEAR(floorAt(247.49f, 5.657f, 80.f, 247.49f, 5.657f), 184.3f, 0.1f);
}

TEST(SoaringFloor, TransitionsUseTheFloor)
{
	// Rig circles 60 m about home: floor = SOAR_ALT_PAT + 1.3 x 60 x 1.418 / 14 = SOAR_ALT_PAT + 7.9 m
	Rig rig;
	rig.p.alt_pat = 150.f;
	rig.in.height = 170.f;
	rig.toGlide();
	EXPECT_NEAR(rig.out.alt_floor, 157.9f, 0.2f);
	EXPECT_EQ(holdHeight(rig, 158.5f, 0.1f), State::GLIDE);
	EXPECT_EQ(holdHeight(rig, 157.5f, 0.1f), State::CLIMB);
	EXPECT_EQ(rig.out.reason, REASON_FLOOR);
}

TEST(SoaringFloor, NoMotorCutWhileFloorAboveCutLess30)
{
	Rig rig;
	rig.p.alt_pat = 215.f;
	rig.in.height = 200.f;
	rig.step(2.1f);
	ASSERT_EQ(rig.out.state, State::CLIMB);
	// floor 222.9 m above the cut less 30 m, 220 m
	EXPECT_EQ(holdHeight(rig, 260.f, 5.f), State::CLIMB);
	rig.p.alt_pat = 205.f;
	EXPECT_EQ(holdHeight(rig, 260.f, 0.1f), State::GLIDE);
}

TEST(SoaringVario, NoEntryOnMotorEnergyAfterTheCut)
{
	Rig rig;
	rig.p.min_crse_s = 0.f;
	rig.in.height = 100.f;
	rig.step(2.1f);
	ASSERT_EQ(rig.out.state, State::CLIMB);

	while (rig.out.state == State::CLIMB && rig.t < 200.f) {
		rig.step(kDt);
	}

	ASSERT_EQ(rig.out.state, State::GLIDE);
	float peak = 0.f;

	for (int i = 0; i < 30 * 50; i++) {
		rig.step(kDt);
		ASSERT_EQ(rig.out.state, State::GLIDE) << i;
		peak = std::max(peak, rig.out.vario_netto_slow);
	}

	// 4.4 m/s through the 0.5 s and 2 s filters: 4.4 x 0.5 / 1.5 x (e^-0.46 - e^-1.85) = 0.69 m/s, under 2.46
	EXPECT_LE(peak, 0.75f);
	EXPECT_GT(peak, 0.5f);
}

// lift of netto 3 m/s while soaring waits: the entry filter reads it, and GLIDE must start from 0
static float nettoAtGlideEntry(bool from_off)
{
	Rig rig;
	rig.p.min_crse_s = 0.f;
	rig.toGlide();
	rig.w = 3.f;

	if (from_off) {
		rig.p.enabled = false;

	} else {
		rig.in.allowed = false;
		rig.in.inhibit_reason = REASON_BATTERY;
	}

	rig.step(10.f);
	EXPECT_LT(rig.out.state, State::CLIMB);
	EXPECT_GT(rig.out.vario_netto_slow, 2.8f);
	rig.p.enabled = true;
	rig.in.allowed = true;
	rig.in.inhibit_reason = REASON_NONE;

	const float t_end = rig.t + 5.f;

	while (rig.out.state < State::CLIMB && rig.t < t_end) {
		rig.step(kDt);
	}

	EXPECT_EQ(rig.out.state, State::GLIDE);
	return rig.out.vario_netto_slow;
}

TEST(SoaringVario, NettoResetOnGlideEntryFromInhibitAndOff)
{
	EXPECT_FLOAT_EQ(nettoAtGlideEntry(false), 0.f);
	EXPECT_FLOAT_EQ(nettoAtGlideEntry(true), 0.f);
}

static void toState(Rig &rig, State state)
{
	if (state == State::CLIMB) {
		rig.in.height = 100.f;
		rig.step(2.1f);

	} else if (state == State::GLIDE) {
		rig.toGlide();

	} else {
		rig.toThermal();
	}

	ASSERT_EQ(rig.out.state, state);
}

TEST(SoaringLatch, OperatorCommandEndsEveryActiveState)
{
	const State states[] = {State::CLIMB, State::GLIDE, State::THERMAL};

	const uint8_t kinds[] = {OPERATOR_MODE, OPERATOR_NAV};

	for (State state : states) {
		for (uint8_t kind : kinds) {
			Rig rig;
			toState(rig, state);
			rig.in.operator_cmd = kind;
			rig.step(kDt);
			rig.in.operator_cmd = 0;
			EXPECT_EQ(rig.out.state, State::INHIBIT) << (int)state << " " << (int)kind;
			EXPECT_EQ(rig.out.reason, REASON_OPERATOR) << (int)state << " " << (int)kind;
		}
	}
}

TEST(SoaringLatch, HoldsUntilDisabledOrDisarmed)
{
	Rig rig;
	rig.toGlide();
	rig.in.operator_cmd = OPERATOR_NAV;
	rig.step(kDt);
	rig.in.operator_cmd = 0;
	rig.step(5.f);
	EXPECT_EQ(rig.out.reason, REASON_OPERATOR);
	rig.in.allowed = false;
	rig.in.inhibit_reason = REASON_MODE;
	rig.step(1.f);
	EXPECT_EQ(rig.out.reason, REASON_OPERATOR);
	rig.in.allowed = true;
	rig.step(5.f);
	EXPECT_EQ(rig.out.state, State::INHIBIT);
	EXPECT_EQ(rig.out.reason, REASON_OPERATOR);

	rig.p.enabled = false;
	rig.step(kDt);
	EXPECT_EQ(rig.out.state, State::OFF);
	rig.p.enabled = true;
	rig.step(2.1f);
	EXPECT_EQ(rig.out.state, State::GLIDE);

	rig.in.operator_cmd = OPERATOR_MODE;
	rig.step(kDt);
	rig.in.operator_cmd = 0;
	rig.in.armed = false;
	rig.step(kDt);
	rig.in.armed = true;
	// a new flight waits for its own clean turn, 27 s on the 60 m circle
	rig.step(2.1f);
	EXPECT_EQ(rig.out.state, State::INHIBIT);
	EXPECT_EQ(rig.out.reason, REASON_NONE);
	rig.step(27.f);
	EXPECT_EQ(rig.out.state, State::GLIDE);
}

TEST(SoaringLatch, ModeFailsafeAndMotorEndsLatch)
{
	const uint8_t reasons[] = {REASON_MODE, REASON_FAILSAFE, REASON_MOTOR};

	for (uint8_t reason : reasons) {
		Rig rig;
		rig.toGlide();
		rig.in.allowed = false;
		rig.in.inhibit_reason = reason;
		rig.step(kDt);
		rig.in.allowed = true;
		rig.step(5.f);
		EXPECT_EQ(rig.out.state, State::INHIBIT) << (int)reason;
		EXPECT_EQ(rig.out.reason, reason);
	}

	// a battery inhibit is not latched
	Rig rig;
	rig.toGlide();
	rig.in.allowed = false;
	rig.in.inhibit_reason = REASON_BATTERY;
	rig.step(kDt);
	rig.in.allowed = true;
	rig.step(2.1f);
	EXPECT_EQ(rig.out.state, State::GLIDE);
}

// One command in INHIBIT with the given reason, which clears 1 s later; returns the output 5 s after that.
static Output afterCommandInInhibit(uint8_t reason, uint8_t kind, bool glide_first)
{
	Rig rig;

	if (glide_first) {
		rig.toGlide();
	}

	if (reason == REASON_WIND) {
		rig.in.wind_valid = false;

	} else if (reason != REASON_NONE) {
		rig.in.allowed = false;
		rig.in.inhibit_reason = reason;
	}

	rig.step(0.5f);
	EXPECT_EQ(rig.out.state, State::INHIBIT);
	rig.in.operator_cmd = kind;
	rig.step(kDt);
	rig.in.operator_cmd = 0;
	rig.step(0.5f);
	rig.in.wind_valid = true;
	rig.in.allowed = true;
	rig.in.inhibit_reason = REASON_NONE;
	rig.step(5.f);
	return rig.out;
}

TEST(SoaringLatch, CommandInAnInhibitOtherThanModeIsLatched)
{
	const uint8_t reasons[] = {REASON_WIND, REASON_BATTERY, REASON_AIRSPEED, REASON_HOME};

	for (uint8_t reason : reasons) {
		for (bool glide_first : {true, false}) {
			const Output o = afterCommandInInhibit(reason, OPERATOR_NAV, glide_first);
			EXPECT_EQ(o.state, State::INHIBIT) << (int)reason << glide_first;
			EXPECT_EQ(o.reason, REASON_OPERATOR) << (int)reason << glide_first;
		}
	}
}

TEST(SoaringLatch, CommandBeforeSoaringStartsIsLatched)
{
	Rig rig;
	rig.step(1.f);
	ASSERT_EQ(rig.out.state, State::INHIBIT);
	ASSERT_EQ(rig.out.reason, REASON_NONE);
	rig.in.operator_cmd = OPERATOR_NAV;
	rig.step(kDt);
	rig.in.operator_cmd = 0;
	rig.step(5.f);
	EXPECT_EQ(rig.out.state, State::INHIBIT);
	EXPECT_EQ(rig.out.reason, REASON_OPERATOR);
}

TEST(SoaringLatch, ModeCommandOrModeInhibitIsNotLatched)
{
	// selecting Hold is a mode command, and a command outside Hold is not addressed to soaring
	EXPECT_EQ(afterCommandInInhibit(REASON_MODE, OPERATOR_NAV, false).state, State::GLIDE);
	EXPECT_EQ(afterCommandInInhibit(REASON_MODE, OPERATOR_MODE, false).state, State::GLIDE);
	EXPECT_EQ(afterCommandInInhibit(REASON_WIND, OPERATOR_MODE, true).state, State::GLIDE);
	EXPECT_EQ(afterCommandInInhibit(REASON_NONE, OPERATOR_MODE, false).state, State::GLIDE);
}

TEST(SoaringStates, WindInvalidInhibitsEveryState)
{
	const State states[] = {State::CLIMB, State::GLIDE, State::THERMAL};

	for (State state : states) {
		Rig rig;
		toState(rig, state);
		rig.in.wind_valid = false;
		rig.step(kDt);
		EXPECT_EQ(rig.out.state, State::INHIBIT) << (int)state;
		EXPECT_EQ(rig.out.reason, REASON_WIND) << (int)state;
		rig.in.wind_valid = true;
		rig.step(2.1f);
		EXPECT_GE(rig.out.state, State::CLIMB) << (int)state;
	}
}

// Glides to the 100 m floor, then the motor gives no thrust for dead_s seconds after the crossing.
static float restartFails(float dead_s)
{
	Rig rig;
	rig.in.height = 106.f;
	rig.toGlide();
	rig.motor = false;

	while (rig.out.state == State::GLIDE && rig.t < 100.f) {
		rig.step(kDt);
	}

	EXPECT_EQ(rig.out.reason, REASON_FLOOR);
	const float t_floor = rig.t;

	while (rig.out.state == State::CLIMB && rig.t < t_floor + 20.f) {
		rig.motor = rig.t - t_floor >= dead_s;
		rig.step(kDt);
	}

	return rig.out.state == State::INHIBIT && rig.out.reason == REASON_MOTOR ? rig.t - t_floor : -1.f;
}

TEST(SoaringRestart, DeadMotorEndsSoaringAfterFourSeconds)
{
	// 2 s for the throttle to rise, then 2 s of energy rate under -0.8 m/s
	const float t = restartFails(1000.f);
	EXPECT_GE(t, 3.98f);
	EXPECT_LE(t, 4.5f);
}

TEST(SoaringRestart, LateHealthyRestartKeepsClimbing)
{
	EXPECT_LT(restartFails(0.f), 0.f);
	EXPECT_LT(restartFails(3.5f), 0.f);
}

TEST(SoaringRestart, WeakClimbIsNoMotorFailure)
{
	// a motor that holds the energy rate at -0.3 m/s, in sink, still adds energy: over -0.8 m/s
	Rig rig;
	rig.in.height = 106.f;
	rig.toGlide();
	rig.climb_powered = -0.3f;

	while (rig.out.state == State::GLIDE && rig.t < 100.f) {
		rig.step(kDt);
	}

	ASSERT_EQ(rig.out.state, State::CLIMB);
	rig.step(20.f);
	EXPECT_EQ(rig.out.state, State::CLIMB);
}

// CLIMB holds cut + 5 m with the cut gate closed and flies through 1.5 m/s of sink for 3 s at this throttle.
static uint8_t sinkInClimb(float throttle)
{
	Rig rig;
	rig.p.alt_pat = 215.f;
	rig.in.height = 200.f;
	rig.step(2.1f);
	EXPECT_EQ(rig.out.state, State::CLIMB);
	rig.in.height = 255.f;
	rig.climb_powered = 0.f;
	rig.step(5.f);
	rig.throttle = throttle;
	rig.w = -1.5f;
	rig.step(3.f);
	rig.w = 0.f;
	rig.step(1.f);
	return rig.out.reason;
}

TEST(SoaringRestart, SinkAtPartThrottleIsNoMotorFailure)
{
	// FD_FW_TL_THR 0.7: below it the motor is not expected to add energy
	EXPECT_EQ(sinkInClimb(0.5f), REASON_NONE);
	EXPECT_EQ(sinkInClimb(0.69f), REASON_NONE);
	EXPECT_EQ(sinkInClimb(0.7f), REASON_MOTOR);
}

TEST(SoaringRestart, MotorEndIsLatched)
{
	Rig rig;
	rig.in.height = 106.f;
	rig.toGlide();
	rig.motor = false;
	rig.step(10.f);
	ASSERT_EQ(rig.out.reason, REASON_MOTOR);
	rig.motor = true;
	rig.step(10.f);
	EXPECT_EQ(rig.out.state, State::INHIBIT);
	EXPECT_EQ(rig.out.reason, REASON_MOTOR);
}

// Circles at true airspeed v in a wind toward the east: in the air mass at constant bank, or on a ground-fixed
// orbit as the Hold flies it. The pitot reads v + pitot_add. dir -1 turns left.
struct Circle {
	Soaring soar;
	Params p;
	Input in;
	Output out;
	float t{0.f};
	float v{15.84f};
	float radius{36.5f};
	float wind_e{8.f};
	float pitot_add{0.f};
	bool ground_fixed{false};
	float chi{0.f};
	float psi{0.f};
	float t_latch{-1.f};
	float t_active{-1.f};
	int jitter_every{0};
	int n_sample{0};

	Circle()
	{
		p.enabled = true;
		in.armed = true;
		in.allowed = true;
		in.inhibit_reason = REASON_NONE;
		in.wind_valid = true;
		in.height = 200.f;
		in.wind_e = wind_e;
	}

	void fly(float seconds, float dir = 1.f)
	{
		const int n = (int)lroundf(seconds / kDt);

		for (int i = 0; i < n; i++) {
			t += kDt;
			float vn;
			float ve;
			float rate;

			if (ground_fixed) {
				const float c = -sinf(chi) * 0.f + cosf(chi) * wind_e;
				const float gs = sinf(chi) * wind_e + sqrtf(v * v - c * c);
				vn = gs * cosf(chi);
				ve = gs * sinf(chi);
				const float psi_new = atan2f(ve - wind_e, vn);
				float d = psi_new - psi;
				d = atan2f(sinf(d), cosf(d));
				rate = d / kDt;
				psi = psi_new;
				chi += dir * gs / radius * kDt;

			} else {
				rate = dir * v / radius;
				psi += rate * kDt;
				vn = v * cosf(psi);
				ve = v * sinf(psi) + wind_e;
			}

			in.time_us = (uint64_t)lroundf(t * 1e6f);
			in.vn = vn;
			in.ve = ve;
			in.x += vn * kDt;
			in.y += ve * kDt;
			in.vz = tableSinkTurn(v, fabsf(atanf(v * rate / kG)));
			in.cas = in.tas = v + pitot_add;
			in.roll = atanf(v * rate / kG);
			n_sample++;
			in.yaw = (jitter_every > 0 && n_sample % jitter_every == 0) ? psi - 2.f * rate * kDt : psi;
			in.wind_e = wind_e;
			soar.update(in, p, out);

			if (t_latch < 0.f && out.reason == REASON_AIRSPEED) {
				t_latch = t;
			}

			if (t_active < 0.f && out.state >= State::CLIMB) {
				t_active = t;
			}
		}
	}
};

TEST(SoaringTurnCheck, OverReadEndsSoaringWithinOneTurn)
{
	// 35 deg thermalling circle: 36.5 m, 14.48 s per turn
	Circle c;
	c.pitot_add = 4.f;
	c.fly(20.f);
	EXPECT_GT(c.t_latch, 0.f);
	EXPECT_LE(c.t_latch, 14.6f);
	EXPECT_LT(c.t_active, 0.f);
	EXPECT_NEAR(c.out.aspd_excess, 4.f, 0.2f);
	EXPECT_NEAR(c.out.aspd_turn, c.v, 0.2f);

	Circle left;
	left.pitot_add = 4.f;
	left.fly(20.f, -1.f);
	EXPECT_GT(left.t_latch, 0.f);
	EXPECT_LE(left.t_latch, 14.6f);

	// latched through a clean turn, cleared by SOAR_EN 0 then 1, restarted after the next clean turn
	c.pitot_add = 0.f;
	c.fly(16.f);
	EXPECT_EQ(c.out.state, State::INHIBIT);
	EXPECT_EQ(c.out.reason, REASON_AIRSPEED);
	c.p.enabled = false;
	c.fly(kDt);
	c.p.enabled = true;
	c.fly(16.f);
	EXPECT_GT(c.t_active, 0.f);
}

TEST(SoaringTurnCheck, OverReadDuringSoaringEndsItWithinTwoTurns)
{
	Circle c;
	c.fly(20.f);
	ASSERT_GT(c.t_active, 0.f);
	const float t0 = c.t;
	c.pitot_add = 4.f;
	c.fly(30.f);
	EXPECT_EQ(c.out.reason, REASON_AIRSPEED);
	EXPECT_LE(c.t_latch - t0, 2.f * 14.48f + 0.2f);
}

TEST(SoaringTurnCheck, GroundOrbitInWindWithSmallOverRead)
{
	// a time-weighted mean reads 2.17 m/s high on this orbit, so +2 m/s would pass 2.5
	Circle c;
	c.ground_fixed = true;
	c.radius = 60.f;
	c.pitot_add = 2.f;
	c.fly(100.f);
	EXPECT_LT(c.t_latch, 0.f);
	EXPECT_NEAR(c.out.aspd_excess, 2.f, 0.1f);
	EXPECT_GT(c.t_active, 0.f);
}

TEST(SoaringTurnCheck, NoStartBeforeTheFirstCleanTurn)
{
	// ground-fixed 60 m orbit in 8 m/s wind: one turn takes 29.8 s
	Circle c;
	c.ground_fixed = true;
	c.radius = 60.f;
	c.fly(29.f);
	EXPECT_EQ(c.out.state, State::INHIBIT);
	EXPECT_LT(c.t_active, 0.f);
	c.fly(1.5f);
	EXPECT_GT(c.t_active, 29.f);
}

TEST(SoaringTurnCheck, PitotDropoutClearsTheVerdict)
{
	// the Rig circles 60 m at 14 m/s: one turn takes 26.9 s; the clean turn of its start ended at 27.0 s
	Rig battery;
	battery.step(1.f);
	battery.in.allowed = false;
	battery.in.inhibit_reason = REASON_BATTERY;
	battery.step(1.5f);
	battery.in.allowed = true;
	battery.step(2.1f);
	EXPECT_EQ(battery.out.state, State::GLIDE);

	Rig pitot;
	pitot.step(1.f);
	pitot.in.allowed = false;
	pitot.in.inhibit_reason = REASON_AIRSPEED;
	pitot.step(1.5f);
	pitot.in.allowed = true;
	pitot.step(20.f);
	EXPECT_EQ(pitot.out.state, State::INHIBIT);
	EXPECT_EQ(pitot.out.reason, REASON_NONE);
	// next verdict at 53.9 s, then 2 s good
	pitot.step(10.f);
	EXPECT_EQ(pitot.out.state, State::GLIDE);
}

TEST(SoaringTurnCheck, SingleReversedYawSamplesKeepTheTurn)
{
	// every 25th yaw sample reads 2 steps back, as turbulence does: the turn still ends after 360 deg
	Circle c;
	c.jitter_every = 25;
	c.fly(15.f);
	EXPECT_FALSE(std::isnan(c.out.aspd_excess));
	EXPECT_NEAR(c.out.aspd_excess, 0.f, 0.2f);
	c.fly(2.5f);
	EXPECT_GT(c.t_active, 0.f);
}

TEST(SoaringTurnCheck, ReversalOf30DegRestartsTheTurn)
{
	// 340 deg one way, a reversal, then 80 deg on: a kept turn reaches 360 deg net, a restarted one 80 deg at most
	// [deg/s] the default circle: 15.84 m/s on 36.5 m
	const float deg = 15.84f / 36.5f / kDeg;
	Circle keep;
	keep.wind_e = 0.f;
	keep.fly(340.f / deg);
	keep.fly(25.f / deg, -1.f);
	keep.fly(80.f / deg);
	EXPECT_FALSE(std::isnan(keep.out.aspd_excess));

	Circle restart;
	restart.wind_e = 0.f;
	restart.fly(340.f / deg);
	restart.fly(35.f / deg, -1.f);
	restart.fly(80.f / deg);
	EXPECT_TRUE(std::isnan(restart.out.aspd_excess));
}

TEST(SoaringTurnCheck, STurnsGiveNoVerdict)
{
	// ten 180 deg turns each way, then two full circles: headings are covered unevenly until the circles
	Circle c;
	c.wind_e = 0.f;
	const float half = kPi * c.radius / c.v;

	for (int i = 0; i < 10; i++) {
		c.fly(half, 1.f);
		c.fly(half, -1.f);
	}

	EXPECT_TRUE(std::isnan(c.out.aspd_excess));
	c.fly(2.f * 2.f * half + 0.5f, 1.f);
	EXPECT_LT(c.t_latch, 0.f);
	EXPECT_NEAR(c.out.aspd_excess, 0.f, 0.2f);
}

// Closed loop: the aircraft circles the commanded centre in the air mass through an Allen thermal.
struct Flight {
	float wind_e{0.f};
	float thermal_drift_e{0.f};	// thermal centre speed east beyond the wind
	float seconds{150.f};
	unsigned seed{1};

	std::vector<float> centre_err;
	float climb{0.f};
	float avail{0.f};
	float entry_t{-1.f};
	bool bounded{true};
	State final_state{State::OFF};

	void fly()
	{
		Rig rig;
		Allen allen;
		std::mt19937 gen(seed);
		std::normal_distribution<float> gauss(0.f, 1.f);
		float turbulence = 0.f;
		float tc_x = 0.f;
		float tc_y = 0.f;
		const float wind_n = 0.f;

		rig.p.max_rad = 5000.f;
		rig.p.gf_max_hor = 0.f;
		rig.in.wind_n = wind_n;
		rig.in.wind_e = wind_e;
		rig.in.height = 220.f;
		// straight east in GLIDE, crossing the thermal about 34 s in
		rig.in.x = 0.f;
		rig.in.y = -480.f;
		rig.psi = 90.f * kDeg;
		rig.rate = 0.f;
		rig.step(0.02f);

		float h_start = 0.f;
		float t_start = -1.f;
		float avail_sum = 0.f;
		int avail_n = 0;
		const float t_end = rig.t + seconds;

		while (rig.t < t_end) {
			tc_x += wind_n * kDt;
			tc_y += (wind_e + thermal_drift_e) * kDt;

			// Gauss-Markov turbulence, 0.4 m/s, 1 s
			turbulence += -turbulence * kDt + 0.4f * sqrtf(2.f * kDt) * gauss(gen);
			const float r = hypotf(rig.in.x - tc_x, rig.in.y - tc_y);
			rig.w = allen(r, rig.in.height) + turbulence;

			if (rig.out.state == State::THERMAL) {
				const float rx = (rig.in.x - rig.out.centre_x);
				const float ry = (rig.in.y - rig.out.centre_y);
				const float d = hypotf(rx, ry);
				const float dir = rig.out.loiter_ccw ? -1.f : 1.f;
				const float tx = -dir * ry / d;
				const float ty = dir * rx / d;
				const float e = std::max(-1.5f, std::min(1.5f, (d - rig.out.loiter_radius) / 20.f));
				const float dx = tx - e * rx / d;
				const float dy = ty - e * ry / d;
				float err = atan2f(dy, dx) - rig.psi;
				err = atan2f(sinf(err), cosf(err));
				const float max_rate = kG * tanf(45.f * kDeg) / rig.speed;
				rig.rate = std::max(-max_rate, std::min(max_rate, dir * rig.speed / rig.out.loiter_radius + 0.8f * err));

				if (entry_t < 0.f) {
					entry_t = rig.t;
				}

				const float since = rig.t - entry_t;

				if (since >= 45.f) {
					if (t_start < 0.f) {
						t_start = rig.t;
						h_start = rig.in.height;
					}

					centre_err.push_back(hypotf(rig.out.thermal_x - tc_x, rig.out.thermal_y - tc_y));
					avail_sum += allen(rig.out.loiter_radius, rig.in.height)
						     - tableSinkTurn(rig.out.airspeed_sp, rig.p.thml_bank);
					avail_n++;
					climb = (rig.in.height - h_start) / std::max(rig.t - t_start, kDt);
				}

				bounded &= rig.out.thermal_w >= 0.f && rig.out.thermal_w <= 10.f
					   && rig.out.thermal_r >= 20.f && rig.out.thermal_r <= 200.f
					   && fabsf(rig.out.thermal_x - rig.in.x) <= 300.f && fabsf(rig.out.thermal_y - rig.in.y) <= 300.f;

			} else {
				rig.rate = 0.f;

				if (entry_t >= 0.f) {
					break;
				}
			}

			const float target = rig.out.state == State::THERMAL ? rig.out.airspeed_sp : 14.f;
			rig.accel = (target - rig.speed) / 2.f;
			rig.step(kDt);
		}

		avail = avail_n > 0 ? avail_sum / (float)avail_n : 0.f;
		final_state = rig.out.state;
	}

	float medianError()
	{
		if (centre_err.empty()) {
			return 1e9f;
		}

		std::sort(centre_err.begin(), centre_err.end());
		return centre_err[centre_err.size() / 2];
	}
};

TEST(SoaringClosedLoop, CentresCalmThermal)
{
	for (unsigned seed = 1; seed <= 10; seed++) {
		Flight f;
		f.seed = seed;
		f.fly();
		ASSERT_GE(f.entry_t, 0.f) << seed;
		ASSERT_GE(f.centre_err.size(), 1000u) << seed;
		EXPECT_LE(f.medianError(), 20.f) << seed;
		EXPECT_TRUE(f.bounded) << seed;
		// strong day near 250 m: 3.5 m/s at the 35 m circle less 1.9 m/s sink
		EXPECT_GE(f.avail, 1.2f) << seed;
		EXPECT_GE(f.climb / f.avail, 0.75f) << seed;
	}
}

TEST(SoaringClosedLoop, CentresThermalDriftingAgainstWindEstimate)
{
	float sum = 0.f;

	for (unsigned seed = 1; seed <= 10; seed++) {
		Flight f;
		f.seed = seed;
		f.thermal_drift_e = 1.5f;
		f.fly();
		ASSERT_GE(f.centre_err.size(), 1000u) << seed;
		EXPECT_GE(f.climb / f.avail, 0.6f) << seed;
		sum += f.medianError();
	}

	// the 4-state model has no drift state: single seeds reach 35 m, the bound holds on the mean
	EXPECT_LE(sum / 10.f, 30.f);
}

TEST(SoaringClosedLoop, CentresThermalDriftingWithWind)
{
	Flight f;
	f.wind_e = 5.f;
	f.fly();
	ASSERT_GE(f.entry_t, 0.f);
	ASSERT_GE(f.centre_err.size(), 1000u);
	EXPECT_LE(f.medianError(), 30.f);
	EXPECT_GE(f.climb / f.avail, 0.6f);
}

TEST(SoaringClosedLoop, NoEntryInTurbulenceWithoutThermal)
{
	Rig rig;
	std::mt19937 gen(7);
	std::normal_distribution<float> gauss(0.f, 1.f);
	float turbulence = 0.f;
	rig.in.height = 300.f;
	rig.p.max_rad = 5000.f;
	rig.p.gf_max_hor = 0.f;

	for (int i = 0; i < 600 * 50; i++) {
		turbulence += -turbulence * kDt + 0.4f * sqrtf(2.f * kDt) * gauss(gen);
		rig.w = turbulence;
		rig.in.height = 300.f;
		rig.step(kDt);
		ASSERT_NE(rig.out.state, State::THERMAL) << rig.t;
	}
}

TEST(SoaringClosedLoop, BoundedEstimatorAfterLiftEnds)
{
	Rig rig;
	rig.toThermal();
	rig.w = 0.f;
	float worst_w = 0.f;

	for (int i = 0; i < 40 * 50 && rig.out.state == State::THERMAL; i++) {
		rig.step(kDt);

		if (rig.out.state == State::THERMAL) {
			ASSERT_TRUE(std::isfinite(rig.out.thermal_w) && std::isfinite(rig.out.thermal_x) && std::isfinite(rig.out.thermal_y));
			ASSERT_GE(rig.out.thermal_r, 20.f);
			ASSERT_LE(rig.out.thermal_r, 200.f);
			worst_w = std::max(worst_w, rig.out.thermal_w);
		}
	}

	EXPECT_LE(worst_w, 10.f);
	EXPECT_EQ(rig.out.state, State::GLIDE);
	EXPECT_EQ(rig.out.reason, REASON_WEAK_LIFT);
}

TEST(SoaringCanary, WrongPolarIsCaught)
{
	Params p;
	EXPECT_NONFATAL_FAILURE(EXPECT_NEAR(Soaring::sink(p, 14.f, 1.f, 0.f), 1.41f + 0.1f, 0.03), "");
}
