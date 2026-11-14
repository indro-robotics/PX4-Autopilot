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

	Rig()
	{
		p.enabled = true;
		p.gf_max_ver = 450.f;
		p.gf_max_hor = 500.f;
		in.allowed = true;
		in.inhibit_reason = REASON_NONE;
		in.height = 200.f;
		in.wind_valid = true;
		in.hold_x = 0.f;
		in.hold_y = 0.f;
		in.nav_radius = 60.f;
		in.x = 0.f;
		in.y = -60.f;
	}

	void step(float seconds)
	{
		const int n = (int)lroundf(seconds / kDt);

		for (int i = 0; i < n; i++) {
			t += kDt;
			psi += rate * kDt;
			speed += accel * kDt;
			const float roll = atanf(speed * rate / kG);
			const float climb = w - tableSinkTurn(speed, roll) - speed * accel / kG;
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

		while (out.state == State::GLIDE && t < p.min_crse_s + 3.f) {
			step(kDt);
		}

		ASSERT_EQ(out.state, State::THERMAL);
	}
};

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
		soar.update(in, p, out);

		if (i > 100) {
			ASSERT_NEAR(out.vario_netto, 0.f, 0.05) << v;
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

static State holdHeight(Rig &rig, float height, float seconds)
{
	for (int i = 0; i < (int)lroundf(seconds / kDt); i++) {
		rig.in.height = height;
		rig.step(kDt);
	}

	return rig.out.state;
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

TEST(SoaringStates, NoThermalEntryWithoutWindOrOutsideBand)
{
	Rig rig;
	rig.toGlide();
	rig.in.wind_valid = false;
	rig.w = 3.f;
	EXPECT_EQ(holdHeight(rig, 200.f, 40.f), State::GLIDE);

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

TEST(SoaringStates, GlideAboveCeilingAtMaximumAirspeed)
{
	Rig rig;
	rig.in.height = 410.f;
	rig.toGlide();
	EXPECT_FLOAT_EQ(rig.out.airspeed_sp, rig.p.airspd_max);
	rig.in.height = 390.f;
	rig.step(0.02f);
	EXPECT_FLOAT_EQ(rig.out.airspeed_sp, rig.p.airspd_min);
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

TEST(SoaringStates, NewSetpointExit)
{
	Rig rig;
	rig.toThermal();
	rig.in.hold_x = 150.f;
	rig.step(0.02f);
	EXPECT_EQ(rig.out.state, State::GLIDE);
	EXPECT_EQ(rig.out.reason, REASON_NEW_SETPOINT);
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

		while (rig.t < seconds) {
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
