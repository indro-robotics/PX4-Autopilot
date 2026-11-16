// Tests of the soaring library: estimator, variometer, speed to fly, controller states and thermal drift.
// Licence: GPLv3 or later, see COPYING.

#include <gtest/gtest.h>
#include <gtest/gtest-spi.h>

#include <algorithm>
#include <cmath>
#include <math.h>
#include <random>
#include <vector>

#include "SoaringController.hpp"

using namespace soaring;
using SC = SoaringController;
using State = SC::State;

static_assert(SC::REASON_TERRAIN == 18 && SC::REASON_HEIGHT_LOST == 19 && SC::REASON_DRIFT == 20, "reason numbering");

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

// sink of a polar K, CD0, B at airspeed v and bank phi, exact cosine
static float polarSink(float k, float cd0, float b, float v, float phi)
{
	const float c = cosf(phi);
	return cd0 * v * v * v / k + b * k / (v * c * c);
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

// airframe values of the X8: polar, heights 100, 250, 400, bank 35, radius 350; the rest ArduPilot's defaults
static SC::Params x8Params()
{
	SC::Params p;
	p.enable = true;
	p.polar_cd0 = 0.0297f;
	p.polar_b = 0.0702f;
	p.polar_k = 80.2f;
	p.alt_min = 100.f;
	p.alt_cutoff = 250.f;
	p.alt_max = 400.f;
	p.thml_bank = 35.f;
	p.max_radius = 350.f;
	p.gf_max_ver = 450.f;
	p.gf_max_hor = 500.f;
	return p;
}

// ArduPilot's plane-soaring autotest model: polar, noise, cutoff, airspeeds 8 to 20 with cruise 9
static SC::Params apParams()
{
	SC::Params p;
	p.enable = true;
	p.polar_k = 71.1f;
	p.polar_cd0 = 0.05f;
	p.polar_b = 0.045f;
	p.q2 = 0.1f;
	p.r = 0.2f;
	p.alt_cutoff = 150.f;
	p.vspeed = 0.55f;
	p.min_thml_s = 25.f;
	p.thml_arspd = 9.f;
	p.crse_arspd = 9.f;
	p.airspd_min = 8.f;
	p.airspd_trim = 9.f;
	p.airspd_max = 20.f;
	return p;
}

// Steps the controller at 50 Hz with a point-mass aircraft in the air w (m/s up); it circles 60 m about the origin.
struct Rig {
	SC soar;
	SC::Params p{x8Params()};
	SC::Input in;
	SC::Output out;
	bool ap_model{false};		// sink from ArduPilot's plane-soaring polar, else the X8 table
	float t{0.f};
	float w{0.f};
	float speed{14.f};
	float accel{0.f};
	float psi{0.f};
	float rate{14.f / 60.f};
	bool motor{true};		// CLIMB adds energy for a climb_powered climb
	float climb_powered{3.f};	// [m/s]
	float pitot_add{0.f};		// [m/s] pitot reading above the true airspeed

	explicit Rig(bool clean_turn = true, bool ap = false) : ap_model(ap)
	{
		if (ap) {
			p = apParams();
			speed = 9.f;
			rate = 9.f / 60.f;
		}

		in.armed = true;
		in.allowed = true;
		in.inhibit_reason = SC::REASON_NONE;
		in.height = 200.f;
		in.wind_valid = true;
		in.nav_radius = 60.f;
		in.x = 0.f;
		in.y = -60.f;

		// soaring starts only after one full turn with a pitot that agrees with GPS
		if (clean_turn) {
			p.enable = false;
			step(2.f * kPi / rate + 0.1f);
			p.enable = true;
			in.height = 200.f;
			in.x = 0.f;
			in.y = -60.f;
		}
	}

	float sinkTurn(float v, float roll) const
	{
		return ap_model ? polarSink(p.polar_k, p.polar_cd0, p.polar_b, v, roll) : tableSinkTurn(v, roll);
	}

	void step(float seconds)
	{
		const int n = (int)lroundf(seconds / kDt);

		for (int i = 0; i < n; i++) {
			t += kDt;
			psi += rate * kDt;
			speed += accel * kDt;
			const float roll = atanf(speed * rate / kG);
			const float sink = sinkTurn(speed, fabsf(roll));
			const float powered = (motor && out.state == State::CLIMB) ? sink + climb_powered : 0.f;
			const float climb = w + powered - sink - speed * accel / kG;
			in.time_us = (uint64_t)lroundf(t * 1e6f);
			in.vn = speed * cosf(psi) + in.wind_n;
			in.ve = speed * sinf(psi) + in.wind_e;
			in.x += in.vn * kDt;
			in.y += in.ve * kDt;
			in.vz = -climb;
			in.height += climb * kDt;
			in.cas = speed + pitot_add;
			in.tas = speed + pitot_add;
			in.roll = roll;
			in.yaw = psi;
			in.accel_x = accel;
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

static void toState(Rig &rig, State state)
{
	if (state == State::CLIMB) {
		rig.in.height = 95.f;
		rig.step(2.1f);

	} else if (state == State::GLIDE) {
		rig.toGlide();

	} else {
		rig.toThermal();
	}

	ASSERT_EQ(rig.out.state, state);
}

// A thermal column: the base travels with carry times the wind, the centre leans downwind lean metres per metre above z0.
struct Column {
	bool gaussian{false};		// ArduPilot's simulated updraft W exp(-r^2/R^2), else Allen's bell
	float w_g{2.f};
	float r_g{80.f};
	float carry{0.f};
	float lean{0.f};
	float z0{200.f};
	float x0{0.f};
	float y0{0.f};
	Allen allen;

	void centre(float t, float z, float wind_n, float wind_e, float &cx, float &cy) const
	{
		const float wind = hypotf(wind_n, wind_e);
		const float k = lean * std::max(0.f, z - z0);
		cx = x0 + carry * wind_n * t + (wind > 0.1f ? k * wind_n / wind : 0.f);
		cy = y0 + carry * wind_e * t + (wind > 0.1f ? k * wind_e / wind : 0.f);
	}

	float lift(float r, float z) const
	{
		return gaussian ? w_g * expf(-r * r / (r_g * r_g)) : allen(r, z);
	}
};

// Closed loop: GLIDE east across the column, then the aircraft circles the commanded centre in the air mass.
struct Flight {
	Column column;
	float wind_e{0.f};
	float drift{0.f};
	float turbulence_sd{0.4f};
	float seconds{150.f};
	float measure_from{20.f};	// [s] after entry
	bool ap{false};
	unsigned seed{1};
	float start_h{220.f};
	float roll_lim{50.f};		// [deg] FW_R_LIM of the X8

	std::vector<float> centre_err;
	std::vector<float> lag;		// commanded centre to estimate
	float gain{0.f};
	float entry_t{-1.f};
	float entry_h{0.f};
	bool bounded{true};
	State final_state{State::OFF};
	uint8_t final_reason{0};

	void fly()
	{
		Rig rig(true, ap);
		std::mt19937 gen(seed);
		std::normal_distribution<float> gauss(0.f, 1.f);
		float turbulence = 0.f;
		rig.p.drift = drift;
		const float cruise = rig.p.crse_arspd > 0.f ? rig.p.crse_arspd : rig.p.airspd_min;
		rig.p.max_radius = -1.f;
		rig.p.gf_max_hor = 0.f;
		rig.in.wind_e = wind_e;
		rig.in.height = start_h;
		rig.speed = cruise;
		// straight east in GLIDE, crossing the column's centre about 30 s in
		float cx;
		float cy;
		column.centre(30.f, start_h, 0.f, wind_e, cx, cy);
		rig.in.x = cx;
		rig.in.y = cy - 30.f * (cruise + wind_e);
		const float t0 = rig.t;
		rig.psi = 90.f * kDeg;
		rig.rate = 0.f;
		rig.step(kDt);
		const float t_end = rig.t + seconds;

		while (rig.t < t_end) {
			column.centre(rig.t - t0, rig.in.height, 0.f, wind_e, cx, cy);
			turbulence += -turbulence * kDt + turbulence_sd * sqrtf(2.f * kDt) * gauss(gen);
			rig.w = column.lift(hypotf(rig.in.x - cx, rig.in.y - cy), rig.in.height) + turbulence;

			if (rig.out.state == State::THERMAL) {
				const float rx = rig.in.x - rig.out.centre_x;
				const float ry = rig.in.y - rig.out.centre_y;
				const float d = std::max(hypotf(rx, ry), 0.1f);
				const float dir = rig.out.loiter_ccw ? -1.f : 1.f;
				const float tx = -dir * ry / d;
				const float ty = dir * rx / d;
				const float e = std::max(-1.5f, std::min(1.5f, (d - rig.out.loiter_radius) / 20.f));
				float err = atan2f(ty - e * ry / d, tx - e * rx / d) - rig.psi;
				err = atan2f(sinf(err), cosf(err));
				const float max_rate = kG * tanf(roll_lim * kDeg) / rig.speed;
				rig.rate = std::max(-max_rate, std::min(max_rate, dir * rig.speed / rig.out.loiter_radius + 0.8f * err));

				if (entry_t < 0.f) {
					entry_t = rig.t;
					entry_h = rig.in.height;
				}

				if (rig.t - entry_t >= measure_from) {
					centre_err.push_back(hypotf(rig.out.thermal_x - cx, rig.out.thermal_y - cy));
					lag.push_back(hypotf(rig.out.centre_x - rig.out.thermal_x, rig.out.centre_y - rig.out.thermal_y));
				}

				gain = rig.in.height - entry_h;
				bounded &= rig.out.thermal_w >= 0.f && rig.out.thermal_w <= 10.f && rig.out.thermal_r >= 40.f
					   && rig.out.thermal_r <= 200.f;

			} else {
				rig.rate = 0.f;

				if (entry_t >= 0.f) {
					break;
				}
			}

			// PX4 holds the airspeed setpoint at or above FW_AIRSPD_MIN times the root of the load factor
			const float sp_min = rig.p.airspd_min / sqrtf(cosf(rig.in.roll));
			rig.accel = (std::max(rig.out.airspeed_sp, sp_min) - rig.speed) / 2.f;
			rig.step(kDt);
		}

		final_state = rig.out.state;
		final_reason = rig.out.reason;
	}

	static float median(std::vector<float> v)
	{
		if (v.empty()) {
			return 1e9f;
		}

		std::sort(v.begin(), v.end());
		return v[v.size() / 2];
	}
};

// ---------------------------------------------------------------- estimator

TEST(PortEstimator, JacobianMatchesNumericDerivative)
{
	const float states[3][4] = {{2.5f, 70.f, 30.f, -20.f}, {1.f, 40.f, -50.f, 10.f}, {4.f, 120.f, 5.f, 80.f}};
	const float aircraft[3][2] = {{0.f, 0.f}, {-20.f, 35.f}, {60.f, 40.f}};
	const float h[4] = {1e-3f, 1e-2f, 1e-2f, 1e-2f};

	for (int s = 0; s < 3; s++) {
		ExtendedKalmanFilter ekf;
		matrix::Vector<float, 4> H;
		ekf.X = matrix::Vector<float, 4>(states[s]);
		ekf.measurementpredandjacobian(H, aircraft[s][0], aircraft[s][1]);

		for (int i = 0; i < 4; i++) {
			matrix::Vector<float, 4> dummy;
			ekf.X = matrix::Vector<float, 4>(states[s]);
			ekf.X(i) += h[i];
			const float up = ekf.measurementpredandjacobian(dummy, aircraft[s][0], aircraft[s][1]);
			ekf.X(i) -= 2.f * h[i];
			const float down = ekf.measurementpredandjacobian(dummy, aircraft[s][0], aircraft[s][1]);
			const float numeric = (up - down) / (2.f * h[i]);
			EXPECT_NEAR(H(i), numeric, 1e-3f + 1e-3f * fabsf(numeric)) << s << " " << i;
		}
	}
}

TEST(PortEstimator, CentresArduPilotThermal)
{
	// ArduPilot's own simulated thermal: 2 m/s, 80 m, leaning downwind by wind (z + 100) / W, plane-soaring polar
	for (float wind : {0.f, 3.f}) {
		Flight f;
		f.ap = true;
		f.column.gaussian = true;
		f.column.lean = wind / f.column.w_g;
		f.column.z0 = -100.f;
		f.column.y0 = -wind / f.column.w_g * (220.f + 100.f);
		f.wind_e = wind;
		f.turbulence_sd = 0.f;
		f.measure_from = 45.f;
		f.seconds = 200.f;
		f.fly();
		ASSERT_GE(f.entry_t, 0.f) << wind;
		ASSERT_GE(f.centre_err.size(), 500u) << wind;
		EXPECT_LE(Flight::median(f.centre_err), 20.f) << wind;
		EXPECT_LE(*std::max_element(f.centre_err.begin(), f.centre_err.end()), 20.f) << wind;
		EXPECT_GT(f.gain, 0.f) << wind;
	}
}

TEST(PortEstimator, CentresAllenThermalInCalm)
{
	for (unsigned seed = 1; seed <= 5; seed++) {
		Flight f;
		f.seed = seed;
		f.fly();
		ASSERT_GE(f.entry_t, 0.f) << seed;
		ASSERT_GE(f.centre_err.size(), 1000u) << seed;
		EXPECT_LE(Flight::median(f.centre_err), 20.f) << seed;
		EXPECT_TRUE(f.bounded) << seed;
		EXPECT_GT(f.gain, 0.f) << seed;
	}
}

TEST(PortEstimator, BoundsHoldAfterLiftEnds)
{
	Rig rig;
	rig.toThermal();
	rig.w = 0.f;

	for (int i = 0; i < 40 * 50 && rig.out.state == State::THERMAL; i++) {
		rig.step(kDt);

		if (rig.out.state == State::THERMAL) {
			ASSERT_TRUE(std::isfinite(rig.out.thermal_w) && std::isfinite(rig.out.thermal_x) && std::isfinite(rig.out.thermal_y));
			ASSERT_GE(rig.out.thermal_w, 0.f);
			ASSERT_LE(rig.out.thermal_w, 10.f);
			ASSERT_GE(rig.out.thermal_r, 40.f);
			ASSERT_LE(rig.out.thermal_r, 200.f);
		}
	}

	EXPECT_EQ(rig.out.state, State::GLIDE);
}

TEST(PortEstimator, BoundsOnEveryState)
{
	// zero covariance leaves the update to the bounds alone; the centre bound is per axis, so 424 m on the diagonal
	const float cases[2][4] = {{12.f, 250.f, 400.f, 400.f}, {-1.f, 30.f, -400.f, 100.f}};
	const float bounded[2][4] = {{10.f, 200.f, 300.f, 300.f}, {0.f, 40.f, -300.f, 100.f}};

	for (int c = 0; c < 2; c++) {
		ExtendedKalmanFilter ekf;
		ekf.reset(matrix::Vector<float, 4>(cases[c]), matrix::SquareMatrix<float, 4> {}, matrix::SquareMatrix<float, 4> {}, 1.f);
		ekf.update(0.f, 0.f, 0.f, 0.f, 0.f);

		for (int i = 0; i < 4; i++) {
			EXPECT_FLOAT_EQ(ekf.X(i), bounded[c][i]) << c << " " << i;
		}
	}
}

TEST(PortEstimator, Canary)
{
	ExtendedKalmanFilter ekf;
	matrix::Vector<float, 4> H;
	const float s[4] = {2.f, 80.f, 10.f, 0.f};
	ekf.X = matrix::Vector<float, 4>(s);
	ekf.measurementpredandjacobian(H, 0.f, 0.f);
	// dW/dx at 10 m on an 80 m, 2 m/s thermal is -2 x 2 x 10 / 6400 x 0.9845 = -0.00615: the opposite sign must fail
	EXPECT_NONFATAL_FAILURE(EXPECT_NEAR(H(2), 0.00615f, 1e-4f), "");
}

// ---------------------------------------------------------------- variometer

// Feeds the controller a still-air flight at airspeed v, bank and acceleration; returns the netto after n samples.
static SC::Output stillAir(float v, float bank, int n)
{
	SC soar;
	SC::Params p = x8Params();
	SC::Input in;
	SC::Output out;
	in.cas = in.tas = v;
	in.roll = bank;
	in.vz = tableSinkTurn(v, bank);

	for (int i = 1; i <= n; i++) {
		in.time_us = (uint64_t)i * 20000;
		soar.update(in, p, out);
	}

	return out;
}

TEST(PortVario, PolarMatchesStudyTable)
{
	for (int i = 2; i <= 6; i++) {
		EXPECT_NEAR(stillAir(kTableV[i], 0.f, 10).sink_polar, kTableSink[i], 0.03) << kTableV[i];
	}
}

TEST(PortVario, NettoZeroInStillAir)
{
	const float cases[3][2] = {{14.f, 0.f}, {18.f, 0.f}, {14.f / sqrtf(cosf(35.f * kDeg)), 35.f * kDeg}};

	for (const auto &c : cases) {
		const SC::Output out = stillAir(c[0], c[1], 500);
		EXPECT_NEAR(out.vario_netto, 0.f, 0.05) << c[0];
		EXPECT_NEAR(out.vario_netto_slow, 0.f, 0.05) << c[0];
	}
}

TEST(PortVario, NettoWhileTradingSpeedForHeight)
{
	// energy holds: climb = -sink - V dV/dt / g; the bias filter (time constant 20 tau / 2 pi) takes a share of a steady dV/dt
	SC soar;
	SC::Params p = x8Params();
	SC::Input in;
	SC::Output out;
	const float dvdt = -0.25f;
	float v = 18.f;
	float t_dec = 0.f;

	for (int i = 1; i <= 500 + 800; i++) {
		const float a = i > 500 ? dvdt : 0.f;
		in.time_us = (uint64_t)i * 20000;
		in.cas = in.tas = v;
		in.vz = tableSink(v) + v * a / kG;
		in.accel_x = a;
		soar.update(in, p, out);

		if (i > 500) {
			t_dec += kDt;
		}

		// the 5-sample mean takes 4 samples to carry a step in acceleration
		if (i > 504) {
			// tau 2 pi V / (g tan 35): at 14 m/s 12.8 s, so 40.8 s for the bias filter
			const float tc = 20.f * (2.f * kPi * 14.f / (kG * tanf(35.f * kDeg))) / (2.f * kPi);
			const float bound = fabsf(dvdt) * (1.f - expf(-t_dec / tc)) * v / kG + 0.05f;
			ASSERT_LE(fabsf(out.vario_netto), bound) << v;
		}

		v += a * kDt;
	}

	EXPECT_NEAR(v, 14.f, 0.05f);
}

TEST(PortVario, AccelerometerBiasRemoved)
{
	// a 0.2 m/s2 bias at a steady 14 m/s reads 0.29 m/s of netto until the 40.8 s bias filter takes it: e^-7.4 of it after 300 s
	SC soar;
	SC::Params p = x8Params();
	SC::Input in;
	SC::Output out;
	in.cas = in.tas = 14.f;
	in.vz = tableSink(14.f);
	in.accel_x = 0.2f;

	for (int i = 1; i <= 300 * 50; i++) {
		in.time_us = (uint64_t)i * 20000;
		soar.update(in, p, out);
	}

	EXPECT_NEAR(out.vario_netto, 0.f, 0.03f);
}

TEST(PortVario, ThermallingSinkAtThermallingSpeed)
{
	// glide at 14.0 m/s; the 35 deg turn is flown at 14 / sqrt(cos 35) = 15.47 m/s: table 1.41 x n^1.5 = 1.90 m/s
	Rig rig;
	rig.toGlide();
	const float vth = 14.f / sqrtf(cosf(35.f * kDeg));
	EXPECT_NEAR(rig.out.climb_expected, rig.out.vario_netto_slow - tableSinkTurn(vth, 35.f * kDeg), 0.03f);
}

TEST(PortVario, ExactCosineWithRollHeldTo60Deg)
{
	// 15 m/s: exact cosine to 60 deg, then held; the two-term series reads 7.07 against 4.46 m/s at 70 deg
	const float at60 = polarSink(80.2f, 0.0297f, 0.0702f, 15.f, 60.f * kDeg);
	EXPECT_NEAR(stillAir(15.f, 45.f * kDeg, 10).sink_polar, polarSink(80.2f, 0.0297f, 0.0702f, 15.f, 45.f * kDeg), 0.01f);

	for (float deg : {60.f, 70.f, 81.03f, 89.f}) {
		const float s = stillAir(15.f, deg * kDeg, 10).sink_polar;
		EXPECT_TRUE(std::isfinite(s)) << deg;
		EXPECT_NEAR(s, at60, 0.01f) << deg;
	}
}

TEST(PortVario, NonFiniteSampleLeavesNoTrace)
{
	// one NaN nose acceleration, one NaN climb flagged valid and one NaN roll, then 60 s of still air at 14 m/s
	SC soar;
	SC::Params p = x8Params();
	SC::Input in;
	SC::Output out;
	in.cas = in.tas = 14.f;

	for (int i = 1; i <= 3050; i++) {
		in.time_us = (uint64_t)i * 20000;
		in.vz = i == 20 ? NAN : tableSink(14.f);
		in.accel_x = i == 30 ? NAN : 0.f;
		in.roll = i == 40 ? NAN : 0.f;
		soar.update(in, p, out);
	}

	EXPECT_NEAR(out.vario_netto, 0.f, 0.05f);
	EXPECT_NEAR(out.vario_netto_slow, 0.f, 0.05f);
	EXPECT_TRUE(std::isfinite(out.vario_te));
}

TEST(PortVario, ThermalBankHeldToArduPilotRange)
{
	// SOAR_THML_BANK 0 flies the 20 deg floor: 14 / sqrt(cos 20) = 14.44 m/s on a 58.4 m circle, and the orbit filter moves
	Rig rig;
	rig.p.thml_bank = 0.f;
	rig.p.max_radius = -1.f;
	rig.p.gf_max_hor = 0.f;
	rig.toThermal();
	const float v = 14.f / sqrtf(cosf(20.f * kDeg));
	EXPECT_NEAR(rig.out.loiter_radius, v * v / (kG * tanf(20.f * kDeg)), 0.1f);
	rig.step(10.f);
	EXPECT_GT(rig.out.climb_avg, 0.1f);
}

TEST(PortVario, Canary)
{
	EXPECT_NONFATAL_FAILURE(EXPECT_NEAR(stillAir(14.f, 0.f, 10).sink_polar, 1.41f + 0.1f, 0.03), "");
}

// ---------------------------------------------------------------- speed to fly

// root of 2 CD0 V^3 / K - 2 B K / V = m, held to the airspeed limits
static float maccready(float m)
{
	float lo = 14.f;
	float hi = 23.f;

	for (int i = 0; i < 60; i++) {
		const float v = 0.5f * (lo + hi);
		(2.f * 0.0297f * v * v * v / 80.2f - 2.f * 0.0702f * 80.2f / v > m ? hi : lo) = v;
	}

	return 0.5f * (lo + hi);
}

static float speedToFly(float wz, float wexp)
{
	Variometer::PolarParams polar{80.2f, 0.0297f, 0.0702f};
	SpeedToFly stf(polar);

	for (int i = 0; i < 50; i++) {
		stf.update(0.f, wz, wexp, polar.K / (23.f * 23.f), polar.K / (14.f * 14.f));
	}

	return stf.speed_to_fly();
}

TEST(PortSpeedToFly, FourSpeeds)
{
	// SOAR_VSPEED 2 calm; 1 m/s up; 1 m/s down; SOAR_VSPEED 3 calm
	const float cases[4][3] = {{0.f, 2.f, 15.5f}, {1.f, 2.f, 14.0f}, {-1.f, 2.f, 17.0f}, {0.f, 3.f, 17.0f}};

	for (const auto &c : cases) {
		const float v = speedToFly(c[0], c[1]);
		EXPECT_NEAR(v, c[2], 0.2f) << c[0] << " " << c[1];
		EXPECT_NEAR(v, maccready(c[1] - c[0]), 0.05f) << c[0] << " " << c[1];
	}
}

TEST(PortSpeedToFly, UnsolvedFliesCruiseSpeed)
{
	Rig rig;
	rig.p.crse_arspd = -1.f;
	rig.p.vspeed = 2.f;
	rig.toGlide();
	rig.step(2.f);
	EXPECT_NEAR(rig.out.airspeed_sp, maccready(2.f), 0.2f);
	rig.in.wind_n = NAN;
	rig.step(kDt);
	EXPECT_FLOAT_EQ(rig.out.airspeed_sp, rig.p.airspd_min);
	rig.in.wind_n = 0.f;
	rig.step(1.f);
	EXPECT_NEAR(rig.out.airspeed_sp, maccready(2.f), 0.2f);
}

TEST(PortSpeedToFly, Canary)
{
	EXPECT_NONFATAL_FAILURE(EXPECT_NEAR(speedToFly(0.f, 2.f), 15.5f + 0.5f, 0.2f), "");
}

// ---------------------------------------------------------------- states

// floor of a still aircraft at 14 m/s, north metres from home with the wind toward the north
static float floorAt(float north, float wind_n)
{
	SC soar;
	SC::Params p = x8Params();
	SC::Input in;
	SC::Output out;
	in.armed = true;
	in.x = north;
	in.cas = in.tas = 14.f;
	in.wind_n = wind_n;
	in.wind_valid = true;
	soar.update(in, p, out);
	return out.alt_floor;
}

TEST(PortStates, FloorFromGlideReachHome)
{
	// SOAR_ALT_PAT 80 + 1.3 x 350 m x 1.418 m/s / 14 m/s = 126.1 m; an 8 m/s headwind home leaves 6 m/s: 187.5 m
	EXPECT_NEAR(floorAt(150.f, 0.f), 100.f, 0.05f);
	EXPECT_NEAR(floorAt(350.f, 0.f), 126.1f, 0.2f);
	EXPECT_NEAR(floorAt(350.f, 8.f), 187.5f, 0.3f);
	EXPECT_NEAR(floorAt(350.f, -8.f), 80.f + 1.3f * 350.f * 1.418f / 22.f, 0.2f);
}

TEST(PortStates, OffWhenDisabledFromEveryActiveState)
{
	for (State state : {State::CLIMB, State::GLIDE, State::THERMAL}) {
		Rig rig;
		toState(rig, state);
		rig.p.enable = false;
		rig.step(kDt);
		EXPECT_EQ(rig.out.state, State::OFF) << (int)state;
	}
}

TEST(PortStates, InhibitWhenNotAllowed)
{
	Rig rig;
	rig.toGlide();
	rig.in.allowed = false;
	rig.in.inhibit_reason = SC::REASON_BATTERY;
	rig.step(kDt);
	EXPECT_EQ(rig.out.state, State::INHIBIT);
	EXPECT_EQ(rig.out.reason, SC::REASON_BATTERY);
	rig.in.allowed = true;
	rig.step(2.1f);
	EXPECT_EQ(rig.out.state, State::GLIDE);
}

TEST(PortStates, InhibitBandAndRadius)
{
	Rig band;
	band.p.alt_cutoff = band.p.alt_min + 29.f;
	band.step(5.f);
	EXPECT_EQ(band.out.reason, SC::REASON_BAND);

	Rig ceiling;
	ceiling.p.gf_max_ver = 260.f;
	ceiling.step(5.f);
	EXPECT_EQ(ceiling.out.reason, SC::REASON_BAND);

	// SOAR_MAX_RADIUS 350 under 500 - 100: Hold 300 m out plus its 60 m radius
	Rig radius;
	radius.in.hold_x = 300.f;
	radius.step(5.f);
	EXPECT_EQ(radius.out.reason, SC::REASON_RADIUS);
	radius.in.hold_x = 280.f;
	radius.step(2.1f);
	EXPECT_EQ(radius.out.state, State::GLIDE);
}

TEST(PortStates, WindInvalidInhibitsEveryActiveState)
{
	for (State state : {State::CLIMB, State::GLIDE, State::THERMAL}) {
		Rig rig;
		toState(rig, state);
		rig.in.wind_valid = false;
		rig.step(kDt);
		EXPECT_EQ(rig.out.state, State::INHIBIT) << (int)state;
		EXPECT_EQ(rig.out.reason, SC::REASON_WIND) << (int)state;
	}
}

TEST(PortStates, GoodForTwoSecondsBeforeGlide)
{
	Rig rig;
	rig.step(1.9f);
	EXPECT_EQ(rig.out.state, State::INHIBIT);
	rig.step(0.2f);
	EXPECT_EQ(rig.out.state, State::GLIDE);
}

TEST(PortStates, ClimbUnderFloorThenGlideAboveCutoff)
{
	Rig rig;
	rig.in.height = 95.f;
	rig.step(2.1f);
	EXPECT_EQ(rig.out.state, State::CLIMB);
	EXPECT_FLOAT_EQ(rig.out.alt_sp, 255.f);
	// the 3 m/s climb adds 0.06 m per sample to the height held
	EXPECT_EQ(holdHeight(rig, 249.9f, kDt), State::CLIMB);
	EXPECT_EQ(holdHeight(rig, 250.f, kDt), State::GLIDE);
	// the entry filter restarts from 0 at the cut
	EXPECT_FLOAT_EQ(rig.out.vario_netto_slow, 0.f);
}

TEST(PortStates, GlideToClimbUnderFloor)
{
	Rig rig;
	rig.toGlide();
	// the glide takes 0.03 m per sample off the height held
	EXPECT_EQ(holdHeight(rig, 100.05f, kDt), State::GLIDE);
	EXPECT_EQ(holdHeight(rig, 99.95f, kDt), State::CLIMB);
	EXPECT_EQ(rig.out.reason, SC::REASON_FLOOR);
}

TEST(PortStates, GlideToClimbOnLowAirspeedThenGlideBar)
{
	Rig rig;
	rig.in.height = 300.f;
	rig.toGlide();
	// threshold 14 sqrt(n) - 2 = 12.1 m/s on the 60 m circle
	rig.speed = 11.5f;
	rig.step(0.9f);
	EXPECT_EQ(rig.out.state, State::GLIDE);
	rig.step(0.2f);
	EXPECT_EQ(rig.out.state, State::CLIMB);
	EXPECT_EQ(rig.out.reason, SC::REASON_AIRSPEED_LOW);
	rig.speed = 14.f;
	rig.step(19.5f);
	EXPECT_EQ(rig.out.state, State::CLIMB);
	rig.step(1.f);
	EXPECT_EQ(rig.out.state, State::GLIDE);
}

TEST(PortStates, ThermalEntryAfterMinimumCruise)
{
	Rig rig;
	rig.toGlide();
	rig.w = 3.f;
	rig.step(rig.p.min_crse_s - 0.3f);
	EXPECT_EQ(rig.out.state, State::GLIDE);
	// 3 m/s less the 1.90 m/s thermalling sink
	EXPECT_NEAR(rig.out.climb_expected, 1.1f, 0.05f);
	rig.step(0.4f);
	EXPECT_EQ(rig.out.state, State::THERMAL);
	EXPECT_EQ(rig.out.thermal_count, 1);
	EXPECT_FALSE(rig.out.loiter_ccw);
	// 15.47 m/s at 35 deg: 34.8 m
	EXPECT_NEAR(rig.out.loiter_radius, 15.47f * 15.47f / (kG * tanf(35.f * kDeg)), 0.1f);

	// the turn follows the sign of NAV_LOITER_RAD
	Rig left;
	left.p.nav_loiter_rad = -60.f;
	left.toThermal();
	EXPECT_TRUE(left.out.loiter_ccw);
}

TEST(PortStates, PositionResetMovesEstimateAndCircle)
{
	Rig rig;
	rig.toThermal();
	rig.step(5.f);
	const float ex = rig.out.thermal_x - rig.in.x;
	const float cx = rig.out.centre_x - rig.in.x;
	const float sx = rig.in.x;
	rig.in.x += 100.f;
	rig.in.delta_x = 100.f;
	rig.in.xy_reset_counter++;
	rig.step(kDt);
	// one sample moves the aircraft 0.28 m and the estimate by its update
	EXPECT_NEAR(rig.out.thermal_x - rig.in.x, ex, 1.f);
	EXPECT_NEAR(rig.out.centre_x - rig.in.x, cx, 1.f);
	EXPECT_GT(rig.in.x, sx + 99.f);
	EXPECT_EQ(rig.out.state, State::THERMAL);
}

TEST(PortStates, NoThermalEntryOutsideBandOrOnWeakLift)
{
	Rig low;
	low.toGlide();
	low.w = 3.f;
	EXPECT_EQ(holdHeight(low, 99.95f, 40.f), State::CLIMB);
	EXPECT_EQ(low.out.thermal_count, 0);

	Rig high;
	high.toGlide();
	high.w = 3.f;
	EXPECT_EQ(holdHeight(high, 400.f, 40.f), State::GLIDE);

	// the 60 m circle about 345 m north keeps 2 x 34.8 m beyond SOAR_MAX_RADIUS 350
	Rig far;
	far.in.x = 345.f;
	far.toGlide();
	far.w = 3.f;
	EXPECT_EQ(holdHeight(far, 200.f, 40.f), State::GLIDE);
	EXPECT_EQ(far.out.thermal_count, 0);

	// expected climb 2.55 - 1.90 = 0.65 under SOAR_VSPEED 0.7
	Rig weak;
	weak.toGlide();
	weak.w = 2.55f;
	EXPECT_EQ(holdHeight(weak, 200.f, 40.f), State::GLIDE);

	// SOAR_VSPEED 10 glides only
	Rig glide_only;
	glide_only.p.vspeed = 10.f;
	glide_only.toGlide();
	glide_only.w = 6.f;
	EXPECT_EQ(holdHeight(glide_only, 200.f, 40.f), State::GLIDE);
}

TEST(PortStates, ThermalRefusedWithSoaringOff)
{
	Rig rig;
	rig.p.enable = false;
	rig.w = 4.f;
	EXPECT_EQ(holdHeight(rig, 200.f, 60.f), State::OFF);
	EXPECT_EQ(rig.out.thermal_count, 0);
}

TEST(PortStates, ThermalToClimbUnderFloorAndOnLowAirspeed)
{
	Rig floor;
	floor.in.height = 125.f;
	floor.toThermal();
	EXPECT_EQ(holdHeight(floor, 99.f, kDt), State::CLIMB);
	EXPECT_EQ(floor.out.reason, SC::REASON_FLOOR);

	Rig slow;
	slow.toThermal();
	slow.speed = 11.f;
	slow.step(1.1f);
	EXPECT_EQ(slow.out.state, State::CLIMB);
	EXPECT_EQ(slow.out.reason, SC::REASON_AIRSPEED_LOW);
}

TEST(PortStates, ThermalEstimatorFaultExit)
{
	Rig rig;
	rig.toThermal();
	rig.in.wind_n = NAN;
	rig.step(kDt);
	EXPECT_EQ(rig.out.state, State::GLIDE);
	EXPECT_EQ(rig.out.reason, SC::REASON_ESTIMATOR);
}

TEST(PortStates, ThermalRadiusExitWhenCircleLeavesTheLimit)
{
	Rig rig;
	rig.in.x = 180.f;
	rig.p.drift = 1.f;
	rig.toThermal();
	// the base travels north at 8 m/s toward SOAR_MAX_RADIUS 350
	rig.in.wind_n = 8.f;
	float radius = 0.f;

	for (int i = 0; i < 30 * 50 && rig.out.state == State::THERMAL; i++) {
		radius = rig.out.loiter_radius;
		rig.step(kDt);
	}

	EXPECT_EQ(rig.out.state, State::GLIDE);
	EXPECT_EQ(rig.out.reason, SC::REASON_RADIUS);
	EXPECT_NEAR(hypotf(rig.out.centre_x, rig.out.centre_y) + radius, 350.f, 1.f);
}

TEST(PortStates, ThermalCeilingExitFiveSecondsAhead)
{
	Rig rig;
	rig.toThermal();
	rig.step(30.f);
	ASSERT_EQ(rig.out.state, State::THERMAL);
	const float climb = rig.out.climb_avg;
	ASSERT_GT(climb, 0.8f);
	EXPECT_EQ(holdHeight(rig, 399.f - 5.f * climb, kDt), State::THERMAL);
	EXPECT_EQ(holdHeight(rig, 401.f - 5.f * climb, kDt), State::GLIDE);
	EXPECT_EQ(rig.out.reason, SC::REASON_CEILING);
}

TEST(PortStates, NoNewThermalForMinimumCruiseAfterAnExit)
{
	// the ceiling exit leaves the lift and the entry filter as they were: a new thermal must wait SOAR_MIN_CRSE_S
	Rig rig;
	rig.toThermal();
	rig.step(30.f);
	ASSERT_EQ(rig.out.state, State::THERMAL);
	const float h = 401.f - 5.f * rig.out.climb_avg;
	ASSERT_EQ(holdHeight(rig, h, kDt), State::GLIDE);
	const uint16_t count = rig.out.thermal_count;
	EXPECT_EQ(holdHeight(rig, h, rig.p.min_crse_s - 0.5f), State::GLIDE);
	EXPECT_EQ(rig.out.thermal_count, count);
}

TEST(PortStates, NoThermalEntryWithinThirtyMetresOfTheCeiling)
{
	// with SOAR_MIN_CRSE_S 0 only the entry gate keeps a ceiling exit from re-entering on the next sample
	Rig rig;
	rig.p.min_crse_s = 0.f;
	rig.toThermal();
	rig.step(30.f);
	ASSERT_EQ(rig.out.state, State::THERMAL);
	const float h = 401.f - 5.f * rig.out.climb_avg;
	ASSERT_EQ(holdHeight(rig, h, kDt), State::GLIDE);
	const uint16_t count = rig.out.thermal_count;
	EXPECT_EQ(holdHeight(rig, h, 10.f), State::GLIDE);
	EXPECT_EQ(rig.out.thermal_count, count);
	// 400 m less 30 m less 1 m
	EXPECT_EQ(holdHeight(rig, 369.f, 1.f), State::THERMAL);
	EXPECT_EQ(rig.out.thermal_count, count + 1);
}

TEST(PortStates, NoThermalEntryOnThePoweredClimbBeforeGlide)
{
	// the variometer reads the motor climb of an inhibited Hold as lift: w stands for it until the GLIDE stops the motor
	Rig rig(true);
	rig.p.min_crse_s = 0.f;
	rig.in.allowed = false;
	rig.w = 3.f;
	rig.step(20.f);
	ASSERT_EQ(rig.out.state, State::INHIBIT);
	rig.in.allowed = true;

	while (rig.out.state != State::GLIDE && rig.t < 60.f) {
		rig.step(kDt);
	}

	ASSERT_EQ(rig.out.state, State::GLIDE);
	rig.w = 0.f;
	rig.step(10.f);
	EXPECT_EQ(rig.out.state, State::GLIDE);
	EXPECT_EQ(rig.out.thermal_count, 0);
}

TEST(PortStates, ThermalWeakExitOnlyAfterMinimumTime)
{
	Rig rig;
	rig.toThermal();
	rig.w = 0.f;
	rig.step(rig.p.min_thml_s - 0.5f);
	EXPECT_EQ(rig.out.state, State::THERMAL);
	rig.step(1.f);
	EXPECT_EQ(rig.out.state, State::GLIDE);
	EXPECT_EQ(rig.out.reason, SC::REASON_WEAK_LIFT);
}

TEST(PortStates, ThermalHeightLostUnderEntryHeight)
{
	Rig rig;
	rig.toThermal();
	const float entry = rig.in.height;
	rig.step(rig.p.min_thml_s + 1.f);
	ASSERT_EQ(rig.out.state, State::THERMAL);
	EXPECT_EQ(holdHeight(rig, entry + 0.5f, kDt), State::THERMAL);
	EXPECT_EQ(holdHeight(rig, entry - 0.5f, kDt), State::GLIDE);
	EXPECT_EQ(rig.out.reason, SC::REASON_HEIGHT_LOST);
}

TEST(PortStates, ThermalDriftExitAfterMinimumTime)
{
	Rig rig;
	rig.p.max_drift = 50.f;
	rig.p.max_radius = -1.f;
	rig.p.gf_max_hor = 0.f;
	rig.p.drift = 1.f;
	rig.toThermal();
	rig.in.wind_e = 6.f;
	rig.step(rig.p.min_thml_s - 0.5f);
	EXPECT_EQ(rig.out.state, State::THERMAL);
	rig.step(1.f);
	EXPECT_EQ(rig.out.state, State::GLIDE);
	EXPECT_EQ(rig.out.reason, SC::REASON_DRIFT);
}

TEST(PortStates, OperatorCommandEndsEveryActiveStateLatched)
{
	for (State state : {State::CLIMB, State::GLIDE, State::THERMAL}) {
		Rig rig;
		toState(rig, state);
		rig.in.operator_cmd = SC::OPERATOR_NAV;
		rig.step(kDt);
		rig.in.operator_cmd = 0;
		rig.step(5.f);
		EXPECT_EQ(rig.out.state, State::INHIBIT) << (int)state;
		EXPECT_EQ(rig.out.reason, SC::REASON_OPERATOR) << (int)state;
		rig.p.enable = false;
		rig.step(kDt);
		rig.p.enable = true;
		rig.step(2.1f);
		EXPECT_GE(rig.out.state, State::CLIMB) << (int)state;
	}
}

TEST(PortStates, GapIsAModeChangeWhateverTheReason)
{
	for (uint8_t reason : {SC::REASON_NONE, SC::REASON_HOME, SC::REASON_AIRSPEED}) {
		Rig rig;
		toState(rig, State::THERMAL);
		rig.t += 120.f;
		rig.in.allowed = reason == SC::REASON_NONE;
		rig.in.inhibit_reason = reason;
		rig.step(kDt);
		rig.in.allowed = true;
		rig.in.inhibit_reason = SC::REASON_NONE;
		rig.step(5.f);
		EXPECT_EQ(rig.out.state, State::INHIBIT) << (int)reason;
		EXPECT_EQ(rig.out.reason, SC::REASON_MODE) << (int)reason;
	}
}

TEST(PortStates, MotorAndTerrainEndsAreLatched)
{
	for (uint8_t reason : {SC::REASON_MOTOR, SC::REASON_TERRAIN, SC::REASON_FAILSAFE}) {
		Rig rig;
		rig.toGlide();
		rig.in.allowed = false;
		rig.in.inhibit_reason = reason;
		rig.step(kDt);
		rig.in.allowed = true;
		rig.in.inhibit_reason = SC::REASON_NONE;
		rig.step(5.f);
		EXPECT_EQ(rig.out.state, State::INHIBIT) << (int)reason;
		EXPECT_EQ(rig.out.reason, reason);
	}
}

TEST(PortStates, PitotOverReadEndsSoaringWithinOneTurn)
{
	// the 60 m circle at 14 m/s takes 26.9 s; a pitot 4 m/s high passes the 2.5 m/s bound
	Rig rig;
	rig.toGlide();
	rig.pitot_add = 4.f;
	rig.step(2.f * kPi / rig.rate + 0.5f);
	EXPECT_EQ(rig.out.state, State::INHIBIT);
	EXPECT_EQ(rig.out.reason, SC::REASON_AIRSPEED);
	EXPECT_GT(rig.out.aspd_excess, 2.5f);
}

TEST(PortStates, CutStepsPitchByTheMotorPathAngleOnly)
{
	// 3 m/s up at 18 m/s plus the 1.41 m/s glide sink at 14 m/s: 15.3 deg nose down
	Rig rig;
	rig.in.height = 95.f;
	rig.step(2.1f);
	ASSERT_EQ(rig.out.state, State::CLIMB);
	rig.speed = 18.f;

	while (rig.out.state == State::CLIMB && rig.t < 120.f) {
		rig.step(kDt);
	}

	ASSERT_EQ(rig.out.state, State::GLIDE);
	EXPECT_EQ(rig.out.cut_count, 1);
	EXPECT_NEAR(rig.out.cut_pitch_step, -(3.f / 18.f + 1.41f / 14.f), 0.004f);

	// 8 m/s of lift under CLIMB: the step holds the climb to FW_T_CLMB_MAX 3 m/s
	Rig lift;
	lift.in.height = 95.f;
	lift.step(2.1f);
	ASSERT_EQ(lift.out.state, State::CLIMB);
	lift.w = 8.f;

	while (lift.out.state == State::CLIMB && lift.t < 120.f) {
		lift.step(kDt);
	}

	ASSERT_EQ(lift.out.state, State::GLIDE);
	EXPECT_NEAR(lift.out.cut_pitch_step, -(3.f / 14.f + 1.41f / 14.f), 0.004f);

	// INHIBIT to GLIDE: the glide angle alone
	Rig hold;
	hold.w = 3.f;
	hold.step(2.1f);
	ASSERT_EQ(hold.out.state, State::GLIDE);
	EXPECT_NEAR(hold.out.cut_pitch_step, -1.41f / 14.f, 0.004f);
}

TEST(PortStates, Canary)
{
	Rig rig;
	rig.step(2.1f);
	EXPECT_NONFATAL_FAILURE(EXPECT_EQ(rig.out.state, State::CLIMB), "");
}

// ---------------------------------------------------------------- drift

TEST(PortDrift, StepTerms)
{
	const matrix::Vector2f wind{0.f, 6.f};
	// lean alone: 6 x 0.02 x 1.5 / 3.7 = 0.0486 m
	EXPECT_NEAR(SC::drift_step(wind, 0.02f, 1.5f, 3.7f, 0.f)(1), 0.0486f, 1e-4f);
	// base alone: 6 x 0.02 x 1.0
	EXPECT_NEAR(SC::drift_step(wind, 0.02f, 0.f, 3.7f, 1.f)(1), 0.12f, 1e-5f);
	// both: the base carries half the wind and the other half leans the column
	EXPECT_NEAR(SC::drift_step(wind, 0.02f, 1.5f, 3.7f, 0.5f)(1), 0.5f * 0.0486f + 0.06f, 1e-4f);
	EXPECT_FLOAT_EQ(SC::drift_step(wind, 0.02f, 1.5f, 3.7f, 0.f)(0), 0.f);
	// an upright column carried with the wind moves with the air whatever the climb
	EXPECT_NEAR(SC::drift_step(wind, 0.02f, 1.5f, 3.7f, 1.f)(1), 0.12f, 1e-5f);
}

TEST(PortDrift, DriftHeldToZeroToOne)
{
	// param set takes any value: -1 would move the estimate 4.18 m/s upwind at 6 m/s, 1.2 faster than the air
	const matrix::Vector2f wind{0.f, 6.f};
	EXPECT_FLOAT_EQ(SC::drift_step(wind, 1.f, 1.f, 3.3f, -1.f)(1), SC::drift_step(wind, 1.f, 1.f, 3.3f, 0.f)(1));
	EXPECT_FLOAT_EQ(SC::drift_step(wind, 1.f, 1.f, 3.3f, 1.2f)(1), 6.f);
}

TEST(PortDrift, LeanRatioHeldToZeroToOne)
{
	const matrix::Vector2f wind{0.f, 6.f};
	// sinking 1 m/s on a 0.05 m/s strength: 6 x 0.02 x (-1) / 0.05 = -2.4 m per step unheld
	EXPECT_FLOAT_EQ(SC::drift_step(wind, 0.02f, -1.f, 0.05f, 0.f)(1), 0.f);
	EXPECT_FLOAT_EQ(SC::drift_step(wind, 0.02f, 2.f, 1.f, 0.f)(1), 0.12f);
	EXPECT_FLOAT_EQ(SC::drift_step(wind, 0.02f, 1.f, 0.f, 0.f)(1), 0.12f);
	EXPECT_FLOAT_EQ(SC::drift_step(wind, 0.02f, 0.f, 0.f, 0.f)(1), 0.f);
}

// X8 in an Allen column at 6 m/s wind from 260 m, where the bell peaks near 3.7 m/s; the lean is wind over that peak
static Flight x8Column(float carry, float lean, float drift, unsigned seed)
{
	Flight f;
	f.start_h = 260.f;
	f.wind_e = 6.f;
	f.column.carry = carry;
	f.column.lean = lean;
	f.drift = drift;
	f.seed = seed;
	f.fly();
	return f;
}

static void expectHeld(const Flight &f, const char *name)
{
	ASSERT_GE(f.entry_t, 0.f) << name;
	ASSERT_GE(f.centre_err.size(), 500u) << name << " left " << (int)f.final_reason;
	EXPECT_LE(Flight::median(f.centre_err), 30.f) << name;
	EXPECT_GT(f.gain, 0.f) << name;
}

TEST(PortDrift, LeaningColumnHeldWithDrift0)
{
	for (unsigned seed = 1; seed <= 3; seed++) {
		expectHeld(x8Column(0.f, 6.f / 3.7f, 0.f, seed), "leaning");
	}
}

TEST(PortDrift, CarriedColumnHeldWithDrift1)
{
	for (unsigned seed = 1; seed <= 3; seed++) {
		expectHeld(x8Column(1.f, 0.f, 1.f, seed), "carried");
	}
}

TEST(PortDrift, HalfCarriedColumnHeldWithDrift05)
{
	// a base carried at half the wind leans by the other half: 3 m/s over the 3.7 m/s peak
	for (unsigned seed = 1; seed <= 3; seed++) {
		expectHeld(x8Column(0.5f, 3.f / 3.7f, 0.5f, seed), "half");
	}
}

TEST(PortDrift, CircleCentreKeepsUpWithTheEstimate)
{
	// carried at 6 m/s, a 6.8 s smoothing trails by 41 m; the circle must hold the estimate inside half its 34.8 m radius
	const Flight f = x8Column(1.f, 0.f, 1.f, 1);
	ASSERT_GE(f.lag.size(), 500u);
	EXPECT_LE(*std::max_element(f.lag.begin(), f.lag.end()), 17.4f);
}

TEST(PortDrift, Canary)
{
	EXPECT_NONFATAL_FAILURE(EXPECT_NEAR(SC::drift_step(matrix::Vector2f{0.f, 6.f}, 0.02f, 0.f, 3.7f, 1.f)(1), -0.12f, 1e-5f),
				"");
}
