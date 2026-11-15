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
#include <string.h>
#include <lib/range_select/RangeSelect.hpp>
#include <drivers/uavcan/sensors/radar_limit.hpp>

using range_select::Selection;
using range_select::selectDownwardRange;

static constexpr uint64_t NOW = 10'000'000;
static constexpr uint64_t STALE = NOW - 100'000;

static distance_sensor_s sample(float d, float max_d, uint64_t t = NOW - 10'000, int8_t quality = 100, float min_d = 0.1f)
{
	distance_sensor_s s{};
	s.timestamp = t;
	s.current_distance = d;
	s.min_distance = min_d;
	s.max_distance = max_d;
	s.signal_quality = quality;
	s.orientation = distance_sensor_s::ROTATION_DOWNWARD_FACING;
	return s;
}

static Selection pick(const distance_sensor_s *s, int n, float &floor_state)
{
	return selectDownwardRange(s, n, STALE, floor_state);
}

struct Fed {
	bool sent;
	uint64_t time_us;
	float rng;
	int8_t quality;
	float min_d, max_d;
	uint64_t used_time;
};

// Reference: the EKF2 range input as the mean of every fresh in-range downward sample.
static Fed meanOfAll(const distance_sensor_s *s, int count, uint64_t timestamp_stale)
{
	distance_sensor_s newest{};
	distance_sensor_s good{};
	float sum = 0.f;
	uint64_t time_sum = 0;
	int n = 0;

	for (int i = 0; i < count; i++) {
		const distance_sensor_s &distance_sensor = s[i];

		if ((distance_sensor.timestamp > timestamp_stale)
		    && (distance_sensor.orientation == distance_sensor_s::ROTATION_DOWNWARD_FACING)) {

			if (distance_sensor.timestamp > newest.timestamp) {
				newest = distance_sensor;
			}

			if ((distance_sensor.signal_quality != 0)
			    && (distance_sensor.current_distance >= distance_sensor.min_distance)
			    && (distance_sensor.current_distance <= distance_sensor.max_distance)) {

				sum += distance_sensor.current_distance;
				time_sum += distance_sensor.timestamp;
				n++;

				if (distance_sensor.timestamp > good.timestamp) {
					good = distance_sensor;
				}
			}
		}
	}

	if (newest.timestamp == 0) {
		return Fed{false, 0, 0.f, 0, 0.f, 0.f, 0};
	}

	const distance_sensor_s &used = (n > 0) ? good : newest;
	return Fed{true, (n > 0) ? time_sum / n : used.timestamp, (n > 0) ? sum / n : used.current_distance,
		   used.signal_quality, used.min_distance, used.max_distance, used.timestamp};
}

static Fed fedBySelection(const distance_sensor_s *s, int count, uint64_t timestamp_stale, float &floor_state)
{
	const Selection sel = selectDownwardRange(s, count, timestamp_stale, floor_state);

	if (sel.used < 0) {
		return Fed{false, 0, 0.f, 0, 0.f, 0.f, 0};
	}

	const distance_sensor_s &used = s[sel.used];
	return Fed{true, sel.timestamp, sel.distance, used.signal_quality, used.min_distance, used.max_distance, used.timestamp};
}

// One sensor state per digit: unpublished, stale, in range, quality 0, quality unknown, under min, over max, at max, not downward.
static distance_sensor_s equalSensor(int state, int index, int step, bool same_time)
{
	const float max_d = 50.f;
	const uint64_t t = NOW - 1'000 * (1 + ((same_time ? 0 : index * 7) + step) % 90);
	const float d = 3.f + 1.37f * index + 0.011f * step;

	switch (state) {
	case 0: return distance_sensor_s{};

	case 1: return sample(d, max_d, STALE - 1'000);

	case 2: return sample(d, max_d, t);

	case 3: return sample(d, max_d, t, 0);

	case 4: return sample(d, max_d, t, -1);

	case 5: return sample(0.05f, max_d, t);

	case 6: return sample(max_d + 0.5f, max_d, t);

	case 7: return sample(max_d, max_d, t);

	default: {
			distance_sensor_s s = sample(d, max_d, t);
			s.orientation = distance_sensor_s::ROTATION_FORWARD_FACING;
			return s;
		}
	}
}

static bool sameBits(float a, float b)
{
	return memcmp(&a, &b, sizeof(float)) == 0;
}

static void expectSameFeed(int sensors)
{
	constexpr int STATES = 9;
	int combos = 1;

	for (int i = 0; i < sensors; i++) {
		combos *= STATES;
	}

	float floor_state = 0.f;
	int sent = 0;

	for (int step = 0; step < 2 * combos; step++) {
		distance_sensor_s s[4] {};
		int code = step % combos;

		for (int i = 0; i < sensors; i++) {
			s[i] = equalSensor(code % STATES, i, step, step >= combos);
			code /= STATES;
		}

		const Fed ref = meanOfAll(s, sensors, STALE);
		const Fed got = fedBySelection(s, sensors, STALE, floor_state);

		ASSERT_EQ(got.sent, ref.sent) << "step " << step;
		ASSERT_TRUE(sameBits(floor_state, 0.f)) << "step " << step;

		if (ref.sent) {
			sent++;
			ASSERT_EQ(got.time_us, ref.time_us) << "step " << step;
			ASSERT_TRUE(sameBits(got.rng, ref.rng)) << "step " << step;
			ASSERT_EQ(got.quality, ref.quality) << "step " << step;
			ASSERT_TRUE(sameBits(got.min_d, ref.min_d)) << "step " << step;
			ASSERT_TRUE(sameBits(got.max_d, ref.max_d)) << "step " << step;
			ASSERT_EQ(got.used_time, ref.used_time) << "step " << step;
		}
	}

	EXPECT_GT(sent, combos);
}

TEST(RangeSelectTest, OneSensorFeedsAsTheMean)
{
	expectSameFeed(1);
}

TEST(RangeSelectTest, TwoEqualSensorsFeedAsTheMean)
{
	expectSameFeed(2);
}

TEST(RangeSelectTest, FourEqualSensorsFeedAsTheMean)
{
	expectSameFeed(4);
}

TEST(RangeSelectTest, MeanOfTwoLidars)
{
	const distance_sensor_s s[] {sample(20.f, 50.f, NOW - 30'000), sample(21.f, 50.f, NOW - 10'000)};
	float floor_state = 0.f;
	const Selection sel = pick(s, 2, floor_state);
	EXPECT_EQ(sel.count, 2);
	EXPECT_FLOAT_EQ(sel.distance, 20.5f);
	EXPECT_EQ(sel.timestamp, NOW - 20'000);
	EXPECT_EQ(sel.used, 1);
	EXPECT_FLOAT_EQ(sel.range_max, 50.f);
}

TEST(RangeSelectTest, ShortestGroupWinsOverRadar)
{
	const distance_sensor_s s[] {sample(30.f, 100.f), sample(31.f, 50.f), sample(31.4f, 50.f)};
	float floor_state = 0.f;
	const Selection sel = pick(s, 3, floor_state);
	EXPECT_EQ(sel.count, 2);
	EXPECT_FLOAT_EQ(sel.distance, 31.2f);
	EXPECT_FLOAT_EQ(sel.range_max, 100.f);
	EXPECT_FLOAT_EQ(floor_state, 0.f);
}

TEST(RangeSelectTest, GroupSpansOnePointTwoOfTheShortest)
{
	const distance_sensor_s inside[] {sample(20.f, 50.f), sample(22.f, 60.f)};
	const distance_sensor_s outside[] {sample(20.f, 50.f), sample(22.f, 60.5f)};
	float floor_state = 0.f;
	EXPECT_EQ(pick(inside, 2, floor_state).count, 2);
	const Selection sel = pick(outside, 2, floor_state);
	EXPECT_EQ(sel.count, 1);
	EXPECT_FLOAT_EQ(sel.distance, 20.f);
}

TEST(RangeSelectTest, RadarAlonePastLidarRange)
{
	float floor_state = 0.f;
	const distance_sensor_s over[] {sample(52.f, 50.f), sample(60.f, 100.f)};
	Selection sel = pick(over, 2, floor_state);
	EXPECT_EQ(sel.count, 1);
	EXPECT_FLOAT_EQ(sel.distance, 60.f);
	EXPECT_EQ(sel.used, 1);
	EXPECT_FLOAT_EQ(floor_state, 100.f);

	const distance_sensor_s no_return[] {sample(0.f, 50.f, NOW - 10'000, 0), sample(60.f, 100.f)};
	sel = pick(no_return, 2, floor_state);
	EXPECT_EQ(sel.count, 1);
	EXPECT_FLOAT_EQ(sel.distance, 60.f);
}

TEST(RangeSelectTest, LidarRejoinsUnderNineTenthsOfItsRange)
{
	float floor_state = 0.f;
	const distance_sensor_s out[] {sample(50.5f, 50.f), sample(50.8f, 100.f)};
	ASSERT_FLOAT_EQ(pick(out, 2, floor_state).distance, 50.8f);

	// 0.9 x 50 m = 45 m
	const distance_sensor_s above[] {sample(45.5f, 50.f), sample(45.8f, 100.f)};
	Selection sel = pick(above, 2, floor_state);
	EXPECT_FLOAT_EQ(sel.distance, 45.8f);
	EXPECT_FLOAT_EQ(floor_state, 100.f);

	const distance_sensor_s below[] {sample(44.5f, 50.f), sample(44.8f, 100.f)};
	sel = pick(below, 2, floor_state);
	EXPECT_FLOAT_EQ(sel.distance, 44.5f);
	EXPECT_FLOAT_EQ(floor_state, 0.f);

	const distance_sensor_s back_up[] {sample(49.5f, 50.f), sample(49.8f, 100.f)};
	EXPECT_FLOAT_EQ(pick(back_up, 2, floor_state).distance, 49.5f);
}

TEST(RangeSelectTest, LidarAloneIgnoresHysteresisOnceRadarGoes)
{
	float floor_state = 0.f;
	const distance_sensor_s out[] {sample(50.5f, 50.f), sample(50.8f, 100.f)};
	pick(out, 2, floor_state);
	ASSERT_FLOAT_EQ(floor_state, 100.f);

	const distance_sensor_s radar_stale[] {sample(47.f, 50.f), sample(47.f, 100.f, STALE - 1)};
	EXPECT_EQ(pick(radar_stale, 2, floor_state).count, 0);
	EXPECT_FLOAT_EQ(floor_state, 0.f);
	const Selection sel = pick(radar_stale, 2, floor_state);
	EXPECT_EQ(sel.count, 1);
	EXPECT_FLOAT_EQ(sel.distance, 47.f);
}

TEST(RangeSelectTest, StaleAndQualityZeroDropped)
{
	float floor_state = 0.f;
	const distance_sensor_s s[] {sample(10.f, 50.f, STALE), sample(11.f, 50.f, NOW - 10'000, 0), sample(12.f, 50.f, NOW - 20'000)};
	const Selection sel = pick(s, 3, floor_state);
	EXPECT_EQ(sel.count, 1);
	EXPECT_FLOAT_EQ(sel.distance, 12.f);
	EXPECT_EQ(sel.used, 2);
}

TEST(RangeSelectTest, NothingGoodPassesTheNewestFreshSample)
{
	float floor_state = 0.f;
	const distance_sensor_s s[] {sample(60.f, 50.f, NOW - 20'000), sample(0.f, 50.f, NOW - 5'000, 0), sample(5.f, 50.f, STALE)};
	const Selection sel = pick(s, 3, floor_state);
	EXPECT_EQ(sel.count, 0);
	EXPECT_EQ(sel.used, 1);
	EXPECT_FLOAT_EQ(sel.distance, 0.f);
	EXPECT_EQ(sel.timestamp, NOW - 5'000);

	const distance_sensor_s none[] {sample(5.f, 50.f, STALE), distance_sensor_s{}};
	EXPECT_EQ(pick(none, 2, floor_state).used, -1);
}

static Selection pickSplit(const distance_sensor_s *s, int n, float &floor_state)
{
	return selectDownwardRange(s, n, STALE, floor_state, true);
}

TEST(RangeSelectTest, FrozenHighLidarPullsTheMeanOfTwo)
{
	// lidar 0 frozen at 25 m over ground that rose 10 m, lidar 1 live at 15 m
	const distance_sensor_s s[] {sample(25.f, 50.f, NOW - 30'000), sample(15.f, 50.f, NOW - 10'000)};
	float floor_state = 0.f;
	EXPECT_FLOAT_EQ(pick(s, 2, floor_state).distance, 20.f);
	const Selection sel = pickSplit(s, 2, floor_state);
	EXPECT_EQ(sel.count, 1);
	EXPECT_EQ(sel.used, 1);
	EXPECT_FLOAT_EQ(sel.distance, 15.f);
	EXPECT_EQ(sel.timestamp, NOW - 10'000);
}

TEST(RangeSelectTest, FrozenHighLidarWinsOverRadar)
{
	// lidar frozen at 25 m, radar live at 15 m: the shortest group wins unless the split rule acts
	const distance_sensor_s s[] {sample(25.f, 50.f), sample(15.f, 100.f, NOW - 20'000)};
	float floor_state = 0.f;
	EXPECT_FLOAT_EQ(pick(s, 2, floor_state).distance, 25.f);
	const Selection sel = pickSplit(s, 2, floor_state);
	EXPECT_EQ(sel.used, 1);
	EXPECT_FLOAT_EQ(sel.distance, 15.f);
	EXPECT_EQ(sel.timestamp, NOW - 20'000);
	EXPECT_FLOAT_EQ(sel.range_max, 100.f);
}

TEST(RangeSelectTest, SplitOfTwoMetresKeepsTheGroup)
{
	float floor_state = 0.f;
	const distance_sensor_s lidars[] {sample(20.f, 50.f), sample(22.f, 50.f)};
	EXPECT_FLOAT_EQ(pickSplit(lidars, 2, floor_state).distance, 21.f);
	const distance_sensor_s near_radar[] {sample(30.f, 50.f), sample(28.f, 100.f)};
	EXPECT_FLOAT_EQ(pickSplit(near_radar, 2, floor_state).distance, 30.f);
	const distance_sensor_s far_radar[] {sample(30.f, 50.f), sample(27.9f, 100.f)};
	EXPECT_FLOAT_EQ(pickSplit(far_radar, 2, floor_state).distance, 27.9f);
}

TEST(RangeSelectTest, SplitIgnoresSamplesThatAreNotGood)
{
	float floor_state = 0.f;
	// a stale, quality 0 or out of range reading never wins on the split
	const distance_sensor_s s[] {sample(25.f, 50.f), sample(5.f, 50.f, STALE), sample(6.f, 50.f, NOW - 10'000, 0), sample(0.05f, 50.f)};
	const Selection sel = pickSplit(s, 4, floor_state);
	EXPECT_EQ(sel.used, 0);
	EXPECT_FLOAT_EQ(sel.distance, 25.f);
}

TEST(RangeSelectTest, FrozenSampleLeavesTheFeed)
{
	// EKF2's feed in the flare: one lidar frozen at 3 m, the other live and descending at 1 m/s, both at 50 Hz
	for (int frozen = 0; frozen < 2; frozen++) {
		range_select::StuckCheck checks[2] {};
		float floor_state = 0.f;

		for (int k = 0; k < 20; k++) {
			const uint64_t t = NOW + k * 20'000;
			const float h = 3.f - 0.02f * k;
			distance_sensor_s s[2] {sample(h, 50.f, t), sample(h, 50.f, t)};
			s[frozen].current_distance = 3.f;
			range_select::dropFrozen(s, 2, checks, h);
			const Selection sel = selectDownwardRange(s, 2, t - 100'000, floor_state);

			// the 10th sample of the same distance is the first one dropped
			if (k < 9) {
				EXPECT_FLOAT_EQ(sel.distance, 0.5f * (3.f + h)) << frozen << " " << k;

			} else {
				EXPECT_EQ(sel.count, 1) << frozen << " " << k;
				EXPECT_FLOAT_EQ(sel.distance, h) << frozen << " " << k;
			}
		}
	}
}

TEST(RangeSelectTest, FrozenLoneSensorFeedsQualityZero)
{
	// one lidar frozen at 30 m: from the 10th repeat EKF2 still receives each sample, marked invalid, at its own time
	range_select::StuckCheck checks[1] {};
	float floor_state = 0.f;

	for (int k = 0; k < 20; k++) {
		const uint64_t t = NOW + k * 20'000;
		distance_sensor_s s[1] {sample(30.f, 50.f, t)};
		range_select::dropFrozen(s, 1, checks, 100.f - 0.02f * k);
		const Fed fed = fedBySelection(s, 1, t - 100'000, floor_state);
		ASSERT_TRUE(fed.sent) << k;
		EXPECT_EQ(fed.time_us, t) << k;
		EXPECT_EQ(fed.quality, k < 9 ? 100 : 0) << k;
	}
}

TEST(RangeSelectTest, RadarNodeAtTheDefaultsKeepsTheStockFeed)
{
	// UAVCAN_RNG_MIN 0, UAVCAN_RNG_MAX 999, UAVCAN_RDR_MIN and UAVCAN_RDR_MAX -1: a lidar node and a radar node, both good
	const distance_sensor_s s[] {sample(20.f, 999.f, NOW - 10'000, 100, 0.f),
				     sample(21.5f, radarLimit(-1.f, 999.f), NOW - 10'000, 100, radarLimit(-1.f, 0.f))
				    };
	float floor_state = 0.f;
	const Selection sel = pick(s, 2, floor_state);
	EXPECT_EQ(sel.count, 2);
	EXPECT_FLOAT_EQ(sel.distance, 20.75f);
	// a radar limit at 0 or above is the radar's own
	EXPECT_FLOAT_EQ(radarLimit(100.f, 50.f), 100.f);
	EXPECT_FLOAT_EQ(radarLimit(0.5f, 0.08f), 0.5f);
	EXPECT_FLOAT_EQ(radarLimit(0.f, 0.08f), 0.f);
}

TEST(RangeSelectCanary, WrongMeanIsCaught)
{
	const distance_sensor_s s[] {sample(20.f, 50.f), sample(21.f, 50.f)};
	float floor_state = 0.f;
	EXPECT_NONFATAL_FAILURE(EXPECT_FLOAT_EQ(pick(s, 2, floor_state).distance, 20.6f), "");
}
