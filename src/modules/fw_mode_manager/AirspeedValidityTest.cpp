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

#include "airspeed_validity.hpp"

namespace
{

using airspeed_validity::valid;

TEST(AirspeedValidity, ASampleSetsTheValueAndTheTimeOfTheValidity)
{
	float airspeed = 0.f;
	uint64_t time = 0;
	// 60 s after boot no sample has come: no airspeed
	EXPECT_FALSE(valid(60000000, time));
	airspeed_validity::sample(15.2f, false, 60000000, airspeed, time);
	EXPECT_FLOAT_EQ(airspeed, 15.2f);
	EXPECT_EQ(time, 60000000u);
	EXPECT_TRUE(valid(60020000, time));
	// the validity ends one second after the last sample
	EXPECT_TRUE(valid(60999999, time));
	EXPECT_FALSE(valid(61000000, time));
	airspeed_validity::sample(14.1f, false, 61500000, airspeed, time);
	EXPECT_FLOAT_EQ(airspeed, 14.1f);
	EXPECT_TRUE(valid(61520000, time));
}

TEST(AirspeedValidity, ASyntheticOrUnreadSampleSetsNeither)
{
	float airspeed = 15.2f;
	uint64_t time = 60000000;
	// a synthetic airspeed is the ground speed less the wind estimate: the consumers take it for no airspeed
	airspeed_validity::sample(17.f, true, 60500000, airspeed, time);
	EXPECT_FLOAT_EQ(airspeed, 15.2f);
	EXPECT_EQ(time, 60000000u);
	airspeed_validity::sample(NAN, false, 60500000, airspeed, time);
	EXPECT_FLOAT_EQ(airspeed, 15.2f);
	EXPECT_EQ(time, 60000000u);
	airspeed_validity::sample(INFINITY, false, 60500000, airspeed, time);
	EXPECT_FLOAT_EQ(airspeed, 15.2f);
	EXPECT_EQ(time, 60000000u);
	// so a sensor that stops reading ends the validity one second after its last sample
	EXPECT_FALSE(valid(61000000, time));
}

TEST(AirspeedValidity, CanaryAStaleSampleCountsAsValid)
{
	// must fail: the suite shows a failure when the validity outlives its second
	EXPECT_NONFATAL_FAILURE(EXPECT_TRUE(valid(61000000, 60000000)), "");
}

} // namespace
