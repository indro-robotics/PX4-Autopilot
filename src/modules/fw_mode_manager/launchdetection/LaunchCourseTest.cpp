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

#include "LaunchCourse.hpp"

using launchdetection::LaunchCourse;

static constexpr double kLat0 = 43.47;
static constexpr double kLon0 = -80.54;
static constexpr double kEarthRadius = 6371000.0;
static constexpr float kDistance = 20.f;
static constexpr hrt_abstime kNow = 100000000;

// A 3D fix the given metres north and east of the launch point, age_s old at kNow.
static sensor_gps_s fix(double north, double east, float age_s = 0.1f, uint8_t fix_type = sensor_gps_s::FIX_TYPE_3D)
{
	sensor_gps_s gps{};
	gps.timestamp = kNow - (hrt_abstime)(age_s * 1e6f);
	gps.latitude_deg = kLat0 + north / kEarthRadius * 180.0 / M_PI;
	gps.longitude_deg = kLon0 + east / (kEarthRadius * cos(kLat0 * M_PI / 180.0)) * 180.0 / M_PI;
	gps.fix_type = fix_type;
	return gps;
}

static float bearing_deg(double north, double east)
{
	return (float)(atan2(east, north) * 180.0 / M_PI);
}

TEST(LaunchCourseTest, CourseIsTheTrackFromDetectionAtTheDistance)
{
	LaunchCourse course;
	const double n = 20.1 * cos(-60.0 * M_PI / 180.0);
	const double e = 20.1 * sin(-60.0 * M_PI / 180.0);
	EXPECT_TRUE(isnan(course.update(fix(0, 0), kNow, kDistance)));
	EXPECT_TRUE(isnan(course.update(fix(n / 2, e / 2), kNow, kDistance)));
	// 0.01 deg: flat-earth fixes differ from the sphere by under 0.001 deg over 20 m
	EXPECT_NEAR(math::degrees(course.update(fix(n, e), kNow, kDistance)), -60.f, 0.01f);
}

TEST(LaunchCourseTest, UnderTheDistanceGivesNoCourse)
{
	LaunchCourse course;
	course.update(fix(0, 0), kNow, kDistance);
	EXPECT_TRUE(isnan(course.update(fix(19.9, 0), kNow, kDistance)));
	EXPECT_NEAR(math::degrees(course.update(fix(20.1, 0), kNow, kDistance)), 0.f, 0.01f);
}

TEST(LaunchCourseTest, StartIsTheFirstFixAfterReset)
{
	LaunchCourse course;
	course.update(fix(0, 0), kNow, kDistance);
	course.update(fix(0, 10), kNow, kDistance);
	EXPECT_NEAR(math::degrees(course.update(fix(20, 10), kNow, kDistance)), bearing_deg(20, 10), 0.01f);
}

TEST(LaunchCourseTest, CourseIsSetOncePerLaunch)
{
	LaunchCourse course;
	course.update(fix(0, 0), kNow, kDistance);
	EXPECT_FALSE(isnan(course.update(fix(0, 25), kNow, kDistance)));
	EXPECT_TRUE(isnan(course.update(fix(30, 25), kNow, kDistance)));
	EXPECT_TRUE(isnan(course.update(fix(0, 60), kNow, kDistance)));
}

TEST(LaunchCourseTest, ResetStartsANewLaunch)
{
	LaunchCourse course;
	course.update(fix(0, 0), kNow, kDistance);
	course.update(fix(0, 25), kNow, kDistance);
	course.reset();
	EXPECT_TRUE(isnan(course.update(fix(0, 25), kNow, kDistance)));
	EXPECT_NEAR(math::degrees(course.update(fix(-15, 40), kNow, kDistance)), bearing_deg(-15, 15), 0.01f);
}

TEST(LaunchCourseTest, DistanceZeroKeepsTheHeading)
{
	LaunchCourse course;
	course.update(fix(0, 0), kNow, 0.f);
	EXPECT_TRUE(isnan(course.update(fix(100, 0), kNow, 0.f)));
	EXPECT_TRUE(isnan(course.update(fix(200, 0), kNow, 0.f)));
}

TEST(LaunchCourseTest, FixUnder3DNeitherStartsNorEnds)
{
	LaunchCourse course;
	course.update(fix(0, 50, 0.1f, sensor_gps_s::FIX_TYPE_2D), kNow, kDistance);
	course.update(fix(0, 0), kNow, kDistance);
	EXPECT_TRUE(isnan(course.update(fix(25, 0, 0.1f, sensor_gps_s::FIX_TYPE_2D), kNow, kDistance)));
	EXPECT_NEAR(math::degrees(course.update(fix(25, 0), kNow, kDistance)), 0.f, 0.01f);
}

TEST(LaunchCourseTest, FixOlderThanThreeTenthsOfASecondIsIgnored)
{
	LaunchCourse course;
	course.update(fix(0, 50, 0.31f), kNow, kDistance);
	course.update(fix(0, 0, 0.29f), kNow, kDistance);
	EXPECT_TRUE(isnan(course.update(fix(25, 0, 0.31f), kNow, kDistance)));
	EXPECT_NEAR(math::degrees(course.update(fix(25, 0, 0.29f), kNow, kDistance)), 0.f, 0.01f);
}
