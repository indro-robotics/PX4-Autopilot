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

#pragma once

#include <drivers/drv_hrt.h>
#include <lib/geo/geo.h>
#include <uORB/topics/sensor_gps.h>

namespace launchdetection
{

using namespace time_literals;

static constexpr hrt_abstime kCourseFixMaxAge = 300_ms;

// [m] a position error of 5 m over a 20 m track turns the course by up to 14 deg
static constexpr float kCourseEphMax = 2.f;

// climb-out course from the GNSS track: a steel launcher rail beside the compass bends the heading at detection
class LaunchCourse
{
public:
	void reset()
	{
		_start_lat = NAN;
		_done = false;
	}

	/**
	 * @param gps latest vehicle_gps_position
	 * @param now current time [us]
	 * @param distance FW_LAUN_CRS_DST [m], 0 disables
	 * @return bearing [rad] from the first fix of type 3D to RTK fixed with eph under kCourseEphMax after reset() to
	 * the first one distance away, once; NAN otherwise
	 */
	float update(const sensor_gps_s &gps, const hrt_abstime now, const float distance)
	{
		if (_done || !(distance > 0.f) || gps.fix_type < sensor_gps_s::FIX_TYPE_3D
		    || gps.fix_type > sensor_gps_s::FIX_TYPE_RTK_FIXED || !(gps.eph < kCourseEphMax)
		    || now > gps.timestamp + kCourseFixMaxAge) {
			return NAN;
		}

		if (!PX4_ISFINITE(_start_lat)) {
			_start_lat = gps.latitude_deg;
			_start_lon = gps.longitude_deg;
			return NAN;
		}

		if (get_distance_to_next_waypoint(_start_lat, _start_lon, gps.latitude_deg, gps.longitude_deg) < distance) {
			return NAN;
		}

		_done = true;
		return get_bearing_to_next_waypoint(_start_lat, _start_lon, gps.latitude_deg, gps.longitude_deg);
	}

private:
	double _start_lat{NAN};
	double _start_lon{NAN};
	bool _done{false};
};

} // namespace launchdetection
