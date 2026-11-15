/****************************************************************************
 *
 *   Copyright (c) 2013-2022 PX4 Development Team. All rights reserved.
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

/**
 * @file LaunchDetector.h
 * Auto launch detection for catapult/hand-launch vehicles
 *
 * @author Thomas Gubler <thomasgubler@gmail.com>
 */

#ifndef LAUNCHDETECTOR_H
#define LAUNCHDETECTOR_H

#include <mathlib/math/Limits.hpp>
#include <px4_platform_common/module_params.h>
#include <uORB/topics/launch_detection_status.h>

namespace launchdetection
{

// Info delay threshold (to publish info every kInfoDelay seconds)
static constexpr float kInfoDelay = 4.f;

// [m/s] a detection that never reached this speed was a knock on the rail, not a launch
static constexpr float kKnockSpeed = 2.f;

// [rad] attitude limits at the gate decision; outside them the shot left the rail rolled or tumbling
static constexpr float kRollMax = math::radians(30.f);
static constexpr float kPitchMin = math::radians(-30.f);
static constexpr float kPitchMax = math::radians(45.f);

// a pilot who climbs an aborted launch away hands a flying aircraft back: the latch would glide it down at idle in the next auto mode
inline bool abortLatch(bool latched, bool armed, bool detector_aborted, bool pilot_attitude_mode, bool landed,
		       bool airspeed_valid, float airspeed, float airspeed_min)
{
	const bool flown_away = pilot_attitude_mode && !landed && airspeed_valid && airspeed > airspeed_min;
	return armed && !flown_away && (latched || detector_aborted);
}

class __EXPORT LaunchDetector : public ModuleParams
{
public:
	LaunchDetector(ModuleParams *parent) : ModuleParams(parent) {}
	~LaunchDetector() = default;

	LaunchDetector(const LaunchDetector &) = delete;
	LaunchDetector operator=(const LaunchDetector &) = delete;

	/**
	 * @brief Reset launch detection state machine.
	 */
	void reset();

	/**
	 * @brief Updates the state machine based on the current vehicle condition.
	 *
	 * @param dt Time step [us]
	 * @param accel_x Measured acceleration in body x [m/s/s]
	 * @param speed Inertial speed [m/s], NAN when invalid
	 * @param roll Roll angle [rad]
	 * @param pitch Pitch angle [rad]
	 */
	void update(const float dt, const float accel_x, const float speed, const float roll, const float pitch);

	/**
	 * @return true after a launch too slow for FW_LAUN_SPD_MIN or outside the attitude limits, until reset()
	 */
	bool aborted() const { return aborted_; }

	/**
	 * @return state for launch_detection_status: with FW_LAUN_SPD_MIN above 0 a detection reads Waiting until the
	 * speed reaches kKnockSpeed, so a knock on the rail never counts as a takeoff
	 */
	uint getPublishedState() const;

	/**
	 * @brief Get the Launch Detected state
	 *
	 * @return uint (aligned with launch_detection_status_s::launch_detection_state)
	 */
	uint getLaunchDetected() const;

	/**
	 * @brief Forces state of launch detection state machine to STATE_FLYING.
	 */
	void forceSetFlyState() { state_ = launch_detection_status_s::STATE_FLYING; }

private:
	/**
	 * Motor delay counter [s]
	 */
	float motor_delay_counter_{0.f};

	/**
	 * Info delay counter (to publish info every kInfoDelay seconds) [s]
	 */
	float info_delay_counter_s_{kInfoDelay};

	/**
	 *  Counter for how long the measured acceleration is above the defined threshold [s]
	 */
	float acceleration_detected_counter_{0.f};

	/**
	 * Current state of the launch detection state machine [launch_detection_status_s::launch_detection_state]
	 */
	uint state_{launch_detection_status_s::STATE_WAITING_FOR_LAUNCH};

	float speed_max_{0.f};

	bool aborted_{false};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::FW_LAUN_AC_THLD>) param_fw_laun_ac_thld_,
		(ParamFloat<px4::params::FW_LAUN_AC_T>) param_fw_laun_ac_t_,
		(ParamFloat<px4::params::FW_LAUN_MOT_DEL>) param_fw_laun_mot_del_,
		(ParamFloat<px4::params::FW_LAUN_SPD_MIN>) param_fw_laun_spd_min_
	)
};

} // namespace launchdetection

#endif // LAUNCHDETECTOR_H
