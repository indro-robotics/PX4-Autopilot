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

/**
 * @file GotoSession.hpp
 *
 * Goto stream governance for the Auto flight task. While a goto stream is fresh the task target is
 * the goto setpoint; when the stream goes stale the target is the vehicle state at that instant;
 * a navigator target that carries a horizontal position, or one the session does not govern, ends
 * the session. Without a goto stream the session never governs.
 */

#pragma once

#include <cmath>
#include <stdint.h>
#include <matrix/math.hpp>

class GotoSession
{
public:
	struct Input {
		uint64_t timestamp{0}; // goto_setpoint timestamp, 0 while none was received
		matrix::Vector3f position{NAN, NAN, NAN};
		bool control_heading{false};
		float heading{NAN};
		bool set_max_horizontal_speed{false};
		float max_horizontal_speed{NAN};
		bool set_max_vertical_speed{false};
		float max_vertical_speed{NAN};
	};

	// Same freshness window as GotoControl, which stops publishing at this age.
	static constexpr uint64_t TIMEOUT_US = 500000;

	/**
	 * @param governed the navigator target type accepts goto governance
	 * @param has_xy the navigator target carries a horizontal position
	 * @return true on the cycle the stream went stale and the vehicle state became the target
	 */
	bool update(const uint64_t now, const Input &in, const bool governed, const bool has_xy,
		    const matrix::Vector3f &position, const float yaw)
	{
		if (!governed || has_xy) {
			_state = State::none;
			return false;
		}

		const bool stream_fresh = (in.timestamp != 0) && (now < in.timestamp + TIMEOUT_US) && in.position.isAllFinite();

		if (stream_fresh) {
			_state = State::fresh;
			_target = in.position;
			_heading = (in.control_heading && std::isfinite(in.heading)) ? in.heading : NAN;
			_max_horizontal_speed = (in.set_max_horizontal_speed
						 && std::isfinite(in.max_horizontal_speed)) ? in.max_horizontal_speed : NAN;
			_max_vertical_speed = (in.set_max_vertical_speed
					       && std::isfinite(in.max_vertical_speed)) ? in.max_vertical_speed : NAN;
			return false;
		}

		if (_state == State::fresh) {
			_state = State::held;
			_max_horizontal_speed = NAN;
			_max_vertical_speed = NAN;

			if (position.isAllFinite()) {
				_target = position;
				_heading = std::isfinite(yaw) ? yaw : NAN;
			}

			return true;
		}

		return false;
	}

	void clear() { _state = State::none; }

	bool governs() const { return _state != State::none; }
	bool fresh() const { return _state == State::fresh; }

	const matrix::Vector3f &target() const { return _target; }
	float heading() const { return _heading; } // NAN while the session controls no heading
	float maxHorizontalSpeed() const { return _max_horizontal_speed; } // NAN for the vehicle default
	float maxVerticalSpeed() const { return _max_vertical_speed; } // NAN for the vehicle default

	// Estimator position reset: the held target follows the estimate, a NAN axis stays.
	void shiftTarget(const matrix::Vector3f &delta)
	{
		if (_state != State::held) {
			return;
		}

		for (size_t i = 0; i < 3; i++) {
			if (std::isfinite(delta(i))) {
				_target(i) += delta(i);
			}
		}
	}

	void shiftHeading(const float delta)
	{
		if ((_state == State::held) && std::isfinite(_heading) && std::isfinite(delta)) {
			_heading = matrix::wrap_pi(_heading + delta);
		}
	}

private:
	enum class State { none, fresh, held };

	State _state{State::none};
	matrix::Vector3f _target{};
	float _heading{NAN};
	float _max_horizontal_speed{NAN};
	float _max_vertical_speed{NAN};
};
