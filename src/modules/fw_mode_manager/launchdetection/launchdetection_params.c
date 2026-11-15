/****************************************************************************
 *
 *   Copyright (c) 2013, 2014 PX4 Development Team. All rights reserved.
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
 * @file launchdetection_params.c
 *
 * Parameters for launchdetection
 *
 * @author Thomas Gubler <thomasgubler@gmail.com>
 */

/**
 * Trigger acceleration threshold
 *
 * Launch is detected when acceleration in body forward direction is above FW_LAUN_AC_THLD for FW_LAUN_AC_T seconds.
 *
 * @unit m/s^2
 * @min 0
 * @decimal 1
 * @increment 0.5
 * @group FW Auto Takeoff
 */
PARAM_DEFINE_FLOAT(FW_LAUN_AC_THLD, 30.0f);

/**
 * Trigger time
 *
 * Launch is detected when acceleration in body forward direction is above FW_LAUN_AC_THLD for FW_LAUN_AC_T seconds.
 *
 * @unit s
 * @min 0.0
 * @max 5.0
 * @decimal 2
 * @increment 0.05
 * @group FW Auto Takeoff
 */
PARAM_DEFINE_FLOAT(FW_LAUN_AC_T, 0.05f);

/**
 * Motor delay
 *
 * Start the motor(s) this amount of seconds after launch is detected.
 *
 * @unit s
 * @min 0.0
 * @max 10.0
 * @decimal 1
 * @increment 0.5
 * @group FW Auto Takeoff
 */
PARAM_DEFINE_FLOAT(FW_LAUN_MOT_DEL, 0.0f);

/**
 * Minimum launch speed
 *
 * The motor starts after FW_LAUN_MOT_DEL only when the inertial speed since the launch detection reached this value. A launch whose acceleration ends below it, or that leaves the rail rolled past 30 deg or pitched outside -30 to 45 deg, keeps the throttle at FW_THR_IDLE, and in every auto mode levels the wings and holds FW_LND_FL_PMAX pitch, until disarm or until a pilot flies the aircraft in Manual, Stabilized or Acro above FW_AIRSPD_MIN; one under 2 m/s re-arms the launch detection and does not count as a takeoff. Manual, Stabilized and Acro keep the pilot's throttle after such a launch. A launch fired while a knock on the rail is still judged starts the motor up to FW_LAUN_MOT_DEL late. The speed is inertial: a headwind lowers it below the airspeed and a tailwind raises it. The estimate at the rail lip reads up to 0.7 m/s off the true speed. 0 disables the check. Needs FW_LAUN_MOT_DEL above 0.
 *
 * @unit m/s
 * @min 0.0
 * @max 20.0
 * @decimal 1
 * @increment 0.5
 * @group FW Auto Takeoff
 */
PARAM_DEFINE_FLOAT(FW_LAUN_SPD_MIN, 0.0f);

/**
 * Launch course distance
 *
 * GNSS ground distance from the launch detection after which the Takeoff course is the GNSS track over that distance, set once per launch, in place of the heading at detection. A takeoff waypoint over 10 m away still sets the course. 0 keeps the heading at detection.
 *
 * @unit m
 * @min 0.0
 * @max 100.0
 * @decimal 0
 * @increment 1.0
 * @group FW Auto Takeoff
 */
PARAM_DEFINE_FLOAT(FW_LAUN_CRS_DST, 0.0f);

/**
 * Takeoff speed weight
 *
 * TECS speed weight in Takeoff once the launch detection starts the motor. 2 flies FW_TKO_AIRSPD with pitch alone, above the Takeoff pitch floor. -1 uses FW_T_SPDWEIGHT.
 *
 * @min -1.0
 * @max 2.0
 * @decimal 1
 * @increment 0.1
 * @group FW Auto Takeoff
 */
PARAM_DEFINE_FLOAT(FW_TKO_SPDWT, -1.0f);

/**
 * Takeoff pitch floor height
 *
 * Height above the launch point from which the Takeoff pitch floor is FW_LND_FL_PMIN instead of FW_TKO_PITCH_MIN, for the rest of the Takeoff. 0 sets no height. A thrust loss flag (FD_FW_TL_TIME) sets the same floor at any height.
 *
 * @unit m
 * @min 0.0
 * @max 100.0
 * @decimal 1
 * @increment 1.0
 * @group FW Auto Takeoff
 */
PARAM_DEFINE_FLOAT(FW_TKO_PMIN_ALT, 0.0f);
