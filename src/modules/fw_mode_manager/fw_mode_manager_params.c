/****************************************************************************
 *
 *   Copyright (c) 2013-2025 PX4 Development Team. All rights reserved.
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
 * NPFG period
 *
 * Period of NPFG control law.
 *
 * @unit s
 * @min 1.0
 * @max 100.0
 * @decimal 1
 * @increment 0.1
 * @group FW NPFG Control
 */
PARAM_DEFINE_FLOAT(NPFG_PERIOD, 10.0f);

/**
 * NPFG damping ratio
 *
 * Damping ratio of NPFG control law.
 *
 * @min 0.10
 * @max 1.00
 * @decimal 2
 * @increment 0.01
 * @group FW NPFG Control
 */
PARAM_DEFINE_FLOAT(NPFG_DAMPING, 0.7f);

/**
 * Enable automatic lower bound on the NPFG period
 *
 * Avoids limit cycling from a too aggressively tuned period/damping combination.
 * If false, also disables upper bound NPFG_PERIOD_UB.
 *
 * @boolean
 * @group FW NPFG Control
 */
PARAM_DEFINE_INT32(NPFG_LB_PERIOD, 1);

/**
 * Enable automatic upper bound on the NPFG period
 *
 * Adapts period to maintain track keeping in variable winds and path curvature.
 *
 * @boolean
 * @group FW NPFG Control
 */
PARAM_DEFINE_INT32(NPFG_UB_PERIOD, 1);

/**
 * Roll time constant
 *
 * Time constant of roll controller command / response, modeled as first order delay.
 * Used to determine lower period bound. Setting zero disables automatic period bounding.
 *
 * @unit s
 * @min 0.00
 * @max 2.00
 * @decimal 2
 * @increment 0.05
 * @group FW NPFG Control
 */
PARAM_DEFINE_FLOAT(NPFG_ROLL_TC, 0.5f);

/**
 * NPFG switch distance multiplier
 *
 * Multiplied by the track error boundary to determine when the aircraft switches
 * to the next waypoint and/or path segment. Should be less than 1.
 *
 * @min 0.1
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group FW NPFG Control
 */
PARAM_DEFINE_FLOAT(NPFG_SW_DST_MLT, 0.32f);

/**
 * Period safety factor
 *
 * Multiplied by period for conservative minimum period bounding (when period lower
 * bounding is enabled). 1.0 bounds at marginal stability.
 *
 * @min 1.0
 * @max 10.0
 * @decimal 1
 * @increment 0.1
 * @group FW NPFG Control
 */
PARAM_DEFINE_FLOAT(NPFG_PERIOD_SF, 1.5f);


/**
 * Minimum pitch angle setpoint
 *
 * Applies in any altitude controlled flight mode.
 *
 * @unit deg
 * @min -60.0
 * @max 0.0
 * @decimal 1
 * @increment 0.5
 * @group FW General
 */
PARAM_DEFINE_FLOAT(FW_P_LIM_MIN, -30.0f);

/**
 * Maximum pitch angle setpoint
 *
 * Applies in any altitude controlled flight mode.
 *
 * @unit deg
 * @min 0.0
 * @max 80.0
 * @decimal 1
 * @increment 0.5
 * @group FW General
 */
PARAM_DEFINE_FLOAT(FW_P_LIM_MAX, 30.0f);

/**
 * Maximum roll angle setpoint
 *
 * Applies in any altitude controlled flight mode.
 *
 * @unit deg
 * @min 35.0
 * @max 75.0
 * @decimal 1
 * @increment 0.5
 * @group FW General
 */
PARAM_DEFINE_FLOAT(FW_R_LIM, 50.0f);

/**
 * Throttle limit max
 *
 * Applies in any altitude controlled flight mode.
 * Should be set accordingly to achieve FW_T_CLMB_MAX.
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group FW General
 */
PARAM_DEFINE_FLOAT(FW_THR_MAX, 1.0f);

/**
 * Throttle limit min
 *
 * Applies in any altitude controlled flight mode.
 * Usually set to 0 but can be increased to prevent the motor from stopping when
 * descending, which can increase achievable descent rates.
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group FW General
 */
PARAM_DEFINE_FLOAT(FW_THR_MIN, 0.0f);

/**
 * Idle throttle
 *
 * This is the minimum throttle while on the ground ("landed") in auto modes.
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group FW General
 */
PARAM_DEFINE_FLOAT(FW_THR_IDLE, 0.0f);

/**
 * Maximum landing slope angle
 *
 * Typically the desired landing slope angle when landing configuration (flaps, airspeed) is enabled.
 * Set this value within the vehicle's performance limits.
 *
 * @unit deg
 * @min 1.0
 * @max 45.0
 * @decimal 1
 * @increment 0.5
 * @group FW Auto Landing
 */
PARAM_DEFINE_FLOAT(FW_LND_ANG, 5.0f);

/**
 * Minimum pitch during takeoff.
 *
 * @unit deg
 * @min -5.0
 * @max 80.0
 * @decimal 1
 * @increment 0.5
 * @group FW Auto Takeoff
 */
PARAM_DEFINE_FLOAT(FW_TKO_PITCH_MIN, 10.0f);

/**
 * Takeoff Airspeed
 *
 * The calibrated airspeed setpoint during the takeoff climbout.
 *
 * If set <= 0, FW_AIRSPD_MIN will be set by default.
 *
 * @unit m/s
 * @min -1.0
 * @decimal 1
 * @increment 0.1
 * @group FW Auto Takeoff
 */
PARAM_DEFINE_FLOAT(FW_TKO_AIRSPD, -1.0f);

/**
 * Landing flare altitude (relative to landing altitude)
 *
 * The flare starts at max(FW_LND_FLALT, FW_LND_FL_TIME * sink rate). With FW_LND_FL_TTD above 0 it starts at
 * FW_LND_FL_TTD * sink rate, at least at FW_LND_FLALT and, with FW_LND_FLALT above 0, at most at twice FW_LND_FLALT.
 *
 * @unit m
 * @min 0.0
 * @decimal 1
 * @increment 0.5
 * @group FW Auto Landing
 */
PARAM_DEFINE_FLOAT(FW_LND_FLALT, 0.5f);

/**
 * Use terrain estimation during landing.
 *
 * This is critical for detecting when to flare, and should be enabled if possible.
 *
 * If enabled and no measurement is found within a given timeout, the landing waypoint altitude will be used OR the landing
 * will be aborted, depending on the criteria set in FW_LND_ABORT.
 *
 * If disabled, FW_LND_ABORT terrain based criteria are ignored.
 *
 * @min 0
 * @max 2
 * @value 0 Disable the terrain estimate
 * @value 1 Use the terrain estimate to trigger the flare (only)
 * @value 2 Calculate landing glide slope relative to the terrain estimate
 * @group FW Auto Landing
 */
PARAM_DEFINE_INT32(FW_LND_USETER, 1);

/**
 * Early landing configuration deployment
 *
 * Allows to deploy the landing configuration (flaps, landing airspeed, etc.) already in
 * the loiter-down waypoint before the final approach.
 * Otherwise is enabled only in the final approach.
 *
 * @boolean
 *
 * @group FW Auto Landing
 */
PARAM_DEFINE_INT32(FW_LND_EARLYCFG, 0);

/**
 * Flare, minimum pitch
 *
 * Minimum pitch during landing flare. Also the pitch floor after a thrust loss (FD_FW_TL_TIME) in Takeoff and, with a
 * valid range to the ground, below twice FW_LND_FLALT outside a landing, and the Takeoff floor above FW_TKO_PMIN_ALT.
 *
 * @unit deg
 * @min -5
 * @max 15.0
 * @decimal 1
 * @increment 0.5
 * @group FW Auto Landing
 */
PARAM_DEFINE_FLOAT(FW_LND_FL_PMIN, 2.5f);

/**
 * Flare, maximum pitch
 *
 * Maximum pitch during landing flare. Also both pitch limits of the glide after an aborted launch (FW_LAUN_SPD_MIN)
 * in every auto mode.
 *
 * @unit deg
 * @min 0
 * @max 45.0
 * @decimal 1
 * @increment 0.5
 * @group FW Auto Landing
 */
PARAM_DEFINE_FLOAT(FW_LND_FL_PMAX, 15.0f);

/**
 * Landing airspeed
 *
 * The calibrated airspeed setpoint during landing.
 *
 * If set <= 0, landing airspeed = FW_AIRSPD_MIN by default.
 *
 * @unit m/s
 * @min -1.0
 * @decimal 1
 * @increment 0.1
 * @group FW Auto Landing
 */
PARAM_DEFINE_FLOAT(FW_LND_AIRSPD, -1.f);

/**
 * Altitude time constant factor for landing and low-height flight
 *
 * The TECS altitude time constant (FW_T_ALT_TC) is multiplied by this value.
 *
 * @unit
 * @min 0.2
 * @max 1.0
 * @decimal 1
 * @increment 0.1
 * @group FW Auto Landing
 */
PARAM_DEFINE_FLOAT(FW_LND_THRTC_SC, 1.0f);

/**
 * Speed <--> Altitude weight
 *
 * Adjusts the amount of weighting that the pitch control
 * applies to speed vs height errors.
 * 0 -> control height only
 * 2 -> control speed only (gliders)
 *
 * @min 0.0
 * @max 2.0
 * @decimal 1
 * @increment 1.0
 * @group FW General
 */
PARAM_DEFINE_FLOAT(FW_T_SPDWEIGHT, 1.0f);

/**
 * Custom stick configuration
 *
 * Applies in manual Position and Altitude flight modes.
 *
 * @min 0
 * @max 3
 * @bit 0 Alternative stick configuration (height rate on throttle stick, airspeed on pitch stick)
 * @bit 1 Enable airspeed setpoint via sticks in altitude and position flight mode
 * @group FW General
 */
PARAM_DEFINE_INT32(FW_POS_STK_CONF, 2);

/**
 * Default target climbrate.
 *
 * In auto modes: default climb rate output by controller to achieve altitude setpoints.
 * In manual modes: maximum climb rate setpoint.
 *
 * @unit m/s
 * @min 0.1
 * @decimal 2
 * @increment 0.01
 * @group FW General
 */
PARAM_DEFINE_FLOAT(FW_T_CLMB_R_SP, 3.0f);

/**
 * Default target sinkrate.
 *
 * In auto modes: default sink rate output by controller to achieve altitude setpoints.
 * In manual modes: maximum sink rate setpoint.
 *
 * @unit m/s
 * @min 0.1
 * @decimal 2
 * @increment 0.01
 * @group FW General
 */
PARAM_DEFINE_FLOAT(FW_T_SINK_R_SP, 2.0f);

/**
 * GPS failure loiter time
 *
 * The time the system should do open loop loiter and wait for GPS recovery
 * before it starts descending. Set to 0 to disable. Roll angle is set to FW_GPSF_R.
 * Does only apply for fixed-wing vehicles or VTOLs with NAV_FORCE_VT set to 0.
 *
 * @unit s
 * @min 0
 * @group FW General
 */
PARAM_DEFINE_INT32(FW_GPSF_LT, 30);

/**
 * GPS failure fixed roll angle
 *
 * Roll angle in GPS failure loiter mode.
 *
 * @unit deg
 * @min 0.0
 * @max 60.0
 * @decimal 1
 * @increment 0.5
 * @group FW General
 */
PARAM_DEFINE_FLOAT(FW_GPSF_R, 15.0f);


/**
 * The aircraft's wing span (length from tip to tip).
 *
 * This is used for limiting the roll setpoint near the ground. (if multiple wings, take the longest span)
 *
 * @unit m
 * @min 0.1
 * @decimal 1
 * @increment 0.1
 * @group FW General
 */
PARAM_DEFINE_FLOAT(FW_WING_SPAN, 3.0);

/**
 * Height (AGL) of the wings when the aircraft is on the ground.
 *
 * This is used to constrain a minimum altitude below which we keep wings level to avoid wing tip strike. It's safer
 * to give a slight margin here (> 0m)
 *
 * @unit m
 * @min 0.0
 * @decimal 1
 * @increment 1
 * @group FW General
 */
PARAM_DEFINE_FLOAT(FW_WING_HEIGHT, 0.5);

/**
 * Landing flare time
 *
 * Time over which the flare limits ramp in. With FW_LND_FL_TTD at 0 the flare starts at
 * max(FW_LND_FLALT, FW_LND_FL_TIME * sink rate); FW_LND_FL_TTD above 0 takes its place in the trigger.
 *
 * @unit s
 * @min 0.1
 * @max 5.0
 * @decimal 1
 * @increment 0.1
 * @group FW Auto Landing
 */
PARAM_DEFINE_FLOAT(FW_LND_FL_TIME, 1.0f);

/**
 * Landing touchdown time (since flare start)
 *
 * This is the time after the start of flaring that we expect the vehicle to touch the runway.
 * At this time, a 0.5s clamp down ramp will engage, constraining the pitch setpoint to RWTO_PSP.
 * If enabled, ensure that RWTO_PSP is configured appropriately for full gear contact on ground roll.
 *
 * Set to -1.0 to disable touchdown clamping. E.g. it may not be desirable to clamp on belly landings.
 *
 * The touchdown time will be constrained to be greater than or equal to the flare time (FW_LND_FL_TIME).
 *
 * @unit s
 * @min -1.0
 * @max 5.0
 * @decimal 1
 * @increment 0.1
 * @group FW Auto Landing
 */
PARAM_DEFINE_FLOAT(FW_LND_TD_TIME, -1.0f);

/**
 * Landing flare sink rate
 *
 * TECS will attempt to control the aircraft to this sink rate via pitch angle (throttle killed during flare).
 * With FW_LND_FL_KP above 0 the flare law flies it.
 *
 * @unit m/s
 * @min 0.0
 * @max 2
 * @decimal 2
 * @increment 0.1
 * @group FW Auto Landing
 */
PARAM_DEFINE_FLOAT(FW_LND_FL_SINK, 0.25f);

/**
 * Landing flare pitch gain
 *
 * Flare pitch setpoint per m/s of height rate error, in deg per m/s. On range data with FW_LND_USETER
 * above 0 the height rate setpoint runs from the entry sink at the flare height to FW_LND_FL_SINK on the ground.
 *
 * Set to 0 to leave the flare pitch to TECS.
 *
 * @min 0.0
 * @max 20.0
 * @decimal 1
 * @increment 0.5
 * @group FW Auto Landing
 */
PARAM_DEFINE_FLOAT(FW_LND_FL_KP, 0.0f);

/**
 * Landing flare pitch integral gain
 *
 * Flare pitch rate per m/s of height rate error, in deg/s per m/s. Acts only with FW_LND_FL_KP above 0.
 *
 * @min 0.0
 * @max 10.0
 * @decimal 1
 * @increment 0.5
 * @group FW Auto Landing
 */
PARAM_DEFINE_FLOAT(FW_LND_FL_KI, 0.0f);

/**
 * Landing flare time to touchdown
 *
 * The flare starts at this time to ground at the current sink rate, at least at FW_LND_FLALT and, with FW_LND_FLALT
 * above 0, at most at twice FW_LND_FLALT.
 *
 * Set to 0 to use FW_LND_FL_TIME.
 *
 * @unit s
 * @min 0.0
 * @max 5.0
 * @decimal 1
 * @increment 0.1
 * @group FW Auto Landing
 */
PARAM_DEFINE_FLOAT(FW_LND_FL_TTD, 0.0f);

/**
 * Landing speed weight
 *
 * TECS speed weight on the landing orbit and the landing approach, in place of FW_T_SPDWEIGHT.
 *
 * Set to -1 to use FW_T_SPDWEIGHT.
 *
 * @min -1.0
 * @max 2.0
 * @decimal 1
 * @increment 0.1
 * @group FW Auto Landing
 */
PARAM_DEFINE_FLOAT(FW_LND_SPDWT, -1.0f);

/**
 * Landing airspeed headwind scale
 *
 * Fraction of the headwind on the straight approach added to the landing airspeed, capped at FW_AIRSPD_TRIM.
 *
 * Set to 0 to disable.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.05
 * @group FW Auto Landing
 */
PARAM_DEFINE_FLOAT(FW_LND_WIND_SC, 0.0f);

/**
 * Maximum lateral position offset for the touchdown point
 *
 * @unit m
 * @min 0.0
 * @max 10.0
 * @decimal 1
 * @increment 1
 * @group FW Auto Landing
 */
PARAM_DEFINE_FLOAT(FW_LND_TD_OFF, 3.0);

/**
 * Landing touchdown nudging option.
 *
 * Approach angle nudging: shifts the touchdown point laterally while keeping the approach entrance point constant
 * Approach path nudging: shifts the touchdown point laterally along with the entire approach path
 *
 * This is useful for manually adjusting the landing point in real time when map or GNSS errors cause an offset from the
 * desired landing vector. Nudging is done with yaw stick, constrained to FW_LND_TD_OFF (in meters) and the direction is
 * relative to the vehicle heading (stick deflection to the right = land point moves to the right as seen by the vehicle).
 *
 * @min 0
 * @max 2
 * @value 0 Disable nudging
 * @value 1 Nudge approach angle
 * @value 2 Nudge approach path
 * @group FW Auto Landing
 */
PARAM_DEFINE_INT32(FW_LND_NUDGE, 2);

/**
 * Bit mask to set the automatic landing abort conditions.
 *
 * Terrain estimation:
 * bit 0: Abort if terrain is not found
 * bit 1: Abort if terrain times out (after a first successful measurement)
 *
 * The last estimate is always used as ground, whether the last valid measurement or the land waypoint, depending on the
 * selected abort criteria, until an abort condition is entered. If FW_LND_USETER == 0, these bits are ignored.
 *
 * TODO: Extend automatic abort conditions
 * e.g. glide slope tracking error (horizontal and vertical)
 *
 * @min 0
 * @max 3
 * @bit 0 Abort if terrain is not found (only applies to mission landings)
 * @bit 1 Abort if terrain times out (after a first successful measurement)
 * @group FW Auto Landing
 */
PARAM_DEFINE_INT32(FW_LND_ABORT, 3);

/**
 * Fixed-wing launch detection
 *
 * Enables automatic launch detection based on measured acceleration. Use for hand- or catapult-launched vehicles.
 * Not compatible with runway takeoff.
 *
 * @boolean
 * @group FW Auto Takeoff
 */
PARAM_DEFINE_INT32(FW_LAUN_DETCN_ON, 0);

/**
 * Flaps setting during take-off
 *
 * Sets a fraction of full flaps during take-off.
 * Also applies to flaperons if enabled in the mixer/allocation.
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group FW Auto Takeoff
 */
PARAM_DEFINE_FLOAT(FW_FLAPS_TO_SCL, 0.0f);

/**
 * Flaps setting during landing
 *
 * Sets a fraction of full flaps during landing.
 * Also applies to flaperons if enabled in the mixer/allocation.
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group FW Auto Landing
 */
PARAM_DEFINE_FLOAT(FW_FLAPS_LND_SCL, 1.0f);

/**
 * Spoiler landing setting
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group FW Auto Landing
 */
PARAM_DEFINE_FLOAT(FW_SPOILERS_LND, 0.f);

/**
 * Thrust loss wings-level height
 *
 * Height below which an auto mode flies wings level once the thrust loss check (FD_FW_TL_TIME) has set the motor
 * failure flag. The distance to the active position setpoint, for an orbit or a circular landing to its circle, over
 * FW_TL_GLIDE raises it; a straight landing is exempt. Below FW_TL_LVL_ALT itself, not the raised height, the flag holds
 * until landed.
 * The height is the range to the ground, and the height above the estimator origin without a valid range.
 *
 * @unit m
 * @min 0.0
 * @max 100.0
 * @decimal 1
 * @increment 1.0
 * @group FW General
 */
PARAM_DEFINE_FLOAT(FW_TL_LVL_ALT, 20.f);

/**
 * Thrust loss glide ratio
 *
 * Horizontal distance per metre of height that a thrust loss glide is assumed to reach in still air. Once the motor
 * failure flag is set, the wings-level height (FW_TL_LVL_ALT) rises by the distance to the active position setpoint
 * over this ratio, and an orbit ahead of a landing item ends once the height above the touchdown point is at most the
 * distance to it over this ratio plus 10 m. Wind scales the ratio by ground speed over airspeed, held to 0.5 to 1.2:
 * along the course to the setpoint for the wings-level height, along the landing course for the orbit. With the flag
 * set an auto mode banks at most 30 deg.
 *
 * @min 1.0
 * @max 30.0
 * @decimal 1
 * @increment 0.5
 * @group FW General
 */
PARAM_DEFINE_FLOAT(FW_TL_GLIDE, 8.f);

/**
 * Terrain follow enable
 *
 * Follow bits act only when set in flight; a bit stored at boot waits until it is cleared and set again.
 * Follow and guard need EKF2_RNG_CTRL above 0.
 *
 * @min 0
 * @max 7
 * @bit 0 Follow in Hold
 * @bit 1 Follow in Mission
 * @bit 2 Ground clearance guard in Hold, Mission and Return
 * @group FW Terrain Follow
 */
PARAM_DEFINE_INT32(FW_TF_EN, 0);

/**
 * Terrain follow height above ground
 *
 * Flown between FW_TF_FLOOR + max(FW_TF_SINK x FW_TF_TTG + 1, 7) and 0.75 x the sensor range x cos(FW_TF_BANK),
 * 30.7 m with 50 m lidars. A Hold above 40 m does not follow. The steepest slope held is FW_T_CLMB_MAX over the
 * ground speed: 16.7 percent at 18 m/s with FW_T_CLMB_MAX 3.
 *
 * @unit m
 * @min 15
 * @max 100
 * @decimal 1
 * @increment 1
 * @group FW Terrain Follow
 */
PARAM_DEFINE_FLOAT(FW_TF_HAGL, 25.f);

/**
 * Terrain follow floor
 *
 * Height above ground under which the pull-up starts; the guard lifts under 1.5 times it.
 *
 * @unit m
 * @min 3
 * @max 50
 * @decimal 1
 * @increment 0.5
 * @group FW Terrain Follow
 */
PARAM_DEFINE_FLOAT(FW_TF_FLOOR, 8.f);

/**
 * Terrain follow height error time constant
 *
 * Time constant of the height error in follow and guard uplift.
 *
 * @unit s
 * @min 1
 * @max 10
 * @decimal 1
 * @increment 0.5
 * @group FW Terrain Follow
 */
PARAM_DEFINE_FLOAT(FW_TF_TC, 4.f);

/**
 * Terrain follow maximum descent rate
 *
 * Descent rate limit in follow and guard uplift.
 *
 * @unit m/s
 * @min 0.5
 * @max 5
 * @decimal 1
 * @increment 0.1
 * @group FW Terrain Follow
 */
PARAM_DEFINE_FLOAT(FW_TF_SINK, 2.f);

/**
 * Terrain follow bank limit
 *
 * Bank limit while the follower holds the altitude.
 *
 * @unit deg
 * @min 20
 * @max 40
 * @decimal 0
 * @increment 1
 * @group FW Terrain Follow
 */
PARAM_DEFINE_FLOAT(FW_TF_BANK, 35.f);

/**
 * Terrain follow climb on range loss
 *
 * Climb above the altitude where the range was lost, held until the range returns.
 *
 * @unit m
 * @min 0
 * @max 50
 * @decimal 1
 * @increment 1
 * @group FW Terrain Follow
 */
PARAM_DEFINE_FLOAT(FW_TF_LOSS_UP, 10.f);

/**
 * Terrain follow time to floor that starts the pull-up, 0 disables
 *
 * The pull-up starts once the closing rate on the ground times this time exceeds the height above FW_TF_FLOOR.
 *
 * @unit s
 * @min 0
 * @max 10
 * @decimal 1
 * @increment 0.5
 * @group FW Terrain Follow
 */
PARAM_DEFINE_FLOAT(FW_TF_TTG, 3.f);

/**
 * Terrain follow descent limit under the planned altitude
 *
 * Follow flies at most this far under the planned altitude.
 *
 * @unit m
 * @min 0
 * @max 100
 * @decimal 1
 * @increment 1
 * @group FW Terrain Follow
 */
PARAM_DEFINE_FLOAT(FW_TF_MAXDN, 10.f);
