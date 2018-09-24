/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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
 * @file FlightTaskManualPosition.cpp
 */

#include "FlightTaskManualPosition.hpp"
#include <mathlib/mathlib.h>
#include <float.h>

using namespace matrix;

bool FlightTaskManualPosition::updateInitialize()
{
	bool ret = FlightTaskManualAltitude::updateInitialize();
	// require valid position / velocity in xy
	return ret && PX4_ISFINITE(_position(0))
	       && PX4_ISFINITE(_position(1))
	       && PX4_ISFINITE(_velocity(0))
	       && PX4_ISFINITE(_velocity(1));
}

bool FlightTaskManualPosition::activate()
{

	// all requirements from altitude-mode still have to hold
	bool ret = FlightTaskManualAltitude::activate();

	// set task specific constraint
	if (_constraints.speed_xy >= MPC_VEL_MANUAL.get()) {
		_constraints.speed_xy = MPC_VEL_MANUAL.get();
	}

	_position_setpoint(0) = _position(0);
	_position_setpoint(1) = _position(1);
	_velocity_setpoint(0) = _velocity_setpoint(1) = 0.0f;
	_velocity_scale = _constraints.speed_xy;

	// for position-controlled mode, we need a valid position and velocity state
	// in NE-direction
	return ret;
}

#include <mathlib/math/filter/LowPassFilter2p.hpp>
void FlightTaskManualPosition::_scaleSticks()
{
	/* Use same scaling as for FlightTaskManualAltitude */
	FlightTaskManualAltitude::_scaleSticks();

	/* Constrain length of stick inputs to 1 for xy*/
	Vector2f stick_xy(_sticks_expo(0), _sticks_expo(1));

	float mag = math::constrain(stick_xy.length(), 0.0f, 1.0f);

	if (mag > FLT_EPSILON) {
		stick_xy = stick_xy.normalized() * mag;
	}

	// scale the stick inputs
	if (PX4_ISFINITE(_sub_vehicle_local_position->get().vxy_max)) {
		// estimator provides vehicle specific max

		// use the minimum of the estimator and user specified limit
		_velocity_scale = fminf(_constraints.speed_xy, _sub_vehicle_local_position->get().vxy_max);
		// Allow for a minimum of 0.3 m/s for repositioning
		_velocity_scale = fmaxf(_velocity_scale, 0.3f);

	} else if (stick_xy.length() > 0.5f) {
		// raise the limit at a constant rate up to the user specified value

		if (_velocity_scale < _constraints.speed_xy) {
			_velocity_scale += _deltatime * MPC_ACC_HOR_ESTM.get();

		} else {
			_velocity_scale = _constraints.speed_xy;

		}
	}

	// scale velocity to its maximum limits
	Vector2f vel_sp_xy = stick_xy * _velocity_scale;

	/* Rotate setpoint into local frame. */
	_rotateIntoHeadingFrame(vel_sp_xy);
	/*
	changed params:
		Symbols: x = used, + = saved, * = unsaved
		x + COM_OF_LOSS_T [91,168] : 5.0000
		x + COM_RC_IN_MODE [99,176] : 2
		x + COM_RC_LOSS_T [100,177] : 1.0000
		x + MC_AIRMODE [247,542] : 1
		x + MC_PITCHRATE_MAX [253,548] : 1200.0000
		x + MC_PITCHRATE_P [254,549] : 0.2000
		x + MC_PITCH_P [255,550] : 6.0000
		x + MC_ROLLRATE_MAX [261,556] : 1200.0000
		x + MC_ROLLRATE_P [262,557] : 0.2000
		x + MC_ROLL_P [263,558] : 6.0000
		x + MC_YAWRATE_MAX [274,569] : 800.0000
		x + MPC_ACC_HOR [289,600] : 20.0000
		x + MPC_ACC_HOR_MAX [291,602] : 20.0000
		x + MPC_HOLD_DZ [296,607] : 0.0000
		x + MPC_HOLD_MAX_XY [297,608] : 0.0000
		x + MPC_HOLD_MAX_Z [298,609] : 2.0000
		x + MPC_POS_MODE [309,621] : 0
		x + MPC_THR_MIN [312,624] : 0.0000
		x + MPC_TILTMAX_AIR [313,625] : 70.0000
		x + MPC_VEL_MANUAL [318,630] : 20.0000
		x + MPC_XY_VEL_MAX [324,636] : 20.0000
		x + MPC_Z_VEL_I [331,643] : 0.1500
		x + MPC_Z_VEL_MAX_DN [332,644] : 5.0000
		x + MPC_Z_VEL_MAX_UP [333,645] : 5.0000
		x + MPC_Z_VEL_P [334,646] : 0.6000
	*/
	/*
	Vector2f vel_xy(_velocity(0), _velocity(1));
	static Vector2f vel_prev = vel_xy;
	static math::LowPassFilter2p lp_filters[2]{{125, 10}, {125, 10}};
	Vector2f vel_xy_smooth = Vector2f(lp_filters[0].apply(vel_xy(0)), lp_filters[1].apply(vel_xy(1)));
	Vector2f accel_diff = (vel_sp_xy - vel_xy_smooth) / _deltatime;
	Vector2f accel_cur = (vel_xy - vel_prev) / _deltatime;
	Vector2f jerk_diff = (accel_diff - accel_cur) / _deltatime;
	float max_jerk = 30000.f;

	// velocity-dependent max jerk
	float min_jerk_limit_param = 30000.f;
	if (min_jerk_limit_param > 0.001f) {
		max_jerk *= sqrtf(vel_xy_smooth.norm());
	//		max_jerk *= vel_xy_smooth.norm();
		if (max_jerk < min_jerk_limit_param) max_jerk = min_jerk_limit_param;
	}
	// - seems to work basically, velocity-based as well
	// - problems:
	//   - steps in attitude sp (or: attitude still changes very fast on certain events like direction change)
	//   - small twitches in attitude, even with the filter

	//printf("jerk: %.3f\n", (double)jerk_diff.norm());
	if (jerk_diff.norm_squared() > max_jerk * max_jerk && max_jerk > 0.001f) {
		jerk_diff = max_jerk * jerk_diff.normalized();
		vel_sp_xy = (jerk_diff * _deltatime + accel_cur) * _deltatime + vel_xy_smooth;
	}
	vel_prev = vel_xy;
	//*/

	_velocity_setpoint(0) = vel_sp_xy(0);
	_velocity_setpoint(1) = vel_sp_xy(1);
}

void FlightTaskManualPosition::_updateXYlock()
{
	/* If position lock is not active, position setpoint is set to NAN.*/
	const float vel_xy_norm = Vector2f(&_velocity(0)).length();
	const bool apply_brake = Vector2f(&_velocity_setpoint(0)).length() < FLT_EPSILON;
	const bool stopped = (MPC_HOLD_MAX_XY.get() < FLT_EPSILON || vel_xy_norm < MPC_HOLD_MAX_XY.get());

	if (apply_brake && stopped && !PX4_ISFINITE(_position_setpoint(0))) {
		_position_setpoint(0) = _position(0);
		_position_setpoint(1) = _position(1);

	} else if (PX4_ISFINITE(_position_setpoint(0)) && apply_brake) {
		// Position is locked but check if a reset event has happened.
		// We will shift the setpoints.
		if (_sub_vehicle_local_position->get().xy_reset_counter
		    != _reset_counter) {
			_position_setpoint(0) = _position(0);
			_position_setpoint(1) = _position(1);
			_reset_counter = _sub_vehicle_local_position->get().xy_reset_counter;
		}

	} else {
		/* don't lock*/
		_position_setpoint(0) = NAN;
		_position_setpoint(1) = NAN;
	}

	_position_setpoint(0) = NAN;
	_position_setpoint(1) = NAN;
}

void FlightTaskManualPosition::_updateSetpoints()
{
	FlightTaskManualAltitude::_updateSetpoints(); // needed to get yaw and setpoints in z-direction
	_thrust_setpoint *= NAN; // don't require any thrust setpoints
	_updateXYlock(); // check for position lock
}
