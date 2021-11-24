/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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
 * @file ActuatorEffectivenessTiltrotorVTOL.hpp
 *
 * Actuator effectiveness for tiltrotor VTOL
 *
 * @author Julien Lecoeur <julien.lecoeur@gmail.com>
 */

#include "ActuatorEffectivenessTiltrotorVTOL.hpp"

ActuatorEffectivenessTiltrotorVTOL::ActuatorEffectivenessTiltrotorVTOL()
{
	setFlightPhase(FlightPhase::HOVER_FLIGHT);
}
bool
ActuatorEffectivenessTiltrotorVTOL::getEffectivenessMatrix(Configuration &configuration, bool force)
{
	if (!_updated && !force) {
		return false;
	}

	// Trim
	float tilt = 0.0f;

	switch (_flight_phase) {
	case FlightPhase::HOVER_FLIGHT:  {
			tilt = 0.0f;
			break;
		}

	case FlightPhase::FORWARD_FLIGHT: {
			tilt = 1.5f;
			break;
		}

	case FlightPhase::TRANSITION_FF_TO_HF:
	case FlightPhase::TRANSITION_HF_TO_FF: {
			tilt = 1.0f;
			break;
		}
	}

	ActuatorVector &trim = configuration.trim;

	// Trim: half throttle, tilted motors
	trim(0) = 0.5f;
	trim(1) = 0.5f;
	trim(2) = 0.5f;
	trim(3) = 0.5f;
	trim(4) = tilt;
	trim(5) = tilt;
	trim(6) = tilt;
	trim(7) = tilt;

	// Effectiveness
	const float tiltrotor_vtol[NUM_AXES][NUM_ACTUATORS] = {
		{-0.5f * cosf(trim(4)),  0.5f * cosf(trim(5)),  0.5f * cosf(trim(6)), -0.5f * cosf(trim(7)), 0.5f * trim(0) *sinf(trim(4)), -0.5f * trim(1) *sinf(trim(5)), -0.5f * trim(2) *sinf(trim(6)), 0.5f * trim(3) *sinf(trim(7)), -0.5f, 0.5f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
		{ 0.5f * cosf(trim(4)), -0.5f * cosf(trim(5)),  0.5f * cosf(trim(6)), -0.5f * cosf(trim(7)), -0.5f * trim(0) *sinf(trim(4)),  0.5f * trim(1) *sinf(trim(5)), -0.5f * trim(2) *sinf(trim(6)), 0.5f * trim(3) *sinf(trim(7)), 0.f, 0.f, 0.5f, 0.f, 0.f, 0.f, 0.f, 0.f},
		{-0.5f * sinf(trim(4)),  0.5f * sinf(trim(5)),  0.5f * sinf(trim(6)), -0.5f * sinf(trim(7)), -0.5f * trim(0) *cosf(trim(4)), 0.5f * trim(1) *cosf(trim(5)), 0.5f * trim(2) *cosf(trim(6)), -0.5f * trim(3) *cosf(trim(7)), 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
		{ 0.25f * sinf(trim(4)), 0.25f * sinf(trim(5)), 0.25f * sinf(trim(6)), 0.25f * sinf(trim(7)), 0.25f * trim(0) *cosf(trim(4)), 0.25f * trim(1) *cosf(trim(5)), 0.25f * trim(2) *cosf(trim(6)), 0.25f * trim(3) *cosf(trim(7)), 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
		{ 0.f,  0.f,  0.f,  0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
		{-0.25f * cosf(trim(4)), -0.25f * cosf(trim(5)), -0.25f * cosf(trim(6)), -0.25f * cosf(trim(7)), 0.25f * trim(0) *sinf(trim(4)), 0.25f * trim(1) *sinf(trim(5)), 0.25f * trim(2) *sinf(trim(6)), 0.25f * trim(3) *sinf(trim(7)), 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f}
	};
	EffectivenessMatrix &matrix = configuration.effectiveness_matrices[0];
	matrix = EffectivenessMatrix(tiltrotor_vtol);
	configuration.num_actuators[(int)ActuatorType::MOTORS] = 4;
	configuration.num_actuators[(int)ActuatorType::SERVOS] = 6;
	configuration.next_actuator_index[0] = 10;

	// Temporarily disable a few controls (WIP)
	for (size_t j = 4; j < 8; j++) {
		matrix(3, j) = 0.0f;
		matrix(4, j) = 0.0f;
		matrix(5, j) = 0.0f;
	}


	_updated = false;
	return true;
}

void
ActuatorEffectivenessTiltrotorVTOL::setFlightPhase(const FlightPhase &flight_phase)
{
	ActuatorEffectiveness::setFlightPhase(flight_phase);

	_updated = true;
}
