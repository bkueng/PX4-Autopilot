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
 * @file ActuatorEffectivenessMultirotor.hpp
 *
 * Actuator effectiveness computed from rotors position and orientation
 *
 * @author Julien Lecoeur <julien.lecoeur@gmail.com>
 */

#include "ActuatorEffectivenessMultirotor.hpp"

#include "ActuatorEffectivenessTilts.hpp"

using namespace matrix;

ActuatorEffectivenessMultirotor::ActuatorEffectivenessMultirotor(ModuleParams *parent, bool tilt_support,
		bool yaw_disabled)
	: ModuleParams(parent), _tilt_support(tilt_support), _yaw_disabled(yaw_disabled)
{
	for (int i = 0; i < NUM_ROTORS_MAX; ++i) {
		char buffer[17];
		snprintf(buffer, sizeof(buffer), "CA_MC_R%u_PX", i);
		_param_handles[i].position_x = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_MC_R%u_PY", i);
		_param_handles[i].position_y = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_MC_R%u_PZ", i);
		_param_handles[i].position_z = param_find(buffer);

		snprintf(buffer, sizeof(buffer), "CA_MC_R%u_AX", i);
		_param_handles[i].axis_x = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_MC_R%u_AY", i);
		_param_handles[i].axis_y = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_MC_R%u_AZ", i);
		_param_handles[i].axis_z = param_find(buffer);

		snprintf(buffer, sizeof(buffer), "CA_MC_R%u_CT", i);
		_param_handles[i].thrust_coef = param_find(buffer);

		if (!_yaw_disabled) {
			snprintf(buffer, sizeof(buffer), "CA_MC_R%u_KM", i);
			_param_handles[i].moment_ratio = param_find(buffer);
		}

		if (_tilt_support) {
			snprintf(buffer, sizeof(buffer), "CA_MC_R%u_TILT", i);
			_param_handles[i].tilt_index = param_find(buffer);
		}
	}

	_count_handle = param_find("CA_MC_R_COUNT");

	updateParams();
}

void ActuatorEffectivenessMultirotor::updateParams()
{
	ModuleParams::updateParams();

	int32_t count = 0;

	if (param_get(_count_handle, &count) != 0) {
		PX4_ERR("param_get failed");
		return;
	}

	_geometry.num_rotors = count;

	for (int i = 0; i < _geometry.num_rotors; ++i) {
		Vector3f &position = _geometry.rotors[i].position;
		param_get(_param_handles[i].position_x, &position(0));
		param_get(_param_handles[i].position_y, &position(1));
		param_get(_param_handles[i].position_z, &position(2));

		Vector3f &axis = _geometry.rotors[i].axis;
		param_get(_param_handles[i].axis_x, &axis(0));
		param_get(_param_handles[i].axis_y, &axis(1));
		param_get(_param_handles[i].axis_z, &axis(2));

		param_get(_param_handles[i].thrust_coef, &_geometry.rotors[i].thrust_coef);

		if (_yaw_disabled) {
			_geometry.rotors[i].moment_ratio = 0.f;

		} else {
			param_get(_param_handles[i].moment_ratio, &_geometry.rotors[i].moment_ratio);
		}

		if (_tilt_support) {
			int32_t tilt_param{0};
			param_get(_param_handles[i].tilt_index, &tilt_param);
			_geometry.rotors[i].tilt_index = tilt_param - 1;

		} else {
			_geometry.rotors[i].tilt_index = -1;
		}
	}
}

bool
ActuatorEffectivenessMultirotor::getEffectivenessMatrix(Configuration &configuration, bool force)
{
	if (_updated || force) {
		_updated = false;

		if (configuration.num_actuators[(int)ActuatorType::SERVOS] > 0) {
			PX4_ERR("Wrong actuator ordering: servos need to be after motors");
			return false;
		}

		int num_actuators = computeEffectivenessMatrix(_geometry,
				    configuration.effectiveness_matrices[configuration.selected_matrix],
				    configuration.num_actuators_matrix[configuration.selected_matrix],
				    configuration.num_actuators[(int)ActuatorType::MOTORS]);
		configuration.actuatorsAdded(ActuatorType::MOTORS, num_actuators);
		return true;
	}

	return false;
}

int
ActuatorEffectivenessMultirotor::computeEffectivenessMatrix(const MultirotorGeometry &geometry,
		EffectivenessMatrix &effectiveness, int actuator_start_index, int configuration_index_offset)
{
	int num_actuators = 0;

	for (int i = 0; i < math::min(NUM_ROTORS_MAX, geometry.num_rotors); i++) {

		if (i + actuator_start_index >= NUM_ACTUATORS || i + configuration_index_offset >= NUM_ROTORS_MAX) {
			break;
		}

		++num_actuators;

		// Get rotor axis
		Vector3f axis = geometry.rotors[i + configuration_index_offset].axis;

		// Normalize axis
		float axis_norm = axis.norm();

		if (axis_norm > FLT_EPSILON) {
			axis /= axis_norm;

		} else {
			// Bad axis definition, ignore this rotor
			continue;
		}

		// Get rotor position
		const Vector3f &position = geometry.rotors[i + configuration_index_offset].position;

		// Get coefficients
		float ct = geometry.rotors[i + configuration_index_offset].thrust_coef;
		float km = geometry.rotors[i + configuration_index_offset].moment_ratio;

		if (fabsf(ct) < FLT_EPSILON) {
			continue;
		}

		// Compute thrust generated by this rotor
		matrix::Vector3f thrust = ct * axis;

		// Compute moment generated by this rotor
		matrix::Vector3f moment = ct * position.cross(axis) - ct * km * axis;

		// Fill corresponding items in effectiveness matrix
		for (size_t j = 0; j < 3; j++) {
			effectiveness(j, i + actuator_start_index) = moment(j);
			effectiveness(j + 3, i + actuator_start_index) = thrust(j);
		}
	}

	return num_actuators;
}

void ActuatorEffectivenessMultirotor::updateAxisFromTilts(const ActuatorEffectivenessTilts &tilts, float tilt_control)
{
	if (!PX4_ISFINITE(tilt_control)) {
		tilt_control = -1.f;
	}

	for (int i = 0; i < _geometry.num_rotors; ++i) {
		int tilt_index = _geometry.rotors[i].tilt_index;

		if (tilt_index == -1 || tilt_index >= tilts.count()) {
			continue;
		}

		const ActuatorEffectivenessTilts::Params &tilt = tilts.config(tilt_index);
		float tilt_angle = math::lerp(tilt.min_angle, tilt.max_angle, (tilt_control + 1.f) / 2.f);
		float tilt_direction = math::radians((float)tilt.tilt_direction);
		_geometry.rotors[i].axis = tiltedAxis(tilt_angle, tilt_direction);
	}

}

Vector3f ActuatorEffectivenessMultirotor::tiltedAxis(float tilt_angle, float tilt_direction)
{
	Vector3f axis{0.f, 0.f, -1.f};
	return Dcmf{Eulerf{0.f, -tilt_angle, tilt_direction}} * axis;
}
