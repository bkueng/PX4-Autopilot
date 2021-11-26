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
 * @file ActuatorEffectivenessStandardVTOL.hpp
 *
 * Actuator effectiveness for standard VTOL
 *
 * @author Julien Lecoeur <julien.lecoeur@gmail.com>
 */

#pragma once

#include "ActuatorEffectiveness.hpp"
#include "ActuatorEffectivenessMultirotor.hpp"
#include "ActuatorEffectivenessControlSurfaces.hpp"

class ActuatorEffectivenessStandardVTOL : public ModuleParams, public ActuatorEffectiveness
{
public:
	ActuatorEffectivenessStandardVTOL(ModuleParams *parent);
	virtual ~ActuatorEffectivenessStandardVTOL() = default;

	bool getEffectivenessMatrix(Configuration &configuration, bool force) override;

	const char *name() const override { return "Standard VTOL"; }

	int numMatrices() const override { return 2; }

	void getDesiredAllocationMethod(AllocationMethod allocation_method_out[MAX_NUM_MATRICES]) const override
	{
		static_assert(MAX_NUM_MATRICES >= 2, "expecting at least 2 matrices");
		allocation_method_out[0] = AllocationMethod::SEQUENTIAL_DESATURATION;
		allocation_method_out[1] = AllocationMethod::PSEUDO_INVERSE;
	}

private:
	ActuatorEffectivenessMultirotor _mc_rotors;
	ActuatorEffectivenessControlSurfaces _control_surfaces;
};
