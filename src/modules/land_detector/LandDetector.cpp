/****************************************************************************
 *
 *   Copyright (c) 2013-2016 PX4 Development Team. All rights reserved.
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

/*
 * @file LandDetector.cpp
 *
 * @author Johan Jansen <jnsn.johan@gmail.com>
 * @author Julian Oes <julian@oes.ch>
 */

#include "LandDetector.h"

#include <cfloat>

#include <px4_config.h>
#include <px4_defines.h>
#include <drivers/drv_hrt.h>
#include "uORB/topics/parameter_update.h"

namespace land_detector
{

LandDetector::LandDetector() :
	_cycle_perf(perf_alloc(PC_ELAPSED, "land_detector_cycle"))
{
}

LandDetector::~LandDetector()
{
	perf_free(_cycle_perf);
}

int LandDetector::start()
{
	/* schedule a cycle to start things */
	return work_queue(HPWORK, &_work, (worker_t)&LandDetector::cycleTrampoline, this, 0);
}

void
LandDetector::cycleTrampoline(void *arg)
{
	LandDetector *dev = reinterpret_cast<LandDetector *>(arg);

	dev->_cycle();
}

void LandDetector::cycle()
{
	perf_begin(_cycle_perf);

	if (object == nullptr) { // not initialized yet
		// Advertise the first land detected uORB.
		_landDetected.timestamp = hrt_absolute_time();
		_landDetected.freefall = false;
		_landDetected.landed = false;
		_landDetected.ground_contact = false;
		_landDetected.maybe_landed = false;

		_p_total_flight_time_high = param_find("LND_FLIGHT_T_HI");
		_p_total_flight_time_low = param_find("LND_FLIGHT_T_LO");

		// Initialize uORB topics.
		_initialize_topics();

		_check_params(true);

		object = this;
	}

	_check_params(false);
	_update_topics();
	_update_state();

	const bool land_detected = (_state == LandDetectionState::LANDED);
	const bool freefall_detected = (_state == LandDetectionState::FREEFALL);
	const bool maybe_landed_detected = (_state == LandDetectionState::MAYBE_LANDED);
	const bool ground_contact_detected = (_state == LandDetectionState::GROUND_CONTACT);
	const float alt_max = _get_max_altitude();

	// Only publish very first time or when the result has changed.
	if ((_landDetectedPub == nullptr) ||
	    (_landDetected.landed != land_detected) ||
	    (_landDetected.freefall != freefall_detected) ||
	    (_landDetected.maybe_landed != maybe_landed_detected) ||
	    (_landDetected.ground_contact != ground_contact_detected) ||
	    (fabsf(_landDetected.alt_max - alt_max) > FLT_EPSILON)) {

		hrt_abstime now = hrt_absolute_time();

		if (!land_detected && _landDetected.landed) {
			// We did take off
			_takeoff_time = now;

		} else if (_takeoff_time != 0 && land_detected && !_landDetected.landed) {
			// We landed
			_total_flight_time += now - _takeoff_time;
			_takeoff_time = 0;
			uint32_t flight_time = (_total_flight_time >> 32) & 0xffffffff;
			param_set_no_notification(_p_total_flight_time_high, &flight_time);
			flight_time = _total_flight_time & 0xffffffff;
			param_set_no_notification(_p_total_flight_time_low, &flight_time);
		}

		_landDetected.timestamp = hrt_absolute_time();
		_landDetected.landed = land_detected;
		_landDetected.freefall = freefall_detected;
		_landDetected.maybe_landed = maybe_landed_detected;
		_landDetected.ground_contact = ground_contact_detected;
		_landDetected.alt_max = alt_max;

		int instance;
		orb_publish_auto(ORB_ID(vehicle_land_detected), &_landDetectedPub, &_landDetected,
				 &instance, ORB_PRIO_DEFAULT);
	}

	perf_end(_cycle_perf);

	if (!should_exit()) {

		// Schedule next cycle.
		work_queue(HPWORK, &_work, (worker_t)&LandDetector::cycleTrampoline, this,
			   USEC2TICK(1000000 / land_detector_update_rate_hz));

	} else {
		exitAndCleanup();
	}
}
void LandDetector::checkParams(const bool force)
{
	bool updated;
	parameter_update_s param_update;

	orb_check(_parameterSub, &updated);

	if (updated) {
		orb_copy(ORB_ID(parameter_update), _parameterSub, &param_update);
	}

	if (updated || force) {
		_update_params();
		uint32_t flight_time;
		param_get(_p_total_flight_time_high, (int32_t *)&flight_time);
		_total_flight_time = ((uint64_t)flight_time) << 32;
		param_get(_p_total_flight_time_low, (int32_t *)&flight_time);
		_total_flight_time |= flight_time;
	}
}

void LandDetector::updateState()
{
	/* when we are landed we also have ground contact for sure but only one output state can be true at a particular time
	 * with higher priority for landed */
	_freefall_hysteresis.set_state_and_update(_get_freefall_state());
	_landed_hysteresis.set_state_and_update(_get_landed_state());
	_maybe_landed_hysteresis.set_state_and_update(_get_maybe_landed_state());
	_ground_contact_hysteresis.set_state_and_update(_get_ground_contact_state());

	if (_freefall_hysteresis.get_state()) {
		_state = LandDetectionState::FREEFALL;

	} else if (_landed_hysteresis.get_state()) {
		_state = LandDetectionState::LANDED;

	} else if (_maybe_landed_hysteresis.get_state()) {
		_state = LandDetectionState::MAYBE_LANDED;

	} else if (_ground_contact_hysteresis.get_state()) {
		_state = LandDetectionState::GROUND_CONTACT;

	} else {
		_state = LandDetectionState::FLYING;
	}
}

bool LandDetector::orbUpdate(const struct orb_metadata *meta, int handle, void *buffer)
{
	bool new_data = false;

	// check if there is new data to grab
	if (orb_check(handle, &new_data) != OK) {
		return false;
	}

	if (!new_data) {
		return false;
	}

	if (orb_copy(meta, handle, buffer) != OK) {
		return false;
	}

	return true;
}


} // namespace land_detector
