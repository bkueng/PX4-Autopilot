#include "BlockSegwayController.hpp"

using matrix::Eulerf;
using matrix::Quatf;

void BlockSegwayController::update()
{
	// wait for a sensor update, check for exit condition every 100 ms
	if (px4_poll(&_attPoll, 1, 100) < 0) { return; } // poll error

	uint64_t new_time_stamp = hrt_absolute_time();
	float dt = (new_time_stamp - _timeStamp) / 1.0e6f;
	_timeStamp = new_time_stamp;

	// check for sane values of dt
	// to prevent large control responses
	if (dt > 1.0f || dt < 0) { return; }

	// set dt for all child blocks
	setDt(dt);

	// check for new updates
	if (_param_update.updated()) { updateParams(); }

	// get new information from subscriptions
	updateSubscriptions();

	actuator_controls_s &actuators = _actuators.get();

	// default all output to zero unless handled by mode
	for (unsigned i = 2; i < actuator_controls_s::num_actuator_controls; i++) {
		actuators.control[i] = 0.0f;
	}

	const uint8_t &nav_state = _status.get().nav_state;

	// only update guidance in auto mode
	if (nav_state == vehicle_status_s::navigation_state_auto_mission) {
		// update guidance
	}

	Eulerf euler = Eulerf(Quatf(_att.get().q));

	// compute speed command
	float spd_cmd = -th2v.update(euler.theta()) - q2v.update(_att.get().pitchspeed);

	// handle autopilot modes
	if (nav_state == vehicle_status_s::navigation_state_auto_mission ||
	    nav_state == vehicle_status_s::navigation_state_altctl ||
	    nav_state == vehicle_status_s::navigation_state_posctl) {

		actuators.control[0] = spd_cmd;
		actuators.control[1] = spd_cmd;

	} else if (nav_state == vehicle_status_s::navigation_state_stab) {
		actuators.control[0] = spd_cmd;
		actuators.control[1] = spd_cmd;

	} else if (nav_state == vehicle_status_s::navigation_state_manual) {
		actuators.control[CH_LEFT] = _manual.get().z;
		actuators.control[CH_RIGHT] = -_manual.get().x;
	}

	// update all publications
	updatePublications();
}
