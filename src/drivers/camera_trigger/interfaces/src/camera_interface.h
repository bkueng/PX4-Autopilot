/**
 * @file camera_interface.h
 */

#pragma once

#include <systemlib/param/param.h>
#include <px4_log.h>

#define arraySize(a) (sizeof((a))/sizeof(((a)[0])))

class CameraInterface
{
public:

	/**
	 * Constructor
	 */
	CameraInterface();

	/**
	 * Destructor.
	 */
	virtual ~CameraInterface();

	/**
	 * trigger the camera
	 * @param enable
	 */
	virtual void trigger(bool enable) {}

	/**
	 * send command to turn the camera on/off
	 * @param enable
	 */
	virtual void sendTogglePower(bool enable) {}

	/**
	 * send command to prevent the camera from sleeping
	 * @param enable
	 */
	virtual void sendKeepAlive(bool enable) {}

	/**
	 * Display info.
	 */
	virtual void info() {}

	/**
	 * Checks if the interface has support for
	 * camera power control
	 * @return true if power control is supported
	 */
	virtual bool hasPowerControl() { return false; }

	/**
	 * Checks if the camera connected to the interface
	 * is turned on.
	 * @return true if camera is on
	 */
	virtual bool isPoweredOn() { return true; }

protected:

	/**
	 * setup the interface
	 */
	virtual void setup() {}

	/**
	 * get the hardware configuration
	 */
	void getPins();

	param_t _p_pin;

	int _pins[6];

};
