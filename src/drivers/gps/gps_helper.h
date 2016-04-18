/****************************************************************************
 *
 *   Copyright (c) 2012-2014 PX4 Development Team. All rights reserved.
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
 * @file gps_helper.h
 * @author Thomas Gubler <thomasgubler@student.ethz.ch>
 * @author Julian Oes <joes@student.ethz.ch>
 */

#pragma once

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <array>

// TODO: this number seems wrong
#define GPS_EPOCH_SECS 1234567890ULL

class GPSHelper
{
public:
	enum class OutputMode {
		GPS = 0,    ///< normal GPS output
		RTCM        ///< request RTCM output. This is used for (fixed position) base stations
	};

	GPSHelper(int fd, bool support_inject_data = false);
	virtual ~GPSHelper();

	/**
	 * configure the device
	 * @param baud will be set to the baudrate (output parameter)
	 * @return 0 on success, <0 otherwise
	 */
	virtual int configure(unsigned &baud, OutputMode output_mode) = 0;

	/**
	 * receive & handle new data from the device
	 * @param timeout
	 * @return <0 on error, otherwise a bitset:
	 *         bit 0 set: got gps position update
	 *         bit 1 set: got satellite info update
	 */
	virtual int receive(unsigned timeout) = 0;

	/**
	 * set the Baudrate
	 * @param fd
	 * @param baud
	 * @return 0 on success, <0 on error
	 */
	int setBaudrate(const int &fd, unsigned baud);

	float getPositionUpdateRate();
	float getVelocityUpdateRate();
	void resetUpdateRates();
	void storeUpdateRates();

	/**
	 * Start or restart the survey-in procees. This is only used in RTCM ouput mode.
	 * It will be called automatically after configuring.
	 * @return 0 on success, <0 on error
	 */
	virtual int restartSurveyIn() { return 0; }


	/**
	 * This is an abstraction for the poll on serial used.
	 *
	 * @param fd: serial file descriptor
	 * @param buf: pointer to read buffer
	 * @param buf_length: size of read buffer
	 * @param timeout: timeout time in us
	 * @return: 0 for nothing read, or poll timed out
	 *	    < 0 for error
	 *	    > 0 number of bytes read
	 */
	int pollOrRead(int fd, uint8_t *buf, size_t buf_length, uint64_t timeout);

protected:
	int _fd; ///< open file descriptor

	uint8_t _rate_count_lat_lon;
	uint8_t _rate_count_vel;

	float _rate_lat_lon = 0.0f;
	float _rate_vel = 0.0f;

	uint64_t _interval_rate_start;

private:
	/**
	 * check for new messages on the inject data topic & handle them
	 */
	void handleInjectDataTopic();

	/**
	 * send data to the device, such as an RTCM stream
	 * @param data
	 * @param len
	 */
	inline bool injectData(uint8_t *data, size_t len);

	std::array<int, 4> _orb_inject_data_fd;
	int _orb_inject_data_next = 0;
};
