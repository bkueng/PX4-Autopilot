/****************************************************************************
 *
 *   Copyright (c) 2014, 2015 PX4 Development Team. All rights reserved.
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

#pragma once

/// @file mavlink_log_handler.h
/// @author px4dev, Gus Grubba <mavlink@grubba.com>

#include <dirent.h>
#include <queue.h>
#include <time.h>
#include <stdio.h>
#include <cstdbool>
#include <v2.0/mavlink_types.h>
#include <drivers/drv_hrt.h>

class Mavlink;

// Log Listing Helper
class LogListHelper
{
public:
	LogListHelper();
	~LogListHelper();

public:
	static void deleteAll(const char *dir);

public:

	bool        getEntry(int idx, uint32_t &size, uint32_t &date, char *filename = 0, int filename_len = 0);
	bool        openForTransmit();
	size_t      getLogData(uint8_t len, uint8_t *buffer);

	enum {
		LOG_HANDLER_IDLE,
		LOG_HANDLER_LISTING,
		LOG_HANDLER_SENDING_DATA
	};

	int         next_entry;
	int         last_entry;
	int         log_count;

	int         current_status;
	uint16_t    current_log_index;
	uint32_t    current_log_size;
	uint32_t    current_log_data_offset;
	uint32_t    current_log_data_remaining;
	FILE       *current_log_filep;
	char        current_log_filename[128];

private:
	void        init();
	bool        getSessionDate(const char *path, const char *dir, time_t &date);
	void        scanLogs(FILE *f, const char *dir, time_t &date);
	bool        getLogTimeSize(const char *path, const char *file, time_t &date, uint32_t &size);
};

// MAVLink LOG_* Message Handler
class MavlinkLogHandler
{
public:
	MavlinkLogHandler(Mavlink *mavlink);

	// Handle possible LOG message
	void handleMessage(const mavlink_message_t *msg);

	/**
	 * Handle sending of messages. Call this regularly at a fixed frequency.
	 * @param t current time
	 */
	void send(const hrt_abstime t);

	unsigned getSize();

private:
	void logMessage(const mavlink_message_t *msg);
	void logRequestList(const mavlink_message_t *msg);
	void logRequestData(const mavlink_message_t *msg);
	void logRequestErase(const mavlink_message_t *msg);
	void logRequestEnd(const mavlink_message_t *msg);

	size_t logSendListing();
	size_t logSendData();

	LogListHelper    *_pLogHandlerHelper;
	Mavlink *_mavlink;
};
