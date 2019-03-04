/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
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
 * @file mavlink_log.h
 * MAVLink text logging.
 *
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Julian Oes <julian@oes.ch>
 */

#pragma once

#include <px4_log.h>
#include <uORB/uORB.h>
#include <uORB/topics/event.h>

/**
 * The maximum string length supported.
 */
#define MAVLINK_LOG_MAXLEN			50

#ifdef __cplusplus
extern "C" {
#endif
__EXPORT void mavlink_vasprintf(int severity, orb_advert_t *mavlink_log_pub, const char *fmt, ...);
#ifdef __cplusplus
}
#endif

#define _MSG_PRIO_DEBUG		7
#define _MSG_PRIO_INFO		6
#define _MSG_PRIO_NOTICE	5
#define _MSG_PRIO_WARNING	4
#define _MSG_PRIO_ERROR		3
#define _MSG_PRIO_CRITICAL	2
#define _MSG_PRIO_ALERT		1
#define _MSG_PRIO_EMERGENCY	0

/*
 * The va_args implementation here is not beautiful, but obviously we run into the same issues
 * the GCC devs saw, and are using their solution:
 *
 * http://gcc.gnu.org/onlinedocs/cpp/Variadic-Macros.html
 */

/**
 * Send a mavlink info message (not printed to console).
 *
 * @param _pub		Pointer to the uORB advert;
 * @param _text		The text to log;
 */
#define mavlink_log_info(_pub, _text, ...)	mavlink_vasprintf(_MSG_PRIO_INFO, _pub, _text, ##__VA_ARGS__);

/**
 * Send a mavlink warning message and print to console.
 *
 * @param _pub		Pointer to the uORB advert;
 * @param _text		The text to log;
 */
#define mavlink_log_warning(_pub, _text, ...) \
	do { \
		mavlink_vasprintf(_MSG_PRIO_WARNING, _pub, _text, ##__VA_ARGS__); \
		PX4_WARN(_text, ##__VA_ARGS__); \
	} while(0);

/**
 * Send a mavlink emergency message and print to console.
 *
 * @param _pub		Pointer to the uORB advert;
 * @param _text		The text to log;
 */
#define mavlink_log_emergency(_pub, _text, ...) \
	do { \
		mavlink_vasprintf(_MSG_PRIO_EMERGENCY, _pub, _text, ##__VA_ARGS__); \
		PX4_ERR(_text, ##__VA_ARGS__); \
	} while(0);

/**
 * Send a mavlink critical message and print to console.
 *
 * @param _pub		Pointer to the uORB advert;
 * @param _text		The text to log;
 */
#define mavlink_log_critical(_pub, _text, ...) \
	do { \
		mavlink_vasprintf(_MSG_PRIO_CRITICAL, _pub, _text, ##__VA_ARGS__); \
		PX4_WARN(_text, ##__VA_ARGS__); \
	} while(0);

/**
 * Send a mavlink emergency message and print to console.
 *
 * @param _pub		Pointer to the uORB advert;
 * @param _text		The text to log;
 */
#define mavlink_and_console_log_info(_pub, _text, ...)			\
	do { \
		mavlink_log_info(_pub, _text, ##__VA_ARGS__); \
		PX4_INFO(_text, ##__VA_ARGS__); \
	} while(0);

struct mavlink_logmessage {
	char text[MAVLINK_LOG_MAXLEN + 1];
	unsigned char severity;
};

struct mavlink_logbuffer {
	unsigned int start;
	unsigned int size;
	int count;
	struct mavlink_logmessage *elems;
};


// events interface
#include <string.h>

// Console event output can be disabled if needed (also removes all strings).
// Helpful for debugging and to ensure the system is not spammed with events.
// it's not very fancy, it does not print arguments
//#define CONSOLE_PRINT_EVENT(log_level, id, name, str) PX4_INFO_RAW("Event %s: %s\n", name, str)
#define CONSOLE_PRINT_EVENT(log_level, id, name, str)

#define PX4_EVENTS_INITIAL_SEQUENCE (65535 - 10) // initialize with a high number so it wraps soon


#ifdef __cplusplus

void px4_publish_event(orb_advert_t *event_pub, event_s &event);

////////// auto-generated part
#define EVENTS_MAX_ARGUMENTS_SIZE 16 // maximum number of bytes for all arguments

enum class EVENT_LOGLEVEL {
	Emergency = 0,
	Alert = 1,
	Critical = 2,
	Error = 3,
	Warning = 4,
	Notice = 5,
	Info = 6,
	Protocol = 7,

	Disabled = 8,
};

// enums
namespace enums
{
enum class GPS_FIX : uint8_t {
	no_fix = 0,
	oned_fix = 1,
	twod_fix = 2,
	threed_fix = 3,
};
} /* namespace enums */

// type-safe methods for each event
static inline void px4_publish_event_first_event(orb_advert_t *event_pub)   // Event ID: 0
{
	event_s event;
	event.id = 0;
	event.log_level_internal = (uint8_t)EVENT_LOGLEVEL::Info;
	event.log_level_external = (uint8_t)EVENT_LOGLEVEL::Info;
	px4_publish_event(event_pub, event);
	CONSOLE_PRINT_EVENT(EVENT_LOGLEVEL::Info, event.id, "FIRST_EVENT", "First test event");
}
static inline void px4_publish_event_test(orb_advert_t *event_pub, float float_argument,
		uint32_t int_argument)   // Event ID: 1
{
	event_s event;
	event.id = 1;
	event.log_level_internal = (uint8_t)EVENT_LOGLEVEL::Warning;
	event.log_level_external = (uint8_t)EVENT_LOGLEVEL::Warning;
	memcpy(event.arguments, &float_argument, sizeof(float));
	memcpy(event.arguments + sizeof(float), &int_argument, sizeof(uint32_t));
	px4_publish_event(event_pub, event);
	CONSOLE_PRINT_EVENT(EVENT_LOGLEVEL::Warning, event.id, "TEST", "Second test event arg2={2}, arg1={1}");
}
static inline void px4_publish_event_gps_fix_test(orb_advert_t *event_pub, enums::GPS_FIX gps_fix)   // Event ID: 2
{
	event_s event;
	event.id = 2;
	event.log_level_internal = (uint8_t)EVENT_LOGLEVEL::Disabled;
	event.log_level_external = (uint8_t)EVENT_LOGLEVEL::Warning;
	memcpy(event.arguments, &gps_fix, sizeof(uint8_t));
	px4_publish_event(event_pub, event);
	CONSOLE_PRINT_EVENT(EVENT_LOGLEVEL::Disabled, event.id, "GPS_FIX_TEST", "GPS: {1}");
}

static inline void px4_publish_event_test_logged(orb_advert_t *event_pub, uint8_t argument)   // Event ID: 3
{
	event_s event;
	event.id = 3;
	event.log_level_internal = (uint8_t)EVENT_LOGLEVEL::Warning;
	event.log_level_external = (uint8_t)EVENT_LOGLEVEL::Disabled;
	memcpy(event.arguments, &argument, sizeof(uint8_t));
	px4_publish_event(event_pub, event);
	CONSOLE_PRINT_EVENT(EVENT_LOGLEVEL::Warning, event.id, "TEST_LOGGED", "This message is only logged");
}
////////// end of auto-generated part

#undef CONSOLE_PRINT_EVENT
#endif


