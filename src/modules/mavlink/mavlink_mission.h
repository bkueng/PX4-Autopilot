/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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
 * @file mavlink_mission.h
 * Implementation of the MAVLink mission protocol.
 * Documentation:
 * - http://qgroundcontrol.org/mavlink/mission_interface
 * - http://qgroundcontrol.org/mavlink/waypoint_protocol
 *
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Julian Oes <julian@px4.io>
 * @author Anton Babushkin <anton@px4.io>
 */

#pragma once

#include <dataman/dataman.h>
#include <uORB/uORB.h>

#include "mavlink_bridge_header.h"
#include "mavlink_rate_limiter.h"

enum MAVLINK_WPM_STATES {
	MAVLINK_WPM_STATE_IDLE = 0,
	MAVLINK_WPM_STATE_SENDLIST,
	MAVLINK_WPM_STATE_GETLIST,
	MAVLINK_WPM_STATE_ENUM_END
};

enum MAVLINK_WPM_CODES {
	MAVLINK_WPM_CODE_OK = 0,
	MAVLINK_WPM_CODE_ERR_WAYPOINT_ACTION_NOT_SUPPORTED,
	MAVLINK_WPM_CODE_ERR_WAYPOINT_FRAME_NOT_SUPPORTED,
	MAVLINK_WPM_CODE_ERR_WAYPOINT_OUT_OF_BOUNDS,
	MAVLINK_WPM_CODE_ERR_WAYPOINT_MAX_NUMBER_EXCEEDED,
	MAVLINK_WPM_CODE_ENUM_END
};

#define MAVLINK_MISSION_PROTOCOL_TIMEOUT_DEFAULT 5000000    ///< Protocol communication action timeout in useconds
#define MAVLINK_MISSION_RETRY_TIMEOUT_DEFAULT 500000        ///< Protocol communication retry timeout in useconds

class Mavlink;

class MavlinkMissionManager
{
public:
	explicit MavlinkMissionManager(Mavlink *mavlink);

	~MavlinkMissionManager();

	/**
	 * Handle sending of messages. Call this regularly at a fixed frequency.
	 * @param t current time
	 */
	void send(const hrt_abstime t);

	void handleMessage(const mavlink_message_t *msg);

	void setVerbose(bool v) { _verbose = v; }

	void checkActiveMission(void);

private:
	enum MAVLINK_WPM_STATES _state;					///< Current state
	enum MAV_MISSION_TYPE _mission_type;				///< mission type of current transmission (only one at a time possible)

	uint64_t		_time_last_recv;
	uint64_t		_time_last_sent;
	uint64_t		_time_last_reached;			///< last time when the vehicle reached a waypoint

	uint32_t		_action_timeout;
	uint32_t		_retry_timeout;

	bool			_int_mode;				///< Use accurate int32 instead of float

	unsigned		_filesystem_errcount;			///< File system error count

	static int		dataman_id;				///< Global Dataman storage ID for active mission
	int			_my_dataman_id;				///< class Dataman storage ID
	static bool		dataman_init;				///< Dataman initialized

	static unsigned		count[3];				///< Count of items in (active) mission for each MAV_MISSION_TYPE
	static int		current_seq;				///< Current item sequence in active mission

	static int		last_reached;				///< Last reached waypoint in active mission (-1 means nothing reached)

	int			_transfer_dataman_id;			///< Dataman storage ID for current transmission
	unsigned		_transfer_count;			///< Items count in current transmission
	unsigned		_transfer_seq;				///< Item sequence in current transmission
	unsigned		_transfer_current_seq;			///< Current item ID for current transmission (-1 means not initialized)
	unsigned		_transfer_partner_sysid;		///< Partner system ID for current transmission
	unsigned		_transfer_partner_compid;		///< Partner component ID for current transmission
	static bool		transfer_in_progress;			///< Global variable checking for current transmission

	int			_offboard_mission_sub;
	int			_mission_result_sub;
	orb_advert_t		_offboard_mission_pub;

	static uint16_t geofence_update_counter;
	bool		_geofence_locked; ///< if true, we currently hold the dm_lock for the geofence (transaction in progress)

	MavlinkRateLimiter	_slow_rate_limiter;

	bool _verbose;

	Mavlink *_mavlink;

	static constexpr unsigned int	filesystem_errcount_notify_limit =
		2;	///< Error count limit before stopping to report FS errors
	static constexpr unsigned	max_count[] = {
		DM_KEY_WAYPOINTS_OFFBOARD_0_MAX,
		DM_KEY_FENCE_POINTS_MAX - 1,
		DM_KEY_SAFE_POINTS_MAX - 1
	};	/**< Maximum number of mission items for each type
					(fence & save points use the first item for the stats) */

	/** get the maximum number of item count for the current _mission_type */
	inline unsigned currentMaxItemCount();

	/** get the number of item count for the current _mission_type */
	inline unsigned currentItemCount();

	/* do not allow top copying this class */
	MavlinkMissionManager(MavlinkMissionManager &);
	MavlinkMissionManager &operator = (const MavlinkMissionManager &);

	void initOffboardMission();

	int updateActiveMission(int dataman_id, unsigned count, int seq);

	/** store the geofence count to dataman */
	int updateGeofenceCount(unsigned count);

	/** store the safepoint count to dataman */
	int updateSafepointCount(unsigned count);

	/** load geofence stats from dataman */
	int loadGeofenceStats();

	/** load safe point stats from dataman */
	int loadSafepointStats();

	/**
	 *  @brief Sends an waypoint ack message
	 */
	void sendMissionAck(uint8_t sysid, uint8_t compid, uint8_t type);

	/**
	 *  @brief Broadcasts the new target waypoint and directs the MAV to fly there
	 *
	 *  This function broadcasts its new active waypoint sequence number and
	 *  sends a message to the controller, advising it to fly to the coordinates
	 *  of the waypoint with a given orientation
	 *
	 *  @param seq The waypoint sequence number the MAV should fly to.
	 */
	void sendMissionCurrent(uint16_t seq);

	void sendMissionCount(uint8_t sysid, uint8_t compid, uint16_t count, MAV_MISSION_TYPE mission_type);

	void sendMissionItem(uint8_t sysid, uint8_t compid, uint16_t seq);

	void sendMissionRequest(uint8_t sysid, uint8_t compid, uint16_t seq);

	/**
	 *  @brief emits a message that a waypoint reached
	 *
	 *  This function broadcasts a message that a waypoint is reached.
	 *
	 *  @param seq The waypoint sequence number the MAV has reached.
	 */
	void sendMissionItemReached(uint16_t seq);

	void handleMissionAck(const mavlink_message_t *msg);

	void handleMissionSetCurrent(const mavlink_message_t *msg);

	void handleMissionRequestList(const mavlink_message_t *msg);

	void handleMissionRequest(const mavlink_message_t *msg);
	void handleMissionRequestInt(const mavlink_message_t *msg);
	void handleMissionRequestBoth(const mavlink_message_t *msg);

	void handleMissionCount(const mavlink_message_t *msg);

	void handleMissionItem(const mavlink_message_t *msg);
	void handleMissionItemInt(const mavlink_message_t *msg);
	void handleMissionItemBoth(const mavlink_message_t *msg);

	void handleMissionClearAll(const mavlink_message_t *msg);

	/**
	 * Parse mavlink MISSION_ITEM message to get mission_item_s.
	 *
	 * @param mavlink_mission_item pointer to mavlink_mission_item_t or mavlink_mission_item_int_t
	 *			       depending on _int_mode
	 * @param mission_item	       pointer to mission_item to construct
	 */
	int parseMavlinkMissionItem(const mavlink_mission_item_t *mavlink_mission_item, struct mission_item_s *mission_item);

	/**
	 * Format mission_item_s as mavlink MISSION_ITEM(_INT) message.
	 *
	 * @param mission_item:		pointer to the existing mission item
	 * @param mavlink_mission_item: pointer to mavlink_mission_item_t or mavlink_mission_item_int_t
	 *				depending on _int_mode.
	 */
	int formatMavlinkMissionItem(const struct mission_item_s *mission_item,
					mavlink_mission_item_t *mavlink_mission_item);

	/**
	 * set _state to idle (and do necessary cleanup)
	 */
	void switchToIdleState();
};
