/****************************************************************************
 *
 *   Copyright (C) 2014 PX4 Development Team. All rights reserved.
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

/// @file mavlink_ftp_test.h
///	@author Don Gagne <don@thegagnes.com>

#pragma once

#include <unit_test.h>
#include "../mavlink_bridge_header.h"
#include "../mavlink_ftp.h"

class MavlinkFtpTest : public UnitTest
{
public:
	MavlinkFtpTest();
	virtual ~MavlinkFtpTest();

	virtual bool run_tests(void);

	static void receiveMessageHandlerGeneric(const mavlink_file_transfer_protocol_t *ftp_req, void *worker_data);

	/// Worker data for stream handler
	struct burst_info {
		MavlinkFtpTest		*ftp_test_class;
		int			burst_state;
		bool			single_packet_file;
		uint32_t		file_size;
		uint8_t		*file_bytes;
	};

	static void receiveMessageHandlerBurst(const mavlink_file_transfer_protocol_t *ftp_req, void *worker_data);

	static const uint8_t server_system_id = 50;	///< System ID for server
	static const uint8_t server_component_id = 1;	///< Component ID for server
	static const uint8_t server_channel = 0;		///< Channel to send to

	static const uint8_t client_system_id = 1;	///< System ID for client
	static const uint8_t client_component_id = 0;	///< Component ID for client

	// We don't want any of these
	MavlinkFtpTest(const MavlinkFtpTest &);
	MavlinkFtpTest &operator=(const MavlinkFtpTest &);

private:
	virtual void _init(void);
	virtual void _cleanup(void);

	bool ackTest(void);
	bool badOpcodeTest(void);
	bool badDatasizeTest(void);
	bool listTest(void);
	bool listEofTest(void);
	bool openBadfileTest(void);
	bool openTerminateTest(void);
	bool terminateBadsessionTest(void);
	bool readTest(void);
	bool readBadsessionTest(void);
	bool burstTest(void);
	bool removedirectoryTest(void);
	bool createdirectoryTest(void);
	bool removefileTest(void);

	void receiveMessageHandlerGeneric(const mavlink_file_transfer_protocol_t *ftp_req);
	void setupFtpMsg(const MavlinkFTP::payload_header *payload_header, uint8_t size, const uint8_t *data,
			    mavlink_message_t *msg);
	bool decodeMessage(const mavlink_file_transfer_protocol_t *ftp_msg, const MavlinkFTP::payload_header **payload);
	bool sendReceiveMsg(MavlinkFTP::payload_header	*payload_header,
			       uint8_t				size,
			       const uint8_t			*data,
			       const MavlinkFTP::payload_header	**payload_reply);
	void cleanupMicrosd(void);

	/// A single download test case
	struct download_test_case {
		const char	*file;
		const uint16_t	length;
		bool		singlePacketRead;
		bool		exactlyFillPacket;
	};

	/// The set of test cases for download testing
	static const DownloadTestCase rg_download_test_cases[];

	/// States for stream download handler
	enum {
		burst_state_first_ack,
		burst_state_last_ack,
		burst_state_nak_eof,
		burst_state_complete
	};

	bool receiveMessageHandlerBurst(const mavlink_file_transfer_protocol_t *ftp_req, burst_info *burst_info);

	MavlinkFTP	*_ftp_server;
	uint16_t	_expected_seq_number;

	mavlink_file_transfer_protocol_t _reply_msg;

	static const char unittest_microsd_dir[];
	static const char unittest_microsd_file[];
};

bool mavlink_ftp_test(void);
