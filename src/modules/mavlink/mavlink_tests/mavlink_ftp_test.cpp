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

/// @file mavlink_ftp_test.cpp
///	@author Don Gagne <don@thegagnes.com>

#include <sys/stat.h>
#include <crc32.h>
#include <stdio.h>
#include <fcntl.h>

#include "mavlink_ftp_test.h"
#include "../mavlink_ftp.h"

#ifdef __PX4_NUTTX
#define PX4_MAVLINK_TEST_DATA_DIR "/etc"
#else
#define PX4_MAVLINK_TEST_DATA_DIR "ROMFS/px4fmu_test"
#endif

/// @brief Test case file name for Read command. File are generated using mavlink_ftp_test_data.py
const MavlinkFtpTest::download_test_case MavlinkFtpTest::rg_download_test_cases[] = {
	{ PX4_MAVLINK_TEST_DATA_DIR  "/unit_test_data/mavlink_tests/test_238.data",	MAVLINK_MSG_FILE_TRANSFER_PROTOCOL_FIELD_PAYLOAD_LEN - sizeof(MavlinkFTP::payload_header) - 1,	true, false },	// Read takes less than single packet
	{ PX4_MAVLINK_TEST_DATA_DIR  "/unit_test_data/mavlink_tests/test_239.data",	MAVLINK_MSG_FILE_TRANSFER_PROTOCOL_FIELD_PAYLOAD_LEN - sizeof(MavlinkFTP::payload_header),	true, true },	// Read completely fills single packet
	{ PX4_MAVLINK_TEST_DATA_DIR  "/unit_test_data/mavlink_tests/test_240.data",	MAVLINK_MSG_FILE_TRANSFER_PROTOCOL_FIELD_PAYLOAD_LEN - sizeof(MavlinkFTP::payload_header) + 1,	false, false },	// Read take two packets
};

const char MavlinkFtpTest::unittest_microsd_dir[] = PX4_ROOTFSDIR "/fs/microsd/ftp_unit_test_dir";
const char MavlinkFtpTest::unittest_microsd_file[] = PX4_ROOTFSDIR "/fs/microsd/ftp_unit_test_dir/file";

MavlinkFtpTest::MavlinkFtpTest() :
	_ftp_server(nullptr),
	_expected_seq_number(0),
	_reply_msg{}
{
}

MavlinkFtpTest::~MavlinkFtpTest()
{

}

/// @brief Called before every test to initialize the FTP Server.
void MavlinkFtpTest::_init()
{
	_expected_seq_number = 0;
	_ftp_server = new MavlinkFTP(nullptr);
	_ftp_server->set_unittest_worker(MavlinkFtpTest::receiveMessageHandlerGeneric, this);

	_cleanup_microsd();
}

/// @brief Called after every test to take down the FTP Server.
void MavlinkFtpTest::_cleanup()
{
	delete _ftp_server;

	_cleanup_microsd();
}

/// @brief Tests for correct behavior of an Ack response.
bool MavlinkFtpTest::ackTest()
{
	MavlinkFTP::payload_header		payload;
	const MavlinkFTP::payload_header		*reply;

	payload.opcode = MavlinkFTP::kCmdNone;

	bool success = _send_receive_msg(&payload,	// FTP payload header
					 0,		// size in bytes of data
					 nullptr,	// Data to start into FTP message payload
					 &reply);	// Payload inside FTP message response

	if (!success) {
		return false;
	}

	ut_compare("Didn't get Ack back", reply->opcode, MavlinkFTP::kRspAck);
	ut_compare("Incorrect payload size", reply->size, 0);

	return true;
}

/// @brief Tests for correct response to an invalid opcpde.
bool MavlinkFtpTest::badOpcodeTest()
{
	MavlinkFTP::payload_header		payload;
	const MavlinkFTP::payload_header		*reply;

	payload.opcode = 0xFF;	// bogus opcode

	bool success = _send_receive_msg(&payload,	// FTP payload header
					 0,		// size in bytes of data
					 nullptr,	// Data to start into FTP message payload
					 &reply);	// Payload inside FTP message response

	if (!success) {
		return false;
	}

	ut_compare("Didn't get Nak back", reply->opcode, MavlinkFTP::kRspNak);
	ut_compare("Incorrect payload size", reply->size, 1);
	ut_compare("Incorrect error code", reply->data[0], MavlinkFTP::kErrUnknownCommand);

	return true;
}

/// @brief Tests for correct reponse to a payload which an invalid data size field.
bool MavlinkFtpTest::badDatasizeTest()
{
	mavlink_message_t			msg;
	MavlinkFTP::payload_header		payload;
	const MavlinkFTP::payload_header		*reply;

	payload.opcode = MavlinkFTP::kCmdListDirectory;

	_setup_ftp_msg(&payload, 0, nullptr, &msg);

	// Set the data size to be one larger than is legal
	((MavlinkFTP::payload_header *)((mavlink_file_transfer_protocol_t *)msg.payload64)->payload)->size =
		MAVLINK_MSG_FILE_TRANSFER_PROTOCOL_FIELD_PAYLOAD_LEN + 1;

	_ftp_server->handle_message(&msg);

	if (!_decode_message(&_reply_msg, &reply)) {
		return false;
	}

	ut_compare("Didn't get Nak back", reply->opcode, MavlinkFTP::kRspNak);
	ut_compare("Incorrect payload size", reply->size, 1);
	ut_compare("Incorrect error code", reply->data[0], MavlinkFTP::kErrInvalidDataSize);

	return true;
}

bool MavlinkFtpTest::listTest()
{
	MavlinkFTP::payload_header		payload;
	const MavlinkFTP::payload_header		*reply;

	char response1[] = "Dempty_dir|Ftest_238.data\t238|Ftest_239.data\t239|Ftest_240.data\t240";
#ifdef __PX4_NUTTX
	// expected directory layout only valid on nuttx
	char response2[] = "Ddev|Detc|Dfs|Dobj";
#endif /* __PX4_NUTTX */

	struct test_case {
		const char	*dir;		///< Directory to run List command on
		char		*response;	///< Expected response entries from List command
		int		response_count;	///< Number of directories that should be returned
		bool		success;	///< true: List command should succeed, false: List command should fail
	};
	struct test_case rg_test_cases[] = {
		{ "/bogus",				nullptr,	0,	false },
		{ PX4_MAVLINK_TEST_DATA_DIR  "/unit_test_data/mavlink_tests",	response1,	4,	true },
#ifdef __PX4_NUTTX
		// expected directory layout only valid on nuttx
		{ "/",					response2,	4,	true },
#endif /* __PX4_NUTTX */
	};

	for (size_t i = 0; i < sizeof(rg_test_cases) / sizeof(rg_test_cases[0]); i++) {
		const struct test_case *test = &rg_test_cases[i];

		payload.opcode = MavlinkFTP::kCmdListDirectory;
		payload.offset = 0;

		bool success = _send_receive_msg(&payload,		// FTP payload header
						 strlen(test->dir) + 1,	// size in bytes of data
						 (uint8_t *)test->dir,	// Data to start into FTP message payload
						 &reply);		// Payload inside FTP message response

		if (!success) {
			return false;
		}

		if (test->success) {
			ut_compare("Didn't get Ack back", reply->opcode, MavlinkFTP::kRspAck);
			ut_compare("Incorrect payload size", reply->size, strlen(test->response) + 1);

			// The return order of directories from the List command is not repeatable. So we can't do a direct comparison
			// to a hardcoded return result string.

			// Convert null terminators to seperator char so we can use strok to parse returned data
			char list_entry[256];

			for (uint8_t j = 0; j < reply->size - 1; j++) {
				if (reply->data[j] == 0) {
					list_entry[j] = '|';

				} else {
					list_entry[j] = reply->data[j];
				}
			}

			list_entry[reply->size - 1] = 0;

			// Loop over returned directory entries trying to find then in the response list
			char *dir;
			int response_count = 0;
			dir = strtok(list_entry, "|");

			while (dir != nullptr) {
				ut_assert("Returned directory not found in expected response", strstr(test->response, dir));
				response_count++;
				dir = strtok(nullptr, "|");
			}

			ut_compare("Incorrect number of directory entires returned", test->response_count, response_count);

		} else {
			ut_compare("Didn't get Nak back", reply->opcode, MavlinkFTP::kRspNak);
			ut_compare("Incorrect payload size", reply->size, 2);
			ut_compare("Incorrect error code", reply->data[0], MavlinkFTP::kErrFailErrno);
		}
	}

	return true;
}

/// @brief Tests for correct response to a List command on a valid directory, but with an offset that
/// is beyond the last directory entry.
bool MavlinkFtpTest::listEofTest()
{
	MavlinkFTP::payload_header		payload;
	const MavlinkFTP::payload_header		*reply;
	const char				*dir = "/";

	payload.opcode = MavlinkFTP::kCmdListDirectory;
	payload.offset = 4;	// offset past top level dirs

	bool success = _send_receive_msg(&payload,	// FTP payload header
					 strlen(dir) + 1,	// size in bytes of data
					 (uint8_t *)dir,	// Data to start into FTP message payload
					 &reply);	// Payload inside FTP message response

	if (!success) {
		return false;
	}

	ut_compare("Didn't get Nak back", reply->opcode, MavlinkFTP::kRspNak);
	ut_compare("Incorrect payload size", reply->size, 1);
	ut_compare("Incorrect error code", reply->data[0], MavlinkFTP::kErrEOF);

	return true;
}

/// @brief Tests for correct response to an Open command on a file which does not exist.
bool MavlinkFtpTest::openBadfileTest()
{
	MavlinkFTP::payload_header		payload;
	const MavlinkFTP::payload_header		*reply;
	const char				*dir = "/foo";	// non-existent file

	payload.opcode = MavlinkFTP::kCmdOpenFileRO;
	payload.offset = 0;

	bool success = _send_receive_msg(&payload,	// FTP payload header
					 strlen(dir) + 1,	// size in bytes of data
					 (uint8_t *)dir,	// Data to start into FTP message payload
					 &reply);	// Payload inside FTP message response

	if (!success) {
		return false;
	}

	ut_compare("Didn't get Nak back", reply->opcode, MavlinkFTP::kRspNak);
	ut_compare("Incorrect payload size", reply->size, 2);
	ut_compare("Incorrect error code", reply->data[0], MavlinkFTP::kErrFailErrno);

	return true;
}

/// @brief Tests for correct reponse to an Open command on a file, followed by Terminate
bool MavlinkFtpTest::openTerminateTest()
{
	MavlinkFTP::payload_header		payload;
	const MavlinkFTP::payload_header		*reply;

	for (size_t i = 0; i < sizeof(rg_download_test_cases) / sizeof(rg_download_test_cases[0]); i++) {
		struct stat st;
		const DownloadTestCase *test = &rg_download_test_cases[i];

		payload.opcode = MavlinkFTP::kCmdOpenFileRO;
		payload.offset = 0;

		bool success = _send_receive_msg(&payload,		// FTP payload header
						 strlen(test->file) + 1,	// size in bytes of data
						 (uint8_t *)test->file,	// Data to start into FTP message payload
						 &reply);		// Payload inside FTP message response

		if (!success) {
			return false;
		}

		ut_compare("stat failed", stat(test->file, &st), 0);

		ut_compare("Didn't get Ack back", reply->opcode, MavlinkFTP::kRspAck);
		ut_compare("Incorrect payload size", reply->size, sizeof(uint32_t));
		ut_compare("File size incorrect", *((uint32_t *)&reply->data[0]), (uint32_t)st.st_size);

		payload.opcode = MavlinkFTP::kCmdTerminateSession;
		payload.session = reply->session;
		payload.size = 0;

		success = _send_receive_msg(&payload,	// FTP payload header
					    0,		// size in bytes of data
					    nullptr,	// Data to start into FTP message payload
					    &reply);	// Payload inside FTP message response

		if (!success) {
			return false;
		}

		ut_compare("Didn't get Ack back", reply->opcode, MavlinkFTP::kRspAck);
		ut_compare("Incorrect payload size", reply->size, 0);
	}

	return true;
}

/// @brief Tests for correct reponse to a Terminate command on an invalid session.
bool MavlinkFtpTest::terminateBadsessionTest()
{
	MavlinkFTP::payload_header		payload;
	const MavlinkFTP::payload_header		*reply;
	const char				*file = rg_download_test_cases[0].file;

	payload.opcode = MavlinkFTP::kCmdOpenFileRO;
	payload.offset = 0;

	bool success = _send_receive_msg(&payload,	// FTP payload header
					 strlen(file) + 1,	// size in bytes of data
					 (uint8_t *)file,	// Data to start into FTP message payload
					 &reply);	// Payload inside FTP message response

	if (!success) {
		return false;
	}

	ut_compare("Didn't get Ack back", reply->opcode, MavlinkFTP::kRspAck);

	payload.opcode = MavlinkFTP::kCmdTerminateSession;
	payload.session = reply->session + 1;
	payload.size = 0;

	success = _send_receive_msg(&payload,	// FTP payload header
				    0,		// size in bytes of data
				    nullptr,	// Data to start into FTP message payload
				    &reply);	// Payload inside FTP message response

	if (!success) {
		return false;
	}

	ut_compare("Didn't get Nak back", reply->opcode, MavlinkFTP::kRspNak);
	ut_compare("Incorrect payload size", reply->size, 1);
	ut_compare("Incorrect error code", reply->data[0], MavlinkFTP::kErrInvalidSession);

	return true;
}

/// @brief Tests for correct reponse to a Read command on an open session.
bool MavlinkFtpTest::readTest()
{
	MavlinkFTP::payload_header		payload;
	const MavlinkFTP::payload_header		*reply;

	for (size_t i = 0; i < sizeof(rg_download_test_cases) / sizeof(rg_download_test_cases[0]); i++) {
		struct stat st;
		const DownloadTestCase *test = &rg_download_test_cases[i];

		// Read in the file so we can compare it to what we get back
		ut_compare("stat failed", stat(test->file, &st), 0);
		uint8_t *bytes = new uint8_t[st.st_size];
		ut_assert("new failed", bytes != nullptr);
		int fd = ::open(test->file, O_RDONLY);
		ut_assert("open failed", fd != -1);
		int bytes_read = ::read(fd, bytes, st.st_size);
		ut_compare("read failed", bytes_read, st.st_size);
		::close(fd);

		// Test case data files are created for specific boundary conditions
		ut_compare("Test case data files are out of date", test->length, st.st_size);

		payload.opcode = MavlinkFTP::kCmdOpenFileRO;
		payload.offset = 0;

		bool success = _send_receive_msg(&payload,		// FTP payload header
						 strlen(test->file) + 1,	// size in bytes of data
						 (uint8_t *)test->file,	// Data to start into FTP message payload
						 &reply);		// Payload inside FTP message response

		if (!success) {
			return false;
		}

		ut_compare("Didn't get Ack back", reply->opcode, MavlinkFTP::kRspAck);

		payload.opcode = MavlinkFTP::kCmdReadFile;
		payload.session = reply->session;
		payload.offset = 0;

		success = _send_receive_msg(&payload,	// FTP payload header
					    0,		// size in bytes of data
					    nullptr,	// Data to start into FTP message payload
					    &reply);	// Payload inside FTP message response

		if (!success) {
			return false;
		}

		ut_compare("Didn't get Ack back", reply->opcode, MavlinkFTP::kRspAck);
		ut_compare("Offset incorrect", reply->offset, 0);

		uint32_t full_packet_bytes = MAVLINK_MSG_FILE_TRANSFER_PROTOCOL_FIELD_PAYLOAD_LEN - sizeof(MavlinkFTP::payload_header);
		uint32_t expected_bytes = test->singlePacketRead ? (uint32_t)st.st_size : full_packet_bytes;
		ut_compare("Payload size incorrect", reply->size, expected_bytes);
		ut_compare("File contents differ", memcmp(reply->data, bytes, expected_bytes), 0);

		payload.offset += expected_bytes;

		if (test->singlePacketRead) {
			// Try going past EOF
			success = _send_receive_msg(&payload,	// FTP payload header
						    0,		// size in bytes of data
						    nullptr,	// Data to start into FTP message payload
						    &reply);	// Payload inside FTP message response

			if (!success) {
				return false;
			}

			ut_compare("Didn't get Nak back", reply->opcode, MavlinkFTP::kRspNak);

		} else {
			success = _send_receive_msg(&payload,	// FTP payload header
						    0,		// size in bytes of data
						    nullptr,	// Data to start into FTP message payload
						    &reply);	// Payload inside FTP message response

			if (!success) {
				return false;
			}

			ut_compare("Didn't get Ack back", reply->opcode, MavlinkFTP::kRspAck);
			ut_compare("Offset incorrect", reply->offset, payload.offset);

			expected_bytes = (uint32_t)st.st_size - full_packet_bytes;
			ut_compare("Payload size incorrect", reply->size, expected_bytes);
			ut_compare("File contents differ", memcmp(reply->data, &bytes[payload.offset], expected_bytes), 0);
		}

		payload.opcode = MavlinkFTP::kCmdTerminateSession;
		payload.session = reply->session;
		payload.size = 0;

		success = _send_receive_msg(&payload,	// FTP payload header
					    0,		// size in bytes of data
					    nullptr,	// Data to start into FTP message payload
					    &reply);	// Payload inside FTP message response

		if (!success) {
			return false;
		}

		ut_compare("Didn't get Ack back", reply->opcode, MavlinkFTP::kRspAck);
		ut_compare("Incorrect payload size", reply->size, 0);
	}

	return true;
}

/// @brief Tests for correct reponse to a Read command on an open session.
bool MavlinkFtpTest::burstTest()
{
	MavlinkFTP::payload_header		payload;
	const MavlinkFTP::payload_header		*reply;
	burst_info				burst_info;



	for (size_t i = 0; i < sizeof(rg_download_test_cases) / sizeof(rg_download_test_cases[0]); i++) {
		struct stat st;
		const DownloadTestCase *test = &rg_download_test_cases[i];

		// Read in the file so we can compare it to what we get back
		ut_compare("stat failed", stat(test->file, &st), 0);
		uint8_t *bytes = new uint8_t[st.st_size];
		ut_assert("new failed", bytes != nullptr);
		int fd = ::open(test->file, O_RDONLY);
		ut_assert("open failed", fd != -1);
		int bytes_read = ::read(fd, bytes, st.st_size);
		ut_compare("read failed", bytes_read, st.st_size);
		::close(fd);

		// Test case data files are created for specific boundary conditions
		ut_compare("Test case data files are out of date", test->length, st.st_size);

		payload.opcode = MavlinkFTP::kCmdOpenFileRO;
		payload.offset = 0;

		bool success = _send_receive_msg(&payload,		// FTP payload header
						 strlen(test->file) + 1,	// size in bytes of data
						 (uint8_t *)test->file,	// Data to start into FTP message payload
						 &reply);		// Payload inside FTP message response

		if (!success) {
			return false;
		}

		ut_compare("Didn't get Ack back", reply->opcode, MavlinkFTP::kRspAck);

		// Setup for burst response handler
		burst_info.burst_state = burst_state_first_ack;
		burst_info.single_packet_file = test->singlePacketRead;
		burst_info.file_size = st.st_size;
		burst_info.file_bytes = bytes;
		burst_info.ftp_test_class = this;
		_ftp_server->set_unittest_worker(MavlinkFtpTest::receiveMessageHandlerBurst, &burst_info);

		// Send the burst command, message response will be handled by _receive_message_handler_stream
		payload.opcode = MavlinkFTP::kCmdBurstReadFile;
		payload.session = reply->session;
		payload.offset = 0;

		mavlink_message_t msg;
		_setup_ftp_msg(&payload, 0, nullptr, &msg);
		_ftp_server->handle_message(&msg);

		// First packet is sent using stream mechanism, so we need to force it out ourselves
		hrt_abstime t = 0;
		_ftp_server->send(t);

		ut_compare("Incorrect sequence of messages", burst_info.burst_state, burst_state_complete);

		// Put back generic message handler
		_ftp_server->set_unittest_worker(MavlinkFtpTest::receiveMessageHandlerGeneric, this);

		// Terminate session
		payload.opcode = MavlinkFTP::kCmdTerminateSession;
		payload.session = reply->session;
		payload.size = 0;

		success = _send_receive_msg(&payload,	// FTP payload header
					    0,		// size in bytes of data
					    nullptr,	// Data to start into FTP message payload
					    &reply);	// Payload inside FTP message response

		if (!success) {
			return false;
		}

		ut_compare("Didn't get Ack back", reply->opcode, MavlinkFTP::kRspAck);
		ut_compare("Incorrect payload size", reply->size, 0);
	}

	return true;
}

/// @brief Tests for correct reponse to a Read command on an invalid session.
bool MavlinkFtpTest::readBadsessionTest()
{
	MavlinkFTP::payload_header		payload;
	const MavlinkFTP::payload_header		*reply;
	const char				*file = rg_download_test_cases[0].file;

	payload.opcode = MavlinkFTP::kCmdOpenFileRO;
	payload.offset = 0;

	bool success = _send_receive_msg(&payload,		// FTP payload header
					 strlen(file) + 1,	// size in bytes of data
					 (uint8_t *)file,	// Data to start into FTP message payload
					 &reply);		// Payload inside FTP message response

	if (!success) {
		return false;
	}

	ut_compare("Didn't get Ack back", reply->opcode, MavlinkFTP::kRspAck);

	payload.opcode = MavlinkFTP::kCmdReadFile;
	payload.session = reply->session + 1;	// Invalid session
	payload.offset = 0;

	success = _send_receive_msg(&payload,	// FTP payload header
				    0,		// size in bytes of data
				    nullptr,	// Data to start into FTP message payload
				    &reply);	// Payload inside FTP message response

	if (!success) {
		return false;
	}

	ut_compare("Didn't get Nak back", reply->opcode, MavlinkFTP::kRspNak);
	ut_compare("Incorrect payload size", reply->size, 1);
	ut_compare("Incorrect error code", reply->data[0], MavlinkFTP::kErrInvalidSession);

	return true;
}

bool MavlinkFtpTest::removedirectoryTest()
{
	MavlinkFTP::payload_header		payload;
	const MavlinkFTP::payload_header		*reply;
	int					fd;

	struct test_case {
		const char	*dir;
		bool		success;
		bool		deleteFile;
	};
	static const struct test_case rg_test_cases[] = {
		{ "/bogus",						false,	false },
		{ PX4_MAVLINK_TEST_DATA_DIR "/unit_test_data/mavlink_tests/empty_dir",	false,	false },
		{ unittest_microsd_dir,				false,	false },
		{ unittest_microsd_file,				false,	false },
		{ unittest_microsd_dir,				true,	true },
	};

	ut_compare("mkdir failed", ::mkdir(unittest_microsd_dir, S_IRWXU | S_IRWXG | S_IRWXO), 0);
	ut_assert("open failed", (fd = ::open(unittest_microsd_file, O_CREAT | O_EXCL, S_IRWXU | S_IRWXG | S_IRWXO)) != -1);
	::close(fd);

	for (size_t i = 0; i < sizeof(rg_test_cases) / sizeof(rg_test_cases[0]); i++) {
		const struct test_case *test = &rg_test_cases[i];

		if (test->deleteFile) {
			ut_compare("unlink failed", ::unlink(unittest_microsd_file), 0);
		}

		payload.opcode = MavlinkFTP::kCmdRemoveDirectory;
		payload.offset = 0;

		bool success = _send_receive_msg(&payload,		// FTP payload header
						 strlen(test->dir) + 1,	// size in bytes of data
						 (uint8_t *)test->dir,	// Data to start into FTP message payload
						 &reply);		// Payload inside FTP message response

		if (!success) {
			return false;
		}

		if (test->success) {
			ut_compare("Didn't get Ack back", reply->opcode, MavlinkFTP::kRspAck);
			ut_compare("Incorrect payload size", reply->size, 0);

		} else {
			ut_compare("Didn't get Nak back", reply->opcode, MavlinkFTP::kRspNak);
			ut_compare("Incorrect payload size", reply->size, 2);
			ut_compare("Incorrect error code", reply->data[0], MavlinkFTP::kErrFailErrno);
		}
	}

	return true;
}

bool MavlinkFtpTest::createdirectoryTest()
{
	MavlinkFTP::payload_header		payload;
	const MavlinkFTP::payload_header		*reply;

	struct test_case {
		const char	*dir;
		bool		success;
		bool		fail_exists;
	};
	static const struct test_case rg_test_cases[] = {
		{ "/etc/bogus",			false, false },
		{ unittest_microsd_dir,	true, false },
		{ unittest_microsd_dir,	false, true },
		{ "/fs/microsd/bogus/bogus",	false, false },
	};

	for (size_t i = 0; i < sizeof(rg_test_cases) / sizeof(rg_test_cases[0]); i++) {
		const struct test_case *test = &rg_test_cases[i];

		payload.opcode = MavlinkFTP::kCmdCreateDirectory;
		payload.offset = 0;

		bool success = _send_receive_msg(&payload,		// FTP payload header
						 strlen(test->dir) + 1,	// size in bytes of data
						 (uint8_t *)test->dir,	// Data to start into FTP message payload
						 &reply);		// Payload inside FTP message response

		if (!success) {
			return false;
		}

		if (test->success) {
			ut_compare("Didn't get Ack back", reply->opcode, MavlinkFTP::kRspAck);
			ut_compare("Incorrect payload size", reply->size, 0);

		} else if (test->fail_exists) {
			ut_compare("Didn't get Nak back", reply->opcode, MavlinkFTP::kRspNak);
			ut_compare("Incorrect payload size", reply->size, 1);
			ut_compare("Incorrect error code", reply->data[0], MavlinkFTP::kErrFailFileExists);

		} else {
			ut_compare("Didn't get Nak back", reply->opcode, MavlinkFTP::kRspNak);
			ut_compare("Incorrect payload size", reply->size, 2);
			ut_compare("Incorrect error code", reply->data[0], MavlinkFTP::kErrFailErrno);
		}
	}

	return true;
}

bool MavlinkFtpTest::removefileTest()
{
	MavlinkFTP::payload_header		payload;
	const MavlinkFTP::payload_header		*reply;
	int					fd;

	struct test_case {
		const char	*file;
		bool		success;
	};
	static const struct test_case rg_test_cases[] = {
		{ "/bogus",			false },
#ifdef __PX4_NUTTX
		// file can actually be deleted on linux
		{ _rgDownloadTestCases[0].file,	false },
#endif /* __PX4_NUTTX */
		{ unittest_microsd_dir,	false },
		{ unittest_microsd_file,	true },
		{ unittest_microsd_file,	false },
	};

	ut_compare("mkdir failed", ::mkdir(unittest_microsd_dir, S_IRWXU | S_IRWXG | S_IRWXO), 0);
	ut_assert("open failed", (fd = ::open(unittest_microsd_file, O_CREAT | O_EXCL, S_IRWXU | S_IRWXG | S_IRWXO)) != -1);
	::close(fd);

	for (size_t i = 0; i < sizeof(rg_test_cases) / sizeof(rg_test_cases[0]); i++) {
		const struct test_case *test = &rg_test_cases[i];

		payload.opcode = MavlinkFTP::kCmdRemoveFile;
		payload.offset = 0;

		bool success = _send_receive_msg(&payload,		// FTP payload header
						 strlen(test->file) + 1,	// size in bytes of data
						 (uint8_t *)test->file,	// Data to start into FTP message payload
						 &reply);		// Payload inside FTP message response

		if (!success) {
			return false;
		}

		if (test->success) {
			ut_compare("Didn't get Ack back", reply->opcode, MavlinkFTP::kRspAck);
			ut_compare("Incorrect payload size", reply->size, 0);

		} else {
			ut_compare("Didn't get Nak back", reply->opcode, MavlinkFTP::kRspNak);
			ut_compare("Incorrect payload size", reply->size, 2);
			ut_compare("Incorrect error code", reply->data[0], MavlinkFTP::kErrFailErrno);
		}
	}

	return true;
}

/// Static method used as callback from MavlinkFTP for generic use. This method will be called by MavlinkFTP when
/// it needs to send a message out on Mavlink.
void MavlinkFtpTest::receiveMessageHandlerGeneric(const mavlink_file_transfer_protocol_t *ftp_req, void *worker_data)
{
	((MavlinkFtpTest *)worker_data)->_receive_message_handler_generic(ftp_req);
}

void MavlinkFtpTest::receiveMessageHandlerGeneric(const mavlink_file_transfer_protocol_t *ftp_req)
{
	// Move the message into our own member variable
	memcpy(&_reply_msg, ftp_req, sizeof(mavlink_file_transfer_protocol_t));
}

/// Static method used as callback from MavlinkFTP for stream download testing. This method will be called by MavlinkFTP when
/// it needs to send a message out on Mavlink.
void MavlinkFtpTest::receiveMessageHandlerBurst(const mavlink_file_transfer_protocol_t *ftp_req, void *worker_data)
{
	burst_info *burst_info = (burst_info *)worker_data;
	burst_info->ftp_test_class->_receive_message_handler_burst(ftp_req, burst_info);
}

bool MavlinkFtpTest::receiveMessageHandlerBurst(const mavlink_file_transfer_protocol_t *ftp_msg,
		burst_info *burst_info)
{
	hrt_abstime t = 0;
	const MavlinkFTP::payload_header *reply;
	uint32_t full_packet_bytes = MAVLINK_MSG_FILE_TRANSFER_PROTOCOL_FIELD_PAYLOAD_LEN - sizeof(MavlinkFTP::payload_header);
	uint32_t expected_bytes;

	_decode_message(ftp_msg, &reply);

	switch (burst_info->burst_state) {
	case burst_state_first_ack:
		ut_compare("Didn't get Ack back", reply->opcode, MavlinkFTP::kRspAck);
		ut_compare("Offset incorrect", reply->offset, 0);

		expected_bytes = burst_info->single_packet_file ? burst_info->file_size : full_packet_bytes;
		ut_compare("Payload size incorrect", reply->size, expected_bytes);
		ut_compare("burst_complete incorrect", reply->burst_complete, 0);
		ut_compare("File contents differ", memcmp(reply->data, burst_info->file_bytes, expected_bytes), 0);

		// Setup for next expected message
		burst_info->burst_state = burst_info->single_packet_file ? burst_state_nak_eof : burst_state_last_ack;

		ut_assert("Remaining stream packets missing", _ftp_server->get_size());
		_ftp_server->send(t);
		break;

	case burst_state_last_ack:
		ut_compare("Didn't get Ack back", reply->opcode, MavlinkFTP::kRspAck);
		ut_compare("Offset incorrect", reply->offset, full_packet_bytes);

		expected_bytes = burst_info->file_size - full_packet_bytes;
		ut_compare("Payload size incorrect", reply->size, expected_bytes);
		ut_compare("burst_complete incorrect", reply->burst_complete, 0);
		ut_compare("File contents differ", memcmp(reply->data, &burst_info->file_bytes[full_packet_bytes], expected_bytes), 0);

		// Setup for next expected message
		burst_info->burst_state = burst_state_nak_eof;

		ut_assert("Remaining stream packets missing", _ftp_server->get_size());
		_ftp_server->send(t);
		break;

	case burst_state_nak_eof:
		// Signal complete
		burst_info->burst_state = burst_state_complete;
		ut_compare("All packets should have been seent", _ftp_server->get_size(), 0);
		break;

	}

	return true;
}

/// @brief Decode and validate the incoming message
bool MavlinkFtpTest::decodeMessage(const mavlink_file_transfer_protocol_t	*ftp_msg,	///< Incoming FTP message
				     const MavlinkFTP::payload_header		**payload)	///< Payload inside FTP message response
{
	//warnx("_decode_message");

	// Make sure the targets are correct
	ut_compare("Target network non-zero", ftp_msg->target_network, 0);
	ut_compare("Target system id mismatch", ftp_msg->target_system, client_system_id);
	ut_compare("Target component id mismatch", ftp_msg->target_component, client_component_id);

	*payload = reinterpret_cast<const MavlinkFTP::payload_header *>(ftp_msg->payload);

	// Make sure we have a good sequence number
	ut_compare("Sequence number mismatch", (*payload)->seq_number, _expected_seq_number);
	_expected_seq_number++;

	return true;
}

/// @brief Initializes an FTP message into a mavlink message
void MavlinkFtpTest::setupFtpMsg(const MavlinkFTP::payload_header	*payload_header,	///< FTP payload header
				    uint8_t				size,			///< size in bytes of data
				    const uint8_t			*data,			///< Data to start into FTP message payload
				    mavlink_message_t			*msg)			///< Returned mavlink message
{
	uint8_t payload_bytes[MAVLINK_MSG_FILE_TRANSFER_PROTOCOL_FIELD_PAYLOAD_LEN];
	MavlinkFTP::payload_header *payload = reinterpret_cast<MavlinkFTP::payload_header *>(payload_bytes);

	memcpy(payload, payload_header, sizeof(MavlinkFTP::payload_header));

	payload->seq_number = _expected_seq_number++;
	payload->size = size;

	if (size != 0) {
		memcpy(payload->data, data, size);
	}

	payload->burst_complete = 0;
	payload->padding = 0;

	msg->checksum = 0;
	mavlink_msg_file_transfer_protocol_pack(client_system_id,		// Sender system id
						client_component_id,	// Sender component id
						msg,			// Message to pack payload into
						0,			// Target network
						server_system_id,		// Target system id
						server_component_id,	// Target component id
						payload_bytes);		// Payload to pack into message
}

/// @brief Sends the specified FTP message to the server and returns response
bool MavlinkFtpTest::sendReceiveMsg(MavlinkFTP::payload_header	*payload_header,	///< FTP payload header
				       uint8_t				size,			///< size in bytes of data
				       const uint8_t			*data,			///< Data to start into FTP message payload
				       const MavlinkFTP::payload_header	**payload_reply)	///< Payload inside FTP message response
{
	mavlink_message_t msg;

	_setup_ftp_msg(payload_header, size, data, &msg);
	_ftp_server->handle_message(&msg);
	return _decode_message(&_reply_msg, payload_reply);
}

/// @brief Cleans up an files created on microsd during testing
void MavlinkFtpTest::cleanupMicrosd()
{
	::unlink(unittest_microsd_file);
	::rmdir(unittest_microsd_dir);
}

/// @brief Runs all the unit tests
bool MavlinkFtpTest::run_tests()
{
	ut_run_test(_ack_test);
	ut_run_test(_bad_opcode_test);
	ut_run_test(_bad_datasize_test);

	// TODO FIX: Incorrect payload size - (reply->size:1) (2:2) (../src/modules/mavlink/mavlink_tests/mavlink_ftp_test.cpp:243)
	//ut_run_test(_list_test);

	// TODO FIX: Didn't get Nak back - (reply->opcode:128) (MavlinkFTP::kRspNak:129) (../src/modules/mavlink/mavlink_tests/mavlink_ftp_test.cpp:271)
	//ut_run_test(_list_eof_test);

	ut_run_test(_open_badfile_test);
	ut_run_test(_open_terminate_test);
	ut_run_test(_terminate_badsession_test);
	ut_run_test(_read_test);
	ut_run_test(_read_badsession_test);
	ut_run_test(_burst_test);
	ut_run_test(_removedirectory_test);

	// TODO FIX: Didn't get Nak back - (reply->opcode:128) (MavlinkFTP::kRspNak:129) (../../src/modules/mavlink/mavlink_tests/mavlink_ftp_test.cpp:730)
	//ut_run_test(_createdirectory_test);
	ut_run_test(_removefile_test);

	return (_tests_failed == 0);

}

ut_declare_test(mavlink_ftp_test, MavlinkFtpTest)
