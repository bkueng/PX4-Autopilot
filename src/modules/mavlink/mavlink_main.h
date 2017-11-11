/****************************************************************************
 *
 *   Copyright (c) 2012-2017 PX4 Development Team. All rights reserved.
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
 * @file mavlink_main.h
 *
 * MAVLink 2.0 protocol interface definition.
 *
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

#pragma once

#include <px4_posix.h>

#include <stdbool.h>
#ifdef __PX4_NUTTX
#include <nuttx/fs/fs.h>
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <drivers/device/device.h>
#endif
#include <systemlib/param/param.h>
#include <systemlib/perf_counter.h>
#include <pthread.h>
#include <systemlib/mavlink_log.h>
#include <drivers/device/ringbuffer.h>

#include <uORB/uORB.h>
#include <uORB/topics/mission.h>
#include <uORB/topics/mission_result.h>
#include <uORB/topics/telemetry_status.h>

#include "mavlink_bridge_header.h"
#include "mavlink_orb_subscription.h"
#include "mavlink_stream.h"
#include "mavlink_messages.h"
#include "mavlink_shell.h"
#include "mavlink_ulog.h"

enum Protocol {
	SERIAL = 0,
	UDP,
	TCP,
};

class Mavlink
{

public:
	/**
	 * Constructor
	 */
	Mavlink();

	/**
	 * Destructor, also kills the mavlinks task.
	 */
	~Mavlink();

	/**
	* Start the mavlink task.
	 *
	 * @return		OK on success.
	 */
	static int		start(int argc, char *argv[]);

	/**
	 * Display the mavlink status.
	 */
	void			displayStatus();

	static int		streamCommand(int argc, char *argv[]);

	static int		instanceCount();

	static Mavlink		*newInstance();

	static Mavlink		*getInstance(unsigned instance);

	static Mavlink 		*getInstanceForDevice(const char *device_name);

	static Mavlink 		*getInstanceForNetworkPort(unsigned long port);

	mavlink_message_t 	*getBuffer() { return &_mavlink_buffer; }

	mavlink_status_t 	*getStatus() { return &_mavlink_status; }

	/**
	 * Set the MAVLink version
	 *
	 * Currently supporting v1 and v2
	 *
	 * @param version MAVLink version
	 */
	void			setProtoVersion(unsigned version);

	static int		destroyAllInstances();

	static int		getStatusAllInstances();

	/**
	 * Set all instances to verbose mode
	 *
	 * This is primarily intended for analysis and
	 * not intended for normal operation
	 */
	static int		setVerboseAllInstances(bool enabled);

	static bool		instanceExists(const char *device_name, Mavlink *self);

	static void		forwardMessage(const mavlink_message_t *msg, Mavlink *self);

	static int		getUartFd(unsigned index);

	int			getUartFd();

	/**
	 * Get the MAVLink system id.
	 *
	 * @return		The system ID of this vehicle
	 */
	int			getSystemId();

	/**
	 * Get the MAVLink component id.
	 *
	 * @return		The component ID of this vehicle
	 */
	int			getComponentId();

	const char *_device_name;

	enum MAVLINK_MODE {
		MAVLINK_MODE_NORMAL = 0,
		MAVLINK_MODE_CUSTOM,
		MAVLINK_MODE_ONBOARD,
		MAVLINK_MODE_OSD,
		MAVLINK_MODE_MAGIC,
		MAVLINK_MODE_CONFIG,
		MAVLINK_MODE_IRIDIUM
	};

	enum BROADCAST_MODE {
		BROADCAST_MODE_OFF = 0,
		BROADCAST_MODE_ON
	};

	static const char *mavlinkModeStr(enum MAVLINK_MODE mode)
	{
		switch (mode) {
		case MAVLINK_MODE_NORMAL:
			return "Normal";

		case MAVLINK_MODE_CUSTOM:
			return "Custom";

		case MAVLINK_MODE_ONBOARD:
			return "Onboard";

		case MAVLINK_MODE_OSD:
			return "OSD";

		case MAVLINK_MODE_MAGIC:
			return "Magic";

		case MAVLINK_MODE_CONFIG:
			return "Config";

		case MAVLINK_MODE_IRIDIUM:
			return "Iridium";

		default:
			return "Unknown";
		}
	}

	void			setMode(enum MAVLINK_MODE);
	enum MAVLINK_MODE	getMode() { return _mode; }

	bool			getHilEnabled() { return _hil_enabled; }

	bool			getUseHilGps() { return _use_hil_gps; }

	bool			getForwardExternalsp() { return _forward_externalsp; }

	bool			getFlowControlEnabled() { return _flow_control_enabled; }

	bool			getForwardingOn() { return _forwarding_on; }

	bool			getConfigLinkOn() { return config_link_on; }

	void			setConfigLinkOn(bool on) { config_link_on = on; }

	bool			isConnected() { return ((_rstatus.heartbeat_time > 0) && (hrt_absolute_time() - _rstatus.heartbeat_time < 3000000)); }

	bool			broadcastEnabled() { return _broadcast_mode > BROADCAST_MODE_OFF; }

	/**
	 * Set the boot complete flag on all instances
	 *
	 * Setting the flag unblocks parameter transmissions, which are gated
	 * beforehand to ensure that the system is fully initialized.
	 */
	static void		setBootComplete();

	/**
	 * Get the free space in the transmit buffer
	 *
	 * @return free space in the UART TX buffer
	 */
	unsigned		getFreeTxBuf();

	static int		startHelper(int argc, char *argv[]);

	/**
	 * Enable / disable Hardware in the Loop simulation mode.
	 *
	 * @param hil_enabled	The new HIL enable/disable state.
	 * @return		OK if the HIL state changed, ERROR if the
	 *			requested change could not be made or was
	 *			redundant.
	 */
	int			setHilEnabled(bool hil_enabled);

	/**
	 * Set manual input generation mode
	 *
	 * Set to true to generate RC_INPUT messages on the system bus from
	 * MAVLink messages.
	 *
	 * @param generation_enabled If set to true, generate RC_INPUT messages
	 */
	void			setManualInputModeGeneration(bool generation_enabled) { _generate_rc = generation_enabled; }

	/**
	 * Set communication protocol for this mavlink instance
	 */
	void 			setProtocol(Protocol p) { _protocol = p; }

	/**
	 * Set verbose mode
	 */
	void			setVerbose(bool v);

	bool			getVerbose() const { return _verbose; }

	/**
	 * Get the manual input generation mode
	 *
	 * @return true if manual inputs should generate RC data
	 */
	bool			getManualInputModeGeneration() { return _generate_rc; }


	/**
	 * This is the beginning of a MAVLINK_START_UART_SEND/MAVLINK_END_UART_SEND transaction
	 */
	void 			beginSend();

	/**
	 * Send bytes out on the link.
	 *
	 * On a network port these might actually get buffered to form a packet.
	 */
	void			sendBytes(const uint8_t *buf, unsigned packet_len);

	/**
	 * Flush the transmit buffer and send one MAVLink packet
	 *
	 * @return the number of bytes sent or -1 in case of error
	 */
	int             	sendPacket();

	/**
	 * Resend message as is, don't change sequence number and CRC.
	 */
	void			resendMessage(mavlink_message_t *msg) { mavlink_resend_uart(_channel, msg); }

	void			handleMessage(const mavlink_message_t *msg);

	MavlinkOrbSubscription *addOrbSubscription(const orb_id_t topic, int instance = 0);

	int			getInstanceId();

#ifndef __PX4_QURT
	/**
	 * Enable / disable hardware flow control.
	 *
	 * @param enabled	True if hardware flow control should be enabled
	 */
	int			enableFlowControl(bool enabled);
#endif

	mavlink_channel_t	getChannel();

	void			configureStreamThreadsafe(const char *stream_name, float rate = -1.0f);

	bool			_task_should_exit;	/**< if true, mavlink task should exit */

	orb_advert_t		*getMavlinkLogPub() { return &_mavlink_log_pub; }

	/**
	 * Send a status text with loglevel INFO
	 *
	 * @param string the message to send (will be capped by mavlink max string length)
	 */
	void			sendStatustextInfo(const char *string);

	/**
	 * Send a status text with loglevel CRITICAL
	 *
	 * @param string the message to send (will be capped by mavlink max string length)
	 */
	void			sendStatustextCritical(const char *string);

	/**
	 * Send a status text with loglevel EMERGENCY
	 *
	 * @param string the message to send (will be capped by mavlink max string length)
	 */
	void			sendStatustextEmergency(const char *string);

	/**
	 * Send a status text with loglevel, the difference from mavlink_log_xxx() is that message sent
	 * only on this mavlink connection. Useful for reporting communication specific, not system-wide info
	 * only to client interested in it. Message will be not sent immediately but queued in buffer as
	 * for mavlink_log_xxx().
	 *
	 * @param string the message to send (will be capped by mavlink max string length)
	 * @param severity the log level
	 */
	void			sendStatustext(unsigned char severity, const char *string);

	/**
	 * Send the capabilities of this autopilot in terms of the MAVLink spec
	 */
	void 			sendAutopilotCapabilites();

	/**
	 * Send the protocol version of MAVLink
	 */
	void			sendProtocolVersion();

	MavlinkStream 		*getStreams() const { return _streams; }

	float			getRateMult();

	float			getBaudrate() { return _baudrate; }

	/* Functions for waiting to start transmission until message received. */
	void			setHasReceivedMessages(bool received_messages) { _received_messages = received_messages; }
	bool			getHasReceivedMessages() { return _received_messages; }
	void			setWaitToTransmit(bool wait) { _wait_to_transmit = wait; }
	bool			getWaitToTransmit() { return _wait_to_transmit; }
	bool			shouldTransmit() { return (boot_complete && (!_wait_to_transmit || (_wait_to_transmit && _received_messages))); }

	bool			messageBufferWrite(const void *ptr, int size);

	void			lockMessageBufferMutex(void) { pthread_mutex_lock(&_message_buffer_mutex); }
	void			unlockMessageBufferMutex(void) { pthread_mutex_unlock(&_message_buffer_mutex); }

	/**
	 * Count a transmision error
	 */
	void			countTxerr();

	/**
	 * Count transmitted bytes
	 */
	void			countTxbytes(unsigned n) { _bytes_tx += n; };

	/**
	 * Count bytes not transmitted because of errors
	 */
	void			countTxerrbytes(unsigned n) { _bytes_txerr += n; };

	/**
	 * Count received bytes
	 */
	void			countRxbytes(unsigned n) { _bytes_rx += n; };

	/**
	 * Get the receive status of this MAVLink link
	 */
	struct telemetry_status_s	&getRxStatus() { return _rstatus; }

	ringbuffer::RingBuffer	*getLogbuffer() { return &_logbuffer; }

	unsigned		getSystemType() { return _system_type; }

	Protocol 		getProtocol() { return _protocol; }

	unsigned short		getNetworkPort() { return _network_port; }

	unsigned short		getRemotePort() { return _remote_port; }

	int 			getSocketFd() { return _socket_fd; };
#ifdef __PX4_POSIX
	struct sockaddr_in 	*getClientSourceAddress() { return &_src_addr; }

	void			setClientSourceInitialized() { _src_addr_initialized = true; }

	bool			getClientSourceInitialized() { return _src_addr_initialized; }
#else
	bool			get_client_source_initialized() { return true; }
#endif

	uint64_t		getStartTime() { return _mavlink_start_time; }

	static bool		bootComplete() { return boot_complete; }

	bool			isUsbUart() { return _is_usb_uart; }

	bool			acceptingCommands() { return true; /* non-trivial side effects ((!_config_link_on) || (_mode == MAVLINK_MODE_CONFIG));*/ }

	bool			verbose() { return _verbose; }

	int			getDataRate()		{ return _datarate; }
	void			setDataRate(int rate) { if (rate > 0) { _datarate = rate; } }

	unsigned		getMainLoopDelay() const { return _main_loop_delay; }

	/** get the Mavlink shell. Create a new one if there isn't one. It is *always* created via MavlinkReceiver thread.
	 *  Returns nullptr if shell cannot be created */
	MavlinkShell		*getShell();
	/** close the Mavlink shell if it is open */
	void			closeShell();

	/** get ulog streaming if active, nullptr otherwise */
	MavlinkULog		*getUlogStreaming() { return _mavlink_ulog; }
	void			tryStartUlogStreaming(uint8_t target_system, uint8_t target_component)
	{
		if (_mavlink_ulog) { return; }

		_mavlink_ulog = MavlinkULog::tryStart(_datarate, 0.7f, target_system, target_component);
	}
	void			requestStopUlogStreaming()
	{
		if (_mavlink_ulog) { _mavlink_ulog_stop_requested = true; }
	}


	void setUorbMainFd(int fd, unsigned int interval);

	bool ftpEnabled() const { return _ftp_on; }

protected:
	Mavlink			*_next;

private:
	int			_instance_id;

	orb_advert_t		_mavlink_log_pub;
	bool			_task_running;
	static bool		boot_complete;
	static constexpr unsigned mavlink_max_instances = 4;
	static constexpr unsigned mavlink_min_interval = 1500;
	static constexpr unsigned mavlink_max_interval = 10000;
	static constexpr float mavlink_min_multiplier = 0.0005f;
	mavlink_message_t _mavlink_buffer;
	mavlink_status_t _mavlink_status;

	/* states */
	bool			_hil_enabled;		/**< Hardware In the Loop mode */
	bool			_generate_rc;		/**< Generate RC messages from manual input MAVLink messages */
	bool			_use_hil_gps;		/**< Accept GPS HIL messages (for example from an external motion capturing system to fake indoor gps) */
	bool			_forward_externalsp;	/**< Forward external setpoint messages to controllers directly if in offboard mode */
	bool			_is_usb_uart;		/**< Port is USB */
	bool			_wait_to_transmit;  	/**< Wait to transmit until received messages. */
	bool			_received_messages;	/**< Whether we've received valid mavlink messages. */

	unsigned		_main_loop_delay;	/**< mainloop delay, depends on data rate */

	MavlinkOrbSubscription	*_subscriptions;
	MavlinkStream		*_streams;

	MavlinkShell			*_mavlink_shell;
	MavlinkULog			*_mavlink_ulog;
	volatile bool			_mavlink_ulog_stop_requested;

	MAVLINK_MODE 		_mode;

	mavlink_channel_t	_channel;
	int32_t			_radio_id;

	ringbuffer::RingBuffer		_logbuffer;
	unsigned int		_total_counter;

	pthread_t		_receive_thread;

	bool			_verbose;
	bool			_forwarding_on;
	bool			_ftp_on;
#ifndef __PX4_QURT
	int			_uart_fd;
#endif
	int			_baudrate;
	int			_datarate;		///< data rate for normal streams (attitude, position, etc.)
	int			_datarate_events;	///< data rate for params, waypoints, text messages
	float			_rate_mult;
	hrt_abstime		_last_hw_rate_timestamp;

	/**
	 * If the queue index is not at 0, the queue sending
	 * logic will send parameters from the current index
	 * to len - 1, the end of the param list.
	 */
	unsigned int		_mavlink_param_queue_index;

	bool			_mavlink_link_termination_allowed;

	char 			*_subscribe_to_stream;
	float			_subscribe_to_stream_rate;
	bool 			_udp_initialised;

	bool			_flow_control_enabled;
	uint64_t		_last_write_success_time;
	uint64_t		_last_write_try_time;
	uint64_t		_mavlink_start_time;
	int32_t			_protocol_version_switch;
	int32_t			_protocol_version;

	unsigned		_bytes_tx;
	unsigned		_bytes_txerr;
	unsigned		_bytes_rx;
	uint64_t		_bytes_timestamp;
	float			_rate_tx;
	float			_rate_txerr;
	float			_rate_rx;

#ifdef __PX4_POSIX
	struct sockaddr_in _myaddr;
	struct sockaddr_in _src_addr;
	struct sockaddr_in _bcast_addr;
	bool _src_addr_initialized;
	bool _broadcast_address_found;
	bool _broadcast_address_not_found_warned;
	bool _broadcast_failed_warned;
	uint8_t _network_buf[MAVLINK_MAX_PACKET_LEN];
	unsigned _network_buf_len;
#endif
	int _socket_fd;
	Protocol	_protocol;
	unsigned short _network_port;
	unsigned short _remote_port;

	struct telemetry_status_s	_rstatus;			///< receive status

	struct mavlink_message_buffer {
		int write_ptr;
		int read_ptr;
		int size;
		char *data;
	};

	mavlink_message_buffer	_message_buffer;

	pthread_mutex_t		_message_buffer_mutex;
	pthread_mutex_t		_send_mutex;

	bool			_param_initialized;
	int32_t			_broadcast_mode;

	param_t			_param_system_id;
	param_t			_param_component_id;
	param_t			_param_proto_ver;
	param_t			_param_radio_id;
	param_t			_param_system_type;
	param_t			_param_use_hil_gps;
	param_t			_param_forward_externalsp;
	param_t			_param_broadcast;

	unsigned		_system_type;
	static bool		config_link_on;

	perf_counter_t		_loop_perf;			/**< loop performance counter */
	perf_counter_t		_txerr_perf;			/**< TX error counter */

	void			mavlinkUpdateSystem();

#ifndef __PX4_QURT
	int			mavlinkOpenUart(int baudrate, const char *uart_name);
#endif

	static int		intervalFromRate(float rate);

	static constexpr unsigned radio_buffer_critical_low_percentage = 25;
	static constexpr unsigned radio_buffer_low_percentage = 35;
	static constexpr unsigned radio_buffer_half_percentage = 50;

	int configureStream(const char *stream_name, const float rate = -1.0f);

	/**
	 * Adjust the stream rates based on the current rate
	 *
	 * @param multiplier if greater than 1, the transmission rate will increase, if smaller than one decrease
	 */
	void adjustStreamRates(const float multiplier);

	int messageBufferInit(int size);

	void messageBufferDestroy();

	int messageBufferCount();

	int messageBufferIsEmpty();

	int messageBufferGetPtr(void **ptr, bool *is_part);

	void messageBufferMarkRead(int n);

	void passMessage(const mavlink_message_t *msg);

	/**
	 * Check the configuration of a connected radio
	 *
	 * This convenience function allows to re-configure a connected
	 * radio without removing it from the main system harness.
	 */
	void checkRadioConfig();

	/**
	 * Update rate mult so total bitrate will be equal to _datarate.
	 */
	void updateRateMult();

	void findBroadcastAddress();

	void initUdp();

	/**
	 * Main mavlink task.
	 */
	int		taskMain(int argc, char *argv[]);

	/* do not allow copying this class */
	Mavlink(const Mavlink &);
	Mavlink operator=(const Mavlink &);
};
