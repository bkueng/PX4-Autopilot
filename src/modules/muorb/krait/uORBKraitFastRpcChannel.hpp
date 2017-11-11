/****************************************************************************
 *
 * Copyright (C) 2015 Mark Charlebois. All rights reserved.
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

#ifndef _uORBKraitFastRpcChannel_hpp_
#define _uORBKraitFastRpcChannel_hpp_

#include <stdint.h>
#include <string>
#include <pthread.h>
#include "uORB/uORBCommunicator.hpp"
#include "px4muorb_KraitRpcWrapper.hpp"
#include <map>
#include "drivers/drv_hrt.h"

namespace u_orb
{
class KraitFastRpcChannel;
}

class u_orb::KraitFastRpcChannel : public u_orb_communicator::IChannel
{
public:
	/**
	 * static method to get the IChannel Implementor.
	 */
	static u_orb::KraitFastRpcChannel *getInstance()
	{
		if (instance_ptr == nullptr) {
			instance_ptr = new u_orb::KraitFastRpcChannel();
		}

		return instance_ptr;
	}

	/**
	 * Static method to check if there is an instance.
	 */
	static bool isInstance()
	{
		return (instance_ptr != nullptr);
	}

	/**
	 * @brief Interface to notify the remote entity of a topic being advertised.
	 *
	 * @param messageName
	 * 	This represents the uORB message name(aka topic); This message name should be
	 * 	globally unique.
	 * @return
	 * 	0 = success; This means the messages is successfully sent to the receiver
	 * 		Note: This does not mean that the receiver as received it.
	 *  otherwise = failure.
	 */
	virtual int16_t topic_advertised(const char *message_name);

	/**
	 * @brief Interface to notify the remote entity of a topic being unadvertised
	 * and is no longer publishing messages.
	 *
	 * @param messageName
	 * 	This represents the uORB message name(aka topic); This message name should be
	 * 	globally unique.
	 * @return
	 * 	0 = success; This means the messages is successfully sent to the receiver
	 * 		Note: This does not mean that the receiver as received it.
	 *  otherwise = failure.
	 */
	virtual int16_t topicUnadvertised(const char *message_name);

	/**
	 * @brief Interface to notify the remote entity of interest of a
	 * subscription for a message.
	 *
	 * @param messageName
	 * 	This represents the uORB message name; This message name should be
	 * 	globally unique.
	 * @param msgRate
	 * 	The max rate at which the subscriber can accept the messages.
	 * @return
	 * 	0 = success; This means the messages is successfully sent to the receiver
	 * 		Note: This does not mean that the receiver as received it.
	 *  otherwise = failure.
	 */
	virtual int16_t add_subscription(const char *message_name, int32_t msg_rate_in_hz);


	/**
	 * @brief Interface to notify the remote entity of removal of a subscription
	 *
	 * @param messageName
	 * 	This represents the uORB message name; This message name should be
	 * 	globally unique.
	 * @return
	 * 	0 = success; This means the messages is successfully sent to the receiver
	 * 		Note: This does not necessarily mean that the receiver as received it.
	 *  otherwise = failure.
	 */
	virtual int16_t remove_subscription(const char *message_name);

	/**
	 * Register Message Handler.  This is internal for the IChannel implementer*
	 */
	virtual int16_t register_handler(u_orb_communicator::IChannelRxHandler *handler);


	//=========================================================================
	//     INTERFACES FOR Data messages
	//=========================================================================

	/**
	 * @brief Sends the data message over the communication link.
	 * @param messageName
	 * 	This represents the uORB message name; This message name should be
	 * 	globally unique.
	 * @param length
	 * 	The length of the data buffer to be sent.
	 * @param data
	 * 	The actual data to be sent.
	 * @return
	 *  0 = success; This means the messages is successfully sent to the receiver
	 * 		Note: This does not mean that the receiver as received it.
	 *  otherwise = failure.
	 */
	virtual int16_t send_message(const char *message_name, int32_t length, uint8_t *data);


	void start();
	void stop();

private: // data members
	static u_orb::KraitFastRpcChannel *instance_ptr;
	u_orb_communicator::IChannelRxHandler *_RxHandler;
	pthread_t   _RecvThread;
	bool _ThreadStarted;
	bool _ThreadShouldExit;

	static const int32_t control_msg_type_add_subscriber = 1;
	static const int32_t control_msg_type_remove_subscriber = 2;
	static const int32_t data_msg_type = 3;
	static const int32_t control_msg_type_advertise = 4;
	static const int32_t control_msg_type_unadvertise = 5;

	struct bulk_transfer_header {
		uint16_t _MsgType;
		uint16_t _MsgNameLen;
		uint16_t _DataLen;
	};

	px4muorb::KraitRpcWrapper _KraitWrapper;

	std::map<std::string, int32_t> _AdspSubscriberCache;
	std::map<std::string, hrt_abstime> _AdspSubscriberSampleTimestamp;
	//hrt_abstime  _SubCacheSampleTimestamp;
	static const hrt_abstime sub_cache_refresh_rate = 1000000; // 1 second;

private://class members.
	/// constructor.
	KraitFastRpcChannel();

	static void  *threadStart(void *handler);

	void fastrpcRecvThread();

};

#endif /* _uORBKraitFastRpcChannel_hpp_ */
