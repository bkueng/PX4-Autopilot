/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
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

/*
 * @file sumd.h
 *
 * RC protocol definition for Graupner HoTT transmitter (SUMD/SUMH Protocol)
 *
 * @author Marco Bauer <marco@wtns.de>
 */

#include <stdbool.h>
#include <stdio.h>
#include "sumd.h"
#include "common_rc.h"

enum SUMD_DECODE_STATE {
	SUMD_DECODE_STATE_UNSYNCED = 0,
	SUMD_DECODE_STATE_GOT_HEADER,
	SUMD_DECODE_STATE_GOT_STATE,
	SUMD_DECODE_STATE_GOT_LEN,
	SUMD_DECODE_STATE_GOT_DATA,
	SUMD_DECODE_STATE_GOT_CRC,
	SUMD_DECODE_STATE_GOT_CRC16_BYTE_1,
	SUMD_DECODE_STATE_GOT_CRC16_BYTE_2
};

/*
const char *decode_states[] = {"UNSYNCED",
			       "GOT_HEADER",
			       "GOT_STATE",
			       "GOT_LEN",
			       "GOT_DATA",
			       "GOT_CRC",
			       "GOT_CRC16_BYTE_1",
			       "GOT_CRC16_BYTE_2"
			      };
*/

uint8_t 	crc8 	= 0x00;
uint16_t 	crc16 = 0x0000;
bool 		sumd 		= true;
bool		crc_ok		= false;
bool		debug		= false;


/* define range mapping here, -+100% -> 1000..2000 */
#define SUMD_RANGE_MIN 0.0f
#define SUMD_RANGE_MAX 4096.0f

#define SUMD_TARGET_MIN 1000.0f
#define SUMD_TARGET_MAX 2000.0f

/* pre-calculate the floating point stuff as far as possible at compile time */
#define SUMD_SCALE_FACTOR ((SUMD_TARGET_MAX - SUMD_TARGET_MIN) / (SUMD_RANGE_MAX - SUMD_RANGE_MIN))
#define SUMD_SCALE_OFFSET (int)(SUMD_TARGET_MIN - (SUMD_SCALE_FACTOR * SUMD_RANGE_MIN + 0.5f))

static enum SUMD_DECODE_STATE decode_state = SUMD_DECODE_STATE_UNSYNCED;
static uint8_t rxlen;

static ReceiverFcPacketHoTT &rxpacket = rc_decode_buf._hottrxpacket;

uint16_t sumd_crc16(uint16_t crc, uint8_t value)
{
	int i;
	crc ^= (uint16_t)value << 8;

	for (i = 0; i < 8; i++) {
		crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
	}

	return crc;
}

uint8_t sumd_crc8(uint8_t crc, uint8_t value)
{
	crc += value;
	return crc;
}

int sumd_decode(uint8_t byte, uint8_t *rssi, uint8_t *rx_count, uint16_t *channel_count, uint16_t *channels,
		uint16_t max_chan_count, bool *failsafe)
{

	int ret = 1;

	switch (decode_state) {
	case SUMD_DECODE_STATE_UNSYNCED:
		if (debug) {
			printf(" SUMD_DECODE_STATE_UNSYNCED \n") ;
		}

		if (byte == SUMD_HEADER_ID) {
			rxpacket.header = byte;
			sumd = true;
			rxlen = 0;
			crc16 = 0x0000;
			crc8 = 0x00;
			crc_ok = false;
			crc16 = sumd_crc16(crc16, byte);
			crc8 = sumd_crc8(crc8, byte);
			decode_state = SUMD_DECODE_STATE_GOT_HEADER;

			if (debug) {
				printf(" SUMD_DECODE_STATE_GOT_HEADER: %x \n", byte) ;
			}

		} else {
			ret = 3;
		}

		break;

	case SUMD_DECODE_STATE_GOT_HEADER:
		if (byte == SUMD_ID_SUMD || byte == SUMD_ID_FAILSAFE || byte == SUMD_ID_SUMH) {
			rxpacket.status = byte;

			if (byte == SUMD_ID_SUMH) {
				sumd = false;
			}

			if (sumd) {
				crc16 = sumd_crc16(crc16, byte);

			} else {
				crc8 = sumd_crc8(crc8, byte);
			}

			decode_state = SUMD_DECODE_STATE_GOT_STATE;

			if (debug) {
				printf(" SUMD_DECODE_STATE_GOT_STATE: %x \n", byte) ;
			}

		} else {
			decode_state = SUMD_DECODE_STATE_UNSYNCED;
		}

		break;

	case SUMD_DECODE_STATE_GOT_STATE:
		if (byte >= 2 && byte <= SUMD_MAX_CHANNELS) {
			rxpacket.length = byte;

			if (sumd) {
				crc16 = sumd_crc16(crc16, byte);

			} else {
				crc8 = sumd_crc8(crc8, byte);
			}

			rxlen++;
			decode_state = SUMD_DECODE_STATE_GOT_LEN;

			if (debug) {
				printf(" SUMD_DECODE_STATE_GOT_LEN: %x (%d) \n", byte, byte) ;
			}

		} else {
			decode_state = SUMD_DECODE_STATE_UNSYNCED;
		}

		break;

	case SUMD_DECODE_STATE_GOT_LEN:
		rxpacket.sumd_data[rxlen] = byte;

		if (sumd) {
			crc16 = sumd_crc16(crc16, byte);

		} else {
			crc8 = sumd_crc8(crc8, byte);
		}

		rxlen++;

		if (rxlen <= ((rxpacket.length * 2))) {
			if (debug) {
				printf(" SUMD_DECODE_STATE_GOT_DATA[%d]: %x\n", rxlen - 2, byte) ;
			}

		} else {
			decode_state = SUMD_DECODE_STATE_GOT_DATA;

			if (debug) {
				printf(" SUMD_DECODE_STATE_GOT_DATA -- finish --\n") ;
			}

		}

		break;

	case SUMD_DECODE_STATE_GOT_DATA:
		rxpacket.crc16_high = byte;

		if (debug) {
			printf(" SUMD_DECODE_STATE_GOT_CRC16[1]: %x   [%x]\n", byte, ((crc16 >> 8) & 0xff)) ;
		}

		if (sumd) {
			decode_state = SUMD_DECODE_STATE_GOT_CRC;

		} else {
			decode_state = SUMD_DECODE_STATE_GOT_CRC16_BYTE_1;
		}

		break;

	case SUMD_DECODE_STATE_GOT_CRC16_BYTE_1:
		rxpacket.crc16_low = byte;

		if (debug) {
			printf(" SUMD_DECODE_STATE_GOT_CRC16[2]: %x   [%x]\n", byte, (crc16 & 0xff)) ;
		}

		decode_state = SUMD_DECODE_STATE_GOT_CRC16_BYTE_2;

		break;

	case SUMD_DECODE_STATE_GOT_CRC16_BYTE_2:
		rxpacket.telemetry = byte;

		if (debug) {
			printf(" SUMD_DECODE_STATE_GOT_SUMH_TELEMETRY: %x\n", byte) ;
		}

		decode_state = SUMD_DECODE_STATE_GOT_CRC;

		break;

	case SUMD_DECODE_STATE_GOT_CRC:
		if (sumd) {
			rxpacket.crc16_low = byte;

			if (debug) {
				printf(" SUMD_DECODE_STATE_GOT_CRC[2]: %x   [%x]\n\n", byte, (crc16 & 0xff)) ;
			}

			if (crc16 == (uint16_t)(rxpacket.crc16_high << 8) + rxpacket.crc16_low) {
				crc_ok = true;
			}

		} else {
			rxpacket.crc8 = byte;

			if (debug) {
				printf(" SUMD_DECODE_STATE_GOT_CRC8_SUMH: %x   [%x]\n\n", byte, crc8) ;
			}

			if (crc8 == rxpacket.crc8) {
				crc_ok = true;
			}
		}

		if (crc_ok) {
			if (debug) {
				printf(" CRC - OK \n") ;
			}

			if (sumd) {
				if (debug) {
					printf(" Got valid SUMD Packet\n") ;
				}

			} else {
				if (debug) {
					printf(" Got valid SUMH Packet\n") ;
				}

			}

			if (debug) {
				printf(" RXLEN: %d  [Chans: %d] \n\n", rxlen - 1, (rxlen - 1) / 2) ;
			}

			ret = 0;
			unsigned i;
			uint8_t cnt = *rx_count + 1;
			*rx_count = cnt;

			*rssi = 100;

			/* failsafe flag */
			*failsafe = (rxpacket.status == SUMD_ID_FAILSAFE);

			/* received Channels */
			if ((uint16_t)rxpacket.length > max_chan_count) {
				rxpacket.length = (uint8_t) max_chan_count;
			}

			*channel_count = (uint16_t)rxpacket.length;

			/* decode the actual packet */
			/* reorder first 4 channels */

			/* ch1 = roll -> sumd = ch2 */
			channels[0] = (uint16_t)((rxpacket.sumd_data[1 * 2 + 1] << 8) | rxpacket.sumd_data[1 * 2 + 2]) >> 3;
			/* ch2 = pitch -> sumd = ch2 */
			channels[1] = (uint16_t)((rxpacket.sumd_data[2 * 2 + 1] << 8) | rxpacket.sumd_data[2 * 2 + 2]) >> 3;
			/* ch3 = throttle -> sumd = ch2 */
			channels[2] = (uint16_t)((rxpacket.sumd_data[0 * 2 + 1] << 8) | rxpacket.sumd_data[0 * 2 + 2]) >> 3;
			/* ch4 = yaw -> sumd = ch2 */
			channels[3] = (uint16_t)((rxpacket.sumd_data[3 * 2 + 1] << 8) | rxpacket.sumd_data[3 * 2 + 2]) >> 3;

			/* we start at channel 5(index 4) */
			unsigned chan_index = 4;

			for (i = 4; i < rxpacket.length; i++) {
				if (debug) {
					printf("ch[%d] : %x %x [ %x    %d ]\n", i + 1, rxpacket.sumd_data[i * 2 + 1], rxpacket.sumd_data[i * 2 + 2],
					       ((rxpacket.sumd_data[i * 2 + 1] << 8) | rxpacket.sumd_data[i * 2 + 2]) >> 3,
					       ((rxpacket.sumd_data[i * 2 + 1] << 8) | rxpacket.sumd_data[i * 2 + 2]) >> 3);
				}

				channels[chan_index] = (uint16_t)((rxpacket.sumd_data[i * 2 + 1] << 8) | rxpacket.sumd_data[i * 2 + 2]) >> 3;
				/* convert values to 1000-2000 ppm encoding in a not too sloppy fashion */
				//channels[chan_index] = (uint16_t)(channels[chan_index] * SUMD_SCALE_FACTOR + .5f) + SUMD_SCALE_OFFSET;

				chan_index++;
			}

		} else {
			/* decoding failed */
			ret = 4;

			if (debug) {
				printf(" CRC - fail \n") ;
			}

		}

		decode_state = SUMD_DECODE_STATE_UNSYNCED;
		break;
	}

	return ret;
}
