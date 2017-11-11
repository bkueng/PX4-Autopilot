/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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

#ifndef DRIVERS_MS5525_AIRSPEED_HPP_
#define DRIVERS_MS5525_AIRSPEED_HPP_

#include <drivers/airspeed/airspeed.h>
#include <drivers/device/i2c.h>
#include <drivers/drv_airspeed.h>
#include <math.h>
#include <mathlib/math/filter/LowPassFilter2p.hpp>
#include <px4_config.h>
#include <sys/types.h>
#include <systemlib/airspeed.h>
#include <systemlib/perf_counter.h>
#include <uORB/topics/differential_pressure.h>
#include <uORB/uORB.h>

/* The MS5525DSO address is 111011Cx, where C is the complementary value of the pin CSB */
static constexpr uint8_t i2_c_address_1_m_s5525_dso = 0x76;

static constexpr const char path_m_s5525[] = "/dev/ms5525";

/* Measurement rate is 100Hz */
static constexpr unsigned meas_rate = 100;
static constexpr float meas_driver_filter_freq = 1.2f;
static constexpr uint64_t conversion_interval = (1000000 / meas_rate); /* microseconds */

class MS5525 : public Airspeed
{
public:
	MS5525(uint8_t bus, uint8_t address = i2_c_address_1_m_s5525_dso, const char *path = path_m_s5525) :
		Airspeed(bus, address, conversion_interval, path)
	{
	}

	~MS5525() override = default;

private:

	/**
	* Perform a poll cycle; collect from the previous measurement
	* and start a new one.
	*/
	void cycle() override;

	int measure() override;
	int collect() override;

	// temperature is read once every 10 cycles
	math::LowPassFilter2p _filter{meas_rate * 0.9, meas_driver_filter_freq};

	static constexpr uint8_t cmd_reset = 0x1E; // ADC reset command
	static constexpr uint8_t cmd_adc_read = 0x00; // ADC read command

	static constexpr uint8_t cmd_prom_start = 0xA0; // Prom read command (first)

	// D1 - pressure convert commands
	// Convert D1 (OSR=256)  0x40
	// Convert D1 (OSR=512)  0x42
	// Convert D1 (OSR=1024) 0x44
	// Convert D1 (OSR=2048) 0x46
	// Convert D1 (OSR=4096) 0x48
	static constexpr uint8_t cmd_convert_pres = 0x44;

	// D2 - temperature convert commands
	// Convert D2 (OSR=256)  0x50
	// Convert D2 (OSR=512)  0x52
	// Convert D2 (OSR=1024) 0x54
	// Convert D2 (OSR=2048) 0x56
	// Convert D2 (OSR=4096) 0x58
	static constexpr uint8_t cmd_convert_temp = 0x54;

	uint8_t _current_cmd{cmd_convert_pres};

	unsigned _pressure_count{0};

	// Qx Coefficients Matrix by Pressure Range
	//  5525DSO-pp001DS (Pmin = -1, Pmax = 1)
	static constexpr uint8_t q1 = 15;
	static constexpr uint8_t q2 = 17;
	static constexpr uint8_t q3 = 7;
	static constexpr uint8_t q4 = 5;
	static constexpr uint8_t q5 = 7;
	static constexpr uint8_t q6 = 21;

	// calibration coefficients from prom
	uint16_t _C1{0};
	uint16_t _C2{0};
	uint16_t _C3{0};
	uint16_t _C4{0};
	uint16_t _C5{0};
	uint16_t _C6{0};

	int64_t _Tref{0};

	// last readings for D1 (uncompensated pressure) and D2 (uncompensated temperature)
	uint32_t _D1{0};
	uint32_t _D2{0};

	bool initMs5525();
	bool _inited{false};

	uint8_t promCrc4(uint16_t n_prom[]) const;

};

#endif /* DRIVERS_MS5525_AIRSPEED_HPP_ */
